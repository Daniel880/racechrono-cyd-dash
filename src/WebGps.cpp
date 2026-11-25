// Strona /gps: diagnostyka GPS - rzeczywista czestotliwosc (zgubione epoki), wysylka do RaceChrono,
// czas do pierwszego fixa, satelity z sila sygnalu, szum wejscia radiowego i test zaklocen
// (ekran / zapis SD wylaczone na 20 s). Strona odpytuje /gps.json co sekunde.
#include "WebPages.h"

#ifdef OTA_WIFI

#include <Arduino.h>
#include <WebServer.h>

#include "Dashboard.h"
#include "GpsReceiver.h"
#include "GpxLogger.h"
#include "RaceChronoBle.h"
#include "SysLog.h"

namespace web {
namespace {

WebServer* srv = nullptr;
constexpr uint32_t kTestMs = 20000;
constexpr uint32_t kFreshMs = 5000;  // starsze dane UBX (przychodza raz na sekunde) = brak

// Satelity w JSON: [system, nr, C/N0, wysokosc, flagi] - znaczenie flag w gpsrx::SatInfo.
const char kBody[] = R"html(<p><a href='/'>Start</a> &middot; <a href='/files'>Pliki</a> &middot; <a href='/log'>Log</a></p>
<p id='err' class='err'></p>
<h3>Czestotliwosc</h3><table id='tR'></table>
<h3>Pierwszy fix</h3><table id='tF'></table>
<h3>Sygnal</h3><table id='tS'></table>
<canvas id='cv' width='420' height='150' style='width:100%;border:1px solid #999'></canvas>
<p><small>Zielona: srednie C/N0 4 najmocniejszych satelitow (dBHz, skala z lewej). Czerwona: szum
z modulu. Tlo szare: ekran wylaczony, niebieskie: zapis SD wstrzymany.</small></p>
<p><button onclick="tst('screen')">Ekran wylaczony 20 s</button>
<button onclick="tst('sd')">Zapis SD wstrzymany 20 s</button></p>
<p id='tT'></p>
<p><small>Test zaklocen: poczekaj, az wykres sie uspokoi, wlacz test i porownaj. Wzrost C/N0 o 3 dB
i wiecej albo wyrazny spadek szumu przy wylaczonym ekranie / zapisie SD = to zrodlo zaklocen.
Przy czystym niebie 4 najmocniejsze satelity maja zwykle 40-48 dBHz; ponizej 35 = slaby sygnal
(dach, zasloniete niebo, antena, zaklocenia). WiFi tej strony tez nadaje - porownuj tylko pomiary
przy otwartej stronie.</small></p>
<h3>Systemy</h3><table id='tG'></table>
<h3>Satelity</h3><table id='tV'></table>
<script>
const GN={0:'GPS',1:'SBAS',2:'Galileo',3:'BeiDou',5:'QZSS',6:'GLONASS'};
const QI=['brak','szuka','zlapany','nieuzyteczny','kod','kod+faza','kod+faza','kod+faza'];
const OR=['-','efemerydy','almanach','inna'];
const JAM=['nieznany','brak','OSTRZEZENIE','KRYTYCZNY'];
const ANT=['start','nieznana','OK','ZWARCIE','ROZWARCIE'];
const ST=['brak modulu','brak danych','brak fixa','fix'];
let H=[],prev=null;
const R=(k,v)=>'<tr><td>'+k+'</td><td>'+v+'</td></tr>';
const avg=a=>a.length?a.reduce((x,y)=>x+y,0)/a.length:0;
const sec=x=>(x/1000).toFixed(1)+' s';
function tst(w){fetch('/test?w='+w,{method:'POST'}).then(r=>r.text()).then(t=>tT.textContent=t)
 .catch(()=>tT.textContent='Blad polaczenia')}
function draw(){const c=cv.getContext('2d'),W=cv.width,Hh=cv.height,dx=W/120;c.clearRect(0,0,W,Hh);
 H.forEach((p,i)=>{if(p.scr||p.sdp){c.fillStyle=p.scr?'#ddd':'#cde';c.fillRect(i*dx,0,dx+1,Hh)}});
 const yc=v=>Hh-(v-15)/40*Hh;c.font='10px sans-serif';
 for(let v=20;v<=50;v+=10){c.strokeStyle='#eee';c.beginPath();c.moveTo(0,yc(v));c.lineTo(W,yc(v));
  c.stroke();c.fillStyle='#888';c.fillText(v,2,yc(v)-2)}
 const nz=H.map(p=>p.noise).filter(v=>v!=null),mx=nz.length?Math.max(...nz)*1.25:1;
 const line=(k,col,sc)=>{c.strokeStyle=col;c.lineWidth=2;c.beginPath();let on=false;
  H.forEach((p,i)=>{const v=p[k];if(v==null){on=false;return}if(on)c.lineTo(i*dx,sc(v));
  else{c.moveTo(i*dx,sc(v));on=true}});c.stroke()};
 line('top4','#080',yc);line('noise','#c00',v=>Hh-v/mx*Hh)}
async function poll(){let d;
 try{d=await(await fetch('/gps.json')).json();err.textContent=''}
 catch(e){err.textContent='Brak polaczenia z wyswietlaczem';return}
 const r=d.rate;
 let s=R('Tryb',d.hz+' Hz ('+d.mode+')'+(d.run?'':' - modul w konfiguracji'));
 if(r.length){s+=R('Epoki na sekunde','min '+Math.min(...r)+', srednio '+avg(r).toFixed(2)+', max '+
  Math.max(...r)+' (ostatnie '+r.length+' s)');s+=R('Ostatnie 15 s',r.slice(-15).join(' '))}
 s+=R('Zgubione epoki',d.miss+' z '+d.ep+
  (d.gaps?'; przerwy w danych ponad 3 s: '+d.gaps:''));
 let b=d.ble.c?'polaczone':'nie polaczone';
 if(d.ble.c&&prev&&prev.ble.c){const dt=(d.up-prev.up)/1000;
  if(dt>0)b+=', wysylane '+((d.ble.s-prev.ble.s)/dt).toFixed(1)+' pozycji/s'}
 s+=R('RaceChrono',b+'; nieudane wysylki: '+d.ble.f+' z '+d.ble.s);
 tR.innerHTML=s;
 s=R('Od wlaczenia ESP',d.ff?sec(d.ff):'jeszcze nie bylo (od startu '+sec(d.up)+')');
 if(d.nav)s+=R('Wg modulu GPS','TTFF '+(d.nav.ttff?sec(d.nav.ttff):'-')+', modul dziala od '+
  Math.round(d.nav.msss/1000)+' s'+(d.nav.msss>d.up+5000?' (pracowal juz przed startem ESP)':''));
 s+=R('Norma M10','zimny start ok. 30 s przy czystym niebie; ponad ~60 s = slaby sygnal');
 s+=R('Utraty fixa',d.lost);
 tF.innerHTML=s;
 const L=d.sat?d.sat:[],tr=L.filter(x=>x[2]>0),us=L.filter(x=>x[4]&1);
 const top=tr.map(x=>x[2]).sort((a,b)=>b-a).slice(0,4),top4=top.length?avg(top):null;
 s=R('Fix',ST[d.st]+(d.used?', '+d.used+' satelitow, HDOP '+d.hdop.toFixed(1)+
  (d.acc?', blad ok. '+d.acc.toFixed(1)+' m':''):''));
 if(d.sat){s+=R('C/N0 4 najmocniejszych',top4?top4.toFixed(1)+' dBHz':'-');
  s+=R('C/N0 uzytych, srednio',us.length?avg(us.map(x=>x[2])).toFixed(1)+' dBHz':'-');
  s+=R('Satelity','uzyte '+us.length+', slyszane '+tr.length+', na liscie '+L.length)}
 else s+=R('Satelity','brak danych z modulu');
 if(d.rf)s+=R('Wejscie radiowe','szum '+d.rf.noise+', AGC '+d.rf.agc+' / 8191, zaklocenia waskopasmowe '+
  d.rf.cw+' / 255, detektor zaklocen: '+JAM[d.rf.jam]+', antena: '+(ANT[d.rf.ant]||d.rf.ant));
 tS.innerHTML=s;
 H.push({top4:top4,noise:d.rf?d.rf.noise:null,scr:d.scr>0,sdp:d.sdp>0});if(H.length>120)H.shift();draw();
 tT.textContent=(d.scr>0?'Ekran wylaczony jeszcze '+Math.ceil(d.scr/1000)+' s. ':'')+
  (d.sdp>0?'Zapis SD wstrzymany jeszcze '+Math.ceil(d.sdp/1000)+' s.':'');
 const sy={};L.forEach(x=>{const k=GN[x[0]]||('system '+x[0]),o=sy[k]=sy[k]||{v:0,t:0,u:0,c:[]};
  o.v++;if(x[2]>0){o.t++;o.c.push(x[2])}if(x[4]&1)o.u++});
 s='<tr><th>System</th><th>Uzyte</th><th>Slyszane</th><th>Na liscie</th><th>C/N0 sr.</th></tr>';
 for(const k in sy){const o=sy[k];s+='<tr><td>'+k+'</td><td>'+o.u+'</td><td>'+o.t+'</td><td>'+o.v+
  '</td><td>'+(o.c.length?avg(o.c).toFixed(1):'-')+'</td></tr>'}
 tG.innerHTML=L.length?s:'';
 s='<tr><th>System</th><th>Nr</th><th>C/N0</th><th>Wys.</th><th>Uzyty</th><th>Sygnal</th><th>Orbita</th></tr>';
 L.slice().sort((a,b)=>b[2]-a[2]).forEach(x=>{s+='<tr><td>'+(GN[x[0]]||x[0])+'</td><td>'+x[1]+
  '</td><td><span style="display:inline-block;width:'+Math.min(100,x[2]*2)+'px;background:'+
  (x[2]>=40?'#4a4':x[2]>=30?'#cb3':'#c44')+'">&nbsp;</span> '+x[2]+'</td><td>'+
  (Math.abs(x[3])<=90?x[3]+'&deg;':'-')+'</td><td>'+(x[4]&1?'tak':'')+'</td><td>'+QI[(x[4]>>1)&7]+
  '</td><td>'+OR[(x[4]>>4)&3]+'</td></tr>'});
 tV.innerHTML=L.length?s:'';
 prev=d}
setInterval(poll,1000);poll();
</script></body></html>)html";

void handlePage() {
  WebServer& s = *srv;
  s.setContentLength(CONTENT_LENGTH_UNKNOWN);
  s.send(200, "text/html", "");
  s.sendContent_P(kHead);
  s.sendContent_P(kBody);
  s.sendContent("");
}

void handleJson() {
  static gpsrx::Diag d;  // ~650 B - nie na stosie zadania loop
  gpsrx::diag(d);
  const gpsrx::Quality q = gpsrx::quality();
  const rc::GpsTx tx = rc::gpsTx();

  String j;
  j.reserve(2800);
  j += "{\"hz\":";
  j += d.hz;
  j += ",\"mode\":\"";
  j += d.modeName;
  j += "\",\"run\":";
  j += d.running ? 1 : 0;
  j += ",\"up\":";
  j += millis();
  j += ",\"rate\":[";
  for (uint8_t i = 0; i < d.rateLen; i++) {
    if (i) j += ',';
    j += d.rate[i];
  }
  j += "],\"ep\":";
  j += d.epochs;
  j += ",\"miss\":";
  j += d.missed;
  j += ",\"gaps\":";
  j += d.gaps;
  j += ",\"ff\":";
  j += d.firstFixMs;
  j += ",\"lost\":";
  j += d.fixLost;
  j += ",\"st\":";
  j += (int)gpsrx::state();
  j += ",\"used\":";
  j += q.used;
  j += ",\"hdop\":";
  j += String(q.hdop, 1);
  j += ",\"acc\":";
  j += String(q.accM, 1);
  j += ",\"ble\":{\"c\":";
  j += rc::isConnected() ? 1 : 0;
  j += ",\"s\":";
  j += tx.sent;
  j += ",\"f\":";
  j += tx.failed;
  j += "},\"nav\":";
  if (d.navAge < kFreshMs) {
    j += "{\"fix\":";
    j += d.navFix;
    j += ",\"ttff\":";
    j += d.navTtff;
    j += ",\"msss\":";
    j += d.navMsss;
    j += '}';
  } else {
    j += "null";
  }
  j += ",\"rf\":";
  if (d.rfAge < kFreshMs) {
    j += "{\"noise\":";
    j += d.noise;
    j += ",\"agc\":";
    j += d.agc;
    j += ",\"cw\":";
    j += d.cw;
    j += ",\"jam\":";
    j += d.jam;
    j += ",\"ant\":";
    j += d.ant;
    j += '}';
  } else {
    j += "null";
  }
  j += ",\"sat\":";
  if (d.satAge < kFreshMs) {
    j += '[';
    for (uint8_t i = 0; i < d.numSats; i++) {
      const gpsrx::SatInfo& si = d.sats[i];
      if (i) j += ',';
      j += '[';
      j += si.gnss;
      j += ',';
      j += si.sv;
      j += ',';
      j += si.cno;
      j += ',';
      j += (int)si.elev;
      j += ',';
      j += si.flags;
      j += ']';
    }
    j += ']';
  } else {
    j += "null";
  }
  j += ",\"scr\":";
  j += dash::blankLeftMs();
  j += ",\"sdp\":";
  j += gpx::pauseLeftMs();
  j += '}';
  srv->send(200, "application/json", j);
}

void handleTest() {
  const String w = srv->arg("w");
  if (w == "screen") {
    dash::blankFor(kTestMs);
    srv->send(200, "text/plain", "Ekran wylaczony na 20 s");
  } else if (w == "sd") {
    if (gpx::state() != gpx::State::Logging && gpx::state() != gpx::State::Waiting) {
      srv->send(400, "text/plain", "Zapis na SD i tak nie dziala");
      return;
    }
    gpx::pauseFor(kTestMs);
    srv->send(200, "text/plain", "Zapis SD wstrzymany na 20 s (dane z tego czasu nie trafia do pliku)");
  } else {
    srv->send(400, "text/plain", "Nieznany test");
  }
}

}  // namespace

void addGpsRoutes(WebServer& server) {
  srv = &server;
  server.on("/gps", HTTP_GET, handlePage);
  server.on("/gps.json", HTTP_GET, handleJson);
  server.on("/test", HTTP_POST, handleTest);
}

}  // namespace web

#endif  // OTA_WIFI

// ============================================================
//  KONTROL ESP32 - v183   (robot v183 ile eslesir)
//
//  GEREKLI KUTUPHANE:
//    Library Manager -> "WebSockets" by Markus Sattler (arduinoWebSockets)
//
//  v182'ye gore DEGISEN: FEEDFORWARD LEAN (FF egim) girisi.
//   - Surus kartina "FF Lean (derece)" girisi eklendi. Gaza basinca
//     robot integrali beklemeden throttle ile orantili direkt egilir.
//   - struct'taki kullanilmayan ff_cruise alani FF egimi tasir
//     (PROTOKOL DEGISMEDI). Bos birakilirsa -1 (sentinel) gider,
//     robot kendi varsayilanini (8 derece) korur.
//   - 'f' komutu -> robota canli FF_LEAN_MAX.
//   - Teshis kartinda "FF (derece)" gostergesi (anlik ffLean, robottan).
//
//  ---- v182'den miras: canli motor akimi slider'i ----
//  ---- v181'den miras: cascade + rampa ----
//  ---- v180'den miras: WebSocket haberlesme ----
//   1) SURUS kartina "Ivme (Hz/s)" girisi eklendi -> struct'taki
//      kullanilmayan drive_accel alani devreye alindi. PROTOKOL DEGISMEDI.
//      Bu, hedef hizin rampalanmasini kontrol eder: joystick'in basamak
//      atlamasini yumusatir, dis dongu overshoot'unu kaynaginda keser.
//   2) 'd' komutu artik 3 argumanli: d <maxHz> <maxLean> <accelHz>
//   3) Yeni CASCADE TESHIS karti:
//        iTerm    = Ki * integral (ic dongu). 255'e dayaniyorsa robot
//                   cikis limitindedir -> daha fazla hiz FIZIKSEL OLARAK
//                   mumkun degil, kazanc arttirmak ise yaramaz.
//        Rampa Hz = rampalanmis hedef hiz (ham hedefe ne kadar yaklasti)
//        vHata    = dis dongu hiz hatasi (Hz)
//        Doyma    = Kp_vel'in doyma esigi = maxLean/(Kp_vel*0.001).
//                   Hiz hatasi bu esigin ustundeyken Kp_vel'in HICBIR
//                   etkisi yoktur; egim zaten maxLean'de kilitlidir.
//   4) KpV slider araligi 0-100 -> 0-10 (0.05 adim). Eski aralik
//      kullanisli bolgenin ~75 kati uzunluktaydi, ince ayar imkansizdi.
//
//  Yeni telemetri alanlari robot tarafinda bos struct alanlarina
//  yazildigi icin paket boyutu ve duzeni AYNI kaldi.
// ============================================================

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ==================== AYAR ====================
const char*   AP_SSID = "BalansRobot";
const char*   AP_PASS = "12345678";
const uint8_t WIFI_CH = 1;          // robot v181 ile AYNI olmali

uint8_t robotMAC[] = {0x30, 0x76, 0xF5, 0x94, 0x49, 0xB8};

// ==================== STRUCT'LAR (DEGISMEDI) ====================
typedef struct {
  float    angle;
  float    target_angle;
  float    error;
  float    rate;
  float    motorSpeed;
  float    throttle;
  float    tgtMS;
  float    fMS;
  float    velIntegral;
  uint8_t  frozen;
  float    kpVel;
  float    kiVel;
  float    kpAngle;
  float    kdAngle;
  float    tiltRaw;
  float    ffDeg;
  float    driveHz;
  float    driveErrorHz;
  float    driveIntegral;
  float    kpStill;
  float    kdStill;
  float    kiAngle;
  float    ffBreak;
  float    driveMaxHz;
  float    driveAccelHz;
  float    balAngle;
  float    maxLean;
  uint32_t cmdAge;
  uint32_t ms;
  float    angleScale;
} TelemetriPaketi;

typedef struct {
  float targetAngle;
  float steering;
  float kp, kd, ki;
  bool  stop;
  float balanceAngle;
  float maxLean;
  float kp_vel;
  float ki_vel;
  float ff_break;
  float ff_cruise;
  float drive_max_hz;
  float drive_accel;
  float kp_drive;
  float ki_drive;
} KomutPaketi;

TelemetriPaketi lastTelem;
KomutPaketi     komut;
esp_now_peer_info_t peerInfo;

bool              telemEverSeen = false;
volatile uint32_t lastTelemLocalMs = 0;
portMUX_TYPE      telemMux = portMUX_INITIALIZER_UNLOCKED;

volatile float joyY = 0.0f;
volatile float joyX = 0.0f;

volatile uint32_t txTot = 0, txFail = 0;
volatile uint32_t txWinTot = 0, txWinFail = 0;
volatile float    txLossPct = 0.0f;

WebServer        server(80);
WebSocketsServer wsServer(81);

// ==================== ESP-NOW CALLBACK'LER ====================
void espnowAlCallback(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(TelemetriPaketi)) {
    portENTER_CRITICAL_ISR(&telemMux);
    memcpy(&lastTelem, data, sizeof(TelemetriPaketi));
    telemEverSeen    = true;
    lastTelemLocalMs = millis();
    portEXIT_CRITICAL_ISR(&telemMux);
  }
}

void espnowGonderCallback(const wifi_tx_info_t *tx_info, esp_now_send_status_t s) {
  txTot++; txWinTot++;
  if (s != ESP_NOW_SEND_SUCCESS) { txFail++; txWinFail++; }
  if (txWinTot >= 100) {
    txLossPct = (100.0f * txWinFail) / txWinTot;
    txWinTot = 0; txWinFail = 0;
  }
}

// ==================== WEB SAYFASI ====================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
<title>Balans Robot v181</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent}
body{background:#1e1e2e;color:#cdd6f4;font-family:sans-serif;padding:12px 18vw 12px 12px;user-select:none}
h2{color:#89b4fa;text-align:center;margin-bottom:12px;font-size:18px}
.card{background:#313244;border-radius:12px;padding:14px;margin-bottom:12px}
.card h3{color:#a6adc8;font-size:10px;margin-bottom:10px;letter-spacing:1px;text-transform:uppercase}
.row{display:flex;gap:8px;margin-bottom:8px}
.row:last-child{margin-bottom:0}
.stat{flex:1;background:#1e1e2e;border-radius:8px;padding:8px;text-align:center}
.stat span{display:block;font-size:10px;color:#a6adc8;margin-bottom:2px}
.stat b{font-size:15px}
.joy-pad-wrap{display:flex;flex-direction:column;align-items:center;gap:10px;margin:10px 0}
#joy-pad-base{width:220px;height:220px;border-radius:50%;background:#45475a;position:relative;touch-action:none;box-shadow:inset 0 0 22px rgba(0,0,0,0.35)}
#joy-pad-base::before{content:'';position:absolute;left:50%;top:50%;width:6px;height:6px;border-radius:50%;background:#585b70;transform:translate(-50%,-50%)}
#joy-pad-base::after{content:'';position:absolute;left:6px;top:6px;right:6px;bottom:6px;border-radius:50%;border:1px dashed #585b70;opacity:0.5}
#joy-pad-knob{width:64px;height:64px;border-radius:50%;background:#89b4fa;position:absolute;left:78px;top:78px;box-shadow:0 3px 8px rgba(0,0,0,0.4);transition:left 0.06s,top 0.06s}
#joy-pad-knob.active{transition:none;background:#b4d0fb}
.pad-readout{display:flex;gap:18px}
.pad-readout .stat{min-width:70px}
.hareket-row{display:flex;align-items:center;justify-content:center;gap:18px;flex-wrap:wrap}
.btn-quickstop{width:76px;height:76px;border-radius:50%;background:#f38ba8;color:#1e1e2e;font-size:12px;font-weight:bold;line-height:1.3;border:none;cursor:pointer;flex-shrink:0}
.btn-quickstop.active{background:#a6e3a1}
.confirm-tick{display:inline-flex;align-items:center;justify-content:center;width:24px;height:24px;border-radius:50%;background:#1e1e2e;color:#585b70;font-size:13px;font-weight:bold;flex-shrink:0;transition:background .2s,color .2s}
.confirm-tick.pending{background:#f9e2af;color:#1e1e2e}
.confirm-tick.ok{background:#a6e3a1;color:#1e1e2e}
.pid-row{display:flex;align-items:center;gap:10px;margin-bottom:10px}
.pid-label{width:44px;font-size:12px;color:#a6adc8;font-weight:bold}
.pid-row input[type=range]{flex:1;-webkit-appearance:none;appearance:none;height:38px;background:transparent}
.pid-row input[type=range]::-webkit-slider-runnable-track{height:8px;background:#45475a;border-radius:4px}
.pid-row input[type=range]::-webkit-slider-thumb{-webkit-appearance:none;width:30px;height:30px;border-radius:50%;background:#89b4fa;margin-top:-11px;box-shadow:0 2px 6px rgba(0,0,0,0.4)}
.pid-row input[type=range]::-moz-range-track{height:8px;background:#45475a;border-radius:4px}
.pid-row input[type=range]::-moz-range-thumb{width:30px;height:30px;border:none;border-radius:50%;background:#89b4fa}
.num-row{display:flex;align-items:center;gap:10px;margin-bottom:10px}
.num-label{flex:1;font-size:12px;color:#a6adc8}
input[type=number]{width:70px;background:#1e1e2e;border:1px solid #45475a;border-radius:6px;color:#cdd6f4;padding:5px 8px;font-size:13px;text-align:center}
button{padding:10px;border:none;border-radius:10px;font-size:14px;font-weight:bold;cursor:pointer}
.btn-stop{background:#f38ba8;color:#1e1e2e;width:100%;margin-bottom:6px}
.btn-resume{background:#a6e3a1;color:#1e1e2e;width:100%;margin-bottom:6px}
.btn-send{background:#cba6f7;color:#1e1e2e;width:100%;margin-top:4px}
.btn-sync{background:#89b4fa;color:#1e1e2e;width:100%;margin-bottom:6px}
#status{text-align:center;font-size:11px;color:#585b70;margin-top:6px}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;background:#a6e3a1;margin-right:5px;vertical-align:middle}
.dot.off{background:#f38ba8}
.hint{font-size:10px;color:#585b70;margin-top:-4px;margin-bottom:8px;line-height:1.4}
</style>
</head>
<body>
<h2>&#129302; Balans Robot v181</h2>

<div class="card">
  <h3>Cascade Te&#351;his</h3>
  <div class="row">
    <div class="stat"><span>iTerm /255</span><b id="v_iterm" style="color:#f38ba8">--</b></div>
    <div class="stat"><span>Rampa Hz</span><b id="v_ramp" style="color:#f9e2af">--</b></div>
    <div class="stat"><span>FF (&deg;)</span><b id="v_ff" style="color:#89dceb">--</b></div>
    <div class="stat"><span>Ak&#305;m (mA)</span><b id="v_cur" style="color:#a6e3a1">--</b></div>
  </div>
  <div class="hint">iTerm 255'e dayan&#305;yorsa robot &#231;&#305;k&#305;&#351; limitindedir; daha fazla h&#305;z m&#252;mk&#252;n de&#287;il. H&#305;z hatas&#305; "Doyma" de&#287;erinin &#252;st&#252;ndeyken KpV'nin hi&#231;bir etkisi yoktur.</div>
</div>

<div class="card">
  <h3>Ba&#287;lant&#305; Sa&#287;l&#305;&#287;&#305;</h3>
  <div class="row">
    <div class="stat"><span>WS RTT (ms)</span><b id="v_rtt" style="color:#89b4fa">--</b></div>
    <div class="stat"><span>Robot (ms)</span><b id="v_link" style="color:#a6e3a1">--</b></div>
    <div class="stat"><span>CmdAge (ms)</span><b id="v_cmdage" style="color:#f9e2af">--</b></div>
    <div class="stat"><span>TX kay&#305;p</span><b id="v_loss" style="color:#fab387">--</b></div>
  </div>
</div>

<div class="card">
  <h3>Canl&#305; Veri</h3>
  <div class="row">
    <div class="stat"><span>A&#231;&#305; (&deg;)</span><b id="v_angle" style="color:#89b4fa">--</b></div>
    <div class="stat"><span>Hedef (&deg;)</span><b id="v_target" style="color:#cba6f7">--</b></div>
    <div class="stat"><span>Hata (&deg;)</span><b id="v_error" style="color:#f38ba8">--</b></div>
    <div class="stat"><span>Motor Hz</span><b id="v_speed" style="color:#a6e3a1">--</b></div>
  </div>
  <div class="row">
    <div class="stat"><span>&#214;l&#231;&#252;len Hz</span><b id="v_fms" style="color:#89b4fa">--</b></div>
    <div class="stat"><span>Ham Hedef Hz</span><b id="v_tgt" style="color:#f9e2af">--</b></div>
    <div class="stat"><span>vTilt (&deg;)</span><b id="v_vtilt" style="color:#fab387">--</b></div>
    <div class="stat"><span>Vint</span><b id="v_vint" style="color:#fab387">--</b></div>
  </div>
  <div class="row">
    <div class="stat"><span>BNO / Durum</span><b id="v_frz" style="color:#a6e3a1">--</b></div>
    <div class="stat"><span>Scale</span><b id="v_scale" style="color:#f9e2af">--</b></div>
  </div>
</div>

<div class="card">
  <h3>Hareket <span class="dot" id="conn-dot"></span><span id="conn-txt">Ba&#287;lan&#305;yor</span></h3>
  <div class="hareket-row">
    <div class="joy-pad-wrap">
      <div id="joy-pad-base"><div id="joy-pad-knob"></div></div>
      <div class="pad-readout">
        <div class="stat"><span>Gaz</span><b id="v_thr" style="color:#89b4fa">0.00</b></div>
        <div class="stat"><span>D&#246;n&#252;&#351;</span><b id="v_str" style="color:#cba6f7">0.00</b></div>
      </div>
    </div>
    <button class="btn-quickstop" id="quickStopBtn" onclick="toggleQuickStop()">&#9940;<br>DUR</button>
  </div>
  <div class="hint">Tam yukar&#305; = d&#252;z ileri &middot; tam a&#351;a&#287;&#305; = d&#252;z geri &middot; tam sa&#287;/sol = yerinde d&#246;n&#252;&#351; &middot; aradaki a&#231;&#305;lar ikisinin kombinasyonu.</div>
</div>

<div class="card">
  <h3>Daire &#199;iz</h3>
  <div class="row">
    <button class="btn-send" onclick="driveCircle(1.0,1,0.15)">1m &#231;ap &#8635; Sa&#287;a</button>
    <button class="btn-send" onclick="driveCircle(1.0,-1,0.15)">1m &#231;ap &#8634; Sola</button>
  </div>
  <div class="row">
    <button class="btn-send" onclick="driveCircle(0.5,1,0.08)">50cm &#231;ap &#8635; Sa&#287;a</button>
    <button class="btn-send" onclick="driveCircle(0.5,-1,0.08)">50cm &#231;ap &#8634; Sola</button>
  </div>
  <div class="row">
    <button class="btn-send" style="background:#89dceb" onclick="driveCircle(0.3,1,0.05)">30cm &#231;ap &#8635; Sa&#287;a</button>
    <button class="btn-send" style="background:#89dceb" onclick="driveCircle(0.3,-1,0.05)">30cm &#231;ap &#8634; Sola</button>
  </div>
  <button class="btn-stop" onclick="stopCircle()">&#9632; Daireyi Durdur</button>
  <div class="hint">Her buton kendi g&#252;venli h&#305;z&#305;n&#305; otomatik ayarlar (tekerlek aras&#305; 259mm'ye g&#246;re hesapland&#305;) &middot; iki tekerlek de her zaman ileri y&#246;nde d&#246;ner, geri d&#246;nme yok &middot; joystick'e dokunmak da daireyi an&#305;nda keser.</div>
</div>

<div class="card">
  <h3>Parametreler (robottan)</h3>
  <button class="btn-sync" onclick="pullParams()">&#8635; Robottan Al</button>
  <div id="sync-note" style="font-size:11px;color:#585b70;text-align:center">Slider'lar robot ba&#287;lan&#305;nca otomatik dolar</div>
</div>

<div class="card">
  <h3>A&#231;&#305; PID</h3>
  <div class="pid-row"><span class="pid-label">Kp</span><input type="range" id="kp" min="10" max="15" step="0.1" value="10" oninput="syncNum('kp');markIdle('kp')"><input type="number" id="kp-num" min="10" max="15" step="0.1" value="10" style="width:60px" oninput="numToSlider('kp');markIdle('kp')"><span class="confirm-tick" id="tick-kp">&#8211;</span></div>
  <div class="pid-row"><span class="pid-label">Kd</span><input type="range" id="kd" min="0.2" max="0.5" step="0.01" value="0.2" oninput="syncNum('kd');markIdle('kd')"><input type="number" id="kd-num" min="0.2" max="0.5" step="0.01" value="0.2" style="width:60px" oninput="numToSlider('kd');markIdle('kd')"><span class="confirm-tick" id="tick-kd">&#8211;</span></div>
  <div class="pid-row"><span class="pid-label">Ki</span><input type="range" id="ki" min="0" max="100" step="1" value="0" oninput="syncNum('ki');markIdle('ki')"><input type="number" id="ki-num" min="0" max="100" step="1" value="0" style="width:60px" oninput="numToSlider('ki');markIdle('ki')"><span class="confirm-tick" id="tick-ki">&#8211;</span></div>
  <div class="hint">Ki de&#287;i&#351;ince robot integral tavan&#305;n&#305; (255/Ki) otomatik g&#252;nceller. Kp: 10-15, Kd: 0.2-0.5, Ki: 0-100 aral&#305;&#287;&#305;yla s&#305;n&#305;rl&#305;.</div>
  <button class="btn-send" onclick="sendPID()">PID G&#246;nder</button>
</div>

<div class="card">
  <h3>H&#305;z PI (d&#305;&#351; d&#246;ng&#252;)</h3>
  <div class="pid-row"><span class="pid-label">KpV</span><input type="range" id="kpv" min="0" max="10" step="0.05" value="0" oninput="syncNum('kpv');markIdle('kpv')"><input type="number" id="kpv-num" min="0" max="50" step="0.05" value="0" style="width:60px" oninput="numToSlider('kpv');markIdle('kpv')"><span class="confirm-tick" id="tick-kpv">&#8211;</span></div>
  <div class="pid-row"><span class="pid-label">KiV</span><input type="range" id="kiv" min="0" max="20" step="0.1" value="0" oninput="syncNum('kiv');markIdle('kiv')"><input type="number" id="kiv-num" min="0" max="20" step="0.1" value="0" style="width:60px" oninput="numToSlider('kiv');markIdle('kiv')"><span class="confirm-tick" id="tick-kiv">&#8211;</span></div>
  <div class="hint">KpV'nin faydal&#305; tavan&#305; = MaxLean / (0.001 &times; MaxHz). &#220;st&#252;nde tam gazda etkisizdir, sadece hedefe yak&#305;n b&#246;lgeyi sertle&#351;tirir.</div>
  <button class="btn-send" onclick="sendVelPID()">H&#305;z PI G&#246;nder</button>
</div>

<div class="card">
  <h3>S&#252;r&#252;&#351;</h3>
  <div class="pid-row"><span class="pid-label" style="width:auto;font-size:11px">Max H&#305;z (m/s)</span><input type="range" id="dmax" min="0.03" max="0.45" step="0.005" value="0.1" oninput="syncNum('dmax');markIdle('dmax')"><input type="number" id="dmax-num" min="0.03" max="0.45" step="0.005" value="0.1" style="width:60px" oninput="numToSlider('dmax');markIdle('dmax')"><span class="confirm-tick" id="tick-dmax">&#8211;</span></div>
  <div class="row" style="margin-top:2px">
    <button class="btn-send" style="background:#a6e3a1" onclick="setSpeedMode(0.15)">Sakin</button>
    <button class="btn-send" style="background:#f9e2af" onclick="setSpeedMode(0.30)">Orta</button>
    <button class="btn-send" style="background:#f38ba8" onclick="setSpeedMode(0.45)">H&#305;zl&#305;</button>
  </div>
  <div class="num-row"><span class="num-label">Max E&#287;ilme A&#231;&#305;s&#305; (&deg;)</span><input type="number" id="maxlean" value="4" step="0.5" placeholder="4" oninput="markIdle('maxlean')"><span class="confirm-tick" id="tick-maxlean">&#8211;</span></div>
  <div class="num-row"><span class="num-label">&#304;vme (Hz/s)</span><input type="number" id="daccel" value="4000" step="250" placeholder="4000" style="width:80px" oninput="markIdle('daccel')"><span class="confirm-tick" id="tick-daccel">&#8211;</span></div>
  <div class="hint">&#304;vme = hedef h&#305;z rampas&#305;. D&#252;&#351;&#252;k de&#287;er yumu&#351;ak kalk&#305;&#351;/overshoot yok, y&#252;ksek de&#287;er sert tepki. Max E&#287;ilme ivmenin ger&#231;ek kolu; KpV de&#287;il.</div>
  <div class="pid-row"><span class="pid-label" style="width:auto;font-size:11px">Ak&#305;m</span><input type="range" id="cur" min="200" max="1500" step="25" value="1000" oninput="syncNum('cur');liveCurrent()"><input type="number" id="cur-num" min="200" max="1500" step="25" value="1000" style="width:60px" oninput="numToSlider('cur');liveCurrent()"><span class="confirm-tick" id="tick-cur">&#8211;</span></div>
  <div class="hint">Motor ak&#305;m&#305; (mA) = holding torque. D&#252;&#351;&#252;r&#252;rse robot egik acida tutunamaz, ilerlemeye ba&#351;lar. YAVA&#350; indir, ACIL DUR haz&#305;r olsun. &#199;ok d&#252;&#351;&#252;kte dik dururken de d&#252;&#351;er. Bu slider CANLI g&#246;nderir.</div>
  <div class="num-row"><span class="num-label">FF Lean (&deg;)</span><input type="number" id="fflean" value="8" step="0.5" placeholder="8" style="width:80px" oninput="markIdle('fflean')"><span class="confirm-tick" id="tick-fflean">&#8211;</span></div>
  <div class="hint">FF Lean = gaza bas&#305;nca robotun DIREKT egilme a&#231;&#305;s&#305; (throttle ile orant&#305;l&#305;). Integrali beklemeden h&#305;zl&#305; tepki. H&#305;z PI bunun &#252;st&#252;ne ince ayar yapar. 0 = kapal&#305;. Gaza bas&#305;nca robot GER&#304; giderse Max E&#287;ilme &#351;imdilik ayn&#305; kals&#305;n, robot kodunda THROTTLE_DIR = -1 yap.</div>
  <button class="btn-send" onclick="sendDrive()">S&#252;r&#252;&#351; G&#246;nder</button>
</div>

<div class="card">
  <h3>Denge Ayarlar&#305;</h3>
  <div class="num-row"><span class="num-label">Balance Angle (&deg;)</span><input type="number" id="balanceAngle" value="" step="0.1" placeholder="--" oninput="markIdle('balanceAngle')"><span class="confirm-tick" id="tick-balanceAngle">&#8211;</span></div>
  <button class="btn-send" onclick="sendBalance()">G&#246;nder</button>
</div>

<div class="card">
  <h3>G&#252;venlik</h3>
  <button class="btn-stop" onclick="doStop()">&#9940; AC&#304;L DUR</button>
  <button class="btn-resume" onclick="doResume()">&#9654; Normale D&#246;n</button>
</div>

<div id="status">Ba&#287;lan&#305;yor...</div>

<script>
var PAD_SIZE=220, PAD_KNOB=64, PAD_R=(PAD_SIZE-PAD_KNOB)/2;
var DEADZONE=0.08;

var padBase=document.getElementById('joy-pad-base');
var padKnob=document.getElementById('joy-pad-knob');

var padActive=false;
var joyThr=0, joyStr=0;
var paramsSynced=false;
var quickStopped=false;

// ---------- Dur / Devam ----------
function updateQuickBtn(){
  var btn=document.getElementById('quickStopBtn');
  btn.classList.toggle('active', quickStopped);
  btn.innerHTML = quickStopped ? '&#9654;<br>DEVAM' : '&#9940;<br>DUR';
}
function doStop(){   quickStopped=true;  wsSend('s'); updateQuickBtn(); }
function doResume(){ quickStopped=false; wsSend('r'); updateQuickBtn(); }
function toggleQuickStop(){ quickStopped ? doResume() : doStop(); }

// ---------- WebSocket ----------
var ws=null, wsUp=false, rtt=0, pingSent=0;
function wsConnect(){
  ws=new WebSocket('ws://'+location.hostname+':81/');
  ws.onopen=function(){
    wsUp=true;
    document.getElementById('status').textContent='WS ba\u011fl\u0131 \u2713';
  };
  ws.onclose=function(){
    wsUp=false;
    document.getElementById('status').textContent='WS koptu, tekrar deneniyor...';
    var dot=document.getElementById('conn-dot'); if(dot) dot.className='dot off';
    document.getElementById('conn-txt').textContent='Ba\u011flant\u0131 Yok';
    setTimeout(wsConnect,700);
  };
  ws.onerror=function(){ try{ws.close();}catch(e){} };
  ws.onmessage=function(ev){
    var m=ev.data;
    if(m.charAt(0)==='T'){ rtt=Date.now()-parseInt(m.substring(2),10); return; }
    var d;
    try{ d=JSON.parse(m); }catch(e){ return; }
    render(d);
  };
}
function wsSend(s){ if(ws && ws.readyState===1) ws.send(s); }
wsConnect();

setInterval(function(){
  if(!wsUp) return;
  wsSend('j '+joyThr.toFixed(3)+' '+joyStr.toFixed(3));
}, 50);

setInterval(function(){
  if(!wsUp) return;
  pingSent=Date.now();
  wsSend('t '+pingSent);
}, 500);

// ---------- Joystick (tek pad: x=donus, y=gaz) ----------
function padUpdate(clientX, clientY){
  var rect=padBase.getBoundingClientRect();
  var cx=rect.left+PAD_SIZE/2;
  var cy=rect.top+PAD_SIZE/2;
  var dx=clientX-cx;
  var dy=clientY-cy;
  var dist=Math.sqrt(dx*dx+dy*dy);
  if(dist>PAD_R){ var k=PAD_R/dist; dx*=k; dy*=k; }
  padKnob.style.left=(PAD_SIZE/2-PAD_KNOB/2+dx)+'px';
  padKnob.style.top =(PAD_SIZE/2-PAD_KNOB/2+dy)+'px';
  var x=dx/PAD_R;    // -1(sol)..+1(sag)
  var y=-dy/PAD_R;   // -1(geri)..+1(ileri) - ekran Y'si asagi pozitif oldugu icin ters cevriliyor
  if(Math.abs(x)<DEADZONE) x=0;
  if(Math.abs(y)<DEADZONE) y=0;
  joyStr=x; joyThr=y;
  document.getElementById('v_thr').textContent=y.toFixed(2);
  document.getElementById('v_str').textContent=x.toFixed(2);
}
function padStart(e){ padActive=true; padKnob.classList.add('active'); var t=e.touches?e.touches[0]:e; padUpdate(t.clientX,t.clientY); e.preventDefault(); }
function padMove(e){ if(!padActive) return; var t=e.touches?e.touches[0]:e; padUpdate(t.clientX,t.clientY); e.preventDefault(); }
function padEnd(){
  if(!padActive) return;
  padActive=false; padKnob.classList.remove('active');
  padKnob.style.left=(PAD_SIZE/2-PAD_KNOB/2)+'px';
  padKnob.style.top =(PAD_SIZE/2-PAD_KNOB/2)+'px';
  joyThr=0; joyStr=0;
  document.getElementById('v_thr').textContent='0.00';
  document.getElementById('v_str').textContent='0.00';
}

padBase.addEventListener('touchstart', padStart, {passive:false});
padBase.addEventListener('touchmove',  padMove,  {passive:false});
padBase.addEventListener('touchend',   padEnd);
padBase.addEventListener('touchcancel',padEnd);
padBase.addEventListener('mousedown',  padStart);
document.addEventListener('mousemove', padMove);
document.addEventListener('mouseup',   padEnd);

// ---------- Parametreler ----------
var DECIMALS={kp:1, kd:2, ki:0, kpv:2, kiv:1, dmax:3, cur:0};

// ---------- Hz <-> m/s donusumu ----------
// Tekerlek: 110mm cap, motor: 1.8deg/adim (200 adim/tur), 16 mikroadim
// -> 3200 mikroadim/tur. Robot protokolu HALA Hz kullanir (PROTOKOL
// DEGISMEDI); sadece bu arayuzde m/s girip Hz'e cevirip yolluyoruz.
var STEPS_PER_REV = 3200;
var WHEEL_DIAM_M  = 0.110;
var HZ_PER_MPS = STEPS_PER_REV / (Math.PI * WHEEL_DIAM_M);
function mpsToHz(mps){ return mps * HZ_PER_MPS; }
function hzToMps(hz){  return hz / HZ_PER_MPS; }

// ---------- Gonderim onay tikleri ----------
// Her slider/alan icin son gonderilen deger 'sentVal' icinde tutulur.
// Robottan gelen telemetri, o degeri yankiladiginda (CONFIRM_MAP alaniyla
// eslesince) tik yesile doner -> robot degeri gercekten aldi ve isledi.
var sentVal={};
function setTick(key, state){
  var el=document.getElementById('tick-'+key);
  if(!el) return;
  el.className='confirm-tick '+state;
  el.textContent = state==='ok' ? '✓' : (state==='pending' ? '…' : '–');
}
function markSent(key, val){
  sentVal[key]=parseFloat(val);
  setTick(key,'pending');
}
function markIdle(key){
  delete sentVal[key];   // eski onaylanmis deger silinmezse, robotun degismemis
  setTick(key,'idle');   // eski degeri ile yanlislikla yesile donebilir
}
var CONFIRM_MAP={
  kp:'kpStill', kd:'kdStill', ki:'kiAngle',
  kpv:'kpVel', kiv:'kiVel',
  dmax:'driveMax', maxlean:'maxLean', daccel:'driveAcc',
  cur:'cur', fflean:'ffMax', balanceAngle:'balAngle'
};
function checkConfirmations(d){
  for(var key in CONFIRM_MAP){
    if(sentVal[key]===undefined) continue;
    var field=CONFIRM_MAP[key];
    if(d[field]===undefined) continue;
    var tol=(key==='dmax'||key==='daccel'||key==='cur') ? 1.5 : 0.06;
    if(Math.abs(d[field]-sentVal[key])<tol) setTick(key,'ok');
  }
}

// Akim slider'i CANLI gonderir (buton beklemeden), ama cok sik paket
// gitmesin diye 80 ms throttle.
var lastCurMs=0;
function liveCurrent(){
  var now=Date.now();
  if(now-lastCurMs<80) return;
  lastCurMs=now;
  var v=getVal('cur');
  wsSend('c '+v);
  markSent('cur', v);
}
function setSlider(id, v){
  document.getElementById(id).value=v;
  var numEl=document.getElementById(id+'-num');
  if(numEl) numEl.value=parseFloat(v).toFixed(DECIMALS[id]);
}
function syncNum(id){
  var numEl=document.getElementById(id+'-num');
  if(numEl) numEl.value=document.getElementById(id).value;
}
function numToSlider(id){
  var numEl=document.getElementById(id+'-num');
  var v=parseFloat(numEl.value);
  if(isNaN(v)) return;
  document.getElementById(id).value=v;
}
function getVal(id){
  var numEl=document.getElementById(id+'-num');
  if(numEl && numEl.value!=='') return numEl.value;
  return document.getElementById(id).value;
}
function syncParams(d){
  setSlider('kp',   d.kpStill); sentVal.kp=d.kpStill;   setTick('kp','ok');
  setSlider('kd',   d.kdStill); sentVal.kd=d.kdStill;   setTick('kd','ok');
  setSlider('ki',   d.kiAngle); sentVal.ki=d.kiAngle;   setTick('ki','ok');
  setSlider('kpv',  d.kpVel);   sentVal.kpv=d.kpVel;    setTick('kpv','ok');
  setSlider('kiv',  d.kiVel);   sentVal.kiv=d.kiVel;    setTick('kiv','ok');
  setSlider('dmax', hzToMps(d.driveMax).toFixed(3));sentVal.dmax=d.driveMax;setTick('dmax','ok');
  document.getElementById('balanceAngle').value=d.balAngle.toFixed(1);
  sentVal.balanceAngle=d.balAngle; setTick('balanceAngle','ok');
  if(d.maxLean!==undefined){
    document.getElementById('maxlean').value=d.maxLean.toFixed(1);
    sentVal.maxlean=d.maxLean; setTick('maxlean','ok');
  }
  if(d.driveAcc!==undefined && d.driveAcc>0){
    document.getElementById('daccel').value=d.driveAcc.toFixed(0);
    sentVal.daccel=d.driveAcc; setTick('daccel','ok');
  }
  if(d.cur!==undefined && d.cur>0){
    setSlider('cur', d.cur);
    sentVal.cur=d.cur; setTick('cur','ok');
  }
  if(d.ffMax!==undefined && d.ffMax>=0){
    document.getElementById('fflean').value=d.ffMax.toFixed(1);
    sentVal.fflean=d.ffMax; setTick('fflean','ok');
  }
  document.getElementById('sync-note').textContent='Robottan al\u0131nd\u0131 \u2713';
  paramsSynced=true;
}
function pullParams(){ paramsSynced=false; }

function sendPID(){
  var kp=getVal('kp'), kd=getVal('kd'), ki=getVal('ki');
  wsSend('p '+kp+' '+kd+' '+ki);
  markSent('kp',kp); markSent('kd',kd); markSent('ki',ki);
}
function sendVelPID(){
  var kpv=getVal('kpv'), kiv=getVal('kiv');
  wsSend('v '+kpv+' '+kiv);
  markSent('kpv',kpv); markSent('kiv',kiv);
}
function setSpeedMode(mps){
  setSlider('dmax', mps.toFixed(3));
  sendDrive();
}

// ---------- Daire cizme ----------
// Tekerlekler arasi mesafe (merkezden merkeze), fiziksel olcum: 259mm.
// NOT: cap yaricapi (W/2=12.95cm) altina inen daireler ic tekerlegin
// GERI donmesini gerektiriyor (kaymaya/kararsizliga acik) - o yuzden
// butonlar hep D >= ~30cm ile, iki tekerlek de HEP ILERI kalacak
// sekilde seciliyor. Gercek fiziksel olcu kullaniliyor, kayma icin
// yapay buyutme yapilmadi.
var TRACK_WIDTH_M = 0.259;

// Robot tarafindaki TURN_AUTHORITY_HZ ile AYNI kalmali (main.cpp).
// 4000 guvenlik sorunu yarattigi icin 500'e geri cekildi.
var HZ_PER_TURN_AUTHORITY = 500;

// dir: +1 = saga daire, -1 = sola daire. speedMps: bu daire icin guvenli
// sabit hiz (buton kendi hizini tasir, o an secili Hiz Modu'na bakmaz).
function driveCircle(diameterM, dir, speedMps){
  setSlider('dmax', speedMps.toFixed(3));
  sendDrive();   // robotun MaxHz'ini bu hiza sabitle (throttle=1 -> tam bu hiz)

  var R  = diameterM / 2;
  var dv = speedMps * TRACK_WIDTH_M / R;   // ic/dis tekerlek hiz farki (m/s)
  // Guvenlik: ic tekerlek asla negatife (geri donme) dusmesin - fark
  // en fazla 2xspeedMps olsun, yoksa ic tekerlek hizi = speedMps - dv/2 < 0 olur.
  if (dv > 2 * speedMps) dv = 2 * speedMps;
  var dHz = dv * HZ_PER_MPS;               // Hz farkina cevir
  var steer = dir * (dHz / (2 * HZ_PER_TURN_AUTHORITY));   // turning formulunun tersi
  steer = Math.max(-1, Math.min(1, steer));

  joyThr = 1.0;
  joyStr = steer;
  document.getElementById('v_thr').textContent = joyThr.toFixed(2);
  document.getElementById('v_str').textContent = joyStr.toFixed(2);
}
function stopCircle(){
  joyThr = 0; joyStr = 0;
  document.getElementById('v_thr').textContent = '0.00';
  document.getElementById('v_str').textContent = '0.00';
  padKnob.style.left = (PAD_SIZE/2 - PAD_KNOB/2) + 'px';
  padKnob.style.top  = (PAD_SIZE/2 - PAD_KNOB/2) + 'px';
}
function sendDrive(){
  var dmaxMps=parseFloat(getVal('dmax'));
  var dmaxHz=Math.round(mpsToHz(dmaxMps));
  var ml=document.getElementById('maxlean').value; if(ml==='') ml='4';
  var da=document.getElementById('daccel').value;  if(da==='') da='4000';
  wsSend('d '+dmaxHz+' '+ml+' '+da);
  markSent('dmax',dmaxHz); markSent('maxlean',ml); markSent('daccel',da);

  var cur=getVal('cur');
  wsSend('c '+cur);   // akimi da yolla
  markSent('cur',cur);

  var ff=document.getElementById('fflean').value; if(ff==='') ff='-1';  // sentinel
  wsSend('f '+ff);              // FF lean
  if(ff!=='-1') markSent('fflean',ff); else setTick('fflean','idle');
}
function sendBalance(){
  var ba=document.getElementById('balanceAngle').value; if(ba==='') ba='-999';
  wsSend('b '+ba);
  if(ba!=='-999') markSent('balanceAngle',ba); else setTick('balanceAngle','idle');
}

// ---------- Ekran ----------
function render(d){
  document.getElementById('v_angle').textContent =d.angle.toFixed(2);
  document.getElementById('v_target').textContent=d.target.toFixed(2);
  document.getElementById('v_error').textContent =d.error.toFixed(2);
  document.getElementById('v_speed').textContent =Math.round(d.speed);
  document.getElementById('v_fms').textContent   =Math.round(d.fMS);
  document.getElementById('v_tgt').textContent   =Math.round(d.tgt);
  document.getElementById('v_vtilt').textContent =d.tilt.toFixed(2);
  document.getElementById('v_vint').textContent  =d.vint.toFixed(0);
  document.getElementById('v_scale').textContent =d.scale.toFixed(2)+'x';
  document.getElementById('v_rtt').textContent   =rtt;
  document.getElementById('v_loss').textContent  =d.loss.toFixed(1)+'%';
  document.getElementById('v_cmdage').textContent=d.cmdAge;

  // --- cascade teshis ---
  var it=document.getElementById('v_iterm');
  it.textContent=Math.round(d.iTerm);
  it.style.color=(Math.abs(d.iTerm)>240)?'#f38ba8':'#a6e3a1';
  document.getElementById('v_ramp').textContent=Math.round(d.ramp);
  document.getElementById('v_ff').textContent=(d.ffDeg!==undefined?d.ffDeg.toFixed(2):'--');
  var cur=document.getElementById('v_cur');
  cur.textContent=Math.round(d.cur);
  cur.style.color=(d.cur<350)?'#f38ba8':'#a6e3a1';   // cok dusukse uyar

  var lk=document.getElementById('v_link');
  lk.textContent=d.age;
  lk.style.color=(d.age<300)?'#a6e3a1':'#f38ba8';

  var f=document.getElementById('v_frz');
  f.textContent=d.frz?'DON':'ok';
  f.style.color=d.frz?'#f38ba8':'#a6e3a1';

  var linkOK=d.seen && (d.age<500);
  var dot=document.getElementById('conn-dot'); if(dot) dot.className=linkOK?'dot':'dot off';
  document.getElementById('conn-txt').textContent=linkOK?'Robot Ba\u011fl\u0131':'Robot Yok';

  if(!paramsSynced && d.seen) syncParams(d);
  checkConfirmations(d);
}
</script>
</body>
</html>
)rawliteral";

void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

// ==================== WEBSOCKET ====================
static int parseArgs(char *s, float *out, int maxn) {
  int n = 0;
  char *tok = strtok(s, " ,");
  while (tok && n < maxn) { out[n++] = atof(tok); tok = strtok(NULL, " ,"); }
  return n;
}

void wsEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t len) {
  if (type == WStype_CONNECTED) {
    Serial.print("WS client baglandi #"); Serial.println(num);
    return;
  }
  if (type == WStype_DISCONNECTED) {
    joyY = 0.0f; joyX = 0.0f;
    Serial.print("WS client ayrildi #"); Serial.println(num);
    return;
  }
  if (type != WStype_TEXT || len < 1) return;

  char *p = (char *)payload;
  char  c = p[0];
  float a[4];

  switch (c) {
    case 'j': {
      if (parseArgs(p + 1, a, 2) == 2) {
        joyY = constrain(a[0], -1.0f, 1.0f);
        joyX = constrain(a[1], -1.0f, 1.0f);
      }
      break;
    }
    case 't': {
      String r = "T "; r += (p + 2);
      wsServer.sendTXT(num, r);
      break;
    }
    case 'p': {
      if (parseArgs(p + 1, a, 3) == 3) { komut.kp = a[0]; komut.kd = a[1]; komut.ki = a[2]; }
      break;
    }
    case 'v': {
      if (parseArgs(p + 1, a, 2) == 2) { komut.kp_vel = a[0]; komut.ki_vel = a[1]; }
      break;
    }
    case 'd': {                                   // v181: 3 arguman
      int n = parseArgs(p + 1, a, 3);
      if (n >= 2) { komut.drive_max_hz = a[0]; komut.maxLean = a[1]; }
      if (n >= 3) { komut.drive_accel  = a[2]; }
      break;
    }
    case 'b': {
      if (parseArgs(p + 1, a, 1) == 1) komut.balanceAngle = a[0];
      break;
    }
    case 'c': {                                   // v182: motor akimi (mA)
      if (parseArgs(p + 1, a, 1) == 1) komut.kp_drive = a[0];
      break;
    }
    case 'f': {                                   // v183: FF lean (derece)
      if (parseArgs(p + 1, a, 1) == 1) komut.ff_cruise = a[0];
      break;
    }
    case 's': komut.stop = true;  joyY = 0; joyX = 0; break;
    case 'r': komut.stop = false; joyY = 0; joyX = 0; break;
  }
}

void broadcastTelemetry() {
  TelemetriPaketi snap;
  bool     seen;
  uint32_t lastMs;
  portENTER_CRITICAL(&telemMux);
  memcpy(&snap, &lastTelem, sizeof(TelemetriPaketi));
  seen   = telemEverSeen;
  lastMs = lastTelemLocalMs;
  portEXIT_CRITICAL(&telemMux);

  uint32_t age = seen ? (millis() - lastMs) : 99999;

  String j = "{";
  j += "\"angle\":"    + String(snap.angle,        3) + ",";
  j += "\"target\":"   + String(snap.target_angle, 3) + ",";
  j += "\"error\":"    + String(snap.error,        3) + ",";
  j += "\"speed\":"    + String(snap.motorSpeed,   1) + ",";
  j += "\"fMS\":"      + String(snap.fMS,          1) + ",";
  j += "\"tgt\":"      + String(snap.tgtMS,        1) + ",";
  j += "\"vint\":"     + String(snap.velIntegral,  1) + ",";
  j += "\"tilt\":"     + String(snap.tiltRaw,      3) + ",";
  j += "\"frz\":"      + String(snap.frozen ? "true" : "false") + ",";
  j += "\"age\":"      + String(age) + ",";
  j += "\"cmdAge\":"   + String(snap.cmdAge) + ",";
  j += "\"loss\":"     + String(txLossPct, 1) + ",";
  // --- v181 cascade teshis ---
  j += "\"iTerm\":"    + String(snap.driveIntegral, 1) + ",";
  j += "\"ramp\":"     + String(snap.driveHz,       1) + ",";
  j += "\"vErr\":"     + String(snap.driveErrorHz,  1) + ",";
  j += "\"driveAcc\":" + String(snap.driveAccelHz,  0) + ",";
  j += "\"cur\":"      + String(snap.ffBreak,       0) + ",";
  j += "\"ffDeg\":"    + String(snap.ffDeg,         2) + ",";
  j += "\"kpStill\":"  + String(snap.kpStill,      1) + ",";
  j += "\"kdStill\":"  + String(snap.kdStill,      1) + ",";
  j += "\"kiAngle\":"  + String(snap.kiAngle,      2) + ",";
  j += "\"kpVel\":"    + String(snap.kpVel,        3) + ",";
  j += "\"kiVel\":"    + String(snap.kiVel,        2) + ",";
  j += "\"driveMax\":" + String(snap.driveMaxHz,   0) + ",";
  j += "\"balAngle\":" + String(snap.balAngle,     2) + ",";
  j += "\"maxLean\":"  + String(snap.maxLean,      1) + ",";
  j += "\"scale\":"    + String(snap.angleScale,   2) + ",";
  j += "\"seen\":"     + String(seen ? "true" : "false");
  j += "}";
  wsServer.broadcastTXT(j);
}

// ==================== TASK'LAR ====================
void webTask(void *param) {
  uint32_t lastBcast = 0;
  for (;;) {
    server.handleClient();
    wsServer.loop();
    if (millis() - lastBcast >= 100) {
      lastBcast = millis();
      broadcastTelemetry();
    }
    vTaskDelay(1);
  }
}

void radioTask(void *param) {
  TickType_t last = xTaskGetTickCount();
  const TickType_t per = pdMS_TO_TICKS(50);
  for (;;) {
    komut.targetAngle = joyY;
    komut.steering    = joyX;
    esp_now_send(robotMAC, (uint8_t *)&komut, sizeof(komut));
    vTaskDelayUntil(&last, per);
  }
}

void serialPrintTask(void *param) {
  bool     linkWarned = false;
  uint32_t downSince  = 0;

  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(200));
    TelemetriPaketi snap;
    bool     seen;
    uint32_t lastMs;
    portENTER_CRITICAL(&telemMux);
    memcpy(&snap, &lastTelem, sizeof(TelemetriPaketi));
    seen   = telemEverSeen;
    lastMs = lastTelemLocalMs;
    portEXIT_CRITICAL(&telemMux);

    uint32_t age = seen ? (millis() - lastMs) : 99999;

    if (seen) {
      if (age > 300 && !linkWarned) {
        linkWarned = true; downSince = lastMs;
        Serial.println("!!! LINK KOPTU");
      } else if (age <= 300 && linkWarned) {
        linkWarned = false;
        Serial.print("*** LINK GERI GELDI (kesinti ~");
        Serial.print(lastMs - downSince);
        Serial.println(" ms)");
      }
    }

    Serial.print("A:");        Serial.print(snap.angle,         2);
    Serial.print(" | T:");     Serial.print(snap.target_angle,  2);
    Serial.print(" | E:");     Serial.print(snap.error,         2);
    Serial.print(" | Spd:");   Serial.print(snap.motorSpeed,    0);
    Serial.print(" | fMS:");   Serial.print(snap.fMS,           0);
    Serial.print(" | ramp:");  Serial.print(snap.driveHz,       0);
    Serial.print(" | vErr:");  Serial.print(snap.driveErrorHz,  0);
    Serial.print(" | iT:");    Serial.print(snap.driveIntegral, 0);
    Serial.print(" | vTilt:"); Serial.print(snap.tiltRaw,       2);
    Serial.print(" | Frz:");   Serial.print(snap.frozen ? "DON" : "ok");
    Serial.print(" | Cmd:");   Serial.print(snap.cmdAge);
    Serial.print(" | Link:");  Serial.print(age);
    Serial.print(" | TXloss:");Serial.print(txLossPct, 1); Serial.print("%");
    Serial.print(" | ms:");    Serial.println(snap.ms);
  }
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("KUMANDA v181 (WebSocket + cascade teshis)");

  komut.targetAngle  = 0.0;
  komut.steering     = 0.0;
  komut.kp           = 0.0;
  komut.kd           = 0.0;
  komut.ki           = 0.0;
  komut.stop         = false;
  komut.balanceAngle = -999.0;
  komut.maxLean      = 0.0;
  komut.kp_vel       = 0.0;
  komut.ki_vel       = 0.0;
  komut.ff_break     = 0.0;
  komut.ff_cruise    = -1.0;   // v183: sentinel - robot kendi FF varsayilanini korur
  komut.drive_max_hz = 0.0;
  komut.drive_accel  = 0.0;
  komut.kp_drive     = 0.0;
  komut.ki_drive     = 0.0;

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASS, WIFI_CH);

  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);

  Serial.print("Kumanda STA MAC (robotta pcMAC bu olmali): ");
  Serial.println(WiFi.macAddress());
  Serial.print("AP IP: "); Serial.println(WiFi.softAPIP());
  Serial.print("Kanal: "); Serial.println(WIFI_CH);

  server.on("/", handleRoot);
  server.begin();

  wsServer.begin();
  wsServer.onEvent(wsEvent);

  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init HATASI"); while (1); }
  esp_now_register_recv_cb(espnowAlCallback);
  esp_now_register_send_cb(espnowGonderCallback);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, robotMAC, 6);
  peerInfo.channel = WIFI_CH;
  peerInfo.ifidx   = WIFI_IF_STA;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) { Serial.println("Peer eklenemedi"); while (1); }

  xTaskCreatePinnedToCore(webTask,         "webTask",    12288, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(serialPrintTask, "serialTask",  4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(radioTask,       "radioTask",   4096, NULL, 3, NULL, 1);
}

void loop() { vTaskDelay(1000); }

// ==================== ESP-IDF GIRIS NOKTASI ====================
extern "C" void app_main() {
  initArduino();
  setup();
  for (;;) {
    loop();
  }
}
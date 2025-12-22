#include "driver/uart.h"
#include <WiFi.h>
#include <WebServer.h>

// --- WIFI SETTINGS ---
const char* ssid = "Comm_Relay_Manager"; 
const char* password = ""; 

WebServer server(80);

// --- HARDWARE CONFIG ---
#define BUFFER_EN_PIN 25
#define JR_TX_PIN 17
#define JR_RX_PIN 16
#define JR_UART UART_NUM_1

#define RE_TX_PIN 32  
#define RE_RX_PIN 33  
#define RE_UART UART_NUM_2

#define BUF_SIZE 1024
#define CRC_POLY 0xD5

// --- CONSTANTS ---
#define CRSF_SYNC_STICK     0xEE 
#define CRSF_SYNC_TELEM     0xC8 
#define CRSF_SYNC_RADIO     0xEA 
#define CRSF_TYPE_STICKS    0x16 
#define CRSF_TYPE_LINKSTAT  0x14 
#define CRSF_TYPE_HEARTBEAT 0x3A 

// --- GLOBALS ---
volatile uint16_t currentChannels[16];   
volatile int disp_rssi = -130; 
volatile int disp_lq = 0;
volatile int disp_snr = 0;

// TIMERS
unsigned long lastPacketTime = 0;    
unsigned long lastRcInputTime = 0;   
unsigned long lastTelemTime = 0;     

TaskHandle_t RadioTaskHandle = NULL; 
volatile int throttleScale = 100; 

// --- HTML DASHBOARD (NO EMOJIS / COMPACT MOBILE) ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>Relay node Manager</title>
  <style>
    /* RESET & THEME */
    * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
    body { font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif; background: #f2f4f6; color: #333; height: 100vh; display: flex; flex-direction: column; }
    :root { --blue: #0066cc; --green: #28a745; --red: #dc3545; --bg: #fff; }

    /* LAYOUT - MOBILE FIRST DESIGN */
    .top-bar {
        background: #fff;
        padding: 15px;
        border-bottom: 1px solid #ddd;
        display: flex;
        justify-content: space-between;
        align-items: center;
        position: sticky;
        top: 0;
        z-index: 100;
    }

    .brand { font-size: 1.1rem; font-weight: 900; color: #000; letter-spacing: -0.5px; text-transform: uppercase; }
    
    .nav { display: flex; gap: 15px; }
    .nav-link { 
        font-size: 0.9rem; 
        font-weight: 700; 
        color: #999; 
        text-decoration: none; 
        cursor: pointer;
        padding: 5px 10px;
        border-radius: 6px;
        transition: 0.2s;
    }
    .nav-link.active { color: #000; background: #eee; }

    .main { flex: 1; padding: 15px; overflow-y: auto; display: none; }
    .main.active { display: block; }

    /* CARDS */
    .card { background: #fff; padding: 15px; border-radius: 12px; box-shadow: 0 2px 8px rgba(0,0,0,0.04); margin-bottom: 15px; border: 1px solid #e5e5e5; }
    .card-head { font-size: 0.7rem; color: #888; font-weight: 700; text-transform: uppercase; margin-bottom: 5px; letter-spacing: 0.5px; }
    .card-val { font-size: 1.8rem; font-weight: 800; color: #222; letter-spacing: -1px; }
    .unit { font-size: 0.9rem; font-weight: 600; color: #999; margin-left: 2px; }

    /* STATS ROW (SIDE BY SIDE) */
    .stats-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 15px; }
    .stats-card { background: #fff; padding: 15px; border-radius: 12px; border: 1px solid #e5e5e5; text-align: center; }
    .stats-card .card-val { font-size: 1.6rem; }

    /* STATUS BADGE */
    .status-badge { 
        padding: 4px 10px; 
        border-radius: 4px; 
        font-size: 0.7rem; 
        font-weight: 700; 
        text-transform: uppercase; 
        background: #eee; 
        color: #666; 
        display: inline-block;
    }

    /* GRAPH */
    .chart-box { height: 140px; width: 100%; position: relative; }
    canvas { width: 100%; height: 100%; }

    /* CHANNELS */
    .ch-row { display: flex; align-items: center; margin-bottom: 6px; height: 18px; }
    .ch-lbl { width: 30px; font-size: 0.65rem; color: #999; font-weight: 600; }
    .ch-track { flex: 1; height: 6px; background: #f0f0f0; border-radius: 3px; position: relative; overflow: hidden; }
    .ch-bar { height: 100%; background: var(--blue); width: 50%; border-radius: 3px; }
    .ch-center { position: absolute; left: 50%; top:0; bottom:0; width: 1px; background: #ddd; z-index: 5; }

    /* SLIDER */
    input[type=range] { width: 100%; margin: 20px 0; height: 6px; background: #ddd; border-radius: 3px; outline: none; -webkit-appearance: none; }
    input[type=range]::-webkit-slider-thumb { -webkit-appearance: none; width: 24px; height: 24px; background: var(--blue); border-radius: 50%; border: 3px solid #fff; box-shadow: 0 2px 5px rgba(0,0,0,0.2); }
    .limit-display { text-align: center; font-size: 3rem; font-weight: 800; color: var(--blue); }

    /* Desktop Tweaks */
    @media (min-width: 768px) {
        body { max-width: 600px; margin: 0 auto; border-left: 1px solid #eee; border-right: 1px solid #eee; }
    }
  </style>
</head>
<body>

  <div class="top-bar">
    <div class="brand">RELAY NODE MANAGER</div>
    <div class="nav">
      <div class="nav-link active" onclick="sw(0)">DASHBOARD</div>
      <div class="nav-link" onclick="sw(1)">LIMITS</div>
    </div>
  </div>

  <div id="t0" class="main active">
    
    <div style="display:flex; justify-content:space-between; align-items:center; margin-bottom:15px;">
        <div class="card-head" style="margin:0;">SYSTEM STATUS</div>
        <div id="status" class="status-badge">CONNECTING</div>
    </div>

    <div class="card" style="padding:10px;">
        <div class="card-head" style="margin-left:5px;">SIGNAL HISTORY</div>
        <div class="chart-box">
            <canvas id="gr"></canvas>
        </div>
    </div>

    <div class="stats-row">
        <div class="stats-card">
            <div class="card-head">LINK QUALITY</div>
            <div class="card-val" id="lq">--</div>
            <div class="unit">%</div>
        </div>
        <div class="stats-card">
            <div class="card-head">SIGNAL RSSI</div>
            <div class="card-val" id="rssi">--</div>
            <div class="unit">dBm</div>
        </div>
    </div>

    <div class="card">
        <div class="card-head">CHANNEL MONITOR</div>
        <div id="chs" style="margin-top:10px;"></div>
    </div>
  </div>

  <div id="t1" class="main">
    <div class="card">
        <div class="card-head">THROTTLE OUTPUT LIMIT</div>
        <div class="limit-display"><span id="ld">100</span>%</div>
        <input type="range" min="0" max="100" value="100" id="ls" oninput="ul(this.value)">
        <p style="text-align:center; color:#999; font-size:0.8rem; margin-top:10px;">
            Reduces maximum throttle for training safety.
        </p>
    </div>
  </div>

<script>
  // NAVIGATION
  function sw(i) {
    document.querySelectorAll('.main').forEach(e => e.classList.remove('active'));
    document.querySelectorAll('.nav-link').forEach(e => e.classList.remove('active'));
    document.getElementById('t'+i).classList.add('active');
    document.querySelectorAll('.nav-link')[i].classList.add('active');
  }

  // LIMITER
  function ul(v) {
    document.getElementById('ld').innerText = v;
    fetch('/set_limit?val=' + v);
  }

  // BUILD CHANNELS
  let h = '';
  for(let i=0; i<16; i++) {
    h += `<div class="ch-row"><div class="ch-lbl">CH${i+1}</div><div class="ch-track"><div class="ch-center"></div><div class="ch-bar" id="b${i}"></div></div></div>`;
  }
  document.getElementById('chs').innerHTML = h;

  // GRAPH SETUP
  const cvs = document.getElementById('gr');
  const ctx = cvs.getContext('2d');
  const dpr = window.devicePixelRatio || 1;
  
  function resize() {
      const r = cvs.getBoundingClientRect();
      cvs.width = r.width * dpr;
      cvs.height = r.height * dpr;
      ctx.scale(dpr, dpr);
  }
  window.addEventListener('resize', resize);
  setTimeout(resize, 100); // Init delay

  let hist = new Array(100).fill(-130);

  function draw() {
    const r = cvs.getBoundingClientRect();
    const w = r.width;
    const h = r.height;
    ctx.clearRect(0, 0, w, h);
    
    // Grid Lines
    ctx.strokeStyle = '#f0f0f0'; ctx.lineWidth = 1;
    let y1 = h - ((-60 + 130) / 110) * h;
    ctx.beginPath(); ctx.moveTo(0, y1); ctx.lineTo(w, y1); ctx.stroke();
    
    // Line
    ctx.strokeStyle = '#0066cc'; ctx.lineWidth = 2; ctx.lineJoin = 'round';
    ctx.beginPath();
    let step = w / (hist.length - 1);
    for(let i=0; i<hist.length; i++) {
        let v = hist[i];
        if(v < -130) v = -130; if(v > -20) v = -20;
        let y = h - ((v + 130) / 110) * h;
        if(i===0) ctx.moveTo(0, y); else ctx.lineTo(i*step, y);
    }
    ctx.stroke();
  }

  // DATA LOOP
  setInterval(() => {
    fetch("/data").then(r => r.json()).then(d => {
      document.getElementById("lq").innerText = d.lq;
      document.getElementById("rssi").innerText = d.rssi;
      
      const st = document.getElementById("status");
      if(d.status === 0) { st.innerText = "RC LOST"; st.style.background="#ffebee"; st.style.color="#dc3545"; }
      else if(d.status === 1) { st.innerText = "MODULE OFF"; st.style.background="#fff3cd"; st.style.color="#856404"; }
      else if(d.status === 2) { st.innerText = "DRONE LOST"; st.style.background="#cce5ff"; st.style.color="#004085"; }
      else { st.innerText = "ONLINE"; st.style.background="#d4edda"; st.style.color="#155724"; }

      d.channels.forEach((v, i) => {
         let p = (v / 2047) * 100;
         let el = document.getElementById("b"+i);
         if(el) el.style.width = p + "%";
      });

      hist.push(d.rssi);
      hist.shift();
      draw();
    }).catch(e => console.log(e));
  }, 100);
</script>
</body>
</html>
)rawliteral";

// --- RADIO LOGIC (UNCHANGED) ---
uint8_t calc_crc8(const uint8_t* payload, int length) {
  uint8_t crc = 0;
  for (int i = 0; i < length; i++) {
    crc ^= payload[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ CRC_POLY;
      else crc = crc << 1;
    }
  }
  return crc & 0xFF;
}

void handleRoot() { server.send(200, "text/html; charset=utf-8", index_html); }
void handleLimit() { if (server.hasArg("val")) throttleScale = server.arg("val").toInt(); server.send(200, "text/plain", "OK"); }

void handleData() {
  int statusCode = 0;
  unsigned long now = millis();
  if (now - lastRcInputTime < 100) {
      if (now - lastPacketTime < 100) {
          if (now - lastTelemTime < 1000) statusCode = 3; 
          else statusCode = 2; 
      } else statusCode = 1; 
  } else statusCode = 0; 

  String json = "{";
  json += "\"rssi\":" + String(disp_rssi) + ",";
  json += "\"lq\":" + String(disp_lq) + ",";
  json += "\"status\":" + String(statusCode) + ",";
  json += "\"channels\":[";
  for(int i=0; i<16; i++) { json += String(currentChannels[i]); if(i < 15) json += ","; }
  json += "]}";
  server.send(200, "application/json", json);
}

void packCrsfChannels(uint8_t* payload, const volatile uint16_t* channels) {
    memset(payload, 0, 22);
    unsigned int bitIndex = 0;
    for (int i = 0; i < 16; i++) {
        uint32_t value = channels[i];
        if (i == 2 && throttleScale < 100) {
             long zero = 172; 
             if (value < zero) value = zero;
             long range = value - zero;
             range = (range * throttleScale) / 100;
             value = zero + range;
        }
        for (int b = 0; b < 11; b++) {
            if (value & (1 << b)) payload[bitIndex / 8] |= (1 << (bitIndex % 8));
            bitIndex++;
        }
    }
}

void sendStickPacket() {
  uint8_t packet[26];
  packet[0] = CRSF_SYNC_STICK; packet[1] = 24; packet[2] = CRSF_TYPE_STICKS;
  packCrsfChannels(&packet[3], currentChannels);
  packet[25] = calc_crc8(&packet[2], 23);
  digitalWrite(BUFFER_EN_PIN, LOW); // TX
  delayMicroseconds(2);
  uart_write_bytes(JR_UART, packet, 26);
  delayMicroseconds(680); 
  digitalWrite(BUFFER_EN_PIN, HIGH); // RX
  lastPacketTime = millis();
}

void processReceiverInput() {
  size_t len = 0; uart_get_buffered_data_len(RE_UART, &len);
  if (len < 26) return; 
  uint8_t buf[len]; uart_read_bytes(RE_UART, buf, len, 0); 
  for(int i=0; i < len - 4; i++) {
    if((buf[i] == CRSF_SYNC_STICK || buf[i] == CRSF_SYNC_TELEM) && buf[i+2] == CRSF_TYPE_STICKS) {
        lastRcInputTime = millis();
        uint8_t* p = &buf[i+3];
        unsigned int bIdx = 0;
        for (int ch = 0; ch < 16; ch++) {
            currentChannels[ch] = 0; 
            for (int b = 0; b < 11; b++) {
                int byteIdx = bIdx / 8; int bitPos = bIdx % 8;
                if (p[byteIdx] & (1 << bitPos)) currentChannels[ch] |= (1 << b);
                bIdx++;
            }
        }
    }
  }
}

void processJrModule() {
  size_t len = 0; uart_get_buffered_data_len(JR_UART, &len);
  if(len > 0) {
      uint8_t buf[len]; uart_read_bytes(JR_UART, buf, len, 0); 
      for(int i=0; i<len; i++) {
          if(buf[i] == CRSF_SYNC_RADIO || buf[i] == CRSF_SYNC_TELEM) {
              if(i + 2 < len) {
                  uint8_t type = buf[i+2];
                  if (type == CRSF_TYPE_HEARTBEAT) sendStickPacket(); 
                  if (type == CRSF_TYPE_LINKSTAT && (i+13 <= len)) {
                      uint8_t* p = &buf[i+3];
                      disp_rssi = (int8_t)p[0]; disp_lq = p[2]; disp_snr = (int8_t)p[3];
                      lastTelemTime = millis();
                  }
              }
          }
      }
  }
}

void RadioTask(void * parameter) {
  for(;;) {
    processReceiverInput();
    processJrModule();
    if (millis() - lastPacketTime > 20) sendStickPacket();
    vTaskDelay(1 / portTICK_PERIOD_MS); 
  }
}

void setup() {
  pinMode(BUFFER_EN_PIN, OUTPUT); digitalWrite(BUFFER_EN_PIN, HIGH); 
  Serial.begin(115200);

  // Radio Config
  uart_config_t jr_c = { .baud_rate = 400000, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_APB };
  uart_driver_install(JR_UART, BUF_SIZE * 2, 0, 0, NULL, 0); uart_param_config(JR_UART, &jr_c);
  uart_set_pin(JR_UART, JR_TX_PIN, JR_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_line_inverse(JR_UART, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

  uart_config_t re_c = { .baud_rate = 420000, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_APB };
  uart_driver_install(RE_UART, BUF_SIZE * 2, 0, 0, NULL, 0); uart_param_config(RE_UART, &re_c);
  uart_set_pin(RE_UART, RE_TX_PIN, RE_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  
  for(int i=0; i<16; i++) currentChannels[i] = 992; sendStickPacket();
  xTaskCreatePinnedToCore(RadioTask, "RadioTask", 10000, NULL, 5, &RadioTaskHandle, 1);

  // WiFi
  WiFi.softAP(ssid, password);
  server.on("/", handleRoot);
  server.on("/set_limit", handleLimit);
  server.on("/data", handleData);
  server.begin();
}

void loop() {
  server.handleClient();
  delay(2);
}
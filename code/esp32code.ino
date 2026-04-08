#include "driver/uart.h"
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// --- DEFAULTS ---
#define DEF_SSID "GhostRadio_Manager"
#define DEF_PASS "" 

// --- OBJECTS ---
WebServer server(80);
Preferences preferences;

// --- CONFIG STRUCT ---
struct Config {
  // Hardware (Saved to Flash)
  int re_tx; int re_rx;
  int jr_tx; int jr_rx;
  int buf_en;
  int baud_re; int baud_jr;
  
  // Operational (RAM)
  int throttle_lim = 100; 
} config;

// --- HARDWARE ---
#define JR_UART UART_NUM_1
#define RE_UART UART_NUM_2
#define BUF_SIZE 1024
#define CRC_POLY 0xD5

// --- PROTOCOL ---
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
volatile int link_status = 0; 

unsigned long lastPacketTime = 0;    
unsigned long lastRcInputTime = 0;   
unsigned long lastTelemTime = 0;     

TaskHandle_t RadioTaskHandle = NULL; 

// --- HTML DASHBOARD ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <title>Ghost Manager</title>
  <style>
    * { box-sizing: border-box; margin: 0; padding: 0; -webkit-tap-highlight-color: transparent; }
    body { font-family: -apple-system, sans-serif; background: #f2f4f6; color: #333; height: 100vh; display: flex; flex-direction: column; }
    .top-bar { background: #fff; padding: 15px; border-bottom: 1px solid #ddd; display: flex; justify-content: space-between; align-items: center; }
    .brand { font-size: 1.1rem; font-weight: 900; color: #000; }
    .nav { display: flex; gap: 10px; }
    .nav-btn { font-size: 0.85rem; font-weight: 700; color: #888; padding: 6px 12px; border-radius: 6px; cursor: pointer; transition: 0.2s; }
    .nav-btn.active { color: #fff; background: #0066cc; }
    .main { flex: 1; padding: 15px; overflow-y: auto; display: none; }
    .main.active { display: block; }
    .card { background: #fff; padding: 15px; border-radius: 12px; margin-bottom: 15px; border: 1px solid #e5e5e5; }
    .card h4 { font-size: 0.75rem; color: #888; text-transform: uppercase; margin-bottom: 10px; }
    .input-group { margin-bottom: 15px; }
    .input-group label { display: block; font-size: 0.8rem; font-weight: 600; margin-bottom: 5px; color: #555; }
    .input-group input { width: 100%; padding: 10px; border: 1px solid #ddd; border-radius: 6px; }
    
    .btn { width: 100%; padding: 15px; font-weight: 700; border: none; border-radius: 8px; font-size: 1rem; cursor: pointer; margin-top:5px; }
    .btn-green { background: #28a745; color: #fff; }
    .btn-blue { background: #0066cc; color: #fff; margin-bottom: 15px; }
    
    input[type=range] { width: 100%; margin: 15px 0; height: 8px; background: #ddd; border-radius: 4px; outline: none; -webkit-appearance: none; }
    .lim-val { text-align: center; font-size: 3rem; font-weight: 800; color: #0066cc; }
    .stats-row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; }
    canvas { width: 100%; height: 120px; }
    .ch-row { display: flex; align-items: center; height: 18px; margin-bottom: 4px; }
    .ch-lbl { width: 30px; font-size: 0.6rem; color: #999; font-family: monospace; }
    .ch-track { flex: 1; height: 6px; background: #eee; border-radius: 3px; }
    .ch-bar { height: 100%; background: #0066cc; width: 50%; border-radius: 3px; }
  </style>
</head>
<body>

  <div class="top-bar">
    <div class="brand">GHOST V6.7</div>
    <div class="nav">
      <div class="nav-btn active" onclick="sw(0)">DASH</div>
      <div class="nav-btn" onclick="sw(1)">SETUP</div>
    </div>
  </div>

  <div id="t0" class="main active">
    <div style="display:flex; justify-content:space-between; margin-bottom:10px; align-items:center;">
        <div style="font-weight:700; color:#555;">STATUS</div>
        <div id="st" style="font-weight:800; color:#000;">CONNECTING...</div>
    </div>
    <div class="card"><h4>Signal History</h4><canvas id="gr"></canvas></div>
    <div class="stats-row">
        <div class="card" style="text-align:center;"><h4>LQ</h4><h2 id="lq">--%</h2></div>
        <div class="card" style="text-align:center;"><h4>RSSI</h4><h2 id="rssi">--dBm</h2></div>
    </div>
    <div class="card"><h4>Channels</h4><div id="chs"></div></div>
    
    <div class="card">
        <h4>Throttle Limiter</h4>
        <div class="lim-val"><span id="disp_lim">100</span>%</div>
        <input type="range" id="slider_lim" min="0" max="100" value="100" oninput="setLimit(this.value)">
    </div>
  </div>

  <div id="t1" class="main">
    <form onsubmit="save(event)">
      <button type="button" class="btn btn-blue" onclick="fillDefaults()">LOAD STANDARD PRESET</button>

      <div class="card">
        <h4>Pin Configuration</h4>
        <div class="input-group"><label>RX TX Pin</label><input type="number" id="cfg_retx"></div>
        <div class="input-group"><label>RX RX Pin</label><input type="number" id="cfg_rerx"></div>
        <div class="input-group"><label>Mod TX Pin</label><input type="number" id="cfg_jrtx"></div>
        <div class="input-group"><label>Mod RX Pin</label><input type="number" id="cfg_jrrx"></div>
        <div class="input-group"><label>Buffer Pin</label><input type="number" id="cfg_buf"></div>
      </div>
      <div class="card">
        <h4>Baud Rates</h4>
        <div class="input-group"><label>RX Baud</label><input type="number" id="cfg_rebaud"></div>
        <div class="input-group"><label>Mod Baud</label><input type="number" id="cfg_jrbaud"></div>
      </div>
      <button type="submit" class="btn btn-green">SAVE & REBOOT</button>
    </form>
  </div>

<script>
  function sw(i) {
    document.querySelectorAll('.main').forEach(e => e.classList.remove('active'));
    document.querySelectorAll('.nav-btn').forEach(e => e.classList.remove('active'));
    document.getElementById('t'+i).classList.add('active');
    document.querySelectorAll('.nav-btn')[i].classList.add('active');
    if(i===1) loadCfg(); 
  }
  function setLimit(val) {
    document.getElementById('disp_lim').innerText = val;
    fetch('/set_limit?val=' + val); 
  }
  function fillDefaults() {
    document.getElementById("cfg_retx").value = 32;
    document.getElementById("cfg_rerx").value = 33;
    document.getElementById("cfg_jrtx").value = 17;
    document.getElementById("cfg_jrrx").value = 16;
    document.getElementById("cfg_buf").value = 25;
    document.getElementById("cfg_rebaud").value = 420000;
    document.getElementById("cfg_jrbaud").value = 400000;
  }

  // --- CHART & DATA LOGIC ---
  const ctx = document.getElementById('gr').getContext('2d');
  let hist = new Array(80).fill(-130);
  
  function draw() {
    const w = ctx.canvas.clientWidth; const h = ctx.canvas.clientHeight;
    ctx.canvas.width = w; ctx.canvas.height = h;
    ctx.clearRect(0,0,w,h);
    ctx.strokeStyle='#eee'; ctx.beginPath(); 
    let y1 = h - ((-60+130)/110)*h; ctx.moveTo(0,y1); ctx.lineTo(w,y1); ctx.stroke();
    ctx.strokeStyle='#0066cc'; ctx.lineWidth=2; ctx.beginPath();
    let step = w/(hist.length-1);
    for(let i=0; i<hist.length; i++){
        let v=hist[i]; if(v<-130)v=-130; if(v>-20)v=-20;
        let y=h-((v+130)/110)*h;
        if(i===0)ctx.moveTo(0,y); else ctx.lineTo(i*step,y);
    }
    ctx.stroke();
  }
  setInterval(() => {
    fetch("/data").then(r=>r.json()).then(d=>{
      document.getElementById("lq").innerText=d.lq+"%";
      document.getElementById("rssi").innerText=d.rssi+"dBm";
      const s = document.getElementById("st");
      if(d.s===3){ s.innerText="ONLINE"; s.style.color="#155724"; }
      else if(d.s===0){s.innerText="RC LOST";s.style.color="#dc3545";}
      else if(d.s===1){s.innerText="MOD OFF";s.style.color="#856404";}
      else{s.innerText="WAITING...";s.style.color="#004085";}
      d.ch.forEach((v,i)=>{
        let el=document.getElementById("b"+i);
        if(el)el.style.width=(v/2047)*100+"%";
      });
      hist.push(d.rssi); hist.shift(); draw();
    });
  }, 200);
  function loadCfg() {
    fetch("/get_cfg").then(r=>r.json()).then(d=>{
        document.getElementById("cfg_retx").value = d.retx;
        document.getElementById("cfg_rerx").value = d.rerx;
        document.getElementById("cfg_jrtx").value = d.jrtx;
        document.getElementById("cfg_jrrx").value = d.jrrx;
        document.getElementById("cfg_buf").value = d.buf;
        document.getElementById("cfg_rebaud").value = d.bre;
        document.getElementById("cfg_jrbaud").value = d.bjr;
    });
  }
  function save(e) {
    e.preventDefault();
    if(!confirm("Reboot?")) return;
    const d = {
        retx: document.getElementById("cfg_retx").value,
        rerx: document.getElementById("cfg_rerx").value,
        jrtx: document.getElementById("cfg_jrtx").value,
        jrrx: document.getElementById("cfg_jrrx").value,
        buf: document.getElementById("cfg_buf").value,
        bre: document.getElementById("cfg_rebaud").value,
        bjr: document.getElementById("cfg_jrbaud").value
    };
    fetch("/save_cfg", {
        method: "POST", headers: {"Content-Type": "application/json"}, body: JSON.stringify(d)
    }).then(r=>{ alert("Saved! Rebooting..."); location.reload(); });
  }
  fetch("/get_cfg").then(r=>r.json()).then(d=>{
      document.getElementById("slider_lim").value = d.lim;
      document.getElementById("disp_lim").innerText = d.lim;
  });
  let h=''; for(let i=0;i<16;i++) h+=`<div class="ch-row"><div class="ch-lbl">CH${i+1}</div><div class="ch-track"><div class="ch-bar" id="b${i}"></div></div></div>`;
  document.getElementById("chs").innerHTML=h;
</script>
</body>
</html>
)rawliteral";

uint8_t calc_crc8(const uint8_t* payload, int length) {
  uint8_t crc = 0;
  for (int i = 0; i < length; i++) {
    crc ^= payload[i];
    for (int j = 0; j < 8; j++) { if (crc & 0x80) crc = (crc << 1) ^ CRC_POLY; else crc = crc << 1; }
  }
  return crc & 0xFF;
}

// --- CONFIG MANAGER ---
void loadConfig() {
  preferences.begin("ghost", true);
  config.re_tx = preferences.getInt("retx", 32);
  config.re_rx = preferences.getInt("rerx", 33);
  config.jr_tx = preferences.getInt("jrtx", 17);
  config.jr_rx = preferences.getInt("jrrx", 16);
  config.buf_en = preferences.getInt("buf", 25);
  int bre = preferences.getInt("bre", 420000); 
  config.baud_re = (bre < 9600) ? 420000 : bre;
  int bjr = preferences.getInt("bjr", 400000); 
  config.baud_jr = (bjr < 9600) ? 400000 : bjr;
  preferences.end();
  
  // *** AUTO-FIX ZEROS ***
  // If we loaded zeros (bad config), force defaults immediately
  if(config.re_tx == 0 || config.jr_tx == 0) {
      Serial.println("Detected Zeros! Forcing Defaults...");
      config.re_tx = 32; config.re_rx = 33;
      config.jr_tx = 17; config.jr_rx = 16;
      config.buf_en = 25;
      config.baud_re = 420000; config.baud_jr = 400000;
  }
}

void saveConfig() {
  preferences.begin("ghost", false);
  preferences.putInt("retx", config.re_tx);
  preferences.putInt("rerx", config.re_rx);
  preferences.putInt("jrtx", config.jr_tx);
  preferences.putInt("jrrx", config.jr_rx);
  preferences.putInt("buf", config.buf_en);
  preferences.putInt("bre", config.baud_re);
  preferences.putInt("bjr", config.baud_jr);
  preferences.end();
}

void resetToFactory() {
  preferences.begin("ghost", false);
  preferences.clear();
  preferences.end();
  delay(1000);
  ESP.restart();
}

// --- WEB ---
void handleRoot() { server.send(200, "text/html; charset=utf-8", index_html); }
void handleLimit() {
  if (server.hasArg("val")) {
    int val = server.arg("val").toInt();
    if (val < 0) val = 0; if (val > 100) val = 100;
    config.throttle_lim = val;
    server.send(200, "text/plain", "OK");
  } else server.send(400, "text/plain", "Bad");
}
void handleData() {
  String j = "{";
  j += "\"rssi\":" + String(disp_rssi) + ",";
  j += "\"lq\":" + String(disp_lq) + ",";
  j += "\"s\":" + String(link_status) + ",";
  j += "\"ch\":[";
  for(int i=0; i<16; i++) { j += String(currentChannels[i]); if(i<15) j+=","; }
  j += "]}";
  server.send(200, "application/json", j);
}
void handleGetCfg() {
    String j = "{";
    j += "\"lim\":" + String(config.throttle_lim) + ",";
    j += "\"retx\":" + String(config.re_tx) + ",";
    j += "\"rerx\":" + String(config.re_rx) + ",";
    j += "\"jrtx\":" + String(config.jr_tx) + ",";
    j += "\"jrrx\":" + String(config.jr_rx) + ",";
    j += "\"buf\":" + String(config.buf_en) + ",";
    j += "\"bre\":" + String(config.baud_re) + ",";
    j += "\"bjr\":" + String(config.baud_jr);
    j += "}";
    server.send(200, "application/json", j);
}
void handleSaveCfg() {
    if (server.hasArg("plain")) {
        String body = server.arg("plain");
        auto getVal = [&](String key) {
            int start = body.indexOf(key) + key.length() + 2; 
            int end = body.indexOf(",", start);
            if(end == -1) end = body.indexOf("}", start);
            return body.substring(start, end).toInt();
        };
        config.re_tx = getVal("retx");
        config.re_rx = getVal("rerx");
        config.jr_tx = getVal("jrtx");
        config.jr_rx = getVal("jrrx");
        config.buf_en = getVal("buf");
        int b1 = getVal("bre"); config.baud_re = (b1 < 9600) ? 420000 : b1;
        int b2 = getVal("bjr"); config.baud_jr = (b2 < 9600) ? 400000 : b2;
        saveConfig();
        server.send(200, "text/plain", "OK");
        delay(500);
        ESP.restart(); 
    }
}

// --- RADIO ---
void packCrsfChannels(uint8_t* payload, const volatile uint16_t* channels) {
    memset(payload, 0, 22);
    unsigned int bitIndex = 0;
    for (int i = 0; i < 16; i++) {
        uint32_t value = channels[i];
        if (i == 2 && config.throttle_lim < 100) {
             long zero = 172; if (value < zero) value = zero;
             long range = value - zero;
             range = (range * config.throttle_lim) / 100;
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
  
  digitalWrite(config.buf_en, LOW); // TX (Module)
  delayMicroseconds(2);
  uart_write_bytes(JR_UART, packet, 26);
  uart_wait_tx_done(JR_UART, 20 / portTICK_PERIOD_MS); // Wait until safe
  digitalWrite(config.buf_en, HIGH); // RX (Module)
  
  lastPacketTime = millis();
}

void processReceiverInput() {
  static uint8_t rx_buf[64];
  static int rx_idx = 0;
  static int expected_len = 0;

  size_t len = 0; 
  uart_get_buffered_data_len(RE_UART, &len);
  if (len == 0) return;

  uint8_t buf[len]; 
  uart_read_bytes(RE_UART, buf, len, 0);

  for (int i = 0; i < len; i++) {
      uint8_t b = buf[i];

      if (rx_idx == 0) {
          if (b == CRSF_SYNC_STICK || b == CRSF_SYNC_TELEM) {
              rx_buf[rx_idx++] = b;
          }
      } else if (rx_idx == 1) {
          if (b > 60 || b < 2) { 
              rx_idx = 0; // Invalid length, reset
          } else {
              rx_buf[rx_idx++] = b;
              expected_len = b + 2; // Sync byte + length byte + payload
          }
      } else {
          rx_buf[rx_idx++] = b;
          
          if (rx_idx == expected_len) {
              // --- FULL RC PACKET ASSEMBLED ---
              if (rx_buf[2] == CRSF_TYPE_STICKS && expected_len >= 26) {
                  lastRcInputTime = millis();
                  uint8_t* p = &rx_buf[3];
                  unsigned int bIdx = 0;
                  
                  for (int ch = 0; ch < 16; ch++) {
                      currentChannels[ch] = 0; 
                      for (int bit = 0; bit < 11; bit++) {
                          int byteIdx = bIdx / 8; int bitPos = bIdx % 8;
                          if (p[byteIdx] & (1 << bitPos)) currentChannels[ch] |= (1 << bit);
                          bIdx++;
                      }
                  }
              }
              rx_idx = 0; // Reset for the next packet
          }
      }
  }
}

void processJrModule() {
  static uint8_t rx_buf[64];
  static int rx_idx = 0;
  static int expected_len = 0;

  size_t len = 0; 
  uart_get_buffered_data_len(JR_UART, &len);
  if (len == 0) return;

  uint8_t buf[len]; 
  uart_read_bytes(JR_UART, buf, len, 0); 

  for (int i = 0; i < len; i++) {
      uint8_t b = buf[i];

      if (rx_idx == 0) {
          if (b == CRSF_SYNC_RADIO || b == CRSF_SYNC_TELEM) {
              rx_buf[rx_idx++] = b;
          }
      } else if (rx_idx == 1) {
          if (b > 60 || b < 2) { 
              rx_idx = 0; // Invalid length, reset
          } else {
              rx_buf[rx_idx++] = b;
              expected_len = b + 2; 
          }
      } else {
          rx_buf[rx_idx++] = b;
          
          if (rx_idx == expected_len) {
              // --- FULL TELEMETRY PACKET ASSEMBLED ---
              uint8_t type = rx_buf[2];

              if (type == CRSF_TYPE_HEARTBEAT) {
                  sendStickPacket(); // Reply to module heartbeat
              } else {
                  // ECHO TELEMETRY TO RADIO (Non-blocking)
                  uart_write_bytes(RE_UART, rx_buf, expected_len);
                  
                  // Read stats for the web dashboard
                  if (type == CRSF_TYPE_LINKSTAT && expected_len >= 14) {
                      disp_rssi = (int8_t)rx_buf[3]; 
                      disp_lq = rx_buf[5]; 
                      disp_snr = (int8_t)rx_buf[6];
                      lastTelemTime = millis();
                  }
              }
              rx_idx = 0; // Reset for the next packet
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
  Serial.begin(115200);
  Serial.println("\n--- GHOST RELAY BOOT V6.7 ---");

  // 0. FACTORY RESET TRIGGER (Hold BOOT)
  pinMode(0, INPUT_PULLUP);
  if(digitalRead(0) == LOW) {
    resetToFactory();
  }

  // 1. Config
  loadConfig();
  Serial.println("Config Loaded.");

  // 2. Pins
  pinMode(config.buf_en, OUTPUT); digitalWrite(config.buf_en, HIGH); 

  // 3. UARTs
  uart_config_t jr_c = { .baud_rate = config.baud_jr, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_APB };
  uart_driver_install(JR_UART, BUF_SIZE * 2, 0, 0, NULL, 0); uart_param_config(JR_UART, &jr_c);
  uart_set_pin(JR_UART, config.jr_tx, config.jr_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_set_line_inverse(JR_UART, UART_SIGNAL_RXD_INV | UART_SIGNAL_TXD_INV);

  uart_config_t re_c = { .baud_rate = config.baud_re, .data_bits = UART_DATA_8_BITS, .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1, .flow_ctrl = UART_HW_FLOWCTRL_DISABLE, .source_clk = UART_SCLK_APB };
  uart_driver_install(RE_UART, BUF_SIZE * 2, 0, 0, NULL, 0); uart_param_config(RE_UART, &re_c);
  uart_set_pin(RE_UART, config.re_tx, config.re_rx, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  
  // 4. Init Default Channels
  for(int i=0; i<16; i++) currentChannels[i] = 992; 
  sendStickPacket();

  // 5. Start Radio (Core 0)
  xTaskCreatePinnedToCore(RadioTask, "RadioTask", 10000, NULL, 5, &RadioTaskHandle, 0);

  // 6. Start WiFi
  WiFi.softAP(DEF_SSID, DEF_PASS);
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/set_limit", handleLimit);
  server.on("/get_cfg", handleGetCfg);
  server.on("/save_cfg", HTTP_POST, handleSaveCfg);
  server.begin();
  Serial.println("Web Server Started.");
}

void loop() {
  server.handleClient();
  
  unsigned long now = millis();
  if (now - lastRcInputTime < 100) {
      if (now - lastPacketTime < 100) {
          if (now - lastTelemTime < 1000) link_status = 3; else link_status = 2;
      } else link_status = 1;
  } else link_status = 0;

  delay(2);
}

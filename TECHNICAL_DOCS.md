# ESP32-CRSF-Relay — Technická dokumentace

> Tento soubor je nyní považován za legacy verzi.
>
> Nová, strukturovaná dokumentace je připravena pro MkDocs v adresáři `docs/`.
>
> Lokální spuštění:
> ```powershell
> pip install -r requirements-docs.txt
> mkdocs serve
> ```
>
> Vstupní stránka nové dokumentace: `docs/index.md`

> **Verze:** 1.1  
> **Naposledy aktualizováno:** 2025-12-22  
> **Platforma:** ESP32 (WROOM-32) · Arduino Framework  
> **Jazyk:** C++ (`.ino`)  
> **Licence:** MIT  

---

## Obsah

1. [Přehled projektu](#1-přehled-projektu)
2. [Struktura repozitáře](#2-struktura-repozitáře)
3. [Architektura systému](#3-architektura-systému)
4. [Hardwarové rozhraní](#4-hardwarové-rozhraní)
5. [Implementace protokolu CRSF](#5-implementace-protokolu-crsf)
6. [Referenční přehled modulů firmware](#6-referenční-přehled-modulů-firmware)
7. [Webový dashboard a HTTP API](#7-webový-dashboard-a-http-api)
8. [Návrh half-duplex PCB](#8-návrh-half-duplex-pcb)
9. [Instrukce pro build a flash](#9-instrukce-pro-build-a-flash)
10. [Referenční konfigurace](#10-referenční-konfigurace)
11. [Řešení problémů](#11-řešení-problémů)

---

## 1. Přehled projektu

**ESP32-CRSF-Relay** je CRSF relay node, který rozšiřuje komunikační dosah rádiového spojení ExpressLRS (ELRS). Funguje jako transparentní most: ELRS přijímač dekóduje příchozí CRSF rámce kanálů kniplů ze vzdáleného vysílače a přes UART je předává do ESP32. ESP32 tyto rámce znovu zabalí a odešle do výkonného ELRS TX modulu, který signál přeposílá dál do dronu nebo jiného vozidla.

V reálném testování relay node přibližně **zdvojnásobil efektivní komunikační dosah** oproti přímému jednolinkovému spojení. Relay nepřidává znatelnou latenci, smyčka přeposílání běží v submilisekundových intervalech na vyhrazeném jádru.

### Hlavní cíle návrhu

| Cíl | Implementace |
| :--- | :--- |
| Nulová ztráta paketů | Rádiová úloha připnutá na **Core 1** s FreeRTOS prioritou 5 |
| Monitoring v reálném čase | WiFi dashboard na **Core 0** se vzorkováním po 100 ms |
| Bezpečnost pro trénink | Nastavitelný limiter plynu (škálování kanálu 3, 0–100 %) |
| Bez externí aplikace | Vlastní WiFi AP + vestavěný HTML/JS dashboard |
| Minimální hardware | Jedno ESP32 + vlastní half-duplex PCB + ELRS moduly |

---

## 2. Struktura repozitáře

```
ESP32-CRSF-Relay/
├── .gitignore
├── LICENSE                              # MIT licence
├── README.md                            # Dokumentace pro uživatele
├── TECHNICAL_DOCS.md                    # ← Tento soubor
├── code/
│   └── esp32code.ino                    # Hlavní firmware (jeden soubor, 426 řádků)
├── PCB/
│   ├── halfduplextranslator.kicad_pcb   # KiCad PCB layout
│   ├── halfduplextranslator.kicad_pro   # KiCad projektový soubor
│   └── halfduplextranslator.kicad_sch   # KiCad schéma
├── halfduplextranslator.kicad_pcb       # (kopie PCB souborů v kořeni)
├── halfduplextranslator.kicad_pro
└── halfduplextranslator.kicad_sch
```

### Klíčové soubory

| Soubor | Účel |
| :--- | :--- |
| `code/esp32code.ino` | Kompletní firmware: UART drivery, CRSF parsing, balení kanálů, half-duplex přepínání, WiFi AP, web server a vestavěný HTML dashboard |
| `PCB/halfduplextranslator.kicad_sch` | Schéma převodní desky full-duplex na half-duplex |
| `PCB/halfduplextranslator.kicad_pcb` | PCB layout (KiCad 7+) |

---

## 3. Architektura systému

### 3.1 Datový tok na vysoké úrovni

```
┌──────────────┐    CRSF/UART     ┌─────────────┐    Half-Duplex    ┌──────────────┐
│  ELRS         │  ─────────────►  │   ESP32       │  ──────────────►  │  ELRS TX      │
│  Receiver     │    420 kbaud     │   Relay Node  │    400 kbaud     │  Module       │
│  (Input)      │  ◄─────────────  │               │  ◄──────────────  │  (Output)     │
└──────────────┘    (RX pad)       └─────────────┘    via PCB         └──────────────┘
                                         │
                                         │ WiFi AP (192.168.4.1)
                                         ▼
                                   ┌─────────────┐
                                   │  Web Browser  │
                                   │  (Dashboard)  │
                                   └─────────────┘
```

### 3.2 Rozdělení úloh mezi dvě jádra

| Jádro | Úloha | Priorita | Odpovědnost |
| :--- | :--- | :--- | :--- |
| **Core 1** | `RadioTask` | 5 (vysoká) | Čtení UART přijímače, čtení UART modulu, odesílání stick paketů, half-duplex přepínání |
| **Core 0** | `loop()` (výchozí) | 1 (normální) | Správa WiFi AP, HTTP server (`WebServer.handleClient()`) |

**Proč je to důležité:** Operace WiFi stacku (TCP handshake, HTTP odpovědi) mohou vnášet jitter 10–50 ms. Izolací rádiového zpracování na samostatné jádro relay node zajišťuje plynulé přeposílání CRSF rámců bez ohledu na webový provoz.

### 3.3 Smyčka radio úlohy (`RadioTask`)

```
RadioTask (Core 1, nekonečná smyčka):
│
├── processReceiverInput()    // Čte ELRS RX UART, dekóduje kanály kniplů
├── processJrModule()         // Čte UART TX modulu, zpracuje heartbeat a telemetrii
├── if (>20 ms od posledního TX) → sendStickPacket()   // Keepalive
└── vTaskDelay(1 ms)          // Krátké uvolnění CPU
```

### 3.4 Latence

Relay nepřidává **žádnou znatelnou latenci**. Smyčka `RadioTask` běží přibližně každou 1 ms (uvolnění přes `vTaskDelay`) a přeposílání rámce se spouští ihned po přijetí heartbeat od TX modulu. Teoretická přidaná latence je omezena periodou smyčky (~1 ms) plus časem UART přenosu (~650 µs pro 26 bajtů při 400 kbaud), celkem pod 2 ms na jeden hop.

---

## 4. Hardwarové rozhraní

### 4.1 Přiřazení GPIO pinů

| Makro | GPIO | Směr | UART | Funkce |
| :--- | :--- | :--- | :--- | :--- |
| `RE_TX_PIN` | 32 | Výstup → RX pad přijímače | `UART_NUM_2` | Odesílání dat do ELRS přijímače |
| `RE_RX_PIN` | 33 | Vstup ← TX pad přijímače | `UART_NUM_2` | Příjem CRSF rámců z ELRS přijímače |
| `JR_TX_PIN` | 17 | Výstup → datová linka PCB | `UART_NUM_1` | Přenos stick paketů do TX modulu |
| `JR_RX_PIN` | 16 | Vstup ← datová linka PCB | `UART_NUM_1` | Příjem telemetrie/heartbeatů z TX modulu |
| `BUFFER_EN_PIN` | 25 | Výstup (digitální) | — | Řídí half-duplex směr na bufferu SN74HC125DR |

### 4.2 Konfigurace UART

| Parametr | UART přijímače (`RE_UART`) | UART modulu (`JR_UART`) |
| :--- | :--- | :--- |
| Baud rate | **420,000** | **400,000** |
| Datové bity | 8 | 8 |
| Parita | Žádná | Žádná |
| Stop bity | 1 | 1 |
| Flow control | Žádný | Žádný |
| Zdroj hodin | `UART_SCLK_APB` | `UART_SCLK_APB` |
| Inverze signálu | Žádná | **RXD + TXD invertované** |
| Velikost RX bufferu | 2048 bajtů (`BUF_SIZE * 2`) | 2048 bajtů (`BUF_SIZE * 2`) |
| Velikost TX bufferu | 0 (blokující zápis) | 0 (blokující zápis) |

> **Poznámka:** UART JR modulu používá invertované signály (`uart_set_line_inverse`), protože CRSF protokol na JR-bay modulech používá invertovanou UART signalizaci.

### 4.3 Otestovaný hardware

#### Vstupní přijímače (ELRS RX)

| Přijímač | Frekvence | Poznámky |
| :--- | :--- | :--- |
| **HGLRC ELRS 2.4G** | 2.4 GHz | Otestováno a potvrzeno jako funkční |
| **Radiomaster XR1 Nano ELRS** | 868 MHz | Dvoupásmový (868 MHz / 2.4 GHz), testováno na 868 MHz |

#### Výstupní TX modul

| Modul | Poznámky |
| :--- | :--- |
| **Emax Aeris Link Nano 900Mhz** | Testovaný TX modul, bylo zvoleno 900MHz kvůli hrozbě rušení dvou 2,4GHz spojení v těstné blízosti|

### 4.4 Napájení

Celý relay node je napájen z **2S LiPo baterie** (jmenovitě 7.4V, rozsah 5–8.4V):

```
2S LiPo baterie (7.4 V)
├──► Step-Down regulátor ──► ESP32 (5V / 3.3V přes onboard regulátor)
└──► JST 2pin port ───────────► ELRS TX modul (přímé napájení z baterie)
```

- **ESP32** je napájena přes step-down (buck) modul, který převádí napětí 2S baterie na úroveň vhodnou pro vestavěný regulátor ESP32.
- **TX modul** (např. Emax Aeris Link Nano) je napájen přímo z baterie přes JST 2pinkonektor, který podporuje plný rozsah 2S napětí.
- **ELRS přijímač** bere napájení z 3.3V nebo 5V pinu ESP32 (minimální odběr proudu).

> ⚠️ **Nenapájejte vysokovýkonové TX moduly (>250 mW) přes USB nebo napěťové piny ESP32.** Moduly jako Emax Aeris mohou při 1W+ odebírat až 2A, což způsobí brownouty, restarty a failsafe stavy.

### 4.5 Half-duplex přepínací protokol

TX modul komunikuje přes jednu datovou linku (half-duplex). ESP32 používá `BUFFER_EN_PIN` (GPIO 25) pro řízení směru **SN74HC125DR** quad tri-state bufferu na vlastní PCB:

```
sendStickPacket():
  1. BUFFER_EN_PIN → LOW          // Povolit výstup bufferu (TX režim)
  2. delayMicroseconds(2)         // Čas na ustálení
  3. uart_write_bytes(packet, 26) // Odeslat 26bajtový CRSF rámec
  4. delayMicroseconds(680)       // Počkat na dokončení přenosu
  5. BUFFER_EN_PIN → HIGH         // Zakázat výstup bufferu (RX režim / vysoká impedance)
```

**Rozpad časování:**
- Při 400 kbaud je jeden bajt ≈ 25 µs → 26 bajtů ≈ 650 µs
- Zpoždění 680 µs pokrývá kompletní přenos rámce + rezervu
- Předzpoždění 2 µs zajišťuje přepnutí bufferu před začátkem dat

---

## 5. Implementace protokolu CRSF

### 5.1 CRSF sync bajty

| Makro | Hodnota | Původ | Popis |
| :--- | :--- | :--- | :--- |
| `CRSF_SYNC_STICK` | `0xEE` | Modul/Handset | Sync bajt pro stick/channel rámce odesílané do modulu |
| `CRSF_SYNC_TELEM` | `0xC8` | Flight Controller | Sync bajt pro telemetrické rámce |
| `CRSF_SYNC_RADIO` | `0xEA` | Rádio/Modul | Sync bajt pro rámce původem z rádia (heartbeat, config) |

### 5.2 Typy CRSF rámců

| Makro | Hodnota | Popis |
| :--- | :--- | :--- |
| `CRSF_TYPE_STICKS` | `0x16` | RC kanály zabalené dohromady — 16 kanálů × 11 bitů |
| `CRSF_TYPE_LINKSTAT` | `0x14` | Statistiky linku — RSSI, LQ, SNR |
| `CRSF_TYPE_HEARTBEAT` | `0x3A` | Heartbeat modulu — spouští přenos stick paketu |

### 5.3 Struktura rámce stick kanálů (26 bajtů)

```
Bajt:  [0]    [1]    [2]     [3..24]           [25]
       Sync   Len    Type    Payload (22 B)    CRC8
       0xEE   24     0x16    zabalené kanály   kontrolní součet
```

- **Payload:** 16 kanálů × 11 bitů = 176 bitů = 22 bajtů, baleno LSB-first
- **Rozsah kanálu:** 0–2047 (11 bitů), střed na 992
- **CRC8 polynom:** `0xD5` (počítáno přes bajty `[2..24]`, tj. type + payload)

### 5.4 Algoritmus balení kanálů (`packCrsfChannels`)

Každý z 16 kanálů je 11bitové unsigned celé číslo. Bity se sekvenčně balí do 22 bajtů v pořadí LSB-first:

```
Kanál 0: bity [0..10]    → bity payloadu 0–10
Kanál 1: bity [11..21]   → bity payloadu 11–21
Kanál 2: bity [22..32]   → bity payloadu 22–32  (zde se aplikuje limiter plynu)
...
Kanál 15: bity [165..175]
```

**Omezení plynu (kanál 3 / index 2):**

Když `throttleScale < 100`, kanál 3 se škáluje:

```
zero_point = 172          // CRSF hodnota reprezentující 0% plyn
range = channel_value - zero_point
scaled_range = (range × throttleScale) / 100
output = zero_point + scaled_range
```

Tím se zachovává pozice nulového plynu při proporcionálním snížení maxima.

### 5.5 Výpočet CRC-8

```cpp
uint8_t calc_crc8(const uint8_t* payload, int length) {
    uint8_t crc = 0;
    for (int i = 0; i < length; i++) {
        crc ^= payload[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) crc = (crc << 1) ^ 0xD5;
            else crc = crc << 1;
        }
    }
    return crc & 0xFF;
}
```

- **Polynom:** `0xD5` (standardní CRSF CRC-8)
- **Init:** `0x00`
- **Vstup:** Bajty rámce od pole type po konec payloadu (bajty `[2..24]`)

### 5.6 Parsování statistik linku

Když je z TX modulu přijat `CRSF_TYPE_LINKSTAT` (`0x14`) rámec:

```
Offsety bajtů payloadu (od bajtu [i+3]):
  p[0] = Uplink RSSI (Ant. 1)    → cast na int8_t → disp_rssi
  p[2] = Link Quality (0–100%)   → disp_lq
  p[3] = SNR                     → cast na int8_t → disp_snr
```

---

## 6. Referenční přehled modulů firmware

### 6.1 Funkce

| Funkce | Popis | Běží na |
| :--- | :--- | :--- |
| `setup()` | Inicializuje GPIO, oba UARTy, nastaví kanály na střed (992), spustí `RadioTask` na Core 1, spustí WiFi AP a HTTP server | Core 0 |
| `loop()` | Volá `server.handleClient()` každé 2 ms | Core 0 |
| `RadioTask(void*)` | Nekonečná smyčka: zpracování přijímače, zpracování modulu, keepalive TX | Core 1 |
| `processReceiverInput()` | Čte `RE_UART`, hledá CRSF stick rámce, rozbaluje 16 kanálů | Core 1 |
| `processJrModule()` | Čte `JR_UART`, zpracovává heartbeat (spouští TX) a link statistiky | Core 1 |
| `sendStickPacket()` | Sestaví 26bajtový CRSF rámec, provede half-duplex TX přes SN74HC125DR | Core 1 |
| `packCrsfChannels(payload, channels)` | Bitově sbalí 16 × 11bit kanálů do 22 bajtů, aplikuje limiter plynu | Core 1 |
| `calc_crc8(payload, length)` | Spočítá CRC-8 s polynomem `0xD5` | Core 1 |
| `handleRoot()` | Vrací vestavěný HTML dashboard | Core 0 |
| `handleData()` | Vrací JSON telemetrii (`/data` endpoint) | Core 0 |
| `handleLimit()` | Přijímá parametr škálování plynu (`/set_limit?val=N`) | Core 0 |

### 6.2 Globální proměnné

| Proměnná | Typ | Volatile | Popis |
| :--- | :--- | :--- | :--- |
| `currentChannels[16]` | `uint16_t[]` | ✅ | Poslední dekódované hodnoty RC kanálů (0–2047) |
| `disp_rssi` | `int` | ✅ | Aktuální RSSI v dBm (výchozí: -130) |
| `disp_lq` | `int` | ✅ | Aktuální Link Quality v procentech (0–100) |
| `disp_snr` | `int` | ✅ | Aktuální Signal-to-Noise Ratio |
| `throttleScale` | `int` | ✅ | Limit plynu v procentech (0–100, výchozí: 100) |
| `lastPacketTime` | `unsigned long` | — | Čas posledního odeslaného stick paketu |
| `lastRcInputTime` | `unsigned long` | — | Čas posledního přijatého RC vstupu |
| `lastTelemTime` | `unsigned long` | — | Čas poslední přijaté telemetrie |
| `RadioTaskHandle` | `TaskHandle_t` | — | FreeRTOS handle radio úlohy |

### 6.3 Časové konstanty

| Událost | Práh | Účel |
| :--- | :--- | :--- |
| Keepalive TX | 20 ms | Pokud heartbeat nespustí TX do 20 ms, odešle se stick paket i tak |
| Timeout RC vstupu | 100 ms | Pokud nepřijde stick rámec 100 ms → stav: `RC LOST` |
| Timeout paketu do modulu | 100 ms | Pokud se neodešle paket do modulu 100 ms → stav: `MODULE OFF` |
| Timeout telemetrie | 1000 ms | Pokud nepřijdou link statistiky 1 s → stav: `DRONE LOST` |
| Interval načítání dashboardu | 100 ms | Web frontend načítá `/data` každých 100 ms |
| Uvolnění rádiové úlohy | 1 ms | `vTaskDelay(1 / portTICK_PERIOD_MS)` — minimální kooperativní yield |

---

## 7. Webový dashboard a HTTP API

### 7.1 WiFi Access Point

| Parametr | Hodnota |
| :--- | :--- |
| SSID | `Comm_Relay_Manager` |
| Heslo | *(žádné — otevřená síť)* |
| IP adresa | `192.168.4.1` |
| Režim | SoftAP |

### 7.2 HTTP endpointy

| Metoda | Cesta | Handler | Popis |
| :--- | :--- | :--- | :--- |
| GET | `/` | `handleRoot()` | Vrátí kompletní HTML/CSS/JS dashboard (uložený v `PROGMEM`) |
| GET | `/data` | `handleData()` | Vrátí živou telemetrii jako JSON |
| GET | `/set_limit?val=N` | `handleLimit()` | Nastaví škálování plynu na `N` (0–100) |

### 7.3 Schéma JSON odpovědi `/data`

```json
{
  "rssi": -65,
  "lq": 98,
  "status": 3,
  "channels": [992, 992, 172, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992, 992]
}
```

| Pole | Typ | Rozsah | Popis |
| :--- | :--- | :--- | :--- |
| `rssi` | int | -130 až 0 | Uplink RSSI v dBm |
| `lq` | int | 0–100 | Kvalita linku v procentech |
| `status` | int | 0–3 | Kód stavu systému (viz níže) |
| `channels` | int[] | každý 0–2047 | 16 hodnot RC kanálů |

### 7.4 Stavové kódy

| Kód | Štítek | Barva | Podmínka |
| :--- | :--- | :--- | :--- |
| 0 | `RC LOST` | 🔴 Červená | Žádný RC vstup z přijímače déle než 100 ms |
| 1 | `MODULE OFF` | 🟡 Žlutá | RC vstup je OK, ale do modulu nešel paket déle než 100 ms |
| 2 | `DRONE LOST` | 🔵 Modrá | RC + modul OK, ale žádná telemetrie déle než 1000 ms |
| 3 | `ONLINE` | 🟢 Zelená | Všechny systémy v normálu |

**Logika určení stavu (vyhodnocováno v pořadí):**
```
if (now - lastRcInputTime >= 100 ms)     → status = 0 (RC LOST)
else if (now - lastPacketTime >= 100 ms) → status = 1 (MODULE OFF)
else if (now - lastTelemTime >= 1000 ms) → status = 2 (DRONE LOST)
else                                      → status = 3 (ONLINE)
```

### 7.5 Funkce dashboardu

Webové rozhraní je jednostránková aplikace vložená v `PROGMEM` (~3.5 KB) se dvěma záložkami:

**Záložka Dashboard:**
- **Stavový badge systému** — barevný indikátor aktuálního stavu
- **Graf historie signálu** — posouvající se čárový graf RSSI (100 bodů, canvas)
- **Statistiky** — karty Link Quality (%) a RSSI (dBm)
- **Monitor kanálů** — 16 horizontálních indikátorů pozice kanálů v reálném čase

**Záložka Limits:**
- **Limiter výstupu plynu** — slider rozsahu (0–100 %), který škáluje výstup kanálu 3
- Aktualizace se odesílají okamžitě přes `fetch('/set_limit?val=N')`

---

## 8. Návrh half-duplex PCB

### 8.1 Účel

ESP32 používá full-duplex UART (oddělené TX a RX vodiče), zatímco ELRS TX moduly typicky používají jednu half-duplex datovou linku pro vysílání i příjem. Vlastní PCB tento rozdíl řeší pomocí **SN74HC125DR** quad tri-state bufferu.

### 8.2 Buffer IC: SN74HC125DR

| Parametr | Hodnota |
| :--- | :--- |
| Výrobce | Texas Instruments |
| Pouzdro | SOIC-14 |
| Typ | Quad Bus Buffer Gate se 3stavovými výstupy |
| Napájecí napětí | 2V – 6V |
| Propagation delay | ~7 ns (typ. při 4.5V) |
| Output enable | Aktivní LOW (řízeno přímo `BUFFER_EN_PIN` / GPIO 25) |
| Použité hradlo | **1 ze 4** (jedno hradlo pro řízení TX směru) |

SN74HC125DR obsahuje čtyři nezávislá tri-state bufferová hradla. V tomto návrhu je použito pouze **jedno hradlo** a řídí TX směr z ESP32 na datovou linku modulu. Zbývající tři hradla nejsou použita. Každé hradlo má pin output enable (`OE`, aktivní low). Když je `OE` LOW, buffer aktivně řídí výstup; když je `OE` HIGH, výstup přejde do vysoké impedance a efektivně se od datové linky odpojí.

### 8.3 Soubory projektu KiCad

| Soubor | Popis |
| :--- | :--- |
| `PCB/halfduplextranslator.kicad_sch` | Schéma zapojení |
| `PCB/halfduplextranslator.kicad_pcb` | PCB layout |
| `PCB/halfduplextranslator.kicad_pro` | Metadata KiCad projektu |

### 8.4 Princip fungování

```
ESP32 GPIO 25 (BUFFER_EN):
  LOW  → Výstup bufferu povolen (TX režim: ESP32 → modul)
  HIGH → Výstup bufferu ve vysoké impedanci (RX režim: modul → ESP32)

                          ┌────────────────────┐
ESP32 GPIO 17 (JR_TX) ──►│ SN74HC125DR Gate 1  │──► Single Data Line ──► TX modul
                          │ OE = GPIO 25        │
                          └────────────────────┘
ESP32 GPIO 16 (JR_RX) ◄───────────────────────────── Single Data Line ◄── TX modul
```

Při vysílání:
1. `BUFFER_EN` přejde na LOW → hradlo 1 SN74HC125DR se povolí, propouští ESP32 TX na datovou linku
2. Data se vysílají při 400 kbaud (invertovaně)
3. Po dokončení přenosu (~680 µs pro 26 bajtů) přejde `BUFFER_EN` na HIGH
4. Výstup bufferu přejde do vysoké impedance, telemetrie z modulu tak může jít přímo do ESP32 RX

### 8.5 Kdy PCB není potřeba

Pokud používáte ELRS **přijímač flashnutý jako TX modul** (místo dedikovaného TX modulu), PCB není potřeba, protože přijímače mají oddělené TX a RX UART pady a full-duplex komunikace je nativní.

---

## 9. Instrukce pro build a flash

### 9.1 Předpoklady

- **Arduino IDE** (1.8+ nebo 2.x) s nainstalovaným **ESP32 board package**
- Volba desky: `ESP32 Dev Module` (nebo `DOIT ESP32 DEVKIT V1`)


### 9.2 Konfigurace ELRS přijímače

Jak vstupní ELRS přijímač s příslušným ELRS ovladačem, tak ELRS vysílač s přijímačem na straně dronu musí být v ELRS Configuratoru nastaveny se shodnou **bind phrase**. ždý pár odlišnou. 


### 9.3 Nastavení uploadu

| Nastavení | Hodnota |
| :--- | :--- |
| Board | ESP32 Dev Module |
| Upload Speed | 921600 |
| CPU Frequency | 240 MHz (výchozí) |
| Flash Frequency | 80 MHz |
| Flash Size | 4 MB (výchozí) |
| Partition Scheme | Default 4MB with spiffs |

### 9.4 Kroky

1. Otevřete `code/esp32code.ino` v Arduino IDE.
2. Vyberte správnou desku a COM port.
3. Klikněte na **Upload**.
4. Po nahrání ESP32 automaticky:
   - Inicializuje oba UARTy
   - Spustí radio úlohu na Core 1
   - Začne vysílat WiFi síť `Comm_Relay_Manager`
5. Připojte se k WiFi a přejděte na `http://192.168.4.1`.

---

## 10. Referenční konfigurace

Veškerá konfigurace je přes `#define` makra a konstanty na začátku souboru `esp32code.ino`:

### 10.1 WiFi

```cpp
const char* ssid = "Comm_Relay_Manager";  // Změňte pro vlastní název sítě
const char* password = "";                 // Nastavte heslo pro zabezpečený přístup
```

### 10.2 Mapování pinů

```cpp
#define BUFFER_EN_PIN 25    // Řízení half-duplex směru (SN74HC125DR OE, Gate 1)
#define JR_TX_PIN     17    // TX do modulu (přes PCB)
#define JR_RX_PIN     16    // RX z modulu (přes PCB)
#define RE_TX_PIN     32    // TX do přijímače
#define RE_RX_PIN     33    // RX z přijímače
```

### 10.3 Ladění protokolu

```cpp
#define BUF_SIZE  1024      // Alokace UART bufferu (skutečný RX buffer = BUF_SIZE * 2)
#define CRC_POLY  0xD5      // CRC-8 polynom (CRSF standard — neměnit)
```

### 10.4 Baud rates

| UART | Baud rate | Poznámky |
| :--- | :--- | :--- |
| Přijímač (`RE_UART`) | 420,000 | Standardní ELRS CRSF baud rate |
| Modul (`JR_UART`) | 400,000 | Standardní CRSF JR-bay baud rate (invertovaný) |
| Debug Serial | 115,200 | USB serial (aktuálně se pro výstup nepoužívá) |

---

## 11. Řešení problémů

### Stav: RC LOST

| Možná příčina | Řešení |
| :--- | :--- |
| Přijímač není napájen | Zkontrolujte 3.3V/5V napájení ELRS přijímače |
| Špatné zapojení | Ověřte GPIO 33 ← TX přijímače, GPIO 32 → RX přijímače |
| Přijímač není spárován | Ověřte shodu bind phrase mezi přijímačem a vysílačem |
| Neshoda baud rate | Ujistěte se, že přijímač je nastaven na CRSF výstup při 420 kbaud |

### Stav: MODULE OFF

| Možná příčina | Řešení |
| :--- | :--- |
| TX modul není napájen | Zajistěte adekvátní externí napájení přes 2S LiPo (viz sekce 4.4) |
| PCB není připojeno | Zkontrolujte zapojení PCB na GPIO 16, 17 a 25 |
| Selhání SN74HC125DR | Ověřte pájené spoje buffer IC a napájecí napětí |
| Problém se step-down regulátorem | Potvrďte, že buck modul dává správné napětí pro ESP32 |

### Stav: DRONE LOST

| Možná příčina | Řešení |
| :--- | :--- |
| Dron je mimo dosah | Přesuňte dron blíže nebo zvyšte TX výkon |
| Přijímač v dronu není spárován | Ověřte shodu bind phrase mezi přijímačem dronu a TX modulem relay nodu |
| Telemetrie je vypnutá | Zapněte telemetrii v ELRS Lua scriptu nebo konfigurátoru |

### Brownouty / restarty

| Možná příčina | Řešení |
| :--- | :--- |
| Vysokovýkonový modul na USB | **Nikdy** nenapájejte moduly >250 mW z USB. Použijte 2S baterii (5–21V) přes JST 2pin|
| Nedostatečný proud | Ujistěte se, že zdroj zvládne ≥2A pro 1W+ moduly |
| Přetížený step-down | Ověřte, že buck konvertor je dimenzován na kombinovaný odběr ESP32 + periferií |

### Dashboard se nenačítá

| Možná příčina | Řešení |
| :--- | :--- |
| Nejste připojeni k WiFi | Připojte se k `Comm_Relay_Manager` (otevřená, bez hesla) |
| Špatná IP adresa | Otevřete `http://192.168.4.1` |
| Cache prohlížeče | Proveďte tvrdý refresh (`Ctrl+Shift+R`) nebo zkuste anonymní režim |

---


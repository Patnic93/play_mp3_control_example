# ESP32-LyraT Internet Radio Player with UART/I2C Control

ESP32-LyraT V4.3 audio player dat een HTTP MP3 stream afspeelt over WiFi.
Een Waveshare ESP32-S3 front-end stuurt de speler aan via UART.
De LyraT stuurt ook temperature- en vochtigheidsmeting (AHT20 sensor) terug naar de Waveshare.

## Features

- Streamt live internetradio via WiFi (HTTP/HTTPS MP3)
- Standaard stream: `http://icecast.omroep.nl/radio2-bb-mp3` (NPO Radio 2)
- Fysieke knoppen op het LyraT-board (play/pause/stop/volume)
- **UART interface** voor aansturing door Waveshare front-end (URL, volume, play/pause/stop, EQ)
- Status-terugkoppeling over UART (playing / paused / stopped / error + volume)
- **AHT20 sensor** (I2C, GPIO 18/23): temperatuur en vochtigheid, op verzoek teruggestuurd via UART
- Captive portal WiFi setup: als de WiFi verbinding mislukt, start de LyraT een AP (`playmp3_setup_XXXXXX`) zodat credentials ingesteld kunnen worden via een browser
- WiFi credentials worden opgeslagen in NVS en na herstart automatisch hergebruikt

## Hardware

- **Board:** ESP32-LyraT V4.3
- **ESP-IDF:** v5.5.3
- **ESP-ADF:** `C:\esp\esp-adf` (stel `ADF_PATH` in)

## Build & Flash

```bash
idf.py -p PORT flash monitor
```

Of gebruik de ESP-IDF extensie in VS Code.

## WiFi instellen

WiFi credentials staan in `main/wifi_secrets.h` (git-ignored, niet in repo):

```c
#define LYRAT_WIFI_SSID "jouw-ssid"
#define LYRAT_WIFI_PWD  "jouw-wachtwoord"
```

Als de verbinding mislukt of `wifi_secrets.h` nog niet ingevuld is, start de LyraT automatisch een captive portal AP (`playmp3_setup_XXXXXX`). Verbind met dit netwerk en stel de credentials in via de browser. De credentials worden opgeslagen in NVS.

## Fysieke knoppen (LyraT)

| Knop  | Actie                         |
|-------|-------------------------------|
| Play  | Start / Pause / Resume        |
| Set   | Stop                          |
| Mode  | Herstart huidige stream       |
| Vol+  | Volume omhoog (stap +10)      |
| Vol-  | Volume omlaag (stap -10)      |

## AHT20 Temperatuur- & vochtigheidssensor

| AHT20 pin | LyraT GPIO |
|-----------|------------|
| VIN       | 3.3V       |
| GND       | GND        |
| SDA       | GPIO **18** |
| SCL       | GPIO **23** |

De sensor deelt de I2C-bus met de audiocodec (I2C_NUM_0). De driver hergebruikt de bestaande bus als ADF die al heeft aangemaakt.

De AHT20-waarden worden elke 2 seconden gelogd op de seriële monitor:
```
I (1234) AHT20: Temp: 21.4 °C   Vochtigheid: 58.3 %
```

## UART-communicatieprotocol (Waveshare ↔ LyraT)

**Instellingen:** `115200 baud, 8N1`
**Transport:** length-prefixed frames — elk bericht begint met één lengte-byte `[LEN]` gevolgd door `LEN` payload bytes.

```
Frame formaat:
  [LEN:1 byte]  [PAYLOAD: LEN bytes]
```

### Wiring

| Waveshare GPIO | LyraT GPIO | Functie |
|---|---|---|
| GPIO 15 (TX) | GPIO 15 (RX) | Commando's Waveshare → LyraT |
| GPIO 16 (RX) | GPIO 13 (TX) | Antwoorden / events LyraT → Waveshare |
| GND | GND | Gemeenschappelijke massa |

### Richting: Waveshare → LyraT (commando's)

| Opcode | Naam | Payload | Beschrijving |
|---|---|---|---|
| `0x01` | `CMD_PLAY` | _(geen)_ | Start of hervat het afspelen |
| `0x02` | `CMD_STOP` | _(geen)_ | Stopt het afspelen volledig |
| `0x03` | `CMD_PAUSE` | _(geen)_ | Pauzeert het afspelen |
| `0x04` | `CMD_VOLUME` | `[vol:uint8]` | Stel volume in (0–100) |
| `0x05` | `CMD_SET_URL` | `[len:uint8][url:len bytes]` | Stel stream-URL in en start direct afspelen |
| `0x06` | `CMD_STATUS` | _(geen)_ | Vraag huidige status + volume op; LyraT antwoordt met STATUS_REPLY |
| `0x07` | `CMD_EQ` | `[band:uint8 0–9][gain:int8 dB]` | Stel gain in voor één EQ-band (ca. −13…+13 dB) |
| `0x0A` | `CMD_REQUEST_TIME` | _(geen)_ | Vraagt de LyraT om zijn UTC-tijd te sturen (antwoord: `CMD_TIME_SYNC`) |
| `0x0C` | `CMD_REQUEST_SENSOR` | _(geen)_ | Vraagt de LyraT om AHT20 temperatuur + vochtigheid (antwoord: `CMD_SENSOR_DATA`) |

### Richting: LyraT → Waveshare (antwoorden & events)

| Opcode | Naam | Payload | Beschrijving |
|---|---|---|---|
| `0x06` | `STATUS_REPLY` | `[0x06][status:uint8][volume:uint8]` | Antwoord op `CMD_STATUS` |
| `0x08` | `AUDIO_LEVEL` | `[0x08][peak_L:uint8][peak_R:uint8]` | Unsolicited — audioniveau links/rechts (0–255) voor VU-meter |
| `0x09` | `CMD_TIME_SYNC` | `[0x09][epoch_b3][epoch_b2][epoch_b1][epoch_b0]` | UTC epoch als big-endian uint32 |
| `0x0B` | `CMD_SENSOR_DATA` | `[0x0B][temp_hi][temp_lo][hum_hi][hum_lo]` | AHT20 data als big-endian int16, eenheid 1/10 (bijv. `0x00D8` = 21.6 °C, `0x01D1` = 46.5 %) |

### Statuscodes (in STATUS_REPLY)

| Byte | Betekenis |
|------|-----------|
| `0x00` | Stopped |
| `0x01` | Playing |
| `0x02` | Paused |
| `0xFF` | Error |

## Audio Pipeline

```
[WiFi] --> [HTTP stream] --> [MP3 decoder] --> [I2S] --> [Codec] --> [Speaker]
```

## Projectstructuur

```
main/
  play_mp3_control_example.c   - Hoofdapplicatie, WiFi, captive portal, audio pipeline, knoppen
  i2c_slave_ctrl.h/.c          - Opcode-definities en I2C slave driver
  uart_ctrl.h/.c               - UART driver en command parser / reply sender
  aht20.h/.c                   - AHT20 temperatuur- en vochtigheidssensor driver
  wifi_secrets.h               - WiFi credentials (git-ignored, lokaal)
  wifi_secrets.example.h       - Voorbeeld credentials
components/
  my_board/                    - Custom board-definitie (LyraT V4.3)
```


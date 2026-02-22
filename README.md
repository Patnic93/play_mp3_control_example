# NPO Radio 2 Internet Stream Player with I2C Control

ESP32 LyraT audio player that streams NPO Radio 2 (or any HTTP MP3 stream) over WiFi.
A front-end microcontroller can control playback via I2C.

## Features

- Streams live internet radio over WiFi (HTTP/HTTPS MP3)
- Default stream: `http://icecast.omroep.nl/radio2-bb-mp3` (NPO Radio 2)
- Physical buttons on the LyraT board for play/pause/stop/volume
- I2C slave interface for front-end control (change stream URL, volume, play/pause/stop)
- Status readback over I2C (playing / paused / stopped / error + current volume)

## Hardware

- **Board:** ESP32-LyraT V4.3
- **ESP-IDF:** v5.5.3
- **ESP-ADF:** located at `C:\esp\esp-adf` (set `ADF_PATH`)

## Build & Flash

```bash
idf.py -p PORT flash monitor
```

Or use the ESP-IDF extension in VS Code.

## Configuration

WiFi credentials and default stream URL are defined at the top of
`main/play_mp3_control_example.c`:

```c
#define WIFI_SSID        "your-ssid"
#define WIFI_PASSWORD    "your-password"
#define RADIO_STREAM_URL "http://icecast.omroep.nl/radio2-bb-mp3"
```

## Physical Button Controls (LyraT)

| Button | Action                        |
|--------|-------------------------------|
| Play   | Start / Pause / Resume        |
| Set    | Stop                          |
| Mode   | Restart current stream        |
| Vol+   | Volume up (step +10)          |
| Vol-   | Volume down (step -10)        |

## I2C Slave Interface

The LyraT acts as an **I2C slave** at address **0x42**.
A front-end microcontroller (master) can send commands at any time.

### Wiring

```
LyraT GPIO 21 (SDA)  ---[4.7kOhm pull-up to 3.3V]---  Front-end SDA
LyraT GPIO 22 (SCL)  ---[4.7kOhm pull-up to 3.3V]---  Front-end SCL
GND                  ---                               GND
```

- I2C address: `0x42` (7-bit)
- Max speed: 400 kHz

### Command Set

All commands are sent as a single I2C write transaction.

#### CMD_PLAY — `0x01`
Start or resume playback.
```
Master writes: [0x01]
```

#### CMD_STOP — `0x02`
Stop playback.
```
Master writes: [0x02]
```

#### CMD_PAUSE — `0x03`
Pause playback.
```
Master writes: [0x03]
```

#### CMD_VOLUME — `0x04`
Set volume level (0 = mute, 100 = maximum).
```
Master writes: [0x04][vol]
```
Example - set volume to 75%:
```
[0x04][0x4B]
```

#### CMD_SET_URL — `0x05`
Set a new HTTP stream URL and start playing immediately.
```
Master writes: [0x05][len][url bytes...]
```
- `len`: number of URL bytes (1..255)
- `url bytes`: ASCII characters of the URL, no null terminator needed

Example - switch to NPO Radio 1:
```
[0x05][0x26][h][t][t][p][:]...  (38 bytes for the URL)
```

#### CMD_STATUS — `0x06`
Request current status. After sending this, the master performs a
2-byte I2C read. Insert at least 5 ms between write and read.

```
Master writes: [0x06]
-- wait >= 5 ms --
Master reads:  [status][volume]
```

| status byte | meaning  |
|-------------|----------|
| `0x00`      | Stopped  |
| `0x01`      | Playing  |
| `0x02`      | Paused   |
| `0xFF`      | Error    |

The status is also updated automatically whenever a command is executed
or a physical button is pressed, so the master can poll at any time
without sending `0x06` first.

### Arduino / ESP32 Master Example

> This example targets an Arduino or ESP32 (Arduino framework) acting as I2C master.
> For a plain ESP32 master, replace `Wire.begin(SDA_PIN, SCL_PIN)` with
> `i2c_master_init()` from ESP-IDF.

```cpp
#include <Wire.h>

// Change these to the SDA/SCL pins of your master board
#define MASTER_SDA_PIN  21
#define MASTER_SCL_PIN  22

#define LYRAT_ADDR      0x42   // 7-bit I2C address of the LyraT slave

// Player status codes returned by lyrat_get_status()
#define LYRAT_STOPPED   0x00
#define LYRAT_PLAYING   0x01
#define LYRAT_PAUSED    0x02
#define LYRAT_ERROR     0xFF

// -----------------------------------------------------------------------
// Command helpers
// -----------------------------------------------------------------------

// CMD_PLAY (0x01) - start or resume playback
void lyrat_play() {
    Wire.beginTransmission(LYRAT_ADDR);
    Wire.write(0x01);
    Wire.endTransmission();
}

// CMD_STOP (0x02) - stop playback
void lyrat_stop() {
    Wire.beginTransmission(LYRAT_ADDR);
    Wire.write(0x02);
    Wire.endTransmission();
}

// CMD_PAUSE (0x03) - pause playback
void lyrat_pause() {
    Wire.beginTransmission(LYRAT_ADDR);
    Wire.write(0x03);
    Wire.endTransmission();
}

// CMD_VOLUME (0x04) - set volume 0 (mute) .. 100 (max)
void lyrat_set_volume(uint8_t vol) {
    if (vol > 100) vol = 100;
    Wire.beginTransmission(LYRAT_ADDR);
    Wire.write(0x04);
    Wire.write(vol);
    Wire.endTransmission();
}

// CMD_SET_URL (0x05) - set HTTP stream URL and start playing immediately
// url must be a null-terminated ASCII string, max 255 characters
void lyrat_set_url(const char* url) {
    uint8_t len = (uint8_t)strlen(url);
    Wire.beginTransmission(LYRAT_ADDR);
    Wire.write(0x05);
    Wire.write(len);
    Wire.write((const uint8_t*)url, len);
    Wire.endTransmission();
}

// CMD_STATUS (0x06) - read current status byte and volume
// Returns: LYRAT_STOPPED / LYRAT_PLAYING / LYRAT_PAUSED / LYRAT_ERROR
// *volume receives the current volume level (0-100)
uint8_t lyrat_get_status(uint8_t* volume) {
    Wire.beginTransmission(LYRAT_ADDR);
    Wire.write(0x06);
    Wire.endTransmission(true);

    delay(5);  // give the LyraT at least 5 ms to prepare the reply

    uint8_t n = Wire.requestFrom((uint8_t)LYRAT_ADDR, (uint8_t)2);
    if (n < 2) {
        if (volume) *volume = 0;
        return LYRAT_ERROR;
    }
    uint8_t status = Wire.read();
    if (volume) *volume = Wire.read();
    return status;
}

// -----------------------------------------------------------------------
// Arduino sketch
// -----------------------------------------------------------------------

void setup() {
    Serial.begin(115200);

    // For ESP32 boards pass the SDA/SCL pins; for standard Arduino omit them
    Wire.begin(MASTER_SDA_PIN, MASTER_SCL_PIN);
    Wire.setClock(400000);   // 400 kHz Fast Mode

    delay(500);  // wait for LyraT to boot

    Serial.println("Connecting to LyraT...");

    // Start with volume at 60 and play the default NPO Radio 2 stream
    lyrat_set_volume(60);
    lyrat_play();
    Serial.println("Play command sent.");
}

// Status labels for Serial output
static const char* status_str(uint8_t s) {
    switch (s) {
        case LYRAT_STOPPED: return "STOPPED";
        case LYRAT_PLAYING: return "PLAYING";
        case LYRAT_PAUSED:  return "PAUSED";
        default:            return "ERROR";
    }
}

void loop() {
    // Poll status every 5 seconds and print to Serial
    static uint32_t last_poll = 0;
    if (millis() - last_poll >= 5000) {
        last_poll = millis();
        uint8_t vol;
        uint8_t st = lyrat_get_status(&vol);
        Serial.printf("Status: %s  Volume: %u\n", status_str(st), vol);
    }

    // Example: switch to NPO Radio 1 when 'u' is typed in Serial Monitor
    if (Serial.available()) {
        char c = Serial.read();
        if (c == 'u') {
            lyrat_set_url("http://icecast.omroep.nl/radio1-bb-mp3");
            Serial.println("Switched to NPO Radio 1");
        } else if (c == '2') {
            lyrat_set_url("http://icecast.omroep.nl/radio2-bb-mp3");
            Serial.println("Switched to NPO Radio 2");
        } else if (c == 'p') {
            lyrat_pause();
            Serial.println("Paused");
        } else if (c == 'r') {
            lyrat_play();
            Serial.println("Resumed");
        } else if (c == 's') {
            lyrat_stop();
            Serial.println("Stopped");
        } else if (c == '+') {
            uint8_t vol;
            lyrat_get_status(&vol);
            if (vol <= 90) lyrat_set_volume(vol + 10);
        } else if (c == '-') {
            uint8_t vol;
            lyrat_get_status(&vol);
            if (vol >= 10) lyrat_set_volume(vol - 10);
        }
    }
}
```

## Audio Pipeline

```
[WiFi] --> [HTTP stream] --> [MP3 decoder] --> [I2S] --> [Codec] --> [Speaker]
```

## Project Structure

```
main/
  play_mp3_control_example.c   - Main application, WiFi, audio pipeline, button handling
  i2c_slave_ctrl.h             - I2C command definitions and public API
  i2c_slave_ctrl.c             - I2C slave driver and command parser
components/
  my_board/                    - Custom board definition (LyraT V4.3)
```

## Technical Support

- ESP-IDF / ESP-ADF questions: [esp32.com forum](https://esp32.com/viewforum.php?f=20)
- Bug reports: [GitHub Issues](https://github.com/Patnic93/play_mp3_control_example/issues)


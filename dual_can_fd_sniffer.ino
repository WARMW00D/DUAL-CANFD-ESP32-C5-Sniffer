/*
  dual_can_fd_sniffer — сниффер двух CAN-шин, бинарный лог на SD, портал.
  Плата: ESP32-C5 Mini, без PSRAM, без BLE.
  Сжатие: heatshrink (LZSS) — src/heatshrink/.
  WiFi: 5 ГГц (Wi-Fi 6 / 802.11ax), канал 36 (не-DFS).
*/
#include "esp_wifi.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_timer.h"
#include "esp_sleep.h"
#include <HWCDC.h>
#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <sys/time.h>
#include <Preferences.h>
#include <stdarg.h>
#include <vector>

extern "C" {
  #include "src/heatshrink/heatshrink_encoder.h"
}

// ---------- Пины ----------
#define PIN_CAN0_RX     5
#define PIN_CAN1_RX     4
#define PIN_SD_CS      10
#define PIN_SD_MOSI     9
#define PIN_SD_SCK      8
#define PIN_SD_MISO     7
#define PIN_RTC_SDA     1
#define PIN_RTC_SCL     6
#define PIN_BOOT       28
#define PIN_ACC         0  // PC817 collector; external 10k pull-up to ESP 3V3
#define ACC_ON_LEVEL    LOW
#define SLEEP_GRACE_MS  5000UL

// ---------- Настройки ----------
#define SD_SPI_HZ       20000000UL
#define LOG_FILE_MAX    (10UL * 1024UL * 1024UL)
#define LOG_IDLE_CLOSE_MS  10000UL
#define LOG_ANCHOR_MS      60000UL
#define LOG_FLUSH_MS       500UL
#define BOOT_HOLD_MS       100

#define AP_SSID       "C5-SNIFFER"
#define AP_PASS       "canlogger"
#define AP_TIMEOUT_MS (5UL * 60UL * 1000UL)

#define ERRLOG_PATH "/errors.log"
#define ERRLOG_OLD_PATH "/errors.old.log"
#define ERRLOG_MAX_BYTES (256UL * 1024UL)

// ---------- Структуры ----------
struct BusConfig {
  bool enabled;
  char name[8];
  uint32_t bitrate;
  uint8_t fd_mode;      // 0 = Classic, 1 = FD
  uint32_t data_bitrate;
};

static BusConfig busCfg[2];
static Preferences prefs;
static bool uiEn = false;

struct Rec {
  uint32_t ts_us;
  uint32_t id;
  uint8_t  bus_fl;
  uint8_t  len;
  uint8_t  data[64];
  uint8_t  pad[2];
};

struct BusCtx {
  twai_node_handle_t node;
  uint8_t       busIndex;
  Rec  ring[512];
  volatile uint32_t head;
  volatile uint32_t tail;
  volatile uint32_t dropCount;
  volatile uint32_t rxCount;
};
static BusCtx busCtx[2];

static uint8_t wbuf[16384];
static size_t  wlen = 0;
static File    logFile;
static bool    sdReady = false;
static bool    paused = false;
static bool    apActive = false;
static bool    hadClient = false;
static uint32_t apStartMs = 0;
static uint32_t lastFlushMs = 0;
static uint32_t lastAnchorMs = 0;
static uint32_t lastCanActivityMs = 0;
static String  curLogPath;
static uint32_t fileFrames = 0;
static uint32_t fileBytes = 0;
static uint32_t totalFrames = 0;
static bool    safeMode = false;
static bool    bootArmed = false;
static uint32_t bootDownMs = 0;
static SemaphoreHandle_t errMutex = nullptr;
static uint32_t activityMark[2] = {0, 0};   // для ожидания новой активности

// SD benchmark runs in small steps from loop(), keeping HTTP responsive.
struct SdTestState {
  bool running = false;
  uint8_t stage = 0;  // 0 idle, 1 write, 2 verify, 3 log simulation, 4 done, 5 error
  String path, error;
  File file;
  uint32_t started = 0, stageStart = 0, nextWrite = 0, lastFlush = 0;
  uint32_t index = 0, written = 0, read = 0, simulated = 0;
  uint32_t writeMs = 0, readMs = 0, simMs = 0;
  uint32_t maxWriteUs = 0, slowWrites = 0, badWords = 0;
  uint32_t maxSimWriteUs = 0, maxFlushUs = 0, failures = 0;
};
static SdTestState sdTest;
static uint32_t sdTestBuf[1024];
static const uint32_t SD_TEST_BYTES = 4UL * 1024UL * 1024UL;

// ---------- Язык ----------
#define TR(ru, en) String(uiEn ? (en) : (ru))
void loadUiLanguage() {
  prefs.begin("ui", true);
  uiEn = prefs.getString("lang", "ru") == "en";
  prefs.end();
}
void saveUiLanguage(bool en) {
  uiEn = en;
  prefs.begin("ui", false);
  prefs.putString("lang", en ? "en" : "ru");
  prefs.end();
}

// ---------- Ошибки ----------
static void errWriteLine(const String& line) {
  File f = SD.open(ERRLOG_PATH, FILE_APPEND);
  if (!f) return;
  f.print(line);
  size_t sz = f.size();
  f.close();
  if (sz > ERRLOG_MAX_BYTES) {
    SD.remove(ERRLOG_OLD_PATH);
    SD.rename(ERRLOG_PATH, ERRLOG_OLD_PATH);
  }
}

void errLog(const char* fmt, ...) {
  char msg[256];
  va_list ap; va_start(ap, fmt); vsnprintf(msg, sizeof(msg), fmt, ap); va_end(ap);
  time_t t = time(nullptr);
  struct tm tm; gmtime_r(&t, &tm);
  char ts[24];
  snprintf(ts, sizeof(ts), "%04d-%02d-%02d %02d:%02d:%02d", tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  String line = String(ts) + " [fw 0.2.5, " + String(millis()) + " ms] " + msg + "\r\n";
  Serial.print("[ERR] " + line);
  if (!errMutex) errMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(errMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    if (sdReady) errWriteLine(line);
    xSemaphoreGive(errMutex);
  }
}

// ---------- Конфигурация ----------
static const uint32_t NOMINAL_RATES[] = {100000, 125000, 250000, 500000, 800000, 1000000};
static const uint32_t DATA_RATES[] = {100000, 125000, 250000, 500000, 800000, 1000000,
                                     2000000, 4000000, 5000000, 8000000};

static bool rateAllowed(uint32_t rate, bool dataPhase) {
  const uint32_t* rates = dataPhase ? DATA_RATES : NOMINAL_RATES;
  size_t count = dataPhase ? sizeof(DATA_RATES) / sizeof(DATA_RATES[0])
                          : sizeof(NOMINAL_RATES) / sizeof(NOMINAL_RATES[0]);
  for (size_t i = 0; i < count; i++) if (rates[i] == rate) return true;
  return false;
}

static bool validBusConfig(const BusConfig& c) {
  return rateAllowed(c.bitrate, false) && c.fd_mode <= 1 &&
         (c.fd_mode == 0 ? c.data_bitrate == 0 :
          rateAllowed(c.data_bitrate, true) && c.data_bitrate >= c.bitrate);
}

static BusConfig defaultBusConfig(int i) {
  BusConfig c{};
  c.enabled = true;
  snprintf(c.name, sizeof(c.name), "CAN%d", i);
  c.bitrate = 500000;
  return c;
}

void loadConfig() {
  for (int i = 0; i < 2; i++) busCfg[i] = defaultBusConfig(i);
  if (!prefs.begin("can", true)) return;
  // One blob makes subsequent two-bus updates atomic. Migrate old keys.
  if (prefs.getBytesLength("buses_v2") == sizeof(busCfg)) {
    prefs.getBytes("buses_v2", busCfg, sizeof(busCfg));
  } else {
    for (int i = 0; i < 2; i++) {
      const char* key = i == 0 ? "bus0" : "bus1";
      if (prefs.getBytesLength(key) == sizeof(BusConfig))
        prefs.getBytes(key, &busCfg[i], sizeof(BusConfig));
    }
  }
  prefs.end();
  for (int i = 0; i < 2; i++) {
    busCfg[i].name[sizeof(busCfg[i].name) - 1] = '\0';
    if (!busCfg[i].name[0]) snprintf(busCfg[i].name, sizeof(busCfg[i].name), "CAN%d", i);
    if (busCfg[i].fd_mode == 0) busCfg[i].data_bitrate = 0;
    if (!validBusConfig(busCfg[i])) {
      errLog("CAN%d: invalid stored configuration; using Classic 500 kbit/s", i);
      busCfg[i] = defaultBusConfig(i);
    }
  }
}

static bool saveConfig(const BusConfig* config) {
  if (!prefs.begin("can", false)) return false;
  bool ok = prefs.putBytes("buses_v2", config, sizeof(busCfg)) == sizeof(busCfg);
  prefs.end();
  return ok;
}

// ---------- timegm ----------
static time_t timegm_utc(struct tm* tm) {
  char* savedPtr = getenv("TZ");
  String saved = savedPtr ? String(savedPtr) : String();
  setenv("TZ", "UTC0", 1); tzset();
  time_t t = mktime(tm);
  if (saved.length()) setenv("TZ", saved.c_str(), 1); else unsetenv("TZ");
  tzset();
  return t;
}

// ---------- PCF8563 ----------
static inline uint8_t bcd2dec(uint8_t b) { return (b & 0x0F) + ((b >> 4) * 10); }
static inline uint8_t dec2bcd(uint8_t d) { return ((d / 10) << 4) | (d % 10); }

class PCF8563 {
public:
  TwoWire* wire = nullptr;
  bool present = false;
  bool begin(TwoWire* w) {
    wire = w;
    wire->beginTransmission(0x51);
    if (wire->endTransmission() != 0) { present = false; return false; }
    present = true;
    writeReg(0x00, 0x00); writeReg(0x01, 0x00);
    return true;
  }
  bool lostPower() { return !present || (readReg(0x02) & 0x80) != 0; }
  time_t now() {
    if (!present) return 0;
    wire->beginTransmission(0x51); wire->write((uint8_t)0x02);
    if (wire->endTransmission(false) != 0) return 0;
    if (wire->requestFrom((uint8_t)0x51, (uint8_t)7) != 7) return 0;
    uint8_t sec = wire->read() & 0x7F; uint8_t min = wire->read() & 0x7F;
    uint8_t hour = wire->read() & 0x3F; uint8_t day = wire->read() & 0x3F;
    (void)        wire->read();
    uint8_t month = wire->read() & 0x1F; uint8_t year = wire->read();
    struct tm tm{};
    tm.tm_year = 100 + bcd2dec(year); tm.tm_mon = bcd2dec(month) - 1;
    tm.tm_mday = bcd2dec(day); tm.tm_hour = bcd2dec(hour);
    tm.tm_min = bcd2dec(min); tm.tm_sec = bcd2dec(sec);
    return timegm_utc(&tm);
  }
  bool adjust(time_t epoch) {
    if (!present) return false;
    struct tm tm; gmtime_r(&epoch, &tm);
    wire->beginTransmission(0x51); wire->write((uint8_t)0x02);
    wire->write(dec2bcd(tm.tm_sec) & 0x7F); wire->write(dec2bcd(tm.tm_min) & 0x7F);
    wire->write(dec2bcd(tm.tm_hour) & 0x3F); wire->write(dec2bcd(tm.tm_mday) & 0x3F);
    wire->write((uint8_t)(tm.tm_wday & 0x07)); wire->write(dec2bcd(tm.tm_mon + 1) & 0x1F);
    wire->write(dec2bcd(tm.tm_year % 100));
    return wire->endTransmission() == 0;
  }
private:
  void writeReg(uint8_t r, uint8_t v) { wire->beginTransmission(0x51); wire->write(r); wire->write(v); wire->endTransmission(); }
  uint8_t readReg(uint8_t r) {
    wire->beginTransmission(0x51); wire->write(r);
    if (wire->endTransmission(false) != 0) return 0xFF;
    if (wire->requestFrom((uint8_t)0x51, (uint8_t)1) != 1) return 0xFF;
    return wire->read();
  }
};

static PCF8563 rtc;
static WebServer server(80);

// ---------- HsLog ----------
class HsLog {
public:
  heatshrink_encoder* enc = nullptr;
  File* file = nullptr;
  uint32_t totalBytes = 0;
  uint8_t outbuf[4096];

  bool begin(File* f, uint8_t window_sz2 = 10, uint8_t lookahead_sz2 = 5) {
    file = f; totalBytes = 0;
    enc = heatshrink_encoder_alloc(window_sz2, lookahead_sz2);
    if (!enc) return false;
    heatshrink_encoder_reset(enc);
    return true;
  }
  void write(const uint8_t* data, size_t len) {
    if (!enc || !file) return;
    totalBytes += len;
    const uint8_t* p = data;
    size_t remaining = len;
    while (remaining > 0) {
      size_t sunk = 0;
      if (heatshrink_encoder_sink(enc, (uint8_t*)p, remaining, &sunk) < 0) return;
      p += sunk; remaining -= sunk;
      size_t total_polled = 0;
      HSE_poll_res pres;
      do {
        size_t polled = 0;
        pres = heatshrink_encoder_poll(enc, outbuf, sizeof(outbuf), &polled);
        if (pres < 0) return;
        if (polled > 0) file->write(outbuf, polled);
        total_polled += polled;
      } while (pres == HSER_POLL_MORE);
      if (sunk == 0 && total_polled == 0) break;
    }
  }
  void end() {
    if (!enc || !file) return;
    HSE_finish_res fres = heatshrink_encoder_finish(enc);
    while (fres == HSER_FINISH_MORE) {
      size_t polled = 0;
      if (heatshrink_encoder_poll(enc, outbuf, sizeof(outbuf), &polled) < 0) break;
      if (polled > 0) file->write(outbuf, polled);
      fres = heatshrink_encoder_finish(enc);
    }
    heatshrink_encoder_free(enc);
    enc = nullptr; file = nullptr;
  }
};
static HsLog hsLog;

// ---------- Утилиты ----------
static String fmtDateTime(time_t t) {
  struct tm tm; gmtime_r(&t, &tm);
  char b[24]; snprintf(b, sizeof(b), "%04d-%02d-%02d %02d:%02d:%02d", tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
  return String(b);
}

static void stopAp() { WiFi.softAPdisconnect(true); WiFi.mode(WIFI_OFF); apActive = false; }

// ---------- Ожидание любого включённого канала ----------
static bool anyBusActive() {
  for (int i = 0; i < 2; i++) {
    if (!busCfg[i].enabled) continue;
    if ((uint32_t)(busCtx[i].rxCount - activityMark[i]) != 0) return true;
  }
  return false;
}

// ---------- Логирование ----------
static void flushToSd() {
  if (!logFile || wlen == 0) return;
  hsLog.write(wbuf, wlen);
  static const uint8_t END[4] = {0xFF,0xFF,0xFF,0xFF};
  logFile.write(END, 4);
  fileBytes = logFile.size(); wlen = 0; lastFlushMs = millis();
}

static void appendBuf(const uint8_t* d, size_t n) {
  if (wlen + n > sizeof(wbuf)) flushToSd();
  memcpy(wbuf + wlen, d, n);
  wlen += n;
}

static bool openLogFile() {
  if (logFile) { logFile.close(); logFile = File(); }
  time_t now = time(nullptr); struct tm tm; gmtime_r(&now, &tm);
  char dateDir[16]; snprintf(dateDir, sizeof(dateDir), "/%04d-%02d-%02d", tm.tm_year+1900, tm.tm_mon+1, tm.tm_mday);
  if (!SD.exists(dateDir)) SD.mkdir(dateDir);
  File d = SD.open(dateDir);
  if (!d) return false;
  int maxIdx = -1; File f = d.openNextFile();
  while (f) {
    int idx;
    if (sscanf(f.name(), "can_%d.hs", &idx) == 1 && idx > maxIdx) maxIdx = idx;
    f = d.openNextFile();
  }
  d.close();
  char path[48]; snprintf(path, sizeof(path), "%s/can_%04d.hs", dateDir, maxIdx + 1);
  logFile = SD.open(path, FILE_WRITE);
  if (!logFile) return false;
  curLogPath = path; fileFrames = 0; fileBytes = 0;
  uint8_t hdr[58]; memset(hdr, 0, sizeof(hdr));
  memcpy(hdr, "C5CAN\0\0\0", 8); hdr[8] = 1; memcpy(hdr + 9, "0.2.5\0\0\0\0\0\0\0", 12); hdr[21] = 2;
  for (int i = 0; i < 2; i++) {
    int off = 22 + i * 14; hdr[off] = i; uint16_t br = busCfg[i].bitrate / 1000;
    hdr[off+1] = br & 0xFF; hdr[off+2] = (br >> 8) & 0xFF;
    hdr[off+3] = busCfg[i].fd_mode;
    uint16_t dbr = busCfg[i].data_bitrate / 1000;
    hdr[off+4] = dbr & 0xFF; hdr[off+5] = (dbr >> 8) & 0xFF;
    for (int k = 0; k < 8 && busCfg[i].name[k]; k++) hdr[off+6+k] = busCfg[i].name[k];
  }
  uint32_t us = (uint32_t)esp_timer_get_time(); uint32_t sec = (uint32_t)time(nullptr);
  memcpy(hdr+50, &us, 4); memcpy(hdr+54, &sec, 4);
  wlen = 0; if (!hsLog.begin(&logFile)) { logFile.close(); return false; }
  hsLog.write(hdr, sizeof(hdr)); logFile.flush(); fileBytes = logFile.size();
  lastAnchorMs = millis(); lastFlushMs = millis();
  Serial.printf("LOG: %s opened\n", path);
  return true;
}

static void closeLogFile(uint8_t reason) {
  if (!logFile) return;
  uint32_t us = (uint32_t)esp_timer_get_time();
  uint8_t rec[6]; memcpy(rec, &us, 4); rec[4] = 0x80 | reason; rec[5] = 0;
  appendBuf(rec, 6); flushToSd(); hsLog.end();
  logFile.flush(); logFile.close(); logFile = File();
  Serial.printf("LOG: closed %s (%u frames)\n", curLogPath.c_str(), (unsigned)fileFrames);
  curLogPath = "";
}

static void encodeFrame(const Rec* r) {
  uint8_t rec[10 + 64];
  memcpy(rec, &r->ts_us, 4); rec[4] = r->bus_fl; memcpy(rec+5, &r->id, 4); rec[9] = r->len;
  memcpy(rec + 10, r->data, r->len); appendBuf(rec, 10 + r->len); fileFrames++; totalFrames++;
}

static void writeAnchor() {
  uint32_t us = (uint32_t)esp_timer_get_time(); uint32_t sec = (uint32_t)time(nullptr);
  uint8_t rec[10]; memcpy(rec, &us, 4); rec[4] = 0x80 | 0; rec[5] = 4; memcpy(rec+6, &sec, 4);
  appendBuf(rec, 10);
}

// ---------- CAN ----------
static bool IRAM_ATTR onRx(twai_node_handle_t h, const twai_rx_done_event_data_t*, void* ctx) {
  BusCtx* b = (BusCtx*)ctx;
  uint32_t head = b->head, next = (head + 1) & 511;
  uint8_t discard[64];
  bool full = next == b->tail;
  twai_frame_t f; memset(&f, 0, sizeof(f));
  f.buffer = full ? discard : b->ring[head].data; f.buffer_len = 64;
  if (twai_node_receive_from_isr(h, &f) != ESP_OK) return false;
  if (full) { b->dropCount++; return false; }
  if (f.header.fdf && !busCfg[b->busIndex].fd_mode) return false;
  uint32_t us = (uint32_t)esp_timer_get_time();
  uint8_t busfl = b->busIndex;
  if (f.header.ide) busfl |= 0x08;
  if (f.header.rtr) busfl |= 0x10;
  if (f.header.fdf) busfl |= 0x20;
  if (f.header.brs) busfl |= 0x40;
  b->ring[head].ts_us = us; b->ring[head].id = f.header.id; b->ring[head].bus_fl = busfl;
  b->ring[head].len = f.header.fdf ? twaifd_dlc2len(f.header.dlc) : (f.header.dlc > 8 ? 8 : f.header.dlc);
  b->head = next; b->rxCount++;
  return false;
}

static bool startBus(BusCtx* b, uint8_t idx, const BusConfig& config) {
  if (!validBusConfig(config)) return false;
  uint32_t bitrate = config.bitrate;
  uint32_t fd_bitrate = config.fd_mode ? config.data_bitrate : 0;
  b->busIndex = idx;
  twai_onchip_node_config_t cfg; memset(&cfg, 0, sizeof(cfg));
  cfg.io_cfg.rx = (gpio_num_t)(idx == 0 ? PIN_CAN0_RX : PIN_CAN1_RX);
  cfg.io_cfg.tx = (gpio_num_t)-1; cfg.io_cfg.quanta_clk_out = (gpio_num_t)-1; cfg.io_cfg.bus_off_indicator = (gpio_num_t)-1;
  cfg.bit_timing.bitrate = bitrate; cfg.bit_timing.sp_permill = 800;
  if (fd_bitrate) { cfg.data_timing.bitrate = fd_bitrate; cfg.data_timing.sp_permill = 750; }
  cfg.fail_retry_cnt = 0; cfg.tx_queue_depth = 1; cfg.flags.enable_listen_only = 1;
  esp_err_t e = twai_new_node_onchip(&cfg, &b->node);
  if (e != ESP_OK) {
    b->node = nullptr;
    errLog("CAN%d: create failed (%s), mode=%s nominal=%lu data=%lu", idx,
           esp_err_to_name(e), config.fd_mode ? "FD" : "Classic",
           (unsigned long)bitrate, (unsigned long)fd_bitrate);
    return false;
  }
  twai_event_callbacks_t cb; memset(&cb, 0, sizeof(cb)); cb.on_rx_done = onRx;
  e = twai_node_register_event_callbacks(b->node, &cb, b);
  if (e == ESP_OK) e = twai_node_enable(b->node);
  if (e != ESP_OK) {
    errLog("CAN%d: start failed: %s", idx, esp_err_to_name(e));
    twai_node_delete(b->node); b->node = nullptr;
    return false;
  }
  Serial.printf("CAN%d: %s nominal=%lu data=%lu bit/s, listen-only\n", idx,
                config.fd_mode ? "FD" : "Classic", (unsigned long)bitrate,
                (unsigned long)fd_bitrate);
  return true;
}

static bool ringHasData() { return busCtx[0].tail != busCtx[0].head || busCtx[1].tail != busCtx[1].head; }
static void drainToDummy() { for (int i = 0; i < 2; i++) busCtx[i].tail = busCtx[i].head; }

static void dispatch() {
  bool any = false;
  while (true) {
    bool h0 = busCtx[0].tail != busCtx[0].head, h1 = busCtx[1].tail != busCtx[1].head;
    if (!h0 && !h1) break;
    any = true; int pick;
    if (h0 && h1) {
      int32_t d = (int32_t)(busCtx[0].ring[busCtx[0].tail].ts_us - busCtx[1].ring[busCtx[1].tail].ts_us);
      pick = (d <= 0) ? 0 : 1;
    } else { pick = h0 ? 0 : 1; }
    BusCtx* b = &busCtx[pick];
    encodeFrame((Rec*)&b->ring[b->tail]);
    b->tail = (b->tail + 1) & 511;
  }
  if (any) lastCanActivityMs = millis();
}

// ---------- Портал ----------
const char* PAGE_STYLE =
  "<style>"
  "body{background:#0d0d0d;color:#d8d8d8;font-family:'Consolas','Courier New',monospace;padding:16px;max-width:720px;margin:0 auto;}"
  "h2,h3{color:#4caf50;} a{color:#4caf50;}"
  "input,button,select{background:#1a1a1a;color:#d8d8d8;border:1px solid #4caf50;padding:6px 10px;font-family:inherit;border-radius:4px;max-width:100%;box-sizing:border-box;}"
  "button{cursor:pointer;} button:hover{background:#4caf50;color:#0d0d0d;}"
  ".langbar{text-align:right;margin:0 0 8px 0;} button.lang{background:#c62828;border-color:#c62828;color:#fff;font-weight:bold;}"
  "ul{list-style:none;padding-left:0;} .fold{display:flex;align-items:flex-start;gap:8px;margin:6px 0;}"
  ".meta{font-weight:normal;color:#9a9a9a;} li{padding:4px 0;border-bottom:1px solid #222;} hr{border-color:#333;}"
  "</style>";

String htmlHead(const String& title) {
  String back = server.uri();
  return "<!DOCTYPE html><html lang='" + String(uiEn ? "en" : "ru") + "'><head><meta charset='utf-8'>"
         "<meta name='viewport' content='width=device-width, initial-scale=1'><title>" + title + "</title>" + String(PAGE_STYLE) + "</head><body>"
         "<form class='langbar' method='POST' action='/set-lang'><input type='hidden' name='back' value='" + back + "'>"
         "<button class='lang' type='submit' name='lang' value='" + String(uiEn ? "ru" : "en") + "'>" +
         String(uiEn ? "Русский" : "English") + "</button></form>";
}

void handleSetLang() {
  saveUiLanguage(server.arg("lang") == "en");
  String back = server.arg("back");
  if (back != "/logs" && back != "/errors" && back != "/config" && back != "/sd-test" && back != "/clock") back = "/";
  server.sendHeader("Location", back); server.send(303);
}

bool isValidDateFolder(const String& name) {
  if (name.length() != 10) return false;
  int Y, M, D;
  if (sscanf(name.c_str(), "%d-%d-%d", &Y, &M, &D) != 3) return false;
  return (Y >= 2020 && Y <= 2099 && M >= 1 && M <= 12 && D >= 1 && D <= 31);
}

void handleDeleteDay() {
  if (sdTest.running) { server.send(409, "text/plain", "SD test running; wait until completion"); return; }
  String body = server.arg("date");
  body.trim();
  if (!isValidDateFolder(body)) { server.send(400, "text/plain", "bad date"); return; }
  String path = "/" + body;
  File dir = SD.open(path);
  if (!dir) { server.send(404, "text/plain", "Folder not found"); return; }
  File f = dir.openNextFile();
  while (f) {
    String fp = String(path) + "/" + f.name();
    if (!f.isDirectory()) SD.remove(fp.c_str());
    f = dir.openNextFile();
  }
  dir.close();
  bool ok = SD.rmdir(path.c_str());
  server.send(ok ? 200 : 500, "text/plain", ok ? "deleted" : "failed");
}

void handleRoot() {
  time_t rtcT = rtc.present ? rtc.now() : 0;
  String state;
  if (!sdReady) state = TR("НЕТ SD", "NO SD");
  else if (paused) state = TR("ПАУЗА (портал)", "PAUSED (portal)");
  else if (logFile) state = TR("ЗАПИСЬ", "LOGGING");
  else if (anyBusActive()) state = TR("ОЖИДАНИЕ", "IDLE");
  else state = TR("ЖДУ CAN", "WAITING FOR CAN");

  String busStatus;
  for (int i = 0; i < 2; i++) {
    const BusConfig& c = busCfg[i];
    busStatus += "<li>CAN" + String(i) + ": ";
    if (!c.enabled) busStatus += TR("выключен", "disabled");
    else {
      busStatus += String(c.fd_mode ? "CAN FD" : "Classic CAN") + " / " + String(c.bitrate / 1000);
      if (c.fd_mode) busStatus += " / " + String(c.data_bitrate / 1000);
      busStatus += " kbit/s — " + (busCtx[i].node ? TR("работает", "running") : TR("ОШИБКА ЗАПУСКА", "START FAILED"));
    }
    busStatus += "</li>";
  }
  String html = htmlHead(TR("Сниффер C5", "C5 Sniffer")) +
    "<h2>" + TR("Состояние", "Status") + "</h2><ul>"
    "<li>" + TR("Состояние", "State") + ": <b>" + state + "</b></li>"
    "<li>" + TR("RTC", "RTC") + ": " + (rtc.present ? fmtDateTime(rtcT) : "not found") + "</li>"
    "<li>" + TR("Heap", "Heap") + ": " + String(ESP.getFreeHeap() / 1024) + " KB</li>"
    "<li>" + TR("SD", "SD") + ": " + (sdReady ? TR("OK", "OK") : TR("ОШИБКА", "FAIL")) + "</li>"
    "<li>" + TR("Файл", "File") + ": " + (curLogPath.length() ? curLogPath : "—") + "</li>"
    "<li>" + TR("Кадров в файле", "Frames in file") + ": " + String(fileFrames) + "</li>"
    "<li>" + TR("Принято CAN0", "RX CAN0") + ": " + String((uint32_t)busCtx[0].rxCount) + " (drop " + String((uint32_t)busCtx[0].dropCount) + ")</li>"
    "<li>" + TR("Принято CAN1", "RX CAN1") + ": " + String((uint32_t)busCtx[1].rxCount) + " (drop " + String((uint32_t)busCtx[1].dropCount) + ")</li>"
    + busStatus + "</ul><p><a href='/config'>" + TR("Настройки шин", "Bus settings") + "</a></p>"
    "<p><a href='/logs'>" + TR("Логи на карте", "Logs on card") + "</a></p>"
    "<p><a href='/errors'>" + TR("Журнал ошибок", "Error log") + "</a></p>"
    "<p><a href='/clock'>" + TR("Дата и время", "Date and time") + "</a></p>"
    "<p><a href='/sd-test'>" + TR("Тест SD-карты", "SD card test") + "</a></p></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

static String htmlEscape(const String& value) {
  String out;
  for (size_t i = 0; i < value.length(); i++) {
    char c = value[i];
    if (c == '&') out += "&amp;";
    else if (c == '<') out += "&lt;";
    else if (c == '>') out += "&gt;";
    else if (c == '\'') out += "&#39;";
    else if (c == '"') out += "&quot;";
    else out += c;
  }
  return out;
}

void handleConfigPage() {
  String html = htmlHead(TR("Настройки CAN", "CAN Settings")) +
    "<h2>" + TR("Настройки шин", "Bus settings") + "</h2><p><a href='/'>&larr; " + TR("на главную", "home") + "</a></p>"
    "<p>" + TR("CAN FD принимает также Classic CAN. Скорости должны соответствовать шине.",
                 "CAN FD also receives Classic CAN. Bitrates must match the bus.") + "</p>"
    "<form method='POST' action='/api/config'>";
  for (int i = 0; i < 2; i++) {
    String p = String("b") + String(i);
    const BusConfig& c = busCfg[i];
    html += "<fieldset><legend>CAN" + String(i) + "</legend>"
      "<label><input type='checkbox' name='" + p + "_en' " + String(c.enabled ? "checked" : "") + "> " + TR("Включён", "Enabled") + "</label><br>"
      "<label>" + TR("Имя", "Name") + " <input name='" + p + "_name' value='" + htmlEscape(String(c.name)) + "' maxlength='7'></label><br>"
      "<label>" + TR("Режим", "Mode") + " <select name='" + p + "_mode' id='" + p + "_mode' onchange='fdMode(" + String(i) + ")'>"
      "<option value='0'" + String(c.fd_mode ? "" : " selected") + ">Classic CAN</option>"
      "<option value='1'" + String(c.fd_mode ? " selected" : "") + ">CAN FD</option></select></label><br>"
      "<label>" + TR("Номинальная скорость", "Nominal bitrate") + " <select name='" + p + "_bitrate'>";
    for (uint32_t br : NOMINAL_RATES)
      html += "<option value='" + String(br) + "'" + String(c.bitrate == br ? " selected" : "") + ">" + String(br / 1000) + " kbit/s</option>";
    html += "</select></label><br><label id='" + p + "_data_row'>" + TR("Скорость данных FD (BRS)", "FD data bitrate (BRS)") +
      " <select name='" + p + "_data' id='" + p + "_data'>";
    uint32_t selected = c.fd_mode ? c.data_bitrate : 2000000;
    for (uint32_t br : DATA_RATES)
      html += "<option value='" + String(br) + "'" + String(selected == br ? " selected" : "") + ">" + String(br / 1000) + " kbit/s</option>";
    html += "</select></label></fieldset><br>";
  }
  html += "<p>" + TR("Без BRS весь FD-кадр идёт на номинальной скорости. BRS определяется по принятому кадру автоматически.",
                      "Without BRS, the entire FD frame uses the nominal bitrate. BRS is detected from each received frame.") + "</p>"
    "<button type='submit'>" + TR("Сохранить и перезагрузить", "Save and reboot") + "</button></form>"
    "<script>function fdMode(i){let p='b'+i;let fd=document.getElementById(p+'_mode').value==='1';"
    "document.getElementById(p+'_data_row').hidden=!fd;document.getElementById(p+'_data').disabled=!fd;}fdMode(0);fdMode(1);</script></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

static uint32_t parseRate(const String& text, bool dataPhase) {
  const uint32_t* rates = dataPhase ? DATA_RATES : NOMINAL_RATES;
  size_t count = dataPhase ? sizeof(DATA_RATES) / sizeof(DATA_RATES[0])
                          : sizeof(NOMINAL_RATES) / sizeof(NOMINAL_RATES[0]);
  for (size_t i = 0; i < count; i++) if (text == String(rates[i])) return rates[i];
  return 0;
}

void handleConfigSave() {
  if (sdTest.running) { server.send(409, "text/plain", "SD test running; wait until completion"); return; }
  BusConfig next[2]{};
  for (int i = 0; i < 2; i++) {
    String p = String("b") + String(i);
    next[i].enabled = server.hasArg(p + "_en");
    String name = server.arg(p + "_name");
    if (!name.length()) name = String("CAN") + String(i);
    if (name.length() > 7) {
      server.send(400, "text/plain; charset=utf-8", TR("Имя: максимум 7 байт", "Name: maximum 7 bytes")); return;
    }
    memcpy(next[i].name, name.c_str(), name.length());
    String mode = server.arg(p + "_mode");
    if (mode != "0" && mode != "1") {
      server.send(400, "text/plain; charset=utf-8", TR("Неверный режим CAN", "Invalid CAN mode")); return;
    }
    next[i].fd_mode = mode == "1";
    next[i].bitrate = parseRate(server.arg(p + "_bitrate"), false);
    next[i].data_bitrate = next[i].fd_mode ? parseRate(server.arg(p + "_data"), true) : 0;
    if (!validBusConfig(next[i])) {
      server.send(400, "text/plain; charset=utf-8", TR("Неверная скорость: данные FD должны быть не медленнее номинальной скорости", "Invalid bitrate: FD data bitrate must be at least the nominal bitrate")); return;
    }
  }
  if (!saveConfig(next)) {
    server.send(500, "text/plain; charset=utf-8", TR("Не удалось сохранить настройки", "Failed to save settings")); return;
  }
  memcpy(busCfg, next, sizeof(busCfg));
  server.send(200, "text/plain; charset=utf-8", TR("Сохранено, перезагрузка...", "Saved, rebooting..."));
  delay(500); ESP.restart();
}

// ---------- Установка часов: вручную или часы телефона/браузера ----------
// Calendar fields represent local wall time. No time-zone conversion.
static bool clockValueValid(uint64_t value) {
  return value >= 1577836800ULL && value < 4102444800ULL; // local years 2020..2099
}

static bool parseClockLocal(const String& text, time_t* out) {
  size_t n = text.length();
  if (n != 16 && n != 19) return false;
  for (size_t i = 0; i < n; i++) {
    if (i == 4 || i == 7) { if (text[i] != '-') return false; }
    else if (i == 10) { if (text[i] != 'T') return false; }
    else if (i == 13 || i == 16) { if (text[i] != ':') return false; }
    else if (text[i] < '0' || text[i] > '9') return false;
  }
  int y, mon, day, h, m, sec = 0;
  int got = sscanf(text.c_str(), "%d-%d-%dT%d:%d:%d", &y, &mon, &day, &h, &m, &sec);
  if (got != (n == 19 ? 6 : 5) || y < 2020 || y > 2099 || mon < 1 || mon > 12 ||
      day < 1 || day > 31 || h > 23 || m > 59 || sec > 59) return false;
  struct tm fields{};
  fields.tm_year = y - 1900; fields.tm_mon = mon - 1; fields.tm_mday = day;
  fields.tm_hour = h; fields.tm_min = m; fields.tm_sec = sec;
  // UTC helpers pack/unpack fields without applying any offset or DST.
  time_t value = timegm_utc(&fields);
  struct tm check{}; gmtime_r(&value, &check);
  if (check.tm_year != y - 1900 || check.tm_mon != mon - 1 || check.tm_mday != day ||
      check.tm_hour != h || check.tm_min != m || check.tm_sec != sec) return false;
  *out = value; return true;
}

void handleClockPage() {
  time_t sys = time(nullptr);
  time_t external = rtc.present ? rtc.now() : 0;
  bool rtcValid = rtc.present && !rtc.lostPower() && clockValueValid((uint64_t)external);
  String html = htmlHead(TR("Дата и время", "Date and time")) +
    "<h2>" + TR("Дата и время", "Date and time") + "</h2><p><a href='/'>" + TR("На главную", "Home") + "</a></p>"
    "<ul><li>" + TR("Системные часы (местное время)", "System clock (local time)") + ": " +
    (clockValueValid((uint64_t)sys) ? fmtDateTime(sys) : TR("не установлены", "not set")) + "</li>"
    "<li>PCF8563: " + (!rtc.present ? TR("не найден", "not found") :
      (rtcValid ? fmtDateTime(external) : TR("время недостоверно — требуется установка", "time invalid — please set"))) + "</li></ul>"
    "<p>" + TR("Введите местное время вручную или возьмите его с телефона. В RTC, системных часах и логах используется местное время без пересчёта часовых поясов.",
                 "Enter local time manually or use your phone clock. RTC, system clock and logs use local wall time without time-zone conversion.") + "</p>"
    "<h3>" + TR("Установка вручную", "Set manually") + "</h3>"
    "<form method='POST' action='/clock/set'>"
    "<input type='datetime-local' id='clockdt' name='dt' step='1' required min='2020-01-01T00:00:00' max='2099-12-31T23:59:59'>"
    "<br><br><button type='submit'>" + TR("Установить", "Set time") + "</button></form>"
    "<h3>" + TR("Синхронизация с телефоном", "Synchronise with phone") + "</h3>"
    "<form method='POST' action='/clock/set' onsubmit='this.elements.dt.value=localValue(new Date())'>"
    "<input type='hidden' name='dt'><button type='submit'>" + TR("Взять время телефона", "Use phone time") + "</button></form>"
    "<noscript>" + TR("Для синхронизации с телефоном включите JavaScript в браузере. Ручная установка доступна без него.", "Phone sync requires JavaScript. Manual setting also works without it.") + "</noscript>"
    "<p>" + TR("Интернет не нужен. После установки время применяется сразу, без перезагрузки. Если RTC отсутствует или недоступен, обновятся только системные часы — до потери питания/перезапуска.",
                 "No internet is needed. Time changes immediately without rebooting. If the RTC is missing or unavailable, only the system clock is updated, until power loss/restart.") + "</p>"
    "<script>const pad=n=>String(n).padStart(2,'0');"
    "function localValue(d){return d.getFullYear()+'-'+pad(d.getMonth()+1)+'-'+pad(d.getDate())+'T'+pad(d.getHours())+':'+pad(d.getMinutes())+':'+pad(d.getSeconds());}"
    "document.getElementById('clockdt').value='" +
      (clockValueValid((uint64_t)sys) ? fmtDateTime(sys) : String()) + "'.replace(' ','T');"
    "if(!document.getElementById('clockdt').value)document.getElementById('clockdt').value=localValue(new Date());"
    "</script></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleClockSet() {
  if (sdTest.running) { server.send(409, "text/plain; charset=utf-8", TR("Дождитесь завершения теста SD", "Wait for the SD test to finish")); return; }
  time_t epoch;
  if (!parseClockLocal(server.arg("dt"), &epoch)) {
    server.send(400, "text/plain; charset=utf-8", TR("Некорректные дата/время: допустимы 2020–2099 годы", "Invalid date/time: supported years are 2020–2099")); return;
  }
  // The clock must not jump inside an open log, even on the first HTTP request.
  hadClient = true; paused = true; closeLogFile(3); drainToDummy();
  struct timeval tv = { epoch, 0 };
  if (settimeofday(&tv, nullptr) != 0) {
    server.send(500, "text/plain; charset=utf-8", TR("Не удалось установить системное время", "Cannot set the system clock")); return;
  }
  bool rtcSaved = rtc.present && rtc.adjust(epoch);
  if (rtcSaved) {
    time_t check = rtc.now();
    rtcSaved = !rtc.lostPower() && check >= epoch && check <= epoch + 2;
  }
  for (int i = 0; i < 2; i++) activityMark[i] = busCtx[i].rxCount;
  String message = rtcSaved ? TR("Время установлено: PCF8563 и системные часы обновлены.", "Time set: PCF8563 and system clock updated.") :
    TR("Системное время установлено, но в RTC сохранить не удалось (модуль отсутствует или ошибка I2C).", "System time set, but RTC persistence failed (missing module or I2C error).");
  String html = htmlHead(TR("Время установлено", "Time set")) + "<h2>" + message + "</h2><p>" + TR("Местное время: ", "Local time: ") + fmtDateTime(epoch) +
    "</p><p><a href='/clock'>" + TR("Вернуться к настройке часов", "Back to clock settings") + "</a></p></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

// ---------- Тест качества SD: алгоритм c5_test_sd_spi, только 20 МГц ----------
static uint32_t sdPatternWord(uint32_t i) { return i * 2654435761u ^ (i >> 7); }

static void sdTestFinish(const String& error) {
  sdTest.file.close();
  if (sdTest.path.length() && SD.exists(sdTest.path.c_str()) && !SD.remove(sdTest.path.c_str())) {
    sdTest.error = TR("Не удалось удалить временный файл: ", "Cannot remove temporary file: ") + sdTest.path;
  }
  if (error.length()) sdTest.error = error;
  sdTest.stage = sdTest.error.length() ? 5 : 4;
  sdTest.running = false;
  drainToDummy();
  for (int i = 0; i < 2; i++) activityMark[i] = busCtx[i].rxCount;
  // Keep the normal portal pause. loop() resumes recording after disconnect.
}

static void sdTestFlush() {
  uint32_t t = micros();
  sdTest.file.flush();
  uint32_t dt = micros() - t;
  if (dt > sdTest.maxFlushUs) sdTest.maxFlushUs = dt;
}

static void sdTestStep() {
  if (!sdTest.running) return;
  if (sdTest.stage == 1) {
    for (size_t k = 0; k < 1024; k++) sdTestBuf[k] = sdPatternWord(sdTest.index++);
    uint32_t t = micros();
    size_t n = sdTest.file.write((uint8_t*)sdTestBuf, sizeof(sdTestBuf));
    uint32_t dt = micros() - t;
    if (dt > sdTest.maxWriteUs) sdTest.maxWriteUs = dt;
    if (dt > 50000) sdTest.slowWrites++;
    sdTest.written += n;
    if (n != sizeof(sdTestBuf) || sdTest.file.getWriteError()) {
      sdTest.failures++; sdTestFinish(TR("Ошибка записи 4 КБ", "4 KB write failed")); return;
    }
    if (sdTest.written >= SD_TEST_BYTES) {
      sdTestFlush();
      bool failed = sdTest.file.getWriteError();
      sdTest.file.close();
      sdTest.writeMs = millis() - sdTest.stageStart;
      if (failed) { sdTest.failures++; sdTestFinish(TR("Ошибка flush", "Flush failed")); return; }
      sdTest.file = SD.open(sdTest.path.c_str(), FILE_READ);
      if (!sdTest.file) { sdTestFinish(TR("Ошибка открытия для чтения", "Cannot open for reading")); return; }
      sdTest.stage = 2; sdTest.index = 0; sdTest.stageStart = millis();
    }
  } else if (sdTest.stage == 2) {
    int n = sdTest.file.read((uint8_t*)sdTestBuf, sizeof(sdTestBuf));
    if (n != (int)sizeof(sdTestBuf)) { sdTest.failures++; sdTestFinish(TR("Короткое чтение", "Short read")); return; }
    sdTest.read += n;
    for (size_t k = 0; k < 1024; k++)
      if (sdTestBuf[k] != sdPatternWord(sdTest.index++)) sdTest.badWords++;
    if (sdTest.read >= SD_TEST_BYTES) {
      sdTest.readMs = millis() - sdTest.stageStart;
      sdTest.file.close();
      if (!SD.remove(sdTest.path.c_str())) { sdTestFinish(TR("Не удалось удалить тестовый файл", "Cannot delete test file")); return; }
      sdTest.file = SD.open(sdTest.path.c_str(), FILE_WRITE);
      if (!sdTest.file) { sdTestFinish(TR("Ошибка открытия имитации лога", "Cannot open log simulation")); return; }
      sdTest.stage = 3; sdTest.stageStart = millis();
      sdTest.lastFlush = sdTest.nextWrite = sdTest.stageStart;
    }
  } else if (sdTest.stage == 3) {
    uint32_t now = millis();
    if ((uint32_t)(now - sdTest.stageStart) >= 20000UL) {
      sdTestFlush(); sdTest.simMs = millis() - sdTest.stageStart;
      if (sdTest.file.getWriteError()) sdTest.failures++;
      sdTestFinish(String()); return;
    }
    if ((int32_t)(now - sdTest.nextWrite) < 0) return;
    for (size_t k = 0; k < 256; k++) sdTestBuf[k] = sdPatternWord(sdTest.index++);
    uint32_t t = micros();
    size_t n = sdTest.file.write((uint8_t*)sdTestBuf, 1024);
    uint32_t dt = micros() - t;
    if (dt > sdTest.maxSimWriteUs) sdTest.maxSimWriteUs = dt;
    sdTest.simulated += n;
    if (n != 1024 || sdTest.file.getWriteError()) {
      sdTest.failures++; sdTestFinish(TR("Ошибка имитации лога", "Log simulation write failed")); return;
    }
    if ((uint32_t)(millis() - sdTest.lastFlush) >= 1000) {
      sdTestFlush(); sdTest.lastFlush = millis();
      if (sdTest.file.getWriteError()) { sdTest.failures++; sdTestFinish(TR("Ошибка flush", "Flush failed")); return; }
    }
    sdTest.nextWrite = millis() + 5;
  }
}

void handleSdTestStart() {
  if (sdTest.running) { server.send(409, "text/plain", "Test already running"); return; }
  if (!sdReady || safeMode) { server.send(503, "text/plain", "SD unavailable"); return; }
  // Close the log before any benchmark writes, even on the first HTTP request.
  hadClient = true;
  paused = true; closeLogFile(3); drainToDummy();
  if (SD.totalBytes() < SD.usedBytes() || SD.totalBytes() - SD.usedBytes() < 5UL * 1024UL * 1024UL) {
    server.send(507, "text/plain; charset=utf-8", TR("Нужно минимум 5 МиБ свободного места", "At least 5 MiB free space required")); return;
  }
  sdTest = SdTestState();
  // Never overwrite an existing user file (including leftovers after a reset).
  for (unsigned i = 0; i < 1000; i++) {
    String path = String("/c5_sdtest_") + String(i) + ".tmp";
    if (!SD.exists(path.c_str())) { sdTest.path = path; break; }
  }
  if (!sdTest.path.length()) { server.send(409, "text/plain", "No free temporary filename"); return; }
  sdTest.file = SD.open(sdTest.path.c_str(), FILE_WRITE);
  if (!sdTest.file) { sdTest.stage = 5; sdTest.error = TR("Не удалось создать файл", "Cannot create test file"); }
  else { sdTest.running = true; sdTest.stage = 1; sdTest.started = sdTest.stageStart = millis(); }
  server.sendHeader("Location", "/sd-test"); server.send(303);
}

void handleSdTestPage() {
  String html = htmlHead(TR("Тест SD-карты", "SD card test"));
  html += "<h2>" + TR("Тест SD-карты", "SD card test") + "</h2><p><a href='/'>" + TR("На главную", "Home") + "</a></p>"
    "<p>SPI: <b>20 MHz</b>. " + TR("Запись/проверка 4 МиБ блоками 4 КБ, затем 20 секунд имитации лога: 1 КБ, пауза 5 мс, flush раз в секунду.",
                                    "Write/verify 4 MiB in 4 KB blocks, then 20 seconds of log simulation: 1 KB, 5 ms pause, flush every second.") + "</p>"
    "<p>" + TR("Запись CAN приостановлена на время теста и подключения к порталу. Проверка использует отдельный временный файл; существующие логи не изменяются.",
                 "CAN logging is paused during the test and portal connection. A separate temporary file is used; existing logs are unchanged.") + "</p>";
  if (sdReady) html += "<p>" + TR("Размер карты", "Card size") + ": " + String((uint32_t)(SD.cardSize() / (1024ULL * 1024ULL))) + " MiB; " +
    TR("занято", "used") + ": " + String((uint32_t)(SD.usedBytes() / (1024ULL * 1024ULL))) + " MiB</p>";
  String status;
  if (sdTest.stage == 0) status = TR("Не запускался", "Not started");
  else if (sdTest.stage == 1) status = TR("Запись 4 МиБ", "Writing 4 MiB");
  else if (sdTest.stage == 2) status = TR("Чтение и проверка", "Reading and verifying");
  else if (sdTest.stage == 3) status = TR("Имитация лога", "Log simulation");
  else if (sdTest.stage == 5) status = TR("Тест прерван: ", "Test stopped: ") + htmlEscape(sdTest.error);
  else status = (sdTest.failures || sdTest.badWords) ? TR("Обнаружены ошибки", "Errors detected") : TR("Тест завершён без ошибок", "Test completed without errors");
  html += "<h3>" + status + "</h3><ul>";
  html += "<li>" + TR("Записано / прочитано", "Written / read") + ": " + String(sdTest.written / 1024) + " / " + String(sdTest.read / 1024) + " KiB</li>";
  if (sdTest.writeMs) html += "<li>" + TR("Скорость записи", "Write speed") + ": " + String((uint32_t)((uint64_t)sdTest.written * 1000 / 1024 / sdTest.writeMs)) + " KiB/s</li>";
  if (sdTest.readMs) html += "<li>" + TR("Скорость чтения", "Read speed") + ": " + String((uint32_t)((uint64_t)sdTest.read * 1000 / 1024 / sdTest.readMs)) + " KiB/s</li>";
  html += "<li>" + TR("Макс. запись 4 КБ", "Max 4 KB write") + ": " + String(sdTest.maxWriteUs) + " us</li>"
    "<li>" + TR("Записей дольше 50 мс", "Writes longer than 50 ms") + ": " + String(sdTest.slowWrites) + "</li>"
    "<li>" + TR("Ошибок данных (32-битных слов)", "Data errors (32-bit words)") + ": " + String(sdTest.badWords) + "</li>"
    "<li>" + TR("Сбоев ввода/вывода", "I/O failures") + ": " + String(sdTest.failures) + "</li>"
    "<li>" + TR("Имитация лога", "Log simulation") + ": " + String(sdTest.simulated / 1024) + " KiB</li>"
    "<li>" + TR("Макс. запись 1 КБ", "Max 1 KB write") + ": " + String(sdTest.maxSimWriteUs) + " us</li>"
    "<li>" + TR("Макс. flush (все этапы)", "Max flush (all stages)") + ": " + String(sdTest.maxFlushUs) + " us</li></ul>";
  if (sdTest.running) html += "<p>" + TR("Можно оставить страницу открытой — результаты обновляются автоматически.", "Keep this page open; results refresh automatically.") + "</p><script>setTimeout(()=>location.reload(),2000);</script>";
  else if (sdReady && !safeMode) html += "<form method='POST' action='/sd-test/start'><button>" + TR("Запустить тест", "Run test") + "</button></form>";
  html += "<p>" + TR("Это проверка скорости, задержек и целостности тестового файла, а не всей ёмкости карты. HTTP-запросы могут снижать измеренную скорость.",
                     "This checks speed, latency and test-file integrity, not the entire card capacity. HTTP requests can reduce measured throughput.") + "</p></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

// ---------- Скачивание (TAR) ----------
void tarWriteOctalField(uint8_t* header, int offset, int fieldLen, unsigned long value) {
  char buf[16];
  snprintf(buf, sizeof(buf), "%0*lo", fieldLen - 1, value);
  memcpy(header + offset, buf, fieldLen - 1);
  header[offset + fieldLen - 1] = '\0';
}

void tarBuildHeader(uint8_t* header, const String& name, uint32_t size) {
  memset(header, 0, 512);
  strncpy((char*)header, name.c_str(), 100);
  tarWriteOctalField(header, 100, 8, 0644);
  tarWriteOctalField(header, 108, 8, 0);
  tarWriteOctalField(header, 116, 8, 0);
  tarWriteOctalField(header, 124, 12, size);
  tarWriteOctalField(header, 136, 12, (unsigned long)time(nullptr));
  memset(header + 148, ' ', 8);
  header[156] = '0';
  memcpy(header + 257, "ustar", 5);
  header[262] = '\0'; header[263] = '0'; header[264] = '0';
  unsigned int sum = 0;
  for (int i = 0; i < 512; i++) sum += header[i];
  char chkbuf[7];
  snprintf(chkbuf, sizeof(chkbuf), "%06o", sum);
  memcpy(header + 148, chkbuf, 6);
  header[154] = '\0'; header[155] = ' ';
}

bool dlSendAll(WiFiClient& client, const uint8_t* data, size_t len) {
  uint32_t lastProgress = millis();
  while (len > 0) {
    if (!client.connected()) return false;
    size_t n = client.write(data, len);
    if (n > 0) { data += n; len -= n; lastProgress = millis(); }
    else {
      if (millis() - lastProgress > 15000) return false;
      delay(2);
    }
  }
  return true;
}

void handleDownloadTar() {
  if (sdTest.running) { server.send(409, "text/plain", "SD test running; wait until completion"); return; }
  String date = server.arg("date");
  if (!isValidDateFolder(date) || !SD.exists("/" + date)) {
    server.send(404, "text/plain", "Folder not found"); return;
  }
  uint64_t totalSize = 1024;
  File dir = SD.open("/" + date);
  if (!dir) { server.send(500, "text/plain", "SD error"); return; }
  File f = dir.openNextFile();
  while (f) {
    if (!f.isDirectory()) {
      size_t sz = f.size();
      totalSize += 512 + ((sz + 511) / 512) * 512;
    }
    f = dir.openNextFile();
  }
  dir.close();

  String tarName = date + ".tar";
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + tarName + "\"");
  server.setContentLength((size_t)totalSize);
  server.send(200, "application/octet-stream", "");

  WiFiClient client = server.client();
  bool ok = true;
  uint8_t header[512];
  uint8_t buf[1024];

  dir = SD.open("/" + date);
  f = dir.openNextFile();
  while (f && ok) {
    if (!f.isDirectory()) {
      String fname = String(f.name());
      if (fname.lastIndexOf('/') != -1) fname = fname.substring(fname.lastIndexOf('/') + 1);
      String arcName = date + "/" + fname;
      uint32_t sz = f.size();
      tarBuildHeader(header, arcName, sz);
      if (!dlSendAll(client, header, 512)) { ok = false; break; }
      uint32_t rem = sz;
      while (rem > 0 && ok) {
        size_t toRead = rem < sizeof(buf) ? rem : sizeof(buf);
        f.read(buf, toRead);
        if (!dlSendAll(client, buf, toRead)) ok = false;
        rem -= toRead;
      }
      if (rem == 0 && (sz % 512) != 0) {
        uint32_t pad = 512 - (sz % 512);
        memset(buf, 0, pad);
        if (!dlSendAll(client, buf, pad)) ok = false;
      }
    }
    f = dir.openNextFile();
  }
  dir.close();
  if (ok) {
    memset(buf, 0, 1024);
    dlSendAll(client, buf, 1024);
  }
  delay(20);
  client.stop();
}

void handleDownload() {
  if (sdTest.running) { server.send(409, "text/plain", "SD test running; wait until completion"); return; }
  String path = server.arg("file");
  if (!path.startsWith("/") || path.indexOf("..") != -1) {
    server.send(400, "text/plain", "Invalid path"); return;
  }
  File f = SD.open(path, FILE_READ);
  if (!f || f.isDirectory()) {
    server.send(404, "text/plain", "File not found"); return;
  }
  String fname = path.substring(path.lastIndexOf('/') + 1);
  server.sendHeader("Content-Disposition", "attachment; filename=\"" + fname + "\"");
  server.setContentLength(f.size());
  server.send(200, "application/octet-stream", "");
  WiFiClient client = server.client();
  uint8_t buf[4096];
  while (f.available() && client.connected()) {
    size_t n = f.read(buf, sizeof(buf));
    if (n > 0) client.write(buf, n);
  }
  f.close(); delay(20); client.stop();
}

void handleLogsPage() {
  String date = server.arg("date");
  String html = htmlHead(TR("Логи", "Logs"));

  if (date.length() > 0) {
    if (!isValidDateFolder(date) || !SD.exists("/" + date)) {
      server.send(404, "text/plain", "Folder not found"); return;
    }
    html += "<h2>" + TR("Логи за ", "Logs for ") + date + "</h2>"
            "<p><a href='/logs'>&larr; " + TR("Назад к списку дат", "Back to dates") + "</a> | "
            "<a href='/download_tar?date=" + date + "'>" + TR("Скачать весь день (TAR)", "Download whole day (TAR)") + "</a></p><ul>";
    File dir = SD.open("/" + date);
    if (dir) {
      File f = dir.openNextFile();
      while (f) {
        if (!f.isDirectory()) {
          String fname = String(f.name());
          if (fname.lastIndexOf('/') != -1) fname = fname.substring(fname.lastIndexOf('/') + 1);
          size_t sz = f.size();
          html += "<li>" + fname + " (" + String(sz / 1024) + " KB) — <a href='/download?file=/" + date + "/" + fname + "'>" + TR("Скачать", "Download") + "</a></li>";
        }
        f = dir.openNextFile();
      }
      dir.close();
    }
    html += "</ul>";
  } else {
    html += "<h2>" + TR("Логи по датам", "Logs by date") + "</h2>"
            "<p><a href='/'>&larr; " + TR("На главную", "Home") + "</a></p>";
    File root = SD.open("/");
    bool found = false;
    if (root) {
      File d = root.openNextFile();
      while (d) {
        if (d.isDirectory()) {
          String dname = String(d.name());
          if (dname.lastIndexOf('/') != -1) dname = dname.substring(dname.lastIndexOf('/') + 1);
          if (isValidDateFolder(dname)) {
            found = true;
            html += "<div class='fold'><b>" + dname + "</b> — "
                    "<a href='/logs?date=" + dname + "'>" + TR("Открыть", "Open") + "</a> | "
                    "<a href='/download_tar?date=" + dname + "'>" + TR("Скачать TAR", "Download TAR") + "</a> | "
                    "<form style='display:inline' action='/api/delete-day' method='POST' onsubmit=\"return confirm('" + TR("Удалить папку?", "Delete folder?") + "')\">"
                    "<input type='hidden' name='date' value='" + dname + "'>"
                    "<button type='submit' style='border:none;background:none;color:#c62828;cursor:pointer'>" + TR("Удалить", "Delete") + "</button></form></div>";
          }
        }
        d = root.openNextFile();
      }
      root.close();
    }
    if (!found) html += "<p>" + TR("Логов пока нет", "No logs yet") + "</p>";
  }
  html += "</body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleErrorsPage() {
  String html = htmlHead(TR("Ошибки", "Errors")) + "<h2>" + TR("Журнал ошибок", "Error log") + "</h2>"
    "<p><a href='/'>&larr; " + TR("на главную", "home") + "</a></p>";
  if (SD.exists(ERRLOG_PATH)) {
    File f = SD.open(ERRLOG_PATH);
    if (f) {
      html += "<pre style='white-space:pre-wrap;font-size:12px;background:#161616;padding:10px;'>";
      while (f.available()) html += (char)f.read();
      f.close(); html += "</pre>";
    }
    html += "<form action='/api/delete-errors' method='POST'><button type='submit' style='border-color:#c62828;color:#ff6b6b'>" + TR("Очистить", "Clear") + "</button></form>";
  } else {
    html += "<p>" + TR("Ошибок нет", "No errors") + "</p>";
  }
  server.send(200, "text/html; charset=utf-8", html);
}

void handleDeleteErrors() {
  if (sdTest.running) { server.send(409, "text/plain", "SD test running; wait until completion"); return; }
  SD.remove(ERRLOG_PATH); SD.remove(ERRLOG_OLD_PATH);
  server.sendHeader("Location", "/errors"); server.send(303);
}

static void initSD() {
  SPI.begin(PIN_SD_SCK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  if (!SD.begin(PIN_SD_CS, SPI, SD_SPI_HZ)) { sdReady = false; errLog("SD: init failed"); return; }
  if (SD.cardType() == CARD_NONE) { sdReady = false; errLog("SD: no card"); return; }
  sdReady = true;
}

static void enterSafeMode() {
  if (safeMode) return;
  if (sdTest.running) sdTestFinish(TR("Остановлен кнопкой BOOT", "Stopped by BOOT"));
  safeMode = true;
  Serial.println("=== SAFE MODE ===");
  for (int i = 0; i < 2; i++) if (busCtx[i].node) twai_node_disable(busCtx[i].node);
  if (logFile) closeLogFile(3);
  flushToSd(); SD.end(); sdReady = false;
  if (apActive) stopAp();
  Serial.println("Card is safe to remove. Press RESET to continue.");
}

static void pollBootButton() {
  if (safeMode) return;
  int s = digitalRead(PIN_BOOT);
  if (!bootArmed) { if (s == HIGH) bootArmed = true; return; }
  if (s == LOW) { if (bootDownMs == 0) bootDownMs = millis(); if (millis() - bootDownMs >= BOOT_HOLD_MS) enterSafeMode(); }
  else bootDownMs = 0;
}

// Native USB host presence (SOF), independent of an open serial monitor.
// USB chargers / power-only cables cannot be detected without VBUS sensing.
static bool usbHostConnected() {
  return HWCDC::isPlugged();
}

static bool sleepBlocked() {
  return digitalRead(PIN_ACC) == ACC_ON_LEVEL || apActive || sdTest.running || usbHostConnected();
}

static void resumeAfterSleepCancelled() {
  if (safeMode) return;
  for (int i = 0; i < 2; i++) {
    if (busCtx[i].node && twai_node_enable(busCtx[i].node) != ESP_OK)
      errLog("CAN%d: resume after cancelled sleep failed", i);
    activityMark[i] = busCtx[i].rxCount;
  }
}

static void goToSleepUntilAccOn() {
  if (sleepBlocked()) return;
  // C5 uses EXT1, not the S3 example's EXT0 API. The external pull-up
  // keeps GPIO0 defined while the digital domain is powered down.
  esp_err_t e = esp_sleep_enable_ext1_wakeup_io(
      1ULL << PIN_ACC, ESP_EXT1_WAKEUP_ANY_LOW);
  if (e != ESP_OK) {
    errLog("ACC wake configuration failed: %s", esp_err_to_name(e));
    return;  // never sleep without a working ACC wake source
  }
  int stopped = 0;
  for (int i = 0; i < 2; i++) {
    if (safeMode || !busCtx[i].node) continue;
    e = twai_node_disable(busCtx[i].node);
    if (e != ESP_OK) {
      errLog("CAN%d: stop before sleep failed: %s", i, esp_err_to_name(e));
      for (int j = 0; j < i; j++)
        if (stopped & (1 << j)) twai_node_enable(busCtx[j].node);
      return;
    }
    stopped |= 1 << i;
  }
  // Producers are stopped: drain a finite queue, finish the compressor.
  if (logFile && !paused && sdReady) dispatch();
  else drainToDummy();
  closeLogFile(3);  // existing shutdown reason, compatible with unpacker
  // SD finalisation can take time: check again before committing to sleep.
  if (sleepBlocked()) { resumeAfterSleepCancelled(); return; }
  server.stop();
  WiFi.mode(WIFI_OFF);
  if (sdReady) { SD.end(); sdReady = false; }
  Serial.println("ACC OFF: deep sleep; wake on GPIO0 LOW (ACC ON)");
  delay(20);
  // A host/ACC arriving during teardown requires full peripheral init.
  if (sleepBlocked()) { ESP.restart(); return; }
  esp_deep_sleep_start();
}

static void sleepLoop() {
  static bool timing = false;
  static uint32_t since = 0;
  if (sleepBlocked()) { timing = false; return; }
  if (!timing) { timing = true; since = millis(); }
  if ((uint32_t)(millis() - since) >= SLEEP_GRACE_MS) {
    goToSleepUntilAccOn();
    timing = false;  // back off if sleep was cancelled or failed
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis(); while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println("\n=== dual_can_fd_sniffer 0.2.5 ===");
  pinMode(PIN_BOOT, INPUT_PULLUP);
  pinMode(PIN_ACC, INPUT_PULLUP);
  
  loadUiLanguage();
  loadConfig();

  Wire.begin(PIN_RTC_SDA, PIN_RTC_SCL, 100000);
  if (rtc.begin(&Wire) && !rtc.lostPower()) {
    time_t t = rtc.now(); struct timeval tv = { t, 0 }; settimeofday(&tv, nullptr);
    Serial.printf("RTC: %s\n", fmtDateTime(t).c_str());
  } else if (!rtc.present) { Serial.println("RTC: not found"); }

  initSD();
  if (!errMutex) errMutex = xSemaphoreCreateMutex();

// --- WiFi 5 ГГц (Wi-Fi 6 / 802.11ax) ---
WiFi.mode(WIFI_AP);
esp_err_t cc = esp_wifi_set_country_code("RU", true);
Serial.printf("Country code RU: %s\n", esp_err_to_name(cc));

WiFi.setBandMode(WIFI_BAND_MODE_5G_ONLY);

uint8_t proto = WIFI_PROTOCOL_11A | WIFI_PROTOCOL_11N
              | WIFI_PROTOCOL_11AC | WIFI_PROTOCOL_11AX;
esp_err_t pe = esp_wifi_set_protocol(WIFI_IF_AP, proto);
Serial.printf("802.11ax: %s\n", esp_err_to_name(pe));

esp_wifi_set_max_tx_power(40);   // 10 dBm

bool apOk = WiFi.softAP(AP_SSID, AP_PASS, 36);
if (!apOk) {
  Serial.println("AP 5G: FAIL, fallback 2.4G");
  WiFi.setBandMode(WIFI_BAND_MODE_2G_ONLY);
  apOk = WiFi.softAP(AP_SSID, AP_PASS, 6);
}
Serial.printf("AP: %s, IP %s, 10 dBm\n",
              apOk ? "OK" : "FAIL", WiFi.softAPIP().toString().c_str());
apStartMs = millis();
apActive = apOk;


  server.on("/", handleRoot);
  server.on("/set-lang", HTTP_POST, handleSetLang);
  server.on("/logs", handleLogsPage);
  server.on("/config", handleConfigPage);
  server.on("/clock", handleClockPage);
  server.on("/clock/set", HTTP_POST, handleClockSet);
  server.on("/sd-test", handleSdTestPage);
  server.on("/sd-test/start", HTTP_POST, handleSdTestStart);
  server.on("/api/config", HTTP_POST, handleConfigSave);
  server.on("/api/delete-day", HTTP_POST, handleDeleteDay);
  server.on("/download", handleDownload);
  server.on("/download_tar", handleDownloadTar);
  server.on("/errors", handleErrorsPage);
  server.on("/api/delete-errors", HTTP_POST, handleDeleteErrors);
  server.begin();

  if (busCfg[0].enabled) startBus(&busCtx[0], 0, busCfg[0]);
  if (busCfg[1].enabled) startBus(&busCtx[1], 1, busCfg[1]);
  drainToDummy();
}

void loop() {
  pollBootButton();
  if (safeMode) { sleepLoop(); delay(100); return; }
  server.handleClient();

  if (apActive) {
    int n = WiFi.softAPgetStationNum();
    if (n > 0) {
      if (!paused) {
        for (int i = 0; i < 2; i++) activityMark[i] = busCtx[i].rxCount;
        closeLogFile(3);
        drainToDummy();
        paused = true;
      }
      hadClient = true;
    } else {
      if (sdTest.running) { /* keep portal available until the test completes */ }
      else if (hadClient) { stopAp(); paused = false; }
      else if (millis() - apStartMs > AP_TIMEOUT_MS) stopAp();
    }
  }

  // Run before all logging early returns, including absent SD / silent CAN.
  sdTestStep();
  sleepLoop();
  if (!sdReady || paused) { drainToDummy(); delay(sdTest.running ? 1 : 20); return; }
  if (!logFile) {
    if (!anyBusActive()) { drainToDummy(); delay(30); return; }
    if (!openLogFile()) { drainToDummy(); delay(1000); return; }
    for (int i = 0; i < 2; i++) activityMark[i] = busCtx[i].rxCount;
  }

  dispatch();

  // fileBytes is the compressed file size, refreshed by flushToSd().
  if (logFile && fileBytes >= LOG_FILE_MAX) {
    for (int i = 0; i < 2; i++) activityMark[i] = busCtx[i].rxCount;
    closeLogFile(1);
  }
  if (logFile && lastCanActivityMs && millis() - lastCanActivityMs > LOG_IDLE_CLOSE_MS) {
    for (int i = 0; i < 2; i++) activityMark[i] = busCtx[i].rxCount;
    closeLogFile(2);
  }
  if (logFile && millis() - lastAnchorMs > LOG_ANCHOR_MS) { writeAnchor(); lastAnchorMs = millis(); }
  if (logFile && wlen > 0 && millis() - lastFlushMs > LOG_FLUSH_MS) flushToSd();

  delay(1);
}

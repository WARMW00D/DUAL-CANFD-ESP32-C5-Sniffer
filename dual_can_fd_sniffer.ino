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
static uint32_t bothActiveMark[2] = {0, 0};   // для ожидания обеих шин

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
  String line = String(ts) + " [fw 0.1.0, " + String(millis()) + " ms] " + msg + "\r\n";
  Serial.print("[ERR] " + line);
  if (!errMutex) errMutex = xSemaphoreCreateMutex();
  if (xSemaphoreTake(errMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
    if (sdReady) errWriteLine(line);
    xSemaphoreGive(errMutex);
  }
}

// ---------- Конфигурация ----------
void loadConfig() {
  prefs.begin("can", true);
  prefs.getBytes("bus0", &busCfg[0], sizeof(BusConfig));
  prefs.getBytes("bus1", &busCfg[1], sizeof(BusConfig));
  prefs.end();
  if (busCfg[0].bitrate == 0) {
    busCfg[0] = {true, "CAN0", 500000, 0, 0};
    busCfg[1] = {true, "CAN1", 500000, 0, 0};
  }
}
void saveConfig() {
  prefs.begin("can", false);
  prefs.putBytes("bus0", &busCfg[0], sizeof(BusConfig));
  prefs.putBytes("bus1", &busCfg[1], sizeof(BusConfig));
  prefs.end();
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

// ---------- Ожидание обеих шин ----------
static bool bothBusesActive() {
  for (int i = 0; i < 2; i++) {
    if (!busCfg[i].enabled) continue;
    if ((uint32_t)(busCtx[i].rxCount - bothActiveMark[i]) == 0) return false;
  }
  return true;
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
  memcpy(hdr, "C5CAN\0\0\0", 8); hdr[8] = 1; memcpy(hdr + 9, "0.1.0\0\0\0\0\0\0\0", 12); hdr[21] = 2;
  for (int i = 0; i < 2; i++) {
    int off = 22 + i * 14; hdr[off] = i; uint16_t br = busCfg[i].bitrate / 1000;
    hdr[off+1] = br & 0xFF; hdr[off+2] = (br >> 8) & 0xFF;
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
  if (next == b->tail) { b->dropCount++; return false; }
  twai_frame_t f; memset(&f, 0, sizeof(f));
  f.buffer = b->ring[head].data; f.buffer_len = 64;
  if (twai_node_receive_from_isr(h, &f) != ESP_OK) return false;
  uint32_t us = (uint32_t)esp_timer_get_time();
  uint8_t busfl = b->busIndex;
  if (f.header.ide) busfl |= 0x08; if (f.header.rtr) busfl |= 0x10;
  if (f.header.fdf) busfl |= 0x20; if (f.header.brs) busfl |= 0x40;
  b->ring[head].ts_us = us; b->ring[head].id = f.header.id; b->ring[head].bus_fl = busfl;
  b->ring[head].len = f.header.fdf ? twaifd_dlc2len(f.header.dlc) : (f.header.dlc > 8 ? 8 : f.header.dlc);
  b->head = next; b->rxCount++;
  return false;
}

static bool startBus(BusCtx* b, uint8_t idx, uint32_t bitrate, uint32_t fd_bitrate) {
  b->busIndex = idx;
  twai_onchip_node_config_t cfg; memset(&cfg, 0, sizeof(cfg));
  cfg.io_cfg.rx = (gpio_num_t)(idx == 0 ? PIN_CAN0_RX : PIN_CAN1_RX);
  cfg.io_cfg.tx = (gpio_num_t)-1; cfg.io_cfg.quanta_clk_out = (gpio_num_t)-1; cfg.io_cfg.bus_off_indicator = (gpio_num_t)-1;
  cfg.bit_timing.bitrate = bitrate; cfg.bit_timing.sp_permill = 800;
  if (fd_bitrate) { cfg.data_timing.bitrate = fd_bitrate; cfg.data_timing.sp_permill = 750; }
  cfg.fail_retry_cnt = 0; cfg.tx_queue_depth = 1; cfg.flags.enable_listen_only = 1;
  esp_err_t e = twai_new_node_onchip(&cfg, &b->node);
  if (e != ESP_OK) return false;
  twai_event_callbacks_t cb; memset(&cb, 0, sizeof(cb)); cb.on_rx_done = onRx;
  e = twai_node_register_event_callbacks(b->node, &cb, b);
  if (e != ESP_OK) return false;
  e = twai_node_enable(b->node);
  return e == ESP_OK;
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
  if (back != "/logs" && back != "/errors" && back != "/config") back = "/";
  server.sendHeader("Location", back); server.send(303);
}

bool isValidDateFolder(const String& name) {
  if (name.length() != 10) return false;
  int Y, M, D;
  if (sscanf(name.c_str(), "%d-%d-%d", &Y, &M, &D) != 3) return false;
  return (Y >= 2020 && Y <= 2099 && M >= 1 && M <= 12 && D >= 1 && D <= 31);
}

void handleDeleteDay() {
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
  else if (bothBusesActive()) state = TR("ОЖИДАНИЕ", "IDLE");
  else state = TR("ЖДУ ОБЕ ШИНЫ", "WAITING FOR BOTH BUSES");

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
    "</ul><p><a href='/config'>" + TR("Настройки шин", "Bus settings") + "</a></p>"
    "<p><a href='/logs'>" + TR("Логи на карте", "Logs on card") + "</a></p>"
    "<p><a href='/errors'>" + TR("Журнал ошибок", "Error log") + "</a></p></body></html>";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfigPage() {
  String html = htmlHead(TR("Настройки CAN", "CAN Settings")) +
    "<h2>" + TR("Настройки шин", "Bus settings") + "</h2><p><a href='/'>&larr; " + TR("на главную", "home") + "</a></p>"
    "<form method='POST' action='/api/config'>"
    "<h3>CAN0</h3>"
    "<input type='checkbox' name='b0_en' " + String(busCfg[0].enabled ? "checked" : "") + "> " + TR("Включена", "Enabled") + "<br>"
    "<input type='text' name='b0_name' value='" + String(busCfg[0].name) + "' maxlength='7' placeholder='CAN0'><br>"
    "<select name='b0_bitrate'>";
    uint32_t brs[] = {100000, 125000, 250000, 500000, 1000000};
    for (auto br : brs) html += "<option value='" + String(br) + "'" + (busCfg[0].bitrate == br ? " selected" : "") + ">" + String(br/1000) + " kbit/s</option>";
    html += "</select><br><br><h3>CAN1</h3>"
    "<input type='checkbox' name='b1_en' " + String(busCfg[1].enabled ? "checked" : "") + "> " + TR("Включена", "Enabled") + "<br>"
    "<input type='text' name='b1_name' value='" + String(busCfg[1].name) + "' maxlength='7' placeholder='CAN1'><br>"
    "<select name='b1_bitrate'>";
    for (auto br : brs) html += "<option value='" + String(br) + "'" + (busCfg[1].bitrate == br ? " selected" : "") + ">" + String(br/1000) + " kbit/s</option>";
    html += "</select><br><br>"
    "<button type='submit'>" + TR("Сохранить и перезагрузить", "Save and reboot") + "</button></form>";
  server.send(200, "text/html; charset=utf-8", html);
}

void handleConfigSave() {
  busCfg[0].enabled = server.hasArg("b0_en");
  strncpy(busCfg[0].name, server.arg("b0_name").c_str(), 7);
  busCfg[0].bitrate = server.arg("b0_bitrate").toInt();
  busCfg[1].enabled = server.hasArg("b1_en");
  strncpy(busCfg[1].name, server.arg("b1_name").c_str(), 7);
  busCfg[1].bitrate = server.arg("b1_bitrate").toInt();
  saveConfig();
  server.send(200, "text/plain", "OK, rebooting...");
  delay(500); ESP.restart();
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
  if (safeMode) return; safeMode = true;
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
  return digitalRead(PIN_ACC) == ACC_ON_LEVEL || apActive || usbHostConnected();
}

static void resumeAfterSleepCancelled() {
  if (safeMode) return;
  for (int i = 0; i < 2; i++) {
    if (busCtx[i].node && twai_node_enable(busCtx[i].node) != ESP_OK)
      errLog("CAN%d: resume after cancelled sleep failed", i);
    bothActiveMark[i] = busCtx[i].rxCount;
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
  Serial.println("\n=== dual_can_fd_sniffer 0.1.0 ===");
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
  server.on("/api/config", HTTP_POST, handleConfigSave);
  server.on("/api/delete-day", HTTP_POST, handleDeleteDay);
  server.on("/download", handleDownload);
  server.on("/download_tar", handleDownloadTar);
  server.on("/errors", handleErrorsPage);
  server.on("/api/delete-errors", HTTP_POST, handleDeleteErrors);
  server.begin();

  if (busCfg[0].enabled) startBus(&busCtx[0], 0, busCfg[0].bitrate, busCfg[0].data_bitrate);
  if (busCfg[1].enabled) startBus(&busCtx[1], 1, busCfg[1].bitrate, busCfg[1].data_bitrate);
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
        for (int i = 0; i < 2; i++) bothActiveMark[i] = busCtx[i].rxCount;
        closeLogFile(3);
        drainToDummy();
        paused = true;
      }
      hadClient = true;
    } else {
      if (hadClient) { stopAp(); paused = false; }
      else if (millis() - apStartMs > AP_TIMEOUT_MS) stopAp();
    }
  }

  // Run before all logging early returns, including absent SD / silent CAN.
  sleepLoop();
  if (!sdReady || paused) { drainToDummy(); delay(20); return; }
  if (!logFile) {
    if (!bothBusesActive()) { drainToDummy(); delay(30); return; }
    if (!openLogFile()) { drainToDummy(); delay(1000); return; }
    for (int i = 0; i < 2; i++) bothActiveMark[i] = busCtx[i].rxCount;
  }

  dispatch();

  if (hsLog.totalBytes >= LOG_FILE_MAX) {
    for (int i = 0; i < 2; i++) bothActiveMark[i] = busCtx[i].rxCount;
    closeLogFile(1);
  }
  if (logFile && lastCanActivityMs && millis() - lastCanActivityMs > LOG_IDLE_CLOSE_MS) {
    for (int i = 0; i < 2; i++) bothActiveMark[i] = busCtx[i].rxCount;
    closeLogFile(2);
  }
  if (logFile && millis() - lastAnchorMs > LOG_ANCHOR_MS) { writeAnchor(); lastAnchorMs = millis(); }
  if (logFile && wlen > 0 && millis() - lastFlushMs > LOG_FLUSH_MS) flushToSd();

  delay(1);
}

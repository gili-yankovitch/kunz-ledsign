/// @file    main.cpp
/// @brief   Interactive LED-sign firmware, ported from the Arduino sketch
///          (FastLED + WiFi + WebServer + ArduinoJson) to the pure ESP-IDF
///          framework.
///
/// Mapping of the Arduino dependencies onto ESP-IDF:
///   FastLED      -> led_strip RMT driver + local CRGB/scale8 helpers
///   WiFi.h       -> esp_wifi station + event handlers
///   WebServer    -> esp_http_server (httpd)
///   ArduinoJson  -> cJSON (bundled with ESP-IDF)
///   Serial       -> ESP_LOG
///
/// The 16x96 matrix exposes an HTTP API on port 80:
///   GET  /        interactive pixel editor
///   GET  /docs    API documentation
///   POST /update  replace the whole frame (JSON array of 96x16 hex colors)
///   POST /text    render centered text

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "cJSON.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "led_strip.h"
#include "nvs_flash.h"

#include "secrets.h"

static const char *TAG = "ledsign";

#define LED_PIN 2
#define BRIGHTNESS 64

static const uint8_t kMatrixWidth = 16;
static const uint8_t kMatrixHeight = 96;
static const bool kMatrixSerpentineLayout = true;
static const bool kMatrixVertical = false;

#define NUM_LEDS (kMatrixWidth * kMatrixHeight)

// Largest POST body we will buffer. A full /update frame is ~16 KB of JSON;
// 64 KB leaves generous headroom while still rejecting absurd payloads.
#define MAX_BODY_LEN (64 * 1024)

static const unsigned long TEST_PATTERN_STEP_MS = 500;

// ---------------------------------------------------------------------------
// Minimal FastLED-compatible color type + helpers (Arduino-free)
// ---------------------------------------------------------------------------

// Plain aggregate so it stays trivially copyable (memset/memcpy-safe).
struct CRGB
{
  uint8_t r, g, b;
};

static const CRGB COLOR_RED = {255, 0, 0};
static const CRGB COLOR_GREEN = {0, 255, 0};
static const CRGB COLOR_BLUE = {0, 0, 255};

/// Milliseconds since boot, replacing Arduino's millis().
static inline unsigned long millis()
{
  return static_cast<unsigned long>(esp_timer_get_time() / 1000);
}

static inline void delay_ms(uint32_t ms)
{
  vTaskDelay(pdMS_TO_TICKS(ms));
}

/// FastLED's "fixed" scale8: scale one byte by a second one (0..255).
static inline uint8_t scale8(uint8_t i, uint8_t scale)
{
  return static_cast<uint8_t>((static_cast<uint16_t>(i) * static_cast<uint16_t>(scale + 1)) >> 8);
}

// Frame buffer. The original kept a "+1" safety pixel so leds[-1] was a legal
// scratch address; preserved here for behavioral parity.
static CRGB leds_plus_safety_pixel[NUM_LEDS + 1];
static CRGB *const leds(leds_plus_safety_pixel + 1);

static led_strip_handle_t strip;
// Guards the led buffer + strip against concurrent access from the HTTP server
// task and the WiFi event task. Recursive so show() can be called from within
// an already-locked critical section.
static SemaphoreHandle_t ledMutex;

#define LOCK_LEDS() xSemaphoreTakeRecursive(ledMutex, portMAX_DELAY)
#define UNLOCK_LEDS() xSemaphoreGiveRecursive(ledMutex)

static volatile bool gWifiConnected = false;
static bool splashActive = false;

/// Push the frame buffer to the strip, applying global brightness
/// (FastLED.setBrightness equivalent).
static void show()
{
  LOCK_LEDS();
  for (int i = 0; i < NUM_LEDS; i++)
  {
    led_strip_set_pixel(strip, i,
                        scale8(leds[i].r, BRIGHTNESS),
                        scale8(leds[i].g, BRIGHTNESS),
                        scale8(leds[i].b, BRIGHTNESS));
  }
  led_strip_refresh(strip);
  UNLOCK_LEDS();
}

static void clearLeds()
{
  memset(leds, 0, sizeof(CRGB) * NUM_LEDS);
}

static void fillSolid(CRGB color)
{
  for (int i = 0; i < NUM_LEDS; i++)
    leds[i] = color;
}

// ---------------------------------------------------------------------------
// XY mapping (unchanged)
// ---------------------------------------------------------------------------

uint16_t XY(uint8_t x, uint8_t y)
{
  uint16_t i;

  if (kMatrixSerpentineLayout == false)
  {
    if (kMatrixVertical == false)
    {
      i = (y * kMatrixWidth) + x;
    }
    else
    {
      i = kMatrixHeight * (kMatrixWidth - (x + 1)) + y;
    }
  }

  if (kMatrixSerpentineLayout == true)
  {
    if (kMatrixVertical == false)
    {
      if (y & 0x01)
      {
        uint8_t reverseX = (kMatrixWidth - 1) - x;
        i = (y * kMatrixWidth) + reverseX;
      }
      else
      {
        i = (y * kMatrixWidth) + x;
      }
    }
    else
    {
      if (x & 0x01)
      {
        i = kMatrixHeight * (kMatrixWidth - (x + 1)) + y;
      }
      else
      {
        i = kMatrixHeight * (kMatrixWidth - x) - (y + 1);
      }
    }
  }

  return i;
}

// ---------------------------------------------------------------------------
// 5x7 bitmap font, drawn rotated 90 degrees (unchanged from the sketch)
// ---------------------------------------------------------------------------

constexpr int GLYPH_WIDTH = 5;
constexpr int GLYPH_SPACING = 1;
constexpr int DIGIT_HEIGHT = 7;

struct Glyph
{
  uint8_t height;
  uint8_t rows[7];
};

const Glyph FONT_DIGITS[10] = {
    {7, {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}}, // 0
    {7, {0b00100, 0b01100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}}, // 1
    {7, {0b01110, 0b10001, 0b00001, 0b00010, 0b00100, 0b01000, 0b11111}}, // 2
    {7, {0b01110, 0b10001, 0b00001, 0b00110, 0b00001, 0b10001, 0b01110}}, // 3
    {7, {0b00010, 0b00110, 0b01010, 0b10010, 0b11111, 0b00010, 0b00010}}, // 4
    {7, {0b11111, 0b10000, 0b10000, 0b11110, 0b00001, 0b10001, 0b01110}}, // 5
    {7, {0b01110, 0b10000, 0b10000, 0b11110, 0b10001, 0b10001, 0b01110}}, // 6
    {7, {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b01000, 0b01000}}, // 7
    {7, {0b01110, 0b10001, 0b10001, 0b01110, 0b10001, 0b10001, 0b01110}}, // 8
    {7, {0b01110, 0b10001, 0b10001, 0b01111, 0b00001, 0b00001, 0b01110}}, // 9
};
const Glyph FONT_DOT = {2, {0b01100, 0b01100}};

const Glyph FONT_LETTERS[26] = {
    {7, {0b01110, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}}, // A
    {7, {0b11110, 0b10001, 0b10001, 0b11110, 0b10001, 0b10001, 0b11110}}, // B
    {7, {0b01111, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b01111}}, // C
    {7, {0b11110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b11110}}, // D
    {7, {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b11111}}, // E
    {7, {0b11111, 0b10000, 0b10000, 0b11110, 0b10000, 0b10000, 0b10000}}, // F
    {7, {0b01111, 0b10000, 0b10000, 0b10011, 0b10001, 0b10001, 0b01111}}, // G
    {7, {0b10001, 0b10001, 0b10001, 0b11111, 0b10001, 0b10001, 0b10001}}, // H
    {7, {0b01110, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b01110}}, // I
    {7, {0b00111, 0b00010, 0b00010, 0b00010, 0b00010, 0b10010, 0b01100}}, // J
    {7, {0b10001, 0b10010, 0b10100, 0b11000, 0b10100, 0b10010, 0b10001}}, // K
    {7, {0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b10000, 0b11111}}, // L
    {7, {0b10001, 0b11011, 0b10101, 0b10001, 0b10001, 0b10001, 0b10001}}, // M
    {7, {0b10001, 0b11001, 0b10101, 0b10011, 0b10001, 0b10001, 0b10001}}, // N
    {7, {0b01110, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}}, // O
    {7, {0b11110, 0b10001, 0b10001, 0b11110, 0b10000, 0b10000, 0b10000}}, // P
    {7, {0b01110, 0b10001, 0b10001, 0b10001, 0b10101, 0b10010, 0b01101}}, // Q
    {7, {0b11110, 0b10001, 0b10001, 0b11110, 0b10100, 0b10010, 0b10001}}, // R
    {7, {0b01111, 0b10000, 0b10000, 0b01110, 0b00001, 0b00001, 0b11110}}, // S
    {7, {0b11111, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100, 0b00100}}, // T
    {7, {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01110}}, // U
    {7, {0b10001, 0b10001, 0b10001, 0b10001, 0b10001, 0b01010, 0b00100}}, // V
    {7, {0b10001, 0b10001, 0b10001, 0b10101, 0b10101, 0b11011, 0b10001}}, // W
    {7, {0b10001, 0b10001, 0b01010, 0b00100, 0b01010, 0b10001, 0b10001}}, // X
    {7, {0b10001, 0b10001, 0b01010, 0b00100, 0b00100, 0b00100, 0b00100}}, // Y
    {7, {0b11111, 0b00001, 0b00010, 0b00100, 0b01000, 0b10000, 0b11111}}, // Z
};
const Glyph FONT_SPACE = {7, {0, 0, 0, 0, 0, 0, 0}};
const Glyph FONT_HYPHEN = {7, {0, 0, 0, 0b11111, 0, 0, 0}};

static const Glyph *glyphFor(char c)
{
  if (c >= '0' && c <= '9') return &FONT_DIGITS[c - '0'];
  if (c >= 'a' && c <= 'z') c = c - 'a' + 'A';
  if (c >= 'A' && c <= 'Z') return &FONT_LETTERS[c - 'A'];
  if (c == '.') return &FONT_DOT;
  if (c == '-') return &FONT_HYPHEN;
  if (c == ' ') return &FONT_SPACE;
  return nullptr;
}

static void drawGlyph(const Glyph *g, int xOff, int yOff, CRGB color)
{
  for (int r = 0; r < g->height; r++)
  {
    uint8_t row = g->rows[r];
    for (int c = 0; c < GLYPH_WIDTH; c++)
    {
      if (!(row & (1 << (GLYPH_WIDTH - 1 - c)))) continue;
      int x = xOff + (g->height - 1 - r);
      int y = yOff + c;
      if (x < 0 || x >= kMatrixWidth) continue;
      if (y < 0 || y >= kMatrixHeight) continue;
      leds[XY(x, y)] = color;
    }
  }
}

static void drawString(const char *s, int xStart, int yStart, CRGB color)
{
  int y = yStart;
  while (*s)
  {
    const Glyph *g = glyphFor(*s);
    if (g)
    {
      drawGlyph(g, xStart, y, color);
      y += GLYPH_WIDTH + GLYPH_SPACING;
    }
    s++;
  }
}

static int measureString(const char *s)
{
  int total = 0;
  bool any = false;
  while (*s)
  {
    if (glyphFor(*s))
    {
      total += GLYPH_WIDTH + GLYPH_SPACING;
      any = true;
    }
    s++;
  }
  if (any) total -= GLYPH_SPACING;
  return total;
}

// ---------------------------------------------------------------------------
// Frame helpers (unchanged logic)
// ---------------------------------------------------------------------------

static CRGB wifiIndicatorColor()
{
  return gWifiConnected ? COLOR_GREEN : COLOR_RED;
}

static void showSplash(const char *ip)
{
  LOCK_LEDS();
  clearLeds();
  int totalH = measureString(ip);
  int yStart = (kMatrixHeight - totalH) / 2;
  if (yStart < 0) yStart = 0;
  int xStart = (kMatrixWidth - DIGIT_HEIGHT) / 2;
  drawString(ip, xStart, yStart, COLOR_GREEN);
  show();
  UNLOCK_LEDS();
}

static void runStartupTestPattern()
{
  const CRGB colors[3] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE};
  for (int i = 0; i < 3; i++)
  {
    fillSolid(colors[i]);
    show();
    delay_ms(TEST_PATTERN_STEP_MS);
  }
  clearLeds();
  show();
}

static void applyMirror()
{
  for (uint8_t x = 0; x < kMatrixWidth; x++)
  {
    for (uint8_t y = 0; y < kMatrixHeight / 2; y++)
    {
      uint16_t a = XY(x, y);
      uint16_t b = XY(x, kMatrixHeight - 1 - y);
      CRGB tmp = leds[a];
      leds[a] = leds[b];
      leds[b] = tmp;
    }
  }
}

static int hexNibble(char c)
{
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  return -1;
}

static bool parseHexColor(const char *s, CRGB &out)
{
  if (s == nullptr) return false;
  if (*s == '#') s++;
  for (int i = 0; i < 6; i++)
  {
    if (hexNibble(s[i]) < 0) return false;
  }
  if (s[6] != '\0') return false;
  int r = (hexNibble(s[0]) << 4) | hexNibble(s[1]);
  int g = (hexNibble(s[2]) << 4) | hexNibble(s[3]);
  int b = (hexNibble(s[4]) << 4) | hexNibble(s[5]);
  out = CRGB{(uint8_t)r, (uint8_t)g, (uint8_t)b};
  return true;
}

// ---------------------------------------------------------------------------
// HTTP server
// ---------------------------------------------------------------------------

static httpd_handle_t httpServer = nullptr;

static esp_err_t sendError(httpd_req_t *req, const char *status, const char *message)
{
  httpd_resp_set_status(req, status);
  httpd_resp_set_type(req, "application/json");
  char body[160];
  snprintf(body, sizeof(body), "{\"error\":\"%s\"}", message);
  httpd_resp_sendstr(req, body);
  return ESP_OK;
}

/// Read the full request body into a heap buffer (caller frees). Returns
/// nullptr on missing/oversized body or a recv error.
static char *recvBody(httpd_req_t *req)
{
  size_t total = req->content_len;
  if (total == 0 || total > MAX_BODY_LEN) return nullptr;
  char *buf = static_cast<char *>(malloc(total + 1));
  if (buf == nullptr) return nullptr;
  size_t received = 0;
  while (received < total)
  {
    int r = httpd_req_recv(req, buf + received, total - received);
    if (r <= 0)
    {
      free(buf);
      return nullptr;
    }
    received += r;
  }
  buf[total] = '\0';
  return buf;
}

static bool queryHasMirror(httpd_req_t *req)
{
  char query[64];
  if (httpd_req_get_url_query_str(req, query, sizeof(query)) != ESP_OK) return false;
  char val[8];
  if (httpd_query_key_value(query, "mirror", val, sizeof(val)) != ESP_OK) return false;
  return strcmp(val, "1") == 0;
}

static const char INDEX_HTML[] = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>LED Sign</title>
<style>
body{margin:0;font-family:system-ui,sans-serif;background:#1a1a1a;color:#eee}
.nav{display:flex;align-items:center;gap:18px;background:#000;padding:10px 16px;border-bottom:1px solid #444}
.brand{font-weight:bold;font-size:16px}
.nav a{color:#9cf;text-decoration:none}
.nav a:hover{text-decoration:underline}
main{padding:12px;max-width:1400px;margin:0 auto}
.tb{display:flex;gap:12px;align-items:center;margin-bottom:12px;flex-wrap:wrap}
.palette{display:flex;gap:4px;flex-wrap:wrap;margin-bottom:12px}
.sw{width:28px;height:28px;border:2px solid #444;cursor:pointer;padding:0;border-radius:3px;background-clip:padding-box}
.sw:hover{border-color:#888}
.sw.active{border-color:#fff}
.sw.eraser{background-color:#000;background-image:linear-gradient(45deg,#444 25%,transparent 25%,transparent 75%,#444 75%),linear-gradient(45deg,#444 25%,transparent 25%,transparent 75%,#444 75%);background-size:10px 10px;background-position:0 0,5px 5px}
button{padding:6px 14px;font-size:14px;cursor:pointer;background:#444;color:#eee;border:1px solid #666;border-radius:3px}
button:hover{background:#555}
input[type=color]{width:40px;height:30px;border:1px solid #666;background:none;cursor:pointer;padding:0}
input[type=text]{padding:6px 8px;font-size:14px;background:#222;color:#eee;border:1px solid #666;border-radius:3px}
hr{border:none;border-top:1px solid #444;margin:16px 0}
.wrap{overflow-x:auto;max-width:100%;display:flex;justify-content:safe center}
.grid{display:grid;grid-template-columns:repeat(96,12px);grid-template-rows:repeat(16,12px);gap:1px;background:#333;padding:2px;width:max-content;user-select:none;touch-action:none;border:1px solid #555}
.cell{background:#000;cursor:crosshair}
#status{font-size:13px;opacity:.75;margin-left:8px}
</style>
</head>
<body>
<nav class="nav">
<span class="brand">Kunz LED dashboard</span>
<a href="/">Dashboard</a>
<a href="/docs">Docs</a>
</nav>
<main>
<div class="palette" id="pal">
<button class="sw eraser" title="Eraser" data-c="#000000"></button>
<button class="sw" style="background:#ffffff" title="White" data-c="#ffffff"></button>
<button class="sw" style="background:#808080" title="Gray" data-c="#808080"></button>
<button class="sw" style="background:#ff0000" title="Red" data-c="#ff0000"></button>
<button class="sw" style="background:#ff8000" title="Orange" data-c="#ff8000"></button>
<button class="sw" style="background:#ffff00" title="Yellow" data-c="#ffff00"></button>
<button class="sw" style="background:#00ff00" title="Green" data-c="#00ff00"></button>
<button class="sw" style="background:#008000" title="Dark green" data-c="#008000"></button>
<button class="sw" style="background:#00ffff" title="Cyan" data-c="#00ffff"></button>
<button class="sw" style="background:#0000ff" title="Blue" data-c="#0000ff"></button>
<button class="sw" style="background:#800080" title="Purple" data-c="#800080"></button>
<button class="sw" style="background:#ff00ff" title="Magenta" data-c="#ff00ff"></button>
<button class="sw" style="background:#ffc0cb" title="Pink" data-c="#ffc0cb"></button>
<button class="sw" style="background:#a0522d" title="Brown" data-c="#a0522d"></button>
</div>
<div class="tb">
<label>Color <input type="color" id="color" value="#ff0000"></label>
<label><input type="checkbox" id="mirror"> Mirror</label>
<button id="clr">Clear</button>
<span id="status"></span>
</div>
<div class="wrap"><div class="grid" id="g"></div></div>
<hr>
<div class="tb">
<label>Or text <input type="text" id="txt" maxlength="16" placeholder="HELLO" autocomplete="off"></label>
<button id="sendTxt">Stamp text</button>
</div>
<script>
const MX=16,MY=96;
const g=document.getElementById('g');
const ci=document.getElementById('color');
const mr=document.getElementById('mirror');
const st=document.getElementById('status');
const pal=document.getElementById('pal');
pal.addEventListener('click',e=>{
 const sw=e.target.closest('.sw');
 if(!sw)return;
 ci.value=sw.dataset.c;
 pal.querySelectorAll('.sw.active').forEach(s=>s.classList.remove('active'));
 sw.classList.add('active');
});
ci.addEventListener('input',()=>pal.querySelectorAll('.sw.active').forEach(s=>s.classList.remove('active')));
const px=Array.from({length:MY},()=>Array(MX).fill('#000000'));
let drawing=false,inFlight=false,dirty=false;
async function flush(){
 if(inFlight||!dirty)return;
 inFlight=true;dirty=false;
 st.textContent='syncing...';
 try{
  const url='/update'+(mr.checked?'?mirror=1':'');
  const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(px)});
  st.textContent=r.ok?'synced':'error '+r.status;
 }catch(e){st.textContent='fail: '+e.message}
 finally{inFlight=false;if(dirty)flush()}
}
function mark(){dirty=true;flush()}
mr.addEventListener('change',mark);
for(let row=0;row<MX;row++){
 for(let col=0;col<MY;col++){
  const c=document.createElement('div');
  c.className='cell';
  const my=col,mx=row;
  const paint=()=>{
   if(px[my][mx]===ci.value)return;
   px[my][mx]=ci.value;c.style.background=ci.value;mark();
  };
  c.addEventListener('pointerdown',e=>{drawing=true;paint();e.preventDefault()});
  c.addEventListener('pointerenter',()=>{if(drawing)paint()});
  g.appendChild(c);
 }
}
document.addEventListener('pointerup',()=>drawing=false);
document.addEventListener('pointercancel',()=>drawing=false);
document.getElementById('clr').onclick=()=>{
 for(const c of g.children)c.style.background='#000';
 for(let y=0;y<MY;y++)for(let x=0;x<MX;x++)px[y][x]='#000000';
 lastStamp=[];
 mark();
};
const FONT={
 '0':{h:7,r:[0b01110,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110]},
 '1':{h:7,r:[0b00100,0b01100,0b00100,0b00100,0b00100,0b00100,0b01110]},
 '2':{h:7,r:[0b01110,0b10001,0b00001,0b00010,0b00100,0b01000,0b11111]},
 '3':{h:7,r:[0b01110,0b10001,0b00001,0b00110,0b00001,0b10001,0b01110]},
 '4':{h:7,r:[0b00010,0b00110,0b01010,0b10010,0b11111,0b00010,0b00010]},
 '5':{h:7,r:[0b11111,0b10000,0b10000,0b11110,0b00001,0b10001,0b01110]},
 '6':{h:7,r:[0b01110,0b10000,0b10000,0b11110,0b10001,0b10001,0b01110]},
 '7':{h:7,r:[0b11111,0b00001,0b00010,0b00100,0b01000,0b01000,0b01000]},
 '8':{h:7,r:[0b01110,0b10001,0b10001,0b01110,0b10001,0b10001,0b01110]},
 '9':{h:7,r:[0b01110,0b10001,0b10001,0b01111,0b00001,0b00001,0b01110]},
 '.':{h:2,r:[0b01100,0b01100]},
 ' ':{h:7,r:[0,0,0,0,0,0,0]},
 '-':{h:7,r:[0,0,0,0b11111,0,0,0]},
 'A':{h:7,r:[0b01110,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001]},
 'B':{h:7,r:[0b11110,0b10001,0b10001,0b11110,0b10001,0b10001,0b11110]},
 'C':{h:7,r:[0b01111,0b10000,0b10000,0b10000,0b10000,0b10000,0b01111]},
 'D':{h:7,r:[0b11110,0b10001,0b10001,0b10001,0b10001,0b10001,0b11110]},
 'E':{h:7,r:[0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b11111]},
 'F':{h:7,r:[0b11111,0b10000,0b10000,0b11110,0b10000,0b10000,0b10000]},
 'G':{h:7,r:[0b01111,0b10000,0b10000,0b10011,0b10001,0b10001,0b01111]},
 'H':{h:7,r:[0b10001,0b10001,0b10001,0b11111,0b10001,0b10001,0b10001]},
 'I':{h:7,r:[0b01110,0b00100,0b00100,0b00100,0b00100,0b00100,0b01110]},
 'J':{h:7,r:[0b00111,0b00010,0b00010,0b00010,0b00010,0b10010,0b01100]},
 'K':{h:7,r:[0b10001,0b10010,0b10100,0b11000,0b10100,0b10010,0b10001]},
 'L':{h:7,r:[0b10000,0b10000,0b10000,0b10000,0b10000,0b10000,0b11111]},
 'M':{h:7,r:[0b10001,0b11011,0b10101,0b10001,0b10001,0b10001,0b10001]},
 'N':{h:7,r:[0b10001,0b11001,0b10101,0b10011,0b10001,0b10001,0b10001]},
 'O':{h:7,r:[0b01110,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110]},
 'P':{h:7,r:[0b11110,0b10001,0b10001,0b11110,0b10000,0b10000,0b10000]},
 'Q':{h:7,r:[0b01110,0b10001,0b10001,0b10001,0b10101,0b10010,0b01101]},
 'R':{h:7,r:[0b11110,0b10001,0b10001,0b11110,0b10100,0b10010,0b10001]},
 'S':{h:7,r:[0b01111,0b10000,0b10000,0b01110,0b00001,0b00001,0b11110]},
 'T':{h:7,r:[0b11111,0b00100,0b00100,0b00100,0b00100,0b00100,0b00100]},
 'U':{h:7,r:[0b10001,0b10001,0b10001,0b10001,0b10001,0b10001,0b01110]},
 'V':{h:7,r:[0b10001,0b10001,0b10001,0b10001,0b10001,0b01010,0b00100]},
 'W':{h:7,r:[0b10001,0b10001,0b10001,0b10101,0b10101,0b11011,0b10001]},
 'X':{h:7,r:[0b10001,0b10001,0b01010,0b00100,0b01010,0b10001,0b10001]},
 'Y':{h:7,r:[0b10001,0b10001,0b01010,0b00100,0b00100,0b00100,0b00100]},
 'Z':{h:7,r:[0b11111,0b00001,0b00010,0b00100,0b01000,0b10000,0b11111]}
};
const FW=5,FSP=1,FDH=7;
function gFor(c){if(c>='a'&&c<='z')c=c.toUpperCase();return FONT[c]}
function measureTxt(s){let t=0,any=false;for(const c of s){if(gFor(c)){t+=FW+FSP;any=true}}return any?t-FSP:0}
function paintCell(mx,my,color){
 if(mx<0||mx>=MX||my<0||my>=MY)return false;
 px[my][mx]=color;
 g.children[mx*MY+my].style.background=color;
 return true;
}
let lastStamp=[];
function drawTxt(s,color,record){
 const totalH=measureTxt(s);
 const yStart=Math.max(0,Math.floor((MY-totalH)/2));
 const xStart=Math.floor((MX-FDH)/2);
 let y=yStart;
 for(const c of s){
  const gp=gFor(c);
  if(!gp)continue;
  for(let r=0;r<gp.h;r++){
   const row=gp.r[r];
   for(let col=0;col<FW;col++){
    if(!(row&(1<<(FW-1-col))))continue;
    const mx=xStart+(gp.h-1-r),my=y+col;
    if(paintCell(mx,my,color)&&record)record.push([mx,my]);
   }
  }
  y+=FW+FSP;
 }
}
const txt=document.getElementById('txt');
function stampText(){
 const t=txt.value.trim();
 if(!t)return;
 for(const[mx,my]of lastStamp)paintCell(mx,my,'#000000');
 lastStamp=[];
 drawTxt(t,ci.value,lastStamp);
 mark();
}
document.getElementById('sendTxt').onclick=stampText;
txt.addEventListener('keydown',e=>{if(e.key==='Enter'){e.preventDefault();stampText()}});
</script>
</main>
</body>
</html>)rawliteral";

static const char DOCS_HTML[] = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Docs &mdash; Kunz LED dashboard</title>
<style>
body{margin:0;font-family:system-ui,sans-serif;background:#1a1a1a;color:#eee;line-height:1.5}
.nav{display:flex;align-items:center;gap:18px;background:#000;padding:10px 16px;border-bottom:1px solid #444}
.brand{font-weight:bold;font-size:16px}
.nav a{color:#9cf;text-decoration:none}
.nav a:hover{text-decoration:underline}
main{padding:16px;max-width:780px}
h1{margin-top:0}
h2{margin-top:32px;border-bottom:1px solid #333;padding-bottom:4px}
code{background:#0e0e0e;padding:2px 5px;border-radius:3px;font-family:ui-monospace,monospace;font-size:13px}
pre{background:#0e0e0e;padding:12px;border-radius:4px;border:1px solid #333;overflow-x:auto;font-family:ui-monospace,monospace;font-size:13px;line-height:1.4}
ul{padding-left:22px}
</style>
</head>
<body>
<nav class="nav">
<span class="brand">Kunz LED dashboard</span>
<a href="/">Dashboard</a>
<a href="/docs">Docs</a>
</nav>
<main>
<h1>API</h1>
<p>The sign exposes a small HTTP API on port 80. All endpoints accept JSON and respond with JSON.</p>

<h2>POST /update</h2>
<p>Replace every pixel on the 16&times;96 matrix.</p>
<p><b>Body</b> &mdash; a JSON array of 96 inner arrays, each containing 16 hex color strings (<code>"#rrggbb"</code>, leading <code>#</code> optional). The outer index is the long matrix axis (0&hellip;95); the inner index is the short axis (0&hellip;15).</p>
<p><b>Optional query</b> &mdash; <code>?mirror=1</code> flips the frame along the long axis after it is written.</p>
<p><b>Example</b>:</p>
<pre>curl -X POST http://&lt;device-ip&gt;/update \
  -H 'Content-Type: application/json' \
  --data-binary @frame.json</pre>
<p>where <code>frame.json</code> is a 96-row by 16-column array, e.g. solid red:</p>
<pre>[
  ["#ff0000","#ff0000", &hellip; 16 entries &hellip;],
  &hellip; 96 rows total &hellip;
]</pre>
<p><b>Responses</b>:</p>
<ul>
<li><code>200 {"status":"ok"}</code> on success.</li>
<li><code>400 {"error":"&lt;reason&gt;"}</code> for parse errors, wrong dimensions, or invalid hex strings.</li>
</ul>

<h2>POST /text</h2>
<p>Render centered text using the rotated 5&times;7 font (same one used for the boot IP splash).</p>
<p><b>Body</b>:</p>
<pre>{
  "text": "HELLO",
  "color": "#00ff00",
  "mirror": false
}</pre>
<ul>
<li><code>text</code> &mdash; required string. Supports A&ndash;Z (case-insensitive), 0&ndash;9, <code>.</code>, <code>-</code>, and space. About 16 characters fit on the long axis.</li>
<li><code>color</code> &mdash; optional hex; defaults to green.</li>
<li><code>mirror</code> &mdash; optional boolean; defaults to false.</li>
</ul>

<h2>WiFi indicator</h2>
<p>The first LED in the chain is reserved as a connectivity indicator &mdash; green when associated, red when not. Frames sent to <code>/update</code> and <code>/text</code> overwrite every other pixel; the indicator is restored after each frame is applied.</p>
</main>
</body>
</html>)rawliteral";

static esp_err_t handleRoot(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t handleDocs(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, DOCS_HTML, HTTPD_RESP_USE_STRLEN);
}

// Frame staging buffer. Only ever touched from the single HTTP server task, so
// it can be static (and kept off the limited handler stack).
static CRGB pending[NUM_LEDS];

static esp_err_t handleUpdate(httpd_req_t *req)
{
  char *body = recvBody(req);
  if (body == nullptr) return sendError(req, "400 Bad Request", "missing or oversized body");

  cJSON *doc = cJSON_Parse(body);
  free(body);
  if (doc == nullptr) return sendError(req, "400 Bad Request", "parse error");

  esp_err_t result;
  if (!cJSON_IsArray(doc))
  {
    result = sendError(req, "400 Bad Request", "expected top-level array");
  }
  else if (cJSON_GetArraySize(doc) != kMatrixHeight)
  {
    result = sendError(req, "400 Bad Request", "expected 96 rows");
  }
  else
  {
    bool ok = true;
    const char *errMsg = nullptr;
    for (uint8_t y = 0; y < kMatrixHeight && ok; y++)
    {
      cJSON *row = cJSON_GetArrayItem(doc, y);
      if (!cJSON_IsArray(row))
      {
        ok = false;
        errMsg = "row is not an array";
        break;
      }
      if (cJSON_GetArraySize(row) != kMatrixWidth)
      {
        ok = false;
        errMsg = "expected 16 columns per row";
        break;
      }
      for (uint8_t x = 0; x < kMatrixWidth; x++)
      {
        const char *hex = cJSON_GetStringValue(cJSON_GetArrayItem(row, x));
        CRGB color;
        if (!parseHexColor(hex, color))
        {
          ok = false;
          errMsg = "invalid hex color";
          break;
        }
        pending[XY(x, y)] = color;
      }
    }

    if (!ok)
    {
      result = sendError(req, "400 Bad Request", errMsg);
    }
    else
    {
      LOCK_LEDS();
      memcpy(leds, pending, sizeof(pending));
      if (queryHasMirror(req)) applyMirror();
      splashActive = false;
      leds[0] = wifiIndicatorColor();
      show();
      UNLOCK_LEDS();
      httpd_resp_set_type(req, "application/json");
      httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
      result = ESP_OK;
    }
  }

  cJSON_Delete(doc);
  return result;
}

static esp_err_t handleText(httpd_req_t *req)
{
  char *body = recvBody(req);
  if (body == nullptr) return sendError(req, "400 Bad Request", "missing or oversized body");

  cJSON *doc = cJSON_Parse(body);
  free(body);
  if (doc == nullptr) return sendError(req, "400 Bad Request", "parse error");

  esp_err_t result;
  const char *text = cJSON_GetStringValue(cJSON_GetObjectItem(doc, "text"));
  if (text == nullptr)
  {
    result = sendError(req, "400 Bad Request", "missing 'text'");
  }
  else
  {
    CRGB color = COLOR_GREEN;
    const char *colorStr = cJSON_GetStringValue(cJSON_GetObjectItem(doc, "color"));
    if (colorStr != nullptr && !parseHexColor(colorStr, color))
    {
      result = sendError(req, "400 Bad Request", "invalid color");
    }
    else
    {
      bool mirror = cJSON_IsTrue(cJSON_GetObjectItem(doc, "mirror"));
      LOCK_LEDS();
      clearLeds();
      int totalH = measureString(text);
      int yStart = (kMatrixHeight - totalH) / 2;
      if (yStart < 0) yStart = 0;
      int xStart = (kMatrixWidth - DIGIT_HEIGHT) / 2;
      drawString(text, xStart, yStart, color);
      if (mirror) applyMirror();
      splashActive = false;
      leds[0] = wifiIndicatorColor();
      show();
      UNLOCK_LEDS();
      httpd_resp_set_type(req, "application/json");
      httpd_resp_sendstr(req, "{\"status\":\"ok\"}");
      result = ESP_OK;
    }
  }

  cJSON_Delete(doc);
  return result;
}

static esp_err_t handleNotFound(httpd_req_t *req, httpd_err_code_t err)
{
  return sendError(req, "404 Not Found", "not found");
}

static void startServer()
{
  if (httpServer != nullptr) return;
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.stack_size = 8192;       // headroom for cJSON parsing of large frames
  config.lru_purge_enable = true; // reclaim sockets under load
  if (httpd_start(&httpServer, &config) != ESP_OK)
  {
    ESP_LOGE(TAG, "Failed to start HTTP server");
    httpServer = nullptr;
    return;
  }

  httpd_uri_t root = {"/", HTTP_GET, handleRoot, nullptr};
  httpd_uri_t docs = {"/docs", HTTP_GET, handleDocs, nullptr};
  httpd_uri_t update = {"/update", HTTP_POST, handleUpdate, nullptr};
  httpd_uri_t text = {"/text", HTTP_POST, handleText, nullptr};
  httpd_register_uri_handler(httpServer, &root);
  httpd_register_uri_handler(httpServer, &docs);
  httpd_register_uri_handler(httpServer, &update);
  httpd_register_uri_handler(httpServer, &text);
  httpd_register_err_handler(httpServer, HTTPD_404_NOT_FOUND, handleNotFound);

  ESP_LOGI(TAG, "HTTP server listening on port 80 (GET /, GET /docs, POST /update, POST /text)");
}

// ---------------------------------------------------------------------------
// WiFi station
// ---------------------------------------------------------------------------

static void onWifiConnected(const char *ip)
{
  gWifiConnected = true;
  ESP_LOGI(TAG, "Connected. IP: %s", ip);
  showSplash(ip);
  splashActive = true;
  startServer();
}

static void onWifiDisconnected()
{
  bool wasConnected = gWifiConnected;
  gWifiConnected = false;
  if (wasConnected)
  {
    LOCK_LEDS();
    leds[0] = wifiIndicatorColor();
    show();
    UNLOCK_LEDS();
  }
}

static void wifiEventHandler(void *arg, esp_event_base_t base, int32_t id, void *data)
{
  if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START)
  {
    esp_wifi_connect();
  }
  else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED)
  {
    onWifiDisconnected();
    esp_wifi_connect(); // keep retrying, like the Arduino auto-reconnect loop
  }
  else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP)
  {
    ip_event_got_ip_t *event = static_cast<ip_event_got_ip_t *>(data);
    char ip[16];
    snprintf(ip, sizeof(ip), IPSTR, IP2STR(&event->ip_info.ip));
    onWifiConnected(ip);
  }
}

static void startWifi()
{
  ESP_ERROR_CHECK(esp_netif_init());
  ESP_ERROR_CHECK(esp_event_loop_create_default());
  esp_netif_create_default_wifi_sta();

  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  ESP_ERROR_CHECK(esp_wifi_init(&cfg));

  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      WIFI_EVENT, ESP_EVENT_ANY_ID, &wifiEventHandler, nullptr, nullptr));
  ESP_ERROR_CHECK(esp_event_handler_instance_register(
      IP_EVENT, IP_EVENT_STA_GOT_IP, &wifiEventHandler, nullptr, nullptr));

  wifi_config_t wc = {};
  strncpy(reinterpret_cast<char *>(wc.sta.ssid), WIFI_SSID, sizeof(wc.sta.ssid) - 1);
  strncpy(reinterpret_cast<char *>(wc.sta.password), WIFI_PASSWORD, sizeof(wc.sta.password) - 1);

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
  ESP_ERROR_CHECK(esp_wifi_start());

  ESP_LOGI(TAG, "Connecting to WiFi '%s'...", WIFI_SSID);
}

// ---------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------

static void setupStrip()
{
  led_strip_config_t strip_config = {};
  strip_config.strip_gpio_num = LED_PIN;
  strip_config.max_leds = NUM_LEDS;
  strip_config.led_model = LED_MODEL_WS2812;
  strip_config.color_component_format = LED_STRIP_COLOR_COMPONENT_FMT_GRB;
  strip_config.flags.invert_out = false;

  led_strip_rmt_config_t rmt_config = {};
  rmt_config.clk_src = RMT_CLK_SRC_DEFAULT;
  rmt_config.resolution_hz = 10 * 1000 * 1000;
  rmt_config.mem_block_symbols = 64;
  rmt_config.flags.with_dma = false;

  ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &strip));
  ESP_ERROR_CHECK(led_strip_clear(strip));
}

extern "C" void app_main(void)
{
  // NVS is required by the WiFi stack.
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);

  ledMutex = xSemaphoreCreateRecursiveMutex();
  setupStrip();

  runStartupTestPattern();

  // Red until associated; the WiFi events drive it green + splash + server.
  LOCK_LEDS();
  leds[0] = COLOR_RED;
  show();
  UNLOCK_LEDS();

  startWifi();
  // Setup complete. The WiFi event task and HTTP server task carry on from here.
}

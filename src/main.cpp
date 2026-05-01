#include <FastLED.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

#include "secrets.h"

#define LED_PIN 2
#define COLOR_ORDER GRB
#define CHIPSET WS2811
#define BRIGHTNESS 64

const uint8_t kMatrixWidth = 16;
const uint8_t kMatrixHeight = 96;
const bool kMatrixSerpentineLayout = true;
const bool kMatrixVertical = false;

#define NUM_LEDS (kMatrixWidth * kMatrixHeight)
CRGB leds_plus_safety_pixel[NUM_LEDS + 1];
CRGB *const leds(leds_plus_safety_pixel + 1);

const unsigned long WIFI_CHECK_INTERVAL_MS = 1000;
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 20000;
const unsigned long TEST_PATTERN_STEP_MS = 500;

WebServer server(80);

bool splashActive = false;
unsigned long lastWifiCheckMs = 0;
bool lastWifiConnected = false;
bool serverStarted = false;

void handleUpdate();
void handleRoot();
void handleDocs();
void handleText();
static void sendError(int code, const char *message);

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

// 5-wide bitmap font for digits and '.'. Each row stored in low 5 bits, MSB = leftmost.
// Digits are 7 rows tall; the '.' is a compact 2-row dot so a full IP fits along the long axis.
// Glyphs are drawn rotated 90° clockwise: a tall digit sits across matrix-x and chars stride
// along matrix-y.
constexpr int GLYPH_WIDTH = 5;
constexpr int GLYPH_SPACING = 1;
constexpr int DIGIT_HEIGHT = 7; // max glyph height; sets the rotated x-extent

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

static CRGB wifiIndicatorColor()
{
  return (WiFi.status() == WL_CONNECTED) ? CRGB::Green : CRGB::Red;
}

static void showSplash(const String &ip)
{
  FastLED.clear();
  int totalH = measureString(ip.c_str());
  int yStart = (kMatrixHeight - totalH) / 2;
  if (yStart < 0) yStart = 0;
  int xStart = (kMatrixWidth - DIGIT_HEIGHT) / 2;
  drawString(ip.c_str(), xStart, yStart, CRGB::Green);
  FastLED.show();
}

static void runStartupTestPattern()
{
  const CRGB colors[3] = {CRGB::Red, CRGB::Green, CRGB::Blue};
  for (int i = 0; i < 3; i++)
  {
    fill_solid(leds, NUM_LEDS, colors[i]);
    FastLED.show();
    delay(TEST_PATTERN_STEP_MS);
  }
  FastLED.clear();
  FastLED.show();
}

static bool connectWifi(unsigned long timeoutMs)
{
  WiFi.mode(WIFI_STA);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.printf("Connecting to WiFi '%s'", WIFI_SSID);
  unsigned long deadline = millis() + timeoutMs;
  while (WiFi.status() != WL_CONNECTED && millis() < deadline)
  {
    delay(250);
    Serial.print('.');
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

static void beginSplash()
{
  Serial.print("Connected. IP: ");
  Serial.println(WiFi.localIP());
  showSplash(WiFi.localIP().toString());
  splashActive = true;
}

static void startServer()
{
  if (serverStarted) return;
  server.on("/", HTTP_GET, handleRoot);
  server.on("/docs", HTTP_GET, handleDocs);
  server.on("/update", HTTP_POST, handleUpdate);
  server.on("/text", HTTP_POST, handleText);
  server.onNotFound([]() { sendError(404, "not found"); });
  server.begin();
  serverStarted = true;
  Serial.println("HTTP server listening on port 80 (GET /, GET /docs, POST /update, POST /text)");
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
  out = CRGB(r, g, b);
  return true;
}

static void sendError(int code, const char *message)
{
  String body = "{\"error\":\"";
  body += message;
  body += "\"}";
  server.send(code, "application/json", body);
}

const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
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

void handleRoot()
{
  server.send_P(200, "text/html", INDEX_HTML);
}

const char DOCS_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
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

void handleDocs()
{
  server.send_P(200, "text/html", DOCS_HTML);
}

void handleUpdate()
{
  if (!server.hasArg("plain"))
  {
    sendError(400, "missing body");
    return;
  }
  String body = server.arg("plain");

  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, body);
  if (err)
  {
    sendError(400, err.c_str());
    return;
  }
  if (!doc.is<JsonArray>())
  {
    sendError(400, "expected top-level array");
    return;
  }
  JsonArray rows = doc.as<JsonArray>();
  if (rows.size() != kMatrixHeight)
  {
    sendError(400, "expected 96 rows");
    return;
  }

  CRGB pending[NUM_LEDS];

  for (uint8_t y = 0; y < kMatrixHeight; y++)
  {
    JsonVariant rowVar = rows[y];
    if (!rowVar.is<JsonArray>())
    {
      sendError(400, "row is not an array");
      return;
    }
    JsonArray row = rowVar.as<JsonArray>();
    if (row.size() != kMatrixWidth)
    {
      sendError(400, "expected 16 columns per row");
      return;
    }
    for (uint8_t x = 0; x < kMatrixWidth; x++)
    {
      const char *hex = row[x].as<const char *>();
      CRGB color;
      if (!parseHexColor(hex, color))
      {
        sendError(400, "invalid hex color");
        return;
      }
      pending[XY(x, y)] = color;
    }
  }

  memcpy(leds, pending, sizeof(pending));
  if (server.arg("mirror") == "1") applyMirror();
  splashActive = false;
  leds[0] = wifiIndicatorColor();
  FastLED.show();
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void handleText()
{
  if (!server.hasArg("plain"))
  {
    sendError(400, "missing body");
    return;
  }
  JsonDocument doc;
  DeserializationError err = deserializeJson(doc, server.arg("plain"));
  if (err)
  {
    sendError(400, err.c_str());
    return;
  }
  const char *text = doc["text"].as<const char *>();
  if (text == nullptr)
  {
    sendError(400, "missing 'text'");
    return;
  }
  CRGB color = CRGB::Green;
  const char *colorStr = doc["color"].as<const char *>();
  if (colorStr != nullptr && !parseHexColor(colorStr, color))
  {
    sendError(400, "invalid color");
    return;
  }

  FastLED.clear();
  int totalH = measureString(text);
  int yStart = (kMatrixHeight - totalH) / 2;
  if (yStart < 0) yStart = 0;
  int xStart = (kMatrixWidth - DIGIT_HEIGHT) / 2;
  drawString(text, xStart, yStart, color);
  if (doc["mirror"] | false) applyMirror();

  splashActive = false;
  leds[0] = wifiIndicatorColor();
  FastLED.show();
  server.send(200, "application/json", "{\"status\":\"ok\"}");
}

void setup()
{
  Serial.begin(115200);

  FastLED.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalSMD5050);
  FastLED.setBrightness(BRIGHTNESS);

  runStartupTestPattern();

  leds[0] = CRGB::Red;
  FastLED.show();
  lastWifiConnected = false;

  if (connectWifi(WIFI_CONNECT_TIMEOUT_MS))
  {
    lastWifiConnected = true;
    leds[0] = CRGB::Green;
    FastLED.show();
    beginSplash();
    startServer();
  }
  else
  {
    Serial.println("WiFi connect timed out; staying red and will keep retrying.");
  }
  lastWifiCheckMs = millis();
}

void loop()
{
  if (serverStarted) server.handleClient();

  unsigned long now = millis();

  if (!splashActive && now - lastWifiCheckMs >= WIFI_CHECK_INTERVAL_MS)
  {
    lastWifiCheckMs = now;
    bool connected = (WiFi.status() == WL_CONNECTED);
    if (connected != lastWifiConnected)
    {
      lastWifiConnected = connected;
      leds[0] = wifiIndicatorColor();
      FastLED.show();
    }
    if (connected && !serverStarted)
    {
      beginSplash();
      startServer();
    }
  }
}

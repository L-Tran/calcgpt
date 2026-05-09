// CALCGPT Firmware v1.1 - fixed timing/race conditions
#include <TICL.h>
#include <CBL2.h>
#include <TIVar.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <UrlEncode.h>

constexpr auto TIP  = D0;
constexpr auto RING = D2;

constexpr auto MAXHDRLEN    = 16;
constexpr auto MAXDATALEN   = 4096;
constexpr auto MAXARGS      = 5;
constexpr auto MAXSTRARGLEN = 256;

const char* AP_SSID = "CALCGPT";
const char* AP_PASS = "calcgpt123";

CBL2        cbl;
Preferences prefs;
WebServer   server(80);

String savedSSID, savedPass, savedAPIKey, savedTelegramToken, savedTelegramChatID;

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
//
// The calculator and the ESP32 talk through three TI variables:
//   C  -> command id  (calc -> esp, write)
//   S  -> status      (esp -> calc, read: 0=busy, 1=done)
//   E  -> error       (esp -> calc, read: 0=ok, 1=error)
//   Str0 (0xAA) -> message (esp -> calc, read)
//
// Old code's bug: as soon as the calc wrote C, we kicked a FreeRTOS task that
// scribbled all over `status`, `error`, and `message[]` while the calc was
// still polling them. The calc could read S=1 left over from the *previous*
// command before the new task even started, or read message[] mid-write.
//
// Fix: a strict state machine, all shared state guarded by a mutex, and the
// calc only ever sees status=1 once the response is fully assembled.

enum RunState {
  STATE_IDLE,        // no command in flight; last result still readable
  STATE_PENDING,     // calc sent C, waiting for all args before dispatch
  STATE_RUNNING,     // background task executing
  STATE_DONE         // result is ready; status=1 will be reported
};

volatile RunState runState = STATE_IDLE;
SemaphoreHandle_t stateMutex = NULL;

// All of these are guarded by stateMutex:
int    currentArg = 0;
char   strArgs[MAXARGS][MAXSTRARGLEN];
double realArgs[MAXARGS];
int    command       = -1;
int    pendingCommand = -1;
volatile bool replyError   = false;
volatile bool replyReady   = false;  // what the calc reads as "S"
char   message[MAXSTRARGLEN];

// Chat history - only touched from the API task or from sync commands while
// runState == STATE_IDLE, so no separate lock needed.
const int MAX_HISTORY = 10;
String historyRoles[MAX_HISTORY];
String historyMessages[MAX_HISTORY];
int    historyCount = 0;

// Paged response buffer. Written only by the API task; read by cmdSendPage()
// which is called either from the API task (initial page) or from the calc
// thread when it asks for a new page via V. We snapshot under the mutex.
String fullResponse = "";
int    PAGE_PAGE    = 0;

uint8_t header[MAXHDRLEN];
uint8_t data[MAXDATALEN];

TaskHandle_t apiTask = NULL;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

#define LOCK()   xSemaphoreTake(stateMutex, portMAX_DELAY)
#define UNLOCK() xSemaphoreGive(stateMutex)

void cmdConnect();
void cmdDisconnect();
void cmdConfigure();
void cmdGPT();
void cmdReply();
void cmdClearChat();
void cmdLauncher();
void cmdTelegramSend();
void cmdTelegramRecv();
void renderPage();   // renamed from cmdSendPage; pure render, no status

struct Command {
  int         id;
  const char* name;
  int         num_args;
  void        (*fp)();
  bool        wifi;
  bool        async;
};

struct Command commands[] = {
  {0, "connect",    0, cmdConnect,      false, true },
  {1, "disconnect", 0, cmdDisconnect,   false, false},
  {2, "gpt",        1, cmdGPT,          true,  true },
  {3, "reply",      1, cmdReply,        true,  true },
  {4, "clearChat",  0, cmdClearChat,    false, false},
  {5, "launcher",   0, cmdLauncher,     false, false},
  {6, "configure",  0, cmdConfigure,    false, false},
  {7, "tg_send",    1, cmdTelegramSend, true,  true },
  {8, "tg_recv",    0, cmdTelegramRecv, true,  true },
};

constexpr int NUMCOMMANDS = sizeof(commands) / sizeof(struct Command);
constexpr int MAXCOMMAND  = 8;

uint8_t launcher_program[] = {
  0xd6, 0x4d, 0x3f, 0xe1, 0x3f, 0xe6, 0x2a, 0x43, 0x41, 0x4c, 0x43, 0x47, 0x50, 0x54, 0x2a, 0x2b,
  0x2a, 0x43, 0x48, 0x41, 0x54, 0x2a, 0x2b, 0x41, 0x2b, 0x2a, 0x54, 0x45, 0x4c, 0x45, 0x47, 0x52,
  0x41, 0x4d, 0x2a, 0x2b, 0x42, 0x2b, 0x2a, 0x53, 0x45, 0x54, 0x54, 0x49, 0x4e, 0x47, 0x53, 0x2a,
  0x2b, 0x43, 0x2b, 0x2a, 0x51, 0x55, 0x49, 0x54, 0x2a, 0x2b, 0x44, 0x3f, 0xd6, 0x41, 0x3f, 0xe1,
  0x3f, 0xde, 0x2a, 0x54, 0x59, 0x50, 0x45, 0x29, 0x51, 0x55, 0x45, 0x53, 0x54, 0x49, 0x4f, 0x4e,
  0x3e, 0x2a, 0x3f, 0xdc, 0xaa, 0x00, 0x3f, 0x32, 0x04, 0x43, 0x3f, 0xe7, 0x43, 0x3f, 0xe7, 0xaa,
  0x00, 0x3f, 0xd3, 0x57, 0x2b, 0x31, 0x2b, 0x32, 0x30, 0x30, 0x3f, 0xd4, 0x3f, 0xd6, 0x5a, 0x3f,
  0xe8, 0x53, 0x3f, 0xce, 0x53, 0x6a, 0x30, 0x3e, 0xd7, 0x5a, 0x3f, 0x30, 0x04, 0x56, 0x3f, 0xe7,
  0x56, 0x3f, 0xe8, 0xaa, 0x00, 0x3f, 0xd7, 0x51, 0x3f, 0xd6, 0x51, 0x3f, 0xe1, 0x3f, 0xe0, 0x31,
  0x2b, 0x31, 0x2b, 0xaa, 0x00, 0x3f, 0xd8, 0x3f, 0xe6, 0x2a, 0x4f, 0x50, 0x54, 0x49, 0x4f, 0x4e,
  0x53, 0x2a, 0x2b, 0x2a, 0x50, 0x52, 0x45, 0x56, 0x29, 0x50, 0x47, 0x2a, 0x2b, 0x50, 0x2b, 0x2a,
  0x4e, 0x45, 0x58, 0x54, 0x29, 0x50, 0x47, 0x2a, 0x2b, 0x4e, 0x2b, 0x2a, 0x52, 0x45, 0x50, 0x4c,
  0x59, 0x2a, 0x2b, 0x52, 0x2b, 0x2a, 0x4e, 0x45, 0x57, 0x29, 0x43, 0x48, 0x41, 0x54, 0x2a, 0x2b,
  0x41, 0x2b, 0x2a, 0x42, 0x41, 0x43, 0x4b, 0x2a, 0x2b, 0x4d, 0x3f, 0xd6, 0x50, 0x3f, 0x56, 0x71,
  0x31, 0x04, 0x56, 0x3f, 0xce, 0x56, 0x6b, 0x30, 0x3e, 0x30, 0x04, 0x56, 0x3f, 0xe7, 0x56, 0x3f,
  0xe8, 0xaa, 0x00, 0x3f, 0xd7, 0x51, 0x3f, 0xd6, 0x4e, 0x3f, 0x56, 0x70, 0x31, 0x04, 0x56, 0x3f,
  0xe7, 0x56, 0x3f, 0xe8, 0xaa, 0x00, 0x3f, 0xd7, 0x51, 0x3f, 0xd6, 0x52, 0x3f, 0xe1, 0x3f, 0xde,
  0x2a, 0x59, 0x4f, 0x55, 0x52, 0x29, 0x52, 0x45, 0x50, 0x4c, 0x59, 0x3e, 0x2a, 0x3f, 0xdc, 0xaa,
  0x00, 0x3f, 0x33, 0x04, 0x43, 0x3f, 0xe7, 0x43, 0x3f, 0xe7, 0xaa, 0x00, 0x3f, 0xd3, 0x57, 0x2b,
  0x31, 0x2b, 0x32, 0x30, 0x30, 0x3f, 0xd4, 0x3f, 0xd6, 0x59, 0x3f, 0xe8, 0x53, 0x3f, 0xce, 0x53,
  0x6a, 0x30, 0x3e, 0xd7, 0x59, 0x3f, 0x30, 0x04, 0x56, 0x3f, 0xe7, 0x56, 0x3f, 0xe8, 0xaa, 0x00,
  0x3f, 0xd7, 0x51, 0x3f, 0xd6, 0x42, 0x3f, 0xe1, 0x3f, 0xe6, 0x2a, 0x54, 0x45, 0x4c, 0x45, 0x47,
  0x52, 0x41, 0x4d, 0x2a, 0x2b, 0x2a, 0x53, 0x45, 0x4e, 0x44, 0x29, 0x4d, 0x53, 0x47, 0x2a, 0x2b,
  0x46, 0x2b, 0x2a, 0x52, 0x45, 0x41, 0x44, 0x29, 0x4d, 0x53, 0x47, 0x2a, 0x2b, 0x47, 0x2b, 0x2a,
  0x42, 0x41, 0x43, 0x4b, 0x2a, 0x2b, 0x4d, 0x3f, 0xd6, 0x46, 0x3f, 0xe1, 0x3f, 0xde, 0x2a, 0x53,
  0x45, 0x4e, 0x44, 0x29, 0x4d, 0x53, 0x47, 0x3e, 0x2a, 0x3f, 0xdc, 0xaa, 0x00, 0x3f, 0x37, 0x04,
  0x43, 0x3f, 0xe7, 0x43, 0x3f, 0xe7, 0xaa, 0x00, 0x3f, 0xd3, 0x57, 0x2b, 0x31, 0x2b, 0x32, 0x30,
  0x30, 0x3f, 0xd4, 0x3f, 0xd6, 0x54, 0x3f, 0xe8, 0x53, 0x3f, 0xce, 0x53, 0x6a, 0x30, 0x3e, 0xd7,
  0x54, 0x3f, 0xe8, 0xaa, 0x00, 0x3f, 0xe1, 0x3f, 0xe0, 0x31, 0x2b, 0x31, 0x2b, 0xaa, 0x00, 0x3f,
  0xd8, 0x3f, 0xd7, 0x42, 0x3f, 0xd6, 0x47, 0x3f, 0xe1, 0x3f, 0x38, 0x04, 0x43, 0x3f, 0xe7, 0x43,
  0x3f, 0xd3, 0x57, 0x2b, 0x31, 0x2b, 0x31, 0x30, 0x30, 0x3f, 0xd4, 0x3f, 0xd6, 0x55, 0x3f, 0xe8,
  0x53, 0x3f, 0xce, 0x53, 0x6a, 0x30, 0x3e, 0xd7, 0x55, 0x3f, 0xe8, 0xaa, 0x00, 0x3f, 0xe1, 0x3f,
  0xe0, 0x31, 0x2b, 0x31, 0x2b, 0xaa, 0x00, 0x3f, 0xd8, 0x3f, 0xd7, 0x42, 0x3f, 0xd6, 0x43, 0x3f,
  0xe1, 0x3f, 0xe6, 0x2a, 0x53, 0x45, 0x54, 0x54, 0x49, 0x4e, 0x47, 0x53, 0x2a, 0x2b, 0x2a, 0x43,
  0x4f, 0x4e, 0x4e, 0x45, 0x43, 0x54, 0x2a, 0x2b, 0x48, 0x2b, 0x2a, 0x44, 0x49, 0x53, 0x43, 0x4f,
  0x4e, 0x4e, 0x43, 0x54, 0x2a, 0x2b, 0x49, 0x2b, 0x2a, 0x43, 0x4f, 0x4e, 0x46, 0x49, 0x47, 0x55,
  0x52, 0x45, 0x2a, 0x2b, 0x4a, 0x2b, 0x2a, 0x42, 0x41, 0x43, 0x4b, 0x2a, 0x2b, 0x4d, 0x3f, 0xd6,
  0x48, 0x3f, 0x30, 0x04, 0x43, 0x3f, 0xe7, 0x43, 0x3f, 0xd3, 0x57, 0x2b, 0x31, 0x2b, 0x31, 0x30,
  0x30, 0x3f, 0xd4, 0x3f, 0xd6, 0x58, 0x3f, 0xe8, 0x53, 0x3f, 0xce, 0x53, 0x6a, 0x30, 0x3e, 0xd7,
  0x58, 0x3f, 0xe8, 0xaa, 0x00, 0x3f, 0xe1, 0x3f, 0xe0, 0x31, 0x2b, 0x31, 0x2b, 0xaa, 0x00, 0x3f,
  0xd8, 0x3f, 0xd7, 0x43, 0x3f, 0xd6, 0x49, 0x3f, 0x31, 0x04, 0x43, 0x3f, 0xe7, 0x43, 0x3f, 0xd3,
  0x57, 0x2b, 0x31, 0x2b, 0x31, 0x30, 0x30, 0x3f, 0xd4, 0x3f, 0xd6, 0x4f, 0x3f, 0xe8, 0x53, 0x3f,
  0xce, 0x53, 0x6a, 0x30, 0x3e, 0xd7, 0x4f, 0x3f, 0xe8, 0xaa, 0x00, 0x3f, 0xe1, 0x3f, 0xe0, 0x31,
  0x2b, 0x31, 0x2b, 0xaa, 0x00, 0x3f, 0xd8, 0x3f, 0xd7, 0x43, 0x3f, 0xd6, 0x4a, 0x3f, 0x36, 0x04,
  0x43, 0x3f, 0xe7, 0x43, 0x3f, 0xd3, 0x57, 0x2b, 0x31, 0x2b, 0x31, 0x30, 0x30, 0x3f, 0xd4, 0x3f,
  0xd6, 0x4b, 0x3f, 0xe8, 0x53, 0x3f, 0xce, 0x53, 0x6a, 0x30, 0x3e, 0xd7, 0x4b, 0x3f, 0xe8, 0xaa,
  0x00, 0x3f, 0xe1, 0x3f, 0xe0, 0x31, 0x2b, 0x31, 0x2b, 0xaa, 0x00, 0x3f, 0xd8, 0x3f, 0xd7, 0x43,
  0x3f, 0xd6, 0x44, 0x3f, 0xe1, 0x3f, 0xde, 0x2a, 0x47, 0x4f, 0x4f, 0x44, 0x42, 0x59, 0x45, 0x2d,
  0x2a, 0x3f, 0xd9
};
size_t launcher_program_len = 771;

void fixStrVar(char* str) {
  int end = strlen(str);
  while (end > 0 && (str[end-1] < 32 || str[end-1] > 126)) end--;
  str[end] = '\0';
}

// Set the "loading" state under lock. Called when the calc dispatches a new
// command. Crucially this clears replyReady BEFORE the API task starts, so
// the calc cannot see leftover S=1 from a previous command.
void enterRunning(const char* loadingMsg) {
  LOCK();
  replyReady = false;
  replyError = false;
  strncpy(message, loadingMsg, MAXSTRARGLEN - 1);
  message[MAXSTRARGLEN - 1] = '\0';
  runState = STATE_RUNNING;
  UNLOCK();
}

// Atomically publish a final result to the calc.
void publishResult(bool err, const char* msg) {
  LOCK();
  replyError = err;
  strncpy(message, msg, MAXSTRARGLEN - 1);
  message[MAXSTRARGLEN - 1] = '\0';
  replyReady = true;        // calc may now see S=1
  runState   = STATE_DONE;
  UNLOCK();
  Serial.print(err ? "ERROR: " : "SUCCESS: ");
  Serial.println(msg);
}

void setError(const char* err)   { publishResult(true,  err); }
void setSuccess(const char* msg) { publishResult(false, msg); }

// ---------------------------------------------------------------------------
// Web config (unchanged)
// ---------------------------------------------------------------------------
const char* configPage =
  "<!DOCTYPE html><html><head><title>CALCGPT Setup</title>"
  "<style>body{font-family:monospace;background:#000;color:#0f0;padding:20px;}"
  "h1,h3{color:#0f0;}input{background:#111;color:#0f0;border:1px solid #0f0;"
  "padding:8px;width:100%;margin:8px 0;}button{background:#0f0;color:#000;"
  "border:none;padding:10px 20px;cursor:pointer;font-weight:bold;}</style>"
  "</head><body><h1>CALCGPT SETUP</h1><form action='/save' method='POST'>"
  "<h3>WiFi</h3><input type='text' name='ssid' placeholder='WiFi Name'>"
  "<input type='password' name='pass' placeholder='WiFi Password'>"
  "<h3>OpenAI</h3><input type='text' name='apikey' placeholder='OpenAI API Key (sk-...)'>"
  "<h3>Telegram (optional)</h3><input type='text' name='tgtoken' placeholder='Telegram Bot Token'>"
  "<input type='text' name='tgchat' placeholder='Telegram Chat ID'><br><br>"
  "<button type='submit'>SAVE AND CONNECT</button></form></body></html>";

void handleRoot() { server.send(200, "text/html", configPage); }

void handleSave() {
  savedSSID           = server.arg("ssid");
  savedPass           = server.arg("pass");
  savedAPIKey         = server.arg("apikey");
  savedTelegramToken  = server.arg("tgtoken");
  savedTelegramChatID = server.arg("tgchat");
  prefs.begin("calcgpt", false);
  prefs.putString("ssid",    savedSSID);
  prefs.putString("pass",    savedPass);
  prefs.putString("apikey",  savedAPIKey);
  prefs.putString("tgtoken", savedTelegramToken);
  prefs.putString("tgchat",  savedTelegramChatID);
  prefs.end();
  server.send(200, "text/html",
    "<html><body style='background:#000;color:#0f0;font-family:monospace;padding:20px'>"
    "<h1>SAVED!</h1><p>Run CONNECT on calculator.</p></body></html>");
}

void startAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
  server.on("/",     handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
  Serial.println("AP started: CALCGPT / 192.168.4.1");
}

// ---------------------------------------------------------------------------
// Sync commands
// ---------------------------------------------------------------------------
void cmdConnect() {
  if (WiFi.status() == WL_CONNECTED) { setSuccess("already connected!"); return; }
  if (savedSSID == "") { setError("no credentials saved"); return; }
  WiFi.mode(WIFI_STA);
  WiFi.begin(savedSSID.c_str(), savedPass.c_str());
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 40) { delay(500); attempts++; }
  if (WiFi.status() == WL_CONNECTED) setSuccess("connected!");
  else                               setError("failed to connect");
}

void cmdDisconnect() { WiFi.disconnect(true); setSuccess("disconnected!"); }

void cmdConfigure() {
  WiFi.disconnect(true); startAP();
  setSuccess("Join CALCGPT wifi then 192.168.4.1");
}

void cmdClearChat() {
  historyCount = 0;
  LOCK(); fullResponse = ""; PAGE_PAGE = 0; UNLOCK();
  setSuccess("chat cleared");
}

// ---------------------------------------------------------------------------
// Pure page renderer. Snapshots fullResponse + PAGE_PAGE under the lock,
// then writes a formatted result through publishResult(). Safe to call from
// either the API task (after building fullResponse) or the calc thread.
// ---------------------------------------------------------------------------
void renderPage() {
  const int COLS       = 16;
  const int ROWS       = 7;
  const int PAGE_CHARS = COLS * ROWS;

  String localResp;
  int    localPage;
  LOCK();
  localResp = fullResponse;
  localPage = PAGE_PAGE;
  UNLOCK();

  int total = localResp.length();
  if (total == 0) { setSuccess(""); return; }

  int start = localPage * PAGE_CHARS;
  if (start >= total) {
    localPage = (total - 1) / PAGE_CHARS;
    start = localPage * PAGE_CHARS;
    LOCK(); PAGE_PAGE = localPage; UNLOCK();
  }

  String chunk = localResp.substring(start, min(start + PAGE_CHARS, total));
  while ((int)chunk.length() < PAGE_CHARS) chunk += ' ';

  String formatted = "";
  for (int i = 0; i < (int)chunk.length(); i++) {
    formatted += chunk[i];
    if ((i + 1) % COLS == 0 && i + 1 < (int)chunk.length()) formatted += '\n';
  }

  publishResult(false, formatted.c_str());
  Serial.println("PAGE: " + formatted);
}

// ---------------------------------------------------------------------------
// Async commands. These run in a FreeRTOS task. They MUST end by either
// publishResult() (via setError/setSuccess) or renderPage().
// ---------------------------------------------------------------------------
static bool callOpenAI(const String& userMsg, String& outReply, String& outErr) {
  if (historyCount < MAX_HISTORY) {
    historyRoles[historyCount]    = "user";
    historyMessages[historyCount] = userMsg;
    historyCount++;
  }

  WiFiClientSecure client; client.setInsecure();
  HTTPClient http;
  http.begin(client, "https://api.openai.com/v1/chat/completions");
  http.addHeader("Content-Type", "application/json");
  http.addHeader("Authorization", "Bearer " + savedAPIKey);
  http.setTimeout(15000);

  DynamicJsonDocument doc(4096);
  doc["model"]      = "gpt-3.5-turbo";
  doc["max_tokens"] = 300;
  JsonArray messages = doc.createNestedArray("messages");
  JsonObject sys = messages.createNestedObject();
  sys["role"] = "system";
  sys["content"] = "You are a helpful assistant on a TI-84 calculator. Be concise.";
  for (int i = 0; i < historyCount; i++) {
    JsonObject msg = messages.createNestedObject();
    msg["role"]    = historyRoles[i];
    msg["content"] = historyMessages[i];
  }

  String body; serializeJson(doc, body);
  int code = http.POST(body);

  if (code == 200) {
    String resp = http.getString();
    DynamicJsonDocument rdoc(4096);
    deserializeJson(rdoc, resp);
    outReply = rdoc["choices"][0]["message"]["content"].as<String>();
    outReply.trim();
    if (historyCount < MAX_HISTORY) {
      historyRoles[historyCount]    = "assistant";
      historyMessages[historyCount] = outReply;
      historyCount++;
    }
    http.end();
    return true;
  }
  outErr = "http err " + String(code);
  http.end();
  return false;
}

void cmdGPT() {
  // Note: previously this reset historyCount=0, which made cmdGPT really
  // "start a new chat". We keep that behavior for backwards compat but do
  // it explicitly + clear the response buffer atomically.
  historyCount = 0;
  LOCK(); fullResponse = ""; PAGE_PAGE = 0; UNLOCK();

  String reply, err;
  if (!callOpenAI(String(strArgs[0]), reply, err)) { setError(err.c_str()); return; }

  LOCK(); fullResponse = reply; PAGE_PAGE = 0; UNLOCK();
  renderPage();
}

void cmdReply() {
  String reply, err;
  if (!callOpenAI(String(strArgs[0]), reply, err)) { setError(err.c_str()); return; }
  LOCK(); fullResponse = reply; PAGE_PAGE = 0; UNLOCK();
  renderPage();
}

void cmdTelegramSend() {
  if (savedTelegramToken == "" || savedTelegramChatID == "") {
    setError("no telegram credentials"); return;
  }
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http; http.setTimeout(10000);
  String url = "https://api.telegram.org/bot" + savedTelegramToken +
               "/sendMessage?chat_id=" + savedTelegramChatID +
               "&text=" + urlEncode(String(strArgs[0]));
  http.begin(client, url);
  int code = http.GET();
  http.end();
  if (code == 200) setSuccess("message sent!");
  else             setError(("tg err " + String(code)).c_str());
}

void cmdTelegramRecv() {
  if (savedTelegramToken == "") { setError("no telegram token"); return; }
  WiFiClientSecure client; client.setInsecure();
  HTTPClient http; http.setTimeout(10000);
  String url = "https://api.telegram.org/bot" + savedTelegramToken +
               "/getUpdates?limit=1&offset=-1";
  http.begin(client, url);
  int code = http.GET();
  if (code != 200) { http.end(); setError(("tg err " + String(code)).c_str()); return; }
  String resp = http.getString();
  http.end();
  DynamicJsonDocument doc(4096);
  deserializeJson(doc, resp);
  String text = doc["result"][0]["message"]["text"].as<String>();
  String from = doc["result"][0]["message"]["from"]["first_name"].as<String>();
  setSuccess((from + ": " + text).c_str());
}

// ---------------------------------------------------------------------------
// Launcher transfer (sync, queued so it runs outside the CBL ISR/handler)
// ---------------------------------------------------------------------------
void (*queued_action)() = NULL;

int sendProgramVariable(const char* name, uint8_t* program, size_t variableSize) {
  Serial.print("transferring: "); Serial.println(name);
  uint8_t msg_header[4] = {COMP83P, RTS, 13, 0};
  uint8_t rtsdata[13]; memset(rtsdata, 0, 13);
  rtsdata[0] = variableSize & 0xff;
  rtsdata[1] = variableSize >> 8;
  rtsdata[2] = VarTypes82::VarProgram;
  int nameSize = strlen(name);
  memcpy(&rtsdata[3], name, min(nameSize, 8));
  int dataLength = 0;
  if (cbl.send(msg_header, rtsdata, 13)) return 1;
  cbl.resetLines();
  if (cbl.get(msg_header, NULL, &dataLength, 0) || msg_header[1] != ACK) return 1;
  if (cbl.get(msg_header, NULL, &dataLength, 0) || msg_header[1] != CTS) return 1;
  msg_header[1] = ACK; msg_header[2] = 0; msg_header[3] = 0;
  if (cbl.send(msg_header, NULL, 0)) return 1;
  msg_header[1] = DATA;
  msg_header[2] = variableSize & 0xff;
  msg_header[3] = variableSize >> 8;
  if (cbl.send(msg_header, program, variableSize)) return 1;
  if (cbl.get(msg_header, NULL, &dataLength, 0) || msg_header[1] != ACK) return 1;
  msg_header[1] = EOT; msg_header[2] = 0; msg_header[3] = 0;
  if (cbl.send(msg_header, NULL, 0)) return 1;
  Serial.println("transfer complete");
  return 0;
}

void _sendLauncher() {
  sendProgramVariable("CALCGPT", launcher_program, launcher_program_len);
}

void cmdLauncher() {
  queued_action = _sendLauncher;
  setSuccess("queued transfer");
}

// ---------------------------------------------------------------------------
// API task
// ---------------------------------------------------------------------------
void apiTaskFunc(void* param) {
  int cmd = pendingCommand;
  Serial.print("Running async command: "); Serial.println(cmd);
  switch (cmd) {
    case 0: cmdConnect();      break;
    case 2: cmdGPT();          break;
    case 3: cmdReply();        break;
    case 7: cmdTelegramSend(); break;
    case 8: cmdTelegramRecv(); break;
  }
  // Make absolutely sure something published a result. If the command path
  // somehow forgot, surface that as an error rather than leaving the calc
  // hanging on S=0.
  LOCK();
  bool ready = replyReady;
  UNLOCK();
  if (!ready) publishResult(true, "no response");

  apiTask = NULL;
  vTaskDelete(NULL);
}

// ---------------------------------------------------------------------------
// CBL2 callbacks - called from cbl.eventLoopTick() on the main core
// ---------------------------------------------------------------------------
int onReceived(uint8_t type, enum Endpoint model, int datalen) {
  char varName = header[3];

  // Command write: C
  if (varName == 'C') {
    if (type != VarTypes82::VarReal) return -1;
    int cmd = TIVar::realToLong8x(data, model);
    if (cmd < 0 || cmd > MAXCOMMAND) return -1;

    // Reject if a task is already running. The calc should poll S first.
    LOCK();
    if (runState == STATE_RUNNING) { UNLOCK(); return -1; }
    // Reset args + flip to PENDING so onRequest reports S=0 immediately.
    command    = cmd;
    currentArg = 0;
    for (int i = 0; i < MAXARGS; ++i) {
      memset(strArgs[i], 0, MAXSTRARGLEN);
      realArgs[i] = 0;
    }
    replyReady = false;
    replyError = false;
    strncpy(message, "loading...", MAXSTRARGLEN);
    runState = STATE_PENDING;
    UNLOCK();
    return 0;
  }

  // Page request: V (jump to page N of the existing response)
  if (varName == 'V') {
    if (type != VarTypes82::VarReal) return -1;
    LOCK();
    // Don't clobber an in-flight response.
    if (runState == STATE_RUNNING) { UNLOCK(); return -1; }
    PAGE_PAGE = TIVar::realToLong8x(data, model);
    UNLOCK();
    renderPage();
    return 0;
  }

  // Clear chat shortcut: X
  if (varName == 'X') {
    historyCount = 0;
    LOCK(); fullResponse = ""; PAGE_PAGE = 0; UNLOCK();
    return 0;
  }

  // Otherwise: an argument for the pending command.
  LOCK();
  if (runState != STATE_PENDING || currentArg >= MAXARGS) { UNLOCK(); return -1; }
  switch (type) {
    case VarTypes82::VarString:
      strncpy(strArgs[currentArg],
              TIVar::strVarToString8x(data, model).c_str(), MAXSTRARGLEN);
      fixStrVar(strArgs[currentArg]);
      currentArg++;
      break;
    case VarTypes82::VarReal:
      realArgs[currentArg++] = TIVar::realToFloat8x(data, model);
      break;
    default:
      UNLOCK(); return -1;
  }
  UNLOCK();
  return 0;
}

int onRequest(uint8_t type, enum Endpoint model, int* headerlen,
              int* datalen, data_callback* data_callback) {
  char varName  = header[3];
  char strIndex = header[4];
  memset(header, 0, sizeof(header));

  switch (varName) {
    case 0xAA: {
      if (type != VarTypes82::VarString) return -1;
      LOCK();
      String snapshot = String(message);
      UNLOCK();
      *datalen = TIVar::stringToStrVar8x(snapshot, data, model);
      TIVar::intToSizeWord(*datalen, header);
      header[2] = VarTypes82::VarString;
      header[3] = 0xAA;
      header[4] = strIndex;
      *headerlen = 13;
      break;
    }
    case 'E': {
      if (type != VarTypes82::VarReal) return -1;
      LOCK(); long e = replyError ? 1 : 0; UNLOCK();
      *datalen = TIVar::longToReal8x(e, data, model);
      TIVar::intToSizeWord(*datalen, header);
      header[2] = VarTypes82::VarReal;
      header[3] = 'E';
      header[4] = '\0';
      *headerlen = 13;
      break;
    }
    case 'S': {
      if (type != VarTypes82::VarReal) return -1;
      LOCK(); long s = replyReady ? 1 : 0; UNLOCK();
      *datalen = TIVar::longToReal8x(s, data, model);
      TIVar::intToSizeWord(*datalen, header);
      header[2] = VarTypes82::VarReal;
      header[3] = 'S';
      header[4] = '\0';
      *headerlen = 13;
      break;
    }
    default: return -1;
  }
  return 0;
}

// ---------------------------------------------------------------------------
// setup / loop
// ---------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("[CALCGPT]");

  stateMutex = xSemaphoreCreateMutex();

  prefs.begin("calcgpt", true);
  savedSSID           = prefs.getString("ssid",    "");
  savedPass           = prefs.getString("pass",    "");
  savedAPIKey         = prefs.getString("apikey",  "");
  savedTelegramToken  = prefs.getString("tgtoken", "");
  savedTelegramChatID = prefs.getString("tgchat",  "");
  prefs.end();

  if (savedSSID == "") startAP();

  cbl.setLines(TIP, RING);
  cbl.resetLines();
  cbl.setupCallbacks(header, data, MAXDATALEN, onReceived, onRequest);
  pinMode(TIP,  INPUT);
  pinMode(RING, INPUT);

  strncpy(message, "CALCGPT ready", MAXSTRARGLEN);
  // Initial state: nothing has been requested. replyReady=false so the calc
  // does not mistake the boot banner for a response from a command it sent.
  replyReady = false;
  replyError = false;
  runState   = STATE_IDLE;

  memset(data,   0, MAXDATALEN);
  memset(header, 0, MAXHDRLEN);

  Serial.println("[ready]");
}

void loop() {
  server.handleClient();

  if (queued_action) {
    delay(1000);
    void (*tmp)() = queued_action;
    queued_action = NULL;
    tmp();
  }

  // Dispatch a pending command once all expected args have arrived.
  LOCK();
  bool dispatchSync  = false;
  bool dispatchAsync = false;
  int  dispatchCmd   = -1;
  int  dispatchIdx   = -1;
  if (runState == STATE_PENDING && command >= 0 && command <= MAXCOMMAND) {
    for (int i = 0; i < NUMCOMMANDS; ++i) {
      if (commands[i].id == command && commands[i].num_args == currentArg) {
        dispatchCmd = command;
        dispatchIdx = i;
        if (commands[i].wifi && !WiFi.isConnected()) {
          // Will publish the error after we drop the lock.
          dispatchSync = false; dispatchAsync = false;
          runState = STATE_RUNNING;
          UNLOCK();
          setError("wifi not connected");
          return;
        }
        if (commands[i].async) {
          if (apiTask != NULL) { UNLOCK(); return; } // shouldn't happen
          pendingCommand = command;
          runState = STATE_RUNNING;
          dispatchAsync = true;
        } else {
          runState = STATE_RUNNING;
          dispatchSync = true;
        }
        command = -1;
        break;
      }
    }
  }
  UNLOCK();

  if (dispatchAsync) {
    xTaskCreate(apiTaskFunc, "api", 16384, NULL, 1, &apiTask);
  } else if (dispatchSync) {
    commands[dispatchIdx].fp();   // will call publishResult itself
  }

  cbl.eventLoopTick();
}

/* =====================================================================
   IDPS SMART VENDING - ESP32-CAM
   - Joins WiFi: kazuha / kaedeharakazuha
   - Publishes its own IP to Firebase:
       /vendingMachine/machines/Machine 1/state/camera/ip

   - Serves:
       GET /capture?stage=before|after|security&reason=...

   - Captures JPEG
   - Saves JPEG into microSD card
   - Returns JSON:
       {"image_url":"http://CAM_IP/photo/xxx.jpg","stage":"after"}

   - No ImgBB
   - No Firebase image upload
   - Main ESP32 code does not need change

   Board: AI Thinker ESP32-CAM
   ===================================================================== */

#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include "FS.h"
#include "SD_MMC.h"
#include "mbedtls/base64.h"
#include "esp_task_wdt.h"

// ImgBB: photos try to upload here first; if it fails, we fall back to serving
// the photo from this CAM's SD card over the local network.
const char *IMGBB_API_KEY = "bb26a08e59f81bab87b8b493f6c1a21f";

// ===================== WiFi =====================
// MUST match the main ESP32's WiFi — the main board reaches this CAM by local IP,
// so both have to be on the same network.
const char *ssid     = "idp123";
const char *password = "idptest123";

// ===================== Firebase =====================
const char *DATABASE_URL = "https://idp-vending-machine-default-rtdb.asia-southeast1.firebasedatabase.app";

// Machine 1 must be URL encoded as Machine%201
String root   = "/vendingMachine/machines/Machine%201";
String pState = root + "/state";

// ===================== ESP32-CAM AI Thinker Pins =====================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// Onboard flash LED
#define FLASH_LED_PIN      4

// ===================== Server =====================
WebServer server(80);
#define WDT_TIMEOUT_S 50   // watchdog timeout (an ImgBB upload alone can take ~20s)

bool sdReady = false;

// =====================================================================
// FIREBASE HELPERS
// =====================================================================
String fbURL(String path) {
  return String(DATABASE_URL) + path + ".json";
}

void fbPutStr(String path, String value) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(fbURL(path));
  http.addHeader("Content-Type", "application/json");

  String payload = "\"" + value + "\"";
  int code = http.PUT(payload);

  Serial.print("Firebase PUT ");
  Serial.print(path);
  Serial.print(" code: ");
  Serial.println(code);

  http.end();
}

void publishCamIP() {
  String ip = WiFi.localIP().toString();
  fbPutStr(pState + "/camera/ip", ip);

  Serial.print("CAM IP published: ");
  Serial.println(ip);
}

// =====================================================================
// WIFI
// =====================================================================
void connectWiFi() {
  WiFi.begin(ssid, password);

  Serial.print("Connecting WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println();
  Serial.print("ESP32-CAM IP: ");
  Serial.println(WiFi.localIP());
}

void checkWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    connectWiFi();
    publishCamIP();
  }
}

// =====================================================================
// CAMERA INIT
// =====================================================================
bool initCamera() {
  camera_config_t config;

  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;

  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;

  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;

  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;

  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode    = CAMERA_GRAB_LATEST;   // always return the freshest frame

  if (psramFound()) {
    config.frame_size   = FRAMESIZE_SVGA;   // 800x600
    config.jpeg_quality = 12;
    config.fb_count     = 2;
  } else {
    config.frame_size   = FRAMESIZE_VGA;   // 640x480
    config.jpeg_quality = 14;
    config.fb_count     = 1;
  }

  esp_err_t err = esp_camera_init(&config);

  if (err != ESP_OK) {
    Serial.print("Camera init failed. Error: 0x");
    Serial.println(err, HEX);
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();

  // Optional image adjustment
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);

  Serial.println("Camera ready");
  return true;
}

// =====================================================================
// SD CARD INIT
// =====================================================================
bool initSDCard() {
  /*
     true = 1-bit SD mode.
     This is recommended for ESP32-CAM because GPIO4 is also the flash LED.
  */
  if (!SD_MMC.begin("/sdcard", true)) {
    Serial.println("SD Card Mount Failed");
    return false;
  }

  uint8_t cardType = SD_MMC.cardType();

  if (cardType == CARD_NONE) {
    Serial.println("No SD card inserted");
    return false;
  }

  Serial.println("SD Card ready");

  uint64_t cardSize = SD_MMC.cardSize() / (1024 * 1024);
  Serial.print("SD Card Size: ");
  Serial.print(cardSize);
  Serial.println(" MB");

  return true;
}

// =====================================================================
// SAFE FILE NAME
// =====================================================================
String cleanText(String text) {
  text.replace(" ", "_");
  text.replace("/", "_");
  text.replace("\\", "_");
  text.replace(":", "_");
  text.replace(";", "_");
  text.replace(",", "_");
  text.replace(".", "_");
  text.replace("@", "_");
  text.replace("?", "_");
  text.replace("&", "_");
  text.replace("=", "_");
  text.replace("%", "_");
  text.replace("#", "_");

  if (text.length() == 0) text = "unknown";
  return text;
}

// =====================================================================
// =====================================================================
// IMGBB UPLOAD (base64 form POST). Returns public URL or "" on failure.
// =====================================================================
String urlEncode(const char *src, size_t len) {
  static const char hex[] = "0123456789ABCDEF";
  String out; out.reserve(len + len / 2);
  for (size_t i = 0; i < len; i++) {
    char c = src[i];
    if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
        c == '-' || c == '_' || c == '.' || c == '~') out += c;
    else { out += '%'; out += hex[(c >> 4) & 0xF]; out += hex[c & 0xF]; }
  }
  return out;
}

String uploadToImgBB(camera_fb_t *fb) {
  if (fb == NULL) return "";
  // base64-encode the JPEG
  size_t b64Len = 0;
  mbedtls_base64_encode(NULL, 0, &b64Len, fb->buf, fb->len);
  if (b64Len == 0) return "";
  unsigned char *b64 = (unsigned char *)malloc(b64Len + 1);
  if (!b64) { Serial.println("ImgBB: malloc failed (image too big)"); return ""; }
  size_t written = 0;
  if (mbedtls_base64_encode(b64, b64Len + 1, &written, fb->buf, fb->len) != 0) { free(b64); return ""; }
  b64[written] = '\0';
  String image = urlEncode((const char *)b64, written);
  free(b64);

  WiFiClientSecure client; client.setInsecure();
  HTTPClient http; http.setTimeout(20000);
  String apiUrl = String("https://api.imgbb.com/1/upload?key=") + IMGBB_API_KEY;
  if (!http.begin(client, apiUrl)) return "";
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int code = http.POST("image=" + image);
  String resp = (code > 0) ? http.getString() : "";
  http.end();
  if (code != 200) { Serial.printf("ImgBB HTTP %d\n", code); return ""; }

  // crude JSON extract of "url":"..."
  int k = resp.indexOf("\"url\":\"");
  if (k == -1) return "";
  k += 7;
  int e = resp.indexOf("\"", k);
  if (e == -1) return "";
  String url = resp.substring(k, e);
  url.replace("\\/", "/");
  Serial.println("ImgBB OK: " + url);
  return url;
}

// =====================================================================
// SAVE PHOTO TO ESP32-CAM SD SERVER
// =====================================================================
String savePhotoToCamServer(camera_fb_t *fb, String stage, String reason) {
  if (!sdReady || fb == NULL) {
    Serial.println("SD not ready or camera frame empty");
    return "";
  }

  stage  = cleanText(stage);
  reason = cleanText(reason);

  String filename = "/" + stage + "_" + reason + "_" + String(millis()) + ".jpg";

  File file = SD_MMC.open(filename, FILE_WRITE);

  if (!file) {
    Serial.println("Failed to open file on SD");
    return "";
  }

  size_t written = file.write(fb->buf, fb->len);
  file.close();

  if (written != fb->len) {
    Serial.println("File write incomplete");
    return "";
  }

  String imageUrl = "http://" + WiFi.localIP().toString() + "/photo" + filename;

  Serial.print("Saved photo: ");
  Serial.println(filename);

  Serial.print("Image URL: ");
  Serial.println(imageUrl);

  return imageUrl;
}

// Add CORS headers so the admin web page can call the CAM from any origin.
void addCORSHeaders() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Access-Control-Allow-Methods", "GET, OPTIONS");
  server.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}
// Browsers send OPTIONS pre-flight before cross-origin GETs
void handleOptions() { addCORSHeaders(); server.send(204); }

// =====================================================================
// CAPTURE ENDPOINT
// =====================================================================
void handleCapture() {
  String stage  = server.hasArg("stage")  ? server.arg("stage")  : "unknown";
  String reason = server.hasArg("reason") ? server.arg("reason") : "manual";
  String uid    = server.hasArg("uid")    ? server.arg("uid")    : "";
  String email  = server.hasArg("email")  ? server.arg("email")  : "";

  Serial.println();
  Serial.println("========== CAPTURE REQUEST ==========");
  Serial.println("Stage : " + stage);
  Serial.println("Reason: " + reason);
  Serial.println("UID   : " + uid);
  Serial.println("Email : " + email);

  if (!sdReady) {
    Serial.println("SD not ready — will rely on ImgBB only for this capture.");
  }

  // Turn on flash before capture
  digitalWrite(FLASH_LED_PIN, HIGH);
  delay(400);  // let flash + auto-exposure stabilise

  // Flush stale buffered frames so the photo we KEEP is the current scene.
  // With CAMERA_GRAB_LATEST + fb_count=2, draining 6 frames guarantees
  // the sensor has cycled fully under the new flash brightness.
  for (int f = 0; f < 6; f++) {
    camera_fb_t *tmp = esp_camera_fb_get();
    if (tmp) esp_camera_fb_return(tmp);
    delay(60);
  }
  delay(80);   // one extra settle after the last flush before the final grab
  camera_fb_t *fb = esp_camera_fb_get();   // the fresh frame we keep

  // Turn off flash after capture
  digitalWrite(FLASH_LED_PIN, LOW);

  if (!fb) {
    Serial.println("Camera capture failed");
    server.send(500, "application/json",
                "{\"image_url\":\"\",\"error\":\"camera_capture_failed\"}");
    return;
  }

  // Storage preference from the caller (?store=both|imgbb|sd). Admin sets this in
  // Firebase; the main ESP32 forwards it. Default "both" keeps old behaviour.
  // Doing only the chosen path saves time (ImgBB upload alone is ~1-3s).
  String store = server.hasArg("store") ? server.arg("store") : "both";
  // DIAGNOSTIC: if this prints "both" (or the arg is missing) when admin chose "sd",
  // the MAIN ESP32 is on OLD firmware that doesn't forward &store=. Re-flash it.
  Serial.println("Store : " + store + (server.hasArg("store") ? "" : "  (no store arg sent!)"));
  bool wantSD    = (store == "sd"    || store == "both");
  bool wantImgBB = (store == "imgbb" || store == "both");

  String sdUrl = "", imgbbUrl = "";
  if (wantSD)    sdUrl    = savePhotoToCamServer(fb, stage, reason);
  esp_task_wdt_reset();   // the ImgBB upload below can take ~20s; pet the watchdog first
  if (wantImgBB) imgbbUrl = uploadToImgBB(fb);
  // DIAGNOSTIC: if store="sd" but SD url is "(none)", the SD card write failed
  // (card not seated / not FAT32) and the safety-net fell back to ImgBB.
  Serial.println("SD url   : " + (sdUrl.length()    ? sdUrl    : String("(none)")));
  Serial.println("ImgBB url: " + (imgbbUrl.length() ? imgbbUrl : String("(none)")));

  // Safety net so Camera Logs never breaks: if the chosen storage produced no
  // URL, fall back to the other one before giving up.
  if (sdUrl == "" && imgbbUrl == "") {
    if (!wantSD)         sdUrl    = savePhotoToCamServer(fb, stage, reason);
    else if (!wantImgBB) imgbbUrl = uploadToImgBB(fb);
  }

  esp_camera_fb_return(fb);

  // Prefer the storage the admin asked for; fall back to whatever we actually got.
  String imageUrl = (store == "sd") ? (sdUrl   != "" ? sdUrl   : imgbbUrl)
                                    : (imgbbUrl != "" ? imgbbUrl : sdUrl);

  if (imageUrl == "") {
    server.send(500, "application/json",
                "{\"image_url\":\"\",\"error\":\"upload_and_sd_failed\"}");
    return;
  }

  String json = "{";
  json += "\"image_url\":\"" + imageUrl + "\",";
  json += "\"imgbb\":\"" + imgbbUrl + "\",";
  json += "\"sd\":\"" + sdUrl + "\",";
  json += "\"stage\":\"" + stage + "\",";
  json += "\"reason\":\"" + reason + "\"";
  json += "}";

  addCORSHeaders();
  server.send(200, "application/json", json);

  Serial.println("Capture done");
  Serial.println("=====================================");
}

// =====================================================================
// DELETE PHOTO FROM SD (admin-triggered only)
// =====================================================================
void handleDeletePhoto() {
  if (!sdReady) { server.send(500, "text/plain", "sd_not_ready"); return; }
  if (!server.hasArg("file")) { server.send(400, "text/plain", "missing file"); return; }
  String path = server.arg("file");
  if (!path.startsWith("/")) path = "/" + path;
  if (SD_MMC.exists(path.c_str())) {
    SD_MMC.remove(path.c_str());
    Serial.println("Deleted from SD: " + path);
    server.send(200, "application/json", "{\"deleted\":true}");
  } else {
    server.send(404, "application/json", "{\"deleted\":false,\"error\":\"not_found\"}");
  }
}

// =====================================================================
// HOME PAGE
// =====================================================================
void handleRoot() {
  String html = "";
  html += "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<title>IDPS ESP32-CAM</title>";
  html += "</head><body>";
  html += "<h2>IDPS ESP32-CAM Server</h2>";
  html += "<p>Camera IP: ";
  html += WiFi.localIP().toString();
  html += "</p>";
  html += "<p>Use:</p>";
  html += "<pre>/capture?stage=before&reason=test</pre>";
  html += "<p>Photos are served from:</p>";
  html += "<pre>/photo/filename.jpg</pre>";
  html += "</body></html>";

  server.send(200, "text/html", html);
}

// =====================================================================
// TEST CAPTURE PAGE
// =====================================================================
void handleTestCapture() {
  String url = "/capture?stage=test&reason=manual";
  server.sendHeader("Location", url);
  server.send(302, "text/plain", "");
}

// =====================================================================
// SETUP
// =====================================================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println();
  Serial.println("=====================================");
  Serial.println("IDPS ESP32-CAM - SD Local Server Mode");
  Serial.println("No ImgBB upload");
  Serial.println("=====================================");

  pinMode(FLASH_LED_PIN, OUTPUT);
  digitalWrite(FLASH_LED_PIN, LOW);

  if (!initCamera()) {
    Serial.println("Camera failed. Restarting in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  sdReady = initSDCard();

  connectWiFi();
  publishCamIP();

  // Web routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/capture", HTTP_GET, handleCapture);
  server.on("/capture", HTTP_OPTIONS, handleOptions);   // CORS pre-flight
  server.on("/delete", HTTP_GET, handleDeletePhoto);
  server.on("/test", HTTP_GET, handleTestCapture);

  /*
     This makes photos accessible like:
     http://CAM_IP/photo/before_purchase_start_12345.jpg

     A file saved as:
     /before_purchase_start_12345.jpg

     becomes:
     /photo/before_purchase_start_12345.jpg
  */
  server.serveStatic("/photo", SD_MMC, "/");

  server.begin();

  Serial.println("ESP32-CAM server started");
  Serial.print("Open: http://");
  Serial.println(WiFi.localIP());

  // Hardware watchdog — generous timeout because an ImgBB upload can block ~20s.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_init(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);
}

// =====================================================================
// LOOP
// =====================================================================
void loop() {
  esp_task_wdt_reset();
  checkWiFi();
  server.handleClient();
}
/* =====================================================================
   IDPS SMART VENDING - ESP32 MAIN CONTROLLER
   Firebase structure: memory-doc style
   /vendingMachine/machines/Machine 1/{state,control,user,inventory,
                                       camera,weight,checkout,admin,
                                       security,log}
   Hardware: 3x HX711 load cells, door sensor, solenoid relay,
             buzzer, alarm LED, SSD1306 OLED.
   WiFi: kazuha / kaedeharakazuha
   CAM IP is auto-discovered from /state/camera/ip (CAM publishes it).
   ===================================================================== */

#include <WiFi.h>
#include <HTTPClient.h>
#include "HX711.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <limits.h>
#include "esp_task_wdt.h"

// ============== 1. WIFI ==============
const char* WIFI_SSID     = "idp123";
const char* WIFI_PASSWORD = "idptest123";

// ============== 2. FIREBASE ==============
const char* DATABASE_URL = "https://idp-vending-machine-default-rtdb.asia-southeast1.firebasedatabase.app";

// "Machine 1" -> URL encoded "Machine%201"
String root      = "/vendingMachine/machines/Machine%201";
String pState    = root + "/state";
String pControl  = root + "/control";
String pUser     = root + "/user";
String pInv      = root + "/inventory";
String pCamera   = root + "/camera";
String pWeight   = root + "/weight";
String pCheckout = root + "/checkout";
String pAdmin    = root + "/admin";
String pSecurity = root + "/security";
String pLog      = root + "/log";

// ============== 3. OLED ==============
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ============== 4. HX711 PINS ==============
#define HX1_DT 32
#define HX1_SCK 4
#define HX2_DT 33
#define HX2_SCK 25
#define HX3_DT 26
#define HX3_SCK 27
HX711 scale[3];
const int HX_DT[3]  = {HX1_DT, HX2_DT, HX3_DT};
const int HX_SCK[3] = {HX1_SCK, HX2_SCK, HX3_SCK};

// ============== 5. DOOR / OUTPUTS ==============
#define DOOR_SENSOR_PIN 13     // INPUT_PULLUP : LOW = closed(0), HIGH = open(1)
#define DOOR_LOCK_PIN   14     // solenoid relay
#define BUZZER_PIN      23
#define ALARM_LED_PIN   19

const unsigned long SOLENOID_UNLOCK_DURATION = 5000;  // ms coil energized (heat-safe). Door opens within this window.
const unsigned long NO_OPEN_REMINDER_MS = 10000;      // beep reminder if door not opened by 10s
const unsigned long NO_OPEN_CHECKOUT_MS = 20000;      // auto-checkout if still not opened by 20s

// ============== 6. SLOT DATA (loaded from Firebase, admin-set) ==============
struct Slot {
  String name;
  float  price;        // RM
  int    qty;          // current qty (ESP32 calculated)
  int    maxQty;
  long   tare;         // empty tray raw value
  long   unitRaw;      // raw value of exactly ONE item
  long   noise;        // measured peak-to-peak noise of empty tray (for light items)
  String calStatus;    // not_calibrated / calibrated
  int    en;           // enabled 1/0
  String status;       // empty / normal / need_calibration

  long   startRaw;     // raw at session start
  long   finalRaw;     // raw at session end
  int    takenQty;     // qty taken this session
};
Slot slot[3];

// Detection is relative to the lighter of: a fraction of one unit, OR the sensor noise
// floor measured at tare. This lets very light items (Milo, chips ~150-200g) calibrate
// and count reliably instead of being rejected by a fixed gram guess.
const float DETECT_MIN_RAW_FRACTION = 0.40; // >40% of one unit counts as "something present"
const float VALID_TOLERANCE_FRACTION = 0.20; // +/-20% of one unit when matching integer count
const long  ABS_MIN_UNIT_RAW = 300;          // absolute floor: a real item must move raw by >300
const float NOISE_MARGIN = 4.0;               // an item must exceed noise * this to register

// ============== 7. SESSION STATE ==============
int   lastDoorVal = -1;
bool  sessionActive = false;        // unlock granted, purchase in progress
bool  doorWasOpened = false;
bool  checkoutInProgress = false;
bool  noDoorOpenReminder = false;

bool  solenoidActive = false;
unsigned long solenoidStart = 0;
unsigned long sessionUnlockTime = 0;   // when the purchase session unlocked (for no-open timing)
int   lastUnlockRead = 0;

String activeSession = "";
String activeUid = "";
String activeEmail = "";

bool  trayWarn = false;            // case 2: invalid weight buzzer
bool  trayWarnPhoto = false;
bool  wrongActive = false;         // a wrong-placement condition is currently active
unsigned long lastWrongBeep = 0;   // last 5-beep burst time (every 10s while wrong)
bool  doorOpenAlarmActive = false; // door left open > limit
unsigned long doorOpenStart = 0;
unsigned long lastDoorOpenBeep = 0;
const unsigned long DOOR_OPEN_LIMIT = 300000;   // 5 min
const unsigned long DOOR_OPEN_BEEP_INTERVAL = 5000; // every 5s
bool  unauthOpen = false;          // case 1: unauthorized door buzzer
bool  unauthPhoto = false;
bool  restockActive = false;       // admin restock mode: alarms suspended
unsigned long restockStart = 0;
int   adminUnlockZeroCount = 0;    // debounce for ending restock
const unsigned long RESTOCK_MAX_DURATION = 1800000; // 30 min safety only

String camIP = "";                 // auto-discovered ESP32-CAM IP
bool   cameraEnabled = true;       // cached from Firebase control/camera_enabled
String camStorage = "both";        // "both" | "imgbb" | "sd" — cached from control/camera_storage
String sessionBeforeUrl = "";      // THIS session's before-photo URL ("" if camera was off)

// ============== 8. TIMING ==============
unsigned long tFbRead = 0, tInv = 0, tDoor = 0, tFast = 0, tAdmin = 0, tHeart = 0;
const unsigned long FB_READ_INT   = 500;
const unsigned long INV_INT       = 1500;
const unsigned long DOOR_INT      = 80;
const unsigned long FAST_INT      = 200;
const unsigned long ADMIN_INT     = 700;
const unsigned long HEART_INT     = 10000;   // online-heartbeat write interval
#define WDT_TIMEOUT_S 60                      // watchdog: auto-reset if a loop hangs this long

// ============== PROTOTYPES ==============
int  getDoorVal();
void finishCheckout(String reason);
void handleUnauthorized(int v);
void fastTrayCheck();
void lockSolenoid();
String camRequest(String stage, String reason);

// =====================================================================
// FIREBASE HELPERS
// =====================================================================
String fbURL(String p){ return String(DATABASE_URL) + p + ".json"; }

String jesc(String v){
  v.replace("\\","\\\\"); v.replace("\"","\\\""); v.replace("\n","\\n"); v.replace("\r","\\r");
  return v;
}

String fbGetStr(String p){
  HTTPClient h; h.begin(fbURL(p)); int c=h.GET(); String r="";
  if(c==200){ r=h.getString(); r.trim();
    if(r=="null") r="";
    else if(r.startsWith("\"")&&r.endsWith("\"")) r=r.substring(1,r.length()-1);
    r.replace("\\/","/");
  }
  h.end(); return r;
}
long fbGetInt(String p){
  String s=fbGetStr(p); if(s=="") return 0; return s.toInt();
}
float fbGetFloat(String p){
  String s=fbGetStr(p); if(s=="") return 0.0; return s.toFloat();
}
void fbPutInt(String p,long v){
  HTTPClient h; h.begin(fbURL(p)); h.addHeader("Content-Type","application/json");
  h.PUT(String(v)); h.end();
}
void fbPutFloat(String p,float v){
  HTTPClient h; h.begin(fbURL(p)); h.addHeader("Content-Type","application/json");
  h.PUT(String(v,2)); h.end();
}
void fbPutStr(String p,String v){
  HTTPClient h; h.begin(fbURL(p)); h.addHeader("Content-Type","application/json");
  h.PUT("\""+jesc(v)+"\""); h.end();
}
void fbPostJson(String p,String j){
  HTTPClient h; h.begin(fbURL(p)); h.addHeader("Content-Type","application/json");
  h.POST(j); h.end();
}
String jval(String json,String key){
  String k="\""+key+"\":\""; int s=json.indexOf(k);
  if(s==-1) return ""; s+=k.length();
  int e=json.indexOf("\"",s); if(e==-1) return "";
  String v=json.substring(s,e); v.replace("\\/","/"); return v;
}

// ---- BATCHING HELPERS ----------------------------------------------------
// Fetch a whole subtree as raw JSON in ONE request (used to read /inventory,
// /control, /admin in a single GET instead of many field-by-field reads).
String fbGetRaw(String p){
  HTTPClient h; h.begin(fbURL(p)); h.setTimeout(4000); int c=h.GET(); String r="";
  if(c==200){ r=h.getString(); r.trim(); }
  h.end(); return r;
}
// Update many fields in ONE request. Firebase applies a PATCH atomically, and
// keys containing '/' are treated as deep paths (e.g. "slot/s1/qty").
void fbPatchJson(String p,String j){
  HTTPClient h; h.begin(fbURL(p)); h.addHeader("Content-Type","application/json");
  h.sendRequest("PATCH",j); h.end();
}
// PUT raw JSON (e.g. a server-timestamp object) without string-quoting it.
void fbPutRaw(String p,String j){
  HTTPClient h; h.begin(fbURL(p)); h.addHeader("Content-Type","application/json");
  h.PUT(j); h.end();
}
// Read one value from a FLAT JSON object. Handles "key":"str", "key":num,
// "key":true/false, "key":null. Returns "" if missing/null.
String jget(String json,String key){
  String k="\""+key+"\":";
  int s=json.indexOf(k); if(s==-1) return "";
  s+=k.length();
  while(s<(int)json.length() && json[s]==' ') s++;
  if(s>=(int)json.length()) return "";
  if(json[s]=='\"'){                       // quoted string value
    int e=json.indexOf('\"',s+1); if(e==-1) return "";
    String v=json.substring(s+1,e); v.replace("\\/","/"); return v;
  }
  int e=s;                                 // number / bool / null up to , or }
  while(e<(int)json.length() && json[e]!=',' && json[e]!='}') e++;
  String v=json.substring(s,e); v.trim();
  if(v=="null") return "";
  return v;
}
// Extract the {...} object substring for a key (e.g. "s1") from a parent dump.
String jobj(String json,String key){
  String k="\""+key+"\":{";
  int s=json.indexOf(k); if(s==-1) return "";
  s+=k.length()-1;                         // land on the opening '{'
  int depth=0;
  for(int i=s;i<(int)json.length();i++){
    char ch=json[i];
    if(ch=='{') depth++;
    else if(ch=='}'){ depth--; if(depth==0) return json.substring(s,i+1); }
  }
  return "";
}

// =====================================================================
// OLED
// =====================================================================
void oled(String l1,String l2=""){
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE); display.setTextSize(2);
  display.setCursor(0,10); display.println(l1);
  if(l2!=""){ display.setCursor(0,35); display.println(l2); }
  display.display();
}
// Show up to 3 item lines (e.g. "2x Milo" / "1x Cola") then the total.
void oledCheckoutMulti(float total){
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);
  int y=2; bool any=false;
  for(int i=0;i<3;i++){
    if(slot[i].takenQty>0){
      display.setCursor(0,y);
      display.print(slot[i].takenQty); display.print("x ");
      String nm=slot[i].name; if(nm.length()>14) nm=nm.substring(0,14);
      display.println(nm);
      y+=11; any=true;
    }
  }
  if(!any){ display.setCursor(0,2); display.println("No item taken"); }
  display.setTextSize(2); display.setCursor(0,46);
  display.print("RM "); display.print(total,2);
  display.display();
}
void oledCheckout(String name,int q,float total){
  display.clearDisplay(); display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1); display.setCursor(0,8);
  display.print(q); display.print(" x "); display.println(name);
  display.setTextSize(2); display.setCursor(0,34);
  display.print("RM "); display.print(total,2);
  display.display();
}

// =====================================================================
// BUZZER
// =====================================================================
void beep(){ digitalWrite(BUZZER_PIN,HIGH); delay(180); digitalWrite(BUZZER_PIN,LOW); delay(80); }
void beep4(){ for(int i=0;i<4;i++){ beep(); delay(120);} }

// =====================================================================
// HX711
// =====================================================================
long rawQuick(int i){ if(scale[i].is_ready()) return scale[i].read(); return 0; }
long rawAvg(int i,int n){
  // Faster: single reads with short waits instead of read_average(3)+delay(60).
  long t=0; int c=0;
  for(int k=0;k<n;k++){
    if(scale[i].is_ready()){ t+=scale[i].read(); c++; }
    delay(12);
  }
  if(c==0) return slot[i].tare; return t/c;
}
// Measure peak-to-peak noise of the (empty) tray.
long measureNoise(int i){
  long mn=LONG_MAX, mx=LONG_MIN;
  for(int k=0;k<10;k++){
    if(scale[i].is_ready()){ long r=scale[i].read(); if(r<mn)mn=r; if(r>mx)mx=r; }
    delay(12);
  }
  if(mn==LONG_MAX) return 0;
  long pp=mx-mn; if(pp<0)pp=0; return pp;
}
// net raw above the tare baseline
long netRaw(int i,long raw){ long n=raw-slot[i].tare; if(n<0) n=0; return n; }

// Smallest raw change that counts as "something is really there" for this slot.
// Uses the larger of (noise*margin) and a small fraction of one unit, but never
// below the absolute floor. This adapts to light items and to each load cell.
long detectFloor(int i){
  long noiseFloor = (long)(slot[i].noise * NOISE_MARGIN);
  long unitFloor  = (slot[i].unitRaw>0) ? (long)(slot[i].unitRaw*DETECT_MIN_RAW_FRACTION) : 0;
  long f = (noiseFloor>unitFloor) ? noiseFloor : unitFloor;
  if(f < ABS_MIN_UNIT_RAW/3) f = ABS_MIN_UNIT_RAW/3;  // tiny safety floor
  return f;
}

// current integer qty from a raw reading
int qtyFromRaw(int i,long raw){
  if(slot[i].unitRaw<=0) return 0;
  long net=netRaw(i,raw);
  if(net < detectFloor(i)) return 0;
  double exact = (double)net/(double)slot[i].unitRaw;
  int low = (int)floor(exact);
  double frac = exact - low;
  // Only round UP if we're clearly past the midpoint (frac > 0.55).
  // This stops sensor drift from turning 5 items (5.50-ish) into 6.
  int q = (frac > 0.55) ? (low+1) : low;
  if(q<0) q=0; if(q>slot[i].maxQty) q=slot[i].maxQty;
  return q;
}

// case2: weight on tray must be close to an integer multiple of unitRaw (0,1,2,...).
// Anything in between (a foreign object like a mouse, or a wrong product) => invalid => beep.
bool trayValid(int i,long raw,int *matched){
  if(slot[i].unitRaw<=0){ if(matched)*matched=0; return true; } // not calibrated -> don't nag
  long net=netRaw(i,raw);
  if(net < detectFloor(i)){ if(matched)*matched=0; return true; } // empty tray = fine

  // Find the nearest integer count and how far off we are.
  double exact = (double)net/(double)slot[i].unitRaw;
  long nearest = (long)llround(exact);
  if(nearest < 1) nearest = 1;

  // Above the max the tray can physically hold => something foreign is on it.
  if(nearest > slot[i].maxQty){ if(matched)*matched=-1; return false; }

  long target = (long)slot[i].unitRaw * nearest;
  long tol    = (long)(slot[i].unitRaw * VALID_TOLERANCE_FRACTION);
  if(labs(net-target) <= tol){ if(matched)*matched=(int)nearest; return true; }

  // net is clearly between two valid counts (e.g. 5 cans + a mouse) => invalid.
  if(matched)*matched=-1; return false;
}

// =====================================================================
// LOAD INVENTORY FROM FIREBASE (admin-set, dynamic)
// =====================================================================
void applySlotDefaults(int i){
  if(slot[i].maxQty<=0) slot[i].maxQty=5;
  if(slot[i].noise<=0) slot[i].noise=50;   // conservative default until first tare
  if(slot[i].calStatus=="") slot[i].calStatus="not_calibrated";
}
// Per-field load — kept as the fallback path if the batched read fails.
void loadSlot(int i){
  String b=pInv+"/s"+String(i+1);
  slot[i].name      = fbGetStr(b+"/name");
  slot[i].price     = fbGetFloat(b+"/price");
  slot[i].maxQty    = (int)fbGetInt(b+"/max");
  slot[i].tare      = fbGetInt(b+"/tare");
  slot[i].unitRaw   = fbGetInt(b+"/unit_raw");
  slot[i].noise     = fbGetInt(b+"/noise");
  slot[i].calStatus = fbGetStr(b+"/cal_status");
  slot[i].en        = (int)fbGetInt(b+"/en");
  slot[i].qty       = (int)fbGetInt(b+"/qty");
  slot[i].status    = fbGetStr(b+"/status");
  applySlotDefaults(i);
}
// Parse one slot's fields out of an already-fetched JSON object string.
void loadSlotFromObj(int i,String o){
  slot[i].name      = jget(o,"name");
  slot[i].price     = jget(o,"price").toFloat();
  slot[i].maxQty    = jget(o,"max").toInt();
  slot[i].tare      = jget(o,"tare").toInt();
  slot[i].unitRaw   = jget(o,"unit_raw").toInt();
  slot[i].noise     = jget(o,"noise").toInt();
  slot[i].calStatus = jget(o,"cal_status");
  slot[i].en        = jget(o,"en").toInt();
  slot[i].qty       = jget(o,"qty").toInt();
  slot[i].status    = jget(o,"status");
  applySlotDefaults(i);
}
// ONE GET of the whole /inventory subtree, parsed locally.
// Was 30 separate HTTP reads (10 fields x 3 slots) ~4s -> now ~0.3s.
void loadAllSlots(){
  String all = fbGetRaw(pInv);
  if(all!="" && all!="null" && all.indexOf("\"s1\"")!=-1){
    for(int i=0;i<3;i++){
      String o = jobj(all,"s"+String(i+1));
      if(o!="") loadSlotFromObj(i,o);
      else      loadSlot(i);            // per-slot fallback if one object is missing
    }
  } else {
    for(int i=0;i<3;i++) loadSlot(i);   // full fallback if the batched read failed
  }
}

void writeSlotRuntime(int i){
  String b=pInv+"/s"+String(i+1);
  fbPutInt(b+"/qty",slot[i].qty);
  fbPutStr(b+"/status",slot[i].status);
}

// =====================================================================
// WIFI
// =====================================================================
void connectWiFi(){
  WiFi.begin(WIFI_SSID,WIFI_PASSWORD);
  Serial.print("WiFi");
  while(WiFi.status()!=WL_CONNECTED){ Serial.print("."); delay(500); }
  Serial.println(); Serial.print("ESP32 IP: "); Serial.println(WiFi.localIP());
}
void checkWiFi(){ if(WiFi.status()!=WL_CONNECTED){ connectWiFi(); } }

// =====================================================================
// SOLENOID
// =====================================================================
void lockSolenoid(){ digitalWrite(DOOR_LOCK_PIN,LOW); solenoidActive=false; }
void unlockSolenoid(){ digitalWrite(DOOR_LOCK_PIN,HIGH); solenoidActive=true; solenoidStart=millis(); }

// =====================================================================
// DOOR
// =====================================================================
int getDoorVal(){ return (digitalRead(DOOR_SENSOR_PIN)==LOW)?0:1; }  // 0 closed, 1 open

// =====================================================================
// CAMERA (auto IP discovery + JSON response)
// =====================================================================
void refreshCamIP(){
  String ip=fbGetStr(pState+"/camera/ip");
  if(ip!="") camIP=ip;
}
String camRequest(String stage,String reason){
  if(!cameraEnabled){
    Serial.println("Camera disabled — skipping capture for: "+stage);
    return "";
  }
  if(camIP=="") refreshCamIP();
  if(camIP==""){
    Serial.println("CAM IP unknown");
    fbPutStr(pCamera+"/status","no_cam_ip");
    fbPutStr(pLog+"/error","camera offline: no IP in state/camera/ip");
    return "";
  }

  // tell DB what we are doing
  fbPutStr(pCamera+"/stage",stage);
  fbPutStr(pCamera+"/reason",reason);
  fbPutStr(pCamera+"/status","capturing");

  String url="http://"+camIP+"/capture?stage="+stage+"&reason="+reason
             +"&uid="+activeUid+"&email="+activeEmail
             +"&store="+camStorage;   // tell CAM which storage to use (saves time)
  Serial.println("CAM request: stage="+stage+" store="+camStorage);   // diagnostic
  // Short timeout: a missing/offline CAM must NOT stall the machine for ~5s.
  HTTPClient h;
  h.begin(url);

  // SD is faster, ImgBB needs longer upload time
  if (camStorage == "sd") {
  h.setTimeout(8000);
  } else if (camStorage == "imgbb") {
  h.setTimeout(30000);
  } else {
  h.setTimeout(35000);   // both = slowest
  }

  esp_task_wdt_reset();
  int c = h.GET();
  esp_task_wdt_reset();

  String imgUrl = "";
  
  if(c==200){
    String resp=h.getString();
    imgUrl=jval(resp,"image_url");
    if(imgUrl=="") fbPutStr(pLog+"/error","camera replied but no image_url (ImgBB fail?)");
  } else {
    Serial.print("CAM fail "); Serial.println(c);
    fbPutStr(pLog+"/error","camera HTTP fail code "+String(c)+" at "+camIP);
  }
  h.end();

  fbPutStr(pCamera+"/status","ready");
  fbPutStr(pCamera+"/last",imgUrl);
  if(stage=="before")        fbPutStr(pCamera+"/before_url",imgUrl);
  else if(stage=="after")    fbPutStr(pCamera+"/after_url",imgUrl);
  else                       fbPutStr(pCamera+"/security_url",imgUrl);

  // Append to cameraLogs so the admin "Camera Logs" view shows every photo taken.
  if(imgUrl!=""){
    String j="{";
    j+="\"imageUrl\":\""+jesc(imgUrl)+"\",";
    j+="\"stage\":\""+jesc(stage)+"\",";
    j+="\"reason\":\""+jesc(reason)+"\",";
    j+="\"userEmail\":\""+jesc(activeEmail)+"\",";
    j+="\"timestamp\":{\".sv\":\"timestamp\"}}";
    fbPostJson("/vendingMachine/cameraLogs",j);
  }
  return imgUrl;
}

// =====================================================================
// SECURITY ALERT
// =====================================================================
void pushSecurity(String reason,String imgUrl){
  long count=fbGetInt(pSecurity+"/count")+1;
  fbPutInt(pSecurity+"/alarm",1);
  fbPutStr(pSecurity+"/reason",reason);
  fbPutInt(pSecurity+"/count",count);

  // Human-readable message for the admin banner — always matches THIS event.
  String friendly = reason;
  if(reason=="unauthorized_door_open")        friendly="Unauthorized door open!";
  else if(reason=="item_moved_between_trays") friendly="Item moved to the wrong tray";
  else if(reason=="door_open_over_5min")      friendly="Door left open over 5 minutes";
  else if(reason=="invalid_tray_weight")      friendly="Wrong item / weight on tray";
  fbPutInt(pSecurity+"/notify_admin",1);
  fbPutStr(pSecurity+"/notify_reason",friendly);

  String j="{";
  j+="\"machine\":\"Machine 1\",";
  j+="\"reason\":\""+jesc(reason)+"\",";
  j+="\"uid\":\""+jesc(activeUid)+"\",";
  j+="\"email\":\""+jesc(activeEmail)+"\",";
  j+="\"image_url\":\""+jesc(imgUrl)+"\",";
  j+="\"timestamp_ms\":"+String(millis())+",";
  j+="\"timestamp\":{\".sv\":\"timestamp\"},";
  j+="\"resolved\":false}";
  fbPostJson("/vendingMachine/securityAlerts",j);
}

// =====================================================================
// TRAY WARNING (case 2)
// =====================================================================
void clearTrayWarn(){
  trayWarn=false; trayWarnPhoto=false;
  wrongActive=false; lastWrongBeep=0;
  digitalWrite(BUZZER_PIN,LOW); digitalWrite(ALARM_LED_PIN,LOW);
  fbPutInt(pState+"/buzzer",0);
}

// =====================================================================
// START UNLOCK SESSION (with payment gate)
// =====================================================================
void startSession(int affordable){
  Serial.println(">>> UNLOCK received from website — affordable="+String(affordable));
  if(checkoutInProgress) return;
  if(sessionActive){ Serial.println("already active"); return; }
  if(restockActive){
    // Admin is restocking: refuse the unlock and tell the website.
    Serial.println("Blocked: admin restocking");
    fbPutInt(pControl+"/unlock",0);
    lastUnlockRead=0;
    fbPutStr(pState+"/phase","restocking_busy");
    return;
  }

  oled("Preparing","please wait");
  fbPutStr(pState+"/phase","preparing");

  // who is buying
  activeUid   = fbGetStr(pUser+"/uid");
  activeEmail = fbGetStr(pUser+"/email");
  activeSession = fbGetStr(pUser+"/session");
  if(activeSession==""){ activeSession=String(millis()); fbPutStr(pUser+"/session",activeSession); }

  // PAYMENT GATE: balance must afford at least 1 of the cheapest enabled+stocked item.
  // The website is authoritative for balance; it sets control/affordable = 1 before unlock=1.
  // 'affordable' is read from the SAME batched control snapshot that detected unlock=1,
  // so it can never race against a second GET (that was falsely blocking valid buyers).
  if(affordable!=1){
    Serial.println("Blocked: cannot afford 1 item");
    oled("Top up","needed");
    fbPutInt(pControl+"/unlock",0);
    lastUnlockRead=0;
    fbPutStr(pState+"/phase","insufficient_balance");
    delay(2000);
    oled("Welcome");
    return;
  }

  // fresh checkout node
  fbPutStr(pCheckout+"/session",activeSession);
  fbPutStr(pCheckout+"/status","processing");
  fbPutFloat(pCheckout+"/total",0.00);
  fbPutStr(pCheckout+"/items","");
  fbPutStr(pCheckout+"/img","");

  // record START weights BEFORE unlocking (more samples = stable billing baseline)
  loadAllSlots();
  for(int i=0;i<3;i++){
    slot[i].startRaw = rawAvg(i,2);
    fbPutInt(pWeight+"/start/s"+String(i+1),slot[i].startRaw);
  }

  // before photo — capture the URL for THIS session ("" when camera is disabled,
  // so the receipt/Purchase Review never shows a stale photo from a past sale).
  sessionBeforeUrl = camRequest("before","purchase_start");

  sessionActive=true; doorWasOpened=false; noDoorOpenReminder=false;
  trayWarn=false; trayWarnPhoto=false; unauthOpen=false; unauthPhoto=false;
  wrongActive=false; lastWrongBeep=0; doorOpenAlarmActive=false; doorOpenStart=0;

  fbPutStr(pState+"/phase","unlocked");
  fbPutInt(pControl+"/ready",0);
  sessionUnlockTime=millis();
  unlockSolenoid();
  fbPutInt(pState+"/solenoid",1);
  beep();
  oled("Pls choose","item");
}

void handleAutoLock(){
  if(restockActive){
    // Single heat-safe pulse: drop coil after its window. Admin opens within 5s.
    if(solenoidActive && millis()-solenoidStart>=SOLENOID_UNLOCK_DURATION){
      lockSolenoid(); fbPutInt(pState+"/solenoid",0);
    }
    // safety: auto-end restock if admin forgot to close
    if(millis()-restockStart>=RESTOCK_MAX_DURATION) endRestock();
    return;
  }

  // Auto-drop the solenoid coil after its energize window (prevents overheating).
  // Single heat-safe pulse: the buyer opens within 5s. If not, the no-open flow
  // (10s reminder, 20s checkout) handles it.
  if(solenoidActive && millis()-solenoidStart>=SOLENOID_UNLOCK_DURATION){
    lockSolenoid(); fbPutInt(pState+"/solenoid",0);
  }

  // Two-stage handling when the user never opens the door after unlocking:
  if(sessionActive && !doorWasOpened && !checkoutInProgress){
    unsigned long since = millis()-sessionUnlockTime;

    // STAGE 1 @ 10s: remind with 5 beeps AND re-open for one more 5s window.
    if(since>=NO_OPEN_REMINDER_MS && !noDoorOpenReminder){
      noDoorOpenReminder=true;
      Serial.println("No door open after 10s. 5-beep reminder + re-open.");
      oled("Please open","the door");
      for(int i=0;i<5;i++) beep();
      unlockSolenoid(); fbPutInt(pState+"/solenoid",1);  // second 5s window
      fbPutStr(pState+"/phase","reminder_open_door");
    }

    // STAGE 2 @ 20s: still not opened -> checkout with 0 item.
    if(since>=NO_OPEN_CHECKOUT_MS){
      Serial.println("Still no door open after 20s. Jumping to checkout.");
      finishCheckout("door_not_opened_timeout");
    }
  }
}

// =====================================================================
// UNAUTHORIZED DOOR (case 1)
// =====================================================================
void handleUnauthorized(int v){
  if(restockActive) return;   // admin is restocking, ignore door alarms
  if(v==1 && !sessionActive && !checkoutInProgress){
    if(!unauthOpen){
      unauthOpen=true; unauthPhoto=true;
      digitalWrite(ALARM_LED_PIN,HIGH);
      oled("Close door","now!");
      beep4();
      String img=camRequest("security","unauthorized_door");
      pushSecurity("unauthorized_door_open",img);
      fbPutStr(pState+"/phase","unauthorized_door");
    }
  }
  if(v==0 && unauthOpen){
    unauthOpen=false; unauthPhoto=false;
    digitalWrite(BUZZER_PIN,LOW); digitalWrite(ALARM_LED_PIN,LOW);
    fbPutInt(pState+"/buzzer",0);
    fbPutInt(pSecurity+"/alarm",0);
    oled("Welcome");
    fbPutStr(pState+"/phase","locked");
  }
}

// =====================================================================
// FAST TRAY CHECK during door open (local, no Firebase spam)
// =====================================================================
void fastTrayCheck(){
  if(restockActive) return;   // no wrong-weight nagging while admin restocks
  if(!sessionActive) return;
  if(getDoorVal()!=1) return;      // ONLY check while the door is actually open
                                   // (door shut => nothing can be removed/swapped)
  if(doorOpenAlarmActive) return;  // 5-min door alarm owns the buzzer (higher priority)
  unsigned long now=millis();
  if(now-tFast<FAST_INT) return; tFast=now;

  // Simplified: any tray weight that is not a clean item count (1,2,3... units) is
  // flagged as "Wrong item" — whether it's a swapped/exchanged item or a foreign
  // object makes no difference to the user, the fix is the same: put it back right.
  bool wrong=false;
  for(int i=0;i<3;i++){
    if(slot[i].en!=1 || slot[i].unitRaw<=0) continue;
    long raw=rawQuick(i); if(raw==0) continue;
    int m=-1;
    if(!trayValid(i,raw,&m)){ wrong=true; break; }   // foreign / wrong item on this tray
  }

  if(wrong){
    // Ring 5 times every 10 seconds while it stays wrong (NOT continuous).
    if(!wrongActive){
      wrongActive=true;
      lastWrongBeep=0;                 // force an immediate first burst
      oled("Wrong item","place correctly");
      if(!trayWarnPhoto){
        trayWarnPhoto=true;

        // Start alarm immediately BEFORE camera request
        digitalWrite(ALARM_LED_PIN,HIGH);
        fbPutInt(pState+"/buzzer",1);
        for(int b=0;b<2;b++) beep();      // 2 rings
        fbPutInt(pState+"/buzzer",0);

        String img=camRequest("security","wrong_item_on_tray");
        pushSecurity("invalid_tray_weight", img);
        
         // Prevent instant extra 2-beep burst immediately after photo
        lastWrongBeep = millis();
      }
    }
    if(lastWrongBeep==0 || (millis()-lastWrongBeep >= 10000)){
      lastWrongBeep=millis();
      digitalWrite(ALARM_LED_PIN,HIGH);
      fbPutInt(pState+"/buzzer",1);
      for(int b=0;b<4;b++) beep();      // 4 rings
      fbPutInt(pState+"/buzzer",0);
    }
  } else {
    if(wrongActive){
      wrongActive=false; trayWarnPhoto=false; lastWrongBeep=0;
      digitalWrite(BUZZER_PIN,LOW); digitalWrite(ALARM_LED_PIN,LOW);
      fbPutInt(pState+"/buzzer",0);
      if(getDoorVal()==1) oled("Pls choose","item");
    }
  }
}

// 5-minute door-left-open alarm: 5 beeps every 5s + security alert until closed
void checkDoorOpenTooLong(){
  if(restockActive){ doorOpenStart=0; return; }   // admin restocking: no nag
  int v=getDoorVal();
  if(v!=1){                 // door closed -> clear alarm
    if(doorOpenAlarmActive){
      doorOpenAlarmActive=false;
      digitalWrite(BUZZER_PIN,LOW); digitalWrite(ALARM_LED_PIN,LOW);
      fbPutInt(pState+"/buzzer",0);
      fbPutInt(pSecurity+"/alarm",0);
    }
    doorOpenStart=0;
    return;
  }

  // door is open: start timer once
  if(doorOpenStart==0){ doorOpenStart=millis(); return; }

  if(millis()-doorOpenStart >= DOOR_OPEN_LIMIT){
    if(!doorOpenAlarmActive){
      doorOpenAlarmActive=true;
      lastDoorOpenBeep=0;
      oled("Close the","door!");
      fbPutInt(pSecurity+"/alarm",1);
      fbPutStr(pSecurity+"/reason","door_open_over_5min");
      String img=camRequest("security","door_open_over_5min");
      pushSecurity("door_open_over_5min",img);
    }
    // 5 beeps every 5 seconds until door closes
    if(millis()-lastDoorOpenBeep >= DOOR_OPEN_BEEP_INTERVAL){
      lastDoorOpenBeep=millis();
      for(int i=0;i<5;i++) beep();
      digitalWrite(ALARM_LED_PIN,HIGH);
    }
  }
}

// =====================================================================
// CHECKOUT
// =====================================================================
void writeCheckout(String reason,String img){
  float total=0; String items="";
  String slotJson="";
  for(int i=0;i<3;i++){
    float sub=slot[i].takenQty*slot[i].price;
    if(slot[i].takenQty>0){
      total+=sub;
      if(items!="") items+=",";
      items+=String(slot[i].takenQty)+"x "+slot[i].name;
    }
    long delta    = slot[i].startRaw - slot[i].finalRaw;          // positive = removed
    long expected = (long)slot[i].unitRaw * slot[i].takenQty;     // for mismatch detection
    String b="slot/s"+String(i+1);
    if(slotJson!="") slotJson+=",";
    slotJson+="\""+b+"/name\":\""+jesc(slot[i].name)+"\",";
    slotJson+="\""+b+"/qty\":"+String(slot[i].takenQty)+",";
    slotJson+="\""+b+"/price\":"+String(slot[i].price,2)+",";
    slotJson+="\""+b+"/subtotal\":"+String(sub,2)+",";
    slotJson+="\""+b+"/delta_raw\":"+String(delta)+",";
    slotJson+="\""+b+"/expected_raw\":"+String(expected);
  }
  String beforeUrl = sessionBeforeUrl;   // this session only — not the stale camera node

  // ONE atomic PATCH writes the entire receipt (was ~22 separate PUTs ~4s -> ~0.4s).
  // Because Firebase applies the whole PATCH atomically, the website never sees a
  // half-written checkout — when status flips to "done", every field is already there.
  String j="{";
  j+="\"session\":\""+jesc(activeSession)+"\",";
  j+="\"reason\":\""+jesc(reason)+"\",";
  j+="\"total\":"+String(total,2)+",";
  j+="\"items\":\""+jesc(items)+"\",";
  j+="\"img\":\""+jesc(img)+"\",";
  j+="\"before_url\":\""+jesc(beforeUrl)+"\",";   // for admin Purchase Review / AI verify
  j+="\"after_url\":\""+jesc(img)+"\",";
  j+="\"user_uid\":\""+jesc(activeUid)+"\",";
  j+="\"user_email\":\""+jesc(activeEmail)+"\",";
  j+="\"ai_verify\":\"pending\",";
  j+=slotJson+",";
  j+="\"status\":\"done\"";                       // website waits for this + matching session
  j+="}";
  fbPatchJson(pCheckout,j);
}

void finishCheckout(String reason){
  if(checkoutInProgress) return;
  checkoutInProgress=true;
  Serial.print("CHECKOUT: "); Serial.println(reason);

  fbPutStr(pState+"/phase","checkout_processing");
  fbPutInt(pControl+"/ready",0);
  clearTrayWarn();
  lockSolenoid(); fbPutInt(pState+"/solenoid",0);

  delay(500);
  delay(300);   // let trays settle after the door closes before final read
  String name1="";
  int totalQty=0; float total=0;
  String weightJson="{"; String invJson="{";   // batched: 6 writes -> 2 PATCHes
  for(int i=0;i<3;i++){
    slot[i].finalRaw=rawAvg(i,15);
    if(i>0){ weightJson+=","; invJson+=","; }
    weightJson+="\"s"+String(i+1)+"\":"+String(slot[i].finalRaw);

    long taken = slot[i].startRaw - slot[i].finalRaw;  // positive = removed
    if(slot[i].unitRaw>0 && taken>detectFloor(i)){
      int q=(int)round((double)taken/(double)slot[i].unitRaw);
      if(q<0)q=0; if(q>slot[i].qty)q=slot[i].qty;
      slot[i].takenQty=q;
    } else slot[i].takenQty=0;

    // update remaining stock from the new final reading
    slot[i].qty = qtyFromRaw(i,slot[i].finalRaw);
    slot[i].status = (slot[i].qty<=0)?"empty":"normal";
    invJson+="\"s"+String(i+1)+"/qty\":"+String(slot[i].qty)
            +",\"s"+String(i+1)+"/status\":\""+jesc(slot[i].status)+"\"";

    total += slot[i].takenQty*slot[i].price;
    totalQty += slot[i].takenQty;
    if(slot[i].takenQty>0 && name1=="") name1=slot[i].name;
  }
  weightJson+="}"; invJson+="}";
  fbPatchJson(pWeight+"/final",weightJson);   // all final weights in one request
  fbPatchJson(pInv,invJson);                  // all remaining stock in one request
  if(name1=="") name1="No item";

  fbPutStr(pState+"/phase","checkout_display");
  oledCheckoutMulti(total);

  String img=camRequest("after","purchase_done");

  writeCheckout(reason,img);

  // signal website to settle payment
  fbPutStr(pState+"/phase","checkout_done");
  fbPutInt(pControl+"/ready",1);
  fbPutInt(pControl+"/unlock",0);
  lastUnlockRead=0;

  esp_task_wdt_reset();   // the OLED display delays below are long; keep watchdog fed
  delay(3000);   // receipt already on the user's app; this is just the OLED display
  oled("Thank","You");
  delay(1500);
  oled("Welcome");

  fbPutStr(pState+"/phase","locked");
  sessionActive=false; doorWasOpened=false; noDoorOpenReminder=false; checkoutInProgress=false;
  activeSession="";
  Serial.println("CHECKOUT DONE");
}

// =====================================================================
// ADMIN RESTOCK MODE (no payment, alarms suspended)
// =====================================================================
void startRestock(){
  if(sessionActive || checkoutInProgress){ Serial.println("restock blocked: busy"); return; }
  restockActive=true; restockStart=millis();
  adminUnlockZeroCount=0;
  doorOpenAlarmActive=false; doorOpenStart=0;   // don't let door-open alarm trigger
  clearTrayWarn();
  unlockSolenoid();
  fbPutInt(pState+"/solenoid",1);
  fbPutStr(pState+"/phase","restocking");
  fbPutInt(pSecurity+"/alarm",0);
  fbPutInt(pSecurity+"/notify_admin",0);
  beep();
  oled("Restock","mode");
  Serial.println("RESTOCK started");
}

void endRestock(){
  if(!restockActive) return;
  restockActive=false;
  adminUnlockZeroCount=0;
  lockSolenoid();
  fbPutInt(pState+"/solenoid",0);
  oled("Counting","stock...");
  // recalculate stock for every slot from the new weights.
  // 15 samples = high precision: restock count must be exact for inventory accuracy.
  for(int i=0;i<3;i++){
    if(slot[i].en!=1 || slot[i].unitRaw<=0) continue;
    long raw=rawAvg(i,15);
    slot[i].qty=qtyFromRaw(i,raw);
    slot[i].status=(slot[i].qty<=0)?"empty":"normal";
    writeSlotRuntime(i);
  }
  fbPutInt(pControl+"/admin_unlock",0);
  fbPutInt(pState+"/door",getDoorVal());
  fbPutStr(pState+"/phase","locked");
  oled("Welcome");
  Serial.println("RESTOCK ended, stock recalculated");
}

// =====================================================================
// DOOR PROCESSING
// =====================================================================
void processDoor(){
  int v=getDoorVal();
  if(lastDoorVal==-1){ lastDoorVal=v; fbPutInt(pState+"/door",v); return; }
  if(v!=lastDoorVal){
    fbPutInt(pState+"/door",v);

    // RESTOCK: door open = fine; door closed = finish restock & recount
    if(restockActive){
      if(v==1) fbPutStr(pState+"/phase","restock_open");
      if(lastDoorVal==1 && v==0){ lastDoorVal=v; endRestock(); return; }
      lastDoorVal=v; return;
    }

    if(lastDoorVal==0 && v==1 && sessionActive){
      doorWasOpened=true; fbPutStr(pState+"/phase","user_taking");
    }
    if(lastDoorVal==1 && v==0 && sessionActive){
      clearTrayWarn(); lastDoorVal=v; finishCheckout("door_closed"); return;
    }
    if(v==1 && !sessionActive && !checkoutInProgress) handleUnauthorized(v);
    if(v==0 && unauthOpen) handleUnauthorized(v);
  }
  lastDoorVal=v;
}

// =====================================================================
// ADMIN COMMANDS (tare / calibrate / stock_check)  admin/last is target slot key
// =====================================================================
void doTare(int i){
  long raw=rawAvg(i,10);   // 10 samples for a precise empty-tray baseline
  slot[i].tare=raw;
  slot[i].noise=measureNoise(i);   // baseline noise for light-item detection
  String b=pInv+"/s"+String(i+1);
  fbPutInt(b+"/tare",raw);
  fbPutInt(b+"/noise",slot[i].noise);
  fbPutStr(b+"/status","need_calibration");
  Serial.print("Tare s"); Serial.print(i+1); Serial.print(" = "); Serial.print(raw);
  Serial.print("  noise="); Serial.println(slot[i].noise);
}
void doCalibrate(int i){
  long raw=rawAvg(i,8);
  long total=raw-slot[i].tare;
  // admin/cal_count lets admin calibrate from N items at once (much more accurate).
  int n=(int)fbGetInt(pAdmin+"/cal_count");
  if(n<1) n=1;
  // Accept light items: the total must beat the noise floor, not a fixed gram guess.
  long minTotal = (long)(slot[i].noise * NOISE_MARGIN) * n;
  if(minTotal < ABS_MIN_UNIT_RAW * n) minTotal = ABS_MIN_UNIT_RAW * n;
  if(total < minTotal){
    Serial.print("calibrate FAIL: total="); Serial.print(total);
    Serial.print(" < min="); Serial.println(minTotal);
    slot[i].calStatus="not_calibrated";
    return;
  }
  long unit = total / n;                 // average raw of ONE item
  slot[i].unitRaw=unit; slot[i].qty=n; slot[i].calStatus="calibrated"; slot[i].status="normal";
  String b=pInv+"/s"+String(i+1);
  fbPutInt(b+"/unit_raw",unit);
  fbPutInt(b+"/unit_g",unit);            // compatibility
  fbPutInt(b+"/qty",n);
  fbPutStr(b+"/cal_status","calibrated");
  fbPutStr(b+"/status","normal");
  Serial.print("Calibrate s"); Serial.print(i+1);
  Serial.print(" from "); Serial.print(n); Serial.print(" item(s), unit_raw="); Serial.println(unit);
}
void doStockCheck(int i){
  long raw=rawAvg(i,8);
  slot[i].qty=qtyFromRaw(i,raw);
  slot[i].status=(slot[i].qty<=0)?"empty":"normal";
  writeSlotRuntime(i);
  Serial.print("StockCheck s"); Serial.print(i+1); Serial.print(" qty="); Serial.println(slot[i].qty);
}
void readAdminCommands(){
  // ONE batched read of the admin node (was 4 separate HTTP reads).
  String a = fbGetRaw(pAdmin);
  if(a=="" || a=="null") return;
  int tare=jget(a,"tare").toInt();
  int cal =jget(a,"calibrate").toInt();
  int chk =jget(a,"stock_check").toInt();
  int s   =jget(a,"slot").toInt();               // 1..3
  if(s<1||s>3) return;
  int i=s-1;

  if(tare==1||cal==1||chk==1) loadSlot(i);   // refresh latest tare/unit before acting

  bool did=false; String msg="";
  if(tare==1){
    doTare(i); fbPutInt(pAdmin+"/tare",0); did=true;
    msg="Tare done for slot "+String(s);
    oled("Tare OK","slot "+String(s));
  }
  if(cal==1){
    doCalibrate(i); fbPutInt(pAdmin+"/calibrate",0); did=true;
    if(slot[i].calStatus=="calibrated"){
      int n=(int)fbGetInt(pAdmin+"/cal_count"); if(n<1)n=1;
      msg="Calibrated slot "+String(s)+" from "+String(n)+" item(s)";
      oled("Calib OK","slot "+String(s));
    } else {
      msg="Calibrate FAILED slot "+String(s)+" - check items on tray";
      oled("Calib FAIL","check item");
    }
  }
  if(chk==1){
    doStockCheck(i); fbPutInt(pAdmin+"/stock_check",0); did=true;
    msg="Stock counted slot "+String(s)+": "+String(slot[i].qty);
    oled("Stock="+String(slot[i].qty),"slot "+String(s));
  }
  if(did){
    fbPutInt(pAdmin+"/done",1);
    fbPutStr(pAdmin+"/last","s"+String(s));
    fbPutStr(pAdmin+"/last_result",msg);   // admin website shows this
    // confirmation beep: 1 short = success, 2 short = calibrate fail
    if(cal==1 && slot[i].calStatus!="calibrated"){ beep(); beep(); }
    else { beep(); }
    Serial.println(msg);
    // OLED returns to normal after a moment if not in a session
    if(!sessionActive && !restockActive){ delay(800); oled("Welcome"); }
  }
}

// =====================================================================
// NORMAL STOCK UPDATE (idle only)
// =====================================================================
void updateStockIdle(){
  if(sessionActive && getDoorVal()==1) return;  // fast check owns the cells
  for(int i=0;i<3;i++){
    if(slot[i].en!=1 || slot[i].unitRaw<=0) continue;
    long raw=rawAvg(i,4);
    int q=qtyFromRaw(i,raw);
    String st=(q<=0)?"empty":"normal";
    if(q!=slot[i].qty || st!=slot[i].status){
      slot[i].qty=q; slot[i].status=st; writeSlotRuntime(i);
    }
  }
}

// =====================================================================
// FIREBASE COMMAND POLL
// =====================================================================
void readControl(){
  // ONE batched read of the whole control node (was 4-5 separate HTTP reads
  // every 500ms — the hottest poll in the system).
  String c = fbGetRaw(pControl);
  if(c=="" || c=="null") return;   // failed/empty read: never act on phantom values

  int unlock = jget(c,"unlock").toInt();
  int affordable = /*1;*/ jget(c,"affordable").toInt();   // read atomically with unlock (no race)
  int reset  = jget(c,"reset").toInt();

  // Cache camera enabled state (null/missing = leave as-is, default ON)
  String camEnRaw = jget(c,"camera_enabled");
  if(camEnRaw!="") cameraEnabled = (camEnRaw.toInt()!=0);
  // Cache camera storage preference ("both" | "imgbb" | "sd")
  String camStRaw = jget(c,"camera_storage");
  if(camStRaw!=""){
    if(camStRaw!=camStorage) Serial.println("camStorage changed -> "+camStRaw);  // diagnostic
    camStorage = camStRaw;
  }

  String auRaw = jget(c,"admin_unlock");

  // Only act on a CONFIRMED value. A failed/empty Firebase read returns ""
  // and must NOT be treated as 0 (that was falsely ending restock).
  if(auRaw!=""){
    int adminUnlock = auRaw.toInt();
    if(adminUnlock==1 && !restockActive){
      adminUnlockZeroCount=0;
      startRestock();
    }
    else if(adminUnlock==0 && restockActive){
      // require 2 consecutive confirmed 0 reads before ending (debounce)
      adminUnlockZeroCount++;
      if(adminUnlockZeroCount>=2){ adminUnlockZeroCount=0; endRestock(); }
    }
    else {
      adminUnlockZeroCount=0;
    }
  }

  if(unlock==1 && lastUnlockRead!=1) startSession(affordable);
  lastUnlockRead=unlock;
  if(reset==1){
    fbPutInt(pControl+"/reset",0);
    clearTrayWarn(); lockSolenoid(); fbPutInt(pState+"/solenoid",0);
    sessionActive=false; checkoutInProgress=false; unauthOpen=false; restockActive=false;
    doorOpenAlarmActive=false; doorOpenStart=0;
    fbPutStr(pState+"/phase","locked"); oled("Welcome");
  }
}

// =====================================================================
// SETUP
// =====================================================================
void setup(){
  Serial.begin(115200); delay(400);
  Serial.println("\nESP32 MAIN - memory-doc structure");

  pinMode(DOOR_LOCK_PIN,OUTPUT); lockSolenoid();
  pinMode(BUZZER_PIN,OUTPUT); digitalWrite(BUZZER_PIN,LOW);
  pinMode(ALARM_LED_PIN,OUTPUT); digitalWrite(ALARM_LED_PIN,LOW);
  pinMode(DOOR_SENSOR_PIN,INPUT_PULLUP);

  for(int i=0;i<3;i++) scale[i].begin(HX_DT[i],HX_SCK[i]);

  Wire.begin(21,22);
  if(!display.begin(SSD1306_SWITCHCAPVCC,0x3C)) Serial.println("OLED missing");
  else oled("Welcome");

  connectWiFi();

  // state
  fbPutInt(pState+"/esp32",1);
  fbPutInt(pState+"/door",getDoorVal());
  fbPutStr(pState+"/phase","locked");
  fbPutInt(pState+"/solenoid",0);
  fbPutInt(pState+"/buzzer",0);

  // control
  fbPutInt(pControl+"/unlock",0);
  fbPutInt(pControl+"/ready",1);
  fbPutInt(pControl+"/reset",0);
  fbPutInt(pControl+"/affordable",0);
  fbPutInt(pControl+"/admin_unlock",0);

  // security/log
  fbPutInt(pSecurity+"/alarm",0);
  fbPutInt(pSecurity+"/notify_admin",0);
  fbPutStr(pLog+"/event","boot");

  // admin command channel
  fbPutInt(pAdmin+"/tare",0);
  fbPutInt(pAdmin+"/calibrate",0);
  fbPutInt(pAdmin+"/stock_check",0);
  fbPutInt(pAdmin+"/done",0);
  fbPutInt(pAdmin+"/cal_count",1);

  loadAllSlots();
  refreshCamIP();
  lastDoorVal=getDoorVal();

  // Hardware watchdog — started AFTER boot so a slow WiFi/Firebase boot can't trip
  // it. If any single loop iteration ever hangs (e.g. a stuck HTTP call) longer
  // than WDT_TIMEOUT_S, the board auto-resets and recovers instead of freezing.
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t wdtCfg = { .timeout_ms = WDT_TIMEOUT_S * 1000, .idle_core_mask = 0, .trigger_panic = true };
  esp_task_wdt_init(&wdtCfg);
#else
  esp_task_wdt_init(WDT_TIMEOUT_S, true);
#endif
  esp_task_wdt_add(NULL);

  Serial.println("Ready. Website sets control/affordable=1 then control/unlock=1 to vend.");
}

// =====================================================================
// LOOP
// =====================================================================
void loop(){
  unsigned long now=millis();
  esp_task_wdt_reset();   // pet the watchdog every iteration
  checkWiFi();

  // Online heartbeat so the website/admin can detect an offline machine.
  if(now-tHeart>=HEART_INT){
    tHeart=now;
    fbPutRaw(pState+"/heartbeat","{\".sv\":\"timestamp\"}");
  }

  // Always-on, time-critical work (local + emergency control poll).
  // readControl() still runs every cycle so control/reset and admin_unlock are
  // never ignored — that is the emergency channel.
  if(now-tFbRead>=FB_READ_INT){ tFbRead=now; readControl(); }
  if(now-tDoor>=DOOR_INT){ tDoor=now; processDoor(); }
  handleAutoLock();
  fastTrayCheck();
  checkDoorOpenTooLong();

  // ---- MODE GATING (progress-by-progress) ----
  // During a live purchase the load cells + door are time-critical. The admin
  // command poll (4 blocking Firebase reads) and the idle stock poll would each
  // stall the loop ~150ms+ and slow fastTrayCheck/door response. Skip them while
  // busy; they resume the moment the machine returns to idle.
  bool busy = sessionActive || checkoutInProgress;

  // Admin tare/calibrate/stock-check: only when NOT in a user purchase.
  // (Still allowed during restock, so the admin can calibrate while refilling.)
  if(!busy){
    if(now-tAdmin>=ADMIN_INT){ tAdmin=now; readAdminCommands(); }
  }
  // Idle stock recount: only when truly idle — not buying, not restocking.
  if(!busy && !restockActive){
    if(now-tInv>=INV_INT){ tInv=now; updateStockIdle(); }
  }
}

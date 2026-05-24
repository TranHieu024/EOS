//// final ok moi chuc nang toi 5/5
#include "HX710.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Preferences.h>
#include <math.h> 
#include <sys/time.h>
#include <esp_sleep.h> 

// --- THINGSBOARD & WIFI ---
#include <WiFi.h>
#include <PubSubClient.h>

const char* WIFI_SSID = "h5g";
const char* WIFI_PASS = "19024705";

const char* TB_SERVER = "thingsboard.cloud";
const char* TB_TOKEN  = "1pR9oKFunZfW7UOCc4Kb";   

WiFiClient espClient;
PubSubClient mqttClient(espClient);

int upload_step = 0; 
String tb_status_msg = "";

// ==========================================
// 1. CẤU HÌNH PIN VÀ HARDWARE
// ==========================================
const int PIN_SCK       = 18;
const int PIN_DOUT      = 34;
const int PIN_VALVE_IN3 = 26; 
const int PIN_VALVE_IN4 = 25; 
const int PIN_PUMP_IN1  = 27; 
const int PIN_PUMP_IN2  = 32; 

const int BTN_START     = 23; // Nút bấm Đo
const int BTN_TIME      = 13;  
const int BTN_USER      = 33; // Công tắc gạt (LOW = User 1, HIGH = User 2)
const int BTN_UP        = 19; 
const int BTN_DOWN      = 14; 
const int BTN_MEM       = 4;

#define SCREEN_WIDTH 128 
#define SCREEN_HEIGHT 64 
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

HX710 ps;
Preferences preferences;  

long zero_offset = 0;
const float SCALE_FACTOR = 25000.0;  
const float SCALE_CORRECTION = 173.0 / 605.81; 
int current_pwm = 165; 


unsigned long last_interaction_time = 0;
int current_user = 1;

// interrupt
volatile bool flag_start = false;
volatile unsigned long last_isr_time = 0;

void IRAM_ATTR isrStart() {
  unsigned long now = millis();
  if (now - last_isr_time > 300) { 
    flag_start = true;
    last_isr_time = now;
  }
}


bool buttonPressed(int pin) {
  static bool last_state[40]; 
  static bool initialized = false;
  
  if (!initialized) {
    for(int i = 0; i < 40; i++) last_state[i] = HIGH;
    initialized = true;
  }

  bool state = digitalRead(pin);
  if (state == LOW && last_state[pin] == HIGH) {
    delay(30); 
    last_state[pin] = state; 
    if (digitalRead(pin) == LOW) {
      last_interaction_time = millis(); // Reset hẹn giờ ngủ
      return true; 
    }
  }
  last_state[pin] = state; 
  return false;
}


enum SystemState { IDLE, MEASURING, RESULT, MEMORY_VIEW, SET_TIME };
SystemState currentState = IDLE;

struct BPRecord {
  uint8_t sys; uint8_t dia; uint8_t bpm;
  uint8_t day; uint8_t month; uint16_t year;
  uint8_t hour; uint8_t min;
};
BPRecord history[10]; 
int historyCount = 0;
int currentMemIndex = 0; 

int timeSetStep = 0; 
int t_day = 04, t_mon = 5, t_year = 2026, t_hr = 7, t_min = 30;

#define MAX_SAMPLES 2000
long adc_buffer[MAX_SAMPLES];
unsigned long time_buffer[MAX_SAMPLES];
int sample_count = 0;
int final_SYS = 0, final_DIA = 0, final_BPM = 0;

void loadDataFromFlash(int user) {
  preferences.begin("bp_data", false);
  current_pwm = preferences.getInt("saved_pwm", 169);
  String cntKey = "hCnt" + String(user);
  String datKey = "hist" + String(user);
  
  historyCount = preferences.getInt(cntKey.c_str(), 0);
  if (historyCount > 0) {
    preferences.getBytes(datKey.c_str(), history, sizeof(history));
  } else {
    for(int i=0; i<10; i++) { history[i].sys = 0; history[i].dia = 0; history[i].bpm = 0; }
  }
  preferences.end();
}

void saveRecord(int sys, int dia, int bpm, int user) {
  struct timeval tv; gettimeofday(&tv, NULL);
  struct tm* tm_info = localtime(&tv.tv_sec);
  
  for(int i = 9; i > 0; i--) history[i] = history[i-1]; 
  
  history[0].sys = sys; history[0].dia = dia; history[0].bpm = bpm;
  history[0].day = tm_info->tm_mday; history[0].month = tm_info->tm_mon + 1;
  history[0].year = tm_info->tm_year + 1900;
  history[0].hour = tm_info->tm_hour; history[0].min = tm_info->tm_min;
  
  if (historyCount < 10) historyCount++;
  
  String cntKey = "hCnt" + String(user);
  String datKey = "hist" + String(user);
  
  preferences.begin("bp_data", false);
  preferences.putBytes(datKey.c_str(), history, sizeof(history));
  preferences.putInt(cntKey.c_str(), historyCount);
  preferences.end();
}

void updateRTC() {
  struct tm tm;
  tm.tm_year = t_year - 1900; tm.tm_mon = t_mon - 1; tm.tm_mday = t_day;
  tm.tm_hour = t_hr; tm.tm_min = t_min; tm.tm_sec = 0;
  time_t t = mktime(&tm);
  struct timeval now = { .tv_sec = t };
  settimeofday(&now, NULL);
}

void drawIdleScreen() {
  display.clearDisplay();
  struct timeval tv; gettimeofday(&tv, NULL);
  struct tm* tm_info = localtime(&tv.tv_sec);

  display.setTextSize(1); display.setCursor(0, 0); 
  display.printf("%02d/%02d/%04d   %02d:%02d", tm_info->tm_mday, tm_info->tm_mon + 1, tm_info->tm_year + 1900, tm_info->tm_hour, tm_info->tm_min);
  display.drawLine(0, 10, 128, 10, WHITE); 
  
  display.setTextSize(2); display.setCursor(25, 18); 
  display.printf("USER %d", current_user);
  
  display.setTextSize(1); display.setCursor(15, 40); 
  display.print("READY TO MEASURE");
  
  display.setCursor(0, 55); display.printf("Valve: %d  (Auto Sleep)", current_pwm);
  display.display();
}

void drawSetTimeScreen() {
  display.clearDisplay();
  display.setTextSize(1); display.setCursor(0, 0); display.print("--- SET TIME ---");
  display.drawLine(0, 10, 128, 10, WHITE);
  
  display.setCursor(10, 20); display.printf("Date: %02d / %02d / %04d", t_day, t_mon, t_year);
  display.setCursor(10, 35); display.printf("Time: %02d : %02d", t_hr, t_min);
  
  display.setCursor(0, 55); display.print("Setting: ");
  if (timeSetStep == 0) display.print("DAY");
  else if (timeSetStep == 1) display.print("MONTH");
  else if (timeSetStep == 2) display.print("YEAR");
  else if (timeSetStep == 3) display.print("HOUR");
  else if (timeSetStep == 4) display.print("MINUTE");
  display.display();
}

void drawMemoryScreen() {
  display.clearDisplay();
  display.setTextSize(1); display.setCursor(0, 0); display.printf("- MEMORY (USER %d) -", current_user);
  display.drawLine(0, 10, 128, 10, WHITE);
  
  if (historyCount == 0 || history[currentMemIndex].sys == 0) {
    display.setCursor(20, 30); display.print("NO RECORDS YET");
  } else {
    BPRecord r = history[currentMemIndex];
    display.setCursor(0, 15); display.printf("No.%d", currentMemIndex + 1);
    display.setCursor(50, 15); display.printf("%02d/%02d  %02d:%02d", r.day, r.month, r.hour, r.min);
    
    display.setTextSize(2); display.setCursor(10, 30); display.printf("%d/%d", r.sys, r.dia);
    display.setTextSize(1); display.setCursor(10, 52); display.printf("PULSE: %d bpm", r.bpm);
  }
  display.display();
}

void drawMeasuringScreen(int p, String txt) {
  display.clearDisplay();
  display.setTextSize(1); display.setCursor(0, 0); 
  display.printf("USER %d - %s", current_user, txt.c_str());
  display.drawLine(0, 10, 128, 10, WHITE);
  
  display.setTextSize(3); display.setCursor(25, 30);
  if(p < 100) display.print(" "); if(p < 10) display.print(" ");
  display.print(p);
  
  display.setTextSize(1); display.setCursor(85, 45); display.print("mmHg");
  display.display();
}

void drawResultScreen(int sys, int dia, int bpm, String status_msg) {
  display.clearDisplay();
  display.setTextSize(1); display.setCursor(0, 0); 
  display.printf("--- USER %d RESULT ---", current_user);
  display.drawLine(0, 10, 128, 10, WHITE);
  
  display.setCursor(0, 15); display.print("SYS:");
  display.setTextSize(2); display.setCursor(30, 15); display.print(sys);
  
  display.setTextSize(1); display.setCursor(0, 35); display.print("DIA:");
  display.setTextSize(2); display.setCursor(30, 35); display.print(dia);

  display.setTextSize(1); display.setCursor(0, 55); display.print("PUL:");
  display.setCursor(30, 55); display.print(bpm); 
  
  // Hiển thị trạng thái MQTT ở góc dưới bên phải
  display.setCursor(68, 55); 
  display.print(status_msg);
  
  display.display();
}

void drawErrorScreen(String errorMsg) {
  display.clearDisplay();
  display.setTextSize(2); display.setCursor(35, 15); display.print("ERROR");
  display.setTextSize(1); display.setCursor(0, 40); 
  int16_t x1, y1; uint16_t w, h;
  display.getTextBounds(errorMsg, 0, 40, &x1, &y1, &w, &h);
  display.setCursor((128 - w) / 2, 40);
  display.print(errorMsg);
  display.display();
}


void setValve(int pwm) { analogWrite(PIN_VALVE_IN4, pwm); }
void pumpPWM(int pwm) { analogWrite(PIN_PUMP_IN2, pwm); analogWrite(PIN_PUMP_IN1, 0); }

float getPressure() {
  if (ps.isReady()) {
    ps.readAndSelectNextData(HX710_DIFFERENTIAL_INPUT_40HZ);
    return (((ps.getLastDifferentialInput() - zero_offset) / SCALE_FACTOR) * SCALE_CORRECTION);
  }
  return -1;
}

bool checkEmergencyStop() {
  if (flag_start) {
    flag_start = false; 
    setValve(0); pumpPWM(0); 
    currentState = IDLE;     
    last_interaction_time = millis();
    return true;
  }
  return false;
}

void enterSleepMode() {
  Serial.println("INACTIVITY TIMEOUT -> SLEEPING...");
  
  display.ssd1306_command(SSD1306_DISPLAYOFF); 
  setValve(0); 
  pumpPWM(0);
  digitalWrite(PIN_SCK, HIGH); 
  
  gpio_wakeup_enable(GPIO_NUM_23, GPIO_INTR_LOW_LEVEL);
  esp_sleep_enable_gpio_wakeup();
  
  WiFi.mode(WIFI_OFF);
  esp_light_sleep_start(); 
  
  Serial.println("WOKE UP!");
  digitalWrite(PIN_SCK, LOW); 
  display.ssd1306_command(SSD1306_DISPLAYON);
  
  last_interaction_time = millis();
  flag_start = false; 
  currentState = IDLE;
}

// thuat toan chinh gausssian
struct Peak { float time_s; float dc_press; float ac_amp; };

void calculateBloodPressure() {
  float* pressure = new float[sample_count]; float* dc = new float[sample_count]; float* ac = new float[sample_count];
  for (int i=0; i<sample_count; i++) pressure[i] = ((adc_buffer[i]-zero_offset)/SCALE_FACTOR)*SCALE_CORRECTION;

  int window = 20; 
  for (int i=0; i<sample_count; i++) {
    float sum=0; int count=0;
    for (int j=max(0,i-window/2); j<=min(sample_count-1,i+window/2); j++) { sum+=pressure[j]; count++; }
    dc[i] = sum/count; ac[i] = pressure[i]-dc[i];  
  }

  Peak peaks[100]; int num_peaks = 0; float max_amp_overall = 0.0;
  for (int i=15; i<sample_count-15; i++) {
    bool is_max = true;
    for (int j=i-15; j<=i+15; j++) { if (ac[j]>ac[i]) { is_max = false; break; } }
    if (is_max && ac[i]>0.05) {
      peaks[num_peaks].time_s = (time_buffer[i]-time_buffer[0])/1000.0;
      peaks[num_peaks].dc_press = dc[i]; peaks[num_peaks].ac_amp = ac[i];
      if (ac[i]>max_amp_overall) max_amp_overall = ac[i];
      num_peaks++; i+=15; if (num_peaks>=100) break;
    }
  }

  if (num_peaks > 4) {
    double S4=0, S3=0, S2=0, S1=0, S0=0, SYX2=0, SYX1=0, SYX0=0;
    for (int i=0; i<num_peaks; i++) {
      if (peaks[i].ac_amp < max_amp_overall * 0.15) continue;
      double x = peaks[i].dc_press; double y = log(peaks[i].ac_amp); double x2 = x*x;
      S4 += x2*x2; S3 += x2*x; S2 += x2; S1 += x; SYX2 += y*x2; SYX1 += y*x; SYX0 += y; S0 += 1;
    }
    double D  = S4*(S2*S0-S1*S1) - S3*(S3*S0-S1*S2) + S2*(S3*S1-S2*S2);
    double DA = SYX2*(S2*S0-S1*S1) - S3*(SYX1*S0-SYX0*S1) + S2*(SYX1*S1-SYX0*S2);
    double DB = S4*(SYX1*S0-SYX0*S1) - SYX2*(S3*S0-S1*S2) + S2*(S3*SYX0-SYX1*S2);
    double A_coef = DA/D, B_coef = DB/D;

    if (D!=0 && A_coef<0) {
      float sigma = sqrt(-1.0/(2.0*A_coef)); float map_p = -B_coef/(2.0*A_coef);    
      float sys_r = (sigma>20.0)?0.54:0.58; float dia_r = (sigma>20.0)?0.82:0.86;
      final_SYS = round(map_p + sigma*sqrt(-2.0*log(sys_r)));
      final_DIA = round(map_p - sigma*sqrt(-2.0*log(dia_r)));
      if (final_SYS>250 || final_DIA<30 || final_SYS<=final_DIA) goto FALLBACK;
    } else {
      FALLBACK:
      int mx_id=0; for(int i=1;i<num_peaks;i++) if(peaks[i].ac_amp>peaks[mx_id].ac_amp) mx_id=i;
      final_SYS = peaks[0].dc_press; 
      for(int i=mx_id;i>=0;i--) if(peaks[i].ac_amp < peaks[mx_id].ac_amp*0.55){ final_SYS=peaks[i].dc_press; break; }
      final_DIA = peaks[num_peaks-1].dc_press; 
      for(int i=mx_id;i<num_peaks;i++) if(peaks[i].ac_amp < peaks[mx_id].ac_amp*0.82){ final_DIA=peaks[i].dc_press; break; }
    }
    final_BPM = round(60.0 * (num_peaks - 1) / (peaks[num_peaks-1].time_s - peaks[0].time_s));
  } else {
    final_SYS = 0; final_DIA = 0; final_BPM = 0; 
  }
  delete[] pressure; delete[] dc; delete[] ac; 
}


void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_OFF);
  
  pinMode(BTN_TIME,  INPUT_PULLUP);
  pinMode(BTN_MEM,   INPUT_PULLUP);
  pinMode(BTN_UP,    INPUT_PULLUP);
  pinMode(BTN_DOWN,  INPUT_PULLUP);
  pinMode(BTN_USER,  INPUT_PULLUP); 
  
  pinMode(BTN_START, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(BTN_START), isrStart, FALLING);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) Serial.println(F("OLED failed"));
  display.clearDisplay(); display.setTextColor(WHITE);
  display.setCursor(15, 30); display.println("Booting System..."); display.display();

  current_user = (digitalRead(BTN_USER) == LOW) ? 1 : 2;
  loadDataFromFlash(current_user); 

  ps.initialize(PIN_SCK, PIN_DOUT);
  pinMode(PIN_PUMP_IN1, OUTPUT); pinMode(PIN_PUMP_IN2, OUTPUT);
  pinMode(PIN_VALVE_IN4, OUTPUT); pinMode(PIN_VALVE_IN3, OUTPUT);
  digitalWrite(PIN_VALVE_IN3, 0); setValve(0); pumpPWM(0); 
  
  display.clearDisplay(); display.setCursor(10, 30); display.println("Calibrating Zero..."); display.display();
  long sum = 0;
  for(int i=0; i<30; i++) {
    while(!ps.isReady());
    ps.readAndSelectNextData(HX710_DIFFERENTIAL_INPUT_40HZ);
    sum += ps.getLastDifferentialInput();
    delay(10);
  }
  zero_offset = sum / 30;
  
  flag_start = false;
  updateRTC();
  last_interaction_time = millis();
}


void loop() {
  
  int new_user = (digitalRead(BTN_USER) == LOW) ? 1 : 2;
  if (new_user != current_user) {
    current_user = new_user;
    loadDataFromFlash(current_user); 
    currentMemIndex = 0;
    last_interaction_time = millis();
  }

  if (currentState == IDLE || currentState == RESULT) {
    if (millis() - last_interaction_time > 15000 && upload_step == 0) { // Không sleep khi đang upload
      enterSleepMode();
    }
  }

  switch (currentState) {
    case IDLE: {
      drawIdleScreen();
      
      if (buttonPressed(BTN_UP)) {
        current_pwm = min(255, current_pwm + 1);
        preferences.putInt("saved_pwm", current_pwm);
      }
      if (buttonPressed(BTN_DOWN)) {
        current_pwm = max(0, current_pwm - 1);
        preferences.putInt("saved_pwm", current_pwm);
      }
      
      if (buttonPressed(BTN_TIME)) { timeSetStep = 0; currentState = SET_TIME; }
      if (buttonPressed(BTN_MEM))  { currentMemIndex = 0; currentState = MEMORY_VIEW; }
      
      if (flag_start) { 
         flag_start = false; 
         last_interaction_time = millis();
         currentState = MEASURING; 
      }
      
      delay(50); 
      break;
    }
    
    case SET_TIME: {
      drawSetTimeScreen();
      
      if (buttonPressed(BTN_UP)) {
        if (timeSetStep == 0) t_day = (t_day >= 31) ? 1 : t_day + 1;
        else if (timeSetStep == 1) t_mon = (t_mon >= 12) ? 1 : t_mon + 1;
        else if (timeSetStep == 2) t_year++;
        else if (timeSetStep == 3) t_hr = (t_hr >= 23) ? 0 : t_hr + 1;
        else if (timeSetStep == 4) t_min = (t_min >= 59) ? 0 : t_min + 1;
      }
      if (buttonPressed(BTN_DOWN)) {
        if (timeSetStep == 0) t_day = (t_day <= 1) ? 31 : t_day - 1;
        else if (timeSetStep == 1) t_mon = (t_mon <= 1) ? 12 : t_mon - 1;
        else if (timeSetStep == 2) t_year--;
        else if (timeSetStep == 3) t_hr = (t_hr <= 0) ? 23 : t_hr - 1;
        else if (timeSetStep == 4) t_min = (t_min <= 0) ? 59 : t_min - 1;
      }
      
      if (buttonPressed(BTN_TIME)) {
        timeSetStep++;
        if (timeSetStep > 4) { updateRTC(); currentState = IDLE; }
      }
      
      if (flag_start) { flag_start = false; currentState = IDLE; }
      last_interaction_time = millis(); 
      break;
    }
    
    case MEMORY_VIEW: {
      drawMemoryScreen();
      if (buttonPressed(BTN_UP) && currentMemIndex > 0) currentMemIndex--;
      if (buttonPressed(BTN_DOWN) && currentMemIndex < historyCount - 1) currentMemIndex++;
      
      if (flag_start || buttonPressed(BTN_MEM)) { 
        flag_start = false; 
        currentState = IDLE; 
      }
      break;
    }

    case MEASURING: {
      setValve(255); 
      delay(100); 
      pumpPWM(255); 
      
      float p = 0; unsigned long last_update = millis();
      while (p < 100.0) {
        if (checkEmergencyStop()) break;
        if (ps.isReady()) {
          p = getPressure();
          if (millis() - last_update > 150) { drawMeasuringScreen((int)p, "PUMPING..."); last_update = millis(); }
        }
      }
      if(currentState == IDLE) break; 

      pumpPWM(210); setValve(255);
      const int W_SIZE = 40; float buf[W_SIZE]; int idx=0, cnt=0, no_pulse=0;
      while (p < 200.0) { 
        if (checkEmergencyStop()) break;
        if (ps.isReady()) {
          p = getPressure(); buf[idx] = p; idx = (idx + 1) % W_SIZE; cnt++;
          if (millis() - last_update > 150) { drawMeasuringScreen((int)p, "DETECTING SYS..."); last_update = millis(); }
          if (cnt >= W_SIZE) {
            float max_jmp = 0;
            for (int i=1; i<W_SIZE; i++) {
               float jump = abs(buf[(idx - i + W_SIZE)%W_SIZE] - buf[(idx - i - 1 + W_SIZE)%W_SIZE]);
               if (max_jmp < jump) max_jmp = jump;
            }
            if (max_jmp < 0.7) no_pulse++; else no_pulse = 0; 
            if (no_pulse > 30) {
               float p_target = p + 10; 
               while(p < p_target) { if(checkEmergencyStop()) break; p = getPressure(); }
               break; 
            }
          }
        }
      }
      if(currentState == IDLE) break;

      pumpPWM(0); setValve(current_pwm);
      sample_count = 0; 
      while (true) {
        if (checkEmergencyStop()) break; 
        if (ps.isReady()) {
          ps.readAndSelectNextData(HX710_DIFFERENTIAL_INPUT_40HZ);
          long raw = ps.getLastDifferentialInput();
          unsigned long now = millis();
          float current_p = ((raw - zero_offset) / SCALE_FACTOR) * SCALE_CORRECTION;
          
          if (sample_count < MAX_SAMPLES) { adc_buffer[sample_count] = raw; time_buffer[sample_count] = now; sample_count++; }
          if (now - last_update > 200) { drawMeasuringScreen((int)current_p, "DEFLATING..."); last_update = now; }

          if (current_p < 80) setValve(current_pwm - 5);
          else if (current_p < 120) setValve(current_pwm - 3);
          else setValve(current_pwm); 

          if (current_p > 0 && current_p < 55) break; 
        }
      }
      if(currentState == IDLE) break;

      setValve(0); delay(500); 
      display.clearDisplay(); display.setCursor(10, 30); display.print("Analyzing..."); display.display();
      
      calculateBloodPressure();
      
      if (final_SYS > 0 && final_DIA > 0) {
        saveRecord(final_SYS, final_DIA, final_BPM, current_user); 
        
        
        upload_step = 1; 
        tb_status_msg = ""; 
        
        currentState = RESULT;
      } else {
        drawErrorScreen("Measurement Failed!");
        delay(3000); currentState = IDLE;
      }
      last_interaction_time = millis(); 
      break;
    }
    
    case RESULT: {
      if (upload_step == 1) {
      
        tb_status_msg = "connecting";
        drawResultScreen(final_SYS, final_DIA, final_BPM, tb_status_msg); 
        
        WiFi.mode(WIFI_STA);
        WiFi.begin(WIFI_SSID, WIFI_PASS);
        
        unsigned long startAttempt = millis();
        bool wifi_ok = false;

        while (millis() - startAttempt < 6000) {
          if (WiFi.status() == WL_CONNECTED) {
            wifi_ok = true;
            break;
          }
          delay(100);
        }
        
        if (wifi_ok) {
          tb_status_msg = "connected";
          drawResultScreen(final_SYS, final_DIA, final_BPM, tb_status_msg);
          delay(300); 
          
          mqttClient.setServer(TB_SERVER, 1883);
         
          if (mqttClient.connect("ESP32_BloodMonitor", TB_TOKEN, NULL)) {
           
            String payload = "{";
            if (current_user == 1) {
              payload += "\"user1_sys\":" + String(final_SYS) + ",";
              payload += "\"user1_dia\":" + String(final_DIA) + ",";
              payload += "\"user1_bpm\":" + String(final_BPM);
            } else {
              payload += "\"user2_sys\":" + String(final_SYS) + ",";
              payload += "\"user2_dia\":" + String(final_DIA) + ",";
              payload += "\"user2_bpm\":" + String(final_BPM);
            }
            payload += "}";
            

            if (mqttClient.publish("v1/devices/me/telemetry", payload.c_str())) {
              tb_status_msg = "fetched"; 
            } else {
              tb_status_msg = "tb error";
            }
            mqttClient.disconnect();
          } else {
            tb_status_msg = "tb error";
          }
        } else {
          tb_status_msg = "wifi fail";
        }
        

        WiFi.disconnect(true, true);
        WiFi.mode(WIFI_OFF);
        
       
        drawResultScreen(final_SYS, final_DIA, final_BPM, tb_status_msg);
        upload_step = 0; 
        last_interaction_time = millis();
      } else {

        drawResultScreen(final_SYS, final_DIA, final_BPM, tb_status_msg);
      }
      
     
      if (flag_start) {
        flag_start = false;
        last_interaction_time = millis();
        currentState = IDLE;
      }
      break;
    }
  }
}

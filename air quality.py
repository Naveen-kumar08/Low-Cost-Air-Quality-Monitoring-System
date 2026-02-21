// ===== BLYNK SETTINGS =====
#define BLYNK_TEMPLATE_ID   "TMPL34ncqjSrD"
#define BLYNK_TEMPLATE_NAME "Air quality monitoring system"
#define BLYNK_AUTH_TOKEN    "DbUdWzh-CJjfzY76DXTSenVtM83PsPuF"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <LiquidCrystal.h>
#include <DHT.h>
#include <SPI.h>
#include <SD.h>
#include <RTClib.h>
#include <math.h>

// ===== RTC =====
RTC_DS3231 rtc;

// ===== LCD PINS =====
LiquidCrystal lcd(21, 22, 26, 27, 25, 33);

// ===== DHT SENSOR =====
#define DHTPIN 32
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// ===== LED & BUZZER PINS =====
#define GREEN 4
#define YELLOW 12
#define RED 13
#define BUZZER 15

// ===== MQ SENSOR PINS =====
#define MQ135PIN 34
#define MQ2PIN   35
#define MQ4PIN   36
#define MQ9PIN   39

#define RL_VALUE 1
#define CALIBRATION_SAMPLE_TIMES 30
#define READ_SAMPLE_INTERVAL 50
#define READ_SAMPLE_TIMES 5
#define WARMUP_TIME 600000UL // 10 minutes in ms

// ===== SENSOR VARIABLES =====
float Ro_MQ135 = 10;
float Ro_MQ2   = 10;
float Ro_MQ4   = 10;
float Ro_MQ9   = 10;

// ===== GAS THRESHOLDS =====
struct GasThreshold { int normal; int moderate; int high; };
GasThreshold mq135_CO2   = {400, 800, 1200};
GasThreshold mq2_LPG     = {50, 150, 300};
GasThreshold mq2_Smoke   = {50, 200, 400};
GasThreshold mq4_CH4     = {50, 200, 400};
GasThreshold mq9_CO      = {10, 50, 100};

// ===== WIFI =====
char ssid[] = "Naveen";
char pass[] = "12345678";

// ===== GLOBAL VARIABLES =====
unsigned long lastSensorTime = 0;
const int sensorInterval = 2000;
int sensorIndex = 0;
unsigned long warmupStart = 0;
bool warmupDone = false;

// Smoothed sensor values
float CO2_avg=0, LPG_avg=0, Smoke_avg=0, CH4_avg=0, CO9_avg=0;
float t_val=0, h_val=0;

// ===== FUNCTIONS =====
float MQResistanceCalculation(int raw_adc) {
  if(raw_adc <= 0) raw_adc = 1;
  return ((float)RL_VALUE * (4095 - raw_adc) / raw_adc);
}

float MQRead(int pin){
  float rs=0;
  for(int i=0;i<READ_SAMPLE_TIMES;i++){
    rs += MQResistanceCalculation(analogRead(pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  return rs/READ_SAMPLE_TIMES;
}

float MQCalibration(int pin, float clean_air_factor){
  float RS_AIR_val=0;
  for(int i=0;i<CALIBRATION_SAMPLE_TIMES;i++){
    RS_AIR_val += MQResistanceCalculation(analogRead(pin));
    delay(READ_SAMPLE_INTERVAL);
  }
  RS_AIR_val /= CALIBRATION_SAMPLE_TIMES;
  return RS_AIR_val / clean_air_factor;
}

float smooth(float newVal,float oldVal){
  return (0.7*oldVal)+(0.3*newVal);
}

String GasQuality(float value, GasThreshold thresh){
  if(value <= thresh.normal) return "Normal";
  else if(value <= thresh.moderate) return "Moderate";
  else return "High";
}

// ===== LED & Buzzer =====
void WarningLED(String quality){
  digitalWrite(GREEN, LOW);
  digitalWrite(YELLOW, LOW);
  digitalWrite(RED, LOW);
  digitalWrite(BUZZER, LOW);

  if(quality=="Normal"){
    digitalWrite(GREEN,HIGH);
    Blynk.virtualWrite(V9,255);
    Blynk.virtualWrite(V10,0);
    Blynk.virtualWrite(V11,0);
    Blynk.virtualWrite(V12,0);
  } else if(quality=="Moderate"){
    digitalWrite(YELLOW,HIGH);
    digitalWrite(BUZZER,HIGH); delay(150); digitalWrite(BUZZER,LOW);
    Blynk.virtualWrite(V9,0);
    Blynk.virtualWrite(V10,255);
    Blynk.virtualWrite(V11,0);
    Blynk.virtualWrite(V12,255);
  } else if(quality=="High"){
    digitalWrite(RED,HIGH);
    digitalWrite(BUZZER,HIGH);
    Blynk.virtualWrite(V9,0);
    Blynk.virtualWrite(V10,0);
    Blynk.virtualWrite(V11,255);
    Blynk.virtualWrite(V12,255);
  }
}

// ===== SD SAVE & LOAD Ro =====
void SaveRoToSD() {
  File f = SD.open("/RoValues.txt", FILE_WRITE);
  if(f){
    f.println(Ro_MQ135);
    f.println(Ro_MQ2);
    f.println(Ro_MQ4);
    f.println(Ro_MQ9);
    f.close();
    Serial.println("Ro values saved to SD");
  } else Serial.println("Failed to save Ro to SD");
}

void LoadRoFromSD() {
  if(!SD.begin(5)){ Serial.println("SD fail"); return; }
  File f = SD.open("/RoValues.txt");
  if(f){
    Ro_MQ135 = f.parseFloat();
    Ro_MQ2   = f.parseFloat();
    Ro_MQ4   = f.parseFloat();
    Ro_MQ9   = f.parseFloat();
    f.close();
    Serial.println("Ro values loaded from SD");
    warmupDone = true;
  } else Serial.println("Ro file not found, 10m warmup needed");
}

// ===== TEMP & HUM COMPENSATION =====
float TempHumidityComp(float rs_ro_ratio, float temperature, float humidity){
  // Simple compensation using formula: ratio adjusted by temp & hum
  float ratio = rs_ro_ratio;
  ratio = ratio * (1 + 0.01*(temperature-20)) * (1 + 0.01*(humidity-65)); 
  return ratio;
}

// ===== SETUP =====
void setup(){
  Serial.begin(115200);

  pinMode(GREEN, OUTPUT);
  pinMode(YELLOW, OUTPUT);
  pinMode(RED, OUTPUT);
  pinMode(BUZZER, OUTPUT);

  dht.begin();
  rtc.begin();
  SD.begin(5);

  lcd.begin(16,2);
  lcd.clear();
  lcd.print("Connecting WiFi");
  WiFi.begin(ssid, pass);

  unsigned long startTime = millis();
  while(WiFi.status() != WL_CONNECTED && millis()-startTime < 10000){
    lcd.setCursor(0,1); lcd.print(".");
    delay(500);
  }

  lcd.clear();
  if(WiFi.status()==WL_CONNECTED){
    lcd.print("WiFi Connected");
    Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
    delay(2000);
  } else lcd.print("WiFi Failed");

  // Load Ro if saved
  LoadRoFromSD();

  warmupStart = millis();
}

// ===== LOOP =====
void loop(){
  Blynk.run();
  unsigned long currentMillis = millis();

  // ===== Warmup countdown =====
  if(!warmupDone){
    unsigned long elapsed = currentMillis - warmupStart;
    if(elapsed >= WARMUP_TIME){
      warmupDone = true;
      lcd.clear(); lcd.print("Warmup Done"); delay(1000);

      lcd.clear(); lcd.print("Calibrating...");
      Ro_MQ135 = MQCalibration(MQ135PIN, 3.59);
      Ro_MQ2   = MQCalibration(MQ2PIN, 9.577);
      Ro_MQ4   = MQCalibration(MQ4PIN, 4.434);
      Ro_MQ9   = MQCalibration(MQ9PIN, 9.799);

      SaveRoToSD();
      lcd.clear(); lcd.print("Calibration OK"); delay(1000);
    } else {
      int remaining = (WARMUP_TIME - elapsed)/1000;
      lcd.setCursor(0,0); lcd.print("MQ Warmup 10m   ");
      lcd.setCursor(0,1); lcd.print(remaining/60); lcd.print("m ");
      lcd.print(remaining%60); lcd.print("s   ");
    }
  } else {
    // ===== Sensor reading every 2 sec =====
    if(currentMillis - lastSensorTime >= sensorInterval){
      lastSensorTime = currentMillis;
      sensorIndex++; if(sensorIndex>5) sensorIndex=0;

      float t = dht.readTemperature();
      float h = dht.readHumidity();
      if(!isnan(t)) t_val = t;
      if(!isnan(h)) h_val = h;

      float r135 = TempHumidityComp(MQRead(MQ135PIN)/Ro_MQ135, t_val, h_val);
      float CO2  = fmax(0.0, pow(10,((-2.890*log10(r135))+2.055)));

      float r2 = TempHumidityComp(MQRead(MQ2PIN)/Ro_MQ2, t_val, h_val);
      float LPG   = fmax(0.0, pow(10,((-2.123*log10(r2))+2.758)));
      float Smoke = fmax(0.0, pow(10,((-2.331*log10(r2))+3.596)));

      float r4 = TempHumidityComp(MQRead(MQ4PIN)/Ro_MQ4, t_val, h_val);
      float CH4 = fmax(0.0, pow(10,((-2.849*log10(r4))+2.997)));

      float r9 = TempHumidityComp(MQRead(MQ9PIN)/Ro_MQ9, t_val, h_val);
      float CO9 = fmax(0.0, pow(10,((-2.199*log10(r9))+2.766)));

      // Smooth values
      CO2_avg   = smooth(CO2, CO2_avg);
      LPG_avg   = smooth(LPG, LPG_avg);
      Smoke_avg = smooth(Smoke, Smoke_avg);
      CH4_avg   = smooth(CH4, CH4_avg);
      CO9_avg   = smooth(CO9, CO9_avg);

      // ===== LCD & Blynk display =====
      lcd.clear();
      switch(sensorIndex){
        case 0:
          lcd.print("Temp:"); lcd.print(t_val,1); lcd.print("C");
          lcd.setCursor(0,1); lcd.print("Hum:"); lcd.print(h_val,0); lcd.print("%");
          Blynk.virtualWrite(V7, t_val); Blynk.virtualWrite(V8, h_val); break;
        case 1:
          lcd.print("MQ2 LPG:"); lcd.print((int)LPG_avg); lcd.print("ppm");
          lcd.setCursor(0,1); lcd.print(GasQuality(LPG_avg,mq2_LPG));
          Blynk.virtualWrite(V2, LPG_avg); WarningLED(GasQuality(LPG_avg,mq2_LPG)); break;
        case 2:
          lcd.print("MQ2 Smoke:"); lcd.print((int)Smoke_avg); lcd.print("ppm");
          lcd.setCursor(0,1); lcd.print(GasQuality(Smoke_avg,mq2_Smoke));
          Blynk.virtualWrite(V3, Smoke_avg); WarningLED(GasQuality(Smoke_avg,mq2_Smoke)); break;
        case 3:
          lcd.print("MQ4 CH4:"); lcd.print((int)CH4_avg); lcd.print("ppm");
          lcd.setCursor(0,1); lcd.print(GasQuality(CH4_avg,mq4_CH4));
          Blynk.virtualWrite(V4, CH4_avg); WarningLED(GasQuality(CH4_avg,mq4_CH4)); break;
        case 4:
          lcd.print("MQ9 CO:"); lcd.print((int)CO9_avg); lcd.print("ppm");
          lcd.setCursor(0,1); lcd.print(GasQuality(CO9_avg,mq9_CO));
          Blynk.virtualWrite(V6, CO9_avg); WarningLED(GasQuality(CO9_avg,mq9_CO)); break;
        case 5:
          lcd.print("MQ135 CO2:"); lcd.print((int)CO2_avg); lcd.print("ppm");
          lcd.setCursor(0,1); lcd.print(GasQuality(CO2_avg,mq135_CO2));
          Blynk.virtualWrite(V0, CO2_avg); WarningLED(GasQuality(CO2_avg,mq135_CO2)); break;
      }
    }

    // ===== SD Logging every 10 sec =====
    static unsigned long lastLog=0;
    if(millis()-lastLog>=10000){
      lastLog=millis();
      DateTime now = rtc.now();
      File f = SD.open("/airlog.txt", FILE_APPEND);
      if(f){
        f.print(now.hour()); f.print(":"); f.print(now.minute()); f.print(",");
        f.print("CO2="); f.print((int)CO2_avg); f.print("ppm,");
        f.print("Smoke="); f.print((int)Smoke_avg); f.print("ppm,");
        f.print("LPG="); f.print((int)LPG_avg); f.print("ppm,");
        f.print("CH4="); f.print((int)CH4_avg); f.print("ppm,");
        f.print("CO9="); f.print((int)CO9_avg); f.print("ppm,");
        f.print("T="); f.print(t_val,1); f.print("C,");
        f.print("H="); f.print(h_val,0); f.print("%");
        f.println(); f.close();
      }
    }
  }
}

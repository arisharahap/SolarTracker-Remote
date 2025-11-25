//yg pake blynk
/*
 * =================================================================
 * SKEMA PROGRAM SOLAR TRACKER DUAL AXIS (VERSI 6.1 - BLYNK ON-DEMAND)
 * =================================================================
 * Fitur:
 * - Pengiriman data ke Blynk diubah menjadi on-demand (Tombol Scan V1)
 * - Update LCD tetap berjalan 1 detik sekali.
 * - (Fitur lain dari v6.0 tetap ada)
 */

// --- Pustaka (Libraries) ---
// ... (Semua #include tetap sama) ...
#include <Wire.h>
#include <AccelStepper.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include "MPU9250.h" 
#include <Adafruit_INA219.h>    
#include <LiquidCrystal_I2C.h> 
#include <WiFi.h>                
#include <BlynkSimpleEsp32.h>    

// --- KREDENSIAL (WAJIB DIISI) ---
#define BLYNK_TEMPLATE_ID "TMPL_xxxxxxx"
#define BLYNK_TEMPLATE_NAME "Solar Tracker"
#define BLYNK_AUTH_TOKEN "TOKEN_DARI_EMAIL_ANDA"
char ssid[] = "NAMA_WIFI_RUMAH_ANDA";
char pass[] = "PASSWORD_WIFI_RUMAH_ANDA";

// --- Konfigurasi Pin (Tidak berubah) ---
// ... (Salin semua pin define) ...
#define STEP_PIN_AZ 21//14 
#define DIR_PIN_AZ  14//12 
#define STEP_PIN_EL 10//27 
#define DIR_PIN_EL  11//26 
#define GPS_RX_PIN  18 
#define GPS_TX_PIN  17 
#define LDR_PIN_E   4//35 
#define LDR_PIN_W   5//34 
#define LDR_PIN_N   6//32 
#define LDR_PIN_S   7//33 
#define MODE_SWITCH_PIN   13
#define MANUAL_UP_PIN     15
#define MANUAL_DOWN_PIN   2 
#define MANUAL_LEFT_PIN   4 
#define MANUAL_RIGHT_PIN  5

// --- Definisi Mode (Tidak berubah) ---
#define MODE_AUTO_HYBRID      0 
#define MODE_MANUAL_BUTTONS   1 
#define MODE_FORCE_LDR        2 
#define MODE_FORCE_GPS        3 

// --- Objek Global (Tidak berubah) ---
// ... (Salin semua objek global) ...
AccelStepper stepperAzimuth(AccelStepper::DRIVER, STEP_PIN_AZ, DIR_PIN_AZ); 
AccelStepper stepperElevation(AccelStepper::DRIVER, STEP_PIN_EL, DIR_PIN_EL); 
TinyGPSPlus gps; 
MPU9250 imu(Wire, 0x68); 
Adafruit_INA219 ina219(0x40);
LiquidCrystal_I2C lcd(0x27, 20, 4);
BlynkTimer timer; 

// --- Variabel Global untuk Data (Tidak berubah) ---
// ... (Salin semua variabel global) ...
float currentAzimuth = 0.0;
float currentElevation = 0.0;
float targetAzimuth = 0.0;
float targetElevation = 0.0;
float busVoltage = 0.0;
float current_mA = 0.0;
float power_W = 0.0;
int ldrE = 0, ldrW = 0, ldrN = 0, ldrS = 0;
int currentMode = MODE_AUTO_HYBRID;   
unsigned long lastModeSwitchTime = 0; 
const long debounceDelay = 50;        
unsigned long lastLcdUpdate = 0; 
const long dataUpdateInterval = 1000; // Hanya untuk LCD
char lcdBuffer[21];                   
unsigned long lastAstroUpdate = 0; 
const long astroUpdateInterval = 1800000; 
bool virtualBtnMode = false; 
bool virtualBtnUp = false;   
bool virtualBtnDown = false; 
bool virtualBtnLeft = false; 
bool virtualBtnRight = false; 

// --- Konstanta untuk Tuning (Tidak berubah) ---
// ... (Salin semua konstanta) ...
const int LDR_TOLERANCE    = 50; 
const int CLOUDY_THRESHOLD = 300; 
#define ANGLE_TOLERANCE  0.5   
#define ASTRO_GAIN_AZ    50.0  
#define ASTRO_GAIN_EL    50.0  
#define LDR_GAIN_AZ      1.0
#define LDR_GAIN_EL      1.0
#define MANUAL_SPEED     500   

// =================================================================
//   SETUP: Inisialisasi Sistem
// =================================================================
void setup() {
  Serial.begin(115200);
  Wire.begin(); 
  
  // 1. Inisialisasi LCD
  lcd.init();      
  lcd.backlight(); 
  lcd.setCursor(0, 0);
  lcd.print("Solar Tracker v6.1"); // Versi baru
  lcd.setCursor(0, 1);
  lcd.print("Connecting WiFi...");

  // 2. Inisialisasi Tombol Fisik
  // ... (Salin semua pinModes) ...
  pinMode(MODE_SWITCH_PIN, INPUT_PULLUP); 
  pinMode(MANUAL_UP_PIN, INPUT_PULLUP);   
  pinMode(MANUAL_DOWN_PIN, INPUT_PULLUP); 
  pinMode(MANUAL_LEFT_PIN, INPUT_PULLUP); 
  pinMode(MANUAL_RIGHT_PIN, INPUT_PULLUP);
  
  // 3. Inisialisasi Sensor & Motor
  // ... (Salin inisialisasi sensor) ...
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN); 
  if (!imu.begin()) { Serial.println("Gagal MPU"); while (1); }
  if (!ina219.begin()) { Serial.println("Gagal INA"); while (1); }
  stepperAzimuth.setMaxSpeed(400);   
  stepperAzimuth.setAcceleration(200); 
  stepperElevation.setMaxSpeed(400); 
  stepperElevation.setAcceleration(200); 
  runHomingSequence(); 
  
  // 4. Inisialisasi Blynk
  // ... (Salin inisialisasi Blynk) ...
  Serial.println("Menghubungkan ke WiFi...");
  WiFi.begin(ssid, pass);
  Blynk.config(BLYNK_AUTH_TOKEN);
  Blynk.connect(); 
  if (Blynk.connected()) {
    Serial.println("Terhubung ke Blynk!");
    lcd.setCursor(0, 1);
    lcd.print("WiFi Terhubung!     ");
    lcd.setCursor(0, 2);
    lcd.print("Blynk Online!       ");
  } else {
    Serial.println("Gagal terhubung ke Blynk.");
    lcd.setCursor(0, 1);
    lcd.print("WiFi/Blynk Gagal!   ");
  }
  
  // 5. Setup Timer (DIUBAH)
  timer.setInterval(50L, runTrackerLogic); // Loop logika utama setiap 50ms
  timer.setInterval(1000L, updateLCD); // <-- DIUBAH: Timer ini HANYA update LCD
                                       // (Fungsi sendDataToBlynk DIHAPUS dari timer)
  delay(1000); 
  lcd.clear(); 
}

// =================================================================
//   LOOP: Logika Utama (Tidak Berubah)
// =================================================================
void loop() {
  Blynk.run(); // Wajib untuk menjaga koneksi Blynk
  timer.run(); // Wajib untuk menjalankan semua fungsi timer
}

// =================================================================
//   FUNGSI LOGIKA UTAMA (Tidak Berubah)
// =================================================================
/**
 * @brief Ini adalah pengganti loop() utama. 
 * Semua logika inti tracker dipanggil dari sini oleh timer.
 */
void runTrackerLogic() {
  // ... (Fungsi ini tidak berubah sama sekali) ...
  unsigned long currentMillis = millis(); 
  updateGps();
  updateIMU();
  readLDRs();
  updateINA219();
  handleModeSwitch(currentMillis); 
  switch (currentMode) {
    case MODE_AUTO_HYBRID:
      int avgLight = (ldrN + ldrS + ldrW + ldrE) / 4; 
      if (avgLight > CLOUDY_THRESHOLD) { 
        runLdrMode();
      } else {
        if (currentMillis - lastAstroUpdate >= astroUpdateInterval) { 
          lastAstroUpdate = currentMillis;
          updateAstroTarget(); 
        }
        runAstroPControl(); 
      }
      break;
    case MODE_MANUAL_BUTTONS:
      runManualMode(); 
      break;
    case MODE_FORCE_LDR:
      runLdrMode(); 
      break;
    case MODE_FORCE_GPS:
      if (currentMillis - lastAstroUpdate >= astroUpdateInterval) { 
        lastAstroUpdate = currentMillis;
        updateAstroTarget(); 
      }
      runAstroPControl(); 
      break;
  }
  stepperAzimuth.runSpeed();
  stepperElevation.runSpeed();
}


// =================================================================
//   FUNGSI INTEGRASI BLYNK (DIUBAH)
// =================================================================

/**
 * @brief (BARU) Mengirim data sensor ke Blynk (On-Demand)
 * Dipanggil oleh Tombol Scan V1
 */
void sendBlynkDataNow() {
  // 1. Kirim Mode
  Blynk.virtualWrite(V0, getModeString());
  
  // 2. Kirim Data Sensor
  Blynk.virtualWrite(V2, busVoltage);
  Blynk.virtualWrite(V3, current_mA / 1000.0);
  Blynk.virtualWrite(V4, power_W);
  Blynk.virtualWrite(V5, currentElevation);
  Blynk.virtualWrite(V6, currentAzimuth);
  
  // 3. (V1 tidak lagi dikirim, karena V1 adalah tombol scan)
}


/**
 * @brief (DIHAPUS) Fungsi 'sendDataToBlynk()' yang lama dihapus
 * karena telah digantikan oleh 'sendBlynkDataNow()' (on-demand)
 * dan 'updateLCD()' (on-timer).
 */
// void sendDataToBlynk() { ... } // <-- DIHAPUS


/**
 * @brief (Tidak Berubah) Fungsi ini dipanggil saat ESP terhubung ke Blynk
 */
BLYNK_CONNECTED() {
  Blynk.syncVirtual(V10, V11, V12, V13, V14);
}

/**
 * @brief (BARU) Menerima perintah dari Tombol Scan (V1)
 */
BLYNK_WRITE(V1) {
  if (param.asInt() == 1) { // Tombol "Push" ditekan
    Serial.println("Perintah Scan (V1) diterima. Mengirim data ke Blynk...");
    sendBlynkDataNow(); // Panggil fungsi pengiriman data
  }
}

/**
 * @brief (Tidak Berubah) Menerima perintah dari Tombol Mode (V10)
 */
BLYNK_WRITE(V10) {
  if (param.asInt() == 1) { 
    if (millis() - lastModeSwitchTime > debounceDelay) {
      switchMode(); 
      lastModeSwitchTime = millis();
    }
  }
}

// --- (Tidak Berubah) Menerima perintah dari Tombol Manual (V11-V14) ---
BLYNK_WRITE(V11) { virtualBtnUp = param.asInt(); }
BLYNK_WRITE(V12) { virtualBtnDown = param.asInt(); }
BLYNK_WRITE(V13) { virtualBtnLeft = param.asInt(); }
BLYNK_WRITE(V14) { virtualBtnRight = param.asInt(); }


// =================================================================
//   FUNGSI-FUNGSI MODE (DIUBAH: switchMode)
// =================================================================

/**
 * @brief (DIUBAH) Fungsi terpusat untuk mengganti mode
 */
void switchMode() {
  currentMode++; 
  if (currentMode > MODE_FORCE_GPS) { 
    currentMode = MODE_AUTO_HYBRID;
  }
  
  stepperAzimuth.setSpeed(0);
  stepperElevation.setSpeed(0);
  stepperAzimuth.stop(); 
  stepperElevation.stop();
  
  Serial.print("MODE BERUBAH KE: ");
  Serial.println(getModeString());
  
  // Paksa update data segera
  sendBlynkDataNow(); // <-- DIUBAH: Panggil fungsi baru
  updateLCD();        // Panggil updateLCD secara manual saat mode ganti
}

// --- (Sisa fungsi tidak berubah) ---
// (runManualMode, getModeString, runLdrMode, updateAstroTarget, 
//  runAstroPControl, updateLCD, dan semua fungsi helper
//  TIDAK BERUBAH dari kode v6.0)
// ... (Salin semua fungsi sisanya dari kode sebelumnya) ...
void runManualMode() {
  if (digitalRead(MANUAL_UP_PIN) == LOW || virtualBtnUp) {
    stepperElevation.setSpeed(MANUAL_SPEED);
  } 
  else if (digitalRead(MANUAL_DOWN_PIN) == LOW || virtualBtnDown) {
    stepperElevation.setSpeed(-MANUAL_SPEED);
  }
  else {
    stepperElevation.setSpeed(0);
  }
  if (digitalRead(MANUAL_LEFT_PIN) == LOW || virtualBtnLeft) { 
    stepperAzimuth.setSpeed(-MANUAL_SPEED); 
  }
  else if (digitalRead(MANUAL_RIGHT_PIN) == LOW || virtualBtnRight) { 
    stepperAzimuth.setSpeed(MANUAL_SPEED); 
  }
  else {
    stepperAzimuth.setSpeed(0);
  }
}
const char* getModeString() {
  switch (currentMode) {
    case MODE_AUTO_HYBRID:    return "AUTO";
    case MODE_MANUAL_BUTTONS: return "MANU";
    case MODE_FORCE_LDR:      return "LDR ";
    case MODE_FORCE_GPS:      return "GPS ";
    default:                  return "ERR ";
  }
}
void runLdrMode() {
  int errorVertical = ldrN - ldrS; 
  int errorHorizontal = ldrW - ldrE; 
  if (abs(errorHorizontal) > LDR_TOLERANCE) stepperAzimuth.setSpeed(errorHorizontal * LDR_GAIN_AZ); 
  else stepperAzimuth.setSpeed(0);
  if (abs(errorVertical) > LDR_TOLERANCE) stepperElevation.setSpeed(errorVertical * LDR_GAIN_EL); 
  else stepperElevation.setSpeed(0);
}
void updateAstroTarget() {
  if (!gps.location.isValid() || !gps.date.isValid() || !gps.time.isValid()) return;
  calculateSolarPosition(
    gps.location.lat(), gps.location.lng(),
    gps.date.year(), gps.date.month(), gps.date.day(),
    gps.time.hour(), gps.time.minute(), gps.time.second(),
    targetAzimuth, targetElevation 
  );
  Serial.println("=============== TARGET POSISI ASTRO DIPERBARUI ===============");
}
void runAstroPControl() {
  float errorAz = targetAzimuth - currentAzimuth;
  float errorEl = targetElevation - currentElevation;
  if (abs(errorAz) > ANGLE_TOLERANCE) stepperAzimuth.setSpeed(errorAz * ASTRO_GAIN_AZ);
  else stepperAzimuth.setSpeed(0); 
  if (abs(errorEl) > ANGLE_TOLERANCE) stepperElevation.setSpeed(errorEl * ASTRO_GAIN_EL);
  else stepperElevation.setSpeed(0); 
}
void updateLCD() { 
  lcd.setCursor(0, 0);
  if (gps.date.isValid() && gps.time.isValid()) {
    sprintf(lcdBuffer, "%02d/%02d/%04d  %02d:%02d:%02d",
            gps.date.day(), gps.date.month(), gps.date.year(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    sprintf(lcdBuffer, "Mencari GPS...      ");
  }
  lcd.print(lcdBuffer);
  lcd.setCursor(0, 1);
  sprintf(lcdBuffer, "V:%5.2fV  A:%5.2fA  ", busVoltage, current_mA / 1000.0);
  lcd.print(lcdBuffer);
  lcd.setCursor(0, 2);
  sprintf(lcdBuffer, "P:%5.2fW            ", power_W);
  lcd.print(lcdBuffer);
  lcd.setCursor(0, 3);
  sprintf(lcdBuffer, "X(El):%4.1f Y(Az):%4.1f %s", 
          currentElevation, 
          currentAzimuth, 
          getModeString());
  lcd.print(lcdBuffer);
}
void updateINA219() { 
  busVoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();
  power_W    = ina219.getPower_mW() / 1000.0;
}
void updateIMU() {
  if (imu.readSensor()) {
    currentElevation = imu.getPitch(); 
    currentAzimuth = imu.getYaw();   
  }
}
void updateGps() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read()); 
  }
}
void readLDRs() {
  ldrE = analogRead(LDR_PIN_E); 
  ldrW = analogRead(LDR_PIN_W); 
  ldrN = analogRead(LDR_PIN_N); 
  ldrS = analogRead(LDR_PIN_S); 
}
void runHomingSequence() {
  Serial.println("Menjalankan Homing Sequence (Placeholder)...");
  Serial.println("Homing Selesai.");
}
void calculateSolarPosition(float lat, float lon, int y, int m, int d, 
                            int h, int min, int s,
                            float &outTargetAz, float &outTargetEl) 
{
  // (Logika perhitungan SPA yang kompleks ada di sini)
  float timezone = lon / 15.0; 
  float localHour = h + min / 60.0 + s / 3600.0; 
  int dayOfYear = 0;
  for(int i = 1; i < m; i++) { 
    if(i == 2) dayOfYear += (y % 4 == 0) ? 29 : 28; 
    else if(i == 4 || i == 6 || i == 9 || i == 11) dayOfYear += 30; 
    else dayOfYear += 31; 
  }
  dayOfYear += d; 
  float B = (360.0 / 365.0) * (dayOfYear - 81); 
  float B_rad = B * DEG_TO_RAD; 
  float eot = 9.87 * sin(2 * B_rad) - 7.53 * cos(B_rad) - 1.5 * sin(B_rad); 
  float lstm = 15 * timezone; 
  float timeCorrection = 4 * (lon - lstm) + eot; 
  float lst = localHour + timeCorrection / 60.0; 
  float hourAngle = 15 * (lst - 12); 
  float declination_rad = 23.45 * sin( (360.0/365.0) * (dayOfYear - 81) * DEG_TO_RAD ) * DEG_TO_RAD; 
  float lat_rad = lat * DEG_TO_RAD; 
  float hourAngle_rad = hourAngle * DEG_TO_RAD; 
  float altitude_rad = asin(sin(lat_rad) * sin(declination_rad) + cos(lat_rad) * cos(declination_rad) * cos(hourAngle_rad)); 
  float azimuth_rad = acos( (sin(declination_rad) - sin(altitude_rad) * sin(lat_rad)) / (cos(altitude_rad) * cos(lat_rad)) ); 
  if (hourAngle > 0) { 
    azimuth_rad = (2 * PI) - azimuth_rad; 
  }
  outTargetEl = altitude_rad * RAD_TO_DEG; 
  outTargetAz = azimuth_rad * RAD_TO_DEG; 
  if (outTargetEl < 0) outTargetEl = 0; 
}
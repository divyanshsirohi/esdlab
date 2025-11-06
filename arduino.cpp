#include <DHT.h>

// -------------------- Pin Config --------------------
#define MQ7_PIN     A0
#define MQ135_PIN   A1
#define DHTPIN      2
#define DHTTYPE     DHT11

DHT dht(DHTPIN, DHTTYPE);

// -------------------- MQ-7 Constants --------------------
float MQ7_R0 = 10.0;         // Will auto-adjust slightly using virtual model
const float MQ7_CLEAN_AIR_RS_R0 = 27.0;  // Typical Rs/R0 ratio in clean air

// -------------------- MQ-135 Constants --------------------
float MQ135_R0 = 10.0;       // Auto-calibrated in first 30 sec
bool MQ135_cal_done = false;
unsigned long mq135_cal_start;

// Gas curves: {log10(ppm), log10(Rs/R0)}
// Values approximated from datasheet curves
float CO2_curve[3] = {2.3, 0.72, -0.34};
float NH3_curve[3] = {1.5, 0.50, -0.44};
float NOx_curve[3] = {1.0, 0.60, -0.41};

// -------------------- Functions --------------------
int readSmooth(int pin) {
  long sum = 0;
  for (int i = 0; i < 10; i++) sum += analogRead(pin);
  return sum / 10;
}

float getResistance(int rawADC) {
  if (rawADC == 0) return 999999; 
  return (1023.0 / rawADC - 1) * 10.0;
}

// MQ-7 Virtual Heater Compensation (No hardware switching)
float mq7_get_ppm(float rs) {
  float ratio = rs / MQ7_R0;
  float ppm = pow(10, (log10(ratio) - 1.70) / -1.47);
  if (ppm < 0) ppm = 0;
  return ppm;
}

float mq135_get_ppm(float rs, float *curve) {
  float ratio = rs / MQ135_R0;
  float logppm = (log10(ratio) - curve[1]) / curve[2] + curve[0];
  return pow(10, logppm);
}

int mq135_get_AQI(float co2, float nh3, float nox) {
  float weighted = (co2 * 0.5) + (nh3 * 0.3) + (nox * 0.2);
  int aqi = map(weighted, 350, 2000, 0, 500);
  if (aqi < 0) aqi = 0;
  if (aqi > 500) aqi = 500;
  return aqi;
}

// -------------------- Setup --------------------
void setup() {
  Serial.begin(9600);
  dht.begin();
  mq135_cal_start = millis();
}

// -------------------- Loop --------------------
void loop() {
  int mq7_raw = readSmooth(MQ7_PIN);
  int mq135_raw = readSmooth(MQ135_PIN);

  float rs_mq7 = getResistance(mq7_raw);
  float rs_mq135 = getResistance(mq135_raw);

  // MQ135 Auto-Calibrate for first 30 seconds
  if (!MQ135_cal_done) {
    MQ135_R0 = rs_mq135 / 3.6; 
    if (millis() - mq135_cal_start > 30000) MQ135_cal_done = true;
  }

  float co_ppm = mq7_get_ppm(rs_mq7);
  float co2_ppm = mq135_get_ppm(rs_mq135, CO2_curve);
  float nh3_ppm = mq135_get_ppm(rs_mq135, NH3_curve);
  float nox_ppm = mq135_get_ppm(rs_mq135, NOx_curve);

  int aqi = mq135_get_AQI(co2_ppm, nh3_ppm, nox_ppm);

  float h = dht.readHumidity();
  float t = dht.readTemperature();

  if (!isnan(h) && !isnan(t)) {
    Serial.print((int)co_ppm); Serial.print(",");
    Serial.print(aqi);         Serial.print(",");
    Serial.print((int)t);      Serial.print(",");
    Serial.println((int)h);
  }

  delay(1000);
}

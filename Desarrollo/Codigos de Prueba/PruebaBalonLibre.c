#include <Wire.h>
#include <Adafruit_TCS34725.h>

#define TRIG_IZQ  22
#define ECHO_IZQ  21
#define TRIG_CENT 18
#define ECHO_CENT 34
#define TRIG_DER  17
#define ECHO_DER  16
#define SDA_PIN   19
#define SCL_PIN   23

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

#define MUESTRAS_POR_SESION 30
#define DELAY_MUESTRA       500

int  muestras_tomadas = 0;
bool sesion_activa    = false;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(TRIG_IZQ,  OUTPUT); pinMode(ECHO_IZQ,  INPUT);
  pinMode(TRIG_CENT, OUTPUT); pinMode(ECHO_CENT, INPUT);
  pinMode(TRIG_DER,  OUTPUT); pinMode(ECHO_DER,  INPUT);

  if (tcs.begin()) Serial.println("# TCS34725 OK");
  else             Serial.println("# TCS34725 ERROR");

  Serial.println("# RECOLECTOR - BALON LIBRE (clase 2)");
  Serial.println("# Pon el balon naranja quieto frente al sensor");
  Serial.println("# 'S' → Iniciar | 'P' → Pausar | 'R' → Reiniciar");
  Serial.println("D_IZQ,D_CENT,D_DER,R,G,B,CLASE");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'S' || cmd == 's') {
      sesion_activa    = true;
      muestras_tomadas = 0;
      Serial.println("# → Sesion iniciada. Balon quieto frente al sensor.");
    }
    if (cmd == 'P' || cmd == 'p') {
      sesion_activa = false;
      Serial.println("# → Pausado.");
    }
    if (cmd == 'R' || cmd == 'r') {
      muestras_tomadas = 0;
      Serial.println("# → Reiniciado.");
    }
  }

  if (!sesion_activa) return;

  if (muestras_tomadas >= MUESTRAS_POR_SESION) {
    sesion_activa = false;
    Serial.printf("# ✓ %d muestras capturadas. Guarda como datos_balon_libre.txt\n", muestras_tomadas);
    return;
  }

  float di = leerDistancia(TRIG_IZQ,  ECHO_IZQ);
  float dc = leerDistancia(TRIG_CENT, ECHO_CENT);
  float dd = leerDistancia(TRIG_DER,  ECHO_DER);

  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  if (di == 0.0 && dc == 0.0 && dd == 0.0) {
    Serial.println("# ⚠ Ultrasonicos en 0 — descartada");
    delay(DELAY_MUESTRA);
    return;
  }
  if (c == 0) {
    Serial.println("# ⚠ Sin color — descartada");
    delay(DELAY_MUESTRA);
    return;
  }

  muestras_tomadas++;
  Serial.printf("%.1f,%.1f,%.1f,%d,%d,%d,2\n", di, dc, dd, r, g, b);
  Serial.printf("# Muestra %d/%d\n", muestras_tomadas, MUESTRAS_POR_SESION);

  delay(DELAY_MUESTRA);
}

float leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 30000);
  return (dur == 0) ? 0.0 : (dur * 0.0343) / 2.0;
}
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
#define DELAY_MUESTRA       400  // mas rapido para capturar movimiento

int  muestras_tomadas = 0;
bool sesion_activa    = false;

float prev_di = 0, prev_dc = 0, prev_dd = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(TRIG_IZQ,  OUTPUT); pinMode(ECHO_IZQ,  INPUT);
  pinMode(TRIG_CENT, OUTPUT); pinMode(ECHO_CENT, INPUT);
  pinMode(TRIG_DER,  OUTPUT); pinMode(ECHO_DER,  INPUT);

  if (tcs.begin()) Serial.println("# TCS34725 OK");
  else             Serial.println("# TCS34725 ERROR");

  Serial.println("# RECOLECTOR - BALON DISPUTADO (clase 3)");
  Serial.println("# Alguien debe mover el balon frente al robot durante la captura");
  Serial.println("# 'S' → Iniciar | 'P' → Pausar | 'R' → Reiniciar");
  Serial.println("D_IZQ,D_CENT,D_DER,VAR_IZQ,VAR_CENT,VAR_DER,R,G,B,CLASE");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'S' || cmd == 's') {
      sesion_activa    = true;
      muestras_tomadas = 0;
      prev_di = prev_dc = prev_dd = 0;
      Serial.println("# → Sesion iniciada. Mueve el balon frente al robot.");
    }
    if (cmd == 'P' || cmd == 'p') {
      sesion_activa = false;
      Serial.println("# → Pausado.");
    }
    if (cmd == 'R' || cmd == 'r') {
      muestras_tomadas = 0;
      prev_di = prev_dc = prev_dd = 0;
      Serial.println("# → Reiniciado.");
    }
  }

  if (!sesion_activa) return;

  if (muestras_tomadas >= MUESTRAS_POR_SESION) {
    sesion_activa = false;
    Serial.printf("# ✓ %d muestras capturadas. Guarda como datos_balon_disputado.txt\n", muestras_tomadas);
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

  // Variacion respecto a lectura anterior
  float var_i = (prev_di == 0) ? 0 : abs(di - prev_di);
  float var_c = (prev_dc == 0) ? 0 : abs(dc - prev_dc);
  float var_d = (prev_dd == 0) ? 0 : abs(dd - prev_dd);

  if (prev_di == 0 && prev_dc == 0 && prev_dd == 0) {
    prev_di = di; prev_dc = dc; prev_dd = dd;
    Serial.println("# Primera lectura de referencia tomada...");
    delay(DELAY_MUESTRA);
    return;
  }

  muestras_tomadas++;
  Serial.printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,%d,%d,%d,3\n",
                di, dc, dd, var_i, var_c, var_d, r, g, b);
  Serial.printf("# Muestra %d/%d | VAR: IZQ=%.1f CENT=%.1f DER=%.1f\n",
                muestras_tomadas, MUESTRAS_POR_SESION, var_i, var_c, var_d);

  prev_di = di; prev_dc = dc; prev_dd = dd;

  delay(DELAY_MUESTRA);
}

float leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 30000);
  return (dur == 0) ? 0.0 : (dur * 0.0343) / 2.0;
}
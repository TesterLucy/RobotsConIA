// ─────────────────────────────────────────
//  PINES HC-SR04
// ─────────────────────────────────────────
#define TRIG_IZQ  22
#define ECHO_IZQ  21
#define TRIG_CENT 18
#define ECHO_CENT 34
#define TRIG_DER  17
#define ECHO_DER  16

#define MUESTRAS_POR_SESION 30
#define DELAY_MUESTRA       400  // más rápido para capturar movimiento

int muestras_tomadas = 0;
bool sesion_activa   = false;

// Lecturas anteriores para calcular variación
float prev_di = 0, prev_dc = 0, prev_dd = 0;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_IZQ,  OUTPUT); pinMode(ECHO_IZQ,  INPUT);
  pinMode(TRIG_CENT, OUTPUT); pinMode(ECHO_CENT, INPUT);
  pinMode(TRIG_DER,  OUTPUT); pinMode(ECHO_DER,  INPUT);

  Serial.println("# RECOLECTOR - CLASE: OPONENTE (1)");
  Serial.println("# El oponente DEBE moverse durante la captura");
  Serial.println("# 'S' → Iniciar | 'P' → Pausar | 'R' → Reiniciar");
  Serial.println("D_IZQ,D_CENT,D_DER,VAR_IZQ,VAR_CENT,VAR_DER,CLASE");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'S' || cmd == 's') {
      sesion_activa    = true;
      muestras_tomadas = 0;
      prev_di = prev_dc = prev_dd = 0;
      Serial.println("# → Sesión iniciada. Mueve el oponente frente al robot.");
    }
    if (cmd == 'P' || cmd == 'p') {
      sesion_activa = false;
      Serial.println("# → Pausado.");
    }
    if (cmd == 'R' || cmd == 'r') {
      muestras_tomadas = 0;
      prev_di = prev_dc = prev_dd = 0;
      Serial.println("# → Contador reiniciado.");
    }
  }

  if (!sesion_activa) return;

  if (muestras_tomadas >= MUESTRAS_POR_SESION) {
    sesion_activa = false;
    Serial.printf("# ✓ %d muestras capturadas. Copia las líneas sin # a datos_oponente.txt\n", muestras_tomadas);
    return;
  }

  float di = leerDistancia(TRIG_IZQ,  ECHO_IZQ);
  float dc = leerDistancia(TRIG_CENT, ECHO_CENT);
  float dd = leerDistancia(TRIG_DER,  ECHO_DER);

  // Descartar si todos en 0
  if (di == 0.0 && dc == 0.0 && dd == 0.0) {
    Serial.println("# ⚠ Todos los sensores en 0 — muestra descartada");
    delay(DELAY_MUESTRA);
    return;
  }

  // Calcular variación respecto a lectura anterior
  float var_i = abs(di - prev_di);
  float var_c = abs(dc - prev_dc);
  float var_d = abs(dd - prev_dd);

  // Primera muestra no tiene variación aún, no contarla
  if (prev_di == 0 && prev_dc == 0 && prev_dd == 0) {
    prev_di = di; prev_dc = dc; prev_dd = dd;
    Serial.println("# Primera lectura tomada como referencia...");
    delay(DELAY_MUESTRA);
    return;
  }

  muestras_tomadas++;
  // Clase 1 = OPONENTE
  Serial.printf("%.1f,%.1f,%.1f,%.1f,%.1f,%.1f,1\n",
                di, dc, dd, var_i, var_c, var_d);
  Serial.printf("# Muestra %d/%d | Variación: IZQ=%.1f CENT=%.1f DER=%.1f\n",
                muestras_tomadas, MUESTRAS_POR_SESION, var_i, var_c, var_d);

  prev_di = di;
  prev_dc = dc;
  prev_dd = dd;

  delay(DELAY_MUESTRA);
}

float leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 30000);
  return (dur == 0) ? 0.0 : (dur * 0.0343) / 2.0;
}
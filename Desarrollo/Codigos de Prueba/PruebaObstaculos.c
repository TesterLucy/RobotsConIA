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
#define DELAY_MUESTRA       600

int muestras_tomadas = 0;
bool sesion_activa   = false;

void setup() {
  Serial.begin(115200);

  pinMode(TRIG_IZQ,  OUTPUT); pinMode(ECHO_IZQ,  INPUT);
  pinMode(TRIG_CENT, OUTPUT); pinMode(ECHO_CENT, INPUT);
  pinMode(TRIG_DER,  OUTPUT); pinMode(ECHO_DER,  INPUT);

  Serial.println("# RECOLECTOR - CLASE: PARED/OBSTÁCULO (0)");
  Serial.println("# 'S' → Iniciar | 'P' → Pausar | 'R' → Reiniciar");
  Serial.println("D_IZQ,D_CENT,D_DER,CLASE");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'S' || cmd == 's') {
      sesion_activa = true;
      muestras_tomadas = 0;
      Serial.println("# → Sesión iniciada. Pon el robot frente a la PARED.");
    }
    if (cmd == 'P' || cmd == 'p') {
      sesion_activa = false;
      Serial.println("# → Pausado.");
    }
    if (cmd == 'R' || cmd == 'r') {
      muestras_tomadas = 0;
      Serial.println("# → Contador reiniciado.");
    }
  }

  if (!sesion_activa) return;

  if (muestras_tomadas >= MUESTRAS_POR_SESION) {
    sesion_activa = false;
    Serial.printf("# ✓ %d muestras capturadas. Copia las líneas sin # a datos_pared.txt\n", muestras_tomadas);
    return;
  }

  float di = leerDistancia(TRIG_IZQ,  ECHO_IZQ);
  float dc = leerDistancia(TRIG_CENT, ECHO_CENT);
  float dd = leerDistancia(TRIG_DER,  ECHO_DER);

  // Descartar lecturas inválidas
  if (di == 0.0 && dc == 0.0 && dd == 0.0) {
    Serial.println("# ⚠ Todos los sensores en 0 — muestra descartada");
    delay(DELAY_MUESTRA);
    return;
  }

  muestras_tomadas++;
  Serial.printf("%.1f,%.1f,%.1f,0\n", di, dc, dd);
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
#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <esp_now.h>
#include <WiFi.h>

// ─── PINES MOTORES (TB6612FNG) ────────────────────────
#define PWMA 13
#define AIN1 12
#define AIN2 14
#define PWMB 32
#define BIN1 26
#define BIN2 25
#define STBY 27

// ─── PINES ULTRASONIDOS ───────────────────────────────
const int TRIG_IZQ  = 17; const int ECHO_IZQ  = 16;
const int TRIG_CENT = 18; const int ECHO_CENT = 34;
const int TRIG_DER  = 22; const int ECHO_DER  = 21;

// ─── SENSOR COLOR TCS34725 ────────────────────────────
const int COLOR_INT = 35;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ─── NORMALIZACIÓN NN ─────────────────────────────────
const float X_MIN[9] = { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
const float X_MAX[9] = { 326.7f, 370.7f, 429.4f, 234.7f, 339.5f, 395.2f, 1849.0f, 1008.0f, 805.0f };

// ─── PESOS CAPA 1 (9→12) ──────────────────────────────
const float W1[9][12] = {
  { 1.005823f, -0.815607f,  0.577965f,  0.313271f,  0.545100f, -1.183759f, -1.081539f,  0.101231f, -0.364387f,  0.733150f, -1.315404f, -0.687704f },
  { 0.748521f, -0.572753f,  1.491075f, -1.152886f, -1.360221f,  0.794736f, -0.863820f, -0.569753f,  1.460140f, -1.735379f, -0.789549f,  1.408094f },
  {-0.678845f,  0.817216f, -2.435160f,  0.334982f,  1.959602f, -0.364716f,  4.022729f, -1.071270f, -1.971540f,  1.978401f,  0.983011f, -4.114726f },
  {-0.933778f,  0.969292f, 10.871201f, -9.276482f, 10.105311f,-10.968744f,-11.756814f,  0.465063f,-10.454165f, 10.520631f,-11.889908f,  1.939801f },
  {-0.503037f,  0.668362f, 12.317353f, -8.884113f, 11.586618f,-11.436604f,-11.283238f,  0.164965f,-11.500468f, 11.078697f,-12.261329f,  2.908277f },
  {-0.717122f,  0.885246f, 10.953985f, -8.780297f, 10.454718f,-11.169803f,-10.873724f, -0.266044f,-10.912119f, 10.997339f,-10.999943f,  4.155910f },
  {-8.719503f,  8.424869f,  3.069640f,  7.035437f,  1.013482f, -1.867741f, -0.135516f,  9.789619f, -0.912426f,  1.227475f,  3.724462f,-11.038841f },
  {-10.005598f, 9.596388f,  7.106573f,  4.252138f, -1.617121f, -0.298039f,  1.164054f, 10.223232f,  1.318834f, -1.321644f,  3.307976f, -8.986072f },
  {-10.199025f,10.012968f,  7.807174f,  2.799011f, -2.228036f, -1.136594f,  0.708384f, 10.289812f,  1.401762f, -1.598476f,  1.806483f, -8.277536f }
};
const float BIAS1[12] = { 0.640483f,-0.693225f,-0.719822f,-0.547216f, 0.297862f, 0.828719f, 0.119187f,-0.448577f,-0.147602f, 0.108630f,-0.123247f, 0.247207f };

// ─── PESOS CAPA 2 (12→4) ──────────────────────────────
const float W2[12][4] = {
  { 3.058809f,  4.068694f, -2.748021f, -6.033177f },
  {-3.757207f, -4.266846f,  1.394397f,  3.484326f },
  {-7.186811f,  1.363811f, -0.608394f,  3.569252f },
  {-0.171885f, -3.397344f,  2.850711f, -0.684319f },
  { 0.000357f,  2.595394f, -3.802498f,  2.143354f },
  { 2.989235f, -1.501108f,  2.577510f, -4.511124f },
  { 2.988691f, -4.700216f,  2.079373f, -1.442839f },
  {-3.377712f, -3.050286f,  1.636486f,  1.971505f },
  { 0.538099f, -2.299687f,  3.716113f, -2.987345f },
  {-0.383471f,  2.694342f, -4.027016f,  2.624729f },
  { 1.753034f, -4.403143f,  3.045432f, -1.439449f },
  { 0.298115f,  4.544530f, -4.818734f, -1.957475f }
};
const float BIAS2[4] = { -0.139407f, 0.702186f, -0.504691f, -0.376316f };

// ─── ESP-NOW ──────────────────────────────────────────
// ⚠️ Cambia esta MAC por la que imprima el portero en su Serial Monitor
uint8_t macPortero[] = {0xD4, 0xE9, 0xF4, 0xAB, 0x3D, 0x18};

typedef struct {
  int8_t  direccion;   // -1=izq, 0=centro, 1=der
  uint8_t estado;      // 0=BUSCAR,1=ATACAR,2=PELEAR,3=EVITAR
  float   distCentro;
} MensajeJugador;

MensajeJugador msgJ;
bool espnowListo = false;

void setupESPNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(2000);
  Serial.print("MAC Jugador: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init FALLO");
    return;
  }
  esp_now_peer_info_t peer;
  memset(&peer, 0, sizeof(peer));
  memcpy(peer.peer_addr, macPortero, 6);
  peer.channel = 0;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ESP-NOW add peer FALLO");
    return;
  }
  espnowListo = true;
  Serial.println("ESP-NOW OK");
}

void enviarAlPortero(int dir, int est, float dc) {
  if (!espnowListo) return;
  msgJ.direccion  = (int8_t)dir;
  msgJ.estado     = (uint8_t)est;
  msgJ.distCentro = dc;
  esp_now_send(macPortero, (uint8_t*)&msgJ, sizeof(msgJ));
}

// ─── ESTADO ───────────────────────────────────────────
enum Estado { BUSCAR, ATACAR, PELEAR, EVITAR };
Estado estadoActual = BUSCAR;

float d_izq, d_cent, d_der;
float p_izq, p_cent, p_der;
uint16_t colorR, colorG, colorB, colorC;

// ─── PSO / GA ─────────────────────────────────────────
float pso_vel = 0.5f, pso_pBest = 0.5f, pso_gBest = 0.5f;
const float PSO_W=0.7f, PSO_C1=1.5f, PSO_C2=1.5f;
float genes[3]     = { 0.6f, 0.8f, 0.5f };
float fitness_acum = 0.0f;
int   generacion   = 0;
uint32_t t_buscar  = 0;
int   fase_buscar  = 0;

// ─── NN ───────────────────────────────────────────────
float sigmoid_fn(float x) { return 1.0f / (1.0f + expf(-x)); }

int clasificar(float di, float dc, float dd,
               float vi, float vc, float vd,
               float cr, float cg, float cb,
               float* probs) {
  float inp[9] = { di, dc, dd, vi, vc, vd, cr, cg, cb };
  for (int i = 0; i < 9; i++) {
    float rng = X_MAX[i] - X_MIN[i];
    if (rng < 0.001f) rng = 1.0f;
    inp[i] = constrain((inp[i] - X_MIN[i]) / rng, 0.0f, 1.0f);
  }
  float h[12];
  for (int j = 0; j < 12; j++) {
    float s = BIAS1[j];
    for (int i = 0; i < 9; i++) s += inp[i] * W1[i][j];
    h[j] = sigmoid_fn(s);
  }
  float out[4], mx = -1e9f, sx = 0;
  for (int k = 0; k < 4; k++) {
    float s = BIAS2[k];
    for (int j = 0; j < 12; j++) s += h[j] * W2[j][k];
    out[k] = s;
    if (s > mx) mx = s;
  }
  int best = 0;
  for (int k = 0; k < 4; k++) { out[k] = expf(out[k]-mx); sx += out[k]; }
  for (int k = 0; k < 4; k++) { probs[k] = out[k]/sx; if (probs[k]>probs[best]) best=k; }
  return best;
}

// ─── SENSORES ─────────────────────────────────────────
float medirDistancia(int trg, int ech) {
  digitalWrite(trg, LOW);  delayMicroseconds(2);
  digitalWrite(trg, HIGH); delayMicroseconds(10);
  digitalWrite(trg, LOW);
  long dur = pulseIn(ech, HIGH, 30000);
  return (dur == 0) ? 0.0f : (dur * 0.0343f) / 2.0f;
}

void leerSensores() {
  p_izq=d_izq; p_cent=d_cent; p_der=d_der;
  d_izq  = medirDistancia(TRIG_IZQ,  ECHO_IZQ);
  d_cent = medirDistancia(TRIG_CENT, ECHO_CENT);
  d_der  = medirDistancia(TRIG_DER,  ECHO_DER);
  tcs.getRawData(&colorR, &colorG, &colorB, &colorC);
}

// ─── MOTORES ──────────────────────────────────────────
void moverMotorA(int speed) {
  if      (speed > 0) { digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);  }
  else if (speed < 0) { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH); speed = -speed; }
  else                { digitalWrite(AIN1, LOW);  digitalWrite(AIN2, LOW);  speed = 0; }
  ledcWrite(PWMA, constrain(speed, 0, 255));
}

void moverMotorB(int speed) {
  if      (speed > 0) { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH); }
  else if (speed < 0) { digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);  speed = -speed; }
  else                { digitalWrite(BIN1, LOW);  digitalWrite(BIN2, LOW);  speed = 0; }
  ledcWrite(PWMB, constrain(speed, 0, 255));
}

void mover(int sA, int sB) { moverMotorA(sA); moverMotorB(sB); }

// ─── SETUP ────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // ESP-NOW primero (necesita WiFi antes de Wire)
  setupESPNow();

  Wire.begin(19, 23);
  pinMode(COLOR_INT, INPUT);
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, 1000, 8);
  ledcAttach(PWMB, 1000, 8);
  pinMode(TRIG_IZQ,  OUTPUT); pinMode(ECHO_IZQ,  INPUT);
  pinMode(TRIG_CENT, OUTPUT); pinMode(ECHO_CENT, INPUT);
  pinMode(TRIG_DER,  OUTPUT); pinMode(ECHO_DER,  INPUT);

  if (tcs.begin()) Serial.println("TCS34725 OK");
  else { Serial.println("TCS34725 ERROR - revisar SDA=19 SCL=23"); while(1); }

  mover(0, 0);
  Serial.println("JUGADOR listo.");
}

// ─── LOOP ─────────────────────────────────────────────
void loop() {
  leerSensores();

  float var_i = fabsf(d_izq  - p_izq);
  float var_c = fabsf(d_cent - p_cent);
  float var_d = fabsf(d_der  - p_der);

  float probs[4];
  int clase = clasificar(d_izq, d_cent, d_der,
                         var_i, var_c, var_d,
                         (float)colorR, (float)colorG, (float)colorB,
                         probs);

  // Transición de estado
  if (probs[clase] > 0.65f) {
    if      (clase == 0) estadoActual = EVITAR;
    else if (clase == 1) estadoActual = ATACAR;
    else if (clase == 2) estadoActual = ATACAR;
    else if (clase == 3) estadoActual = PELEAR;
  } else {
    estadoActual = BUSCAR;
  }

  switch (estadoActual) {

    case ATACAR: {
      int vel = (int)(180 + genes[2] * 75);
      if      (d_cent > 0 && d_cent < 40)                 mover(vel, vel);
      else if (d_izq > 0 && d_izq < d_der && d_izq < 60) mover(vel/2, vel);
      else if (d_der > 0 && d_der < d_izq && d_der < 60) mover(vel, vel/2);
      else                                                 mover(vel, vel);
      fitness_acum += genes[0] * 5.0f;
      break;
    }

    case PELEAR:
      mover((int)(200+genes[2]*55), (int)(200+genes[2]*55));
      break;

    case EVITAR:
      mover(-200, -200); delay(300);
      if (d_izq >= d_der) mover(200, -200);
      else                mover(-200, 200);
      delay(400);
      mover(0, 0);
      estadoActual = BUSCAR;
      break;

    case BUSCAR: {
      float r1 = (float)random(100)/100.0f;
      float r2 = (float)random(100)/100.0f;
      pso_vel = PSO_W*pso_vel + PSO_C1*r1*(pso_pBest-pso_vel) + PSO_C2*r2*(pso_gBest-pso_vel);
      pso_vel = constrain(pso_vel, 0.3f, 1.0f);

      uint32_t ahora = millis();
      if (ahora - t_buscar > (uint32_t)(600 + pso_vel*300)) {
        t_buscar    = ahora;
        fase_buscar = (fase_buscar + 1) % 3;
      }
      int vb = (int)(pso_vel * 220);
      if      (fase_buscar == 0) mover(vb, vb);
      else if (fase_buscar == 1) mover(vb, vb/3);
      else                       mover(vb/3, vb);
      break;
    }
  }

  // ─── GA ───────────────────────────────────────────────
  generacion++;
  if (generacion % 10 == 0) {
    for (int i = 0; i < 3; i++) {
      float mut = (float)(random(200)-100)/1000.0f;
      genes[i] = constrain(genes[i]+mut, 0.1f, 1.0f);
    }
  }
  if (probs[clase] > 0.75f && (clase == 2 || clase == 3)) {
    fitness_acum += genes[1]*10.0f;
    genes[1]  = constrain(genes[1]+0.02f, 0.1f, 1.0f);
    pso_pBest = pso_vel;
  }
  if (probs[clase] > 0.75f && clase == 0) fitness_acum -= 2.0f;

  // ─── ENVIAR POSICIÓN AL PORTERO ───────────────────────
  // Determina en qué lado está el objeto de interés
  int dirEnviar = 0;
  if      (d_izq  > 0 && d_izq  < d_der && d_izq  < d_cent && d_izq  < 80) dirEnviar = -1; // izq
  else if (d_der  > 0 && d_der  < d_izq && d_der  < d_cent && d_der  < 80) dirEnviar =  1; // der
  // centro o indefinido → 0
  enviarAlPortero(dirEnviar, (int)estadoActual, d_cent);

  // ─── DEBUG ────────────────────────────────────────────
  const char* nombres[] = { "PARED","OPONENTE","BALON_LIBRE","BALON_DISPUTADO" };
  const char* estados[] = { "BUSCAR","ATACAR","PELEAR","EVITAR" };
  Serial.printf("[NN:%s %.0f%%] EST:%s | IZQ:%.1f CENT:%.1f DER:%.1f | pso:%.2f fit:%.1f | →PORTERO dir=%d\n",
                nombres[clase], probs[clase]*100,
                estados[estadoActual],
                d_izq, d_cent, d_der, pso_vel, fitness_acum,
                dirEnviar);
  delay(20);
}
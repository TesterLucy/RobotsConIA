// ═══════════════════════════════════════════════════════════════
//  RED NEURONAL ENTRENADA - Clasificador de objetos
//  Arquitectura: 9 entradas → 12 neuronas ocultas → 4 salidas
//  Clases: 0=Pared, 1=Oponente, 2=Balón libre, 3=Balón disputado
//  Precisión: 96.13% sobre datos reales
//  Entrenado con GA + MLP sobre 2841 muestras
// ═══════════════════════════════════════════════════════════════

#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <math.h>

// ─── PINES ───────────────────────────────────────────────────
#define TRIG_IZQ  22
#define ECHO_IZQ  21
#define TRIG_CENT 18
#define ECHO_CENT 34
#define TRIG_DER  17
#define ECHO_DER  16
#define SDA_PIN   19
#define SCL_PIN   23

Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ─── NORMALIZACIÓN (min-max aprendida del entrenamiento) ─────
const float X_MIN[9] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
const float X_MAX[9] = {326.7f, 370.7f, 429.4f, 234.7f, 339.5f, 395.2f, 1849.0f, 1008.0f, 805.0f};

// ─── PESOS CAPA 1 (9→12) ────────────────────────────────────
const float W1[9][12] = {
  {1.005823f, -0.815607f, 0.577965f, 0.313271f, 0.545100f, -1.183759f, -1.081539f, 0.101231f, -0.364387f, 0.733150f, -1.315404f, -0.687704f},
  {0.748521f, -0.572753f, 1.491075f, -1.152886f, -1.360221f, 0.794736f, -0.863820f, -0.569753f, 1.460140f, -1.735379f, -0.789549f, 1.408094f},
  {-0.678845f, 0.817216f, -2.435160f, 0.334982f, 1.959602f, -0.364716f, 4.022729f, -1.071270f, -1.971540f, 1.978401f, 0.983011f, -4.114726f},
  {-0.933778f, 0.969292f, 10.871201f, -9.276482f, 10.105311f, -10.968744f, -11.756814f, 0.465063f, -10.454165f, 10.520631f, -11.889908f, 1.939801f},
  {-0.503037f, 0.668362f, 12.317353f, -8.884113f, 11.586618f, -11.436604f, -11.283238f, 0.164965f, -11.500468f, 11.078699f, -12.261329f, 2.908277f},
  {-0.717122f, 0.885246f, 10.953985f, -8.780297f, 10.454718f, -11.169803f, -10.873724f, -0.266044f, -10.912119f, 10.997339f, -10.999943f, 4.155910f},
  {-8.719503f, 8.424869f, 3.069640f, 7.035437f, 1.013482f, -1.867741f, -0.135516f, 9.789619f, -0.912426f, 1.227475f, 3.724462f, -11.038841f},
  {-10.005598f, 9.596388f, 7.106573f, 4.252138f, -1.617121f, -0.298039f, 1.164054f, 10.223232f, 1.318834f, -1.321644f, 3.307976f, -8.986072f},
  {-10.199025f, 10.012968f, 7.807174f, 2.799011f, -2.228036f, -1.136594f, 0.708384f, 10.289812f, 1.401762f, -1.598476f, 1.806483f, -8.277536f}
};
const float B1[12] = {0.640483f, -0.693225f, -0.719822f, -0.547216f, 0.297862f, 0.828719f, 0.119187f, -0.448577f, -0.147602f, 0.108630f, -0.123247f, 0.247207f};

// ─── PESOS CAPA 2 (12→4) ────────────────────────────────────
const float W2[12][4] = {
  {3.058809f, 4.068694f, -2.748021f, -6.033177f},
  {-3.757207f, -4.266846f, 1.394397f, 3.484326f},
  {-7.186811f, 1.363811f, -0.608394f, 3.569252f},
  {-0.171885f, -3.397344f, 2.850711f, -0.684319f},
  {0.000357f, 2.595394f, -3.802498f, 2.143354f},
  {2.989235f, -1.501108f, 2.577510f, -4.511124f},
  {2.988691f, -4.700216f, 2.079373f, -1.442839f},
  {-3.377712f, -3.050286f, 1.636486f, 1.971505f},
  {0.538099f, -2.299687f, 3.716113f, -2.987345f},
  {-0.383471f, 2.694342f, -4.027016f, 2.624729f},
  {1.753034f, -4.403143f, 3.045432f, -1.439449f},
  {0.298115f, 4.544530f, -4.818734f, -1.957475f}
};
const float B2[4] = {-0.139407f, 0.702186f, -0.504691f, -0.376316f};

// ─── VARIABLES PARA VARIACIÓN ────────────────────────────────
float prev_di = 0, prev_dc = 0, prev_dd = 0;

// ─── FUNCIONES ───────────────────────────────────────────────
float sigmoid(float x) {
  return 1.0f / (1.0f + expf(-x));
}

float leerDistancia(int trig, int echo) {
  digitalWrite(trig, LOW);  delayMicroseconds(2);
  digitalWrite(trig, HIGH); delayMicroseconds(10);
  digitalWrite(trig, LOW);
  long dur = pulseIn(echo, HIGH, 30000);
  return (dur == 0) ? 0.0f : (dur * 0.0343f) / 2.0f;
}

// Clasificar: retorna clase (0-3) y llena probs[4] con probabilidades
int clasificar(float di, float dc, float dd,
               float var_i, float var_c, float var_d,
               float r, float g, float b,
               float* probs) {

  // Vector de entrada
  float inp[9] = {di, dc, dd, var_i, var_c, var_d, r, g, b};

  // Normalizar
  for (int i = 0; i < 9; i++) {
    float rng = X_MAX[i] - X_MIN[i];
    if (rng < 0.001f) rng = 1.0f;
    inp[i] = (inp[i] - X_MIN[i]) / rng;
    if (inp[i] < 0) inp[i] = 0;
    if (inp[i] > 1) inp[i] = 1;
  }

  // Capa oculta
  float h[12];
  for (int j = 0; j < 12; j++) {
    float sum = B1[j];
    for (int i = 0; i < 9; i++) sum += inp[i] * W1[i][j];
    h[j] = sigmoid(sum);
  }

  // Capa de salida (softmax)
  float out[4];
  float max_val = -1e9f, sum_exp = 0;
  for (int k = 0; k < 4; k++) {
    float s = B2[k];
    for (int j = 0; j < 12; j++) s += h[j] * W2[j][k];
    out[k] = s;
    if (s > max_val) max_val = s;
  }
  for (int k = 0; k < 4; k++) {
    out[k] = expf(out[k] - max_val);
    sum_exp += out[k];
  }
  int best = 0;
  for (int k = 0; k < 4; k++) {
    probs[k] = out[k] / sum_exp;
    if (probs[k] > probs[best]) best = k;
  }
  return best;
}

// ─── SETUP ───────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  pinMode(TRIG_IZQ,  OUTPUT); pinMode(ECHO_IZQ,  INPUT);
  pinMode(TRIG_CENT, OUTPUT); pinMode(ECHO_CENT, INPUT);
  pinMode(TRIG_DER,  OUTPUT); pinMode(ECHO_DER,  INPUT);

  if (tcs.begin()) Serial.println("TCS34725 OK");
  else             Serial.println("TCS34725 ERROR");

  Serial.println("Clasificador listo. Clases: 0=Pared 1=Oponente 2=Balon_libre 3=Balon_disputado");
}

// ─── LOOP ────────────────────────────────────────────────────
void loop() {
  float di = leerDistancia(TRIG_IZQ,  ECHO_IZQ);
  float dc = leerDistancia(TRIG_CENT, ECHO_CENT);
  float dd = leerDistancia(TRIG_DER,  ECHO_DER);

  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);

  float var_i = (prev_di > 0) ? fabsf(di - prev_di) : 0;
  float var_c = (prev_dc > 0) ? fabsf(dc - prev_dc) : 0;
  float var_d = (prev_dd > 0) ? fabsf(dd - prev_dd) : 0;

  prev_di = di; prev_dc = dc; prev_dd = dd;

  float probs[4];
  int clase = clasificar(di, dc, dd, var_i, var_c, var_d,
                         (float)r, (float)g, (float)b, probs);

  const char* nombres[] = {"PARED", "OPONENTE", "BALON_LIBRE", "BALON_DISPUTADO"};

  Serial.printf("Clase: %d (%s) | Confianza: %.1f%%\n",
                clase, nombres[clase], probs[clase]*100);
  Serial.printf("  Probs → Pared:%.2f Opon:%.2f BalonL:%.2f BalonD:%.2f\n",
                probs[0], probs[1], probs[2], probs[3]);
  Serial.printf("  Dist → IZQ:%.1f CENT:%.1f DER:%.1f | Color R:%d G:%d B:%d\n\n",
                di, dc, dd, r, g, b);

  delay(400);
}

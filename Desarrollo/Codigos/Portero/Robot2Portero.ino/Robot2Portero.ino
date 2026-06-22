#include <Wire.h>
#include <Adafruit_TCS34725.h>
#include <esp_now.h>
#include <WiFi.h>

// ─── PINES MOTORES ────────────────────────────────────
#define PWMA 32
#define AIN1 26
#define AIN2 25
#define PWMB 13
#define BIN1 14
#define BIN2 12
#define STBY 27

// ─── PINES ULTRASONIDOS ───────────────────────────────
const int TRIG_IZQ  = 17; const int ECHO_IZQ  = 34;
const int TRIG_CENT = 16; const int ECHO_CENT =  4;
const int TRIG_DER  =  2; const int ECHO_DER  = 15;

// ─── SENSOR COLOR ─────────────────────────────────────
const int COLOR_INT = 18;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

// ─── NN PESOS ─────────────────────────────────────────
const float X_MIN[9] = {0,0,0,0,0,0,0,0,0};
const float X_MAX[9] = {326.7f,370.7f,429.4f,234.7f,339.5f,395.2f,1849.0f,1008.0f,805.0f};

const float W1[9][12] = {
  { 1.005823f,-0.815607f, 0.577965f, 0.313271f, 0.545100f,-1.183759f,-1.081539f, 0.101231f,-0.364387f, 0.733150f,-1.315404f,-0.687704f},
  { 0.748521f,-0.572753f, 1.491075f,-1.152886f,-1.360221f, 0.794736f,-0.863820f,-0.569753f, 1.460140f,-1.735379f,-0.789549f, 1.408094f},
  {-0.678845f, 0.817216f,-2.435160f, 0.334982f, 1.959602f,-0.364716f, 4.022729f,-1.071270f,-1.971540f, 1.978401f, 0.983011f,-4.114726f},
  {-0.933778f, 0.969292f,10.871201f,-9.276482f,10.105311f,-10.968744f,-11.756814f,0.465063f,-10.454165f,10.520631f,-11.889908f,1.939801f},
  {-0.503037f, 0.668362f,12.317353f,-8.884113f,11.586618f,-11.436604f,-11.283238f,0.164965f,-11.500468f,11.078697f,-12.261329f,2.908277f},
  {-0.717122f, 0.885246f,10.953985f,-8.780297f,10.454718f,-11.169803f,-10.873724f,-0.266044f,-10.912119f,10.997339f,-10.999943f,4.155910f},
  {-8.719503f, 8.424869f, 3.069640f, 7.035437f, 1.013482f,-1.867741f,-0.135516f, 9.789619f,-0.912426f, 1.227475f, 3.724462f,-11.038841f},
  {-10.005598f,9.596388f, 7.106573f, 4.252138f,-1.617121f,-0.298039f, 1.164054f,10.223232f, 1.318834f,-1.321644f, 3.307976f,-8.986072f},
  {-10.199025f,10.012968f,7.807174f, 2.799011f,-2.228036f,-1.136594f, 0.708384f,10.289812f, 1.401762f,-1.598476f, 1.806483f,-8.277536f}
};
const float BIAS1[12] = {0.640483f,-0.693225f,-0.719822f,-0.547216f,0.297862f,0.828719f,0.119187f,-0.448577f,-0.147602f,0.108630f,-0.123247f,0.247207f};

const float W2[12][4] = {
  { 3.058809f, 4.068694f,-2.748021f,-6.033177f},
  {-3.757207f,-4.266846f, 1.394397f, 3.484326f},
  {-7.186811f, 1.363811f,-0.608394f, 3.569252f},
  {-0.171885f,-3.397344f, 2.850711f,-0.684319f},
  { 0.000357f, 2.595394f,-3.802498f, 2.143354f},
  { 2.989235f,-1.501108f, 2.577510f,-4.511124f},
  { 2.988691f,-4.700216f, 2.079373f,-1.442839f},
  {-3.377712f,-3.050286f, 1.636486f, 1.971505f},
  { 0.538099f,-2.299687f, 3.716113f,-2.987345f},
  {-0.383471f, 2.694342f,-4.027016f, 2.624729f},
  { 1.753034f,-4.403143f, 3.045432f,-1.439447f},
  { 0.298115f, 4.544530f,-4.818734f,-1.957475f}
};
const float BIAS2[4] = {-0.139407f,0.702186f,-0.504691f,-0.376316f};

// ─── PERCEPTRONES ─────────────────────────────────────
const float W_PELIGRO[5] = {-2.5f,-1.2f,-1.2f,2.0f,2.5f};
const float B_PELIGRO    =  0.8f;
float perceptronPeligro(float dc,float di,float dd,float op,float bal){
  float z=B_PELIGRO+W_PELIGRO[0]*dc+W_PELIGRO[1]*di+W_PELIGRO[2]*dd+W_PELIGRO[3]*op+W_PELIGRO[4]*bal;
  return 1.0f/(1.0f+expf(-z));
}

// Perceptrón de alineación con el jugador:
// Decide si girar izq o der basado en datos locales + dirección recibida
// Entradas: [di_n, dd_n, dir_jugador_n]
// salida >0.5 → girar izquierda, <0.5 → girar derecha
const float W_ALIN[3] = {-1.5f, 1.5f, 2.0f};
const float B_ALIN    =  0.0f;
float perceptronAlineacion(float di_n, float dd_n, float dir_jug_n){
  float z = B_ALIN + W_ALIN[0]*di_n + W_ALIN[1]*dd_n + W_ALIN[2]*dir_jug_n;
  return 1.0f/(1.0f+expf(-z));
}

// ─── ESP-NOW: datos del jugador ───────────────────────
typedef struct {
  int8_t  direccion;   // -1=izq, 0=centro, 1=der
  uint8_t estado;      // 0-3
  float   distCentro;
} MensajeJugador;

volatile MensajeJugador datosJugador = {0, 0, 999.0f};
volatile bool hayDatosNuevos = false;

void onDataRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len){
  if(len == sizeof(MensajeJugador)){
    memcpy((void*)&datosJugador, data, sizeof(MensajeJugador));
    hayDatosNuevos = true;
  }
}

// ─── ESTADOS PORTERO ──────────────────────────────────
// QUIETO   → parado, solo escucha
// ORIENTAR → gira suavemente para mirar al jugador
// BLOQUEAR → amenaza detectada, bloquea
// DESPEJAR → pelota muy cerca, la despeja
enum EstadoPortero { QUIETO, ORIENTAR, BLOQUEAR, DESPEJAR };
EstadoPortero estado = QUIETO;

float d_izq,d_cent,d_der;
float p_izq,p_cent,p_der;
uint16_t colorR,colorG,colorB,colorC;

float pso_vel=0.6f, pso_pBest=0.6f, pso_gBest=0.6f;
const float PSO_W=0.7f, PSO_C1=1.4f, PSO_C2=1.6f;
float genes[3]    = {0.5f, 0.45f, 0.7f};
float fitness_acum= 0.0f;
int   generacion  = 0;
uint32_t t_estado = 0;
int desplazamiento= 0;

// ─── NN ───────────────────────────────────────────────
float sigmoid_fn(float x){ return 1.0f/(1.0f+expf(-x)); }

int clasificar(float di,float dc,float dd,
               float vi,float vc,float vd,
               float cr,float cg,float cb,float* probs){
  float inp[9]={di,dc,dd,vi,vc,vd,cr,cg,cb};
  for(int i=0;i<9;i++){
    float rng=X_MAX[i]-X_MIN[i]; if(rng<0.001f)rng=1.0f;
    inp[i]=constrain((inp[i]-X_MIN[i])/rng,0.0f,1.0f);
  }
  float h[12];
  for(int j=0;j<12;j++){
    float s=BIAS1[j];
    for(int i=0;i<9;i++) s+=inp[i]*W1[i][j];
    h[j]=sigmoid_fn(s);
  }
  float out[4],mx=-1e9f,sx=0;
  for(int k=0;k<4;k++){
    float s=BIAS2[k];
    for(int j=0;j<12;j++) s+=h[j]*W2[j][k];
    out[k]=s; if(s>mx)mx=s;
  }
  int best=0;
  for(int k=0;k<4;k++){out[k]=expf(out[k]-mx);sx+=out[k];}
  for(int k=0;k<4;k++){probs[k]=out[k]/sx;if(probs[k]>probs[best])best=k;}
  return best;
}

// ─── SENSORES ─────────────────────────────────────────
float medirDistancia(int trg,int ech){
  digitalWrite(trg,LOW);  delayMicroseconds(2);
  digitalWrite(trg,HIGH); delayMicroseconds(10);
  digitalWrite(trg,LOW);
  long dur=pulseIn(ech,HIGH,30000);
  return (dur==0)?0.0f:(dur*0.0343f)/2.0f;
}
void leerSensores(){
  p_izq=d_izq; p_cent=d_cent; p_der=d_der;
  d_izq =medirDistancia(TRIG_IZQ, ECHO_IZQ);
  d_cent=medirDistancia(TRIG_CENT,ECHO_CENT);
  d_der =medirDistancia(TRIG_DER, ECHO_DER);
  tcs.getRawData(&colorR,&colorG,&colorB,&colorC);
}

// ─── MOTORES ──────────────────────────────────────────
void moverMotorA(int speed){
  if      (speed>0){digitalWrite(AIN1,LOW); digitalWrite(AIN2,HIGH);}
  else if (speed<0){digitalWrite(AIN1,HIGH);digitalWrite(AIN2,LOW); speed=-speed;}
  else             {digitalWrite(AIN1,LOW); digitalWrite(AIN2,LOW); speed=0;}
  ledcWrite(PWMA,constrain(speed,0,255));
}
void moverMotorB(int speed){
  if      (speed>0){digitalWrite(BIN1,LOW); digitalWrite(BIN2,HIGH);}
  else if (speed<0){digitalWrite(BIN1,HIGH);digitalWrite(BIN2,LOW); speed=-speed;}
  else             {digitalWrite(BIN1,LOW); digitalWrite(BIN2,LOW); speed=0;}
  ledcWrite(PWMB,constrain(speed,0,255));
}
void mover(int sA,int sB){ moverMotorA(sA); moverMotorB(sB); }

// Giro suave en sitio (velocidad reducida para el portero)
void girarIzq(int vel){ mover(-vel, vel); }
void girarDer(int vel){ mover( vel,-vel); }
void parar()          { mover(0,0); }

// ─── SETUP ────────────────────────────────────────────
void setup(){
  Serial.begin(115200);
  delay(2000);
  // mac Jugador Jugador: A4:F0:0F:5D:B7:64
  // ── WiFi PRIMERO (para que la MAC esté lista) ──
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(300);
  Serial.print("MAC Portero: "); Serial.println(WiFi.macAddress());

  // ── ESP-NOW ────────────────────────────────────
  if(esp_now_init() != ESP_OK){
    Serial.println("ERROR esp_now_init"); return;
  }
  esp_now_register_recv_cb(onDataRecv);

  // ── PINES ─────────────────────────────────────
  Wire.begin(19, 21);
  pinMode(COLOR_INT, INPUT);
  pinMode(STBY, OUTPUT); digitalWrite(STBY, HIGH);
  pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  ledcAttach(PWMA, 1000, 8);
  ledcAttach(PWMB, 1000, 8);
  pinMode(TRIG_IZQ,  OUTPUT); pinMode(ECHO_IZQ,  INPUT);
  pinMode(TRIG_CENT, OUTPUT); pinMode(ECHO_CENT, INPUT);
  pinMode(TRIG_DER,  OUTPUT); pinMode(ECHO_DER,  INPUT);

  // ── COLOR ──────────────────────────────────────
  if(tcs.begin()) Serial.println("TCS34725 OK");
  else{ Serial.println("TCS34725 ERROR"); while(1); }

  parar();
  Serial.println("PORTERO listo. Esperando datos del jugador...");
}

// ─── LOOP ─────────────────────────────────────────────
void loop(){
  leerSensores();

  float var_i=fabsf(d_izq -p_izq);
  float var_c=fabsf(d_cent-p_cent);
  float var_d=fabsf(d_der -p_der);

  float probs[4];
  int clase=clasificar(d_izq,d_cent,d_der,
                       var_i,var_c,var_d,
                       (float)colorR,(float)colorG,(float)colorB,
                       probs);

  float di_n=constrain(d_izq /400.0f,0,1);
  float dc_n=constrain(d_cent/400.0f,0,1);
  float dd_n=constrain(d_der /400.0f,0,1);

  float peligro=perceptronPeligro(dc_n,di_n,dd_n,probs[1],probs[2]+probs[3]);

  // Dirección del jugador normalizada a [0,1]: -1→0, 0→0.5, 1→1
  float dir_jug_n = (datosJugador.direccion + 1) / 2.0f;
  float alineacion = perceptronAlineacion(di_n, dd_n, dir_jug_n);

  // PSO
  float r1=(float)random(100)/100.0f, r2=(float)random(100)/100.0f;
  pso_vel=PSO_W*pso_vel+PSO_C1*r1*(pso_pBest-pso_vel)+PSO_C2*r2*(pso_gBest-pso_vel);
  pso_vel=constrain(pso_vel,0.3f,0.9f);
  int vel_giro=(int)(100+pso_vel*60); // velocidad de giro conservadora

  uint32_t ahora=millis();
  float umbral=genes[0];

  switch(estado){

    // ── QUIETO: estado base, mínimo movimiento ──────────
    case QUIETO:
      parar();
      // Prioridad 1: pelota muy cerca → despejar
      if(d_cent>0 && d_cent<15 && probs[clase]>0.65f){
        estado=DESPEJAR; t_estado=ahora;
      }
      // Prioridad 2: amenaza alta → bloquear
      else if(peligro > umbral && probs[clase]>0.60f){
        estado=BLOQUEAR; t_estado=ahora;
        desplazamiento=(alineacion>0.5f)?1:-1;
      }
      // Prioridad 3: dato nuevo del jugador → orientarse
      else if(hayDatosNuevos && datosJugador.direccion != 0){
        estado=ORIENTAR; t_estado=ahora;
        desplazamiento=(datosJugador.direccion < 0)?1:-1; // gira hacia donde está el jugador
        hayDatosNuevos=false;
      }
      break;

    // ── ORIENTAR: giro corto y suave para mirar al jugador
    case ORIENTAR:{
      // Giro muy breve (150ms máx) para no alejarse de posición
      uint32_t dur_orientar = 150;
      if(ahora-t_estado < dur_orientar){
        if(desplazamiento>0) girarIzq(vel_giro);
        else                 girarDer(vel_giro);
      } else {
        parar();
        estado=QUIETO;
      }
      // Interrumpir si hay peligro
      if(peligro > umbral){ estado=BLOQUEAR; t_estado=ahora; }
      break;
    }

    // ── BLOQUEAR: giro para interponerse en la trayectoria
    case BLOQUEAR:{
      uint32_t dur=(uint32_t)(250+genes[1]*300);
      if(ahora-t_estado < dur){
        if(desplazamiento>0) girarIzq(vel_giro+20);
        else                 girarDer(vel_giro+20);
      } else {
        parar();
        pso_pBest=pso_vel;
        fitness_acum+=5.0f;
        estado=QUIETO;
      }
      // Pelota muy cerca → cambiar a despejar
      if(d_cent>0 && d_cent<12){ estado=DESPEJAR; t_estado=ahora; }
      break;
    }

    // ── DESPEJAR: único momento en que avanza, luego vuelve
    case DESPEJAR:
      if(ahora-t_estado < 280){
        int fuerza=(int)(155+genes[2]*80);
        mover(fuerza,fuerza);
      } else {
        mover(-155,-155); delay(180); // retrocede a posición
        parar();
        fitness_acum+=8.0f;
        estado=QUIETO; t_estado=ahora;
      }
      break;
  }

  // ── GA: mutación cada 10 ciclos ─────────────────────
  generacion++;
  if(generacion%10==0){
    for(int i=0;i<3;i++){
      float mut=(float)(random(200)-100)/1000.0f;
      genes[i]=constrain(genes[i]+mut,0.1f,1.0f);
    }
  }
  if(probs[clase]>0.80f && clase>0){
    genes[0]=constrain(genes[0]-0.01f,0.1f,0.9f);
    pso_pBest=pso_vel;
  }

  // ── Debug ────────────────────────────────────────────
  const char* noms[]={"PARED","OPONENTE","BALON_LIBRE","BALON_DISP"};
  const char* ests[]={"QUIETO","ORIENTAR","BLOQUEAR","DESPEJAR"};
  Serial.printf("[NN:%s %.0f%%] EST:%s | PELIGRO:%.2f ALIN:%.2f | JUG:dir=%d dist=%.1f | IZQ:%.1f C:%.1f DER:%.1f\n",
    noms[clase],probs[clase]*100,ests[estado],
    peligro,alineacion,
    datosJugador.direccion,datosJugador.distCentro,
    d_izq,d_cent,d_der);

  delay(20);
}
#include <Arduino.h>
#include <Encoder.h>
#include <L298NX2.h>

// Definição dos pinos
const int Enc1A = 2;
const int Enc1B = 3;
const int Enc2A = 20;
const int Enc2B = 21;
const int PWM1 = 9;
const int PWM2 = 10;
const int ENABLE = 11;

// Inicialização dos encoders
Encoder EncoderMotor(Enc1A, Enc1B);
Encoder EncoderPendulo(Enc2A, Enc2B);

// Variáveis globais para armazenar as posições
long countMotor = 0;
long countPendulo = 0;
float velocidadeAngularMotor = 0;
float velocidadeAngularPendulo = 0;
//valor final de cada variável
float posicaoMotor = 0;
float posicaoPendulo = 1;
unsigned long ultimoTempoMotor = 0;
unsigned long ultimoTempoPendulo = 0;
float grausMotor = 0;
float grausPendulo = 0;
const int histMax = 6;

// Vetores para armazenar os valores de pulsos
float CountsMotor[histMax] = {0,0,0,0,0};
float CountsPendulo[histMax] = {0,0,0,0,1500};

// Vetores para armazenar os valores de posição e velocidade
float PosicoesAngularMotor[histMax] = {0,0,0,0,0};
float velocidadesAngularMotor[histMax] = {0,0,0,0,0};
float PosicoesAngularPendulo[histMax] = {0,0,0,0,1500};
float velocidadesAngularPendulo[histMax] = {0,0,0,0,0};

float countsMotorAcumulados = 0;
float countsPenduloAcumulados = 0;
float PosicoesPenduloAcumuladas = 1500;
float velocidadeAngularPenduloAcumulada = 0;
float PosicoesMotorAcumuladas = 0;
float velocidadeAngularMotorAcumulada = 0;
int dir = 0;
int stepsdone = 0;
int passo = 0;

double tempo = 0.0;
double tempoInicial = 0.0;
double tempoAnterior = 0.0;

// Create a 4x1 matrix to store the K values
float K[4][1] = {{-1.7602}, {-171.0905}, {-4.2499}, {-4.3065}}; //{-1.60}, {-0.0443}, {2.563}, {-2.264} antigos


void acquisicaoMotor();
void acquisicaoPendulo();
int calcularPwm(float posicaoAngularMotor, float velocidadeAngularMotor, float posicaoPendulo, float velocidadeAngularPendulo, float K[4][1]);
void controlePendulo(int dir, int pwm);
void DoStep(int posicaoPendulo, int dir, int pwm);

void setup() {
  Serial.begin(9600);
  delay(1000);
  pinMode(Enc1A, INPUT_PULLUP);
  pinMode(Enc1B, INPUT_PULLUP);
  pinMode(Enc2A, INPUT_PULLUP);
  pinMode(Enc2B, INPUT_PULLUP);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  //Serial.println("Monitor Ligado.");
}

void loop() {
  acquisicaoPendulo();
  while(posicaoPendulo != 1){
    tempoInicial = micros() / 1000000.0;
    acquisicaoPendulo();
    acquisicaoMotor();
    DoStep(posicaoPendulo,dir,calcularPwm(posicaoMotor, velocidadeAngularMotor, posicaoPendulo, velocidadeAngularPendulo, K));
    if(stepsdone >= passo){
      stepsdone = 0;
    }
    tempo = tempoInicial - tempoAnterior;
    Serial.print(tempo);
    Serial.print(", ");
    Serial.print(countMotor);
    Serial.print(", ");
    Serial.print(grausMotor);
    Serial.print(", ");
    Serial.print(posicaoMotor);
    Serial.print(", ");
    Serial.print(velocidadeAngularMotor);
    Serial.print(", ");
    Serial.print(countPendulo);
    Serial.print(", ");
    Serial.print(grausPendulo);
    Serial.print(", ");
    Serial.print(posicaoPendulo);
    Serial.print(", ");
    // Serial.println(velocidadeAngularPendulo);
  }
}

void DoStep(int posicaoPendulo, int dir, int pwm ){
  if(posicaoPendulo > -1.67){
    passo = map(posicaoPendulo, 1, 0, 0, 5);
    dir = 1;
  }
  else if(posicaoPendulo < -1.67){
    passo = map(posicaoPendulo, 2, 1, 0, 5);
    dir = -1;
  }
  pwm = map(pwm, 0, 100, 0, 255);
  if (stepsdone <= passo && dir > 0){
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, LOW);
    analogWrite(ENABLE, pwm);
    //Serial.println("Motor A no sentido horário");
    stepsdone++;
  }
  else if (stepsdone <= passo && dir < 0){
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, HIGH);
    analogWrite(ENABLE, pwm);
    //Serial.println("Motor A no sentido anti-horário");
    stepsdone++;
  }
  else{
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
    analogWrite(ENABLE, 0);
    //Serial.println("Motor A parado");
  }
}

int calcularPwm(float posicaoAngularMotor, float velocidadeAngularMotor, float posicaoAngularPendulo, float velocidadeAngularPendulo, float K[4][1]){
  
  int pwm = K[0][0]*posicaoAngularMotor + K[1][0]*velocidadeAngularMotor + K[2][0]*posicaoAngularPendulo + K[3][0]*velocidadeAngularPendulo;

  return pwm;
}

void acquisicaoMotor() {
  long newCountsMotor = EncoderMotor.read();
  unsigned long tempoAtual = millis();
  
  countsMotorAcumulados -= CountsMotor[0];
  PosicoesMotorAcumuladas = PosicoesAngularMotor[0];
  velocidadeAngularMotorAcumulada -= velocidadesAngularMotor[0];
  for (int j = 0; j < histMax - 1; j++) {
    CountsMotor[j] = CountsMotor[j + 1];
    PosicoesAngularMotor[j] = PosicoesAngularMotor[j + 1]; 
    velocidadesAngularMotor[j] = velocidadesAngularMotor[j + 1];
  }
  CountsMotor[histMax - 1] = newCountsMotor;

  countsMotorAcumulados += CountsMotor[histMax - 1];

  countMotor = countsMotorAcumulados / histMax;
  long deltaPos = newCountsMotor - CountsMotor[histMax - 1];
  unsigned long deltaTime = tempoAtual - ultimoTempoMotor;

  ultimoTempoMotor = tempoAtual;

  // Cálculo de velocidade linear e angular
  grausMotor = (countMotor * 360.0) / 2000.0; // Posição em graus
  int posicaoAngularMotor = grausMotor * PI / 180.0; // Posição em radianos

  PosicoesAngularMotor[histMax - 1] = posicaoAngularMotor;
  PosicoesMotorAcumuladas += PosicoesAngularMotor[histMax - 1];

  posicaoMotor = PosicoesMotorAcumuladas / histMax;


  int velocidadeAngular = (deltaPos * 360.0 / 2000.0) * (PI / 180.0) / (deltaTime / 1000.0); // Velocidade angular em rad/s
  velocidadesAngularMotor[histMax - 1] = velocidadeAngular;
  velocidadeAngularMotorAcumulada += velocidadesAngularMotor[histMax - 1];
  velocidadeAngularMotor = velocidadeAngularMotorAcumulada / histMax;


  // Serial.print("Posição do Motor (Counts): ");
  // Serial.print(countMotor);
  // Serial.print("Posição do Motor (rad): ");
  // Serial.println(posicaoMotor);
  // Serial.print("Velocidade Angular Motor (rad/s): ");
  // Serial.println(velocidadeAngularMotor);
}

void acquisicaoPendulo() {
  long newCountsPendulo = EncoderPendulo.read();
  unsigned long tempoAtual = millis();

  countsPenduloAcumulados -= CountsPendulo[0];
  PosicoesPenduloAcumuladas = PosicoesAngularPendulo[0];
  velocidadeAngularPenduloAcumulada -= velocidadesAngularPendulo[0];
  for (int j = 0; j < histMax - 1; j++) {
    CountsPendulo[j] = CountsPendulo[j + 1];
    PosicoesAngularPendulo[j] = PosicoesAngularPendulo[j + 1]; 
    velocidadesAngularPendulo[j] = velocidadesAngularPendulo[j + 1];
  }
  CountsPendulo[histMax - 1] = newCountsPendulo;

  countsPenduloAcumulados += CountsPendulo[histMax - 1];

  countPendulo = countsPenduloAcumulados / histMax;
  long deltaPos = newCountsPendulo - countPendulo;


  unsigned long deltaTime = tempoAtual - ultimoTempoPendulo;

  ultimoTempoPendulo = tempoAtual;

  // Cálculo de velocidade linear e angular
  grausPendulo = (countPendulo * 360.0) / 300.0; // Posição em graus
  int posicaoAngularPendulo = grausPendulo * PI / 180.0; // Posição em radianos

  PosicoesAngularPendulo[histMax - 1] = posicaoAngularPendulo;
  PosicoesPenduloAcumuladas += PosicoesAngularPendulo[histMax - 1];

  posicaoPendulo = PosicoesPenduloAcumuladas / histMax;

  //dir = posicaoPendulo > 0 ? 1 : -1;

  int velocidadeAngular = (deltaPos * 360.0 / 300.0) * (PI / 180.0) / (deltaTime / 1000.0); // Velocidade angular em rad/s
  velocidadesAngularPendulo[histMax - 1] = velocidadeAngular;
  velocidadeAngularPenduloAcumulada += velocidadesAngularPendulo[histMax - 1];
  velocidadeAngularPendulo = velocidadeAngularPenduloAcumulada / histMax;


  // Serial.print("Posição do Pendulo (Counts): ");
  // Serial.print(countPendulo);
  // Serial.print("Direção do Pendulo: ");
  // Serial.println(dir);
  // Serial.print("Posição do Pendulo (rad): ");
  // Serial.println(posicaoPendulo);
  // Serial.print("Velocidade Angular Pendulo (rad/s): ");
  // Serial.println(velocidadeAngularPendulo);
}

void controlePendulo(int dir, int pwm) {

  //Serial.print("PWM: ");
  if (pwm<0){
    pwm = -pwm;
  }
  pwm = map(pwm, 0, 100, 0, 255);
  //Serial.println(pwm);
  if (dir > 0) {
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, HIGH);
    analogWrite(ENABLE, pwm);
    Serial.println("Motor A no sentido horário");
  } else if (dir < 0){
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, LOW);
    analogWrite(ENABLE, pwm);
    Serial.println("Motor A no sentido anti-horário");
  }
  else{
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
    analogWrite(ENABLE, 0);
    Serial.println("Motor A parado");
  }
}
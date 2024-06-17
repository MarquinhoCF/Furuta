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
float CountsPendulo[histMax] = {0,0,0,0,0};

// Vetores para armazenar os valores de posição e velocidade
float PosicoesAngularMotor[histMax] = {0,0,0,0,0};
float velocidadesAngularMotor[histMax] = {0,0,0,0,0};
float PosicoesAngularPendulo[histMax] = {0,0,0,0,0};
float velocidadesAngularPendulo[histMax] = {0,0,0,0,0};

float countsMotorAcumulados = 0;
float countsPenduloAcumulados = 0;
float PosicoesPenduloAcumuladas = 0;
float velocidadeAngularPenduloAcumulada = 0;
float PosicoesMotorAcumuladas = 0;
float velocidadeAngularMotorAcumulada = 0;



// Create a 4x1 matrix to store the K values
float K[4][1] = {{-1.60}, {-0.0443}, {2.563}, {-2.264}}; //{-1.60}, {-0.0443}, {2.563}, {-2.264} antigos


void acquisicaoMotor();
void acquisicaoPendulo();
int calcularPwm(float posicaoAngularMotor, float velocidadeAngularMotor, float posicaoPendulo, float velocidadeAngularPendulo, float K[4][1]);
void controlePendulo(float posicaoAngularMotor,float posicaoAngularPendulo, int pwm);

void setup() {
  Serial.begin(112500);
  pinMode(Enc1A, INPUT_PULLUP);
  pinMode(Enc1B, INPUT_PULLUP);
  pinMode(Enc2A, INPUT_PULLUP);
  pinMode(Enc2B, INPUT_PULLUP);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  Serial.println("Monitor Ligado.");
}

void loop() {
  while(posicaoPendulo != 0){
    acquisicaoMotor();
    acquisicaoPendulo();
    controlePendulo(posicaoMotor ,posicaoPendulo, calcularPwm(posicaoMotor, velocidadeAngularMotor, posicaoPendulo, velocidadeAngularPendulo, K));
    if(posicaoPendulo != 0 && posicaoMotor == 0){
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, LOW);
    analogWrite(ENABLE, 255);
    Serial.println("Dando um toquinho");
    }
  }
  if (posicaoPendulo == 1){
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
    analogWrite(ENABLE, 0);
    Serial.println("Motor A parado");
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
  long deltaPos = newCountsMotor - countMotor;
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


  Serial.print("Posição do Motor (Counts): ");
  Serial.print(countMotor);
  Serial.print("Posição do Motor (rad): ");
  Serial.println(posicaoMotor);
  Serial.print("Velocidade Angular Motor (rad/s): ");
  Serial.println(velocidadeAngularMotor);


  delay(100);

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
  grausPendulo = (countPendulo * 360.0) / 2000.0; // Posição em graus
  int posicaoAngularPendulo = grausPendulo * PI / 180.0; // Posição em radianos

  PosicoesAngularPendulo[histMax - 1] = posicaoAngularPendulo;
  PosicoesPenduloAcumuladas += PosicoesAngularPendulo[histMax - 1];

  posicaoPendulo = PosicoesPenduloAcumuladas / histMax;


  int velocidadeAngular = (deltaPos * 360.0 / 2000.0) * (PI / 180.0) / (deltaTime / 1000.0); // Velocidade angular em rad/s
  velocidadesAngularPendulo[histMax - 1] = velocidadeAngular;
  velocidadeAngularPenduloAcumulada += velocidadesAngularPendulo[histMax - 1];
  velocidadeAngularPendulo = velocidadeAngularPenduloAcumulada / histMax;


  Serial.print("Posição do Motor (Counts): ");
  Serial.print(countPendulo);
  Serial.print("Posição do Motor (rad): ");
  Serial.println(posicaoPendulo);
  Serial.print("Velocidade Angular Motor (rad/s): ");
  Serial.println(velocidadeAngularPendulo);


  delay(100);

}

void controlePendulo(float posicaoMotor, float posicaoAngularPendulo, int pwm) {

  Serial.print("PWM: ");
  if (pwm<0){
    pwm = -pwm;
  }
  Serial.println(pwm);
  pwm = map(pwm, 0, 100, 0, 255);
  if (posicaoMotor < -0.3  && posicaoMotor < 0 && posicaoAngularPendulo < 0) {
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, LOW);
    analogWrite(ENABLE, pwm);
    Serial.println("Motor A no sentido horário");
  } else if (posicaoMotor > 0.3 && posicaoMotor > 0 && posicaoAngularPendulo > 0) {
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, HIGH);
    analogWrite(ENABLE, pwm);
    Serial.println("Motor A no sentido anti-horário");
  }
    
}
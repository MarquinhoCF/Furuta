#include <Arduino.h>
#include <Encoder.h>
#include <L298NX2.h>

// Definição dos pinos
const int Enc1A = 2;
const int Enc1B = 3;
const int Enc2A = 4;
const int Enc2B = 5;
const int PWM1 = 7;
const int PWM2 = 8;
const int ENABLE = 9;

// Inicialização dos encoders
Encoder EncoderMotor(Enc1A, Enc1B);
Encoder EncoderPendulo(Enc2A, Enc2B);

// Variáveis globais para armazenar as posições
long posicaoMotor = 0;
long posicaoPendulo = 0;
unsigned long ultimoTempoMotor = 0;
unsigned long ultimoTempoPendulo = 0;
float grausMotor = 0;
float grausPendulo = 0;
float posicaoAngularMotor = 0;
float posicaoAngularPendulo = 0;
float velocidadeAngularMotor = 0;
float velocidadeAngularPendulo = 0;

// Create a 4x1 matrix to store the K values
float K[4][1] = {{0}, {0}, {0}, {0}};

void acquisicaoMotor();
void acquisicaoPendulo();
int calcularPwm(float posicaoAngularMotor, float velocidadeAngularMotor, float posicaoPendulo, float velocidadeAngularPendulo, float K[4][1]);
void controlePendulo(float posicaoAngularPendulo, int pwm);

void setup() {
  Serial.begin(112500);
  pinMode(Enc1A, INPUT);
  pinMode(Enc1B, INPUT);
  pinMode(Enc2A, INPUT);
  pinMode(Enc2B, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(ENABLE, OUTPUT);

  Serial.println("Monitor Ligado.");
}

void loop() {
  acquisicaoMotor();
  acquisicaoPendulo();
  if(posicaoAngularPendulo != 0){
    while(posicaoAngularPendulo!=0){
      acquisicaoMotor();
      acquisicaoPendulo();
      controlePendulo(posicaoAngularPendulo, calcularPwm(posicaoAngularMotor, velocidadeAngularMotor, posicaoAngularPendulo, velocidadeAngularPendulo, K));
    }
  }
  if(posicaoAngularPendulo == 0){
    posicaoAngularMotor = 0;
    velocidadeAngularMotor = 0;
    posicaoAngularPendulo = 0;
    velocidadeAngularPendulo = 0;
    grausMotor = 0;
    grausPendulo = 0;
  }
}

int calcularPwm(float posicaoAngularMotor, float velocidadeAngularMotor, float posicaoPendulo, float velocidadeAngularPendulo, float K[4][1]){
  
  int pwm = K[0][0]*posicaoAngularMotor + K[1][0]*velocidadeAngularMotor + K[2][0]*posicaoAngularPendulo + K[3][0]*velocidadeAngularPendulo;

  return pwm;
}

void acquisicaoMotor() {
  long novaPosMotor = EncoderMotor.read();
  unsigned long tempoAtual = millis();

  if (novaPosMotor != posicaoMotor) {
    if ((novaPosMotor >= 2000) || (novaPosMotor <= -2000)) {
      EncoderMotor.readAndReset();
      novaPosMotor = 0;
    }
    long deltaPos = novaPosMotor - posicaoMotor;
    unsigned long deltaTime = tempoAtual - ultimoTempoMotor;

    posicaoMotor = novaPosMotor;
    ultimoTempoMotor = tempoAtual;

    // Cálculo de velocidade linear e angular
    grausMotor = (posicaoMotor * 360.0) / 2000.0; // Posição em graus
    posicaoAngularMotor = grausMotor * PI / 180.0; // Posição em radianos
    velocidadeAngularMotor = (deltaPos * 360.0 / 2000.0) * (PI / 180.0) / (deltaTime / 1000.0); // Velocidade angular em rad/s

    Serial.println("Posição do Motor (Counts): ");
    Serial.print(posicaoMotor);
    Serial.println("Posição do Motor (rad): ");
    Serial.print(posicaoAngularMotor);
    Serial.println("Velocidade Angular Motor (rad/s): ");
    Serial.print(velocidadeAngularMotor);
  
  }
}

void acquisicaoPendulo() {
  long novaPosPendulo = EncoderPendulo.read();
  unsigned long tempoAtual = millis();

  if (novaPosPendulo != posicaoPendulo) {
    if ((novaPosPendulo >= 600) || (novaPosPendulo <= -600)) {
      EncoderPendulo.readAndReset();
      novaPosPendulo = 0;
    }


    long deltaPos = novaPosPendulo - posicaoPendulo;
    unsigned long deltaTime = tempoAtual - ultimoTempoPendulo;

    posicaoPendulo = novaPosPendulo;
    ultimoTempoPendulo = tempoAtual;

    // Cálculo de velocidade linear e angular
    grausPendulo = (posicaoPendulo * 360.0) / 600.0; // Posição em graus
    posicaoAngularPendulo = grausPendulo * PI / 180.0; // Posição em radianos
    velocidadeAngularPendulo = (deltaPos * 360.0 / 600.0) * (PI / 180.0) / (deltaTime / 1000.0); // Velocidade angular em rad/s

    Serial.println("Posição Pêndulo (Counts): ");
    Serial.print(posicaoPendulo);
    Serial.println("Posição Pêndulo (rad): ");
    Serial.print(posicaoAngularPendulo);
    Serial.println("Velocidade Angular Pêndulo (rad/s): ");
    Serial.print(velocidadeAngularPendulo);
  }
}

void controlePendulo(float posicaoAngularPendulo, int pwm) {
  int pwmValue = map(pwm, 0, 100, 0, 255);

  if (posicaoAngularPendulo < -1) {
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, LOW);
    for (int i = 0; i <= pwmValue; i += 10) {
      analogWrite(ENABLE, i);
      delay(10); // Pequena pausa para criar a rampa de aceleração
    }
    Serial.println("Motor A no sentido horário");
  } else if (posicaoAngularPendulo > 1) {
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, HIGH);
    for (int i = 0; i <= pwmValue; i += 10) {
      analogWrite(ENABLE, i);
      delay(10); // Pequena pausa para criar a rampa de aceleração
    }
    Serial.println("Motor A no sentido anti-horário");
  } else {
    for (int i = pwmValue; i >= 0; i -= 10) {
      analogWrite(ENABLE, i);
      delay(10); // Pequena pausa para criar a rampa de desaceleração
    }
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
    Serial.println("Motor A parado");
  }
}
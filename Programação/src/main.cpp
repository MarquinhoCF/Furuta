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
float K[4][1] = {{5}, {5}, {5}, {5}}; //{-1.60}, {-0.0443}, {2.563}, {-2.264} antigos


void acquisicaoMotor();
void acquisicaoPendulo();
int calcularPwm(float posicaoAngularMotor, float velocidadeAngularMotor, float posicaoPendulo, float velocidadeAngularPendulo, float K[4][1]);
void controlePendulo(float posicaoAngularPendulo, int pwm);

void setup() {
  Serial.begin(9600);
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
  while(posicaoAngularPendulo != 1 or posicaoAngularPendulo != -1){
    acquisicaoMotor();
    acquisicaoPendulo();
    controlePendulo(posicaoAngularPendulo, calcularPwm(posicaoAngularMotor, velocidadeAngularMotor, posicaoAngularPendulo, velocidadeAngularPendulo, K));
    if(posicaoAngularPendulo == 0 && posicaoAngularMotor == 0){
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, LOW);
    analogWrite(ENABLE, 255);
    Serial.println("Dando um toquinho");
    }
  }
  if (posicaoAngularPendulo == 1 or posicaoAngularPendulo == -1){
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
    for (int i = 255; i >= 255; i -= 255) {
      analogWrite(ENABLE, i);
    }
    Serial.println("Motor A parado");
  }
  
}

int calcularPwm(float posicaoAngularMotor, float velocidadeAngularMotor, float posicaoAngularPendulo, float velocidadeAngularPendulo, float K[4][1]){
  
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

    // Serial.print("Posição do Motor (Counts): ");
    // Serial.print(posicaoMotor);
    // Serial.print("Posição do Motor (rad): ");
    // Serial.println(posicaoAngularMotor);
    // Serial.print("Velocidade Angular Motor (rad/s): ");
    // Serial.println(velocidadeAngularMotor);


    //delay(100);
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

    // Serial.print("Posicao Pendulo (Counts): ");
    // Serial.println(posicaoPendulo);
    // Serial.print("Posicao Pendulo (rad): ");
    // Serial.println(posicaoAngularPendulo);
    // Serial.print("Velocidade Angular Pendulo (rad/s): ");
    // Serial.println(velocidadeAngularPendulo);

    //delay(100);
  }
}

void controlePendulo(float posicaoAngularPendulo, int pwm) {

  Serial.print("PWM: ");
  if (pwm<0){
    pwm = -pwm;
  }
  Serial.println(pwm);
  pwm = map(pwm, 0, 100, 0, 255);
  if (posicaoAngularMotor < -0.3  && posicaoAngularMotor < 0 && posicaoAngularPendulo < 0) {
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, LOW);
    analogWrite(ENABLE, pwm);
    Serial.println("Motor A no sentido horário");
  } else if (posicaoAngularMotor > 0.3 && posicaoAngularMotor > 0 && posicaoAngularPendulo > 0) {
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, HIGH);
    analogWrite(ENABLE, pwm);
    Serial.println("Motor A no sentido anti-horário");
  }
    
}
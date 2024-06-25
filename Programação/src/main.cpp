#include <Arduino.h>
#include <Encoder.h>

// Definition of pins
const int Enc1A = 2;
const int Enc1B = 3;
const int Enc2A = 20;
const int Enc2B = 21;
const int PWM1 = 9;
const int PWM2 = 10;
const int ENABLE = 11;

// Initialization of encoders
Encoder EncoderMotor(Enc1A, Enc1B);
Encoder EncoderPendulo(Enc2A, Enc2B);

// Global variables to store positions
long countMotor = 0;
long countPendulo = 0;
double velocidadeAngularMotor = 0;
double velocidadeAngularPendulo = 0;
double posicaoMotor = 0;
double posicaoPendulo = 0;
double ultimoTempoMotor = 0;
double ultimoTempoPendulo = 0;
double grausMotor = 0;
double grausPendulo = 0;
const int histMax = 6;

// Arrays to store pulse values
double CountsMotor[histMax] = {0,0,0,0,0};
double CountsPendulo[histMax] = {0,0,0,0,0};

// Arrays to store position and velocity values
double PosicoesAngularMotor[histMax] = {0,0,0,0,0};
double velocidadesAngularMotor[histMax] = {0,0,0,0,0};
double PosicoesAngularPendulo[histMax] = {0,0,0,0,0};
double velocidadesAngularPendulo[histMax] = {0,0,0,0,0};
double ErroRadPendulo = 0;
double ErroGrausMotor = 0;
double ErroRadMotor = 0;
double ErroGrausPendulo = 0;

double countsMotorAcumulados = 0;
double countsPenduloAcumulados = 0;
double PosicoesPenduloAcumuladas = 0;
double velocidadeAngularPenduloAcumulada = 0;
double PosicoesMotorAcumuladas = 0;
double velocidadeAngularMotorAcumulada = 0;
double grausPenduloAnt = 0;
double grausMotorAnt = 0;

double setPoint_SwingUp_GrausPendulo = 180.00;
double setPoint_Equilibrio_GrausPendulo = 180.00;
double setPointRad_Motor = 0;
double setPointRad_Pendulo = 1;
double setPointGraus_Motor = 0;
double tempo = 0.0;
double tempoInicial = 0.0;
double tempoAnterior = 0.0;
bool stop = 0;
bool mode = 0;
int dir = 0;

double deltaErroGrausPendulo = 0;
double ErroAntGrausPendulo = 0;

double ErroLimiteEsquerdo = 0.00;
double LimiteEsquerdo = 90.00;
double ErroLimiteDireito = 0.00;
double LimiteDireito = -90.00;

double deltaErroGrausMotor = 0;
double ErroAntGraus = 0;


// Create a 4x1 matrix to store the K values
double K[4][1] ={{-1},{-1.279},{-30},{-40}}; // New values{{-0.1}, {-0.1279}, {-15.6637}, {-20.101}}; // Old values

// Function declarations
void resetEquilibrio();
void Equilibrar();
void SwingUp(); 
void acquisicaoMotor();
void acquisicaoPendulo();
int calcularPwm(double posicaoAngularMotor, double velocidadeAngularMotor, double posicaoPendulo, double velocidadeAngularPendulo, double K[4][1]);
void controlePendulo(double pwm, int mode);
void DoStep(int posicaoPendulo, int dir, double pwm);
void motorParado();
void resetSwingUp();

void setup() {
  Serial.begin(9600);
  pinMode(Enc1A, INPUT_PULLUP);
  pinMode(Enc1B, INPUT_PULLUP);
  pinMode(Enc2A, INPUT_PULLUP);
  pinMode(Enc2B, INPUT_PULLUP);
  pinMode(PWM1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  //attachInterrupt(digitalPinToInterrupt(Enc1A), acquisicaoMotor, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(Enc2A), acquisicaoPendulo, CHANGE);
}

void loop() {
  Serial.println("tá no loop");
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    if (command == "Equilibrar") {
      Serial.println("Equilibrar");
      Equilibrar();
      Serial.println("Saiu Equilibrar");
    }
    if (command == "SwingUp") {
      Serial.println("SwingUp");
      SwingUp();
      Serial.println("Saiu SwingUp");
    }
  }
}

void Equilibrar(){
  mode = 0;
  Serial.println("passou aq");
  while (true) {
    if(Serial.available() > 0){
      String command = Serial.readStringUntil('\n');
      if (command == "Reset") {
        resetSwingUp(); 
      }
      if (command == "Start") {
        stop = 0;
        Serial.println("Start");
        EncoderMotor.readAndReset();
        EncoderPendulo.readAndReset();
        resetSwingUp();
        EncoderPendulo.write(0);
        while(stop == 0){
          Serial.println("Tá aq dentro");
          acquisicaoPendulo();
          acquisicaoMotor();
          while (ErroGrausPendulo != 0 && ErroGrausMotor != 0) {
            tempoInicial = micros() / 1000000.0;
            acquisicaoPendulo();
            acquisicaoMotor();
            double pwm = calcularPwm(posicaoMotor, velocidadeAngularMotor, posicaoPendulo, velocidadeAngularPendulo, K);
            controlePendulo(pwm, mode);
            tempo = tempoInicial - tempoAnterior;
            // Serial.print(tempo);
            // Serial.print(", ");
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
            Serial.print(velocidadeAngularPendulo);
            Serial.print(", ");
            Serial.print(pwm);
            Serial.print(", ");
            Serial.print(mode);
            Serial.print(", ");
            Serial.println(dir);

            if (Serial.available() > 0) {
              String command = Serial.readStringUntil('\n');
              if (command == "Stop") {
                digitalWrite(PWM1, LOW);
                digitalWrite(PWM2, LOW);
                analogWrite(ENABLE, 0);
                break;
                stop = 1;
              }
            }
          }
          if(ErroGrausPendulo == 0){
            digitalWrite(PWM1, LOW);
            digitalWrite(PWM2, LOW);
            analogWrite(ENABLE, 0);
          }
          if (Serial.available() > 0) {
            String command = Serial.readStringUntil('\n');
            if (command == "Stop") {
              break;
            }
          }
        }
      }
      if (command == "Stop") {
        break;
      }
    }
  }
}


void SwingUp(){
  mode = 1;
  Serial.println("passou aq");
  while (true) {
    if(Serial.available() > 0){
      String command = Serial.readStringUntil('\n');
      if (command == "Reset") {
        resetEquilibrio(); 
      }
      if (command == "Start") {
        stop = 0;
        Serial.println("Start");
        EncoderMotor.readAndReset();
        EncoderPendulo.readAndReset();
        resetEquilibrio();
        EncoderPendulo.write(1400);
        while(stop == 0){
          Serial.println("Tá aq dentro");
          acquisicaoPendulo();
          acquisicaoMotor();
          while (ErroGrausPendulo != 0  && ErroGrausMotor != 0) {
            tempoInicial = micros() / 1000000.0;
            acquisicaoPendulo();
            acquisicaoMotor();
            double pwm = calcularPwm(posicaoMotor, velocidadeAngularMotor, posicaoPendulo, velocidadeAngularPendulo, K);
            controlePendulo(pwm, mode);
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
            Serial.print(velocidadeAngularPendulo);
            Serial.print(", ");

            if (Serial.available() > 0) {
              String command = Serial.readStringUntil('\n');
              if (command == "Stop") {
                digitalWrite(PWM1, LOW);
                digitalWrite(PWM2, LOW);
                analogWrite(ENABLE, 0);
                stop = 1;
                break;
              }
            }
          }
          if(ErroGrausPendulo == 0){
            digitalWrite(PWM1, LOW);
            digitalWrite(PWM2, LOW);
            analogWrite(ENABLE, 0);
          }
          if (Serial.available() > 0) {
              String command = Serial.readStringUntil('\n');
              if (command == "Stop") {
                break;
              }
          }
        }
      }
      if (command == "Stop") {
        break;
      }
    }
  }
}

void resetEquilibrio(){
  // Reset all list variables and accumulators
  for (int i = 0; i < histMax; i++) {
    CountsMotor[i] = 0;
    CountsPendulo[i] = 1400;
    PosicoesAngularMotor[i] = 0;
    velocidadesAngularMotor[i] = 0;
    PosicoesAngularPendulo[i] = 1;
    velocidadesAngularPendulo[i] = 0;
  }
  countsMotorAcumulados = 0;
  countsPenduloAcumulados = 7000;
  PosicoesPenduloAcumuladas = 5;
  velocidadeAngularPenduloAcumulada = 0;
  PosicoesMotorAcumuladas = 0;
  velocidadeAngularMotorAcumulada = 0;
  EncoderMotor.readAndReset();
  EncoderPendulo.readAndReset();
  EncoderPendulo.write(1400);
}

void resetSwingUp(){
  // Reset all list variables and accumulators
  for (int i = 0; i < histMax; i++) {
    CountsMotor[i] = 0;
    CountsPendulo[i] = 0;
    PosicoesAngularMotor[i] = 0;
    velocidadesAngularMotor[i] = 0;
    PosicoesAngularPendulo[i] = 0;
    velocidadesAngularPendulo[i] = 0;
  }
  countsMotorAcumulados = 0;
  countsPenduloAcumulados = 0;
  PosicoesPenduloAcumuladas = 0;
  velocidadeAngularPenduloAcumulada = 0;
  PosicoesMotorAcumuladas = 0;
  velocidadeAngularMotorAcumulada = 0;
  EncoderMotor.readAndReset();
  EncoderPendulo.readAndReset();
  EncoderPendulo.write(0);
}

int calcularPwm(double posicaoAngularMotor, double velocidadeAngularMotor, double posicaoAngularPendulo, double velocidadeAngularPendulo, double K[4][1]){
  
  double pwm = K[0][0]*ErroRadMotor + K[1][0]*velocidadeAngularMotor + K[2][0]*ErroRadPendulo + K[3][0]*velocidadeAngularPendulo;

  return pwm;
}

void acquisicaoMotor() {
  long newCountsMotor = EncoderMotor.read();
  double tempoAtual = millis();
  
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

  countMotor = countsMotorAcumulados / (histMax-1);

  long deltaPos = newCountsMotor - CountsMotor[histMax - 2];

  unsigned long deltaTime = tempoAtual - ultimoTempoMotor;

  ultimoTempoMotor = tempoAtual;

  grausMotor = (countMotor * 360.0) / 2400.0; // Position in degrees
  double posicaoAngularMotor = grausMotor * PI / 180.0; // Position in radians

  PosicoesAngularMotor[histMax - 1] = posicaoAngularMotor;
  PosicoesMotorAcumuladas += PosicoesAngularMotor[histMax - 1];

  posicaoMotor = PosicoesMotorAcumuladas / (histMax-1);

  double deltaGrausMotor = grausMotor - grausMotorAnt;
  grausMotorAnt = grausMotor;

  int velocidadeAngular = (deltaGrausMotor) * ( PI / 180.0) / (deltaTime / 1000.0); // Angular velocity in rad/s
  velocidadesAngularMotor[histMax - 1] = velocidadeAngular;
  velocidadeAngularMotorAcumulada += velocidadesAngularMotor[histMax - 1];
  velocidadeAngularMotor = velocidadeAngularMotorAcumulada / (histMax-1);

  ErroRadMotor = setPointRad_Motor - posicaoMotor;
  ErroGrausMotor = setPointGraus_Motor - grausMotor ;

  ErroLimiteEsquerdo = grausMotor - LimiteEsquerdo;
  ErroLimiteDireito = LimiteDireito - grausMotor;
}

void acquisicaoPendulo() {
  long newCountsPendulo = EncoderPendulo.read();
  double tempoAtual = millis();

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

  countPendulo = countsPenduloAcumulados / (histMax-1);
  float deltaPos = newCountsPendulo - CountsPendulo[histMax - 2];

  double deltaTime = tempoAtual - ultimoTempoPendulo;

  ultimoTempoPendulo = tempoAtual;

  grausPendulo = (countPendulo * 360.0) / 2800.0; // Position in degrees
  double posicaoAngularPendulo = grausPendulo * PI / 180.0; // Position in radians

  PosicoesAngularPendulo[histMax - 1] = posicaoAngularPendulo;
  PosicoesPenduloAcumuladas += PosicoesAngularPendulo[histMax - 1];
  
  double deltaGraus = grausPendulo - grausPenduloAnt;
  grausPenduloAnt = grausPendulo;

  posicaoPendulo = PosicoesPenduloAcumuladas / (histMax-1);

  int velocidadeAngular = (deltaGraus) * ( PI / 180.0) / (deltaTime / 1000.0); // Angular velocity in rad/s
  velocidadesAngularPendulo[histMax - 1] = velocidadeAngular;
  velocidadeAngularPenduloAcumulada += velocidadesAngularPendulo[histMax - 1];
  velocidadeAngularPendulo = velocidadeAngularPenduloAcumulada / (histMax-1);

  ErroRadPendulo = setPointRad_Pendulo - posicaoPendulo;
  ErroGrausPendulo = setPoint_Equilibrio_GrausPendulo - grausPendulo;
  deltaErroGrausPendulo = ErroGrausPendulo - ErroAntGrausPendulo;
  ErroAntGrausPendulo = ErroGrausPendulo;
}

void motorParado(){
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  analogWrite(ENABLE, 0);
}
void controlePendulo(double pwm, int mode) {

  pwm = abs(pwm);
  pwm = map(pwm, 0, 100, 0, 255);

  if(deltaErroGrausPendulo > 0 or (ErroLimiteDireito > 0 && (ErroLimiteEsquerdo < -90 && ErroLimiteEsquerdo < 0))){
      dir = -1;
    }
  if(deltaErroGrausPendulo < 0 or (ErroLimiteEsquerdo > 0 && (ErroLimiteDireito < -90 && ErroLimiteDireito < 0))){
    dir = 1;
  }
  if(mode == 1){
    if(dir == 1){
      digitalWrite(PWM1, HIGH);
      digitalWrite(PWM2, LOW);
      analogWrite(ENABLE, pwm);
    }
    if(dir == -1){
      digitalWrite(PWM1, LOW);
      digitalWrite(PWM2, HIGH);
      analogWrite(ENABLE, pwm);
    }
  }
  if(mode == 0){
    if(dir == -1){
      digitalWrite(PWM1, LOW);
      digitalWrite(PWM2, HIGH);
      analogWrite(ENABLE, pwm);
      //Serial.println("Direita");
    }
    if(dir == 1){
      digitalWrite(PWM1, HIGH);
      digitalWrite(PWM2, LOW);
      analogWrite(ENABLE, pwm);
      //Serial.println("Esquerda");
    }
  }
}

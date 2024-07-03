#include <Arduino.h>
#include <Encoder.h>
// Definição dos pinos
const int Enc1A = 2; // Pino 1A do encoder
const int Enc1B = 3; // Pino 1B do encoder
const int Enc2A = 20; // Pino 2A do encoder
const int Enc2B = 21; // Pino 2B do encoder
const int PWM1 = 9; // Pino PWM 1
const int PWM2 = 10; // Pino PWM 2
const int ENABLE = 11; // Pino de habilitação

// Inicialização dos encoders
Encoder EncoderMotor(Enc1A, Enc1B); // Encoder para o motor
Encoder EncoderPendulo(Enc2A, Enc2B); // Encoder para o pêndulo

// Variáveis globais para armazenar as posições
long countMotor = 0; // Contagem do encoder do motor
long countPendulo = 0; // Contagem do encoder do pêndulo
double velocidadeAngularMotor = 0; // Velocidade angular do motor
double velocidadeAngularPendulo = 0; // Velocidade angular do pêndulo
double posicaoMotor = 0; // Posição do motor em radianos
double posicaoPendulo = 0; // Posição do pêndulo em radianos
double ultimoTempoMotor = 0; // Último tempo para o motor
double ultimoTempoPendulo = 0; // Último tempo para o pêndulo
double grausMotor = 0; // Posição do motor em graus
double grausPendulo = 0; // Posição do pêndulo em graus
const int histMax = 6; // Tamanho máximo do histórico

// Arrays para armazenar os valores de pulsos
double CountsMotor[histMax] = {0,0,0,0,0}; // Contagem de pulsos do motor
double CountsPendulo[histMax] = {0,0,0,0,0}; // Contagem de pulsos do pêndulo

// Arrays para armazenar os valores de posição e velocidade
double PosicoesAngularMotor[histMax] = {0,0,0,0,0}; // Posições angulares do motor
double velocidadesAngularMotor[histMax] = {0,0,0,0,0}; // Velocidades angulares do motor
double PosicoesAngularPendulo[histMax] = {0,0,0,0,0}; // Posições angulares do pêndulo
double velocidadesAngularPendulo[histMax] = {0,0,0,0,0}; // Velocidades angulares do pêndulo
double ErroRadPendulo = 0; // Erro de posição do pêndulo em radianos
double ErroGrausMotor = 0; // Erro de posição do motor em graus
double ErroRadMotor = 0; // Erro de posição do motor em radianos
double ErroGrausPendulo = 0; // Erro de posição do pêndulo em graus

double countsMotorAcumulados = 0; // Contagem acumulada do motor
double countsPenduloAcumulados = 0; // Contagem acumulada do pêndulo
double PosicoesPenduloAcumuladas = 0; // Posições acumuladas do pêndulo
double velocidadeAngularPenduloAcumulada = 0; // Velocidades angulares acumuladas do pêndulo
double PosicoesMotorAcumuladas = 0; // Posições acumuladas do motor
double velocidadeAngularMotorAcumulada = 0; // Velocidades angulares acumuladas do motor
double grausPenduloAnt = 0; // Posição anterior do pêndulo em graus
double grausMotorAnt = 0; // Posição anterior do motor em graus

double setPoint_SwingUp_GrausPendulo = 180.00; // Setpoint de Swing-up para o pêndulo em graus
double setPoint_Equilibrio_GrausPendulo = 180.00; // Setpoint de equilíbrio para o pêndulo em graus
double setPointRad_Motor = 0; // Setpoint para a posição do motor em radianos
double setPointRad_Pendulo = 1; // Setpoint para a posição do pêndulo em radianos
double setPointGraus_Motor = 0; // Setpoint para a posição do motor em graus
double tempo = 0.0; // Intervalo de tempo
double tempoInicial = 0.0; // Tempo inicial
double tempoAnterior = 0.0; // Tempo anterior
double tempoUltimoDegrau = 0.0; // Tempo do último degrau
double degrauTempo = 0.3; // Tempo do degrau
bool stop = 0; // Flag de parada
bool mode = 0; // Flag de modo

// Variáveis para armazenar o estado atual e o tempo do último degrau
bool estadoDegrau = LOW; // Estado do degrau
int dir = 0; // Flag de direção

double deltaErroGrausPendulo = 0; // Variação do erro de posição do pêndulo em graus
double ErroAntGrausPendulo = 0; // Erro de posição anterior do pêndulo em graus

double ErroLimiteEsquerdo = 0.00; // Erro do limite esquerdo
double LimiteEsquerdo = 90.00; // Limite esquerdo em graus
double ErroLimiteDireito = 0.00; // Erro do limite direito
double LimiteDireito = -90.00; // Limite direito em graus

double deltaErroGrausMotor = 0; // Variação do erro de posição do motor em graus
double ErroAntGraus = 0; // Erro de posição anterior do motor em graus

// Criação de uma matriz 4x1 para armazenar os valores de K
double K[4][1] ={{-1},{-1.279},{-30},{-40}}; // Novos valores{{-0.1}, {-0.1279}, {-15.6637}, {-20.101}}; // Valores antigos

// Declaração das funções
void resetEquilibrio();
void Equilibrar();
void SwingUp(); 
void acquisicaoMotor();
void acquisicaoPendulo();
int calcularPwm(double posicaoAngularMotor, double velocidadeAngularMotor, double posicaoPendulo, double velocidadeAngularPendulo, double K[4][1]);
void controlePendulo(double pwm, int mode);
void motorParado();
void resetSwingUp();
void Acquisitar();

void setup() {
  Serial.begin(9600); // Inicializa a comunicação serial
  pinMode(Enc1A, INPUT_PULLUP); // Define o pino 1A do encoder como entrada com resistor de pull-up
  pinMode(Enc1B, INPUT_PULLUP); // Define o pino 1B do encoder como entrada com resistor de pull-up
  pinMode(Enc2A, INPUT_PULLUP); // Define o pino 2A do encoder como entrada com resistor de pull-up
  pinMode(Enc2B, INPUT_PULLUP); // Define o pino 2B do encoder como entrada com resistor de pull-up
  pinMode(PWM1, OUTPUT); // Define o pino PWM 1 como saída
  pinMode(PWM2, OUTPUT); // Define o pino PWM 2 como saída
  pinMode(ENABLE, OUTPUT); // Define o pino de habilitação como saída
}

void loop() {
  // Imprime uma mensagem para indicar que o código está na função loop
  //Serial.println("tá no loop");

  // Verifica se há algum dado disponível no buffer serial
  if (Serial.available() > 0) {
    // Lê o comando recebido do buffer serial
    String command = Serial.readStringUntil('\n');

    // Verifica se o comando é "Equilibrar"
    if (command == "Equilibrar") {
      // Imprime uma mensagem para indicar que o código está na função Equilibrar
      //Serial.println("Equilibrar");
      // Chama a função Equilibrar
      Equilibrar();
      // Imprime uma mensagem para indicar que o código saiu da função Equilibrar
      //Serial.println("Saiu Equilibrar");
    }

    // Verifica se o comando é "SwingUp"
    if (command == "SwingUp") {
      // Imprime uma mensagem para indicar que o código está na função SwingUp
      //Serial.println("SwingUp");
      // Chama a função SwingUp
      SwingUp();
    }

    // Verifica se o comando é "Acquisitar"
    if (command == "Acquisitar") {
      // Imprime uma mensagem para indicar que o código está na função Acquisitar
      //Serial.println("Acquisitar");
      // Chama a função Acquisitar
      Acquisitar();
    }
  }
}

void Equilibrar(){
  mode = 0;
  //Serial.println("passou aq");
  while (true) {
    if(Serial.available() > 0){
      String command = Serial.readStringUntil('\n');
      if (command == "Reset") {
        resetSwingUp(); 
      }
      if (command == "Start") {
        stop = 0;
        //Serial.println("Start");
        EncoderMotor.readAndReset();
        EncoderPendulo.readAndReset();
        resetSwingUp();
        EncoderPendulo.write(0);
        while(stop == 0){
          //Serial.println("Tá aq dentro");
          acquisicaoPendulo();
          acquisicaoMotor();
          while (ErroGrausPendulo != 0 && ErroGrausMotor != 0) {
            tempoInicial = micros() / 1000000.0;
            acquisicaoPendulo();
            acquisicaoMotor();
            double pwm = calcularPwm(posicaoMotor, velocidadeAngularMotor, posicaoPendulo, velocidadeAngularPendulo, K);
            pwm = abs(pwm);
            double volts = map(pwm, 0, 100, 0, 24);
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
            Serial.print(pwm);
            Serial.print(", ");
            Serial.println(volts);

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
  //Serial.println("passou aq");
  while (true) {
    if(Serial.available() > 0){
      String command = Serial.readStringUntil('\n');
      if (command == "Reset") {
        resetEquilibrio(); 
      }
      if (command == "Start") {
        stop = 0;
        //Serial.println("Start");
        EncoderMotor.readAndReset();
        EncoderPendulo.readAndReset();
        resetEquilibrio();
        EncoderPendulo.write(1400);
        while(stop == 0){
          //Serial.println("Tá aq dentro");
          acquisicaoPendulo();
          acquisicaoMotor();
          while (ErroGrausPendulo != 0  && ErroGrausMotor != 0) {
            tempoInicial = micros() / 1000000.0;
            acquisicaoPendulo();
            acquisicaoMotor();
            double pwm = calcularPwm(posicaoMotor, velocidadeAngularMotor, posicaoPendulo, velocidadeAngularPendulo, K);
            pwm = abs(pwm);
            double volts = map(pwm, 0, 100, 0, 24);
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
            Serial.print(pwm);
            Serial.print(", ");
            Serial.println(volts);

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

void Acquisitar(){
  //Serial.println("passou aq");
  while (true) {
    if(Serial.available() > 0){
      String command = Serial.readStringUntil('\n');
      if (command == "Reset") {
        resetSwingUp(); 
      }
      if (command == "Start") {
        stop = 0;
        //Serial.println("Start");
        EncoderMotor.readAndReset();
        EncoderPendulo.readAndReset();
        resetSwingUp();
        EncoderPendulo.write(0);
        while(stop == 0){
          //Serial.println("Tá aq dentro");
          acquisicaoPendulo();
          acquisicaoMotor();
          while (ErroGrausPendulo != 0  && ErroGrausMotor != 0) {
            tempoInicial = micros() / 1000000.0;
            acquisicaoPendulo();
            acquisicaoMotor();
            int pwm = 255;
            double volts = map(pwm, 0, 255, 0, 24);
            // Atualize o tempo atual
            double tempoAtual = micros() / 1000000.0;
            // Verifique se o tempo do degrau expirou
            if (tempoAtual - tempoUltimoDegrau >= degrauTempo) {
              // Mude o estado do degrau
              estadoDegrau = !estadoDegrau;

              // Atualize o pino do LED com o novo estado
              digitalWrite(PWM1, estadoDegrau);
              digitalWrite(PWM2, !estadoDegrau);
              analogWrite(ENABLE, pwm);

              // Atualize o tempo do último degrau
              tempoUltimoDegrau = tempoAtual;
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
            Serial.print(velocidadeAngularPendulo);
            Serial.print(", ");
            Serial.print(pwm);
            Serial.print(", ");
            Serial.println(volts);

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
  if(newCountsMotor == CountsMotor[histMax - 1] && newCountsMotor == CountsMotor[histMax - 2] && newCountsMotor == CountsMotor[histMax - 3] && newCountsMotor == CountsMotor[histMax - 4] && newCountsMotor == CountsMotor[histMax - 5]) {
    EncoderMotor.readAndReset();
    newCountsMotor = EncoderMotor.read();
  }
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

  grausMotorAnt = grausMotor;

  int velocidadeAngular = (deltaPos) * ( PI / 180.0) / (deltaTime / 1000.0); // Angular velocity in rad/s
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
  if(newCountsPendulo == CountsPendulo[histMax - 1] && newCountsPendulo == CountsPendulo[histMax - 2] && newCountsPendulo == CountsPendulo[histMax - 3] && newCountsPendulo == CountsPendulo[histMax - 4] && newCountsPendulo == CountsPendulo[histMax - 5]) {
    EncoderPendulo.readAndReset();
    newCountsPendulo = EncoderPendulo.read();
  }
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
  grausPenduloAnt = grausPendulo;

  posicaoPendulo = PosicoesPenduloAcumuladas / (histMax-1);

  int velocidadeAngular = (deltaPos) * ( PI / 180.0) / (deltaTime / 1000.0); // Angular velocity in rad/s
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
  pwm = map(pwm, 0, 100, 0, 255);

  if(deltaErroGrausPendulo > 0 or (ErroLimiteDireito > 0 && (ErroLimiteEsquerdo < -70 && ErroLimiteEsquerdo < 0))){
      dir = -1;
    }
  if(deltaErroGrausPendulo < 0 or (ErroLimiteEsquerdo > 0 && (ErroLimiteDireito < -70 && ErroLimiteDireito < 0))){
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

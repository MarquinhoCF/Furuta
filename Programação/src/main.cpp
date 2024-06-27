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
bool stop = 0; // Flag de parada
bool mode = 0; // Flag de modo
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
int calcularPwm(double posicaoAngularMotor, double velocidadeAngularMotor, double posicaoAngularPendulo, double velocidadeAngularPendulo, double K[4][1]);
void controlePendulo(double pwm, int mode);
void DoStep(int posicaoPendulo, int dir, double pwm);
void motorParado();
void resetSwingUp();

void setup() {
  Serial.begin(9600);
  pinMode(Enc1A, INPUT_PULLUP); // Define o pino 1A do encoder como entrada com resistor de pull-up
  pinMode(Enc1B, INPUT_PULLUP); // Define o pino 1B do encoder como entrada com resistor de pull-up
  pinMode(Enc2A, INPUT_PULLUP); // Define o pino 2A do encoder como entrada com resistor de pull-up
  pinMode(Enc2B, INPUT_PULLUP); // Define o pino 2B do encoder como entrada com resistor de pull-up
  pinMode(PWM1, OUTPUT); // Define o pino PWM 1 como saída
  pinMode(PWM2, OUTPUT); // Define o pino PWM 2 como saída
  pinMode(ENABLE, OUTPUT); // Define o pino de habilitação como saída
  //attachInterrupt(digitalPinToInterrupt(Enc1A), acquisicaoMotor, CHANGE);
  //attachInterrupt(digitalPinToInterrupt(Enc2A), acquisicaoPendulo, CHANGE);
}
void loop() {
  // Imprime uma mensagem para indicar que o código está na função loop
  Serial.println("tá no loop");

  // Verifica se há algum dado disponível no buffer serial
  if (Serial.available() > 0) {
    // Lê o comando recebido do buffer serial
    String command = Serial.readStringUntil('\n');

    // Verifica se o comando é "Equilibrar"
    if (command == "Equilibrar") {
      // Imprime uma mensagem para indicar que o código está na função Equilibrar
      Serial.println("Equilibrar");
      // Chama a função Equilibrar
      Equilibrar();
      // Imprime uma mensagem para indicar que o código saiu da função Equilibrar
      Serial.println("Saiu Equilibrar");
    }

    // Verifica se o comando é "SwingUp"
    if (command == "SwingUp") {
      // Imprime uma mensagem para indicar que o código está na função SwingUp
      Serial.println("SwingUp");
      // Chama a função SwingUp
      SwingUp();
    }
  }
}
void Equilibrar(){
  // Define a flag de modo como 0 para indicar que o código está no modo Equilibrar
  mode = 0;

  // Executa um loop infinito até encontrar uma instrução break
  while (true) {
    // Verifica se há algum dado disponível no buffer serial
    if(Serial.available() > 0){
      // Lê o comando recebido do buffer serial
      String command = Serial.readStringUntil('\n');

      // Verifica se o comando é "Reset"
      if (command == "Reset") {
        // Chama a função resetSwingUp
        resetSwingUp(); 
      }

      // Verifica se o comando é "Start"
      if (command == "Start") {
        // Define a flag de parada como 0 para indicar que o código não deve parar
        stop = 0;
        // Imprime uma mensagem para indicar que o código foi iniciado
        Serial.println("Start");
        // Reinicia as contagens dos encoders do motor e do pêndulo
        EncoderMotor.readAndReset();
        EncoderPendulo.readAndReset();
        // Chama a função resetSwingUp
        resetSwingUp();
        // Define a posição do pêndulo como 0
        EncoderPendulo.write(0);

        // Executa um loop até que a flag de parada seja definida como 1
        while(stop == 0){
          // Imprime uma mensagem para indicar que o código está dentro do loop
          Serial.println("");
          // Lê os valores dos encoders do motor e do pêndulo
          acquisicaoPendulo();
          acquisicaoMotor();

          // Executa um loop até que os erros de posição do motor e do pêndulo sejam ambos 0
          while (ErroGrausPendulo != 0 && ErroGrausMotor != 0) {
            // Obtém o tempo atual em segundos
            tempoInicial = micros() / 1000000.0;
            // Lê os valores dos encoders do motor e do pêndulo
            acquisicaoPendulo();
            acquisicaoMotor();
            // Calcula o valor do PWM com base nas posições e velocidades atuais do motor e do pêndulo
            double pwm = calcularPwm(posicaoMotor, velocidadeAngularMotor, posicaoPendulo, velocidadeAngularPendulo, K);
            // Controla o pêndulo usando o valor do PWM calculado
            controlePendulo(pwm, mode);
            // Calcula o intervalo de tempo
            tempo = tempoInicial - tempoAnterior;

            // Imprime os valores para fins de depuração
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

            // Verifica se há algum dado disponível no buffer serial
            if (Serial.available() > 0) {
              // Lê o comando recebido do buffer serial
              String command = Serial.readStringUntil('\n');
              // Verifica se o comando é "Stop"
              if (command == "Stop") {
                // Para o motor e o pêndulo
                digitalWrite(PWM1, LOW);
                digitalWrite(PWM2, LOW);
                analogWrite(ENABLE, 0);
                // Sai do loop
                break;
                // Define a flag de parada como 1 para indicar que o código deve parar
                stop = 1;
              }
            }
          }

          // Verifica se o erro de posição do pêndulo é 0
          if(ErroGrausPendulo == 0){
            // Para o motor e o pêndulo
            digitalWrite(PWM1, LOW);
            digitalWrite(PWM2, LOW);
            analogWrite(ENABLE, 0);
          }

          // Verifica se há algum dado disponível no buffer serial
          if (Serial.available() > 0) {
            // Lê o comando recebido do buffer serial
            String command = Serial.readStringUntil('\n');
            // Verifica se o comando é "Stop"
            if (command == "Stop") {
              // Sai do loop
              break;
            }
          }
        }
      }

      // Verifica se o comando é "Stop"
      if (command == "Stop") {
        // Sai do loop
        break;
      }
    }
  }
}

void SwingUp(){
  mode = 1; // Define a flag de modo como 1
  Serial.println("passou aq"); // Imprime uma mensagem no monitor serial
  while (true) { // Loop infinito
    if(Serial.available() > 0){ // Verifica se há dados disponíveis no buffer serial
      String command = Serial.readStringUntil('\n'); // Lê o comando recebido do buffer serial
      if (command == "Reset") { // Verifica se o comando é "Reset"
        resetEquilibrio(); // Chama a função resetEquilibrio()
      }
      if (command == "Start") { // Verifica se o comando é "Start"
        stop = 0; // Define a flag de parada como 0
        Serial.println("Start"); // Imprime uma mensagem no monitor serial
        EncoderMotor.readAndReset(); // Lê e reseta o encoder do motor
        EncoderPendulo.readAndReset(); // Lê e reseta o encoder do pêndulo
        resetEquilibrio(); // Chama a função resetEquilibrio()
        EncoderPendulo.write(1400); // Escreve um valor no encoder do pêndulo
        while(stop == 0){ // Loop enquanto a flag de parada for 0
          Serial.println("Tá aq dentro"); // Imprime uma mensagem no monitor serial
          acquisicaoPendulo(); // Chama a função acquisicaoPendulo()
          acquisicaoMotor(); // Chama a função acquisicaoMotor()
          while (ErroGrausPendulo != 0  && ErroGrausMotor != 0) { // Loop enquanto o erro de posição do pêndulo e do motor não forem 0
            tempoInicial = micros() / 1000000.0; // Obtém o tempo inicial em segundos
            acquisicaoPendulo(); // Chama a função acquisicaoPendulo()
            acquisicaoMotor(); // Chama a função acquisicaoMotor()
            double pwm = calcularPwm(posicaoMotor, velocidadeAngularMotor, posicaoPendulo, velocidadeAngularPendulo, K); // Calcula o valor do PWM
            controlePendulo(pwm, mode); // Chama a função controlePendulo()
            tempo = tempoInicial - tempoAnterior; // Calcula o intervalo de tempo
            Serial.print(tempo); // Imprime o intervalo de tempo no monitor serial
            Serial.print(", "); // Imprime o separador
            Serial.print(countMotor); // Imprime a contagem do encoder do motor no monitor serial
            Serial.print(", "); // Imprime o separador
            Serial.print(grausMotor); // Imprime a posição do motor em graus no monitor serial
            Serial.print(", "); // Imprime o separador
            Serial.print(posicaoMotor); // Imprime a posição do motor em radianos no monitor serial
            Serial.print(", "); // Imprime o separador
            Serial.print(velocidadeAngularMotor); // Imprime a velocidade angular do motor no monitor serial
            Serial.print(", "); // Imprime o separador
            Serial.print(countPendulo); // Imprime a contagem do encoder do pêndulo no monitor serial
            Serial.print(", "); // Imprime o separador
            Serial.print(grausPendulo); // Imprime a posição do pêndulo em graus no monitor serial
            Serial.print(", "); // Imprime o separador
            Serial.print(posicaoPendulo); // Imprime a posição do pêndulo em radianos no monitor serial
            Serial.print(", "); // Imprime o separador
            Serial.print(velocidadeAngularPendulo); // Imprime a velocidade angular do pêndulo no monitor serial
            Serial.print(", "); // Imprime o separador

            if (Serial.available() > 0) { // Verifica se há dados disponíveis no buffer serial
              String command = Serial.readStringUntil('\n'); // Lê o comando recebido do buffer serial
              if (command == "Stop") { // Verifica se o comando é "Stop"
                digitalWrite(PWM1, LOW); // Define o pino PWM1 como LOW
                digitalWrite(PWM2, LOW); // Define o pino PWM2 como LOW
                analogWrite(ENABLE, 0); // Define o pino ENABLE como 0
                stop = 1; // Define a flag de parada como 1
                break; // Sai do loop
              }
            }
          }
          if(ErroGrausPendulo == 0){ // Verifica se o erro de posição do pêndulo é 0
            digitalWrite(PWM1, LOW); // Define o pino PWM1 como LOW
            digitalWrite(PWM2, LOW); // Define o pino PWM2 como LOW
            analogWrite(ENABLE, 0); // Define o pino ENABLE como 0
          }
          if (Serial.available() > 0) { // Verifica se há dados disponíveis no buffer serial
              String command = Serial.readStringUntil('\n'); // Lê o comando recebido do buffer serial
              if (command == "Stop") { // Verifica se o comando é "Stop"
                break; // Sai do loop
              }
          }
        }
      }
      if (command == "Stop") { // Verifica se o comando é "Stop"
        break; // Sai do loop
      }
    }
  }
}
// Função para redefinir o estado de equilíbrio
void resetEquilibrio(){
  // Redefinir todas as variáveis de lista e acumuladores
  for (int i = 0; i < histMax; i++) {
    CountsMotor[i] = 0; // Redefinir contagem de pulsos do motor
    CountsPendulo[i] = 1400; // Redefinir contagem de pulsos do pêndulo
    PosicoesAngularMotor[i] = 0; // Redefinir posições angulares do motor
    velocidadesAngularMotor[i] = 0; // Redefinir velocidades angulares do motor
    PosicoesAngularPendulo[i] = 1; // Redefinir posições angulares do pêndulo
    velocidadesAngularPendulo[i] = 0; // Redefinir velocidades angulares do pêndulo
  }
  countsMotorAcumulados = 0; // Redefinir contagem acumulada do motor
  countsPenduloAcumulados = 7000; // Redefinir contagem acumulada do pêndulo
  PosicoesPenduloAcumuladas = 5; // Redefinir posições acumuladas do pêndulo
  velocidadeAngularPenduloAcumulada = 0; // Redefinir velocidades angulares acumuladas do pêndulo
  PosicoesMotorAcumuladas = 0; // Redefinir posições acumuladas do motor
  velocidadeAngularMotorAcumulada = 0; // Redefinir velocidades angulares acumuladas do motor
  EncoderMotor.readAndReset(); // Redefinir encoder do motor
  EncoderPendulo.readAndReset(); // Redefinir encoder do pêndulo
  EncoderPendulo.write(1400); // Definir posição do encoder do pêndulo como 1400
}
// Função para redefinir o estado de swing-up
void resetSwingUp(){
  // Redefinir todas as variáveis de lista e acumuladores
  for (int i = 0; i < histMax; i++) {
    CountsMotor[i] = 0; // Redefinir contagem de pulsos do motor
    CountsPendulo[i] = 0; // Redefinir contagem de pulsos do pêndulo
    PosicoesAngularMotor[i] = 0; // Redefinir posições angulares do motor
    velocidadesAngularMotor[i] = 0; // Redefinir velocidades angulares do motor
    PosicoesAngularPendulo[i] = 0; // Redefinir posições angulares do pêndulo
    velocidadesAngularPendulo[i] = 0; // Redefinir velocidades angulares do pêndulo
  }
  countsMotorAcumulados = 0; // Redefinir contagem acumulada do motor
  countsPenduloAcumulados = 0; // Redefinir contagem acumulada do pêndulo
  PosicoesPenduloAcumuladas = 0; // Redefinir posições acumuladas do pêndulo
  velocidadeAngularPenduloAcumulada = 0; // Redefinir velocidades angulares acumuladas do pêndulo
  PosicoesMotorAcumuladas = 0; // Redefinir posições acumuladas do motor
  velocidadeAngularMotorAcumulada = 0; // Redefinir velocidades angulares acumuladas do motor
  EncoderMotor.readAndReset(); // Redefinir encoder do motor
  EncoderPendulo.readAndReset(); // Redefinir encoder do pêndulo
  EncoderPendulo.write(0); // Definir posição do encoder do pêndulo como 0
}

// Função para calcular o valor do PWM
int calcularPwm(double posicaoAngularMotor, double velocidadeAngularMotor, double posicaoAngularPendulo, double velocidadeAngularPendulo, double K[4][1]){
  
  double pwm = K[0][0]*ErroRadMotor + K[1][0]*velocidadeAngularMotor + K[2][0]*ErroRadPendulo + K[3][0]*velocidadeAngularPendulo;

  return pwm;
}

// Função para adquirir dados do motor
void acquisicaoMotor() {
  long newCountsMotor = EncoderMotor.read(); // Ler o valor do encoder do motor
  double tempoAtual = millis(); // Obter o tempo atual
  
  countsMotorAcumulados -= CountsMotor[0]; // Subtrair a contagem de pulsos mais antiga do motor
  PosicoesMotorAcumuladas = PosicoesAngularMotor[0]; // Definir a posição do motor como o valor mais antigo
  velocidadeAngularMotorAcumulada -= velocidadesAngularMotor[0]; // Subtrair a velocidade angular mais antiga do motor
  for (int j = 0; j < histMax - 1; j++) {
    CountsMotor[j] = CountsMotor[j + 1]; // Deslocar as contagens de pulsos do motor
    PosicoesAngularMotor[j] = PosicoesAngularMotor[j + 1]; // Deslocar as posições angulares do motor
    velocidadesAngularMotor[j] = velocidadesAngularMotor[j + 1]; // Deslocar as velocidades angulares do motor
  }
  CountsMotor[histMax - 1] = newCountsMotor; // Armazenar a nova contagem de pulsos do motor

  countsMotorAcumulados += CountsMotor[histMax - 1]; // Adicionar a nova contagem de pulsos do motor

  countMotor = countsMotorAcumulados / (histMax-1); // Calcular a contagem média de pulsos do motor

  long deltaPos = newCountsMotor - CountsMotor[histMax - 2]; // Calcular a variação na posição do motor

  unsigned long deltaTime = tempoAtual - ultimoTempoMotor; // Calcular o intervalo de tempo

  ultimoTempoMotor = tempoAtual; // Atualizar o último tempo para o motor

  grausMotor = (countMotor * 360.0) / 2400.0; // Calcular a posição do motor em graus
  double posicaoAngularMotor = grausMotor * PI / 180.0; // Calcular a posição do motor em radianos

  PosicoesAngularMotor[histMax - 1] = posicaoAngularMotor; // Armazenar a nova posição angular do motor
  PosicoesMotorAcumuladas += PosicoesAngularMotor[histMax - 1]; // Adicionar a nova posição angular do motor

  posicaoMotor = PosicoesMotorAcumuladas / (histMax-1); // Calcular a posição média do motor

  double deltaGrausMotor = grausMotor - grausMotorAnt; // Calcular a variação na posição do motor em graus
  grausMotorAnt = grausMotor; // Atualizar a posição anterior do motor

  int velocidadeAngular = (deltaGrausMotor) * ( PI / 180.0) / (deltaTime / 1000.0); // Calcular a velocidade angular do motor em rad/s
  velocidadesAngularMotor[histMax - 1] = velocidadeAngular; // Armazenar a nova velocidade angular do motor
  velocidadeAngularMotorAcumulada += velocidadesAngularMotor[histMax - 1]; // Adicionar a nova velocidade angular do motor
  velocidadeAngularMotor = velocidadeAngularMotorAcumulada / (histMax-1); // Calcular a velocidade angular média do motor

  ErroRadMotor = setPointRad_Motor - posicaoMotor; // Calcular o erro de posição do motor em radianos
  ErroGrausMotor = setPointGraus_Motor - grausMotor ; // Calcular o erro de posição do motor em graus

  ErroLimiteEsquerdo = grausMotor - LimiteEsquerdo; // Calcular o erro do limite esquerdo
  ErroLimiteDireito = LimiteDireito - grausMotor; // Calcular o erro do limite direito

}

void acquisicaoPendulo() {
  // Ler a contagem do encoder do pêndulo e o tempo atual
  long newCountsPendulo = EncoderPendulo.read();
  double tempoAtual = millis();

  // Atualizar as contagens acumuladas, posições e velocidades
  countsPenduloAcumulados -= CountsPendulo[0];
  PosicoesPenduloAcumuladas = PosicoesAngularPendulo[0];
  velocidadeAngularPenduloAcumulada -= velocidadesAngularPendulo[0];
  for (int j = 0; j < histMax - 1; j++) {
    CountsPendulo[j] = CountsPendulo[j + 1];
    PosicoesAngularPendulo[j] = PosicoesAngularPendulo[j + 1]; 
    velocidadesAngularPendulo[j] = velocidadesAngularPendulo[j + 1];
  }
  CountsPendulo[histMax - 1] = newCountsPendulo;

  // Atualizar as contagens acumuladas
  countsPenduloAcumulados += CountsPendulo[histMax - 1];

  // Calcular a contagem média
  countPendulo = countsPenduloAcumulados / (histMax-1);

  // Calcular a variação na posição e o intervalo de tempo
  double deltaPos = newCountsPendulo - CountsPendulo[histMax - 2];
  double deltaTime = tempoAtual - ultimoTempoPendulo;

  // Atualizar o último tempo para o pêndulo
  ultimoTempoPendulo = tempoAtual;

  // Calcular a posição do pêndulo em graus e radianos
  grausPendulo = (countPendulo * 360.0) / 2800.0; // Posição em graus
  double posicaoAngularPendulo = grausPendulo * PI / 180.0; // Posição em radianos

  // Atualizar a posição e as posições acumuladas
  PosicoesAngularPendulo[histMax - 1] = posicaoAngularPendulo;
  PosicoesPenduloAcumuladas += PosicoesAngularPendulo[histMax - 1];
  
  // Atualizar a posição anterior do pêndulo
  grausPenduloAnt = grausPendulo;

  // Calcular a posição média
  posicaoPendulo = PosicoesPenduloAcumuladas / (histMax-1);

  // Calcular a velocidade angular em rad/s
  int velocidadeAngular = (deltaPos) * ( PI / 180.0) / (deltaTime / 1000.0);
  velocidadesAngularPendulo[histMax - 1] = velocidadeAngular;
  velocidadeAngularPenduloAcumulada += velocidadesAngularPendulo[histMax - 1];
  velocidadeAngularPendulo = velocidadeAngularPenduloAcumulada / (histMax-1);

  // Calcular o erro de posição do pêndulo em radianos e graus
  ErroRadPendulo = setPointRad_Pendulo - posicaoPendulo;
  ErroGrausPendulo = setPoint_Equilibrio_GrausPendulo - grausPendulo;

  // Calcular a variação no erro de posição do pêndulo em graus
  deltaErroGrausPendulo = ErroGrausPendulo - ErroAntGrausPendulo;
  ErroAntGrausPendulo = ErroGrausPendulo;
}

void motorParado(){
  // Parar o motor
  digitalWrite(PWM1, LOW);
  digitalWrite(PWM2, LOW);
  analogWrite(ENABLE, 0);
}

// Função para controlar o pêndulo
void controlePendulo(double pwm, int mode) {

  pwm = abs(pwm); // Garantir que o pwm seja positivo
  pwm = map(pwm, 0, 100, 0, 255); // Mapear o valor do pwm para o intervalo 0-255

  // Determinar a direção com base na variação do erro de posição do pêndulo e nos erros de limite
  if(deltaErroGrausPendulo > 0 or (ErroLimiteDireito > 0 && (ErroLimiteEsquerdo < -70 && ErroLimiteEsquerdo < 0))){
      dir = -1; // Definir a direção como -1 (direita)
    }
  if(deltaErroGrausPendulo < 0 or (ErroLimiteEsquerdo > 0 && (ErroLimiteDireito < -70 && ErroLimiteDireito < 0))){
    dir = 1; // Definir a direção como 1 (esquerda)
  }
  if(mode == 1){ // Se o modo for 1 (modo normal)
    if(dir == 1){ // Se a direção for 1 (esquerda)
      digitalWrite(PWM1, HIGH); // Definir o pino PWM1 como HIGH
      digitalWrite(PWM2, LOW); // Definir o pino PWM2 como LOW
      analogWrite(ENABLE, pwm); // Definir o pino ENABLE com o valor do pwm
    }
    if(dir == -1){ // Se a direção for -1 (direita)
      digitalWrite(PWM1, LOW); // Definir o pino PWM1 como LOW
      digitalWrite(PWM2, HIGH); // Definir o pino PWM2 como HIGH
      analogWrite(ENABLE, pwm); // Definir o pino ENABLE com o valor do pwm
    }
  }
  if(mode == 0){ // Se o modo for 0 (modo reverso)
    if(dir == -1){ // Se a direção for -1 (direita)
      digitalWrite(PWM1, LOW); // Definir o pino PWM1 como LOW
      digitalWrite(PWM2, HIGH); // Definir o pino PWM2 como HIGH
      analogWrite(ENABLE, pwm); // Definir o pino ENABLE com o valor do pwm
      //Serial.println("Direita");
    }
    if(dir == 1){ // Se a direção for 1 (esquerda)
      digitalWrite(PWM1, HIGH); // Definir o pino PWM1 como HIGH
      digitalWrite(PWM2, LOW); // Definir o pino PWM2 como LOW
      analogWrite(ENABLE, pwm); // Definir o pino ENABLE com o valor do pwm
      //Serial.println("Esquerda");
    }
  }
}

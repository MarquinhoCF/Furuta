#include<Arduino.h>
#include<Encoder.h>
#include<L298NX2.h>

const int Enc1A = 2;
const int Enc1B = 3;
const int Enc2A = 4;
const int Enc2B = 5;
const int PWM1 = 7;
const int PWM2 = 8;
const int ENABLE = 9;
Encoder EncoderMotor(2, 3);
Encoder EncoderPendulo(4, 5);
long positionMotor  = -999;
long velocMotor = -999;
long grausMotor = -360;
long VelocidadeAngularMotor = -999;
long positionPendulo = -999;
long velocPendulo = -999;
long grausPendulo = -360;
long VelocidadeAngularPendulo = -999;


void AcquisicaoMotor();
void AcquisicaoPendulo();
void controlePendulo(long grausPendulo, int pwm);


void setup() {
  
  Serial.begin(9600);
  pinMode(Enc1A,INPUT);
  pinMode(Enc1B,INPUT);
  pinMode(Enc2A,INPUT);
  pinMode(Enc2B,INPUT);
  pinMode(PWM1,OUTPUT);
  pinMode(PWM2,OUTPUT);
  pinMode(ENABLE,OUTPUT);


  Serial.println("Monitor Ligado.");
  // put your setup code here, to run once:
}

void loop() {

  AcquisicaoMotor();
  AcquisicaoPendulo();
  controlePendulo(grausPendulo, 40);

}

void AcquisicaoMotor(){
  long newPosMotor = EncoderMotor.read();
  if (newPosMotor != positionMotor) {
    if(newPosMotor > 2000){
      EncoderMotor.readAndReset();
    }
    positionMotor = newPosMotor;
    velocMotor = positionMotor/(millis()*2000); //velocidade linear
    grausMotor = ((positionMotor*360)/2000)/180; //já converte pra pi rad
    VelocidadeAngularMotor = grausMotor/millis(); //velocidade angular em pi rad/s

  }
  Serial.println("Posição: ");
    Serial.println(positionMotor);
    Serial.println("Velocidade: ");
    Serial.println(velocMotor);
    Serial.println("Graus de posição:");
    Serial.println(grausMotor);
    Serial.println("Velocidade Angular:");
    Serial.println(VelocidadeAngularMotor);
}

void AcquisicaoPendulo(){
  long newPosPendulo = EncoderPendulo.read();
  if (newPosPendulo != positionPendulo) {
    if(newPosPendulo > 800){
      EncoderPendulo.readAndReset();
    }
    positionPendulo = newPosPendulo;
    velocPendulo = positionPendulo/(millis()*800); //velocidade linear
    grausPendulo = ((positionPendulo*360)/800)/180; //já converte pra pi rad
    VelocidadeAngularPendulo = grausPendulo/millis(); //velocidade angular em pi rad/s

  }
  Serial.println("Posição: ");
    Serial.println(positionPendulo);
    Serial.println("Velocidade: ");
    Serial.println(velocPendulo);
    Serial.println("Graus de posição:");
    Serial.println(grausPendulo);
    Serial.println("Velocidade Angular:");
    Serial.println(VelocidadeAngularPendulo);
}

void controlePendulo(long grausPendulo, int pwm){
  if (grausPendulo < -1 ){
    digitalWrite(PWM1, HIGH);
    digitalWrite(PWM2, LOW);
    for (int i = 0; i < pwm; i=i+10){ 
      analogWrite(ENABLE, ((i*255)/100));
    }
    Serial.println("Motor A no sentido horario");
  }
  if (grausPendulo > 1 ){
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, HIGH);
    for (int i = 0; i < pwm; i=i+10){ 
      analogWrite(ENABLE, ((i*255)/100));
    }
    Serial.println("Motor A no sentido anti-horario");
  }
  else{
    digitalWrite(PWM1, LOW);
    digitalWrite(PWM2, LOW);
    for (int i = 0; i < pwm; i=i-10){ 
      analogWrite(ENABLE, ((i*255)/100));
    }
    Serial.println("Motor A parado");
  }

}
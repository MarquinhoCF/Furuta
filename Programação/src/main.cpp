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
Encoder myEnc(2, 3);
long position  = -999;
long veloc = -999;
long graus = -360;
long distancia = -999;

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
  long newPos = myEnc.read();
  if (newPos != position) {
    position = newPos;
    veloc = position/(millis()*2000);
    graus = position/360;
  }
  Serial.println("Posição: ");
    Serial.println(position);
    Serial.println("Velocidade: ");
    Serial.println(veloc);
    Serial.println("Graus de posição:");
    Serial.println(graus);
  // With any substantial delay added, Encoder can only track
  // very slow motion.  You may uncomment this line to see
  // how badly a delay affects your encoder.
  delay(50);

 //Gira o Motor A no sentido horario
 digitalWrite(PWM1, HIGH);
 digitalWrite(PWM2, LOW);
 delay(500);
 //Para o motor A
 digitalWrite(PWM1, HIGH);
 digitalWrite(PWM2, HIGH);
 delay(500);
 
 //Gira o Motor A no sentido anti-horario 
 digitalWrite(PWM1, LOW);
 digitalWrite(PWM2, HIGH);
 delay(500);
 //Para o motor A
 digitalWrite(PWM1, HIGH);
 digitalWrite(PWM2, HIGH);
 delay(500);
}
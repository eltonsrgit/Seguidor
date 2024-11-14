#include <SoftwareSerial.h>

SoftwareSerial BT(53, 52); // rx, tx

#include <MPU6050_tockn.h>
#include <Wire.h>

#include "Acelerometro.h"
#include "encoder.h"
#include "encoder2.h"

#define motorEsq1 6
#define motorEsq2 7
#define motorDir1 4
#define motorDir2 5

#define S1 12
#define S2 11
#define S3 10
#define S4 9
#define S5 8

int linha = 1;
int pista = 0;

bool running = false;
bool printing = false;
bool mpu = false;
int Sensores[] = {S1, S2, S3, S4, S5}; // vetor para sensores
int Leitura[5] = {0}; 

int I = 0, P = 0, D = 0, PID = 0;

float Kp = 100.0;  // Ganho Proporcional = 80
float Ki = 0.0;  // Ganho Integral = 1
float Kd = 30.0;  // Ganho Derivativo = 60

int vel_esq = 100, vel_dir = vel_esq + 95; //inicial

int Speed_esq = 0, Speed_dir = 0; //final com pid

float erro = 0, erro_anterior = 0; // Variáveis de erro

unsigned long time = 0;
unsigned long last_time = 0;
unsigned long dt = time - last_time;

void seguidorStop(){
  digitalWrite (motorEsq1, LOW);
  digitalWrite (motorEsq2, LOW);
  digitalWrite (motorDir1, LOW);
  digitalWrite (motorDir2, LOW);
}

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);

  pinMode(motorEsq1, OUTPUT);
  pinMode(motorEsq2, OUTPUT);
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);

  Wire.begin();
  BT.println("Espere 3 segundos...");

  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  pinMode(CH1, INPUT);
  pinMode(CH2, INPUT);

  pinMode(CH3, INPUT);
  pinMode(CH4, INPUT);

  attachInterrupt(digitalPinToInterrupt(CH1), Contador_Pulso1, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH2), Contador_Pulso2, CHANGE);  
  attachInterrupt(digitalPinToInterrupt(CH3), Contador_Pulso3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(CH4), Contador_Pulso4, CHANGE);

  BT.println("Kp: " + String(Kp));
  BT.println("Ki: " + String(Ki));
  BT.println("Kd: " + String(Kd));
  BT.println("###  MENU  ###");
  BT.println("pid.value = valores atuais P.I.D.");
  BT.println("'Kp, Ki ou Kd' seguido de .change(valor inteiro) altera valores.");
  BT.println("M1 = stop.");
  BT.println("M2 = run.");
  BT.println("SEGUIDOR PRONTO!");

}

void leituraSensores(){
  for (int i = 0; i < 5; i++) {
    Leitura[i] = digitalRead(Sensores[i]);
  }
}



float calculoErro() {
  leituraSensores();
  if      (Leitura[0] == pista && Leitura[1] == pista && Leitura[2] == linha && Leitura[3] == pista && Leitura[4] == pista) { erro =  0; }
  else if (Leitura[0] == linha && Leitura[1] == linha && Leitura[2] == linha && Leitura[3] == linha && Leitura[4] == linha) { erro =  0; }
  else if (Leitura[0] == pista && Leitura[1] == pista && Leitura[2] == linha && Leitura[3] == linha && Leitura[4] == pista) { erro =  1; }
  else if (Leitura[0] == pista && Leitura[1] == linha && Leitura[2] == linha && Leitura[3] == pista && Leitura[4] == pista) { erro = -1; }
  else if (Leitura[0] == pista && Leitura[1] == pista && Leitura[2] == pista && Leitura[3] == linha && Leitura[4] == linha) { erro =  2; }
  else if (Leitura[0] == linha && Leitura[1] == linha && Leitura[2] == pista && Leitura[3] == pista && Leitura[4] == pista) { erro = -2; }
  else if (Leitura[0] == pista && Leitura[1] == pista && Leitura[2] == pista && Leitura[3] == pista && Leitura[4] == linha) { erro =  2.5; }
  else if (Leitura[0] == linha && Leitura[1] == pista && Leitura[2] == pista && Leitura[3] == pista && Leitura[4] == pista) { erro = -2.5; }

  
  return erro;
}


void pid(){
  calculoErro();
  time = millis();
  if (erro == 0) { 
    I = 0;
  }

  P = erro;
  I += erro;
  if (I > 255){
    I = 255;
  }
  else if (I < 255){
    I = -255;
  }
  D = 1000*(erro - erro_anterior) / dt ? dt : 1;
  PID = (Kp*P) + (Ki*I) + (Kd*D);
  erro_anterior = erro;
  last_time = time;
  Serial.println(D);
  Serial.print("erro: ");
  Serial.println(erro);
}

void Motores(){
  //leituraSensores();
  //calculoErro();
  pid();
  if (PID >=0 ){
    Speed_esq = constrain(vel_esq, 0, 255);
    Speed_dir = constrain (vel_dir - PID, 0, 255);
  }
  else {
    Speed_esq = constrain(vel_esq + PID, 0 , 255);
    Speed_dir = constrain(vel_dir, 0, 255);
  }
  analogWrite(motorEsq1, Speed_esq);
  analogWrite(motorEsq2, 0);
  analogWrite(motorDir1, Speed_dir);
  analogWrite(motorDir2, 0);
}

void loop() {
  if (BT.available()) {
    String input = BT.readStringUntil('\n'); // Lê toda a entrada até encontrar uma nova linha
    seguidorStop();

    if (input.indexOf("run") != -1) {
      running = true;  
      BT.println("Seguidor em movimento.");
    }

    if (input.indexOf("stop") != -1){
      running = false;
      seguidorStop();
      BT.println("Seguidor parou.");
    }

    if (input.indexOf("kp.change(") != -1 && input.indexOf(")") != -1) {
      int startIndex = input.indexOf("(") + 1;
      int endIndex = input.indexOf(")");
      String valueStr = input.substring(startIndex, endIndex);
      float newValue = valueStr.toFloat();
      Kp = newValue;
      BT.println("Kp alterado para: " + String(Kp));
    }

    if (input.indexOf("ki.change(") != -1 && input.indexOf(")") != -1) {
      int startIndex = input.indexOf("(") + 1;
      int endIndex = input.indexOf(")");
      String valueStr = input.substring(startIndex, endIndex);
      float newValue = valueStr.toFloat();
      Ki = newValue;
      BT.println("Ki alterado para: " + String(Ki));
    }

    if (input.indexOf("kd.change(") != -1 && input.indexOf(")") != -1) {
      int startIndex = input.indexOf("(") + 1;
      int endIndex = input.indexOf(")");
      String valueStr = input.substring(startIndex, endIndex);
      float newValue = valueStr.toFloat();
      Kd = newValue;
      BT.println("Kd alterado para: " + String(Kd));
    }

    if (input.indexOf("v.change(") != -1 && input.indexOf(")") != -1) {
      int startIndex = input.indexOf("(") + 1;
      int endIndex = input.indexOf(")");
      String valueStr = input.substring(startIndex, endIndex);
      float newValue = valueStr.toFloat();
      vel_esq = newValue;
      BT.println("velocidade alterada para: " + String(vel_esq));
    }

    if (input.indexOf("pid") != -1) {
      BT.println("Valores atuais:");
      BT.println("Kp: " + String(Kp));
      BT.println("Ki: " + String(Ki));
      BT.println("Kd: " + String(Kd));
    }

    if (input.indexOf("rpm") != -1) {
      printing = true;
      while (printing){
        BT.print("RPM E: ");
        BT.print(pega_velocidade());
        BT.print(" / RPM D: ");
        BT.println(pega_velocidade2());
        String p = BT.readStringUntil('\n');
        if (p.indexOf("b") != -1){
          printing = false;
        }
      }
    }

    if (input.indexOf("v") != -1) {
      BT.println("Valor atual:");
      BT.println("Motores: " + String(vel_esq));
    }

    if (input.indexOf("mpu") != -1) {
      mpu = true;   
      while(mpu){
        dadosMPU();
        if (BT.available()) {
          String input = BT.readStringUntil('\n');
          if (input.indexOf("mpu.stop") != -1) {
            mpu = false;
            BT.println("Dados do MPU pausados.");
          }
        }
      }
    }
    if (input.indexOf("line") != -1){
      for (int i = 0; i < 5; i++) {
        Leitura[i] = digitalRead(Sensores[i]);
        BT.print(Leitura[i] );
        BT.print(" ");
      }
      BT.println();
    }
  }

  if (running) {
    Motores();
  }
}

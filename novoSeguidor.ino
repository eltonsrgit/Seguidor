// Inclusão das bibliotecas
#include <MPU6050_tockn.h> // Giroscópio e Acelerômetro
#include <Wire.h> // Necessária para o Giroscópio e Acelerômetro
#include <SoftwareSerial.h> // Para o módulo Bluetooth

// Declaração de objetos
MPU6050 mpu6050(Wire);
SoftwareSerial BT(53, 52);

// Definição dos pinos dos motores
#define motorDir1 4
#define motorDir2 5
#define motorEsq1 6
#define motorEsq2 7

// Definição dos pinos dos sensores
#define S1 12
#define S2 11
#define S3 10
#define S4 9
#define S5 8

// Variáveis para a identificação digital do sinal dos sensores
bool linha = 1; // se sinal é 1 ou true, superfície branca
bool pista = 0; // se sinal é 0 ou false, superfície preta

bool running = false;

// Variáveis para sensores e PID
int Sensores[] = {S1, S2, S3, S4, S5}; // Vetor para sensores
bool Leitura[5] = {0}; // Vetor para guardar o sinal recebido pelos sensores

float Kp = 80.0; // Ganho proporcional
float Ki = 0.0;  // Ganho integral
float Kd = 50.0; // Ganho derivativo

int vel_base = 150; // Velocidade base
float erro_linha = 0, erro_linha_anterior = 0;
float P = 0, I = 0, D = 0, PID = 0;

// Tempo para o PID
unsigned long last_time = 0;

// Função para parar os motores
void seguidorStop() {
  digitalWrite(motorEsq1, LOW);
  digitalWrite(motorEsq2, LOW);
  digitalWrite(motorDir1, LOW);
  digitalWrite(motorDir2, LOW);
}

#include "Bluetooth.h"

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  BT.println("Espere alguns segundos...");
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  for (int i = 0; i < 5; i++) {
    pinMode(Sensores[i], INPUT);
  }

  pinMode(motorEsq1, OUTPUT);
  pinMode(motorEsq2, OUTPUT);
  pinMode(motorDir1, OUTPUT);
  pinMode(motorDir2, OUTPUT);

  BT.println("### MENU ###");
  BT.println("v para mostrar velocidade base atual.");
  BT.println("v.change(valor 0 a 255) para mudar velocidade base atual.");
  BT.println("pid.value = valores atuais P.I.D.");
  BT.println("'Kp, Ki ou Kd' seguido de .change(valor) altera valores.");
  BT.println("M1 = stop.");
  BT.println("M2 = run.");

  BT.println("SEGUIDOR PRONTO!");
}

// Função para ler os sensores
void leituraSensores() {
  for (int i = 0; i < 5; i++) {
    Leitura[i] = digitalRead(Sensores[i]);
  }
}

// Função para calcular o erro
void calculoErroSensor() {
  leituraSensores();

  float peso[] = {-2, -1, 0, 1, 2}; // Pesos associados a cada sensor
  float soma_pesos = 0;
  int ativos = 0;

  for (int i = 0; i < 5; i++) {
    if (Leitura[i]) {
      soma_pesos += peso[i];
      ativos++;
    }
  }

  if (ativos > 0) {
    erro_linha = soma_pesos / ativos; // Média ponderada
  } else {
    // Caso nenhum sensor esteja ativo, manter o último erro ou buscar a linha
    erro_linha = (erro_linha > 0) ? 2 : -2;
  }
}

// Controlador PID
void pid() {
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;

  calculoErroSensor();

  P = erro_linha;
  I += erro_linha * dt;

  if (I > 255) I = 255;       // Limitação da integral
  else if (I < -255) I = -255;

  D = (erro_linha - erro_linha_anterior) / dt;

  PID = (Kp * P) + (Ki * I) + (Kd * D);

  erro_linha_anterior = erro_linha;
  last_time = current_time;

  Serial.print("Erro: ");
  Serial.println(erro_linha);
  Serial.print("PID: ");
  Serial.println(PID);
}

// Função para controlar os motores
void Motores() {
  pid();

  int velocidade_esq = vel_base + PID;
  int velocidade_dir = vel_base - PID;

  // Constranger as velocidades para não ultrapassar os limites
  velocidade_esq = constrain(velocidade_esq, 0, 255);
  velocidade_dir = constrain(velocidade_dir, 0, 255);

  analogWrite(motorEsq1, velocidade_esq);
  analogWrite(motorEsq2, 0);
  analogWrite(motorDir1, velocidade_dir);
  analogWrite(motorDir2, 0);
}


void loop() {
  if (BT.available()) {
    String input = BT.readStringUntil('\n');
    ComandoBluetooth(input);
  }

  if (running) {
    Motores();
  }
}

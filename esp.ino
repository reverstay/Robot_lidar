#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define MOTOR_A1 25
#define MOTOR_A2 26
#define MOTOR_B1 32
#define MOTOR_B2 33
#define ENCODER_A 15
#define ENCODER_B 2

const int frequencia = 5000;
const int canalMotorA1 = 0;
const int canalMotorA2 = 1;
const int canalMotorB1 = 2;
const int canalMotorB2 = 3;
const int resolucao = 8;

volatile long contadorEncoderA = 0;
volatile long contadorEncoderB = 0;

const long tempoDebounce = 5; // tempo de debounce em ms
volatile long ultimoTempoInterrupcaoA = 0;
volatile long ultimoTempoInterrupcaoB = 0;

const int pulsosPorRevolucao = 20;

float Kp = 12;
float Ki = 1.5;
float Kd = 0.16;
float ultimoErroA = 0;
float ultimoErroB = 0;
float integralA = 0;
float integralB = 0;

float IRAM_ATTR computePID(float setpoint, float valorMedido, float &ultimoErro, float &integral) {
    float erro = setpoint - valorMedido;
    integral += erro;
    float derivativo = erro - ultimoErro;

    float saida = Kp * erro + Ki * integral + Kd * derivativo;
    ultimoErro = erro;

    return saida;
}

void IRAM_ATTR interrupcaoEncoderA() {
  long tempoInterrupcao = millis();
  if (tempoInterrupcao - ultimoTempoInterrupcaoA > tempoDebounce) {
    contadorEncoderA++;
    ultimoTempoInterrupcaoA = tempoInterrupcao;
  }
}

void IRAM_ATTR interrupcaoEncoderB() {
  long tempoInterrupcao = millis();
  if (tempoInterrupcao - ultimoTempoInterrupcaoB > tempoDebounce) {
    contadorEncoderB++;
    ultimoTempoInterrupcaoB = tempoInterrupcao;
  }
}

void tarefaEncoder(void * parameter) {
  long ultimoContadorEncoderA = 0;
  long ultimoContadorEncoderB = 0;

  while (1) {
    long contadorAtualEncoderA = lerEncoderA();
    long contadorAtualEncoderB = lerEncoderB();

    float rpmA = ((contadorAtualEncoderA - ultimoContadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float rpmB = ((contadorAtualEncoderB - ultimoContadorEncoderB) / float(pulsosPorRevolucao)) * 60;


    Serial.print("RPM A: "); Serial.print(rpmA); Serial.print(" ");
    Serial.print("RPM B: "); Serial.println(rpmB);

    ultimoContadorEncoderA = contadorAtualEncoderA;
    ultimoContadorEncoderB = contadorAtualEncoderB;

    delay(500);
  }
}

void setup() {
  // Configura os pinos como saída PWM
  ledcSetup(canalMotorA1, frequencia, resolucao);
  ledcSetup(canalMotorA2, frequencia, resolucao);
  ledcSetup(canalMotorB1, frequencia, resolucao);
  ledcSetup(canalMotorB2, frequencia, resolucao);

  ledcAttachPin(MOTOR_A1, canalMotorA1);
  ledcAttachPin(MOTOR_A2, canalMotorA2);
  ledcAttachPin(MOTOR_B1, canalMotorB1);
  ledcAttachPin(MOTOR_B2, canalMotorB2);

  // Configuração dos encoders
  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A), interrupcaoEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), interrupcaoEncoderB, CHANGE);

  // Inicia a comunicação Serial
  Serial.begin(115200);
  Serial.println("Aguardando comando...");

  xTaskCreatePinnedToCore(tarefaEncoder, "tarefaEncoder", 10000, NULL, 1, NULL, 1);
}

void loop() {
  float RPM = 110.00;
  if (Serial.available()) {
    char comando = Serial.read();
    switch (comando) {
      case 'W':
        pararMotores();
        andarParaFrente(RPM, RPM);
        break;
      case 'S':
        pararMotores();
        andarParaTras(RPM, RPM);
        break;
      case 'A':
        pararMotores();
        virarEsquerda(RPM);
        break;
      case 'D':
        pararMotores();
        virarDireita(RPM);
        break;
      case 'M':
        RPM = RPM + 100.00;
        andarParaFrente(RPM, RPM);
        break;
      case 'N':
        RPM = RPM - 100.00;
        andarParaFrente(RPM, RPM);
        break;
      default:
        pararMotores();
        break;
    }
    Serial.println("Aguardando comando...");
  }
}

void andarParaFrente(float targetRpmA, float targetRpmB) {
    float rpmA = ((lerEncoderA() - contadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float rpmB = ((lerEncoderB() - contadorEncoderB) / float(pulsosPorRevolucao)) * 60;

    float PID_A = computePID(targetRpmA, rpmA, ultimoErroA, integralA);
    float PID_B = computePID(targetRpmB, rpmB, ultimoErroB, integralB);

    float potenciaMotorA = constrain(PID_A, 0, 255);
    float potenciaMotorB = constrain(PID_B, 0, 255);

    ledcWrite(canalMotorA1, potenciaMotorA);
    ledcWrite(canalMotorA2, 0);
    ledcWrite(canalMotorB1, potenciaMotorB);
    ledcWrite(canalMotorB2, 0);
}

void andarParaTras(float targetRpmA, float targetRpmB) {
    float RpmA = ((lerEncoderA() - contadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float RpmB = ((lerEncoderB() - contadorEncoderB) / float(pulsosPorRevolucao)) * 60;

    float PID_A = computePID(targetRpmA, RpmA, ultimoErroA, integralA);
    float PID_B = computePID(targetRpmB, RpmB, ultimoErroB, integralB);

    // Certifique-se de que a potência esteja no intervalo permitido, por exemplo, 0-255
    float potenciaMotorA = constrain(PID_A, 0, 255);
    float potenciaMotorB = constrain(PID_B, 0, 255);

    ledcWrite(canalMotorA1, 0);
    ledcWrite(canalMotorA2, potenciaMotorA);
    ledcWrite(canalMotorB1, 0);
    ledcWrite(canalMotorB2, potenciaMotorB);
}

void virarEsquerda(float targetRpm) {
    float currentRpmA = ((lerEncoderA() - contadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float currentRpmB = ((lerEncoderB() - contadorEncoderB) / float(pulsosPorRevolucao)) * 60;

    float potenciaMotorA = computePID(targetRpm, currentRpmA, ultimoErroA, integralA);
    float potenciaMotorB = computePID(targetRpm, currentRpmB, ultimoErroB, integralB);  // Invertemos a direção do motor B

    potenciaMotorA = constrain(potenciaMotorA, 0, 255);
    potenciaMotorB = constrain(potenciaMotorB, 0, 255);

    ledcWrite(canalMotorA1, 0);
    ledcWrite(canalMotorA2, potenciaMotorA);
    ledcWrite(canalMotorB1, potenciaMotorB);
    ledcWrite(canalMotorB2, 0);
}

void virarDireita(float targetRpm) {
    float currentRpmA = ((lerEncoderA() - contadorEncoderA) / float(pulsosPorRevolucao)) * 60;
    float currentRpmB = ((lerEncoderB() - contadorEncoderB) / float(pulsosPorRevolucao)) * 60;

    float potenciaMotorA = computePID(targetRpm, currentRpmA, ultimoErroA, integralA);
    float potenciaMotorB = computePID(targetRpm, currentRpmB, ultimoErroB, integralB);

    potenciaMotorA = constrain(potenciaMotorA, 0, 255);
    potenciaMotorB = constrain(potenciaMotorB, 0, 255);

    ledcWrite(canalMotorA1, potenciaMotorA);
    ledcWrite(canalMotorA2, 0);
    ledcWrite(canalMotorB1, 0);
    ledcWrite(canalMotorB2, potenciaMotorB);
}

void pararMotores() {
  ledcWrite(canalMotorA1, 0);
  ledcWrite(canalMotorA2, 0);
  ledcWrite(canalMotorB1, 0);
  ledcWrite(canalMotorB2, 0);
}

long lerEncoderA() {
  noInterrupts();
  long contagem = contadorEncoderA;
  interrupts();
  return contagem;
}

long lerEncoderB() {
  noInterrupts();
  long contagem = contadorEncoderB;
  interrupts();
  return contagem;
}

void zerarEncoderA() {
  noInterrupts();
  contadorEncoderA = 0;
  interrupts();
}

void zerarEncoderB() {
  noInterrupts();
  contadorEncoderB = 0;
  interrupts();
}

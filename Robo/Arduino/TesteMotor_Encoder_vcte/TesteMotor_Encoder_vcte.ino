#include <AFMotor.h>

AF_DCMotor motorD(1);   // Motor direito na porta M1
AF_DCMotor motorE(2);   // Motor esquerdo na porta M2

// Pinos dos encoders
static uint8_t ps2D_A = 20;   // Encoder da roda direita - canal A
static uint8_t ps2D_B = 21;   // Encoder da roda direita - canal B
static uint8_t ps2E_A = 18;   // Encoder da roda esquerda - canal A
static uint8_t ps2E_B = 19;   // Encoder da roda esquerda - canal B

volatile unsigned long pulsosD_A = 0;
volatile unsigned long pulsosD_B = 0;
volatile unsigned long pulsosE_A = 0;
volatile unsigned long pulsosE_B = 0;

unsigned long tempoAnterior = 0;
const unsigned long intervaloTeste = 3000; // 3 segundos de teste por etapa
const uint8_t velocidade = 100;            // velocidade constante do motor

void setup() {
  Serial.begin(9600);

  pinMode(ps2D_A, INPUT_PULLUP);
  pinMode(ps2D_B, INPUT_PULLUP);
  pinMode(ps2E_A, INPUT_PULLUP);
  pinMode(ps2E_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ps2D_A), contaPulsoD_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ps2D_B), contaPulsoD_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ps2E_A), contaPulsoE_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ps2E_B), contaPulsoE_B, CHANGE);

  motorD.setSpeed(0);
  motorE.setSpeed(0);
  motorD.run(RELEASE);
  motorE.run(RELEASE);

  Serial.println("Iniciando teste de motores + encoders...");
  delay(1000);

  // Liga os motores UMA vez para frente, com velocidade constante
  motorD.run(FORWARD);
  motorE.run(FORWARD);
  motorD.setSpeed(velocidade);
  motorE.setSpeed(velocidade);
}

void contaPulsoD_A() {
  pulsosD_A++;
}

void contaPulsoD_B() {
  pulsosD_B++;
}

void contaPulsoE_A() {
  pulsosE_A++;
}

void contaPulsoE_B() {
  pulsosE_B++;
}

void loop() {
  Serial.println("\n--- Motores girando para FRENTE (velocidade constante 150) ---");

  pulsosD_A = 0; pulsosD_B = 0;
  pulsosE_A = 0; pulsosE_B = 0;

  tempoAnterior = millis();
  while (millis() - tempoAnterior < intervaloTeste) {
    Serial.print("Direita -> A (pino 20): ");
    Serial.print(pulsosD_A);
    Serial.print("   |   B (pino 21): ");
    Serial.print(pulsosD_B);

    Serial.print("   ||   Esquerda -> A (pino 18): ");
    Serial.print(pulsosE_A);
    Serial.print("   |   B (pino 19): ");
    Serial.println(pulsosE_B);

    // Garante que a velocidade permaneça constante mesmo se algo a alterar
    motorD.setSpeed(velocidade);
    motorE.setSpeed(velocidade);

    delay(200);
  }
}

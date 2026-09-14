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

void setup() {
  Serial.begin(9600);

  pinMode(ps2D, INPUT_PULLUP);
  pinMode(ps2E, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ps2D), contaPulsoD, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ps2E), contaPulsoE, CHANGE);

  motorD.setSpeed(0);
  motorE.setSpeed(0);
  motorD.run(RELEASE);
  motorE.run(RELEASE);

  Serial.println("Iniciando teste de motores + encoders...");
  delay(1000);
}

void contaPulsoD() {
  pulsosD++;
}

void contaPulsoE() {
  pulsosE++;
}

void loop() {
  // ---------- Teste 1: girar os dois motores para frente ----------
  Serial.println("\n--- Teste: Motores para FRENTE (velocidade 150) ---");
  pulsosD = 0; pulsosE = 0;
  motorD.run(FORWARD);
  motorE.run(FORWARD);
  motorD.setSpeed(150);
  motorE.setSpeed(150);

  tempoAnterior = millis();
  while (millis() - tempoAnterior < intervaloTeste) {
    Serial.print("Pulsos D (pino 20): ");
    Serial.print(pulsosD);
    Serial.print("   |   Pulsos E (pino 21): ");
    Serial.println(pulsosE);
    delay(200);
  }
}

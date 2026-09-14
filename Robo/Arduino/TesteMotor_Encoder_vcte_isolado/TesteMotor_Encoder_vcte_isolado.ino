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
const unsigned long intervaloTeste = 10000; // 3 segundos de teste por etapa
const uint8_t velocidade = 155;            // velocidade constante do motor

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

  Serial.println("Iniciando teste sequencial de motores + encoders...");
  delay(1000);
}

void contaPulsoD_A() { pulsosD_A++; }
void contaPulsoD_B() { pulsosD_B++; }
void contaPulsoE_A() { pulsosE_A++; }
void contaPulsoE_B() { pulsosE_B++; }

void testaMotor(const char* nome, AF_DCMotor &motor,
                 volatile unsigned long &pulsosA, volatile unsigned long &pulsosB) {
  Serial.print("\n=== Testando motor: ");
  Serial.print(nome);
  Serial.println(" ===");

  noInterrupts();
  pulsosA = 0;
  pulsosB = 0;
  interrupts();

  motor.run(FORWARD);
  motor.setSpeed(velocidade);

  tempoAnterior = millis();
  while (millis() - tempoAnterior < intervaloTeste) {
    motor.setSpeed(velocidade); // garante velocidade constante

    noInterrupts();
    unsigned long a = pulsosA, b = pulsosB;
    interrupts();

    Serial.print(nome);
    Serial.print(" -> A: ");
    Serial.print(a);
    Serial.print("   |   B: ");
    Serial.println(b);

    delay(200);
  }

  motor.setSpeed(0);
  motor.run(RELEASE);

  noInterrupts();
  unsigned long totalA = pulsosA, totalB = pulsosB;
  interrupts();

  Serial.print(">>> Total ");
  Serial.print(nome);
  Serial.print(" -> A: ");
  Serial.print(totalA);
  Serial.print("   |   B: ");
  Serial.println(totalB);

  //delay(1000); // pausa antes do próximo motor
}

void loop() {
  // Gira o motor DIREITO primeiro
  //testaMotor("DIREITA", motorD, pulsosD_A, pulsosD_B);

  // Depois gira o motor ESQUERDO
  testaMotor("ESQUERDA", motorE, pulsosE_A, pulsosE_B);

  //Serial.println("\n--- Ciclo completo. Aguardando 3s para repetir... ---");
  //delay(3000);
}

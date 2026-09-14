/*
  ============================================================================
  Teste de Encoder em Modo Quadratura (x4) - DDMR com TT-Encoder-Motor
  ============================================================================
  Baseado no datasheet TT-Encoder-Motor (nulllaborg/tt-encoder-motor):
    - 12 PPR no eixo do motor (disco Hall)
    - Redução (gear ratio) 1:90
    - Sinais OUT_A (fase A) e OUT_B (fase B) para posição/direção

  Diferença em relação ao exemplo "single-frequency" (RISING em A apenas):
  Aqui usamos CHANGE em ambos os canais (A e B) de cada roda, com uma
  tabela de transição de estados de quadratura (decodificação x4).
  Isso quadruplica a resolução efetiva (12 * 90 * 4 = 4320 contagens/volta
  na saída da caixa de redução) e é imune a perda de pulso em borda única.

  Requer placa com 4 pinos de interrupção externa disponíveis
  (ex.: Arduino Mega 2560 -> pinos 2, 3, 18, 19, 20, 21).

  Ligação sugerida (conforme "Interface Definition" do datasheet):
    Encoder Direito : OUT_A -> pino 20 | OUT_B -> pino 21
    Encoder Esquerdo: OUT_A -> pino 18 | OUT_B -> pino 19
    5V do encoder    -> 5V do Arduino (aceita 3-12V, use 5V por simplicidade)
    GND do encoder   -> GND comum com o Arduino e o driver de motor
  ============================================================================
*/

#include <AFMotor.h>

/* ---------------- MOTORES (SHIELD AFMotor) ---------------- */
AF_DCMotor motorE(1);   // Motor esquerdo na porta M1
AF_DCMotor motorD(2);   // Motor direito na porta M2

/* ---------------- PINOS DOS ENCODERS ---------------- */
static const uint8_t kPinD_A = 20;   // Encoder da roda direita - canal A
static const uint8_t kPinD_B = 21;   // Encoder da roda direita - canal B
static const uint8_t kPinE_A = 18;   // Encoder da roda esquerda - canal A
static const uint8_t kPinE_B = 19;   // Encoder da roda esquerda - canal B

/* ---------------- CONTADORES (ISR) ---------------- */
volatile int32_t countD = 0;   // contagem bruta (x4) roda direita
volatile int32_t countE = 0;   // contagem bruta (x4) roda esquerda

volatile uint8_t stateD = 0;   // estado atual (2 bits: A<<1 | B) - direita
volatile uint8_t stateE = 0;   // estado atual (2 bits: A<<1 | B) - esquerda

/*
  Tabela de transição de quadratura.
  Índice = (estado_anterior << 2) | estado_novo, cada estado = (A<<1 | B)
  Valor  = +1 (frente), -1 (ré), 0 (transição inválida/ruído)
*/
static const int8_t kQuadTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

/* ---------------- CONSTANTES DO ENCODER (DATASHEET) ---------------- */
const double PPR        = 12.0;   // pulsos por rotação do eixo do encoder
const double GEAR_RATIO = 90.0;   // redução 1:90
const double QUAD_MULT  = 4.0;    // decodificação em quadratura x4

// Contagens por revolução NA SAÍDA da caixa de redução (com x4)
const double COUNTS_PER_REV = PPR * GEAR_RATIO * QUAD_MULT;  // 12*90*4 = 4320

// Conversão de contagem -> radianos (na saída da caixa de redução)
const double RAD_PER_COUNT = 2.0 * PI / COUNTS_PER_REV;

/* ---------------- TEMPO DE AMOSTRAGEM ---------------- */
const unsigned long INTERVALO_TESTE = 70000UL; // 70 ms, em microssegundos
unsigned long time1 = 0;
unsigned long time2 = 0;

/* ============================================================
   ISRs - Roda Direita
   ============================================================ */
void IsrD_A() {
  uint8_t a = digitalRead(kPinD_A);
  uint8_t b = digitalRead(kPinD_B);
  uint8_t newState = (a << 1) | b;
  countD += kQuadTable[(stateD << 2) | newState];
  stateD = newState;
}

void IsrD_B() {
  uint8_t a = digitalRead(kPinD_A);
  uint8_t b = digitalRead(kPinD_B);
  uint8_t newState = (a << 1) | b;
  countD += kQuadTable[(stateD << 2) | newState];
  stateD = newState;
}

/* ============================================================
   ISRs - Roda Esquerda
   ============================================================ */
void IsrE_A() {
  uint8_t a = digitalRead(kPinE_A);
  uint8_t b = digitalRead(kPinE_B);
  uint8_t newState = (a << 1) | b;
  countE += kQuadTable[(stateE << 2) | newState];
  stateE = newState;
}

void IsrE_B() {
  uint8_t a = digitalRead(kPinE_A);
  uint8_t b = digitalRead(kPinE_B);
  uint8_t newState = (a << 1) | b;
  countE += kQuadTable[(stateE << 2) | newState];
  stateE = newState;
}

/* ============================================================
   SETUP
   ============================================================ */
void setup() {
  Serial.begin(9600);

  pinMode(kPinD_A, INPUT_PULLUP);
  pinMode(kPinD_B, INPUT_PULLUP);
  pinMode(kPinE_A, INPUT_PULLUP);
  pinMode(kPinE_B, INPUT_PULLUP);

  // Inicializa o estado atual de cada encoder antes de habilitar as ISRs
  stateD = (digitalRead(kPinD_A) << 1) | digitalRead(kPinD_B);
  stateE = (digitalRead(kPinE_A) << 1) | digitalRead(kPinE_B);

  attachInterrupt(digitalPinToInterrupt(kPinD_A), IsrD_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinD_B), IsrD_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinE_A), IsrE_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinE_B), IsrE_B, CHANGE);

  // Teste simples de acionamento dos motores (ajuste velocidade/sentido)
  motorE.setSpeed(150);
  motorD.setSpeed(150);
  motorE.run(FORWARD);
  motorD.run(FORWARD);

  Serial.println("time_us, pulsosD, velD_rad_s, pulsosE, velE_rad_s");

  time1 = micros();
}

/* ============================================================
   LOOP
   ============================================================ */
void loop() {
  time2 = micros() - time1;

  if (time2 >= INTERVALO_TESTE) {
    // Copia e zera os contadores de forma segura (sem interrupções)
    noInterrupts();
    int32_t pulsosD = countD;
    int32_t pulsosE = countE;
    countD = 0;
    countE = 0;
    interrupts();

    double dt = time2 / 1000000.0;  // segundos

    // Velocidade angular na saída da caixa de redução (rad/s)
    double velD = pulsosD * RAD_PER_COUNT / dt;
    double velE = pulsosE * RAD_PER_COUNT / dt;

    Serial.print(time2);
    Serial.print(", ");
    Serial.print(pulsosD);
    Serial.print(", ");
    Serial.print(velD, 6);
    Serial.print(", ");
    Serial.print(pulsosE);
    Serial.print(", ");
    Serial.println(velE, 6);

    time1 = micros();
  }
}

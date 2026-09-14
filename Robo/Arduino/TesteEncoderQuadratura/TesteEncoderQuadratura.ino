/*
  ============================================================
  TESTE DE ENCODER EM QUADRATURA x4
  Arduino Mega 2560

  Encoder Direito:
    A -> pino 20
    B -> pino 21

  Encoder Esquerdo:
    A -> pino 19
    B -> pino 18

  Encoder:
    12 PPR no eixo do motor
    Redução 1:90

  Quadratura x4:
    12 * 90 * 4 = 4320 contagens por volta da roda
  ============================================================
*/

// ============================================================
// PINOS DOS ENCODERS
// ============================================================

const uint8_t ps2D_A = 20;
const uint8_t ps2D_B = 21;

const uint8_t ps2E_A = 19;
const uint8_t ps2E_B = 18;


// ============================================================
// MÁSCARAS DOS PINOS NO PORTD
// Arduino Mega 2560
//
// Pino 18 -> PD3
// Pino 19 -> PD2
// Pino 20 -> PD1
// Pino 21 -> PD0
// ============================================================

const uint8_t MASK_D_A = (1 << PD1);  // pino 20
const uint8_t MASK_D_B = (1 << PD0);  // pino 21

const uint8_t MASK_E_B = (1 << PD3);  // pino 18
const uint8_t MASK_E_A = (1 << PD2);  // pino 19


// ============================================================
// CONTADORES
// ============================================================

volatile int32_t countD = 0;
volatile int32_t countE = 0;

volatile uint8_t stateD = 0;
volatile uint8_t stateE = 0;


// ============================================================
// TABELA DE DECODIFICAÇÃO EM QUADRATURA
//
// estado = (A << 1) | B
//
//              Novo estado
//             00  01  10  11
//
// anterior 00  0  +1  -1   0
//          01 -1   0   0  +1
//          10 +1   0   0  -1
//          11  0  -1  +1   0
// ============================================================

const int8_t quadTable[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};


// ============================================================
// ATUALIZA ENCODER DIREITO
// ============================================================

void updateD() {

  uint8_t port = PIND;

  uint8_t newState =
      ((port & MASK_D_A) ? 0b10 : 0) |
      ((port & MASK_D_B) ? 0b01 : 0);

  uint8_t index = (stateD << 2) | newState;

  countD += quadTable[index];

  stateD = newState;
}


// ============================================================
// ATUALIZA ENCODER ESQUERDO
// ============================================================

void updateE() {

  uint8_t port = PIND;

  uint8_t newState =
      ((port & MASK_E_A) ? 0b10 : 0) |
      ((port & MASK_E_B) ? 0b01 : 0);

  uint8_t index = (stateE << 2) | newState;

  countE += quadTable[index];

  stateE = newState;
}


// ============================================================
// INTERRUPÇÕES
// ============================================================

void ISR_D_A() {
  updateD();
}

void ISR_D_B() {
  updateD();
}

void ISR_E_A() {
  updateE();
}

void ISR_E_B() {
  updateE();
}


// ============================================================
// CONSTANTES
// ============================================================

const float PPR = 12.0;
const float GEAR_RATIO = 90.0;
const float QUAD_MULT = 4.0;

const float COUNTS_PER_REV =
    PPR * GEAR_RATIO * QUAD_MULT;

// 4320 contagens = 1 volta
const float RAD_PER_COUNT =
    2.0 * PI / COUNTS_PER_REV;


constexpr int8_t kSignD = +1;   // roda direita
constexpr int8_t kSignE = -1;   // roda esquerda (motor espelhado)
// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);

  // Entradas dos encoders
  pinMode(ps2D_A, INPUT_PULLUP);
  pinMode(ps2D_B, INPUT_PULLUP);

  pinMode(ps2E_A, INPUT_PULLUP);
  pinMode(ps2E_B, INPUT_PULLUP);


  // ----------------------------------------------------------
  // Inicializa os estados atuais dos encoders
  // ----------------------------------------------------------

  uint8_t port = PIND;

  stateD =
      ((port & MASK_D_A) ? 0b10 : 0) |
      ((port & MASK_D_B) ? 0b01 : 0);

  stateE =
      ((port & MASK_E_A) ? 0b10 : 0) |
      ((port & MASK_E_B) ? 0b01 : 0);


  // ----------------------------------------------------------
  // Interrupções em ambos os canais
  // ----------------------------------------------------------

  attachInterrupt(
      digitalPinToInterrupt(ps2D_A),
      ISR_D_A,
      CHANGE
  );

  attachInterrupt(
      digitalPinToInterrupt(ps2D_B),
      ISR_D_B,
      CHANGE
  );

  attachInterrupt(
      digitalPinToInterrupt(ps2E_A),
      ISR_E_A,
      CHANGE
  );

  attachInterrupt(
      digitalPinToInterrupt(ps2E_B),
      ISR_E_B,
      CHANGE
  );


  Serial.println();
  Serial.println("==========================================");
  Serial.println(" TESTE DE ENCODER - QUADRATURA x4");
  Serial.println("==========================================");

  Serial.print("Contagens por volta: ");
  Serial.println(COUNTS_PER_REV);

  Serial.println();
  Serial.println("Gire as rodas manualmente.");
  Serial.println("Formato:");
  Serial.println("countD | countE | voltasD | voltasE");
  Serial.println("------------------------------------------");
}


// ============================================================
// LOOP
// ============================================================

void loop() {

  // Copia os contadores de forma segura
  noInterrupts();

  int32_t pulsosD = countD * kSignD;
  int32_t pulsosE = countE * kSignE;

  interrupts();


  // Converte contagens para voltas
  float voltasD =
      (float)pulsosD / COUNTS_PER_REV;

  float voltasE =
      (float)pulsosE / COUNTS_PER_REV;


  // ----------------------------------------------------------
  // Exibe os resultados
  // ----------------------------------------------------------

  Serial.print(pulsosD);
  Serial.print(" | ");

  Serial.print(pulsosE);
  Serial.print(" | ");

  Serial.print(voltasD, 4);
  Serial.print(" | ");

  Serial.println(voltasE, 4);


  delay(200);
}

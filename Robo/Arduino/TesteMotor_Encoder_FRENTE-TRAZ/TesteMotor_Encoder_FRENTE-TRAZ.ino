#include <AFMotor.h>

AF_DCMotor motorE(1);   // Motor esquerdo na porta M1
AF_DCMotor motorD(2);   // Motor direito na porta M2


// ============================================================
// PINOS DOS ENCODERS
// ============================================================

const uint8_t ps2D_A = 20;   // Encoder direito - canal A
const uint8_t ps2D_B = 21;   // Encoder direito - canal B

const uint8_t ps2E_A = 19;   // Encoder esquerdo - canal A
const uint8_t ps2E_B = 18;   // Encoder esquerdo - canal B


// ============================================================
// MÁSCARAS DO PORTD
//
// Mega 2560:
//
// Pino 18 -> PD3
// Pino 19 -> PD2
// Pino 20 -> PD1
// Pino 21 -> PD0
// ============================================================

const uint8_t MASK_D_A = (1 << PD1);  // pino 20
const uint8_t MASK_D_B = (1 << PD0);  // pino 21

const uint8_t MASK_E_A = (1 << PD2);  // pino 19
const uint8_t MASK_E_B = (1 << PD3);  // pino 18


// ============================================================
// CONTADORES
// ============================================================

volatile long countD = 0;
volatile long countE = 0;


// Estado anterior dos encoders
volatile uint8_t stateD = 0;
volatile uint8_t stateE = 0;


// ============================================================
// TABELA DE DECODIFICAÇÃO EM QUADRATURA
//
// Índice = (estado_anterior << 2) | novo_estado
//
//             NOVO ESTADO
//             00   01   10   11
//
// ANTERIOR
//    00       0   -1   +1    0
//    01      +1    0    0   -1
//    10      -1    0    0   +1
//    11       0   +1   -1    0
//
// O sinal depende da sequência dos sinais A e B.
// ============================================================

const int8_t quadTable[16] = {
     0, -1, +1,  0,
    +1,  0,  0, -1,
    -1,  0,  0, +1,
     0, +1, -1,  0
};


// ============================================================
// CONFIGURAÇÃO DO SENTIDO
//
// Se algum encoder estiver contando ao contrário,
// basta trocar +1 por -1.
//
// ============================================================

const int8_t kEncoderSignD = 1;
const int8_t kEncoderSignE = -1;


// ============================================================
// PARÂMETROS DO TESTE
// ============================================================

unsigned long tempoAnterior = 0;

const unsigned long intervaloTeste = 3000;
const unsigned long intervaloPausa = 500;

const uint8_t velocidade = 150;


// ============================================================
// LEITURA DO ESTADO DO ENCODER DIREITO
// ============================================================

void updateD() {

  // Lê a porta D inteira
  uint8_t port = PIND;

  // Extrai A e B e transforma em:
  //
  // A B
  // 0 0 -> 00
  // 0 1 -> 01
  // 1 0 -> 10
  // 1 1 -> 11

  uint8_t newState =
      ((port & MASK_D_A) ? 0b10 : 0) |
      ((port & MASK_D_B) ? 0b01 : 0);


  // Combina estado anterior e novo estado
  uint8_t index =
      (stateD << 2) | newState;


  // Atualiza contador usando a tabela
  countD += quadTable[index];


  // Novo estado passa a ser o anterior
  stateD = newState;
}


// ============================================================
// LEITURA DO ESTADO DO ENCODER ESQUERDO
// ============================================================

void updateE() {

  uint8_t port = PIND;

  uint8_t newState =
      ((port & MASK_E_A) ? 0b10 : 0) |
      ((port & MASK_E_B) ? 0b01 : 0);


  uint8_t index =
      (stateE << 2) | newState;


  countE += quadTable[index];


  stateE = newState;
}


// ============================================================
// INTERRUPÇÕES
// ============================================================

// Encoder direito
void ISR_D_A() {
  updateD();
}

void ISR_D_B() {
  updateD();
}


// Encoder esquerdo
void ISR_E_A() {
  updateE();
}

void ISR_E_B() {
  updateE();
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  Serial.begin(9600);


  // ----------------------------------------------------------
  // Entradas dos encoders
  // ----------------------------------------------------------

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
  // Interrupções nos quatro canais
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


  // ----------------------------------------------------------
  // Inicialização dos motores
  // ----------------------------------------------------------

  motorD.setSpeed(0);
  motorE.setSpeed(0);

  motorD.run(RELEASE);
  motorE.run(RELEASE);


  Serial.println();
  Serial.println("==============================================");
  Serial.println(" TESTE DE MOTORES + ENCODERS EM QUADRATURA");
  Serial.println("==============================================");

  Serial.println("Direita:  A = 20 | B = 21");
  Serial.println("Esquerda: A = 19 | B = 18");

  Serial.println();
  Serial.println("Contagem positiva/negativa indica o sentido.");
  Serial.println();

  delay(1000);
}


// ============================================================
// EXECUTA UMA ETAPA DE TESTE
// ============================================================

void executaEtapa(
  uint8_t sentido,
  const char* nomeSentido
) {

  Serial.print("\n--- Motores girando para ");
  Serial.print(nomeSentido);
  Serial.print(" (");
  Serial.print(sentido);
  Serial.println(" ---");


  // ----------------------------------------------------------
  // Zera os contadores
  // ----------------------------------------------------------

  noInterrupts();


  countD = 0;
  countE = 0;

  interrupts();


  // ----------------------------------------------------------
  // Liga os motores
  // ----------------------------------------------------------

 
 configurarMotor(velocidade * (sentido == 2 ? -1: 1), velocidade * (sentido == 2 ? -1: 1));


  tempoAnterior = millis();


  // ----------------------------------------------------------
  // Executa durante intervaloTeste
  // ----------------------------------------------------------

  while (millis() - tempoAnterior < intervaloTeste) {

    long pulsosD;
    long pulsosE;


    // Leitura segura dos contadores
    noInterrupts();

    pulsosD = countD * kEncoderSignD;
    pulsosE = countE * kEncoderSignE;

    interrupts();


    // --------------------------------------------------------
    // Determina o sentido através do sinal do contador
    // --------------------------------------------------------

    const char* sentidoD;

    if (pulsosD > 0) {
      sentidoD = "POSITIVO";
    }
    else if (pulsosD < 0) {
      sentidoD = "NEGATIVO";
    }
    else {
      sentidoD = "PARADO";
    }


    const char* sentidoE;

    if (pulsosE > 0) {
      sentidoE = "POSITIVO";
    }
    else if (pulsosE < 0) {
      sentidoE = "NEGATIVO";
    }
    else {
      sentidoE = "PARADO";
    }


    // --------------------------------------------------------
    // Exibe resultados
    // --------------------------------------------------------

    Serial.print("Direita:  ");
    Serial.print(pulsosD);
    Serial.print(" -> ");
    Serial.print(sentidoD);


    Serial.print("   ||   Esquerda: ");
    Serial.print(pulsosE);
    Serial.print(" -> ");
    Serial.println(sentidoE);


    // Mantém velocidade
    motorD.setSpeed(velocidade);
    motorE.setSpeed(velocidade);


    delay(200);
  }


  // ----------------------------------------------------------
  // Para os motores antes de inverter
  // ----------------------------------------------------------

  motorD.run(RELEASE);
  motorE.run(RELEASE);

  delay(intervaloPausa);
}


// ============================================================
// LOOP
// ============================================================

// Função para configurar o motor e a direção
void configurarMotor(int EsfControleD, int EsfControleE) {
    Serial.print(EsfControleD);
    Serial.print(" -> ");
    Serial.print(EsfControleE);
    motorD.run(EsfControleD >= 0 ? FORWARD : BACKWARD);
    motorE.run(EsfControleE >= 0 ? FORWARD : BACKWARD);
    motorD.setSpeed(abs(EsfControleD)); 
    motorE.setSpeed(abs(EsfControleE));
}


void loop() {

  // Teste para frente
  executaEtapa(FORWARD, "FRENTE");


  // Teste para trás
  executaEtapa(BACKWARD, "TRÁS");
}

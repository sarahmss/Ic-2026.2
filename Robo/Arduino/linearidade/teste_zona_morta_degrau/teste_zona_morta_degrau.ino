/*
  ============================================================================
  Teste de Resposta ao Degrau (PWM 0 -> 255) para Identificacao da Zona Morta
  ============================================================================
  Baseado no sketch de encoder em quadratura (x4) do DDMR com TT-Encoder-Motor.

  O QUE ESTE SKETCH FAZ:
    Para cada motor (Esquerdo e Direito), separadamente:
      1. Para o motor por completo e aguarda estabilizar (encoder zerado).
      2. Aplica um degrau de PWM (0, PWM_STEP, 2*PWM_STEP, ..., 255).
      3. Durante STEP_DURATION_MS, amostra a velocidade angular a cada
         SAMPLE_INTERVAL_MS (usando o delta de contagem do encoder).
      4. Envia cada amostra pela Serial em formato CSV.
      5. Repete para o proximo nivel de PWM.
    O outro motor fica desligado durante o teste do motor atual, para isolar
    o efeito de cada driver/motor individualmente.

  COMO GERAR O ARQUIVO .CSV:
    Este sketch NAO grava em SD (nao ha modulo SD no hardware original).
    Ele imprime linhas CSV pela porta Serial (9600 baud). Para salvar como
    arquivo .csv, use uma das opcoes abaixo:

      a) Arduino IDE: Ferramentas > Monitor Serial, deixe rodar ate o fim
         ("FIM DO TESTE"), depois copie todo o conteudo e cole em um
         arquivo .csv (ou .txt renomeado para .csv).

      b) Linha de comando (Linux/Mac), captura direta para arquivo:
           cat /dev/ttyACM0 > dados_zona_morta.csv
         (ajuste a porta serial conforme seu sistema)

      c) arduino-cli:
           arduino-cli monitor -p /dev/ttyACM0 -c baudrate=9600 \
             | tee dados_zona_morta.csv

      d) PuTTY (Windows) em modo "Logging to file", formato "All session
         output", salvando como dados_zona_morta.csv

  COMO ANALISAR A ZONA MORTA:
    Apos importar o CSV (Excel, Python/pandas, etc.), para cada motor:
      - Calcule a velocidade media em regime permanente (ultimas amostras
        de cada nivel de PWM, quando o transitorio ja passou).
      - Plote velocidade_media x PWM.
      - A zona morta e a faixa de PWM (a partir de 0) em que a velocidade
        media permanece proxima de zero (dentro do ruido do encoder);
        o PWM onde a velocidade comeca a crescer de forma consistente e o
        limite pratico da zona morta.

  Ajuste os parametros abaixo conforme a necessidade do experimento.
  ============================================================================
*/

#include <AFMotor.h>

/* ---------------- MOTORES (SHIELD AFMotor) ---------------- */
AF_DCMotor motorD(1);   // Motor direito na porta M1
AF_DCMotor motorE(2);   // Motor esquerdo na porta M2

/* ---------------- PINOS DOS ENCODERS ---------------- */
static const uint8_t kPinD_A = 20;   // Encoder da roda direita - canal A
static const uint8_t kPinD_B = 21;   // Encoder da roda direita - canal B
static const uint8_t kPinE_A = 18;   // Encoder da roda esquerda - canal A
static const uint8_t kPinE_B = 19;   // Encoder da roda esquerda - canal B

/*
  Pinos 18/19/20/21 no Mega 2560 estao todos na PORTD, permitindo ler
  os 4 sinais com uma unica leitura de PIND (1 ciclo de clock).
    Pino 18 (Encoder E, canal A) -> PD3
    Pino 19 (Encoder E, canal B) -> PD2
    Pino 20 (Encoder D, canal A) -> PD1
    Pino 21 (Encoder D, canal B) -> PD0
*/
static constexpr uint8_t kMaskD_A = (1 << PD1);  // pino 20
static constexpr uint8_t kMaskD_B = (1 << PD0);  // pino 21
static constexpr uint8_t kMaskE_A = (1 << PD3);  // pino 18
static constexpr uint8_t kMaskE_B = (1 << PD2);  // pino 19

/* ---------------- CONTADORES (ISR) ---------------- */
volatile int32_t countD = 0;   // contagem bruta (x4) roda direita
volatile int32_t countE = 0;   // contagem bruta (x4) roda esquerda

volatile uint8_t stateD = 0;   // estado atual (2 bits: A<<1 | B) - direita
volatile uint8_t stateE = 0;   // estado atual (2 bits: A<<1 | B) - esquerda

static const int8_t kQuadTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

constexpr int8_t kSignD = +1;   // roda direita
constexpr int8_t kSignE = -1;   // roda esquerda (motor espelhado)

/* ---------------- CONSTANTES DO ENCODER (DATASHEET) ---------------- */
const double PPR        = 12.0;   // pulsos por rotacao do eixo do encoder
const double GEAR_RATIO = 90.0;   // reducao 1:90
const double QUAD_MULT  = 4.0;    // decodificacao em quadratura x4
const double COUNTS_PER_REV = PPR * GEAR_RATIO * QUAD_MULT;  // 4320
const double RAD_PER_COUNT  = 2.0 * PI / COUNTS_PER_REV;

/* ---------------- PARAMETROS DO TESTE DE DEGRAU ---------------- */
const int   PWM_MIN            = 0;
const int   PWM_MAX            = 255;
const int   PWM_STEP           = 5;     // incremento do degrau
const unsigned long STEP_DURATION_MS   = 1000;  // duracao de coleta por nivel de PWM
const unsigned long SETTLE_TIME_MS     = 500;  // tempo parado antes do proximo degrau
const unsigned long SAMPLE_INTERVAL_MS = 70;   // periodo de amostragem da velocidade

/* ============================================================
   ISR - Roda Direita
   ============================================================ */
static inline void UpdateD() {
  const uint8_t port = PIND;
  const uint8_t newState =
      ((port & kMaskD_A) ? 0b10 : 0) |
      ((port & kMaskD_B) ? 0b01 : 0);
  countD += kQuadTable[(stateD << 2) | newState];
  stateD = newState;
}
void IsrD_A() { UpdateD(); }
void IsrD_B() { UpdateD(); }

/* ============================================================
   ISR - Roda Esquerda
   ============================================================ */
static inline void UpdateE() {
  const uint8_t port = PIND;
  const uint8_t newState =
      ((port & kMaskE_A) ? 0b10 : 0) |
      ((port & kMaskE_B) ? 0b01 : 0);
  countE += kQuadTable[(stateE << 2) | newState];
  stateE = newState;
}
void IsrE_A() { UpdateE(); }
void IsrE_B() { UpdateE(); }

/* ============================================================
   TESTE DE DEGRAU PARA UM MOTOR
   motor        : motor sob teste
   outro        : motor que deve permanecer parado
   count        : referencia ao contador volatile do encoder do motor sob teste
   sign         : sinal de correcao de sentido (kSignD ou kSignE)
   nomeMotor    : rotulo impresso no CSV ("E" ou "D")
   ============================================================ */
void testeDegrauMotor(AF_DCMotor &motor, AF_DCMotor &outro,
                       volatile int32_t &count, int8_t sign,
                       const char *nomeMotor) {

  // Garante que o outro motor esta parado durante todo o teste
  outro.setSpeed(0);
  outro.run(RELEASE);

  for (int pwm = PWM_MIN; pwm <= PWM_MAX; pwm += PWM_STEP) {

    // 1) Para o motor e aguarda estabilizar
    motor.run(RELEASE);
    motor.setSpeed(0);
    delay(SETTLE_TIME_MS);

    // 2) Zera o contador de forma segura
    noInterrupts();
    count = 0;
    interrupts();

    // 3) Aplica o degrau de PWM
    motor.setSpeed(pwm);
    motor.run(FORWARD);

    unsigned long tStart = millis();
    unsigned long tNext  = tStart;
    int32_t lastCount = 0;

    while (millis() - tStart < STEP_DURATION_MS) {
      if (millis() >= tNext) {
        noInterrupts();
        int32_t countAtual = count;
        interrupts();

        unsigned long tAmostra = millis() - tStart;
        int32_t deltaCount = countAtual - lastCount;
        double deltaT_s = SAMPLE_INTERVAL_MS / 1000.0;
        double vel = (deltaCount * sign) * RAD_PER_COUNT / deltaT_s;

        // CSV: motor,pwm,t_ms,count_acumulado,delta_count,vel_rad_s
        Serial.print(nomeMotor);
        Serial.print(",");
        Serial.print(pwm);
        Serial.print(",");
        Serial.print(tAmostra);
        Serial.print(",");
        Serial.print(countAtual * sign);
        Serial.print(",");
        Serial.print(deltaCount * sign);
        Serial.print(",");
        Serial.print(vel, 6);
        Serial.println(",");

        lastCount = countAtual;
        tNext += SAMPLE_INTERVAL_MS;
      }
    }
  }

  // Para o motor ao final da varredura
  motor.run(RELEASE);
  motor.setSpeed(0);
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

  {
    const uint8_t port = PIND;
    stateD = ((port & kMaskD_A) ? 0b10 : 0) | ((port & kMaskD_B) ? 0b01 : 0);
    stateE = ((port & kMaskE_A) ? 0b10 : 0) | ((port & kMaskE_B) ? 0b01 : 0);
  }

  attachInterrupt(digitalPinToInterrupt(kPinD_A), IsrD_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinD_B), IsrD_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinE_A), IsrE_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinE_B), IsrE_B, CHANGE);

  // Garante os dois motores parados no inicio
  motorE.run(RELEASE);
  motorD.run(RELEASE);
  motorE.setSpeed(0);
  motorD.setSpeed(0);

  delay(2000); // tempo para abrir o monitor serial / iniciar captura

  Serial.println("motor,pwm,t_ms,count_acumulado,delta_count,vel_rad_s");

  // ---- Teste do motor DIREITO ----
  testeDegrauMotor(motorD, motorE, countD, kSignD, "D");

  // ---- Teste do motor ESQUERDO ----
  testeDegrauMotor(motorE, motorD, countE, kSignE, "E");

  Serial.println("FIM DO TESTE");
}

/* ============================================================
   LOOP (vazio - o teste roda uma unica vez no setup)
   ============================================================ */
void loop() {
  // Nada a fazer aqui. Para repetir o teste, resete a placa.
}

#include <AFMotor.h>

// ============================================================
// TESTE EM MALHA ABERTA: Curva Tensão (PWM) x Velocidade (rad/s)
// ============================================================
// Objetivo: aplicar degraus de PWM, aguardar regime permanente,
// medir a velocidade angular via encoder e registrar o par
// (PWM, velocidade) para posterior plotagem da curva S/Z
// (zona morta + saturação + atrito).

//  Encoder Direito: M1
//    A -> pino 20 PD1
//    B -> pino 21 PD0
//  Encoder Esquerdo: M2
//    A -> pino 19 PD2
//   B -> pino 18 PD3
// ============================================================

AF_DCMotor motorD(1);   // Motor a ser testado (ajuste para 2 se for o esquerdo)

//----------- Configuração do Encoder (mesma lógica do código original) ------
static const uint8_t kPinA = 20;   // canal A do encoder do motor sob teste
static const uint8_t kPinB = 21;   // canal B do encoder do motor sob teste

static constexpr uint8_t kMaskA = (1 << PD1);  // pino 20
static constexpr uint8_t kMaskB = (1 << PD0);  // pino 21

volatile int32_t count = 0;
volatile uint8_t state = 0;

static const int8_t kQuadTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

const double kPPR       = 12.0;
const double kGearRatio = 90.0;
const double kQuadMult  = 4.0;
const double kCountsPerRev = kPPR * kGearRatio * kQuadMult;   // 4320 contagens/volta
const double kRadPerCount  = 2.0 * PI / kCountsPerRev;

constexpr int8_t kSign = -1;   // 

//----------- Parâmetros do Ensaio -------------------------------------------
const int   PWM_MIN        = -255;   // varredura simétrica (inclui sentido reverso)
const int   PWM_MAX        =  255;
const int   PWM_STEP       =  10;    // resolução da varredura (quanto menor, mais fino perto da zona morta)
const unsigned long T_ACOMODACAO = 10000;  // ms: tempo para atingir regime permanente
const unsigned long T_MEDICAO    = 12000;  // ms: janela de medição da velocidade média 12s
const unsigned long T_AMOSTRAGEM = 20;    // ms: intervalo entre leituras dentro da janela

void setup() {
  Serial.begin(9600);
  motorD.setSpeed(0);
  motorD.run(RELEASE);

  pinMode(kPinA, INPUT_PULLUP);
  pinMode(kPinB, INPUT_PULLUP);

  {
    const uint8_t port = PIND;
    state = ((port & kMaskA) ? 0b10 : 0) | ((port & kMaskB) ? 0b01 : 0);
  }

  attachInterrupt(digitalPinToInterrupt(kPinA), UpdateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinB), UpdateEncoder, CHANGE);

  Serial.println("PWM,Velocidade_rad_s");

  delay(2000);  // tempo para abrir o monitor serial / preparar o log
}

void loop() {
  // Varredura decrescente: PWM_MIN <- PWM_MAX
  for (int pwm = PWM_MAX; pwm >= PWM_MIN; pwm -= PWM_STEP) {
    double vel = medirVelocidadeRegimePermanente(pwm);
    Serial.print(pwm);
    Serial.print(",");
    Serial.println(vel, 6);
  }

  // Para o motor ao final e trava o programa (evita repetir o ensaio sozinho)
  motorD.run(RELEASE);
  motorD.setSpeed(0);
  Serial.println("FIM_ENSAIO");
  while (true) { delay(1000); }
}

// ------------------------------------------------------------
// Aplica o PWM, espera acomodação, mede a velocidade média
// em uma janela de tempo (regime permanente)
// ------------------------------------------------------------
double medirVelocidadeRegimePermanente(int pwm) {
  // Aplica o comando ao motor
  motorD.run(pwm >= 0 ? FORWARD : BACKWARD);
  motorD.setSpeed(abs(pwm));

  // Zera contador e aguarda o transitório passar
  noInterrupts();
  count = 0;
  interrupts();
  delay(T_ACOMODACAO);

  // Mede a velocidade média durante a janela de regime permanente
  // (várias sub-amostras para suavizar ruído de medição)
  int   nAmostras   = T_MEDICAO / T_AMOSTRAGEM;
  double somaVel    = 0;
  unsigned long tAnterior = micros();

  noInterrupts();
  count = 0;
  interrupts();

  for (int i = 0; i < nAmostras; i++) {
    delay(T_AMOSTRAGEM);

    noInterrupts();
    int32_t pulsos = count * kSign;
    count = 0;
    interrupts();

    unsigned long tAtual = micros();
    double dt = (tAtual - tAnterior) / 1000000.0;
    tAnterior = tAtual;

    double velInst = pulsos * kRadPerCount / dt;
    somaVel += velInst;
  }

  return somaVel / nAmostras;  // velocidade média em rad/s (regime permanente)
}

// ------------------------------------------------------------
// ISR de decodificação em quadratura (idêntica à lógica original)
// ------------------------------------------------------------
void UpdateEncoder() {
  const uint8_t port = PIND;
  const uint8_t newState =
      ((port & kMaskA) ? 0b10 : 0) |
      ((port & kMaskB) ? 0b01 : 0);
  count += kQuadTable[(state << 2) | newState];
  state = newState;
}

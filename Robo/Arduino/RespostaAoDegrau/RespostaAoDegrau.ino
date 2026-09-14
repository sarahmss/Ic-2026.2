#include <AFMotor.h>

AF_DCMotor motorD(2);

//  Encoder Direito:
//    A -> pino 20
//    B -> pino 21
//  Encoder Esquerdo:
//    A -> pino 19 PD2
//   B -> pino 18 PD3

//----------- Encoder (mesma lógica de sempre) -----------------------------
static const uint8_t kPinA = 19;
static const uint8_t kPinB = 18;

static constexpr uint8_t kMaskA = (1 << PD2); // 19
static constexpr uint8_t kMaskB = (1 << PD3); // 18

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
const double kCountsPerRev = kPPR * kGearRatio * kQuadMult;
const double kRadPerCount  = 2.0 * PI / kCountsPerRev;
constexpr int8_t kSign = +1;

//----------- Parâmetros do ensaio de degrau ---------------------------------
const int PWM_DEGRAU = 200;              // <-- altere para testar diferentes amplitudes
const unsigned long DT_AMOSTRA_US = 20000;  // 20 ms entre amostras (mais fino que o Ts=70ms do controle)
const unsigned long T_TOTAL_MS = 20000;      // duração total do registro (ajuste se precisar de mais tempo)

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

  Serial.println("t_ms,velocidade_rad_s");
  delay(2000);  // tempo para abrir o monitor/gravador serial

  // Garante motor parado e encoder zerado antes do degrau
  motorD.run(RELEASE);
  motorD.setSpeed(0);
  delay(1000);
  noInterrupts();
  count = 0;
  interrupts();

  // -------- Aplica o degrau --------
  unsigned long t0 = millis();
  unsigned long tAmostraAnteriorUs = micros();

  motorD.run(PWM_DEGRAU >= 0 ? FORWARD : BACKWARD);
  motorD.setSpeed(abs(PWM_DEGRAU));

  while (millis() - t0 < T_TOTAL_MS) {
    unsigned long agoraUs = micros();
    if (agoraUs - tAmostraAnteriorUs >= DT_AMOSTRA_US) {
      double dt = (agoraUs - tAmostraAnteriorUs) / 1000000.0;
      tAmostraAnteriorUs = agoraUs;

      noInterrupts();
      int32_t pulsos = count * kSign;
      count = 0;
      interrupts();

      double vel = pulsos * kRadPerCount / dt;

      Serial.print(millis() - t0);
      Serial.print(",");
      Serial.println(vel, 6);
    }
  }

  motorD.run(RELEASE);
  motorD.setSpeed(0);
  Serial.println("FIM_DEGRAU");
}

void loop() {
  // nada — ensaio de degrau é feito uma vez no setup()
}

void UpdateEncoder() {
  const uint8_t port = PIND;
  const uint8_t newState =
      ((port & kMaskA) ? 0b10 : 0) |
      ((port & kMaskB) ? 0b01 : 0);
  count += kQuadTable[(state << 2) | newState];
  state = newState;
}

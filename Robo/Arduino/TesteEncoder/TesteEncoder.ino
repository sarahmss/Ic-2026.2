// Pinos dos encoders
static uint8_t ps2D_A = 20;   // Encoder da roda direita - canal A
static uint8_t ps2D_B = 21;   // Encoder da roda direita - canal B
static uint8_t ps2E_A = 18;   // Encoder da roda esquerda - canal A
static uint8_t ps2E_B = 19;   // Encoder da roda esquerda - canal B

volatile unsigned long pulsosD_A = 0;
volatile unsigned long pulsosD_B = 0;
volatile unsigned long pulsosE_A = 0;
volatile unsigned long pulsosE_B = 0;

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
  Serial.print("Roda Direita -> A (pino 20): ");
  Serial.print(pulsosD_A);
  Serial.print("   |   B (pino 21): ");
  Serial.println(pulsosD_B);

  Serial.print("Roda Esquerda -> A (pino 18): ");
  Serial.print(pulsosE_A);
  Serial.print("   |   B (pino 19): ");
  Serial.println(pulsosE_B);

  Serial.println("-----------------------------------------");

  delay(200);
}

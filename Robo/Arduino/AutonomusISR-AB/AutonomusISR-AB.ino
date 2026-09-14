
#include <PID_v1.h>
#include <AFMotor.h>
#include <math.h>

AF_DCMotor motorD(1);             //Seleciona o motor direito na porta 1
AF_DCMotor motorE(2);             //Seleciona o motor esquerdo na porta 2

//----------- Configuração Motores --------------------------
/*
  ANTES: só o canal A de cada encoder era lido (ps2D=20, ps2E=21), e a
  direção do giro NÃO era medida -- era assumida igual ao sinal do
  esforço de controle do PID (dirD/dirE). Isso é frágil: se a roda
  travar, patinar ou não seguir exatamente o comando, a "velocidade"
  medida mentia, porque o sinal vinha do comando e não do encoder.

  AGORA: os dois canais (A e B) de cada encoder são lidos, permitindo
  decodificação em quadratura x4 (ver metodologia completa nos
  comentários abaixo). A direção passa a ser medida de verdade.

  No Arduino Mega 2560, os pinos 18, 19, 20 e 21 caem todos na mesma
  porta física (PORTD/PIND) -- por isso escolhemos os canais B nesses
  pinos: permite ler os 4 sinais com 1 única leitura de registrador
  por interrupção, em vez de 4 chamadas a digitalRead().

    Pino 18 (Encoder Esquerdo, canal B) -> PD3
    Pino 19 (Encoder Esquerdo, canal A) -> PD2
    Pino 20 (Encoder Direito,  canal A) -> PD1  (era ps2D)
    Pino 21 (Encoder Direito,  canal B) -> PD0  (era ps2E, agora vira B)
*/
static const uint8_t kPinD_A = 20;   // Encoder direito - canal A (pino original ps2D)
static const uint8_t kPinD_B = 21;   // Encoder direito - canal B (NOVO)
static const uint8_t kPinE_A = 19;   // Encoder esquerdo - canal A (NOVO)
static const uint8_t kPinE_B = 18;   // Encoder esquerdo - canal B (NOVO)

static constexpr uint8_t kMaskD_A = (1 << PD1);  // pino 20
static constexpr uint8_t kMaskD_B = (1 << PD0);  // pino 21
static constexpr uint8_t kMaskE_A = (1 << PD2);  // pino 19
static constexpr uint8_t kMaskE_B = (1 << PD3);  // pino 18

// Contadores agora são SIGNED: o sinal já sai correto da própria
// decodificação de quadratura (não depende mais do comando do PID).
volatile int32_t countD = 0, countE = 0;
volatile uint8_t stateD = 0, stateE = 0;   // estado atual (2 bits: A<<1 | B) de cada encoder

/*
  Tabela de transição de quadratura (decodificação x4).
  Índice = (estado_anterior << 2) | estado_novo, cada estado = (A<<1 | B)
  Valor  = +1 (avanço), -1 (recuo), 0 (transição inválida/ruído)
*/
static const int8_t kQuadTable[16] = {
   0, -1,  1,  0,
   1,  0,  0, -1,
  -1,  0,  0,  1,
   0,  1, -1,  0
};

/* ---------------- CONSTANTES DO ENCODER (DATASHEET) ---------------- */
const double kPPR       = 12.0;   // pulsos por rotação do eixo do encoder
const double kGearRatio = 90.0;   // redução 1:90
const double kQuadMult  = 4.0;    // decodificação em quadratura x4
const double kCountsPerRev = kPPR * kGearRatio * kQuadMult;   // 4320 contagens/volta
const double kRadPerCount  = 2.0 * PI / kCountsPerRev;

/*
  Correção de sinal por lado (montagem espelhada dos motogeradores).
  Calibre girando cada roda manualmente no sentido de avanço do robô
  e conferindo se o contador daquele lado fica positivo; se não,
  inverta o sinal aqui.
*/
constexpr int8_t kSignD = +1;   // roda direita
constexpr int8_t kSignE = -1;   // roda esquerda

double velD = 0, velE = 0;                  // Velocidades das rodas em rad/s

//----------- Odometria --------------------------------------
static double pi = 3.141592;
static double r=0.032;                       //Raio em metros
static double l=0.124;                       //Comprimento em metros
double x0=0, y0 = 0, theta0 = 0;      //Posição inicial
double x, y, theta;                   //Posição atual
//----------- PID --------------------------------------------

static double Kpe = 9.0, Kie = 60, Kde = 0;   // Ganhos da roda esquerda
//double Kpd = 5, Kid = 55, Kdd = 0;   // Ganhos da roda direita
static double Kpd = 9.0, Kid = 60, Kdd = 0; // Ganhos da roda direita
double SetpointD = 0, InputD = 0, EsfControleD=0;  //  Variaveis relacionadas ao PID direito
double  SetpointE = 0, InputE = 0, EsfControleE=0; //  Variaveis relacionadas ao PID esquerdo

PID myPIDe(&InputE, &EsfControleE, &SetpointE, Kpe, Kie, Kde, DIRECT); // Declaraçao do PID esquerdo
PID myPIDd(&InputD, &EsfControleD, &SetpointD, Kpd, Kid, Kdd, DIRECT); // Declaraçao do PID direito


//----------- Fuzzy --------------------------------------------
static uint8_t n_points = 101;  //Numero de pontos das funções de pertinência
double phi = 0;      //Erro angular
double phi_norm = 0; //Modulo do erro angular
// PARAMETROS DOS CONTROLADORES
double ErroAgParametros[5] ={0.0126,0.2452,0.2405,0.4736,1.3946}; //15it RNN2 - best

double angular_error[11];     //Vetor para armazenar os valores fuzzy para erro_angular
double distancia[2];          //Vetor para armazenar os valores fuzzy para erro_escalar
double fuzzy_linear_speed[5];
double fuzzy_angular_speed[3];
double fuzzy_agg_linear_speed[101];
double fuzzy_agg_angular_speed[101];
double x_linear_speed[101];
double x_angular_speed[101];
double ErroL = 100;   //Norma do erro entre posição e objetivo

struct FuzzyOutputs {
    double ReFzD=0;
    double ReFzE=0;
};
FuzzyOutputs FZout;

//-------------Trajetórias Salvas -------------------------------------

//Reta: v1
//double objVect[2][2] = {{0.5,0},{1,0}};
//uint8_t tamVect = 2;

//Zigzag Reto: v1 v2
//double objVect[17][2] = {{0.25,0},{0.5,0},{0.75,0},{1,0},{0.75,0.175},{0.5,0.35},{0.25,0.525},{0,0.7},{0.25,0.7},{0.5,0.7},{0.75,0.7},{1,0.7},{0.75,0.525},{0.5,0.35},{0.25,0.175},{0,0},{0.25,0}};
//uint8_t tamVect = 17;

//ZZX 1: v1 v2
//double objVect[21][2] = {{0.0000,0.7000},{1.0000,0.7000},{0.8889,0.5829},{0.7778,0.4780},{0.6667,0.3842},{0.5556,0.3002},{0.4445,0.2251},{0.3333,0.1579},{0.2222,0.0978},{0.1111,0.0439},{0.0000,-0.0042},{1.0000,-0.0042},{0.8889,0.0439},{0.7778,0.0978},{0.6667,0.1579},{0.5556,0.2251},{0.4445,0.3002},{0.3333,0.3842},{0.2222,0.4780},{0.1111,0.5829},{0.0000,0.7000}};
//uint8_t tamVect = 21;

//ZZY 1 v2
//double objVect[21][2] = {{0.7000, 0.0000},{0.7000, 1.0000},{0.5829, 0.8889},{0.4780, 0.7778},{0.3842, 0.6667},{0.3002, 0.5556},{0.2251, 0.4445},{0.1579, 0.3333},{0.0978, 0.2222},{0.0439, 0.1111},{-0.0042, 0.0000},{-0.0042, 1.0000},{0.0439, 0.8889},{0.0978, 0.7778},{0.1579, 0.6667},{0.2251, 0.5556},{0.3002, 0.4445},{0.3842, 0.3333},{0.4780, 0.2222},{0.5829, 0.1111},{0.7000, 0.0000}};
//uint8_t tamVect = 21;

//ZZY 2: v1 v2
//double objVect[15][2] = {{-0.0000,0.7324},{-0.5020,0.6103},{-0.9199,0.4883},{-1.2679,0.3662},{-1.5578,0.2441},{-1.7991,0.1221},{-2.0000,0.0000},{-2.0000,0.7324},{-1.7991,0.6103},{-1.5578,0.4883},{-1.2679,0.3662},{-0.9199,0.2441},{-0.5020,0.1221},{-0.0000,0.0000},{-0.0000,0.7324}};
//uint8_t tamVect = 15;

//ZZX 1 INV: v1 v2
//double objVect[22][2] = {{0.0000, 0.0000}, {1.0500, 0.0000}, {0.9333, 0.1052}, {0.8167, 0.1993}, {0.7000, 0.2835}, {0.5833, 0.3588}, {0.4667, 0.4262}, {0.3500, 0.4866}, {0.2333, 0.5406}, {0.1167, 0.5889}, {0.0000, 0.6321}, {1.0500, 0.6321}, {0.9333, 0.5889}, {0.8167, 0.5406}, {0.7000, 0.4866}, {0.5833, 0.4262}, {0.4667, 0.3588}, {0.3500, 0.2835}, {0.2333, 0.1993}, {0.1167, 0.1052}, {0.0000, 0.0000}, {0.0000, 0.0000}};
//uint8_t tamVect = 22;

//ZZX 2: v1 v2
//double objVect[14][2] = {{0.7324,-0.0000},{0.6103,-0.5020},{0.4883,-0.9199},{0.3662,-1.2679},{0.2441,-1.5578},{0.1221,-1.7991},{0.0000,-2.0000},{0.7324,-2.0000},{0.6103,-1.7991},{0.4883,-1.5578},{0.3662,-1.2679},{0.2441,-0.9199},{0.1221,-0.5020},{0.0000,-0.0000}};
//uint8_t tamVect = 14;

//ZZX 2 INV: v1 v2
//double objVect[15][2] = {{0.7324,0.0000},{0.6103,0.5020},{0.4883,0.9199},{0.3662,1.2679},{0.2441,1.5578},{0.1221,1.7991},{0.0000,2.0000},{0.7324,2.0000},{0.6103,1.7991},{0.4883,1.5578},{0.3662,1.2679},{0.2441,0.9199},{0.1221,0.5020},{0.0000,0.0000},{0.7324,0.0000}};
//uint8_t tamVect = 15;

//Losango 1: v1  v2
//double objVect[9][2] = {{0.0,0.0},{0.25,0.5},{0.5,1},{0.75,0.5},{1,0},{0.75,-0.5},{0.5,-1},{0.25,-0.5},{0,0}}; //Diagonal
//uint8_t tamVect = 9;

//Losango 2: v1 v2
//double objVect[9][2] = {{0.5,0.25},{1,0.5},{1.5,0.25},{2,0},{1.5,-0.25},{1,-0.5},{0.5,-0.25},{0,0},{0.5,0.25}};
//uint8_t tamVect = 9;
//------------------

//Arco aberto (meio circulo, raio 1) v1 v2
//double objVect[20][2] = {{0.0000,0.0000},{0.0136,0.1646},{0.0542,0.3247},{0.1205,0.4759},{0.2109,0.6142},{0.3227,0.7357},{0.4531,0.8372},{0.5983,0.9158},{0.7545,0.9694},{0.9174,0.9966},{1.0826,0.9966},{1.2455,0.9694},{1.4017,0.9158},{1.5469,0.8372},{1.6773,0.7357},{1.7891,0.6142},{1.8795,0.4759},{1.9458,0.3247},{1.9864,0.1646},{2.0000,0.0000}};
//uint8_t tamVect = 20;

//S-curve (senoide) v1 v2
//double objVect[11][2] = {{0.0000,0.0000},{0.1000,0.0500},{0.2000,0.0900},{0.3000,0.0900},{0.4000,0.0500},{0.5000,0.0000},{0.6000,-0.0500},{0.7000,-0.0900},{0.8000,-0.0900},{0.9000,-0.0500},{1.00.0000}
//uint8_t tamVect = 11;

//Senoide x1 v2
//double objVect[21][2] = {{-0.0042,0.0000},{-0.0042,1.0000},{0.0439,0.8889},{0.0978,0.7778},{0.1579,0.6667},{0.2251,0.5556},{0.3002,0.4445},{0.3842,0.3333},{0.4780,0.2222},{0.5829,0.1111},{0.7000,0.0000},{0.7000,1.0000},{0.5829,0.8889},{0.4780,0.7778},{0.3842,0.6667},{0.3002,0.5556},{0.2251,0.4445},{0.1579,0.3333},{0.0978,0.2222},{0.0439,0.1111},{-0.0042,0.0000}};
//uint8_t tamVect = 21;

//Circulo completo (raio 1m) x1 x2
//double objVect[60][2] = {{0.0000,0.0000},{0.0057,0.1063},{0.0226,0.2114},{0.0506,0.3141},{0.0894,0.4132},{0.1384,0.5077},{0.1973,0.5964},{0.2652,0.6783},{0.3415,0.7526},{0.4252,0.8183},{0.5154,0.8748},{0.6112,0.9213},{0.7113,0.9574},{0.8147,0.9827},{0.9202,0.9968},{1.0266,0.9996},{1.1327,0.9912},{1.2373,0.9714},{1.3392,0.9407},{1.4373,0.8993},{1.5304,0.8477},{1.6175,0.7866},{1.6976,0.7165},{1.7698,0.6382},{1.8333,0.5528},{1.8874,0.4611},{1.9313,0.3642},{1.9648,0.2631},{1.9873,0.1591},{1.9986,0.0532},{1.9986,-0.0532},{1.9873,-0.1591},{1.9648,-0.2631},{1.9313,-0.3642},{1.8874,-0.4611},{1.8333,-0.5528},{1.7698,-0.6382},{1.6976,-0.7165},{1.6175,-0.7866},{1.5304,-0.8477},{1.4373,-0.8993},{1.3392,-0.9407},{1.2373,-0.9714},{1.1327,-0.9912},{1.0266,-0.9996},{0.9202,-0.9968},{0.8147,-0.9827},{0.7113,-0.9574},{0.6112,-0.9213},{0.5154,-0.8748},{0.4252,-0.8183},{0.3415,-0.7526},{0.2652,-0.6783},{0.1973,-0.5964},{0.1384,-0.5077},{0.0894,-0.4132},{0.0506,-0.3141},{0.0226,-0.2114},{0.0057,-0.1063},{0.0000,0.0000}};
//uint8_t tamVect = 60;

//ZZXarredondado (lemniscata - formato de infinito)
//double objVect[60][2] = {{0.5000,0.0000},{0.4468,-0.0528},{0.3943,-0.1033},{0.3429,-0.1491},{0.2933,-0.1881},{0.2461,-0.2187},{0.2017,-0.2394},{0.1607,-0.2492},{0.1236,-0.2478},{0.0907,-0.2352},{0.0625,-0.2119},{0.0392,-0.1791},{0.0211,-0.1382},{0.0085,-0.0910},{0.0014,-0.0398},{0.0000,0.0133},{0.0042,0.0658},{0.0141,0.1153},{0.0295,0.1596},{0.0502,0.1966},{0.0760,0.2248},{0.1066,0.2429},{0.1416,0.2499},{0.1808,0.2457},{0.2235,0.2303},{0.2694,0.2046},{0.3179,0.1696},{0.3684,0.1269},{0.4204,0.0785},{0.4734,0.0266},{0.5266,-0.0266},{0.5796,-0.0785},{0.6316,-0.1269},{0.6821,-0.1696},{0.7306,-0.2046},{0.7765,-0.2303},{0.8192,-0.2457},{0.8584,-0.2499},{0.8934,-0.2429},{0.9240,-0.2248},{0.9498,-0.1966},{0.9705,-0.1596},{0.9859,-0.1153},{0.9958,-0.0658},{1.0000,-0.0133},{0.9986,0.0398},{0.9915,0.0910},{0.9789,0.1382},{0.9608,0.1791},{0.9375,0.2119},{0.9093,0.2352},{0.8764,0.2478},{0.8393,0.2492},{0.7983,0.2394},{0.7539,0.2187},{0.7067,0.1881},{0.6571,0.1491},{0.6057,0.1033},{0.5532,0.0528},{0.5000,0.0000}};
//uint8_t tamVect = 60;

//Fig6 v1
//double objVect[30][2] = {{0.0000,0.0000},{0.0712,0.3036},{0.1420,0.5018},{0.2123,0.6130},{0.2822,0.6547},{0.3519,0.6425},{0.4212,0.5908},{0.4903,0.5125},{0.5592,0.4191},{0.6280,0.3206},{0.6966,0.2257},{0.7652,0.1414},{0.8337,0.0735},{0.9022,0.0263},{0.9707,0.0026},{1.0392,0.0038},{1.1078,0.0299},{1.1763,0.0794},{1.2449,0.1494},{1.3136,0.2355},{1.3822,0.3321},{1.4510,0.4317},{1.5197,0.5258},{1.5884,0.6043},{1.6572,0.6556},{1.7259,0.6668},{1.7946,0.6234},{1.8632,0.5096},{1.9317,0.3080},{2.0000,0.0000}};
//uint8_t tamVect= 30;

//Fig10 v1
//double objVect[30][2] = {{0.0000,0.0000},{0.0159,0.0264},{0.0596,0.0948},{0.1252,0.1890},{0.2067,0.2927},{0.2982,0.3898},{0.3937,0.4640},{0.4873,0.4991},{0.5738,0.4823},{0.6527,0.4215},{0.7252,0.3322},{0.7924,0.2303},{0.8557,0.1313},{0.9161,0.0511},{0.9750,0.0052},{1.0336,0.0085},{1.0929,0.0592},{1.1541,0.1423},{1.2182,0.2421},{1.2863,0.3431},{1.3595,0.4297},{1.4390,0.4864},{1.5256,0.4978},{1.6183,0.4589},{1.7119,0.3834},{1.8009,0.2868},{1.8797,0.1846},{1.9428,0.0924},{1.9848,0.0257},{2.0000,0.0000}};
//uint8_t tamVect= 30;



double objetivo[2];
double erro[2];
//--------------Variaveis de Controle ----------------------------------
bool parar = 0;
unsigned long time1 = 0;
unsigned long time2 = 0;
unsigned long delta_time = 0;

void setup() {
  Serial.begin(9600);
  motorD.setSpeed(0);
  pinMode(kPinD_A, INPUT_PULLUP);
  pinMode(kPinD_B, INPUT_PULLUP);
  pinMode(kPinE_A, INPUT_PULLUP);
  pinMode(kPinE_B, INPUT_PULLUP);
  y0 = objVect[0][1];

  // Inicialização PID
  myPIDd.SetOutputLimits(-255, 255); myPIDe.SetOutputLimits(-255, 255);
  myPIDd.SetSampleTime(100); myPIDe.SetSampleTime(100);
  myPIDd.SetMode(AUTOMATIC); myPIDe.SetMode(AUTOMATIC);

  // Estado inicial de cada encoder (lido do registrador, antes de habilitar as ISRs)
  {
    const uint8_t port = PIND;
    stateD = ((port & kMaskD_A) ? 0b10 : 0) | ((port & kMaskD_B) ? 0b01 : 0);
    stateE = ((port & kMaskE_A) ? 0b10 : 0) | ((port & kMaskE_B) ? 0b01 : 0);
  }

  //Configurar Interrupções (2 por roda: canal A e canal B)
  attachInterrupt(digitalPinToInterrupt(kPinD_A), IsrD_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinD_B), IsrD_B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinE_A), IsrE_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(kPinE_B), IsrE_B, CHANGE);
}
void loop(){
  if (!parar){
    if (Serial.available() > 0) {
      char recebido = Serial.read();
      if (recebido == '3') {   // compara com o caractere '3'
        parar = 1;
        //Serial.println("Arduino pronto para iniciar loop");
      }
    }
    return; // enquanto não autorizado, sai do loop aqui
  }

    if (parar){
      for (int i=0; i<tamVect; i++){
        objetivo[0] = objVect[i][0];
        objetivo[1] = objVect[i][1];
        erro[0] =  objetivo[0] - x;
        erro[1] = objetivo[1] - y;
        ErroL = sqrt(pow(erro[0],2) + pow(erro[1],2));

        while (ErroL > 0.03){
          //--------Execução Controle--------------
          ErroL = CallFuzzyControl(objetivo, x, y);
          InputE = velE; InputD = velD;
          SetpointE = FZout.ReFzE; SetpointD = FZout.ReFzD;
          myPIDd.Compute(); myPIDe.Compute();
          configurarMotor(EsfControleD,EsfControleE);
          time2 = micros() - time1;
          if (time2 < 70000){
            delayMicroseconds(70000 - time2);
          }
          time2 =  micros() - time1;
          noInterrupts();
          int32_t pulsosD = countD * kSignD;   // sinal já vem da quadratura, não do comando
          int32_t pulsosE = countE * kSignE;
          countD = 0;
          countE = 0;
          interrupts();
          double dt = time2 / 1000000.0;       // segundos
          velD = pulsosD * kRadPerCount / dt;
          velE = pulsosE * kRadPerCount / dt;
          //Serial.println(time2);
          Odometria(velD, velE, time2); //Odometria 
          send_data(time2-time1);
          time1 = micros();
        } //while erroL
      } //for TamVect
      parar = false;
      motorD.run(RELEASE);
      motorE.run(RELEASE);
      Serial.write(5);
    } //parar
    //Serial.println(time2);
  } //loop

//Função Envio de dados
void send_data(int tciclo){
  Serial.write(2);
  Serial.print(x);Serial.print(",");
  Serial.print(y);Serial.print(",");
  Serial.print(theta);Serial.print(",");
  Serial.print(velD);Serial.print(",");
  Serial.print(velE);Serial.print(",");
  Serial.print(FZout.ReFzD);Serial.print(",");
  Serial.print(FZout.ReFzE);Serial.print(",");
  Serial.print(EsfControleD);Serial.print(",");
  Serial.print(EsfControleE);//Serial.print(",");
  //Serial.print(tciclo);
  Serial.write(3); 
}
//Funções de interrupções
double CallFuzzyControl(double objetivo[2],double x, double y){
    double erro[2];
    erro[0] =  objetivo[0] - x;
    erro[1] = objetivo[1] - y;
    ErroL = sqrt(pow(erro[0],2) + pow(erro[1],2));
    //double phi2 = atan2(erro[1],erro[0]);
    phi = atan2(erro[1],erro[0]) - theta;
    phi_norm = 2*atan(tan(phi/2));
    FuzzyControl(phi_norm, ErroL);
    return ErroL;
}
/* ============================================================
   ISR - Roda Direita (uma única leitura de PIND para A e B)
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
   ISR - Roda Esquerda (uma única leitura de PIND para A e B)
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
//Função para implementação da odometria
void Odometria(double velD,double velE, unsigned long t){
    x    = (double)x0+r/2*t*cos(theta0)*(velD + velE)/1000000;
    y    = (double)y0+r/2*t*sin(theta0)*(velD + velE)/1000000;
    theta = (double)theta0+r/l*t*(velD - velE)/1000000;

    x0 = x;
    y0 = y;
    theta0 = theta;
}

// Função para configurar o motor e a direção
void configurarMotor(int EsfControleD, int EsfControleE) {
    motorD.run(EsfControleD >= 0 ? FORWARD : BACKWARD);
    motorE.run(EsfControleE >= 0 ? FORWARD : BACKWARD);
    motorD.setSpeed(abs(EsfControleD)); 
    motorE.setSpeed(abs(EsfControleE));
}


// Função para calcular o valor máximo entre dois números
double maxValue(double a, double b) {
    return (a > b) ? a : b;
}

// Função para calcular o valor mínimo entre dois números
double minValue(double a, double b) {
    return (a < b) ? a : b;
}

// Função para calcular o valor máximo entre mais de dois números
double maxOfArray(double values[], int size) {
    double maxVal = values[0];
    for (int i = 1; i < size; i++) {
        maxVal = maxValue(maxVal, values[i]);
    }
    return maxVal;
}

// Função para multiplicação elemento a elemento e soma dos produtos
double sumProduct(double* array1, double* array2, int size) {
    double result = 0.0;
    for (int i = 0; i < size; i++) {
        result += array1[i] * array2[i];
    }
    return result;
}

// Função para soma dos elementos de um array
double sumArray(double* array, int size) {
    double result = 0.0;
    for (int i = 0; i < size; i++) {
        result += array[i];
    }
    return result;
}

// Função Gaussiana
double gauss_mf(double x, double a, double b) {
    double exponent = -pow(x - b, 2) / (2 * pow(a, 2));
    return exp(exponent);
}

// Função Linear Crescente
double lins_mf(double x, double a, double b) {
    if (x <= a) return 0.0;
    if (x >= b) return 1.0;
    return (x - a) / (b - a);
}

// Função Linear Decrescente
double linz_mf(double x, double a, double b) {
    if (x <= a) return 1.0;
    if (x >= b) return 0.0;
    return (b - x) / (b - a);
}

// Função Triangular
double tri_mf(double x, double a, double b, double c) {
    if (x <= a || x >= c) {
        return 0.0;
    } else if (x > a && x <= b) {
        return (x - a) / (b - a);
    } else if (x > b && x < c) {
        return (c - x) / (c - b);
    }
    return 0.0;
}

// Função Pi
double pi_mf(double x, double a, double b, double c, double d) {
    double y = 0.0;
    
    if (x > b && x < c) {
        y = 1.0;
    } else if (x > a && x <= b) {
        y = 2 * pow((x - a) / (b - a), 2);
    } else if (x >= c && x < d) {
        y = 2 * pow((x - d) / (d - c), 2);
    } else if (x > (a + b) / 2 && x <= b) {
        y = 1 - 2 * pow((x - b) / (b - a), 2);
    } else if (x >= c && x < (c + d) / 2) {
        y = 1 - 2 * pow((x - c) / (d - c), 2);
    }
    return y;
}

void pi_mf_vect(const double* x, double* y, int size, double a, double b, double c, double d) {
    for (int i = 0; i < size; ++i) {
        if (x[i] > b && x[i] < c) {
            y[i] = 1.0;
        } else if (x[i] > a && x[i] <= b) {
            y[i] = 2 * pow((x[i] - a) / (b - a), 2);
        } else if (x[i] >= c && x[i] < d) {
            y[i] = 2 * pow((x[i] - d) / (d - c), 2);
        } else if (x[i] > (a + b) / 2 && x[i] <= b) {
            y[i] = 1 - 2 * pow((x[i] - b) / (b - a), 2);
        } else if (x[i] >= c && x[i] < (c + d) / 2) {
            y[i] = 1 - 2 * pow((x[i] - c) / (d - c), 2);
        } else {
            y[i] = 0.0;  // Definindo o valor padrão para y[i] quando nenhuma das condições é atendida
        }
    }
}


// Função de Controle Fuzzy
void FuzzyControl(double phi, double ErroL) {
    // Fuzzyficação

    // Input Angular Error
    angular_error[0] = gauss_mf(phi, ErroAgParametros[0], -pi);
    angular_error[1] = gauss_mf(phi, ErroAgParametros[1], ErroAgParametros[3] - pi);
    angular_error[2] = gauss_mf(phi, ErroAgParametros[2], ErroAgParametros[4] - pi);
    angular_error[3] = gauss_mf(phi, ErroAgParametros[2], -ErroAgParametros[4]);
    angular_error[4] = gauss_mf(phi, ErroAgParametros[1], -ErroAgParametros[3]);
    angular_error[5] = gauss_mf(phi, ErroAgParametros[0], 0);
    angular_error[6] = gauss_mf(phi, ErroAgParametros[1], ErroAgParametros[3]);
    angular_error[7] = gauss_mf(phi, ErroAgParametros[2], ErroAgParametros[4]);
    angular_error[8] = gauss_mf(phi, ErroAgParametros[2], pi - ErroAgParametros[4]);
    angular_error[9] = gauss_mf(phi, ErroAgParametros[1], pi - ErroAgParametros[3]);
    angular_error[10] = gauss_mf(phi, ErroAgParametros[0], pi);

  
    // Input Distância
    distancia[0] = linz_mf(l, 0.01, 0.02);
    distancia[1] = lins_mf(l, 0.01, 0.02);
    

    // Inferência

    fuzzy_linear_speed[0] = minValue(distancia[1], maxValue(angular_error[0], angular_error[10]));
    fuzzy_linear_speed[1] = minValue(distancia[1], maxValue(angular_error[1], angular_error[9]));


    double arr1[] = {angular_error[2], angular_error[3], angular_error[7], angular_error[8], distancia[0]};
    fuzzy_linear_speed[2] = minValue(1, maxOfArray(arr1, 5));
    fuzzy_linear_speed[3] = minValue(distancia[1], maxValue(angular_error[4], angular_error[6]));
    fuzzy_linear_speed[4] = minValue(distancia[1], angular_error[5]);


        // Implementando a fórmula para fuzzy_angular_speed
    double arr2[] = {angular_error[3], angular_error[4], angular_error[8], angular_error[9]};
    fuzzy_angular_speed[0] = minValue(distancia[1], maxOfArray(arr2, 4));

    double arr3[] = {angular_error[0], angular_error[10], angular_error[5], distancia[0]};
    fuzzy_angular_speed[1] = minValue(1, maxOfArray(arr3, 4));

    double arr4[] = {angular_error[1], angular_error[2], angular_error[6], angular_error[7]};
    fuzzy_angular_speed[2] = minValue(distancia[1], maxOfArray(arr4, 4));
    // Simulando os valores de x_linear_speed e x_angular_speed usando linspace
    for (int i = 0; i < n_points; i++) {
        x_linear_speed[i] = -0.15 + i * (0.15 - (-0.15)) / (n_points - 1);
        x_angular_speed[i] = -0.5 + i * (0.5 - (-0.5)) / (n_points - 1);
    }
    
//Agg Linear Speed
    double y1[n_points], y2[n_points], y3[n_points], y4[n_points], y5[n_points];

    pi_mf_vect(x_linear_speed, y1, n_points, -0.16, -0.15, -0.1, -0.05);
    pi_mf_vect(x_linear_speed, y2, n_points, -0.1, -0.05, -0.05, 0);
    pi_mf_vect(x_linear_speed, y3, n_points, -0.05, 0, 0, 0.05);
    pi_mf_vect(x_linear_speed, y4, n_points, 0, 0.05, 0.05, 0.1);
    pi_mf_vect(x_linear_speed, y5, n_points, 0.05, 0.1, 0.15, 0.16);

    double min1[n_points], min2[n_points], min3[n_points], min4[n_points], min5[n_points];

    for (int i = 0; i < n_points; ++i) {
        min1[i] = minValue(fuzzy_linear_speed[0], y1[i]);
        min2[i] = minValue(fuzzy_linear_speed[1], y2[i]);
        min3[i] = minValue(fuzzy_linear_speed[2], y3[i]);
        min4[i] = minValue(fuzzy_linear_speed[3], y4[i]);
        min5[i] = minValue(fuzzy_linear_speed[4], y5[i]);
    }
    for (int i = 0; i < n_points; ++i) {
      double aggl[5] = {min1[i],min2[i],min3[i],min4[i],min5[i]};
        fuzzy_agg_linear_speed[i] = maxOfArray(aggl,5);
    }

//Agg angular speed
    double h1[101],h2[101],h3[101];

    for (int i =0; i<n_points;i++){
      h1[i] =minValue(fuzzy_angular_speed[0],linz_mf(x_angular_speed[i], -0.5, 0));
      h2[i] =minValue(fuzzy_angular_speed[1],tri_mf(x_angular_speed[i], -0.5, 0, 0.5));
      h3[i] =minValue(fuzzy_angular_speed[2],lins_mf(x_angular_speed[i], 0, 0.5));
    }
        for (int i = 0; i < n_points; ++i) {
      double agga[3] = {h1[i],h2[i],h3[i]};
        fuzzy_agg_angular_speed[i] = maxOfArray(agga,3);
    }

    // Desfuzzyficação

    double speed_linear = sumProduct(x_linear_speed, fuzzy_agg_linear_speed, n_points) / sumArray(fuzzy_agg_linear_speed, n_points);
    double speed_angular = sumProduct(x_angular_speed, fuzzy_agg_angular_speed, n_points) / sumArray(fuzzy_agg_angular_speed, n_points);

    // Separando os valores de u
    FZout.ReFzD = (1 / r) * (speed_linear + speed_angular * l/2) ;
    FZout.ReFzE = (1 / r) * (speed_linear - speed_angular * l/2) ;//ganho de 1.2
}

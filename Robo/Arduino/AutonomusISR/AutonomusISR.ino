#include <PID_v1.h>
#include <AFMotor.h>
#include <math.h>

AF_DCMotor motorD(1);             //Seleciona o motor direito na porta 1
AF_DCMotor motorE(2);             //Seleciona o motor esquerdo na porta 2

//----------- Configuração Motores --------------------------
static uint8_t ps2D = 20, ps2E = 18;        //Pinos conectador aos encoders
volatile uint32_t countE = 0, countD = 0;   //Contadores de pulsos
double velD = 0, velE = 0;                  // Velocidades das rodas em rad/s

//----------- Odometria --------------------------------------
static double pi = 3.141592;
static double r=0.032;                       //Raio em metros
static double l=0.124;                       //Comprimento em metros
double x0=0, y0 = 0, theta0 = 0;      //Posição inicial
double x, y, theta;                   //Posição atual
int dirE = 1;                   //Variáveis de direção 
int dirD = 1;
//----------- PID --------------------------------------------

static double Kpe = 9.0, Kie = 60, Kde = 0;   // Ganhos da roda esquerda
//double Kpd = 5, Kid = 55, Kdd = 0;   // Ganhos da roda direita
static double Kpd = 9.0, Kid = 60, Kdd = 0;
double SetpointD = 0, InputD = 0, EsfControleD=0;  //  Variaveis relacionadas ao PID direito
double  SetpointE = 0, InputE = 0, EsfControleE=0; //  Variaveis relacionadas ao PID esquerdo

PID myPIDe(&InputE, &EsfControleE, &SetpointE, Kpe, Kie, Kde, DIRECT); // Declaraçao do PID esquerdo
PID myPIDd(&InputD, &EsfControleD, &SetpointD, Kpd, Kid, Kdd, DIRECT); // Declaraçao do PID direito


//----------- Fuzzy --------------------------------------------
static uint8_t n_points = 101;  //Numero de pontos das funções de pertinência
double phi = 0;      //Erro angular
double phi_norm = 0; //Modulo do erro angular
// PARAMETROS DOS CONTROLADORES
//double ErroAgParametros[5] = {pi/11, pi/11, pi/11, pi/11, 2*pi/11}; // ???
//double ErroAgParametros[5] ={0.097042795889918,0.0217112314006104,0.23500978192182,1.5707963267949,0.284457870618802}; //18 IT
//double ErroAgParametros[5] = {0.1571, 0.1571, 0.1571, 0.6283, 1.2566}; //inicial
//double ErroAgParametros[5] ={0.0927844807795898,0.0707516617233648,0.153225699349153,1.5707963267949,0.220754800808791}; //50IT
//double ErroAgParametros[5] ={0.08856785888658815,0.10983925376762384,0.07708347990115272,0.12611798731780352,1.5495182595845884}; //15it RNN1
//double ErroAgParametros[5] ={0.2658,0.1847,0.1453,0.2410,1.2736}; //20it RNN2
double ErroAgParametros[5] ={0.0126,0.2452,0.2405,0.4736,1.3946}; //15it RNN2 - best
//double ErroAgParametros[5] ={0.3336,0.2747,0.2400,0.2852,1.517}; //10it RNN2

//0.08856785888658815,0.10983925376762384,0.07708347990115272,0.12611798731780352,1.5495182595845884
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

const double kPPR       = 12.0;   // pulsos por rotação do eixo do encoder
const double kGearRatio = 90.0;   // redução 1:90
const double kQuadMult  = 1.0;    // decodificação em quadratura x4
const double kCountsPerRev = kPPR * kGearRatio * kQuadMult;   // 4320 contagens/volta
const double kRadPerCount  = 2.0 * PI / kCountsPerRev;
//-------------Trajetórias Salvas -------------------------------------
//Super ZZX Duplas
// float objVect[44][2] = {
//     {0.0000f,0.7000f},{1.0000f,0.7000f},{0.8889f,0.5829f},{0.7778f,0.4780f},
//     {0.6667f,0.3842f},{0.5556f,0.3002f},{0.4445f,0.2251f},{0.3333f,0.1579f},
//     {0.2222f,0.0978f},{0.1111f,0.0439f},{0.0000f,-0.0042f},{1.0000f,-0.0042f},
//     {0.8889f,0.0439f},{0.7778f,0.0978f},{0.6667f,0.1579f},{0.5556f,0.2251f},
//     {0.4445f,0.3002f},{0.3333f,0.3842f},{0.2222f,0.4780f},{0.1111f,0.5829f},
//     {0.0000f,0.7000f},{0.0000f,0.7000f},{0.0000f,0.7000f},{0.0000f,-0.3000f},
//     {0.1171f,-0.1889f},{0.2220f,-0.0778f},{0.3158f,0.0333f},{0.3998f,0.1444f},
//     {0.4749f,0.2555f},{0.5421f,0.3667f},{0.6022f,0.4778f},{0.6561f,0.5889f},
//     {0.7042f,0.7000f},{0.7042f,-0.3000f},{0.6561f,-0.1889f},{0.6022f,-0.0778f},
//     {0.5421f,0.0333f},{0.4749f,0.1444f},{0.3998f,0.2555f},{0.3158f,0.3667f},
//     {0.2220f,0.4778f},{0.1171f,0.5889f},{0.0000f,0.7000f},{0.0000f,0.7000f}
// };
// uint8_t tamVect = 44;

//float objVect[14][2] = {{0.7324, -0.0000},{0.6103, -0.5020},{0.4883, -0.9199},{0.3662, -1.2679},{0.2441, -1.5578},{0.1221, -1.7991},{0.0000, -2.0000},{0.7324, -2.0000},{0.6103, -1.7991},{0.4883, -1.5578},{0.3662, -1.2679},{0.2441, -0.9199},{0.1221, -0.5020},{0.0000, -0.0000}};
//uint8_t tamVect = 14;

//Zigzag Reto:
//double objVect[16][2]={{0.25,0},{0.5,0},{0.75,0},{1,0},{0.75,0.175},{0.5,0.35},{0.25,0.525},{0,0.7},{0.25,0.7},{0.5,0.7},{0.75,0.7},{1,0.7},{0.75,0.525},{0.5,0.35},{0.25,0.175},{0,0}}; //Diagonal
//uint8_t tamVect = 16;

//ZZX 1:
//double objVect[22][2] = {{0.0000, 0.7000}, {1.0000, 0.7000}, {0.8889, 0.5829}, {0.7778, 0.4780}, {0.6667, 0.3842}, {0.5556, 0.3002}, {0.4445, 0.2251}, {0.3333, 0.1579}, {0.2222, 0.0978}, {0.1111, 0.0439}, {0.0000, -0.0042}, {1.0000, -0.0042}, {0.8889, 0.0439}, {0.7778, 0.0978}, {0.6667, 0.1579}, {0.5556, 0.2251}, {0.4445, 0.3002}, {0.3333, 0.3842}, {0.2222, 0.4780}, {0.1111, 0.5829}, {0.0000, 0.7000}, {0.0000, 0.7000}};
//uint8_t tamVect = 22;

//ZZX INV:
//double objVect[22][2] = {{0.0000, 0.0000}, {1.0500, 0.0000}, {0.9333, 0.1052}, {0.8167, 0.1993}, {0.7000, 0.2835}, {0.5833, 0.3588}, {0.4667, 0.4262}, {0.3500, 0.4866}, {0.2333, 0.5406}, {0.1167, 0.5889}, {0.0000, 0.6321}, {1.0500, 0.6321}, {0.9333, 0.5889}, {0.8167, 0.5406}, {0.7000, 0.4866}, {0.5833, 0.4262}, {0.4667, 0.3588}, {0.3500, 0.2835}, {0.2333, 0.1993}, {0.1167, 0.1052}, {0.0000, 0.0000}, {0.0000, 0.0000}};
//uint8_t tamVect = 22;

//Losango:
//double objVect[9][2]={{0.0,0.0},{0.25,0.5},{0.5,1},{0.75,0.5},{1,0},{0.75,-0.5},{0.5,-1},{0.25,-0.5},{0,0}};//Diagonal
//uint8_t tamVect = 9;

//Infinito:
//double objVect[23][2]={{0.1818,0.5406},{0.3636,0.9096},{0.5455,0.9898},{0.7273,0.7557},{0.9091,0.2817},{1.0991,-0.2817},{1.2727,-0.7557},{1.4545,-0.9898},{1.6364,-0.9096},{1.8182,-0.5406},{2,0},{1.8182,0.5406},{1.6364,0.9096},{1.4545,0.9898},{1.2727,0.7557},{1.0909,0.2817},{0.9091,-0.2817},{0.7273,-0.7557},{0.5455,-0.9898},{0.3636,-0.9096},{0.1818,-0.5406},{0,0}};
//uint8_t tamVect = 23;

//Losango 2:
//double objVect[8][2] = {{0.5,0.25},{1,0.5},{1.5,0.25},{2,0},{1.5,-0.25},{1,-0.5},{0.5,-0.25},{0,0}};
//uint8_t tamVect = 8;

// Reta:
double objVect[2][2] = {{0.5,0},{1,0}};
uint8_t tamVect = 2;

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
  motorE.setSpeed(0);
  pinMode(ps2D, INPUT);
  pinMode(ps2E, INPUT);
  y0 = objVect[0][1];

  // Inicialização PID
  myPIDd.SetOutputLimits(-255, 255); myPIDe.SetOutputLimits(-255, 255);
  myPIDd.SetSampleTime(100); myPIDe.SetSampleTime(100);
  myPIDd.SetMode(AUTOMATIC); myPIDe.SetMode(AUTOMATIC);
  //Configurar Interrupções
attachInterrupt(digitalPinToInterrupt(ps2D), Contador_D, CHANGE);
attachInterrupt(digitalPinToInterrupt(ps2E), Contador_E, CHANGE);
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
          velD = (double)dirD*(countD * kRadPerCount) / (time2);
          velE = (double)dirE*(countE * kRadPerCount) / (time2);
          countD = 0;
          countE = 0;
          interrupts();
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

//Calcular velocidade
void calcular_velocidade(int t){
  velD = (double)dirD*(countD * 65449.847) / (t*33.5);
  velE = (double)dirE*(countE * 65449.847) / (t*33.5);
}
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
void Contador_D(){
  countD ++;
}
void Contador_E(){
  countE ++;
}
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
    dirD = EsfControleD >= 0 ? 1 : -1;

    motorE.run(EsfControleE >= 0 ? FORWARD : BACKWARD);
    dirE = EsfControleE >= 0 ? 1 : -1;
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

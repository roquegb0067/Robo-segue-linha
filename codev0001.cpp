
// ==========================================
// 1. MAPEAMENTO DE PINOS DA PONTE H E SENSORES
// ==========================================
const int IN1 = 6;
const int IN2 = 7;
const int ENA = 5;
const int IN3 = 9;
const int IN4 = 8;
const int ENB = 10;

const int leftS = A3;   // Sensor 0 (Ponta Esquerda)
const int centerL = A4; // Sensor 1 (Centro Esquerda)
const int center = 11;  // Sensor 2 (Centro - Digital) -> *Recomendado trocar por Analógico se possível
const int centerR = A5; // Sensor 3 (Centro Direita)
const int rightL = A2;  // Sensor 4 (Ponta Direita)

// ==========================================
// 2. CONSTANTES E VARIÁVEIS DO PID
// ==========================================
// Se o robô continuar sem curvar, aumente o Kp (ex: 0.1, 0.15) e o Kd (ex: 1.5, 2.0)
float Kp = 0.12;  
float Ki = 0.0001;
float Kd = 1.8;  

int erro = 0;
int erroAnterior = 0;
float P = 0, I = 0, D = 0;
float PID = 0;

const int VELOCIDADE_BASE = 120;
const int VELOCIDADE_MAX = 250;
int ultimaPosicao = 2000; // Guarda o último lado visto se o robô perder a linha

void setup() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
 
  pinMode(leftS, INPUT);
  pinMode(centerL, INPUT);
  pinMode(center, INPUT);
  pinMode(centerR, INPUT);
  pinMode(rightL, INPUT);
}

void loop() {
  int posicao = calcularPosicao();
 
  // O objetivo é manter na posição 2000
  erro = posicao - 2000;
 
  P = erro;
  I = I + erro;
  I = constrain(I, -500, 500); // Anti-windup: evita que o termo Integral cresça infinitamente
  D = erro - erroAnterior;
 
  erroAnterior = erro;
 
  PID = (Kp * P) + (Ki * I) + (Kd * D);
 
  ajustarMotores(PID);
}

// ==========================================
// 3. FUNÇÃO QUE CALCULA A POSIÇÃO NA LINHA
// ==========================================
int calcularPosicao() {
  // ATENÇÃO: Verifique se a linha é PRETA ou BRANCA.
  // Este mapeamento assume que o valor analógico SOBE na linha.
  int minAnalog = 35;
  int maxAnalog = 400;

  int s0 = map(constrain(analogRead(leftS), minAnalog, maxAnalog), minAnalog, maxAnalog, 0, 1000);
  int s1 = map(constrain(analogRead(centerL), minAnalog, maxAnalog), minAnalog, maxAnalog, 0, 1000);
  int s2 = digitalRead(center) ? 1000 : 0;
  int s3 = map(constrain(analogRead(centerR), minAnalog, maxAnalog), minAnalog, maxAnalog, 0, 1000);
  int s4 = map(constrain(analogRead(rightL), minAnalog, maxAnalog), minAnalog, maxAnalog, 0, 1000);

  long somaPonderada = ((long)s0 * 0) + ((long)s1 * 1000) + ((long)s2 * 2000) + ((long)s3 * 3000) + ((long)s4 * 4000);
  long somaLeituras = s0 + s1 + s2 + s3 + s4;

  // Se o robô perder a linha (soma for muito baixa), ele lembra da última posição
  if (somaLeituras < 200) {
    if (ultimaPosicao < 2000) return 0;    // Força curva acentuada para a esquerda
    if (ultimaPosicao > 2000) return 4000; // Força curva acentuada para a direita
    return 2000;
  }

  ultimaPosicao = somaPonderada / somaLeituras;
  return ultimaPosicao;
}

// ==========================================
// 4. FUNÇÃO QUE APLICA O PID NOS MOTORES (CORRIGIDA)
// ==========================================
void ajustarMotores(float valorPID) {
  // SINAL INVERTIDO:
  // Se erro positivo (robô na direita), valorPID é positivo -> Acelera Motor A (Esquerdo) e desacelera B para virar à esquerda.
  int velMotorA = VELOCIDADE_BASE + valorPID; // Motor A (Esquerda)
  int velMotorB = VELOCIDADE_BASE - valorPID; // Motor B (Direita)

  // Garante que os motores não fiquem com valores negativos ou acima do máximo permitido
  velMotorA = constrain(velMotorA, 0, VELOCIDADE_MAX);
  velMotorB = constrain(velMotorB, 0, VELOCIDADE_MAX);

  // Sentido de rotação para frente
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, velMotorA);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, velMotorB);
}

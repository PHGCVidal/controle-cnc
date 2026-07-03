#include <Arduino.h>

#define EN_PIN    D8
#define X_STEP    D5
#define X_DIR     D2
#define Y_STEP    D6
#define Y_DIR     D3

// Intervalo dinâmico entre passos em microssegundos (calculado pelo PID)
unsigned long xIntervaloMicros = 0; 
unsigned long yIntervaloMicros = 0;

unsigned long xLastStepTime = 0;
unsigned long yLastStepTime = 0;
unsigned long ultimoTempoComando = 0;

bool motorAtivo = false;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(X_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);

  digitalWrite(EN_PIN, HIGH); // Motores desligados no início
}

void loop() {
  // 1. LEITURA DOS COMANDOS DE VELOCIDADE DO PYTHON
  // O Python vai mandar: "FrequenciaX,FrequenciaY\n" (ex: "500.0,-250.0\n")
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    
    int virgulaIndex = comando.indexOf(',');
    if (virgulaIndex > 0) {
      float freqX = comando.substring(0, virgulaIndex).toFloat();
      float freqY = comando.substring(virgulaIndex + 1).toFloat();
      
      // Define a direção do motor X
      digitalWrite(X_DIR, freqX >= 0 ? HIGH : LOW);
      // Calcula o período em microssegundos (Período = 1.000.000 / Frequência)
      xIntervaloMicros = (abs(freqX) > 1.0) ? (1000000UL / abs(freqX)) : 0;

      // Define a direção do motor Y
      digitalWrite(Y_DIR, freqY >= 0 ? HIGH : LOW);
      yIntervaloMicros = (abs(freqY) > 1.0) ? (1000000UL / abs(freqY)) : 0;
      
      ultimoTempoComando = millis();
    }
  }

  // 2. WATCHDOG DE SEGURANÇA
  if (millis() - ultimoTempoComando > 1000) {
    xIntervaloMicros = 0;
    yIntervaloMicros = 0;
  }

  // 3. GESTÃO DE POTÊNCIA (ENABLE)
  bool precisaMover = (xIntervaloMicros > 0) || (yIntervaloMicros > 0);
  if (precisaMover && !motorAtivo) {
    digitalWrite(EN_PIN, LOW); // Liga os drivers
    motorAtivo = true;
  } else if (!precisaMover && motorAtivo) {
    digitalWrite(EN_PIN, HIGH); // Desliga os drivers para não esquentar
    motorAtivo = false;
  }

  // 4. GERAÇÃO DE PULSOS TEMPORIZADOS (INTERPOLAÇÃO NATURAL)
  unsigned long tempoAtual = micros();

  // Canal do Motor X
  if (xIntervaloMicros > 0 && (tempoAtual - xLastStepTime >= xIntervaloMicros)) {
    xLastStepTime = tempoAtual;
    digitalWrite(X_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(X_STEP, LOW);
  }

  // Canal do Motor Y
  if (yIntervaloMicros > 0 && (tempoAtual - yLastStepTime >= yIntervaloMicros)) {
    yLastStepTime = tempoAtual;
    digitalWrite(Y_STEP, HIGH);
    delayMicroseconds(2);
    digitalWrite(Y_STEP, LOW);
  }
}
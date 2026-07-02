#include <Arduino.h>

// Mapeamento Blindado (Shield V4 + Nano ESP32)
#define EN_PIN    D8
#define X_STEP    D5
#define X_DIR     D2
#define Y_STEP    D6
#define Y_DIR     D3

long currentX = 0;
long targetX = 0;
long currentY = 0;
long targetY = 0;

unsigned long lastStepTime = 0;
unsigned int tempoEntrePassos = 2000; 

bool motorAtivo = false; // controla se o driver está habilitado

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 5000)) {
    delay(10);
  }
  Serial.setTxTimeoutMs(0); 
  
  pinMode(EN_PIN, OUTPUT);
  pinMode(X_STEP, OUTPUT);
  pinMode(X_DIR, OUTPUT);
  pinMode(Y_STEP, OUTPUT);
  pinMode(Y_DIR, OUTPUT);

  digitalWrite(EN_PIN, HIGH); // Começa desabilitado (motor "solto", sem aquecer)
  digitalWrite(X_DIR, HIGH);
  digitalWrite(Y_DIR, LOW); 
}

void loop() {
  // 1. LER COMANDOS
  if (Serial.available() > 0) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();
    
    int virgulaIndex = comando.indexOf(',');
    
    if (virgulaIndex > 0) {
      String strX = comando.substring(0, virgulaIndex);
      String strY = comando.substring(virgulaIndex + 1);
      
      targetX = strX.toInt();
      targetY = strY.toInt();
      
      Serial.print("OK:X=");
      Serial.print(targetX);
      Serial.print(",Y=");
      Serial.println(targetY);
    }
  }

  // 2. VERIFICAR SE PRECISA HABILITAR/DESABILITAR O DRIVER
  bool precisaMover = (currentX != targetX) || (currentY != targetY);

  if (precisaMover && !motorAtivo) {
    digitalWrite(EN_PIN, LOW);  // habilita driver (LOW = ligado no CNC Shield)
    motorAtivo = true;
  } else if (!precisaMover && motorAtivo) {
    digitalWrite(EN_PIN, HIGH); // desabilita driver, motor solto, sem aquecer
    motorAtivo = false;
  }

  // 3. EXECUTAR OS MOVIMENTOS
  unsigned long tempoAtual = micros();
  
  if (tempoAtual - lastStepTime >= tempoEntrePassos) {
    lastStepTime = tempoAtual;
    
    if (currentX != targetX) {
      if (targetX > currentX) {
        digitalWrite(X_DIR, HIGH); 
        currentX++;
      } else {
        digitalWrite(X_DIR, LOW);  
        currentX--;
      }
      digitalWrite(X_STEP, HIGH);
      delayMicroseconds(2);
      digitalWrite(X_STEP, LOW);
    }

    if (currentY != targetY) {
      if (targetY > currentY) {
        digitalWrite(Y_DIR, HIGH); 
        currentY++;
      } else {
        digitalWrite(Y_DIR, LOW);  
        currentY--;
      }
      digitalWrite(Y_STEP, HIGH);
      delayMicroseconds(2);
      digitalWrite(Y_STEP, LOW);
    }
  }
}
#include <PID_v1.h>

// ============================================
// CONFIGURACIÓN DE PINES (SIN CAMBIOS)
// ============================================

#define ENA 8     // PIN PWM (Confirmado en Mega)
#define IN1 9
#define IN2 10
#define IN3 11
#define IN4 12
#define ENB 13    // PIN PWM (Confirmado en Mega)

// Pines de encoders (SIN CAMBIOS)
#define ENCODER1_A 2    // Motor DERECHO
#define ENCODER1_B 3
#define ENCODER2_A 18   // Motor IZQUIERDO
#define ENCODER2_B 19

// ============================================
// VARIABLES GLOBALES
// ============================================

volatile long encoderCount1 = 0;
volatile long encoderCount2 = 0;

unsigned long lastTime1 = 0;
unsigned long lastTime2 = 0;

const int PPR = 360;

// Variables para el PID: Ahora serán POSITIVAS o CERO
double targetRPM_PID1 = 0;   // Módulo del target RPM Derecho
double targetRPM_PID2 = 0;   // Módulo del target RPM Izquierdo

// Variables que reciben el comando (puede ser negativo)
double targetRPM_DERECHO_COMMAND = 0;
double targetRPM_IZQUIERDO_COMMAND = 0;

double actualRPM1 = 0;
double actualRPM2 = 0;

double outputPWM1 = 0;
double outputPWM2 = 0;

// ============================================
// DEAD TIME (SIN CAMBIOS)
// ============================================
const int DEADTIME = 60;   // ms

int lastDir1 = 0;
int lastDir2 = 0;

// ============================================
// PID (AJUSTADO Y CORREGIDO PARA USAR MAGNITUDES POSITIVAS)
// ============================================

// ¡ATENCIÓN! Kp, Ki, Kd REDUCIDOS DRÁSTICAMENTE (Tuning inicial)
// Esto debe reducir el efecto de 'máxima velocidad'.
// Los RPM target son ahora targetRPM_PIDX (siempre positivos)
PID pid1(&actualRPM1, &outputPWM1, &targetRPM_PID1, 0.5, 0.1, 0.05, DIRECT);
PID pid2(&actualRPM2, &outputPWM2, &targetRPM_PID2, 0.5, 0.1, 0.05, DIRECT);

// ============================================
// INTERRUPCIONES (SIN CAMBIOS)
// ============================================

void encoderISR1() {
  int stateB = digitalRead(ENCODER1_B);
  encoderCount1 += (stateB == HIGH) ? 1 : -1;
}

void encoderISR2() {
  int stateB = digitalRead(ENCODER2_B);
  encoderCount2 += (stateB == HIGH) ? 1 : -1;
}

// ============================================
// FUNCIÓN DE DEAD TIME AUTOMÁTICO (SIN CAMBIOS)
// ============================================

void applyDeadTimeIfNeeded(int motor, int newDir) {
  if (motor == 1) { 
    if (newDir != lastDir1) {
      lastDir1 = newDir;
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, 0); 
      delay(DEADTIME);
    }
  } else { 
    if (newDir != lastDir2) {
      lastDir2 = newDir;
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, 0); 
      delay(DEADTIME);
    }
  }
}

// ============================================
// CONTROL MOTOR (FINAL)
// ============================================

void setMotorSpeed(int motor, double pwm, int direction) {

  pwm = constrain(pwm, 0, 255);

  applyDeadTimeIfNeeded(motor, direction);

  if (motor == 1) { // Motor DERECHO
    if (direction == 1) { 
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
    } else if (direction == -1) { 
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
    } else {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
    }

    analogWrite(ENA, (int)pwm);

  } else { // Motor IZQUIERDO (Lógica de giro corregida)
    if (direction == 1) { 
      digitalWrite(IN3, HIGH); 
      digitalWrite(IN4, LOW);
    } else if (direction == -1) { 
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    } else {
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }

    analogWrite(ENB, (int)pwm);
  }
}

// ============================================
// CÁLCULO RPM (IMPORTANTE: USA ABS)
// ============================================

void calculateRPM() {

  unsigned long t = millis();

  if (t - lastTime1 >= 100) {
    // Calculamos siempre la MAGNITUD de la velocidad (RPMs son positivos)
    actualRPM1 = (abs(encoderCount1) * 600.0) / (PPR * (t - lastTime1));
    encoderCount1 = 0;
    lastTime1 = t;
  }

  if (t - lastTime2 >= 100) {
    actualRPM2 = (abs(encoderCount2) * 600.0) / (PPR * (t - lastTime2));
    encoderCount2 = 0;
    lastTime2 = t;
  }
}

// ============================================
// PROCESO DE COMANDOS (AJUSTADO PARA EL NUEVO TARGET)
// ============================================

void processCommand(String command) {

  command.trim();

  if (command.startsWith("SPEED")) {

    int i1 = command.indexOf(" ");
    int i2 = command.indexOf(" ", i1 + 1);

    if (i1 > 0 && i2 > 0) {
      // 1. Guardar el comando original (incluye signo)
      targetRPM_IZQUIERDO_COMMAND = command.substring(i1 + 1, i2).toFloat(); 
      targetRPM_DERECHO_COMMAND = command.substring(i2 + 1).toFloat();    

      targetRPM_IZQUIERDO_COMMAND = constrain(targetRPM_IZQUIERDO_COMMAND, -120, 120);
      targetRPM_DERECHO_COMMAND = constrain(targetRPM_DERECHO_COMMAND, -120, 120);

      // 2. Establecer el Target RPM del PID como MAGNITUD (siempre positivo)
      targetRPM_PID1 = abs(targetRPM_DERECHO_COMMAND);
      targetRPM_PID2 = abs(targetRPM_IZQUIERDO_COMMAND);

      Serial.println("OK SPEED");
    }
  }

  else if (command == "STOP") {
    targetRPM_DERECHO_COMMAND = 0;
    targetRPM_IZQUIERDO_COMMAND = 0;
    targetRPM_PID1 = 0;
    targetRPM_PID2 = 0;

    setMotorSpeed(1, 0, 0);
    setMotorSpeed(2, 0, 0);

    Serial.println("OK STOP");
  }

  else if (command == "GET_RPM") {
    Serial.print("RPM ");
    Serial.print(actualRPM1); 
    Serial.print(" ");
    Serial.println(actualRPM2); 
  }

  else {
    Serial.println("ERROR");
  }
}

// ============================================
// SETUP (SIN CAMBIOS)
// ============================================

void setup() {

  Serial.begin(115200);
  Serial.setTimeout(10);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(ENCODER1_A, INPUT_PULLUP);
  pinMode(ENCODER1_B, INPUT_PULLUP);
  pinMode(ENCODER2_A, INPUT_PULLUP);
  pinMode(ENCODER2_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(ENCODER1_A), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER2_A), encoderISR2, RISING);
  
  pid1.SetMode(AUTOMATIC);
  pid2.SetMode(AUTOMATIC);
  pid1.SetOutputLimits(0, 255);
  pid2.SetOutputLimits(0, 255);
  pid1.SetSampleTime(50);
  pid2.SetSampleTime(50);

  setMotorSpeed(1, 0, 0);
  setMotorSpeed(2, 0, 0);

  Serial.println("ARDUINO READY");
}

// ============================================
// LOOP
// ============================================

void loop() {

  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    processCommand(command);
  }

  calculateRPM();

  // El PID ahora usa targetRPM_PIDX (que es positivo) y actualRPMX (positivo)
  pid1.Compute();
  pid2.Compute();

  // La dirección se toma del comando original (puede ser negativo)
  int dir1 = (targetRPM_DERECHO_COMMAND > 0) ? 1 : (targetRPM_DERECHO_COMMAND < 0 ? -1 : 0);
  int dir2 = (targetRPM_IZQUIERDO_COMMAND > 0) ? 1 : (targetRPM_IZQUIERDO_COMMAND < 0 ? -1 : 0);

  // Se aplica el PWM (salida del PID) y la dirección (salida del comando)
  setMotorSpeed(1, outputPWM1, dir1);
  setMotorSpeed(2, outputPWM2, dir2);

  delay(10); 
}
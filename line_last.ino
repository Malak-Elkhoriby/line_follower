#include <Arduino.h>
#include <QTRSensors.h>

// ===================== MOTOR PINS =====================
// RIGHT MOTOR
#define IN1 16
#define IN2 4
#define ENA 25
// LEFT MOTOR
#define IN3 5
#define IN4 17
#define ENB 26

// ===================== PWM =====================
#define CH_1 1
#define CH_2 2

#define PWM_FREQ 1000
#define PWM_RES 8

// ===================== SPEED =====================
#define MAX_SPEED 200
#define BASE_SPEED 120

// ===================== QTR =====================
QTRSensors qtr;
const uint8_t SensorCount = 5;
uint16_t sensorValues[SensorCount];

// Sensor pins
const uint8_t sensorPins[SensorCount] = {27, 32, 33, 34, 35};

// ===================== PID =====================
float Kp = 0.12;
float Ki = 0;
float Kd = 0.8;

float P = 0;
float I = 0;
float D = 0;

float last_error = 0;

// ===================== VARIABLES =====================
int lastDirection = 0;
// -1 = left
//  1 = right

// ===================== MOTOR SETUP =====================
void setup_Motor()
{
  ledcSetup(CH_1, PWM_FREQ, PWM_RES);
  ledcSetup(CH_2, PWM_FREQ, PWM_RES);

  ledcAttachPin(ENA, CH_1);
  ledcAttachPin(ENB, CH_2);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
}

// ===================== DIRECTION =====================
void setDirection(bool R1, bool R2, bool L1, bool L2)
{
  digitalWrite(IN1, R1);
  digitalWrite(IN2, R2);

  digitalWrite(IN3, L1);
  digitalWrite(IN4, L2);
}

// ===================== SPEED =====================
void setSpeed(int leftSpeed, int rightSpeed)
{
  leftSpeed = constrain(leftSpeed, -MAX_SPEED, MAX_SPEED);
  rightSpeed = constrain(rightSpeed, -MAX_SPEED, MAX_SPEED);

  // RIGHT MOTOR
  if(rightSpeed >= 0)
  {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else
  {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    rightSpeed = -rightSpeed;
  }

  // LEFT MOTOR
  if(leftSpeed >= 0)
  {
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else
  {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    leftSpeed = -leftSpeed;
  }

  ledcWrite(CH_1, rightSpeed);
  ledcWrite(CH_2, leftSpeed);
}

// ===================== PID =====================

float computePID(float error)
{
  P = error;

  I += error;

  D = error - last_error;

  float correction =
  (Kp * P) +
  (Ki * I) +
  (Kd * D);

  last_error = error;

  return correction;
}

// ===================== SETUP =====================

void setup()
{
  Serial.begin(115200);

  setup_Motor();

  // QTR setup
  qtr.setTypeAnalog();

  qtr.setSensorPins(sensorPins, SensorCount);

  delay(500);

  // ================= CALIBRATION =================

  Serial.println("Calibrating...");

  for (uint16_t i = 0; i < 250; i++)
  {
    qtr.calibrate();
    delay(5);
  }

  Serial.println("Calibration Done!");
  delay(3000);
}

// ===================== LOOP =====================
void loop()
{
  // Read line position
  uint16_t position = qtr.readLineBlack(sensorValues);

  float error = 2000 - position;

  // ================= LOST LINE =================

  bool lineLost = true;

  for(int i = 0; i < SensorCount; i++)
  {
    if(sensorValues[i] < 900)
    {
      lineLost = false;
      break;
    }
  }

  // ================= IF LINE LOST =================

  if(lineLost)
  {
    I = 0;
    last_error = 0;

    if(lastDirection == 1)
    {
      setSpeed(100, -100);
    }
    else if(lastDirection == -1)
    {
      setSpeed(-100, 100);
    }
    else
    {
      setSpeed(0, 0);
    }

    return;
  }

  // ================= HARD TURN OVERRIDE =================

  if(abs(error) > 1200)
  {
    if(error > 0)
      setSpeed(100, -100);
    else
      setSpeed(-100, 100);

    return;
  }

  // ================= NORMAL TRACKING =================

  static float filteredError = 2000;
  filteredError = 0.7 * filteredError + 0.3 * error;

  float correction = computePID(filteredError);

  static float filteredCorr = 0;
  filteredCorr = 0.7 * filteredCorr + 0.3
   * correction;

  I = constrain(I, -200, 200);

  if(filteredCorr > 20)
    lastDirection = 1;
  else if(filteredCorr < -20)
    lastDirection = -1;

  int baseSpeed;

  if(abs(error) < 200)
  {
    baseSpeed = 150;
  }
  else
  {
    baseSpeed = 100;
  }

  int leftSpeed = baseSpeed + correction;
  int rightSpeed = baseSpeed - correction;

  setSpeed(leftSpeed, rightSpeed);

  Serial.print("Position: ");
  Serial.print(position);

  Serial.print("  Error: ");
  Serial.print(error);

  Serial.print("  Correction: ");
  Serial.println(correction);
}

#include <Wire.h>
#include <PID_v1.h>

#define MPU_ADDR 0x68

int16_t ax, ay, az;
int16_t gx, gy, gz;

float angle = 0;
float bias = 0;

float P[2][2] = {
  {0,0},
  {0,0}
};

float Q_angle = 0.001;
float Q_bias  = 0.003;
float R_measure = 0.01;

double setpoint = -3;

double input = 0;
double output = 0;

double Kp = 58;
double Ki = 0;
double Kd = 2.0;

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

unsigned long lastTime;

#define PWM_FREQ 20000
#define PWM_RES 8

#define CH1 0
#define CH2 1

int ENA = 25;
int IN1 = 26;
int IN2 = 27;

int ENB = 33;
int IN3 = 32;
int IN4 = 14;

float kalman(float newAngle, float newRate, float dt) {

  float rate = newRate - bias;
  angle += dt * rate;

  P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
  P[0][1] -= dt * P[1][1];
  P[1][0] -= dt * P[1][1];
  P[1][1] += Q_bias * dt;

  float S = P[0][0] + R_measure;

  float K[2];
  K[0] = P[0][0] / S;
  K[1] = P[1][0] / S;

  float y = newAngle - angle;

  angle += K[0] * y;
  bias += K[1] * y;

  float P00 = P[0][0];
  float P01 = P[0][1];

  P[0][0] -= K[0] * P00;
  P[0][1] -= K[0] * P01;
  P[1][0] -= K[1] * P00;
  P[1][1] -= K[1] * P01;

  return angle;
}

void setup() {

  Serial.begin(115200);

  Wire.begin(21, 22);
  Wire.setClock(400000);

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  ledcSetup(CH1, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENA, CH1);

  ledcSetup(CH2, PWM_FREQ, PWM_RES);
  ledcAttachPin(ENB, CH2);

  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(-255, 255);
  pid.SetSampleTime(5);

  lastTime = millis();
}

void loop() {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);

  Wire.requestFrom(MPU_ADDR, 14);

  ax = Wire.read() << 8 | Wire.read();
  ay = Wire.read() << 8 | Wire.read();
  az = Wire.read() << 8 | Wire.read();

  Wire.read();
  Wire.read();

  gx = Wire.read() << 8 | Wire.read();
  gy = Wire.read() << 8 | Wire.read();
  gz = Wire.read() << 8 | Wire.read();

  float dt = (millis() - lastTime) / 1000.0;
  lastTime = millis();

  if (dt <= 0) return;

  float accelAngle = atan2(ax, az) * 180 / PI;

  float gyroRate = -gy / 131.0;

  float kalmanAngle = kalman(accelAngle, gyroRate, dt);

  input = kalmanAngle;

  pid.Compute();

  int motorSpeed;

  if (abs(input - setpoint) < 10) {
    motorSpeed = output * 0.55;
  } else {
    motorSpeed = output;
  }

  if (abs(input - setpoint) < 1.0) {
    motorSpeed = 0;
  }

  moveMotor(motorSpeed);

  Serial.print("Angulo: ");
  Serial.print(input);

  Serial.print(" | PID: ");
  Serial.print(output);

  Serial.print(" | Motor: ");
  Serial.println(motorSpeed);

  delayMicroseconds(1000);
}

void moveMotor(int speed) {

  if (speed > 0) {

    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);

    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);

  } else {

    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);

    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }

  int pwm = abs(speed);

  if (pwm > 0) {
    pwm = map(pwm, 0, 255, 100, 255);
  }

  pwm = constrain(pwm, 0, 255);

  ledcWrite(CH1, pwm);
  ledcWrite(CH2, pwm);
}
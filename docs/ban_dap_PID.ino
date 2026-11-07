volatile long encoderCount = 0;

// ================= Encoder =================
const int encoderA = 2;
const int encoderB = 3;

// ================= Analog input =================
const int chanGa    = A0;
const int chanPhanh = A1;
const int chanLui   = A2;

// ================= IBT-2 =================
const int RL_EN = 8;   // R_EN và L_EN nối chung
const int R_PWM = 9;
const int L_PWM = 10;

// ================= Calibration pedal =================
const int ga_min = 335;
const int ga_max = 635;

const int lui_min = 400;
const int lui_max = 635;

const int phanh_max = 860;
const int phanh_min = 650;

// ================= PID trả lái =================
float Kp = 0.65;
float Ki = 0.001;
float Kd = 0.075;

float setpoint = 0.0f;
float error = 0.0f;
float prevError = 0.0f;
float integral = 0.0f;

unsigned long prevTime = 0;

// ================= Giới hạn =================
const long encoderMin = -360;
const long encoderMax =  360;

const int pwmMin = 0;
const int pwmMax = 60;
const int centerDeadband = 4;

// =====================================================
float scaleToPercent(int value, int in_min, int in_max) {
  if (in_min == in_max) return 0.0f;

  float percent = (float)(value - in_min) * 100.0f / (float)(in_max - in_min);

  if (percent < 0.0f) percent = 0.0f;
  if (percent > 100.0f) percent = 100.0f;

  return percent;
}

// =====================================================
void setup() {
  Serial.begin(115200);

  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  pinMode(RL_EN, OUTPUT);
  pinMode(R_PWM, OUTPUT);
  pinMode(L_PWM, OUTPUT);

  digitalWrite(RL_EN, HIGH);   // bật mạch cầu
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);

  attachInterrupt(digitalPinToInterrupt(encoderA), readEncoder, RISING);

  prevTime = millis();
}

// =====================================================
void loop() {
  long count;

  noInterrupts();
  count = encoderCount;
  interrupts();

  count = constrain(count, encoderMin, encoderMax);

  int rawGa    = analogRead(chanGa);
  int rawPhanh = analogRead(chanPhanh);
  int rawLui   = analogRead(chanLui);

  float ga    = scaleToPercent(rawGa, ga_min, ga_max);
  float lui   = scaleToPercent(rawLui, lui_min, lui_max);
  float phanh = scaleToPercent(rawPhanh, phanh_max, phanh_min);

  if (ga < 20.0f) ga = 0.0f;
  if (lui < 20.0f) lui = 0.0f;
  if (phanh < 20.0f) phanh = 0.0f;

  if (ga > 0.0f && lui > 0.0f) {
    ga = 0.0f;
    lui = 0.0f;
  }

  steeringPID(count);

  Serial.print(count);
  Serial.print("|");
  Serial.print(ga, 1);
  Serial.print("|");
  Serial.print(lui, 1);
  Serial.print("|");
  Serial.println(phanh, 1);

  delay(10);
}

// =====================================================
void steeringPID(long currentPosition) {
  unsigned long now = millis();
  float dt = (now - prevTime) / 1000.0f;
  if (dt <= 0.0f) dt = 0.001f;

  error = setpoint - (float)currentPosition;

  if (abs(error) <= centerDeadband) {
    integral = 0.0f;
    prevError = error;
    prevTime = now;
    stopMotor();
    return;
  }

  integral += error * dt;

  if (integral > 300.0f) integral = 300.0f;
  if (integral < -300.0f) integral = -300.0f;

  float derivative = (error - prevError) / dt;
  float output = Kp * error + Ki * integral + Kd * derivative;

  int pwm = (int)abs(output);
  pwm = constrain(pwm, pwmMin, pwmMax);

  if (output > 0.0f) {
    spinLeft(pwm);
  } else {
    spinRight(pwm);
  }

  prevError = error;
  prevTime = now;
}

// =====================================================
void spinRight(int pwm) {
  digitalWrite(RL_EN, HIGH);
  analogWrite(R_PWM, pwm);
  analogWrite(L_PWM, 0);
}

void spinLeft(int pwm) {
  digitalWrite(RL_EN, HIGH);
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, pwm);
}

void stopMotor() {
  digitalWrite(RL_EN, HIGH);
  analogWrite(R_PWM, 0);
  analogWrite(L_PWM, 0);
}

// =====================================================
void readEncoder() {
  if (digitalRead(encoderB) == HIGH) {
    encoderCount++;
  } else {
    encoderCount--;
  }
}
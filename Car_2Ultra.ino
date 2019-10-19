double Input, Output;

//PID parameters
double Kp = 4 ;
double Ki = 0.0 ;
double Kd = 0.4;
const int Setpoint = 10;
double ki, kd;

//Pin Setup
const int EchoPinL = 2;
const int TrigPinL = 3;
const int EchoPinR = 4;
const int TrigPinR = 5;

double lastInput, lastTime;
const int OutMin = -150;
const int OutMax = 150;

int SampleTime = 1000;
double  systemtime;

float distanceL, distanceR, distance;
double ITerm, error;
int motor1 = 12;
int motor2 = 11;
int motor3 = 10;
int motor4 = 9;
float pwm12 = 13;
float pwm34 = 8;

void setup()
{
  Serial.begin(115200);
  pinMode(TrigPinL, OUTPUT);
  pinMode(EchoPinL, INPUT);
  pinMode(TrigPinR, OUTPUT);
  pinMode(EchoPinR, INPUT);

  pinMode(motor1, OUTPUT);
  pinMode(motor2, OUTPUT);
  pinMode(motor3, OUTPUT);
  pinMode(motor4, OUTPUT);
  pinMode(pwm12, OUTPUT);
  pinMode(pwm34, OUTPUT);
}


void loop()
{
  LeftSensor();
  RightSensor();
  if (distanceL - distanceR > 30)
  {
    TurnLeft();
    Serial.print("Power = ");
    Serial.print("Turn Left");
    Serial.print("\n");
  }
  else if (distanceR - distanceL > 30)
  {
    TurnRight();
    Serial.print("Power = ");
    Serial.print("Turn Right");
    Serial.print("\n");
  }
  else
  {
    distance = (distanceL + distanceR) / 2;
    Input = distance;
    Serial.println(" \n ");
    Serial.println("Left = ");
    Serial.print(distanceL);
    Serial.println("");
    Serial.println("Right = ");
    Serial.print(distanceR);
    unsigned long now = millis();
    double dtime = (now - lastTime);
    systemtime = now / 1000;

    ki = Ki * SampleTime;
    kd = Kd / SampleTime;
    if (dtime >= SampleTime)
    {
      error = -Setpoint + Input;
      ITerm += (ki * error);
      double dInput = (Input - lastInput);
      Output = Kp * error + ITerm - kd * dInput;
      
      if (Output > OutMax) Output = OutMax;
      else if (Output < OutMin) Output = OutMin;
      if (ITerm > OutMax) ITerm = OutMax;
      else if (ITerm < OutMin) ITerm = OutMin;
      lastInput = Input;
      lastTime = now;
    }
    if (distance <= 30)
    {
      Serial.print("\n");
      Serial.print("Power = ");
      Serial.print(Output);
      Serial.print("\n");
      if (Output >= 0)
      {
        motorPWM(Output);
      }
      else if (Output < 0)
      {
        motorPWM_back(Output);
      }

    }
    else
    {
      Serial.print("\n");
      Serial.print("Power = ");
      Serial.print("80");
      Serial.print("\n");
      motorPWM(OutMax);
    }
  }

  delay(10);
}

void LeftSensor()
{
  digitalWrite(TrigPinL, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPinL, LOW);
  distanceL = pulseIn(EchoPinL, HIGH) / 58.00;
}
void RightSensor()
{
  digitalWrite(TrigPinR, LOW);
  delayMicroseconds(2);
  digitalWrite(TrigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(TrigPinR, LOW);
  distanceR = pulseIn(EchoPinR, HIGH) / 58.00;
}
void TurnLeft()
{
  digitalWrite(motor1, 0);
  digitalWrite(motor2, 1);
  analogWrite(pwm12, OutMax);
  digitalWrite(motor3, 1);
  digitalWrite(motor4, 0);
  analogWrite(pwm34, OutMax);

}
void TurnRight()
{
  digitalWrite(motor1, 1);
  digitalWrite(motor2, 0);
  analogWrite(pwm12, OutMax);
  digitalWrite(motor3, 0);
  digitalWrite(motor4, 1);
  analogWrite(pwm34, OutMax);
}
void motorPWM(int pwm) {
  digitalWrite(motor1, 1);
  digitalWrite(motor2, 0);
  analogWrite(pwm12, pwm);
  digitalWrite(motor3, 1);
  digitalWrite(motor4, 0);
  analogWrite(pwm34, pwm);
}
void motorPWM_back(int pwm) {
  digitalWrite(motor1, 0);
  digitalWrite(motor2, 1);
  analogWrite(pwm12, pwm);
  digitalWrite(motor3, 0);
  digitalWrite(motor4, 1);
  analogWrite(pwm34, pwm);
}

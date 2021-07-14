
#pragma region LIGHT_CONTROL

const byte LED_OFF = 0;
const byte LED_RED = 2;
const byte LED_GREEN = 3;
const byte LED_BLUE = 4;

void setLightColor(byte color)
{

  digitalWrite(LED_RED, false);
  digitalWrite(LED_GREEN, false);
  digitalWrite(LED_BLUE, false);

  if (color != LED_OFF)
    digitalWrite(color, true);
}

#pragma endregion

#pragma region RECEIVER_COMMUNICATION

const int THROTTLE_IN = 7;
const int STEER_IN = 6;

const int VALUE_IN_OFFSET_RANGE = 500;
const int VALUE_IN_MIDDLE_IGNORE_NOISE = 4;

const int VALUE_IN_MIN = 796;
const int VALUE_IN_MAX = 2190;

const int VALUE_IN_MIDDLE = (VALUE_IN_MIN + VALUE_IN_MAX) / 2;

const int THROTTLE_STEER_IN_MIN = VALUE_IN_MIN + VALUE_IN_OFFSET_RANGE / 2;
const int THROTTLE_STEER_IN_MAX = VALUE_IN_MAX - VALUE_IN_OFFSET_RANGE / 2 - VALUE_IN_MIDDLE_IGNORE_NOISE * 2;

int throttleValue,
    steerValue;

int throttle,
    steer;

bool disconnected;

void readThrottle()
{
  throttleValue = pulseIn(THROTTLE_IN, HIGH, 50000);

  if (throttleValue > VALUE_IN_MAX || throttleValue < VALUE_IN_MIN)
  {
    disconnected = true;
    return;
  }

  if (throttleValue > (VALUE_IN_MIDDLE - VALUE_IN_MIDDLE_IGNORE_NOISE) &&
      throttleValue < (VALUE_IN_MIDDLE + VALUE_IN_MIDDLE_IGNORE_NOISE))
  {
    throttle = 0;
    return;
  }

  throttleValue -= VALUE_IN_MIDDLE_IGNORE_NOISE * 2;

  throttleValue = constrain(throttleValue, THROTTLE_STEER_IN_MIN, THROTTLE_STEER_IN_MAX);
  throttle = map(throttleValue, THROTTLE_STEER_IN_MIN, THROTTLE_STEER_IN_MAX, -255, 255);
}

void readSteer()
{
  steerValue = pulseIn(STEER_IN, HIGH, 50000);

  if (steerValue > VALUE_IN_MAX || steerValue < VALUE_IN_MIN)
  {
    disconnected = true;
    return;
  }

  steerValue -= VALUE_IN_MIDDLE_IGNORE_NOISE * 2;
  steerValue = constrain(steerValue, THROTTLE_STEER_IN_MIN, THROTTLE_STEER_IN_MAX);
  steer = map(steerValue, THROTTLE_STEER_IN_MIN, THROTTLE_STEER_IN_MAX, -255, 255);
}

#pragma endregion

#pragma region MOTOR_CONTROL

const byte MOTOR_LEFT_FORWARD_PWM = 9;
const byte MOTOR_RIGHT_FORWARD_PWM = 10;

const byte MOTOR_LEFT_BACKWARD_PWM = 3;
const byte MOTOR_RIGHT_BACKWARD_PWM = 11;

const byte MOTOR_LEFT_FORWARD_UNLOCK = 5;
const byte MOTOR_RIGHT_FORWARD_UNLOCK = 8;

const byte MOTOR_LEFT_BACKWARD_UNLOCK = 2;
const byte MOTOR_RIGHT_BACKWARD_UNLOCK = 4;

float powerLeft,
      powerRight;

bool armed;

void arm()
{
  armed = true;
  digitalWrite(MOTOR_LEFT_FORWARD_UNLOCK, true);
  digitalWrite(MOTOR_RIGHT_FORWARD_UNLOCK, true);

  digitalWrite(MOTOR_LEFT_BACKWARD_UNLOCK, true);
  digitalWrite(MOTOR_RIGHT_BACKWARD_UNLOCK, true);
  wait(10);
}
void disarm()
{
  armed = false;
  
  powerLeft = 0;
  powerRight = 0;

  analogWrite(MOTOR_LEFT_FORWARD_PWM, 0);
  analogWrite(MOTOR_RIGHT_FORWARD_PWM, 0);

  analogWrite(MOTOR_LEFT_BACKWARD_PWM, 0);
  analogWrite(MOTOR_RIGHT_BACKWARD_PWM, 0);
  
  digitalWrite(MOTOR_LEFT_FORWARD_UNLOCK, false);
  digitalWrite(MOTOR_RIGHT_FORWARD_UNLOCK, false);

  digitalWrite(MOTOR_LEFT_BACKWARD_UNLOCK, false);
  digitalWrite(MOTOR_RIGHT_BACKWARD_UNLOCK, false);

}

// Change of power is limited to 35% of the maximum power per second
const float POWER_CHANGE_PER_SECOND = 0.7f * 255;

float updateMotorPower(int currentInput, float currentPower, unsigned long ellapsedMicros)
{
  if (currentInput == (int)currentPower)
  {
    return currentInput;
  }

  ellapsedMicros = constrain(ellapsedMicros, 1, 1000000);

  float powerDifference = currentPower - currentInput;
  float powerTimeChange = POWER_CHANGE_PER_SECOND * ((float)ellapsedMicros / 1000 / 1000);

  if (abs(powerDifference) <= powerTimeChange)
  {
    currentPower = (float)currentInput;
    return currentPower;
  }
  else
  {
    bool increasePower = currentPower < currentInput;
    float newPowerValue = currentPower + (increasePower ? powerTimeChange : (-powerTimeChange));
    return newPowerValue;
  }
}

float updateMotorOutput(int currentInput, float currentPower, unsigned long ellapsedMillis, byte pinForwardPWM, byte pinBackwardPWM, byte pinForwardUnlock, byte pinBackwardUnlock)
{
  if (currentInput < 0) {
    // when going backwards, drive slower
    currentInput = map(currentInput, -255, 0, -80, 0);
  }
  
  currentPower = updateMotorPower(currentInput, currentPower, ellapsedMillis);

  if (currentPower == 0)
  {
    analogWrite(pinForwardPWM, 0);
    analogWrite(pinBackwardPWM, 0);
    digitalWrite(pinForwardUnlock, false);
    digitalWrite(pinBackwardUnlock, false);
  }
  else
  {
    digitalWrite(pinForwardUnlock, true);
    digitalWrite(pinBackwardUnlock, true);

    bool forward = currentPower > 0;
    if (forward)
    {
      analogWrite(pinForwardPWM, currentPower);
      analogWrite(pinBackwardPWM, 0);
    }
    else
    {
      analogWrite(pinForwardPWM, 0);
      analogWrite(pinBackwardPWM, abs(currentPower));
    }
  }

  return currentPower;
}

#pragma endregion

#pragma region POWER_OBSERVATION

const byte CURRENT_RIGHT_IN = A0;
const byte CURRENT_LEFT_IN = A1;

const byte VOLTAGE_IN = A7;

const byte MAX_CURRENT = 10;

const float CURRENT_MULT = (float)5 / 1023 * 10;
const float VOLTAGE_MULT = (float)5 / 1023 * 130 / 30;

const float CELL_VOLTAGE_MAX = 4.30f;
const float CELL_VOLTAGE_MIN = 2.75f;

float currentRight,
    currentLeft;
float overCurrentRightIntegral,
    overCurrentLeftIntegral;

float voltage;

int cellCount;
float minBatteryVolt;

bool lowVoltage;
int lowVoltageReducement;

void readCurrent()
{

  float _currentRight = (float)analogRead(CURRENT_RIGHT_IN) * CURRENT_MULT;
  float _currentLeft = (float)analogRead(CURRENT_LEFT_IN) * CURRENT_MULT;

  currentRight = (_currentRight + currentRight * 7) / 8;
  currentLeft = (_currentLeft + currentLeft * 7) / 8;

  Serial.print(currentRight);
  Serial.print("A\t");
  Serial.print(currentLeft);
  Serial.print("A\t");

  float rightReducement = (float)(powerRight + 20) / (255 + 20) * MAX_CURRENT;
  float leftReducement = (float)(powerLeft + 20) / (255 + 20) * MAX_CURRENT;

  if (powerRight == 0)
  {
    rightReducement = 0.01f;
  }
  if (powerLeft == 0)
  {
    leftReducement = 0.01f;
  }

  float rightOvercurrentValue = currentRight - rightReducement;
  float leftOvercurrentValue = currentLeft - leftReducement;

  overCurrentRightIntegral += rightOvercurrentValue;
  overCurrentLeftIntegral += leftOvercurrentValue;

  overCurrentRightIntegral = constrain(overCurrentRightIntegral, 0, MAX_CURRENT);
  overCurrentLeftIntegral = constrain(overCurrentLeftIntegral, 0, MAX_CURRENT);

  bool rightOvercurrent = (overCurrentRightIntegral == MAX_CURRENT) || (currentRight > MAX_CURRENT);
  bool leftOvercurrent = (overCurrentLeftIntegral == MAX_CURRENT) || (currentLeft > MAX_CURRENT);

  if (rightOvercurrent || leftOvercurrent)
  {
    disarm();

    setLightColor(LED_BLUE);

    overCurrentRightIntegral = 0;
    overCurrentLeftIntegral = 0;

    Serial.println();
    Serial.print("over current");

    if (rightOvercurrent)
      Serial.print(" right");
    if (leftOvercurrent)
      Serial.print(" left");
    Serial.println();

    wait(3000);
  }
}

void detectCellCount()
{

  readBatteryVoltage();

  for (int i = 1; i < 5; i++)
  {
    if (voltage < i * CELL_VOLTAGE_MAX)
    {
      if (voltage > i * CELL_VOLTAGE_MIN)
      {
        cellCount = i;
      }
      else
      {
        cellCount = i - 1;
      }
      break;
    }
  }

  minBatteryVolt = cellCount * CELL_VOLTAGE_MIN * 0.9;

  Serial.println();
  Serial.print(cellCount);
  Serial.println(" battery cell(s) detected");
  Serial.print(minBatteryVolt);
  Serial.println("V min bat voltage");
}

void readBatteryVoltage()
{

  voltage = (float)analogRead(VOLTAGE_IN) * VOLTAGE_MULT;

  Serial.print(voltage);
  Serial.print("V\t");

  lowVoltage = voltage < minBatteryVolt;

  if (lowVoltage)
  {
    disarm();

    Serial.println();
    Serial.println("low voltage");
  }
}

#pragma endregion

void setup()
{

  pinMode(MOTOR_LEFT_FORWARD_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD_PWM, OUTPUT);

  pinMode(MOTOR_LEFT_BACKWARD_PWM, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD_PWM, OUTPUT);

  pinMode(MOTOR_LEFT_FORWARD_UNLOCK, OUTPUT);
  pinMode(MOTOR_RIGHT_FORWARD_UNLOCK, OUTPUT);

  pinMode(MOTOR_LEFT_BACKWARD_UNLOCK, OUTPUT);
  pinMode(MOTOR_RIGHT_BACKWARD_UNLOCK, OUTPUT);

  disarm();

  //---------------------------------------------- Set PWM frequency for D9 & D10 ----------------------------
  TCCR1B = TCCR1B & B11111000 | B00000010; // set timer 1 divisor to     8 for PWM frequency of  3921.16 Hz

  //---------------------------------------------- Set PWM frequency for D3 & D11 ----------------------------
  TCCR2B = TCCR2B & B11111000 | B00000010; // set timer 2 divisor to     8 for PWM frequency of  3921.16 Hz

  Serial.begin(115200);

  pinMode(THROTTLE_IN, INPUT);
  pinMode(STEER_IN, INPUT);

  // pinMode(LED_RED, OUTPUT);
  // pinMode(LED_GREEN, OUTPUT);
  // pinMode(LED_BLUE, OUTPUT);

  // digitalWrite(LED_RED, false);
  // digitalWrite(LED_GREEN, false);
  // digitalWrite(LED_BLUE, false);

  disconnected = true;

  // detectCellCount();
}

unsigned long lastMotorUpdateCall;

void loop()
{

  // readCurrent();
  // readBatteryVoltage();

  readThrottle();
  readSteer();

  if (disconnected)
  {
    disarm();

    // setLightColor(LED_RED);
    Serial.println();
    Serial.println("disconnected");

    while (disconnected)
    {
      disconnected = false;
      readSteer();
    }
    lastMotorUpdateCall = micros();
    // setLightColor(LED_GREEN);
    return;
  }

  // if (lowVoltage) {
  //   lowVoltageReducement -= 15;
  //   int value = millis() % 1000;
  //   if (value > 500) {
  //     setLightColor(LED_BLUE);
  //   } else {
  //     setLightColor(LED_OFF);
  //   }
  // }
  // else {
  //   if (disconnected) {
  //     setLightColor(LED_RED);
  //   } else {
  //     setLightColor(LED_GREEN);
  //   }
  //   lowVoltageReducement += 5;
  // }

  // lowVoltageReducement = constrain(lowVoltageReducement, -255 + 20, 0);

  // Serial.println();
  // Serial.print(lowVoltageReducement);
  // Serial.println(" lowVoltageReducement");

  // when right is greater, then the boat should turn right
  // therefore the right motor receives less power
  int rightValue = constrain(throttle * (1 - (float)constrain(steer, 0, 255) / 255) + lowVoltageReducement, -255, 255),
      leftValue = constrain(throttle * (1 + (float)constrain(steer, -255, 0) / 255) + lowVoltageReducement, -255, 255);

  Serial.print('\t');
  Serial.print(leftValue);

  Serial.print('\t');
  Serial.println(rightValue);

  // const byte MOTOR_LEFT_FORWARD_PWM = 9;
  // const byte MOTOR_RIGHT_FORWARD_PWM = 10;

  // const byte MOTOR_LEFT_BACKWARD_PWM = 3;
  // const byte MOTOR_RIGHT_BACKWARD_PWM = 11;

  // const byte MOTOR_LEFT_FORWARD_UNLOCK = 5;
  // const byte MOTOR_RIGHT_FORWARD_UNLOCK = 6;

  // const byte MOTOR_LEFT_BACKWARD_UNLOCK = 2;
  // const byte MOTOR_RIGHT_BACKWARD_UNLOCK = 4;

  unsigned long ellapsedMicros = micros() - lastMotorUpdateCall;
  lastMotorUpdateCall = micros();

  powerRight = updateMotorOutput(leftValue, powerRight, ellapsedMicros, MOTOR_RIGHT_FORWARD_PWM, MOTOR_RIGHT_BACKWARD_PWM, MOTOR_RIGHT_FORWARD_UNLOCK, MOTOR_RIGHT_BACKWARD_UNLOCK);
  powerLeft = updateMotorOutput(rightValue, powerLeft, ellapsedMicros, MOTOR_LEFT_FORWARD_PWM, MOTOR_LEFT_BACKWARD_PWM, MOTOR_LEFT_FORWARD_UNLOCK, MOTOR_LEFT_BACKWARD_UNLOCK);

  Serial.print('\t');
  Serial.print(powerRight);

  Serial.print('\t');
  Serial.println(powerLeft);

  // if (right == 0)
  // {
  //   digitalWrite(MOTOR_RIGHT_FORWARD_UNLOCK, false);
  //   digitalWrite(MOTOR_RIGHT_BACKWARD_UNLOCK, false);
  //   analogWrite(MOTOR_RIGHT_FORWARD_PWM, 0);
  //   analogWrite(MOTOR_RIGHT_BACKWARD_PWM, 0);
  // }
  // else
  // {

  //   digitalWrite(MOTOR_RIGHT_FORWARD_UNLOCK, true);
  //   digitalWrite(MOTOR_RIGHT_BACKWARD_UNLOCK, true);
  //   bool rightForward = right > 0;

  //   if (!rightForward)
  //   {
  //     // when going backwards, drive slower
  //     right = map(right, -255, 0, 80, 0);
  //   }

  //   powerRight = right;
  //   analogWrite(rightForward ? MOTOR_RIGHT_BACKWARD_PWM : MOTOR_RIGHT_FORWARD_PWM, 0);
  //   analogWrite(rightForward ? MOTOR_RIGHT_FORWARD_PWM : MOTOR_RIGHT_BACKWARD_PWM, powerRight);
  // }

  // if (left == 0) {
  //   digitalWrite(MOTOR_LEFT_UNLOCK, false);
  //   analogWrite(MOTOR_LEFT, 0);
  //   powerLeft = 0;
  // }
  // else
  // {
  //   if (leftZero) {
  //     powerLeft = 5;
  //     digitalWrite(MOTOR_LEFT_UNLOCK, true);
  //   } else
  //     powerLeft = (left + powerLeft * 9) / 10;
  //   analogWrite(MOTOR_LEFT, powerLeft);
  // }

  // if (rightZero || leftZero && (powerRight != 0 || powerLeft != 0))
  //   wait(100);

  Serial.println();
}

void wait(unsigned long msec)
{

  unsigned long endTime = millis() + msec;
  while (millis() < endTime)
  {
    // readBatteryVoltage();
    // readCurrent();
  }
}

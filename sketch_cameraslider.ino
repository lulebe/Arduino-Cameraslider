#include <SoftwareSerial.h>

//Pin setup

#define PIN_RX_BT 11
#define PIN_TX_BT 10
#define PIN_MOTOR_DIR 4
#define PIN_MOTOR_STEP 2
#define PIN_SLEEP 3
#define PIN_MS1 7
#define PIN_MS2 6
#define PIN_MS3 5
#define PIN_SENSOR 9
#define PIN_BUZZER 12

//Variable setup

SoftwareSerial bluetooth(PIN_RX_BT, PIN_TX_BT);
int direction = HIGH;
unsigned int halfStepDelayMicro = 500;
unsigned int stepDelayMilli = 0;
bool useMilliDelay = false;
int stepSize = 1;
unsigned int maxStep = 0; //~3680 maximum
unsigned int curStep = 0;
bool running = false;
int totalVoltage = 0;
int batteryCheckCount = 0;
unsigned long lastBatteryCheck = 0;

//initial setup

void setup() {
  pinMode(PIN_MOTOR_DIR, OUTPUT); 
  pinMode(PIN_MOTOR_STEP, OUTPUT);
  pinMode(PIN_SLEEP, OUTPUT);
  pinMode(PIN_MS1, OUTPUT);
  pinMode(PIN_MS2, OUTPUT);
  pinMode(PIN_MS3, OUTPUT);
  pinMode(PIN_SENSOR, INPUT_PULLUP);
  setStepSize(1);
  sleepMotor(true);
  bluetooth.begin(9600);
  delay(2000);
  bluetooth.println("setup done");
  sendReadyMsg();
}

///////////////////////
//Motor operation
///////////////////////

void loop() {
  if (bluetooth.available()) {
    //input style: "ABCXX,YY"
    //A=direction(0 or 1), B=use millimeters, C=use total Time
    //X=maxStep or millimeters, Y=stepDelay (us) or total Time (s)
    String bluetoothInput = bluetooth.readStringUntil('\n');
    parseInput(bluetoothInput);
  }
  digitalWrite(PIN_MOTOR_DIR, direction);
  if (curStep < maxStep) {
    digitalWrite(PIN_MOTOR_STEP, HIGH);
    delayMicroseconds(halfStepDelayMicro);
    digitalWrite(PIN_MOTOR_STEP, LOW);
    delayMicroseconds(halfStepDelayMicro);
    if (useMilliDelay)
      delay(stepDelayMilli);
    curStep++;
  } else {
    if (running) {
      sendDoneMsg();
      running = false;
      setStepSize(1);
      sleepMotor(true);
      checkBatteryExtra();
    }
    maxStep = 0;
    curStep = 0;
  }
  if (millis()-lastBatteryCheck > 15000) {
    totalVoltage += analogRead(A0);
    batteryCheckCount++;
    lastBatteryCheck = millis();
    if (batteryCheckCount == 4)
      checkBattery();
  }
}

void resetToHome() {
  setStepSize(1);
  sleepMotor(false);
  direction = 2;
  digitalWrite(PIN_MOTOR_DIR, HIGH);
  running = true;
  sendStartMsg();
  while (digitalRead(PIN_SENSOR) == HIGH) {
    digitalWrite(PIN_MOTOR_STEP, HIGH);
    delayMicroseconds(500);
    digitalWrite(PIN_MOTOR_STEP, LOW);
    delayMicroseconds(500);
  }
  digitalWrite(PIN_MOTOR_DIR, LOW);
  for (int i = 0; i<10; i++) {
    digitalWrite(PIN_MOTOR_STEP, HIGH);
    delayMicroseconds(500);
    digitalWrite(PIN_MOTOR_STEP, LOW);
    delayMicroseconds(500);
  }
  running = false;
  direction = 0;
  sendDoneMsg();
  sleepMotor(true);
  checkBatteryExtra();
}

///////////////////////
//Input parsing & applying
///////////////////////

void parseInput(String bluetoothString) {
  bluetoothString.trim();
  if ((bluetoothString.charAt(0) - '0') > 1) {
    resetToHome();
    return;
  }
  direction = (bluetoothString.charAt(0) - '0') != 0;
  bool useMillimeters = (bluetoothString.charAt(1) - '0') != 0;
  bool useTotalTimeInSeconds = (bluetoothString.charAt(1) - '0') != 0;
  if (useMillimeters) {
    maxStep = bluetoothString.substring(3, bluetoothString.indexOf(',')).toInt() * 5;
  } else {
    maxStep = bluetoothString.substring(3, bluetoothString.indexOf(',')).toInt();
  }
  float stepDelay = 0.0;
  if (useTotalTimeInSeconds) {
    float totalTimeInSeconds = bluetoothString.substring(bluetoothString.indexOf(',')+1).toFloat();
    float timePerStep = totalTimeInSeconds / ((float) maxStep);
    stepDelay = timePerStep * 1000000.0;
  } else {
    stepDelay = bluetoothString.substring(bluetoothString.indexOf(',')+1).toFloat();
  }

  if (stepDelay >= 32000) {
    stepDelayMilli = (int) (stepDelay / 1000.0) - 16;
    halfStepDelayMicro = 8000;
  } else {
    stepDelayMilli = 0;
    halfStepDelayMicro = ((int) stepDelay) / 2;
  }
  
  if (halfStepDelayMicro >= 8000) {
    setStepSize(16);
    halfStepDelayMicro /= 16;
    maxStep *= 16;
  } else if (halfStepDelayMicro >= 4000) {
    setStepSize(8);
    halfStepDelayMicro /= 8;
    maxStep *= 8;
  } else if (halfStepDelayMicro >= 2000) {
    setStepSize(4);
    halfStepDelayMicro /= 4;
    maxStep *= 4;
  } else if (halfStepDelayMicro >= 1000) {
    setStepSize(2);
    halfStepDelayMicro /= 2;
    maxStep *= 2;
  } else {
    setStepSize(1);
  }
  
  stepDelayMilli /= stepSize;
  if (halfStepDelayMicro < 500)
      halfStepDelayMicro = 500;
  halfStepDelayMicro -= 5;
  useMilliDelay = stepDelayMilli > 0;
  curStep = 0;
  running = true;
  sleepMotor(false);
  checkBatteryExtra();
  sendStartMsg();
}

///////////////////////
//Motor settings
///////////////////////

void setStepSize(int fraction) {
  stepSize = fraction;
  switch (fraction) {
    case 1:
      digitalWrite(PIN_MS1, LOW);
      digitalWrite(PIN_MS2, LOW);
      digitalWrite(PIN_MS3, LOW);
      break;
    case 2:
      digitalWrite(PIN_MS1, HIGH);
      digitalWrite(PIN_MS2, LOW);
      digitalWrite(PIN_MS3, LOW);
      break;
    case 4:
      digitalWrite(PIN_MS1, LOW);
      digitalWrite(PIN_MS2, HIGH);
      digitalWrite(PIN_MS3, LOW);
      break;
    case 8:
      digitalWrite(PIN_MS1, HIGH);
      digitalWrite(PIN_MS2, HIGH);
      digitalWrite(PIN_MS3, LOW);
      break;
    case 16:
      digitalWrite(PIN_MS1, HIGH);
      digitalWrite(PIN_MS2, HIGH);
      digitalWrite(PIN_MS3, HIGH);
      break;
  }
}

void sleepMotor(bool shouldSleep) {
  if (shouldSleep) {
    digitalWrite(PIN_SLEEP, LOW);
  } else {
    digitalWrite(PIN_SLEEP, HIGH);
  }
  delay(5);
}

///////////////////////
//Battery management
///////////////////////

void checkBattery() {
  const int voltage = totalVoltage / 4;
  sendBatteryMsg(voltage);
  totalVoltage = 0;
  batteryCheckCount = 0;
  if (voltage < 700) {
    sleepMotor(true);
    lowBatteryAlarm();
  }
}

void checkBatteryExtra() {
  totalVoltage = 0;
  batteryCheckCount = 0;
  for (int i=0; i<4; i++) {
    totalVoltage += analogRead(A0);
    delay(100);
  }
  const int voltage = totalVoltage / 4;
  totalVoltage = 0;
  sendBatteryMsg(voltage);
  lastBatteryCheck = millis();
  if (voltage < 700) {
    sleepMotor(true);
    lowBatteryAlarm();
  }
}

void lowBatteryAlarm() {
  sendBatteryLowMsg();
  while (true) {
    tone(PIN_BUZZER, 4000);
    delay(1000);
    noTone(PIN_BUZZER);
    delay(500);
  }
}

///////////////////////
//Bluetooth Messages
///////////////////////

void sendReadyMsg() {
  bluetooth.println("status:READY");
}

void sendDoneMsg() {
  bluetooth.println("status:DONE");
  sendReadyMsg();
}

void sendStartMsg() {
  bluetooth.println("status:RUNNING");
  bluetooth.print("data:");
  bluetooth.print(direction);
  bluetooth.print(",");
  bluetooth.print(stepSize);
  bluetooth.print(",");
  bluetooth.print(stepDelayMilli);
  bluetooth.print(",");
  bluetooth.print(halfStepDelayMicro+5);
  bluetooth.print(",");
  bluetooth.println(maxStep);
}

void sendBatteryMsg(int voltage) {
  bluetooth.print("battery:");
  bluetooth.println(voltage);
}

void sendBatteryLowMsg() {
  bluetooth.println("status:BATT_LOW");
}

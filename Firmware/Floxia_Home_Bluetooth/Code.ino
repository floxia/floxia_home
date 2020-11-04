#include <VarSpeedServo.h> // variable speed servo library.
#include <SoftwareSerial.h> // TX RX software library for bluetooth.

int bluetoothTx = 3; // bluetooth Tx to pin 3.
int bluetoothRx = 2; // bluetooth Rx to pin 2.

SoftwareSerial bluetooth(bluetoothTx, bluetoothRx);

VarSpeedServo leftServo;
VarSpeedServo rightServo;

const int leftServoPin = 9;   // left servo to pin 9.
const int rightServoPin = 10; // right servo Rx to pin 10.

const int openSwitchPin = 6; // open switch to pin 6.
const int closeSwitchPin = 7; // close switch to pin 7.

const int statusLed = 11; // close switch to pin 11.

// Settings

int leftServoAngle = 100;  // initial servo angle degrees 0-180.
int rightServoAngle = 100; // initial servo angle degrees 0-180.

int slowSpeed = 255; // slow speed 0-255.
int fastSpeed = 255; // fast speed 0-255.

int maxServoAngle = 160; // max servo angle degrees 0-180.
int midServoAngle = 80;  // mid servo angle degrees 0-180.
int minServoAngle = 0;   // min servo angle degrees 0-180.

int T1 = 0; // open delay between left and right servos in ms.
int T2 = 0; // close delay between left and right servos in ms

int servoSteps = 2; // servo inching degrees 0-180.

int switchLongPress = 1000; // switch long press in ms.

void setup() {

  Serial.begin(9600); // start serial.

  bluetooth.begin(9600); // start bluetooth serial.

  pinMode(openSwitchPin, INPUT_PULLUP);
  pinMode(closeSwitchPin, INPUT_PULLUP);

  //  leftServo.attach(leftServoPin);
  //  rightServo.attach(rightServoPin);

  leftServo.write(leftServoAngle, slowSpeed);
  rightServo.write(rightServoAngle, slowSpeed);

  Serial.print("LEFT SERVO ANGLE = ");
  Serial.println(leftServoAngle);
  Serial.print("RIGHT SERVO ANGLE = ");
  Serial.println(rightServoAngle);

  delay(1000);

}

void loop() {

  if (digitalRead(openSwitchPin) == HIGH) {
    Serial.println("OPEN");
    Serial.println("");
    unsigned long startOpen = millis();

    while (digitalRead(openSwitchPin) == HIGH) {
      Serial.println("OPEN");
      Serial.println("");

      if (millis() - startOpen < switchLongPress) {
        Serial.println("SHORT PRESS (OPEN)");
        Serial.println("");

        digitalWrite(statusLed, HIGH);

        rightServoAngle += servoSteps;
        leftServoAngle += servoSteps;

        if (rightServoAngle >= maxServoAngle) {
          rightServoAngle = maxServoAngle;

        }

        if (leftServoAngle >= maxServoAngle) {
          leftServoAngle = maxServoAngle;
        }

        rightServo.attach(rightServoPin);
        leftServo.attach(leftServoPin);

        rightServo.write(rightServoAngle, slowSpeed);
        delay(T1);
        leftServo.write(leftServoAngle, slowSpeed);

        digitalWrite(statusLed, LOW);

      }

      else if (millis() - startOpen >= switchLongPress) {
        Serial.println("LONG PRESS (OPEN)");
        Serial.println("");

        rightServoAngle = maxServoAngle;
        leftServoAngle = maxServoAngle;

        rightServo.attach(rightServoPin);
        leftServo.attach(leftServoPin);

        rightServo.write(rightServoAngle, fastSpeed);
        delay(T1);
        leftServo.write(leftServoAngle, fastSpeed);


        ledblink();

        delay(1000);

      }
    }
  }

  if (digitalRead(closeSwitchPin) == HIGH) {
    Serial.println("CLOSE");
    Serial.println("");
    unsigned long startClose = millis();

    while (digitalRead(closeSwitchPin) == HIGH) {
      Serial.println("CLOSE");
      Serial.println("");

      if (millis() - startClose < switchLongPress) {
        Serial.println("SHORT PRESS (CLOSE)");
        Serial.println("");

        digitalWrite(statusLed, HIGH);

        rightServoAngle -= servoSteps;
        leftServoAngle -= servoSteps;

        if (rightServoAngle <= minServoAngle) {
          rightServoAngle = minServoAngle;

        }

        if (leftServoAngle <= minServoAngle) {
          leftServoAngle = minServoAngle;
        }

        rightServo.attach(rightServoPin);
        leftServo.attach(leftServoPin);

        leftServo.write(leftServoAngle, slowSpeed);
        delay(T2);
        rightServo.write(rightServoAngle, slowSpeed);

        digitalWrite(statusLed, LOW);

      }

      else if (millis() - startClose >= switchLongPress) {

        Serial.println("LONG PRESS (CLOSE)");
        Serial.println("");

        leftServoAngle = minServoAngle;
        rightServoAngle = minServoAngle;

        leftServo.attach(leftServoPin);
        rightServo.attach(rightServoPin);

        leftServo.write(leftServoAngle, fastSpeed);
        delay(T2);
        rightServo.write(rightServoAngle, fastSpeed);

        ledblink();

        delay(1000);

      }
    }
  }

  while (bluetooth.available() > 0 ) { // receive number from bluetooth

    int bluetoothValue = bluetooth.read();// save the received number to servopos

    Serial.println(bluetoothValue); // serial print servopos current number received from bluetooth

    int Open;
    int Close;
    int HalfOpen;

    if (bluetoothValue >= 160) {

      Open = 1;
      Close = 0;
      HalfOpen = 0;

      digitalWrite(statusLed, HIGH);

      rightServo.write(maxServoAngle, fastSpeed, true); // roate the servo the angle received from the android app
      delay(T1);
      leftServo.write(maxServoAngle, fastSpeed, true);

      ledblink();

    }

    if (bluetoothValue == 80) while (Open == 1)  {

        HalfOpen = 1;
        Open = 0;
        Close = 0;

        digitalWrite(statusLed, HIGH);

        leftServo.write(midServoAngle, fastSpeed, true); // roate the servo the angle received from the android app
        delay(T2);
        rightServo.write(midServoAngle, fastSpeed, true);

        ledblink();

      }

    if (bluetoothValue == 80) while (Close == 1)  {

        HalfOpen = 1;
        Open = 0;
        Close = 0;

        digitalWrite(statusLed, HIGH);

        rightServo.write(midServoAngle, fastSpeed, true); // roate the servo the angle received from the android app
        delay(T1);
        leftServo.write(midServoAngle, fastSpeed, true);

        ledblink();

      }

    else if (bluetoothValue == 0) {

      Open = 0;
      Close = 1;
      HalfOpen = 0;

      digitalWrite(statusLed, HIGH);

      leftServo.write(minServoAngle, fastSpeed, true); // roate the servo the angle received from the android app
      delay(T2);
      rightServo.write(minServoAngle, fastSpeed, true);

      ledblink();

    }
  }
}

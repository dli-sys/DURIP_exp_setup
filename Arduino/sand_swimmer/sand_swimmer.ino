#include <Servo.h>

// Define servo motor pin
const int servoPin = 9;  // Replace with your servo pin

// Define MOSFET pins for body motors and fin motor
const int bodyFrontMotorPin = 5;  // Replace with your body front motor MOSFET pin
const int bodyBackMotorPin = 6;   // Replace with your body back motor MOSFET pin
const int finMotorPin = 7;        // Replace with your fin motor MOSFET pin

// Create servo object
Servo myservo;

// Servo sweep parameters
const int minDegrees = 60;   // Minimum servo angle
const int maxDegrees = 160;  // Maximum servo angle
const int sweepDelay = 100;   // Delay between servo movements (in milliseconds)

void setup() {
  // Attach servo to pin
  myservo.attach(servoPin);

  // Set MOSFET pins as outputs
  pinMode(bodyFrontMotorPin, OUTPUT);
  pinMode(bodyBackMotorPin, OUTPUT);
  pinMode(finMotorPin, OUTPUT);
}

void loop() {
  // Forward stroke (body front motor ON, others OFF)
  digitalWrite(bodyFrontMotorPin, HIGH);
  digitalWrite(bodyBackMotorPin, HIGH);
  digitalWrite(finMotorPin, LOW);
  for (int pos = maxDegrees; pos >= minDegrees; pos -= 1) {
    myservo.write(pos);
    delay(sweepDelay);
  }

  // reset
  digitalWrite(bodyFrontMotorPin, LOW);
  digitalWrite(bodyBackMotorPin, LOW);
  digitalWrite(finMotorPin, LOW);
  delay(1000);  // Adjust the delay as needed for fin motor activation

  // Backward stroke (body back motor ON, others OFF)
  digitalWrite(bodyFrontMotorPin, LOW);
  digitalWrite(bodyBackMotorPin, LOW);
  digitalWrite(finMotorPin, HIGH);

  for (int pos = minDegrees; pos <= maxDegrees; pos += 1) {
    myservo.write(pos);
    delay(sweepDelay/3);
  }



  // reset
  digitalWrite(bodyFrontMotorPin, LOW);
  digitalWrite(bodyBackMotorPin, LOW);
  digitalWrite(finMotorPin, LOW);
  delay(1000);  // Adjust the delay as needed for fin motor activation
}

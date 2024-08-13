#include <Servo.h>

const int servoPin = 9;  // Pin where the servo is connected
Servo myServo;

void setup() {
  Serial.begin(9600);
  myServo.attach(servoPin);
  myServo.write(90);  // Set initial position to middle
}

void loop() {
  if (Serial.available() > 0) {
    String angleStr = Serial.readStringUntil('\n');
    float angle = angleStr.toFloat();
    
    // Debugging output
    Serial.print("Angle received: ");
    Serial.println(angle);

    // Ensure angle is within valid range
    angle = constrain(angle, 0, 180);

    myServo.write(angle);
    delay(15);  // Allow time for the servo to reach the position
  }
}

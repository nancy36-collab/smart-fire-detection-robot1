// ---------------------------------------------------------
// Smart Fire Detection and Localization System 
// Arduino Control Node for L293D Motor Shield (AFMotor)
// Bluetooth Module: BT-05 | RX -> A5, TX -> A2
// ---------------------------------------------------------

#include <AFMotor.h>
#include <SoftwareSerial.h>

// تعريف الأقطاب حسب توصيلك الفعلي: (Arduino RX = A5, Arduino TX = A2)
SoftwareSerial btSerial(A5, A2); 

// تعريف المحركات الأربعة على الشيلد
AF_DCMotor motorFrontLeft(1);  // M1
AF_DCMotor motorRearLeft(2);   // M2
AF_DCMotor motorFrontRight(3); // M3
AF_DCMotor motorRearRight(4);  // M4

// السرعات الافتراضية
const int BASE_SPEED = 110;
const int TURN_SPEED = 90;

void setup() {
  // بدء الاتصال مع البلوتوث
  btSerial.begin(9600); 

  // إيقاف إلزامي لجميع المحركات عند الإقلاع لمنع أي حركة عشوائية
  stopRobot();
}

void loop() {
  // استقبال الأوامر المرسلة من نظام ROS1 عبر الجسر والبلولوتث
  if (btSerial.available() > 0) {
    char command = btSerial.read();
    
    switch (command) {
      case 'F': // Forward
        moveForward();
        break;
      case 'B': // Backward
        moveBackward();
        break;
      case 'C': // Counter-Clockwise (Turn Left)
        turnLeft();
        break;
      case 'D': // Clockwise (Turn Right)
        turnRight();
        break;
      case 'S': // Stop
        stopRobot();
        break;
      default:
        break;
    }
  }
}

// ---------------------------------------------------------
// دالات التحكم بالحركة لعجلات Mecanum
// ---------------------------------------------------------

void moveForward() {
  setSpeedAll(BASE_SPEED);
  motorFrontLeft.run(FORWARD);
  motorRearLeft.run(FORWARD);
  motorFrontRight.run(FORWARD);
  motorRearRight.run(FORWARD);
}

void moveBackward() {
  setSpeedAll(BASE_SPEED);
  motorFrontLeft.run(BACKWARD);
  motorRearLeft.run(BACKWARD);
  motorFrontRight.run(BACKWARD);
  motorRearRight.run(BACKWARD);
}

void turnLeft() { 
  setSpeedAll(TURN_SPEED);
  motorFrontLeft.run(BACKWARD);
  motorRearLeft.run(BACKWARD);
  motorFrontRight.run(FORWARD);
  motorRearRight.run(FORWARD);
}

void turnRight() { 
  setSpeedAll(TURN_SPEED);
  motorFrontLeft.run(FORWARD);
  motorRearLeft.run(FORWARD);
  motorFrontRight.run(BACKWARD);
  motorRearRight.run(BACKWARD);
}

void stopRobot() {
  motorFrontLeft.run(RELEASE);
  motorRearLeft.run(RELEASE);
  motorFrontRight.run(RELEASE);
  motorRearRight.run(RELEASE);
}

// دالة مساعدة لضبط السرعة لجميع المحركات دفعة واحدة
void setSpeedAll(int speed) {
  motorFrontLeft.setSpeed(speed);
  motorRearLeft.setSpeed(speed);
  motorFrontRight.setSpeed(speed);
  motorRearRight.setSpeed(speed);
}

#include "robotic_arm.h"

/*-----------------------------------机械臂相关-------------------------------------*/
void servoGrap() {
  for (PWM_2 = 2000; PWM_2 > 1500; PWM_2 -= 10) {  // 松开爪子
    servo_2.writeMicroseconds(PWM_2);              // 给舵机写入PWM
    delay(5);
  }
  delay(100);
  for (PWM_1 = 1500; PWM_1 < 2500; PWM_1 += 10) {  // 放倒机械臂
    servo_1.writeMicroseconds(PWM_1);              // 给舵机写入PWM
    delay(5);
  }
  delay(100);
  for (PWM_2 = 1500; PWM_2 < 1960; PWM_2 += 10) {  // 夹紧爪子
    servo_2.writeMicroseconds(PWM_2);              // 给舵机写入PWM
    delay(5);
  }
  delay(100);
  for (PWM_1 = 2500; PWM_1 > 1600; PWM_1 -= 10) {  // 立起机械臂
    servo_1.writeMicroseconds(PWM_1);              // 给舵机写入PWM
    delay(5);
  }
}

void servoDrop() {
  for (PWM_1 = 1600; PWM_1 < 2500; PWM_1 += 10) {  // 放倒机械臂
    servo_1.writeMicroseconds(PWM_1);              // 给舵机写入PWM
    delay(5);
  }
  delay(200);
  for (PWM_2 = 1960; PWM_2 > 1500; PWM_2 -= 10) {  // 松开爪子
    servo_2.writeMicroseconds(PWM_2);              // 给舵机写入PWM
    delay(5);
  }
  delay(100);
  for (PWM_1 = 2500; PWM_1 > 1500; PWM_1 -= 10) {  // 立起机械臂
    servo_1.writeMicroseconds(PWM_1);              // 给舵机写入PWM
    delay(5);
  }
  delay(100);
  for (PWM_2 = 1500; PWM_2 < 2000; PWM_2 += 10) {  // 夹紧爪子
    servo_2.writeMicroseconds(PWM_2);              // 给舵机写入PWM
    delay(5);
  }
}
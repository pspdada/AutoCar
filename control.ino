#include "control.h"
#include "AutoCar_def.h"

// 根据红外传感器的输出来决定小车运动模式
uint8_t runMode(void) {
  while (true) {
    updateCTRTstate();   // 更新CTRT数据
    if (isCross == 0) {  // 根据红外传感器的输出来决定运动模式
      if (isAllLow())    // 未检测到黑线时，进入记忆模式
      {
        return STRAIGHT_ON;  // 调试用
        // return REVERSE;  // 调试用
        // isCross = 1;
      } else if (isAllHigh())  // 全为黑线时停车
      {
        return _STOP;
      } else if (CTRTstate[0][0] == HIGH && CTRTstate[6][0] == LOW)  //直角左转
      {
        return TURN_LEFT_HIGH;
      } else if (CTRTstate[6][0] == HIGH && CTRTstate[0][0] == LOW)  //直角右转
      {
        return TURN_RIGHT_HIGH;
      } else if (CTRTstate[1][0] == HIGH)  //中左转
      {
        return TURN_LEFT_MID;
      } else if (CTRTstate[5][0] == HIGH)  //中右转
      {
        return TURN_RIGHT_MID;
      } else if (CTRTstate[2][0] == HIGH)  //低左转
      {
        return TURN_LEFT_LOW;
      } else if (CTRTstate[4][0] == HIGH)  //低右转
      {
        return TURN_RIGHT_LOW;
      } else if (CTRTstate[3][0] == HIGH)  //直走
      {
        return STRAIGHT_ON;
      } else  //直走
      {
        return STRAIGHT_ON;
      }
    }
    if (isCross == 1) {  // 记忆模式
      if (!isAllLow())   // 再次检测到黑线，则退出记忆模式
      {
        isCross = 0;
        continue;
      } else if ((CTRTstate[0][1] == HIGH || CTRTstate[0][2] == HIGH) && (CTRTstate[6][1] == LOW || CTRTstate[6][2] == LOW))  // 直角左转
      {
        return TURN_LEFT_HIGH;
      } else if ((CTRTstate[6][1] == HIGH || CTRTstate[6][2] == HIGH) && (CTRTstate[0][1] == LOW || CTRTstate[0][2] == LOW))  // 直角右转
      {
        return TURN_RIGHT_HIGH;
      } else if (CTRTstate[1][1] == HIGH || CTRTstate[1][2] == HIGH)  // 中左转
      {
        return TURN_LEFT_MID;
      } else if (CTRTstate[5][1] == HIGH || CTRTstate[5][2] == HIGH)  // 中右转
      {
        return TURN_RIGHT_MID;
      } else if (CTRTstate[2][1] == HIGH || CTRTstate[2][2] == HIGH)  // 低左转
      {
        return TURN_LEFT_LOW;
      } else if (CTRTstate[4][1] == HIGH || CTRTstate[4][2] == HIGH)  // 低右转
      {
        return TURN_RIGHT_LOW;
      } else {
        continue;
      }
    }
  }
}

// 根据小车运动模式选择小车两轮的目标速度
void motorControl(void) {
  run_mode mode = runMode();

  switch (mode) {
    case _STOP:  // 停转
      TARGET_V_LEFT = 0;
      TARGET_V_RIGHT = 0;
      break;
    case STRAIGHT_ON:  // 全速直行
      TARGET_V_LEFT = -base_V;
      TARGET_V_RIGHT = base_V;
      break;
    case REVERSE:  // 倒车
      TARGET_V_LEFT = +base_V * 0.3;
      TARGET_V_RIGHT = -base_V * 0.3;
      break;
    case TURN_LEFT_LOW:  // 低左转
      TARGET_V_LEFT = -base_V * 0.5;
      TARGET_V_RIGHT = base_V * 0.9;
      break;
    case TURN_LEFT_MID:  // 中左转
      TARGET_V_LEFT = -base_V * 0.15;
      TARGET_V_RIGHT = base_V * 0.75;
      break;
    case TURN_LEFT_HIGH:  // 直角左转
      TARGET_V_LEFT = 0;
      TARGET_V_RIGHT = base_V * 0.65;
      break;
    case TURN_RIGHT_LOW:  // 低右转
      TARGET_V_LEFT = -base_V * 0.9;
      TARGET_V_RIGHT = base_V * 0.5;
      break;
    case TURN_RIGHT_MID:  // 中右转
      TARGET_V_LEFT = -base_V * 0.75;
      TARGET_V_RIGHT = base_V * 0.15;
      break;
    case TURN_RIGHT_HIGH:  // 直角右转
      TARGET_V_LEFT = -base_V * 0.65;
      TARGET_V_RIGHT = 0;
    default:  // 停车
      TARGET_V_LEFT = 0;
      TARGET_V_RIGHT = 0;
      break;
  }

  // 平滑输出
  TARGET_V_LEFT = TARGET_V_LEFT * 0.5 + f_V_L * 0.3 + ff_V_L * 0.2;
  TARGET_V_RIGHT = TARGET_V_RIGHT * 0.5 + f_V_R * 0.3 + ff_V_R * 0.2;

  // 计算轮子转动速度（弧度制的速度，2π Rn）
  cur_V_LEFT = (encoderVal_LEFT / 780.0) * PI * 2 * (1000 / PERIOD);
  cur_V_RIGHT = (encoderVal_RIGHT / 780.0) * PI * 2 * (1000 / PERIOD);

  encoderVal_RIGHT = 0;
  encoderVal_LEFT = 0;

  carRun();

  // 储存速度
  ff_V_L = f_V_L;
  f_V_L = TARGET_V_LEFT;
  ff_V_R = f_V_R;
  f_V_R = TARGET_V_RIGHT;
}

// 根据小车两轮的目标速度来输出PWM波控制小车
void carRun(void) {
  Output_L = pidController_L(TARGET_V_LEFT, cur_V_LEFT);
  if (TARGET_V_LEFT == 0) {
    digitalWrite(IN_L1, HIGH);
    digitalWrite(IN_L2, HIGH);    
  } else {
    if (Output_L > 0) {
      digitalWrite(IN_L1, LOW);
      digitalWrite(IN_L2, HIGH);
      analogWrite(PWM_LEFT, Output_L);
    } else {
      digitalWrite(IN_L1, HIGH);
      digitalWrite(IN_L2, LOW);
      analogWrite(PWM_LEFT, abs(Output_L));
    }
  }
  Output_R = pidController_R(TARGET_V_RIGHT, cur_V_RIGHT);
  if (TARGET_V_RIGHT == 0) {
    digitalWrite(IN_R1, HIGH);
    digitalWrite(IN_R2, HIGH);
  } else {
    if (Output_R > 0) {
      digitalWrite(IN_R1, LOW);
      digitalWrite(IN_R2, HIGH);
      analogWrite(PWM_RIGHT, Output_R);
    } else {
      digitalWrite(IN_R1, HIGH);
      digitalWrite(IN_R2, LOW);
      analogWrite(PWM_RIGHT, abs(Output_R));
    }
  }
}
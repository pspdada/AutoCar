// 220320726 彭尚品 220320723 黄艺洋
#include "AutoCar_def.h"

//------------------------------------------全局变量--------------------------------------------*/
float TARGET_V_LEFT = 0, f_V_L = 0, ff_V_L = 0;   // 左轮目标速度 当前、上一时刻、上上时刻
float TARGET_V_RIGHT = 0, f_V_R = 0, ff_V_R = 0;  // 右轮目标速度 当前、上一时刻、上上时刻
volatile long encoderVal_LEFT = 0;                // 左轮编码器值
volatile long encoderVal_RIGHT = 0;               // 右轮编码器值
float cur_V_LEFT = 0;                             // 左轮实际转速
float cur_V_RIGHT = 0;                            // 右轮实际转速
short T = PERIOD;                                 // 周期
short Output_R = 0, Output_L = 0;

float u_L = 0, LEFT_eI = 0, LEFT_eII = 0, LEFT_eIII = 0;  // 输出量u、e(k)、e(k-1)、e(k-2)
float u_R = 0, RIGHT_eI = 0, RIGHT_eII = 0, RIGHT_eIII = 0;

// 将CTRT读取的数据存入二维数组中，实现记忆功能
bool CTRTstate[CTRT_CNT][MEMORY_CNT];
bool isCross = 0;  // 判断是否越线，0代表没有越界，正常读取正常输出，1代表越界，进入记忆模式

// 创建舵机对象
Servo servo_1;
Servo servo_2;

// 变量pwm用来存储舵机角度位置
unsigned short PWM_1 = 1500, PWM_2 = 2000;
void pinModeInit() {
  pinMode(ENCODER_LEFT_A, INPUT);
  pinMode(ENCODER_LEFT_B, INPUT);
  pinMode(ENCODER_RIGHT_A, INPUT);
  pinMode(ENCODER_RIGHT_B, INPUT);
  pinMode(PWM_LEFT, OUTPUT);
  pinMode(IN_L1, OUTPUT);
  pinMode(IN_L2, OUTPUT);
  pinMode(PWM_RIGHT, OUTPUT);
  pinMode(IN_R1, OUTPUT);
  pinMode(IN_R2, OUTPUT);
  pinMode(CTRT_PIN_R1, INPUT);
  pinMode(CTRT_PIN_R2, INPUT);
  pinMode(CTRT_PIN_R3, INPUT);
  pinMode(CTRT_PIN_L1, INPUT);
  pinMode(CTRT_PIN_L2, INPUT);
  pinMode(CTRT_PIN_L3, INPUT);
  pinMode(CTRT_PIN_M, INPUT);
}

/*---------------------------------------主函数---------------------------------------*/
void setup() {
  pinModeInit();
  TCCR1B = TCCR1B & B11111000 | B00000001;                          // 9,10 两个管脚的 PWM 由定时器 TIMER1 产生，这句程序改变 PWM 的频率
  attachInterrupt(digitalPinToInterrupt(2), getEncoder_L, CHANGE);  // 设置2引脚为中断
  attachInterrupt(digitalPinToInterrupt(3), getEncoder_R, CHANGE);  // 设置3引脚为中断
  // attachInterrupt(digitalPinToInterrupt(21), getEncoder_R, CHANGE);// 设置21引脚为中断剩余可用于中断的引脚：18 19 20
  Serial.begin(9600);
  memset(CTRTstate, 0, sizeof(bool) * CTRT_CNT * MEMORY_CNT);
  /*
  servo_1.attach(SERVO_1);  // 将20引脚与声明的舵机对象连接起来
  servo_2.attach(SERVO_2);  // 将21引脚与声明的舵机对象连接起来
  servo_1.writeMicroseconds(PWM_1);
  servo_2.writeMicroseconds(PWM_2);
  servoGrap();
  servoDrop();
  */
  MsTimer2::set(PERIOD, motorControl);  // 设定每隔PERIOD时间，执行一次motorControl函数
  MsTimer2::start();
}
uint16_t count_1 = 0;
void loop() {
  count_1++;
  if (count_1 == 1000) {
    /*
   Serial.print("Output_L:");
  Serial.println(Output_L);
  Serial.print("Output_R:");
  Serial.println(Output_R);
  */
    /*
    Serial.print("cur_V_LEFT:");
    Serial.println(cur_V_LEFT);
    Serial.print("cur_V_RIGHT:");
    Serial.println(cur_V_RIGHT);
    Serial.print("TARGET_V_LEFT:");
    Serial.println(TARGET_V_LEFT);
    Serial.print("TARGET_V_RIGHT:");
    Serial.println(TARGET_V_RIGHT);
    Serial.print("\n");
*/

    Serial.print(CTRTstate[0][0]);
    Serial.print("\t");
    Serial.print(CTRTstate[1][0]);
    Serial.print("\t");
    Serial.print(CTRTstate[2][0]);
    Serial.print("\t");
    Serial.print(CTRTstate[3][0]);
    Serial.print("\t");
    Serial.print(CTRTstate[4][0]);
    Serial.print("\t");
    Serial.print(CTRTstate[5][0]);
    Serial.print("\t");
    Serial.print(CTRTstate[6][0]);
    Serial.print("\n");

    count_1 = 0;
  }
}
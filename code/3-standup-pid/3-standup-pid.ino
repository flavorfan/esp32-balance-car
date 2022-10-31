/* =====2022.9.9==================================
   ====小车器件清单==================================
   控制器：esp32
   马达：GA25-370减速电机 ，转速大约280rmp。
   马达驱动模块：DRV8833
   陀螺仪模块：MPU6050
   ====马达测试参数 6v电压供给下=======================
   pwm+为前进，pwm+-为后退，均测试
   左马达IN1 IN2, OUT1 OUT2 , PWM35起转。（mpu6050接口侧）
   右马达IN3 IN4，OUT3 OUT4 , PWM39起转。
   ====陀螺仪测试参数================================
   小车静态平衡时的角度为-1.4度。
   ====PID调试结果==================================
  P  I   D  调试结果1  P:18      I: 0.6      D:0.8
  P  I   D  调试结果2  P:      I:       D:
  ------------------------------------------------------
*/

#include "BluetoothSerial.h"
#include <MPU6050_tockn.h>
#include <Wire.h>

#define IN1 26
#define IN2 27
#define IN3 32
#define IN4 33

float Balance_Angle_raw = 1.2;                              //静态机械平衡角度。
const int leftMotorPwmOffset = 72, rightMotorPwmOffset = 72; //左右轮的启动pwm值，pwm达到一定电压马达才开始转动。

int z_turn_spd = 60;                                         //转弯的幅度，通过两轮的pwm差值来实现转向。
float ENERGY = 6;                                            //前进后退倾角，控制前进后退速度。
float kp = 0, ki = 0, kd = 0;                           //根据调试设置kp ki kd的默认值
//float kp = 18, ki = 0.6, kd = 0.8;                           //根据调试设置kp ki kd的默认值
float angle_kp = -2;                                          //根据调试设置kp ki kd的默认值

float Keep_Angle, bias, integrate;                 //保持角度，角度偏差，偏差积分变量
float AngleX, AngleY, AngleZ, GyroX, GyroY, GyroZ; // mpu6050输出的角度值为浮点数，两位有效小数
int vertical_PWM, angle_PWM, PWM, L_PWM, R_PWM;    //各种PWM计算值

char flag = 's';

int sda_pin = 13, scl_pin = 15;

int debug = 0;

BluetoothSerial SerialBT;
MPU6050 mpu6050(Wire); //实例化mpu对象

void set_pin()
{
  int outpin[4] = {IN1, IN2, IN3, IN4};
  for (int i = 0; i < 4; i++)
  {
    pinMode(outpin[i], OUTPUT);
  }
}

void motor(int left_EN, int right_EN)
{
  left_EN = constrain(left_EN,-255,255);
  right_EN = constrain(right_EN,-255,255);
  if (left_EN == 0)
  {
    analogWrite(IN1, 0);
    analogWrite(IN2, 0);
  }
  if (left_EN < 0)
  {

    analogWrite(IN1, 0);
    analogWrite(IN2, 0 - left_EN);
  }
  if (left_EN > 0)
  {
    analogWrite(IN1, left_EN);
    analogWrite(IN2, 0);
  }
  if (right_EN == 0)
  {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0);
  }
  if (right_EN < 0)
  {
    analogWrite(IN3, 0);
    analogWrite(IN4, 0 - right_EN);
  }
  if (right_EN > 0)
  {
    analogWrite(IN3, right_EN);
    analogWrite(IN4, 0);
  }
}

void serial_debug() //串口调试和控制程序
{
  if (SerialBT.available() > 0)
  {
    char DATA = SerialBT.read();
    delay(5);
    switch (DATA)
    {
    /*=========调试==测定小车静态平衡角度值，和调试合适的P I D参数-------*/

    /*机械平衡点调整*/
    case 'u':
      Keep_Angle += 0.1;
      break;
    case 'd':
      Keep_Angle -= 0.1;
      break; //调节物理平衡点,a前倾，b后仰

    /*直立环调试*/
    case '0':
      kp -= 1;
      break;
    case '1':
      kp += 1;
      break; //调节直立环比例项-+
    case '2':
      ki -= 0.1;
      break;
    case '3':
      ki += 0.1;
      break; //调节直立环积分项-+ki取值范围由限定范围确定。
    case '4':
      kd -= 0.1;
      break;
    case '5':
      kd += 0.1;
      break; //调节直立环微分项-+
    case '6':
      angle_kp -= 0.2;
      break;
    case '7':
      angle_kp += 0.2;
      break; //调节转向环比例项，取值范围由限定范围确定。

    /*控制程序*/
    case 's': //停车
      flag = 's';
      Keep_Angle = Balance_Angle_raw;
      z_turn_spd = 0;
      break;  //调节物理平衡点为机械平衡角度值，原地平衡
    case 'a': //前进
      flag = 'a';
      Keep_Angle = Balance_Angle_raw + ENERGY;
      z_turn_spd = 0;
      break;
    case 'b': //后退
      flag = 'b';
      Keep_Angle = Balance_Angle_raw - ENERGY;
      z_turn_spd = 0;
      break;
    case 'l': //左转
      flag = 'l';
      z_turn_spd = 60;
      break;
    case 'r': //右转
      flag = 'r';
      z_turn_spd = -60;
      break;

    case 'z': //debug
      debug = ! debug;
      break;
    }

    if (kp < 0)
      kp = 0;
    if (ki < 0)
      ki = 0;
    if (kd < 0)
      kd = 0;

    SerialBT.print("Keep_Angle: ");    SerialBT.println(Keep_Angle);
    SerialBT.print("kp:");    SerialBT.print(kp);
    SerialBT.print("  ki:");    SerialBT.print(ki);
    SerialBT.print("  kd:");    SerialBT.println(kd);
    SerialBT.print("  angle_kp:");   SerialBT.println(angle_kp);
    // SerialBT.println("--------------------");
  }
}

//void angle_pwm_calculation()
//{
//  //  AngleZ = mpu6050.getAngleZ(); //获取陀螺仪角度
//  GyroZ = mpu6050.getGyroZ(); //获取陀螺仪角速度
//  angle_PWM = angle_kp * (GyroZ - z_turn_spd);
//  angle_PWM = constrain(angle_PWM, -130, 130);
//}

void angle_pwm_calculation()
{
  //  AngleZ = mpu6050.getAngleZ(); //获取陀螺仪角度
  GyroZ = mpu6050.getGyroZ(); //获取陀螺仪角速度
  angle_PWM = angle_kp * (GyroZ - z_turn_spd);
  angle_PWM = constrain(angle_PWM, -130, 130);
}


void vertical_pwm_calculation()
{
  AngleX = mpu6050.getAngleX();
  GyroX = mpu6050.getGyroX();

  bias = AngleX - Keep_Angle; // 计算角度偏差。bias为小车角度是静态平衡角度的差值。
  integrate += bias; //偏差的积分，integrate为全局变量，一直积累。
  integrate = constrain(integrate, -1000, 1000); //限定误差积分的最大和最小值

  /*==直立PID计算PWM==通过陀螺仪返回数据计算，前倾陀螺仪Y轴为正，后仰陀螺仪Y轴为负。
    前倾车前进，后仰车后退，保持直立。但可能为了直立，车会随时移动。*/
  vertical_PWM = kp * bias + ki * integrate + kd * GyroX;
}

void motor_control()
{
  /*---【补偿右轮pwm差值】------------*/
  if (PWM > 0)
  {
    L_PWM = PWM + leftMotorPwmOffset;  //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
    R_PWM = PWM + rightMotorPwmOffset; //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
  }
  if (PWM < 0)
  {
    L_PWM = PWM - leftMotorPwmOffset;  //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
    R_PWM = PWM - rightMotorPwmOffset; //左右轮的启动pwm达到一定电压马达才开始转动。这里为补偿值。
  }
  if (PWM == 0)
  {
    L_PWM = 0; //
    R_PWM = 0; //
  }

//  /*---【转向判断】-------------*/
  if (flag == 's')
  {
    L_PWM = L_PWM;
    R_PWM = R_PWM;
    // flag == s,使小车保持原地直立stop
  }
  if (flag == 'l') // flag l，left控制两轮的pwm差值，通过左轮减速，右轮加速。使其左转。
  {
    L_PWM += angle_PWM;
    R_PWM -= angle_PWM;
  }
  if (flag == 'r')
  { // flag  r，控制两轮的pwm差值，通过左轮加速，右轮减速。使其右转
    L_PWM += angle_PWM;
    R_PWM -= angle_PWM;
  }

  /*---【控制马达输出】-------------*/
  L_PWM = constrain(L_PWM, -255, 255); //计算出来的PWM限定大小。255为输出上限。
  R_PWM = constrain(R_PWM, -255, 255);

  motor(L_PWM, R_PWM);

  /*--------判断是否小车倒下，此时停止马达和编码器计数-----*/
  if (AngleX > 45 || AngleX < -45) //倾角过大（车倒下时），停止马达输出
  {
    motor(0, 0);
  }
}

void setup()
{

  //Serial.begin(115200);
  SerialBT.begin("ESP32car"); // Bluetooth device name
  Wire.begin(sda_pin, scl_pin);
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  //  mpu6050.setGyroOffsets(*, *, *);
  Keep_Angle = Balance_Angle_raw; //平衡角度初始化为静态平衡时的陀螺仪角度。Keep_Angle可以改变，才可以控制前进后退。
  motor(0, 0);                    //机器启动时马达确保停止。
  delay(50);                      //循环前延时，确保各种初始和准备完成
}

void loop()
{
  /*====串口PID调试+控制===*/
  serial_debug();

  /*====陀螺仪刷新===*/
  mpu6050.update();

  /*====PWM计算====*/
  angle_pwm_calculation();    //转向环计算pwm值
  vertical_pwm_calculation(); //直立环PWM计算

  PWM = vertical_PWM;

  /*====马达输出=====*/
  motor_control();

  if (debug)
  // AngleX
    SerialBT.printf("#%.3f %.3f %.3f\n",bias, integrate, GyroX);

  delay(0.5);
}

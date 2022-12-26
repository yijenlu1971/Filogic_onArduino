#include <MPU6050.h>    //MPU6050的库文件
#include <Wire.h>       //I2C通讯库文件
#include "LTimer.h"

#define PWM_FREQUENCY     20000

MPU6050 mpu6050;     //实例化一个MPU6050对象，对象名称为mpu6050
int16_t ax, ay, az, gx, gy, gz;     //定义三轴加速度，三轴陀螺仪的变量

LTimer timer0(LTIMER_0);
LTimer timer1(LTIMER_1);

//TB6612引脚定义
const int right_R1 = 11;    
const int right_R2 = 8;
const int PWM_R = 36;
const int left_L1 = 9;
const int left_L2 = 10;
const int PWM_L = 35;

///////////////////////角度参数//////////////////////////////
//float angle_X; //由加速度计算关于X轴的倾斜角度变量
//float angle_Y; //由加速度计算关于Y轴的倾斜角度变量
float angle0 = 1; //实际测量出的角度（理想时是0度）
float Gyro_x, Gyro_y, Gyro_z;  //用于陀螺仪算出的各轴角速度
///////////////////////角度参数//////////////////////////////

///////////////////////Kalman_Filter////////////////////////////
float Q_angle = 0.001;  //陀螺仪噪声的协方差
float Q_gyro = 0.003;   //陀螺仪漂移噪声的协方差
float R_angle = 0.5;    //加速度计的协方差
char C_0 = 1;
float dt = 0.005; //dt的取值为滤波器采样时间
float K1 = 0.05;  //含有卡尔曼增益的函数，用于计算最优估计值的偏差
float K_0, K_1, t_0, t_1;
float angle_err;
float q_bias;    //陀螺仪漂移

float angle;
float angle_speed;

float Pdot[4] = { 0, 0, 0, 0};
float P[2][2] = {{ 1, 0 }, { 0, 1 }};
float PCt_0, PCt_1, E;
//////////////////////Kalman_Filter/////////////////////////

//////////////////////PID参数///////////////////////////////
double kp = 34, ki = 0, kd = 0.62;                      //角度环参数
double kp_speed = 3.6, ki_speed = 0.080, kd_speed = 0;  //速度环参数
double setp0 = 0; //角度平衡点
int PD_pwm;  //角度输出
float pwm1 = 0, pwm2 = 0;

//////////////////中断测速计数/////////////////////////////
#define PinA_left   15  //外部中断
#define PinA_right  37  //外部中断
volatile long count_right = 0;//用于计算霍尔编码器计算的脉冲值(volatile long类型是为了确保数值有效）
volatile long count_left = 0;

//////////////////////脉冲计算/////////////////////////
int lz = 0;
int rz = 0;
int rpluse = 0;
int lpluse = 0;
int pulseright, pulseleft;
////////////////////////////////PI变量参数//////////////////////////
float speeds_filterold = 0;
float positions = 0;
double PI_pwm;
int cc;
int speedout;
float speeds_filter;

void setup() 
{
  //设置控制电机的引脚为输出状态
  pinMode(right_R1, OUTPUT);       
  pinMode(right_R2, OUTPUT);
  pinMode(left_L1, OUTPUT);
  pinMode(left_L2, OUTPUT);
  pinMode(PWM_R, OUTPUT);
  pinMode(PWM_L, OUTPUT);

  //赋初始状态值
  digitalWrite(right_R1, 1);
  digitalWrite(right_R2, 0);
  digitalWrite(left_L1,  0);
  digitalWrite(left_L2, 1);
  analogWrite(PWM_R, (1000*1000)/PWM_FREQUENCY, 0);
  analogWrite(PWM_L, (1000*1000)/PWM_FREQUENCY, 0);

  pinMode(PinA_left, INPUT);  //测速码盘输入
  pinMode(PinA_right, INPUT);

  //外部中断，用于计算车轮转速
  attachInterruptWithDebounce(PinA_left, Code_left, CHANGE, 0);  //PinA_left引脚的电平发生改变触发外部中断，执行子函数 Code_left
  attachInterruptWithDebounce(PinA_right, Code_right, CHANGE, 0); //PinA_right引脚的电平发生改变触发外部中断，执行子函数 Code_right

  Wire.begin();         //加入 I2C 总线序列
  Serial.begin(115200); //开启串口，设置波特率为115200
  delay(500);
  mpu6050.initialize(); //初始化MPU6050
  delay(2);

  // initialization
  timer0.begin();
  timer1.begin();

  //5ms定时中断设置是使用timer0 (注意：使用timer0会对pin3 pin11的PWM输出有影响.)
  timer0.start(5, LTIMER_REPEAT_MODE, _callback0, NULL);
  timer1.start(500, LTIMER_REPEAT_MODE, _callback1, NULL);
}

void loop() {
}

void _callback1(void *usr_data)
{
  Serial.println(angle);
  //Serial.println(PD_pwm);
  //Serial.println(pwm1);
  //Serial.println(pwm2);
  //Serial.print("pulseright = ");
  //Serial.println(pulseright);
  //Serial.print("pulseleft = ");
  //Serial.println(pulseleft);
  //Serial.println(PI_pwm);
  //Serial.println(speeds_filter);
  //Serial.println (positions);
}

/////////////////////霍尔计数/////////////////////////
//左测速码盘计数
void Code_left() 
{
  count_left ++;
} 
//右测速码盘计数
void Code_right() 
{
  count_right ++;
} 
////////////////////脉冲计算///////////////////////
void countpluse()
{
  lz = count_left;     //将码盘计到的值赋给lz
  rz = count_right;

  count_left = 0;     //将码盘计数量清零
  count_right = 0;

  lpluse = lz;
  rpluse = rz;

  if ((pwm1 < 0) && (pwm2 < 0)) //判断平衡车小车运动方向 如果后退时（PWM即电机电压为负） 脉冲数为负数
  {
    rpluse = -rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 > 0))  //如果后退时（PWM即电机电压为正） 脉冲数为正数
  {
    rpluse = rpluse;
    lpluse = lpluse;
  }
  else if ((pwm1 < 0) && (pwm2 > 0))  //小车运动方向判断 左旋转 右脉冲数为正数 左脉冲数为负数
  {
    rpluse = rpluse;
    lpluse = -lpluse;
  }
  else if ((pwm1 > 0) && (pwm2 < 0))  //小车运动方向判断 右旋转 右脉冲数为负数 左脉冲数为正数
  {
    rpluse = -rpluse;
    lpluse = lpluse;
  }

  //每5ms进入中断时，脉冲数叠加
  pulseright += rpluse;
  pulseleft += lpluse;
}

/////////////////////////////////中断////////////////////////////
void _callback0(void *usr_data)
{
  countpluse(); //脉冲叠加子函数
  mpu6050.getMotion6(&ax, &ay, &az, &gx, &gy, &gz); //IIC获取MPU6050六轴数据 ax ay az gx gy gz
  angle_calculate(ax, ay, az, gx, gy, gz, 
          dt, Q_angle, Q_gyro, R_angle, C_0, K1); //获取angle 角度和卡曼滤波
  PD(); //角度环 PD控制
  anglePWM();

  if( ++cc >= 8 ) //5*8=40，40ms进入一次速度的PI算法
  {
    speedpiout();   
    cc = 0;
  }
}
///////////////////////////////////////////////////////////

/////////////////////////////倾角计算///////////////////////
void angle_calculate(int16_t ax, int16_t ay, int16_t az,
    int16_t gx, int16_t gy, int16_t gz,
    float dt, float Q_angle, float Q_gyro, float R_angle, float C_0, float K1)
{
  float Angle = -atan2(ay, az) * (180/ PI);  //車輪轉動的角度計算公式, 负号为方向处理
  Gyro_x = -gx / 131;           //車軸的角速度, 负号为方向处理
  Kalman_Filter(Angle, Gyro_x); //卡曼滤波
  Gyro_z = -gz / 131;           //垂直轴線的角速度
}
////////////////////////////////////////////////////////////////

///////////////////////////////KalmanFilter/////////////////////
void Kalman_Filter(double angle_m, double gyro_m)
{
  angle += (gyro_m - q_bias) * dt;          //先验估计
  angle_err = angle_m - angle;
  
  Pdot[0] = Q_angle - P[0][1] - P[1][0];    //先验估计误差协方差的微分
  Pdot[1] = - P[1][1];
  Pdot[2] = - P[1][1];
  Pdot[3] = Q_gyro;
  
  P[0][0] += Pdot[0] * dt;    //先验估计误差协方差微分的积分
  P[0][1] += Pdot[1] * dt;
  P[1][0] += Pdot[2] * dt;
  P[1][1] += Pdot[3] * dt;
  
  //矩阵乘法的中间变量
  PCt_0 = C_0 * P[0][0];
  PCt_1 = C_0 * P[1][0];
  //分母
  E = R_angle + C_0 * PCt_0;
  //增益值
  K_0 = PCt_0 / E;
  K_1 = PCt_1 / E;
  
  t_0 = PCt_0;  //矩阵乘法的中间变量
  t_1 = C_0 * P[0][1];
  
  P[0][0] -= K_0 * t_0;    //后验估计误差协方差
  P[0][1] -= K_0 * t_1;
  P[1][0] -= K_1 * t_0;
  P[1][1] -= K_1 * t_1;
  
  q_bias += K_1 * angle_err;    //后验估计
  angle_speed = gyro_m - q_bias;   //输出值的微分，得出最优角速度
  angle += K_0 * angle_err; ////后验估计，得出最优角度
}

//////////////////角度PD////////////////////
void PD()
{
  PD_pwm = kp * (angle + angle0) + kd * angle_speed; //PD 角度环控制
}

//////////////////速度PI////////////////////
void speedpiout()
{
  float speeds = (pulseleft + pulseright) * 1.0;      //车速 脉冲值
  pulseright = pulseleft = 0;      //清零
  speeds_filterold *= 0.7;         //一阶互补滤波
  speeds_filter = speeds_filterold + speeds * 0.3;
  speeds_filterold = speeds_filter;
  positions += speeds_filter;
  positions = constrain(positions, -3550, 3550);  //抗积分饱和
  PI_pwm = ki_speed * (setp0 - positions) + kp_speed * (setp0 - speeds_filter);      //速度环控制 PI
}
//////////////////速度PI////////////////////


////////////////////////////PWM终值/////////////////////////////
void anglePWM()
{
  pwm2 = -PD_pwm - PI_pwm ; //赋给电机PWM的最终值
  pwm1 = -PD_pwm - PI_pwm ;

  //限定PWM值不能超过255
  if( pwm1 > 255 )  pwm1 = 255;
  if( pwm1 < -255 ) pwm1 = -255;
  if( pwm2 > 255 )  pwm2 = 255;
  if( pwm2 <- 255 ) pwm2=-255;

  //自平衡小车倾斜角度大于45度，电机就会停转
  if( angle > 80 || angle < -80 ) pwm1 = pwm2 = 0;

  //根据PWM的正负来确定电机的转向和转速
  if( pwm2 >= 0 )
  {
    digitalWrite(left_L1, LOW);
    digitalWrite(left_L2, HIGH);
    analogWrite(PWM_L, (1000*1000)/PWM_FREQUENCY, pwm2);
  }
  else
  {
    digitalWrite(left_L1, HIGH);
    digitalWrite(left_L2, LOW);
    analogWrite(PWM_L, (1000*1000)/PWM_FREQUENCY, -pwm2);
  }

  if( pwm1 >= 0 )
  {
    digitalWrite(right_R1, LOW);
    digitalWrite(right_R2, HIGH);
    analogWrite(PWM_R, (1000*1000)/PWM_FREQUENCY, pwm1);
  }
  else
  {
    digitalWrite(right_R1, HIGH);
    digitalWrite(right_R2, LOW);
    analogWrite(PWM_R, (1000*1000)/PWM_FREQUENCY, -pwm1);
  }
}

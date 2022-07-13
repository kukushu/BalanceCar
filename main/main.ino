#include <Wire.h>
#include <MsTimer2.h>
#include <PinChangeInterrupt.h>
//pwm控制小车转速
#define Left_IN1 9
#define Left_IN2 10
#define Right_IN1 5
#define Right_IN2 6

//编码器
#define encode_left_A 8
#define encode_left_B 4
#define encode_right_A 7
#define encode_right_B 2

float filter_a = 0.2;
long begin_ = 0;
float zhongzhi = 3;

int PWM = 0;

float balance_Kp = 30;
float balance_Kd = -5;
float speed_Kp = 0;
float speed_Ki = 0;


float speed_left = 0;
long count_left = 0;
float speed_right = 0;
long count_right = 0;
float new_angle = 0;
float old_angle = 0;
float Pro_angle = 0;
float Acc_angle = 0;
float balance_angle = 0;
float bias = 0;
float speed_angle = 0;
float speed_angle_filter = 0;


int dt = 0;
const int MPU_adress = 0x68;//MPU6050在I²C总线上的地址为0x68
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;

void time_interrupt() {
  sei();
  speed_left = 1.0 * 100 * count_left / (2 * 13);
  count_left = 0;
  speed_right = 1.0 * 100 * count_right / (2 * 13);
  count_right = 0;
}

void count_left_fun() {
  if (digitalRead(encode_left_A) == LOW) {
    if (digitalRead(encode_left_B) == LOW)
      count_left --;
    else
      count_left ++;
  } else {
    if (digitalRead(encode_left_B) == LOW)
      count_left ++;
    else
      count_left --;
  }
}

void count_right_fun() {
  if (digitalRead(encode_right_A) == LOW) {
    if (digitalRead(encode_right_B) == LOW)
      count_right ++;
    else
      count_right --;
  } else {
    if (digitalRead(encode_right_B) == LOW)
      count_right --;
    else
      count_right ++;
  }
}

int Get_balance_PWM() {
  
}

int Get_velocity_PWM() {
  
}

void set_PWM_left(int PWM) {
  if (PWM > 0) {
    analogWrite(Left_IN1, PWM + 30);
    analogWrite(Left_IN2, 0);
  } else {
    analogWrite(Left_IN1, 0);
    analogWrite(Left_IN2, - PWM + 40);
  }
}

void set_PWM_right(int PWM) {
  if (PWM > 0) {
    analogWrite(Right_IN2, PWM + 50);
    analogWrite(Right_IN1, 0);
  } else {
    analogWrite(Right_IN2, 0);
    analogWrite(Right_IN1, - PWM + 60);
  }
}

void read_angle() {
  Wire.beginTransmission(MPU_adress);//启动与mpu6050的通讯
  Wire.write(0x3B);//访问保存加速度和陀螺仪数据的启示地址
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_adress, 14, true);//访问14个寄存器，八种数据，向MPU6050索取数据
  AcX = Wire.read() << 8 | Wire.read();//7中数据14个字节，每种数据占两个字节分别为为数据的高位和低位，将高位和低位组合成两个字节才能得到完整的数据
  AcY = Wire.read() << 8 | Wire.read();//相当于前者×256和后者按位与最后得出十进制数
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();
}

void filt_angle() {
  dt = micros() - begin_;
  begin_ = micros();
  speed_angle = (float)GyY / 32768 * 250;
  Pro_angle = old_angle + speed_angle * ((float)dt / 1000000);
  if (AcZ == 0) AcZ = 1;
  Acc_angle = atan((float)AcX / AcZ) * (180 / PI);
  new_angle = Pro_angle * (1 - filter_a) + Acc_angle * filter_a;
  speed_angle_filter = (new_angle - old_angle) / ((float)dt / 1000000);
  old_angle = new_angle;
}

void setup() {
  pinMode(Right_IN1, OUTPUT);
  pinMode(Right_IN2, OUTPUT);
  pinMode(Left_IN1, OUTPUT);
  pinMode(Left_IN2, OUTPUT);

  analogWrite(Right_IN1, 0);
  analogWrite(Right_IN2, 0);
  analogWrite(Left_IN1, 0);
  analogWrite(Left_IN2, 0);

  pinMode(encode_left_A, INPUT);
  pinMode(encode_left_B, INPUT);
  pinMode(encode_right_A, INPUT);
  pinMode(encode_right_B, INPUT);  
  pinMode(13, OUTPUT);  
  
  delay(200);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encode_left_A), count_left_fun, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encode_right_A), count_right_fun, CHANGE);
  
  Wire.begin();//用begin函数启动I²C数据总线
  Wire.beginTransmission(MPU_adress);//向I²C启动发送地址
  Wire.write(0x6B);//写入MPU6050，0x6B是mpu6050的电源管理寄存器其中Bit6位控制着mpu6050是否在睡眠模式
  Wire.write(0);//用write写0关闭mpu6050的睡眠模式
  Wire.endTransmission(true);//结束Arduino与mpu6050的前期准备工作
  Serial.begin(9600);
  
  read_angle();
  
  old_angle = atan((float)AcX / AcZ) * (180 / PI);
  
  //MsTimer2::set(10, time_interrupt);
  //MsTimer2::start();
  begin_ = micros();
}


void loop() {
  while (1) {
    read_angle();
    filt_angle();
    balance_angle = new_angle;
    bias = balance_angle - zhongzhi;
    
    PWM = balance_Kp * bias + balance_Kd * speed_angle_filter;
    set_PWM_left(PWM);
    set_PWM_right(PWM);

    Serial.print(speed_angle_filter);
    Serial.print("\t\t");
    Serial.print(Pro_angle);
    Serial.print("\t\t");
    Serial.print(Acc_angle);
    Serial.print("\t\t");
    Serial.print(new_angle);
    Serial.print("\t\t");
    Serial.println(PWM);
    digitalWrite(13, !digitalRead(13));
 

  }
}

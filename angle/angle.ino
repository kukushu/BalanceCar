#include <Wire.h>

float new_angle = 0;
float old_angle = 0;
float Pro_angle = 0;
float Acc_angle = 0;
float a = 0.4;

long begin_ = 0;
int dt = 0;
const int MPU_adress = 0x68;//MPU6050在I²C总线上的地址为0x68
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
void setup(){
  Wire.begin();//用begin函数启动I²C数据总线
  Wire.beginTransmission(MPU_adress);//向I²C启动发送地址
  Wire.write(0x6B);//写入MPU6050，0x6B是mpu6050的电源管理寄存器其中Bit6位控制着mpu6050是否在睡眠模式
  Wire.write(0);//用write写0关闭mpu6050的睡眠模式
  Wire.endTransmission(true);//结束Arduino与mpu6050的前期准备工作
  Serial.begin(9600);
  
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

  old_angle = atan((float)AcX / AcZ) * 57.2;
  
  begin_ = micros();
}
void loop(){
  Wire.beginTransmission(MPU_adress);//启动与mpu6050的通讯
  Wire.write(0x3B);//访问保存加速度和陀螺仪数据的启示地址
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_adress, 14, true);//访问14个寄存器，八种数据，向MPU6050索取数据
  begin_ = micros();
  AcX = Wire.read() << 8 | Wire.read();//7中数据14个字节，每种数据占两个字节分别为为数据的高位和低位，将高位和低位组合成两个字节才能得到完整的数据
  AcY = Wire.read() << 8 | Wire.read();//相当于前者×256和后者按位与最后得出十进制数
  AcZ = Wire.read() << 8 | Wire.read();
  Tmp = Wire.read() << 8 | Wire.read();
  GyX = Wire.read() << 8 | Wire.read();
  GyY = Wire.read() << 8 | Wire.read();
  GyZ = Wire.read() << 8 | Wire.read();

  dt = micros() - begin_;
  Pro_angle = old_angle + (float)GyY / 32768 * 250 * ((float)dt / 1000000);
  Acc_angle = atan((float)AcX / AcZ) * (180 / PI);
  new_angle = Pro_angle * (1 - a) + Acc_angle * a;
  Serial.print(Pro_angle);
  Serial.print("\t\t");
  Serial.print(Acc_angle);
  Serial.print("\t\t");
  Serial.println(new_angle);
  old_angle = new_angle;
  begin_ = micros();

    
/*
  Serial.print("AcX = ");Serial.print(AcX);
  Serial.print(" | AcY = ");Serial.print(AcY);
  Serial.print(" | AcZ = ");Serial.print(AcZ);
  Serial.print(" | Tmp = ");Serial.print(Tmp / 340.00 + 36.53);//地址：高八位0X41、低八位0X42直接通过读取寄存器中的值来得到温度数据，温度换算公式为：Temperature = 36.53 + regval/340
  Serial.print(" | GyX = ");Serial.print(GyX);
  Serial.print(" | GyY = ");Serial.print(GyY);
  Serial.print(" | GyZ = ");Serial.println(GyZ);
  delay(1000);
*/
}

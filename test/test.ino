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

int speed_left = 0;
long count_left = 0;
int speed_right = 0;
long count_right = 0;

void time_interrupt();
void count_left_fun();
void count_right_fun();

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
  
  delay(200);

  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encode_left_A), count_left_fun, CHANGE);
  attachPinChangeInterrupt(digitalPinToPinChangeInterrupt(encode_right_A), count_right_fun, CHANGE);

  MsTimer2::set(10, time_interrupt);
  MsTimer2::start();
  
  Serial.begin(9600); 
}
void loop() {
  Serial.print("L: ");
  Serial.print(speed_left);
  Serial.print("  R: ");
  Serial.println(speed_right);
  delay(100);
}

void time_interrupt() {
  sei();
  speed_left = 100 * 60 * count_left / (2 * 13 * 30);
  count_left = 0;
  speed_right = 100 * 60 * count_right / (2 * 13 * 30);
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

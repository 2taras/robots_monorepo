#define dirL 7
#define speedL 6
#define speedR 5
#define dirR 4

#include "Arduino.h"
#include "Octoliner.h"

volatile long encL = 0;
volatile long encR = 0;

Octoliner octoliner;


void e_encL() {
  if(digitalRead(2) == digitalRead(9)) {
    encL++;
  } else {
    encL--;
  }
}
void e_encR() {
  if(digitalRead(3) == digitalRead(10)) {
    encR--;
  } else {
    encR++;
  }
}

void move_(int left, int right) {
  digitalWrite(dirL, left < 0);
  digitalWrite(dirR, right > 0);

  left = abs(constrain(left, -100, 100));
  right = abs(constrain(right, -100, 100));
  left = map(left, 0, 100, 0, 255);
  right = map(right, 0, 100, 0, 255);

  analogWrite(speedL, left);
  analogWrite(speedR, right);
}

void stop_() {
  encL = 0;
  encR = 0;
  float k = 20;
  unsigned long long point = millis();
  while(point + 400 >= millis()) {
    move_(-encL * k, -encR * k);
  }
  move_(0, 0);
}

void move_sync(int speed_) {
  float k = 1.5;
  float err = (encL - encR) * k;
  move_(speed_ - err, speed_ + err);
}

void move_cm(int speed_, int dist) {
  encL = 0;
  encR = 0;
  while(abs(encL) / 35 <= dist) {
    move_sync(speed_);
    delay(10);
  }
}

// Параметры PID — подбери под свой робот
float Kp = 0.5;
float Ki = 0.01;
float Kd = 1.0;

float last_err = 0;
float sum_err = 0;

void line(int speed_) {
  float err = (octoliner.analogRead(6) - octoliner.analogRead(2));

  sum_err += err;
  
  float d_err = err - last_err;
  last_err = err;

  float pid = Kp * err + Ki * sum_err + Kd * d_err;
  
  move_(speed_ - pid, speed_ + pid);
}
void line_cm(int speed_, int dist) {
  encL = 0;
  encR = 0;
  while(encL / 35 <= dist) {
    line(speed_);
    delay(10);
  }
}

#define LINE_TRASHOLD 960
// bool is_cross() {
//   int16_t brightness_value[8];
//   octoliner.analogReadAll(brightness_value);
//   int cnt = 0;
//   for (int i_br_val = 0; i_br_val < 8; i_br_val++) {
//     if (brightness_value[i_br_val] > LINE_TRASHOLD) {cnt++;}
//   }
//   //logger.debug(str_out);
//   return (cnt > 5);
// }
bool is_cross() {
  return (octoliner.analogRead(0) > LINE_TRASHOLD || octoliner.analogRead(7) > LINE_TRASHOLD);
}
void cross(int n = 1){ 
  unsigned long long point = millis();
  while(n) {
    line(85);
    if(is_cross() and point <= millis()) {
      n--;
      point = millis() + 500;
    }
    delay(10);
  }
}

void turn_enc(int angle) {
  encL = 0;
  encR = 0;
  if(angle > 0) {
    while(encL / 5.35 <= angle) {
      move_(70, -70);
    }
  } else {
    while(abs(encL) / 5.35 <= abs(angle)) {
      move_(-70, 70);
    }
  }
}
void turn_line(int dir) {
  // Ограничение на угол поворота для страховки (например, 100°)
  const float max_angle = 100.0;
  const float TICKS_PER_DEG = 5.35; // как в твоём turn_enc
  long max_ticks = max_angle * TICKS_PER_DEG;

  // Сброс энкодеров
  encL = 0;
  encR = 0;

  // PID переменные (локальные, т.к. для каждого поворота свои)
  float local_last_err = 0;
  float local_sum_err = 0;

  // Безопасность: таймаут ~1,5 сек
  unsigned long start = millis();

  // Крутимся, пока не увидим линию или не выйдем за пределы max_ticks по среднему энкодеру, либо не наступил таймаут
  while (
    (
      (dir == 1 && octoliner.analogRead(6) < LINE_TRASHOLD) || 
      (dir == 0 && octoliner.analogRead(2) < LINE_TRASHOLD)
    )
    &&
    ( ((abs(encL) + abs(encR))/2) < max_ticks )
    && (millis() - start < 1500)
  ) 
  {
    float err = (octoliner.analogRead(6) - octoliner.analogRead(2));
    local_sum_err += err;
    float d_err = err - local_last_err;
    local_last_err = err;
    float pid = Kp * err + Ki * local_sum_err + Kd * d_err;

    int base_speed = 70;

    // крутимся с ПИД-коррекцией
    if (dir == 1) { // направо
      move_(base_speed - pid, -base_speed - pid);
    } else {        // налево
      move_(-base_speed - pid, base_speed - pid);
    }

    delay(10);
  }

  stop_(); // плавная остановка
}


void setup() {
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(10, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  attachInterrupt(0, e_encL, CHANGE);
  attachInterrupt(1, e_encR, CHANGE);

  Serial.begin(115200);

  octoliner.begin();
  octoliner.setSensitivity(200);
}
void printEncSignals() {
  Serial.print(" L_A: "); Serial.print(digitalRead(2));
  Serial.print(" L_B: "); Serial.print(digitalRead(9));
  Serial.print("| R_A: "); Serial.print(digitalRead(3));
  Serial.print(" R_B: "); Serial.println(digitalRead(10));
}
void printLineVals() {
  Serial.print("OCTO: "); Serial.print(octoliner.analogRead(7));
  Serial.print(" "); Serial.print(octoliner.analogRead(6));
  Serial.print(" "); Serial.print(octoliner.analogRead(5));
  Serial.print(" "); Serial.print(octoliner.analogRead(4));
  Serial.print(" "); Serial.print(octoliner.analogRead(3));
  Serial.print(" "); Serial.print(octoliner.analogRead(2));
  Serial.print(" "); Serial.print(octoliner.analogRead(1));
  Serial.print(" "); Serial.println(octoliner.analogRead(0));
}
// 930-935
// 1002
void loop() {

  delay(700);

  cross(1);
  move_cm(70, 12);
  stop_();
  turn_enc(-95);
  stop_();
  
  line_cm(80, 50);
  cross(1);
  move_cm(70, 12);
  cross(1);
  move_cm(70, 12);
  
  cross(1);
  move_cm(70, 12);
  stop_();
  turn_enc(-94);
  stop_();

  cross(1);
  move_cm(70, 12);
  
  cross(1);
  move_cm(70, 12);
  stop_();
  turn_enc(-94);
  stop_();

  cross(1);
  move_cm(70, 12);
  cross(1);
  move_cm(70, 12);

  cross(1);
  move_cm(70, 12);
  stop_();
  turn_enc(-94);
  stop_();

  cross(1);
  move_cm(70, 12);
  stop_();

  delay(50000);

  // printEncSignals();
  // delay(100);
  delay(10);
}

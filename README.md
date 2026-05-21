# 자율주행 미로 탈출 로봇 (ESP32 기반 LiDAR & 모터 제어 최적화)

1. 프로젝트 개요

목표: 2D LiDAR와 ESP32를 활용하여 미로를 최단 시간에 완주하고 지정된 위치에 정지하는 자율주행 로봇 개발

핵심 성과: 하드웨어 기구적 편차 보정 및 FreeRTOS 병렬 처리를 통해 미로 완주 시간 1분 20초 -> 55초 이내로 단축

2. 기술 스택 (Tech Stack)

MCU: ESP32-S3 Dev Module

Sensor: 2D LiDAR Sensor, IR Reflective Sensor, 엔코더 내장형 DC 모터

Framework: Arduino IDE (C++)

OS: FreeRTOS (멀티코어 태스크 할당)

3. 핵심 문제 해결 과정 (Troubleshooting)
이 부분을 자소서 내용과 똑같이 적어주면 면접관이 코드랑 자소서를 매칭하기 엄청 편해!

이슈 1: 좌측 모터 출력 편차로 인한 주행 불안정 (우수법 실패)

해결: LEFT_MOTOR_BIAS 1.1 상수를 정의하여 좌측 모터 출력에 가중치를 부여, 기구적 편차를 S/W로 보정하여 직진성 및 우수법(오른쪽 벽 추종) 안정감 확보.

이슈 2: 단일 코어 연산 지연으로 인한 센서 데이터 병목

해결: xTaskCreatePinnedToCore를 활용하여 Core 1에 LiDAR 데이터 수신 및 전처리(lidarTask)를 전담시키고, 메인 루프에서 모터 제어를 분리하여 실시간(Real-time) 제어 환경 구축.

4. 시스템 동작 영상
   - 단순탈출 : https://youtube.com/shorts/41Ar4Lylkko?feature=share
   - 탈출기록 단축 및 정지 : https://youtube.com/shorts/sJ1hzj4Izsg?feature=share

5. 전체 코드

#include <Arduino.h>



// 하드웨어 튜닝 값

// 왼쪽 모터가 오른쪽보다 힘이 약해서 1.1배 더 줍니다. (직진성 보정,우수법 유도)

#define LEFT_MOTOR_BIAS 1.1

// 정지선 인식 감도 

#define LINE_THRESHOLD 3000



hw_timer_t * timer = NULL;

long lcnt = 0, rcnt = 0;

int vd = 0, wd = 0;



bool det = 0, mode = 0;

unsigned int fl_det = 0, fr_det = 0, c_det = 0, l_det = 0, r_det = 0;

unsigned int l_min = 60000, r_min = 60000;



// 정지선감지 관련 변수

int whiteLineCount = 0;   // 흰색 선이 연속으로 감지된 횟수 (오작동 방지)

bool onLine = false;      // 현재 선 위에 있는지

bool mazeFinished = false; // 미로 탈출 완료 플래그



// [타이머 인터럽트] 모터 속도 제어

void ARDUINO_ISR_ATTR onTimer() {

  int vl = lcnt;

  int vr = rcnt;  

  lcnt = 0; rcnt = 0;

 

  int v = vl + vr;

  int w = vl - vr;    

 

  // P제어

  int mv = 6 * vd + (vd - v) * 5;

  int mw = 6 * wd + (wd - w) * 5;



  // 왼쪽 모터에 가중치 적용

  motor(mv + mw, (mv - mw));

}



// 모터 구동 함수 

void motor(int vl, int vr) {

  // 왼쪽 모터 편차 보정 적용

  vl = vl * LEFT_MOTOR_BIAS;



  if (vl > 1023) vl = 1023; else if (vl < -1023) vl = -1023;

  if (vr > 1023) vr = 1023; else if (vr < -1023) vr = -1023;

 

  if (vl > 0) { ledcWrite(15, vl); ledcWrite(16, 0); }  

  else { ledcWrite(16, -vl); ledcWrite(15, 0); }

 

  if (vr > 0) { ledcWrite(11, vr); ledcWrite(12, 0); }  

  else { ledcWrite(12, -vr); ledcWrite(11, 0); }

}



void setup() {

  Serial.begin(500000);

 

  // LED 설정

  pinMode(21, OUTPUT); pinMode(17, OUTPUT);

  pinMode(47, OUTPUT); pinMode(48, OUTPUT);

 

  // 정지선 감지

  pinMode(5, INPUT); pinMode(10, INPUT);

 

  delay(500);

  Serial.println("--- System Start ---");

  delay(1000);



  // 라이다 태스크

  xTaskCreatePinnedToCore(lidarTask, "Lidar Task", 20000, NULL, 10, NULL, 1);



  // 엔코더 및 모터 핀 설정

  pinMode(6, INPUT); pinMode(7, INPUT); pinMode(8, INPUT); pinMode(9, INPUT);

  pinMode(15, OUTPUT); pinMode(16, OUTPUT); pinMode(11, OUTPUT); pinMode(12, OUTPUT);  



  ledcAttach(15, 20000, 10); ledcAttach(16, 20000, 10);

  ledcAttach(11, 20000, 10); ledcAttach(12, 20000, 10);

 

  attachInterrupt(digitalPinToInterrupt(6), ELA, CHANGE); attachInterrupt(digitalPinToInterrupt(7), ELB, CHANGE);

  attachInterrupt(digitalPinToInterrupt(8), ERA, CHANGE); attachInterrupt(digitalPinToInterrupt(9), ERB, CHANGE);



  timer = timerBegin(1000000);

  timerAttachInterrupt(timer, &onTimer);

  timerAlarm(timer, 10000, true, 0);        

}



void loop() {

  // 1. 미로 끝 정지선 감지 로직

  int valL = analogRead(5);

  int valR = analogRead(10);



  // 센서값 확인용 (디버깅)

  static unsigned long lastPrint = 0;

  if (millis() - lastPrint > 500) { // 0.5초마다 출력

    // Serial.print("Sensors: "); Serial.print(valL); Serial.print(" | "); Serial.println(valR);

    lastPrint = millis();

  }



  // 흰색 선 감지 (값이 작으면 흰색)

  if (valL < LINE_THRESHOLD && valR < LINE_THRESHOLD) {

    if (!onLine) {        

      whiteLineCount++;    // 선에 새로 진입할 때만 카운트 +1

      onLine = true;      

    }

  } else {

    onLine = false;        // 선에서 벗어남

  }



  // 두 번 이상 확실히 밟으면 정지 (오인식 방지)

  if (whiteLineCount >= 2) {

    mazeFinished = true;

  }



  // 종료 상태면 모든 동작 멈춤

  if (mazeFinished) {

    vd = 0; wd = 0;

    motor(0, 0);

    lrled(0);    // LED 끄기

    digitalWrite(47, HIGH); digitalWrite(48, HIGH); // 종료 알림 LED

    return;      

  }



  // 2. 주행 및 회피 알고리즘

  if (det == 1) {

    // 평상시 주행 (장애물 없음)

    if (mode == 0) {  

      // 오른쪽이 뚫려있으면 우회전 시도

      if (r_det == 0) {

        vd = 0; wd = 20; // 제자리 우회전

      }

      // 전방에 장애물이 있으면

      else if (fl_det || fr_det) {

        // 좌/우 공간 비교해서 넓은 쪽으로 회전

        if (l_det || r_det) {

          if (l_min < r_min) { vd = 0; wd = 20; } // 오른쪽이 더 넓으면 우회전

          else { vd = 0; wd = -20; }              // 왼쪽이 더 넓으면 좌회전

          mode = 1; // 회전 상태로 변경

        }

        else {

          // 구석에 몰렸을 때 탈출

          if (fl_det) { vd = 0; wd = 20; }

          else { vd = 0; wd = -20; }

          mode = 1;

        }

      }

      // 장애물 없으면 직진

      else {

        vd = 50; wd = 0;

      }

    }

    // 회전 중인 상태 (장애물이 사라질 때까지 회전 유지)

    else {  

      if (c_det == 0) { // 전방이 뚫리면

        vd = 40; wd = 0;

        mode = 0; // 다시 주행 모드로

      }

    }

   

    lrled(wd);

    // 변수 초기화

    det = 0; fl_det = 0; fr_det = 0; c_det = 0; l_det = 0; r_det = 0;  

    l_min = 60000; r_min = 60000;

  }

 

 delay(1);

}



// LED 제어 함수 

void lrled(int p) {  

  if (p > 0)       { digitalWrite(21, !digitalRead(21)); digitalWrite(17, LOW); digitalWrite(47, LOW); digitalWrite(48, HIGH); }

  else if (p < 0)  { digitalWrite(17, !digitalRead(17)); digitalWrite(21, LOW); digitalWrite(47, LOW); digitalWrite(48, HIGH); }

  else             { digitalWrite(47, HIGH); digitalWrite(21, LOW); digitalWrite(17, LOW); digitalWrite(48, LOW); }

}



// 엔코더 인터럽트 함수들

void ELA() { if (digitalRead(6) != digitalRead(7)) { lcnt++; } else { lcnt--; } }

void ELB() { if (digitalRead(6) == digitalRead(7)) { lcnt++; } else { lcnt--; } }      

void ERA() { if (digitalRead(8) == digitalRead(9)) { rcnt++; } else { rcnt--; } }

void ERB() { if (digitalRead(8) != digitalRead(9)) { rcnt++; } else { rcnt--; } }



// 라이다 데이터 처리 태스크

void lidarTask(void * parameter) {

  int p = 0;

  Serial1.begin(460800, SERIAL_8N1, 44, 43);  

  Serial1.write(0xA5); Serial1.write(0x20);  



  while (!Serial1.available()) { delay(1); }        

  while ((Serial1.available()) || (p < 7)) { byte temp = Serial1.read(); p++; delay(1); }



  int n = 0; byte d[5];

  while (1) {

    if (Serial1.available()) {

      d[n] = Serial1.read(); n++;

      if (n > 4) {

        n = 0;

        if (bitRead(d[0], 0) == 1) { det = 1; }  

        unsigned int ang = (d[1] >> 1) + (d[2] << 7);

        unsigned int dis = (d[3] + (d[4] << 8)) >> 2;

        float angle = ang * 2.7269965e-4;

        float xx = dis * sin(angle), yy = dis * cos(angle);

       

        // [장애물 위치 판별] 좌표 범위로 카운팅

        if ((abs(xx) < 65) && (yy > 45) && (yy < 100)) {

          if (xx < 0) { fl_det++; } else { fr_det++; }

        }

        if ((abs(xx) < 55) && (yy > 45) && (yy < 200)) { c_det++; }

       

        if ((xx < -45) && (xx > -150) && (yy > 0) && (yy < 50)) {

          l_det++; if (-xx < l_min) { l_min = -xx; }

        }

        if ((xx > 45) && (xx < 150) && (yy > 0) && (yy < 50)) {

          r_det++; if (xx < r_min) { r_min = xx; }

        }

      }

    }

  }

} 


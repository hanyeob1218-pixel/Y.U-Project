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

미로탈출 주행영상 https://youtube.com/shorts/41Ar4Lylkko?feature=share
미로탈출 및 정지 영상 https://youtube.com/shorts/sJ1hzj4Izsg?feature=share

```markdown
# STM32 하위 안전 제어 펌웨어

이 폴더는 STM32에서 실행되는 실시간 모터 및 안전 제어 펌웨어를 관리한다.

## 주요 역할

- Jetson 명령 수신 및 CRC 검증
- 통신 Watchdog
- 목표 속도 제한
- 좌우 모터 속도 제어
- VESC 통신
- Cliff·ToF 낙차 감지
- E-Stop 입력 감시
- 햅틱 출력
- 센서·모터 고장 판단
- 안전 상태와 진단 정보 전송
- 긴급정지 후 자동 재출발 방지

## 구조

```text
stm32/
├── README.md
├── Core/
│   ├── Inc/
│   └── Src/
├── Drivers/
└── <project>.ioc

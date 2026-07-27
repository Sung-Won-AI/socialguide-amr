Jetson–STM32 통신 규격 초안

상태: Draft  
프로토콜 버전: 1

## 프레임

| 필드 | 크기 | 설명 |
|---|---:|---|
| SOF | 2 bytes | `AA 55` |
| Version | 1 byte | 현재 `1` |
| Message ID | 1 byte | 메시지 종류 |
| Sequence | 1 byte | 0~255 순환 |
| Payload Length | 1 byte | 최대 32 |
| Payload | N bytes | Little-endian |
| CRC16 | 2 bytes | Version부터 Payload까지 계산 |

CRC는 CRC-16/CCITT-FALSE를 사용한다.

## DRIVE_COMMAND

Payload 형식: `<HhhHBB`

| 필드 | 형식 | 단위 |
|---|---|---|
| command_id | uint16 | - |
| linear_velocity | int16 | mm/s |
| angular_velocity | int16 | mrad/s |
| speed_limit | uint16 | mm/s |
| control_flags | uint8 | bit field |
| reserved | uint8 | - |

## ROBOT_STATUS

Payload 형식: `<BHhhHHHHI`

| 필드 | 형식 | 단위 |
|---|---|---|
| system_state | uint8 | 상태 코드 |
| safety_flags | uint16 | bit field |
| left_velocity | int16 | mm/s |
| right_velocity | int16 | mm/s |
| battery_voltage | uint16 | mV |
| motor_error | uint16 | 오류 코드 |
| last_command_id | uint16 | - |
| rx_error_count | uint16 | count |
| uptime | uint32 | ms |

## 안전 규칙

- CRC 또는 길이가 잘못된 명령은 실행하지 않는다.
- 마지막 유효 주행 명령 이후 300ms가 지나면 STM32가 정지한다.
- 통신이 복구돼도 자동으로 재출발하지 않는다.
- Cliff, E-Stop 및 STM32 고장은 Jetson 명령보다 우선한다.



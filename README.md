# 미지 화물 쿼드로터의 로터 완전 고장 대응 능동 적응 제어 — 재현 코드

Reproduction code for *Active Adaptive Control for a Quadrotor with an Unknown Payload under Complete Rotor Failure* (IEIE 2026 Fall Undergraduate Paper Competition).

질량을 모르는 하부 화물을 실은 쿼드로터에서 호버 중 화물 질량을 추정하고, 로터 하나가 완전히 정지하면 추정 질량에 맞춘 선제 추력 주입과 제어 파라미터 전환으로 고도 손실을 줄이는 제어기의 시뮬레이션 코드다. 기반 제어기와 드론 모델은 Beyer 등의 공개 저장소를 그대로 쓴다.

## 실행 환경

- MATLAB R2026a 또는 R2026b, Simulink, Curve Fitting Toolbox (두 릴리스에서 결과가 유효숫자 12자리까지 같음을 확인)
- Windows 에서는 **짧은 경로**(예 `C:\pftc`)에 받을 것. Simulink 가 만드는 C 파일 경로가 260자를 넘으면 빌드가 실패한다.

## 실행

```matlab
cd scripts
verify_release_IRIS      % 대표 6판, 약 3~4분
```

한 판만 돌리려면

```matlab
cd scripts
addPathFtc
ctx = ctx_current_IRIS();
r   = run_case_IRIS(ctx, '다', 0.340, 2);   % 제안 제어기 · 화물 340 g · 2번 로터 고장
```

`kind` 는 `'가'`(무화물 · 기반 제어기), `'나'`(화물 · 기반 제어기), `'다'`(화물 · 제안 제어기), `'무고장'` 이다.

## 기대 결과 (`verify_release_IRIS`)

| 판 | 초기 강하 [m] | 후반 최대 강하 [m] | 최대 기울기 [°] | 추정 질량 [g] |
|---|---|---|---|---|
| 가 0 g, 2번 로터 | 5.150 | 0.636 | 20.72 | – |
| 나 340 g, 2번 로터 | 5.642 | 5.143 | 61.18 | – |
| 다 340 g, 2번 로터 | 1.904 | 0.194 | 19.37 | 337.9 |
| 다 100 g, 2번 로터 | 1.086 | 0.021 | 17.92 | 99.6 |
| 다 340 g, 3번 로터 | 1.803 | 0.195 | 20.20 | 337.9 |

지표 정의 — 초기 강하는 고장(35 s) 뒤 35~45 s 의 최저 고도까지 내려간 거리, 후반 최대 강하와 최대 기울기는 45~77 s 구간의 값이다.

## 구성

| 경로 | 내용 |
|---|---|
| `scripts/` | 제안 제어기 구성과 평가 하네스. 현행 조건 값은 `cfg_current_IRIS.m` 한 곳 |
| `models/QuadcopterSimModel_Loiter_FTC_dseg.slx` | 실험 모델(감지 · 주입 블록을 더한 Simulink 모델) |
| `data/` | 감지 상수, 무화물 기준 회전 속도, 도달 기준점 |
| `modules/` | 원저자 라이브러리 LADAC 와 기체 제원 |

## 출처와 라이선스

- 기반 코드: Y. Beyer, M. Steen, P. Hecker, "Incremental Passive Fault-Tolerant Control for Quadrotors Subjected to Complete Rotor Failures," *J. Guid. Control Dyn.*, 46(10), 2023 — https://github.com/iff-gsc/Passive-FTC-Quadrotor
- 이 저장소는 기반 코드의 라이선스(GNU GPL v3, `LICENSE`)를 따른다. `modules/` 안 파일은 원래 머리말의 라이선스를 유지한다.
- 코드 주석은 한국어다.

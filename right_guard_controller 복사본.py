#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
██████  RIGHT GUARD ·  RIGHT TURN, RIGHT TIME  ██████
        고려대학교(KOREA UNIVERSITY) · KU CreativeX · 창의연구 프로젝트(CRP)
        공감형 · 우회전 교통안전  |  Team Right Guard
        팔레트 — CRIMSON #910023 · IVORY #E7DECE · 신호 RED #E23A2E · AMBER #F4A823 · GREEN #1FA65A
=====================================================================

Right Guard — 우회전 보조 LED 컨트롤러 (프로토타입)
초음파 센서로 보행자(거리·움직임)를 감지하고, 도로교통법 판단 로직에 따라
도로 바닥 LED 신호(STOP 적색 / SLOW 녹색)를 출력하는 라즈베리 파이용 제어 코드.

이 한 파일로 두 가지 모드를 지원합니다.
  • 노트북(하드웨어 없이) 테스트 :   python3 right_guard_controller.py --mock
  • 라즈베리 파이(실제 센서)   :   python3 right_guard_controller.py

핵심 설계
  1) "도플러"의 현실적 구현 = 짧은 간격으로 거리를 반복 측정 → 거리 변화율로
     접근 속도/방향을 추정(=유사 도플러). 보고서엔 "순차 거리 샘플링 기반
     속도 추정"으로 쓰면 기술적으로 정확합니다.
  2) 오감지(낙엽·빗방울·정지물) 억제 = 중앙값 필터 + 상태 히스테리시스.
  3) 페일세이프 = 센서 무응답/예외 시 무조건 STOP(보행자 안전 우선).
  4) 측정 데이터 = 매 프레임 CSV 기록 → 감지율/오감지율/반응시간 산출 근거.

하드웨어 배선(예시, BCM 핀 번호)
  HC-SR04P(3.3V형) :  TRIG=GPIO23, ECHO=GPIO24   ← 3.3V형이라 Echo 분압 불필요
     ※ 일반 HC-SR04(5V형)를 쓰면 ECHO에 저항 분압(예: 1kΩ+2kΩ) 필수!
  적색 LED = GPIO17, 녹색 LED = GPIO27 (각 220~330Ω 직렬)
     ※ 도로 화살표처럼 LED를 많이/밝게 쓸 땐 GPIO 직접구동 금지.
        MOSFET 스위칭 또는 WS2812B 스트립(외부 5V) 사용 → set_signal()에 훅 표시.

담당: 서무성(로직/펌웨어) · 안치우(회로) · 박주언(임계값 튜닝/측정)
"""

import argparse
import csv
import math
import random
import statistics
import sys
import time
from collections import deque
from enum import Enum

# ──────────────────────────────────────────────────────────────────────────
# 튜닝 파라미터 (박주언: 측정하며 이 값들을 조정)
# ──────────────────────────────────────────────────────────────────────────
TRIG_PIN, ECHO_PIN = 23, 24
RED_PIN, GREEN_PIN = 17, 27

MAX_DIST_CM     = 400.0   # 센서 최대 측정 거리
D_WALK_CM       = 60.0    # 이 거리 이내 = 횡단보도 위/근접 → WALKING
D_ENTRY_CM      = 150.0   # 이 거리 이내 = 진입 감지 영역(밖이면 보행자 없음)
V_INTENT_CMS    = 8.0     # 접근 속도(+, cm/s) 임계 — 이상이면 '진입 의도'

MEDIAN_N        = 5       # 중앙값 필터 표본 수 (단발성 스파이크 제거)
VEL_WIN         = 6       # 속도 추정에 쓰는 최근 표본 수
LEAVE_FRAMES    = 6       # 보행자 '없음' 확정에 필요한 연속 프레임(히스테리시스)
PED_CLEAR_HOLD  = 1.5     # 보행자 이탈 후 STOP 유지 시간(s) — 보행자 안전 여유
SENSOR_TIMEOUT  = 0.5     # 센서 무응답이 이 시간(s) 넘으면 페일세이프(STOP)
LOOP_HZ         = 15      # 제어 루프 주기 (반응시간 목표 1초보다 훨씬 빠름)


class Ped(Enum):
    NONE = "NONE"        # 보행자 없음
    INTENT = "INTENT"    # 진입부 접근(통행하려는) 보행자
    WALKING = "WALKING"  # 횡단보도 내 보행자


class Veh(Enum):
    RED = "RED"
    GREEN = "GREEN"


class Sig(Enum):
    STOP = "STOP"        # 정지/적색
    SLOW = "SLOW"        # 서행/녹색


# ──────────────────────────────────────────────────────────────────────────
# 판단 로직 — 진행일지의 의사코드를 그대로 구현
# ──────────────────────────────────────────────────────────────────────────
def decide(ped: Ped, veh: Veh) -> Sig:
    """현행 도로교통법 기반 우회전 가능 여부 → LED 신호."""
    if ped in (Ped.WALKING, Ped.INTENT):   # 27조 1항: 건너는/건너려는 보행자 우선
        return Sig.STOP
    if veh is Veh.RED:                      # 보행자 없음 + 적색
        return Sig.STOP
    if veh is Veh.GREEN:                    # 보행자 없음 + 녹색
        return Sig.SLOW
    return Sig.STOP                         # 알 수 없는 입력 → 안전측(STOP)

    # ── 추후 확장(박주언) ────────────────────────────────────────────────
    # TODO 1) 25조(직진 우선): 좌측 직진 차량 감지 입력을 추가해, 보행자가 없어도
    #         직진 차량 흐름을 방해하면 STOP 유지하도록 분기 추가.
    # TODO 2) RED '정지 후 진행': 적색에서 '일시정지 완료 & 보행자 없음'이면
    #         SLOW로 전이하는 상태(STOPPED→PROCEED)를 둬서 법규와 더 정합화.


# ──────────────────────────────────────────────────────────────────────────
# 보행자 상태 추정기 — 거리 → (중앙값 필터) → 속도 → 상태 (+히스테리시스)
# ──────────────────────────────────────────────────────────────────────────
class PedestrianDetector:
    def __init__(self):
        self.dist_buf = deque(maxlen=MEDIAN_N)      # 중앙값 필터용
        self.vel_buf = deque(maxlen=VEL_WIN)        # (t, dist) 속도 추정용
        self.state = Ped.NONE
        self._leave_count = 0                       # 연속 '없음' 프레임 카운트

    def _velocity_cms(self):
        """접근 속도(+: 가까워짐). 최근 표본의 1차 회귀 기울기로 추정."""
        if len(self.vel_buf) < 2:
            return 0.0
        t0 = self.vel_buf[0][0]
        xs = [t - t0 for t, _ in self.vel_buf]
        ys = [d for _, d in self.vel_buf]
        n = len(xs)
        mx, my = sum(xs) / n, sum(ys) / n
        denom = sum((x - mx) ** 2 for x in xs)
        if denom == 0:
            return 0.0
        slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom  # cm/s
        return -slope   # 거리 감소(기울기 음수) = 접근(+)

    def update(self, raw_dist, now):
        """raw_dist: 이번 프레임 거리(cm) 또는 None(에코 없음=멀리). 반환: Ped 상태."""
        d = MAX_DIST_CM if raw_dist is None else raw_dist
        self.dist_buf.append(d)
        med = statistics.median(self.dist_buf)      # 스파이크 제거된 거리
        self.vel_buf.append((now, med))
        v = self._velocity_cms()

        # 1) 이번 프레임의 '순간' 후보 상태
        if med <= D_WALK_CM:
            cand = Ped.WALKING
        elif med <= D_ENTRY_CM and v >= V_INTENT_CMS:
            cand = Ped.INTENT
        else:
            cand = Ped.NONE

        # 2) 히스테리시스: 보호측(WALKING/INTENT) 진입은 즉시,
        #    해제(→NONE)는 연속 LEAVE_FRAMES 프레임 안정돼야 확정
        if cand in (Ped.WALKING, Ped.INTENT):
            self.state = cand
            self._leave_count = 0
        else:  # cand == NONE
            self._leave_count += 1
            if self._leave_count >= LEAVE_FRAMES:
                self.state = Ped.NONE
        return self.state, med, v


class SignalStabilizer:
    """보행자 이탈 직후에도 일정 시간 STOP을 유지(보행자 클리어런스)."""
    def __init__(self):
        self._last_stop_t = -1e9

    def apply(self, raw_signal: Sig, now) -> Sig:
        if raw_signal is Sig.STOP:
            self._last_stop_t = now
            return Sig.STOP
        if now - self._last_stop_t < PED_CLEAR_HOLD:  # 방금 전까지 STOP였으면 유지
            return Sig.STOP
        return Sig.SLOW


# ──────────────────────────────────────────────────────────────────────────
# 하드웨어 추상화 — 실제(gpiozero) / 모의(mock)
# ──────────────────────────────────────────────────────────────────────────
class RealSensor:
    def __init__(self):
        from gpiozero import DistanceSensor
        # 5V형 HC-SR04라면 ECHO에 분압 회로 필요. 3.3V형(HC-SR04P)은 그대로 OK.
        self.s = DistanceSensor(echo=ECHO_PIN, trigger=TRIG_PIN,
                                max_distance=MAX_DIST_CM / 100.0)

    def read(self):
        d_cm = self.s.distance * 100.0          # m → cm
        return None if d_cm >= MAX_DIST_CM - 1 else d_cm


class RealLed:
    def __init__(self):
        from gpiozero import LED
        self.red, self.green = LED(RED_PIN), LED(GREEN_PIN)

    def set_signal(self, sig: Sig):
        # ── WS2812B 화살표로 교체 시 여기서 스트립 색을 갱신하면 됩니다 ──
        if sig is Sig.STOP:
            self.red.on(); self.green.off()
        else:
            self.green.on(); self.red.off()

    def off(self):
        self.red.off(); self.green.off()


class RealVehicleSignal:
    """차량 신호(RED/GREEN). 프로토타입에선 버튼/토글로 모의 입력.
    실제 시스템에선 신호등 연동 또는 색 센서/카메라로 대체."""
    def __init__(self):
        from gpiozero import Button
        self.btn = Button(22, pull_up=True)  # 누르면 RED 모의

    def read(self) -> Veh:
        return Veh.RED if self.btn.is_pressed else Veh.GREEN


# ── 모의(mock): 노트북에서 시나리오 재생 ──────────────────────────────────
class MockSensor:
    """시간에 따른 보행자 거리 시나리오 + 잡음 + 단발성 스파이크."""
    def __init__(self, start):
        self.start = start

    def read(self):
        t = time.monotonic() - self.start
        if   t < 3:   base = None                      # 보행자 없음
        elif t < 6:   base = 130 - (t - 3) * 28        # 진입부 접근 130→46
        elif t < 10:  base = 30 + 6 * math.sin(t * 3)  # 횡단보도 위(미세 움직임)
        elif t < 13:  base = 30 + (t - 10) * 60         # 이탈 30→210
        elif t < 16:  base = None                      # 없음(이 구간에 스파이크)
        elif t < 20:  base = None                      # 없음(이때 V=RED)
        else:         base = None
        # 13.5초 부근 단발성 오감지 스파이크(낙엽/벌레) → 중앙값 필터가 걸러야 함
        if 13.4 < t < 13.5:
            return 25.0
        if base is None:
            return None
        return max(2.0, base + random.uniform(-2.5, 2.5))  # 측정 잡음


class MockVehicleSignal:
    def __init__(self, start, fixed=None):
        self.start, self.fixed = start, fixed

    def read(self) -> Veh:
        if self.fixed is not None:
            return self.fixed
        t = time.monotonic() - self.start
        return Veh.RED if t >= 16 else Veh.GREEN       # 16초 후 적색 구간


class ConsoleLed:
    SYM = {Sig.STOP: "🔴 STOP", Sig.SLOW: "🟢 SLOW"}

    def set_signal(self, sig: Sig):
        self._cur = sig

    def off(self):
        pass


# ──────────────────────────────────────────────────────────────────────────
# 메인 루프
# ──────────────────────────────────────────────────────────────────────────
def run(mock=False, csv_path=None, fixed_signal=None, duration=None):
    start = time.monotonic()

    if mock:
        sensor = MockSensor(start)
        led = ConsoleLed()
        vsig = MockVehicleSignal(start, fixed=fixed_signal)
        print("[MOCK] 시나리오 재생 — 하드웨어 없이 로직만 검증합니다.\n")
    else:
        try:
            sensor, led, vsig = RealSensor(), RealLed(), RealVehicleSignal()
        except Exception as e:
            print(f"[오류] 하드웨어 초기화 실패: {e}\n  → 노트북이면 --mock 옵션으로 실행하세요.")
            sys.exit(1)
        print("[REAL] 라즈베리 파이 하드웨어 모드. 종료: Ctrl+C\n")

    detector = PedestrianDetector()
    stab = SignalStabilizer()
    period = 1.0 / LOOP_HZ
    last_ok = time.monotonic()
    counts = {s: 0 for s in Sig}

    writer = fh = None
    if csv_path:
        fh = open(csv_path, "w", newline="", encoding="utf-8")
        writer = csv.writer(fh)
        writer.writerow(["t_s", "dist_cm", "vel_cms", "ped_state",
                         "veh_signal", "out_signal", "latency_ms"])

    print(f"{'t(s)':>6} {'dist':>7} {'vel':>7}  {'ped':<8} {'veh':<6} → {'OUT':<5} {'ms':>5}")
    print("─" * 56)
    try:
        while True:
            now = time.monotonic()
            t_rel = now - start
            t0 = time.perf_counter()

            # 1) 센서 읽기 (+ 페일세이프)
            try:
                raw = sensor.read()
                last_ok = now
                ped, med, vel = detector.update(raw, now)
                veh = vsig.read()
                signal = stab.apply(decide(ped, veh), now)
            except Exception as e:
                ped, med, vel, veh = Ped.NONE, float("nan"), 0.0, Veh.RED
                signal = Sig.STOP                          # 예외 → 안전측
                print(f"  [경고] 센서/판단 예외 → STOP 유지: {e}")
            if now - last_ok > SENSOR_TIMEOUT:             # 무응답 지속 → 안전측
                signal = Sig.STOP

            # 2) 출력
            led.set_signal(signal)
            latency_ms = (time.perf_counter() - t0) * 1000.0
            counts[signal] += 1

            # 3) 기록/표시
            if writer:
                writer.writerow([f"{t_rel:.3f}", f"{med:.1f}", f"{vel:.1f}",
                                 ped.value, veh.value, signal.value, f"{latency_ms:.2f}"])
            out_sym = ConsoleLed.SYM[signal]
            print(f"{t_rel:6.1f} {med:6.0f}cm {vel:6.0f}  {ped.value:<8} "
                  f"{veh.value:<6} → {out_sym:<7} {latency_ms:5.1f}")

            if duration and t_rel >= duration:
                break
            time.sleep(max(0.0, period - (time.monotonic() - now)))

    except KeyboardInterrupt:
        print("\n[종료] 사용자 중단.")
    finally:
        led.off()
        if fh:
            fh.close()
            print(f"\n[저장] 측정 로그 → {csv_path}")
        total = sum(counts.values()) or 1
        print(f"[요약] 프레임 {total}개 | "
              f"STOP {counts[Sig.STOP]} ({counts[Sig.STOP]/total*100:.0f}%) · "
              f"SLOW {counts[Sig.SLOW]} ({counts[Sig.SLOW]/total*100:.0f}%)")


def main():
    p = argparse.ArgumentParser(description="Right Guard 우회전 보조 LED 컨트롤러")
    p.add_argument("--mock", action="store_true", help="하드웨어 없이 시나리오로 검증")
    p.add_argument("--csv", metavar="PATH", help="측정 로그 CSV 저장 경로")
    p.add_argument("--signal", choices=["red", "green"], help="차량신호 고정(테스트용)")
    p.add_argument("--duration", type=float, default=21.0,
                   help="--mock 실행 시간(초). 실제 모드에선 무시")
    a = p.parse_args()
    fixed = {"red": Veh.RED, "green": Veh.GREEN}.get(a.signal)
    run(mock=a.mock, csv_path=a.csv, fixed_signal=fixed,
        duration=a.duration if a.mock else None)


if __name__ == "__main__":
    main()

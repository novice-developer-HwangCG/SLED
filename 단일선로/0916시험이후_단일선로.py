from machine import Pin, ADC, PWM
import utime

# ─── 시스템/하드웨어 요약 ────────────────────────────────────────────
# 단일 선로 170kg, 감속비 존재(감속 시 역방향 브레이크 불필요) RF 펄스: 최대 2009~2012us, 최소 986~988us 대역 (한계치 조정)

# ─── pin map & config ────────────────────────────────────────────
MODE_PIN        = 26    # 0=RF, 1=Manual
SAFETY_PIN      = 22

MANUAL_ADC_PIN  = 27
RF_PWM_PIN      = 21

MOTOR_PWM_PIN   = 15
MOTOR_DIR_PIN   = 14
MOTOR_EN_PIN    = 13

LED_PIN         = 0     # 알람/전원 LED (기본: 항상 ON)
EMERGENCY_PIN_1 = 10    # 비상 스위치 (NC, PULL_UP)
EMERGENCY_PIN_2 = 11    # 비상 스위치_2 (NC, PULL_UP)

PWR_ADC_PIN     = 28    # 배터리 전압 측정 ADC

# ─── ADC 범위/초기 센터(보정 전 임시값) ───────────────────────────
RAW_MIN     = 0
RAW_MAX     = 4095
RAW_CENTER  = 2300      # 부팅 보정 전 임시 레퍼런스 2200

# ─── 센터 보정 모드 선택 ──────────────────────────────────────────
USE_FIXED_CENTER = True         # True면 RAW_CENTER 고정, False면 자동보정 사용

# ─── 동작 파라미터 ───────────────────────────────────────────────
DEADZONE        = 200                   # 수동 데드존(+/-)
MANUAL_MAX_PCT  = 45                    # 수동 최대 출력(%): 유선이라 고속 불필요

# RF parameters
MID_POINT        = 1500
RF_PW_MIN        = 1145
RF_PW_MAX        = 1855
RF_RANGE_POS     = RF_PW_MAX - MID_POINT
RF_RANGE_NEG     = MID_POINT - RF_PW_MIN
NEUTRAL_THRESH   = 5
RF_DEADZONE_US   = 20
RAMP_MAX_STEP_US = 10                   # 100ms당 ±10us 정도(아래서 dt 비례 변환)

DUTY_STEP_UP     = 6                    # 프레임당 가속 한계(0..255)
DUTY_STEP_DOWN   = 10                   # 프레임당 감속 한계

# RF 스무딩 상태
rf_pw_smooth     = MID_POINT
rf_duty_smooth   = 0

# SC(3단) 기준 및 저속 한계
SC_LOW_EDGE  = 1400
SC_HIGH_EDGE = 1900
LOW_MIN      = 1300
LOW_MAX      = 1700
MAX_DUTY_LOW = 128                     # 해당 시제 버전 요구사항 유지

# RF 연결 타임아웃 (ms)
RF_TIMEOUT_MS   = 200

# LED 패턴
LED_CYCLE_MS = 3000
BLINK_ON_MS  = 200

# ─── 센터 보정 모드 ──────────────────────────────────────────────
CENTER_HOLD_AFTER_BOOT = True          # 부팅 시 1회 보정 후 세션 고정

# 부팅 센터 보정 파라미터
BOOT_CENTER_MS      = 1200            # 보정 수집 시간
BOOT_CENTER_MAX_DEV = 8               # 안정성 기준(최대-최소)
CENTER_CLAMP_MIN    = DEADZONE + 20
CENTER_CLAMP_MAX    = RAW_MAX - DEADZONE - 20

# (옵션) 연속 보정 모드 파라미터 (이번 버전 기본 미사용)
NEUTRAL_HOLD_MS   = 1000
TIGHT_DZ_RATIO    = 0.5
CENTER_EMA_NUM    = 1
CENTER_EMA_DEN    = 4

# 저전압 컷오프 (대략 %)
BAT_LOW_PCT       = 20
BAT_RECOVER_PCT   = 25
low_batt_active   = False

# ─── 전역 상태 ───────────────────────────────────────────────────
raw_center_cur   = RAW_CENTER
neutral_since_ms = None
last_loop_ms     = utime.ticks_ms()

# ─── 유틸 ────────────────────────────────────────────────────────
def get_manual_dz_bounds():
    return (raw_center_cur - DEADZONE, raw_center_cur + DEADZONE)

def set_dir_by_sign(sign):  # sign>0: FORWARD(0), sign<0: REVERSE(1)
    motor_dir.value(0 if sign > 0 else 1)

def in_deadzone_centered(val: int, center: int, dz: int) -> bool:
    return (center - dz) <= val <= (center + dz)

def led_update(led_pin: Pin, code: int, now_ms: int):
    if code <= 0:
        led_pin.value(1)
        return
    slot = now_ms % LED_CYCLE_MS
    on = False
    for k in range(code):
        start = k * 1000
        if start <= slot < start + BLINK_ON_MS:
            on = True
            break
    led_pin.value(1 if on else 0)

def manual_input_valid(v: int) -> bool:
    """
    수동 조종기 ADC가 '말이 되는 값'인지 체크.
    완전 0 근처나 4095 근처면 조종기 탈거/부동 가능성이 높다고 보고 False.
    필요시 50/4045는 실측 보고 좁히면 됨.
    """
    return 50 < v < 4045  # ★ 수동 입력 유효성 판단

def update_center_auto(now_ms: int, m_mode: int, raw_now: int):
    """CENTER_HOLD_AFTER_BOOT=False일 때만 사용(본 버전 기본 고정)."""
    global neutral_since_ms, raw_center_cur
    if USE_FIXED_CENTER:
        return  # 고정 모드일 땐 보정하지 않음
    if CENTER_HOLD_AFTER_BOOT:
        return  # (세션 고정 모드면) 부팅 후 보정치 유지

    if m_mode != 1:
        neutral_since_ms = None
        return
    tight_dz = max(6, int(DEADZONE * TIGHT_DZ_RATIO))
    if in_deadzone_centered(raw_now, raw_center_cur, tight_dz):
        if neutral_since_ms is None:
            neutral_since_ms = now_ms
        elif utime.ticks_diff(now_ms, neutral_since_ms) >= NEUTRAL_HOLD_MS:
            dc = (raw_now - raw_center_cur) * CENTER_EMA_NUM // CENTER_EMA_DEN
            newc = raw_center_cur + dc
            if newc < CENTER_CLAMP_MIN: newc = CENTER_CLAMP_MIN
            if newc > CENTER_CLAMP_MAX: newc = CENTER_CLAMP_MAX
            raw_center_cur = newc
            neutral_since_ms = now_ms
    else:
        neutral_since_ms = None

# ─── PWMReader ───────────────────────────────────────────────────
class PWMReader:
    def __init__(self, pin_num):
        self.pin = Pin(pin_num, Pin.IN)
        self.last_time   = 0
        self.pulse_width = MID_POINT
        self.last_ms     = utime.ticks_ms()
        self.pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                     handler=self._irq)
    def _irq(self, pin):
        t_us = utime.ticks_us()
        t_ms = utime.ticks_ms()
        self.last_ms = t_ms
        if pin.value():
            self.last_time = t_us
        else:
            self.pulse_width = utime.ticks_diff(t_us, self.last_time)

def duty_target_from_diff(diff_us: int, sc_state: int) -> int:
    """
    diff_us: rf_pw_eff - MID_POINT (µs)
    sc_state: 1=저속, 2=고속
    return: 목표 duty (0..255), 단 저속에서는 0..MAX_DUTY_LOW
    """
    if sc_state == 1:
        span_pos = LOW_MAX  - MID_POINT   # 100
        span_neg = MID_POINT - LOW_MIN    # 100
        if diff_us > 0:
            return min((diff_us * MAX_DUTY_LOW) // max(1, span_pos), MAX_DUTY_LOW)
        else:
            a = -diff_us
            return min((a * MAX_DUTY_LOW) // max(1, span_neg), MAX_DUTY_LOW)
    else:
        span_pos_low = LOW_MAX  - MID_POINT   # 100
        span_neg_low = MID_POINT - LOW_MIN    # 100
        if diff_us > 0:
            A = diff_us
            if A <= span_pos_low:
                return (A * MAX_DUTY_LOW) // max(1, span_pos_low)
            else:
                rest_span = max(1, RF_RANGE_POS - span_pos_low)
                rest_duty = 255 - MAX_DUTY_LOW
                return min(MAX_DUTY_LOW + ((A - span_pos_low) * rest_duty) // rest_span, 255)
        else:
            A = -diff_us
            if A <= span_neg_low:
                return (A * MAX_DUTY_LOW) // max(1, span_neg_low)
            else:
                rest_span = max(1, RF_RANGE_NEG - span_neg_low)
                rest_duty = 255 - MAX_DUTY_LOW
                return min(MAX_DUTY_LOW + ((A - span_neg_low) * rest_duty) // rest_span, 255)

# ─── 하드웨어 초기화 ──────────────────────────────────────────────
reader_rf   = PWMReader(RF_PWM_PIN)
reader_safe = PWMReader(SAFETY_PIN)
pwm_adc     = ADC(MANUAL_ADC_PIN)
pwr_adc     = ADC(PWR_ADC_PIN)

motor_pwm   = PWM(Pin(MOTOR_PWM_PIN)); motor_pwm.freq(1000)
motor_dir   = Pin(MOTOR_DIR_PIN, Pin.OUT)
motor_en    = Pin(MOTOR_EN_PIN, Pin.OUT)  # 0=활성, 1=비활성

led         = Pin(LED_PIN, Pin.OUT); led.value(1)
emg_1       = Pin(EMERGENCY_PIN_1, Pin.IN, Pin.PULL_UP)
emg_2       = Pin(EMERGENCY_PIN_2, Pin.IN, Pin.PULL_UP)
mode_pin    = Pin(MODE_PIN, Pin.IN)

prev_mode   = mode_pin.value()
alarm_active= False
last_blink  = utime.ticks_ms()

# ─── 부팅 시 1회 센터 보정 ─────────────────────────────────────────
if USE_FIXED_CENTER:
    raw_center_cur = RAW_CENTER
else:
    sum_v = 0
    cnt_v = 0
    min_v = 4095
    max_v = 0
    t0 = utime.ticks_ms()
    while utime.ticks_diff(utime.ticks_ms(), t0) < BOOT_CENTER_MS:
        v = pwm_adc.read_u16() >> 4
        if v < min_v: min_v = v
        if v > max_v: max_v = v
        sum_v += v; cnt_v += 1
        led_update(led, 1, utime.ticks_ms())  # 보정 중 1번 패턴
        utime.sleep_ms(5)

    if cnt_v > 0 and (max_v - min_v) <= BOOT_CENTER_MAX_DEV:
        raw_center_cur = sum_v // cnt_v
    else:
        raw_center_cur = RAW_CENTER

# 센터 클램프
if raw_center_cur < CENTER_CLAMP_MIN: raw_center_cur = CENTER_CLAMP_MIN
if raw_center_cur > CENTER_CLAMP_MAX: raw_center_cur = CENTER_CLAMP_MAX

# 부팅 직후 위치가 중앙 밖이면 알람 (수동 입력이 유효할 때만)
initial_raw = pwm_adc.read_u16() >> 4
lo, hi = get_manual_dz_bounds()
if manual_input_valid(initial_raw) and not (lo <= initial_raw <= hi):  # 보정
    alarm_active = True
    last_blink = utime.ticks_ms()

# ─── 메인 루프 ───────────────────────────────────────────────────
while True:
    now_ms = utime.ticks_ms()

    # 프레임 dt
    dt_ms = utime.ticks_diff(now_ms, last_loop_ms)
    if dt_ms < 1: dt_ms = 1
    if dt_ms > 200: dt_ms = 200
    last_loop_ms = now_ms

    step_up   = max(1, (DUTY_STEP_UP   * dt_ms) // 100)
    step_down = max(1, (DUTY_STEP_DOWN * dt_ms) // 100)

    # 비상 스위치
    emg_pressed_1 = (emg_1.value() == 1)
    emg_pressed_2 = (emg_2.value() == 1)
    if emg_pressed_1 or emg_pressed_2:
        led_update(led, 1, now_ms)
        motor_en.value(1); motor_pwm.duty_u16(0)
        utime.sleep_ms(50)
        continue

    # 배터리(대략 %)
    raw_pwr = pwr_adc.read_u16() >> 4
    bat_pct = (raw_pwr * 100) // 4095

    # 저전압 컷/해제
    if bat_pct <= BAT_LOW_PCT:
        low_batt_active = True
    elif low_batt_active and bat_pct >= BAT_RECOVER_PCT:
        low_batt_active = False

    # ─ 모드/스위치/쓰로틀 읽기 ─
    m_hw     = mode_pin.value()                 # 하드웨어 MODE (0=RF, 1=Manual)
    safe_pw  = reader_safe.pulse_width
    rf_pw    = reader_rf.pulse_width
    raw      = pwm_adc.read_u16() >> 4

    # SC 상태
    if safe_pw > SC_HIGH_EDGE:
        sc_state = 0
    elif safe_pw >= SC_LOW_EDGE:
        sc_state = 1
    else:
        sc_state = 2

    # RF 신호 생존 여부
    rf_alive = (utime.ticks_diff(now_ms, reader_rf.last_ms) <= RF_TIMEOUT_MS)

    # 수동 입력 유효 여부
    manual_valid = manual_input_valid(raw)

    # ─ 논리 모드 결정 ─
    # 기본은 하드웨어 MODE를 따르되,
    #  - MODE=Manual(1) 이면서
    #  - RF 신호 살아있고
    #  - 수동 ADC가 비정상 → 케이블 탈거로 보고 RF 유지
    m = m_hw
    if m_hw == 1 and rf_alive and not manual_valid:   # 가짜 Manual 방지
        m = 0

    # RF→Manual 전환 시 중앙 외면 알람 (수동 입력 유효할 때만 체크)
    if prev_mode == 0 and m == 1 and manual_valid:
        lo, hi = get_manual_dz_bounds()
        if not (lo <= raw <= hi):
            alarm_active = True
            last_blink = now_ms
    prev_mode = m

    # LED: 1) 알람 2) 저전압(컷 임계와 동기) 3) 항상 ON
    if alarm_active:
        led_code = 2
    elif bat_pct <= BAT_LOW_PCT:
        led_code = 3
    else:
        led_code = 0
    led_update(led, led_code, now_ms)

    # 알람 중엔 중앙 복귀해야 해제
    if alarm_active:
        lo, hi = get_manual_dz_bounds()
        if lo <= raw <= hi:
            alarm_active = False
        else:
            motor_en.value(1); motor_pwm.duty_u16(0)
            utime.sleep_ms(100)
            continue

    # 저전압 컷
    if low_batt_active:
        motor_en.value(1); motor_pwm.duty_u16(0)
        utime.sleep_ms(50)
        continue

    # RF/Manual 모드별 입력 무시 정책
    if sc_state == 0 or m == 1:
        rf_pw = MID_POINT
    if m == 0:
        raw = 2200     # RF 모드에서 수동 ADC는 사용하지 않음

    # ── 모터 제어 ────────────────────────────────────────────────
    if m == 1:
        # Manual: 비례 제어 (센터 고정)
        diff = raw - raw_center_cur
        if abs(diff) <= DEADZONE:
            motor_en.value(1); motor_pwm.duty_u16(0)
        else:
            motor_en.value(0)
            if diff > 0:
                mag  = diff - DEADZONE
                maxr = RAW_MAX - (raw_center_cur + DEADZONE)
                #set_dir_by_sign(+1)
                set_dir_by_sign(-1) # -> 방향 반대면 여기주석하고 위에꺼 풀기
            else:
                mag  = -diff - DEADZONE
                maxr = (raw_center_cur - DEADZONE) - RAW_MIN
                # set_dir_by_sign(-1)
                set_dir_by_sign(+1)

            pct = (mag * MANUAL_MAX_PCT) // max(1, maxr)
            if   pct < 0: pct = 0
            elif pct > MANUAL_MAX_PCT: pct = MANUAL_MAX_PCT
            motor_pwm.duty_u16(pct * 655)

        # (옵션) 연속 보정 모드만 활성
        update_center_auto(now_ms, m, raw)

    else:
        # RF 연결 체크
        if utime.ticks_diff(now_ms, reader_rf.last_ms) > RF_TIMEOUT_MS:
            motor_en.value(1); motor_pwm.duty_u16(0)
            rf_duty_smooth = 0; rf_pw_smooth = MID_POINT
            utime.sleep_ms(100)
            continue

        # SC 아래=정지
        if sc_state == 0:
            motor_en.value(1); motor_pwm.duty_u16(0)
            rf_duty_smooth = 0; rf_pw_smooth = MID_POINT
            utime.sleep_ms(100)
            continue

        motor_en.value(0)  # 저속/고속 포지션에서는 enable (선로 이동 시 외력 대응)

        # 유효 입력(µs) 결정 (저속 구간은 먼저 클램프)
        rf_pw_eff_in = rf_pw
        if sc_state == 1:
            if rf_pw_eff_in < LOW_MIN:   rf_pw_eff_in = LOW_MIN
            elif rf_pw_eff_in > LOW_MAX: rf_pw_eff_in = LOW_MAX

        # 입력 스무딩(dt 비례)
        target_us = rf_pw_eff_in
        delta_us  = target_us - rf_pw_smooth
        allow_us  = max(1, (RAMP_MAX_STEP_US * dt_ms) // 100)
        if   delta_us >  allow_us: delta_us =  allow_us
        elif delta_us < -allow_us: delta_us = -allow_us
        rf_pw_smooth += delta_us

        rf_pw_eff = rf_pw_smooth
        diff_us   = rf_pw_eff - MID_POINT

        # 중립 데드존: 즉시 컷 (감속비 있으므로 역방향 브레이크 없음)
        if -RF_DEADZONE_US <= diff_us <= RF_DEADZONE_US:
            rf_duty_smooth = 0
            motor_pwm.duty_u16(0)
            utime.sleep_ms(50)
            continue

        # 듀티 타깃
        rf_duty_target = duty_target_from_diff(diff_us, sc_state)

        # 방향
        if diff_us > 0:
            set_dir_by_sign(-1)
        else:
            set_dir_by_sign(+1)

        # 램핑
        delta_d = rf_duty_target - rf_duty_smooth
        if   delta_d > 0:  delta_d = min(delta_d, step_up)
        else:              delta_d = max(delta_d, -step_down)
        rf_duty_smooth += delta_d

        # 작은 듀티 컷 & 출력
        if rf_duty_smooth <= NEUTRAL_THRESH:
            rf_duty_smooth = 0
            motor_pwm.duty_u16(0)
        else:
            motor_pwm.duty_u16(rf_duty_smooth * 257)

    utime.sleep_ms(100)

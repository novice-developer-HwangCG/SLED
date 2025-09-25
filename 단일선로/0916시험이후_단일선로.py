from machine import Pin, ADC, PWM
import utime

# 단일 선로 170kg 아음속 감속비 있음
# 단일 선로 최대 펄스 1855, 최소 펄스 1145

# ─── pin map & config ────────────────────────────────────────────
MODE_PIN        = 26    # 0=RF, 1=Manual
SAFETY_PIN      = 22

MANUAL_ADC_PIN  = 27
RF_PWM_PIN      = 21

MOTOR_PWM_PIN   = 15
MOTOR_DIR_PIN   = 14
MOTOR_EN_PIN    = 13

LED_PIN         = 0     # 알람/전원 LED (기본: 항상 ON)
EMERGENCY_PIN_1   = 10    # 비상 스위치 (NC, PULL_UP)
EMERGENCY_PIN_2   = 11    # 비상 스위치_2 (NC, PULL_UP)

PWR_ADC_PIN     = 28    # 배터리 전압 측정 ADC

# Manual ADC 보정
RAW_MIN         = 21
RAW_MAX         = 4095
RAW_CENTER      = 2200

# Manual 모드 최대 출력(%) — 필요시 100으로 올리면 풀스케일 / Manual 모드는 유선으로 연결 되어 있기 때문에 빠른 속도는 불필요
MANUAL_MAX_PCT = 50

# deadzone always 0
DEADZONE        = 200
MANUAL_DZ_LOWER = RAW_CENTER - DEADZONE
MANUAL_DZ_UPPER = RAW_CENTER + DEADZONE

# RF parameters
MID_POINT       = 1500
RF_PW_MIN       = 1145
RF_PW_MAX       = 1855
RF_RANGE_POS    = RF_PW_MAX - MID_POINT
RF_RANGE_NEG    = MID_POINT - RF_PW_MIN
NEUTRAL_THRESH  = 5
RF_DEADZONE_US = 20
RAMP_MAX_STEP_US = 10

DUTY_STEP_UP   = 6    # 가속 시 프레임당 duty 증가 한계
DUTY_STEP_DOWN = 10    # 감속 시 프레임당

# RF용 스무딩 상태변수
rf_pw_smooth = MID_POINT
rf_duty_smooth = 0

# SC(3단) 기준 및 저속 한계
SC_LOW_EDGE  = 1400
SC_HIGH_EDGE = 1900
LOW_MIN      = 1400
LOW_MAX      = 1600
MAX_DUTY_LOW = 128

# RF 연결 타임아웃 (ms)
RF_TIMEOUT_MS   = 200

# LED 패턴 설정
LED_CYCLE_MS = 3000      # 3초 주기
BLINK_ON_MS  = 200       # 깜빡임 ON 유지 시간(200ms)
# code 0=항상ON, 1=1번, 2=2번, 3=3번 (1초 간격으로 점멸)

last_loop_ms = utime.ticks_ms()

def led_update(led_pin: Pin, code: int, now_ms: int):
    if code <= 0:
        led_pin.value(1)  # 항상 켬
        return
    slot = now_ms % LED_CYCLE_MS
    # 1초 간격으로 code번 점멸 (각 각 200ms ON)
    on = False
    for k in range(code):
        start = k * 1000
        if start <= slot < start + BLINK_ON_MS:
            on = True
            break
    led_pin.value(1 if on else 0)

# PWMReader
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
    # 중립 데드존은 바깥에서 처리
    if sc_state == 1:
        # 저속: ±100µs 스팬을 0..MAX_DUTY_LOW로 선형 매핑
        span_pos = LOW_MAX  - MID_POINT   # 1600-1500 = 100
        span_neg = MID_POINT - LOW_MIN    # 1500-1400 = 100
        if diff_us > 0:
            return min((diff_us * MAX_DUTY_LOW) // max(1, span_pos), MAX_DUTY_LOW)
        else:
            a = -diff_us
            return min((a * MAX_DUTY_LOW) // max(1, span_neg), MAX_DUTY_LOW)
    else:
        # 고속: 경계(±100µs)에서 저속 최대(128)과 연속이 되도록 2구간 선형
        # 구간1: |diff| <= 100 => 기울기 = 128/100
        # 구간2: 100 < |diff| <= 전체스팬 => 128 + (|diff|-100) * (127 / (스팬-100))
        span_pos_low = LOW_MAX  - MID_POINT   # 100
        span_neg_low = MID_POINT - LOW_MIN    # 100

        if diff_us > 0:
            A = diff_us
            if A <= span_pos_low:
                return (A * MAX_DUTY_LOW) // max(1, span_pos_low)  # 0..128
            else:
                rest_span = max(1, RF_RANGE_POS - span_pos_low)    # 512-100 = 412
                rest_duty = 255 - MAX_DUTY_LOW                     # 127
                return min(MAX_DUTY_LOW + ((A - span_pos_low) * rest_duty) // rest_span, 255)
        else:
            A = -diff_us
            if A <= span_neg_low:
                return (A * MAX_DUTY_LOW) // max(1, span_neg_low)  # 0..128
            else:
                rest_span = max(1, RF_RANGE_NEG - span_neg_low)
                rest_duty = 255 - MAX_DUTY_LOW
                return min(MAX_DUTY_LOW + ((A - span_neg_low) * rest_duty) // rest_span, 255)

# 초기화
reader_rf   = PWMReader(RF_PWM_PIN)
reader_safe = PWMReader(SAFETY_PIN)
pwm_adc     = ADC(MANUAL_ADC_PIN)
pwr_adc     = ADC(PWR_ADC_PIN)

motor_pwm   = PWM(Pin(MOTOR_PWM_PIN)); motor_pwm.freq(1000)
motor_dir   = Pin(MOTOR_DIR_PIN, Pin.OUT)
motor_en    = Pin(MOTOR_EN_PIN, Pin.OUT)        # 0=활성, 1=비활성

led         = Pin(LED_PIN, Pin.OUT); led.value(1)  # 전원 켜지면 기본 ON
emg_1         = Pin(EMERGENCY_PIN_1, Pin.IN, Pin.PULL_UP)
emg_2         = Pin(EMERGENCY_PIN_2, Pin.IN, Pin.PULL_UP)

prev_mode   = Pin(MODE_PIN, Pin.IN).value()
alarm_active= False     # 와이퍼 중앙 아님 알람
last_blink  = utime.ticks_ms()

# 부팅시 와이퍼 위치 확인 (중앙 데드존 외면 알람)
initial_raw = pwm_adc.read_u16() >> 4
if not (MANUAL_DZ_LOWER <= initial_raw <= MANUAL_DZ_UPPER):
    alarm_active = True
    last_blink   = utime.ticks_ms()
    # LED 점멸은 루프에서 패턴으로 처리
    # print(f"ALARM on BOOT: raw={initial_raw} not center")

# 루프
while True:
    now_ms = utime.ticks_ms()

    # --- 추가: 프레임 시간 계산/클램프 ---
    dt_ms = utime.ticks_diff(now_ms, last_loop_ms)
    if dt_ms < 1: dt_ms = 1
    if dt_ms > 200: dt_ms = 200    # 일시 정지/지연 후에도 과한 점프 방지
    last_loop_ms = now_ms

    step_up   = max(1, (DUTY_STEP_UP   * dt_ms) // 100)
    step_down = max(1, (DUTY_STEP_DOWN * dt_ms) // 100)

    # ── 비상 스위치 (눌리면 즉시 정지)
    emg_pressed_1 = (emg_1.value() == 1)
    emg_pressed_2 = (emg_2.value() == 1)
    if emg_pressed_1:
        # LED: 3초 주기 1번 점멸
        led_update(led, 1, now_ms)
        motor_en.value(1)
        motor_pwm.duty_u16(0)
        #print("EMERGENCY STOP: switch pressed!")
        utime.sleep_ms(50)
        continue
    elif emg_pressed_2:
        # LED: 3초 주기 1번 점멸
        led_update(led, 1, now_ms)
        motor_en.value(1)
        motor_pwm.duty_u16(0)
        #print("EMERGENCY STOP: switch pressed!")
        utime.sleep_ms(50)
        continue

    # ── 배터리 값 (대략 %)  ※ 실제 스케일은 분압 비율 맞춰 보정 해야 핤 수 있는데 대략적으로 해두기
    raw_pwr = pwr_adc.read_u16() >> 4          # 0–4095
    bat_pct = (raw_pwr * 100) // 4095
    
    m        = Pin(MODE_PIN, Pin.IN).value()    # 0=RF, 1=Manual
    safe_pw  = reader_safe.pulse_width
    if safe_pw > SC_HIGH_EDGE:
        sc_state = 0      # 아래=정지
    elif safe_pw >= SC_LOW_EDGE:
        sc_state = 1      # 중간=저속
    else:
        sc_state = 2      # 위=고속
    safe_val = 1 if sc_state != 0 else 0

    rf_pw    = reader_rf.pulse_width
    raw      = pwm_adc.read_u16() >> 4

    # ── RF→Manual 전환 시, 와이퍼 중앙 아니면 알람 ON
    if prev_mode == 0 and m == 1:
        if not (MANUAL_DZ_LOWER <= raw <= MANUAL_DZ_UPPER):
            alarm_active = True
            last_blink   = now_ms
    prev_mode = m

    # ── LED 우선순위
    # 1) 비상(이미 위에서 처리) 2) 저전압 3) 와이퍼 중앙 아님 4) 항상 켜기
    if bat_pct < 20:
        led_code = 3   # 3번 점멸
    elif alarm_active:
        led_code = 2   # 2번 점멸
    else:
        led_code = 0   # 항상 ON
    led_update(led, led_code, now_ms)

    # ── 와이퍼 알람 처리 (알람 중에는 모터 정지, 중앙 복귀 시 해제)
    if alarm_active:
        if MANUAL_DZ_LOWER <= raw <= MANUAL_DZ_UPPER:
            alarm_active = False
            # LED는 위의 led_update에서 자동으로 항상 ON으로 복귀
        else:
            motor_en.value(1)
            motor_pwm.duty_u16(0)
            #print("ALARM: wiper outside deadzone!")
            utime.sleep_ms(100)
            continue

    # RF/Manual 모드별 RF 입력 무시, ADC 무시
    if safe_val == 0 or m == 1:
        rf_pw = MID_POINT
    if m == 0:
        raw = (MANUAL_DZ_LOWER + MANUAL_DZ_UPPER)//2

    # 4) 모터 제어
    rf_duty   = 0
    direction = "NEUTRAL"

    if m==1:
        # manual: map raw_diff to 25/50/75/100
        # Manual: deadzone ±200
        # diff = raw - RAW_CENTER
        # if abs(diff) <= DEADZONE:
        #     motor_en.value(1)
        #     motor_pwm.duty_u16(0)
        # else:
        #     motor_en.value(0)
        #     if diff > 0:
        #         mag  = diff - DEADZONE
        #         maxr = RAW_MAX - (RAW_CENTER + DEADZONE)
        #         motor_dir.value(0)  # 1
        #         direction = "FORWARD"
        #     else:
        #         mag  = -diff - DEADZONE
        #         maxr = (RAW_CENTER - DEADZONE) - RAW_MIN
        #         motor_dir.value(1)  # 0
        #         direction = "REVERSE"
        #     lvl = (mag * 3 + maxr - 1) // maxr
        #     lvl = max(1, min(3, lvl))

        #     # 4) pct 선택: 35, 40, 45
        #     if lvl == 1:
        #         pct = 40
        #     elif lvl == 2:
        #         pct = 45
        #     else:
        #         pct = 50
        #     motor_pwm.duty_u16(pct * 655)
        #     rf_duty = pct

        # Manual: 비례 제어 (deadzone 밖에서 선형 스케일)
        diff = raw - RAW_CENTER
        if abs(diff) <= DEADZONE:
            motor_en.value(1)
            motor_pwm.duty_u16(0)
            direction = "NEUTRAL"
            rf_duty = 0
        else:
            motor_en.value(0)

            if diff > 0:
                # FORWARD
                mag  = diff - DEADZONE
                maxr = RAW_MAX - (RAW_CENTER + DEADZONE)
                motor_dir.value(1)
                direction = "FORWARD"
            else:
                # REVERSE
                mag  = -diff - DEADZONE
                maxr = (RAW_CENTER - DEADZONE) - RAW_MIN
                motor_dir.value(0)
                direction = "REVERSE"

            # 선형 비례: 0..MANUAL_MAX_PCT (%)
            pct = (mag * MANUAL_MAX_PCT) // max(1, maxr)
            if pct < 0: pct = 0
            if pct > MANUAL_MAX_PCT: pct = MANUAL_MAX_PCT

            # PWM 출력 (0..65535). pct% -> duty_u16
            motor_pwm.duty_u16(pct * 655)

            # 디버그/로깅 변수 재사용
            rf_duty = pct

    else:
        # RF 연결 체크
        if utime.ticks_diff(now_ms, reader_rf.last_ms) > RF_TIMEOUT_MS:
            motor_en.value(1)
            motor_pwm.duty_u16(0)
            rf_duty_smooth = 0
            rf_pw_smooth = MID_POINT
            # RF 신호 끊김 시 즉시 정지
            utime.sleep_ms(100)
            continue

        # SC 아래=정지
        if sc_state == 0:
            motor_en.value(1)
            motor_pwm.duty_u16(0)
            rf_duty_smooth = 0
            rf_pw_smooth = MID_POINT
            utime.sleep_ms(100)
            continue

        motor_en.value(0)   # sc 스위치를 저속, 고속에 두면 enable 신호를 넣어서 외력으로 밀 수 있게 만들어두기 (선로 옮기기 용)

        # ── 저속(1400~1600)일 땐 먼저 클램프 ──
        rf_pw_eff_in = rf_pw
        if sc_state == 1:
            if rf_pw_eff_in < LOW_MIN:  rf_pw_eff_in = LOW_MIN
            elif rf_pw_eff_in > LOW_MAX: rf_pw_eff_in = LOW_MAX
        # 고속(sc_state==2)은 제한 없음

        # ── 입력 스무딩(µs) ──
        target_us = rf_pw_eff_in
        delta_us  = target_us - rf_pw_smooth
        if   delta_us >  RAMP_MAX_STEP_US: delta_us =  RAMP_MAX_STEP_US
        elif delta_us < -RAMP_MAX_STEP_US: delta_us = -RAMP_MAX_STEP_US
        rf_pw_smooth += delta_us
        rf_pw_eff = rf_pw_smooth

        # ── 중립 데드존(µs) ──
        diff_us = rf_pw_eff - MID_POINT
        if -RF_DEADZONE_US <= diff_us <= RF_DEADZONE_US:
            # 중립에서는 안전을 위해 즉시 컷 (기존 동작 유지)
            rf_duty_target = 0
            rf_duty_smooth = 0
            motor_pwm.duty_u16(0)
            direction = "NEUTRAL"
            utime.sleep_ms(50)
            continue

        # ── 듀티 타깃 계산(연속 맵핑) ──
        rf_duty_target = duty_target_from_diff(diff_us, sc_state)

        # 방향 결정 (듀티는 절대값 기준)
        if diff_us > 0:
            motor_dir.value(1); direction = "FORWARD"
        else:
            motor_dir.value(0); direction = "REVERSE"

        # ── 듀티 램핑(부드러운 전환 핵심) ──
        delta_d = rf_duty_target - rf_duty_smooth
        if   delta_d > 0:   # 가속
            delta_d = min(delta_d, step_up)
        else:               # 감속
            delta_d = max(delta_d, -step_down)
        rf_duty_smooth += delta_d

        # ── 아주 작은 듀티 컷 & 출력 ──
        if rf_duty_smooth <= NEUTRAL_THRESH:
            rf_duty_smooth = 0
            motor_pwm.duty_u16(0)
            direction = "NEUTRAL"
        else:
            motor_en.value(0)
            motor_pwm.duty_u16(rf_duty_smooth * 257)

    # 디버그
    # sc_label = ("STOP" if sc_state==0 else ("LOW_SET" if sc_state==1 else "HIGH_SET"))
    # print(f"MODE:{'RF' if m==0 else 'MANUAL'}",
    #       f"| SC:{sc_label}",
    #       f"| BAT:{bat_pct:3d}%",
    #       f"| ADC:{raw:4d}",
    #       f"| RF_pw:{rf_pw:4d}us",
    #       f"| PWM_duty:{rf_duty:3d}",
    #       f"| DIR:{direction}")
    utime.sleep_ms(100)
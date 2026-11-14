#pragma once
/*
 * hw_config.h - everything that is specific to YOUR build lives here.
 *
 * Target board : ESP32-WROOM-32D 30-pin dev module ("ESP32 Dev Module" in Arduino IDE)
 * Wheel drivers: 2x DRV8313 (3-PWM), wheel motors: 2805 gimbal BLDC
 * Encoders     : 2x AS5600 (I2C)      IMU: MPU6050
 * Leg servos   : 2x hobby PWM servos  Battery: 3S LiPo
 *
 * Works with arduino-esp32 core 2.0.x and core 3.x (the LEDC / MAC / timer API differences
 * are handled with ESP_ARDUINO_VERSION_MAJOR checks). On core 3.x you also need a
 * SimpleFOC version that supports it.
 */

/* ===================== Serial ===================== */
// 921600 is a good compromise: fast enough that Serial prints do not stall the
// balance loop, slow enough for CP2102/CH340 boards. Set the Serial Monitor to
// the same baud rate AND to "Newline" line ending (needed for Commander).
#define SERIAL_BAUD 921600

/* ===================== I2C ===================== */
// Matches the real wiring of this robot (same arrangement as the original design):
//  I2C_LEFT  bus (I2Cone) : encoder of the LEFT wheel  = motor 1 (driver on 16/17/18). Encoder only.
//  I2C_RIGHT bus (I2Ctwo) : encoder of the RIGHT wheel = motor 2 (driver on 23/25/26) AND the MPU6050.
// The pairing driver <-> encoder is what matters: a motor can only be aligned with the encoder on ITS shaft.
#define I2C_LEFT_SDA 33
#define I2C_LEFT_SCL 32
#define I2C_RIGHT_SDA 21
#define I2C_RIGHT_SCL 22
#define MPU_INT_PIN 34  // wired but NOT used by this firmware (the library polls)

// I2C speed. setup() probes the MPU6050 at this speed first and automatically falls back
// to 200 kHz and then 100 kHz if it does not answer. The speed that works is used for both buses.
#define I2C_CLOCK_HZ 400000UL
// 1 = do not arm the motors if the MPU6050 is not found (safe default),
// 0 = continue anyway (only for testing legs / the web page without an IMU).
#define IMU_REQUIRED 1

/* ===================== DRV8313 wheel drivers ===================== */
// Motor 1 = LEFT wheel, Motor 2 = RIGHT wheel
#define DRV_L_IN1 16
#define DRV_L_IN2 17
#define DRV_L_IN3 18
#define DRV_L_EN 4  // EN1/EN2/EN3 of the left DRV8313 must be tied together to this pin
#define DRV_L_NFAULT 35

#define DRV_R_IN1 23
#define DRV_R_IN2 25
#define DRV_R_IN3 26
#define DRV_R_EN 19  // EN1/EN2/EN3 of the right DRV8313 tied together to this pin
#define DRV_R_NFAULT 39

#define DRV_NSLEEP 27  // shared nSLEEP of both drivers, HIGH = awake, LOW = kill switch
                       // (nRESET of both DRV8313 must be tied HIGH on the PCB)

// nFAULT is open-drain / active-low. GPIO34/35/39 have NO internal pull-up, so you
// need an external ~10k pull-up to 3.3V on each nFAULT line. Leave this at 0 until
// the boot log shows both pins idling HIGH, then set it to 1.
#define FAULT_MONITOR_ENABLED 0

/* ===================== Wheel motors (2805 gimbal) ===================== */
#define MOTOR_POLE_PAIRS 7  // 14-pole gimbal motors = 7 pole pairs. VERIFY on your motor.

// Supply voltage the drivers are fed with. 3S LiPo: 11.1 V nominal (12.6 full, ~9.9 empty).
#define DRIVER_SUPPLY_V 11.1f

// IMPORTANT: in this SimpleFOC version the torque limit is the DRIVER voltage limit.
// With sine modulation the maximum q-axis voltage that can be applied is about
// DRIVER_VOLTAGE_LIMIT / 2.  (The original 2S design used 8 V -> ~4 V of drive.)
// Rule of thumb for current: I ~= (DRIVER_VOLTAGE_LIMIT/2) / (R_line_to_line / 2).
// Keep that under ~1.5 A (DRV8313 continuous rating is lower than its peak rating).
// Start low for first tests, raise later. Can be changed at runtime with:  V6
#define DRIVER_VOLTAGE_LIMIT 8.0f

#define MOTOR_ALIGN_VOLTAGE 4.0f  // voltage used for the sensor alignment at boot

// Flip these (1 -> -1) if a wheel pushes the WRONG way when the robot tilts.
#define MOTOR1_DIR_FLIP 1
#define MOTOR2_DIR_FLIP 1

/* ===================== IMU orientation ===================== */
// Flip to -1 if the sign of the angle is wrong for how you mounted the MPU6050.
// Pitch: leaning FORWARD must make LQR_angle more positive/negative consistently
// with the wheel direction (see tuning guide step 4).
// The firmware expects the MPU6050 chip's X axis to run FRONT-to-BACK and its Y axis LEFT-to-RIGHT
// (Z up). If tilting the robot front/back moves the legs (roll) instead of driving the wheels
// (pitch), your board is turned 90 degrees: set this to 1 and the X/Y axes are swapped in software.
#define IMU_SWAP_XY 1
#define IMU_PITCH_SIGN 1
#define IMU_ROLL_SIGN -1
// Steering sign. Motor 1 (driver on 16/17/18, encoder on GPIO33/32) is the LEFT wheel, as in the original
// robot, so this stays 1. If the bench steering test shows the wheels HELPING a hand twist instead of
// resisting it (the robot tries to spin away), set it to -1.
#define YAW_OUTPUT_SIGN 1

/* ===================== Leg servos (hobby PWM servos) ===================== */
#define SERVO_L_PIN 13  // left servo : 180 deg = short(retracted), 140 deg = fully extended
#define SERVO_R_PIN 14  // right servo:   0 deg = short(retracted),  40 deg = fully extended

#define SERVO_L_LEDC_CH 4  // LEDC channels 4/5 (share one timer, both 50 Hz)
#define SERVO_R_LEDC_CH 5
#define SERVO_PWM_FREQ 50
#define SERVO_PWM_BITS 14

// Pulse width that corresponds to 0 deg and 180 deg of YOUR servo.
// 500..2500 us suits most MG90S/MG996R/DS-type servos. If the servo buzzes at the
// ends or your angles are off, calibrate these two numbers (see tuning guide).
#define SERVO_US_MIN 500
#define SERVO_US_MAX 2500

// Leg "extension" is in degrees of servo travel measured from fully short.
//   extension 0  -> right servo 0 deg,   left servo 180 deg
//   extension 40 -> right servo 40 deg,  left servo 140 deg
#define LEG_EXT_MIN_DEG 0.0f
#define LEG_EXT_NORMAL_MAX_DEG 20.0f  // normal driving / height slider uses 0..20 deg only
#define LEG_EXT_JUMP_MAX_DEG 40.0f    // 20..40 deg is reserved for jumping

// Per-leg fine trim in degrees (positive = more extension). Use it so that both
// legs have the same real height when the extension command is the same.
#define LEG_TRIM_L_DEG 0.0f
#define LEG_TRIM_R_DEG 0.0f

// The web UI height slider goes 32..85. This maps the whole slider onto 0..20 deg.
#define LEG_UI_H_MIN 32
#define LEG_UI_H_MAX 85

// Max leg speed in normal (non-jump) operation. Hobby servos are far faster than the
// original smart servos, so without this the roll loop would make the legs twitch.
#define LEG_SLEW_DEG_PER_S 90.0f

// Roll (body levelling) loop. Output is in DEGREES of leg extension now (the original
// code output servo steps). Sign: flip IMU_ROLL_SIGN if the wrong leg extends.
#define ROLL_P_DEFAULT 0.7f
#define ROLL_COMP_LIMIT_DEG 10.0f

/* ===================== Jump (time based, was loop-count based) ===================== */
#define JUMP_ENABLED 1
#define JUMP_PUSH_MS 150      // time legs stay fully extended (40 deg)
#define JUMP_LAND_EXT_DEG 5.0f  // crouch used right after the push to absorb the landing
#define JUMP_TOTAL_MS 700     // total time before another jump is allowed

/* ===================== Known quirk of the original code ===================== */
// The original tried to clear the integral of pid_lqr_u by writing error_prev = 0, but
// the integral itself (integral_prev) was never cleared, so it just accumulates.
// The reference robot was tuned WITH this behaviour, so it stays 0 (original behaviour).
// Set to 1 to really clear the integral whenever the small-torque compensator switches off
// (test it on a tied-down robot first).
#define FIX_LQR_INTEGRAL_RESET 0

/* ===================== Battery sense (optional) ===================== */
// You did not list a battery divider, so this is OFF. To enable: wire the battery
// through a divider (e.g. 47k top / 10k bottom -> ratio 5.7) to GPIO36 and set to 1.
// Never connect 12 V directly to an ESP32 pin.
#define BAT_SENSE_ENABLED 0
#define BAT_SENSE_PIN 36
#define BAT_DIVIDER_RATIO 5.7f
#define BAT_NOMINAL_V 11.1f   // reported when sensing is off
#define BAT_EMPTY_V 9.9f      // 3.3 V per cell
#define BAT_FULL_V 12.6f
#define LED_BAT -1            // no status LED fitted (original used GPIO13 = your servo pin!)

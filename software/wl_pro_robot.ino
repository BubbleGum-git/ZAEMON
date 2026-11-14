//Robot controlled header file
#include <MPU6050_tockn.h>
#include "hw_config.h"
#include "leg_servo.h"

// IMU axis mapping (see IMU_SWAP_XY in hw_config.h)
#if IMU_SWAP_XY
#define IMU_PITCH_ANGLE() ((float)mpu6050.getAngleX())
#define IMU_PITCH_GYRO() ((float)mpu6050.getGyroX())
#define IMU_ROLL_ANGLE() ((float)mpu6050.getAngleY())
#else
#define IMU_PITCH_ANGLE() ((float)mpu6050.getAngleY())
#define IMU_PITCH_GYRO() ((float)mpu6050.getGyroY())
#define IMU_ROLL_ANGLE() ((float)mpu6050.getAngleX())
#endif
#include <SimpleFOC.h>
#include <Arduino.h>

//WiFi transmission header file
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <WebServer.h>
#include <WiFi.h>
#include <FS.h>
#include "basic_web.h"
#include "robot.h"
#include "wl_wifi.h"
#include "ble.h"
#include "EEPROM.h"
#include "eeprom_util.h"
#include "feedback_util.h"
#include "web_socket_client_util.h"


/************Instance definition*************/

//Electromotor instance
BLDCMotor motor1 = BLDCMotor(MOTOR_POLE_PAIRS);  // LEFT wheel
BLDCMotor motor2 = BLDCMotor(MOTOR_POLE_PAIRS);  // RIGHT wheel
BLDCDriver3PWM driver1 = BLDCDriver3PWM(DRV_L_IN1, DRV_L_IN2, DRV_L_IN3, DRV_L_EN);
BLDCDriver3PWM driver2 = BLDCDriver3PWM(DRV_R_IN1, DRV_R_IN2, DRV_R_IN3, DRV_R_EN);

//Encoder instance
TwoWire I2Cone = TwoWire(0);
TwoWire I2Ctwo = TwoWire(1);
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

//PID instance
PIDController pid_angle(1, 0, 0, 100000, 8);
PIDController pid_gyro(0.06, 0, 0, 100000, 8);
PIDController pid_distance(0, 0, 0, 100000, 8);
PIDController pid_speed(0.7, 0, 0, 100000, 8);
PIDController pid_yaw_angle(1.0, 0, 0, 100000, 8);
PIDController pid_yaw_gyro(0.04, 0, 0, 100000, 8);
// PID whose integral can really be cleared (integral_prev is protected in the library)
class ResettablePID : public PIDController {
public:
  ResettablePID(float P, float I, float D, float ramp, float limit) : PIDController(P, I, D, ramp, limit) {}
  void reset() { integral_prev = 0.0f; output_prev = 0.0f; error_prev = 0.0f; }
  void reset_error_prev() { error_prev = 0.0f; }  // what the original code did (does NOT clear the integral)
};
ResettablePID pid_lqr_u(1, 15, 0, 100000, 8);
PIDController pid_zeropoint(0.002, 0, 0, 100000, 4);
PIDController pid_roll_angle(ROLL_P_DEFAULT, 0, 0, 100000, ROLL_COMP_LIMIT_DEG);  // output = degrees of leg extension

//Low pass filter instance
LowPassFilter lpf_joyy(0.2);
LowPassFilter lpf_zeropoint(0.1);
LowPassFilter lpf_roll(0.3);

// Commander instance
Commander command = Commander(Serial);

void StabAngle(char* cmd) {
  command.pid(&pid_angle, cmd);
}
void StabGyro(char* cmd) {
  command.pid(&pid_gyro, cmd);
}
void StabDistance(char* cmd) {
  command.pid(&pid_distance, cmd);
}
void StabSpeed(char* cmd) {
  command.pid(&pid_speed, cmd);
}
void StabYawAngle(char* cmd) {
  command.pid(&pid_yaw_angle, cmd);
}
void StabYawGyro(char* cmd) {
  command.pid(&pid_yaw_gyro, cmd);
}
void lpfJoyy(char* cmd) {
  command.lpf(&lpf_joyy, cmd);
}
void StabLqrU(char* cmd) {
  command.pid(&pid_lqr_u, cmd);
}
void StabZeropoint(char* cmd) {
  command.pid(&pid_zeropoint, cmd);
}
void lpfZeropoint(char* cmd) {
  command.lpf(&lpf_zeropoint, cmd);
}
void StabRollAngle(char* cmd) {
  command.pid(&pid_roll_angle, cmd);
}
void lpfRoll(char* cmd) {
  command.lpf(&lpf_roll, cmd);
}

bool sitdown_falg = false;

//void Stabtest_zeropoint(char* cmd) { command.pid(&test_zeropoint, cmd); }

//WebServer instance
WebServer webserver;                                // server
WebSocketsServer websocket = WebSocketsServer(81);  // Define a webSocket server to process messages sent by clients
int joystick_value[2];

//MPU6050 instance
MPU6050 mpu6050(I2Ctwo);  // MPU6050 shares the RIGHT I2C bus (GPIO21/22) with the right wheel encoder (AS5600 #2)

/************parameter definition*************/
#define pi 3.1415927

//LQR Self-balancing controller parameters
float LQR_angle = 0;
float LQR_gyro = 0;
float LQR_speed = 0;
float LQR_distance = 0;
float angle_control = 0;
float gyro_control = 0;
float speed_control = 0;
float distance_control = 0;
float LQR_u = 0;
float angle_zeropoint = 12.6;  // set it for YOUR robot with the serial command "Z" (hold it balanced first)
float distance_zeropoint = -256.0;  //Wheel position shift zero offset \
 (-256 is an impossible displacement value, use it as a sign that it is not refreshed)


//YAW axis control data
float YAW_gyro = 0;
float YAW_angle = 0;
float YAW_angle_last = 0;
float YAW_angle_total = 0;
float YAW_angle_zero_point = -10;
float YAW_output = 0;

//Logical processing flag bit
float robot_speed = 0;          //Record the current wheel speed
float robot_speed_last = 0;     //Record the wheel speed at the previous time
int wrobot_move_stop_flag = 0;  //Record the sign that the rocker stops
int jump_flag = 0;              //Jump period identification
float leg_position_add = 0;     //roll axis balance control quantity (degrees of leg extension)


//Driver state / extra runtime features
bool drivers_armed = false;
bool imu_ok = false;
bool telemetry_on = false;
uint32_t loop_counter = 0;
uint32_t loop_hz = 0;
unsigned long jump_start_ms = 0;
float speed_p_base = 0.0f;  // base P of pid_speed (set with serial command M)

void user_function(char* cmd) {
  if (*cmd == '{') {
    rp.json_test(cmd);
  }
}

// ---- Driver arm / kill (DRV8313 nSLEEP is the kill switch) ----
void motors_arm() {
  digitalWrite(DRV_NSLEEP, HIGH);
  delay(10);  // DRV8313 wake-up time
  drivers_armed = true;
  Serial.println("DRIVERS ARMED (nSLEEP high)");
}
void motors_kill() {
  motor1.target = 0;
  motor2.target = 0;
  digitalWrite(DRV_NSLEEP, LOW);  // both DRV8313 outputs go high-impedance
  drivers_armed = false;
  Serial.println("DRIVERS KILLED (nSLEEP low). Send Y to re-arm.");
}
void check_driver_faults() {
#if FAULT_MONITOR_ENABLED
  static uint8_t fault_cnt = 0;
  if (!drivers_armed) return;
  bool fault = (digitalRead(DRV_L_NFAULT) == LOW) || (digitalRead(DRV_R_NFAULT) == LOW);
  fault_cnt = fault ? (uint8_t)(fault_cnt + 1) : 0;
  if (fault_cnt >= 10) {  // ~10 consecutive loops = debounce
    fault_cnt = 0;
    motors_kill();
    Serial.println("DRV8313 nFAULT asserted (over-current / over-temp / under-voltage). Check wiring and supply.");
  }
#endif
}

// Real integral clear for the small-torque compensator (see hw_config.h)
static inline void lqr_integral_reset() {
#if FIX_LQR_INTEGRAL_RESET
  pid_lqr_u.reset();
#else
  pid_lqr_u.reset_error_prev();  // original behaviour: does NOT clear the accumulated integral
#endif
}

// ---- Serial commands (send with "Newline" line ending) ----
void cmdSetZero(char* cmd) {  // Z : hold the robot balanced by hand, send Z
  angle_zeropoint = LQR_angle;
  Serial.print("angle_zeropoint set to ");
  Serial.println(angle_zeropoint, 3);
}
void cmdSpeedP(char* cmd) {  // M0.7 : base P of the speed loop (height scaling is applied on top)
  float p = atof(cmd);
  if (p >= 0.0f && p < 10.0f) {
    speed_p_base = p;
    Serial.print("speed P base = ");
    Serial.println(p);
  }
}
void cmdKill(char* cmd) { motors_kill(); }
void cmdArm(char* cmd) { motors_arm(); }
void cmdVoltage(char* cmd) {  // V6 : set DRIVER voltage limit (max drive ~ half of it)
  float v = atof(cmd);
  if (v >= 1.0f && v <= DRIVER_SUPPLY_V) {
    driver1.voltage_limit = v;
    driver2.voltage_limit = v;
    Serial.print("driver voltage_limit = ");
    Serial.println(v);
  } else {
    Serial.println("V needs a value between 1 and the supply voltage");
  }
}
void cmdTelemetry(char* cmd) {  // T1 on / T0 off  (Arduino Serial Plotter friendly)
  telemetry_on = (atoi(cmd) != 0);
}
void cmdStatus(char* cmd) {  // S : print a state summary
  Serial.printf("armed=%d loop=%luHz zero=%.3f ang=%.2f vlim=%.1f h=%d extL=%.1f extR=%.1f roll_add=%.2f jump=%d unctrl=%d\n",
                drivers_armed, (unsigned long)loop_hz, angle_zeropoint, LQR_angle - angle_zeropoint, driver1.voltage_limit,
                wrobot.height, legs.extL(), legs.extR(), leg_position_add, jump_flag, rp.uncontrollable);
}

// ---- I2C diagnostics and MPU6050 start-up ----
static void i2c_scan(TwoWire& bus, const char* name) {
  Serial.print("I2C scan, ");
  Serial.print(name);
  Serial.print(":");
  bool any = false;
  for (uint8_t a = 1; a < 127; a++) {
    bus.beginTransmission(a);
    if (bus.endTransmission() == 0) {
      Serial.printf(" 0x%02X", a);
      any = true;
    }
  }
  Serial.println(any ? "" : " (nothing found)");
}

static bool mpu_probe() {
  I2Ctwo.beginTransmission(0x68);
  I2Ctwo.write(0x75);  // WHO_AM_I register
  if (I2Ctwo.endTransmission(false) != 0) return false;
  if (I2Ctwo.requestFrom((uint8_t)0x68, (uint8_t)1) != 1) return false;
  uint8_t id = I2Ctwo.read();
  Serial.printf("MPU6050 answered, WHO_AM_I = 0x%02X (0x68 expected)\n", id);
  return true;
}

static void imu_setup() {
  static const uint32_t clocks[3] = { I2C_CLOCK_HZ, 200000UL, 100000UL };
  uint32_t last = 0xFFFFFFFFUL;
  for (uint8_t i = 0; i < 3 && !imu_ok; i++) {
    uint32_t clk = clocks[i];
    if (clk >= last) continue;  // only try speeds slower than the previous attempt
    last = clk;
    I2Cone.setClock(clk);
    I2Ctwo.setClock(clk);
    delay(20);
    for (uint8_t t = 0; t < 3 && !imu_ok; t++) {
      imu_ok = mpu_probe();
      if (!imu_ok) delay(30);
    }
    if (imu_ok) Serial.printf("MPU6050 OK at I2C clock %lu Hz\n", (unsigned long)clk);
  }
  if (!imu_ok) {
    Serial.println("ERROR: MPU6050 not answering at 0x68 (tried the fast, 200 kHz and 100 kHz clocks).");
    Serial.println("       Check SDA/SCL (not XDA/XCL), VCC, GND and AD0 = GND.");
    return;
  }
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);  // keep the robot STILL
}

void setup() {
  // 1) Put the wheel drivers to sleep FIRST so nothing can move while we boot
  pinMode(DRV_NSLEEP, OUTPUT);
  digitalWrite(DRV_NSLEEP, LOW);
  pinMode(DRV_L_NFAULT, INPUT);  // external pull-ups required (no internal pull on 34/35/39)
  pinMode(DRV_R_NFAULT, INPUT);

  Serial.begin(SERIAL_BAUD);

  // 2) Legs: fully short at power-up
  legs.begin();

  rp.get_pcb_version();
  rp.config_json_init();
  ble_init();
  wifi_init();
  if (get_wifi_state() != WIFI_STATE.CLOSE) {
    webserver.begin();
    webserver.on("/", HTTP_GET, basicWebCallback);
    websocket.begin();
    websocket.onEvent(webSocketEventCallback);
  }

  web_sockets_client_init();

#if BAT_SENSE_ENABLED
  //Voltage detection
  analogSetPinAttenuation(BAT_SENSE_PIN, ADC_11db);
#else
  rp.battery_voltage = BAT_NOMINAL_V;
#endif
  if (LED_BAT >= 0) pinMode(LED_BAT, OUTPUT);

  // Encoder setup: left bus = left wheel encoder only, right bus = right wheel encoder + MPU6050
  I2Cone.begin(I2C_LEFT_SDA, I2C_LEFT_SCL, I2C_CLOCK_HZ);
  I2Ctwo.begin(I2C_RIGHT_SDA, I2C_RIGHT_SCL, I2C_CLOCK_HZ);
  sensor1.init(&I2Cone);
  sensor2.init(&I2Ctwo);
  // SimpleFOC's init() calls Wire.begin() again, which can reset the bus speed
  I2Cone.setClock(I2C_CLOCK_HZ);
  I2Ctwo.setClock(I2C_CLOCK_HZ);

  i2c_scan(I2Cone, "LEFT wheel bus (GPIO33/32), expect 0x36");
  i2c_scan(I2Ctwo, "RIGHT wheel bus + MPU (GPIO21/22), expect 0x36 and 0x68");

  //mpu6050 setup: probes the chip, picks a working I2C speed, then calibrates the gyro
  imu_setup();
  while (!imu_ok && IMU_REQUIRED) {  // drivers are still asleep (nSLEEP low): the robot stays inert
    Serial.println("Motors NOT armed because the IMU is missing. Fix the wiring and press the EN/reset button.");
    delay(2000);
  }

  //Connect the motor object to the encoder object
  motor1.linkSensor(&sensor1);
  motor2.linkSensor(&sensor2);

  //Speed loop PID parameter (unused in torque/voltage mode, kept from the original)
  motor1.PID_velocity.P = 0.05;
  motor1.PID_velocity.I = 1;
  motor1.PID_velocity.D = 0;

  motor2.PID_velocity.P = 0.05;
  motor2.PID_velocity.I = 1;
  motor2.PID_velocity.D = 0;

  // Motor driver setup (3S LiPo). In this SimpleFOC version the DRIVER voltage limit
  // is what caps the torque (max q-voltage ~ voltage_limit / 2).
  motor1.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;
  motor2.voltage_sensor_align = MOTOR_ALIGN_VOLTAGE;
  driver1.voltage_power_supply = DRIVER_SUPPLY_V;
  driver2.voltage_power_supply = DRIVER_SUPPLY_V;
  driver1.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver2.voltage_limit = DRIVER_VOLTAGE_LIMIT;
  driver1.init();
  driver2.init();

  //Connect the motor object to the drive object
  motor1.linkDriver(&driver1);
  motor2.linkDriver(&driver2);

  motor1.torque_controller = TorqueControlType::voltage;
  motor2.torque_controller = TorqueControlType::voltage;
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;

  // Motor-related Settings
  motor1.useMonitoring(Serial);
  motor2.useMonitoring(Serial);

  // 3) Wake the DRV8313s only now, right before the motors are initialised.
  //    The wheels will twitch/rotate a little during FOC alignment: keep them free.
  motors_arm();

  motor1.init();
  motor1.initFOC();
  motor2.init();
  motor2.initFOC();

  rp.m1_direction = (motor1.sensor_direction == CW ? -0.5f : 0.5f) * MOTOR1_DIR_FLIP;
  rp.m2_direction = (motor2.sensor_direction == CCW ? -0.5f : 0.5f) * MOTOR2_DIR_FLIP;

  Serial.println("Motor1 direction:" + String(rp.m1_direction));
  Serial.println("Motor2 direction:" + String(rp.m2_direction));
  Serial.printf("nFAULT idle levels  L=%d R=%d  (must read 1 when idle; if 0 add 10k pull-ups to 3.3V)\n",
                digitalRead(DRV_L_NFAULT), digitalRead(DRV_R_NFAULT));

  // Map motor to commander
  command.add('A', StabAngle, "pid angle");
  command.add('B', StabGyro, "pid gyro");
  command.add('C', StabDistance, "pid distance");
  command.add('D', StabSpeed, "pid speed");
  command.add('E', StabYawAngle, "pid yaw angle");
  command.add('F', StabYawGyro, "pid yaw gyro");
  command.add('G', lpfJoyy, "lpf joyy");
  command.add('H', StabLqrU, "pid lqr u");
  command.add('I', StabZeropoint, "pid zeropoint");
  command.add('J', lpfZeropoint, "lpf zeropoint");
  command.add('K', StabRollAngle, "pid roll angle (deg of leg extension per deg of roll)");
  command.add('L', lpfRoll, "lpf roll");
  command.add('U', user_function, "user function");
  command.add('M', cmdSpeedP, "speed loop base P (D command is overridden by the height table)");
  command.add('Z', cmdSetZero, "set balance zero point from current angle");
  command.add('X', cmdKill, "kill drivers (nSLEEP low)");
  command.add('Y', cmdArm, "re-arm drivers");
  command.add('V', cmdVoltage, "driver voltage limit");
  command.add('T', cmdTelemetry, "telemetry on/off (T1/T0)");
  command.add('S', cmdStatus, "status");

  delay(500);
  Serial.println("Ready. Send S for status, T1 for telemetry, Z to set the zero point.");
}

void loop() {
  loop_counter++;

  if (one_second_tick()) {
    bat_check();  //Voltage detection (only if BAT_SENSE_ENABLED)
    wifi_loop();
    loop_hz = loop_counter;  //balance-loop rate, shown by the S command (aim for > 400 Hz)
    loop_counter = 0;
  }
  if (ten_msec_tick()) {
    ble_loop();
    bat_led_blink();  //Voltage indication LED (disabled when LED_BAT < 0)
    rp.test_log_output();

    static uint8_t telemetry_div = 0;  //20 Hz telemetry for the Arduino Serial Plotter
    if (telemetry_on && ++telemetry_div >= 5) {
      telemetry_div = 0;
      Serial.printf("ang:%.2f,gyro:%.1f,u:%.2f,spd:%.2f,dist:%.2f,roll_add:%.2f,extL:%.1f,extR:%.1f,pitch:%.1f,roll:%.1f\n",
                    LQR_angle - angle_zeropoint, LQR_gyro, LQR_u, LQR_speed, LQR_distance - distance_zeropoint,
                    leg_position_add, legs.extL(), legs.extR(), IMU_PITCH_ANGLE(), IMU_ROLL_ANGLE());
    }
  }

  web_loop();  //Web data update

  mpu6050.update();    //IMU data update
  lqr_balance_loop();  //lqr self-balancing control
  yaw_loop();          //yaw axis steering control
  leg_loop();          //Leg motion control
  check_driver_faults();

  //The self-balancing calculated output torque is assigned to the motor
  motor1.target = (LQR_u + YAW_OUTPUT_SIGN * YAW_output) * rp.m1_direction;
  motor2.target = (LQR_u - YAW_OUTPUT_SIGN * YAW_output) * rp.m2_direction;

  //Shut down output after falling out of control (angle measured from the balance zero point)
  if (abs(LQR_angle - angle_zeropoint) > rp.uncontrollable_angle) {
    rp.uncontrollable = 1;
  }
  if (rp.uncontrollable != 0)  //Delay recovery after lifting
  {
    if (abs(LQR_angle - angle_zeropoint) < rp.recovery_angle) {
      rp.uncontrollable++;
    }
    if (rp.uncontrollable > 200)  //The delay time of 200 program cycles
    {
      rp.uncontrollable = 0;
    }
  }

  if (wrobot.go == 0) {
    robot_sitdown();
  } else {
    sitdown_falg = false;
  }

  //Turn off output (remote control stop or Angle is too large out of control)
  if (rp.uncontrollable != 0) {
    sitdown_falg = true;
    motor1.target = 0;
    motor2.target = 0;
    leg_position_add = 0;
    wrobot.joyx = 0;
    wrobot.joyy = 0;
    wrobot.roll = 0;
    wrobot.height = LEG_UI_H_MIN;
  }

  //Record the last remote control data
  wrobot.dir_last = wrobot.dir;
  wrobot.joyx_last = wrobot.joyx;
  wrobot.joyy_last = wrobot.joyy;

  //The FOC phase voltage is computed iteratively
  motor1.loopFOC();
  motor2.loopFOC();

  //Set the wheel motor output
  motor1.move();
  motor2.move();

  command.run();
}

//lqr self-balancing control
void lqr_balance_loop() {
  /*
  * LQR balance formula, in order to facilitate the adjustment of parameters in actual use, 
  * the formula is decomposed into 4 P controls, 
  * using the PIDController method in commander real-time debugging
  */
  //QR_u = LQR_k1*(LQR_angle - angle_zeropoint) + LQR_k2*LQR_gyro + LQR_k3*(LQR_distance - distance_zeropoint) + LQR_k4*LQR_speed;

  //The negative value is given because, according to the current motor wiring, the positive torque will turn backwards
  LQR_distance = motor1.shaft_angle*rp.m1_direction  + motor2.shaft_angle*rp.m2_direction;
  LQR_speed    = motor1.shaft_velocity*rp.m1_direction  + motor2.shaft_velocity*rp.m2_direction;

  LQR_angle = IMU_PITCH_SIGN * IMU_PITCH_ANGLE();
  LQR_gyro = IMU_PITCH_SIGN * IMU_PITCH_GYRO();
  angle_control = pid_angle(LQR_angle - angle_zeropoint);
  gyro_control = pid_gyro(LQR_gyro);

  //Motion detail optimization processing
  if (wrobot.joyy != 0)  //Handling when there are forward and backward direction motion instructions
  {
    distance_zeropoint = LQR_distance;  //Displacement zero reset
    lqr_integral_reset();               //The output integral is cleared to zero
  }

  if ((wrobot.joyx_last != 0 && wrobot.joyx == 0) || (wrobot.joyy_last != 0 && wrobot.joyy == 0))  //Stop in place processing when motion instruction returns to zero
  {
    wrobot_move_stop_flag = 1;
  }
  if ((wrobot_move_stop_flag == 1) && (abs(LQR_speed) < 0.5)) {
    distance_zeropoint = LQR_distance;  //Displacement zero reset
    wrobot_move_stop_flag = 0;
  }

  if (abs(LQR_speed) > 15)  //Stop in place treatment when being pushed rapidly
  {
    distance_zeropoint = LQR_distance;  //Displacement zero reset
  }

  //Calculate displacement control output
  distance_control = pid_distance(LQR_distance - distance_zeropoint);
  speed_control = pid_speed(LQR_speed - 0.1 * lpf_joyy(wrobot.joyy));

  //Wheel lift detection
  robot_speed_last = robot_speed;  //Record two consecutive wheel speeds
  robot_speed = LQR_speed;
  if (abs(robot_speed - robot_speed_last) > 10 || abs(robot_speed) > 50 || (jump_flag != 0)) /*If the angular speed and angular acceleration of the 
                                                    wheel are too large or in the recovery period after jumping, 
                                                    it is considered that the wheel is off the 
                                                    ground and needs special treatment.
                                                  */
  {
    distance_zeropoint = LQR_distance;     //Displacement zero point reset
    LQR_u = angle_control + gyro_control;  //When the wheel part is off the ground, \
                                          the quantity of the opposite wheel part is not output.\
                                          Conversely, under normal conditions, the balanced torque is fully output
    lqr_integral_reset();                  //The output integral is reset to zero
  } else {
    LQR_u = angle_control + gyro_control + distance_control + speed_control;
  }

  //Trigger conditions: No signal input from the remote control, \
    normal intervention of the wheel position movement control, \
    and not in the recovery period after jumping
  if (abs(LQR_u) < 5 && wrobot.joyy == 0 && abs(distance_control) < 4 && (jump_flag == 0)) {

    LQR_u = pid_lqr_u(LQR_u);  //Compensate for the nonlinearity of small torque
    angle_zeropoint -= pid_zeropoint(lpf_zeropoint(distance_control));  //Center of gravity adaptive
  } else {
    lqr_integral_reset();  //The output integral is reset to zero
  }

  //The balance control parameters are adaptive
  //(the original hard-coded 0.7 / 0.6 / 0.5 here, which silently overwrote any serial "D" tuning;
  // now the base value is set with the serial command M and scaled by height)
  if (wrobot.height < 50) {
    pid_speed.P = speed_p_base;
  } else if (wrobot.height < 64) {
    pid_speed.P = speed_p_base * 0.857f;
  } else {
    pid_speed.P = speed_p_base * 0.714f;
  }
}
void robot_sitdown()
{
  if(rp.uncontrollable != 0)
  { 
    return;
  }
  
  float _angle = LQR_angle - angle_zeropoint;
  //当前没有坐下，调整角度
  if(  _angle > (-1) && sitdown_falg == false )
  {
    wrobot.joyy = -100;
  }
  //此时身体已经为后仰，可以关闭动力，然后自然坐下
  else{
    sitdown_falg = true;
    motor1.target = 0;
    motor2.target = 0;
    leg_position_add = 0;
    wrobot.joyx = 0;
    wrobot.joyy = 0;
    wrobot.roll = 0;
    wrobot.height = LEG_UI_H_MIN;
  }
}
//Leg movement control (hobby PWM servos, extension in degrees)
void leg_loop() {
  jump_loop();
  if (jump_flag == 0)  //Not in a jumping state
  {
    //Body levelling: roll error (deg) -> differential leg extension (deg).
    //Right side down must EXTEND the right leg and RETRACT the left one; if it is
    //the other way round flip IMU_ROLL_SIGN in hw_config.h
    float roll_angle = IMU_ROLL_SIGN * IMU_ROLL_ANGLE() + rp.offset_roll;
    leg_position_add = pid_roll_angle(lpf_roll(roll_angle));

    //Height slider (32..85) -> 0..20 deg normal range; 20..40 deg is jump only
    float h = leg::heightToExt(wrobot.height);
    float extL = leg::clampf(h - leg_position_add, LEG_EXT_MIN_DEG, LEG_EXT_NORMAL_MAX_DEG);
    float extR = leg::clampf(h + leg_position_add, LEG_EXT_MIN_DEG, LEG_EXT_NORMAL_MAX_DEG);
    legs.setTarget(extL, extR);
  }
  legs.update();  //slew-rate limited servo output
}

//Jump control (time based: the original counted loop iterations, which made the
//timing depend on how fast the loop happened to run)
void jump_loop() {
#if JUMP_ENABLED
  if ((wrobot.dir_last == STOP) && (wrobot.dir == JUMP) && (jump_flag == 0) && (rp.uncontrollable == 0)) {
    legs.setTarget(LEG_EXT_JUMP_MAX_DEG, LEG_EXT_JUMP_MAX_DEG, true);  //full extension, full servo speed
    jump_start_ms = millis();
    jump_flag = 1;
  }
  if (jump_flag != 0) {
    unsigned long t = millis() - jump_start_ms;
    if (jump_flag == 1 && t >= JUMP_PUSH_MS) {
      legs.setTarget(JUMP_LAND_EXT_DEG, JUMP_LAND_EXT_DEG, true);  //crouch to absorb the landing
      jump_flag = 2;
    }
    if (t >= JUMP_TOTAL_MS) {
      jump_flag = 0;  //Ready to jump again
    }
  }
#endif
}

//yaw axis steering control
void yaw_loop() {
  //YAW_output = 0.03*(YAW_Kp*YAW_angle_total + YAW_Kd*YAW_gyro);
  yaw_angle_addup();

  YAW_angle_total += wrobot.joyx * 0.002;
  float yaw_angle_control = pid_yaw_angle(YAW_angle_total);
  float yaw_gyro_control = pid_yaw_gyro(YAW_gyro);
  YAW_output = yaw_angle_control + yaw_gyro_control;

}

//Web Data Update
void web_loop() {
  webserver.handleClient();
  websocket.loop();
  web_sockets_client_loop();  // The websocket client continuously makes requests.
  rp.spinOnce();              //Update the control information returned by the web end
}

//The yaw axis Angle accumulation function
void yaw_angle_addup() {
  YAW_angle = (float)mpu6050.getAngleZ();
  YAW_gyro = (float)mpu6050.getGyroZ();

  if (YAW_angle_zero_point == (-10)) {
    YAW_angle_zero_point = YAW_angle;
  }

  float yaw_angle_1, yaw_angle_2, yaw_addup_angle;
  if (YAW_angle > YAW_angle_last) {
    yaw_angle_1 = YAW_angle - YAW_angle_last;
    yaw_angle_2 = YAW_angle - YAW_angle_last - 2 * PI;
  } else {
    yaw_angle_1 = YAW_angle - YAW_angle_last;
    yaw_angle_2 = YAW_angle - YAW_angle_last + 2 * PI;
  }

  if (abs(yaw_angle_1) > abs(yaw_angle_2)) {
    yaw_addup_angle = yaw_angle_2;
  } else {
    yaw_addup_angle = yaw_angle_1;
  }

  YAW_angle_total = YAW_angle_total + yaw_addup_angle;
  YAW_angle_last = YAW_angle;
}

void basicWebCallback(void) {
  Serial.printf("GET / received, free heap: %u, largest block: %u\n",
                ESP.getFreeHeap(), ESP.getMaxAllocHeap());
  webserver.send_P(200, "text/html", basic_web);
}

void webSocketEventCallback(uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
  if (type == WStype_TEXT) {
    String payload_str = String((char*)payload);
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, payload_str);

    String mode_str = doc["mode"];
    if (mode_str == "basic") {
      rp.parseBasic(doc);
    }
  }
}
//Voltage detection (3S LiPo through a resistor divider on BAT_SENSE_PIN)
void bat_check() {
#if BAT_SENSE_ENABLED
  static uint16_t bat_check_num = 2;
  if (bat_check_num > 2) {
    uint32_t voltage = analogReadMilliVolts(BAT_SENSE_PIN);  // calibrated by the core
    rp.battery_voltage = (voltage * BAT_DIVIDER_RATIO) / 1000.0;
    if (rp.battery_voltage < BAT_EMPTY_V) {
      Serial.println("WARNING: battery low, stop and recharge (3S LiPo)");
    }
    bat_check_num = 0;
  } else {
    bat_check_num++;
  }
#endif
}
void bat_led_blink() {
  if (LED_BAT < 0) return;  // no LED fitted
  // Blink pattern:
  // - voltage <= BAT_EMPTY_V : LED off (insufficient)
  // - voltage > BAT_FULL_V - 0.6 : 5 blinks (full)
  // - in between : linear map to 1..4 blinks
  // Sequence: short on/off blinks (200ms on, 200ms off) repeated N times, then pause between sequences.
  static uint8_t target_blinks = 0;       // how many blinks to emit in the next sequence
  static uint8_t blink_index = 0;         // current blink number in the running sequence
  static bool in_sequence = false;        // are we currently emitting a sequence
  static bool led_on = false;             // current LED state within sequence
  static unsigned long last_ts = 0;       // last timestamp for toggle
  static unsigned long seq_pause_ts = 0;  // timestamp when sequence finished (for pause timing)

  const unsigned long blink_on_ms = 200;
  const unsigned long blink_off_ms = 200;
  const unsigned long pause_between_sequences_ms = 1500;

  // determine target_blinks from voltage
  float v = rp.battery_voltage;
  if (v <= BAT_EMPTY_V) {
    target_blinks = 0; // off
  } else if (v > BAT_FULL_V - 0.6f) {
    target_blinks = 5; // full
  } else {
    float ratio = (v - BAT_EMPTY_V) / ((BAT_FULL_V - 0.6f) - BAT_EMPTY_V); // 0..1
    // map ratio to 1..4
    uint8_t mapped = 1 + (uint8_t)(ratio * 4.0f);
    if (mapped > 4) mapped = 4;
    target_blinks = mapped;
  }

  unsigned long now = millis();

  // if no blinks required, ensure LED off and reset state
  if (target_blinks == 0) {
    digitalWrite(LED_BAT, LOW);
    in_sequence = false;
    blink_index = 0;
    led_on = false;
    seq_pause_ts = now;
    return;
  }

  // If not in sequence, wait for pause then start
  if (!in_sequence) {
    if (now - seq_pause_ts >= pause_between_sequences_ms) {
      in_sequence = true;
      blink_index = 0;
      led_on = false;
      last_ts = now;
    } else {
      digitalWrite(LED_BAT, LOW);
      return;
    }
  }

  // In sequence: toggle LED according to durations
  if (!led_on) {
    // currently off; time to turn on for next blink?
    if (now - last_ts >= blink_off_ms) {
      // start ON period
      digitalWrite(LED_BAT, HIGH);
      led_on = true;
      last_ts = now;
    }
  } else {
    // currently on; check if ON period elapsed
    if (now - last_ts >= blink_on_ms) {
      digitalWrite(LED_BAT, LOW);
      led_on = false;
      last_ts = now;
      blink_index++;
      if (blink_index >= target_blinks) {
        // finished sequence
        in_sequence = false;
        seq_pause_ts = now;
        blink_index = 0;
      }
    }
  }
}

//Generate a one-second tick
bool one_second_tick(void) {
  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();
 
  if (currentMillis - lastMillis >= 1000) {
    lastMillis = currentMillis;
    return 1;
  }
  //Overflow handling
  if (lastMillis > currentMillis) {
    lastMillis = currentMillis;
  }

  return 0;
}
//milliseconds
bool ten_msec_tick(void) {
  static unsigned long lastMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - lastMillis >= 10) {
    lastMillis = currentMillis;
    return 1;
  }
  //Overflow handling
  if (lastMillis > currentMillis) {
    lastMillis = currentMillis;
  }

  return 0;
}

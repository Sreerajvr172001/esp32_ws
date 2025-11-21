#include <Arduino.h>

// --- Motor Pin Definitions ---
#define MOTOR_RIGHT_IN1 25
#define MOTOR_RIGHT_IN2 26
#define MOTOR_RIGHT_ENA 27

#define MOTOR_LEFT_IN1 12
#define MOTOR_LEFT_IN2 13
#define MOTOR_LEFT_ENA 14

// --- PWM parameters ---
const int pwmFreq = 1000;
const int pwmResolution = 8;
const int pwmChannel1 = 0;
const int pwmChannel2 = 1;

// --- Encoder Pins ---
#define LEFT_ENC_PIN_A 32
#define LEFT_ENC_PIN_B 33
#define RIGHT_ENC_PIN_A 22
#define RIGHT_ENC_PIN_B 23

// --- Encoder Counters ---
volatile long left_enc_pos = 0;
volatile long right_enc_pos = 0;
volatile long curr_count_left = 0;
volatile long curr_count_right = 0;

volatile long prev_count_left = 0;
volatile long prev_count_right = 0;

volatile float measured_speed_left = 0;
volatile float measured_speed_right = 0;

// --- State Variables ---
volatile float setpoint_ticks_l = 0;
volatile float setpoint_ticks_r = 0;
unsigned long last_millis_cmd = 0;

// PID parameters
struct PIDController
{
  float Kp;
  float Ki;
  float Kd;
  float prev_error;
  float integral;
  float output;
};

PIDController pid_left = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};
PIDController pid_right = {1.0, 0.0, 0.0, 0.0, 0.0, 0.0};

const float CONTROL_INTERVAL = 0.02; //50Hz
long last_control_time = 0;

// Serial interface
HardwareSerial SERIAL_PORT(2);  //#define SERIAL_PORT Serial2

#define SERIAL_BAUD 115200
#define SERIAL2_RX 16 
#define SERIAL2_TX 17 

// Watchdog timeout (ms)
const unsigned long WATCHDOG_MS = 2000;

// Mapping ticks/sec range
const float MAX_TICKS_PER_SEC = 2000;
const float ENCODER_TICKS_PER_REV = 1056;

const int min_pwm = 180;
const int max_pwm = 255;

// Command buffer
String rxbuf = "";

// ISR handlers
void IRAM_ATTR handleLeftEncoder() {
  if (digitalRead(LEFT_ENC_PIN_B) == HIGH) left_enc_pos++;
  else left_enc_pos--;
}

void IRAM_ATTR handleRightEncoder() {
  if (digitalRead(RIGHT_ENC_PIN_B) == HIGH) right_enc_pos++;
  else right_enc_pos--;
}

void updateMeasuredSpeeds()
{
  noInterrupts();
  curr_count_left = left_enc_pos;
  curr_count_right = right_enc_pos;
  interrupts();
  measured_speed_left = (float)(curr_count_left - prev_count_left) / CONTROL_INTERVAL;
  measured_speed_right = (float)(curr_count_right - prev_count_right) / CONTROL_INTERVAL;

  prev_count_left = curr_count_left;
  prev_count_right = curr_count_right;
}

// PWM setup
void setup_pwm_pins() {
  ledcSetup(pwmChannel1, pwmFreq, pwmResolution);
  ledcAttachPin(MOTOR_RIGHT_ENA, pwmChannel1);

  ledcSetup(pwmChannel2, pwmFreq, pwmResolution);
  ledcAttachPin(MOTOR_LEFT_ENA, pwmChannel2);
}

float computePID(PIDController &pid, float target, float measured)
{
  float dt = CONTROL_INTERVAL;
  float error = target - measured;
  pid.integral += error * dt;
  if(pid.integral > MAX_TICKS_PER_SEC) pid.integral = MAX_TICKS_PER_SEC;
  if(pid.integral < -MAX_TICKS_PER_SEC) pid.integral = -MAX_TICKS_PER_SEC;
  float derivative = (error - pid.prev_error) / dt;
  pid.output = pid.Kp * error + pid.Ki * pid.integral + pid.Kd * derivative;
  pid.prev_error = error;
  if(pid.output > MAX_TICKS_PER_SEC) pid.output = MAX_TICKS_PER_SEC;
  if(pid.output < -MAX_TICKS_PER_SEC) pid.output = -MAX_TICKS_PER_SEC;
  return pid.output;
}

void apply_pwm_to_motor(int pwm, int in1, int in2, int ch) {
  
  if (pwm > 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    ledcWrite(ch, pwm); 
  } else if (pwm < 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    ledcWrite(ch, -pwm); 
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    ledcWrite(ch, 0); 
  }
}

int ticks_to_pwm(float ticks) 
{
  if (fabsf(ticks) < 1.0f) return 0;

  float ratio = (float)ticks / MAX_TICKS_PER_SEC;
  int p = (int)round(ratio * max_pwm);

  // Clamp pwm to the min and max range
  if (p > max_pwm) p = max_pwm;
  if (p < -max_pwm) p = -max_pwm;

  // If pwm is too small but non-zero, force it to min_pwm to ensure motion
  if (p > 0 && p < min_pwm) p = min_pwm;
  if (p < 0 && p > -min_pwm) p = -min_pwm;

  return p;
}


void process_command(const String &cmd) {
  String s = cmd;
  s.trim();
  if (s.length() == 0) return;
  char c = s.charAt(0);

  if (c == 'm') 
  {
    SERIAL_PORT.print("OK\n");
    SERIAL_PORT.flush();    
    float l = 0, r = 0;
    int num = sscanf(s.c_str(), "m %f %f", &l, &r);
    if (num == 2) 
    {
      if(l > MAX_TICKS_PER_SEC) l = MAX_TICKS_PER_SEC;
      if(l < -MAX_TICKS_PER_SEC) l = -MAX_TICKS_PER_SEC;
      if(r > MAX_TICKS_PER_SEC) r = MAX_TICKS_PER_SEC;
      if(r < -MAX_TICKS_PER_SEC) r = -MAX_TICKS_PER_SEC;
      noInterrupts();
      setpoint_ticks_l = l;
      setpoint_ticks_r = r;
      interrupts();
      last_millis_cmd = millis();
      //int pwml = ticks_to_pwm(l);
      //int pwr = ticks_to_pwm(r);
      //Serial.printf("Setpoint_L: %f  Setpoint_R: %f", l, r);
      //Serial.println();
      //apply_pwm_to_motor(pwml, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, pwmChannel2);
      //apply_pwm_to_motor(pwr, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, pwmChannel1);
      
    }
  } 
  else if (c == 'e') 
  {
    int l, r;
    noInterrupts();
    l = left_enc_pos;
    r = right_enc_pos;
    interrupts();

    SERIAL_PORT.printf("%d %d\n", l, r);
    SERIAL_PORT.flush();

    // Also print to ESP32 USB Serial monitor for debugging
    //Serial.print("Encoder counts - Left: ");
    //Serial.print(l);
    //Serial.print(" Right: ");
    //Serial.println(r);

  } 
  else if (c == 'r') 
  {
    noInterrupts();
    left_enc_pos = 0;
    right_enc_pos = 0;
    interrupts();
  } 
  else if (c == 'l') 
  {
    float Kp=0, Ki=0, Kd=0;
    int num = sscanf(s.c_str(), "l %f %f %f", &Kp, &Ki, &Kd);
    if (num >= 1) 
    {
      pid_left.Kp = Kp;
      if (num >= 2) 
      {
        pid_left.Ki = Ki;
      } 
      if (num >= 3) 
      {
        pid_left.Kd = Kd;
      }
      SERIAL_PORT.printf("LEFT PID UPDATED: %.3f, %.3f, %.3f\r\n", Kp, Ki, Kd);
    }
  } 
  else if (c == 'n') 
  {
    float Kp=0, Ki=0, Kd=0;
    int num = sscanf(s.c_str(), "n %f %f %f", &Kp, &Ki, &Kd);
    if (num >= 1) 
    {
      pid_right.Kp = Kp;
      if (num >= 2) 
      {
        pid_right.Ki = Ki;
      } 
      if (num >= 3) 
      {
        pid_right.Kd = Kd;
      }
      SERIAL_PORT.printf("RIGHT PID UPDATED: %.3f, %.3f, %.3f\r\n", Kp, Ki, Kd);
    }
  } 
  else if (c == 'p') 
  {
    SERIAL_PORT.print("OK\r\n");
  }
}


void setup() {
  Serial.begin(115200);
  SERIAL_PORT.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL2_RX, SERIAL2_TX);

  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);

  pinMode(LEFT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_PIN_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_PIN_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_PIN_A), handleLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_PIN_A), handleRightEncoder, RISING);

  setup_pwm_pins();

  apply_pwm_to_motor(0, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, pwmChannel2);
  apply_pwm_to_motor(0, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, pwmChannel1);

  last_millis_cmd = 0;
  SERIAL_PORT.print("OK\r\n");
}

void loop() {
  // Read and process incoming commands
  while (SERIAL_PORT.available()) {
    char c = SERIAL_PORT.read();

    if (c == '\r' || c == '\n') {
      if (rxbuf.length() > 0) {
        // Print complete command to USB Serial monitor
        //Serial.print("Received: ");
        //Serial.println(rxbuf);

        // Process the command
        process_command(rxbuf);
        rxbuf = "";
      }
    } else {
      rxbuf += c;
      // Prevent buffer overflow
      if (rxbuf.length() > 200) rxbuf = "";
    }
  }

  // Watchdog: stop motors if no command received for a while
  if (last_millis_cmd != 0 && (millis() - last_millis_cmd) > WATCHDOG_MS) {
    last_millis_cmd = 0;
    apply_pwm_to_motor(0, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, pwmChannel2);
    apply_pwm_to_motor(0, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, pwmChannel1);
  }

  // --- PID Control Loop ---
  unsigned long now = millis();
  if (now - last_control_time >= CONTROL_INTERVAL * 1000) 
  {
    last_control_time = now;

    updateMeasuredSpeeds();

    // Compute PID outputs
    float output_left = computePID(pid_left, setpoint_ticks_l, measured_speed_left);
    float output_right = computePID(pid_right, setpoint_ticks_r, measured_speed_right);
    
    // Convert PID outputs to PWM values
    int pwm_left = ticks_to_pwm(output_left);
    int pwm_right = ticks_to_pwm(output_right);

    // Apply PWM to motors
    apply_pwm_to_motor(pwm_left, MOTOR_LEFT_IN1, MOTOR_LEFT_IN2, pwmChannel2);
    apply_pwm_to_motor(pwm_right, MOTOR_RIGHT_IN1, MOTOR_RIGHT_IN2, pwmChannel1);

    // Debugging output
    Serial.printf("Setpoints L:%f R:%f | Measured L:%f R:%f | PWM L:%d R:%d\n", 
                  setpoint_ticks_l, setpoint_ticks_r, 
                  measured_speed_left, measured_speed_right, 
                  pwm_left, pwm_right);
  }
  
}



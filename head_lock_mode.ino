#include <Wire.h>
#include <Adafruit_MPU6050.h> 
#include <MadgwickAHRS.h>     

// --- MPU SETUP ---
Adafruit_MPU6050 mpu;
Madgwick filter;

// --- MOTOR PINS (Your Configuration) ---
const int ENA = 10; // PWM motor A (Left)
const int IN1 = 8;
const int IN2 = 9;

const int ENB = 11; // PWM motor B (Right)
const int IN3 = 7;
const int IN4 = 6;

// --- PID CONSTANTS (Heading Lock Control) ---
// **TUNING REQUIRED: Start with these values and adjust Kp first.**
float Kp = 5.0;     // Higher Kp is needed for stationary stabilization
float Ki = 0.01;    
float Kd = 0.5;     // Higher Kd helps prevent oscillation
const float YAW_DEAD_BAND = 1.0; // Stop motors if error is within +/- 1 degree

// --- MOVEMENT PARAMETERS ---
float TARGET_YAW_CALIBRATED = 0.0; // The measured heading set during calibration
const int BASE_SPEED = 0;          // **Crucial: Base speed is ZERO for stability**
const int MAX_PWM = 255;
const int MIN_MOTOR_PWM = 50;      // Minimum speed required to overcome motor friction

// --- PID STATE VARIABLES (Yaw Control) ---
float yawError = 0;
float integralError = 0;
float lastError = 0;
float pidCorrection = 0; 

// --- TIMING AND CALIBRATION VARIABLES ---
unsigned long lastTime = 0;
float dt = 0.005; // Loop time (5 milliseconds)
const unsigned long CALIBRATION_DURATION = 5000; // 5 seconds
bool isCalibrated = false;

// --- CALIBRATION VARIABLES ---
float yaw_target_sum = 0.0; // Accumulator for yaw to find reference
int calibrationCount = 0;     

// Sensor readings
float gx_dps, gy_dps, gz_dps; 
float ax_g, ay_g, az_g;     

// =======================================================
// --- PID CONTROL FUNCTION (MODULAR) ---
// =======================================================

static inline float calculatePID(float currentError)
{
    float P = Kp * currentError;

    integralError += currentError * dt;
    // Anti-windup limit
    integralError = constrain(integralError, -100.0, 100.0);
    float I = Ki * integralError;

    float diff = (currentError - lastError) / dt;
    float D = Kd * diff;

    lastError = currentError;

    return P + I + D;
}

// =======================================================
// --- MOTOR CONTROL FUNCTIONS (Forward & Backward) ---
// =======================================================

void motorA_forward(int pwm) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, constrain(pwm, 0, MAX_PWM));
}
void motorA_backward(int pwm) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, constrain(pwm, 0, MAX_PWM));
}

void motorB_forward(int pwm) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, constrain(pwm, 0, MAX_PWM));
}
void motorB_backward(int pwm) {
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, constrain(pwm, 0, MAX_PWM));
}

void stopMotors() {
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
}

// =======================================================
// --- SETUP ---
// =======================================================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  stopMotors();

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found! Check wiring.");
    while (1);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  filter.begin(1.0 / dt); 
  
  Serial.println("MPU6050 connection successful.");
  Serial.println("Starting 5-second CALIBRATION. Set bot facing desired straight line.");

  lastTime = millis();
}

// =======================================================
// --- LOOP ---
// =======================================================

void loop() {
  // --- 1. Timing and Data Acquisition ---
  unsigned long now = millis();
  
  if (now - lastTime < (dt * 1000)) {
    return;
  }
  lastTime = now;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Convert readings
  gx_dps = g.gyro.x * 57.295779;
  gz_dps = g.gyro.z * 57.295779;
  ax_g = a.acceleration.x / 9.80665;
  ay_g = a.acceleration.y / 9.80665;
  az_g = a.acceleration.z / 9.80665;
  
  // Update the Madgwick filter
  filter.updateIMU(gx_dps, 0, gz_dps, ax_g, ay_g, az_g); // Gyro Y set to 0 to minimize noise, focus on Yaw (Z) and Roll/Pitch
  float currentYaw = filter.getYaw(); 

  // --- Range Enforcement: Yaw in range -180 to 180 degrees ---
  if (currentYaw > 180) currentYaw -= 360;
  if (currentYaw < -180) currentYaw += 360;

  // --- 2. Calibration Phase (First 5 seconds) ---
  if (!isCalibrated) {
    stopMotors(); 
    if (now < CALIBRATION_DURATION) {
      yaw_target_sum += currentYaw;
      calibrationCount++;
      return; 
    } else {
      TARGET_YAW_CALIBRATED = yaw_target_sum / calibrationCount;
      Serial.print("Calibration complete. Target Yaw Heading Setpoint: ");
      Serial.println(TARGET_YAW_CALIBRATED, 4);
      Serial.println("Heading Lock is now active. Bot is stationary, ready to correct tilt.");
      isCalibrated = true;
    }
  }

  // --- 3. PID Control Calculation ---
  
  yawError = TARGET_YAW_CALIBRATED - currentYaw; 
  
  // Check for wrap-around error 
  if (yawError > 180.0) {
      yawError -= 360.0;
  } else if (yawError < -180.0) {
      yawError += 360.0;
  }

  // --- 4. Motor Actuation (Heading Lock Logic) ---
  
  // Deadband: If the error is small, stop both motors.
  if (abs(yawError) < YAW_DEAD_BAND) {
      stopMotors();
      integralError = 0; // Reset integral term to prevent windup
      Serial.println("Stable/Stopped (Deadband)");
      return;
  }

  pidCorrection = calculatePID(yawError);

  // Correction magnitude is constrained to MAX_PWM, floored at MIN_MOTOR_PWM
  int motorSpeed = constrain(abs((int)pidCorrection), MIN_MOTOR_PWM, MAX_PWM);
  
  // If PID Correction is required, we only run ONE motor to pivot back.
  
  if (pidCorrection > 0) {
    // Error is Positive: Bot turned Left/CCW (CurrentYaw is too low). 
    // We need to turn RIGHT (CW). This requires the LEFT wheel (Motor A) to run FORWARD.
    motorA_forward(motorSpeed);
    analogWrite(ENB, 0); // Stop Motor B (Right)
  } else {
    // Error is Negative: Bot turned Right/CW (CurrentYaw is too high).
    // We need to turn LEFT (CCW). This requires the RIGHT wheel (Motor B) to run FORWARD.
    analogWrite(ENA, 0); // Stop Motor A (Left)
    motorB_backward(motorSpeed);
  }
  
  // --- 5. Debugging ---
  Serial.print("Yaw: "); Serial.print(currentYaw, 2);
  Serial.print(" | Target: "); Serial.print(TARGET_YAW_CALIBRATED, 2);
  Serial.print(" | Error: "); Serial.print(yawError, 2);
  Serial.print(" | Corr Mag: "); Serial.print(motorSpeed);
  Serial.print(" | Action: ");
  Serial.println(pidCorrection > 0 ? "Turn RIGHT (Motor A FWD)" : "Turn LEFT (Motor B FWD)");
}

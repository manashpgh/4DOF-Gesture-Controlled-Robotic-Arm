#include <Wire.h>
#include <MPU6050.h>
#include <WiFi.h>
#include <PubSubClient.h>

const char* ssid = "Redmi Note 9 Pro";
const char* password = "12345678";
const char* mqtt_server = "10.89.120.186";
const int mqtt_port = 1883;
const char* mqtt_topic = "robot/arm/control";

WiFiClient espClient;
PubSubClient client(espClient);

// Two MPU6050 sensors with different addresses
MPU6050 mpu1(0x68);  // Sensor 1 - Controls CH0 (roll) and CH3 (pitch)
MPU6050 mpu2(0x69);  // Sensor 2 - Controls CH1 (roll) and CH2 (pitch)

// HOME POSITION CONSTANTS
const int HOME_POSITION = 90;  // FIXED: All servos go to 90 degrees at home
const int HOME_MOVE_DELAY = 15;  // Delay between steps for smooth movement (ms)

// Sensor 1 variables (controls CH0 and CH3 via roll and pitch)
int16_t ax1, ay1, az1, gx1, gy1, gz1;
float roll1;      // Controls Servo 1 (CH0)
float pitch1;     // Controls Servo 4 (CH3)
float rollOffset1 = 0;
float pitchOffset1 = 0;
float compRoll1 = 0;
float compPitch1 = 0;
float smoothRoll1 = 90;
float smoothPitch1 = 90;

// Sensor 2 variables (controls CH1 and CH2 via roll and pitch)
int16_t ax2, ay2, az2, gx2, gy2, gz2;
float roll2;      // Controls Servo 2 (CH1)
float pitch2;     // Controls Servo 3 (CH2)
float rollOffset2 = 0;
float pitchOffset2 = 0;
float compRoll2 = 0;
float compPitch2 = 0;
float smoothRoll2 = 90;
float smoothPitch2 = 90;

// Current servo positions
int currentServo1 = HOME_POSITION;  // CH0 - controlled by Sensor 1 roll
int currentServo2 = HOME_POSITION;  // CH1 - controlled by Sensor 2 roll
int currentServo3 = HOME_POSITION;  // CH2 - controlled by Sensor 2 pitch
int currentServo4 = HOME_POSITION;  // CH3 - controlled by Sensor 1 pitch

// System state
bool systemActive = false;

#define SDA_PIN 21
#define SCL_PIN 22

// Complementary filter parameters
const float alpha = 0.96;
const float smoothFactor = 0.15;

// For timing
unsigned long lastTime1 = 0;
unsigned long lastTime2 = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  
  Serial.println("\n=================================");
  Serial.println("Dual MPU6050 - 4 Servo Control");
  Serial.println("With Home Position Feature");
  Serial.println("=================================");
  
  // Initialize Sensor 1 (0x68) - Controls CH0 and CH3
  Serial.print("MPU6050 Sensor 1 (0x68): ");
  mpu1.initialize();
  if (!mpu1.testConnection()) {
    Serial.println("FAILED!");
    Serial.println("Check connections!");
    while (1);
  }
  Serial.println("OK ✓");
  
  mpu1.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu1.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu1.setDLPFMode(MPU6050_DLPF_BW_20);
  
  // Initialize Sensor 2 (0x69) - Controls CH1 and CH2
  Serial.print("MPU6050 Sensor 2 (0x69): ");
  mpu2.initialize();
  if (!mpu2.testConnection()) {
    Serial.println("FAILED!");
    Serial.println("⚠ Check if AD0 pin is connected to 3.3V");
    while (1);
  }
  Serial.println("OK ✓");
  
  mpu2.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  mpu2.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu2.setDLPFMode(MPU6050_DLPF_BW_20);
  
  Serial.println("\nBoth sensors connected!");
  
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  
  // MOVE TO HOME POSITION ON STARTUP
  Serial.println("\n>>> Moving to HOME position...");
  moveToHomePosition();
  Serial.println(">>> Home position reached!");
  
  calibrateIMU();
  
  systemActive = true;  // Activate system after calibration
  lastTime1 = millis();
  lastTime2 = millis();
  
  Serial.println("\n=================================");
  Serial.println("System Ready!");
  Serial.println("Sensor 1 Roll  → Servo 1 (CH0)");
  Serial.println("Sensor 1 Pitch → Servo 4 (CH3)");
  Serial.println("Sensor 2 Roll  → Servo 2 (CH1)");
  Serial.println("Sensor 2 Pitch → Servo 3 (CH2)");
  Serial.println("Home Position: 90° for all servos");
  Serial.println("=================================\n");
}

void setup_wifi() {
  Serial.print("Connecting to WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Connecting to MQTT...");
    String clientId = "ESP32-Dual-" + String(random(0xffff), HEX);
    
    if (client.connect(clientId.c_str())) {
      Serial.println("Connected ✓");
    } else {
      Serial.print("Failed, rc=");
      Serial.println(client.state());
      delay(5000);
    }
  }
}

// Function to move servos to home position smoothly
void moveToHomePosition() {
  if (!client.connected()) {
    reconnect();
  }
  
  // Calculate steps needed for smooth movement
  int maxSteps = max(max(abs(HOME_POSITION - currentServo1), 
                         abs(HOME_POSITION - currentServo2)),
                     max(abs(HOME_POSITION - currentServo3),
                         abs(HOME_POSITION - currentServo4)));
  
  if (maxSteps == 0) {
    // Already at home position
    publishServoPositions(HOME_POSITION, HOME_POSITION, HOME_POSITION, HOME_POSITION);
    return;
  }
  
  // Move smoothly to home position
  for (int step = 0; step <= maxSteps; step++) {
    int servo1 = currentServo1 + (HOME_POSITION - currentServo1) * step / maxSteps;
    int servo2 = currentServo2 + (HOME_POSITION - currentServo2) * step / maxSteps;
    int servo3 = currentServo3 + (HOME_POSITION - currentServo3) * step / maxSteps;
    int servo4 = currentServo4 + (HOME_POSITION - currentServo4) * step / maxSteps;
    
    publishServoPositions(servo1, servo2, servo3, servo4);
    
    Serial.print("Moving to home: S1=");
    Serial.print(servo1);
    Serial.print("° S2=");
    Serial.print(servo2);
    Serial.print("° S3=");
    Serial.print(servo3);
    Serial.print("° S4=");
    Serial.print(servo4);
    Serial.println("°");
    
    delay(HOME_MOVE_DELAY);
  }
  
  // Ensure final position is exactly home
  currentServo1 = HOME_POSITION;
  currentServo2 = HOME_POSITION;
  currentServo3 = HOME_POSITION;
  currentServo4 = HOME_POSITION;
  publishServoPositions(HOME_POSITION, HOME_POSITION, HOME_POSITION, HOME_POSITION);
}

// Function to publish servo positions via MQTT
void publishServoPositions(int servo1, int servo2, int servo3, int servo4) {
  if (!client.connected()) {
    reconnect();
  }
  
  String message = "{\"s1\":" + String(servo1) + 
                   ",\"s2\":" + String(servo2) + 
                   ",\"s3\":" + String(servo3) + 
                   ",\"s4\":" + String(servo4) + "}";
  client.publish(mqtt_topic, message.c_str());
  client.loop();
}

void calibrateIMU() {
  Serial.println("\n===== CALIBRATION =====");
  Serial.println("Keep BOTH sensors FLAT and STILL!");
  Serial.print("Starting in 3");
  delay(1000);
  Serial.print("..2");
  delay(1000);
  Serial.print("..1");
  delay(1000);
  Serial.println("..GO!");
  
  float rollSum1 = 0, pitchSum1 = 0;
  float rollSum2 = 0, pitchSum2 = 0;
  int samples = 100;
  
  Serial.print("Calibrating");
  for (int i = 0; i < samples; i++) {
    // Read Sensor 1
    mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
    float r1 = atan2(ay1, az1) * 180.0 / PI;
    float p1 = atan2(-ax1, sqrt(ay1 * ay1 + az1 * az1)) * 180.0 / PI;
    rollSum1 += r1;
    pitchSum1 += p1;
    
    // Read Sensor 2
    mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
    float r2 = atan2(ay2, az2) * 180.0 / PI;
    float p2 = atan2(-ax2, sqrt(ay2 * ay2 + az2 * az2)) * 180.0 / PI;
    rollSum2 += r2;
    pitchSum2 += p2;
    
    if (i % 10 == 0) Serial.print(".");
    delay(10);
  }
  
  rollOffset1 = rollSum1 / samples;
  pitchOffset1 = pitchSum1 / samples;
  rollOffset2 = rollSum2 / samples;
  pitchOffset2 = pitchSum2 / samples;
  
  // Initialize complementary filters
  compRoll1 = 0;
  compPitch1 = 0;
  compRoll2 = 0;
  compPitch2 = 0;
  
  Serial.println(" Done!");
  Serial.println("\nCalibration Results:");
  Serial.print("  Sensor 1 Roll offset: ");
  Serial.print(rollOffset1, 2);
  Serial.println("°");
  Serial.print("  Sensor 1 Pitch offset: ");
  Serial.print(pitchOffset1, 2);
  Serial.println("°");
  Serial.print("  Sensor 2 Roll offset: ");
  Serial.print(rollOffset2, 2);
  Serial.println("°");
  Serial.print("  Sensor 2 Pitch offset: ");
  Serial.print(pitchOffset2, 2);
  Serial.println("°");
}

void readSensor1() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime1) / 1000.0;
  lastTime1 = currentTime;
  
  if (dt > 0.1) dt = 0.02;  // Sanity check
  
  // Read sensor data
  mpu1.getMotion6(&ax1, &ay1, &az1, &gx1, &gy1, &gz1);
  
  // Calculate angles from accelerometer
  float accelRoll = atan2(ay1, az1) * 180.0 / PI - rollOffset1;
  float accelPitch = atan2(-ax1, sqrt(ay1 * ay1 + az1 * az1)) * 180.0 / PI - pitchOffset1;
  
  // Gyroscope rates (degrees per second)
  float gyroRollRate = gx1 / 131.0;   // 131 LSB/°/s for ±250°/s range
  float gyroPitchRate = gy1 / 131.0;
  
  // Complementary filter: combine gyro and accelerometer
  compRoll1 = alpha * (compRoll1 + gyroRollRate * dt) + (1.0 - alpha) * accelRoll;
  compPitch1 = alpha * (compPitch1 + gyroPitchRate * dt) + (1.0 - alpha) * accelPitch;
  
  // Exponential smoothing
  smoothRoll1 = smoothRoll1 * (1.0 - smoothFactor) + compRoll1 * smoothFactor;
  smoothPitch1 = smoothPitch1 * (1.0 - smoothFactor) + compPitch1 * smoothFactor;
  
  roll1 = smoothRoll1;
  pitch1 = smoothPitch1;
}

void readSensor2() {
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime2) / 1000.0;
  lastTime2 = currentTime;
  
  if (dt > 0.1) dt = 0.02;  // Sanity check
  
  // Read sensor data
  mpu2.getMotion6(&ax2, &ay2, &az2, &gx2, &gy2, &gz2);
  
  // Calculate angles from accelerometer
  float accelRoll = atan2(ay2, az2) * 180.0 / PI - rollOffset2;
  float accelPitch = atan2(-ax2, sqrt(ay2 * ay2 + az2 * az2)) * 180.0 / PI - pitchOffset2;
  
  // Gyroscope rates (degrees per second)
  float gyroRollRate = gx2 / 131.0;   // 131 LSB/°/s for ±250°/s range
  float gyroPitchRate = gy2 / 131.0;
  
  // Complementary filter: combine gyro and accelerometer
  compRoll2 = alpha * (compRoll2 + gyroRollRate * dt) + (1.0 - alpha) * accelRoll;
  compPitch2 = alpha * (compPitch2 + gyroPitchRate * dt) + (1.0 - alpha) * accelPitch;
  
  // Exponential smoothing
  smoothRoll2 = smoothRoll2 * (1.0 - smoothFactor) + compRoll2 * smoothFactor;
  smoothPitch2 = smoothPitch2 * (1.0 - smoothFactor) + compPitch2 * smoothFactor;
  
  roll2 = smoothRoll2;
  pitch2 = smoothPitch2;
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  if (systemActive) {
    // Read both sensors
    readSensor1();  // For CH0 and CH3
    readSensor2();  // For CH1 and CH2
    
    // Map Sensor 1 Roll to Servo 1 (CH0) - 0-180 degrees
    int servo1 = constrain(map(roll1, -60, 60, 0, 180), 0, 180);
    
    // Map Sensor 2 Roll to Servo 2 (CH1) - 0-180 degrees
    int servo2 = constrain(map(roll2, -60, 60, 0, 180), 0, 180);
    
    // Map Sensor 2 Pitch to Servo 3 (CH2) - 0-180 degrees
    int servo3 = constrain(map(pitch2, -60, 60, 0, 180), 0, 180);
    
    // Map Sensor 1 Pitch to Servo 4 (CH3) - 0-180 degrees
    int servo4 = constrain(map(pitch1, -60, 60, 0, 180), 0, 180);
    
    // Update current positions
    currentServo1 = servo1;
    currentServo2 = servo2;
    currentServo3 = servo3;
    currentServo4 = servo4;
    
    // Publish to MQTT
    publishServoPositions(servo1, servo2, servo3, servo4);
    
    // Debug output (every 500ms)
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 500) {
      lastPrint = millis();
      
      Serial.println("┌─────────────────────────────────────────────────┐");
      Serial.print("│ Sensor 1 Roll:  ");
      Serial.print(roll1, 1);
      Serial.print("° → CH0: ");
      Serial.print(servo1);
      Serial.println("°         │");
      Serial.print("│ Sensor 1 Pitch: ");
      Serial.print(pitch1, 1);
      Serial.print("° → CH3: ");
      Serial.print(servo4);
      Serial.println("°         │");
      Serial.print("│ Sensor 2 Roll:  ");
      Serial.print(roll2, 1);
      Serial.print("° → CH1: ");
      Serial.print(servo2);
      Serial.println("°         │");
      Serial.print("│ Sensor 2 Pitch: ");
      Serial.print(pitch2, 1);
      Serial.print("° → CH2: ");
      Serial.print(servo3);
      Serial.println("°         │");
      Serial.println("└─────────────────────────────────────────────────┘");
    }
  }
  
  delay(20); // 50Hz update rate
}

// This function will be called when ESP32 is shutting down or resetting
void returnToHome() {
  Serial.println("\n>>> Returning to HOME position...");
  systemActive = false;  // Stop gesture control
  moveToHomePosition();
  Serial.println(">>> Home position reached!");
  delay(1000);  // Give time for servos to settle
}

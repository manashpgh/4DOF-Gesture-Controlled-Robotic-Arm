#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// WiFi credentials
const char* ssid = "Redmi Note 9 Pro";
const char* password = "12345678";
const char* mqtt_server = "10.89.120.186";
const int mqtt_port = 1883;
const char* mqtt_topic_imu = "robot/arm/control";

WiFiClient espClient;
PubSubClient mqttClient(espClient);
WebServer webServer(80);

// PCA9685 Servo Driver
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// 4 Servo channels
#define SERVO1_CHANNEL 0  // CH0 - Sensor 1 roll
#define SERVO2_CHANNEL 1  // CH1 - Sensor 2 roll
#define SERVO3_CHANNEL 2  // CH2 - Sensor 2 pitch
#define SERVO4_CHANNEL 3  // CH3 - Sensor 1 pitch

// Servo configuration
#define SERVOMIN  150
#define SERVOMAX  600
#define SERVO_FREQ 50

// HOME POSITION
#define HOME_POSITION 90

#define SDA_PIN 21
#define SCL_PIN 22

// Control mode
enum ControlMode {
  MODE_IMU,    // Control via MQTT from MPU6050 sensors
  MODE_WEB     // Control via Web interface
};

ControlMode currentMode = MODE_IMU;  // Default to IMU mode

// Position tracking for 4 servos
struct ServoState {
  int current;
  int target;
  float velocity;
  int history[5];
  int historyIndex;
};

ServoState servos[4];  // 4 SERVOS

// Smoothing parameters
const float ACCELERATION = 2.0;
const float MAX_VELOCITY = 15.0;
const float FRICTION = 0.85;
const int DEADBAND = 2;

float servoMaxSpeed[4] = {15.0, 15.0, 15.0, 15.0};

TaskHandle_t ServoTask;

void setup() {
  Serial.begin(115200);
  
  Wire.begin(SDA_PIN, SCL_PIN);
  
  Serial.println("\n\n\n");
  Serial.println("╔═══════════════════════════════════════╗");
  Serial.println("║   DUAL MODE ROBOT ARM CONTROLLER      ║");
  Serial.println("║   IMU + WEB CONTROL                   ║");
  Serial.println("╚═══════════════════════════════════════╝");
  
  Serial.println("\n[1/5] Initializing PCA9685...");
  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(SERVO_FREQ);
  delay(100);
  
  // Initialize 4 servos to HOME position (90 degrees)
  for (int i = 0; i < 4; i++) {
    servos[i].current = HOME_POSITION;
    servos[i].target = HOME_POSITION;
    servos[i].velocity = 0;
    servos[i].historyIndex = 0;
    for (int j = 0; j < 5; j++) {
      servos[i].history[j] = HOME_POSITION;
    }
    setServoAngle(i, HOME_POSITION);
  }
  
  Serial.println("      ✓ Servos initialized at 90°");
  
  Serial.println("\n[2/5] Connecting to WiFi...");
  setup_wifi();
  
  Serial.println("\n[3/5] Connecting to MQTT...");
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  mqttClient.setBufferSize(512);
  reconnectMQTT();
  
  Serial.println("\n[4/5] Starting Web Server...");
  setup_webserver();
  
  Serial.println("\n[5/5] Setting up mDNS...");
  if (MDNS.begin("robotarm")) {
    Serial.println("      ✓ mDNS responder started");
  }
  
  xTaskCreatePinnedToCore(
    servoControlTask,
    "ServoTask",
    10000,
    NULL,
    1,
    &ServoTask,
    0
  );
  
  printStartupInfo();
}       

void printStartupInfo() {
  Serial.println("\n╔═══════════════════════════════════════╗");
  Serial.println("║          SYSTEM READY! ✓              ║");
  Serial.println("╚═══════════════════════════════════════╝");
  Serial.println();
  Serial.println("▶ CONTROL MODES:");
  Serial.println();
  Serial.println("   MODE 1: IMU CONTROL (Default)");
  Serial.println("   → MQTT Topic: robot/arm/control");
  Serial.println("   → Control via MPU6050 sensors");
  Serial.println();
  Serial.println("   MODE 2: WEB CONTROL");
  Serial.print("   → http://");
  Serial.println(WiFi.localIP());
  Serial.println("   → http://robotarm.local");
  Serial.println();
  Serial.println("▶ YOUR ESP32 IP ADDRESS:");
  Serial.println();
  Serial.print("   ┏━━━━━━━━━━━━━━━━━━━━━━━━━━┓\n   ┃  ");
  Serial.print(WiFi.localIP());
  Serial.println("  ┃\n   ┗━━━━━━━━━━━━━━━━━━━━━━━━━━┛");
  Serial.println();
  Serial.println("▶ SWITCH MODES:");
  Serial.println("   Send 'M' via Serial to toggle modes");
  Serial.println();
  Serial.println("▶ CURRENT MODE:");
  Serial.println("   → IMU CONTROL (MQTT)");
  Serial.println();
  Serial.println("═══════════════════════════════════════");
  Serial.println();
}

void setup_wifi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.print("      Connecting");
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n      ✓ WiFi Connected!");
    Serial.print("      IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\n      ✗ WiFi Connection Failed!");
  }
}

void reconnectMQTT() {
  int attempts = 0;
  while (!mqttClient.connected() && attempts < 3) {
    Serial.print("      Connecting to MQTT...");
    String clientId = "ESP32-Robot-" + String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" ✓");
      mqttClient.subscribe(mqtt_topic_imu);
      Serial.println("      Subscribed to: robot/arm/control");
    } else {
      Serial.print(" Failed, rc=");
      Serial.println(mqttClient.state());
      delay(2000);
      attempts++;
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // Only process MQTT messages in IMU mode
  if (currentMode != MODE_IMU) return;
  
  String message = "";
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  StaticJsonDocument<256> doc;
  DeserializationError error = deserializeJson(doc, message);
  
  if (error) {
    Serial.print("JSON Parse Error: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Update servos
  if (doc.containsKey("s1")) {
    updateServoTarget(0, doc["s1"]);
  }
  if (doc.containsKey("s2")) {
    updateServoTarget(1, doc["s2"]);
  }
  if (doc.containsKey("s3")) {
    updateServoTarget(2, doc["s3"]);
  }
  if (doc.containsKey("s4")) {
    updateServoTarget(3, doc["s4"]);
  }
}

void setup_webserver() {
  webServer.enableCORS(true);
  
  webServer.on("/", HTTP_GET, handleRoot);
  webServer.on("/status", HTTP_GET, handleStatus);
  webServer.on("/servo", HTTP_POST, handleServoControl);
  webServer.on("/servos", HTTP_GET, handleGetServos);
  webServer.on("/mode", HTTP_POST, handleModeSwitch);
  webServer.on("/mode", HTTP_GET, handleGetMode);
  webServer.on("/servo", HTTP_OPTIONS, handleOptions);
  
  webServer.begin();
  Serial.println("      ✓ Web server started on port 80");
}

void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Robot Arm Controller</title>
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            min-height: 100vh;
            display: flex;
            justify-content: center;
            align-items: center;
            padding: 20px;
        }
        .container {
            background: white;
            border-radius: 20px;
            box-shadow: 0 20px 60px rgba(0, 0, 0, 0.3);
            padding: 40px;
            max-width: 600px;
            width: 100%;
        }
        h1 {
            text-align: center;
            color: #333;
            margin-bottom: 10px;
            font-size: 28px;
        }
        .subtitle {
            text-align: center;
            color: #666;
            margin-bottom: 20px;
            font-size: 14px;
        }
        .mode-switch {
            background: #f8f9fa;
            padding: 15px;
            border-radius: 10px;
            margin-bottom: 20px;
            text-align: center;
        }
        .mode-status {
            font-size: 16px;
            font-weight: bold;
            margin-bottom: 10px;
        }
        .mode-imu { color: #28a745; }
        .mode-web { color: #667eea; }
        .btn-mode {
            padding: 10px 30px;
            border: none;
            border-radius: 8px;
            font-size: 14px;
            font-weight: bold;
            cursor: pointer;
            background: #667eea;
            color: white;
        }
        .servo-control {
            margin-bottom: 30px;
            padding: 20px;
            background: #f8f9fa;
            border-radius: 15px;
        }
        .servo-header {
            display: flex;
            justify-content: space-between;
            align-items: center;
            margin-bottom: 15px;
        }
        .servo-label {
            font-size: 18px;
            font-weight: bold;
            color: #333;
        }
        .servo-info {
            font-size: 12px;
            color: #666;
        }
        .servo-value {
            font-size: 24px;
            font-weight: bold;
            color: #667eea;
        }
        .slider {
            -webkit-appearance: none;
            width: 100%;
            height: 8px;
            border-radius: 5px;
            background: #ddd;
            outline: none;
        }
        .slider::-webkit-slider-thumb {
            -webkit-appearance: none;
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
        }
        .slider::-moz-range-thumb {
            width: 25px;
            height: 25px;
            border-radius: 50%;
            background: #667eea;
            cursor: pointer;
            border: none;
        }
        .slider-labels {
            display: flex;
            justify-content: space-between;
            font-size: 12px;
            color: #999;
            margin-top: 5px;
        }
        .button-group {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 15px;
            margin-top: 30px;
        }
        .btn {
            padding: 15px;
            border: none;
            border-radius: 10px;
            font-size: 16px;
            font-weight: bold;
            cursor: pointer;
            color: white;
        }
        .btn-home { background: #28a745; }
        .btn-zero { background: #ffc107; color: #333; }
        .btn-max { background: #dc3545; }
        .disabled {
            opacity: 0.5;
            pointer-events: none;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🤖 Robot Arm Controller</h1>
        <p class="subtitle">Dual Mode: IMU + Web Control</p>

        <div class="mode-switch">
            <div class="mode-status" id="modeStatus">Current Mode: IMU Control</div>
            <button class="btn-mode" onclick="toggleMode()">Switch to Web Mode</button>
        </div>

        <div id="servoControls">
            <div class="servo-control">
                <div class="servo-header">
                    <div>
                        <div class="servo-label">CH0 - Base</div>
                        <div class="servo-info">Sensor 1 Roll</div>
                    </div>
                    <div class="servo-value" id="value0">90°</div>
                </div>
                <input type="range" min="0" max="180" value="90" class="slider" id="servo0" oninput="updateServo(0)">
                <div class="slider-labels"><span>0°</span><span>90°</span><span>180°</span></div>
            </div>

            <div class="servo-control">
                <div class="servo-header">
                    <div>
                        <div class="servo-label">CH1 - Joint 1</div>
                        <div class="servo-info">Sensor 2 Roll</div>
                    </div>
                    <div class="servo-value" id="value1">90°</div>
                </div>
                <input type="range" min="0" max="180" value="90" class="slider" id="servo1" oninput="updateServo(1)">
                <div class="slider-labels"><span>0°</span><span>90°</span><span>180°</span></div>
            </div>

            <div class="servo-control">
                <div class="servo-header">
                    <div>
                        <div class="servo-label">CH2 - Joint 2</div>
                        <div class="servo-info">Sensor 2 Pitch</div>
                    </div>
                    <div class="servo-value" id="value2">90°</div>
                </div>
                <input type="range" min="0" max="180" value="90" class="slider" id="servo2" oninput="updateServo(2)">
                <div class="slider-labels"><span>0°</span><span>90°</span><span>180°</span></div>
            </div>

            <div class="servo-control">
                <div class="servo-header">
                    <div>
                        <div class="servo-label">CH3 - End Effector</div>
                        <div class="servo-info">Sensor 1 Pitch</div>
                    </div>
                    <div class="servo-value" id="value3">90°</div>
                </div>
                <input type="range" min="0" max="180" value="90" class="slider" id="servo3" oninput="updateServo(3)">
                <div class="slider-labels"><span>0°</span><span>90°</span><span>180°</span></div>
            </div>

            <div class="button-group">
                <button class="btn btn-home" onclick="setAllServos(90)">🏠 Home</button>
                <button class="btn btn-zero" onclick="setAllServos(0)">⬇️ Zero</button>
                <button class="btn btn-max" onclick="setAllServos(180)">⬆️ Max</button>
            </div>
        </div>
    </div>

    <script>
        let currentMode = 'imu';

        function updateMode() {
            fetch('/mode')
                .then(response => response.json())
                .then(data => {
                    currentMode = data.mode;
                    const modeStatus = document.getElementById('modeStatus');
                    const servoControls = document.getElementById('servoControls');
                    const modeButton = document.querySelector('.btn-mode');
                    
                    if (currentMode === 'imu') {
                        modeStatus.innerHTML = '<span class="mode-imu">Current Mode: IMU Control (MQTT)</span>';
                        modeButton.textContent = 'Switch to Web Mode';
                        servoControls.classList.add('disabled');
                    } else {
                        modeStatus.innerHTML = '<span class="mode-web">Current Mode: Web Control</span>';
                        modeButton.textContent = 'Switch to IMU Mode';
                        servoControls.classList.remove('disabled');
                    }
                });
        }

        function toggleMode() {
            const newMode = currentMode === 'imu' ? 'web' : 'imu';
            
            fetch('/mode', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ mode: newMode })
            })
            .then(response => response.json())
            .then(data => {
                updateMode();
            });
        }

        function updateServo(channel) {
            if (currentMode === 'imu') return;
            
            const slider = document.getElementById(`servo${channel}`);
            const valueDisplay = document.getElementById(`value${channel}`);
            const angle = slider.value;
            
            valueDisplay.textContent = angle + '°';
            
            fetch('/servo', {
                method: 'POST',
                headers: { 'Content-Type': 'application/json' },
                body: JSON.stringify({ channel: channel, angle: parseInt(angle) })
            });
        }

        function setAllServos(angle) {
            if (currentMode === 'imu') return;
            
            for (let i = 0; i < 4; i++) {
                document.getElementById(`servo${i}`).value = angle;
                document.getElementById(`value${i}`).textContent = angle + '°';
                
                fetch('/servo', {
                    method: 'POST',
                    headers: { 'Content-Type': 'application/json' },
                    body: JSON.stringify({ channel: i, angle: angle })
                });
            }
        }

        window.onload = function() {
            updateMode();
            setInterval(updateMode, 2000);
            
            fetch('/servos')
                .then(response => response.json())
                .then(data => {
                    data.positions.forEach((angle, i) => {
                        document.getElementById(`servo${i}`).value = angle;
                        document.getElementById(`value${i}`).textContent = angle + '°';
                    });
                });
        };
    </script>
</body>
</html>
)rawliteral";
  
  webServer.send(200, "text/html", html);
}

void handleStatus() {
  StaticJsonDocument<200> doc;
  doc["status"] = "ok";
  doc["mode"] = (currentMode == MODE_IMU) ? "imu" : "web";
  
  String response;
  serializeJson(doc, response);
  webServer.send(200, "application/json", response);
}

void handleGetMode() {
  StaticJsonDocument<200> doc;
  doc["mode"] = (currentMode == MODE_IMU) ? "imu" : "web";
  
  String response;
  serializeJson(doc, response);
  webServer.send(200, "application/json", response);
}

void handleModeSwitch() {
  if (webServer.hasArg("plain")) {
    String body = webServer.arg("plain");
    
    StaticJsonDocument<200> doc;
    deserializeJson(doc, body);
    
    String mode = doc["mode"];
    
    if (mode == "imu") {
      currentMode = MODE_IMU;
      Serial.println("\n>>> MODE: IMU CONTROL (MQTT)");
    } else if (mode == "web") {
      currentMode = MODE_WEB;
      Serial.println("\n>>> MODE: WEB CONTROL");
    }
    
    StaticJsonDocument<200> response;
    response["success"] = true;
    response["mode"] = (currentMode == MODE_IMU) ? "imu" : "web";
    
    String responseStr;
    serializeJson(response, responseStr);
    webServer.send(200, "application/json", responseStr);
  }
}

void handleServoControl() {
  // Only allow web control in WEB mode
  if (currentMode != MODE_WEB) {
    webServer.send(403, "application/json", "{\"error\":\"Not in web mode\"}");
    return;
  }
  
  if (webServer.hasArg("plain")) {
    String body = webServer.arg("plain");
    
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, body);
    
    if (error) {
      webServer.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
      return;
    }
    
    int channel = doc["channel"];
    int angle = doc["angle"];
    
    if (channel < 0 || channel > 3 || angle < 0 || angle > 180) {
      webServer.send(400, "application/json", "{\"error\":\"Invalid parameters\"}");
      return;
    }
    
    updateServoTarget(channel, angle);
    
    Serial.print("WEB: CH");
    Serial.print(channel);
    Serial.print(" -> ");
    Serial.print(angle);
    Serial.println("°");
    
    StaticJsonDocument<200> response;
    response["success"] = true;
    response["channel"] = channel;
    response["angle"] = angle;
    
    String responseStr;
    serializeJson(response, responseStr);
    webServer.send(200, "application/json", responseStr);
  }
}

void handleGetServos() {
  StaticJsonDocument<200> doc;
  JsonArray positions = doc.createNestedArray("positions");
  
  for (int i = 0; i < 4; i++) {
    positions.add(servos[i].current);
  }
  
  String response;
  serializeJson(doc, response);
  webServer.send(200, "application/json", response);
}

void handleOptions() {
  webServer.send(200, "text/plain", "");
}

void updateServoTarget(int servoIndex, int newTarget) {
  if (servoIndex >= 4) return;
  
  newTarget = constrain(newTarget, 0, 180);
  
  servos[servoIndex].history[servos[servoIndex].historyIndex] = newTarget;
  servos[servoIndex].historyIndex = (servos[servoIndex].historyIndex + 1) % 5;
  
  int sorted[5];
  memcpy(sorted, servos[servoIndex].history, sizeof(sorted));
  for (int i = 0; i < 4; i++) {
    for (int j = i + 1; j < 5; j++) {
      if (sorted[i] > sorted[j]) {
        int temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }
  int filteredTarget = sorted[2];
  
  if (abs(filteredTarget - servos[servoIndex].current) > DEADBAND) {
    servos[servoIndex].target = filteredTarget;
  }
}

void setServoAngle(uint8_t channel, int angle) {
  angle = constrain(angle, 0, 180);
  int pulse = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(channel, 0, pulse);
}

void servoControlTask(void * parameter) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(20);
  
  for(;;) {
    for (int i = 0; i < 4; i++) {
      int error = servos[i].target - servos[i].current;
      
      float desiredVelocity = 0;
      if (abs(error) > DEADBAND) {
        float normalized = min(abs(error) / 90.0, 1.0);
        desiredVelocity = normalized * servoMaxSpeed[i];
        if (error < 0) desiredVelocity = -desiredVelocity;
      }
      
      float velocityDiff = desiredVelocity - servos[i].velocity;
      servos[i].velocity += constrain(velocityDiff, -ACCELERATION, ACCELERATION);
      
      servos[i].velocity *= FRICTION;
      servos[i].velocity = constrain(servos[i].velocity, -servoMaxSpeed[i], servoMaxSpeed[i]);
      
      float newPos = servos[i].current + servos[i].velocity;
      servos[i].current = constrain((int)newPos, 0, 180);
      
      setServoAngle(i, servos[i].current);
    }
    
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  
  webServer.handleClient();
  
  // Handle Serial commands
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'M' || cmd == 'm') {
      if (currentMode == MODE_IMU) {
        currentMode = MODE_WEB;
        Serial.println("\n>>> MODE: WEB CONTROL");
      } else {
        currentMode = MODE_IMU;
        Serial.println("\n>>> MODE: IMU CONTROL (MQTT)");
      }
    }
  }
  
  delay(2);
}

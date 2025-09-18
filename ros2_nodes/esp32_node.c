#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <micro_ros_arduino.h>
#include <ESP32Servo.h>

// ROS 2
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/int16.h>

// Wi-Fi credentials
const char* ssid = "Project";
const char* password = "88888888";

// MQTT
const char* mqtt_server = "10.42.0.44";
const int mqtt_port = 1883;
const char* mqtt_topic = "esp32/sensor/data";
const char* client_id = "ESP32Client001";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// Kalman filter (from MQTT version)
class Kalman {
public:
  float Q_angle = 0.001;
  float Q_bias = 0.003;
  float R_measure = 0.03;
  float angle = 0.0;
  float bias = 0.0;
  float rate;
  float P[2][2] = {{1.0, 0.0}, {0.0, 1.0}};

  float getAngle(float newAngle, float newRate, float dt) {
    rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    float S = P[0][0] + R_measure;
    float K[2] = {P[0][0] / S, P[1][0] / S};

    float y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
  }
};

// Sensor and Kalman instances
Adafruit_MPU6050 mpu;
Kalman kalmanX, kalmanY;
float yaw = 0.0;
unsigned long previousTime = 0;

// MQTT timing
unsigned long lastMsg = 0;
const long mqttInterval = 500;

// RC setup
const byte numChannels = 4;
const byte channelPins[] = {2, 4, 16, 17};
unsigned long pulseWidth[numChannels];
Servo steeringServo;
Servo motorServo;
const byte steeringServoPin = 19;
const byte motorControlPin = 18;

bool humanMode = true;
unsigned long lastToggleTime = 0;
bool prevToggleState = false;

// ROS setup
rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;
rcl_subscription_t servo1_subscriber;
std_msgs__msg__Int16 servo1_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define RCCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) error_loop(); }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) {} }

void error_loop() { while (1) delay(100); }

unsigned long readPulseWidth(byte pin) {
  return pulseIn(pin, HIGH, 10000);
}
unsigned int pulseToSteeringAngle(unsigned long p) { return map(p, 900, 2100, 0, 180); }
unsigned int pulseToThrottle(unsigned long p) { return map(p, 1000, 2000, 0, 180); }

const float FORWARD_SPEED = 0.2f;
void cmd_vel_callback(const void* msgin) {
  const auto* m = (const geometry_msgs__msg__Twist*)msgin;
  int throttle = (int)constrain((m->linear.x / FORWARD_SPEED) * 180.0f, 0, 180);
  motorServo.write(throttle);
}
void servo_callback1(const void* msgin) {
  const auto* m = (const std_msgs__msg__Int16*)msgin;
  steeringServo.write(constrain(m->data, 0, 180));
}

void setup_wifi() {
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500); Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi connected. IP address:");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi connection failed.");
  }
}

void mqtt_reconnect() {
  while (!mqttClient.connected()) {
    Serial.print("Connecting to MQTT...");
    if (mqttClient.connect(client_id)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
      delay(5000);
    }
  }
}

void publishSensorData() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float accelX = a.acceleration.x;
  float accelY = a.acceleration.y;
  float accelZ = a.acceleration.z;
  float gyroX = g.gyro.x;
  float gyroY = g.gyro.y;
  float gyroZ = g.gyro.z;

  float pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  float roll = atan2(-accelX, accelZ) * 180.0 / PI;

  unsigned long currentTime = millis();
  float dt = (currentTime - previousTime) / 1000.0;
  previousTime = currentTime;

  float filteredPitch = kalmanX.getAngle(pitch, gyroX, dt);
  float filteredRoll = kalmanY.getAngle(roll, gyroY, dt);
  yaw += gyroZ * dt;

  StaticJsonDocument<256> doc;
  doc["pitch"] = filteredPitch;
  doc["roll"] = filteredRoll;
  doc["yaw"] = yaw;
  doc["temperature"] = temp.temperature;

  char buffer[256];
  serializeJson(doc, buffer);
  mqttClient.publish(mqtt_topic, buffer);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  setup_wifi();
  mqttClient.setServer(mqtt_server, mqtt_port);

  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);
  delay(100);
  previousTime = millis();

  for (byte i = 0; i < numChannels; i++) pinMode(channelPins[i], INPUT);
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  steeringServo.setPeriodHertz(50);
  motorServo.setPeriodHertz(50);
  steeringServo.attach(steeringServoPin, 500, 2500);
  motorServo.attach(motorControlPin, 1000, 2000);

  set_microros_transports();
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "microcontroller_node", "", &support));
  RCCHECK(rclc_subscription_init_default(&twist_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"));
  RCCHECK(rclc_subscription_init_default(&servo1_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16), "/servo1"));
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &servo1_subscriber, &servo1_msg, &servo_callback1, ON_NEW_DATA));
}

void loop() {
  // MQTT
  if (!mqttClient.connected()) mqtt_reconnect();
  mqttClient.loop();
  if (millis() - lastMsg > mqttInterval) {
    lastMsg = millis();
    publishSensorData();
  }

  // Always read channel 3 (mode toggle)
  pulseWidth[2] = readPulseWidth(channelPins[2]);
  bool btn = (pulseWidth[2] > 1600);
  if (btn && !prevToggleState && millis() - lastToggleTime > 500) {
    humanMode = !humanMode;
    lastToggleTime = millis();
  }
  prevToggleState = btn;

  if (humanMode) {
    pulseWidth[0] = readPulseWidth(channelPins[0]);  // Steering
    pulseWidth[1] = readPulseWidth(channelPins[1]);  // Throttle
    steeringServo.write(pulseToSteeringAngle(pulseWidth[0]));
    motorServo.write(pulseToThrottle(pulseWidth[1]));
  } else {
    RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(4)));
  }
}


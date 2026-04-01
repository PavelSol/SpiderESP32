#include <WiFi.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <Update.h>
#include <ArduinoOTA.h>

// === КОНФИГУРАЦИЯ ===
#define DEBUG_MODE true
#define EEPROM_SIZE 512
#define MAX_EVENTS 10
#define SAMPLE_COUNT 10

// Уровни логирования
enum LogLevel { DEBUG, INFO, WARNING, ERROR };

// Режимы питания
enum PowerMode { ACTIVE, STANDBY, SLEEP };

// Профили движений
enum MovementProfile { ECO, NORMAL, PERFORMANCE };

// Макросы для логирования
#if DEBUG_MODE
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
#endif

// Настройки точки доступа
const char* ssid = "Robot_Controller";
const char* password = "123456789";

// Порты TCP
#define COMMAND_TCP_PORT 12345
#define SENSOR_TCP_PORT 12346

// Пины
#define MQ7_PIN 34
#define MQ9_PIN 35
#define TRIG_PIN 32
#define ECHO_PIN 33

// Интервалы
const unsigned long SERVO_UPDATE_INTERVAL = 20;
const unsigned long SENSOR_READ_INTERVAL = 1000;
const unsigned long AUTO_SEND_INTERVAL = 1000;
const unsigned long CLIENT_TIMEOUT = 30000;
const unsigned long STACK_CHECK_INTERVAL = 10000;
const unsigned long AUTO_SAVE_INTERVAL = 300000; // 5 мин

// Константы геометрии
const float length_a = 84;
const float length_b = 145;
const float length_c = 72.5;
const float length_side = 145.4;
const float z_absolute = -56;

const float z_default = -100, z_up = -60, z_boot = z_absolute;
const float x_default = 124, x_offset = 0;
const float y_start = 0, y_step = 80;
const float y_default = x_default;

const float pi = 3.1415926;

// Переменные
unsigned long lastServoUpdate = 0;
unsigned long lastSensorRead = 0;
unsigned long lastAutoSend = 0;
unsigned long lastStackCheck = 0;
unsigned long lastSave = 0;

bool ledState = false;
bool servosInitialized = false;
bool autoSendEnabled = true;
bool emergencyStop = false;
PowerMode currentMode = ACTIVE;
MovementProfile currentProfile = NORMAL;

// Датчики
int mq7Value = 0;
int mq9Value = 0;
float distance = 0;
int mq7Samples[SAMPLE_COUNT];
int mq9Samples[SAMPLE_COUNT];
int sampleIndex = 0;

// Позиции
float site_now[4][3];
float site_expect[4][3];
float temp_speed[4][3];

// Скорости
float move_speed;
float speed_multiple = 1;
const float spot_turn_speed = 5;
const float leg_move_speed = 10;
const float body_move_speed = 4;
const float stand_seat_speed = 2;

int rest_counter = 0;
const float KEEP = 255;

// Сервоприводы
Servo servo[4][3];
const int servo_pin[4][3] = {
  {14, 15, 4},
  {17, 16, 5},
  {21, 18, 19},
  {25, 26, 27}
};

// Веб‑сервер
WebServer server(80);

// Кэш кинематики
struct KinematicsCache {
  bool valid;
  float alpha, beta, gamma;
  float x, y, z;
};
KinematicsCache cache[4][3];

// События
struct Event {
  unsigned long timestamp;
  String message;
  LogLevel level;
};
Event eventQueue[MAX_EVENTS];
int eventCount = 0;

// Калибровка
struct ServoCalibration {
  int offset[4][3];
};
ServoCalibration calibration;

struct RobotState {
  float site_now[4][3];
  bool autoSendEnabled;
  unsigned long uptime;
};
RobotState currentState;

// Клиенты
WiFiServer commandServer(COMMAND_TCP_PORT);
WiFiClient commandClient;
WiFiServer sensorServer(SENSOR_TCP_PORT);
WiFiClient sensorClient;

// Прототипы функций
bool is_reach_position(int leg, float x, float y, float z);
void setupWiFi();
void setupWebServer();
void setupOTA();
void initCache();
void loadCalibration();
void saveCalibration();
void loadState();
void saveState();
void autoSaveSettings();
void logMessage(LogLevel level, const char* message);
void addEvent(LogLevel level, const char* message);
void sendRecentEvents();
void runSelfTest();
void calibrateLegPositions();
void simpleServoTest();
void debugKinematicsCheck();
//void checkBattery();
void checkTemperature();
void checkEmergency();
void checkStackUsage();
void monitorConnections();
void handleClientData();
void sendCommandResponse(String message);
void sendSensorData(String message);
void handleNewConnections();
void processCommand(String command);
void setupSensors();
void readSensors();
void sendSensorDataToClient();
void handleAutoSend();
void handleRoot();
void handleCommand();
void checkStackUsage();
bool servo_attach();
void servo_detach();
void safePowerDown();
void setPowerMode(PowerMode mode);
void setMovementProfile(MovementProfile profile);
float getSpeedMultiplier();
void smoothServoMove(int leg, int joint, float targetAngle, float duration);
bool safeServoMove(int leg, int joint, int angle, int maxAttempts);
void safeServoWrite(Servo& servo, int angle);
void cartesian_to_polar(float &alpha, float &beta, float &gamma, float x, float y, float z);
bool cartesian_to_polar_cached(float &alpha, float &beta, float &gamma,
                           float x, float y, float z, int leg, int joint);
void polar_to_servo(int leg, float alpha, float beta, float gamma);
void set_site(int leg, float x, float y, float z);
void wait_reach(int leg);
void wait_all_reach();
void sit();
void stand();
bool is_stand();
void turn_left(unsigned int step);
void turn_right(unsigned int step);
void step_forward(unsigned int step);
void step_back(unsigned int step);
void body_left(int i);
void body_right(int i);
void hand_wave(int i);
void head_up(int i);
void head_down(int i);
void body_dance(int i);
void hand_shake(int i);
void action_cmd(int action_mode);
void do_test();
float readDistance();
//float readBatteryVoltage();
float getTemperature();

// === РЕАЛИЗАЦИЯ ФУНКЦИЙ ===

void setup() {
  Serial.begin(115200);
  logMessage(INFO, "🤖 Robot Controller Starting Up...");

  // Инициализация EEPROM
  EEPROM.begin(EEPROM_SIZE);
  loadCalibration();
  debugKinematicsCheck();
  logMessage(DEBUG, "🛠️  Loaded calibration offsets:");
for (int leg = 0; leg < 4; leg++) {
  char buffer[64];
  snprintf(buffer, sizeof(buffer), "Leg %d: %d, %d, %d",
           leg,
           calibration.offset[leg][0],
           calibration.offset[leg][1],
           calibration.offset[leg][2]);
  logMessage(DEBUG, buffer);
}
  loadState();
  initCache();

  // Настройка WiFi
  setupWiFi();

  // Настройка веб‑сервера
  setupWebServer();

  // Настройка OTA
  setupOTA();

  // Настройка датчиков
  setupSensors();

  // Калибровка ног
  calibrateLegPositions();
  simpleServoTest();

  // Проверка самодиагностики
  runSelfTest();

  logMessage(INFO, "✅ System Ready!");
  if (!servo_attach()) {
  logMessage(ERROR, "❌ Failed to attach servos!");
} else {
  logMessage(INFO, "✅ Servos attached successfully");
  // Краткий тест движения
  delay(1000);
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      servo[leg][joint].write(90);
    }
  }
  logMessage(INFO, "🦾 Servos moved to 90° position");
}
}

void loop() {
  // Обработка OTA
  ArduinoOTA.handle();

  // Обработка веб‑сервера
  server.handleClient();

  // Мониторинг соединений
  monitorConnections();

  // Обработка клиентских данных
  handleClientData();

  // Автоотправка данных датчиков
  handleAutoSend();

  // Проверка температуры
  checkTemperature();

  // Проверка аварийного останова
  checkEmergency();

  // Автосохранение настроек
  autoSaveSettings();

  // Проверка стека
  if (millis() - lastStackCheck > STACK_CHECK_INTERVAL) {
    checkStackUsage();
    lastStackCheck = millis();
  }
}

// === ФУНКЦИИ ОБРАБОТКИ ===
void setupWiFi() {
  WiFi.mode(WIFI_AP);
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);

  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    logMessage(ERROR, "❌ AP Config Failed!");
    return;
  }

  int attempts = 0;
  while (attempts < 5) {
    if (WiFi.softAP(ssid, password)) {
      logMessage(INFO, "✅ Access Point started successfully!");
      return;
    }
    attempts++;
    logMessage(WARNING, ("⚠️ AP Start attempt " + String(attempts) + " failed, retrying...").c_str());
    delay(1000);
  }
  logMessage(ERROR, "🚨 Critical: Could not start AP after 5 attempts!");
  ESP.restart();
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/command", handleCommand);
  server.begin();
  logMessage(INFO, "🌐 Web server started on port 80");
}

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void handleCommand() {
  String command = server.arg("action");
  processCommand(command);
  server.send(200, "text/plain", "Command sent: " + command);
}

void logMessage(LogLevel level, const char* message) {
  switch (level) {
    case DEBUG:
      if (DEBUG_MODE) Serial.print("[DEBUG] ");
      break;
    case INFO:
      Serial.print("[INFO] ");
      break;
    case WARNING:
      Serial.print("[WARNING] ");
      break;
    case ERROR:
      Serial.print("[ERROR] ");
      break;
  }
  Serial.println(message);
}

void addEvent(LogLevel level, const char* message) {
  if (eventCount < MAX_EVENTS) {
    eventQueue[eventCount].timestamp = millis();
    eventQueue[eventCount].message = message;
    eventQueue[eventCount].level = level;
    eventCount++;
  }
}

void debugKinematicsCheck() {
  logMessage(DEBUG, "🔎 Kinematics calculation test:");
  float testX = x_default;
  float testY = y_default;
  float testZ = z_default;

  for (int leg = 0; leg < 4; leg++) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "Leg %d test at (%.1f, %.1f, %.1f)",
             leg, testX, testY, testZ);
    logMessage(DEBUG, buffer);

    float alpha, beta, gamma;
    cartesian_to_polar(alpha, beta, gamma, testX, testY, testZ);

    snprintf(buffer, sizeof(buffer),
             "Calculated angles: alpha=%.1f°, beta=%.1f°, gamma=%.1f°",
             alpha, beta, gamma);
    logMessage(DEBUG, buffer);

    // Проверка на ошибки расчёта
    if (alpha == 0 && beta == 0 && gamma == 0) {
    char message[256];
    snprintf(message, sizeof(message), "⚠️ Kinematics failed for leg %d", leg);
    logMessage(WARNING, message);
}
  }
}

void sendRecentEvents() {
  if (!commandClient.connected()) return;

  commandClient.print("=== RECENT EVENTS ===\r\n");
  for (int i = 0; i < eventCount; i++) {
    commandClient.print(eventQueue[i].timestamp / 1000);
    commandClient.print("s: ");
    switch (eventQueue[i].level) {
      case ERROR: commandClient.print("[ERROR] "); break;
      case WARNING: commandClient.print("[WARNING] "); break;
      default: commandClient.print("[INFO] "); break;
    }
    commandClient.println(eventQueue[i].message);
  }
  commandClient.print("=====================\r\n");
}

void runSelfTest() {
  logMessage(INFO, "🔎 Starting self‑test...");
  // ... реализация полной диагностики (см. предыдущий ответ)
  logMessage(INFO, "🔎 Self‑test completed");
}

void calibrateLegPositions() {
  if (emergencyStop) {
  logMessage(ERROR, "🚨 Calibration blocked by emergency stop");
  return;
}
  logMessage(INFO, "🛠️  Starting leg calibration...");
  float defaultPos[3] = {x_default, y_default, z_default};

  for (int leg = 0; leg < 4; leg++) {
    logMessage(DEBUG, ("Moving leg " + String(leg) + " to default position").c_str());

    set_site(leg, defaultPos[0], defaultPos[1], defaultPos[2]);

    // Ждём достижения позиции с таймаутом
    unsigned long startTime = millis();
    while (millis() - startTime < 3000) {  // 3‑секундный таймаут
      if (is_reach_position(leg, defaultPos[0], defaultPos[1], defaultPos[2])) {
        char buffer[64];
        snprintf(buffer, sizeof(buffer), "✅ Leg %d reached target", leg);
        logMessage(DEBUG, buffer);
        break;
      }
      delay(50);
    }

    if (!is_reach_position(leg, defaultPos[0], defaultPos[1], defaultPos[2])) {
      char buffer[64];
      snprintf(buffer, sizeof(buffer), "⚠️ Leg %d did not reach target", leg);
      logMessage(WARNING, buffer);
    }
  }
  logMessage(INFO, "🛠️  Leg calibration completed");
}

bool is_reach_position(int leg, float x, float y, float z) {
  return (abs(site_now[leg][0] - x) <= 5 &&
          abs(site_now[leg][1] - y) <= 5 &&
          abs(site_now[leg][2] - z) <= 5);
}

void checkTemperature() {
  // Реализация проверки температуры (при наличии датчика)
}

void checkEmergency() {
  if (emergencyStop) {
    servo_detach();
    autoSendEnabled = false;
    commandClient.stop();
    sensorClient.stop();
    logMessage(ERROR, "EMERGENCY STOP ACTIVATED");
    while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
  }
}

void monitorConnections() {
  static unsigned long lastCommandCheck = 0;
  static unsigned long lastSensorCheck = 0;

  if (millis() - lastCommandCheck > 5000) {
    if (!commandClient.connected() && WiFi.softAPgetStationNum() > 0) {
      commandClient = commandServer.available();
    }
    lastCommandCheck = millis();
  }

  if (millis() - lastSensorCheck > 5000) {
    if (!sensorClient.connected() && autoSendEnabled) {
      sensorClient = sensorServer.available();
    }
    lastSensorCheck = millis();
  }
}

unsigned long lastCommandActivity = 0;

void handleClientData() {
  // Обработка командного клиента
  if (commandClient.connected()) {
    if (commandClient.available()) {
      String command = commandClient.readStringUntil('\n');
      processCommand(command);
    } else if (millis() - lastCommandActivity > CLIENT_TIMEOUT) {
      logMessage(WARNING, "⏰ Command client timeout, disconnecting...");
      commandClient.stop();
    }
  }

  // Аналогично для сенсорного клиента
  if (sensorClient.connected()) {
    // ... обработка данных датчиков
  }
}

void processCommand(String command) {
  command.toUpperCase();
  // Реализация обработки команд с параметрами (см. предыдущий ответ)
}

void setupSensors() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  //pinMode(BATTERY_PIN, INPUT);

  // Инициализация I2C для датчиков температуры (если есть)
  Wire.begin();

  logMessage(INFO, "🔌 Sensors initialized");
}

void readSensors() {
  // Усреднение показаний MQ‑7
  int mq7Sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    mq7Samples[i] = analogRead(MQ7_PIN);
    mq7Sum += mq7Samples[i];
    delay(10);
  }
  mq7Value = mq7Sum / SAMPLE_COUNT;

  // Усреднение показаний MQ‑9
  int mq9Sum = 0;
  for (int i = 0; i < SAMPLE_COUNT; i++) {
    mq9Samples[i] = analogRead(MQ9_PIN);
    mq9Sum += mq9Samples[i];
    delay(10);
  }
  mq9Value = mq9Sum / SAMPLE_COUNT;

  // Измерение расстояния
  distance = readDistance();

  lastSensorRead = millis();
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2; // см
}

float getTemperature() {
  // Заглушка — реализация зависит от используемого датчика
  return 25.0; // Возвращаем фиксированное значение для примера
}

void sendSensorDataToClient() {
  if (sensorClient.connected()) {
    String data = "MQ7:" + String(mq7Value) +
                ",MQ9:" + String(mq9Value) +
                ",DIST:" + String(distance, 1) +
                /*",BAT:" + String(readBatteryVoltage(), 2) +*/
                ",TEMP:" + String(getTemperature(), 1);
    sendSensorData(data);
  }
}

void handleAutoSend() {
  if (autoSendEnabled && millis() - lastAutoSend > AUTO_SEND_INTERVAL) {
    readSensors();
    sendSensorDataToClient();
    lastAutoSend = millis();
  }
}

void autoSaveSettings() {
  static unsigned long lastSave = 0;
  if (millis() - lastSave > AUTO_SAVE_INTERVAL) { // Каждые 5 мин
    saveState();
    saveCalibration();
    lastSave = millis();
    logMessage(INFO, "💾 Settings auto‑saved");
  }
}

void loadCalibration() {
  logMessage(DEBUG, "Loading calibration from EEPROM...");
  EEPROM.get(0, calibration);

  char buffer[64];
  for (int leg = 0; leg < 4; leg++) {
    snprintf(buffer, sizeof(buffer), "Leg %d offsets: %d, %d, %d",
             leg,
             calibration.offset[leg][0],
             calibration.offset[leg][1],
             calibration.offset[leg][2]);
    logMessage(DEBUG, buffer);
  }

  // Если все значения -1, используем дефолтные
  bool allMinusOne = true;
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      if (calibration.offset[leg][joint] != -1) {
        allMinusOne = false;
        break;
      }
    }
  }

  if (allMinusOne) {
    logMessage(WARNING, "⚠️ Using default calibration (0,0,0)");
    for (int leg = 0; leg < 4; leg++) {
      for (int joint = 0; joint < 3; joint++) {
        calibration.offset[leg][joint] = 0;
      }
    }
    saveCalibration();
  }
}

void saveCalibration() {
  logMessage(DEBUG, "Saving calibration to EEPROM...");
  EEPROM.put(0, calibration);
  EEPROM.commit();

  // Дополнительная проверка после сохранения
  ServoCalibration testRead;
  EEPROM.get(0, testRead);
  bool match = true;
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      if (testRead.offset[leg][joint] != calibration.offset[leg][joint]) {
        match = false;
      }
    }
  }
  if (match) {
    logMessage(INFO, "🛠️  Calibration saved and verified");
  } else {
    logMessage(ERROR, "❌ Calibration save failed — data mismatch");
  }
}

void simpleServoTest() {
  logMessage(INFO, "🧪 Simple servo test started");

  for (int leg = 0; leg < 4; leg++) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "Testing leg %d", leg);
    logMessage(DEBUG, buffer);

    // Прямое управление без плавного движения
    servo[leg][0].write(60);  // альфа
    delay(500);
    servo[leg][1].write(60);  // бета
    delay(500);
    servo[leg][2].write(60);  // гамма
    delay(1000);

    // Возврат в центр
    servo[leg][0].write(90);
    servo[leg][1].write(90);
    servo[leg][2].write(90);
    delay(800);
  }

  logMessage(INFO, "🧪 Servo test completed");
}

void loadState() {
  EEPROM.get(sizeof(calibration), currentState);
  autoSendEnabled = currentState.autoSendEnabled;
  memcpy(site_now, currentState.site_now, sizeof(site_now));
  logMessage(INFO, "💾 State loaded");
}

void saveState() {
  currentState.autoSendEnabled = autoSendEnabled;
  memcpy(currentState.site_now, site_now, sizeof(site_now));
  currentState.uptime = millis();
  EEPROM.put(sizeof(calibration), currentState);
  EEPROM.commit();
}

void initCache() {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      cache[i][j].valid = false;
    }
  }
}

bool servo_attach() {
  if (servosInitialized) return true;

  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      servo[leg][joint].setPeriodHertz(50);
      servo[leg][joint].attach(servo_pin[leg][joint], 500, 2400);
      delay(50);
    }
  }
  servosInitialized = true;
  logMessage(INFO, "🦾 Servos attached");
  return true;
}

void servo_detach() {
  for (int leg = 0; leg < 4; leg++) {
    for (int joint = 0; joint < 3; joint++) {
      servo[leg][joint].detach();
    }
  }
  servosInitialized = false;
  logMessage(INFO, "🦾 Servos detached");
}

void safePowerDown() {
  servo_detach();
  autoSendEnabled = false;
  setPowerMode(SLEEP);
  logMessage(INFO, "😴 Power down activated");
}

void setPowerMode(PowerMode mode) {
  currentMode = mode;
  switch (mode) {
    case ACTIVE:
      autoSendEnabled = true;
      servosInitialized = true;
      break;
    case STANDBY:
      autoSendEnabled = false;
      // Отключить некритичные датчики
      break;
    case SLEEP:
      safePowerDown();
      break;
  }
}

void setMovementProfile(MovementProfile profile) {
  currentProfile = profile;
  speed_multiple = getSpeedMultiplier();
  const char* profileStr = (profile == ECO) ? "ECO" :
                         (profile == NORMAL) ? "NORMAL" : "PERFORMANCE";
logMessage(INFO, ("🏃‍♂️ Movement profile changed to: " + String(profileStr)).c_str());

}

float getSpeedMultiplier() {
  switch (currentProfile) {
    case ECO: return 0.5;
    case NORMAL: return 1.0;
    case PERFORMANCE: return 2.0;
    default: return 1.0;
  }
}

void smoothServoMove(int leg, int joint, float targetAngle, float duration) {
  float startAngle = servo[leg][joint].read();
  unsigned long startTime = millis();
  unsigned long endTime = startTime + (unsigned long)duration;

  while (millis() < endTime) {
    float progress = (float)(millis() - startTime) / duration;
    float currentAngle = startAngle + (targetAngle - startAngle) * progress;
    safeServoWrite(servo[leg][joint], currentAngle);
    delay(10); // Частота обновления 100 Гц
  }
  safeServoWrite(servo[leg][joint], targetAngle); // Финальная позиция
}

bool safeServoMove(int leg, int joint, int angle, int maxAttempts) {
  for (int attempt = 0; attempt < maxAttempts; attempt++) {
    safeServoWrite(servo[leg][joint], angle);
    if (abs(servo[leg][joint].read() - angle) < 5) {
      return true; // Успех
    }
    delay(50);
  }
  return false; // Ошибка после всех попыток
}

void safeServoWrite(Servo& servo, int angle) {
  angle = constrain(angle, 0, 180);
  servo.write(angle);
}

// Кинематические преобразования
void cartesian_to_polar(float &alpha, float &beta, float &gamma, float x, float y, float z) {
  // Вычисление альфа (угол в плоскости XY)
  alpha = atan2(y, x);

  // Радиус в плоскости XY
  float r = sqrt(x * x + y * y);

  // Общая длина от начала координат до точки
  float l = sqrt(r * r + z * z);

  // Проверка на достижимость
  if (l > length_a + length_b + length_c) {
  char buffer[64]; // Буфер достаточного размера для сообщения

  snprintf(buffer, sizeof(buffer), "⚠️ Point out of reach: %.1f,%.1f,%.1f", x, y, z);
  logMessage(WARNING, buffer);

  alpha = 0;
  beta = 0;
  gamma = 0;
  return;
}

  // Угол бета (плечо)
  float cos_beta = (l * l - length_c * length_c + length_a * length_a + length_b * length_b) /
                  (2 * length_a * sqrt(length_a * length_a + length_b * length_b));
  cos_beta = constrain(cos_beta, -1, 1);
  beta = acos(cos_beta);

  // Угол гамма (предплечье)
  float gamma1 = atan2(z, r);
  float gamma2 = atan2(length_b * sin(beta), length_a + length_b * cos(beta));
  gamma = gamma1 - gamma2;

  // Конвертация в градусы
  alpha = alpha * 180 / pi;
  beta = beta * 180 / pi;
  gamma = gamma * 180 / pi;
}

bool cartesian_to_polar_cached(float &alpha, float &beta, float &gamma,
                           float x, float y, float z, int leg, int joint) {
  // Проверка кэша
  if (cache[leg][joint].valid &&
      cache[leg][joint].x == x &&
      cache[leg][joint].y == y &&
      cache[leg][joint].z == z) {
    alpha = cache[leg][joint].alpha;
    beta = cache[leg][joint].beta;
    gamma = cache[leg][joint].gamma;
    return true; // Результат из кэша
  }

  // Вычисление (ваш существующий код)
  cartesian_to_polar(alpha, beta, gamma, x, y, z);

  // Сохранение в кэш
  cache[leg][joint].valid = true;
  cache[leg][joint].x = x;
  cache[leg][joint].y = y;
  cache[leg][joint].z = z;
  cache[leg][joint].alpha = alpha;
  cache[leg][joint].beta = beta;
  cache[leg][joint].gamma = gamma;

  return false; // Результат не из кэша
}

void polar_to_servo(int leg, float alpha, float beta, float gamma) {
  // Проверка допустимых диапазонов
  if (alpha < -90 || alpha > 90 ||
      beta < -90 || beta > 90 ||
      gamma < -90 || gamma > 90) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer),
             "⚠️ Invalid angles for leg %d: alpha=%.1f°, beta=%.1f°, gamma=%.1f°",
             leg, alpha, beta, gamma);
    logMessage(WARNING, buffer);
    return;
  }

  int alpha_pos = 90 + (int)alpha;
  int beta_pos = 90 - (int)beta;
  int gamma_pos = 90 - (int)gamma;

  // Применяем калибровку
  alpha_pos += calibration.offset[leg][0];
  beta_pos += calibration.offset[leg][1];
  gamma_pos += calibration.offset[leg][2];

  servo[leg][0].write(alpha_pos);
  servo[leg][1].write(beta_pos);
  servo[leg][2].write(gamma_pos);
}

void set_site(int leg, float x, float y, float z) {
  char buffer[128];
  snprintf(buffer, sizeof(buffer),
           "Leg %d: setting position (%.1f, %.1f, %.1f)",
           leg, x, y, z);
  logMessage(DEBUG, buffer);

  site_expect[leg][0] = x;
  site_expect[leg][1] = y;
  site_expect[leg][2] = z;

  float alpha, beta, gamma;
  cartesian_to_polar(alpha, beta, gamma, x, y, z);

  snprintf(buffer, sizeof(buffer),
           "Calculated angles: alpha=%.1f°, beta=%.1f°, gamma=%.1f°",
           alpha, beta, gamma);
  logMessage(DEBUG, buffer);

  polar_to_servo(leg, alpha, beta, gamma);
}

void wait_all_reach() {
  for (int leg = 0; leg < 4; leg++) {
    wait_reach(leg);
  }
}

// === ОСНОВНЫЕ ДВИЖЕНИЯ ===
void sit() {
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, x_default, y_default, z_boot);
  }
  wait_all_reach();
}

void stand() {
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, x_default, y_default, z_default);
  }
  wait_all_reach();
}

bool is_stand() {
  for (int leg = 0; leg < 4; leg++) {
    if (abs(site_now[leg][2] - z_default) > 5) return false;
  }
  return true;
}

void turn_left(unsigned int step) {
  for (unsigned int i = 0; i < step; i++) {
    // Реализация поворота влево
    delay(500);
  }
}

void turn_right(unsigned int step) {
  for (unsigned int i = 0; i < step; i++) {
    // Реализация поворота вправо
    delay(500);
  }
}

void step_forward(unsigned int step) {
  for (unsigned int i = 0; i < step; i++) {
    // Реализация шага вперёд
    delay(500);
  }
}

void step_back(unsigned int step) {
  for (unsigned int i = 0; i < step; i++) {
    // Реализация шага назад
    delay(500);
  }
}

void body_left(int i) {
  // Реализация движения корпуса влево
}

void body_right(int i) {
  // Реализация движения корпуса вправо
}

void hand_wave(int i) {
  // Реализация маха рукой
}

void head_up(int i) {
  // Реализация подъёма головы
}

void head_down(int i) {
  // Реализация опускания головы
}

void body_dance(int i) {
  // Реализация танцевального движения
}

void hand_shake(int i) {
  // Реализация рукопожатия
}

void action_cmd(int action_mode) {
  switch (action_mode) {
    case 1: stand(); break;
    case 2: sit(); break;
    case 3: step_forward(1); break;
    // ... другие действия
  }
}

void do_test() {
  logMessage(INFO, "🧪 Running test sequence...");
  stand();
  delay(1000);
  sit();
  delay(1000);
  stand();
  logMessage(INFO, "✅ Test sequence completed");
}

// === ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ===
void sendCommandResponse(String message) {
  if (commandClient.connected()) {
    commandClient.println(message);
  }
}

void sendSensorData(String message) {
  if (sensorClient.connected()) {
    sensorClient.println(message);
  }
}

void checkStackUsage() {
  unsigned long freeStack = uxTaskGetStackHighWaterMark(NULL);
  if (freeStack < 1024) {
    char buffer[64]; // Буфер достаточного размера для сообщения

    snprintf(buffer, sizeof(buffer), "Stack usage critical: %lu words", freeStack);
    logMessage(WARNING, buffer);
  }
}

void setupOTA() {
  ArduinoOTA.setHostname("Robot-Controller");
  ArduinoOTA.setPassword("admin");

  ArduinoOTA.onStart([]() {
    const char* type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else {
      type = "filesystem";
    }

    char buffer[64]; // Буфер достаточного размера для сообщений

    // Формируем сообщение для logMessage с эмодзи
    snprintf(buffer, sizeof(buffer), "🔄 OTA Update Started: %s", type);
    logMessage(INFO, buffer);

    // Формируем сообщение для addEvent без эмодзи
    snprintf(buffer, sizeof(buffer), "OTA update started: %s", type);
    addEvent(INFO, buffer);
  });

  ArduinoOTA.onEnd([]() {
    logMessage(INFO, "✅ OTA Update Complete!");
    addEvent(INFO, "OTA update completed successfully");
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
  int percent = (progress * 100) / total;
  if (percent % 10 == 0) { // Обновляем каждые 10 %
    char buffer[64]; // Буфер достаточного размера для сообщения

    snprintf(buffer, sizeof(buffer), "📊 OTA Progress: %d%%", percent);
    logMessage(INFO, buffer);
  }
});

  ArduinoOTA.onError([](ota_error_t error) {
  char buffer[64];  // Буфер достаточного размера для сообщений

  // Формируем общее сообщение об ошибке для logMessage
  snprintf(buffer, sizeof(buffer), "❌ OTA Error: %d", error);
  logMessage(ERROR, buffer);

  // Формируем общее сообщение об ошибке для addEvent (без эмодзи)
  snprintf(buffer, sizeof(buffer), "OTA error: %d", error);
  addEvent(ERROR, buffer);

  // Детализация ошибок с конкретными сообщениями
  if (error == OTA_AUTH_ERROR) {
    logMessage(ERROR, "Authentication failed");
  } else if (error == OTA_BEGIN_ERROR) {
    logMessage(ERROR, "Begin failed");
  } else if (error == OTA_CONNECT_ERROR) {
    logMessage(ERROR, "Connect failed");
  } else if (error == OTA_RECEIVE_ERROR) {
    logMessage(ERROR, "Receive failed");
  } else if (error == OTA_END_ERROR) {
    logMessage(ERROR, "End failed");
  }
});

  ArduinoOTA.begin();
  logMessage(INFO, "🔄 OTA update ready");
}

// === ФУНКЦИИ ОТЛАДКИ ===
void debugPrintKinematics(int leg) {
  char buffer[128];

  snprintf(buffer, sizeof(buffer), "Leg %d Pos: %.1f,%.1f,%.1f",
           leg,
           site_now[leg][0],
           site_now[leg][1],
           site_now[leg][2]);
  logMessage(DEBUG, buffer);

  snprintf(buffer, sizeof(buffer), "Exp: %.1f,%.1f,%.1f",
           site_expect[leg][0],
           site_expect[leg][1],
           site_expect[leg][2]);
  logMessage(DEBUG, buffer);
}

void printSystemStatus() {
  char buffer[128];

  logMessage(INFO, "=== SYSTEM STATUS ===");

  const char* modeStr = (currentMode == ACTIVE) ? "ACTIVE" :
                      (currentMode == STANDBY) ? "STANDBY" : "SLEEP";
  snprintf(buffer, sizeof(buffer), "Mode: %s", modeStr);
  logMessage(INFO, buffer);

  const char* profileStr = (currentProfile == ECO) ? "ECO" :
                        (currentProfile == NORMAL) ? "NORMAL" : "PERFORMANCE";
  snprintf(buffer, sizeof(buffer), "Profile: %s", profileStr);
  logMessage(INFO, buffer);

  snprintf(buffer, sizeof(buffer), "AutoSend: %s",
           autoSendEnabled ? "ENABLED" : "DISABLED");
  logMessage(INFO, buffer);

  snprintf(buffer, sizeof(buffer), "Servos: %s",
           servosInitialized ? "ATTACHED" : "DETACHED");
  logMessage(INFO, buffer);

  snprintf(buffer, sizeof(buffer), "Uptime: %lus", millis() / 1000);
  logMessage(INFO, buffer);

  snprintf(buffer, sizeof(buffer), "Free Heap: %u bytes", ESP.getFreeHeap());
  logMessage(INFO, buffer);

  logMessage(INFO, "=====================");
}

// === ОСНОВНЫЕ ЦИКЛЫ ДВИЖЕНИЙ ===
void walkCycle(int steps) {
  for (int i = 0; i < steps; i++) {
    // Шаг 1: подъём ноги
    set_site(0, x_default + 20, y_default, z_up);
    delay(500);

    // Шаг 2: перенос вперёд
    set_site(0, x_default + 60, y_default, z_up);
    delay(500);

    // Шаг 3: опускание
    set_site(0, x_default + 60, y_default, z_default);
    delay(500);

    // Аналогично для других ног в шахматном порядке
    // ...
  }
}

void turnCycle(int degrees) {
  // Реализация поворота на заданный угол
  // ...
}
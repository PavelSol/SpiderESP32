#include <WiFi.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

// Настройки точки доступа
const char* ssid = "Robot_Controller";
const char* password = "123456789";

// Настройки TCP-серверов
#define COMMAND_TCP_PORT 12345    // Порт для команд
#define SENSOR_TCP_PORT 12346     // Порт для данных с датчиков

WiFiServer commandServer(COMMAND_TCP_PORT);  // Сервер для команд
WiFiClient commandClient;                    // Клиент для команд

WiFiServer sensorServer(SENSOR_TCP_PORT);    // Сервер для данных с датчиков
WiFiClient sensorClient;                     // Клиент для данных с датчиков

// Переменные для управления
bool ledState = false;
unsigned long startTime = 0;
bool servosInitialized = false;

// Объявление сервоприводов
Servo servo[4][3];

// Пины сервоприводов - проверьте правильность для ESP32
const int servo_pin[4][3] = { 
  {15, 2, 4},    // Нога 0
  {5, 18, 19},    // Нога 1  
  {21, 22, 23},   // Нога 2
  {14, 12, 13}  // Нога 3
};

// Пины для датчиков
#define MQ7_PIN 34     // Аналоговый пин для MQ-7
#define MQ9_PIN 35     // Аналоговый пин для MQ-9
#define TRIG_PIN 32    // Trig пин HC-SR04
#define ECHO_PIN 33    // Echo пин HC-SR04

// Переменные для датчиков
int mq7Value = 0;
int mq9Value = 0;
float distance = 0;
unsigned long lastSensorRead = 0;
unsigned long lastAutoSend = 0;
const unsigned long SENSOR_READ_INTERVAL = 1000; // Чтение датчиков каждую секунду
const unsigned long AUTO_SEND_INTERVAL = 1000;   // Автоотправка каждую секунду
bool autoSendEnabled = true; // Флаг автоматической отправки

// Константы геометрии робота
const float length_a = 84;
const float length_b = 145;
const float length_c = 72.5;
const float length_side = 145.4; 
const float z_absolute = -56; 

const float z_default = -100, z_up = -60, z_boot = z_absolute;  
const float x_default = 124, x_offset = 0; 
const float y_start = 0, y_step = 80; 
const float y_default = x_default;

// Переменные позиций
float site_now[4][3];
float site_expect[4][3];
float temp_speed[4][3];
float move_speed;
float speed_multiple = 1;
const float spot_turn_speed = 5; 
const float leg_move_speed = 10; 
const float body_move_speed = 4; 
const float stand_seat_speed = 2; 
int rest_counter = 0;

const float KEEP = 255;
const float pi = 3.1415926;

// Предварительные вычисления
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);

const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

// Таймер для сервоприводов
hw_timer_t *servo_timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

// Команды робота
#define W_STAND        0
#define W_SIT          1
#define W_FORWARD      2
#define W_BACKWARD     3
#define W_LEFT         4
#define W_RIGHT        5
#define W_SHAKE        6
#define W_WAVE         7
#define W_DANCE        8

// Прототипы функций
void servo_service();
bool servo_attach(void);
void servo_detach(void);
void set_site(int leg, float x, float y, float z);
void wait_reach(int leg);
void wait_all_reach(void);
void cartesian_to_polar(float &alpha, float &beta, float &gamma, float x, float y, float z);
void polar_to_servo(int leg, float alpha, float beta, float gamma);
void sit(void);
void stand(void);
bool is_stand(void);
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
void do_test(void);

// TCP функции
void sendCommandResponse(String message);
void sendSensorData(String message);
void handleNewConnections();
void sendSystemStatus();
void processCommand(String command);
void handleClientData();
void blinkStatusLED();

// Функции для датчиков
void setupSensors();
void readSensors();
void sendSensorDataToClient();
float readDistance();
void handleAutoSend();

// Обработчик таймера для сервоприводов
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (servosInitialized) {
    servo_service();
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  // Настройка пина для встроенного LED
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  
  Serial.println();
  Serial.println("🤖 Starting Robot TCP Controller...");
  
  // Инициализация датчиков
  Serial.println("🔌 Setting up sensors...");
  setupSensors();
  
  // Инициализация переменных позиций
  Serial.println("📐 Initializing robot positions...");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      site_now[i][j] = 0;
      site_expect[i][j] = 0;
      temp_speed[i][j] = 0;
    }
  }

  // Инициализация позиций
  set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  set_site(2, x_default + x_offset, y_start, z_boot);
  set_site(3, x_default + x_offset, y_start, z_boot);
  
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      site_now[i][j] = site_expect[i][j];
    }
  }

  // Настройка таймера для сервоприводов (20ms период)
  Serial.println("⏰ Setting up servo timer...");
  servo_timer = timerBegin(0, 80, true);
  timerAttachInterrupt(servo_timer, &onTimer, true);
  timerAlarmWrite(servo_timer, 20000, true);
  timerAlarmEnable(servo_timer);

  // Инициализация сервоприводов
  Serial.println("🔧 Attaching servos...");
  if (servo_attach()) {
    servosInitialized = true;
    Serial.println("✅ Servos initialized successfully");
  } else {
    Serial.println("❌ Servo initialization failed!");
    servosInitialized = false;
  }

  // Настройка WiFi после инициализации сервоприводов
  Serial.println("📡 Setting up WiFi...");
  WiFi.mode(WIFI_AP);
  
  // Конфигурация IP адреса
  IPAddress local_IP(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  
  if (!WiFi.softAPConfig(local_IP, gateway, subnet)) {
    Serial.println("❌ AP Config Failed!");
  }
  
  // Запуск точки доступа
  if (WiFi.softAP(ssid, password)) {
    Serial.println("✅ Access Point started successfully!");
  } else {
    Serial.println("❌ AP Start Failed!");
    return;
  }
  
  delay(2000);
  
  // Запуск TCP серверов
  commandServer.begin();
  sensorServer.begin();
  Serial.println("✅ Command TCP Server started on port " + String(COMMAND_TCP_PORT));
  Serial.println("✅ Sensor TCP Server started on port " + String(SENSOR_TCP_PORT));
  
  // Вывод информации о системе
  Serial.println("\n=== ROBOT CONTROLLER ===");
  Serial.print("AP SSID:     ");
  Serial.println(ssid);
  Serial.print("AP IP:       ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Command Port:");
  Serial.println(COMMAND_TCP_PORT);
  Serial.print("Sensor Port: ");
  Serial.println(SENSOR_TCP_PORT);
  Serial.print("Servos:      ");
  Serial.println(servosInitialized ? "READY" : "FAILED");
  Serial.println("Sensors:     MQ-7, MQ-9, HC-SR04");
  Serial.println("Auto Send:   EVERY 1 SECOND to Sensor Port");
  Serial.println("==========================\n");
  
  startTime = millis();
}

void setupSensors() {
  // Настройка пинов для ультразвукового датчика
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  
  // MQ-7 и MQ-9 используют аналоговые пины, не требуют дополнительной настройки
  Serial.println("✅ Sensors initialized: MQ-7, MQ-9, HC-SR04");
}

void readSensors() {
  // Чтение значений с датчиков MQ-7 и MQ-9
  mq7Value = analogRead(MQ7_PIN);
  mq9Value = analogRead(MQ9_PIN);
  
  // Чтение расстояния с HC-SR04
  distance = readDistance();
}

float readDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000); // Таймаут 30ms
  float distance_cm = duration * 0.034 / 2;
  
  // Проверка на корректность значения
  if (distance_cm <= 0 || distance_cm > 400) {
    return -1.0; // Некорректное значение
  }
  
  return distance_cm;
}

void sendSensorDataToClient() {
  String sensorData = "SENSOR_DATA ";
  sensorData += "MQ7:" + String(mq7Value) + ",";
  sensorData += "MQ9:" + String(mq9Value) + ",";
  sensorData += "DISTANCE:" + String(distance, 2);
  
  sendSensorData(sensorData);
}

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

void handleAutoSend() {
  if (autoSendEnabled) {
    unsigned long currentMillis = millis();
    
    if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
      readSensors();
      lastSensorRead = currentMillis;
      
      // Вывод в Serial для отладки
      Serial.print("📊 Sensors - MQ7: ");
      Serial.print(mq7Value);
      Serial.print(" | MQ9: ");
      Serial.print(mq9Value);
      Serial.print(" | Distance: ");
      Serial.print(distance);
      Serial.println(" cm");
    }
    
    if (currentMillis - lastAutoSend >= AUTO_SEND_INTERVAL) {
      sendSensorDataToClient();
      lastAutoSend = currentMillis;
    }
  }
}

bool servo_attach(void) {
  Serial.println("🔌 Attaching servo motors...");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print("  Servo [");
      Serial.print(i);
      Serial.print("][");
      Serial.print(j);
      Serial.print("] on pin ");
      Serial.print(servo_pin[i][j]);
      Serial.print("... ");
      
      if (servo[i][j].attach(servo_pin[i][j])) {
        Serial.println("OK");
        delay(50); // Короткая задержка между инициализацией сервоприводов
      } else {
        Serial.println("FAILED");
        return false;
      }
    }
  }
  return true;
}

void servo_detach(void) {
  Serial.println("🔌 Detaching servo motors...");
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      servo[i][j].detach();
      delay(50);
    }
  }
  servosInitialized = false;
}

void handleNewConnections() {
  // Обработка новых подключений к командному серверу
  if (commandServer.hasClient()) {
    if (commandClient.connected()) {
      WiFiClient newClient = commandServer.available();
      newClient.stop();
      Serial.println("⚠️  New command connection rejected - client already connected");
    } else {
      commandClient = commandServer.available();
      Serial.println("✅ New Command TCP client connected!");
      
      String welcomeMsg = "🤖 Welcome to Robot Command Controller!\r\n";
      welcomeMsg += "Available commands:\r\n";
      welcomeMsg += "0 or STAND    - Stand up\r\n";
      welcomeMsg += "1 or SIT      - Sit down\r\n";
      welcomeMsg += "2 or FORWARD  - Step forward\r\n";
      welcomeMsg += "3 or BACKWARD - Step backward\r\n";
      welcomeMsg += "4 or LEFT     - Turn left\r\n";
      welcomeMsg += "5 or RIGHT    - Turn right\r\n";
      welcomeMsg += "6 or SHAKE    - Hand shake\r\n";
      welcomeMsg += "7 or WAVE     - Hand wave\r\n";
      welcomeMsg += "8 or DANCE    - Dance\r\n";
      welcomeMsg += "TEST          - Run test sequence\r\n";
      welcomeMsg += "STATUS        - System status\r\n";
      welcomeMsg += "SENSORS       - Get sensor data\r\n";
      welcomeMsg += "AUTO_ON       - Enable auto send to sensor port\r\n";
      welcomeMsg += "AUTO_OFF      - Disable auto send to sensor port\r\n";
      welcomeMsg += "HELP          - Show this message\r\n";
      welcomeMsg += "---\r\n";
      welcomeMsg += "📊 Sensor data is automatically sent to port " + String(SENSOR_TCP_PORT) + " every 1 second\r\n";
      welcomeMsg += "Current auto send: " + String(autoSendEnabled ? "ENABLED" : "DISABLED");
      
      commandClient.print(welcomeMsg);
    }
  }
  
  // Обработка новых подключений к серверу датчиков
  if (sensorServer.hasClient()) {
    if (sensorClient.connected()) {
      WiFiClient newClient = sensorServer.available();
      newClient.stop();
      Serial.println("⚠️  New sensor connection rejected - client already connected");
    } else {
      sensorClient = sensorServer.available();
      Serial.println("✅ New Sensor TCP client connected!");
      
      String sensorWelcomeMsg = "📊 Welcome to Robot Sensor Data Stream!\r\n";
      sensorWelcomeMsg += "Sensor data will be sent automatically every 1 second\r\n";
      sensorWelcomeMsg += "Format: SENSOR_DATA MQ7:value,MQ9:value,DISTANCE:value\r\n";
      sensorWelcomeMsg += "Auto send: " + String(autoSendEnabled ? "ENABLED" : "DISABLED") + "\r\n";
      
      sensorClient.print(sensorWelcomeMsg);
      
      // Отправляем текущие данные датчиков при подключении
      readSensors();
      sendSensorDataToClient();
    }
  }
}

void sendSystemStatus() {
  String status = "=== ROBOT STATUS ===\r\n";
  status += "Uptime: " + String(millis() / 1000) + "s\r\n";
  status += "LED State: " + String(ledState ? "ON" : "OFF") + "\r\n";
  status += "Command Clients: " + String(commandClient.connected() ? "1" : "0") + "\r\n";
  status += "Sensor Clients: " + String(sensorClient.connected() ? "1" : "0") + "\r\n";
  status += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\r\n";
  status += "IP: " + WiFi.softAPIP().toString() + "\r\n";
  status += "Command Port: " + String(COMMAND_TCP_PORT) + "\r\n";
  status += "Sensor Port: " + String(SENSOR_TCP_PORT) + "\r\n";
  status += "Servos: " + String(servosInitialized ? "READY" : "NOT READY") + "\r\n";
  status += "Robot State: " + String(is_stand() ? "STANDING" : "SITTING") + "\r\n";
  status += "Auto Send: " + String(autoSendEnabled ? "ENABLED" : "DISABLED") + "\r\n";
  status += "Sensors: MQ-7, MQ-9, HC-SR04\r\n";
  status += "====================";
  
  sendCommandResponse(status);
  Serial.println("📈 Sent system status");
}

void processCommand(String command) {
  command.toUpperCase();
  command.trim();
  
  Serial.print("📨 Processing command: ");
  Serial.println(command);

  // Обновляем данные датчиков перед обработкой команды SENSORS
  if (command == "SENSORS") {
    readSensors();
  }

  if (!servosInitialized && !command.equals("SENSORS") && !command.equals("STATUS") && 
      !command.equals("HELP") && !command.equals("AUTO_ON") && !command.equals("AUTO_OFF")) {
    sendCommandResponse("❌ Servos not initialized! Cannot execute command.");
    return;
  }

  // Обработка команд робота
  if (command == "0" || command == "STAND") {
    sendCommandResponse("🤖 Standing up...");
    stand();
    sendCommandResponse("✅ Robot is standing");
  }
  else if (command == "1" || command == "SIT") {
    sendCommandResponse("🤖 Sitting down...");
    sit();
    sendCommandResponse("✅ Robot is sitting");
  }
  else if (command == "2" || command == "FORWARD") {
    sendCommandResponse("🤖 Moving forward...");
    if (!is_stand()) stand();
    step_forward(1);
    sendCommandResponse("✅ Step forward completed");
  }
  else if (command == "3" || command == "BACKWARD") {
    sendCommandResponse("🤖 Moving backward...");
    if (!is_stand()) stand();
    step_back(1);
    sendCommandResponse("✅ Step backward completed");
  }
  else if (command == "4" || command == "LEFT") {
    sendCommandResponse("🤖 Turning left...");
    if (!is_stand()) stand();
    turn_left(1);
    sendCommandResponse("✅ Turn left completed");
  }
  else if (command == "5" || command == "RIGHT") {
    sendCommandResponse("🤖 Turning right...");
    if (!is_stand()) stand();
    turn_right(1);
    sendCommandResponse("✅ Turn right completed");
  }
  else if (command == "6" || command == "SHAKE") {
    sendCommandResponse("🤖 Hand shaking...");
    hand_shake(3);
    sendCommandResponse("✅ Hand shake completed");
  }
  else if (command == "7" || command == "WAVE") {
    sendCommandResponse("🤖 Hand waving...");
    hand_wave(3);
    sendCommandResponse("✅ Hand wave completed");
  }
  else if (command == "8" || command == "DANCE") {
    sendCommandResponse("🤖 Dancing...");
    body_dance(5);
    sendCommandResponse("✅ Dance completed");
  }
  else if (command == "TEST") {
    sendCommandResponse("🤖 Starting test sequence...");
    do_test();
    sendCommandResponse("✅ Test sequence completed");
  }
  else if (command == "LED_ON" || command == "ON") {
    digitalWrite(2, HIGH);
    ledState = true;
    sendCommandResponse("💡 LED turned ON");
  }
  else if (command == "LED_OFF" || command == "OFF") {
    digitalWrite(2, LOW);
    ledState = false;
    sendCommandResponse("💡 LED turned OFF");
  }
  else if (command == "STATUS") {
    sendSystemStatus();
  }
  else if (command == "SENSORS") {
    sendCommandResponse("📊 Current sensor data - MQ7: " + String(mq7Value) + 
                       ", MQ9: " + String(mq9Value) + 
                       ", Distance: " + String(distance, 2) + " cm");
    Serial.println("📊 Sent sensor data to command client");
  }
  else if (command == "AUTO_ON") {
    autoSendEnabled = true;
    sendCommandResponse("✅ Auto send ENABLED - sensor data will be sent to sensor port every second");
    Serial.println("✅ Auto send enabled");
    
    // Уведомляем сенсорного клиента об изменении статуса
    if (sensorClient.connected()) {
      sensorClient.println("📊 Auto send: ENABLED");
    }
  }
  else if (command == "AUTO_OFF") {
    autoSendEnabled = false;
    sendCommandResponse("✅ Auto send DISABLED - sensor data will not be sent automatically");
    Serial.println("✅ Auto send disabled");
    
    // Уведомляем сенсорного клиента об изменении статуса
    if (sensorClient.connected()) {
      sensorClient.println("📊 Auto send: DISABLED");
    }
  }
  else if (command == "HELP" || command == "?") {
    String helpMsg = "Available commands:\r\n";
    helpMsg += "0, STAND    - Stand up\r\n";
    helpMsg += "1, SIT      - Sit down\r\n";
    helpMsg += "2, FORWARD  - Step forward\r\n";
    helpMsg += "3, BACKWARD - Step backward\r\n";
    helpMsg += "4, LEFT     - Turn left\r\n";
    helpMsg += "5, RIGHT    - Turn right\r\n";
    helpMsg += "6, SHAKE    - Hand shake\r\n";
    helpMsg += "7, WAVE     - Hand wave\r\n";
    helpMsg += "8, DANCE    - Dance\r\n";
    helpMsg += "TEST        - Run test sequence\r\n";
    helpMsg += "STATUS      - System status\r\n";
    helpMsg += "SENSORS     - Get sensor data\r\n";
    helpMsg += "AUTO_ON     - Enable auto send to sensor port\r\n";
    helpMsg += "AUTO_OFF    - Disable auto send to sensor port\r\n";
    helpMsg += "HELP        - This message\r\n";
    helpMsg += "---\r\n";
    helpMsg += "📊 Sensor data is automatically sent to port " + String(SENSOR_TCP_PORT) + " every 1 second when auto send is enabled";
    sendCommandResponse(helpMsg);
  }
  else {
    sendCommandResponse("❌ Unknown command: " + command + "\r\nType HELP for available commands");
  }
}

void handleClientData() {
  // Обработка данных от командного клиента
  if (commandClient.connected() && commandClient.available()) {
    String message = commandClient.readStringUntil('\n');
    message.trim();
    
    if (message.length() > 0) {
      Serial.print("📨 Received command: ");
      Serial.println(message);
      
      processCommand(message);
    }
  }
  
  // Проверка разрыва соединения для командного клиента
  static bool commandWasConnected = false;
  if (commandWasConnected && !commandClient.connected()) {
    Serial.println("❌ Command TCP client disconnected");
    commandWasConnected = false;
  } else if (commandClient.connected()) {
    commandWasConnected = true;
  }
  
  // Проверка разрыва соединения для сенсорного клиента
  static bool sensorWasConnected = false;
  if (sensorWasConnected && !sensorClient.connected()) {
    Serial.println("❌ Sensor TCP client disconnected");
    sensorWasConnected = false;
  } else if (sensorClient.connected()) {
    sensorWasConnected = true;
  }
}

void blinkStatusLED() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  
  unsigned long blinkInterval;
  
  // Разная частота мигания в зависимости от состояния подключений
  if (commandClient.connected() && sensorClient.connected()) {
    blinkInterval = 300; // Быстрое мигание при двух подключениях
  } else if (commandClient.connected() || sensorClient.connected()) {
    blinkInterval = 600; // Средняя скорость при одном подключении
  } else {
    blinkInterval = 1200; // Медленное мигание без подключений
  }
  
  if (millis() - lastBlink > blinkInterval) {
    blinkState = !blinkState;
    
    if (!ledState) {
      digitalWrite(2, blinkState ? HIGH : LOW);
    }
    
    lastBlink = millis();
  }
}

void loop() {
  // Обработка новых TCP подключений
  handleNewConnections();
  
  // Чтение данных от подключенных клиентов
  handleClientData();
  
  // Автоматическая отправка данных с датчиков на сенсорный порт
  handleAutoSend();
  
  // Мигание LED для индикации работы
  blinkStatusLED();
  
  delay(10);
}

// ========== ФУНКЦИИ РОБОТА ==========

void action_cmd(int action_mode) {
  if (!servosInitialized) return;
  
  int n_step = 1;

  switch (action_mode) {
    case W_FORWARD:
      Serial.println("Step forward");
      if (!is_stand()) stand();
      step_forward(n_step);
      break;
    case W_BACKWARD:
      Serial.println("Step back");
      if (!is_stand()) stand();
      step_back(n_step);
      break;
    case W_LEFT:
      Serial.println("Turn left");
      if (!is_stand()) stand();
      turn_left(n_step);
      break;
    case W_RIGHT:
      Serial.println("Turn right");
      if (!is_stand()) stand();
      turn_right(n_step);
      break;
    case W_STAND:
      Serial.println("Stand");
      if (!is_stand()) stand();
      break;
    case W_SIT:
      Serial.println("Sit");
      if (is_stand()) sit();
      break;
    case W_SHAKE:
      Serial.println("Hand shake");
      hand_shake(5);
      break;
    case W_WAVE:
      Serial.println("Hand wave");
      hand_wave(5);
      break;
    case W_DANCE:
      Serial.println("Dance");
      body_dance(5);
      break;
    default:
      Serial.println("Error");
      break;
  }
}

void do_test(void) {
  if (!servosInitialized) return;
  
  Serial.println("Stand");
  stand();
  delay(2000);
  Serial.println("Step forward");
  step_forward(1);
  delay(2000);
  Serial.println("Step back");
  step_back(1);
  delay(2000);
  Serial.println("Turn left");
  turn_left(1);
  delay(2000);
  Serial.println("Turn right");
  turn_right(1);
  delay(2000);
  Serial.println("Hand wave");
  hand_wave(1);
  delay(2000);
  Serial.println("Hand shake");
  hand_shake(1);
  delay(2000);
  Serial.println("Sit");
  sit();
  delay(5000);
}

bool is_stand(void) {
  return (site_now[0][2] == z_default);
}

void sit(void) {
  if (!servosInitialized) return;
  
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

void stand(void) {
  if (!servosInitialized) return;
  
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}

// Остальные функции робота остаются без изменений, но добавьте проверку servosInitialized в начале каждой
void turn_left(unsigned int step) {
  if (!servosInitialized) return;
  move_speed = spot_turn_speed;
  while (step-- > 0) {
    if (site_now[3][1] == y_start) {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    } else {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void turn_right(unsigned int step) {
  if (!servosInitialized) return;
  move_speed = spot_turn_speed;
  while (step-- > 0) {
    if (site_now[2][1] == y_start) {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    } else {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void step_forward(unsigned int step) {
  if (!servosInitialized) return;
  move_speed = leg_move_speed;
  while (step-- > 0) {
    if (site_now[2][1] == y_start) {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    } else {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void step_back(unsigned int step) {
  if (!servosInitialized) return;
  move_speed = leg_move_speed;
  while (step-- > 0) {
    if (site_now[3][1] == y_start) {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    } else {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

void body_left(int i) {
  if (!servosInitialized) return;
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void body_right(int i) {
  if (!servosInitialized) return;
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void hand_wave(int i) {
  if (!servosInitialized) return;
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start) {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++) {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  } else {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++) {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void head_up(int i) {
  if (!servosInitialized) return;
 set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
}

void head_down(int i) {
  if (!servosInitialized) return;
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}

void body_dance(int i) {
  if (!servosInitialized) return;
  float x_tmp;
  float y_tmp;
  float z_tmp;
  float body_dance_speed = 2;
  sit();
  move_speed = 1;
  set_site(0, x_default, y_default, KEEP);
  set_site(1, x_default, y_default, KEEP);
  set_site(2, x_default, y_default, KEEP);
  set_site(3, x_default, y_default, KEEP);
  wait_all_reach();
  
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();
  
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++) {
    if (j > i / 4) move_speed = body_dance_speed * 2;
    if (j > i / 2) move_speed = body_dance_speed * 3;
    set_site(0, KEEP, y_default - 20, KEEP);
    set_site(1, KEEP, y_default + 20, KEEP);
    set_site(2, KEEP, y_default - 20, KEEP);
    set_site(3, KEEP, y_default + 20, KEEP);
    wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP);
    set_site(1, KEEP, y_default - 20, KEEP);
    set_site(2, KEEP, y_default + 20, KEEP);
    set_site(3, KEEP, y_default - 20, KEEP);
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
}

void hand_shake(int i) {
  if (!servosInitialized) return;
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start) {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++) {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  } else {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++) {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void servo_service(void) {
  if (!servosInitialized) return;
  
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}

void set_site(int leg, float x, float y, float z) {
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP) length_x = x - site_now[leg][0];
  if (y != KEEP) length_y = y - site_now[leg][1];
  if (z != KEEP) length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP) site_expect[leg][0] = x;
  if (y != KEEP) site_expect[leg][1] = y;
  if (z != KEEP) site_expect[leg][2] = z;
}

void wait_reach(int leg) {
  while (1) {
    if (site_now[leg][0] == site_expect[leg][0] && 
        site_now[leg][1] == site_expect[leg][1] && 
        site_now[leg][2] == site_expect[leg][2]) {
      break;
    }
    delay(1);
  }
}

void wait_all_reach(void) {
  for (int i = 0; i < 4; i++) {
    wait_reach(i);
  }
}

void cartesian_to_polar(float &alpha, float &beta, float &gamma, float x, float y, float z) {
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

void polar_to_servo(int leg, float alpha, float beta, float gamma) {
  if (!servosInitialized) return;
  
  if (leg == 0) {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  } else if (leg == 1) {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  } else if (leg == 2) {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  } else if (leg == 3) {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }

  servo[leg][0].write(alpha);
  servo[leg][1].write(beta);
  servo[leg][2].write(gamma);
}
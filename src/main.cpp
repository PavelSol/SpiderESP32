#include <WiFi.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

unsigned long lastServoUpdate = 0;
const unsigned long SERVO_UPDATE_INTERVAL = 20; // 20ms = 50Hz

#undef CONFIG_ARDUINO_LOOP_STACK_SIZE
#define CONFIG_ARDUINO_LOOP_STACK_SIZE 16384

// Добавьте эту функцию
void stack_diagnostic() {
  static unsigned long last_check = 0;
  if (millis() - last_check > 10000) {
    last_check = millis();
    Serial.printf("Free heap: %d, Free stack: %d\n", 
                  ESP.getFreeHeap(), 
                  uxTaskGetStackHighWaterMark(NULL));
  }
}

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

const int servo_pin[4][3] = { 
  {14, 15, 4},     // Нога 0 - пины 14,15,4 (все работают)
  {17, 16, 5},   // Нога 1 - пины 17,16,5  
  {12, 18, 19},   // Нога 2 - пины 12,18,19
  {25, 26, 27}    // Нога 3 - пины 25,26,27
};

// Пины для датчиков
#define MQ7_PIN 34     // Аналоговый вход
#define MQ9_PIN 35     // Аналоговый вход
#define TRIG_PIN 32    // Trig для HC-SR04
#define ECHO_PIN 33    // Echo для HC-SR04

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
void executeNextMovementPhase();
bool isWaitingForServo();
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

void setup() {
  Serial.begin(115200);
  delay(2000);

  // Настройка WiFi
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
  Serial.println();
  Serial.println("🤖 Starting Robot TCP Controller...");
  delay(500);

// Запуск TCP серверов
  commandServer.begin();
  sensorServer.begin();
  Serial.println("✅ Command TCP Server started on port " + String(COMMAND_TCP_PORT));
  Serial.println("✅ Sensor TCP Server started on port " + String(SENSOR_TCP_PORT));
  
  delay(2000);

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

  // Инициализация сервоприводов
  Serial.println("🔧 Attaching servos...");
  if (servo_attach()) {
    servosInitialized = true;
    Serial.println("✅ Servos initialized successfully");
  } else {
    Serial.println("❌ Servo initialization failed!");
    servosInitialized = false;
  }
  if (servosInitialized) {
  Serial.println("🔄 Moving to initial positions...");
  
  // Устанавливаем скорость движения
  move_speed = 2; // Медленно
  
  // Даем команду на начальные позиции
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, site_expect[leg][0], site_expect[leg][1], site_expect[leg][2]);
  }
  
  // НЕ ждем здесь - позволим loop() обработать движение
  Serial.println("✅ Initial positions set");
}

  /*// Вывод информации о системе
  Serial.println("\n=== ROBOT CONTROLLER ===");
  Serial.printf("AP SSID:     ", ssid);
  Serial.printf("AP IP:       %s\n", WiFi.softAPIP().toString().c_str());
  Serial.printf("Command Port: %s\n", COMMAND_TCP_PORT);
  Serial.printf("Sensor Port:  %s\n", SENSOR_TCP_PORT);
  Serial.printf("Servos:       %s\n", servosInitialized ? "READY" : "FAILED");
  Serial.println("Sensors:     MQ-7, MQ-9, HC-SR04");
  Serial.println("Auto Send:   EVERY 1 SECOND to Sensor Port");
  Serial.println("==========================\n");
  
  startTime = millis();*/
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
  if (!sensorClient.connected()) return;
  
  // Отправляем по частям, без создания больших строк
  sensorClient.print("SENSOR_DATA ");
  sensorClient.print("MQ7:");
  sensorClient.print(mq7Value);
  sensorClient.print(",");
  sensorClient.print("MQ9:");
  sensorClient.print(mq9Value);
  sensorClient.print(",");
  sensorClient.print("DISTANCE:");
  sensorClient.print(distance, 2);
  sensorClient.println(); // Перевод строки
}

void sendCommandResponse(String message) {
  if (commandClient.connected()) {
    commandClient.println(message);
  }
}

/*void sendSensorData(const char* message) {
  if (sensorClient.connected()) {
    sensorClient.println(message);
  }
}*/

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
  
  // Простая инициализация без лишних проверок
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      int pin = servo_pin[i][j];
      
           // Минимальная настройка
      servo[i][j].setPeriodHertz(50);
      
      // Пробуем подключить самым простым способом
      if (servo[i][j].attach(pin)) {
        servo[i][j].write(90); // Устанавливаем в нейтраль
        delay(100);
      } else {
      }
    }
  }
  
  // Всегда возвращаем true, даже если некоторые не подключились
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
  if (!commandClient.connected()) return;
  
  commandClient.print("=== ROBOT STATUS ===\r\n");
  commandClient.print("Uptime: ");
  commandClient.print(millis() / 1000);
  commandClient.print("s\r\n");
  commandClient.print("LED State: ");
  commandClient.print(ledState ? "ON" : "OFF");
  commandClient.print("\r\n");
  commandClient.print("Command Clients: ");
  commandClient.print(commandClient.connected() ? "1" : "0");
  commandClient.print("\r\n");
  commandClient.print("Sensor Clients: ");
  commandClient.print(sensorClient.connected() ? "1" : "0");
  commandClient.print("\r\n");
  commandClient.print("Free Heap: ");
  commandClient.print(ESP.getFreeHeap());
  commandClient.print(" bytes\r\n");
  commandClient.print("IP: ");
  commandClient.print(WiFi.softAPIP().toString());
  commandClient.print("\r\n");
  commandClient.print("Command Port: ");
  commandClient.print(COMMAND_TCP_PORT);
  commandClient.print("\r\n");
  commandClient.print("Sensor Port: ");
  commandClient.print(SENSOR_TCP_PORT);
  commandClient.print("\r\n");
  commandClient.print("Servos: ");
  commandClient.print(servosInitialized ? "READY" : "NOT READY");
  commandClient.print("\r\n");
  commandClient.print("Robot State: ");
  commandClient.print(is_stand() ? "STANDING" : "SITTING");
  commandClient.print("\r\n");
  commandClient.print("Auto Send: ");
  commandClient.print(autoSendEnabled ? "ENABLED" : "DISABLED");
  commandClient.print("\r\n");
  commandClient.print("====================\r\n");
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

struct MovementState {
  bool isMoving;
  unsigned int stepsRemaining;
  int currentPhase;
  unsigned long phaseStartTime;
} movementState = {false, 0, 0, 0};

void updateMovement() {
  if (!movementState.isMoving) return;
  
  // Проверяем, достигнута ли текущая фаза
  bool phaseComplete = true;
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (abs(site_now[i][j] - site_expect[i][j]) > 1.0) {
        phaseComplete = false;
        break;
      }
    }
    if (!phaseComplete) break;
  }
  
  if (phaseComplete) {
    // Переход к следующей фазе
    movementState.currentPhase++;
    movementState.phaseStartTime = millis();
    
    // Здесь вызывайте следующую фазу движения
    executeNextMovementPhase();
  }
  
  // Таймаут безопасности
  if (millis() - movementState.phaseStartTime > 5000) {
    Serial.println("⚠️ Movement timeout, forcing next phase");
    movementState.currentPhase++;
    movementState.phaseStartTime = millis();
    executeNextMovementPhase();
  }
}

void executeNextMovementPhase() {
  // Здесь логика пошагового выполнения движения
  // В зависимости от movementState.currentPhase
  // устанавливайте новые site_expect для ног
}

void loop() {
  static unsigned long last_heartbeat = 0;
  static unsigned long loop_counter = 0;
  loop_counter++;

  if (millis() - last_heartbeat > 5000) { // Каждые 5 секунд
    last_heartbeat = millis();
    Serial.printf("[OK] Loop heartbeat #%lu, Heap: %d\r\n", 
                  loop_counter, ESP.getFreeHeap());
  }
  
  unsigned long currentMillis = millis();
  
  // Серво обновление
  if (currentMillis - lastServoUpdate >= SERVO_UPDATE_INTERVAL) {
    lastServoUpdate = currentMillis;
    if (servosInitialized) {
      servo_service();
    }
  }
  
  // Сеть
  handleNewConnections();
  handleClientData();
  
  // Датчики (раз в секунду)
  if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
    lastSensorRead = currentMillis;
    readSensors();
    Serial.printf("📊 MQ7:%d MQ9:%d DIST:%.2fcm | Heap:%d\n", 
                  mq7Value, mq9Value, distance, ESP.getFreeHeap());
  }
  
  delay(1);
}


// ========== ФУНКЦИИ РОБОТА ==========
// Добавьте функцию проверки ожидания
bool isWaitingForServo() {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (abs(site_now[i][j] - site_expect[i][j]) > 0.1) {
        return true; // Еще движется
      }
    }
  }
  return false;
}

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
  //wait_all_reach();
}

void stand(void) {
  if (!servosInitialized) return;
  
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, KEEP, KEEP, z_default);
  }
  //wait_all_reach();
}

// Остальные функции робота остаются без изменений, но добавьте проверку servosInitialized в начале каждой
void turn_left(unsigned int step) {
  if (!servosInitialized) return;
  move_speed = spot_turn_speed;
  while (step-- > 0) {
    if (site_now[3][1] == y_start) {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      //wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      //wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      //wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      //wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      //wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      //wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      //wait_all_reach();
    } else {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      //wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      //wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      //wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      //wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      //wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      //wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      //wait_all_reach();
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
      //wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      //wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      //wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      //wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      //wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      //wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      //wait_all_reach();
    } else {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      //wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      //wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      //wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      //wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      //wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      //wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      //wait_all_reach();
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
      //wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      //wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      //wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      //wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      //wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      //wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      //wait_all_reach();
    } else {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      //wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      //wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      //wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      //wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      //wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      //wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      //wait_all_reach();
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
      //wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      //wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      //wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      //wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      //wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      //wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      //wait_all_reach();
    } else {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      //wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      //wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      //wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      //wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      //wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      //wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      //wait_all_reach();
    }
  }
}

void body_left(int i) {
  if (!servosInitialized) return;
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  //wait_all_reach();
}

void body_right(int i) {
  if (!servosInitialized) return;
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  //wait_all_reach();
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
      //wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      //wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    //wait_all_reach();
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
      //wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      //wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    //wait_all_reach();
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
  //wait_all_reach();
}

void head_down(int i) {
  if (!servosInitialized) return;
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  //wait_all_reach();
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
  //wait_all_reach();
  
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  //wait_all_reach();
  
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++) {
    if (j > i / 4) move_speed = body_dance_speed * 2;
    if (j > i / 2) move_speed = body_dance_speed * 3;
    set_site(0, KEEP, y_default - 20, KEEP);
    set_site(1, KEEP, y_default + 20, KEEP);
    set_site(2, KEEP, y_default - 20, KEEP);
    set_site(3, KEEP, y_default + 20, KEEP);
    //wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP);
    set_site(1, KEEP, y_default - 20, KEEP);
    set_site(2, KEEP, y_default + 20, KEEP);
    set_site(3, KEEP, y_default - 20, KEEP);
    //wait_all_reach();
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
      //wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      //wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    //wait_all_reach();
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
      //wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      //wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    //wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void servo_service(void) {
  if (!servosInitialized) return;
  
  static unsigned long lastDebug = 0;
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
  
  // Отладка раз в секунду
  if (millis() - lastDebug > 1000) {
    lastDebug = millis();
    Serial.printf("Servo service running - leg0: (%.1f,%.1f,%.1f)\n", 
                  site_now[0][0], site_now[0][1], site_now[0][2]);
  }
  
  rest_counter++;
}

void set_site(int leg, float x, float y, float z) {
  // Сохраняем текущие позиции как цель, если параметр KEEP
  float target_x = (x == KEEP) ? site_expect[leg][0] : x;
  float target_y = (y == KEEP) ? site_expect[leg][1] : y;
  float target_z = (z == KEEP) ? site_expect[leg][2] : z;

  // Вычисляем разницу
  float dx = target_x - site_now[leg][0];
  float dy = target_y - site_now[leg][1];
  float dz = target_z - site_now[leg][2];

  float distance = sqrt(dx*dx + dy*dy + dz*dz);

  // Защита от деления на ноль и малых расстояний
  if (distance < 0.1) {
    temp_speed[leg][0] = 0;
    temp_speed[leg][1] = 0;
    temp_speed[leg][2] = 0;
  } else {
    temp_speed[leg][0] = (dx / distance) * move_speed * speed_multiple;
    temp_speed[leg][1] = (dy / distance) * move_speed * speed_multiple;
    temp_speed[leg][2] = (dz / distance) * move_speed * speed_multiple;
  }

  // Устанавливаем ожидаемую позицию
  if (x != KEEP) site_expect[leg][0] = x;
  if (y != KEEP) site_expect[leg][1] = y;
  if (z != KEEP) site_expect[leg][2] = z;
}

void wait_reach(int leg) {
  unsigned long startTime = millis();
  unsigned long timeout = 5000; // 5 секунд максимум
  while (1) {
    if (site_now[leg][0] == site_expect[leg][0] && 
        site_now[leg][1] == site_expect[leg][1] && 
        site_now[leg][2] == site_expect[leg][2]) {
      break;
    }

    // Таймаут на случай проблем
    if (millis() - startTime > timeout) {
      Serial.printf("⚠️ Timeout waiting for leg %d\n", leg);
      break;
    }
    
    // Даем время другим процессам
    delay(5);
    
    // ВАЖНО: вызываем servo_service для обновления позиций
    if (servosInitialized) {
      servo_service();
    }
  }
  
}

void wait_all_reach(void) {
  for (int i = 0; i < 4; i++) {
    wait_reach(i);
    
  }
}

void cartesian_to_polar(float &alpha, float &beta, float &gamma, float x, float y, float z) {
  // ------------------------------------------------------------
  // 1. ЗАЩИТА ОТ ПЛОХИХ ВХОДНЫХ ДАННЫХ
  // ------------------------------------------------------------
  if (isnan(x) || isnan(y) || isnan(z) || isinf(x) || isinf(y) || isinf(z)) {
    Serial.println("❌ CRASH GUARD: Bad input to cartesian_to_polar");
    alpha = 90; beta = 90; gamma = 90;
    return;
  }

  // ------------------------------------------------------------
  // 2. ВЫЧИСЛЕНИЕ w И v С ЗАЩИТОЙ
  // ------------------------------------------------------------
  float x_sq = x * x;
  float y_sq = y * y;
  
  // Защита от переполнения и отрицательного корня
  float hypot_sq = x_sq + y_sq;
  if (hypot_sq < 0) hypot_sq = 0;
  
  float w = (x >= 0 ? 1 : -1) * sqrt(hypot_sq);
  float v = w - length_c;

  // ------------------------------------------------------------
  // 3. ВЫЧИСЛЕНИЕ alpha (САМОЕ ОПАСНОЕ МЕСТО)
  // ------------------------------------------------------------
  float v_sq = v * v;
  float z_sq = z * z;
  float under_root_alpha = v_sq + z_sq;

  // Защита корня
  if (under_root_alpha < 0) under_root_alpha = 0;
  float root_alpha = sqrt(under_root_alpha);

  // Защита от деления на ноль
  if (root_alpha < 0.001) root_alpha = 0.001;

  // Вычисление аргумента для acos (самая частая причина NaN)
  float acos_alpha_num = (length_a * length_a) - (length_b * length_b) + v_sq + z_sq;
  float acos_alpha_den = 2 * length_a * root_alpha;
  
  // Самая жесткая защита
  if (acos_alpha_den == 0) acos_alpha_den = 0.001;
  
  float acos_alpha_arg = acos_alpha_num / acos_alpha_den;
  
  // Обрезка до строгого диапазона [-1, 1] для acos
  if (acos_alpha_arg > 1.0) acos_alpha_arg = 1.0;
  if (acos_alpha_arg < -1.0) acos_alpha_arg = -1.0;

  // Финальное вычисление alpha
  alpha = atan2(z, v) + acos(acos_alpha_arg);

  // ------------------------------------------------------------
  // 4. ВЫЧИСЛЕНИЕ beta
  // ------------------------------------------------------------
  float acos_beta_num = (length_a * length_a) + (length_b * length_b) - v_sq - z_sq;
  float acos_beta_den = 2 * length_a * length_b;
  
  if (acos_beta_den == 0) acos_beta_den = 0.001;
  
  float acos_beta_arg = acos_beta_num / acos_beta_den;
  
  if (acos_beta_arg > 1.0) acos_beta_arg = 1.0;
  if (acos_beta_arg < -1.0) acos_beta_arg = -1.0;
  
  beta = acos(acos_beta_arg);

  // ------------------------------------------------------------
  // 5. ВЫЧИСЛЕНИЕ gamma
  // ------------------------------------------------------------
  if (fabs(x) < 0.001 && fabs(y) < 0.001) {
    gamma = 0; // atan2(0,0) недопустим
  } else {
    gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  }

  // ------------------------------------------------------------
  // 6. ПРЕОБРАЗОВАНИЕ В ГРАДУСЫ
  // ------------------------------------------------------------
  alpha = alpha * 180.0 / pi;
  beta = beta * 180.0 / pi;
  gamma = gamma * 180.0 / pi;

  // ------------------------------------------------------------
  // 7. ФИНАЛЬНАЯ ПРОВЕРКА НА ОШИБКИ
  // ------------------------------------------------------------
  if (isnan(alpha) || isnan(beta) || isnan(gamma) ||
      isinf(alpha) || isinf(beta) || isinf(gamma)) {
    
    Serial.printf("❌ MATH ERROR: (%.2f,%.2f,%.2f) -> (%.2f,%.2f,%.2f)\r\n",
                  x, y, z, alpha, beta, gamma);
    
    // Безопасные значения по умолчанию
    alpha = 90;
    beta = 90;
    gamma = 90;
  }
}

void polar_to_servo(int leg, float alpha, float beta, float gamma) {
  if (!servosInitialized) return;
  
  // Проверка на NaN
  if (isnan(alpha) || isnan(beta) || isnan(gamma)) {
    Serial.printf("⚠️ NaN in polar_to_servo for leg %d\n", leg);
    return;
  }
  
  float a = alpha, b = beta, g = gamma;
  
  // Применяем преобразования для каждой ноги
  if (leg == 0) {
    a = 90 - alpha;
    b = beta;
    g = gamma + 90;
  } else if (leg == 1) {
    a = alpha + 90;
    b = 180 - beta;
    g = 90 - gamma;
  } else if (leg == 2) {
    a = alpha + 90;
    b = 180 - beta;
    g = 90 - gamma;
  } else if (leg == 3) {
    a = 90 - alpha;
    b = beta;
    g = gamma + 90;
  }

  // Ограничиваем углы в пределах 0-180 градусов
  a = constrain(a, 0.0, 180.0);
  b = constrain(b, 0.0, 180.0);
  g = constrain(g, 0.0, 180.0);
  
  // Отправляем значения сервоприводам
  servo[leg][0].write((int)a);
  servo[leg][1].write((int)b);
  servo[leg][2].write((int)g);
}
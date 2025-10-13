#include <WiFi.h>
#include <Arduino.h>
#include <ESP32Servo.h>
#include <Wire.h>

// Настройки точки доступа
const char* ssid = "ESP32_TCP_Server";
const char* password = "123456789"; // минимум 8 символов

// Настройки TCP-сервера
#define TCP_PORT 12345
WiFiServer tcpServer(TCP_PORT);
WiFiClient tcpClient;

// Переменные для управления
bool ledState = false;
unsigned long startTime = 0;
unsigned long lastStatusTime = 0;
const unsigned long STATUS_INTERVAL = 10000; // 10 секунд

Servo servo[4][3];

const int servo_pin[4][3] = { {2, 3, 4}, {5, 6, 7}, {8, 9, 10}, {11, 12, 13} };

const float length_a = 84;
const float length_b = 145;
const float length_c = 72.5;
const float length_side = 145.4; 
const float z_absolute = -56; 

const float z_default = -100, z_up = -60, z_boot = z_absolute;  
const float x_default = 124, x_offset = 0; 
const float y_start = 0, y_step = 80; 
const float y_default = x_default;

volatile float site_now[4][3];
volatile float site_expect[4][3];
float temp_speed[4][3];
float move_speed;
float speed_multiple = 1;
const float spot_turn_speed = 5; 
const float leg_move_speed = 10; 
const float body_move_speed = 4; 
const float stand_seat_speed = 2; 
volatile int rest_counter;

const float KEEP = 255;
const float pi = 3.1415926;

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

// Команды
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
void servo_attach(void);
void servo_detach(void);
void set_site(int leg, float x, float y, float z);
void wait_reach(int leg);
void wait_all_reach(void);
void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z);
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

// Обработчик таймера для сервоприводов
void IRAM_ATTR onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  servo_service();
  portEXIT_CRITICAL_ISR(&timerMux);
}

void setup() {
  Wire.begin();
  Serial.begin(115200);
  delay(1000);
  
  // Настройка пина для встроенного LED
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  
  Serial.println();
  Serial.println("🚀 Starting ESP32 as WiFi AP + TCP Server...");
  
  // Запуск точки доступа
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
  
  delay(2000); // Даем время для инициализации
  
  // Запуск TCP сервера
  tcpServer.begin();
  Serial.println("✅ TCP Server started");
  
  // Вывод информации о системе
  Serial.println("\n=== SYSTEM INFORMATION ===");
  Serial.print("AP SSID:     ");
  Serial.println(ssid);
  Serial.print("AP IP:       ");
  Serial.println(WiFi.softAPIP());
  Serial.print("AP MAC:      ");
  Serial.println(WiFi.softAPmacAddress());
  Serial.print("TCP Port:    ");
  Serial.println(TCP_PORT);
  Serial.print("Max Clients: ");
  Serial.println(WiFi.softAPgetStationNum());
  Serial.println("==========================\n");
  
  startTime = millis();
}

void sendResponse(String message) {
  if (tcpClient.connected()) {
    tcpClient.println(message);
  }
}

void handleNewConnections() {
  if (tcpServer.hasClient()) {
    if (tcpClient.connected()) {
      // Если клиент уже подключен, отказываем новому
      WiFiClient newClient = tcpServer.available();
      newClient.stop();
      Serial.println("⚠️  New connection rejected - client already connected");
    } else {
      // Принимаем нового клиента
      tcpClient = tcpServer.available();
      Serial.println("✅ New TCP client connected!");
      
      // Отправляем приветственное сообщение
      String welcomeMsg = "Welcome to ESP32 TCP Server!\r\n";
      welcomeMsg += "Available commands:\r\n";
      welcomeMsg += "LED_ON    - Turn LED ON\r\n";
      welcomeMsg += "LED_OFF   - Turn LED OFF\r\n";
      welcomeMsg += "GET_TEMP  - Get temperature\r\n";
      welcomeMsg += "GET_STATUS - System status\r\n";
      welcomeMsg += "RESTART   - Restart ESP32\r\n";
      
      tcpClient.print(welcomeMsg);
    }
  }
}

void sendSystemStatus() {
  String status = "=== SYSTEM STATUS ===\r\n";
  status += "Uptime: " + String(millis() / 1000) + "s\r\n";
  status += "LED State: " + String(ledState ? "ON" : "OFF") + "\r\n";
  status += "Connected Clients: " + String(WiFi.softAPgetStationNum()) + "\r\n";
  status += "Free Heap: " + String(ESP.getFreeHeap()) + " bytes\r\n";
  status += "IP: " + WiFi.softAPIP().toString() + "\r\n";
  status += "====================";
  
  sendResponse(status);
  Serial.println("📈 Sent system status");
}

void sendAutoStatus() {
  if (tcpClient.connected() && millis() - lastStatusTime > STATUS_INTERVAL) {
    String autoMsg = "[Auto] Uptime: " + String(millis() / 1000) + "s, Clients: " + String(WiFi.softAPgetStationNum());
    tcpClient.println(autoMsg);
    lastStatusTime = millis();
    
    Serial.println("🔄 Sent auto status");
  }
}


void processCommand(String command) {
  command.toUpperCase();
  
  if (command == "LED_ON" || command == "ON") {
    digitalWrite(2, HIGH);
    ledState = true;
    sendResponse("LED turned ON");
    Serial.println("💡 LED turned ON");
  }
  else if (command == "LED_OFF" || command == "OFF") {
    digitalWrite(2, LOW);
    ledState = false;
    sendResponse("LED turned OFF");
    Serial.println("💡 LED turned OFF");
  }
  else if (command == "GET_TEMP" || command == "TEMP") {
    // Имитация данных с датчика температуры
    float temperature = 25.0 + (random(0, 200) / 100.0); // 25.0 - 27.0
    String response = "Temperature: " + String(temperature, 1) + " °C";
    sendResponse(response);
    Serial.println("📊 Sent temperature: " + String(temperature, 1) + " °C");
  }
  else if (command == "GET_STATUS" || command == "STATUS") {
    sendSystemStatus();
  }
  else if (command == "RESTART" || command == "RESET") {
    sendResponse("Restarting ESP32...");
    delay(1000);
    ESP.restart();
  }
  else if (command == "HELP" || command == "?") {
    String helpMsg = "Available commands:\r\n";
    helpMsg += "LED_ON, LED_OFF - Control LED\r\n";
    helpMsg += "GET_TEMP - Get temperature\r\n";
    helpMsg += "GET_STATUS - System status\r\n";
    helpMsg += "RESTART - Restart ESP32\r\n";
    helpMsg += "HELP - This message\r\n";
    sendResponse(helpMsg);
  }
  else {
    // Эхо-ответ для неизвестных команд
    String response = "Unknown command: " + command + "\r\nType HELP for available commands";
    sendResponse(response);
  }
}

void handleClientData() {
  if (tcpClient.connected() && tcpClient.available()) {
    String message = tcpClient.readStringUntil('\n');
    message.trim();
    
    if (message.length() > 0) {
      Serial.print("📨 Received: ");
      Serial.println(message);
      
      // Обработка команды
      processCommand(message);
    }
  }
  
  // Проверка разрыва соединения
  static bool wasConnected = false;
  if (wasConnected && !tcpClient.connected()) {
    Serial.println("❌ TCP client disconnected");
    wasConnected = false;
  } else if (tcpClient.connected()) {
    wasConnected = true;
  }
}

void blinkStatusLED() {
  static unsigned long lastBlink = 0;
  static bool blinkState = false;
  
  // Мигаем медленнее если есть подключенные клиенты
  unsigned long blinkInterval = tcpClient.connected() ? 1000 : 500;
  
  if (millis() - lastBlink > blinkInterval) {
    blinkState = !blinkState;
    
    // Мигаем только если LED не включен командой
    if (!ledState) {
      digitalWrite(2, blinkState ? HIGH : LOW);
    }
    
    lastBlink = millis();
  }
Serial.println("Robot starts initialization - ESP32 Version");

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
  servo_timer = timerBegin(0, 80, true); // 80 MHz / 80 = 1 MHz
  timerAttachInterrupt(servo_timer, &onTimer, true);
  timerAlarmWrite(servo_timer, 20000, true); // 20ms
  timerAlarmEnable(servo_timer);

  servo_attach();
  Serial.println("Servos initialized");
  Serial.println("Robot initialization Complete");
  Serial.println("Commands: 0-stand, 1-sit, 2-forward, 3-backward, 4-left, 5-right, 6-shake, 7-wave, 8-dance");
}


void loop() {
  // Обработка новых TCP подключений
  handleNewConnections();
  
  // Чтение данных от подключенного клиента
  handleClientData();
  
  // Автоматическая отправка статуса
  sendAutoStatus();
  
  // Мигание LED для индикации работы
  blinkStatusLED();
  
  delay(10);
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    if (input.length() > 0) {
      int command = input.toInt();
      if (command >= 0 && command <= 8) {
        action_cmd(command);
      } else {
        Serial.println("Invalid command. Use 0-8");
      }
    }
  }
  
  delay(10);
}

void action_cmd(int action_mode) {
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
  Serial.println("Stand");
  stand();
  delay(2000);
  Serial.println("Step forward");
  step_forward(5);
  delay(2000);
  Serial.println("Step back");
  step_back(5);
  delay(2000);
  Serial.println("Turn left");
  turn_left(5);
  delay(2000);
  Serial.println("Turn right");
  turn_right(5);
  delay(2000);
  Serial.println("Hand wave");
  hand_wave(3);
  delay(2000);
  Serial.println("Hand shake");
  hand_shake(3);
  delay(2000);
  Serial.println("Sit");
  sit();
  delay(5000);
}

// Остальные функции остаются без изменений
void servo_attach(void) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      servo[i][j].attach(servo_pin[i][j]);
      delay(100);
    }
  }
}

void servo_detach(void) {
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      servo[i][j].detach();
      delay(100);
    }
  }
}

bool is_stand(void) {
  return (site_now[0][2] == z_default);
}

void sit(void) {
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

void stand(void) {
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}

void turn_left(unsigned int step) {
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
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void body_right(int i) {
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void hand_wave(int i) {
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
  set_site(0, KEEP, KEEP, site_now[0][2] - i);
  set_site(1, KEEP, KEEP, site_now[1][2] + i);
  set_site(2, KEEP, KEEP, site_now[2][2] - i);
  set_site(3, KEEP, KEEP, site_now[3][2] + i);
  wait_all_reach();
}

void head_down(int i) {
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}

void body_dance(int i) {
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

void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z) {
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

// Функция для отправки broadcast сообщения всем клиентам
void broadcastMessage(String message) {
  // В этой реализации только один клиент, но можно расширить для нескольких
  if (tcpClient.connected()) {
    tcpClient.println("[Broadcast] " + message);
  }
}
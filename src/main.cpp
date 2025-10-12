#include <WiFi.h>

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

void setup() {
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
}

// Функция для отправки broadcast сообщения всем клиентам
void broadcastMessage(String message) {
  // В этой реализации только один клиент, но можно расширить для нескольких
  if (tcpClient.connected()) {
    tcpClient.println("[Broadcast] " + message);
  }
}
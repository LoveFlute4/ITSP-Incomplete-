#include <WiFi.h>

// Motor control pins (keep your original pin assignments)
const int enA = 32;
const int enB = 33;
const int in1 = 14;
const int in2 = 13;
const int in3 = 25;
const int in4 = 26;

// WiFi credentials for Tinkerers' Lab
const char* ssid = "Tinkerers' Lab";
const char* password = "tinker@tl";

// Robot identification
const char* robot_name = "RobotCar";

WiFiServer server(80);

// Connection status variables
bool wifiConnected = false;
unsigned long lastConnectionCheck = 0;
const unsigned long connectionCheckInterval = 30000; // Check every 30 seconds

void setup() {
  Serial.begin(115200);
  Serial.println("Starting ESP32 Robot Car...");
  
  // Configure motor control pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Enable motors (full speed)
  digitalWrite(enA, HIGH);
  digitalWrite(enB, HIGH);
  
  // Initialize motors to stop
  stopMotors();
  
  // Connect to WiFi network
  connectToWiFi();
  
  // Start web server
  server.begin();
  Serial.println("HTTP server started");
  
  // Print connection information
  Serial.println("=====================================");
  Serial.println("ROBOT CAR NETWORK INFORMATION");
  Serial.println("=====================================");
  Serial.print("Robot Name: ");
  Serial.println(robot_name);
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.print("MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.print("Network: ");
  Serial.println(ssid);
  Serial.println("=====================================");
  Serial.println("Ready to receive commands!");
  Serial.println("=====================================");
}

void loop() {
  // Check WiFi connection periodically
  checkWiFiConnection();
  
  // Handle incoming client requests
  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("New client connected");
    String currentLine = "";
    
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        if (c == '\n') {
          if (currentLine.length() == 0) {
            // Send response
            sendResponse(client);
            break;
          } else {
            // Process command
            processCommand(currentLine);
            currentLine = "";
          }
        } else if (c != '\r') {
          currentLine += c;
        }
      }
    }
    
    // Close connection
    client.stop();
    Serial.println("Client disconnected");
  }
}

void connectToWiFi() {
  Serial.print("Connecting to WiFi network: ");
  Serial.println(ssid);
  
  WiFi.begin(ssid, password);
  
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    Serial.print(".");
    attempts++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    Serial.println();
    Serial.println("WiFi connected successfully!");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC address: ");
    Serial.println(WiFi.macAddress());
    Serial.print("Signal strength (RSSI): ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm");
  } else {
    wifiConnected = false;
    Serial.println();
    Serial.println("Failed to connect to WiFi!");
    Serial.println("Please check your network credentials and try again.");
  }
}

void checkWiFiConnection() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastConnectionCheck >= connectionCheckInterval) {
    lastConnectionCheck = currentTime;
    
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost. Attempting to reconnect...");
      wifiConnected = false;
      connectToWiFi();
    } else if (!wifiConnected) {
      wifiConnected = true;
      Serial.println("WiFi connection restored");
    }
  }
}

void sendResponse(WiFiClient &client) {
  // Send HTTP response header
  client.println("HTTP/1.1 200 OK");
  client.println("Content-type:text/html");
  client.println("Connection: close");
  client.println();
  
  // Simple response for headless operation
  client.println("<!DOCTYPE html><html><head><title>Robot Car Control</title></head>");
  client.println("<body><h1>Robot Car Control Panel</h1>");
  client.print("<p><strong>Robot Name:</strong> ");
  client.print(robot_name);
  client.println("</p>");
  client.print("<p><strong>IP Address:</strong> ");
  client.print(WiFi.localIP());
  client.println("</p>");
  client.print("<p><strong>MAC Address:</strong> ");
  client.print(WiFi.macAddress());
  client.println("</p>");
  client.print("<p><strong>Signal Strength:</strong> ");
  client.print(WiFi.RSSI());
  client.println(" dBm</p>");
  client.println("<p><strong>Status:</strong> Online and ready to receive commands</p>");
  client.println("</body></html>");
}

void processCommand(String line) {
  Serial.print("Processing command: ");
  Serial.println(line);
  
  if (line.indexOf("GET /forward") >= 0) {
    moveForward();
    Serial.println("Command: FORWARD");
  }
  else if (line.indexOf("GET /backward") >= 0) {
    moveBackward();
    Serial.println("Command: BACKWARD");
  }
  else if (line.indexOf("GET /left") >= 0) {
    turnLeft();
    Serial.println("Command: LEFT");
  }
  else if (line.indexOf("GET /right") >= 0) {
    turnRight();
    Serial.println("Command: RIGHT");
  }
  else if (line.indexOf("GET /stop") >= 0) {
    stopMotors();
    Serial.println("Command: STOP");
  }
}

// Motor control functions (unchanged from your original code)
void moveForward() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void moveBackward() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void turnLeft() {
  digitalWrite(in1, HIGH);  // Left motor forward
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);   // Right motor backward
  digitalWrite(in4, HIGH);
}

void turnRight() {
  digitalWrite(in1, LOW);   // Left motor backward
  digitalWrite(in2, HIGH);
  digitalWrite(in3, HIGH);  // Right motor forward
  digitalWrite(in4, LOW);
}

void stopMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
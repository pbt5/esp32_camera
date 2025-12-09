/*
 * ESP32-CAM WiFi Direct Connection Firmware
 * Version: 2.0 - Direct WiFi connection to host computer
 *
 * Hardware: ESP32-CAM (AI-Thinker) with OV2640 camera
 * Communication: WiFi TCP (port 8081) + UDP discovery (port 8889)
 *
 * Features:
 *   - WiFi direct connection to host computer
 *   - TCP server for receiving photo commands
 *   - UDP discovery for auto-detection
 *   - JSON protocol for commands and photo data
 *   - Base64 encoded photo transfer
 *
 * Protocol:
 *   RX: {"cmd": "TAKE_PHOTO", "box": 3, "med_name": "Aspirin", "count": 3}
 *   TX: {"type": "photo_data", "box": 3, "filename": "...", "data": "base64...", "size": 12345}
 *   TX: {"type": "photo_complete", "box": 3, "count": 3}
 */

#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "base64.h"

// ==================== Camera Pin Definitions (AI-Thinker ESP32-CAM) ====================
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

// ==================== WiFi Configuration ====================
const char* ssid = "12345";
const char* password = "12345";

// ==================== Static IP Configuration ====================
// Set USE_STATIC_IP to true to use fixed IP address
#define USE_STATIC_IP true

// Static IP settings for ESP32 Hotspot network (192.168.4.x)
// ESP32 Hotspot IP is 192.168.4.1
IPAddress staticIP(192, 168, 4, 200);      // ESP32-CAM fixed IP address
IPAddress gateway(192, 168, 4, 1);          // ESP32 Hotspot IP address
IPAddress subnet(255, 255, 255, 0);         // Subnet mask
IPAddress dns1(192, 168, 4, 1);             // Use hotspot as DNS
IPAddress dns2(8, 8, 8, 8);                 // Fallback DNS (Google)

// ==================== Network Configuration ====================
const int TCP_PORT = 8081;          // TCP server port (different from Main ESP32's 8080)
const int UDP_DISCOVERY_PORT = 8889; // UDP discovery port (different from Main ESP32's 8888)

WiFiServer server(TCP_PORT);
WiFiClient client;
WiFiUDP udp;

// ==================== Camera Configuration ====================
#define CAMERA_FRAME_SIZE FRAMESIZE_VGA     // 640×480
#define JPEG_QUALITY      12                // 0-63, lower = better quality
#define DEFAULT_PHOTO_COUNT 3               // Default photos per request

// ==================== Global Variables ====================
bool cameraReady = false;

// ==================== Setup ====================
void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n========================================");
  Serial.println("ESP32-CAM WiFi Direct v2.0");
  Serial.println("========================================\n");

  // IMPORTANT: Connect to WiFi BEFORE initializing camera
  // This prevents crash due to memory/power issues
  connectToWiFi();

  // Display IP address prominently
  displayIPAddress();

  // Initialize camera AFTER WiFi is connected
  Serial.println("\nInitializing camera...");
  if (initCamera()) {
    Serial.println("[OK] Camera initialized");
    cameraReady = true;
  } else {
    Serial.println("[ERROR] Camera initialization failed!");
    Serial.println("System will restart in 5 seconds...");
    delay(5000);
    ESP.restart();
  }

  // Start TCP server
  server.begin();
  Serial.print("[OK] TCP server started on port ");
  Serial.println(TCP_PORT);

  // Start UDP discovery listener
  udp.begin(UDP_DISCOVERY_PORT);
  Serial.print("[OK] UDP discovery on port ");
  Serial.println(UDP_DISCOVERY_PORT);

  Serial.println("\n========================================");
  Serial.println("ESP32-CAM Ready for connections!");
  Serial.println("========================================\n");
}

// ==================== Main Loop ====================
void loop() {
  handleUDPDiscovery();
  handleTCPClient();
  delay(10);
}

// ==================== WiFi Connection ====================
void connectToWiFi() {
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);

  // Configure static IP if enabled
  #if USE_STATIC_IP
    Serial.println("[CONFIG] Using Static IP configuration");
    Serial.print("  Static IP: ");
    Serial.println(staticIP);
    Serial.print("  Gateway:   ");
    Serial.println(gateway);
    Serial.print("  Subnet:    ");
    Serial.println(subnet);

    if (!WiFi.config(staticIP, gateway, subnet, dns1, dns2)) {
      Serial.println("[WARNING] Static IP configuration failed, using DHCP");
    }
  #else
    Serial.println("[CONFIG] Using DHCP (dynamic IP)");
  #endif

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(500);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("\n[ERROR] WiFi connection failed!");
    Serial.println("Restarting in 3 seconds...");
    delay(3000);
    ESP.restart();
  }

  Serial.println("\n[OK] WiFi Connected!");
}

// ==================== Display IP Address ====================
void displayIPAddress() {
  Serial.println("\n========================================");
  Serial.println("       ESP32-CAM NETWORK INFO");
  Serial.println("========================================");
  Serial.print("   IP Address: ");
  Serial.println(WiFi.localIP());
  #if USE_STATIC_IP
    Serial.println("   IP Type:    STATIC (Fixed)");
  #else
    Serial.println("   IP Type:    DHCP (Dynamic)");
  #endif
  Serial.print("   TCP Port:   ");
  Serial.println(TCP_PORT);
  Serial.print("   UDP Port:   ");
  Serial.println(UDP_DISCOVERY_PORT);
  Serial.print("   MAC:        ");
  Serial.println(WiFi.macAddress());
  Serial.print("   RSSI:       ");
  Serial.print(WiFi.RSSI());
  Serial.println(" dBm");
  Serial.println("========================================\n");
}

// ==================== Camera Initialization ====================
bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame size and quality
  if (psramFound()) {
    config.frame_size = CAMERA_FRAME_SIZE;
    config.jpeg_quality = JPEG_QUALITY;
    config.fb_count = 2;
    Serial.println("[INFO] PSRAM found - using optimized settings");
  } else {
    config.frame_size = FRAMESIZE_QVGA;  // 320×240 fallback
    config.jpeg_quality = 15;
    config.fb_count = 1;
    Serial.println("[INFO] No PSRAM - using minimal settings");
  }

  // Initialize camera
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("[ERROR] Camera init failed: 0x%x\n", err);
    return false;
  }

  // Camera settings for better image quality
  sensor_t * s = esp_camera_sensor_get();
  s->set_brightness(s, 0);
  s->set_contrast(s, 0);
  s->set_saturation(s, 0);
  s->set_whitebal(s, 1);
  s->set_awb_gain(s, 1);
  s->set_exposure_ctrl(s, 1);
  s->set_aec2(s, 1);
  s->set_gain_ctrl(s, 1);

  return true;
}

// ==================== UDP Discovery Handler ====================
void handleUDPDiscovery() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[256];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
      String request = String(incomingPacket);

      // Check if this is a discovery request
      if (request.indexOf("DISCOVER") >= 0 || request.indexOf("CAM") >= 0) {
        Serial.print("[UDP] Discovery request from: ");
        Serial.print(udp.remoteIP());
        Serial.print(":");
        Serial.println(udp.remotePort());

        // Send discovery response
        sendUDPDiscoveryResponse();
      }
    }
  }
}

void sendUDPDiscoveryResponse() {
  StaticJsonDocument<256> doc;
  doc["device"] = "ESP32-CAM";
  doc["ip"] = WiFi.localIP().toString();
  doc["port"] = TCP_PORT;
  doc["type"] = "discovery_response";
  doc["camera_ready"] = cameraReady;

  String response;
  serializeJson(doc, response);

  udp.beginPacket(udp.remoteIP(), udp.remotePort());
  udp.print(response);
  udp.endPacket();

  Serial.println("[UDP] Sent discovery response: " + response);
}

// ==================== TCP Client Handler ====================
void handleTCPClient() {
  // Check for new client connection
  if (!client || !client.connected()) {
    client = server.available();
    if (client) {
      Serial.println("[TCP] New client connected!");
      sendWelcomeMessage();
    }
  }

  // Handle existing client
  if (client && client.connected()) {
    if (client.available()) {
      String message = client.readStringUntil('\n');
      message.trim();

      if (message.length() > 0) {
        Serial.println("[TCP] Received: " + message);
        handleCommand(message);
      }
    }
  }
}

void sendWelcomeMessage() {
  StaticJsonDocument<256> doc;
  doc["type"] = "welcome";
  doc["device"] = "ESP32-CAM";
  doc["ip"] = WiFi.localIP().toString();
  doc["port"] = TCP_PORT;
  doc["camera_ready"] = cameraReady;

  String output;
  serializeJson(doc, output);
  client.println(output);
}

// ==================== Command Handler ====================
void handleCommand(const String& message) {
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    Serial.print("[ERROR] JSON parse error: ");
    Serial.println(error.c_str());
    sendError("Invalid JSON");
    return;
  }

  String cmd = doc["cmd"].as<String>();

  if (cmd == "TAKE_PHOTO") {
    handleTakePhoto(doc);
  } else if (cmd == "GET_FRAME") {
    handleGetFrame();
  } else if (cmd == "PING") {
    sendPong();
  } else if (cmd == "STATUS") {
    sendStatus();
  } else {
    Serial.println("[ERROR] Unknown command: " + cmd);
    sendError("Unknown command");
  }
}

// ==================== Take Photo Handler ====================
void handleTakePhoto(JsonDocument& doc) {
  if (!cameraReady) {
    sendError("Camera not ready");
    return;
  }

  int box = doc["box"] | 0;
  String medName = doc["med_name"] | "Unknown";
  int count = doc["count"] | DEFAULT_PHOTO_COUNT;

  // Sanitize medication name for filename
  medName.replace(" ", "_");
  medName.replace("-", "_");

  Serial.print("[PHOTO] Taking ");
  Serial.print(count);
  Serial.print(" photos for Box ");
  Serial.print(box);
  Serial.print(" (");
  Serial.print(medName);
  Serial.println(")");

  int successCount = 0;

  for (int i = 1; i <= count; i++) {
    // Generate timestamp: YYYYMMDD_HHMMSS format
    // Note: ESP32-CAM doesn't have RTC, so we use millis() as unique ID
    String timestamp = String(millis());

    // Generate filename: boxN_MedName_timestamp_M.jpg
    String filename = "box" + String(box) + "_" + medName + "_" + timestamp + "_" + String(i) + ".jpg";

    Serial.print("[PHOTO] Capturing ");
    Serial.print(i);
    Serial.print("/");
    Serial.print(count);
    Serial.print(": ");

    // Capture photo
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("FAILED (capture)");
      continue;
    }

    Serial.print("OK (");
    Serial.print(fb->len);
    Serial.println(" bytes)");

    // Send photo via TCP
    if (sendPhotoData(box, filename, fb)) {
      successCount++;
    }

    // Return frame buffer
    esp_camera_fb_return(fb);

    // Small delay between photos
    if (i < count) {
      delay(300);
    }
  }

  // Send completion message
  sendPhotoComplete(box, successCount);

  Serial.print("[PHOTO] Complete: ");
  Serial.print(successCount);
  Serial.print("/");
  Serial.print(count);
  Serial.println(" photos sent");
}

// ==================== Send Photo Data ====================
bool sendPhotoData(int box, String filename, camera_fb_t *fb) {
  if (!client || !client.connected()) {
    Serial.println("[ERROR] Client not connected");
    return false;
  }

  // Encode photo to base64
  String base64Data = base64::encode(fb->buf, fb->len);

  // Create JSON message
  // Note: For large photos, we may need to send in chunks
  // Current implementation sends full photo in one message
  StaticJsonDocument<256> doc;
  doc["type"] = "photo_data";
  doc["box"] = box;
  doc["filename"] = filename;
  doc["size"] = fb->len;

  String header;
  serializeJson(doc, header);

  // Send header with base64 data
  // Format: {"type":"photo_data",...,"data":"base64..."}
  String output = header.substring(0, header.length() - 1);  // Remove closing }
  output += ",\"data\":\"";
  output += base64Data;
  output += "\"}";

  client.println(output);

  Serial.print("[TCP] Sent photo: ");
  Serial.print(filename);
  Serial.print(" (");
  Serial.print(base64Data.length());
  Serial.println(" base64 bytes)");

  return true;
}

// ==================== Send Photo Complete ====================
void sendPhotoComplete(int box, int count) {
  StaticJsonDocument<128> doc;
  doc["type"] = "photo_complete";
  doc["box"] = box;
  doc["count"] = count;

  String output;
  serializeJson(doc, output);
  client.println(output);
}

// ==================== Get Single Frame Handler ====================
void handleGetFrame() {
  if (!cameraReady) {
    sendError("Camera not ready");
    return;
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    sendError("Failed to capture frame");
    return;
  }

  // Encode frame to base64
  String base64Data = base64::encode(fb->buf, fb->len);

  // Create JSON message with frame data
  StaticJsonDocument<256> doc;
  doc["type"] = "frame_data";
  doc["size"] = fb->len;
  doc["width"] = fb->width;
  doc["height"] = fb->height;

  String header;
  serializeJson(doc, header);

  // Send header with base64 data
  String output = header.substring(0, header.length() - 1);  // Remove closing }
  output += ",\"data\":\"";
  output += base64Data;
  output += "\"}";

  client.println(output);

  esp_camera_fb_return(fb);
}

// ==================== Helper Functions ====================
void sendPong() {
  StaticJsonDocument<64> doc;
  doc["type"] = "pong";
  doc["camera_ready"] = cameraReady;

  String output;
  serializeJson(doc, output);
  client.println(output);
}

void sendStatus() {
  StaticJsonDocument<256> doc;
  doc["type"] = "status";
  doc["device"] = "ESP32-CAM";
  doc["ip"] = WiFi.localIP().toString();
  doc["port"] = TCP_PORT;
  doc["camera_ready"] = cameraReady;
  doc["wifi_rssi"] = WiFi.RSSI();
  doc["uptime_ms"] = millis();

  String output;
  serializeJson(doc, output);
  client.println(output);
}

void sendError(const char* message) {
  StaticJsonDocument<128> doc;
  doc["type"] = "error";
  doc["message"] = message;

  String output;
  serializeJson(doc, output);
  client.println(output);
}

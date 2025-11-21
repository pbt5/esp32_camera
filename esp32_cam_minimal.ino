#include "esp_camera.h"
#include "camera_config.h"
#include <HardwareSerial.h>

HardwareSerial mySerial(1);
bool take_picture = false;

void setup() {
  Serial.begin(115200);
  mySerial.begin(921600, SERIAL_8N1, 3, 1); // RX=3, TX=1

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA; 
  config.jpeg_quality = 30;
  config.fb_count = 1;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (1);
  }

  Serial.println("Camera ready");
}

// Waits for trigger before taking and sending picture
void loop() {
  if (mySerial.available()) {
    char incoming = mySerial.read();
    if (incoming == 1) {
      take_picture = true;
    }
  }

  if (!take_picture) {
    return; 
  }

  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  delay(30);

  mySerial.write("IMG0", 4);

  uint32_t imgSize = fb->len;
  mySerial.write((uint8_t*)&imgSize, sizeof(imgSize));

  mySerial.write(fb->buf, fb->len);

  esp_camera_fb_return(fb);

  Serial.println("Image sent");

  take_picture = false;
}

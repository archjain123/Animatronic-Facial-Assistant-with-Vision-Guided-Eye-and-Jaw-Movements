// SPDX-FileCopyrightText: 2018 me-no-dev for Espressif Systems
//
// SPDX-License-Identifier: LGPL-2.1-or-later
//
// Modified by Brent Rubell for Adafruit Industries for use with Adafruit
// MEMENTO as a shoulder-mounted camera robot
// Further modified to add ESP-NOW communication for face tracking

#include "TJpg_Decoder.h"
#include "esp_camera.h"
#include "face_recognition_112_v1_s8.hpp"
#include "face_recognition_tool.hpp"
#include "fb_gfx.h"
#include "human_face_detect_mnp01.hpp"
#include "human_face_detect_msr01.hpp"
#include "ra_filter.h"
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include <vector>
#include <esp_now.h>
#include <WiFi.h>

// ==================== ESP-NOW CONFIGURATION ====================
uint8_t receiverMacAddress[] = {0x80, 0xF3, 0xDA, 0x54, 0x75, 0xC4};
//80:f3:da:54:75:c4
typedef struct struct_message {
  int16_t face_x;
  int16_t face_y;
  uint8_t face_detected;
  int16_t face_width;
  int16_t face_height;
  uint32_t timestamp;
} struct_message;

struct_message faceData;

// REQUIRED for ESP-NOW in Arduino Core 3.x
esp_now_peer_info_t peerInfo = {};

// ==================== DISPLAY CONFIGURATION ====================
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RESET);

// Framebuffer class
class PyCameraFB : public GFXcanvas16 {
public:
  PyCameraFB(uint16_t w, uint16_t h) : GFXcanvas16(w, h) {
    free(buffer);
    buffer = NULL;
  };

  void setFB(uint16_t *fb) { buffer = fb; }
};

// ==================== FACE DETECTION CONFIG ====================
camera_fb_t *fb = NULL;
PyCameraFB *pyCameraFb = NULL;

FaceRecognition112V1S8 recognizer;
HumanFaceDetectMSR01 s1(0.1F, 0.5F, 10, 0.2F);
HumanFaceDetectMNP01 s2(0.5F, 0.3F, 5);

bool isTrackingFace;
unsigned long prvDetectTime = 0UL;

int cur_face_box_x_midpoint = 0;
int cur_face_box_y_midpoint = 0;

// ==================== ESP-NOW SEND CALLBACK ====================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// ==================== ESP-NOW INITIALIZATION ====================
bool initESPNow() {
  WiFi.mode(WIFI_STA);
  delay(100);

  Serial.print("MEMENTO MAC Address: ");
  Serial.println(WiFi.macAddress());

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERROR: ESP-NOW init failed");
    return false;
  }

  esp_now_register_send_cb(OnDataSent);

  memset(&peerInfo, 0, sizeof(peerInfo));
  memcpy(peerInfo.peer_addr, receiverMacAddress, 6);
  peerInfo.channel = 0;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("ERROR: Failed to add peer");
    return false;
  }

  Serial.println("ESP-NOW initialized successfully!");
  return true;
}

// ==================== SEND FACE DATA ====================
void sendFaceData() {
  esp_err_t result = esp_now_send(receiverMacAddress, (uint8_t *)&faceData, sizeof(faceData));

  if (result != ESP_OK) {
    Serial.print("ESP-NOW Send Error: ");
    Serial.println(result);
  }
}

// ==================== DRAW FACE BOXES ====================
static void draw_face_boxes(fb_data_t *fb,
                            std::list<dl::detect::result_t> *results) {
  int x, y, w, h;
  uint32_t color = ST77XX_GREEN;

  if (fb->bytes_per_pixel == 2) {
    color = ((color >> 16) & 0x001F) | ((color >> 3) & 0x07E0) |
            ((color << 8) & 0xF800);
  }

  for (auto &prediction : *results) {
    x = prediction.box[0];
    y = prediction.box[1];
    w = prediction.box[2] - x + 1;
    h = prediction.box[3] - y + 1;

    tft.drawFastHLine(x, y, w, color);
    tft.drawFastHLine(x, y + h - 1, w, color);
    tft.drawFastVLine(x, y, h, color);
    tft.drawFastVLine(x + w - 1, y, h, color);

    cur_face_box_x_midpoint = x + (w / 2);
    cur_face_box_y_midpoint = y + (h / 2);

    tft.fillCircle(cur_face_box_x_midpoint, cur_face_box_y_midpoint, 5, ST77XX_BLUE);

    // update the struct to send
    faceData.face_x = cur_face_box_x_midpoint;
    faceData.face_y = cur_face_box_y_midpoint;
    faceData.face_width = w;
    faceData.face_height = h;
  }
}

// ==================== DISPLAY INITIALIZATION ====================
bool initDisplay() {
  pinMode(45, OUTPUT);
  digitalWrite(45, LOW);
  tft.init(240, 240);
  tft.setRotation(1);
  tft.fillScreen(ST77XX_GREEN);
  digitalWrite(45, HIGH);
  return true;
}

// ==================== CAMERA INITIALIZATION ====================
bool initCamera() {
  Wire.begin(34, 33);

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

  config.pin_sccb_sda = -1;
  config.pin_sccb_scl = -1;

  config.pin_pwdn = 21;
  config.pin_reset = 47;

  config.xclk_freq_hz = 20000000;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.frame_size = FRAMESIZE_240X240;
  config.pixel_format = PIXFORMAT_RGB565;
  config.fb_count = 2;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    return false;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1);
  s->set_hmirror(s, 0);

  return true;
}

// ==================== DETECT FACE ====================
std::list<dl::detect::result_t> performFaceDetection() {
  fb = esp_camera_fb_get();
  if (!fb) return {};

  std::list<dl::detect::result_t> cands =
      s1.infer((uint16_t *)fb->buf, { (int)fb->height, (int)fb->width, 3 });

  std::list<dl::detect::result_t> res =
      s2.infer((uint16_t *)fb->buf, { (int)fb->height, (int)fb->width, 3 }, cands);

  return res;
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  Serial.println("\n=== MEMENTO Face Tracker + ESP-NOW ===");

  if (!initCamera()) while (1);
  if (!initDisplay()) while (1);
  pyCameraFb = new PyCameraFB(240, 240);

  if (!initESPNow()) while (1);

  ra_filter_init(&ra_filter, 20);
  recognizer.set_partition(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "fr");
}

// ==================== MAIN LOOP ====================
void loop() {
  auto detected = performFaceDetection();

  if (!detected.empty()) {
    faceData.face_detected = 1;
    faceData.timestamp = millis();

    fb_data_t rfb;
    rfb.width = fb->width;
    rfb.height = fb->height;
    rfb.data = fb->buf;
    rfb.bytes_per_pixel = 2;
    rfb.format = FB_RGB565;

    draw_face_boxes(&rfb, &detected);

  } else {
    faceData.face_detected = 0;
    faceData.face_x = -1;
    faceData.face_y = -1;
    faceData.face_width = 0;
    faceData.face_height = 0;
    faceData.timestamp = millis();
  }

  sendFaceData();

  // swap bytes & draw
  for (uint32_t i = 0; i < fb->len; i += 2) {
    uint8_t t = fb->buf[i];
    fb->buf[i] = fb->buf[i + 1];
    fb->buf[i + 1] = t;
  }

  pyCameraFb->setFB((uint16_t *)fb->buf);
  tft.drawRGBBitmap(0, 0, (uint16_t *)pyCameraFb->getBuffer(), 240, 240);

  esp_camera_fb_return(fb);
}

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <myfunction.h>

#include "esp_camera.h"

// Import required libraries
// #include <Adafruit_MLX90640.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include "html.h"

// Bolometer - Replace with your own pinout
#define I2C_SCL 13
#define I2C_SDA 12

//
// WARNING!!! PSRAM IC required for UXGA resolution and high JPEG quality
//            Ensure ESP32 Wrover Module or other board with PSRAM is selected
//            Partial images will be transmitted if image exceeds buffer size
//
//            You must select partition scheme from the board menu that has at least 3MB APP space.
//            Face Recognition is DISABLED for ESP32 and ESP32-S2, because it takes up from 15
//            seconds to process single frame. Face Detection is ENABLED if PSRAM is enabled as well

// ===================
// Select camera model
// ===================
// #define CAMERA_MODEL_WROVER_KIT // Has PSRAM
// #define CAMERA_MODEL_ESP_EYE // Has PSRAM
// #define CAMERA_MODEL_ESP32S3_EYE // Has PSRAM
// #define CAMERA_MODEL_M5STACK_PSRAM // Has PSRAM
// #define CAMERA_MODEL_M5STACK_V2_PSRAM // M5Camera version B Has PSRAM
// #define CAMERA_MODEL_M5STACK_WIDE // Has PSRAM
// #define CAMERA_MODEL_M5STACK_ESP32CAM // No PSRAM
// #define CAMERA_MODEL_M5STACK_UNITCAM // No PSRAM
#define CAMERA_MODEL_AI_THINKER  // Has PSRAM
// #define CAMERA_MODEL_TTGO_T_JOURNAL // No PSRAM
//  ** Espressif Internal Boards **
// #define CAMERA_MODEL_ESP32_CAM_BOARD
// #define CAMERA_MODEL_ESP32S2_CAM_BOARD
// #define CAMERA_MODEL_ESP32S3_CAM_LCD

#include "camera_pins.h"

// ===========================
// Enter your WiFi credentials
// ===========================

// Bolometer stuff
// Adafruit_MLX90640 mlx;
const size_t thermSize = (32 * 24) * sizeof(float);
const size_t frameSize = thermSize + 30000 * sizeof(char);
size_t imageSize = 0;
char frame[frameSize];  // buffer for full frame of temperatures and image

// Websocket stuff
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

void log(String text) {
	Serial.println(text);
}

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
	// AwsFrameInfo *info = (AwsFrameInfo *)arg;
	// if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
	// 	data[len] = 0;
	// 	String command = String((char *)data);
	// 	if (command == "status") {
	// 	}
	// }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
			 void *arg, uint8_t *data, size_t len) {
	switch (type) {
		case WS_EVT_CONNECT:
			break;
		case WS_EVT_DISCONNECT:
			break;
		case WS_EVT_DATA:
			handleWebSocketMessage(arg, data, len);
			break;
		case WS_EVT_PONG:
		case WS_EVT_ERROR:
			break;
	}
}

void setup() {
	Serial.begin(115200);
	Serial.setDebugOutput(true);
	Serial.println();

	mySmartConfig();

	log("Setting up bolometer");
	// Wire.begin(I2C_SDA, I2C_SCL);
	// Wire.beginTransmission(MLX90640_I2CADDR_DEFAULT);

	// mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire);
	// mlx.setMode(MLX90640_CHESS);
	// mlx.setResolution(MLX90640_ADC_16BIT);
	// mlx90640_resolution_t res = mlx.getResolution();
	// mlx.setRefreshRate(MLX90640_16_HZ);
	// mlx90640_refreshrate_t rate = mlx.getRefreshRate();
	// Wire.setClock(1000000);  // max 1 MHz
	log("Bolometer setup");

	log("Setting up camera");
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
	config.xclk_freq_hz = 10000000;
	config.pixel_format = PIXFORMAT_JPEG;
	config.frame_size = FRAMESIZE_QVGA;
	// config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
	config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
	config.fb_location = CAMERA_FB_IN_PSRAM;
	config.jpeg_quality = 10;
	config.fb_count = 2;

	// if PSRAM IC present, init with UXGA resolution and higher JPEG quality
	//                      for larger pre-allocated frame buffer.
	// 	if (config.pixel_format == PIXFORMAT_JPEG) {
	// 		if (psramFound()) {
	// 			config.jpeg_quality = 10;
	// 			config.fb_count = 2;
	// 			config.grab_mode = CAMERA_GRAB_LATEST;
	// 		} else {
	// 			// Limit the frame size when PSRAM is not available
	// 			config.frame_size = FRAMESIZE_SVGA;
	// 			config.fb_location = CAMERA_FB_IN_DRAM;
	// 		}
	// 	} else {
	// 		// Best option for face detection/recognition
	// 		config.frame_size = FRAMESIZE_240X240;
	// #if CONFIG_IDF_TARGET_ESP32S3
	// 		config.fb_count = 2;
	// #endif
	// 	}

	// #if defined(CAMERA_MODEL_ESP_EYE)
	// 	pinMode(13, INPUT_PULLUP);
	// 	pinMode(14, INPUT_PULLUP);
	// #endif

	// camera init
	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK) {
		Serial.printf("Camera init failed with error 0x%x", err);
		return;
	}

	sensor_t *s = esp_camera_sensor_get();
	// initial sensors are flipped vertically and colors are a bit saturated
	if (s->id.PID == OV3660_PID) {
		s->set_vflip(s, 1);        // flip it back
		s->set_brightness(s, 1);   // up the brightness just a bit
		s->set_saturation(s, -2);  // lower the saturation
	}
	// drop down frame size for higher initial frame rate
	if (config.pixel_format == PIXFORMAT_JPEG) {
		s->set_framesize(s, FRAMESIZE_QVGA);
	}

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
	s->set_vflip(s, 1);
	s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
	s->set_vflip(s, 1);
#endif
	log("Camera setup");

	log("Setting web server");
	ws.onEvent(onEvent);
	server.addHandler(&ws);
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", index_html); });
	server.begin();
	log("Webserver setup");

	pinMode(4, OUTPUT);
	digitalWrite(4, LOW);

	ArduinoOTA.begin();
}

void take_snapshot() {
	camera_fb_t *fb = NULL;
	fb = esp_camera_fb_get();
	if (!fb) {
		log("Camera capture failed. Restarting");
		ESP.restart();
	}
	log("Moving image to frame buffer " + String(fb->len) + " + " + String(thermSize) + " < " + String(frameSize));
	memcpy(&frame[thermSize], fb->buf, fb->len);
	log("Image moved to frame buffer");
	imageSize = fb->len;
	esp_camera_fb_return(fb);
	fb = NULL;
}

void take_thermal() {
	log("Taking thermal data to buffer");
	// mlx.getFrame((float *)frame);
	log("Thermal data taken");
}

unsigned long messageTimestamp = 0;
int messageCounter = 0;

void loop() {
	ArduinoOTA.handle();
	ws.cleanupClients();
	// uint64_t now = millis();
	// if (now - messageTimestamp > 1000) {
	// 	memset(frame, 0, frameSize);
	// 	take_thermal();
	// 	take_snapshot();
	// 	log("Sending data");
	// 	ws.binaryAll(frame, thermSize + imageSize);
	// 	log("Data sent");
	// 	messageTimestamp = now;
	// 	messageCounter++;
	// 	if (messageCounter > 30) {
	// 		sendStatus();
	// 		messageCounter = 0;
	// 	}
	// }
	delay(100);
}
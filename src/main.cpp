#include <Arduino.h>
#include <ArduinoOTA.h>
// #include <WiFi.h>
#include <Wire.h>
#include <ESPAsyncWebServer.h>
#include "esp_camera.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include <myfunction.h>


// Import required libraries
 #include <Adafruit_MLX90640.h>
// #include <AsyncTCP.h>
#include "html.h"

const byte MLX90640_address = 0x33;  // Default 7-bit unshifted address of the MLX90640
paramsMLX90640 mlx90640;
float mlx90640To[768];
#define TA_SHIFT 8  // Default shift for MLX90640 in open air
#define I2C_SCL 14	// pb avec 12 et 13 sur ESP32 CAM
#define I2C_SDA 2

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
Adafruit_MLX90640 mlx;
const size_t thermSize = (32 * 24) * sizeof(float);
const size_t frameSize = thermSize + 30000 * sizeof(char);
size_t imageSize = 0;
char frame[frameSize];  // buffer for full frame of temperatures and image

// Websocket stuff
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
// 	// AwsFrameInfo *info = (AwsFrameInfo *)arg;
// 	// if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
// 	// 	data[len] = 0;
// 	// 	String command = String((char *)data);
// 	// 	if (command == "status") {
// 	// 	}
// 	// }
// }

// void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
// 			 void *arg, uint8_t *data, size_t len) {
// 	switch (type) {
// 		case WS_EVT_CONNECT:
// 			break;
// 		case WS_EVT_DISCONNECT:
// 			break;
// 		case WS_EVT_DATA:
// 			// handleWebSocketMessage(arg, data, len);
// 			break;
// 		case WS_EVT_PONG:
// 		case WS_EVT_ERROR:
// 			break;
// 	}
// }

boolean initCamera(){
	DEBUGLOG("Setting up camera\n");
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
		DEBUGLOG("Camera init failed with error 0x%x", err);
		return false;
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
	DEBUGLOG("Camera setup");
	return true;
}

// Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected() {
	Wire.beginTransmission((uint8_t)MLX90640_address);
	if (Wire.endTransmission() != 0)
		return (false);  // Sensor did not ACK
	return (true);
}

boolean initMLX90640(){
	Wire.begin(I2C_SDA, I2C_SCL, 400000); // Increase I2C clock speed to 400kHz

	if (isConnected() )
		DEBUGLOG("MLX90640 online !\n");
	else
		DEBUGLOG("MLX90640 not detected at default I2C address. Please check wiring.\n");

	// Get device parameters - We only have to do this once
	int status = 0;
	uint16_t eeMLX90640[832];
	status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
	if (status)
		DEBUGLOG("MLX90640 Failed to load system parameters\n");
	else
		DEBUGLOG("MLX90640 system parameters loaded\n");

	status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
	if (status)
		DEBUGLOG("MLX90640 Parameter extraction failed\n");
	else
		DEBUGLOG("MLX90640 Parameter extracted\n");

	int SetRefreshRate = MLX90640_SetRefreshRate(MLX90640_address, 0x03);
	// int SetInterleavedMode = MLX90640_SetInterleavedMode(MLX90640_address);
	int SetChessMode = MLX90640_SetChessMode(MLX90640_address);

	return true;
}


boolean initMLX90640_v2(){
	Wire.begin(I2C_SDA, I2C_SCL, 400000); // Increase I2C clock speed to 400kHz
	Wire.beginTransmission(MLX90640_I2CADDR_DEFAULT);

	mlx.begin(MLX90640_I2CADDR_DEFAULT, &Wire);
	mlx.setMode(MLX90640_CHESS);
	mlx.setResolution(MLX90640_ADC_16BIT);
	mlx90640_resolution_t res = mlx.getResolution();
	mlx.setRefreshRate(MLX90640_16_HZ);
	mlx90640_refreshrate_t rate = mlx.getRefreshRate();
	Wire.setClock(1000000);  // max 1 MHz

	return true;
}

// SETUP
//==========================================================================

void setup() {
	DEBUGINIT();
	mySmartConfig();

	// ESP32 As access point
	// WiFi.mode(WIFI_AP);  // Access Point mode
	// WiFi.softAP(ssid, password);

	initMLX90640_v2();

	DEBUGLOG("Setting web server\n");
	// ws.onEvent(onEvent);
	server.addHandler(&ws);
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) { request->send_P(200, "text/html", index_html); });
	server.begin();
	DEBUGLOG("Webserver setup\n");

	pinMode(4, OUTPUT);
	digitalWrite(4, LOW);

	ArduinoOTA.begin();
}

void take_snapshot() {
	camera_fb_t *fb = NULL;
	fb = esp_camera_fb_get();
	if (!fb) {
		DEBUGLOG("ERROR : Camera capture failed. Restarting\n");
	}
	DEBUGLOG("Moving image to frame buffer %d + %d < %d\n", fb->len ,thermSize,frameSize);
	memcpy(&frame[thermSize], fb->buf, fb->len);
	DEBUGLOG("Image moved to frame buffer\n");
	imageSize = fb->len;
	esp_camera_fb_return(fb);
	fb = NULL;
}

void take_thermal() {
	DEBUGLOG("Taking thermal data to buffer\n");
	mlx.getFrame((float *)frame);
	DEBUGLOG("Thermal data taken\n");
}

unsigned long messageTimestamp = 0;
int messageCounter = 0;

void loop() {
	ArduinoOTA.handle();
	ws.cleanupClients();
	uint64_t now = millis();
	if (now - messageTimestamp > 1000) {
		memset(frame, 0, frameSize);
		take_thermal();
		take_snapshot();
		DEBUGLOG("Sending data ...");
		ws.binaryAll(frame, thermSize + imageSize);
		DEBUGLOG("Data sent\n");
		messageTimestamp = now;
		messageCounter++;
		if (messageCounter > 30) {
			// sendStatus();
			messageCounter = 0;
		}
	}
	delay(100);
}
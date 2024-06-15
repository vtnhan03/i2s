#include <Arduino.h>
#include <WiFi.h>
#include <driver/i2s.h>
#include <esp_task_wdt.h>
#include "I2SMicSampler.h"
#include "ADCSampler.h"
#include "config.h"
#include "Application.h"
#include "SPIFFS.h"
#include "IndicatorLight.h"
#include "WebSocketsClient.h"
#include "DHT.h"

#define DHTPIN 4      // Pin kết nối với cảm biến DHT
#define DHTTYPE DHT22 // DHT 22 (AM2302), AM2321

DHT dht(DHTPIN, DHTTYPE);
const char *webSocketServer = "13.229.105.152";
const int webSocketPort = 2003;
int ledPin1 = 19;
int ledPin2 = 18;
int ledPin3 = 5;
int ledPin4 = 17;
const char *BDPN = "BDPN";
const char *TDPN = "TDPN";
const char *BQPN = "BQPN";
const char *TQPN = "TQPN";
const char *BDPK = "BDPK";
const char *TDPK = "TDPK";
const char *BQPK = "BQPK";
const char *TQPK = "TQPK";
size_t expected_length = strlen(BDPN);
WebSocketsClient webSocket;

// i2s config for using the internal ADC
i2s_config_t adcI2SConfig = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_LSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s config for reading from both channels of I2S
i2s_config_t i2sMemsConfigBothChannels = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = 16000,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_MIC_CHANNEL,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0};

// i2s microphone pins
i2s_pin_config_t i2s_mic_pins = {
    .bck_io_num = I2S_MIC_SERIAL_CLOCK,
    .ws_io_num = I2S_MIC_LEFT_RIGHT_CLOCK,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_MIC_SERIAL_DATA};

// This task does all the heavy lifting for our application
void applicationTask(void *param)
{
  Application *application = static_cast<Application *>(param);

  const TickType_t xMaxBlockTime = pdMS_TO_TICKS(100);
  while (true)
  {
    // wait for some audio samples to arrive
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, xMaxBlockTime);
    if (ulNotificationValue > 0)
    {
      application->run();
    }
  }
}
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.println("Disconnected from WebSocket");
    break;
  case WStype_CONNECTED:
    Serial.println("Connected to WebSocket");
    webSocket.sendTXT("name:ESP");
    break;
  case WStype_TEXT:
    Serial.printf("Received text: %s\n", payload);
    if (memcmp(payload, BDPN, expected_length) == 0)
    {
      digitalWrite(ledPin1, HIGH);
    }
    else if (memcmp(payload, TDPN, expected_length) == 0)
    {
      digitalWrite(ledPin1, LOW);
    }
    else if (memcmp(payload, BQPN, expected_length) == 0)
    {
      digitalWrite(ledPin2, HIGH);
    }
    else if (memcmp(payload, TQPN, expected_length) == 0)
    {
      digitalWrite(ledPin2, LOW);
    }
    else if (memcmp(payload, BDPK, expected_length) == 0)
    {
      digitalWrite(ledPin3, HIGH);
    }
    else if (memcmp(payload, TDPK, expected_length) == 0)
    {
      digitalWrite(ledPin3, LOW);
    }
    else if (memcmp(payload, BQPK, expected_length) == 0)
    {
      digitalWrite(ledPin4, HIGH);
    }
    else if (memcmp(payload, TQPK, expected_length) == 0)
    {
      digitalWrite(ledPin4, LOW);
    }
    break;
  }
}

void loop1(void *pvParameters)
{
  while (true)
  {
    webSocket.loop();
    delay(10);
  }
}
void sendTemperatureAndHumidity(void *pvParameters)
{
  while (true)
  {
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t))
    {
      Serial.println("Failed to read from DHT sensor!");
    }
    else
    {
      String payload = "infor:" + String(t) + "_" + String(h);
      webSocket.sendTXT(payload);
      Serial.println("Sent: " + payload);
    }
    vTaskDelay(5000 / portTICK_PERIOD_MS); // Delay 2 seconds
  }
}
void setup()
{
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  pinMode(ledPin3, OUTPUT);
  pinMode(ledPin4, OUTPUT);
  Serial.begin(115200);
  delay(1000);
  Serial.println("Starting up");
  dht.begin();

  // start up wifi
  // launch WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSWD);
  if (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  Serial.printf("Total heap: %d\n", ESP.getHeapSize());
  Serial.printf("Free heap: %d\n", ESP.getFreeHeap());
  webSocket.begin(webSocketServer, uint16_t(webSocketPort));
  webSocket.onEvent(webSocketEvent);
  // xTaskCreatePinnedToCore(loop1, "loop1", 4096, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(sendTemperatureAndHumidity, "sendTemperatureAndHumidity", 4096, NULL, 1, NULL, 0);

  // startup SPIFFS for the wav files
  // make sure we don't get killed for our long running tasks
  esp_task_wdt_init(10, false);

  // start up the I2S input (from either an I2S microphone or Analogue microphone via the ADC)
#ifdef USE_I2S_MIC_INPUT
  // Direct i2s input from INMP441 or the SPH0645
  I2SSampler *i2s_sampler = new I2SMicSampler(i2s_mic_pins, false);
#else
  // Use the internal ADC
  I2SSampler *i2s_sampler = new ADCSampler(ADC_UNIT_1, ADC_MIC_CHANNEL);
#endif

  // indicator light to show when we are listening
  IndicatorLight *indicator_light = new IndicatorLight();

  // and the intent processor

  // create our application
  Application *application = new Application(i2s_sampler, indicator_light);

  // set up the i2s sample writer task
  TaskHandle_t applicationTaskHandle;
  xTaskCreate(applicationTask, "Application Task", 8192, application, 1, &applicationTaskHandle);

  // start sampling from i2s device - use I2S_NUM_0 as that's the one that supports the internal ADC
#ifdef USE_I2S_MIC_INPUT
  i2s_sampler->start(I2S_NUM_0, i2sMemsConfigBothChannels, applicationTaskHandle);
#else
  i2s_sampler->start(I2S_NUM_0, adcI2SConfig, applicationTaskHandle);
#endif
}

void loop()
{
  webSocket.loop();
  // vTaskDelay(1000);
}
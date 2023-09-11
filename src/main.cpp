#include <Arduino.h>
#include <Wire.h>

#include <SPI.h>
#include <TFT_eSPI.h>
#include "FT62XXTouchScreen.h"

#include "mi_scale_gw_conf.h"
#include "MiScaleGatewayEnums.h"

#include <WiFi.h>
#include <EEPROM.h>
#include <MQTT.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <NimBLEDevice.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>

static const uint16_t screenWidth = 480;
static const uint16_t screenHeight = 320;

TFT_eSPI tft = TFT_eSPI();
FT62XXTouchScreen touchScreen = FT62XXTouchScreen(screenHeight, PIN_SDA, PIN_SCL);

#include "esp_freertos_hooks.h"
#include "ui/ui.h"
#define BUFFER_SIZE (screenWidth * screenHeight / 10)

static lv_disp_draw_buf_t disp_buf;
static lv_color_t *screenBuffer1;

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p);
static void touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data);

static void lv_tick_task(void *arg);
static void lv_handler_task(void *arg);
static void update_screen_task(void *arg);

esp_timer_handle_t ticker_timer;
esp_timer_handle_t handler_timer;
esp_timer_handle_t update_screen_timer;
const esp_timer_create_args_t ticker_timer_args = {
    .callback = &lv_tick_task,
    .name = "lv_tick_task"};
const esp_timer_create_args_t handler_timer_args = {
    .callback = &lv_handler_task,
    .name = "lv_handler_task"};
const esp_timer_create_args_t update_screen_timer_args = {
    .callback = &update_screen_task,
    .name = "update_screen_task"};

MQTTClient mqttClient;
WiFiClient wifiClient;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP);

String deviceMAC;
uint32_t wifiLastTs;
miscale_gateway_wifi_state wifiState = MISCALE_GATEWAY_WL_DISCONNECTED;

String mqttClientName;
uint32_t mqttLastTs;
miscale_gateway_mqtt_state mqttState = MISCALE_GATEWAY_MQTT_DISCONNECTED;

// Weight Scale service
static BLEUUID weightScaleServiceUUID("0000181b-0000-1000-8000-00805f9b34fb");
// Weight Measurement characteristic
static BLEUUID weightScaleCharUUID("00002a9c-0000-1000-8000-00805f9b34fb");
uint32_t bleLastTs;
miscale_gateway_ble_state bleState = MISCALE_GATEWAY_BLE_DISCONNECTED;
BLEClient *pClient;
BLEAdvertisedDevice *weightScaleDevice;
BLERemoteCharacteristic *remoteChr;

int weightScaleYear;
double lastestWeightRead = NAN;
uint8_t bleReadCount = 0;
bool isWeightStable = false;
miscale_gateway_weight_send_state weightSendSuccessful = MISCALE_GATEWAY_WEIGHT_SEND_WAITING;

String wifiSSID = "wifiSSID";
String wifiPassword = "wifiPassword";

miscale_gateway_input_target inputTarget = MISCALE_GATEWAY_NO_TARGET;
miscale_gateway_info_action infoAction = MISCALE_GATEWAY_NO_ACTION;
String inputTemp;
String optionWifiSSID;
String optionWifiPassword;

bool otaStart = false;

void lv_obj_toggle_display(lv_obj_t *obj, bool display);
void lv_obj_toggle_clickable(lv_obj_t *obj, bool clickable);

static void initEEPROM();
static void migrateEEPROMDataVersion();
static void updateEEPROM();
static void displayOptionValues();

static void beginWifi(String &wifiSSID, String &wifiPassword);
static bool isWifiConnected();
static bool isWifiRunning();
static bool isWifiNeedReconnected();

static void beginMqtt();
static bool isMqttConnected();
static bool isMqttRunning();
static bool isMqttNeedReconnected();
void sendWeightMqtt();

void beginBle();
void resetBle();
static bool isBleDeviceFound();
static bool isBleRunning();
static bool isBleConnected();
static void bleWeightScaleCallback(BLERemoteCharacteristic *remoteChr, uint8_t *pData, size_t length, bool isNotify);

void beginOtaFirmware();
static void actualOtaAction();
static uint16_t otaCheckCloudVersion();
static void otaUpdateFirmware();

static void changeInputValueFrom(miscale_gateway_input_target target);
bool isOptionDirty(miscale_gateway_input_target target);
bool isOptionsDirty();
bool isOptionValid(miscale_gateway_input_target target);
bool isOptionsValid();
void restoreSaveToOptions();
void applySaveOptions();
void restoreOptionToTemp(miscale_gateway_input_target target);
void applyValueToOption(String value, miscale_gateway_input_target target);

void transitionToHomeScreen();
void transitionToOptionScreen();
void transitionToInputScreen(miscale_gateway_input_target target);
void changeInfoScreen(String message, bool showOK, bool showCancel);
void transitionToInfoScreen();
void transitionToCheckVersionScreen();
void transitionToOtaFirmwareScreen();

static void updateScaleScreen();

class ScanBleCallback : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice *advertisedDevice)
  {
    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(weightScaleServiceUUID))
    {
      if (advertisedDevice->getName() == "MIBFS")
      {
#if MISCALE_GATEWAY_DEBUG_BLE
        Serial.println("Miscale Found!");
#endif
        BLEDevice::getScan()->stop();
#if MISCALE_GATEWAY_DEBUG_BLE
        Serial.println("Stopping scan and connecting to scale");
#endif
        weightScaleDevice = advertisedDevice;
      }
    }
  }
};
/**
   Callback class for device events
*/
class BleConnectCallback : public BLEClientCallbacks
{
  void onDisconnect(BLEClient *pclient)
  {
#if MISCALE_GATEWAY_DEBUG_BLE
    Serial.println("Disconnected. Reconnecting...");
#endif
    bleState = MISCALE_GATEWAY_BLE_DISCONNECTED;
  }
};

void setup()
{
  Serial.begin(115200);
  pClient = BLEDevice::createClient();

#if MISCALE_GATEWAY_ALLOW_EEPROM
#if MISCALE_GATEWAY_DEBUG_EEPROM
  Serial.println("Read EEPROM");
#endif
  initEEPROM();
#endif
  deviceMAC = WiFi.macAddress();

  String mac = "";
  mac.concat(deviceMAC);
  mac.replace(":", "");
  mac.toUpperCase();

  mqttClientName = "miscale-";
  mqttClientName.concat(mac);

  beginWifi(wifiSSID, wifiPassword);

  mqttClient.begin(MISCALE_GATEWAY_MQTT_SERVER_URL, MISCALE_GATEWAY_MQTT_SERVER_PORT, wifiClient);

  BLEDevice::init("");
  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new ScanBleCallback());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  // Set active scan
  pBLEScan->setActiveScan(true);
  // Scan Forever
  pBLEScan->start(0, nullptr, false);

  // Init LVGL
  lv_init();
  ESP_ERROR_CHECK(esp_timer_create(&ticker_timer_args, &ticker_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(ticker_timer, portTICK_RATE_MS * 1000));

  // Enable TFT
  tft.begin();
  tft.setRotation(1);

  // Enable Backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, 1);

  // Start TouchScreen
  touchScreen.begin();

  // Display Buffer
  screenBuffer1 = (lv_color_t *)ps_malloc(BUFFER_SIZE * sizeof(lv_color_t));
  lv_disp_draw_buf_init(&disp_buf, screenBuffer1, NULL, BUFFER_SIZE);

  // Initialize the display
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &disp_buf;
  lv_disp_drv_register(&disp_drv);

  // Init Touchscreen
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  lv_indev_drv_register(&indev_drv);

  // Init generated ui component (and display it)
  ui_init();
  updateScaleScreen();

  // Init LVGL Update Timer
  ESP_ERROR_CHECK(esp_timer_create(&handler_timer_args, &handler_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(handler_timer, 10 * portTICK_RATE_MS * 1000));

  ESP_ERROR_CHECK(esp_timer_create(&update_screen_timer_args, &update_screen_timer));
  ESP_ERROR_CHECK(esp_timer_start_periodic(update_screen_timer, 50 * portTICK_RATE_MS * 1000));
}

void loop()
{
  timeClient.update();
  mqttClient.loop();

  if (isWifiNeedReconnected())
  {
    beginWifi(wifiSSID, wifiPassword);
  }

  // Set wifi status
  wl_status_t wifiRawStatus = WiFi.status();
  switch (wifiRawStatus)
  {
  case WL_IDLE_STATUS:
    wifiState = MISCALE_GATEWAY_WL_WAITING;
    break;
  case WL_CONNECTED:
    wifiState = MISCALE_GATEWAY_WL_CONNECTED;
    break;
  case WL_DISCONNECTED:
  case WL_CONNECTION_LOST:
    wifiState = MISCALE_GATEWAY_WL_DISCONNECTED;
    break;
  case WL_CONNECT_FAILED:
  case WL_NO_SSID_AVAIL:
  case WL_NO_SHIELD:
    wifiState = MISCALE_GATEWAY_WL_CONNECT_FAILED;
  }

  // check is wifi connected
  if (isWifiConnected() && isMqttNeedReconnected())
  {
    timeClient.begin();
    beginMqtt();
  }

  if (mqttState == MISCALE_GATEWAY_MQTT_WAITING)
  {
    lwmqtt_err_t mqttError = mqttClient.lastError();
    mqttState = mqttError != LWMQTT_SUCCESS ? MISCALE_GATEWAY_MQTT_CONNECT_FAILED : MISCALE_GATEWAY_MQTT_CONNECTED;
  }

  if (otaStart)
  {
    actualOtaAction();
    otaStart = false;
  }

  delay(5);
}

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors(&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}

static void touchpad_read(lv_indev_drv_t *drv, lv_indev_data_t *data)
{
  TouchPoint touchPos = touchScreen.read();
  if (touchPos.touched)
  {
    data->state = LV_INDEV_STATE_PR;
    data->point.x = touchPos.xPos;
    data->point.y = touchPos.yPos;
  }
  else
  {
    data->state = LV_INDEV_STATE_REL;
  }
}

static void lv_tick_task(void *arg)
{
  lv_tick_inc(portTICK_RATE_MS);
}

static void lv_handler_task(void *arg)
{
  lv_task_handler();
}

static void update_screen_task(void *arg)
{
  if (lv_scr_act() == ui_ScaleScreen)
  {
    updateScaleScreen();
  }
}

void lv_obj_toggle_display(lv_obj_t *obj, bool display)
{
  if (display)
  {
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_HIDDEN);
  }
  else
  {
    lv_obj_add_flag(obj, LV_OBJ_FLAG_HIDDEN);
  }
}

void lv_obj_toggle_clickable(lv_obj_t *obj, bool clickable)
{
  if (clickable)
  {
    lv_obj_add_flag(obj, LV_OBJ_FLAG_CLICKABLE);
  }
  else
  {
    lv_obj_clear_flag(obj, LV_OBJ_FLAG_CLICKABLE);
  }
}

void lv_obj_toggle_disabled(lv_obj_t *obj, bool disabled)
{
  if (disabled)
  {
    lv_obj_add_state(obj, LV_STATE_DISABLED);
  }
  else
  {
    lv_obj_clear_state(obj, LV_STATE_DISABLED);
  }
}

static void initEEPROM()
{
  if (!EEPROM.begin(EEPROM_TOTAL_BYTES))
  {
    Serial.println("EEPROM init error");
    return;
  }

  uint64_t header = EEPROM.readULong64(EEPROM_HEADER_ADDR);

#if MISCALE_GATEWAY_DEBUG_EEPROM
  Serial.print("EEPROM header: ");
  Serial.println(EEPROM_HEADER_KEY);
  Serial.print("Actual: ");
  Serial.println(header);
  Serial.print("Equal?: ");
  Serial.println(header == EEPROM_HEADER_KEY ? "T" : "F");
#endif

  if (header != EEPROM_HEADER_KEY)
  {
#if COLDSENSES_DEBUG_EEPROM
    Serial.println("Create New EEPROM Save");
#endif
    EEPROM.writeULong64(EEPROM_HEADER_ADDR, EEPROM_HEADER_KEY);
    EEPROM.writeUShort(EEPROM_VERSION_ADDR, MISCALE_GATEWAY_VERSION);
    EEPROM.writeString(EEPROM_SSID_ADDR, wifiSSID);
    EEPROM.writeString(EEPROM_WIFIPW_ADDR, wifiPassword);
    EEPROM.commit();
#if MISCALE_GATEWAY_DEBUG_EEPROM
    Serial.println("EEPROM Saved");
#endif
  }

  migrateEEPROMDataVersion();

#if MISCALE_GATEWAY_DEBUG_EEPROM
  Serial.println("Load EEPROM Save");
#endif

  // Read values
  wifiSSID = EEPROM.readString(EEPROM_SSID_ADDR);
  wifiPassword = EEPROM.readString(EEPROM_WIFIPW_ADDR);

#if MISCALE_GATEWAY_DEBUG_EEPROM
  Serial.println("EEPROM Values");
  displayOptionValues();
#endif
}

static void migrateEEPROMDataVersion()
{

#if MISCALE_GATEWAY_DEBUG_EEPROM
  Serial.println("Check EEPROM Save Version");
#endif
  uint16_t version = EEPROM.readUShort(EEPROM_VERSION_ADDR);

  if (version < MISCALE_GATEWAY_VERSION)
  {
    EEPROM.writeUShort(EEPROM_VERSION_ADDR, MISCALE_GATEWAY_VERSION);
    EEPROM.commit();
  }

#if MISCALE_GATEWAY_DEBUG_EEPROM
  Serial.println("EEPROM Save Updated");
#endif
}

static void updateEEPROM()
{
#if MISCALE_GATEWAY_DEBUG_EEPROM
  Serial.println("New EEPROM Values");
  displayOptionValues();
#endif

  EEPROM.writeULong64(EEPROM_HEADER_ADDR, EEPROM_HEADER_KEY);
  EEPROM.writeUShort(EEPROM_VERSION_ADDR, MISCALE_GATEWAY_VERSION);
  EEPROM.writeString(EEPROM_SSID_ADDR, wifiSSID);
  EEPROM.writeString(EEPROM_WIFIPW_ADDR, wifiPassword);
  EEPROM.commit();

#if MISCALE_GATEWAY_DEBUG_EEPROM
  bool isSuccess = EEPROM.commit();
  Serial.print("Update EEPROM: ");
  Serial.println(isSuccess ? "T" : "F");
#else
  EEPROM.commit();
#endif
}

static void displayOptionValues()
{
  Serial.print("wifiSSID: ");
  Serial.println(wifiSSID);
  Serial.print("wifiPassword: ");
  Serial.println(wifiPassword);
}

static void beginWifi(String &wifiSSID, String &wifiPassword)
{
  if (isWifiRunning())
  {
    return;
  }
#if MISCALE_GATEWAY_DEBUG_WIFI
  Serial.print("WIFI SSID: ");
  Serial.println(wifiSSID);
  Serial.print("WIFI PW: ");
  Serial.println(wifiPassword);
#endif
  WiFi.begin(wifiSSID.c_str(), wifiPassword.length() > 0 ? wifiPassword.c_str() : NULL);
  wifiLastTs = millis();
}

static bool isWifiConnected()
{
  return wifiState == MISCALE_GATEWAY_WL_CONNECTED;
}

static bool isWifiRunning()
{
  return wifiState != MISCALE_GATEWAY_WL_DISCONNECTED;
}

static bool isWifiNeedReconnected()
{
  return !isWifiConnected() && wifiState != MISCALE_GATEWAY_WL_WAITING && millis() - wifiLastTs > WIFI_RECONNECT_DELAY;
}

static void beginMqtt()
{
  if (isMqttRunning())
  {
    return;
  }
#if MISCALE_GATEWAY_DEBUG_MQTT
  Serial.print("mqttClientName:");
  Serial.println(mqttClientName);
  Serial.print("MAC:");
  Serial.println(deviceMAC);
#endif
  mqttLastTs = millis();
  mqttState = MISCALE_GATEWAY_MQTT_WAITING;
  mqttClient.connect(mqttClientName.c_str());
}

static bool isMqttConnected()
{
  return mqttState == MISCALE_GATEWAY_MQTT_CONNECTED;
}

static bool isMqttRunning()
{
  return mqttState != MISCALE_GATEWAY_MQTT_DISCONNECTED;
}

static bool isMqttNeedReconnected()
{
  return !isMqttConnected() && mqttState != MISCALE_GATEWAY_MQTT_WAITING && millis() - mqttLastTs > MQTT_RECONNECT_DELAY;
}

void sendWeightMqtt()
{
  if (!isMqttConnected() || isnan(lastestWeightRead))
  {
    return;
  }

  weightSendSuccessful = MISCALE_GATEWAY_WEIGHT_SEND_WAITING;

  String topic = "log/";
  topic.concat(WiFi.macAddress());

  uint32_t timestamp = timeClient.getEpochTime();
  String payload = String(timestamp);
  payload.concat(':');
  payload.concat(lastestWeightRead);
  bool isSuccess = mqttClient.publish(topic.c_str(), payload.c_str());
  weightSendSuccessful = isSuccess ? MISCALE_GATEWAY_WEIGHT_SEND_SUCCESS : MISCALE_GATEWAY_WEIGHT_SEND_FAILED;
}

void beginBle()
{
  if (!isBleDeviceFound() || isBleRunning())
  {
    return;
  }

  bleState = MISCALE_GATEWAY_BLE_WAITING;
  isWeightStable = false;
  bleReadCount = 0;
  bleLastTs = millis();

// fake weight test
#if MISCAN_GATEWAY_BLE_MOCKUP
  bleState = MISCALE_GATEWAY_BLE_CONNECTED;
  lastestWeightRead = 200.0;
  isWeightStable = true;
  return;
#endif

#if MISCALE_GATEWAY_DEBUG_BLE
  Serial.println("Establishing communications with scale:");
#endif
  // Connect to the remove BLE Server.
  pClient->connect(weightScaleDevice);

#if MISCALE_GATEWAY_DEBUG_BLE
  Serial.println("Connected to scale");
#endif

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient->getService(weightScaleServiceUUID);
  if (pRemoteService == nullptr)
  {
#if MISCALE_GATEWAY_DEBUG_BLE
    Serial.println("BLE Error: Failed to find service");
#endif
    pClient->disconnect();
    bleState = MISCALE_GATEWAY_BLE_CONNECT_FAILED;
    return;
  }
#if MISCALE_GATEWAY_DEBUG_BLE
  Serial.println("BLE Service found");
#endif
  remoteChr = pRemoteService->getCharacteristic(weightScaleCharUUID);
  if (remoteChr == nullptr)
  {
#if MISCALE_GATEWAY_DEBUG_BLE
    Serial.print("BLE Error: Failed to find characteristic");
#endif
    pClient->disconnect();
    bleState = MISCALE_GATEWAY_BLE_CONNECT_FAILED;
  }

#if MISCALE_GATEWAY_DEBUG_BLE
  Serial.println("BLE Characteristic found");
  Serial.println("BLE Setting callback for notify / indicate");
#endif
  bleState = MISCALE_GATEWAY_BLE_CONNECTED;
  remoteChr->subscribe(bleWeightScaleCallback);
}

void resetBle()
{
  bleState = MISCALE_GATEWAY_BLE_DISCONNECTED;
  lastestWeightRead = NAN;
}

static bool isBleDeviceFound()
{

#if MISCAN_GATEWAY_BLE_MOCKUP
  return true;
#else
  return weightScaleDevice != NULL;
#endif
}

static bool isBleRunning()
{
  return bleState != MISCALE_GATEWAY_BLE_DISCONNECTED;
}

static bool isBleConnected()
{
  return bleState == MISCALE_GATEWAY_BLE_CONNECTED;
}

static void bleWeightScaleCallback(BLERemoteCharacteristic *remoteChr, uint8_t *pData, size_t length, bool isNotify)
{
#if MISCALE_GATEWAY_DEBUG_BLE
  // Console debugging
  Serial.print("Received data. Length = ");
  Serial.print(length);
  Serial.print(". - Data bytes: ");
  for (int i = 0; i < length; i++)
  {
    Serial.print(pData[i]);
    Serial.print("  ");
  }
  Serial.println(" ");
#endif

  boolean temporary = true;
  int rcvdYear = pData[3];

  if (weightScaleYear == 0)
  {
    weightScaleYear = rcvdYear;
  }
  else
  {
    if (rcvdYear > weightScaleYear)
    {
      temporary = false;
    }
  }
  double weight = (pData[2] + pData[12] * 256) * 0.0051;

#if MISCALE_GATEWAY_DEBUG_BLE
  Serial.print("Weight: ");
  Serial.print(weight);
  Serial.print(" Kg - ");
  Serial.println(temporary ? " (Provisional)" : " (Definitive)");
#endif

  lastestWeightRead = weight;

  if (!temporary)
  {
    isWeightStable = true;
  }
  else if (millis() - bleLastTs >= BLE_STABLE_DELAY)
  {
    // read x time before
    if (bleReadCount < BLE_STABLE_READ_REQUIRED)
    {
      bleReadCount += 1;
      bleLastTs = millis();
    }
    else
    {
      isWeightStable = true;
    }
  }
}

String httpGETrequest(String path)
{
  HTTPClient http;
  http.begin(wifiClient, path);
  int httpGETCode = http.GET();

  String getResp = "";
  getResp = http.getString();
  http.end();

  return getResp;
}

void beginOtaFirmware()
{
  otaStart = true;
}

static void actualOtaAction()
{
  uint16_t cloudVersion = otaCheckCloudVersion();
#if MISCALE_GATEWAY_DEBUG_OTA
  Serial.print("Current Firmware version : ");
  Serial.println(MISCALE_GATEWAY_VERSION);
  Serial.print("Cloud version : ");
  Serial.println(cloudVersion);
#endif

  if (true || MISCALE_GATEWAY_VERSION >= cloudVersion)
  {
    lv_label_set_text(ui_InfoLabel, "Firmware up to date.");
    lv_obj_toggle_display(ui_InfoCancelBtn, true);
    lv_obj_toggle_clickable(ui_InfoCancelBtn, true);

#if MISCALE_GATEWAY_DEBUG_OTA
    Serial.println("No update");
#endif
  }
  else
  {
    otaUpdateFirmware();
  }
}

static uint16_t otaCheckCloudVersion()
{
  // check version
  String url = MISCALE_GATEWAY_OTA_SERVER_BASEURL;
  url.concat("/checkupdatedversion?type=");
  url.concat(MISCALE_GATEWAY_OTA_DEVICE_TYPE);

#if MISCALE_GATEWAY_DEBUG_OTA
  Serial.print("CHECK VERSION URL = ");
  Serial.println(url);
#endif

  return (uint16_t)httpGETrequest(url).toInt();
}

static void otaUpdateFirmware()
{
  String url = MISCALE_GATEWAY_OTA_SERVER_BASEURL;
  url.concat("/ota?type=");
  url.concat(MISCALE_GATEWAY_OTA_DEVICE_TYPE);
#if MISCALE_GATEWAY_DEBUG_OTA
  Serial.print("OTA URL = ");
  Serial.println(url);
#endif

  t_httpUpdate_return ret = httpUpdate.update(wifiClient, url);

  String errorUpdateText;
  switch (ret)
  {
  case HTTP_UPDATE_FAILED:
    lv_obj_toggle_display(ui_InfoCancelBtn, true);
    lv_obj_toggle_clickable(ui_InfoCancelBtn, true);

    errorUpdateText = httpUpdate.getLastErrorString();
    lv_label_set_text(ui_InfoLabel, errorUpdateText.c_str());
#if MISCALE_GATEWAY_DEBUG_OTA
    Serial.println("HTTP_UPDATE_FAILED");
    Serial.println(errorUpdateText);
#endif
    break;
  case HTTP_UPDATE_NO_UPDATES:
    lv_label_set_text(ui_InfoLabel, "Firmware up to date.");
    lv_obj_toggle_display(ui_InfoCancelBtn, true);
    lv_obj_toggle_clickable(ui_InfoCancelBtn, true);

#if MISCALE_GATEWAY_DEBUG_OTA
    Serial.println("HTTP_UPDATE_NO_UPDATES");
#endif
    break;
  case HTTP_UPDATE_OK:
    // lv_label_set_text(ui_InfoLabel, "Firmware updated.");
#if MISCALE_GATEWAY_DEBUG_OTA
    Serial.println("HTTP_UPDATE_OK");
#endif
    ESP.restart();
    break;
  default:
    lv_label_set_text(ui_InfoLabel, "Can't update firmware.");
    lv_obj_toggle_display(ui_InfoCancelBtn, true);
    lv_obj_toggle_clickable(ui_InfoCancelBtn, true);
    break;
  }
}

static void changeInputValueFrom(miscale_gateway_input_target target)
{
  switch (target)
  {
  case MISCALE_GATEWAY_TARGET_WIFI_SSID:
    lv_textarea_set_text(ui_InputTextArea, optionWifiSSID.c_str());
    lv_textarea_set_max_length(ui_InputTextArea, SSID_MAXLENGTH);
    break;
  case MISCALE_GATEWAY_TARGET_WIFI_PASSWORD:
    lv_textarea_set_text(ui_InputTextArea, optionWifiPassword.c_str());
    lv_textarea_set_max_length(ui_InputTextArea, WIFIPW_MAXLENGTH);
    break;
  default:
    break;
  }
}

bool isOptionDirty(miscale_gateway_input_target target)
{
  switch (target)
  {
  case MISCALE_GATEWAY_TARGET_WIFI_SSID:
    return !wifiSSID.equals(optionWifiSSID);
  case MISCALE_GATEWAY_TARGET_WIFI_PASSWORD:
    return !wifiPassword.equals(optionWifiPassword);
  default:
    return false;
  }
}

bool isOptionsDirty()
{
  return isOptionDirty(MISCALE_GATEWAY_TARGET_WIFI_SSID) || isOptionDirty(MISCALE_GATEWAY_TARGET_WIFI_PASSWORD);
}

bool isOptionValid(miscale_gateway_input_target target)
{
  switch (target)
  {
  case MISCALE_GATEWAY_TARGET_WIFI_SSID:
    return optionWifiSSID.length() > 0 && optionWifiSSID.length() <= SSID_MAXLENGTH;
  case MISCALE_GATEWAY_TARGET_WIFI_PASSWORD:
    return optionWifiPassword.length() <= WIFIPW_MAXLENGTH;
  default:
    return false;
  }
}

bool isOptionsValid()
{
  return isOptionValid(MISCALE_GATEWAY_TARGET_WIFI_SSID) && isOptionValid(MISCALE_GATEWAY_TARGET_WIFI_PASSWORD);
}

void restoreSaveToOptions()
{
  optionWifiSSID = String(wifiSSID);
  optionWifiPassword = String(wifiPassword);
}

void applySaveOptions()
{

  wifiSSID = String(optionWifiSSID);
  wifiPassword = String(optionWifiPassword);

#if MISCALE_GATEWAY_ALLOW_EEPROM
  updateEEPROM();
#endif

  // WiFi.disconnect();
  wifiState = MISCALE_GATEWAY_WL_DISCONNECTED;
  beginWifi(wifiSSID, wifiPassword);
}

void restoreOptionToTemp(miscale_gateway_input_target target)
{
  switch (target)
  {
  case MISCALE_GATEWAY_TARGET_WIFI_SSID:
    inputTemp = String(optionWifiSSID);
    break;
  case MISCALE_GATEWAY_TARGET_WIFI_PASSWORD:
    inputTemp = String(optionWifiPassword);
    break;
  default:
    break;
  }
}

void applyValueToOption(String value, miscale_gateway_input_target target)
{

  switch (target)
  {
  case MISCALE_GATEWAY_TARGET_WIFI_SSID:
    optionWifiSSID = String(value);
    break;
  case MISCALE_GATEWAY_TARGET_WIFI_PASSWORD:
    optionWifiPassword = String(value);
    break;
  default:
    break;
  }
}

void transitionToHomeScreen()
{
  lv_scr_load_anim(ui_ScaleScreen, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
}

void transitionToOptionScreen()
{
  lv_scr_load_anim(ui_OptionScreen, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
}

void transitionToInputScreen(miscale_gateway_input_target target)
{
  inputTarget = target;
  restoreOptionToTemp(inputTarget);
  changeInputValueFrom(inputTarget);
  lv_scr_load_anim(ui_InputScreen, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
}

void changeInfoScreen(String message, bool showConfirm, bool showCancel)
{

  lv_label_set_text(ui_InfoLabel, message.c_str());

  lv_obj_toggle_display(ui_InfoConfirmBtn, showConfirm);
  lv_obj_toggle_clickable(ui_InfoConfirmBtn, showConfirm);
  lv_obj_toggle_display(ui_InfoCancelBtn, showCancel);
  lv_obj_toggle_clickable(ui_InfoCancelBtn, showCancel);
  lv_label_set_text(ui_InfoCancelLabel, "Cancel");
}

void transitionToInfoScreen()
{
  lv_scr_load_anim(ui_InfoScreen, LV_SCR_LOAD_ANIM_NONE, 0, 0, false);
}

void transitionToCheckVersionScreen()
{
  // update text
  String infoText = "Version: ";
  infoText.concat(MISCALE_GATEWAY_VERSION_FULL);
  infoText.concat('(');
  infoText.concat(MISCALE_GATEWAY_VERSION);
  infoText.concat(")\nMAC: ");

  String wifiMac = WiFi.macAddress();
  wifiMac.toUpperCase();
  infoText.concat(wifiMac);

  lv_label_set_text(ui_InfoLabel, infoText.c_str());

  lv_obj_toggle_display(ui_InfoConfirmBtn, false);
  lv_obj_toggle_clickable(ui_InfoConfirmBtn, false);
  lv_obj_toggle_display(ui_InfoCancelBtn, true);
  lv_label_set_text(ui_InfoCancelLabel, "Back");
  lv_obj_toggle_clickable(ui_InfoCancelBtn, true);

  infoAction = MISCALE_GATEWAY_NO_ACTION;
  transitionToInfoScreen();
}

void transitionToOtaFirmwareScreen()
{
  lv_label_set_text(ui_InfoLabel, "Check Version");

  lv_obj_toggle_display(ui_InfoConfirmBtn, false);
  lv_obj_toggle_clickable(ui_InfoConfirmBtn, false);
  lv_obj_toggle_display(ui_InfoCancelBtn, false);
  lv_label_set_text(ui_InfoCancelLabel, "Back");
  lv_obj_toggle_clickable(ui_InfoCancelBtn, false);

  infoAction = MISCALE_GATEWAY_NO_ACTION;
  transitionToInfoScreen();
}

static void updateScaleScreen()
{
  lv_label_set_text(ui_WeightValueLabel, isnan(lastestWeightRead) ? "-" : String(lastestWeightRead).c_str());

  // check if scan
  bool isBleRun = isBleRunning();
  bool isBleFound = isBleDeviceFound();

  lv_obj_toggle_clickable(ui_WeightStartBtn, isBleFound);
  lv_obj_toggle_disabled(ui_WeightStartBtn, !isBleFound);

  lv_obj_toggle_display(ui_WeightStartBtn, !isBleRun);
  lv_obj_toggle_clickable(ui_WeightResetBtn, isBleRun);
  lv_obj_toggle_display(ui_WeightResetBtn, isBleRun);
  lv_obj_toggle_clickable(ui_WeightSendBtn, isBleRun && isWeightStable);
  lv_obj_toggle_display(ui_WeightSendBtn, isBleRun);
  lv_obj_toggle_disabled(ui_WeightSendBtn, !isBleRun || !isWeightStable);

  String errorText = "WL:";
  errorText.concat(isWifiConnected() ? 'T' : 'F');

  errorText.concat(" MQTT:");
  errorText.concat(isMqttConnected() ? 'T' : 'F');

  errorText.concat(" BLE:");
  errorText.concat(isBleDeviceFound() ? 'T' : 'F');
  errorText.concat(',');
  errorText.concat(isBleRun ? 'T' : 'F');
  errorText.concat(',');
  errorText.concat(isBleConnected() ? 'T' : 'F');

  errorText.concat(" WStable:");
  errorText.concat(isWeightStable ? 'T' : 'F');

  errorText.concat("\nWSend:");
  switch (weightSendSuccessful)
  {
  case MISCALE_GATEWAY_WEIGHT_SEND_WAITING:
    errorText.concat('W');
    break;
  case MISCALE_GATEWAY_WEIGHT_SEND_SUCCESS:
    errorText.concat('T');
    break;
  case MISCALE_GATEWAY_WEIGHT_SEND_FAILED:
  default:
    errorText.concat('F');
    break;
  }

  lv_label_set_text(ui_ErrorLabel, errorText.c_str());
}
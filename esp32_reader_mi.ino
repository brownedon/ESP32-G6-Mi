/**
   A ESP32 BLE client that can read (glucose, raw, ..) data from the dexcom G6 (G5) transmitter.

   Developed in the context of my bachelor thesis at the Technical University (TU) of Darmstadt
   under the supervision of the Telecooperation Lab.

   Specifications Hardware / Software:
   - ESP32-WROOM-32D (ESP32_DevKitc_V4)
   - espressif v1.0.4  (https://dl.espressif.com/dl/package_esp32_index.json)
   - Arduino Studio 1.8.10
   - Dexcom G6 Transmitter 81xxxx (Model: SW11163, Firmware: 1.6.5.25 / 1.6.13.139)
   - Dexcom G6 Plus Transmitter 8Gxxxx (unknown)

   Author: Max Kaiser
   Copyright (c) 2020
   28.05.2020
*/
#include "TinyPICO.h"
#include "BLEDevice.h"
#include "BLEScan.h"
#include "Output.h"
#include <mbedtls/aes.h>
#include "esp_system.h"

//////////////////////////////////////////////////////
//Dexcom Transmitter ID
static std::string transmitterID = "31H4DB";
//
//MI Band MAC
//First time pairing control
boolean authenticated = true;
boolean wait_for_mi = false;
//boolean wait_for_value = false;
//
//const std::string MI_LAB = "fa:ab:33:e3:12:2d";
//amazfit bip
const std::string MI_LAB = "dc:84:41:3b:0f:6f";
//////////////////////////////////////////////////////
RTC_SLOW_ATTR int bootCount = 0;
RTC_SLOW_ATTR int channel = 0;
RTC_SLOW_ATTR boolean gotGlucose = false;
RTC_SLOW_ATTR int reading_count = 0;

int notif_delay;
#define STATE_START_SCAN 0                                                                                              // Set this state to start the scan.
#define STATE_SCANNING   1                                                                                              // Indicates the esp is currently scanning for devices.
#define STATE_SLEEP      2                                                                                              // Finished with reading data from the transmitter.
static int Status      = 0;                                                                                             // Set to 0 to automatically start scanning when esp has started.

//boolean doSleep = false;
RTC_SLOW_ATTR long timeStart = 0;
RTC_SLOW_ATTR long timeEnd = 0;
RTC_SLOW_ATTR int MISS_COUNT  = 0;
//mi
int Alert = 0;
int message_len = 0;
long stuck = 0;

RTC_SLOW_ATTR uint8_t message[12];
RTC_SLOW_ATTR boolean newValue = false;

uint8_t SECRET_KEY[18]  = {0x01, 0x08, 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45};
const uint8_t SHORT_SECRET_KEY[16]  = { 0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39, 0x40, 0x41, 0x42, 0x43, 0x44, 0x45};
uint8_t CONFIRM[2] = {0x02, 0x08};
// Once the KEY is changed, MI Band 2 will see your device as a new client
static uint8_t  _KEY [18] =     {0x01, 0x00, 0x82, 0xb6, 0x5c, 0xd9, 0x91, 0x95, 0x9a, 0x72, 0xe5, 0xcc, 0xb7, 0xaf, 0x62, 0x33, 0xee, 0x35};
static uint8_t  encrypted_num[18] = {0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static uint8_t  _send_rnd_cmd[2] =  {0x02, 0x00};
static uint8_t  auth_key[18];
static uint8_t  none[2] = {0, 0};

// AUTH_SERVER
static BLEUUID service2_uuid("0000fee1-0000-1000-8000-00805f9b34fb");
static BLEUUID alert_notify_sev_uuid("00001811-0000-1000-8000-00805f9b34fb");

// *** CHARACTERISTIC ***
// AUTH_CHAR
static BLEUUID auth_characteristic_uuid("00000009-0000-3512-2118-0009af100700");
static BLEUUID alert_cha_uuid("00002a46-0000-1000-8000-00805f9b34fb");


enum authentication_flags {
  send_key = 0,
  require_random_number = 1,
  send_encrypted_number = 2,
  auth_failed, auth_success = 3,
  waiting = 4
};
static BLEAdvertisedDevice* myMIDevice;
enum dflag {
  error = -1,
  idle = 0,
  scanning = 1,
  connecting2Dev = 2,
  connecting2Serv = 3,
  established = 4,
  waiting4data = 5
};

authentication_flags  auth_flag;
mbedtls_aes_context   aes;
dflag         status = idle;

//mi
boolean auth_step1 = false;
boolean auth_step2 = false;
boolean auth_step3 = false;
// The remote service we wish to connect to.
static BLEUUID    serviceUUID("f8083532-849e-531c-c594-30f1f86a4ea5");                                                  // This service holds all the important characteristics.
static BLEUUID advServiceUUID("0000febc-0000-1000-8000-00805f9b34fb");                                                  // This service gets advertised by the transmitter.
static BLEUUID deviceInformationServiceUUID("180A");                                                                    // The default service for the general device informations.
// The characteristic of the remote serviceUUID service we are interested in.
static BLEUUID  communicationUUID("F8083533-849E-531C-C594-30F1F86A4EA5"); // NOTIFY, READ
static BLEUUID        controlUUID("F8083534-849E-531C-C594-30F1F86A4EA5"); // INDICATE, WRITE
static BLEUUID authenticationUUID("F8083535-849E-531C-C594-30F1F86A4EA5"); // INDICATE, READ, WRITE (G6 Plus INDICATE / WRITE)
static BLEUUID       backfillUUID("F8083536-849E-531C-C594-30F1F86A4EA5"); // NOTIFY, READ, WRITE (G6 Plus NOTIFY)

// The general characteristic of the device information service.
static BLEUUID manufacturerUUID("2A29"); // READ
static BLEUUID        modelUUID("2A24"); // READ
static BLEUUID     firmwareUUID("2A26"); // READ

RTC_SLOW_ATTR static boolean useAlternativeChannel = false;      /* Enable when used concurrently with xDrip / Dexcom CGM */           // Tells the transmitter to use the alternative bt channel.
static boolean bonding = false;                                                                                         // Gets set by Auth handshake "StatusRXMessage" and shows if the transmitter would like to bond with the client.

//always true for "8", start with false for "3"
RTC_SLOW_ATTR static boolean force_rebonding = true;               /* Enable when problems with connecting */                        // When true: disables bonding before auth handshake. Enables bonding after successful authenticated (and before bonding command) so transmitter then can initiate bonding.

// Variables which survives the deep sleep. Uses RTC_SLOW memory.
#define saveLastXValues 12                                                                                              // This saves the last x glucose levels by requesting them through the backfill request.
RTC_SLOW_ATTR static uint16_t glucoseValues[saveLastXValues] = {0};                                                     // Reserve space for 1 hour a 5 min resolution of glucose values.
RTC_SLOW_ATTR static boolean error_last_connection = false;
RTC_SLOW_ATTR static boolean error_current_connection = false;                                                                        // To detect an error in the current session.

// Shared variables (used in the callbacks)
static volatile boolean connected = false;                                                                              // Indicates if the ble client is connected to the transmitter. Used to detect a transmitter timeout.
static std::string AuthCallbackResponse = "";
static std::string ControlCallbackResponse = "";
// Use "volatile" so that the compiler does not optimise code related with
// this variable and delete the empty while loop which is used as a barrier.
static volatile boolean bondingFinished = false;                                                                        // Get set when the bonding has finished, does not indicates if it was successful.

static BLERemoteCharacteristic* pRemoteCommunication;
static BLERemoteCharacteristic* pRemoteControl;
static BLERemoteCharacteristic* pRemoteAuthentication;
static BLERemoteCharacteristic* pRemoteBackfill;
static BLERemoteCharacteristic* pRemoteManufacturer;                                                                    // Uses deviceInformationServiceUUID
static BLERemoteCharacteristic* pRemoteModel;                                                                           // Uses deviceInformationServiceUUID
static BLERemoteCharacteristic* pRemoteFirmware;                                                                        // Uses deviceInformationServiceUUID
static BLEAdvertisedDevice* myDevice = NULL;                                                                            // The remote device (transmitter) found by the scan and set by scan callback function.
static BLEClient* pClient = NULL;                                                                                       // Is global so we can disconnect everywhere when an error occured.
//static BLEClient* pMiClient = NULL;


const int wdtTimeout = 420000;  //time in ms to trigger the watchdog  7 minutes
hw_timer_t *timer = NULL;

  int EST_GLUCOSE = 0;
  float Slope =0;
//
TinyPICO tp = TinyPICO();

void talkToMi();

// last resort to resolve freezes, if timer runs for 11 minutes restart the device
void IRAM_ATTR resetModule() {
  Serial.println("Watchdog timer, restart");
  ets_printf("reboot\n");
  esp_restart();
}

static void notifyCallback_auth(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  Serial.println("notifyCallback_auth");
  Serial.print(pData[0], HEX); Serial.print(":"); Serial.print(pData[1], HEX); Serial.print(":"); Serial.println(pData[2], HEX);

  if (pData[1] == 0x01) {
    Serial.println("# Write Confirm");
    auth_step1 = true;
  }

  if (pData[1] == 0x02) {
    Serial.println("send_encrypted_number");

    mbedtls_aes_init(&aes);
    mbedtls_aes_setkey_enc(&aes, SHORT_SECRET_KEY, 128);
    mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, pData + 3, encrypted_num + 2);
    mbedtls_aes_free(&aes);
    auth_step2 = true;
  }

  if (pData[1] == 0x03) {
    auth_flag = auth_success;
  }
}

class DeviceSearcher: public BLEAdvertisedDeviceCallbacks {
  public:
    void setDevAddr(std::string addr) {
      target_addr = addr;
    }

    void onResult (BLEAdvertisedDevice advertisedDevice) {
      std::string addr_now = advertisedDevice.getAddress().toString();
      if (addr_now.compare(target_addr) == 0) {
        pServerAddress = new BLEAddress(advertisedDevice.getAddress());
        myMIDevice = new BLEAdvertisedDevice(advertisedDevice);
        advertisedDevice.getScan()->stop();
        f_found = true;
      }
    }

    bool isFound() {
      return f_found;
    }

    BLEAddress * getServAddr() {
      return pServerAddress;
    }

  private:
    bool      f_found = false;
    std::string   target_addr;
    BLEAddress    * pServerAddress;
};

class MyMiClientCallback : public BLEClientCallbacks
{
    void onConnect(BLEClient* bleClient)
    {
      Serial.println( "onConnect-MI");
    }
    void onDisconnect(BLEClient* bleClient)
    {
      Serial.println("onDisconnect-MI");
    }
};

class MiBand2 {
  public:
    MiBand2(std::string addr, const uint8_t * key) {
      dev_addr = addr;
      memcpy(auth_key, key, 18);
    }

    ~MiBand2() {
      pMiClient->disconnect();
      Serial.println("# Operation finished.");
    }

    bool scan4Device(uint8_t timeout) {
      DeviceSearcher * ds = new DeviceSearcher();
      ds->setDevAddr(dev_addr);

      BLEScan* pBLEScan = BLEDevice::getScan();
      pBLEScan->setAdvertisedDeviceCallbacks(ds);
      pBLEScan->setActiveScan(true);
      pBLEScan->start(timeout);

      if (!ds->isFound()) {
        return false;
      } else {
        pServerAddress = ds->getServAddr();
        pMiClient = BLEDevice::createClient();

        return true;
      }
    }

    bool connect2Server(BLEAddress pAddress) {
      Serial.println("connect2Server");
      Serial.print("Forming a connection to ");
      Serial.println(pAddress.toString().c_str());

      pMiClient->connect(myMIDevice);

      if (!pMiClient->isConnected()) {
        Serial.println("Device connect failed");
        return false;
      }
      pMiClient->setClientCallbacks(new MyMiClientCallback());
      Serial.println("Connected to the device.");
      // ====================================================================
      // Get useful s/c/d of MI BAND 2
      // --------------------------------------------------------------------
      BLERemoteService * pRemoteService = pMiClient->getService(service2_uuid);
      if (pRemoteService == nullptr)
        return false;
      Serial.println("MIBAND2");
      pRemoteCharacteristicMi = pRemoteService->getCharacteristic(auth_characteristic_uuid);
      Serial.println(" |- CHAR_AUTH");
      if (pRemoteCharacteristicMi == nullptr)
        return false;

      pRemoteService = pMiClient->getService(alert_notify_sev_uuid);
      Serial.println("SVC_ALERT");
      pAlertCharacteristic = pRemoteService->getCharacteristic(alert_cha_uuid);
      Serial.println(" |- CHAR_ALERT");

      f_connected = true;
      // ====================================================================

      // ====================================================================
      // Bind notification
      // --------------------------------------------------------------------
      pRemoteCharacteristicMi->registerForNotify(notifyCallback_auth);
      // ====================================================================
      return true;
    }

    void authStart() {
      Serial.println("authStart");
      auth_flag = require_random_number;
      BLERemoteDescriptor* pauth_descripter;
      pauth_descripter = pRemoteCharacteristicMi->getDescriptor(BLEUUID((uint16_t)0x2902));
      Serial.println("   |- CCCD_AUTH");

      if (authenticated) {
        pRemoteCharacteristicMi->writeValue(CONFIRM, 2);
        Serial.println("# Sent {0x01, 0x08} to CCCD_AUTH");
      } else {
        pRemoteCharacteristicMi->writeValue(SECRET_KEY, 18);
        Serial.println("# Sent secret_key to CCCD_AUTH");
      }
      //need to keep this alive for callbacks to work
      int i = 0;
      while (auth_flag != auth_success) {
        //don't stay in loop forever
        i++;
        if (i > 30) {
          auth_flag = auth_success;
        }
        //delayMicroseconds(1 * 1000);
        delayMicroseconds(1000 * 1000);  //1 second
        //delay(1);
        if (auth_step1) {
          auth_step1 = false;
          pRemoteCharacteristicMi->writeValue(CONFIRM, 2);
        }
        if (auth_step2) {
          auth_step2 = false;
          encrypted_num[1] = 0x08;
          pRemoteCharacteristicMi->writeValue(encrypted_num, 18);
        }

      }
      Serial.println("# Auth succeed.");

    }


    void init(uint8_t timeout) {
      //Serial.println(dev_addr);
      Serial.println("Scanning for MI Band...");
      if (!scan4Device(timeout)) {
        Serial.println("Device not found");
        return;
      }
      Serial.println("Device found");

      Serial.println("Connecting to services...");
      if (!connect2Server(*pServerAddress)) {
        Serial.println("! Failed to connect to services");
        return;
      }
      Serial.println("Starting Authorization");
      authStart();
      status = established;

      //send the glucose
      if (newValue) {

        if (Alert) {
          Serial.println("Alert");
          Alert = false;
          message[0] = 0x03;
          message[1] = 0x01;
          pAlertCharacteristic->writeValue(message, 2, true);
          delayMicroseconds(5000 * 1000);
        }

        Serial.println("Send Message");
        message[0] = 0x05;
        message[1] = 0x01;
        pAlertCharacteristic->writeValue(message, 12, true);
        if ((EST_GLUCOSE < 90 || EST_GLUCOSE > 160) || (sqrt(pow(Slope, 2)) > 1.5)) {
          delayMicroseconds(5000 * 1000);
        }
        newValue = false;

      }
      deinit();
    }

    void deinit() {
      pMiClient->disconnect();
      wait_for_mi = false;
      Serial.println("# Operation finished.");
    }

  private:
    bool          f_found = false;
    bool          f_connected = false;

    std::string       dev_addr;
    BLEClient       * pMiClient;
    BLEAddress        * pServerAddress;
    BLERemoteCharacteristic * pRemoteCharacteristicMi;
    BLERemoteCharacteristic * pAlertCharacteristic;
};

/**
   Callback for the connection.
*/
class MyG6ClientCallback : public BLEClientCallbacks
{
    void onConnect(BLEClient* bleClient)
    {
      Serial.println( "onConnect");
      connected = true;
    }
    void onDisconnect(BLEClient* bleClient)
    {
      Serial.println("onDisconnect-G6");
      connected = false;
    }
};

/**
   Callback class for the secure bonding process.
   The transmitter will request / initiate the bonding.
*/
class MySecurity : public BLESecurityCallbacks
{
    uint32_t onPassKeyRequest()
    {
      return 123456;
    }
    void onPassKeyNotify(uint32_t pass_key) {}
    bool onConfirmPIN(uint32_t pass_key)
    {
      return true;
    }
    bool onSecurityRequest()
    {
      return true;
    }
    void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl)                                                        // This function is the only one that gets currently triggered.
    {
      Serial.println( "G6 pair status = ");
      Serial.println( auth_cmpl.success ? "success" : "fail");
      Serial.println( "onAuthenticationComplete : finished with bonding.");
      bondingFinished = true;                                                                                         // Finished with bonding.
    }
};

class MyMiSecurity : public BLESecurityCallbacks
{
    uint32_t onPassKeyRequest()
    {
      return 123456;
    }
    void onPassKeyNotify(uint32_t pass_key) {}
    bool onConfirmPIN(uint32_t pass_key)
    {
      return true;
    }
    bool onSecurityRequest()
    {
      return true;
    }
    void onAuthenticationComplete(esp_ble_auth_cmpl_t auth_cmpl)                                                        // This function is the only one that gets currently triggered.
    {
      Serial.println( "Mi pair status = ");
      Serial.println( auth_cmpl.success ? "success" : "fail");
      Serial.println( "onAuthenticationComplete : finished with bonding.");
      //bondingFinished = true;                                                                                         // Finished with bonding.
    }
};

/**
   Scan for BLE servers and find the first one that advertises the service we are looking for.
   Also check that the transmitter has the ID in the bluetooth name so that we connect only to this
   dexcom transmitter (if multiple are around).
*/
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks
{
    void onResult(BLEAdvertisedDevice advertisedDevice)                                                                 // Called for each advertising BLE server.
    {
      //SerialPrint(DEBUG, "BLE Advertised Device found: ");
      //SerialPrintln(DEBUG, advertisedDevice.toString().c_str());

      // We have found a device, let us now see if it contains the service we are looking for.
      //reset the watchdog
       timerWrite(timer, 0);
      if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(advServiceUUID) &&              // If the advertised service is the dexcom advertise service (not the main service that contains the characteristics).
          advertisedDevice.haveName() && advertisedDevice.getName() == ("Dexcom" + transmitterID.substr(4, 2)))
      {
        BLEDevice::getScan()->stop();                                                                               // We found our transmitter so stop scanning for now.
        myDevice = new BLEAdvertisedDevice(advertisedDevice);                                                       // Save device as new copy, myDevice also triggers a state change in main loop.
      }
    }
};

/**
   The different callbacks for notify and indicate if new data from the transmitter is available.
*/
static void notifyCommunicationCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  Serial.println( "notifyCommunicationCallback - ");
  printHexArray(pData, length);
}
static void indicateControlCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  Serial.println( "indicateControlCallback - read ");
  printHexArray(pData, length);
  ControlCallbackResponse = uint8ToString(pData, length);
}
static void indicateAuthCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
{
  Serial.println( "indicateAuthCallback - ");
  printHexArray(pData, length);
  AuthCallbackResponse = uint8ToString(pData, length);
}
/*static void notifyBackfillCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify)
  {
  if (!saveBackfill(uint8ToString(pData, length)))
  {
    Serial.println( "Can't parse this backfill data: ");
    //printHexArray(pData, length);
  }
  }
*/
/**
   Connects to the transmitter found by the scan and get services / characteristics.
   Return false if an error occurred.
*/
bool connectToTransmitter()
{
  Serial.println( "G6 Forming a connection to ");
  Serial.println( myDevice->getAddress().toString().c_str());

  pClient = BLEDevice::createClient();                                                                                // We specify the security settings later after we have successful authorised with the transmitter.
  Serial.println( " - Created ");

  pClient->setClientCallbacks(new MyG6ClientCallback());                                                                // Callbacks for onConnect() onDisconnect()
Serial.println("Connect");
  // Connect to the remote BLE Server.
  int i = 0;
  while (! pClient->connect(myDevice)) {
    Serial.println("Retry");
    i++;
    //delay(1);
    delayMicroseconds(1000 * 1000);
    if (i > 5) {
      return false;
    }
  }

  Serial.println( " - Connected to server");

  // Obtain a reference to the service.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.println( "Failed to find our service UUID: ");
    Serial.println( serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  BLERemoteService* pRemoteServiceInfos = pClient->getService(deviceInformationServiceUUID);
  if (pRemoteServiceInfos == nullptr)
  {
    Serial.println( "Failed to find our service UUID: ");
    Serial.println( deviceInformationServiceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println( " - Found our services");

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  getCharacteristic(&pRemoteCommunication, pRemoteService, communicationUUID);
  getCharacteristic(&pRemoteControl, pRemoteService, controlUUID);
  getCharacteristic(&pRemoteAuthentication, pRemoteService, authenticationUUID);
  getCharacteristic(&pRemoteBackfill, pRemoteService, backfillUUID);

  getCharacteristic(&pRemoteManufacturer, pRemoteServiceInfos, manufacturerUUID);
  getCharacteristic(&pRemoteModel, pRemoteServiceInfos, modelUUID);
  getCharacteristic(&pRemoteFirmware, pRemoteServiceInfos, firmwareUUID);
  Serial.println( " - Found our characteristics");

  forceRegisterNotificationAndIndication(indicateAuthCallback, pRemoteAuthentication, false);                         // Needed to work with G6 Plus (and G6) sensor. The command below only works for G6 (81...) transmitter.
  return true;
}

/**
   Reads the device informations which are not dexcom specific.
*/
bool readDeviceInformations()
{
  if (!pRemoteManufacturer->canRead())                                                                                // Check if the characteristic is readable.
    return false;
  Serial.println( "The Manufacturer value was: ");
  Serial.println( pRemoteManufacturer->readValue().c_str());                                                     // Read the value of the device information characteristics.

  if (!pRemoteModel->canRead())
    return false;
  Serial.println( "The Model value was: ");
  Serial.println( pRemoteModel->readValue().c_str());

  if (!pRemoteFirmware->canRead())
    return false;
  Serial.println( "The Firmware value was: ");
  Serial.println( pRemoteFirmware->readValue().c_str());
  return true;
}

MiBand2 dev(MI_LAB, _KEY);
/**
   Set up the ESP32 ble.
*/

void sleep() {
  Serial.println("++++++++++++++++++++++++");
  Serial.print("Sleeping for ");
  long offset = 2 + ((timeEnd - timeStart) / 1000);
  Serial.println(296 - offset);
  Serial.println("++++++++++++++++++++++++");
  tp.DotStar_SetPower( false );
  bootCount = 0;
  gotGlucose = false;

  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
  esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  esp_bt_controller_disable;


  esp_sleep_enable_timer_wakeup((296 - (offset)) * 1000000); // Sleep for 4 minutes 55 seconds
  // delay(1);
  delayMicroseconds(1000 * 1000);
  Serial.flush();
  esp_deep_sleep_start();
}

void setup()
{
  Serial.begin(115200);
  setCpuFrequencyMhz(80);
  timer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(timer, &resetModule, true);  //attach callback
  timerAlarmWrite(timer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(timer); 
  
  //restart the chip every 12 hours
  //if (reading_count > 144) {
  //  reading_count = 0;
   // ESP.restart();
   // }

  tp.DotStar_CycleColor(25);
  tp.DotStar_SetBrightness(50);
  tp.DotStar_Show();
  Serial.println(timeStart);
  Serial.println(timeEnd);
  Serial.println( "Starting ESP32 dexcom client application...");
  Serial.print("Mem ");
  Serial.println(ESP.getFreeHeap());
  BLEDevice::init("");
  BLEScan* pBLEScan = BLEDevice::getScan();                                                                           // Retrieve a Scanner.
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());                                          // Set the callback to informed when a new device was detected.
  pBLEScan->setInterval(100); //100 works                                                                             // The time in ms how long each search intervall last. Important for fast scanning so we dont miss the transmitter waking up.
  pBLEScan->setWindow(99); //60-99 works                                                                              // The actual time that will be searched. Interval - Window = time the esp is doing nothing (used for energy efficiency).
  pBLEScan->setActiveScan(false);  // Possible source of error if we cant connect to the transmitter.
  Serial.println("All BLE Scan stuff started");
}


void talkToMi() {
  //
  //MI
  //
  if (newValue) {
    BLEDevice::setSecurityCallbacks(new MyMiSecurity());
    wait_for_mi = true;
    int i = 0;
    while (wait_for_mi) {
      i++;
      Serial.println("Calling MI");
      dev.init(60);
      if (i > 2) {
        wait_for_mi = false;
      }
    }
  }
}
/**
   This method will perform a full transmitter connect and read data.
*/
bool run()
{
  timeStart = millis(); Serial.println(timeStart);
  Serial.println("+++++++++++SESSION START +++++++++++++");
  Serial.print("Time");
  Serial.println(millis());

  if (error_current_connection) {
    if (force_rebonding) {
      force_rebonding = false;
    } else {
      force_rebonding = true;
    }
  }

  Serial.print("Force_rebonding is set to:");
  if (force_rebonding) {
    Serial.println("true");
  } else {
    Serial.println("false");
  }

  if (!force_rebonding) {
    Serial.println("Setup bonding");
    setup_bonding();
  }
  error_current_connection = true;
  Serial.println("connectToTransmitter");

  if (connectToTransmitter()) {
    Serial.println("We are now connected to the transmitter.");
    Serial.println("Authenticate");
    if (authenticate())        {
      Serial.println("Request Bond");
      if (requestBond())    {
        //Serial.println("Force Register");
        forceRegisterNotificationAndIndication(indicateControlCallback, pRemoteControl, false);

        Serial.println("readGlucose");
        //Read current glucose level to save it.
        if (readGlucose()) {
          error_current_connection = false;
          force_rebonding = true;
          sendDisconnect();
          int x = 0;
          while (connected) {
            x++;
            if (x > 30) {
              esp_sleep_enable_timer_wakeup(1);
              esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
              esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
              esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
              esp_bt_controller_disable;
              esp_deep_sleep_start();
            }
            //delayMicroseconds(1 * 1000);
            //delay(1);
            delayMicroseconds(1000 * 1000);
          }
          talkToMi();
        }
      }
    }
  }

  if (connected) {
    Serial.println("disconnect of last resort");
    sendDisconnect();
  }
  timeEnd = millis(); Serial.println(timeEnd);

  reading_count++;
  Serial.println("+++++++++++SESSION END +++++++++++++");
  timerWrite(timer, 0);
  sleep();
}

/**
   This is the main loop function.
*/
void loop()
{
  Serial.print("Loop Status:");
  Serial.println(Status);
  stuck++;
  if (stuck > 10000) {
    esp_sleep_enable_timer_wakeup(1);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
    esp_bt_controller_disable;
    esp_deep_sleep_start();
  }

  switch (Status)
  {
    case STATE_START_SCAN:  //0
      BLEDevice::getScan()->start(0, true);  // false = maybe helps with connection problems.
      Status = STATE_SCANNING;
      break;

    case STATE_SCANNING:  //1
      if (myDevice != NULL)  {       // A device (transmitter) was found by the scan (callback).
        run();
      }                              // This function is blocking until all tansmitter communication has finished.
      break;

    default:
      esp_sleep_enable_timer_wakeup(1);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_SLOW_MEM, ESP_PD_OPTION_ON);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_FAST_MEM, ESP_PD_OPTION_ON);
      esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
      esp_bt_controller_disable;
      esp_deep_sleep_start();
  }

}

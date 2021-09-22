/**
   General bluetooth low energy (BLE) functionality for the bluedroid stack.

   Author: Max Kaiser
   Copyright (c) 2020
   12.04.2020
*/

#include "BLEDevice.h"
#include "BLEScan.h"
#include "Output.h"

// Byte values for the notification / indication.
const uint8_t bothOff[]        = {0x0, 0x0};
const uint8_t notificationOn[] = {0x1, 0x0};
const uint8_t indicationOn[]   = {0x2, 0x0};
const uint8_t bothOn[]         = {0x3, 0x0};

/**
   Gets and checks the characteristic from the remote service specified by the characteristics UUID.
*/
bool getCharacteristic(BLERemoteCharacteristic** pRemoteCharacteristic, BLERemoteService* pRemoteService, BLEUUID uuid) // Use *pRemoteCharacteristic as an out parameter so get address/pointer of this pointer.
{
  *pRemoteCharacteristic = pRemoteService->getCharacteristic(uuid);                                                   // Write to where the pointer points (the pRemoteCharacteristic pointer address).
  if (*pRemoteCharacteristic == nullptr)
  {
    Serial.println( "Failed to find our characteristic for UUID: ");
    Serial.println( uuid.toString().c_str());
    return false;
  }
  return true;
}

/**
   Write a string to the given characteristic.
*/
bool writeValue(std::string caller, BLERemoteCharacteristic* pRemoteCharacteristic, std::string data)
{
  Serial.println( caller.c_str());
  Serial.println( " - Writing Data = ");
  printHexString(data);
          delayMicroseconds(500 * 1000);

  uint8_t* pdata = reinterpret_cast<uint8_t*>(&data[0]);                                                              // convert std::string to uint8_t pointer
  /* important must be true so we don't flood the transmitter */
  //dbrowne may hang here
  pRemoteCharacteristic->writeValue(pdata, data.length(), true);                                                     // true = wait for response (acknowledgment) from the transmitter.
  Serial.println("Transmitter responded");
  return true;
}

/**
   Register for notification, also check if notification is available.
*/
bool registerForNotification(notify_callback _callback, BLERemoteCharacteristic *pBLERemoteCharacteristic)
{
  if (pBLERemoteCharacteristic->canNotify())                                                                          // Check if the characteristic has the potential to notify.
  {
    pBLERemoteCharacteristic->registerForNotify(_callback);
    Serial.println( " - Registered for notify on UUID: ");
    Serial.println(  pBLERemoteCharacteristic->getUUID().toString().c_str());
    return true;
  }
  else
  {
    Serial.println(  " - Notify NOT available for UUID: ");
    Serial.println(  pBLERemoteCharacteristic->getUUID().toString().c_str());
  }
  return false;
}

/**
   Register for indication AND notification, without checking.
*/
//program can hang here
bool forceRegisterNotificationAndIndication(notify_callback _callback, BLERemoteCharacteristic *pBLERemoteCharacteristic, bool isNotify)
{
            delayMicroseconds(500 * 1000);
 // if (pBLERemoteCharacteristic->canNotify())                                                                          // Check if the characteristic has the potential to notify.
 // {
    pBLERemoteCharacteristic->registerForNotify(_callback, isNotify);                                                   // Register first for indication(/notification) (because this is the correct one)
//  }

  pBLERemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t *)bothOn, 2,true);         // True to wait for acknowledge, set to both, manually set the bytes because there is no such funktion to set both.
  Serial.println( " - FORCE registered for indicate and notify on UUID: ");
  Serial.println(  pBLERemoteCharacteristic->getUUID().toString().c_str());
  return true;

}

/**
   Register for indication, also check if indications are available.
*/
bool registerForIndication(notify_callback _callback, BLERemoteCharacteristic *pBLERemoteCharacteristic)
{
  if (pBLERemoteCharacteristic->canIndicate())
  {
    pBLERemoteCharacteristic->registerForNotify(_callback, false);                                                  // false = indication, true = notification
    Serial.println( " - Registered for indicate on UUID: ");
    Serial.println(  pBLERemoteCharacteristic->getUUID().toString().c_str());
    return true;
  }
  else
  {
    Serial.println(  " - Indicate NOT available for UUID: ");
    Serial.println(  pBLERemoteCharacteristic->getUUID().toString().c_str());
  }
  return false;
}

/**
   Enables BLE bonding.
*/
bool setup_bonding()
{
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);                                                                 // Enable security encryption.
  BLEDevice::setSecurityCallbacks(new MySecurity());

  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setKeySize();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY);
  pSecurity->setCapability(ESP_IO_CAP_IO);
  pSecurity->setRespEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);
  Serial.println(  "Enabled bonding.");
}

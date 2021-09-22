/**
   Dexcom BLE communication functions.

   Author: Max Kaiser
   Copyright (c) 2020
   12.04.2020
*/

#include "BLEDevice.h"
#include "Output.h"

/**
   Wrapper function to send data to the authentication characteristic.
*/
bool AuthSendValue(std::string value)
{
  AuthCallbackResponse = "";                                                                                          // Reset to invalid because we will write to the characteristic and must wait until new data arrived from the notify callback.
  return writeValue("AuthSendValue", pRemoteAuthentication, value);
}

/**
   Wrapper function to send data to the control characteristic.
*/
bool ControlSendValue(std::string value)
{
  ControlCallbackResponse = "";
  return writeValue("ControlSendValue", pRemoteControl, value);
}

/**
   Barrier to wait until new data arrived through the notify callback.
*/
std::string AuthWaitToReceiveValue()
{
  int i = 0;
  while (connected)                                                                                                   // Only loop until we lost connection.
  {
    i++;
    //delayMicroseconds(1 * 1000);
    //delay(1);
    delayMicroseconds(1000 * 1000);
    if (i > 30) {
      Serial.println("timeout");
      return "";
    }
    if (AuthCallbackResponse != "")
    {
      std::string returnValue = AuthCallbackResponse;                                                             // Save the new value.
      AuthCallbackResponse = "";                                                                                  // Reset because we handled the new data.
      return returnValue;
    }
  }
  Serial.println("Error timeout in AuthWaitToReceiveValue");                                                               // The transmitter disconnected so exit.
  return "";
}

/**
   Barrier to wait until new data arrived through the notify callback.
*/
std::string ControlWaitToReceiveValue()
{
  int i = 0;
  //delayMicroseconds(1 * 1000);
  // delay(1);
  while (connected)                                                                                                   // Only loop until we lost connection.
  {
    i++;
    delayMicroseconds(1000 * 1000);
    if (i > 30) {
      Serial.println("timeout");
      return "";
    }
    if (ControlCallbackResponse != "")
    {
      std::string returnValue = ControlCallbackResponse;                                                          // Save the new value.
      ControlCallbackResponse = "";                                                                               // Reset because we handled the new data.
      return returnValue;
    }
  }
  Serial.println("Error timeout in ControlWaitToReceiveValue");                                                            // The transmitter disconnected so exit.
  return "";
}

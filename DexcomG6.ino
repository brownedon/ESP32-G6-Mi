/**
   Functions for authentication and reading data form the dexcom transmitter.

   Author: Max Kaiser
   Copyright (c) 2020
   12.04.2020
*/

#include "mbedtls/aes.h"
#include "Output.h"
#include <math.h>
#include "cgms.h"
static std::string backfillStream = "";
static int backfillExpectedSequence = 1;



/**
   This function will authenticate with the transmitter using a handshake and the transmitter ID.
   Return true if we are authenticated.
*/
bool authenticate()
{
  Serial.println("authenticate()");
  //Send AuthRequestTxMessage
  std::string authRequestTxMessage = {0x01, 0x19, 0xF3, 0x89, 0xF8, 0xB7, 0x58, 0x41, 0x33 };  //0x02                 // 10byte, first byte = opcode (fix), [1] - [8] random bytes as challenge for the transmitter to encrypt,
  authRequestTxMessage += useAlternativeChannel ? 0x01 : 0x02;                                                        // last byte 0x02 = normal bt channel, 0x01 alternative bt channel
  AuthSendValue(authRequestTxMessage);

  //Recv AuthChallengeRXMessage
  std::string authChallengeRxMessage = AuthWaitToReceiveValue();                                                      // Wait until we received data from the notify callback.
  Serial.println("authChallengeRxMessage");
  for (int i = 0; i < authChallengeRxMessage.length(); i++) {
    Serial.print(authChallengeRxMessage[i], HEX); Serial.print(" ");
  }
  Serial.println("");

  if ((authChallengeRxMessage.length() != 17) || (authChallengeRxMessage[0] != 0x03))
  {
    Serial.println( "Error wrong length or opcode!");
    return false;
  }
  std::string tokenHash = "";
  std::string challenge = "";
  for (int i = 1; i < authChallengeRxMessage.length(); i++)                                                           // Start with 1 to skip opcode.
  {
    if (i < 9)
      tokenHash += authChallengeRxMessage[i];
    else
      challenge += authChallengeRxMessage[i];
  }
  //Here we could check if the tokenHash is the encrypted 8 bytes from the authRequestTxMessage ([1] to [8]);
  //To check if the Transmitter is a valid dexcom transmitter (because only the correct one should know the ID).

  //Send AuthChallengeTXMessage
  std::string hash = calculateHash(challenge, transmitterID);                                                         // Calculate the hash from the random 8 bytes the transmitter send us as a challenge.
  std::string authChallengeTXMessage = {0x04};                                                                        // opcode
  authChallengeTXMessage += hash;                                                                                     // in total 9 byte.
  AuthSendValue(authChallengeTXMessage);

  //Recv AuthStatusRXMessage
  std::string authStatusRXMessage = AuthWaitToReceiveValue();
  Serial.println("authStatusRXMessage");
  for (int i = 0; i < authStatusRXMessage.length(); i++) {
    Serial.print(authStatusRXMessage[i], HEX); Serial.print(" ");
  }
  Serial.println("");
  // Response { 0x05, 0x01 = authenticated / 0x02 = not authenticated, 0x01 = no bonding, 0x02 bonding
  if (authStatusRXMessage.length() == 3 && authStatusRXMessage[1] == 1)                                               // correct response is 0x05 0x01 0x02
  {
    Serial.println( "Authenticated!");
    bonding = authStatusRXMessage[2] != 0x01;
    return true;
  }
  else {
    Serial.println( "Authenticated FAILED!");
  }
  return false;
}

/**
   We have successfully authorized and now want to bond.
   First enable the BLE security bonding options and then indicate the transmitter that he can now initiate a bonding.
   Return true if no error occurs.
*/
bool requestBond()
{
  Serial.println("requestBond()");
  if (bonding)
  {
    if (force_rebonding)                                                                                            // Enable bonding after successful auth and before sending bond request to transmitter.
      setup_bonding();

    Serial.println( "Sending Bond Request.");
    //Send KeepAliveTxMessage
    std::string keepAliveTxMessage = {0x06, 0x19};                                                                  // Opcode 2 byte = 0x06, 25 as hex (0x19)
    AuthSendValue(keepAliveTxMessage);
    //Send BondRequestTxMessage
    std::string bondRequestTxMessage = {0x07};                                                                      // Send bond command.
    AuthSendValue(bondRequestTxMessage);
    //Wait for bonding to finish
    Serial.println( "Waiting for bond.");
    //20s timeout on bonding
    int i = 0;
    while (!bondingFinished) {
      i++;
     // delayMicroseconds(1 * 1000);
     //  (1);
             delayMicroseconds(1000 * 1000);
      if (i > 60) {
        Serial.println("request bond timeout");
        return false;
      }
    }
    // Barrier waits until bonding has finished, IMPORTANT to set the bondingFinished variable to sig_atomic_t OR volatile
    //Wait
    Serial.println( "Bonding finished.");
  }
  else {
    Serial.println( "Transmitter does not want to (re)bond so DONT send bond request (already bonded).");
    force_rebonding = false;
  }
  return true;
}

uint32_t transmitterStartTime = 0;


/**
   Reads the glucose values from the transmitter.
*/
bool readGlucose()
{
  std::string glucoseTxMessageG5 = {0x30, 0x53, 0x36};                                                                // G5 = 0x30 the other 2 bytes are the CRC16 XMODEM value in twisted order
  std::string glucoseTxMessageG6 = {0x4e, 0x0a, 0xa9};                                                                // G6 = 0x4e
  // if(transmitterID[0] == 8 || (transmitterID[0] == 2 && transmitterID[1] == 2 && transmitterID[2] == 2))              // Check if G6 or one of the newest G6 plus (>2.18.2.88) see https://github.com/xdrip-js/xdrip-js/issues/87
  ControlSendValue(glucoseTxMessageG6);
  // else
  //     ControlSendValue(glucoseTxMessageG5);

  std::string glucoseRxMessage = ControlWaitToReceiveValue();
  //if (glucoseRxMessage.length() < 16 || glucoseRxMessage[0] != (transmitterID[0] != 8 ? 0x31 : 0x4f))                 // Opcode depends on G5 / G6
  //    return false;

  uint8_t status = (uint8_t)glucoseRxMessage[1];
  uint32_t sequence  = (uint32_t)(glucoseRxMessage[2] +
                                  glucoseRxMessage[3] * 0x100  +
                                  glucoseRxMessage[4] * 0x10000 +
                                  glucoseRxMessage[5] * 0x1000000);
  uint32_t timestamp = (uint32_t)(glucoseRxMessage[6] +
                                  glucoseRxMessage[7] * 0x100  +
                                  glucoseRxMessage[8] * 0x10000 +
                                  glucoseRxMessage[9] * 0x1000000);

  uint16_t glucoseBytes = (uint16_t)(glucoseRxMessage[10] +
                                     glucoseRxMessage[11] * 0x100);
  boolean glucoseIsDisplayOnly = (glucoseBytes & 0xf000) > 0;
  uint16_t glucose = glucoseBytes & 0xfff;
  uint8_t state = (uint8_t)glucoseRxMessage[12];
  int trend = (int)glucoseRxMessage[13];

  uint16_t glucoseEstBytes = (uint16_t)(glucoseRxMessage[14] +
                                        glucoseRxMessage[15] * 0x100);
  uint16_t glucoseEst = glucoseEstBytes & 0xfff;

  if (state != 0x06)                                                                                                  // Not the ok state -> exit
  {
    //SerialPrintf(ERROR, "\nERROR - Session Status / State NOT OK (%d)!\n", state);
    // ExitState("ERROR - We will not continue due to safety reasons (warmup, stopped, waiting for calibration(s), failed or expired.\n");
    gotGlucose = true;
    newValue = false;
    ///////
    /*gotGlucose = true;
    newValue = true;
    handle_glucose(88, timestamp);*/
    ///////
    return true;
  }
  gotGlucose = true;

  if (glucose > 38) {
    handle_glucose(glucose, timestamp);
  } else {
    newValue = false;
  }

  SerialPrintf(DATA, "Glucose - Status:      %d\n", status);
  SerialPrintf(DATA, "Glucose - Sequence:    %d\n", sequence);
  SerialPrintf(DATA, "Glucose - Timestamp:   %d\n", timestamp);                                                       // Seconds since transmitter activation
  SerialPrintf(DATA, "Glucose - DisplayOnly: %s\n", (glucoseIsDisplayOnly ? "true" : "false"));
  SerialPrintf(GLUCOSE, "Glucose - Glucose:     %d\n", glucose);
  SerialPrintf(GLUCOSE, "Glucose - Glucose Est:     %d\n", glucoseEst);
  SerialPrintf(DATA, "Glucose - State:       %d\n", state);
  SerialPrintf(DATA, "Glucose - Trend:       %d\n", trend);

  if (saveLastXValues > 0)                                                                                            // Array is big enouth for min one value.
  {
    for (int i = saveLastXValues - 1; i > 0; i--)                                                                   // Shift all old values back to set the newest to position 0.
      glucoseValues[i] = glucoseValues[i - 1];
    glucoseValues[0] = glucose;
  }


  return true;
}

/**
   Sending command to initiate a disconnect from the transmitter.
*/
bool sendDisconnect()
{
  Serial.println( "Initiating a disconnect.");
  std::string disconnectTxMessage = {0x09};
  ControlSendValue(disconnectTxMessage);
    std::string transmitterdisconnectTxMessage = ControlWaitToReceiveValue();
  // Wait until onDisconnect callback was called and connected status flipped.
  return true;
}

/**
   Encrypt using AES 182 ecb (Electronic Code Book Mode).
*/
std::string encrypt(std::string buffer, std::string id)
{
  mbedtls_aes_context aes;

  std::string key = "00" + id + "00" + id;                                                                            // The key (that also used the transmitter) for the encryption.
  unsigned char output[16];

  mbedtls_aes_init(&aes);
  mbedtls_aes_setkey_enc(&aes, (const unsigned char *)key.c_str(), strlen(key.c_str()) * 8);
  mbedtls_aes_crypt_ecb(&aes, MBEDTLS_AES_ENCRYPT, (const unsigned char *)buffer.c_str(), output);
  mbedtls_aes_free(&aes);

  std::string returnVal = "";
  for (int i = 0; i < 16; i++)                                                                                        // Convert unsigned char array to string.
  {
    returnVal += output[i];
  }
  return returnVal;
}

/**
   Calculates the Hash for the given data.
*/
std::string calculateHash(std::string data, std::string id)
{
  if (data.length() != 8)
  {
    SerialPrintln(ERROR, "cannot hash");
    return NULL;
  }

  data = data + data;                                                                                                 // Use double the data to get 16 byte
  std::string hash = encrypt(data, id);
  return hash.substr(0, 8);                                                                                           // Only use the first 8 byte of the hash (ciphertext)
}

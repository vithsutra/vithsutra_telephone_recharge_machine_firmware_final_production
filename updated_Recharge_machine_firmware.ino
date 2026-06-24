#include <SPI.h>
#include <MFRC522.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <ArduinoJson.h>
#include <BLE2902.h>
#include<string.h>

//defines
#define MACHINE_ID "TELE0017"
#define RFID_CARD_BALANCE_SECTOR_ADDRESS 1
#define RFID_CARD_PHONE_NUMBERS_SECTOR_ADDRESS 2
#define RFID_CARD_BALANCE_BLOCK_ADDRESS 4
#define RFID_CARD_MODE_BLOCK_ADDRESS 5
#define RFID_CARD_PHONE_NUMBER_1_BLOCK_ADDRESS 8
#define RFID_CARD_PHONE_NUMBER_2_BLOCK_ADDRESS 9
#define RFID_CARD_PHONE_NUMBER_3_BLOCK_ADDRESS 10

//RFID Reader Pins
#define RFID_RST_PIN 22   //MFRC522 Reset pin
#define RFID_SS_PIN 5    //MFRC522 SDA/SS pin
//INDICATION PINS
#define BLE_INDICATION_LED_PIN     2
#define BUZZER_PIN 15

//BLE UUIDs
#define SERVICE_UUID  "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-abcd-1234-abcd-12345678abcd"

#define RFID_SUCCESS_CARD_FOUND 0x01
#define RFID_SUCCESS_AUTH 0x02
#define RFID_SUCCESS_WRITE 0x03
#define RFID_SUCCESS_READ 0x04

#define RFID_ERROR_CARD_NOT_FOUND 0x05 
#define RFID_ERROR_AUTH 0x06
#define RFID_ERROR_WRITE 0x07
#define RFID_ERROR_READ 0x08

#define JSON_ERROR_PARSE 0x09
#define JSON_SUCCESS_PARSE 0x0A

static char bleDataBuffer[100];
static bool bleDataAvailable = false;
static StaticJsonDocument<256> json;
//RFID KEYS
static const uint8_t defaultKey[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static const uint8_t privateKey[6] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06};

// NEW: Card state tracking
static bool cardPresent = false;
static unsigned long lastCardCheckTime = 0;
static const unsigned long CARD_CHECK_INTERVAL = 100; // Check every 100ms

//RFID object
static MFRC522 rfid(RFID_SS_PIN,RFID_RST_PIN); //MFRC522 instance

//BLE object
static BLEServer *pServer = nullptr;
static BLEService *pService = nullptr;
static BLECharacteristic *pCharacteristic = nullptr;
static bool deviceConnected = false;

//INDICATERS
void bluetoothConnectionIndicator(){
  digitalWrite(BLE_INDICATION_LED_PIN,HIGH); 
}

void bluetoothDisconnectionIndicator(){
  digitalWrite(BLE_INDICATION_LED_PIN,LOW); 
}

void rfidCardRechargeSuccessIndicater() {
  digitalWrite(BUZZER_PIN,HIGH);
  vTaskDelay(pdMS_TO_TICKS(1000));
  digitalWrite(BUZZER_PIN,LOW);
  vTaskDelay(pdMS_TO_TICKS(1000));
}

//BLE connection Callback functions
class RechargeServerCallbacks : public BLEServerCallbacks{
  void onConnect(BLEServer *pServer){
    deviceConnected = true;
    bluetoothConnectionIndicator();
  }
  
  void onDisconnect(BLEServer *pServer){
    deviceConnected = false;
    bluetoothDisconnectionIndicator();
    BLEDevice::startAdvertising();
  }
};

//BLE write callback function
class RechargeWriteCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String value= pCharacteristic->getValue();
    strcpy(bleDataBuffer,value.c_str());
    bleDataAvailable = true;
  }
};

//RFID Operations

// UPGRADE: Check if card is still present with debounce
bool isCardStillPresent() {
  byte bufferATQA[2];
  byte bufferSize = sizeof(bufferATQA);
  
  // Try twice to avoid false negatives due to weak signal
  for(int i=0; i<2; i++) {
      MFRC522::StatusCode result = rfid.PICC_WakeupA(bufferATQA, &bufferSize);
      if (result == MFRC522::STATUS_OK || result == MFRC522::STATUS_COLLISION) {
        return true;
      }
      vTaskDelay(pdMS_TO_TICKS(5)); 
  }
  return false;
}

// MODIFIED: Only initialize card connection if not already present
uint8_t checkNewRFIDCardPresent() {
  // If card is already present and active, just verify it's still there
  if (cardPresent) {
    if (isCardStillPresent()) {
      return RFID_SUCCESS_CARD_FOUND;
    }
    // Card was removed
    cardPresent = false;
  }
  
  // Look for new card
  if (!rfid.PICC_IsNewCardPresent()){
    return RFID_ERROR_CARD_NOT_FOUND;
  }
   
  if (!rfid.PICC_ReadCardSerial()) {
    return RFID_ERROR_CARD_NOT_FOUND;
  }

  cardPresent = true;
  return RFID_SUCCESS_CARD_FOUND;
}

// MODIFIED: Only end communication when card is removed or explicitly needed
void endRFIDCommunication(){
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  cardPresent = false;
}

// NEW: Periodic card presence check
void checkCardPresence() {
  unsigned long currentTime = millis();
  
  if (cardPresent && (currentTime - lastCardCheckTime > CARD_CHECK_INTERVAL)) {
    lastCardCheckTime = currentTime;
    
    if (!isCardStillPresent()) {
      // Card was removed, clean up
      endRFIDCommunication();
    }
  }
}

uint8_t changeRFIDCardKey(uint8_t sector){
  MFRC522::StatusCode status;

  uint8_t trailerBlock = sector * 4 + 3;
  
  uint8_t sectorTrailer[16];
  memcpy(sectorTrailer,privateKey,6);

  //access bit full access
  sectorTrailer[6]=0xFF;
  sectorTrailer[7]=0x07;
  sectorTrailer[8]=0x80;

  byte keyB[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
  memcpy(&sectorTrailer[9],keyB,6);
    
  // Retry loop
  for(int i=0; i<3; i++) {
    status = (MFRC522::StatusCode) rfid.MIFARE_Write(trailerBlock,sectorTrailer,16);
    if(status == MFRC522::STATUS_OK){
      vTaskDelay(pdMS_TO_TICKS(50)); // Small delay for write stability
      return RFID_SUCCESS_WRITE;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }

  return RFID_ERROR_WRITE;
}

uint8_t authRFIDCard(uint8_t blockAddress,const uint8_t *key){  
  MFRC522::StatusCode status;
  MFRC522::MIFARE_Key keyStruct;
  
  for(byte i=0;i<6;i++){
    keyStruct.keyByte[i]=key[i];
  }

  // Retry loop
  for(int i=0; i<3; i++) {
      status = rfid.PCD_Authenticate(
        MFRC522::PICC_CMD_MF_AUTH_KEY_A,
        blockAddress,
        &keyStruct,
        &rfid.uid
      );

      if(status == MFRC522::STATUS_OK){ 
        return RFID_SUCCESS_AUTH;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
  }

  return RFID_ERROR_AUTH;
}

uint8_t writeModetoRFIDCard(uint8_t mode){
  MFRC522::StatusCode status;
  uint8_t buffer[16]={0};
  buffer[0] = mode;

  // Retry loop
  for(int i=0; i<3; i++) {
      status = (MFRC522::StatusCode) rfid.MIFARE_Write(RFID_CARD_MODE_BLOCK_ADDRESS,buffer, 16);
      if(status == MFRC522::STATUS_OK){
         vTaskDelay(pdMS_TO_TICKS(50));
         return RFID_SUCCESS_WRITE;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  return RFID_ERROR_WRITE;
}

uint8_t readModeFromRFIDCard(uint8_t *mode){
  MFRC522::StatusCode status;
  uint8_t dataBuffer[18];
  uint8_t size = sizeof(dataBuffer);
    
  // Retry loop
  for(int i=0; i<3; i++) {
      size = sizeof(dataBuffer); // Reset size for each attempt
      status = (MFRC522::StatusCode) rfid.MIFARE_Read(RFID_CARD_MODE_BLOCK_ADDRESS,dataBuffer,&size);

      if(status == MFRC522::STATUS_OK){
         *mode = dataBuffer[0];
         return RFID_SUCCESS_READ; 
      }
      vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  return RFID_ERROR_READ;
}

uint8_t writeBalanceToRFIDCard(uint32_t balance){
  MFRC522::StatusCode status;
  
  // Retry loop
  for(int i=0; i<3; i++) {
      status = (MFRC522::StatusCode) rfid.MIFARE_SetValue(RFID_CARD_BALANCE_BLOCK_ADDRESS, balance);
      if(status == MFRC522::STATUS_OK){
         vTaskDelay(pdMS_TO_TICKS(50));
         return RFID_SUCCESS_WRITE;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  return RFID_ERROR_WRITE;
}

uint8_t readBalanceFromRFIDCard(uint32_t *balance){
  MFRC522::StatusCode status;
  
  // Retry loop
  for(int i=0; i<3; i++) {
      status = (MFRC522::StatusCode) rfid.MIFARE_GetValue(RFID_CARD_BALANCE_BLOCK_ADDRESS,(int32_t*)balance);      
      if(status == MFRC522::STATUS_OK){
        return RFID_SUCCESS_READ;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  return RFID_ERROR_READ;
}

uint8_t writePhoneNumberToRFIDCard(uint8_t blockAddr,char *ph){
  MFRC522::StatusCode status;
  uint8_t buffer[16];

  memcpy(buffer,ph,11);
    
  // Retry loop
  for(int i=0; i<3; i++) {
      status = (MFRC522::StatusCode) rfid.MIFARE_Write(blockAddr, buffer,sizeof(buffer));
      if(status == MFRC522::STATUS_OK){
         vTaskDelay(pdMS_TO_TICKS(50));
         return RFID_SUCCESS_WRITE;
      }
      vTaskDelay(pdMS_TO_TICKS(10));
  }

  return RFID_ERROR_WRITE;
}

uint8_t readPhoneNumberFromRFIDCard(uint8_t blockAddr,char *ph){
  MFRC522::StatusCode status;
  uint8_t dataBuffer[18];
  uint8_t size;
  
  // Retry loop
  for(int i=0; i<3; i++) {
      size = sizeof(dataBuffer); // Reset size
      status = (MFRC522::StatusCode) rfid.MIFARE_Read(blockAddr,dataBuffer,&size);

      if(status == MFRC522::STATUS_OK){
        memcpy(ph,dataBuffer,11);
        return RFID_SUCCESS_READ; 
      }
      vTaskDelay(pdMS_TO_TICKS(10));
  }

  return RFID_ERROR_READ;
}

uint8_t incrementRFIDCardBalance(int32_t amount){
  MFRC522::StatusCode status; 

  // Retry loop for the transaction sequence
  for(int i=0; i<3; i++) {
      status =(MFRC522::StatusCode) rfid.MIFARE_Increment(RFID_CARD_BALANCE_BLOCK_ADDRESS,amount);
      if(status != MFRC522::STATUS_OK){
        vTaskDelay(pdMS_TO_TICKS(10));
        continue; // Try again from start
      }
      
      status= (MFRC522::StatusCode) rfid.MIFARE_Transfer(RFID_CARD_BALANCE_BLOCK_ADDRESS);
      if(status == MFRC522::STATUS_OK){
         vTaskDelay(pdMS_TO_TICKS(50));
         return RFID_SUCCESS_WRITE;
      }
      // If transfer fails, we loop back and try increment again
      vTaskDelay(pdMS_TO_TICKS(10));
  }

  return RFID_ERROR_WRITE;
}

uint8_t parseJson(char *buffer){
  DeserializationError err = deserializeJson(json, buffer);     
  if (err){
    return JSON_ERROR_PARSE;
  }

  return JSON_SUCCESS_PARSE;
}

uint8_t getSignalFromRequestPayload(){
  std::string signalStr =json["signal"];
  int signal = std::stoi(signalStr);
  return (uint8_t)signal;
}

uint8_t getModeFromRequestPayload(){
  std::string modeStr =json["mode"];
  int mode = std::stoi(modeStr);
  return (uint8_t)mode;
}

uint32_t getRechargeAmountFromRequestPayload(){
  std::string balanceStr = json["amount"];
  int balance = std::stoi(balanceStr);
  return (uint32_t)balance;
}

void getPhoneNumberFromRequestPayload(char *ph1,char *ph2,char *ph3){
  JsonArray phoneNumbersArray = json["phone_numbers"];
  const char *ptr = phoneNumbersArray[0];
  memcpy(ph1,ptr,11);
  ptr = phoneNumbersArray[1];
  memcpy(ph2,ptr,11);
  ptr = phoneNumbersArray[2];
  memcpy(ph3,ptr,11);
}

void sendBLEResponse(){
  char outputBuffer[100];
  size_t len = serializeJson(json,outputBuffer);
  pCharacteristic->setValue((uint8_t*)outputBuffer,len);
  pCharacteristic->notify();
}

void handleSignal1(){
  if(checkNewRFIDCardPresent() != RFID_SUCCESS_CARD_FOUND) {
    json.clear();
    json["signal"] = "1";
    json["error_status"] = "1";
    sendBLEResponse();
    return;
  }
  
  json.clear();
  json["signal"] = "1";
  json["error_status"] = "0";
  sendBLEResponse();
  // DON'T end communication - keep card active
}

void handleSignal2(){
  uint8_t rfidCardMode = getModeFromRequestPayload();
  uint32_t balance = getRechargeAmountFromRequestPayload();
 
  if(checkNewRFIDCardPresent() != RFID_SUCCESS_CARD_FOUND) {
    json.clear();
    json["signal"] = "2";
    json["error_status"] = "1";
    sendBLEResponse();
    return;
  }

  if(authRFIDCard(RFID_CARD_MODE_BLOCK_ADDRESS, defaultKey) != RFID_SUCCESS_AUTH) {
    json.clear();
    json["signal"] = "2";
    json["error_status"] = "2";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  if(writeModetoRFIDCard(rfidCardMode) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"] = "2";
    json["error_status"] = "3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  if(writeBalanceToRFIDCard(balance) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"]="2";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  if(changeRFIDCardKey(RFID_CARD_BALANCE_SECTOR_ADDRESS) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"]="2";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }
    
  json.clear();
  json["signal"]="2";
  json["error_status"]="0";
  sendBLEResponse();
  // Keep card active for potential next operation
}

void handleSignal3() {
  char ph1[11];
  char ph2[11];
  char ph3[11];

  if(checkNewRFIDCardPresent() != RFID_SUCCESS_CARD_FOUND) {
    json.clear();
    json["signal"] = "3";
    json["error_status"] = "1";
    sendBLEResponse();
    return;
  }

  if(authRFIDCard(RFID_CARD_PHONE_NUMBER_1_BLOCK_ADDRESS, defaultKey) != RFID_SUCCESS_AUTH) {
    json.clear();
    json["signal"] = "3";
    json["error_status"] = "2";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  getPhoneNumberFromRequestPayload(ph1, ph2, ph3);

  if(writePhoneNumberToRFIDCard(RFID_CARD_PHONE_NUMBER_1_BLOCK_ADDRESS, ph1) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"]="3";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  if(writePhoneNumberToRFIDCard(RFID_CARD_PHONE_NUMBER_2_BLOCK_ADDRESS, ph2) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"]="3";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }
    
  if(writePhoneNumberToRFIDCard(RFID_CARD_PHONE_NUMBER_3_BLOCK_ADDRESS, ph3) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"]="3";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  if(changeRFIDCardKey(RFID_CARD_PHONE_NUMBERS_SECTOR_ADDRESS) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"]="3";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  json.clear();
  json["signal"]="3";
  json["error_status"]="0";
  sendBLEResponse();
  // Keep card active
}

void handleSignal4(){  
  uint32_t balance;

  if(checkNewRFIDCardPresent() != RFID_SUCCESS_CARD_FOUND) {
    json.clear();
    json["signal"] = "4";
    json["error_status"] = "1";
    sendBLEResponse();
    return;
  }

  if(authRFIDCard(RFID_CARD_MODE_BLOCK_ADDRESS, privateKey) != RFID_SUCCESS_AUTH) {
    json.clear();
    json["signal"] = "4";
    json["error_status"] = "2";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  if(readBalanceFromRFIDCard(&balance) != RFID_SUCCESS_READ) {
    json.clear();
    json["signal"]="4";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  json.clear();
  json["signal"]="4";
  json["error_status"]="0";
  json["balance"] = String(balance);
  sendBLEResponse(); 
  // Keep card active
}

void handleSignal5(){
  uint32_t amount = getRechargeAmountFromRequestPayload();

  if(checkNewRFIDCardPresent() != RFID_SUCCESS_CARD_FOUND) {
    json.clear();
    json["signal"] = "5";
    json["error_status"] = "1";
    sendBLEResponse();
    return;
  }

  if(authRFIDCard(RFID_CARD_BALANCE_BLOCK_ADDRESS, privateKey) != RFID_SUCCESS_AUTH) {
    json.clear();
    json["signal"] = "5";
    json["error_status"] = "2";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  if(incrementRFIDCardBalance(amount) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"]="5";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  json.clear();
  json["signal"]="5";
  json["error_status"]="0";
  sendBLEResponse();  
  rfidCardRechargeSuccessIndicater();
  // Keep card active
}

void handleSignal6(){
  uint8_t mode;

  if(checkNewRFIDCardPresent() != RFID_SUCCESS_CARD_FOUND) {
    json.clear();
    json["signal"] = "6";
    json["error_status"] = "1";
    sendBLEResponse();
    return;
  }

  if(authRFIDCard(RFID_CARD_MODE_BLOCK_ADDRESS, privateKey) != RFID_SUCCESS_AUTH) {
    json.clear();
    json["signal"] = "6";
    json["error_status"] = "2";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  } 

  if(readModeFromRFIDCard(&mode) != RFID_SUCCESS_READ) {
    json.clear();
    json["signal"] = "6";
    json["error_status"] = "3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  json.clear();
  json["signal"]="6";
  json["error_status"]="0";
  json["mode"] = ( mode  == 1 )? "1" : "2";
  sendBLEResponse();  
  // Keep card active
}

void handleSignal7() {
  char ph1[11];
  char ph2[11];
  char ph3[11];
  
  if(checkNewRFIDCardPresent() != RFID_SUCCESS_CARD_FOUND) {
    json.clear();
    json["signal"] = "7";
    json["error_status"] = "1";
    sendBLEResponse();
    return;
  }

  if(authRFIDCard(RFID_CARD_PHONE_NUMBER_1_BLOCK_ADDRESS, privateKey) != RFID_SUCCESS_AUTH) {
    json.clear();
    json["signal"] = "7";
    json["error_status"] = "2";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  } 

  if(readPhoneNumberFromRFIDCard(RFID_CARD_PHONE_NUMBER_1_BLOCK_ADDRESS, ph1) != RFID_SUCCESS_READ) {
    json.clear();
    json["signal"] = "7";
    json["error_status"] = "3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }
      
  if(readPhoneNumberFromRFIDCard(RFID_CARD_PHONE_NUMBER_2_BLOCK_ADDRESS, ph2) != RFID_SUCCESS_READ) {
    json.clear();
    json["signal"] = "7";
    json["error_status"] = "3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  if(readPhoneNumberFromRFIDCard(RFID_CARD_PHONE_NUMBER_3_BLOCK_ADDRESS, ph3) != RFID_SUCCESS_READ) {
    json.clear();
    json["signal"] = "7";
    json["error_status"] = "3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  json.clear();
  json["signal"]="7";
  json["error_status"]="0";
        
  JsonArray phones = json.createNestedArray("phone_numbers");
  phones.add(ph1);
  phones.add(ph2);
  phones.add(ph3);
  sendBLEResponse(); 
  // Keep card active
}

void handleSignal8(){
  uint8_t mode = getModeFromRequestPayload();

  if(checkNewRFIDCardPresent() != RFID_SUCCESS_CARD_FOUND) {
    json.clear();
    json["signal"] = "8";
    json["error_status"] = "1";
    sendBLEResponse();
    return;
  }

  if(authRFIDCard(RFID_CARD_MODE_BLOCK_ADDRESS, privateKey) != RFID_SUCCESS_AUTH) {
    json.clear();
    json["signal"] = "8";
    json["error_status"] = "2";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  if(writeModetoRFIDCard(mode) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"] = "8";
    json["error_status"] = "3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  } 

  json.clear();
  json["signal"]="8";
  json["error_status"]="0";
  sendBLEResponse();  
  // Keep card active
}

void handleSignal9() {
  char ph1[11];
  char ph2[11];
  char ph3[11];

  if(checkNewRFIDCardPresent() != RFID_SUCCESS_CARD_FOUND) {
    json.clear();
    json["signal"] = "9";
    json["error_status"] = "1";
    sendBLEResponse();
    return;
  }

  if(authRFIDCard(RFID_CARD_PHONE_NUMBER_1_BLOCK_ADDRESS, privateKey) != RFID_SUCCESS_AUTH) {
    json.clear();
    json["signal"] = "9";
    json["error_status"] = "2";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  getPhoneNumberFromRequestPayload(ph1,ph2,ph3);

  if(writePhoneNumberToRFIDCard(RFID_CARD_PHONE_NUMBER_1_BLOCK_ADDRESS, ph1) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"]="9";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  if(writePhoneNumberToRFIDCard(RFID_CARD_PHONE_NUMBER_2_BLOCK_ADDRESS, ph2) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"]="9";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }
  
  if(writePhoneNumberToRFIDCard(RFID_CARD_PHONE_NUMBER_3_BLOCK_ADDRESS, ph3) != RFID_SUCCESS_WRITE) {
    json.clear();
    json["signal"]="9";
    json["error_status"]="3";
    sendBLEResponse();
    endRFIDCommunication();
    return;
  }

  json.clear();
  json["signal"]="9";
  json["error_status"]="0";
      
  sendBLEResponse();
  // Keep card active
}

//Initialize BLE 
void bleInit(){
  BLEDevice::init(MACHINE_ID);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new RechargeServerCallbacks());
  pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  
  pCharacteristic->setCallbacks(new RechargeWriteCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
}

//Initialize RFID
void rfidInit(){
  SPI.begin();
  rfid.PCD_Init();
  
  // UPGRADE 1: Maximize Antenna Gain
  // This is critical for reading through enclosure plastic
  rfid.PCD_SetAntennaGain(rfid.RxGain_38dB);;
}

//Initialize Indicaters
void indicatersInit() {
  pinMode(BLE_INDICATION_LED_PIN,OUTPUT);
  pinMode(BUZZER_PIN,OUTPUT);
}

void setup() { 
  // Serial.begin(9600);
  indicatersInit();
  bleInit();
  rfidInit();
}

void loop() {
  // Periodically check if card is still present
  checkCardPresence();
  
  if(bleDataAvailable){
    bleDataAvailable=false;
    if(parseJson(bleDataBuffer) != JSON_SUCCESS_PARSE) {
      json.clear();
      json["signal"] = "7";
      sendBLEResponse();
      return;
    }
    
    uint8_t signal = getSignalFromRequestPayload();

    switch(signal){
      case 1:
        handleSignal1();
        break;
      case 2:
        handleSignal2();
        break;
      case 3:
        handleSignal3();
        break;
      case 4:
        handleSignal4();
        break;
      case 5:
        handleSignal5();
        break;
      case 6:
        handleSignal6();
        break;
      case 7:
        handleSignal7();
        break;
      case 8:
        handleSignal8();
        break;
      case 9:
        handleSignal9();
        break;  
      default:
        json.clear(); 
        json["signal"] = "8";
        sendBLEResponse(); 
     }
  }

  vTaskDelay(pdMS_TO_TICKS(10));
}
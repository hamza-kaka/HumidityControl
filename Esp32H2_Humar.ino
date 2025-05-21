#include <Arduino.h>
#include <BLEDevice.h>

#define MAX_CONNECTIONS 2
#define SCAN_TIME 5 // seconds

#define HUMIDIFIER_ADDRESS "01:26:3c:00:8c:bc"
#define SENSOR_NAME "LYWSD03MMC"

#define SENSOR_INDEX 0 
#define HUMIDIFIER_INDEX 1

#define SSUUID "ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6"
#define SCUUID "ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6"
#define HSUUID 0xFFF0 //"0000FFF0-0000-1000-8000-00805F9B34FB"
#define HWUUID 0xFFF2 //"0000FFF2-0000-1000-8000-00805F9B34FB"
#define HRUUID 0xFFF1 //"0000FFF1-0000-1000-8000-00805F9B34FB"

#define HEADER0 0xF
#define HEADER1 0xE

#define WDT_TIMEOUT 20000 // time in ms



BLEClient *pClient[MAX_CONNECTIONS];
BLEScan *pBLEScan;
static BLEAdvertisedDevice *ServerBLE[MAX_CONNECTIONS+2];
static BLERemoteCharacteristic* pSensorChar;
static BLERemoteCharacteristic* pHumReadChar;
static BLERemoteCharacteristic* pHumWriteChar;

bool sensorConnected = false;
bool humidConnected = false;
bool sensorRegistered = false;
bool humidRegistered = false;
bool sensorFound = false;
bool humidFound = false;

uint8_t humidifierEnable = 0;
uint8_t humidifierState = 0;
float sensorTemp;
float sensorHumidity;

hw_timer_t *timer = NULL;


// The remote temp/humidity service we wish to connect to.
static BLEUUID sensorServiceUUID(SSUUID);
// The characteristic of the remote temp/humidity service we are interested in.
static BLEUUID sensorCharUUID(SCUUID);

// The remote humidifier service we wish to connect to.
static BLEUUID humidifierServiceUUID(uint16_t(HSUUID));
// The characteristic of the remote humidifier service to write to.
static BLEUUID humidifierWriteCharUUID(uint16_t(HWUUID));
// The characteristic of the remote humidifier service to read from.
static BLEUUID humidifierReadCharUUID(uint16_t(HRUUID));


#undef CONFIG_BTC_TASK_STACK_SIZE
#define CONFIG_BTC_TASK_STACK_SIZE 32768

void ARDUINO_ISR_ATTR resetModule() {
  ets_printf("reboot\n");
  esp_restart();
}

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
    if(pclient->getPeerAddress().toString().c_str() == HUMIDIFIER_ADDRESS)
    {
      humidConnected = true;
    }
    else
    {
      sensorConnected = true;
    }
    
    Serial.printf(" * Connected %s\n", pclient->getPeerAddress().toString().c_str());
  }

  void onDisconnect(BLEClient *pclient)
  {
    if(pclient->getPeerAddress().toString().c_str() == HUMIDIFIER_ADDRESS)
    {
      humidConnected = false;
      humidRegistered = false;
    }
    else
    {
      sensorConnected = false;
      sensorRegistered = false;
    }
    Serial.printf(" * Disconnected %s\n", pclient->getPeerAddress().toString().c_str());
  }
};


/**
 * Scan for BLE servers and find the first one that 
 * advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
/**
     * Called for each advertising BLE server.
     */
    void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, match for its name.

    if (!advertisedDevice.getName().compareTo(SENSOR_NAME)) {

        BLEAdvertisedDevice* bleDevice = new BLEAdvertisedDevice(advertisedDevice);
        ServerBLE[SENSOR_INDEX] = bleDevice;
        Serial.print("BLE address of sensor :");
        Serial.println( bleDevice->toString().c_str());

        sensorFound = true;
    } 
    else if((advertisedDevice.getAddress().toString().c_str() == HUMIDIFIER_ADDRESS) || !advertisedDevice.getName().compareTo("E104-BT52-V2.0")) //
    {
        BLEAdvertisedDevice* bleDevice = new BLEAdvertisedDevice(advertisedDevice);
        ServerBLE[HUMIDIFIER_INDEX] = bleDevice;
        Serial.print("BLE address of humidifier :");
        Serial.println( bleDevice->toString().c_str());

        humidFound = true;
    }

    if(sensorFound && humidFound)
    {
        pBLEScan->stop();
        Serial.print("BLE server devices found ");
    }
    } // onResult
}; // MyAdvertisedDeviceCallbacks

void createBleClientsWithCallbacks()
{
    for (int i = 0; i < MAX_CONNECTIONS; i++)
    {
        pClient[i] = BLEDevice::createClient();
        pClient[i]->setClientCallbacks(new MyClientCallback());
    }
}

static void sensorNotifyCallback(
  BLERemoteCharacteristic *pBLERemoteCharacteristic,
  uint8_t *pData,
  size_t length,
  bool isNotify)
{
float temp;
float humi;
float voltage;
// Serial.print(" + Notify callback for sensor characteristic ");
Serial.println(pBLERemoteCharacteristic->getUUID().toString().c_str());
sensorTemp = temp = (pData[0] | (pData[1] << 8)) * 0.01; //little endian
sensorHumidity = humi = pData[2];
voltage = (pData[3] | (pData[4] << 8)) * 0.001; //little endian
Serial.printf("temp = %.1f C ; humidity = %.1f %% ; voltage = %.3f V\n", temp, humi, voltage);
//pClient->disconnect();
}


uint8_t sensorRegistration()
{
  uint8_t ret = 1;
  if (!pClient[SENSOR_INDEX]->isConnected())
  {
    Serial.println(" - Sensor not connected");
    return 1;
  }

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient[SENSOR_INDEX]->getService(sensorServiceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print(" - Failed to find sensor service UUID: ");
    Serial.println(sensorServiceUUID.toString().c_str());
    pClient[SENSOR_INDEX]->disconnect();
    ret = 1;
  }
  else{
    Serial.println(" + Found our sensor service");
  }


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pSensorChar = pRemoteService->getCharacteristic(sensorCharUUID);
  if (pSensorChar == nullptr)
  {
    Serial.print(" - Failed to find our characteristic UUID: ");
    Serial.println(sensorCharUUID.toString().c_str());
    pClient[SENSOR_INDEX]->disconnect();
    ret = 1;
  }
  else
  {
    Serial.println(" + Found our sensor characteristic");
  }

  if(pSensorChar->canNotify())
  {
    Serial.println(" sensor can notify");
    pSensorChar->registerForNotify(sensorNotifyCallback);
    ret = 0;
  }

  return ret;
}

static void humidifierNotifyCallback(
  BLERemoteCharacteristic *pBLERemoteCharacteristic,
  uint8_t *pData,
  size_t length,
  bool isNotify)
{
uint8_t state;
uint8_t enable;
uint8_t header1;
uint8_t header2;

Serial.print(" + Notify callback humidifier read char ");
Serial.println(pBLERemoteCharacteristic->getUUID().toString().c_str());
header1 = pData[0];
header2 = pData[1];
enable = pData[2];
humidifierState = state = pData[3];
if(header1 != 0x0F || header2 != 0x0E)
{
    Serial.println("Invalid header");
}

Serial.printf("enable = % hhu ; status = % hhu \n", enable, state);
}

uint8_t humidifierRegistration()
{
  if (!pClient[HUMIDIFIER_INDEX]->isConnected())
  {
    Serial.println(" - Humidifier not connected");
    return 1;
  }

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService *pRemoteService = pClient[HUMIDIFIER_INDEX]->getService(humidifierServiceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print(" - Failed to find sensor service UUID: ");
    Serial.println(humidifierServiceUUID.toString().c_str());
    pClient[HUMIDIFIER_INDEX]->disconnect();
  }
  else
  {
    Serial.println(" + Found our humidifier service");
  }
  

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pHumReadChar = pRemoteService->getCharacteristic(humidifierReadCharUUID);
  if (pHumReadChar == nullptr)
  {
    Serial.print(" - Failed to find our read characteristic UUID: ");
    Serial.println(humidifierReadCharUUID.toString().c_str());
    pClient[HUMIDIFIER_INDEX]->disconnect();
  }
  else
  {
    Serial.println(" + Found our humidifier read characteristic");
  }

  if(pHumReadChar->canNotify())
  {
    Serial.println(" humidifier can notify");
    pHumReadChar->registerForNotify(humidifierNotifyCallback);
  }

  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pHumWriteChar = pRemoteService->getCharacteristic(humidifierWriteCharUUID);
  if (pHumWriteChar == nullptr)
  {
    Serial.print(" - Failed to find our write characteristic UUID: ");
    Serial.println(humidifierWriteCharUUID.toString().c_str());
    pClient[HUMIDIFIER_INDEX]->disconnect();
  }
  else
  {
    Serial.println(" + Found our humidifier write characteristic");
  }
  

  return 0;
}

void connectSensor(BLEAddress htSensorAddress)
{
  pClient[SENSOR_INDEX]->connect(htSensorAddress);
}

void connectHumidifier(BLEAddress htHumidifierAddress)
{
  pClient[HUMIDIFIER_INDEX]->connect(htHumidifierAddress);
}

void ProcessSensorData()
{
  // Process the data from the sensor here

  Serial.printf("Temperature: %.1f °C, Humidity: %.1f %%\n", sensorTemp, sensorHumidity);
  humidifierEnable = 1; // Set to 1 to enable the humidifier or whatever you want
  
}
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting HUMAR Esp32H2 application...");
  delay(500);

  BLEDevice::init("ESP32H2");
  createBleClientsWithCallbacks();

  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);   //active scan uses more power, but get results faster
  pBLEScan->setInterval(0x50);
  pBLEScan->setWindow(0x30);

  timer = timerBegin(1000000);                     //timer 1Mhz resolution
  timerAttachInterrupt(timer, &resetModule);       //attach callback
  timerAlarm(timer, WDT_TIMEOUT * 1000, false, 0);  //set time in us
}

// This is the Arduino main loop function.
void loop() {
    timerWrite(timer, 0);  //reset timer (feed watchdog)
    long loopTime = millis();
    delay(1000);
    // sensorConnected = pClient[SENSOR_INDEX]->isConnected();
    // humidConnected = pClient[HUMIDIFIER_INDEX]->isConnected();
    if(pClient[SENSOR_INDEX]->isConnected() && pClient[HUMIDIFIER_INDEX]->isConnected())
    {
        Serial.println("Both devices connected.");
    }
    else
    {
        delay(500);
        sensorFound = false;
        humidFound = false;
        Serial.println("SCANNING.");
        BLEScanResults foundDevices = *(pBLEScan->start(SCAN_TIME));
        Serial.printf("foundDevices.getCount()= %i \n",foundDevices.getCount());
        // for(int i=0;i<foundDevices.getCount();i++)
        // {
        //     BLEAdvertisedDevice res = foundDevices.getDevice(i);
        //     Serial.print(res.getName().c_str());
        //     Serial.print(" @ ");
        //     Serial.println(res.getAddress().toString().c_str());
        // }
        Serial.print("Saved Server DEVICE NAMES\n");
        for(int i=0; i<2; i++)
        {
          if(ServerBLE[i] != nullptr)
          {
              Serial.print(ServerBLE[i]->getName().c_str());
              Serial.print(" @ ");
              Serial.println(ServerBLE[i]->getAddress().toString().c_str());
          }
        }

        if(!pClient[SENSOR_INDEX]->isConnected() && sensorFound)
        {
            connectSensor(ServerBLE[SENSOR_INDEX]->getAddress());
        }
        if(!pClient[HUMIDIFIER_INDEX]->isConnected() && humidFound)
        {
            connectHumidifier(ServerBLE[HUMIDIFIER_INDEX]->getAddress());
        }

    }

    if(pClient[HUMIDIFIER_INDEX]->isConnected() && !humidRegistered)
    {
      humidRegistered = !humidifierRegistration();
    }
    

    if(pClient[SENSOR_INDEX]->isConnected() && !sensorRegistered)
    {
      sensorRegistered = !sensorRegistration();
    }

    ProcessSensorData();

    if(pClient[HUMIDIFIER_INDEX]->isConnected() && humidRegistered && pHumWriteChar->canWrite())
    {
      uint8_t dataSent[4] = {HEADER0, HEADER1, humidifierEnable, humidifierState};
      pHumWriteChar->writeValue(dataSent, sizeof(dataSent), true);
      //pHumWriteChar->writeValue("sup written value", true);
      Serial.println("Write humidifier characteristic");
    }

    loopTime = millis() - loopTime;

    Serial.print("loop time is = ");
    Serial.println(loopTime);  //should be under 15000
    

}
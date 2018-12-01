/*****************************************************************************************************************************
 
                                                   Flow Sensor + Displays + BLE
    
*****************************************************************************************************************************/
#include  "LedControl.h"
#include <string>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define PIN0 14
#define PIN1 12 //todo
#define PIN2 13//todo
#define led_data 21 //Pin location for data to led displays
#define led_load 22 //Pin location for load to led displays
#define led_clk 23 //Pin location for clock to led displays
#define max_displays 3 //maximum number of disolays the design will use
#define hallsensor0 2 //The pin location of the sensor
#define hallsensor1 0
#define hallsensor2 4
#define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // UART service UUID
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"


volatile int NbTopsFan;
float flowCalc0; // Variable to hold value for flow rate
float flowCalc1;
float flowCalc2;
LedControl lc = LedControl(led_data, led_clk, led_load, max_displays);
BLECharacteristic *pCharacteristic;
BLEDescriptor *pDescriptor;
bool deviceConnected = false;
bool deviceNotifying = false;
uint8_t txValue = 0;


/*****************************************************************************************************************************
 
                                                        Custom
                                                      Functions
    
*****************************************************************************************************************************/
float flowMeter()
{
  NbTopsFan=0;
  sei(); //Start Interrupts
  delay(250);
  cli(); //End  Interrupts
  return ((NbTopsFan * 60 / 6.5)/227.125); //Math for flow rate in gpm
}


void printNumber(float f, int n) 
{
  bool dp [8]= {false,false,false,false,false,false,false,false};
  String all;
  all = String(f);
  int temp = all.indexOf('.');
  all.remove(temp, 1);
  dp[temp-1] = true;
  lc.setChar(n, 7, (byte)all[0], dp[0]);
  lc.setChar(n, 6, (byte)all[1], dp[1]);
  lc.setChar(n, 5, (byte)all[2], dp[2]);
  lc.setChar(n, 4, (byte)all[3], dp[3]);
  lc.setChar(n, 3, (byte)all[4], dp[4]);
  lc.setChar(n, 2, (byte)all[5], dp[5]);
  lc.setChar(n, 1, (byte)all[6], dp[6]);
  lc.setChar(n, 0, (byte)all[7], dp[7]);
}

/*****************************************************************************************************************************
 
                                                      Bluetooth 
                                                      Low Energy
    
*****************************************************************************************************************************/
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};
class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      if (rxValue.length() > 0) {
        Serial.println("*********");
        Serial.println("Received Value: ");
        for (int i = 0; i < rxValue.length(); i++)
          {Serial.print(i); Serial.print(": ");Serial.println(rxValue[i]);}

        if (rxValue[0]=='A') {
          digitalWrite(PIN0, HIGH);
        }
        if (rxValue[0]=='B') {
          digitalWrite(PIN0, LOW);
        }

        Serial.println();
        Serial.println("*********");
      }
    }
};
class MyDisCallbacks: public BLEDescriptorCallbacks {
    void onWrite(BLEDescriptor *pDescriptor) {
      uint8_t* rxValue = pDescriptor->getValue();

      if (pDescriptor->getLength() > 0) {
        if (rxValue[0]==1) {
          //deviceNotifying=true;
        } else {
          deviceNotifying=false;
        }
        Serial.println("*********");
        Serial.print("Received Descriptor Value: ");
        for (int i = 0; i < pDescriptor->getLength(); i++)
          Serial.print(rxValue[i]);

        Serial.println();
        Serial.println("*********");
      }
    }
};
void setupBLE()
{
  pinMode(PIN0, OUTPUT);

  // Create the BLE Device
  BLEDevice::init("UART Service");

  // Create the BLE Server
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create a BLE Characteristic
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pDescriptor = new BLE2902();
  pCharacteristic->addDescriptor(pDescriptor);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                       );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pDescriptor->setCallbacks(new MyDisCallbacks());

  // Start the service
  pService->start();

  // Start advertising
  pServer->getAdvertising()->start();
  Serial.println("Waiting a client connection to notify...");
}
/*****************************************************************************************************************************
 
                                                        Interrupts
    
*****************************************************************************************************************************/
 
void rpm () //This is the function that the interupt calls
{
    NbTopsFan++; //This function measures the rising and falling edge of the hall effect sensors signal
}
/*****************************************************************************************************************************
 
                                                        Setup
    
*****************************************************************************************************************************/
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); //This is the setup function where the serial port is initialised,
  setupFlowSensor();
 
  setupLED();
  setupBLE();
}
void setupFlowSensor()
{
  pinMode(hallsensor0, INPUT); //initializes flow rate sensor 0
  pinMode(hallsensor1, INPUT); //initializes flow rate sensor 1
  pinMode(hallsensor2, INPUT); //initializes flow rate sensor 2
  detachInterrupt(digitalPinToInterrupt(hallsensor0)); //Interrupted set for flow sensor 0
  detachInterrupt(digitalPinToInterrupt(hallsensor1)); //Interrupted set for flow sensor 1
  detachInterrupt(digitalPinToInterrupt(hallsensor2)); //Interrupted set for flow sensor 2
}
void setupLED()
{
   for (int i = 0; i < max_displays; i++)
  {
    lc.shutdown(i, false); // Enable display
    lc.setIntensity(i, 8); // Set brightness level (0 is min, 15 is max)
    lc.clearDisplay(0); // Clear display register
  }
  for(int i = 0; i < max_displays; i++)
  {
    printNumber(1111111, i);
    delay(250);
    printNumber(2222222, i);
    delay(250);
    printNumber(3333333, i);
    delay(250);
    printNumber(4444444, i);
    delay(250);
    printNumber(5555555, i);
    delay(250);
    printNumber(6666666, i);
    delay(250);
    printNumber(7777777, i);
    delay(250);
    printNumber(8888888, i);
    delay(250);
    printNumber(9999999, i);
    delay(250);
    printNumber(0, i);
  }
}
/*****************************************************************************************************************************
 
                                                        Main
                                                        Loop
    
*****************************************************************************************************************************/
void loop() {
  // put your main code here, to run repeatedly:
  
  attachInterrupt(digitalPinToInterrupt(hallsensor0), rpm, RISING); //Interrupted set for flow sensor 0
  flowCalc0 = flowMeter(); 
  Serial.print(flowCalc0); Serial.println(" gpm[0]");
  attachInterrupt(digitalPinToInterrupt(hallsensor1), rpm, RISING);
  detachInterrupt(digitalPinToInterrupt(hallsensor0));
  flowCalc1 = flowMeter(); 
  Serial.print(flowCalc1); Serial.println(" gpm[1]");
  attachInterrupt(digitalPinToInterrupt(hallsensor2), rpm, RISING);
  detachInterrupt(digitalPinToInterrupt(hallsensor1));
  flowCalc2 = flowMeter(); 
  Serial.print(flowCalc2); Serial.println(" gpm[2]");
  detachInterrupt(digitalPinToInterrupt(hallsensor2));
  printNumber(flowCalc0, 0);
  printNumber(flowCalc1, 1);
  printNumber(flowCalc2, 2);
}




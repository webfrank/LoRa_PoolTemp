#include <Arduino.h>
#include <CayenneLPP.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "LoRaWan_APP.h"

#define ONE_WIRE_BUS GPIO0

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature sensors(&oneWire);

/*
 * set LoraWan_RGB to 1,the RGB active in loraWan
 * RGB red means sending;
 * RGB green means received done;
 */
#define LoraWan_RGB 0

#define RF_FREQUENCY                                866000000 // Hz

#define TX_OUTPUT_POWER                             14        // dBm

#define LORA_BANDWIDTH                              0         // [0: 125 kHz,
                                                              //  1: 250 kHz,
                                                              //  2: 500 kHz,
                                                              //  3: Reserved]
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE                             1         // [1: 4/5,
                                                              //  2: 4/6,
                                                              //  3: 4/7,
                                                              //  4: 4/8]
#define LORA_PREAMBLE_LENGTH                        8         // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT                         0         // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON                  false
#define LORA_IQ_INVERSION_ON                        false


#define RX_TIMEOUT_VALUE                            1000
#define BUFFER_SIZE                                 30 // Define the payload size here

char txPacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;

CayenneLPP lpp(51);
byte chipID;
float voltage, temp;
static TimerEvent_t wakeUpTimer;
#define SLEEP 300000 //3600000

typedef enum {
    LOWPOWER,
    ReadVoltage,
    ReadTemp,
    TX
} States_t;

States_t state;

void onWakeUp() {
  Serial.println("TIMER");
  Serial.println(millis());
  state = ReadVoltage;
  
  pinMode(Vext,OUTPUT);
  digitalWrite(Vext,LOW);
  delay(50);
  
  // Start up the library
  sensors.begin();
}

//CRC-8 - algoritmo basato sulle formule di CRC-8 di Dallas/Maxim
//codice pubblicato sotto licenza GNU GPL 3.0
byte CRC8(const byte *data, byte len) {
  byte crc = 0x00;
  while (len--) {
    byte extract = *data++;
    for (byte tempI = 8; tempI; tempI--) {
      byte sum = (crc ^ extract) & 0x01;
      crc >>= 1;
      if (sum) {
        crc ^= 0x8C;
      }
      extract >>= 1;
    }
  }
  return crc;
}

float readVoltage() {
  float v = (float)getBatteryVoltage() / 1000;
  Serial.print("Battery voltage is: ");
  Serial.println(v,2);
  return v;
}

float readTemperature() {
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus
  Serial.print("Requesting temperatures...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  // After we got the temperatures, we can print them here.
  // We use the function ByIndex, and as an example get the temperature from the first sensor only.
  Serial.print("Temperature for the device 1 (index 0) is: ");
  Serial.println(sensors.getTempCByIndex(0));

  return sensors.getTempCByIndex(0);
}

void setup() {
  Serial.begin(115200);

  boardInitMcu( );
    
  Radio.Init( &RadioEvents );
  Radio.SetChannel( RF_FREQUENCY );
  Radio.SetTxConfig( MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                                   LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                                   LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                                   true, 0, 0, LORA_IQ_INVERSION_ON, 3000 );
  
  chipID = CRC8((byte*)HW_Get_MFT_SN(), 16);
  Serial.print("NODE ID: "); Serial.println(chipID);

  TimerInit( &wakeUpTimer, onWakeUp );

  state = ReadVoltage;
}

void loop() {
  switch(state) {
    case TX: {
      Serial.println("TX");
      Radio.Standby();
      turnOnRGB(COLOR_SEND,0);

      lpp.reset();
      lpp.addDigitalInput(0, chipID);
      lpp.addVoltage(1, voltage);
      lpp.addTemperature(2, temp);
      
      Radio.Send(lpp.getBuffer(), lpp.getSize()); //send the package out
      delay(100);
      
      state = LOWPOWER;
      TimerSetValue( &wakeUpTimer, SLEEP );
      TimerStart( &wakeUpTimer );

      Radio.Sleep();
      turnOffRGB();
      break;
    }
    case LOWPOWER: {
      Serial.println("LOWPOWER");
      lowPowerHandler();
      break;
    }
    case ReadVoltage: {
      Serial.println("ReadVoltage");
      voltage = readVoltage();
      state = ReadTemp;
      break;
    }
    case ReadTemp: {
      Serial.println("ReadTemp");
      temp = readTemperature();
      state = TX;
      break;
    }
    default: {
      Serial.print(".");
      break;
    }
  }

}
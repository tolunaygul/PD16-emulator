#include <mcp_can.h>
#include <SPI.h>

#include "PCF8574.h"

PCF8574 PCF_01(0x3F);

long unsigned int rxId;           // storage for can data
unsigned char len = 0;            // storage for can data
unsigned char rxBuf[8];           // storage for can data

#define CAN0_INT 2             // Set INT to pin 2
MCP_CAN CAN0(10);                 // set CS pin to 10r

int scaledvalue1 = 0;             // storage for 12 bit analog value
int scaledvalue2 = 0;             // storage for 12 bit analog value
int scaledvalue3 = 0;             // storage for 12 bit analog value
int scaledvalue4 = 0;             // storage for 12 bit analog value

unsigned long task1Interval = 50; // 30ms interval for analogue value sending
unsigned long task2Interval = 2;// 50ms interval for keep aliv frame
unsigned long task3Interval = 10; // 3 second interval for task 3
unsigned long task4Interval = 40; // 4 second interval for task 4
unsigned long task1Millis = 0;    // storage for millis counter
unsigned long task2Millis = 0;    // storage for millis counter
unsigned long task3Millis = 0;    // storage for millis counter
unsigned long task4Millis = 0;    // storage for millis counter

byte DPO1out = 0;                 // storage for DPO output 1
byte DPO2out = 0;                 // storage for DPO output 2
byte DPO3out = 0;                 // storage for DPO output 3
byte DPO4out = 0;                 // storage for DPO output 4

void setup() {
  // start serial port an send a message with delay for starting
  Serial.begin(115200);   
  Serial.println("analog reading to 12 bit test");
  delay(10);

    PCF_01.begin();

  // initialize canbus with 1000kbit and 16mhz xtal
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_16MHZ) == CAN_OK) 
  Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");

  // Set operation mode to normal so the MCP2515 sends acks to received data.
  CAN0.setMode(MCP_NORMAL);  

  digitalWrite(CAN0_INT, HIGH);  // set INT pin high to enable interna pullup
  pinMode(CAN0_INT, INPUT);      // set INT pin to be an input

  Serial.println("All OK");  // all ready to go !
}




void loop() {

  //Serial.println(!digitalRead(CAN0_INT));

  unsigned long currentMillis = millis();  // Get current time in milliseconds

  // Execute task 1 every 1 second
  if (currentMillis - task1Millis >= task1Interval) {
    task1Millis = currentMillis;
    SendKeepAlive();
  }

  // Execute task 2 every 5 seconds
  if (currentMillis - task2Millis >= task2Interval) {
    task2Millis = currentMillis;
    SendAnalogValues();
  }

  // Execute task 3 every 3 seconds
  if (currentMillis - task3Millis >= task3Interval) {
    task3Millis = currentMillis;
   // DriveDigitalPin();
  }

  // Execute task 4 every 4 seconds
  if (currentMillis - task4Millis >= task4Interval) {
    task4Millis = currentMillis;
    task4();
  }

  // read can buffer when interrupted and jump to canread for processing.
  if (!digitalRead(CAN0_INT))  // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);  // Read data: len = data length, buf = data byte(s)
    canRead();
  }
}


void canRead() {
  if (rxId == 0X6E8) 
    {
      if (rxBuf[0] == 0x03)
      {
        if ((rxBuf[1] == 3) || (rxBuf[2] == 0xeE8))
        PCF_01.write(4, !HIGH);
        else
        PCF_01.write(4, !LOW);
      }
  }
}


void DriveDigitalPin() {

}



void task4() {}

void SendKeepAlive() {
  byte KeepAlive[8] = { 0X10, 0x01, 0x21, 0x07, 0x00, 0x00, 0x00, 0x00  };
  CAN0.sendMsgBuf(0x6ED, 0, 8, KeepAlive);
}

byte counter = 0;
byte analogInArray[4] = {A0, A1, A2, A3};
byte analogMagicNumber[4] = {0x80, 0x81, 0x82, 0x83};
void SendAnalogValues()
{
  counter++;
  if(counter > 3) //Limit counter from 0-3
    counter = 0;
  
  unsigned int scaledvalue = map(analogInArray[counter], 1023, 0, 0, 5000); // read analogue value and scale to 12 bit
  
  byte sendData[8] = {analogMagicNumber[counter], 0x00, highByte(scaledvalue), lowByte(scaledvalue), 0x00, 0x00, 0x00, 0x00};
  
  CAN0.sendMsgBuf(0x6EB, 0, 8, sendData); // send the can message onto the bus
}
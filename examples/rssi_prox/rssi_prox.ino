#if defined (SPARK)
#include "XBee.h"
#else
#include <XBee.h>
#include <SoftwareSerial.h>
#endif

#include "FilteringScheme.h"

#if defined (SPARK)
USBSerial nss = Serial;
#else
// Define NewSoftSerial TX/RX pins
// Connect Arduino pin 9 to TX of usb-serial device
uint8_t ssRX = 15;
// Connect Arduino pin 10 to RX of usb-serial device
uint8_t ssTX = 4;
// Remember to connect all devices to a common Ground: XBee, Arduino and USB-Serial device
SoftwareSerial nss(ssRX, ssTX);
#endif

//#define DEBUG_RSSI		//Uncomment to enable debug print statements

int fadeValue = 0;

#if defined (SPARK)
int32_t random(int32_t howsmall, int32_t howbig)
{
  uint32_t r = howsmall;
  howbig = howbig - howsmall + 1;
  r += rand() % howbig;
  return r;
}
#endif

XBee xbee = XBee();
XBeeResponse response = XBeeResponse();
// create reusable response objects for responses we expect to handle 

Rx16Response rx16 = Rx16Response();

uint8_t payload[] = { 
  'T',0,0,0,0,0 };

//Tx64Request zbTx = Tx64Request(addr64, 0, payload, sizeof(payload),0x13);
Tx16Request tx = Tx16Request(0xFFFF, payload, sizeof(payload));
TxStatusResponse txStatus = TxStatusResponse();


const int ledPin = 13;      // the pin that the LED is attached to
int count = 0;
int LEDState = 0;
char fakeGND = 11;
int numNeighbor = 10;
int numLights = 4;
int currentNeighbors[] = {
  0,0,0,0,0,0,0,0,0,0};

long neighborTimers[] = {
  0,0,0,0,0,0,0,0,0,0};

int fadePWMs[] = {
  0,0,0,0};

long neighborAddress[] = {
  0,0,0,0,0,0,0,0,0,0};
char displayArray[] = {
  5,6,9,10};

long start = 0;
int calibrateRSSI[50];
int sortRSSI[50];
int threshold = 55;

long foursTimer = millis();

KalmanFilter kFilters[10];

void setup()
{
  // initialize the serial communication:
  xbee.begin(Serial);
  nss.begin(9600);
  pinMode(ledPin, OUTPUT);
  pinMode(fakeGND, OUTPUT);
  digitalWrite(ledPin,HIGH);
  digitalWrite(fakeGND,LOW);
  nss.println("Start it up!!!");

  start = millis();
  delay(10); 
  float qVal = 0.015; //Set Q Kalman Filter
  float rVal = 1.2; //Set K Kalman Filter
  
  
  for(int i = 0; i < numNeighbor; i++) { //Initialize Kalman Filters for 10 neighbors
    //KalmanFilter(float q, float r, float p, float intial_value);
    kFilters[i].KalmanInit(qVal,rVal,5.0,65);
    pinMode(displayArray[i],OUTPUT);
  }


  payload[1] = threshold;
  displayOn();
  delay(1000);
  fours();
  displayOff();
  
}

void loop() {

  //Find neighbors
  for(int i = 0; i < 3; i++) {
    findNeighbors();
  }
  
  //Broadcast message to neighbors
  sendDataToNeighbors();
  //check for disconnected neighbors
  for(int i = 0; i < numNeighbor; i++) {
#ifdef DEBUG_RSSI
	nss.print("NeighborTimer: ");
	nss.print(neighborTimers[i]);
	nss.print("  MilliTimer: ");
	nss.print(millis());
	nss.print("  Difference: ");
	nss.println(millis()-neighborTimers[i]);
#endif
    if(millis()-neighborTimers[i] > 2000){
      displayNeighbor(i,0);
    }
  }
  delay(100);
}


void findNeighbors() {
  xbee.readPacket();

  if (xbee.getResponse().isAvailable()) {
    // got something

    if (xbee.getResponse().getApiId() == RX_64_RESPONSE || xbee.getResponse().getApiId() == RX_16_RESPONSE) {
      // now fill our rx class     

      xbee.getResponse().getRx16Response(rx16);
      int rssiVal = rx16.getRssi();
      long address = rx16.getRemoteAddress16();
      if (address == 0) {
        if(rx16.getData(0) == 84) {
          threshold = rx16.getData(1); 
        }
      }

      int index = IndexAddressMapper(address);
      neighborTimers[index] = millis();
#ifdef DEBUG_RSSI
		nss.print(count);
		nss.print("   ");
		nss.print(rssiVal);
		nss.print("   ");
#endif
      int x = kFilters[index].measureRSSI(rssiVal);
#ifdef DEBUG_RSSI
		nss.print(x);
#endif
      if(x < threshold) {
        displayNeighbor(index,1);
      }
      else{
        displayNeighbor(index,0);
      }
#ifdef DEBUG_RSSI
		nss.print("    "); 
		nss.print(address);
		nss.print("    "); 
		nss.print(threshold);
		nss.print("    "); 
		nss.print(kFilters[0].r);
		nss.print("    "); 
		nss.println(kFilters[0].q);
#endif
      count++;

    }
  }
}

int IndexAddressMapper(long address) {
  for(int i = 0; i < numNeighbor; i++) {
    if(neighborAddress[i] == address) {
      return i;
    }
  }

  for(int i = 0; i < numNeighbor; i++) {
    if(neighborAddress[i] == 0) {
      neighborAddress[i] = address;
      return i;
    }
  }
}

void displayNeighbor(int index, int set) {
  currentNeighbors[index] = set;
  int total = 0;
  int i;
  for(i = 0; i < numNeighbor; i++) {
    total += currentNeighbors[i];
  }

  (total>3)?(total=3):(total=total); //set total to maximum of 3 because of 0 offset in c++ array

  if(total == 0) {
    displayOff();
    return;
  }
  
  if(total == 3) {
    fours();
    sendDataToNeighbors();
    return;
  }
  
 
  for(i = 0; i <total; i++ ) { //we want to display number of neighbors up to 4...2 people 2 lights
    digitalWrite(displayArray[0],1); //turn on 0 whenever there is a neighbor   
    digitalWrite(displayArray[i+1],1);
  }

  for(i = total; i <numLights-1; i++ ) { //turn off lights when neighbors leave
    digitalWrite(displayArray[i+1],0);
  }

}

void sendDataToNeighbors() {
  xbee.send(tx);
  delay(random(20,50));
}

void displayOn() {
  for(int i = 0; i < numLights; i++) {
    digitalWrite(displayArray[i],HIGH);
  }
}

void displayOff() {
  for(int i = 0; i < numLights; i++) {
    digitalWrite(displayArray[i],LOW);
  }
}

void fours() {
  for(int j = 0; j < 1; j++) {
    for(int i = 0; i < numLights; i++) {
      for(int fadeValue = 255 ; fadeValue >= 0; fadeValue -=10) { 
        // sets the value (range from 0 to 255):
        analogWrite(displayArray[i], fadeValue);        
        // wait for 30 milliseconds to see the dimming effect    
        delay(7); 
      }
    }
    for(int i = numLights-2; i > 0; i--) {
      for(int fadeValue = 255 ; fadeValue >= 0; fadeValue -=10) { 
        // sets the value (range from 0 to 255):
        analogWrite(displayArray[i], fadeValue);        
        // wait for 30 milliseconds to see the dimming effect    
        delay(7); 
      }
    }
  }
  for(int j = 0; j < 5; j++) {
    for(int i = 0; i < numLights; i++) {
      digitalWrite(displayArray[i],HIGH);
    }
    delay(100);
    for(int i = 0; i < numLights; i++) {
      digitalWrite(displayArray[i],LOW);
    }
    sendDataToNeighbors();    
  }
}

int sumFade() {
  int temp = 0;
  for(int i = 0; i < numLights; i++){
    temp += fadePWMs[i];
  }
  return temp;

}


int getRandomIndex() {
  int index = random(0,numLights);
  if(fadePWMs[index] < 255) {
    return index;
  }
  else {
    while (fadePWMs[index] > 255) {
      index = random(0,numLights);
    }
  }
  return index;
}











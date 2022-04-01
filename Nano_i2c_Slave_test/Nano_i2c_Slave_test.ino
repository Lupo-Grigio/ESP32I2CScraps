// Testing I2C client mode on Arduino Nano

#include <Wire.h>

//#define LED_BUILTIN  2
#define DEBUG_BAUD 9600 // nano does not have much power... 9600 baud is stable

////////// stuff that should be put in a library because it'll be shared by a bunch of different sketches
#define SLAVE_ADDRESS 0x52 // our address on both I2C busses, so any master knows where to talk to
                           // I borrowed this ID from the Adafruit Arcade library that maps Wii Nunchuck to I2C

int period = 1000;
unsigned long time_now = 0;

void setup() 
{
  Serial.begin(DEBUG_BAUD);
  
  Wire.begin(SLAVE_ADDRESS); // I2C bus 1 setup SLAVE
 // setup interrupt response functions to recieve data from M4sk
  // Only the primary monster M4sk should ever send data
  Wire.onReceive(receivePrimaryEyesEvent); // register event for when the master writes

  // setup interrupt response functions to send data to M4Sks 
  // When the ESP32 detects a face or recieves a gase change event from
  // Bluetooth, or Web, or controller or whatever
  Wire.onRequest(WriteToPrimaryEvent); // register event
}


void loop() 
{

  time_now = millis();
  while(millis() < time_now + period)
  {
  }
  Serial.print("Status Bump \n");
  
  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ? LOW : HIGH);
}


// READ DATA FROM PRIMARY MASK
// the primary mask is sending us an eye state change message
// read it, parce it, send it to the secondary. 
// TODO read whole message
// TODO parce message
// TODO send to secondary
void receivePrimaryEyesEvent(int dummy)
{
  Serial.print("Slave Requested to READ from I2C");
  
  while(1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  Serial.println(" : ");
}
// Write an Eye change message to primary M4SK
void WriteToPrimaryEvent()
{
  Serial.print("Slave Requested to WRITE on I2C bus : ");
  
  String Msg = String(" Testing TeStInG 1256 \n");

  Serial.print(Msg.c_str());

  Wire.write(Msg.c_str()); 
}

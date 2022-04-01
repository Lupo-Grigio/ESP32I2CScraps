// Simple Arduino sketch to help verify the Monster M4sk i2c bus is connected to something
// this sketch assumes you are running the i2c_slave_Writer sample, or a sketch on a device
// at SLAVE_ADDRESS  which writes a string to the I2C bus 
// 
// There are a bunch of things on the I2C bus to start with, so a bus scan should show a bunch of things...
// Remember the M4SK i2c bus defaults to 5V. So connecting with arduino nano, uno or the like is straight forward
// if connecting to an ESP32.. you ethier need a logic level shifter down to 3v or solder the jumper on the board for 3v
// When wiring the Monster M4sk
// look at the board from the back nose bridge down
// the I2C plug is on the middle of the left, right below the light sensor
// the pin order is (From top to bottom, top being closest to light sensor)
// SCL
// SDA
// +5
// Ground (bottom most )
// the 3 pin socket just below the I2C plug is for 
// On the right and left sides, towards the bottom, are two connectors labeled 
// D2 and D3. These are 3-pin JST digital or analog connectors for sensors or NeoPixels. 
// These pins can be analog inputs or digital I/O.
// They have protection 1K resistors + 3.6V zener diodes so you can drive an LED 
// directly from the output. Connect to them via board.D2 and board.D3 or Arduino 2 and 3. 
// For analog reading in Arduino use A2 for D2 and A3 for D3.

#include <Wire.h>    //use SCl & SDA



#define SLAVE_ADDRESS 0x52 // our address on both I2C busses, so any master knows where to talk to

#define DEBUG_BAUD 115200
int period = 500;
unsigned long time_now = 0;

String IncomingString = String(" ");


void setup() 
{
  Wire.begin(); // Join the I2C bus as master

  // put your setup code here, to run once:
  Serial.begin(DEBUG_BAUD);
  Serial.println("M4SK about to scan I2C bus");

}

void loop() 
{
 time_now = millis();
  while(millis() < time_now + period)
  {
  }
  
  ScanBus();
  
}

void ScanBus()
{
  byte error, address;
  int nDevices;
  Serial.println("Scanning...");
  nDevices = 0;
  for(address = 1; address < 127; address++ ) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("I2C device found at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
      if (address == SLAVE_ADDRESS)
      {
        HandleI2CInput(SLAVE_ADDRESS);
      }
      nDevices++;
    }
    else if (error==4) {
      Serial.print("Unknow error at address 0x");
      if (address<16) {
        Serial.print("0");
      }
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0) {
    Serial.println("No I2C devices found\n");
  }
  else {
    Serial.println("done\n");
  }
}


// Since the serial read buffer may contain an incmplete message, it needs
// to persist between calls to user_loop, thus this global IncomingString variable
// Once a complete message has been recieved and parced IncomingString will be 
// emptied so the next message can be processed during the next call to user_loop

bool HandleI2CInput(int ID) // read from I2C device ID
{
  char c;
  bool ret = true;
  bool EOM = false;
  Wire.requestFrom(ID, 100);    // request 1 bytes from peripheral device #ID

  while(Wire.available()>0 && !EOM)    // slave may send less than requested
  {
    IncomingString += (char)Wire.read();

    if(IncomingString.endsWith("\n") || IncomingString.endsWith("\n "))
     {
      EOM = true;
       Serial.println("EOM");

      // FaceLocationStruct FaceAt;
      ret = true;
      Serial.print("Full Message : ");
      Serial.println(IncomingString);
      
      //ParceIncomingString(IncomingString,  &FaceAt);
      IncomingString.remove(1); // empty the string
      //moveEyesRandomly = false; // stop random eye movement TODO: Time this to prevent jerking
      //LookAt( &FaceAt );
       
     }
  }
    
  return(ret); 

}


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
#define DEBUG // This is a sample after all library code should have serial.print wrapped in #if defined(DEBUG)

// USE the same format string from the sister project to this one
// for example ../ESP32-Eye-Tracker/FaceRepoting.h
// ideally we would just include that here, but for a good explination
// why we can't google " arduino include headder files from other directories"
static const char EyeGaseRangeReportFormat[] = " Face found = %i X = %i Y = %i Blink = %i Pupil = %1 W = %i H = %i W_Min = %i H_Min = %i W_Max = %i H_Max = %i\n ";

typedef struct {
  int IS; // != 1 face not found or data tx/rx muck up co=ords are invalid
  int X; // X co-ord of face
  int Y; // Y co-ord of face
  int Blink; // BLINK Boolean, 0 if not blink 1 if blink
  int Pupil; // pupil diameter value of 0 means the mask will set this to whatever the rest of the code does
  int W; // maximum value along the horizontal, Should be <= SCREEN_WIDTH
  int H; // maximum value along the vertical, should be <= SCREEN_HEIGHT
  
  // A face detected in the field of view will never have it's X and Y values
  // be either 0 or the maximum bounds of the edge of the FOV. 
  // ALSO, resolution could be changed dynamically
  // So we send along the minimum and maximum values along X and Y we have seen
  // this allows the Reciever to decide how to map the range of X and Y 
  // more dynamiclly and approprately
  // this will mean that X and Y will need to be mapped based on a range that is 
  // assumed to be changing for each frame. 
  int W_Min; // minimum range of the horizontal, should be >= 0 and <= W
  int H_Min; // minimum range of the vertical, should be >= 0 and <= H
  int W_Max; // minimum range of the horizontal, should be >= 0 and <= W
  int H_Max; // minimum range of the vertical, should be >= 0 and <= H

} EyeGaseStruct;



int period = 1000;
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
        Serial.println("Found my Slave handling input");
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
    Serial.println("done scanning pausing\n");
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
  Wire.requestFrom(ID, 200);    // request some bytes from peripheral device #ID

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

      //moveEyesRandomly = false; // stop random eye movement TODO: Time this to prevent jerking
      //LookAt( &FaceAt );
       
     }
  }
  Serial.printf("What I actually read %s \n",IncomingString);  
  IncomingString.remove(1);
  
  return(ret); 

}


// Once a complete message has been recieved and parced IncomingString will be 
// emptied so the next message can be processed during the next call to user_loop

bool HandleI2CInputOldMayBeBad(int ID) // read from I2C device ID
{
  char c;
  bool ret = true;
  bool EOM = false;


  int min_message_length = sizeof(EyeGaseRangeReportFormat);

  min_message_length += 60; // at time of writing there are 10 parameters in the message so I'm gonna estimate 3 digits for each parameter then x2
  Serial.printf("Requesting %d bytes from slave address %d \n",min_message_length,ID);
  Wire.requestFrom(ID, min_message_length);    // request bytes from peripheral device #ID
  IncomingString.remove(1); // empty the string

  while(Wire.available()>0 && !EOM)    // slave may send less than requested
  {
    IncomingString += (char)Wire.read();

    if(IncomingString.endsWith("\n") || IncomingString.endsWith("\n "))
     {
      EOM = true;
      Serial.print("EOM:");
      Serial.println(IncomingString);

      //EyeGaseStruct gaseRecieved; // struct instance to recieve...
      
      ret = true;
      
      //ParceIncomingString(IncomingString,  &gaseRecieved);
      //moveEyesRandomly = false; // stop random eye movement TODO: Time this to prevent jerking
      //LookAt( &gaseRecieved );
       
     }
  }
  // Serial.print("What I read :");
  // Serial.println(IncomingString);  
  return(ret); 

}

/*
 * This function is called when a full set of co-ordinates have been read in. 
 * The message that comes from the camrea is pretty verbose, makes it handy for debugging
 * it will look something like Face "found = 1 X = 147 Y = 147 W = 240 H = 320\n"
 * it _may_ have leading and trailing spaces or stuff...
 * in the code running on the other microcontroller, or computer or whatever 
 * Simply copy the sprintf fuction from the sener and reverse it with scanf here. 
 * 
 */
void ParceIncomingString( String str, EyeGaseStruct* eye  )
{
  str.trim(); // remove leading and trailing whitespace
  sscanf(str.c_str(),EyeGaseRangeReportFormat, 
                      &eye->IS,
                      &eye->X,
                      &eye->Y,
                      &eye->Blink,
                      &eye->Pupil,
                      &eye->W,
                      &eye->H,
                      &eye->W_Min,
                      &eye->H_Min,
                      &eye->W_Max,
                      &eye->H_Max
                      );
#if defined(DEBUG)
Serial.printf(EyeGaseRangeReportFormat, 
                      eye->IS,
                      eye->X,
                      eye->Y,
                      eye->Blink,
                      eye->Pupil,
                      eye->W,
                      eye->H,
                      eye->W_Min,
                      eye->H_Min,
                      eye->W_Max,
                      eye->H_Max       
                      );
#endif

}

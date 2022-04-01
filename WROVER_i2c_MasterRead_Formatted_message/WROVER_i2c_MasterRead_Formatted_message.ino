// Simple WROVER sketch to help verify ESP32s can talk on the i2c bus and is connected to something
// this sketch assumes you are running the i2c_slave_Writer sample, or a similar sketch on a device
// at SLAVE_ADDRESS  which writes a string to the I2C bus 
// 
// on the M4sk There are a bunch of things on the I2C bus to start with, so a bus scan should show a bunch of things...
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

#if !defined(LED_BUILTIN)
#define LED_BUILTIN 5 // ESP32 LOLIN boards LED builtin is GPIO 5
#endif

#define I2C_BUS1_SDA 2
#define I2C_BUS1_SCL 15

#define I2C_BUS2_SDA 32
#define I2C_BUS2_SCL 33

#define BUS_FREQ 400000


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



int period = 2000;

String IncomingString = String(" ");


void setup() 
{

  
  // put your setup code here, to run once:
  Serial.begin(DEBUG_BAUD);
  Serial.printf("ESP32 about to JOIN I2C bus on SDA %i SCL %i at Freq %i \n",I2C_BUS1_SDA,I2C_BUS1_SCL,BUS_FREQ);

  Serial.printf(" Wire begin returns %i \n", Wire.begin(I2C_BUS1_SDA,I2C_BUS1_SCL) ); // Join the I2C bus as master
  Serial.printf(" Wire setClock returns %i \n",Wire.setClock(BUS_FREQ));
  Serial.println("WRover about to scan I2C bus");

}

void loop() 
{
  //vTaskDelay(period / portTICK_RATE_MS);
  delay(period);
  ScanBus();
  //digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ? LOW : HIGH);
  Serial.printf("Status bump %d %s\n",LED_BUILTIN,digitalRead(LED_BUILTIN) ? "LOW" : "HIGH" );

}

void ScanBus()
{
  byte error;
  int address;
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
        Serial.println("Found my FIRST Slave testing i2c ");
        //HandleI2CInput(SLAVE_ADDRESS);
        //TestI2CFunctions(address);
        //WTFI2CNotString(address);
        WTFI2CNotStringQuiet(address);

      }
      if (address == SLAVE_ADDRESS+1)
      {
        Serial.println("Found my SECOND Slave testing i2c");
        //HandleI2CInput(SLAVE_ADDRESS+1);
        // TestI2CFunctions(address);
        // BetterTestI2CFunctions(address);
        // WTFI2CREAD(address);
        // WTFI2CNotString(address);
        WTFI2CNotStringQuiet(address);
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

bool TestI2CFunctions(int ID)
{
//Wire.requestFrom(ID, 200);    // request some bytes from peripheral device #ID
  int read_bytes = 129;
  Serial.printf("Master request X %d bytes from %d returns : ", read_bytes, ID);
  Serial.println(Wire.requestFrom(ID, read_bytes));
  Serial.flush();
  vTaskDelay(100 / portTICK_RATE_MS);
  
  Serial.print("Wire is available returns :");
  Serial.println(Wire.available());
  Serial.flush();
  vTaskDelay(100 / portTICK_RATE_MS);

  Serial.print("Wire read returns: ");
  Serial.println((char)Wire.read());
  Serial.flush();
  vTaskDelay(100 / portTICK_RATE_MS);

  IncomingString += (char)Wire.read();
  Serial.printf("Wire second Read %s \n",IncomingString);
  Serial.flush();
  vTaskDelay(100 / portTICK_RATE_MS);
  IncomingString.remove(1);
    
  return true;

}

// Better Handel I2C Input
bool WTFI2CNotStringQuiet(int ID)
{
  char c;
  int i = 0;
  bool ret = true;
  int Length = 129;
  char IncomingArray[Length +2];
  memset(IncomingArray, '\0', sizeof(IncomingArray) / sizeof(IncomingArray[0])); // just blast the array. to see why, print the array during the loop without memset... you'll see the array is not fully allocated, and that each new hunk that gets allocated is filled with garbage

  Wire.requestFrom(ID, Length);    // request some bytes from peripheral device #ID
                                // requesting 130 bytes or more causes Wire.read() to return nothing but garbage
  
  Serial.flush(); // I've been getting a lot of output interlacing with my verbose routiens even at 115200 baud
  Serial.printf("\n Reading available %i from slave: %i \n",Wire.available(),ID);
  Serial.flush();

  while(Wire.available()>0 )    // slave may send less than requested
  {
    c = Wire.read();

    if (isAscii(c) )
    {
      IncomingArray[i] = c;
      if (c == '\n'){Serial.println("EOM");}
    }
    else
    {
      if (i < Length )
      {
        IncomingArray[i] = 0;
      }
    }
    i++;
  }
  Serial.printf(" available returned %i and input converted to string is ", Wire.available());
  String Foo = String(IncomingArray);
  Serial.println(Foo);
  return ret;
}

// Better Handel I2C Input
bool WTFI2CNotString(int ID)
{
  char c;
  int i = 0;
  bool ret = true;
  int Length = 129;
  char IncomingArray[Length +2];
  memset(IncomingArray, '\0', sizeof(IncomingArray) / sizeof(IncomingArray[0]));

  Wire.requestFrom(ID, Length);    // request some bytes from peripheral device #ID
                                // requesting 130 bytes or more causes Wire.read() to return nothing but garbage
  

  Serial.print("Reading : ");
  Serial.printf("Wire Availalbe pre loop %i \n", Wire.available());
  while(Wire.available()>0 )    // slave may send less than requested
  {
    c = Wire.read();
    //Serial.printf("Wire says there are %i to read, read carector # %i as %c\n",Wire.available(),i,c);

    if (isAscii(c) )
    {
      IncomingArray[i] = c;
      Serial.printf("\n char %i is %c buf is now : %s :\n",i,c,IncomingArray);
      Serial.flush();
    }
    else
    {
      if (i < Length )
      {
        IncomingArray[i] = 0;
      }
      //Serial.println("\n non ascii char ");
    }
    i++;
  }
  Serial.printf(" available returned %i ", Wire.available());
  String Foo = String(IncomingArray);
  Serial.println(Foo);
  return ret;
}

bool WTFI2CREAD(int ID)
{
  char c;
  int i = 1;
  bool ret = true;
  String IncomingStringLocal = String(" ");

  Wire.requestFrom(ID, 129);    // request some bytes from peripheral device #ID
                                // requesting 130 bytes or more causes Wire.read() to return nothing but garbage
  
  IncomingStringLocal.remove(1); // clear the input before reading
  Serial.print("Reading : ");
  Serial.printf("Wire Availalbe pre loop %i \n", Wire.available());
  while(Wire.available()>0 )    // slave may send less than requested
  {
    c = Wire.read();
    Serial.printf("Wire says there are %i to read, read carector # %i as %c\n",Wire.available(),i++,c);

    if (isAscii(c) )
    {
      IncomingStringLocal += c;
      Serial.printf("\n IncomingString : %s :\n",IncomingStringLocal);
    }
    else
    {
      Serial.println("\n non ascii char ");
    }
  }
  Serial.printf(" available returned %i ", Wire.available());
  return ret;
}



bool BetterTestI2CFunctions(int ID)
{
  char c;
  bool ret = true;
  bool EOM = false;
  Wire.requestFrom(ID, 129);    // request some bytes from peripheral device #ID
  
  IncomingString.remove(1); // clear the input before reading
  Serial.print("Reading : ");
  
  while(Wire.available()>0 && !EOM)    // slave may send less than requested
  {
    c = Wire.read();
    Serial.printf("read c %c\n",c);
    Serial.flush();
    if (isAscii(c) )
    {
      IncomingString += c;
      Serial.printf("\n IncomingString : %s :\n",IncomingString);
    }
    else
    {
      Serial.println("\n non ascii char ");
      //EOM = true;
    }
    
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
  Serial.println("Wire available or EOM");
  Serial.printf("What I actually read %s \n",IncomingString);  
  
  return(ret); 
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

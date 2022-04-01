// Testing I2C client mode using ESP32 LOLIN
// this sketch sends a fully formatted gase message via I2C to code running on monster m4sk

#include <Wire.h>

#define DEBUG_BAUD 115200 // nano does not have much power... 9600 baud is stable

////////// stuff that should be put in a library because it'll be shared by a bunch of different sketches
#define PRIMARY_SLAVE_ADDRESS 0x52
#define SECOND_SLAVE_ADDRESS PRIMARY_SLAVE_ADDRESS + 1 // our address on both I2C busses, so any master knows where to talk to
                           // I borrowed this ID from the Adafruit Arcade library that maps Wii Nunchuck to I2C

#define DEBUG // This is a sample after all library code should have serial.print wrapped in #if defined(DEBUG)

#if !defined(LED_BUILTIN)
#define LED_BUILTIN 5 // ESP32 LOLIN boards LED builtin is GPIO 5
#endif

#define SDA1 SDA
#define SCL1 SCL

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

void setup() 
{
  Serial.begin(DEBUG_BAUD);

  uint8_t slave_addr = PRIMARY_SLAVE_ADDRESS;
  Wire.begin(slave_addr,SDA1,SCL1,400000); // I2C bus 1 setup SLAVE, i2c fast mode


 // setup interrupt response functions to recieve data from M4sk
  // Only the primary monster M4sk should ever send data
  Wire.onReceive(receivePrimaryEyesEvent); // register event for when the master writes

  // setup interrupt response functions to send data to M4Sks 
  // When the ESP32 detects a face or recieves a gase change event from
  // Bluetooth, or Web, or controller or whatever
  Wire.onRequest(WriteToPrimaryEvent); // register event

  Serial.printf("ESP32 SLAVE Startup ready to talk on I2C bus on address %d \n",PRIMARY_SLAVE_ADDRESS);
}


void loop() 
{

  vTaskDelay(period / portTICK_RATE_MS);

  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ? LOW : HIGH);
  Serial.printf("Slave Status bump %d %d %d %s\n",PRIMARY_SLAVE_ADDRESS,SDA,SCL,digitalRead(LED_BUILTIN) ? "LOW" : "HIGH" );
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
  
  //String Msg = String(" Testing TeStInG 1256 \n");
//String Msg = String( " Face found = 1 X = 002 Y = 030 Blink = 400 Pupil = 5000 W = 60 H = 6 W_Min = 8 H_Min = 009 W_Max = 010 H_Max = 110 \n");
  // String Msg = String( " Sent Message: Slave ID "); // SUCCEED
  String Msg = String("abcd efgh ijklmnopqrstuvwxyz1234567890 ABCDEFGHIJKLMNOPQRSTUVWXYZ \n");
  Msg += PRIMARY_SLAVE_ADDRESS;
  Msg += " 1234567";
  //String Msg = String( " Face found = 1 X = 002 1234567 \n"); // FAIL

  Serial.println(Msg.c_str());

  Wire.write(Msg.c_str()); 
}


// Write an Eye change message to primary M4SK
void WriteToPrimaryEventdoesnotwork()
{
  Serial.print("Slave Requested to WRITE on I2C bus : ");
  
  String Msg = String( " Face found = 1 X = 002 Y = 030 Blink = 400 Pupil = 5000 W = 60 H = 6 W_Min = 8 H_Min = 009 W_Max = 010 H_Max = 110\n");
  
  EyeGaseStruct eye;
  // this data will get set somewhee else by the Face AI, or web or bluetooth
  eye.IS       = 001;
  eye.X        = 002;
  eye.Y        = 031;
  eye.Blink    = 401;
  eye.Pupil    = 051;
  eye.W        = 601;
  eye.H        = 001;
  eye.W_Min    = 001;
  eye.H_Min    = 001;
  eye.W_Max    = 001;
  eye.H_Max    = 1000;
  //FormatEyeMessageString( Msg, &eye  );
  Serial.println(Msg);
  Serial.print(" Length : ");
  Serial.println(Msg.length());
  Wire.write(Msg.c_str()); 
}
// point the eyes at a location
void FormatEyeMessageString(String str, EyeGaseStruct* eye  )
{
 char bug[1000];
 // String bug = String(" ");
  sprintf(bug, EyeGaseRangeReportFormat, 
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
  str.remove(1); // just make sure the string is clean
  str.concat(bug);
}

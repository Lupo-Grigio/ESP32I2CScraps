// Simple WROOM sketch to help verify ESP32s can talk on the i2c bus and is connected to something


#include <Wire.h>    //use SCl & SDA

#define bus1_SLAVE_ADDRESS 0x52 // our address on both I2C busses, so any master knows where to talk to
#define BUS2_SLAVE_ADDRESS 0x52 + 1 // our address on both I2C busses, so any master knows where to talk to

#define DEBUG_BAUD 115200
#define DEBUG // This is a sample after all library code should have serial.print wrapped in #if defined(DEBUG)

#if !defined(LED_BUILTIN)
#define LED_BUILTIN 5 // ESP32 LOLIN boards LED builtin is GPIO 5
#endif

//#define I2C_bus2_SDA SDA
//#define I2C_bus2_SCL SCL
// THESE CAN NOT BE #DEFINE otherwise the C++ environment won't map the Wire call's properly
int I2C_bus2_SDA = SDA;
int I2C_bus2_SCL = SCL;

int BUS_FREQ = 400000;



int period = 2000;

String IncomingString = String(" ");


void setup() 
{

  
  Serial.begin(DEBUG_BAUD);
  Serial.printf("ESP32 about to JOIN I2C bus2 on SDA %i SCL %i at Freq %i \n",I2C_bus2_SDA,I2C_bus2_SCL,BUS_FREQ);
  //Wire.begin();
  Serial.printf(" Wire begin bus2 returns %i \n", Wire.begin(I2C_bus2_SDA,I2C_bus2_SCL) ); // Join the I2C bus as master
  Serial.printf(" Wire setClock bus2 returns %i \n",Wire.setClock(BUS_FREQ));
  Serial.println("WRover about to scan I2C bus2 ");

}

void loop() 
{
  //vTaskDelay(period / portTICK_RATE_MS);
  delay(period);
  ScanBus();
  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ? LOW : HIGH);
  Serial.printf("Status bump bus2 %d %s\n",LED_BUILTIN,digitalRead(LED_BUILTIN) ? "LOW" : "HIGH" );

}

void ScanBus()
{
  byte error;
  int address;
  int nDevices;
  Serial.println("Scanning... bus2");
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
      if (address == bus1_SLAVE_ADDRESS)
      {
        Serial.println("Found my FIRST Slave testing i2c ");
 
        //WTFI2CNotString(address);
        WTFI2CNotStringQuiet(address);

      }
      if (address == BUS2_SLAVE_ADDRESS)
      {
        Serial.println("Found my SECOND Slave testing i2c");

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

// Simple WROVER sketch to help verify ESP32s can talk on the i2c bus and is connected to something
// this sketch assumes you are running the i2c_slave_Writer sample, or a similar sketch on a device
// at CLIENT_ADDRESS  which writes a string to the I2C bus 
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



#define DEBUG_BAUD 115200
#define DEBUG // This is a sample after all library code should have serial.print wrapped in #if defined(DEBUG)

#if !defined(LED_BUILTIN)
int LED_BUILTIN = 5;// ESP32 LOLIN boards LED builtin is GPIO 5
#endif

int BUS1_CLIENT_ADDRESS = 0x52; // our address on both I2C busses, so any master knows where to talk to
int I2C_BUS1_SDA = 2;
int I2C_BUS1_SCL = 15;

int BUS2_CLIENT_ADDRESS = BUS1_CLIENT_ADDRESS + 1; // our address on both I2C busses, so any master knows where to talk to
int I2C_BUS2_SDA = 32;
int I2C_BUS2_SCL = 33;

int BUS_FREQ = 400000;




int period = 2000;

String IncomingString = String(" ");


void setup() 
{
  Serial.begin(DEBUG_BAUD);
  //while (!Serial){ delay(10);}
     
  Serial.println("\n I2C Dual Busses Client test starting ");
  
  /* I2C Bus 1 setup */
  Serial.printf(" Beggining Bus1 begin returns %i \n", Wire.begin(BUS1_CLIENT_ADDRESS,I2C_BUS1_SDA,I2C_BUS1_SCL,BUS_FREQ) ); // Join Bus 1 as an I2C SLAVE
  Wire.onReceive(*ReadFromBus1); // register event for when the master on bus 1 wants to write data to me
  Wire.onRequest(*WriteToBus1);     // register interrupt function for when master on bus 1 wants to recieve data from me 
  Serial.printf("2x I2C busses Bus 1 Slave ID %i SCA is on pin %i SDL is on pin %i \n",BUS1_CLIENT_ADDRESS, I2C_BUS1_SDA, I2C_BUS1_SCL);
  Serial.printf(" Wire setClock for bus 1 returns %i \n",Wire.setClock(BUS_FREQ));

  /* BUS2 setup */
 Serial.printf(" Beggining Bus2 begin returns %i \n", Wire1.begin(BUS2_CLIENT_ADDRESS,I2C_BUS2_SDA,I2C_BUS2_SCL,BUS_FREQ) ); // Join Bus 1 as an I2C SLAVE
  Wire1.onReceive(*ReadFromBus2); // register event for when the master on bus 1 wants to write data to me
  Wire1.onRequest(*WriteToBus2);     // register interrupt function for when master on bus 1 wants to recieve data from me 
  Serial.printf("2x I2C busses Bus2 Slave ID is %i SCA is on pin %i SDL is on pin %i \n",BUS2_CLIENT_ADDRESS, I2C_BUS2_SDA, I2C_BUS2_SCL);
  Serial.printf(" Wire setClock for bus2 returns %i \n",Wire1.setClock(BUS_FREQ));


  pinMode(LED_BUILTIN, OUTPUT);  

}

void loop() 
{
  vTaskDelay(period / portTICK_RATE_MS);
  // delay(period);
  
  digitalWrite(LED_BUILTIN, digitalRead(LED_BUILTIN) ? LOW : HIGH);
  Serial.printf("Dual Client Status bump %d %s\n",LED_BUILTIN,digitalRead(LED_BUILTIN) ? "LOW" : "HIGH" );

}

// READ DATA FROM MASTER on BUS1 

void ReadFromBus1(int dummy)
{
  Serial.print("I2C BUS1 requested client to READ from I2C. READING : ");
  
  while(1 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  Serial.println(" : ");
}

// Write to Bus1
void WriteToBus1()
{
  Serial.print("I2C BUS1  Requested client to WRITE to I2C bus. WRITING : ");
  
 
  // String Msg = String( " Sent Message: Slave ID "); // SUCCEED
  String Msg = String("BUS1  efgh ijklmnopqrstuvwxyz1234567890 ABCDEFGHIJKLMNOPQRSTUVWXYZ : ");
  Msg += BUS1_CLIENT_ADDRESS;
  Msg += " : 1234567";

  Serial.print(Msg.c_str());
  Serial.printf(" write returned %i \n",  Wire.write(Msg.c_str()) );
}

/* READ DATA FROM MASTER on BUS2 */

void ReadFromBus2(int dummy)
{
  Serial.print("I2C BUS2 requested client to READ from I2C. READING : ");
  
  while(1 < Wire1.available()) // loop through all but the last
  {
    char c = Wire1.read(); // receive byte as a character
    Serial.print(c);         // print the character
  }
  Serial.println(" : ");
}

// Write to Bus2
void WriteToBus2()
{
  Serial.print("I2C BUS2  Requested client to WRITE to I2C bus. WRITING : ");
  
 
  // String Msg = String( " Sent Message: Slave ID "); // SUCCEED
  String Msg = String("BUS2  efgh ijklmnopqrstuvwxyz1234567890 ABCDEFGHIJKLMNOPQRSTUVWXYZ : ");
  Msg += BUS2_CLIENT_ADDRESS;
  Msg += " : 1234567";

  Serial.print(Msg.c_str());
  Serial.printf(" write returned %i \n",  Wire1.write(Msg.c_str()) );
}

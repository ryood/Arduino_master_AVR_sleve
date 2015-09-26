// Slave1 Sequencer Board
// Slave2 Ext board

#include <Wire.h>
#include <stdio.h>

#define TWI_SLAVE1_ADDRESS 0x7f
#define TWI_SLAVE2_ADDRESS 0x55

byte val1 = 0;
byte val2 = 0;

void setup()
{
  pinMode(13, OUTPUT);
  
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(1);
  digitalWrite(2, HIGH);
  
  Wire.begin();   // join i2c bus (address optional for master)
  
  Serial.begin(9600);  // start serial for output
  Serial.println("Read Slave Data...");
  
  //delay(1000);    // Serial通信の初期化待ち
}

//#define INC_ROTATE_WITHIN(l,h)  (val)=(++val)>(h)?(l):(val)  

void loop()
{
  
  // Slave1 ---------------------------------------------------------------------
  // Reciever
  //
  Wire.requestFrom(TWI_SLAVE1_ADDRESS, 6);    // request 6 bytes from slave device

  Serial.print("Slave1: ");
  
  while (Wire.available())   // slave may send less than requested
  {
    int x = Wire.read(); // receive a byte as character
    
    Serial.print(x);
    Serial.print(" ");
  }
  //Serial.println("");
  //Wire.endTransmission();     // stop transmitting
  
  /*
  // Transmitter
  //
  Wire.beginTransmission(TWI_SLAVE1_ADDRESS); // transmit to slave device
  Wire.write(val1);             // sends value byte  
  Wire.endTransmission();     // stop transmitting
  val1++;
  if (val1 == 8)
    val1 = 0;
  */
  
  // Slave2 ---------------------------------------------------------------------
  // Reciever
  //
  Wire.requestFrom(TWI_SLAVE2_ADDRESS, 8);    // request 8 bytes from slave device

  Serial.print("Slave2: ");
  while (Wire.available())   // slave may send less than requested
  {
    int x = Wire.read(); // receive a byte as character
    
    Serial.print(x);
    Serial.print(" ");
  }
  Serial.println("");
  
  // Transmitter
  //
  Wire.beginTransmission(TWI_SLAVE2_ADDRESS); // transmit to slave device
  Wire.write(val2);             // sends value byte  
  Wire.endTransmission();     // stop transmitting
  val2++;
  if (val2 == 8)
    val2 = 0;
  
  
  // Wait & LED Blink
  //
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
}



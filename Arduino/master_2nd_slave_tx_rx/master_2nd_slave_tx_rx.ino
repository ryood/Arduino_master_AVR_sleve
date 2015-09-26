// I2C Slaveから8byte受信
// I2C Slaveに1byte送信

#include <Wire.h>
#include <stdio.h>

//#define TWI_SLAVE_ADDRESS  0x7f
#define TWI_SLAVE_ADDRESS 0x55

byte val = 0;

void setup()
{
  pinMode(13, OUTPUT);
  
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(1);
  digitalWrite(2, HIGH);
  
  Serial.begin(9600);  // start serial for output
  Serial.println("Read Slave Data...");
  
  delay(1000);    // Serial通信の初期化待ち
  Wire.begin();   // join i2c bus (address optional for master)
}

void loop()
{
  // Reciever
  //
  Wire.requestFrom(TWI_SLAVE_ADDRESS, 8);    // request 8 bytes from slave device

  while (Wire.available())   // slave may send less than requested
  {
    int x = Wire.read(); // receive a byte as character
    
    Serial.print(x);
    Serial.print(" ");
  }
  Serial.println("");
  
  // Transmitter
  //
  Wire.beginTransmission(TWI_SLAVE_ADDRESS); // transmit to slave device
  Wire.write(val);             // sends value byte  
  Wire.endTransmission();     // stop transmitting
  val++;
  if (val == 8)
    val = 0;
  
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
}


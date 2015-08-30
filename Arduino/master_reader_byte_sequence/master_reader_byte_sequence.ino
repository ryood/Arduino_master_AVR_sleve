// I2C Slaveから8byte受け取る

#include <Wire.h>
#include <stdio.h>

void setup()
{
  pinMode(13, OUTPUT);
  
  pinMode(2, OUTPUT);
  digitalWrite(2, LOW);
  delay(1);
  digitalWrite(2, HIGH);
  
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
  Serial.println("Read Slave Data...");
}

void loop()
{
  Wire.requestFrom(0x7f, 8);    // request 8 bytes from slave device 0x7f

  while (Wire.available())   // slave may send less than requested
  {
    int x = Wire.read(); // receive a byte as character
    
    Serial.print(x);
    Serial.print(" ");
  }
  Serial.println("");
  
  digitalWrite(13, HIGH);
  delay(100);
  digitalWrite(13, LOW);
  delay(100);
}


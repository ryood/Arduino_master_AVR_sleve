// Wire Master Reader
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Reads data from an I2C/TWI slave device
// Refer to the "Wire Slave Sender" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


#include <Wire.h>
#include <stdio.h>

void setup()
{
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(9600);  // start serial for output
}

void loop()
{
  Wire.requestFrom(0x7f, 1);    // request 1 bytes from slave device 0x7f

  while (Wire.available())   // slave may send less than requested
  {
    int x = Wire.read(); // receive a byte as character
    
    Serial.println(x);         // print the character
  }

  delay(500);
}

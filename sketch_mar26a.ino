#include <Wire.h>
#include<SHT2x.h>
#include "Adafruit_SI1145.h"

int controlPin = 7;
Adafruit_SI1145 uv = Adafruit_SI1145();

void setup() {
  // put your setup code here, to run once:
  pinMode(controlPin, OUTPUT);
  Wire.begin();
  Serial.begin(9600);
  digitalWrite(controlPin, LOW);
  Serial.println("Adafruit SI1145 test");
 
  if (! uv.begin()) {
    Serial.println("Didn't find Si1145");
    while (1);
  }
 
  Serial.println("OK!");
  }

void loop() {
  // put your main code here, to run repeatedly:


Serial.println("===================");
  Serial.print("Vis: "); Serial.println(uv.readVisible());
  Serial.print("IR: "); Serial.println(uv.readIR());
 
  // Uncomment if you have an IR LED attached to LED pin!
  //Serial.print("Prox: "); Serial.println(uv.readProx());
 
  float UVindex = uv.readUV();
  // the index is multiplied by 100 so to get the
  // integer index, divide by 100!
  UVindex /= 100.0;  
  Serial.print("UV: ");  Serial.println(UVindex);
  Serial.print("Humidity (%RH):");
  Serial.println(SHT2x.GetHumidity());
  Serial.print("Temperature(C):" );
  Serial.println(SHT2x.GetTemperature());


  delay(1000);

}

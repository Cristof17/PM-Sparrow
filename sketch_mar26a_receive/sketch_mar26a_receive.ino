#include <Wire.h>
#include<SHT2x.h>
#include "Adafruit_SI1145.h"
#include "SparrowTransfer.h"

#define ZONA 1
#define ID 1 

struct Data{
  //put your variable definitions here for the data you want to send
  //THIS MUST BE EXACTLY THE SAME ON THE OTHER ARDUINO
  uint8_t zona;
  uint8_t id;
  uint32_t lumina;
  uint32_t ir;
  uint32_t uv;
  uint16_t hum;
  uint16_t temp;
};

void blinkLED() //blinks the LED
{
  digitalWrite(8,LOW);
  delay(200);
  digitalWrite(8,HIGH);  
}

int controlPin = 7;
SparrowTransfer ST; 
Adafruit_SI1145 uv = Adafruit_SI1145();
Data data;

void setup() {
  // put your setup code here, to run once:
  pinMode(controlPin, OUTPUT);
  Wire.begin();
  Serial.begin(9600);
  digitalWrite(controlPin, LOW);
  pinMode(8, OUTPUT);
  
  ST.begin(details(data));

  data.zona = ZONA;
  data.id = ID;
  data.lumina = 0;
  data.ir = 0;
  data.uv = 0;
  data.hum = 0;
  data.temp = 0;
}

void loop() {
  // put your main code here, to run repeatedly:

  if(SUCCESS == (ST.receiveData())){
 
    Serial.print ("Zona:");
    Serial.println(data.zona);
    Serial.print ("ID:");
    Serial.println(data.id);
    Serial.print ("Lumina:");
    Serial.println(data.lumina);
    Serial.print ("IR:");
    Serial.println(data.ir);
    Serial.print ("UV:");
    Serial.println(data.uv);
    Serial.print ("HUM:");
    Serial.println(data.hum);
    Serial.print ("TEMP:");
    Serial.println(data.temp);
    blinkLED();
  }
  
  delay(500);

}



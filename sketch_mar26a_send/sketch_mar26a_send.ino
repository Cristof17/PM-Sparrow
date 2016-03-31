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
  delay(20);
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
  digitalWrite(8, LOW);
  uv.begin();
  
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

  data.lumina = uv.readVisible();
  data.ir = uv.readIR();
 
  // Uncomment if you have an IR LED attached to LED pin!
  //Serial.print("Prox: "); Serial.println(uv.readProx());
 
  data.uv = uv.readUV()/ 100;
  data.hum = SHT2x.GetHumidity();
  data.temp = SHT2x.GetTemperature();
  ST.sendData();
  Serial.print("UV:");
  Serial.println(data.lumina);
  Serial.print("Temp:");
  Serial.println(data.temp);
  blinkLED();
  delay(1000);

}



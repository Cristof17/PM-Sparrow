#include <Wire.h>
#include<SHT2x.h>
#include "Adafruit_SI1145.h"
#include "SparrowTransfer.h"
#include <avr/sleep.h>

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


ISR(TIMER2_COMPA_vect)
{
  //Serial.println("C");
  
}

void goToSleep(){
   //put our Controller in Power Save mode and the external wake up source
   //is going to be Timer2.
   //let's put the controller to sleep to see what happens with the clock
    //Put the transiever to sleep   
    
    //Power-Save mode
   //SM2 SM1 and SM0 bits in SMCR Register select the sleep mode
   SMCR |= (1 << SM1) | (1 << SM0); //power-save
   
   //PRR bits are set to disanble some modules of the microcontroller
   //EXCEPTIONS ARE THE REGISTER FILE AND TRANSIEVER
   
      //TWI must be disabled
      //PRR0 = PRR0 | (1 << PRTWI );
      //ADC must be disabled
      //PRR0 |= (1 << PRADC);
      //SPI must be disabled
      //PRR0 |= (1 << PRSPI);
      //PGA must be disabled
      //PRR0 |= (1 << PRPGA);
      //Analog comparator must be disabled (it is by default disabled in
      // states != Idle, ADCNRM
      //Brown out Detector //din fuse
      //WatchDog Time no .. //din fuse
      //Input Buffers Disabled (I/O Ports)
      //For analog input pins, the digital buffer should be disabled at all times
      //Digital input buffers can be disabled by writing to the Digital input disable 
      //Register DIDR1 and DIDR0
      //DIDR1 |= (1 << AIN1D) | (1 << AIN0D);
      //On Chip debug system
          //Disable OCDEN Fuse
          //Disable JTAGEN Fuse
          //Write 1 to the JTD bit in MCUCR
      //Internal voltage reference
         //SleepEnable bit in SMCR controller
      SMCR |= (1 << SE);

      PORTE |= (1 << PE7);
      //Wake up the trasciever by calling TRXRST in TRXPR reg
   //when interrupt comes in, the microcontroller takes 4 cycles to execute the 
   //routine and then executes the instruction after the SLEEP call

  Serial.println("Sleeping like a baby. I wake up every 2 hours and cry");
  Serial.flush();
}

void sleepTransciever()
{
      //Sleep the transciever PRTRX24 bit in PRR1
      PRR1 |= (1 << PRTRX24);
}

void unSleepTransciever()
{
      //Sleep the transciever PRTRX24 bit in PRR1
      PRR1 &= ~(1 << PRTRX24);
      ST.begin(details(data));
}

void unSleep()
{
  //TWI must be disabled
      //PRR0 = PRR0 & ~(1 << PRTWI );
      //ADC must be disabled
      //PRR0 &= ~(1 << PRADC);
      //SPI must be disabled
      //PRR0 &= ~(1 << PRSPI);
      //PGA must be disabled
      //PRR0 &= ~(1 << PRPGA);
      //Analog comparator must be disabled (it is by default disabled in
      // states != Idle, ADCNRM
      //Brown out Detector //din fuse
      //WatchDog Time no .. //din fuse
      //Input Buffers Disabled (I/O Ports)
      //For analog input pins, the digital buffer should be disabled at all times
      //Digital input buffers can be disabled by writing to the Digital input disable 
      //Register DIDR1 and DIDR0
      //DIDR1 &= ~(1 << AIN1D) & ~(1 << AIN0D);
      //On Chip debug system
          //Disable OCDEN Fuse
          //Disable JTAGEN Fuse
          //Write 1 to the JTD bit in MCUCR
      //Internal voltage reference
            //SleepEnable bit in SMCR controller
      SMCR &= ~(1 << SE);
      PORTE &= ~(1 << PE7);
}

void setTimer2Interrupt()
{


  //INIT
  cli();
  TCCR2A &= ~(1 << OCIE2A) & ~(1 << TOIE2);
  ASSR = 1<<AS2; // enable async 32k Clock
  TCNT2 = 0;
  OCR2A = 249;
  TIMSK2 |= (1 << OCIE2A);
  TIMSK2 |= (1 << TOIE2);
  
  //set CTC mode with top at OCR2A
  TCCR2A |= (1 << WGM21);
  //256 prescaler
  TCCR2A |= (1 << CS02);

  //enable global intrrupts
  sei();
  
}

void setup() {
  // put your setup code here, to run once:
  pinMode(controlPin, OUTPUT);
  digitalWrite(controlPin, LOW);
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);

  Wire.begin();
  Serial.begin(9600);
  uv.begin();
  ST.begin(details(data));

  data.zona = ZONA;
  data.id = ID;
  data.lumina = 0;
  data.ir = 0;
  data.uv = 0;
  data.hum = 0;
  data.temp = 0;

  setTimer2Interrupt();

  //SLEEP SENSORS
  DDRE |= (1 << PE7);
}

void set_PB2()
{
  DDRB |= (1 << PB2);
  PORTB ^= (1 << PB2);
}

int count = 0;//ideea lui Dan

void loop() {
  // put your main code here, to run repeatedly:

  if (count >= 10){
    sleepTransciever();
    goToSleep();
    sleep_cpu();
    sleep_disable();
    unSleep();
    unSleepTransciever();
  }

  count ++;


  blinkLED();

  data.lumina = uv.readVisible();
  data.ir = uv.readIR();
 
  // Uncomment if you have an IR LED attached to LED pin!
  //Serial.print("Prox: "); Serial.println(uv.readProx());

  set_PB2();
 
  data.uv = uv.readUV()/ 100;
  data.hum = SHT2x.GetHumidity();
  data.temp = SHT2x.GetTemperature();
  ST.sendData();
  Serial.print("Light:");
  Serial.println(data.lumina);
  Serial.print("Temp:");
  Serial.println(data.temp);
  Serial.print("Hum:");
  Serial.println(data.hum);
  Serial.print("Lumina:");
  Serial.println(data.temp);    
  Serial.println("Running");
  Serial.flush();

  set_PB2();

 
}


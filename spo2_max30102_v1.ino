/* This code works with MA30102 + 128x32 OLED i2c + Arduino nano
 * It displays average %spo2 on the screen, updates every 1 sec.
 * The firmware is provided as is. Accuracy of the result depends on many factors e.g. the enclosure, finger pressure,
 * excessive ambient light, therefore accuracy is not guaranteed
 */
#include <Adafruit_GFX.h>        //OLED libraries
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255

#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif

int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid
long lastfingerTime=0;
long contfingerDur=0;

//byte pulseLED = 11; //Must be on PWM pin
//byte readLED = 13; //Blinks with each data read

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT,&Wire,OLED_RESET);//Declaring the display name (display)

void setup()
{
  //Serial.begin(9600); // initialize serial communication at 115200 bits per second:

  display.begin(SSD1306_SWITCHCAPVCC, 0x3C); //Start the OLED display
  display.display();
  delay(3000);

  //pinMode(pulseLED, OUTPUT);
  //pinMode(readLED, OUTPUT);
  /*
  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }*/
  particleSensor.begin(Wire, I2C_SPEED_FAST); //Initialize sensor
  //Serial.println(F("Attach sensor to finger with rubber band. Press any key to start conversion"));
  //while (Serial.available() == 0) ; //wait until user presses a key
  //Serial.read();

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}

void loop()
{ 
  bufferLength = 100; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    while(redBuffer[i] < 10000){
      display.clearDisplay();
      display.setTextSize(1);                    
      display.setTextColor(WHITE);             
      display.setCursor(30,5);                
      display.println("Please Place "); 
      display.setCursor(30,15);
      display.println("your finger ");  
      display.display();
      
      particleSensor.nextSample();
      redBuffer[i] = particleSensor.getRed();

      lastfingerTime = 0;contfingerDur=0;
    }

    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    //Serial.print(F("red="));
    //Serial.print(redBuffer[i], DEC);
    //Serial.print(F(", ir="));
    //Serial.println(irBuffer[i], DEC);
    display.clearDisplay();
    display.setTextSize(2);       
    display.setTextColor(WHITE); 
    display.setCursor(30,0);                
    display.println("Wait...");             
    display.display();

    if (lastfingerTime == 0){lastfingerTime = millis();}
    contfingerDur=millis()-lastfingerTime;
  }
  
  //calculate heart rate and SpO2 after first 100 samples (first 4 seconds of samples)
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
  //Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every 1 second
  while (1)
  {  
    if (contfingerDur > 10000 and validSPO2 == 1){ // wait for 10 secs
      display.clearDisplay();
      display.setTextSize(2);//Near it display the average BPM you can display the BPM if you want
      display.setTextColor(WHITE); 
      display.setCursor(50,0);                
      display.println("%SPO2");             
      display.setCursor(50,18);                
      display.println(spo2); 
      display.display();
    }
    else{
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(WHITE); 
      display.setCursor(30,0);                
      display.println("Wait...");             
      display.display();
    }
    //dumping the first 25 sets of samples in the memory and shift the last 75 sets of samples to the top
    for (byte i = 25; i < 100; i++)
    {
      redBuffer[i - 25] = redBuffer[i];
      irBuffer[i - 25] = irBuffer[i];
    }

    //take 25 sets of samples before calculating the heart rate.Should take 1 sec
    for (byte i = 75; i < 100; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      //digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      while(redBuffer[i] < 10000){
        display.clearDisplay();
        display.setTextSize(1);                    
        display.setTextColor(WHITE);             
        display.setCursor(30,5);                
        display.println("Please Place "); 
        display.setCursor(30,15);
        display.println("your finger ");  
        display.display();
        particleSensor.nextSample();
        redBuffer[i] = particleSensor.getRed();
        lastfingerTime = 0;contfingerDur=0;
      }

      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      
    }

    //After gathering 25 new samples recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
  }
}

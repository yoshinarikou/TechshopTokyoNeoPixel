// NeoPixel Ring simple sketch (c) 2013 Shae Erisson
// released under the GPLv3 license to match the rest of the AdaFruit NeoPixel library

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include <Wire.h>

#define BRIGHTNESS 0.5

#define UINT14_MAX        16383
#define UINT14_MAX_HALF        8191
#define MMA8451ADDRESS 0x1C
uint8_t MMA8451_address;
// Which pin on the Arduino is connected to the NeoPixels?
// On a Trinket or Gemma we suggest changing this to 1
#define PIN            6

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS      10

// When we setup the NeoPixel library, we tell it how many pixels, and which pin to use to send signals.
// Note that for older NeoPixel strips you might need to change the third parameter--see the strandtest
// example for more information on possible values.
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

int delayval = 10; // delay for half a second


void I2C_Write_byte(uint8_t reg, uint8_t data){
  
  Wire.beginTransmission(MMA8451_address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t I2C_Read_byte(uint8_t reg){
  uint8_t data;
  Wire.beginTransmission(MMA8451_address);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MMA8451_address, 1);
  while( Wire.available()){
    data = Wire.read();
  }
  return data;
}

int get_acc(uint8_t axis){
  
  uint8_t data[2];
  
  Wire.beginTransmission(MMA8451_address);
  Wire.write(axis);
  Wire.endTransmission(false);
  Wire.requestFrom(MMA8451_address, 2);
  while( Wire.available()){
    data[0] = Wire.read();
    data[1] = Wire.read();
  }
  
  int i_acc = (data[0] << 6) | (data[1] >> 2);

  if (i_acc > UINT14_MAX_HALF){
    i_acc = i_acc - UINT14_MAX;
  }
  return i_acc;

}

bool MMA8451_begin(uint8_t i2caddr)
{
  int ID;
  Wire.begin();
  MMA8451_address = i2caddr;
  I2C_Write_byte(0x2A,0x01);
  
  //Who am I?
  ID = I2C_Read_byte(0x0D);
  Serial.write("ID = ");
  Serial.print(ID);
  Serial.write("\r\n");
  return true;   
}



void setup() {
  // This is for Trinket 5V 16MHz, you can remove these three lines if you are not using a Trinket
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code

  pixels.begin(); // This initializes the NeoPixel library.
  
    Serial.begin(9600);
    Serial.println("setup");

    /* i2c通信初期化(MMA8451と通信するものとする */
    if(!MMA8451_begin(MMA8451ADDRESS))  {
      Serial.println("Couldnt start");
       while (1);
    }
    Serial.println("Start");
}

void loop() {
  float acc[3];

  acc[0] = (get_acc(0x01)/4096.0);
  acc[1] = (get_acc(0x03)/4096.0);
  acc[2] = (get_acc(0x05)/4096.0);
  Serial.write("acc[0] = ");
  Serial.print(acc[0]);
  Serial.write(", acc[1] = ");
  Serial.print(acc[1]);
  Serial.write(", acc[2] = ");
  Serial.print(acc[2]);
  Serial.write("\r\n");

int data[3];

data[0]= BRIGHTNESS*(125 + acc[0]*120);
data[1]= BRIGHTNESS*(125 + acc[1]*120);
data[2]= BRIGHTNESS*(125 + acc[2]*120);

  Serial.write("data[0] = ");
  Serial.print(data[0]);
  Serial.write(", data[1] = ");
  Serial.print(data[1]);
  Serial.write(", data[2] = ");
  Serial.print(data[2]);
  Serial.write("\r\n");
  
  // For a set of NeoPixels the first NeoPixel is 0, second is 1, all the way up to the count of pixels minus one.

  for(int i=0;i<NUMPIXELS;i++){

    // pixels.Color takes RGB values, from 0,0,0 up to 255,255,255
    pixels.setPixelColor(i, pixels.Color(data[0],data[1],data[2])); // Moderately bright green color.

    pixels.show(); // This sends the updated pixel color to the hardware.

    delay(delayval); // Delay for a period of time (in milliseconds).

  }
}

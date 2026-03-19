#include <Arduino.h>

#include "Adafruit_MPR121.h"

#include "LCD_Driver.h"
#include "Touch_Driver.h"
#include "GUI_Paint.h"

#include <Wire.h>
#include <Audio.h>

// GUItool: begin automatically generated code
AudioSynthWaveformSine   sine1;          //xy=720.2000427246094,508.20001220703125
AudioOutputI2S           i2s;           //xy=880.2000732421875,499.20001220703125
AudioConnection          patchCord1(sine1, 0, i2s, 0);
AudioConnection          patchCord2(sine1, 0, i2s, 1);
// GUItool: end automatically generated code


Adafruit_MPR121 cap5A = Adafruit_MPR121();
Adafruit_MPR121 cap5C = Adafruit_MPR121();

const uint8_t LED_GPIO_BIT = 0b11111100;

uint16_t lasttouched5A = 0;
uint16_t currtouched5A = 0;
uint16_t lasttouched5C = 0;
uint16_t currtouched5C = 0;
UWORD x, y = 0;
// MPR121 GPIO: pin 6 as LED output, pin 0 as touch input
const uint8_t MPR121_LED_PIN = 6;  // MPR121 GPIO pin 6 for LED
unsigned long lastSampleTime = 0;
const unsigned long SAMPLE_INTERVAL_MS = 500;

void Touch_INT_callback() {
  XY = Touch_1IN28_Get_Point();

        x = XY.x_point;
        y = XY.y_point;
        Serial.print("touched at: ");
        Serial.print(x);
        Serial.print(',');
        Serial.print(y);
        Serial.println();
}

void setup() {
  Serial.begin(9600);

  if (!cap5A.begin(0x5A)) {
    Serial.println("MPR121@0x5A not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121@0x5A found!");
    if (!cap5C.begin(0x5C,&Wire,12,6,6)) {
    Serial.println("MPR121@0x5C not found, check wiring?");
      // enable only the GPIO line we want

    while (1);
  }
  Serial.println("MPR121@0x5C found!");
  // Electrode Configuration Register, see datasheet page 15
  cap5C.writeRegister(MPR121_ECR, 0b00000000); //reset ECR
  //delay(100);

  cap5C.writeRegister(MPR121_ECR, 0b00000110);
  //first two bits set calibration mode
  //second two bytes set proximity detection
  //last four bits

    cap5C.writeRegister(MPR121_GPIOEN,   LED_GPIO_BIT);

  cap5C.writeRegister(MPR121_GPIODIR,  LED_GPIO_BIT);

  cap5C.writeRegister(MPR121_GPIOCTL0,  LED_GPIO_BIT);
  cap5C.writeRegister(MPR121_GPIOCTL1,  LED_GPIO_BIT);


  cap5C.writeRegister(MPR121_GPIOCLR,  LED_GPIO_BIT);
  Serial.println("MPR121@0x5C set!");
cap5C.writeRegister(MPR121_GPIOTOGGLE,  LED_GPIO_BIT);
delay(500);
cap5C.writeRegister(MPR121_GPIOTOGGLE,  LED_GPIO_BIT);

  Touch_1IN28_XY XY;
  XY.mode = 1;
  Config_Init();
  LCD_Init();
  if (Touch_1IN28_init(XY.mode) == true)
    Serial.println("Touchscreen OK!");
  else
    Serial.println("Touchscreen NO!");

  LCD_SetBacklight(255);//0-255
  Paint_NewImage(LCD_WIDTH, LCD_HEIGHT, 0, BLACK);
  Paint_Clear(WHITE);
  //attachInterrupt(digitalPinToInterrupt(TP_INT_PIN), Touch_INT_callback, LOW);
  pinMode(TP_INT_PIN, INPUT_PULLUP);
  Paint_DrawString_EN(35, 90, "OMNIPHONE!", &Font20, BLACK, WHITE);


  AudioMemory(15);
  sine1.frequency(111);
  sine1.amplitude(0.5);
}

void loop() {
  // Sample MPR121 electrode 0 and control LED on GPIO pin 6 every 100ms
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = millis();

    cap5C.writeRegister(MPR121_ECR, 0b00000110);

    cap5C.writeRegister(MPR121_GPIOTOGGLE,  LED_GPIO_BIT);
    Serial.print(lastSampleTime);
    for (int i = 0; i < 6; i++) {
      Serial.print(" ");
      Serial.print(cap5C.filteredData(i));
    
    }
    Serial.println("");

    // Get the currently touched pads
    currtouched5C = cap5C.touched();
    currtouched5A = cap5A.touched();


    lasttouched5A = currtouched5A;
    lasttouched5C = currtouched5C;
  }
}


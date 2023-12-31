#include <Arduino.h>

#include "Adafruit_MPR121.h"

#include "1.28inch_Touch_LCD/LCD_Driver.h"
#include "1.28inch_Touch_LCD/Touch_Driver.h"
#include "1.28inch_Touch_LCD/GUI_Paint.h"

#include <Wire.h>
#include <Audio.h>

// GUItool: begin automatically generated code
AudioSynthWaveformSine   sine1;          //xy=720.2000427246094,508.20001220703125
AudioOutputI2S           i2s;           //xy=880.2000732421875,499.20001220703125
AudioConnection          patchCord1(sine1, 0, i2s, 0);
AudioConnection          patchCord2(sine1, 0, i2s, 1);
// GUItool: end automatically generated code


Adafruit_MPR121 cap5B = Adafruit_MPR121();
Adafruit_MPR121 cap5D = Adafruit_MPR121();

uint16_t lasttouched5B = 0;
uint16_t currtouched5B = 0;
uint16_t lasttouched5D = 0;
uint16_t currtouched5D = 0;
UWORD x, y = 0;

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

  if (!cap5B.begin(0x5B)) {
    Serial.println("MPR121@0x5B not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121@0x5B found!");
    if (!cap5D.begin(0x5D)) {
    Serial.println("MPR121@0x5D not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121@0x5D found!");

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
  attachInterrupt(digitalPinToInterrupt(TP_INT_PIN), Touch_INT_callback, LOW);
  pinMode(TP_INT_PIN, INPUT_PULLUP);
  Paint_DrawString_EN(35, 90, "OMNIPHONE!", &Font20, BLACK, WHITE);


  AudioMemory(15);
  sine1.frequency(111);
  sine1.amplitude(0.5);
}

void loop() {
// Get the currently touched pads
  currtouched5D = cap5D.touched();
  currtouched5B = cap5B.touched();
  
  for (uint8_t i=0; i<12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched5B & _BV(i)) && !(lasttouched5B & _BV(i)) ) {
      Serial.print(i); Serial.println(" 5B touched");
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched5B & _BV(i)) && (lasttouched5B & _BV(i)) ) {
      Serial.print(i); Serial.println(" 5B released");
    }

        if ((currtouched5D & _BV(i)) && !(lasttouched5D & _BV(i)) ) {
      Serial.print(i); Serial.println(" 5D touched");
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched5D & _BV(i)) && (lasttouched5D & _BV(i)) ) {
      Serial.print(i); Serial.println(" 5D released");
    }
  }
  lasttouched5B = currtouched5B;
  lasttouched5D = currtouched5D;

    

  delay(100);           
}


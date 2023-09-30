#include <Arduino.h>
#include "Adafruit_MPR121.h"
#include "1.28inch_Touch_LCD/LCD_Driver.h"
#include "1.28inch_Touch_LCD/Touch_Driver.h"
#include "1.28inch_Touch_LCD/GUI_Paint.h"



Adafruit_MPR121 cap = Adafruit_MPR121();

uint16_t lasttouched = 0;
uint16_t currtouched = 0;
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

  if (!cap.begin(0x5B)) {
    Serial.println("MPR121@0x5B not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121@0x5B found!");

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



}

void loop() {
// Get the currently touched pads
  currtouched = cap.touched();
  
  for (uint8_t i=0; i<12; i++) {
    // it if *is* touched and *wasnt* touched before, alert!
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" touched");
    }
    // if it *was* touched and now *isnt*, alert!
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print(i); Serial.println(" released");
    }
  }
  lasttouched = currtouched;

    

  delay(100);           
}


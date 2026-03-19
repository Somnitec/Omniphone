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
const unsigned long SAMPLE_INTERVAL_MS = 10;

unsigned long lastBlinkTime = 0;
const unsigned long BLINK_INTERVAL_MS = 500;

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
void writeAllCapsense(uint8_t reg, uint8_t value){
  cap5A.writeRegister(reg, value);
  //cap5B.writeRegister(reg, value);
  cap5C.writeRegister(reg, value);
  //cap5D.writeRegister(reg, value);
}
void setup() {
  Serial.begin(9600);

  if (!cap5A.begin(0x5B,&Wire,12,6,true, 6)) {
    Serial.println("MPR121@0x5A not found, check wiring?");
    while (1);
  }
  Serial.println("MPR121@0x5A found!");
    if (!cap5C.begin(0x5D,&Wire,12,6,true,6)) {
    Serial.println("MPR121@0x5C not found, check wiring?");
      // enable only the GPIO line we want

    while (1);
  }
  Serial.println("MPR121@0x5C found!");
  // Electrode Configuration Register, see datasheet page 15
  writeAllCapsense(MPR121_ECR, 0b00000000); //reset ECR

  writeAllCapsense(MPR121_CONFIG1, 0b00000001);
//High Sensitivity: (FFI = 6 samples, lowest; CDC=16μA—adjust this if you get 0 or 1023 in filtered data)
  writeAllCapsense(MPR121_CONFIG2, 0b00100000);
  //Fastest Sampling: (CDT= 0.5us, SFI=4, ESI=1ms)



  writeAllCapsense(MPR121_GPIOEN,   LED_GPIO_BIT);
  writeAllCapsense(MPR121_GPIODIR,  LED_GPIO_BIT);
  writeAllCapsense(MPR121_GPIOCTL0,  0);
  writeAllCapsense(MPR121_GPIOCTL1,  LED_GPIO_BIT);



  writeAllCapsense(MPR121_ECR, 0b11000110);
  //first two bits set calibration mode = 11 (On start, it copies the exact current reading to the baseline.	Best for Instruments. Gives you a clean, precise "zero" the moment you power on.)(Baseline tracking and initialize enable. At the first {ESI x SFI}, MPR121 copy the 2nd filter output to 10bit baseline value. Subsequent update is per nominal baseline filter operation)
  //second two bytes set proximity detection = 00 (disable proximity, we just want touch)
  //last four bits = 0110 to enable electrodes 0,1,2,3,4,5 (the ones we have wired up)

  writeAllCapsense(MPR121_GPIOCLR,  LED_GPIO_BIT);

  Serial.println("capsense set!");



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
  sine1.frequency(200);
  sine1.amplitude(0.5);
}

void loop() {
   if (millis() - lastBlinkTime >= BLINK_INTERVAL_MS) {
    lastBlinkTime = millis();
    //writeAllCapsense(MPR121_GPIOTOGGLE,  LED_GPIO_BIT);
    uint8_t pwm0 = 4; // 0..15
uint8_t pwm1 = 10;
cap5C.writeRegister(MPR121_PWM0, (pwm1 << 4) | pwm0);  // 0x81: PWM1/PWM0
uint8_t pwm2 = 7;
uint8_t pwm3 = 7;
cap5C.writeRegister(MPR121_PWM1, (pwm3 << 4) | pwm2);  // 0x82: PWM3/PWM2
  }
// Sample MPR121 electrode 0 and control LED on GPIO pin 6 every 100ms
 
  if (millis() - lastSampleTime >= SAMPLE_INTERVAL_MS) {
    lastSampleTime = millis();


    //Serial.print(lastSampleTime);
    for (int i = 0; i < 6; i++) {
      Serial.print(" ");
      Serial.print(cap5C.filteredData(i));
    
    }
    for (int i = 0; i < 6; i++) {
      Serial.print(" ");
      Serial.print(cap5A.filteredData(i));
    
    }
    Serial.println("");

    // Get the currently touched pads
    currtouched5C = cap5C.touched();
    currtouched5A = cap5A.touched();


    lasttouched5A = currtouched5A;
    lasttouched5C = currtouched5C;
  }
}


/* blink‑gpio2‑noplus.ino
   Blink the LED connected to MPR121 pin‑14 (ELE6 / GPIO2).
   See SparkFun MPR121.pdf §9 and AN3894 §4.2.
*/

#include <Arduino.h>
#include <Wire.h>

#define MPR121_ADDR     0x5C   // address of the 5C device
// useful register addresses from the datasheet
#define ECR         0x5E   // electrode configuration / run

#define GPIOCTL0 0x73
#define GPIOCTL1 0x74
#define GPIODATA     0x75
#define GPIODIR     0x76
#define GPIOEN      0x77
#define GPIOSET     0x78
#define GPIOCLR     0x79
#define GPIOTOGGLE  0x7A

#define PWM0     0x81
#define PWM1     0x82
#define PWM2     0x83
#define PWM3     0x84


// bit number for the LED you want to drive

const uint8_t LED_GPIO_BIT = 0b11111100;

static uint8_t i2cRead(uint8_t reg) {
  Wire.beginTransmission(MPR121_ADDR);
  Wire.write(reg);
  Wire.endTransmission();
  Wire.requestFrom(MPR121_ADDR, (uint8_t)1);
  if (Wire.available()) return Wire.read();
  return 0;
}

static void i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPR121_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void setup() {
  Serial.begin(115200);
  while (!Serial) { }              // wait for USB on Teensy

  Wire.begin();                    // start I²C

  // Electrode Configuration Register, see datasheet page 15
  i2cWrite(ECR, 0b00000000); //reset ECR
  //delay(100);

  i2cWrite(ECR, 0b00000110);
  //first two bits set calibration mode
  //second two bytes set proximity detection
  //last four bits


  // enable only the GPIO line we want
  i2cWrite(GPIOEN,   LED_GPIO_BIT);

  i2cWrite(GPIODIR,  LED_GPIO_BIT);

  i2cWrite(GPIOCTL0,  LED_GPIO_BIT);
  i2cWrite(GPIOCTL1,  LED_GPIO_BIT);


  i2cWrite(GPIOCLR,  LED_GPIO_BIT);
  Serial.println("MPR121 started allegedly");
}

void loop() {

  i2cWrite(GPIOTOGGLE,  LED_GPIO_BIT);
  //  i2cWrite(PWM0,  0b11111111);
  //  i2cWrite(PWM1,  0b11111111);
  //  i2cWrite(PWM2,  0b11111111);
  //  i2cWrite(PWM3,  0b11111111);

  //i2cWrite(GPIOTOGGLE,  0b10000000);

  Serial.println("blink!");
  delay(500);
  //i2cWrite(GPIOTOGGLE,  0b00000100);
  //  Serial.println("blonk!");
  //  delay(500);

  //
  //  i2cWrite(GPIOTOGGLE,  0b00000000);
  //  i2cWrite(PWM0, 0b00000000);
  //  i2cWrite(PWM1, 0b00000000);
  //  i2cWrite(PWM2, 0b00000000);
  //  i2cWrite(PWM3, 0b00000000);
  //  Serial.println("off!");
  //  delay(500);
}

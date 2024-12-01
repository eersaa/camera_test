#include <Arduino.h>
// Camera test for Pico RP2040 + ST7789 240x240 display.
// Eventually this might get merged into existing cameratest sketch
// so the same sketch compiles for either SAMD51 or RP2040, but at
// the moment this diverges a lot (different display, etc.), maybe
// later when we have solid RP2040 Feather support, a good camera
// adapter PCB and some other things firm up.
// REQUIRES EARLE PHILHOWER RP2040 BOARD PACKAGE - Arduino mbed
// one won't work.

#include <Wire.h>            // I2C comm to camera
#include "Adafruit_OV7670.h" // Camera library

// CAMERA CONFIG -----------------------------------------------------------

#if defined(PICO_SDK_VERSION_MAJOR)
#include "hardware/spi.h"

OV7670_arch arch;
OV7670_pins pins = {
  .enable = -1, // Also called PWDN, or set to -1 and tie to GND
  .reset  = 14, // Cam reset, or set to -1 and tie to 3.3V
  .xclk   = 13, // MCU clock out / cam clock in
  .pclk   = 10, // Cam clock out / MCU clock in
  .vsync  = 11, // Also called DEN1
  .hsync  = 12, // Also called DEN2
  .data   = {2, 3, 4, 5, 6, 7, 8, 9}, // Camera parallel data out
  .sda    = 20, // I2C data
  .scl    = 21, // I2C clock
};

#define CAM_I2C Wire

#endif

#define CAM_SIZE OV7670_SIZE_DIV8     ///< 80 x 60
#define CAM_MODE OV7670_COLOR_RGB // RGB plz

Adafruit_OV7670 cam(OV7670_ADDR, &pins, &CAM_I2C, &arch);
// Adafruit_OV7670 cam(0x30, &pins, &CAM_I2C, &arch); // OIV2640 WIP

const char charmap[] = " .:-=+*#%@";

void printAsciiArt() {
  // Pause the camera DMA - hold buffer steady to avoid tearing
  cam.suspend();

  // Take only one channel and output it as ASCII art
  for (uint16_t y = 0; y < cam.height(); y++) {
    char row[cam.width() + 1]; // +1 for the null terminator
    for (uint16_t x = 0; x < cam.width(); x++) {
      uint16_t pixel = cam.getBuffer()[y * cam.width() + x];
      uint8_t gray = (pixel >> 8) & 0xFF;
      gray /= 26; // Scale down to 0-9
      row[x] = charmap[gray];
    }
    row[cam.width()] = '\0'; // Null terminator for the string
    Serial.println(row);
  }

  // Resume camera DMA - release buffer to start next frame
  cam.resume(); // Resume DMA to camera buffer
}

// SETUP - RUNS ONCE ON STARTUP --------------------------------------------

void setup(void) {
  Serial.begin(115200);
  //while(!Serial);
  delay(1000);
  Serial.println(F("Hello! Camera Test"));

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  // These are currently RP2040 Philhower-specific
  Wire.setSDA(pins.sda); // I2C0
  Wire.setSCL(pins.scl);

  // Once started, the camera continually fills a frame buffer
  // automagically; no need to request a frame.
  OV7670_status status = cam.begin(CAM_MODE, CAM_SIZE, 5.0);
  if (status != OV7670_STATUS_OK) {
    Serial.println("Camera begin() fail");
    for(;;);
  }

#if 0
// OV2640 WIP
  cam.writeRegister(0xFF, 0x01);
  uint8_t pid = cam.readRegister(0x0A); // Should be 0x26
  uint8_t ver = cam.readRegister(0x0B); // Should be 0x41
  Serial.println(pid, HEX);
  Serial.println(ver, HEX);
#else
  uint8_t pid = cam.readRegister(OV7670_REG_PID); // Should be 0x76
  uint8_t ver = cam.readRegister(OV7670_REG_VER); // Should be 0x73
  Serial.println(pid, HEX);
  Serial.println(ver, HEX);
#endif
//  cam.test_pattern(OV7670_TEST_PATTERN_COLOR_BAR);
}

// MAIN LOOP - RUNS REPEATEDLY UNTIL RESET OR POWER OFF --------------------

void loop() {

  // This was for empirically testing window settings in src/arch/ov7670.c.
  // Your code doesn't need this. Just keeping around for future reference.
  if(Serial.available()) {
    uint32_t vstart = Serial.parseInt();
    uint32_t hstart = Serial.parseInt();
    uint32_t edge_offset = Serial.parseInt();
    uint32_t pclk_delay = Serial.parseInt();
    while(Serial.read() >= 0); // Delete line ending or other cruft
    OV7670_frame_control((void *)&cam, CAM_SIZE, vstart, hstart,
                         edge_offset, pclk_delay);
  }


  // Pause the camera DMA - hold buffer steady to avoid tearing
//  cam.suspend();

  //cam.capture(); // Manual capture instead of PIO DMA

  // Postprocessing effects. These modify a previously-captured
  // image in memory, they are NOT in-camera effects.
  // Most only work in RGB mode (not YUV).
  //cam.image_negative();
  //cam.image_threshold(150);
  //cam.image_posterize(5);  // # of levels
  //cam.image_mosaic(21, 9); // Tile width, height
  //cam.image_median();
  //cam.image_edges(4);      // 0-31, smaller = more edges

  printAsciiArt();

//  cam.resume(); // Resume DMA to camera buffer
}

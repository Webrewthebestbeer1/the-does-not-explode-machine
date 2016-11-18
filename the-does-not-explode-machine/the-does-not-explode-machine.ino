#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX_AS.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>
#include <MapleCoOS116.h>
#include "TMRpcm.h"
#include "SDFat.h"


#define SWITCH 11
#define LED_POWER 12
#define LED_RUNNING 13
#define ONE_WIRE_BUS PB10
#define SSR PC13
#define SPEAKER 10
#define MAGIC 80.5

#define tft_CS     PA4
#define tft_RST    0
#define tft_DC     PB0
#define SD_CS      PB12

Adafruit_ST7735 tft = Adafruit_ST7735(tft_CS,  tft_DC, tft_RST);
SdFat sd(2);

volatile int encoderState;
volatile int encoderFlag;
boolean A_set;
boolean B_set;
volatile int16_t encoderPos;
volatile int  encoderTimer = millis(); // acceleration measurement
int encoderPinA = PB4; // pin array of all encoder A inputs
int encoderPinB = PB5; // pin array of all encoder B inputs
unsigned int lastEncoderPos;

// timer
#define ENCODER_RATE 1000    // in microseconds;
#define ENCODER_ACC 1
HardwareTimer timer(1);

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

char name[13];

OS_STK   vCubeLoopStk[TASK_STK_SIZE];
OS_STK   vUpdateSSRStk[TASK_STK_SIZE];
OS_STK   vUpdateTempStk[TASK_STK_SIZE];

unsigned long activatedTime, deactivatedTime = millis();
float temp1, temp2, oldTemp1;
bool plinged = false;
int SSR_FREQUENCY = 3000; // in ms

bool sdworked = false;

const float sin_d[] = {
  0, 0.17, 0.34, 0.5, 0.64, 0.77, 0.87, 0.94, 0.98, 1, 0.98, 0.94,
  0.87, 0.77, 0.64, 0.5, 0.34, 0.17, 0, -0.17, -0.34, -0.5, -0.64,
  -0.77, -0.87, -0.94, -0.98, -1, -0.98, -0.94, -0.87, -0.77,
  -0.64, -0.5, -0.34, -0.17
};
const float cos_d[] = {
  1, 0.98, 0.94, 0.87, 0.77, 0.64, 0.5, 0.34, 0.17, 0, -0.17, -0.34,
  -0.5, -0.64, -0.77, -0.87, -0.94, -0.98, -1, -0.98, -0.94, -0.87,
  -0.77, -0.64, -0.5, -0.34, -0.17, 0, 0.17, 0.34, 0.5, 0.64, 0.77,
  0.87, 0.94, 0.98
};
const float d = 5;
float cube1_px[] = {
  -d,  d,  d, -d, -d,  d,  d, -d
};
float cube1_py[] = {
  -d, -d,  d,  d, -d, -d,  d,  d
};
float cube1_pz[] = {
  -d, -d, -d, -d,  d,  d,  d,  d
};

float cube1_p2x[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};
float cube1_p2y[] = {
  0, 0, 0, 0, 0, 0, 0, 0
};

int cube1_r[] = {
  0, 0, 0
};

uint16 cube1_x, cube1_y, cube1_color;

void setup() {
  /*
    pinMode(SWITCH, INPUT);
    pinMode(SPEAKER, OUTPUT);
    pinMode(LED_POWER, OUTPUT);
    pinMode(LED_RUNNING, OUTPUT);
  */
  pinMode(SSR, OUTPUT);
  sensors.begin();
  sensors.setResolution(12);
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(50, 10);
  tft.setTextSize(3);
  tft.setTextWrap(true);
  Serial.begin(9600);
  initEncoder();
  cube1_x = ((tft.width()) / 2);
  cube1_y = ((tft.height()) / 2);
  cube1_color = ST7735_RED;
  tft.setCursor(120,100),
  tft.setTextColor(ST7735_WHITE);
  tft.setTextSize(2);
  if (sd.begin(SD_CS, SD_SCK_MHZ(18))) {
    tft.print("Y");
    sd.ls();
    Serial.println("SD working");
    sdworked = true;
    bmpDraw("lily128.bmp",0,0);
  } else {
    Serial.println("SD not working");
    tft.print("N");
  }
  setupCo();
}

void setupCo() {
  CoInitOS();
  /*
  CoCreateTask(vCubeLoopTask,
               (void *) 0,
               2,
               &vCubeLoopStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );
              */
  CoCreateTask(vUpdateSSRStateTask,
               (void *) 0,
               2,
               &vUpdateSSRStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );
  CoCreateTask(vUpdateTempTask,
               (void *) 0,
               2,
               &vUpdateTempStk[TASK_STK_SIZE - 1],
               TASK_STK_SIZE
              );
  CoStartOS();
  TMRpcm_play(name);
}

void loop() {
  if (lastEncoderPos != encoderPos) {
    /*
      tft.fillScreen(ST7735_RED);
      tft.setCursor(50,10);
      tft.setTextColor(ST7735_WHITE);
      tft.print((char)encoderPos, DEC);
    */
    encoderFlag = LOW;
    lastEncoderPos = encoderPos;
  }
}

static void vCubeLoopTask(void *pdata) {
  while (1) {
    Serial.println((sdworked) ? "yes" : "no");
    cube(cube1_px, cube1_py, cube1_pz, cube1_p2x, cube1_p2y, cube1_r, &cube1_x, &cube1_y, &cube1_color);
    if (oldTemp1 != temp1) {
      tft.setCursor(50, 10);
      tft.setTextSize(3);
      tft.setTextColor(ST7735_BLACK);
      tft.print((char)oldTemp1, DEC);
      tft.setCursor(50, 10);
      tft.setTextColor(ST7735_WHITE);
      tft.print((char)temp1, DEC);
      oldTemp1 = temp1;
    }
    CoTickDelay(205 - 2 * encoderPos);
  }
}

static void vUpdateTempTask(void *pdata) {
  while(1) {
    sensors.requestTemperatures();
    temp1 = sensors.getTempCByIndex(0);
    temp2 = sensors.getTempCByIndex(1);
    CoTickDelay(1000);
  }
}

static void vUpdateSSRStateTask(void *pdata) {
  while (1) {
    updateSSRState();
  }
}

void playPling() {
  // play an a for 200ms
  //tone(SPEAKER, 440, 200);
}

/*
   Slow PWM type control of the SSR
*/
void updateSSRState() {
  float dutyCycle = (float)encoderPos / 100.;
  unsigned long onTime = (float)SSR_FREQUENCY * dutyCycle;
  unsigned long offTime = SSR_FREQUENCY - onTime;
  if (digitalRead(SSR)) {
    if (encoderPos == 100) return;
    if (activatedTime + onTime <= millis()) {
      digitalWrite(SSR, LOW);
      deactivatedTime = millis();
    }
  } else {
    if (encoderPos == 0) return;
    if (deactivatedTime + offTime <= millis()) {
      digitalWrite(SSR, HIGH);
      activatedTime = millis();
    }
  }
}

void incrementEncoder(uint8_t increment) {
  int8_t newEncoderPos = encoderPos + increment;
  if (newEncoderPos > 100) newEncoderPos = 100;
  if (newEncoderPos < 0) newEncoderPos = 0;
  encoderPos = newEncoderPos;
}

void cube(float *px, float *py, float *pz, float *p2x, float *p2y, int *r, uint16 *x, uint16 *y, uint16 *color) {
  for (int i = 0; i < 3; i++) {
    tft.drawLine(p2x[i], p2y[i], p2x[i + 1], p2y[i + 1], ST7735_BLACK);
    tft.drawLine(p2x[i + 4], p2y[i + 4], p2x[i + 5], p2y[i + 5], ST7735_BLACK);
    tft.drawLine(p2x[i], p2y[i], p2x[i + 4], p2y[i + 4], ST7735_BLACK);
  }

  tft.drawLine(p2x[3], p2y[3], p2x[0], p2y[0], ST7735_BLACK);
  tft.drawLine(p2x[7], p2y[7], p2x[4], p2y[4], ST7735_BLACK);
  tft.drawLine(p2x[3], p2y[3], p2x[7], p2y[7], ST7735_BLACK);

  r[0] = r[0] + 1;
  r[1] = r[1] + 1;
  if (r[0] == 36) r[0] = 0;
  if (r[1] == 36) r[1] = 0;
  if (r[2] == 36) r[2] = 0;

  for (int i = 0; i < 8; i++) {
    float px2 = px[i];
    float py2 = cos_d[r[0]] * py[i] - sin_d[r[0]] * pz[i];
    float pz2 = sin_d[r[0]] * py[i] + cos_d[r[0]] * pz[i];

    float px3 = cos_d[r[1]] * px2 + sin_d[r[1]] * pz2;
    float py3 = py2;
    float pz3 = -sin_d[r[1]] * px2 + cos_d[r[1]] * pz2;

    float ax = cos_d[r[2]] * px3 - sin_d[r[2]] * py3;
    float ay = sin_d[r[2]] * px3 + cos_d[r[2]] * py3;
    float az = pz3 - 190;

    p2x[i] = *x + ax * 500 / az;
    p2y[i] = *y + ay * 500 / az;
  }

  for (int i = 0; i < 3; i++) {
    tft.drawLine(p2x[i], p2y[i], p2x[i + 1], p2y[i + 1], *color);
    tft.drawLine(p2x[i + 4], p2y[i + 4], p2x[i + 5], p2y[i + 5], *color);
    tft.drawLine(p2x[i], p2y[i], p2x[i + 4], p2y[i + 4], *color);
  }

  tft.drawLine(p2x[3], p2y[3], p2x[0], p2y[0], *color);
  tft.drawLine(p2x[7], p2y[7], p2x[4], p2y[4], *color);
  tft.drawLine(p2x[3], p2y[3], p2x[7], p2y[7], *color);
}

void readEncoder() {
  if ((gpio_read_bit(PIN_MAP[encoderPinA].gpio_device, PIN_MAP[encoderPinA].gpio_bit) ? HIGH : LOW) != A_set) {
    A_set = !A_set;
    if (A_set && !B_set) {
      incrementEncoder(1);
    }
    encoderTimer = millis();
  }
  if ((gpio_read_bit(PIN_MAP[encoderPinB].gpio_device, PIN_MAP[encoderPinB].gpio_bit) ? HIGH : LOW) != B_set) {
    B_set = !B_set;
    if (B_set && !A_set)
      incrementEncoder(-1);
    encoderTimer = millis();
  }
}

void initEncoder() {
  encoderTimer = millis(); // acceleration measurement
  encoderState = HIGH;
  encoderFlag = HIGH;
  A_set = false;
  B_set = false;
  encoderPos = 0;
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  lastEncoderPos = 1;

  // timer setup for encoder
  timer.pause();
  timer.setPeriod(ENCODER_RATE); // in microseconds
  timer.setChannel1Mode(TIMER_OUTPUT_COMPARE);
  timer.setCompare(TIMER_CH1, 1);  // Interrupt 1 count after each update
  timer.attachCompare1Interrupt(readEncoder);
  timer.refresh();
  timer.resume();
}

#define BUFFPIXEL 20

void bmpDraw(char *filename, uint8_t x, uint16_t y) {

  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();

  //if((x > tft.width()) || (y > tft.height())) return;
tft.setCursor(0,0);
  tft.setTextColor(ST7735_WHITE);
  
  

  // Open requested file on SD card
  if ((bmpFile = sd.open(filename)) == NULL) {
    tft.setCursor(0,0);
    tft.setTextColor(ST7735_WHITE);
    tft.print("NOT");
    return;
  }

  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    // Read DIB header
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      if(true) { //(bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed

        goodBmp = true; // Supported BMP format -- proceed!

        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;

        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
        }

        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;

        // Set TFT address window to clipped image bounds
        tft.setAddrWindow(x, y, x+w-1, y+h-1);

        for (row=0; row<h; row++) { // For each scanline...

          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }

          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
            }

            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.Color565(r,g,b));
          } // end pixel
        } // end scanline
      } // end goodBmp
    }
  }
  if (goodBmp) tft.print("GOOD");
  else tft.print("BAD");
  bmpFile.close();
}

// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.

uint16_t read16(File &f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}

uint32_t read32(File &f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}


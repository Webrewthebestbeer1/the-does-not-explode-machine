#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX_AS.h>
#include <Adafruit_ST7735.h> 
#include <SPI.h>
#include <MapleCoOS116.h>


#define SWITCH 11
#define LED_POWER 12
#define LED_RUNNING 13
#define ONE_WIRE_BUS PB10
#define SSR PC13
#define SPEAKER 10
#define MAGIC 80.5

#define TFT_CS     PA4
#define TFT_RST    0
#define TFT_DC     PA1
#define SD_CS      PA2

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS,  TFT_DC, TFT_RST);

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

OS_STK   vCubeLoopStk[TASK_STK_SIZE];
OS_STK   vUpdateSSRStk[TASK_STK_SIZE];

unsigned long activatedTime, deactivatedTime = millis();
float temp;
bool plinged = false;
int SSR_FREQUENCY = 3000; // in ms

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
  tft.initR(INITR_BLACKTAB);
  tft.setRotation(1);
  tft.fillScreen(ST7735_BLACK);
  tft.setCursor(50,10);
  tft.setTextSize(3);
  tft.setTextWrap(true);
  Serial.begin(9600);
  initEncoder();
  cube1_x = ((tft.width()) / 2);
  cube1_y = ((tft.height()) / 2);
  cube1_color = ST7735_RED;

  setupCo();
}

void setupCo() {
  CoInitOS();
  CoCreateTask(vCubeLoopTask,
    (void *) 0,
    2,
    &vCubeLoopStk[TASK_STK_SIZE - 1],
    TASK_STK_SIZE
  );
  CoCreateTask(vUpdateSSRStateTask,
    (void *) 0,
    2,
    &vUpdateSSRStk[TASK_STK_SIZE - 1],
    TASK_STK_SIZE
  );
  CoStartOS();
}

void loop() {
  //sensors.requestTemperatures();
  //temp = sensors.getTempCByIndex(0);
  updateSSRState();
  
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
  if (temp > MAGIC && !plinged) {
    playPling();
    plinged = true;
  }
}

static void vCubeLoopTask(void *pdata) {
  while(1) {
    cube(cube1_px, cube1_py, cube1_pz, cube1_p2x, cube1_p2y, cube1_r, &cube1_x, &cube1_y, &cube1_color);
    CoTickDelay(105-encoderPos);
  }
}

static void vUpdateSSRStateTask(void *pdata) {
  while(1) {
    updateSSRState();
  }
}

void playPling() {
  // play an a for 200ms
  //tone(SPEAKER, 440, 200);
}

/*
 * Slow PWM type control of the SSR
 */
void updateSSRState() {
  float dutyCycle = (float)encoderPos / 100.;
  Serial.println(dutyCycle, DEC);
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

void setEncoderPos(uint8_t move) {
  int8_t newEncoderPos = encoderPos + move;
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
  for (int i = 0; i < 8; i++)
  {
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
        setEncoderPos(1);
      }
      encoderTimer = millis();
    }
    if ((gpio_read_bit(PIN_MAP[encoderPinB].gpio_device, PIN_MAP[encoderPinB].gpio_bit) ? HIGH : LOW) != B_set) {
      B_set = !B_set;
      if (B_set && !A_set)
        setEncoderPos(-1);
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


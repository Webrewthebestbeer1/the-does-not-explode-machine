#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_GFX_AS.h>
#include <Adafruit_ST7735.h> 
#include <SPI.h>

#define SWITCH 11
#define LED_POWER 12
#define LED_RUNNING 13
#define ONE_WIRE_BUS 2
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

unsigned long activatedTime, deactivatedTime = millis();
float temp;
bool plinged = false;
int SSR_FREQUENCY = 3000; // in ms

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
  Serial.begin(9600);
  initEncoder();
}

void loop() {
  sensors.requestTemperatures();
  temp = sensors.getTempCByIndex(0);
  updateSSRState();
  if (lastEncoderPos != encoderPos) {
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(50,10);
    tft.setTextSize(6);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextWrap(true);
    tft.print((char)encoderPos, DEC);
    encoderFlag = LOW;
    lastEncoderPos = encoderPos;
  }
  if (temp > MAGIC && !plinged) {
    playPling();
    plinged = true;
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


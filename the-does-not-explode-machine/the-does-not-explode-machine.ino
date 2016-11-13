#include <OneWire.h>
#include <DallasTemperature.h>
#include <Encoder.h>

#define SWITCH 11
#define LED_POWER 12
#define LED_RUNNING 13
#define ONE_WIRE_BUS 2
#define SSR 14
#define SPEAKER 10
#define MAGIC 80.5
#define ENC_A 2
#define ENC_B 3
#define ENC_MAX 100
#define ENC_DIV 4 // encoder counts 4 steps per click

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);
Encoder encoder(ENC_A, ENC_B);

unsigned long activatedTime, deactivatedTime;
float temp;
long encoderValue;
bool plinged = false;
int SSR_FREQUENCY = 3000; // in ms

void setup() {
  pinMode(SWITCH, INPUT);
  pinMode(SSR, OUTPUT);
  pinMode(SPEAKER, OUTPUT);
  pinMode(LED_POWER, OUTPUT);
  pinMode(LED_RUNNING, OUTPUT);
  sensors.begin();
  encoder.write(0);
}

void playPling() {
  // play an a for 200ms
  tone(SPEAKER, 440, 200);
}

/*
 * Reads encoder and returns the value as a percentage (0%-100%)
 */
long readEncoder() {
  long value = encoder.read();
  if (value > ENC_MAX * ENC_DIV) {
    encoder.write(ENC_MAX * ENC_DIV);
    return ENC_MAX;
  }
  if (value < 0) {
    encoder.write(0);
    return 0;
  }
  return value / ENC_DIV;
}

/*
 * Slow PWM type control of the SSR
 */
void updateSSRState() {
  float dutyCycle = readEncoder() / 100;
  unsigned long onTime = (float)SSR_FREQUENCY * dutyCycle;
  unsigned long offTime = SSR_FREQUENCY - onTime;
  if (digitalRead(SSR)) {
    if (activatedTime + onTime > millis()) {
      digitalWrite(SSR, 0);
      deactivatedTime = millis();
    }
  } else {
    if (deactivatedTime + offTime > millis()) {
      digitalWrite(SSR, 1);
      activatedTime = millis();
    }
  }
}

void loop() {
  sensors.requestTemperatures();
  temp = sensors.getTempCByIndex(0);
  encoderValue = readEncoder();
  updateSSRState();
  
  if (temp > MAGIC && !plinged) {
    playPling();
    plinged = true;
  }

}

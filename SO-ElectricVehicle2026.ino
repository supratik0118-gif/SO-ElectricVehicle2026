/*
  Raspberry Pi Pico RP2040
  Souce code for Electric Vehicle Event

  Supratik Biswas
  https://github.com/supratik0118-gif/SO-ElectricVehicle2026
*/

/*
  Motor Driver TB6612FNG is used
  Pico GP0 --> PWMA
  Pico GP1 --> PWMB
*/
#include <Adafruit_NeoPixel.h>

#define PIN       16
#define NUMPIXELS 1

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

volatile unsigned int count_left;
volatile unsigned int count_right;
unsigned int count_left_buff;
unsigned int count_right_buff;
bool run = 0;
void setup() {
  // put your setup code here, to run once:
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  count_left = 0;
  count_right = 0;
  Serial.begin(9600);
  attachInterrupt(2, readEncoder_left, FALLING);
  attachInterrupt(3, readEncoder_right, FALLING);
  //noInterrupts();
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(100, 0, 0));
  pixels.show();
  //noInterrupts();

}

void readEncoder_left() {
  count_left++;
}
void readEncoder_right() {
  count_right++;
}


void loop() {
  // put your main code here, to run repeatedly:
  if(BOOTSEL){
    count_left = 0;
    count_right = 0;
    count_left_buff = 0;
    count_right_buff = 0;
    run = 1;
    pixels.setPixelColor(0, pixels.Color(0, 100, 0));
    pixels.show();
    interrupts();
  }

  if(run)
  {
    analogWrite(0, 200);
    analogWrite(1, 200);
    delay(2000);
    analogWrite(0, 0);
    analogWrite(1, 0);
    //noInterrupts();
    pixels.setPixelColor(0, pixels.Color(100, 0, 0));
    pixels.show();
    run = 0;
    Serial.print(count_left);
    Serial.print("\t");
    Serial.println(count_right);


  }

}
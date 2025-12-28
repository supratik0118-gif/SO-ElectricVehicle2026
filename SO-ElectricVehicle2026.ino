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

#define DISTANCE_T      5000
#define TIME_T          5000

#define DISTANCE_TO_ENCODER_COUNT 1.4

#define RGB_PIN         16
#define NUMPIXELS       1

#define LEFT_PWM        0
#define RIGHT_PWN       1

#define LEFT_ENCODER    3
#define RIGHT_ENCODER   2

#define LEFT_ENCODER1   4
#define RIGHT_ENCODER1  5


Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

bool run = 0;

unsigned char distance_phase;

unsigned int encoder_count[5];
const unsigned char pwm[5] ={50, 150, 200, 150, 50};

//const unsigned char pwm[5] ={100, 100, 100, 100, 100};
volatile unsigned int count_left;
volatile unsigned int count_right;
unsigned int count_left_buff;
unsigned int count_right_buff;
unsigned long start_millis;
unsigned long current_millis;
char left_offset;
char right_offset;

void setup() {
  // put your setup code here, to run once:
  distance_phase = 0;
  encoder_count[0] = (unsigned int) (DISTANCE_T * (float)DISTANCE_TO_ENCODER_COUNT/5);
  encoder_count[1] = (unsigned int) (DISTANCE_T * (float)DISTANCE_TO_ENCODER_COUNT/5) + encoder_count[0];
  encoder_count[2] = (unsigned int) (DISTANCE_T * (float)DISTANCE_TO_ENCODER_COUNT/5) + encoder_count[1];
  encoder_count[3] = (unsigned int) (DISTANCE_T * (float)DISTANCE_TO_ENCODER_COUNT/5) + encoder_count[2];
  encoder_count[4] = (unsigned int) (DISTANCE_T * (float)DISTANCE_TO_ENCODER_COUNT/5) + encoder_count[3];
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWN, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_ENCODER1, INPUT);
  count_left = 0;
  count_right = 0;
  Serial.begin(9600);
  attachInterrupt(LEFT_ENCODER, readEncoder_left, FALLING);
  attachInterrupt(RIGHT_ENCODER, readEncoder_right, FALLING);
  //noInterrupts();
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(100, 0, 0));
  pixels.show();
  //noInterrupts();
  delay(5000);
  Serial.println(encoder_count[0]);
  Serial.println(encoder_count[1]);
  Serial.println(encoder_count[2]);
  Serial.println(encoder_count[3]);
  Serial.println(encoder_count[4]);

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
    //interrupts();
    start_millis = millis();
  }

  if(run)
  {
    

    noInterrupts();
    count_left_buff = count_left;
    count_right_buff = count_right;
    interrupts();
    if(count_left > count_right)
    {
      //left_offset = count_left - count_right;
      left_offset = 0;
      right_offset = 0;
    }
    else
    {
      //right_offset = count_right - count_left;
      left_offset = 0;
      right_offset = 0;
    }

    analogWrite(LEFT_PWM, pwm[distance_phase] - left_offset);
    analogWrite(RIGHT_PWN, pwm[distance_phase] - right_offset);

    if(count_right_buff >= encoder_count[distance_phase]){
      distance_phase++;
      if(distance_phase == 5){
        distance_phase = 0;
        analogWrite(LEFT_PWM, 0);
        analogWrite(RIGHT_PWN, 0);
        run = 0;
        pixels.setPixelColor(0, pixels.Color(100, 0, 0));
        pixels.show();
        current_millis = millis();
        Serial.println(current_millis - start_millis);
      }
    }

    Serial.print(count_left);
    Serial.print("\t");
    Serial.println(count_right);


  }

}
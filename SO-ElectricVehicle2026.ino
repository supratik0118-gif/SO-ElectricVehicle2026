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

#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"

#include <Wire.h>

#include <Adafruit_NeoPixel.h>

#define DISTANCE_T      7000
#define TIME_T          5000

#define DISTANCE_TO_ENCODER_COUNT 0.76
#define RGB_PIN         16
#define NUMPIXELS       1

#define LEFT_PWM        0
#define RIGHT_PWM       1

#define LEFT_ENCODER    3
#define RIGHT_ENCODER   2

#define LEFT_ENCODER1   4
#define RIGHT_ENCODER1  5

MPU6050 mpu;


#define OUTPUT_READABLE_YAWPITCHROLL


// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);

bool run = 0;

unsigned char distance_phase;

unsigned int encoder_count[5];
const unsigned char left_pwm[5] = {200, 200, 200, 200, 100};
const unsigned char right_pwm[5] = {200, 200, 200, 200, 100};

//const unsigned char pwm[5] ={100, 100, 100, 100, 100};
volatile unsigned int count_left;
volatile unsigned int count_right;
unsigned int count_left_buff;
unsigned int count_right_buff;
unsigned long start_millis;
unsigned long current_millis;
int left_offset;
int right_offset;
int both_offset;
int time_to_reach;
int distance_to_reach;
float drift;
float start_angle;
float current_angle;

void setup() {
  // put your setup code here, to run once:
  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties

  mpu.initialize();
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

// make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  distance_phase = 0;
  encoder_count[0] = (unsigned int) (DISTANCE_T * (float)DISTANCE_TO_ENCODER_COUNT/5);
  encoder_count[1] = (unsigned int) (DISTANCE_T * (float)DISTANCE_TO_ENCODER_COUNT/5) + encoder_count[0];
  encoder_count[2] = (unsigned int) (DISTANCE_T * (float)DISTANCE_TO_ENCODER_COUNT/5) + encoder_count[1];
  encoder_count[3] = (unsigned int) (DISTANCE_T * (float)DISTANCE_TO_ENCODER_COUNT/5) + encoder_count[2];
  encoder_count[4] = (unsigned int) (DISTANCE_T * (float)DISTANCE_TO_ENCODER_COUNT/5) + encoder_count[3];
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  pinMode(LEFT_ENCODER, INPUT);
  pinMode(RIGHT_ENCODER1, INPUT);
  count_left = 0;
  count_right = 0;
  Serial.begin(115200);
  attachInterrupt(LEFT_ENCODER, readEncoder_left, FALLING);
  //attachInterrupt(RIGHT_ENCODER, readEncoder_right, FALLING);
  //noInterrupts();
  pixels.begin();
  pixels.setPixelColor(0, pixels.Color(100, 0, 0));
  pixels.show();
  //noInterrupts();
  //delay(5000);
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
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //Serial.print("ypr\t");
      //Serial.print(ypr[0] * 180/M_PI);
      //Serial.print("\t");
      //Serial.print(ypr[1] * 180/M_PI);
      //Serial.print("\t");
      //Serial.println(ypr[2] * 180/M_PI);
      //Serial.println(ypr[0]);
    #endif

  }
  if(BOOTSEL){
    count_left = 0;
    count_right = 0;
    count_left_buff = 0;
    count_right_buff = 0;
    run = 1;
    pixels.setPixelColor(0, pixels.Color(0, 100, 0));
    pixels.show();
    //interrupts();
    start_angle = ypr[0];
    left_offset = 0;
    right_offset = 0;
    start_millis = millis();
  }

  if(run)
  {
    

    //noInterrupts();
    count_left_buff = count_left;
    count_right_buff = count_right;
    //interrupts();

    current_angle = ypr[0];
    if(current_angle > start_angle){
      if(right_offset > 0){
        right_offset--;
      }
      else{
        left_offset = (current_angle-start_angle) * 2000;
      }
    }
    else if(current_angle < start_angle){
      if(left_offset > 0){
        left_offset--;
      }
      else{
        right_offset = (start_angle - current_angle) * 2000;
      }
    }
    else{
      left_offset = 0;
      right_offset = 0;
    }

   
    if((distance_phase == 4) || (distance_phase == 4)){
      current_millis = millis();
      time_to_reach = TIME_T - (current_millis - start_millis);
      distance_to_reach = encoder_count[4] - count_left_buff;
      both_offset = distance_to_reach - time_to_reach;
      if(both_offset > 255)
      {
        both_offset = 255;
      }
      else if(both_offset <50)
      {
        both_offset = 50;
      }
      if((time_to_reach < 200) && (both_offset <150) && (distance_to_reach > time_to_reach))
      {
        both_offset += 100;

      }
      if(time_to_reach < 0)
      {
        both_offset = 255;
      }
      analogWrite(LEFT_PWM, both_offset - left_offset);
      analogWrite(RIGHT_PWM, both_offset - right_offset);

    }
    else{
      analogWrite(LEFT_PWM, left_pwm[distance_phase] - left_offset);
      analogWrite(RIGHT_PWM, right_pwm[distance_phase] - right_offset);
    }
    
    
    if(count_left_buff >= encoder_count[distance_phase]){
      distance_phase++;
      if(distance_phase == 5){
        distance_phase = 0;
        analogWrite(LEFT_PWM, 0);
        analogWrite(RIGHT_PWM, 0);
        run = 0;
        pixels.setPixelColor(0, pixels.Color(100, 0, 0));
        pixels.show();
        current_millis = millis();
        Serial.println(current_millis - start_millis);
      }
    }

    Serial.print(count_left_buff);
    Serial.print("\t");
    Serial.println(count_right_buff);
    Serial.print(distance_to_reach);
    Serial.print("\t");
    Serial.print(time_to_reach);
    Serial.print("\t");
    Serial.println(both_offset);


  }

}
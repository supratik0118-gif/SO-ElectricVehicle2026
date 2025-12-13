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

int count_left;
int count_right;
void setup() {
  // put your setup code here, to run once:
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  count_left = 0;
  count_right = 0;
  Serial.begin(9600);
  attachInterrupt(2, readEncoder_left, CHANGE);
  attachInterrupt(3, readEncoder_right, CHANGE);

}

void readEncoder_left() {
  count_left++;
}
void readEncoder_right() {
  count_right++;
}


void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(0, 20);
  analogWrite(1, 20);
  delay(1000);
  Serial.print(count_left);
  Serial.print("\t");
  Serial.println(count_right);
  }



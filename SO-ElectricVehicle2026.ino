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
void setup() {
  // put your setup code here, to run once:
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);

}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(0, 200);
  analogWrite(1, 0);
  delay(1000);
  analogWrite(0, 100);
  analogWrite(1, 100);
  delay(1000);
  analogWrite(0, 0);
  analogWrite(1, 200);
  delay(1000);

}

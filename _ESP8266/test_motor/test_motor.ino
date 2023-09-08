#define chan1 3
#define chan2 4
void setup() {
  pinMode (chan1 , OUTPUT);
  pinMode(chan2, OUTPUT);

}

void loop() {
  digitalWrite(chan1, LOW);
  digitalWrite(chan2, HIGH);

}

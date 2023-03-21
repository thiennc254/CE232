void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(18, OUTPUT);
  pinMode(21, OUTPUT);
  digitalWrite(21, LOW);
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1000);
  digitalWrite(18, HIGH);
  Serial.println("ON");
  delay(1000);
  digitalWrite(18, LOW);
  Serial.println("OFF");

}
double incomingByte;   // for incoming serial data
String string = "Data: ";

void setup() {
  Serial.begin(115200);     // opens serial port, sets data rate to 9600 bps
  pinMode(9, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);
}

void loop() {

  // receive data:
  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.parseFloat();
    int a = incomingByte;
    analogWrite(9, a);
    //Serial.print(incomingByte);
    //sprintf(dataString, "%02X", a); // convert a value to hexa
    //Serial.println(incomingByte);   // send the data
    incomingByte = analogRead(A0);
    Serial.println(incomingByte);   // send the data
    //delay(1000);                  // give the loop some break
  }
}

void my_blink(int pin) {
  digitalWrite(pin, HIGH);
  delay(1000);
  digitalWrite(pin, LOW);
  delay(1000);
}

void pwwm(int pin) {
  digitalWrite(pin, HIGH);
  delay(1000);
  digitalWrite(pin, LOW);
  delay(1000);
}

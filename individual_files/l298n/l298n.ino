// PINS definieren Motor A
int IN1 = 4;
int IN2 = 5;
int speedPinA = 9; // PWM

// PINS definieren Motor B
int IN3 = 6;
int IN4 = 7;
int speedPinB = 10; // PWM


void setup()
{
 // Ausgabe PINS
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(speedPinA,OUTPUT);
 pinMode(speedPinB,OUTPUT);
}

void loop() {
  
 analogWrite(speedPinA, 40); // Geschwindigkeit einstellen 0-255
 analogWrite(speedPinB, 50); 

 // Motor A forwaerts
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 delay(2000);
 
 // Motor A rueckwaerts
 // digitalWrite(IN1, LOW);
 // digitalWrite(IN2, HIGH);
 // delay(2000);

 // Motor A deaktivieren
 // digitalWrite(IN1, LOW);
 // digitalWrite(IN2, LOW);
 // delay(2000);

 // Motor B forwaerts
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, LOW);
 delay(2000);
 
 // Motor B rueckwaerts
 // digitalWrite(IN3, LOW);
 // digitalWrite(IN4, HIGH);
 // delay(2000);

 // Motor B deaktivieren
 // digitalWrite(IN3, LOW);
 // digitalWrite(IN4, LOW);
 // delay(2000);
}

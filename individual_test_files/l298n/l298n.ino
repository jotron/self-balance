// Program: Control 2 DC motors using L298N H Bridge http://qqtrading.com.my/stepper-motor-driver-module-L298N

// Definitions Arduino pins connected to input H Bridge
int IN1 = 4;
int IN2 = 5;
int speedPinA = 9; // Needs to be a PWM pin to be able to control motor speed

int IN3 = 6;
int IN4 = 7;
int speedPinB = 10; // Needs to be a PWM pin to be able to control motor speed


void setup()
{
 // Set the output pins
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(speedPinA,OUTPUT);
 pinMode(speedPinB,OUTPUT);
}

void loop() {
 analogWrite(speedPinA, 100); // Sets speed variable via PWM
 analogWrite(speedPinB, 100); // Sets speed variable via PWM

 // Motor A (MPU--Seite von Kühler-Seite) forwärts
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, LOW);
 delay(2000);
 // festhalten
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, HIGH);
 delay(2000);

 /*
 // Rotates the Motor A counter-clockwise
 digitalWrite(IN1, LOW);
 digitalWrite(IN2, HIGH);
 delay(2000);
 // Motor A
 digitalWrite(IN1, HIGH);
 digitalWrite(IN2, HIGH);
 delay(500);

 // Rotates the Motor B counter-clockwise
 digitalWrite(IN3, LOW);
 digitalWrite(IN4, HIGH);
 delay(2000);
 // Motor B
 digitalWrite(IN3, HIGH);
 digitalWrite(IN4, HIGH);
 delay(500);*/
}

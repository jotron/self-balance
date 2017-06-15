// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

//Gyroskop-Accelerometer initialisieren
MPU6050 accelgyro;

// Variablen definieren
int16_t ax, ay, az; //Accelereometer Rohdaten
int16_t gx, gy, gz; //Gyroskop Rohdaten
double gx_correct, gy_correct, gz_correct; // Gyroskop in Grad/Sekunde
double accXangle, accYangle; // Accelerometer Drehung um entsprechende Achse
double compAngleX; // berechneter Winkel
double timer;
double delta; // Neigung gegenüber Gravitations-Achse

// Input Motor A
int IN1 = 4;
int IN2 = 5;
int speedPinA = 9;

//Input Motor B
int IN3 = 6;
int IN4 = 7;
int speedPinB = 10;

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Serieller Monitor
    Serial.begin(9600);

    // Sensor initialisieren
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // Verbindung testen
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // Gemessene natürliche Fehlwerte
    accelgyro.setXGyroOffset(39);
    accelgyro.setYGyroOffset(-88);
    accelgyro.setZGyroOffset(-6);
    accelgyro.setXAccelOffset(-1994);
    accelgyro.setYAccelOffset(-382);
    accelgyro.setZAccelOffset(1245);


    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);


    //Anfangswinkel
    compAngleX = 180;

    // L298N Pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(speedPinA,OUTPUT);
    pinMode(speedPinB,OUTPUT);
}

void loop() {
    // Rohdaten ablesen
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Accelerometer-Rohdaten in Grad umrechnen
    accXangle = (atan2(ay, az) + PI) * RAD_TO_DEG;
    accYangle = (atan2(ax, az) + PI) * RAD_TO_DEG;

    //Gyroskop in grad/sec umwandeln
    gx_correct = gx / 131;
    gy_correct = gy / 131;
    gz_correct = gz / 131;

    // Komplämentärer Filter
    compAngleX = (0.98 * (compAngleX + (gx_correct * (double)(micros() - timer) / 1000000))) + (0.02 * accXangle);
    delta = 180-compAngleX;

    timer = micros();

    // Serielle Ausgabe
    //   Serial.println(delta);

    //L298N
    if (-0.5 > delta > 0.5) {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, HIGH);

      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, HIGH);
    }
    if (10 > delta > 0.5) {
      analogWrite(speedPinA, 50); // Sets speed variable via PWM
      analogWrite(speedPinB, 50); // Sets speed variable via PWM

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    if (-10 > delta < -0.5) {
      analogWrite(speedPinA, 50); // Sets speed variable via PWM
      analogWrite(speedPinB, 50); // Sets speed variable via PWM

      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, LOW);
    }
    if (delta > 10) {
      analogWrite(speedPinA, 50); // Sets speed variable via PWM
      analogWrite(speedPinB, 50); // Sets speed variable via PWM

      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);

      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
    }
    if (delta < -10) {
      analogWrite(speedPinA, 50); // Sets speed variable via PWM
      analogWrite(speedPinB, 50); // Sets speed variable via PWM

      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);

      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
    }

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(10);
}

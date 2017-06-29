// Alle Bibliotheken muessen installiert sein.
#include "PID_v1.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Wire-Library nur in gewissen Faellen benoetigt
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//Gyroskop-Accelerometer initialisieren
MPU6050 mpu;

// Variablen definieren MPU
bool dmpReady = false;  // wird auf True gesetzt falls Verbindung erfolgreich
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Input Motor A
int IN1 = 4;
int IN2 = 5;
int speedPinA = 9;

// Input Motor B
int IN3 = 6;
int IN4 = 7;
int speedPinB = 10;

// PID variablen definieren
/* Der Sensor misst den Winkel von -180 bis + 180
 * Wir erweitern den Betrag davon auf 255 und setzen 255 als 0° und Zielpunkt für das PID 
 * Der PID gibt, dann wiederum einen Output von 0 bis 255 (235) mit dem man direkt
 * die Motoren ansteuern kann über PWM
*/
double delta; // eigentliche Winkel gegenüber Senkrecht-Achse
double pid_output = 0.0;
double pid_input = 225; // Winkel gegenüber Senkrecht-Achse erweitert
double setpoint = 225; // Zielwert fuer PID-Controller
double Kp = 1;   //Proportional (Schwankstärke)
double Kd = 0; // beruhigen
double Ki = 0; // beruhigen

// PID initialisieren
PID pid(&pid_input, &pid_output, &setpoint, Kp, Ki, Kd, DIRECT);

// Testet Unterbrechung mit MPU
volatile bool mpuInterrupt = false;
// indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void mpu_init() {
    // I2C Verbindung initialisieren
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // Serielle Kommunikation initialisieren sowie MPU initialisieren
    Serial.begin(9600);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // Verbindung testen
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // DMP initialisieren
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Unsere eigenen gemessenen Offsets
    mpu.setXGyroOffset(41);
    mpu.setYGyroOffset(-92);
    mpu.setZGyroOffset(-23);
    mpu.setXAccelOffset(-1659);
    mpu.setYAccelOffset(-329);
    mpu.setZAccelOffset(1230);

    //Falls erfolgreich verbunden DMP gestartet
    if (devStatus == 0) {
        // DMP anstellen
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // Unterbruch-Routine aktivieren
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // dmpReady auf true setzen, damit der Main loop starten kann
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // erwartete Packetgroesse bekommen fuer spaeteren Vergleich
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

}
void setup() {

    //Accelereometer-Gyroskop initialisieren
    mpu_init();


    // L298N Pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(speedPinA,OUTPUT);
    pinMode(speedPinB,OUTPUT);

    //Setup PID
    pid.SetMode(AUTOMATIC); // PID aktivieren
    pid.SetSampleTime(10); // PID nur alle 10ms neu berechnen
    pid.SetOutputLimits(0, 225); // von 0 bis 225 weil + 30 MotorenTraegheit
}

// ================================================================
// ===                         CORE                             ===
// ================================================================
void motors() {
  // Falls Winkel positiv, Motoren in die richtige Richtung antreiben
  if (delta > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
  }
  // Falls Winkel negativ, Motoren in die andere Richtung antreiben
  else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
  
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
   }
   
   // aktivieren für die PID kontrollierte Version
   //analogWrite(speedPinA, pid_output + 20); // Geschwindigkeit einstellen
   //analogWrite(speedPinB, pid_output + 35);

   // aktivieren für die Brute-Force Version
    analogWrite(speedPinA, abs(delta) * 20 - 7);
    analogWrite(speedPinB, abs(delta) * 20 + 7);
}
void mpu_get() {
  // Falls Programm gescheitert hat oder noch nicht bereit ist, nichts tun
    if (!dmpReady) return;

    // warten auf MPU-Signal
    while (!mpuInterrupt && fifoCount < packetSize) {
      //
    }

    // mpuInterrupt wieder auf false resetten und INT_STATUS empfangen
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // Aktuellen FIFOBUFFER zaehlen
    fifoCount = mpu.getFIFOCount();

    // Overflow ueberpruefen und falls ja FIFO leeren
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // Sonst fuer Daten bereit zum ablesen testen
    } else if (mpuIntStatus & 0x02) {
        // Auf korrekte Paketlaenge warten
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Paket lesen
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // Falls mehr als ein Paket verfuegbar FIFO zaehlen
        fifoCount -= packetSize;

        // Eulerschen Winkel abfragen, berechnet vom interen Chip des Sensors
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Winkel in Grad abspeichern (-90 bis +90)
        delta = ypr[2] * 180/M_PI;
        // Winkel erweitern auf (0 bis 255) und umkehren (255 entspricht 0-Grad Winkel)
        pid_input = 225 - abs(ypr[2] * 180/M_PI) * 2.5;
    
    }
}
void loop() {
    mpu_get(); // Winkelmessung bekommen
    
    Serial.print("PID INPUT: " + String(pid_input));
    Serial.print(" PID OUTPUT: " + String(pid_output));
    Serial.println(" Winkel: " + String(delta));
    
    
    // aktivieren fuer die PID-Kontrollierte Version
    // wird eigentlich nur ausgefuehrt wenn 10ms vorbei sind
    //pid.Compute(); // PID -Controller abfragen

    //L298N entsprechend steuern
    motors();

    //delay(10);
}

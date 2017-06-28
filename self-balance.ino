// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "PID_v1.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

//Gyroskop-Accelerometer initialisieren
MPU6050 mpu;

// Variablen definieren MPU
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// Input Motor A
int speedcalc = 0;
int IN1 = 4;
int IN2 = 5;
int speedPinA = 9;

//Input Motor B
int IN3 = 6;
int IN4 = 7;
int speedPinB = 10;

//PID
/* Der Sensor misst den Winkel von -180 bis + 180
 * Wir erweitern den Betrag davon auf 255 und setzen 255 als 0° und Zielpunkt für das PID 
 * Der PID gibt, dann wiederum einen Output von 0 bis 255 (235) mit dem man direkt
 * die Motoren ansteuern kann über PWM
*/
double delta;
double pid_output = 0.0;
double pid_input = 255; // Neigung gegenüber Senkrecht-Achse
double setpoint = 255;
double Kp = 10;   //Schwankstärke
double Kd = 2; // beruhigen
double Ki = 0; // beruhigen
PID pid(&pid_input, &pid_output, &setpoint, Kp, Ki, Kd, DIRECT);


// ================================================================
// ===              MPUINTERRUPT DETECTION ROUTINE              ===
// ================================================================
volatile bool mpuInterrupt = false;
// indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void mpu_init() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(9600);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // Unsere eigenen gemessenen Offsets
    mpu.setXGyroOffset(41);
    mpu.setYGyroOffset(-92);
    mpu.setZGyroOffset(-23);
    mpu.setXAccelOffset(-1659);
    mpu.setYAccelOffset(-329);
    mpu.setZAccelOffset(1230);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
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

}
void setup() {

    //Accelereometer-Garoskop initialisieren
    mpu_init();


    // L298N Pins
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(speedPinA,OUTPUT);
    pinMode(speedPinB,OUTPUT);

    //Setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    //pid.SetOutputLimits(0, 512);
}

// ================================================================
// ===                         CORE                             ===
// ================================================================
void motors() {
  if (delta > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);

        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
  }
  else {
        igitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
  
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
   }
   // aktivieren für die PID kontrollierte Version
   //analogWrite(speedPinA, pid_output + 20);
   //analogWrite(speedPinB, pid_output + 20);
   analogWrite(speedPinA, 100);
   analogWrite(speedPinB, 100);
}
void mpu_get() {
  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        // display Euler angles in degrees
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        
        delta = ypr[2] * 180/M_PI;
        pid_input = 255 - abs(ypr[2] * 180/M_PI) * 2.83;
    
    }
}
void loop() {
    // Winkelmessung
    mpu_get();
    Serial.print("PID INPUT: " + String(pid_input));
    Serial.print(" PID OUTPUT: " + String(pid_output));
    Serial.println(" Winkel: " + String(delta));
    
    
    // aktivieren fuer die PID-Kontrollierte Version
    //pid.Compute();

    //L298N
    motors();

    delay(10);
}

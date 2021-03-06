// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"


MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;
int xangle;
double gx_correct;
double gy_correct;
double gz_correct;
double accXangle;
double accYangle;
double compAngleX;
double timer;
double delta;

#define LED_PIN 13
bool blinkState = false;

void setup() {
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (38400 chosen because it works as well at 8MHz as it does at 16MHz, but
    // it's really up to you depending on your project)
    Serial.begin(9600);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // Custom Offsets
    accelgyro.setXGyroOffset(39);
    accelgyro.setYGyroOffset(-88);
    accelgyro.setZGyroOffset(-6);
    accelgyro.setXAccelOffset(-1994);
    accelgyro.setYAccelOffset(-382);
    accelgyro.setZAccelOffset(1245);


    // configure Arduino LED for
    pinMode(LED_PIN, OUTPUT);

    compAngleX = 180;
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    // Convert accelerometer raw values to degrees
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
      //Serial.println(compAngleX);
    Serial.println(delta);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    delay(10);
}

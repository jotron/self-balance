# Selbst-Balancierender Roboter

### Ziel

Ziel ist es einen Roboter mit 2-Rädern zu erfinden, der von selbst gerade stehen kann in dem er sich ausbalanciert.

Mit einem Sensor soll der Roboter die Abneigung zur Senkrecht-Achse messen. Daraus folgend soll er die Motoren in die richtige Richtung stark genug antreiben.

![balancing](https://cdn.instructables.com/ORIG/F5J/KNAF/HTNO6W4R/F5JKNAFHTNO6W4R.png)

### Bauteile

- Arduino Uno
- L298n H-Brücke
- MPU-6050 Gyroskop-Accelerometer

### Schaltskizze

![Sketch](circuit-diagramms/Sketch.png)

### Libraries

- [PID](https://github.com/br3ttb/Arduino-PID-Library/)
- [I2Cdev](https://github.com/jrowberg/i2cdevlib)
- [MPU6050](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
- Wire

### Arbeitsvorgang

##### MPU-6050

Als aller erstes haben wir den Sensor mit dem Arduino verbunden und versucht die rohen Daten abzulesen. Dabei geholfen hat uns die [offzielle Arduino Dokumentation](https://playground.arduino.cc/Main/MPU-6050) sowie [ein Tutorial](https://create.arduino.cc/projecthub/Aritro/getting-started-with-imu-6-dof-motion-sensor-96e066).

Wir haben anfänglich versucht die rohen Accelerometer und Gyroskop-Werte mittels [Komplementären-Filter](https://bayesianadventures.wordpress.com/2013/10/20/gyroscopes-accelerometers-and-the-complementary-filter/) zu kombinieren um den Winkel zur Senkrechtachse zu bekommen. Das hat zwar funktioniert war aber rechenaufwendig. Deshalb haben wir uns schlussendlich doch auf den *Digital Motion Processor* des Sensor's verlassen. Der Sensor kann nämlich die Daten selber schon auswerten und praktisch die Neigung zurückgeben. ([hilfreiches Beispiel-Programm](https://github.com/jrowberg/i2cdevlib/blob/master/Arduino/MPU6050/examples/MPU6050_DMP6/MPU6050_DMP6.ino)).

Um die Präzision zu erhöhen mussten wir die Sensor-spezifische Fehlabneigungen berechnen. Dazu geholfen hat uns das [Skript](http://wired.chillibasket.com/2015/01/calibrating-mpu6050/) von *Luis Ródenas*.

##### L298n mit 2 DC-Motoren

Als nächstes haben wir die Motoren mit der H-Brücke verkabelt und diese zur externen Stomquelle und zum Arduino. Das Wissen dazu haben wir aus diesen beiden Tutorials erlernt: [Dronebotworkshop](http://dronebotworkshop.com/dc-motors-l298n-h-bridge/), [QQtrading](http://qqtrading.com.my/stepper-motor-driver-module-L298N).

Eine Schwierigkeit bestand darin wie man den Strom zuführen sollte. Wir wollten nämlich auf zwei verschiedene Stromquellen verzichten. Schlussendlich haben wir uns entschieden den Arduino über die H-Brücke zu füttern. Diese besitzt nämlich einen 5V-Regulator. Das einzige Risiko dabei war, dass die Motoren eventuell zu viel elektrischen Lärm machen würden. Dies hat sich aber nicht bestätigt.

##### Steuerung

Zum Schluss musste das Fahrzeug vollständig zusammengebaut werden.

Ausserdem mussten ein System entwickeln welches je nach Neigung des Roboters, die Motoren richtig ansteuerte. Dies erwies sich als ausserordentlich schwierig.


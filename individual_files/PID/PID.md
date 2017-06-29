## Steuerung in Abhängigkeit der Neigung mittels PID

Es musste ein System entwickelt werden welches je nach Neigung des Roboters, die Motoren richtig ansteuerte. Dies erwies sich als ausserordentlich schwierig. Die intuitive Version einfach je nach Neigung mit voller Kraft in die richtige Richtung anzutreiben funktioniert nicht.
Wir haben darum eine Bibliothek gefunden die uns diese Aufgabe erleichtert: die [PID Bibliothek](https://playground.arduino.cc/Code/PIDLibrary).

Wikipedia: "Der [PID-Regler](https://de.wikipedia.org/wiki/Regler#PID-Regler) (proportional–integral–derivative controller) besteht aus den Anteilen des Proportionalen Gliedes, des Integral-Gliedes und des Derivativ-Gliedes." Die Bibliothek vergleicht den Input-Wert mit dem gewünschten Output-Wert und versucht den Stellwert optimal zu erreichen.
Man muss dem Regler lediglich einen Input vorschreiben und das gewünschte Resultat einstellen. Der Regler berechnet dann den Weg der minimalen Abweichung. Wie genau die Formel lautet die der Regler verwendet besprechen wir in dieser Arbeit nicht. Der Wikipedia Artikel wird nämlich ziemlich schnell recht komplex.

Der P-Wert steuert dabei wie stark der Roboter schwankt. Der I-Wert vermeidet "Overshoot" und treibt die Motoren gleichmässiger an. Der D-Wert korrigiert eventuelle Fehler.

### Code

Die Anpassungsparameter sind Kp, Ki & Kd.

```c
#include <PID_v1.h>

//Variablen und Parameter definieren
double pid_output = 0.0;
double pid_input = 255; // Neigung gegenüber Senkrecht-Achse
double setpoint = 255;
double Kp = 10;   //Schwankstärke
double Kd = 2; // beruhigen
double Ki = 0; // beruhigen

//PID aufstellen mit den Einstellungskonstanten und den INPUT/OUTPUT Variablen
PID pid(&pid_input, &pid_output, &setpoint, Kp, Ki, Kd, DIRECT);

void setup()
{
  //anschalten
  pid.SetMode(AUTOMATIC);
  
  //Alle 10ms neuberechnen
  pid.SetSampleTime(10);
}

void loop()
{
  //PID kontroller rechnen lassen, nur wenn 10ms vergangen sind, wird etwas passieren
  pid.Compute();
}
```

Der Regler schreibt automatisch in den pid_output. In unserem Programm bezeichnetein pid_input von 255 eine Neigung von 0° gegenüber der Senkrechtachse und ein pid_input von 0 einen +-90° Winkel. Die Richtung der Motoren wird separat manuell berechnet in abhängigkeit der Neigung.

Um auf die richtigen Konstanten zu kommen, benutzen wir die Ziegler-Nichols Methode:

1. Alle Werte auf 0 stellen
2. P so lang erhöhen bis der Roboter stark hin und her schwankt.
3. I so lang erhöhen bis der Roboter stabil steht
4. D experimentieren

Was uns gestört hat an der PID-Bibiothek ist, dass sie nicht selber die Richtung der Motoren kontrollieren kann. Wenn wir der Bibliothek nämlich Inputs von -255(rückwärts) bis +255(vorwärts) geben und der Roboter stabil steht, wird die Bibliothek den tiefsten Wert ausgeben -255 obwohl sie eigentlich 0 ausgeben sollte. 
An die richtigen Werte zu kommen war ebenfalls sehr schwierig. Bis der Roboter einigermassen balanciert hat, haben wir sicherlich Stunden an den Werten gefeilt.
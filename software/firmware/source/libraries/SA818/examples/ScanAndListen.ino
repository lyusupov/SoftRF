#include <SoftwareSerial.h>
#include <SA818.h>
#include <SA818Controller.h>

SoftwareSerial cmdSerial(4, 5);
SA818 sa818(&cmdSerial);
SA818Controller controller(&sa818);

// On prépare 2 variables
float freqDebut = 145.0;
float freqFin = 145.2;

// On prépare une variable qui va nous servir à arrêter le scan
bool scan = true;

void setup() {
  Serial.begin(115200);
  cmdSerial.begin(9600);

  // sa818.verbose();
  if (controller.connect()) {
    Serial.println("Connexion établie");
  }
  if (controller.setGroup(0, freqDebut, 145.75, 0, 0, 0)) {
    Serial.println("Module paramétré");
  }
}

void chrono(unsigned long timeout) {
  unsigned long timer = millis();
  unsigned long now = millis();
  long sec = timer;
  int nbrSec = 0;
  while (now - timer <= timeout) {
    if (now - sec >= 1000) {
      sec = now;
      Serial.print(".");
      nbrSec++;

      if (nbrSec % 10 == 0) {
        Serial.print(nbrSec);
      }
    }
    now = millis();
  }
  Serial.println();
}

void loop() {

  // Si la variable "scan" est à "true"
  if (scan) {

    // On lance un scan de plage de fréquence
    // et on stocke la fréquence de retour dans la variable "freq"
    float freq = controller.next(freqDebut, freqFin);

    // Si le résultat du dernier scan est "0" on continue le programme
    if (controller.result() == "0") { 

      // Si la méthode next() s’arrête, soit :
      // Une fréquence a été trouvée (donc la fréquence donnée par la méthode
      // est différente de la fréquence donnée en paramètre)
      if (freq != freqDebut) {

        // On affiche la fréquence trouvée
        Serial.print("Fréquence: ");
        Serial.println(freq, 4);

        Serial.print("Écoute");
        controller.setRXF(freq);  // On modifie la fréquence de réception
        controller.update();      // On valide (envoie de la commande setGroup() au module)
        chrono(60000);             // On écoute sur fréquence pendant 60sec

        // On modifie la valeur de la variable "freqDebut" par la valeur de "freq"
        // pour que le prochain scan commence là où a terminé le dernier...
        freqDebut = freq;
      }

      // Si la fréquence trouvée correspond à la fréquence de fin de plage
      if (freq == freqFin) {
        // On a fini de scanner la plage
        Serial.println("Fin du scan");
        // On met la variable "scan" à "false"
        // pour ne pas que le programme recommence à scanner au prochain tour
        scan = false;
      }
    }
  }
}
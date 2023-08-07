#include <SoftwareSerial.h>
#include <SA818.h>
#include <SA818Controller.h>

SoftwareSerial cmdSerial(4, 5);
SA818 sa818(&cmdSerial);
SA818Controller controller(&sa818);

void setup() {
  Serial.begin(115200);
  cmdSerial.begin(9600);

  bool connect = controller.connect();
  if (connect) {
    Serial.println("Connexion établie");
    Serial.println("Code retour: " + controller.response());
  } else {
    Serial.println("Erreur de connexion");
    Serial.println("Erreur: " + controller.response());
  }

  bool group = false;
  if (connect) {
    group = controller.setGroup(0, 145.75, 145.75, 12, 0, -754);
    if (group) {
      Serial.println("Groupe paramétré");
    } else {
      Serial.println("Impossible de paramétrer le groupe");
    }
  }

  if(group) {
    controller.setRXF(165.5);
    controller.setSQ(1);
    controller.setBW(1); // On passe sur un pas de 25 Khz
    group = controller.update(); // On transmet les nouvelles valeurs au module
        
    if(!group) {
      Serial.println("Erreur lors de la mise à jour du groupe.");
      Serial.println(controller.response());
    }
  }
}

void loop() {}
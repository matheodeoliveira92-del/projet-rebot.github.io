#include <Arduino.h>

// ==========================================
// CONFIGURATION DES BROCHES (PINS)
// ==========================================
const int capteurs[8] = {36, 39, 34, 35, 32, 33, 25, 26};
int seuil = 500; // Valeur par défaut si la calibration échoue

// Tableaux pour la calibration
int capteurMin[8] = {4095, 4095, 4095, 4095, 4095, 4095, 4095, 4095};
int capteurMax[8] = {0, 0, 0, 0, 0, 0, 0, 0};

// Moteurs (TB6612FNG) - Canal A: Gauche / Canal B: Droit
const int moteurGauche_AV = 14; // AIN1
const int moteurGauche_AR = 4;  // AIN2
const int moteurGauche_EN = 13; // PWMA
const int moteurDroit_AV  = 16; // BIN1
const int moteurDroit_AR  = 17; // BIN2
const int moteurDroit_EN  = 23; // PWMB
const int STBY_PIN        = 27; // STBY

// ==========================================
// VARIABLES GLOBALES
// ==========================================
const int poids[8] = {0, -2500, -1500, -500, 500, 1500, 2500, 0};
int val[8]; 

// Compteurs et états de navigation
volatile int nb_traitG = 0;
volatile int nb_traitD = 0;
volatile int nb_inter  = 0;

bool L_perdue = false;       
bool L_perdue_05s = false;   
bool L_perdue_50ms = false;

int nbActifs = 0;
float dernierePositionConnue = 0.0f;

bool etatPrecLigneGauche   = false;
bool etatPrecLigneDroite   = false;
bool etatPrecIntersection = false;

bool virageFait0 = false;
bool virageFait1 = false;
bool virageFait2 = false;
bool virageFait3 = false;
bool virageFait4 = false;
bool virageFait5 = false;

bool demitourFait0 = false;
bool demitourFait1 = false;
bool demitourFait2 = false;

bool pauseFait0 = false;
bool pauseFaite1 = false;

bool departHorsPisteFait = false;

// ==========================================
// Variables temporelles
// ==========================================
int timerint = 0;
// ==========================================
// CONFIGURATION PID DIRECTION
// ==========================================
float Kp = 0.035f;
float Ki = 0.000f;
float Kd = 0.015f;

float consigne = 0.0f;
float erreurPrecedente = 0.0f;
float integrale = 0.0f;
unsigned long dernierTempsPID = 0;
const float LIMITE_INTEGRALE = 1000.0f;

// ==========================================
// PUISSANCE MOTEURS (BOUCLE OUVERTE)
// ==========================================
int cibleGauche = 0;
int cibleDroit  = 0;

// vitesseBase correspond maintenant directement au rapport cyclique PWM (0-255)
int vitesseBase = 60; 
float LIMITE_CORRECTION = 80;

// ==========================================
// SETUP
// ==========================================
void setup() {
  Serial.begin(115200);

  // Activation du driver moteur en mettant STBY à HIGH
  pinMode(STBY_PIN, OUTPUT);
  digitalWrite(STBY_PIN, HIGH);

  for (int i = 0; i < 8; i++) pinMode(capteurs[i], INPUT);

  pinMode(moteurGauche_AV, OUTPUT);
  pinMode(moteurGauche_AR, OUTPUT);
  pinMode(moteurDroit_AV,  OUTPUT);
  pinMode(moteurDroit_AR,  OUTPUT);

  ledcAttach(moteurGauche_EN, 1000, 8); // Résolution 8 bits (0-255)
  ledcAttach(moteurDroit_EN,  1000, 8);

  arreterMoteurs();
  dernierTempsPID = micros();
  calibrerCapteurs();
}

void calibrerCapteurs() {
  Serial.println("==========================================");
  Serial.println("DEBUT DE LA CALIBRATION (DUREE : 5 SECONDES)");
  Serial.println("=> Bougez le robot de gauche à droite sur la ligne !");
  Serial.println("==========================================");

  unsigned long tempsDebut = millis();
  
  while (millis() - tempsDebut < 5000) { 
    for (int i = 0; i < 8; i++) {
      int valeurActuelle = analogRead(capteurs[i]);
      if (valeurActuelle < capteurMin[i]) capteurMin[i] = valeurActuelle;
      if (valeurActuelle > capteurMax[i]) capteurMax[i] = valeurActuelle;
    }
    delay(10); 
  }

  int meilleurSeuilGlobal = 0;
  int capteursValides = 0;

  for (int i = 0; i < 8; i++) {
    int delta = capteurMax[i] - capteurMin[i];
    if (delta > 500) { 
      capteursValides++;
      int seuilCalculé = capteurMin[i] + (delta * 0.10f); 
      if (seuilCalculé > meilleurSeuilGlobal) {
        meilleurSeuilGlobal = seuilCalculé;
      }
    }
  }

  if (capteursValides >= 4) { 
    seuil = meilleurSeuilGlobal;
    Serial.print("CALIBRATION REUSSIE ! Nouveau seuil optimal : ");
    Serial.println(seuil);
  } else {
    Serial.print("ECHEC CALIBRATION (Pas assez de contraste). Seuil par défaut conservé : ");
    Serial.println(seuil);
  }
  Serial.println("==========================================");
}

// ==========================================
// FONCTIONS CAPTEURS & POSITION
// ==========================================
void lireCapteurs() {
  for (int i = 0; i < 8; i++) val[i] = analogRead(capteurs[i]);
}

float lirePosition() {
  long somme = 0;
  long somme_poids = 0;
  int actifs = 0;

  for (int i = 0; i < 8; i++) {
    if (val[i] > seuil) {
      somme       += (long)val[i] * poids[i];
      somme_poids += val[i];
      actifs++;
    }
  }
  if (actifs == 0 || somme_poids == 0) return dernierePositionConnue;
  dernierePositionConnue = (float)somme / (float)somme_poids;
  return dernierePositionConnue;
}

void afficherCapteurs(void) {
  for (int i = 0; i < 8; i++) {
    Serial.print(analogRead(capteurs[i]));
    Serial.print("\t");
  }
  float position2 = lirePosition();
  Serial.print(position2);
  Serial.println();
}

// ==========================================
// DÉTECTION ET COMPTAGE
// ==========================================
void detecterEtCompter() {
  static bool encoursLigneGauche = false;
  static bool encoursLigneDroite = false;
  static bool aVuIntersection   = false;

  static unsigned long tempsPerteLigne = 0;
  static bool enTrainDePerdre = false;

  nbActifs = 0;
  for (int i = 0; i < 8; i++) {
    if (val[i] > seuil) nbActifs++;
  }

  int actifsCoteDroit = 0;
  for (int i = 0; i <= 2; i++) { if (val[i] > seuil) actifsCoteDroit++; }

  int actifsCoteGauche = 0;
  for (int i = 5; i <= 7; i++) { if (val[i] > seuil) actifsCoteGauche++; }

  bool centreActif = (val[3] > seuil) || (val[4] > seuil);
  bool intersection = (nbActifs >= 7);
  
  bool ligneGauche = (!intersection) && (val[7] > seuil) && (actifsCoteGauche >= 1) && centreActif;  
  bool ligneDroite = (!intersection) && (val[0] > seuil) && (actifsCoteDroit  >= 1) && centreActif;  

  if (!intersection && (actifsCoteGauche >= 2) && (actifsCoteDroit >= 2)) {
    if (actifsCoteGauche > actifsCoteDroit) {
      ligneGauche = (val[7] > seuil);  
      ligneDroite = false;
    } else {
      ligneDroite = (val[0] > seuil);  
      ligneGauche = false;
    }
  }

  L_perdue = (nbActifs <= 0);

  if (L_perdue) {
    if (!enTrainDePerdre) {
      tempsPerteLigne = millis();
      enTrainDePerdre = true;
    }
    if (millis() - tempsPerteLigne > 50) {
      L_perdue_50ms = true;
    } else {
      L_perdue_50ms = false;
    }
    if (millis() - tempsPerteLigne > 500) {
      L_perdue_05s = true;
    } else {
      L_perdue_05s = false;
    }
  }
  else {
    enTrainDePerdre = false;
    L_perdue_05s = false;
    L_perdue_50ms = false;
  }

  if (ligneGauche && !etatPrecLigneGauche)   encoursLigneGauche = true;
  if (ligneDroite && !etatPrecLigneDroite)   encoursLigneDroite = true;
  if (intersection && !etatPrecIntersection) {
    nb_inter++;
    aVuIntersection = true;
  }

  if (!ligneGauche && etatPrecLigneGauche) {
    if (encoursLigneGauche && !aVuIntersection) nb_traitG++;
    encoursLigneGauche = false;
  }
  if (!ligneDroite && etatPrecLigneDroite) {
    if (encoursLigneDroite && !aVuIntersection) nb_traitD++;
    encoursLigneDroite = false;
  }

  bool ligneNormale = (actifsCoteGauche == 0) && (actifsCoteDroit == 0) && centreActif;
  if (ligneNormale || L_perdue) {
    aVuIntersection   = false;
    encoursLigneGauche = false;
    encoursLigneDroite = false;
  }

  etatPrecLigneGauche  = ligneGauche;
  etatPrecLigneDroite  = ligneDroite;
  etatPrecIntersection = intersection;
}

// ==========================================
// LECTURE DES COMMANDES SÉRIE (PID)
// ==========================================
void lireCommandes() {
  static char buf[32];
  static int idx = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[idx] = '\0';
      if (idx > 1) {
        float v = atof(buf + 1);
        switch (tolower(buf[0])) {
          case 'p': Kp = v; break;
          case 'i': Ki = v; break;
          case 'd': Kd = v; break;
        }
      }
      idx = 0;
    } else if (idx < 31) {
      buf[idx++] = c;
    }
  }
}

// ==========================================
// CALCULS DE NAVIGATION (PID & APPLICATION PWM)
// ==========================================
float calculerPID(float position) {
  unsigned long maintenant = micros();
  float dt = (maintenant - dernierTempsPID) / 1000000.0f;
  dernierTempsPID = maintenant;

  if (dt <= 0.0f) return 0.0f;

  float erreur = consigne - position;
  float P = Kp * erreur;

  integrale += erreur * dt;
  integrale = constrain(integrale, -LIMITE_INTEGRALE, LIMITE_INTEGRALE);
  float I = Ki * integrale;

  float D = Kd * (erreur - erreurPrecedente);
  erreurPrecedente = erreur;

  return constrain(P + I + D, -LIMITE_CORRECTION, LIMITE_CORRECTION);
}

void definirCibles(int cG, int cD) {
  // On limite le PWM entre 0 et 255
  cibleGauche = constrain(cG, 0, 255);
  cibleDroit  = constrain(cD, 0, 255);
}

// Sans les encodeurs, cette fonction se contente d'appliquer directement le PWM aux moteurs
void regulerVitesse() {
  if (cibleGauche == 0 && cibleDroit == 0) {
    ledcWrite(moteurGauche_EN, 0);
    ledcWrite(moteurDroit_EN,  0);
    return;
  }

  digitalWrite(moteurGauche_AV, HIGH);
  digitalWrite(moteurGauche_AR, LOW);
  digitalWrite(moteurDroit_AV,  HIGH);
  digitalWrite(moteurDroit_AR,  LOW);

  ledcWrite(moteurGauche_EN, cibleGauche);
  ledcWrite(moteurDroit_EN,  cibleDroit);
}

void arreterMoteurs() {
  cibleGauche = 0;
  cibleDroit  = 0;
  ledcWrite(moteurGauche_EN, 0);
  ledcWrite(moteurDroit_EN,  0);
  digitalWrite(moteurGauche_AV, LOW);
  digitalWrite(moteurGauche_AR, LOW);
  digitalWrite(moteurDroit_AV,  LOW);
  digitalWrite(moteurDroit_AR,  LOW);
}

// ==========================================
// MANOEUVRES
// ==========================================
void SortirLigneGauche() {
  definirCibles(90, 150); // Ajusté pour le PWM
  unsigned long t0 = millis();
  while (millis() - t0 < 1400) {
    regulerVitesse();
    delay(2);
  }
  while (true) {
    if (millis() - t0 > 4000) break; 
    lireCapteurs();
    if (val[3] > seuil || val[4] > seuil) break;
    regulerVitesse();
    delay(2);
  }
  arreterMoteurs();
  delay(100);
}

void Demitour() {
  ledcWrite(moteurGauche_EN, 0);
  ledcWrite(moteurDroit_EN, 0);
  delay(200);

  // Rotation sur place
  digitalWrite(moteurGauche_AV, LOW);
  digitalWrite(moteurGauche_AR, HIGH);
  digitalWrite(moteurDroit_AV,  HIGH);
  digitalWrite(moteurDroit_AR,  LOW);
  
  ledcWrite(moteurGauche_EN, 60);
  ledcWrite(moteurDroit_EN, 60);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT = 4000; 
  
  // Phase 1 : Rotation à l'aveugle basée sur le TEMPS (Ajustez cette valeur !)
  const unsigned long TEMPS_ROTATION_PARTIEL = 400; // ms
  while (millis() - t0 < TEMPS_ROTATION_PARTIEL) {
    // Attente simple
  }

  // Phase 2 : Finition aux capteurs
  while (true) {
    if (millis() - t0 > TIMEOUT) break;
    lireCapteurs(); 
    if (val[3] > seuil || val[4] > seuil) {
      break; 
    }
  }

  ledcWrite(moteurGauche_EN, 0);
  ledcWrite(moteurDroit_EN, 0);
  delay(100); 
}

void VirageG() {
  ledcWrite(moteurGauche_EN, 0);
  ledcWrite(moteurDroit_EN, 0);
  delay(200);

  digitalWrite(moteurGauche_AV, HIGH);
  digitalWrite(moteurGauche_AR, LOW);
  digitalWrite(moteurDroit_AV,  HIGH);
  digitalWrite(moteurDroit_AR,  LOW);
  
  ledcWrite(moteurGauche_EN, 0);
  ledcWrite(moteurDroit_EN, 120);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT = 3000;
  
  // Phase 1 : Rotation temporelle (Ajustez cette valeur !)
  const unsigned long TEMPS_ROTATION_PARTIEL = 400; // ms
  while (millis() - t0 < TEMPS_ROTATION_PARTIEL) {
    // Attente simple
  }

  while (true) {
    if (millis() - t0 > TIMEOUT) break; 
    lireCapteurs(); 
    if (val[3] > seuil || val[4] > seuil) {
      break; 
    }
  }

  ledcWrite(moteurGauche_EN, 0);
  ledcWrite(moteurDroit_EN, 0);
  delay(100); 
}

void VirageD() {
  ledcWrite(moteurGauche_EN, 0);
  ledcWrite(moteurDroit_EN, 0);
  delay(200);

  digitalWrite(moteurGauche_AV, HIGH);
  digitalWrite(moteurGauche_AR, LOW);
  digitalWrite(moteurDroit_AV,  HIGH);
  digitalWrite(moteurDroit_AR,  LOW);
  
  ledcWrite(moteurGauche_EN, 120);
  ledcWrite(moteurDroit_EN, 0);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT = 3000;
  
  // Phase 1 : Rotation temporelle (Ajustez cette valeur !)
  const unsigned long TEMPS_ROTATION_PARTIEL = 400; // ms
  while (millis() - t0 < TEMPS_ROTATION_PARTIEL) {
    // Attente simple
  }

  while (true) {
    if (millis() - t0 > TIMEOUT) break;
    lireCapteurs(); 
    if (val[3] > seuil || val[4] > seuil) {
      break; 
    }
  }

  ledcWrite(moteurGauche_EN, 0);
  ledcWrite(moteurDroit_EN, 0);
  delay(100); 
}

void PousserBarriere() {
  definirCibles(vitesseBase, vitesseBase);
  unsigned long t0 = millis();
  while (millis() - t0 < 500) {
    regulerVitesse();
    delay(2);
  }
  arreterMoteurs();
  while(true) { delay(1000); } 
}

// Changements de vitesse globale (Ajustés pour des valeurs PWM directes)
void setmodeVR(){ vitesseBase = 120;  LIMITE_CORRECTION = vitesseBase - 2; }
void setmodeVM(){ vitesseBase = 90;   LIMITE_CORRECTION = vitesseBase - 2; }
void setmodeVL(){ vitesseBase = 60;   LIMITE_CORRECTION = vitesseBase - 2; }
void wait(){ delay(2000); }

void pause(unsigned long tpause) 
{
  int i = 0;
  int vitesseBaseprec = vitesseBase;
    unsigned long debut_pause = millis();
    while( millis() - debut_pause < tpause)
    {
      for(i=0;i<=10;i++){
        vitesseBase = max(0, vitesseBase - 10);  LIMITE_CORRECTION = vitesseBase;
        lireCapteurs();
        lireCommandes();
        
        float position = lirePosition();
        detecterEtCompter();

        float correction = calculerPID(position);
        int vitesseD = vitesseBase - round(correction);
        int vitesseG = vitesseBase + round(correction);
        definirCibles(vitesseG, vitesseD);
        
        regulerVitesse();
        delay(2);
      }
      
      arreterMoteurs();
    }
    vitesseBase = vitesseBaseprec;  LIMITE_CORRECTION = vitesseBase - 2;
}

void departHorsPiste() {
  definirCibles(vitesseBase, vitesseBase);
  unsigned long t0 = millis();
  while (millis() - t0 < 500) {
    regulerVitesse();
    delay(2);
  }
  arreterMoteurs();
}

void marcheArriere() {
  ledcWrite(moteurGauche_EN, 0);
  ledcWrite(moteurDroit_EN, 0);
  delay(100);

  digitalWrite(moteurGauche_AV, LOW);
  digitalWrite(moteurGauche_AR, HIGH);
  digitalWrite(moteurDroit_AV,  LOW);
  digitalWrite(moteurDroit_AR,  HIGH);

  ledcWrite(moteurGauche_EN, 90); 
  ledcWrite(moteurDroit_EN, 90);

  unsigned long t0 = millis();
  const unsigned long TIMEOUT = 3000; 
  while (true) {
    if (millis() - t0 > TIMEOUT) break;
  }

  arreterMoteurs();
}

// ==========================================
// BOUCLE PRINCIPALE
// ==========================================
void loop() {
  lireCapteurs();
  lireCommandes();
  
  float position = lirePosition();
  detecterEtCompter(); 

  // Calcul PID et ajustement des vitesses moteurs
  float correction = calculerPID(position);
  int vitesseD = vitesseBase - round(correction);
  int vitesseG = vitesseBase + round(correction);
  definirCibles(vitesseG, vitesseD);

  if (nb_traitG == 1 && !demitourFait0){
    timerint = millis();
    demitourFait0 = true;
  }
  if(demitourFait0 && !pauseFaite1){
    if(millis() - timerint >= 2000){      
        Demitour();
        pauseFaite1 = true;
    }
  }
  if (L_perdue_50ms ){
    PousserBarriere();
  }

  // 2. Comportement lié à la ligne
  else {
    // Sinon (la ligne est toujours là)
    regulerVitesse();
  } 
  delay(2); 
}
/*
 * GSA_TEMPERATURE v2 - tableau de bord pour Citroën GSA
 *
 * Matériel
 *   - Arduino Uno / Nano (ATmega328P)
 *   - 3 sondes DS18B20 sur D7 (résistance de tirage 4,7 kOhm vers 5 V)
 *   - RTC DS3231 en I2C (adresse 0x68)
 *   - écran OLED SH1106 128x64 en I2C (adresse 0x3C)
 *   - 2 boutons poussoirs : D2 vers GND (heures), D3 vers GND (minutes),
 *     100 nF en parallèle sur chaque bouton conseillés
 *   - option tension batterie : pont diviseur 33 kOhm (côté 12 V) / 10 kOhm (côté GND) vers A0,
 *     100 nF entre A0 et GND ; prendre le 12 V avant la diode anti-inversion
 *   - option pression d'huile : capteur 0,5-4,5 V alimenté par le 5 V de l'Arduino, sortie sur A1
 *
 * Bibliothèques (gestionnaire de bibliothèques de l'IDE Arduino)
 *   OneWire, DallasTemperature, U8g2, Time (Paul Stoffregen),
 *   DS1307RTC (compatible DS3231), Timezone (Jack Christensen)
 *
 * Utilisation
 *   - Boutons : un appui = +1 h ou +1 min, maintenu = défilement rapide
 *     (les minutes ne changent pas l'heure, et remettent les secondes à 0).
 *   - Port série à 115200 bauds : T suivi de l'heure Unix UTC (commande date +%s) règle heure et date.
 *   - Une alerte (surchauffe, pression d'huile, tension) fait clignoter la valeur concernée.
 *   La RTC est réglée en UTC ; le passage heure d'été / heure d'hiver est automatique.
 */

#include <Wire.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>
#include <TimeLib.h>
#include <DS1307RTC.h>
#include <Timezone.h>
#include "bitmaps.h"

// ============================================================ configuration

#define RTC_EST_DS3231        1   // 1 avec un DS3231, 0 avec un DS1307
#define OPTION_TENSION        1   // 0 tant que le pont diviseur n'est pas câblé (A0 en l'air = fausses alertes)
#define OPTION_PRESSION_HUILE 0   // 1 une fois le capteur de pression monté

const uint8_t PIN_ONEWIRE    = 7;
const uint8_t PIN_BOUTON_HEURES  = 2;
const uint8_t PIN_BOUTON_MINUTES = 3;
const uint8_t PIN_OLED_RESET = 6;    // comme l'ancien code ; U8X8_PIN_NONE si RST n'est pas câblé
const uint8_t PIN_TENSION    = A0;
const uint8_t PIN_PRESSION   = A1;

// Adresses des sondes : les relever avec le programme IDENTIFIER_SONDES (chauffer une sonde à la fois).
// Une sonde absente ou à l'adresse inconnue s'affiche "--" ; les autres ne sont pas décalées.
DeviceAddress SONDE_MOTEUR = { 0x28, 0x2B, 0x3F, 0x57, 0x04, 0xA0, 0x3C, 0x88 };
DeviceAddress SONDE_EXT    = { 0x28, 0xE4, 0x7A, 0x57, 0x04, 0xD6, 0x3C, 0x14 };
DeviceAddress SONDE_INT    = { 0x28, 0x1C, 0x27, 0x57, 0x04, 0xE1, 0x3D, 0xA3 };

const int16_t SEUIL_CACHE_CALANDRE = 6;     // °C extérieur au démarrage
const int16_t SEUIL_ALERTE_MOTEUR  = 110;   // °C, à ajuster selon l'emplacement de la sonde (DS18B20 : 125 °C max)
const uint8_t RESOLUTION_SONDES    = 10;    // 0,25 °C, conversion en 187,5 ms

// Tension : la mesure est comparée à la référence interne de 1,1 V de l'ATmega328P,
// elle ne dépend donc pas du 5 V réel (USB ou abaisseur).
// Théorique : 1100 mV x (33 + 10) / 10 = 4730. La référence interne varie de 1,0 à 1,2 V selon la puce,
// étalonner une fois au multimètre : nouvelle valeur = valeur actuelle x tension multimètre / tension affichée
const uint32_t CONSTANTE_TENSION_MV = 4570;   // étalonné sur ma Nano (théorique : 4730)
const int16_t  SEUIL_TENSION_HAUTE_DV    = 148;   // 14,8 V : surcharge (régulateur)
const int16_t  SEUIL_TENSION_BASSE_DV    = 118;   // 11,8 V : batterie faible
const int16_t  SEUIL_MOTEUR_TOURNE_DV    = 132;   // au-dessus, l'alternateur charge : le moteur tourne
const uint8_t  DUREE_ALERTE_TENSION_S    = 5;     // ignore le creux de tension du démarreur

// Pression d'huile : capteur 0,5 V = 0 bar, 4,5 V = PRESSION_MAX_DIX
const int16_t PRESSION_MAX_DIX         = 100;   // 10,0 bar, selon le capteur
const int16_t SEUIL_PRESSION_BASSE_DIX = 5;     // 0,5 bar, à ajuster
const uint8_t DUREE_ALERTE_PRESSION_S  = 2;

const unsigned long DUREE_SPLASH_MS         = 5000;
const unsigned long DUREE_CALANDRE_MS       = 4000;
const unsigned long PERIODE_MESURE_MS       = 1000;
const unsigned long PERIODE_CLIGNOT_MS      = 500;
const unsigned long ANTI_REBOND_MS          = 50;
const unsigned long DELAI_REPETITION_MS     = 600;     // bouton maintenu : début du défilement
const unsigned long PERIODE_REPETITION_MS   = 200;     // puis un pas toutes les 200 ms

// Heure légale française : été le dernier dimanche de mars à 2 h, hiver le dernier dimanche d'octobre à 3 h
TimeChangeRule CEST = { "CEST", Last, Sun, Mar, 2, 120 };
TimeChangeRule CET  = { "CET",  Last, Sun, Oct, 3, 60 };
Timezone heureFrance(CEST, CET);

// ============================================================ objets et état

U8G2_SH1106_128X64_NONAME_2_HW_I2C ecran(U8G2_R0, PIN_OLED_RESET);
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sondes(&oneWire);

const int16_t VALEUR_INVALIDE = -999;
int16_t tMoteur     = VALEUR_INVALIDE;
int16_t tExt        = VALEUR_INVALIDE;
int16_t tInt        = VALEUR_INVALIDE;
int16_t tensionDV   = VALEUR_INVALIDE;   // dixièmes de volt
int16_t pressionDix = VALEUR_INVALIDE;   // dixièmes de bar
bool alerteTensionActive  = false;
bool alertePressionActive = false;
time_t heureLocale = 0;   // 0 = heure inconnue
bool aRedessiner = true;

// Défini ici, avant toute fonction : l'IDE Arduino insère ses prototypes automatiques devant la première fonction
struct Bouton {
  uint8_t pin;
  bool dernierLu;
  unsigned long instantDernierLu;
  bool appuye;
  unsigned long prochaineRepetition;
};

Bouton boutonHeures  = { PIN_BOUTON_HEURES,  false, 0, false, 0 };
Bouton boutonMinutes = { PIN_BOUTON_MINUTES, false, 0, false, 0 };

// ============================================================ RTC

const uint8_t RTC_ADRESSE    = 0x68;
const uint8_t RTC_REG_STATUT = 0x0F;   // DS3231, bit 7 = OSF (l'oscillateur s'est arrêté)

int16_t rtcLireStatut() {
  Wire.beginTransmission(RTC_ADRESSE);
  Wire.write(RTC_REG_STATUT);
  if (Wire.endTransmission() != 0) return -1;
  if (Wire.requestFrom(RTC_ADRESSE, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

// Vrai si l'heure stockée n'est pas fiable (RTC absente ou oscillateur arrêté depuis le dernier réglage)
bool rtcHeurePerdue() {
#if RTC_EST_DS3231
  int16_t statut = rtcLireStatut();
  return statut < 0 || (statut & 0x80);
#else
  return false;
#endif
}

void rtcRegler(time_t utc) {
  RTC.set(utc);
#if RTC_EST_DS3231
  int16_t statut = rtcLireStatut();   // efface OSF : l'heure est de nouveau valide
  if (statut >= 0) {
    Wire.beginTransmission(RTC_ADRESSE);
    Wire.write(RTC_REG_STATUT);
    Wire.write((uint8_t)(statut & ~0x80));
    Wire.endTransmission();
  }
#endif
}

void lireHeure() {
  time_t utc = rtcHeurePerdue() ? 0 : RTC.get();   // RTC.get() renvoie 0 si la lecture échoue
  heureLocale = (utc == 0) ? 0 : heureFrance.toLocal(utc);
}

// ============================================================ mesures

int16_t lireSonde(const uint8_t *adresse) {
  float t = sondes.getTempC(adresse);
  if (t <= DEVICE_DISCONNECTED_C) return VALEUR_INVALIDE;   // débranchée ou en défaut
  return (int16_t)lround(t);
}

// Lit le résultat de la conversion précédente puis en relance une (non bloquant)
void lireSondes() {
  tMoteur = lireSonde(SONDE_MOTEUR);
  tExt    = lireSonde(SONDE_EXT);
  tInt    = lireSonde(SONDE_INT);
  sondes.requestTemperatures();
}

// Somme de 16 lectures pour lisser les parasites de l'allumage (la première, après un changement d'entrée, est ignorée)
uint16_t lireAnalogiqueSomme(uint8_t pin) {
  analogRead(pin);
  uint16_t somme = 0;
  for (uint8_t i = 0; i < 16; i++) somme += analogRead(pin);
  return somme;
}

#if OPTION_TENSION
// Somme de 16 lectures de la référence interne de 1,1 V, mesurée par rapport au 5 V
uint16_t lireReferenceInterneSomme() {
  ADMUX = _BV(REFS0) | 0x0E;   // référence AVcc, entrée = référence interne 1,1 V
  delay(2);                    // stabilisation de la référence interne
  uint16_t somme = 0;
  for (uint8_t i = 0; i < 17; i++) {
    ADCSRA |= _BV(ADSC);
    while (ADCSRA & _BV(ADSC));
    if (i > 0) somme += ADC;   // première conversion ignorée
  }
  return somme;
}
#endif

// Médiane de trois valeurs : élimine un pic isolé sans retarder les vraies variations
int16_t mediane3(int16_t a, int16_t b, int16_t c) {
  if (a > b) {
    int16_t t = a;
    a = b;
    b = t;
  }
  if (b > c) b = c;
  return a > b ? a : b;
}

void lireTension() {
#if OPTION_TENSION
  int16_t mesure = VALEUR_INVALIDE;
  uint16_t reference = lireReferenceInterneSomme();
  if (reference >= 1000) {     // sinon lecture aberrante
    uint32_t mv = (uint32_t)lireAnalogiqueSomme(PIN_TENSION) * CONSTANTE_TENSION_MV / reference;
    mesure = (int16_t)((mv + 50) / 100);
  }
  static int16_t precedente1 = VALEUR_INVALIDE, precedente2 = VALEUR_INVALIDE;
  if (precedente1 == VALEUR_INVALIDE) precedente1 = precedente2 = mesure;   // première mesure
  tensionDV = mediane3(mesure, precedente1, precedente2);
  precedente2 = precedente1;
  precedente1 = mesure;
#endif
}

void lirePression() {
#if OPTION_PRESSION_HUILE
  int16_t brut = lireAnalogiqueSomme(PIN_PRESSION) / 16;   // 0,5 V = 102, 4,5 V = 921 (capteur proportionnel au 5 V)
  if (brut < 51 || brut > 972) {                       // hors 0,25-4,75 V : débranché ou en court-circuit
    pressionDix = VALEUR_INVALIDE;
    return;
  }
  int32_t p = (int32_t)(brut - 102) * PRESSION_MAX_DIX / (921 - 102);
  pressionDix = (int16_t)(p < 0 ? 0 : p);
#endif
}

bool alerteMoteur() {
  return tMoteur != VALEUR_INVALIDE && tMoteur >= SEUIL_ALERTE_MOTEUR;
}

#if OPTION_PRESSION_HUILE
bool moteurTourne() {
#if OPTION_TENSION
  return tensionDV >= SEUIL_MOTEUR_TOURNE_DV;
#else
  return true;   // sans mesure de tension, on alerte contact mis, comme le témoin d'origine
#endif
}
#endif

// Appelée une fois par seconde : une alerte ne se déclenche qu'après plusieurs secondes hors limites
void mettreAJourAlertes() {
#if OPTION_TENSION
  static uint8_t secondesTension = 0;
  bool horsLimites = tensionDV >= SEUIL_TENSION_HAUTE_DV || tensionDV <= SEUIL_TENSION_BASSE_DV;
  if (!horsLimites) secondesTension = 0;
  else if (secondesTension < 255) secondesTension++;
  alerteTensionActive = secondesTension >= DUREE_ALERTE_TENSION_S;
#endif
#if OPTION_PRESSION_HUILE
  static uint8_t secondesPression = 0;
  bool basse = pressionDix != VALEUR_INVALIDE && pressionDix < SEUIL_PRESSION_BASSE_DIX && moteurTourne();
  if (!basse) secondesPression = 0;
  else if (secondesPression < 255) secondesPression++;
  alertePressionActive = secondesPression >= DUREE_ALERTE_PRESSION_S;
#endif
}

// ============================================================ affichage
// Mode "page" de U8g2 : tampon de 256 octets au lieu de 1 Ko (l'ATmega328P n'a que 2 Ko de RAM).
// Le dessin est rejoué pour chaque bande de l'écran dans la boucle firstPage() / nextPage().

void afficherImage(const uint8_t *image) {
  ecran.firstPage();
  do {
    ecran.drawXBMP(0, 0, 128, 64, image);
  } while (ecran.nextPage());
}

// Formatage sans snprintf (économise environ 1,2 Ko de flash)
char *deuxChiffres(char *p, uint8_t v) {
  p[0] = '0' + v / 10;
  p[1] = '0' + v % 10;
  p[2] = '\0';
  return p + 2;
}

void texteHeureMinute(char *texte, time_t t) {
  char *p = deuxChiffres(texte, hour(t));
  *p++ = ':';
  deuxChiffres(p, minute(t));
}

void texteEntier(char *texte, int16_t v) {
  if (v == VALEUR_INVALIDE) strcpy(texte, "--");
  else itoa(v, texte, 10);
}

void texteDixiemes(char *texte, int16_t v) {   // valeurs positives uniquement
  if (v == VALEUR_INVALIDE) {
    strcpy(texte, "-.-");
    return;
  }
  itoa(v / 10, texte, 10);
  char *p = texte + strlen(texte);
  *p++ = '.';
  *p++ = '0' + v % 10;
  *p = '\0';
}

void centrer(const char *texte, u8g2_uint_t y) {
  ecran.drawStr((128 - ecran.getStrWidth(texte)) / 2, y, texte);
}

// Texte suivi d'un petit cercle (degré)
void dessinerAvecDegre(u8g2_uint_t x, u8g2_uint_t y, const char *texte, uint8_t hauteurChiffres, uint8_t rayon) {
  x += ecran.drawStr(x, y, texte);
  ecran.drawCircle(x + rayon + 2, y - hauteurChiffres + rayon, rayon);
}

// Grande valeur (logisoso22) suivie d'une unité en petit (helvB10), centrées entre x0 et x0 + largeur
void dessinerAvecUnite(u8g2_uint_t x0, u8g2_uint_t largeur, u8g2_uint_t y, const char *texte, const char *unite) {
  ecran.setFont(u8g2_font_logisoso22_tn);
  u8g2_uint_t l = ecran.getStrWidth(texte);
  ecran.setFont(u8g2_font_helvB10_tr);
  u8g2_uint_t x = x0 + (largeur - (l + 2 + ecran.getStrWidth(unite))) / 2;
  ecran.drawStr(x + l + 2, y, unite);
  ecran.setFont(u8g2_font_logisoso22_tn);
  ecran.drawStr(x, y, texte);
}

// "V" de 10 px de large à la hauteur des chiffres de logisoso16 (la version numérique de la police n'a pas de lettres)
void dessinerV(u8g2_uint_t x, u8g2_uint_t y) {
  for (uint8_t i = 0; i < 2; i++) {
    ecran.drawLine(x + i, y - 16, x + 4 + i, y - 1);
    ecran.drawLine(x + 9 - i, y - 16, x + 5 - i, y - 1);
  }
}

// Heure "HH:MM". Les deux-points clignotent : une demi-seconde sur deux, ils sont dessinés en noir (invisibles),
// ce qui laisse les minutes exactement à la même place
void dessinerHeure(u8g2_uint_t x, u8g2_uint_t y, const char *texteHeure, bool deuxPoints) {
  char heures[3] = { texteHeure[0], texteHeure[1], '\0' };
  x += ecran.drawStr(x, y, heures);
  if (!deuxPoints) ecran.setDrawColor(0);
  x += ecran.drawStr(x, y, ":");
  ecran.setDrawColor(1);
  ecran.drawStr(x, y, texteHeure + 3);
}

void dessinerEcranPrincipal(bool phase, const char *texteHeure) {
  bool deuxPoints = phase || heureLocale == 0;   // fixes si l'heure est inconnue
  char texte[6];

  // Ligne 1 : moteur (et pression d'huile en option), la zone en alerte s'inverse et clignote
  ecran.setFont(u8g2_font_logisoso22_tn);
  texteEntier(texte, tMoteur);
#if OPTION_PRESSION_HUILE
  if (alerteMoteur() && phase) {
    ecran.drawBox(0, 0, 64, 27);
    ecran.setDrawColor(0);
  }
  dessinerAvecDegre((64 - (ecran.getStrWidth(texte) + 9)) / 2, 25, texte, 22, 3);
  ecran.setDrawColor(1);

  if (alertePressionActive && phase) {
    ecran.drawBox(64, 0, 64, 27);
    ecran.setDrawColor(0);
  }
  if (pressionDix != VALEUR_INVALIDE && pressionDix >= 100) texteEntier(texte, pressionDix / 10);   // 10 bar et plus : sans décimale
  else texteDixiemes(texte, pressionDix);
  dessinerAvecUnite(64, 64, 25, texte, "bar");
  ecran.setDrawColor(1);
#else
  if (alerteMoteur() && phase) {
    ecran.drawBox(0, 0, 128, 27);
    ecran.setDrawColor(0);
  }
  ecran.drawXBMP(0, 2, 40, 23, bmp_moteur);
  dessinerAvecDegre(52, 25, texte, 22, 3);
  ecran.setDrawColor(1);
#endif

  // Ligne 2 : intérieur et extérieur
  ecran.setFont(u8g2_font_helvB10_tr);
  u8g2_uint_t x = ecran.drawStr(0, 42, "IN");
  texteEntier(texte, tInt);
  dessinerAvecDegre(x + 4, 42, texte, 11, 2);
  x = 64 + ecran.drawStr(64, 42, "OUT");
  texteEntier(texte, tExt);
  dessinerAvecDegre(x + 4, 42, texte, 11, 2);

  // Ligne 3 : heure, et tension batterie à droite si elle est mesurée
  ecran.setFont(u8g2_font_logisoso16_tn);
#if OPTION_TENSION
  dessinerHeure(1, 64, texteHeure, deuxPoints);
  texteDixiemes(texte, tensionDV);
  u8g2_uint_t l = ecran.getStrWidth(texte);
  x = 126 - (l + 2 + 10);
  if (alerteTensionActive && phase) {
    ecran.drawBox(x - 3, 46, 128 - (x - 3), 18);
    ecran.setDrawColor(0);
  }
  ecran.drawStr(x, 64, texte);
  dessinerV(x + l + 2, 64);
  ecran.setDrawColor(1);
#else
  dessinerHeure((128 - ecran.getStrWidth(texteHeure)) / 2, 64, texteHeure, deuxPoints);
#endif
}

void dessinerEcran(bool phase) {
  char texteHeure[6];
  if (heureLocale == 0) strcpy(texteHeure, "--:--");
  else texteHeureMinute(texteHeure, heureLocale);

  ecran.firstPage();
  do {
    ecran.setFontMode(1);
    ecran.setDrawColor(1);
    dessinerEcranPrincipal(phase, texteHeure);
  } while (ecran.nextPage());
}

// ============================================================ boutons

// Renvoie vrai au moment de l'appui, puis régulièrement tant que le bouton reste enfoncé.
// L'état doit rester stable ANTI_REBOND_MS avant d'être pris en compte (rebonds et parasites).
bool boutonDeclenche(Bouton &b, unsigned long maintenant) {
  bool lu = (digitalRead(b.pin) == LOW);
  if (lu != b.dernierLu) {
    b.dernierLu = lu;
    b.instantDernierLu = maintenant;
  }
  if (maintenant - b.instantDernierLu < ANTI_REBOND_MS) return false;

  if (!lu) {
    b.appuye = false;
    return false;
  }
  if (!b.appuye) {
    b.appuye = true;
    b.prochaineRepetition = maintenant + DELAI_REPETITION_MS;
    return true;
  }
  if ((long)(maintenant - b.prochaineRepetition) >= 0) {
    b.prochaineRepetition += PERIODE_REPETITION_MS;
    return true;
  }
  return false;
}

// Ajuste l'heure locale en gardant la date : les minutes bouclent dans l'heure, les heures dans la journée,
// les secondes repartent de 0. Fonctionne aussi si l'heure était perdue.
void ajusterHeure(uint8_t plusMinutes, uint8_t plusHeures) {
  time_t utc = RTC.get();
  if (utc == 0) return;   // RTC absente
  tmElements_t tm;
  breakTime(heureFrance.toLocal(utc), tm);
  tm.Minute = (tm.Minute + plusMinutes) % 60;
  tm.Hour   = (tm.Hour + plusHeures) % 24;
  tm.Second = 0;
  rtcRegler(heureFrance.toUTC(makeTime(tm)));
  lireHeure();
  aRedessiner = true;
}

// ============================================================ port série

void ecrireHeureSerie(time_t t) {
  char texte[9];
  texteHeureMinute(texte, t);
  texte[5] = ':';
  deuxChiffres(texte + 6, second(t));
  Serial.print(texte);
}

void gererSerie() {
  if (!Serial.available()) return;
  if (Serial.read() != 'T') return;
  long utc = Serial.parseInt();
  if (utc < 1700000000L) {
    Serial.println(F("Format : T suivi de l'heure Unix UTC (date +%s)"));
    return;
  }
  rtcRegler((time_t)utc);
  lireHeure();
  aRedessiner = true;
  Serial.print(F("Heure reglee : "));
  ecrireHeureSerie(heureLocale);
  Serial.println();
}

void ecrireValeurSerie(const __FlashStringHelper *nom, int16_t v, bool dixiemes) {
  Serial.print(nom);
  if (v == VALEUR_INVALIDE) Serial.print(F("--"));
  else if (!dixiemes) Serial.print(v);
  else {
    Serial.print(v / 10);
    Serial.print('.');
    Serial.print(v % 10);
  }
}

void journalSerie() {
  ecrireValeurSerie(F("Moteur "), tMoteur, false);
  ecrireValeurSerie(F(" | Int "), tInt, false);
  ecrireValeurSerie(F(" | Ext "), tExt, false);
#if OPTION_TENSION
  ecrireValeurSerie(F(" | Batt "), tensionDV, true);
#endif
#if OPTION_PRESSION_HUILE
  ecrireValeurSerie(F(" | Huile "), pressionDix, true);
#endif
  Serial.print(F(" | "));
  if (heureLocale == 0) Serial.print(F("heure inconnue"));
  else ecrireHeureSerie(heureLocale);
  Serial.println();
}

// Indique au démarrage quels modules I2C répondent (aide au câblage)
void verifierI2C() {
  const uint8_t adresses[] = { 0x3C, RTC_ADRESSE };
  for (uint8_t i = 0; i < 2; i++) {
    Wire.beginTransmission(adresses[i]);
    bool present = (Wire.endTransmission() == 0);
    Serial.print(i == 0 ? F("Ecran (0x3C) : ") : F("Horloge (0x68) : "));
    Serial.println(present ? F("OK") : F("ne repond pas"));
  }
  if (Wire.getWireTimeoutFlag()) {
    Wire.clearWireTimeoutFlag();
    Serial.println(F("! Bus I2C bloque : verifier SDA/SCL, VCC/GND et les soudures"));
  }
}

void listerSondes() {
  uint8_t n = sondes.getDeviceCount();
  Serial.print(n);
  Serial.println(F(" sonde(s) detectee(s) :"));
  DeviceAddress adr;
  for (uint8_t i = 0; i < n; i++) {
    if (!sondes.getAddress(adr, i)) continue;
    Serial.print(F("  { "));
    for (uint8_t j = 0; j < 8; j++) {
      Serial.print(F("0x"));
      if (adr[j] < 0x10) Serial.print('0');
      Serial.print(adr[j], HEX);
      if (j < 7) Serial.print(F(", "));
    }
    Serial.println(F(" }"));
  }
  if (!sondes.isConnected(SONDE_MOTEUR)) Serial.println(F("! Sonde moteur introuvable"));
  if (!sondes.isConnected(SONDE_EXT))    Serial.println(F("! Sonde exterieure introuvable"));
  if (!sondes.isConnected(SONDE_INT))    Serial.println(F("! Sonde interieure introuvable"));
}

// ============================================================ programme

void setup() {
  MCUSR = 0;          // désarme un éventuel watchdog resté actif après un reset
  wdt_disable();

  Serial.begin(115200);
  Serial.setTimeout(100);
  pinMode(PIN_BOUTON_HEURES, INPUT_PULLUP);
  pinMode(PIN_BOUTON_MINUTES, INPUT_PULLUP);

  // Timeout I2C activé avant l'écran : un module mal câblé ne doit pas figer la Nano au démarrage
  Wire.begin();
  Wire.setWireTimeout(25000, true);
  verifierI2C();

  ecran.setBusClock(100000);          // 100 kHz : plus robuste avec des fils longs, compatible DS1307
  ecran.begin();

  afficherImage(bmp_splash);

  sondes.begin();
  sondes.setResolution(RESOLUTION_SONDES);
  sondes.setWaitForConversion(false);
  sondes.requestTemperatures();       // la conversion se fait pendant le splash
  listerSondes();

  delay(DUREE_SPLASH_MS);
  lireSondes();

  if (tExt != VALEUR_INVALIDE && tExt <= SEUIL_CACHE_CALANDRE) {
    afficherImage(bmp_cache_calandre);
    delay(DUREE_CALANDRE_MS);
  }

  if (rtcHeurePerdue()) Serial.println(F("! Heure perdue : la regler avec les boutons ou la commande T"));
  lireHeure();
  lireTension();
  lirePression();

  wdt_enable(WDTO_8S);
}

void loop() {
  wdt_reset();
  unsigned long maintenant = millis();

  static unsigned long derniereMesure = 0;
  if (maintenant - derniereMesure >= PERIODE_MESURE_MS) {
    derniereMesure = maintenant;
    lireSondes();
    lireTension();
    lirePression();
    mettreAJourAlertes();
    lireHeure();
    journalSerie();
    aRedessiner = true;
  }

  static bool phasePrecedente = false;
  bool phase = (maintenant / PERIODE_CLIGNOT_MS) % 2;
  if (phase != phasePrecedente) {
    phasePrecedente = phase;
    aRedessiner = true;   // deux-points de l'heure et alertes clignotantes
  }

  if (boutonDeclenche(boutonHeures, maintenant))  ajusterHeure(0, 1);
  if (boutonDeclenche(boutonMinutes, maintenant)) ajusterHeure(1, 0);
  gererSerie();

  if (aRedessiner) {
    aRedessiner = false;
    dessinerEcran(phase);
  }
}

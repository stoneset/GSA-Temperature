/*
 * IDENTIFIER_SONDES - trouver l'adresse de chaque sonde DS18B20
 *
 * Même câblage que GSA_TEMPERATURE (sondes sur D4, écran SH1106 en I2C).
 * L'écran et le port série (115200 bauds) affichent, pour chaque sonde :
 *   - son adresse,
 *   - sa température,
 *   - l'écart depuis la mise sous tension.
 *
 * Pour savoir quelle sonde est laquelle, chauffer une seule sonde (main, sèche-cheveux) :
 * c'est la ligne dont l'écart monte, marquée d'une flèche. Ou après un trajet :
 * la plus chaude est la sonde moteur, la plus froide la sonde extérieure.
 * Recopier ensuite chaque adresse dans GSA_TEMPERATURE.ino (SONDE_MOTEUR, SONDE_EXT, SONDE_INT) :
 * le port série les donne directement au bon format.
 */

#include <OneWire.h>
#include <DallasTemperature.h>
#include <U8g2lib.h>

const uint8_t PIN_ONEWIRE    = 4;
const uint8_t PIN_OLED_RESET = 6;
const uint8_t NB_MAX_SONDES  = 3;

U8G2_SH1106_128X64_NONAME_2_HW_I2C ecran(U8G2_R0, PIN_OLED_RESET);
OneWire oneWire(PIN_ONEWIRE);
DallasTemperature sondes(&oneWire);

DeviceAddress adresses[NB_MAX_SONDES];
float temperatures[NB_MAX_SONDES];
float temperaturesDepart[NB_MAX_SONDES];
bool departConnu[NB_MAX_SONDES];
uint8_t nbSondes = 0;

void texteAdresse(char *texte, const uint8_t *adresse) {
  const char hex[] = "0123456789ABCDEF";
  for (uint8_t i = 0; i < 8; i++) {
    texte[2 * i]     = hex[adresse[i] >> 4];
    texte[2 * i + 1] = hex[adresse[i] & 0x0F];
  }
  texte[16] = '\0';
}

// Indice de la sonde qui a le plus chauffé depuis le départ (au moins 1 °C), sinon -1
int8_t sondeChauffee() {
  int8_t indice = -1;
  float ecartMax = 1.0;
  for (uint8_t i = 0; i < nbSondes; i++) {
    float ecart = temperatures[i] - temperaturesDepart[i];
    if (ecart >= ecartMax) {
      ecartMax = ecart;
      indice = i;
    }
  }
  return indice;
}

void mesurer() {
  sondes.begin();                       // relit le bus : détecte une sonde branchée ou débranchée
  sondes.setResolution(12);             // 0,0625 °C, pour voir une sonde chauffer dans la main
  nbSondes = sondes.getDeviceCount();
  if (nbSondes > NB_MAX_SONDES) nbSondes = NB_MAX_SONDES;
  sondes.requestTemperatures();         // bloquant, 750 ms
  for (uint8_t i = 0; i < nbSondes; i++) {
    sondes.getAddress(adresses[i], i);
    temperatures[i] = sondes.getTempC(adresses[i]);
    if (!departConnu[i] && temperatures[i] > DEVICE_DISCONNECTED_C) {
      temperaturesDepart[i] = temperatures[i];
      departConnu[i] = true;
    }
  }
}

void afficherEcran() {
  int8_t chauffee = sondeChauffee();
  ecran.firstPage();
  do {
    ecran.setFont(u8g2_font_5x8_tr);
    char ligne[26];
    itoa(nbSondes, ligne, 10);
    strcat(ligne, nbSondes > 1 ? " sondes" : " sonde");
    ecran.drawStr(0, 7, ligne);
    if (nbSondes == 0) {
      ecran.drawStr(0, 30, "Verifier D4, 5V, GND");
      ecran.drawStr(0, 40, "et la 4,7 kOhm");
    }
    for (uint8_t i = 0; i < nbSondes; i++) {
      u8g2_uint_t y = 17 + 16 * i;
      ligne[0] = '1' + i;
      ligne[1] = ' ';
      texteAdresse(ligne + 2, adresses[i]);
      ecran.drawStr(0, y, ligne);

      char nombre[8];
      if (temperatures[i] <= DEVICE_DISCONNECTED_C) {
        strcpy(ligne, "  erreur de lecture");
      } else {
        strcpy(ligne, "  ");
        dtostrf(temperatures[i], 5, 1, nombre);
        strcat(ligne, nombre);
        strcat(ligne, " C   ");
        float ecart = temperatures[i] - temperaturesDepart[i];
        if (ecart >= 0) strcat(ligne, "+");
        dtostrf(ecart, 3, 1, nombre);
        strcat(ligne, nombre);
      }
      if (i == chauffee) strcat(ligne, " <--");
      ecran.drawStr(0, y + 8, ligne);
    }
  } while (ecran.nextPage());
}

void afficherSerie() {
  Serial.print(nbSondes);
  Serial.println(F(" sonde(s) :"));
  for (uint8_t i = 0; i < nbSondes; i++) {
    Serial.print(F("  Sonde "));
    Serial.print(i + 1);
    Serial.print(F(" : { "));
    for (uint8_t j = 0; j < 8; j++) {
      Serial.print(F("0x"));
      if (adresses[i][j] < 0x10) Serial.print('0');
      Serial.print(adresses[i][j], HEX);
      if (j < 7) Serial.print(F(", "));
    }
    Serial.print(F(" }  "));
    Serial.print(temperatures[i], 2);
    Serial.print(F(" C  ecart "));
    Serial.print(temperatures[i] - temperaturesDepart[i], 2);
    Serial.println(i == sondeChauffee() ? F("  <-- chauffee") : F(""));
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  ecran.setBusClock(100000);
  ecran.begin();
}

void loop() {
  mesurer();
  afficherEcran();
  afficherSerie();
}

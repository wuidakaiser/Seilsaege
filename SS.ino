/* Das Programm steuert das Modell einer Stein-Seilsaege.
    Die obere Kontur muss vor der unteren eingegeben werden. Die Winkel der Polygonpunkte werden in Grad eingegeben. Die Werte fuer die 1. Punkte
    des oberen und unteren Polygons werden vom Programm als zusaetzliche Punkte in den Laengen- und Winkel-Feldern angefuegt. Es kann auch einfach
    durch Dateneingabe eine sternenfoermige Figur mit unterer und oberer Kontur erzeugt werden  */

#include <Stepper.h>
// number of steps per revolution
const int STEPS = 2048 ;  // Schritte pro 360 Grad fuer 28BYJ-48
// set the speed in rpm. Max. 10 - 15
float Phi_RPM = 2.0;
float Ngg_RPM = 2.0;
const byte MS1 = 4;
//const byte MS3 = 3;
const byte MS2 = 15;
// ******************************************
// Pin-Belegung fuer ESP32
#if defined(ESP_PLATFORM)    // nur fuer ESP32 uebersetzen
const int LED_Pin = 2;
const byte x_Pin = 12;
const byte x_Richtungs_Pin = 13;
Stepper Phi_Stepper(STEPS, 14, 26, 27, 25);
Stepper Neigg_Stepper(STEPS, 5, 19, 18, 21);
float Grenzwert = 2500.0;
const int AnalogPin = 33;
#else
// ******************************************

// ******************************************
//// Pin-Belegung fuer Arduino      // nur fuer Arduino-boards uebersetzen
//Stepper Phi_Stepper(STEPS, 2, 4, 3, 5);
//Stepper Neigg_Stepper(STEPS, 8, 10, 9, 11);
Stepper Phi_Stepper(STEPS, 8, 10, 9, 11);
Stepper Neigg_Stepper(STEPS, 2, 4, 3, 5);
//const byte Heizdraht_Pin = 6;
const byte x_Pin = 12;
const byte x_Richtungs_Pin = 13;
float Grenzwert = 500.0;
const int AnalogPin = 0;
// ******************************************
#endif

byte nq = 1;  // nq = Nummer des Quadranten zum Abschneiden
byte Pkt_Nr[2], x_delay = 5;  // x_delay bestimmt die Geschwindigkeit d. Schlittens beim Schneiden
float x1, yy1, x2, y2, x_oben, y_oben, x_unten, y_unten, L_oben, L_unten, Rad_R, Blockdicke,
      Abstand, Achshoehe, ah, d, Lang, Dist, Dist_alt, L_alt, Phi_alt, Phi_next;
float Phi, Delta_Phi, m1, Delta_x, Delta_Neigung, Delta_L, eta, dx_Neigung, L[2];
float x_Schritt, Phi_Schritt, Neigg_Schritt, Seilneigung, Seilneigung_alt, x_Rest, Phi_Rest, Neigg_Rest,
      XS_Rest, NS_Rest, PS_Rest;
int i, j, k, n, x_Schritte, Phi_Schritte, Neigg_Schritte, XS_Summe, NS_Summe, PS_Summe, m, mini = 0, Anzahl = 0;
int nx = 0, n_Ngg = 0, n_PS = 0, Anmarsch = 0, Schwelle = 35;
unsigned long  Pause = 1000, T_alt = 0;
char Kennung, str1[20];
String h;
boolean fertig = false, Eingeschaltet = true;
//
//// ******************************************
//// Datenpaket fuer Polygon1
//const byte Polygon_Punkte = 8;
//float Winkel[][Polygon_Punkte + 1] = {{ 20.0, 60.0, 120.0, 180.0, 210.0, 270.0, 330.0, 360.0}, { 30.0 , 80.0, 120.0, 160.0, 240.0, 300.0, 330.0, 360}};
//float Laenge[][Polygon_Punkte + 1] = {{ 85.0, 100.0, 100.0, 80.0, 90.0, 60.0, 90.0, 90.0}, {90.0, 110.0, 100.0, 90.0, 100.0, 80.0, 90.0, 65.0}};

//// ******************************************
//// Datenpaket fuer Polygon2
//const byte Polygon_Punkte = 6;
//float Winkel[][Polygon_Punkte + 1] = {{ 10.0, 40.0, 130.0, 180.0, 230.0, 300.0}, { 20.0, 60.0, 160.0, 210.0, 270.0, 320.0}};
//float Laenge[][Polygon_Punkte + 1] = {{65.0, 70.0, 75.0, 90.0, 85.0, 80.0}, {70.0, 70.0, 90.0, 85.0, 75.0, 70.0}};
//// ******************************************
//
//// ******************************************
////Datenpaket fuer einen Stern
const int n_Spitzen = 4;
float d_Winkel = 360.0 / (n_Spitzen * 4);
const int Polygon_Punkte = n_Spitzen * 4;
int Ind = 0;
float Laenge[2][Polygon_Punkte + 1];
float Winkel[2][Polygon_Punkte + 1];
float Radius[6] = {75.0, 75.0, 65.0, 65.0, 75.0, 75.0};
float Drehwinkel = 22.5;
//// ******************************************

float Steigung[2][Polygon_Punkte + 1], X_Faktor, N_Faktor, P_Faktor;

struct Punkt {
  float x, y;
};

Punkt Pkt[2];

void setup(void) {
  delay(5000);
  Serial.begin(115200);
  while (!Serial) {
    //     wait for serial port to connect. Needed for native USB port only
  }

  for (i = 0; i < 20; i++) Serial.print(" << >>");
  Serial.print("\n");

  pinMode(2, OUTPUT);  pinMode(3, OUTPUT);  pinMode(4, OUTPUT);  pinMode(5, OUTPUT);  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);  pinMode(10, OUTPUT); pinMode(11, OUTPUT); pinMode(12, OUTPUT); pinMode(13, OUTPUT);

  //// ******************************************
#if defined(ESP_PLATFORM)    // nur fuer ESP32 uebersetzen
  pinMode(14, OUTPUT); pinMode(15, OUTPUT); pinMode(16, OUTPUT); pinMode(17, OUTPUT); pinMode(18, OUTPUT);
  pinMode(19, OUTPUT); pinMode(21, OUTPUT); pinMode(22, OUTPUT); pinMode(23, OUTPUT); pinMode(25, OUTPUT);
  pinMode(26, OUTPUT); pinMode(27, OUTPUT);
#endif
  //// ******************************************

  //  pinMode(Heizdraht_Pin, OUTPUT);
  //  digitalWrite(Heizdraht_Pin, LOW);  // Heizdraht ausschalten
  digitalWrite(MS1, LOW);
  digitalWrite(MS2, LOW);
  Phi_Stepper.setSpeed(Phi_RPM);
  Neigg_Stepper.setSpeed(Ngg_RPM);

  // Besetzen der Felder fuer Winkel und Laengen fuer einen Doppel-Stern
  for (i = 0; i <= 1; i++)  { // Schleife fuer die beiden Sterne
    for (j = 0; j <= n_Spitzen - 1; j++) { // Schleife fuer die Spitzen
      for (k = 0; k <= 3; k++) {  // Schleife fuer die 4 Radien
        Laenge[i][Ind] = Radius[k + i * 2];
        Winkel[i][Ind] = Ind * d_Winkel;
        //        Serial.print(Laenge[i][Ind], 1); Serial.print("\t"); Serial.println(Winkel[i][Ind], 1);
        Ind += 1;
      }
    }
    Ind = 0;
  }

  // Umspeichern, falls ein Stern verdreht wird
  if (Drehwinkel > 0.0) {
    float HW[Polygon_Punkte], HL[Polygon_Punkte];
    n = abs(trunc(Drehwinkel / (360.0 / (n_Spitzen * 4))));
    Serial.println(" n = " + String(n));
    for (i = 0; i <= n - 1; i++) {
      HW[i] = Winkel[0][Polygon_Punkte - n + i];
      HL[i] = Laenge[0][Polygon_Punkte - n + i];
      //      Serial.print("HW \t"); Serial.println( HW[i]);
    }
    for (i = 0; i <= Polygon_Punkte - n; i++) {
      Winkel[0][Polygon_Punkte - i] = Winkel[0][Polygon_Punkte - n - i] + Drehwinkel;
      Laenge[0][Polygon_Punkte - i] = Laenge[0][Polygon_Punkte - n - i];
    }

    for (i = 0; i <= n - 1; i++) {
      Winkel[0][i] = HW[i] + Drehwinkel;
      if (Winkel[0][i] > 360.0) Winkel[0][i] = Winkel[0][i] - 360.0;
      Laenge[0][i] = HL[i];
      //      Serial.print("Winkel \t"); Serial.println( HW[i]);
    }
  }
  //  Besetzen des letzten Punktes mit dem 1. Punkt
  Laenge[0][Polygon_Punkte] = Laenge[0][0];
  Laenge[1][Polygon_Punkte] = Laenge[1][0];
  Winkel[0][Polygon_Punkte] = Winkel[0][0] + 360.0;
  Winkel[1][Polygon_Punkte] = Winkel[1][0] + 360.0;

  // Berechnung und Ausgabe der Steigungen der beiden Polygone
  for (k = 0; k <= 1; k++) {
    if (k == 0) {
      Serial.println(" "); Serial.println("Steigungen oben ");
    } else
    { Serial.println(" "); Serial.println("Steigungen unten ");
    }
    Serial.println(" Tangens         Winkel (Grad");

    // Berechnung der Steigung vom letzten zum ersten Punkt.
    x1 = Laenge[k][Polygon_Punkte - 1] * cos(Winkel[k][Polygon_Punkte - 1] * PI / 180.0);
    yy1 = Laenge[k][Polygon_Punkte - 1] * sin(Winkel[k][Polygon_Punkte - 1] * PI / 180.0);
    x2 = Laenge[k][0] * cos(Winkel[k][0] * PI / 180.0);
    y2 = Laenge[k][0] * sin(Winkel[k][0] * PI / 180.0);
    Steigung[k][0] = (y2 - yy1) / (x2 - x1);
    dtostrf(Steigung[k][0], 8, 2, str1); Serial.print(str1); Serial.print("\t\t");
    h = atan2((y2 - yy1) , (x2 - x1)) * 57.3; Serial.println(h);

    // Berechnung der Steigungen der restlichen Polygonpunkte
    for (j = 1; j <= Polygon_Punkte - 1; j++) {
      x1 = Laenge[k][j - 1] * cos(Winkel[k][j - 1] * PI / 180.0);
      yy1 = Laenge[k][j - 1] * sin(Winkel[k][j - 1] * PI / 180.0);
      x2 = Laenge[k][j] * cos(Winkel[k][j] * PI / 180.0);
      y2 = Laenge[k][j] * sin(Winkel[k][j] * PI / 180.0);
      Steigung[k][j] = (y2 - yy1) / (x2 - x1);
      dtostrf(Steigung[k][j], 8, 2, str1); Serial.print(str1); Serial.print("\t\t");
      dtostrf(atan2((y2 - yy1) , (x2 - x1)) * 57.3, 5, 2, str1); Serial.println(str1);
    }

    Steigung[k][Polygon_Punkte] = Steigung[k][0];
    dtostrf(Steigung[k][Polygon_Punkte], 8, 2, str1);
    Serial.print(str1); Serial.print("\t\t"); Serial.println(h);
  }

  // Ausgabe der Eingabedaten
  Serial.println("\n");
  for (k = 0; k <= 1; k++) {
    Serial.print("Laenge "); Serial.print(k);
    for (i = 0; i <= Polygon_Punkte; i++) {
      Serial.print("\t"); Serial.print(Laenge[k][i], 1);
    }
    Serial.print("\n");
    Serial.print("Winkel "); Serial.print(k);
    for (i = 0; i <= Polygon_Punkte; i++) {
      Serial.print("\t"); Serial.print(Winkel[k][i], 1);
      Winkel[k][i] = Winkel[k][i] * PI / 180.0;
    }
    Serial.println("\n");
  }

  // Besetzen der fixen Daten
  Abstand = 221.0;  // Abstand zwischen Drehtellermitte und Mitte Neigungsgalgen in Startposition
  Rad_R  = 57.0; Blockdicke = 60.0; Achshoehe = 31.0;
  Serial.print("\n");
  Serial.print("  Rad_R= "); Serial.print(Rad_R); Serial.print("  Achshoehe= "); Serial.print(Achshoehe);
  Serial.print("  Blockdicke= "); Serial.print(Blockdicke); Serial.print("\n");
  Phi = 0.0;
  Delta_Phi = 2.0 * PI / 180;    // Grad Schrittweite im Azimuth; bestimmt die Genauigkeit
  Phi_Schritt = 2 * PI / STEPS / 65 * 25 ; //  Schrittweite (Rad) d. Drehtisches; hier: 28BYJ-48, Untersetzung 25:64 Zaehne
  Neigg_Schritt = 2 * PI / STEPS / 44 * 14; //  Schrittweite der Seilneigung; hier: 28BYJ-48, Untersetzung 14:44 Zaehne
  x_Schritt = 1.8 * PI / 180 / 72 * 15 * 6.5; // Schrittweite (mm) pro Schritt in x-Richtung; Untersetzung 15:72 Zaehne, Zahnriemenrad-Radius = 6.5 mm
  //  x_Schritt = 0.042; // gemessen
  Serial.println("\t x_Schritt (mm) Neigg_Schritt (Grad)  Phi_Schritt (Grad)");
  Serial.print("\t"); Serial.print(x_Schritt, 5); Serial.print("\t\t"); Serial.print( Neigg_Schritt * 180 / PI, 5); Serial.print("\t\t\t");
  Serial.println( Phi_Schritt * 180 / PI, 5);
  j = 0; n = 0; m = 0;
  x_Rest = 0.0; Phi_Rest = 0.0; Neigg_Rest = 0.0;
  XS_Rest = 0.0; PS_Rest = 0.0; NS_Rest = 0.0;   // Zaehler fuer die restlichen Schritte in x-, Neigung- und Phi-Richtung
  XS_Summe = 0; NS_Summe = 0; PS_Summe = 0;

  // Berechnen der Seilneigung zum Start
  yy1 = Laenge[0][0] * sin(Winkel[0][0]) ;
  x1 = Laenge[0][0] * cos(Winkel[0][0]);
  L[0] = x1 - yy1 / Steigung[0][0];
  yy1 = Laenge[1][0] * sin(Winkel[1][0]);
  x1 = Laenge[1][0] * cos(Winkel[1][0]);
  L[1] = x1 - yy1 / Steigung[1][0];

  Serial.print(" L_oben = "); Serial.print(L[0], 2);  Serial.print(" L_unten = "); Serial.println(L[1], 2);
  if (L[0] == L[1]) {
    Seilneigung = PI / 2;
    Dist = Rad_R;
  }
  else {
    Seilneigung = atan2(Blockdicke, L[0] - L[1]);
    Dist = Achshoehe / tan(Seilneigung) + Rad_R / sin(Seilneigung);
  }
  while (analogRead(AnalogPin) < Grenzwert) {
    Serial.println(" Warten auf Start"); Serial.print(" Pin AnalogPin = "); Serial.println(analogRead(AnalogPin));
    delay(2000);
  }

  // Einrichten des Seilgalgens auf die Seilneigung beim Start
  Anzahl = round((PI / 2 - Seilneigung) / Neigg_Schritt);
  Neigg_Stepper.step(-Anzahl); // Richtung ueberpruefen!
  Serial.print(" Seilneigung = "); Serial.print(Seilneigung * 180 / PI);
  Serial.print(" Dist = "); Serial.print(Dist); Serial.print(" Anzahl = "); Serial.println(Anzahl);

  // Berechnen des Anmarschweges des x-Schlittens zum Beginn des Polygons
  // Dist ist der Abstand des unteren Schnittpkt. von der senkrechten Achsprojektion
  Anmarsch = round((Abstand - L[1] - Dist) / x_Schritt);
  Serial.print("\n\t"); Serial.print(Anmarsch); Serial.println(" Schritte zur Startposition - Heizdraht einschalten!");
  //  digitalWrite(Heizdraht_Pin, HIGH);  // Heizdraht einschalten
  //  delay(5000);    //  5 Sek. Vorheizzeit
  digitalWrite(x_Richtungs_Pin, LOW);   // Richtung ueberpruefen!
  digitalWrite(LED_Pin, HIGH);
  j = 0;
  for (i = 0; i <= Anmarsch; i++) {
    digitalWrite(x_Pin, HIGH);
    delay(x_delay);
    digitalWrite(x_Pin, LOW);
    delay(x_delay);
    j++;
    if (j == round(Anmarsch / 50)) {
      j = 0; Serial.print(".");
    }
  }
  digitalWrite(LED_Pin, LOW);
  Serial.print("\n\t Startposition erreicht. \n");
  Serial.print("\n\t"); Serial.print(n_Spitzen); Serial.print("-er Stern um ");  Serial.print(Drehwinkel, 1);
  Serial.println(" Grad verdreht");

  //// ******************************************
#if defined(ESP_PLATFORM)    // nur fuer ESP32 uebersetzen
  //  touchAttachInterrupt(T0, Restart, Schwelle);
  //  touchAttachInterrupt(T4, Ein_Aus, Schwelle);
#endif
  //// ******************************************
}  // Ende von setup

//// ******************************************
//#if defined(ESP_PLATFORM)    // nur fuer ESP32 uebersetzen
//void Restart() {
//  ESP.restart();
//}
//
//void Ein_Aus() {
//  if ((millis() - T_alt) > Pause) {
//    Eingeschaltet = !Eingeschaltet;
//    if (Eingeschaltet) Serial.println("Eingeschaltet");
//    else Serial.println("\t\t Ausgeschaltet");
//    T_alt = millis();
//  }
//}
//#endif
//// ******************************************

void Schreiten(char N) {
  switch (N) {
    case 'X':
      Anzahl = round(X_Faktor + XS_Rest);
      XS_Rest = X_Faktor + XS_Rest - Anzahl ;
      XS_Summe += Anzahl;
      nx += Anzahl;
      if (x_Schritte >= 0) digitalWrite(x_Richtungs_Pin, HIGH); // Richtung überprüfen!
      else digitalWrite(x_Richtungs_Pin, LOW);
      digitalWrite(LED_Pin, HIGH);
      for (j = 1; j <= abs(Anzahl); j++) {
        digitalWrite(x_Pin, HIGH);
        delay(x_delay);
        digitalWrite(x_Pin, LOW);
        delay(x_delay);
      }
      digitalWrite(LED_Pin, LOW);
      break;
    case 'N':
      Anzahl = round(N_Faktor + NS_Rest);
      NS_Rest = N_Faktor + NS_Rest - Anzahl;
      NS_Summe += Anzahl;
      n_Ngg += Anzahl;
      Neigg_Stepper.step(Anzahl); // Richtung überprüfen!
      break;
    case 'P':
      //  Jetzt den Drehteller drehen
      Anzahl = round(P_Faktor + PS_Rest);
      PS_Rest = P_Faktor + PS_Rest - Anzahl;
      PS_Summe += Anzahl;
      //  Serial.print("Delta_Phi= "); Serial.print(Delta_Phi); Serial.print("  Phi_Rest= "); Serial.print(Phi_Rest);
      Phi_Stepper.step(-abs(Anzahl));  // Richtung überprüfen!
      n_PS += Anzahl;
      break;
    default:
      // if nothing else matches, do the default
      // default is optional
      break;
  }
}

void loop () {
  byte nq = 1, num;  // nq = Nummer des Quadranten zum Abschneiden
  float HiWi;
  Pkt_Nr[0] = 0; Pkt_Nr[1] = 0;  // Pkt_Nr ist der Index des Polygonpunktes

  while (!fertig) {
    while (Phi <= 2.0 * PI + 0.01) {
      //      while (!Eingeschaltet) {
      while (analogRead(AnalogPin) < Grenzwert) {
        Serial.println(" loop: Ausgeschaltet!");
        delay(2000);
      };
      if ((Phi > nq * 120 * PI / 180.0) and (nq < 3)) {
        //       Serial.print("Abschnitt bei "); Serial.println(Phi * 180 / PI, 1);
        digitalWrite(x_Richtungs_Pin, HIGH);  // Zurueckfahren zum Abschneiden. Richtung überprüfen!
        for (int l = 1;  l <= round(30.0 / x_Schritt); l++) {
          digitalWrite(x_Pin, HIGH);
          delay(x_delay);
          digitalWrite(x_Pin, LOW);
          delay(x_delay);
        }
        digitalWrite(x_Richtungs_Pin, LOW);  // Wieder schnell zur Schneidestelle vorfahren. Richtung überprüfen!
        for (int l = 1;  l <= round(30.0 / x_Schritt); l++) {
          digitalWrite(x_Pin, HIGH);
          delay(5);
          digitalWrite(x_Pin, LOW);
          delay(5);
        }
        nq++;
      }

      for (k = 0; k <= 1; k++) {
        j = Pkt_Nr[k];
        x1 = Laenge[k][j] * cos(Winkel[k][j]);
        yy1 = Laenge[k][j] * sin(Winkel[k][j]);
        if (Phi == Winkel[k][j]) {    // am naechsten Konturpunkt
          //  Serial.print(" Phi = "); Serial.print(Phi * 180 / PI); Serial.print(" = Winkel = "); Serial.println(HiWi * 180 / PI);
          //          Serial.printf(" Phi = %6.2f   = Winkel = %6.2f    k = %d   j = %d\n", Phi * 180 / PI, Winkel[k][j] * 180 / PI, k, j);
          Pkt_Nr[k] += 1;
          j = Pkt_Nr[k];
        }
        else
        { if (Winkel[0][Pkt_Nr[0]] < Winkel[1][Pkt_Nr[1]]) num = 0;
          else num = 1;
          HiWi = Winkel[num][Pkt_Nr[num]];
          if (Phi > HiWi) {
            //            Serial.printf(" Phi = %6.2f   > Winkel = %6.2f    k = %d   j = %d\n", Phi * 180 / PI, HiWi * 180 / PI, k, j);
            //             Serial.print(" Phi = "); Serial.print(Phi * 180 / PI); Serial.print(" > Winkel = "); Serial.println(HiWi * 180 / PI);
            Phi =  HiWi;
            if (abs(Phi - Phi_alt) < 0.001) {
              Phi = Phi + 0.001;
              //  Serial.printf(" Phi korrigiert = %6.2f\n", Phi * 180 / PI);
            }
            Pkt_Nr[num] += 1;
            n = n - 1;
          }
        }
        m1 = Steigung[k][j];
        if (abs(cos(Phi)) < 1.0E-5) {   // Phi ist 90 oder 270 Grad
          Pkt[k].x = 0.0;
          Pkt[k].y = yy1 - x1 * Steigung[k][j];
          L[k] = abs(Pkt[k].y);
        }
        else
        { m1 = tan(Phi);
          Pkt[k].x = (yy1 - x1 * Steigung[k][j]) / (m1 - Steigung[k][j]);
          Pkt[k].y = m1 * Pkt[k].x;
          L[k] = sqrt(sq(Pkt[k].x) + sq(Pkt[k].y));
        }
        if (j <= (Polygon_Punkte - 1)) {
          x1 = Laenge[k][j] * cos(Winkel[k][j]);
          yy1 = Laenge[k][j] * sin(Winkel[k][j]);
        }
        if (k == 0) {
          x_oben = Pkt[k].x;
          y_oben = Pkt[k].y;
        }
        else
        { if (L[0] == L[1])  Seilneigung = PI / 2;
          else Seilneigung = atan2(Blockdicke, L[0] - L[1]) ;
          if (Phi == 0.0) {
            Seilneigung_alt = Seilneigung;
            L_alt = L[1];
          }
          Delta_Neigung =  Seilneigung - Seilneigung_alt;
          Delta_L = L[1] - L_alt;
          x_unten = Pkt[k].x;
          y_unten = Pkt[k].y;
          Seilneigung_alt = Seilneigung;
        }
      }  // Ende k-Schleife

      L_alt = L[1];
      if (abs(Seilneigung - PI / 2) < 1.0E-4) Dist = Rad_R;
      else Dist = Achshoehe / tan(Seilneigung) + Rad_R / sin(Seilneigung);
      if (Phi == 0.0)  {
        Dist_alt = Dist;
      }
      else
      { dx_Neigung = Dist_alt - Dist;
        Dist_alt = Dist;
        Delta_x = Delta_L - dx_Neigung;

        //  Bestimmung der Schritte  fuer die Schrittmotoren
        // Serial.print(" Delta_x ");    Serial.print(Delta_x, 5); Serial.print(" x_Rest "); Serial.print(x_Rest, 5); Serial.print("  Delta_x + x_Rest "); Serial.print(Delta_x + x_Rest, 5);
        // Serial.print("    x_Schritte "); Serial.println(x_Schritte);
        x_Schritte = round((Delta_x + x_Rest) / x_Schritt);
        x_Rest = (Delta_x + x_Rest) - x_Schritte * x_Schritt;
        Neigg_Schritte = round((Delta_Neigung + Neigg_Rest) / Neigg_Schritt);
        Neigg_Rest = (Delta_Neigung + Neigg_Rest) - Neigg_Schritte * Neigg_Schritt;
        Phi_Schritte = round((Phi - Phi_alt + Phi_Rest) / Phi_Schritt);
        Phi_Rest = (Phi - Phi_alt + Phi_Rest) - Phi_Schritte * Phi_Schritt;

        // Bestimmen des Schrittmotors mit den wenigsten Schritten
        mini = Phi_Schritte;
        if (x_Schritte != 0) mini = min(mini, abs(x_Schritte));
        if (Neigg_Schritte != 0) mini = min(mini, abs(Neigg_Schritte));
        //  Serial.print(" mini = "); Serial.println(mini);
        X_Faktor = float(x_Schritte) / mini;
        N_Faktor = float(Neigg_Schritte) / mini;
        P_Faktor = float(Phi_Schritte) / mini;
        // Serial.println(" x_Schritte  Neigg_Schritte  Phi_Schritte   Kennung  X_Fakt   N_Fakt   P_Fakt");
        // Serial.print(x_Schritte); Serial.print("\t\t"); Serial.print(Neigg_Schritte);  Serial.print("\t\t"); Serial.print(Phi_Schritte);
        // Serial.print("\t");  Serial.print(Kennung); Serial.print("\t"); Serial.print(X_Faktor); Serial.print("\t");
        // Serial.print(N_Faktor); Serial.print(" \t");  Serial.println(P_Faktor);
        if (Phi > 0)
          do {     // die Reihenfolge der Schritte pruefen und durchfuehren
            if (Delta_x > 0) { // Zuerst die x_Schritte, dann die Neigungsschritte
              Schreiten('X');
              Schreiten('N');
            }
            else { // Zuerst die Neigungsschritte, dann die x_Schritte
              Schreiten('N');
              Schreiten('X');
            }
            Schreiten('P');   // Jetzt die Phi-Schritte
            // Serial.print(" XS_Summe = "); Serial.print(XS_Summe); Serial.print("  XS_Rest = "); Serial.print(XS_Rest);
            // Serial.print(" NS_Summe = "); Serial.print(NS_Summe); Serial.print("  NS_Rest = "); Serial.print(NS_Rest);
            // Serial.print(" PS_Summe = "); Serial.print(PS_Summe); Serial.print("  PS_Rest = "); Serial.println(PS_Rest);
          } while (PS_Summe < Phi_Schritte);
      }
      XS_Summe = 0; NS_Summe = 0; PS_Summe = 0;
      if (m == 0 || m == 20) { // nach 20 Zeilen Ueberschrift erneut drucken
        Serial.print("\n");
        Serial.print("  Phi    x_oben  y_oben  x_unten y_unten  L_oben  L_unten ");
        Serial.print(" Seilneigg  dx_Ngg  Delta_L  Delta_x  x_Rest x_Schritte Delta_Neigg  Ng_Rest Ng_Schritte nx n_Ngg PS  Phi_Rest  PSS");
        Serial.print("\n");
        m = 0;
      }

      dtostrf(Phi * 180 / PI, 6, 1, str1);  Serial.print(str1); dtostrf(x_oben, 9, 2, str1);   Serial.print(str1);
      dtostrf(y_oben, 8, 2, str1);      Serial.print(str1); dtostrf(x_unten, 8, 2, str1);  Serial.print(str1);
      dtostrf(y_unten, 8, 2, str1);     Serial.print(str1); dtostrf(L[0], 9, 2, str1);  Serial.print(str1);
      dtostrf(L[1], 9, 2, str1);     Serial.print(str1);
      dtostrf(Seilneigung * 180 / PI, 10, 2, str1);  Serial.print(str1);  dtostrf(dx_Neigung, 9, 2, str1); Serial.print(str1);
      dtostrf(Delta_L, 9, 3, str1);     Serial.print(str1); dtostrf(Delta_x, 9, 4, str1);  Serial.print(str1);
      dtostrf(x_Rest, 9, 4, str1);     Serial.print(str1);
      dtostrf(x_Schritte, 8, 0, str1);  Serial.print(str1); dtostrf(Delta_Neigung * 180 / PI, 13, 3, str1); Serial.print(str1);
      dtostrf(Neigg_Rest * 180 / PI, 9, 4, str1); Serial.print(str1); dtostrf(Neigg_Schritte, 8, 0, str1); Serial.print(str1);
      dtostrf(nx, 8, 0, str1); Serial.print(str1);  dtostrf(n_Ngg, 5, 0, str1); Serial.print(str1);
      dtostrf(Phi_Schritte, 4, 0, str1); Serial.print(str1);
      dtostrf(Phi_Rest * 180 / PI, 10, 5, str1); Serial.print(str1); dtostrf(n_PS, 5, 0, str1); Serial.print(str1); Serial.print("\n");
      Phi_alt = Phi;
      n = n + 1;
      Phi = n * Delta_Phi;
      m = m + 1;
    } // while Phi loop
    fertig = true;
    // Zurueck zur Startposition
    Serial.print("\n\t"); Serial.print(Anmarsch); Serial.println(" Schritte zurueck zur Startposition");
    digitalWrite(x_Richtungs_Pin, HIGH); // Richtung überprüfen!
    for (i = 0; i <= Anmarsch; i++) {
      digitalWrite(x_Pin, HIGH);
      delay(5);
      digitalWrite(x_Pin, LOW);
      delay(5);
    }
    // Rueckstellung des Seilgalgens auf die Startstellung (senkrechte Stellung)
    Anzahl = round((PI / 2 - Seilneigung) / Neigg_Schritt);
    Neigg_Stepper.step(Anzahl); // Richtung ueberpruefen!
    // digitalWrite(Heizdraht_Pin, LOW); // Heizdraht ausschalten
    Serial.println("\n\t Startposition erreicht - Heizdraht ausgeschaltet.");
  }  // while fertig loop
}

/****************************************************
*****************************************************
  Mini Vario mit Bluetooth
    Bluetooth Modul ist H-06
    Barometer Modul ist MS5611
    Mini PRO 3.3V 8 MHz oder Leonardo
    oder !!!beliebiges!!! Arduino
*****************************************************

Notiz:  
  * Filter ist ein Mischung aus Exp-Filter und 
    Mittelwert aus "mittel_n"-Werte. 
  * Hier ist die Erweiterung zum Akku-Ladezustand 
    auslesen.

*****************************************************
  Auf der Misst von Ivaylo gewachsen.
  2018-08-05
*****************************************************
****************************************************/

#include <Wire.h>
#include <MS5611.h>
MS5611 bpm;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////   Variablen die Mann aendern kann!   /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float min_steigen = 0.20;               //Minimale Steigen (Standard Wert ist 0.4m/s).
float max_sinken = -3.50;               //Maximales Sinken (Standard Wert ist - 1.1m/s).

long leseZeit = 125;                    //Interval zum lesen vom Baro audio Vario, Standard(min) ist 150.
long leseZeitBT = 100;                  //Interval zum lesen vom Baro fuer BT, Standard(min) ist 100.

long konst_frqz = 150;                  //Audio Frequenz beim konstante Frequenz Einstellung.
long max_frqz = 2000;                   //Maximale Audio Frequenz beim variable Frequenz Einstellung.

short bt_pin = 2;                       //Bluetooth Pin definieren. Fuer Leonardo 14. Fuer die Anderen 2.

int a_pin1 = 6;                         //Lautsprecher Pin definieren! <<= fuer normalen Mini Pro

// Fileter Einstellungen!!!   Hier Verengerungen nur sehr vorsichtig vornehmen!!!
float FehlerV = 3.000 * min_steigen;    //Gewichtung fuer Vario Filter berechnen. 0.1 > FehlerV < 1.0

float mittel_n = 6;                     // Anzahl Werte fuer Mittelwert bilden.
float kal[7];                           // kal[n] ==> n = mittel_n +1

short BatV = A3;                        //Akku Spannung Pin definieren! <<= fuer normalen Mini Pro
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long Druck, Druck0, DruckB;

int PinBT,  XOR, c, startCH = 0, Vbat;
float Vario, VarioR, Hoehe, AvrgV, Batt, Temp;

unsigned long  dZeit, ZeitE, ZeitS, ZeitPip;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////




// SETUP//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  leseZeit = leseZeit - 24;

  Serial.begin(9600);
  //Serial1.begin(9600);

  pinMode(bt_pin, INPUT);                 // Definiert den Pin für der BT Schalter.
  PinBT = digitalRead(bt_pin);            // Definiere SChalter Zustand fuer BT.
  //PinBT = 0;                              // Wenn keine BT-Modul eingebaut ist - 0. Die obere Zwei auskommentieren.

  pinMode(7, OUTPUT);                     // Pin zum BT Versorgung.
  pinMode(8, OUTPUT);                     // Pin zum BT Versorgung.


  // Initialize MS5611 sensor!
  // Ultra high resolution: MS5611_ULTRA_HIGH_RES
  // (default) High resolution: MS5611_HIGH_RES
  // Standard: MS5611_STANDARD
  // Low power: MS5611_LOW_POWER
  // Ultra low power: MS5611_ULTRA_LOW_POWER
  while (!bpm.begin(MS5611_ULTRA_HIGH_RES))
  {
    delay(500);
  }

  //BT umbenennen START
  if (PinBT == 1)
  {
    digitalWrite(7, HIGH);               // BT Versorgung einschalten.
    digitalWrite(8, HIGH);               // BT Versorgung einschalten.
    delay(1000);
    Serial.begin(9600);                  //fuer MiniPro
    //Serial1.begin(9600);                 //fuer BT - Leonardo.
    /*/ On-Off | Hier zwischen // die * entfernen um die BT Name zu aendern.
      Serial.print("AT");
      delay(1500);
      Serial.print("AT+NAMEIvkosVario"); //BT Name vergeben
      delay(500);
      //Serial.print("AT+RESET");
      delay(500);//*/
    // PIN ist 1234 oder 0000 <= #################################################################################
  }
  else
  {
    digitalWrite(7, LOW);               // BT Versorgung einschalten.
    digitalWrite(8, LOW);               // BT Versorgung einschalten.
  }
  //BT umbenennen ENDE */


  // Spielt die Start-Tonfolge.
  tone(a_pin1 , 100, 150);
  delay(200);
  tone(a_pin1 , 200, 150);
  delay(200);
  tone(a_pin1 , 400, 150);
  delay(200);
  tone(a_pin1 , 700, 150);
  delay(200);
  tone(a_pin1 , 1100, 150);
  delay(200);
  tone(a_pin1 , 1600, 150);
  delay(200);
  // */

  ZeitS = micros();

}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ENDE SETUP/////////////////////////////////////////////////////////////////////////////////////////////////////


// LOOP///////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void loop()
{

  if (PinBT == 0)
  {
    dZeit = (micros() - ZeitS);
    if (float(dZeit) / 1000 >= float(leseZeit) )
    {
      SteigenBerechnen();

        //BT Taster;dZeit[ms];Druck[Pa];Hoehe[m];VarioR[m/s];Vario[m/s]
        //  Zum aktivieren der Ausgabe * zwischen // loeschen.
      
        Serial.print(PinBT);
        Serial.print("; ");
      
        Serial.print(float(dZeit) / 1000, 2);
        Serial.print("; ");
      
        Serial.print(Druck);
        Serial.print("; ");
      
        Serial.print(Hoehe, 2);
        Serial.print("; ");
      
        Serial.print(VarioR, 2);
        Serial.print("; ");
      
        Serial.print(Vario, 2);//
        Serial.println(); // */

    }
    PiepserX();
  }
  else
  {
    Bloetooth();
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ENDE LOOP//////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//=> Unterfunktionen und Programme   /////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////


// Luftdrucksensor auslesen ######################################################################################
// ###############################################################################################################
void BaroAuslesen()
{
  Temp = bpm.readTemperature();
  Druck = bpm.readPressure(true);
  Hoehe = bpm.getAltitude(Druck);
}
// ###############################################################################################################
// ENDE ##########################################################################################################



// Steigen berechnen #############################################################################################
// ###############################################################################################################
void SteigenBerechnen()
{
  BaroAuslesen();

  int i;

  if (startCH == 0)
  {
    kal[0] = Hoehe;
    startCH = 1;
  }

  // Steigwerte berechnen.
  dZeit = (micros() - ZeitS);
  ZeitS = micros();

  VarioR = ((Hoehe - kal[0]) / (float(dZeit) / 1000000));

  //VarioR=0.500; // Ton Test ! In normalen Betrieb auskommentieren!  #################

  //kal[1] = VarioR;
  kal[1] = 0.5* VarioR + 0.5* kal[1];  //##############################################

  kal[0] = Hoehe;

  // Filter fuer die Steigung anwenden.
  // > Mittelwert bilden.
  AvrgV = 0;
  i = 1;
  for (i; i <= mittel_n; i++) {
    AvrgV = AvrgV + kal[i];
  }
  AvrgV = AvrgV / mittel_n;
  AvrgV = (AvrgV  + Vario) / 2;
  // < Mittelwert bilden.

  if (FehlerV > 1.000) FehlerV = 1.000;
  Vario = FehlerV * AvrgV + (1 - FehlerV) * Vario;


  i = mittel_n;
  for (i; i > 1; i--) {
    kal[i] = kal[i - 1];
  }

}
// ###############################################################################################################
// ENDE ##########################################################################################################



// Akku Spannung im % ############################################################################################
// ###############################################################################################################
void AkkuVolt()
{
	Vbat = analogRead(BatV);
	Batt = 1000.0 + 100.0*(1 - (4.16 - Vbat*(3.30/1023.00)/0.769047619)/0.85);
  //Batt = Vbat*1.00;
  //Batt =   Vbat*(3.3 / 1023.0)/0.769;
}
// #############################################################################################################*/ 
// ENDE ##########################################################################################################



// Piepser #######################################################################################################
// ###############################################################################################################
void PiepserX()
{
  //Vario = 1.00; // Ton Test! In normalen Betrieb auskommentieren!

  float frequency = 2.94269*Vario*Vario*Vario - 71.112246*Vario*Vario + 614.136517*Vario + 30.845693;

  float duration = -38.00*Vario + 400.00; // Variable Pause

  frequency = int(frequency);
  duration = long(duration);

  // Wenn Steigen groesser als min_steigen
  if ( Vario >= min_steigen)
  {
    if ( Vario <= 7  )
    {
      if ( (millis() - ZeitPip) >= (unsigned long)(2 * duration) )
      {
        ZeitPip = millis();
        tone( a_pin1 , int(frequency), int(duration) );
      }
    }
    if ( Vario > 10)
    {
      if ( Vario > 13) noTone(a_pin1);
      else tone(a_pin1 , int(max_frqz / 2));
      delay(0);
    }
  }

  // Wenn Sinken kleiner als max_sinken
  if ( Vario < max_sinken)
  {
    tone(a_pin1 , 300, 150);
    delay(125);
    tone(a_pin1 , 200, 150);
    delay(150);
    tone(a_pin1 , 100, 150);
    delay(175);
  }

  if ( Vario < min_steigen) noTone(a_pin1);
}
// ###############################################################################################################
// ENDE ##########################################################################################################



// Bloetooth #####################################################################################################
// ###############################################################################################################
/*  Verschiedene Kommunikationsprotokolle moeglich.  */

void Bloetooth()
{


  // Start "Blue Fly Vario" sentence =============================================================================
  // =============================================================================================================
  /* Ausgabe im BlueFlyVario Format.     The standard BlueFlyVario outp ut mode. This sends raw
    pressure measurements in the form "PRS XXXXX\n": XXXXX is the raw (unfiltered) pressure
    measurement in hexadecimal pascals. */
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.
  //Temp = bpm.readTemperature();
  Druck = bpm.readPressure();

  Serial.print("PRS ");               //Ausgabe an der BT fuer MiniPro.
  Serial.println( Druck, HEX);        //BT-Serial Schnittstelle ungefiltert.  Fuer MiniPro.

  //Serial1.print("PRS ");               //Ausgabe an der BT fuer Leonardo.
  //Serial1.println( Druck, HEX);        //BT-Serial Schnittstelle ungefiltert.  Fuer Leonardo.

  delay(leseZeitBT - 73);

  // Wenn XCSoar verwendet wird die Zeile drunter mit "//..." auskommentieren.
  //delay(leseZeitBT - 22); //Wenn XCTrack benutzt wird Zeile aktiv lassen.

  // Ende "BlueFlyVario" sentence =========================================================================== */

  // =>>

  // Start "LXNAV - LXWP0" sentence ==============================================================================
  // =============================================================================================================
  /* Send LXWP0 output mode for use with a range of apps:
      "$LXWP0,loger_stored (Y/N), IAS (kph), baroaltitude (m), vario (m/s),,,,,,heading of plane,
      windcourse (deg),windspeed (kph)*checksum \r\n" */
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.
      SteigenBerechnen();

      String s = "LXWP0,N,,";
      s = String(s+ String(Hoehe,1) + "," + String(Vario,2) + ",,,,,,,,"  );

    // Checksum berechnen und als int ausgeben
    // wird als HEX benötigt im NMEA Datensatz
    // zwischen $ und * rechnen
      int i, XOR, c;
      XOR = 0;

      for (i = 0; i < s.length(); i++) {
          c = (unsigned char)s.charAt(i);
          if (c == '*') break;
          if (c!='$') XOR ^= c;
      }
    // Checksum berechnen

      // Fuer MiniPro:
      Serial.print("$");
      Serial.print(s);
      Serial.print("*");
      Serial.println(XOR,HEX);

      // Fuer Leonardo:
      //Serial.print("$");
      //Serial.print(s);
      //Serial.print("*");
      //Serial.println(XOR,HEX); //

    delay(leseZeitBT - 73);

  // Ende "LXNAV - LXWP0" sentence ========================================================================== */

  // =>>

  // Start "LK8EX1" sentence =====================================================================================
  // =============================================================================================================
  // Send $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
  // On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.  
    Temp = bpm.readTemperature(true);
    Druck = 0.250* bpm.readPressure(true) +  0.750* Druck;
    //SteigenBerechnen();
    AkkuVolt();
    
    String s = "LK8EX1,";
    //s = String(s + String(Druck,0) + ",99999,9999," + String(Temp,1) + "," + String(Batt,0) + ",");
    s = String(s + String(Druck,DEC) + ",99999,9999," + String(Temp,1) + "," + String(Batt,1) + ",");

    // Checksum berechnen und als int ausgeben
    // wird als HEX benötigt im NMEA Datensatz
    // zwischen $ und * rechnen
    int i, XOR, c;
    XOR = 0;

    for (i = 0; i < s.length(); i++) {
    c = (unsigned char)s.charAt(i);
    if (c == '*') break;
    if (c!='$') XOR ^= c;
    }
    // Checksum berechnen

        // Fuer MiniPro:
        Serial.print("$");
        Serial.print(s);
        Serial.print("*");
        Serial.println(XOR,HEX);

        // Fuer Leonardo:
        //Serial.print("$");
        //Serial.print(s);
        //Serial.print("*");
        //Serial.println(XOR,HEX); // 
    
    delay(leseZeitBT - 30);
  // Ende "LK8EX1" sentence ================================================================================= */

  // =>>

  // Start "Custom BFV" sentence =================================================================================
  // =============================================================================================================
  /* Custom BFV sentence: This sends a NMEA like sentence in the
    following format: "$BFV,pressure(Pa),vario(cm/s), temp(deg C),
    battery(%),pitotDiffPressure(pa)*checksum\r\n". Pressure (the filtered pressure as an
    unsigned integer), vario (the filtered vario as an signed integer) and temp(signed float) are
    always sent. Battery % (unsigned integer) is only sent for models which include a battery;
    otherwise "0" is sent. pitotDiffPressure (signed integer) is only sent when the hardware
    setting usePitot is enabled. */
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.
      SteigenBerechnen();

      String s = "BFV,";
      s = String(s + String(Druck) + "," + String(Vario*100,0) + "," + String(Temp,2) + ",0"  );

    // Checksum berechnen
    // und als int ausgeben wird als HEX benötigt.
    // Im NMEA Datensatz zwischen $ und * rechnen.
      int i, XOR, c;
      XOR = 0;

      for (i = 0; i < s.length(); i++) {
          c = (unsigned char)s.charAt(i);
          if (c == '*') break;
          if (c!='$') XOR ^= c;
      }

      // Fuer MiniPro:
      Serial.print("$");
      Serial.print(s);
      Serial.print("*");
      Serial.println(XOR,HEX);

      // Fuer Leonardo:
      //Serial.print("$");
      //Serial.print(s);
      //Serial.print("*");
      //Serial.println(XOR,HEX); //

    delay(leseZeitBT - 24);

  // Ende "Custom BFV sentence" ============================================================================= */



  // Start Normale Daten Ausgabe =================================================================================
  // =============================================================================================================
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.

    // Zum Testen ueber Serial-Port !!!-> nicht vergessen VarioR aus zu kommentieren.
    // Temp.[C°];Druck[Pa];Hoehe[m];dZeit[ms];VarioR[m/s];Vario[m/s];BT Taster
    // Zum Ausgabe aktivieren * zwischen // löschen.

    SteigenBerechnen();

    Serial.print(Temp, 2);
    Serial.print("; ");

    Serial.print(Druck);
    Serial.print("; ");

    Serial.print(Hoehe, 2);
    Serial.print("; ");

    Serial.print(dZeit/1000, 3);
    Serial.print("; ");

    Serial.print(VarioR, 2);
    Serial.print("; ");

    Serial.print(Vario, 2);
    Serial.print("; ");

    Serial.print(PinBT);
    Serial.println();

    delay(leseZeit - 4);

    // Ende Normale Daten Ausgabe ============================================================================= */
}
// ###############################################################################################################
// ENDE ##########################################################################################################

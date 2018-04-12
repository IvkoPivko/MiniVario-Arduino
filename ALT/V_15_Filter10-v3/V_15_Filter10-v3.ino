/****************************************************
  Mini Vario mit Bluetooth
    Bluetooth Modul ist H-06
    Barometer Modul ist MS5611
    Mini PRO 3.3V 8 MHz oder Leonardo
    oder !!!belibiges!!! Arduino

  Auf der Misst von Ivaylo gewachsen.
  2018-04-12
****************************************************/

#include <Wire.h>
#include <MS5611.h>
MS5611 bpm;



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////   Variablen die Mann aendern kann!   /////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float min_steigen = 0.20;               //Minimale Steigen (Standart Wert ist 0.4m/s).
float max_sinken = -3.50;               //Maximales Sinken (Standart Wert ist - 1.1m/s).

long leseZeit = 150;                    //Interval zum lesen vom Baro audio Vario, Standart(min) ist 150.
long leseZeitBT = 100;                  //Interval zum lesen vom Baro fuer BT, Standart(min) ist 100.

long konst_frqz = 150;                  //Audio Frequez beim konstante Frequenz Einstellung.
long max_frqz = 1800;                   //Maximale Audio Frequenz beim variable Frequenz Einstellung.

//short bt_pin = 2;                     //Bluetooth Pin definieren. Fuer Leonardo 14. Fuer die Anderen 2.

int a_pin1 = 6;                         //Lautsprecher Pin definieren! <<= fuer normalen Mini Pro
//int a_pin1 = 9;                       //Lautsprecher Pin definieren! <<= fuer Leonardo
//int a_pin1 = 5;                       //Lautsprecher Pin definieren! <<= fuer Ivkos Micro Vario

// Fileter Einstellungen!!!
float FehlerV = 3.500 * min_steigen;    //Gewichtung fuer Vario Filter berechnen. 0.1 > FehlerV < 1.0

float mittel_n = 8;                     // Anzahl Werte fuer Mittelwert bilden.
float kal[9];                           // kal[n] ==> n = mittel_n +1
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
long Druck, Druck0, DruckB;

int PinBT,  XOR, c, startCH = 0;
float Vario, VarioR, Hoehe, AvrgV;

unsigned long  dZeit, ZeitE, ZeitS, ZeitPip;
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////




// SETUP//////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  leseZeit = leseZeit - 24;

  Serial.begin(9600);
  //Serial1.begin(9600);

  //pinMode(bt_pin, INPUT);                 // Definiert den Pin für der BT Schalter.
  //PinBT = digitalRead(bt_pin);            // Definiere SChalter Zustand fuer BT.
  PinBT = 0;                              // Wenn keine BT-Modul eingebaut ist - 0.

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

  //BT umbenenen START
  if (PinBT == 0)
  {
    digitalWrite(7, HIGH);               // BT Versorgung einschalten.
    digitalWrite(8, HIGH);               // BT Versorgung einschalten.
    delay(1000);
    //Serial.begin(9600);                  //fuer MiniPro
    //Serial1.begin(9600);                 //fuer BT - Leonardo.
    /*/
      Serial.print("AT");
      delay(1500);
      Serial.print("AT+NAMEIvkosVario");
      delay(500);
      //Serial.print("AT+RESET");
      delay(500);//*/
    // PIN ist 1234 oder 0000 <= #############################################################################
  }
  else
  {
    digitalWrite(7, LOW);               // BT Versorgung einschalten.
    digitalWrite(8, LOW);               // BT Versorgung einschalten.
  }
  //BT umbenenen ENDE */


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
    }
    PipserX();
    //ZeitE = micros();
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
  Druck = bpm.readPressure();
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

  //VarioR=0.500; // Ton Test !!!!!!!!!!!!!!!!!!!!!!!!!################################

  kal[1] = VarioR;

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
// ###############################################################################################################
// ENDE ##########################################################################################################



// Pipser ########################################################################################################
// ###############################################################################################################
void PipserX()
{
  //Vario = 1.00; // Ton Test!

  float frequency = float(max_frqz) / (1.00 + pow((Vario - 11.30) / -6.80, 5.10));
  float duration = (-60.00 * Vario + 500) * 0.75; // Variable Pause
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

}
// ###############################################################################################################
// ENDE ##########################################################################################################



// Bloetooth #####################################################################################################
// ###############################################################################################################
/*  Verschiedene Komuniktionsprotokole möglich.  */

void Bloetooth()
{
  // Start "Blue Fly Vario" sentence ===========================================================================
  // ===========================================================================================================
  /* Ausgabe im BlueFlyVario Format.     The standard BlueFlyVario outp ut mode. This sends raw
    pressure measurements in the form "PRS XXXXX\n": XXXXX is the raw (unfiltered) pressure
    measurement in hexadecimal pascals. */
  // On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.
  //Temp = bpm.readTemperature();
  Druck = bpm.readPressure();

  Serial.print("PRS ");               //Ausgabe an der BT fuer MiniPro.
  Serial.println( Druck, HEX);        //BT-Serial schnitstelle ungefiltert.  Fuer MiniPro.

  //Serial1.print("PRS ");               //Ausgabe an der BT fuer Leonardo.
  //Serial1.println( Druck, HEX);        //BT-Serial schnitstelle ungefiltert.  Fuer Leonardo.

  // Wenn XCSoar vervender wird die Zeile drunter mitt "//..." auskomentieren.
  //delay(leseZeitBT - 22); //Wenn XCTrack benutzt wird Zeille aktiv lassen.

  // Ende "BlueFlyVario" sentence =========================================================================== */

  // =>>

  // Start "LXNAV - LXWP0" sentence ============================================================================
  // ===========================================================================================================
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

  // Start "LK8EX1" sentence ===================================================================================
  // ===========================================================================================================
  // Send $LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.
    SteigenBerechnen();

    String s = "LK8EX1,";
    s = String(s + String(Druck,DEC) + "," + String(Hoehe,2) + "," + String(Vario*100,0) + "," + String(Temp,1) + ",1.1,");

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

    delay(leseZeitBT - 23);
    // Ende "LK8EX1" sentence ================================================================================= */

  // =>>

  // Start "Custom BFV" sentence ===============================================================================
  // ===========================================================================================================
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



  // Start Normale Daten Ausgabe ===============================================================================
  // ===========================================================================================================
  /*/ On-Off | Hier zwischen // ein * setzen dann ist es deaktiviert.

    // Zum Testen ueber Serial-Port !!!-> nicht vergesen VarioR aus zu kommentiren.
    //Temp.[C°];Druck[Pa];Hoehe[m];dZeit[ms];VarioR[m/s];Vario[m/s];BT Taster
    //  Zum Asugabe aktiwieren * zwischen // löschen.

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


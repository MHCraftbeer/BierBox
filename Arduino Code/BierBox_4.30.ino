////////////////
////////////////
////////////////
//anstatt eines arduino uno wird jetzt ein NodeMcu 0.9 dev board verwendet
//benötigt die dateien zum Board über den Boardverwalter um mit der arduino ide zu funktionieren
/*Changelog
 *2.01 Temperaturmessungen, Heizungsregelung, Buzzer, Timer, Button, Automatik Start, Automatik
 *2.02 Automatik verbessert, allg. Verbesserungen
 *2.03 Rührersteuerung integriert, Hopfentimer, Hopfengaben Reminder, Manuell, allg. Verbesserungen
 *2.04 Signal LEDs für Blende, allg. Verbesserungen
 *2.05 allg. Verbesserugen, mit GoBetwino angefangen
 *2.06 GoBetwino integriert (Arduino sendet über USB LogDaten an gobetwino.exe und speichert sie in Datei; http://www.mikmo.dk/gobetwino.html), globale Variablen nachvollziehbarer gemacht, Changelog erstellt, allg. Verbesserungen
 *2.07 paar Bugs entfernt: LED Takt OK, Buzzer Takt OK;
 *2.08 LCD Keypad Shield Steuerung wieder eingebaut, Variablen im Sketch nach oben gestelllt, Keypad funktioniert, Manueller Modus bis auf Timer auch schon
 *2.09 manueller modus fertig, manuellen alarm und meldung eingebaut
 *2.10 manueller modus komplestt fertig, mit automaischen angefangen
 *2.11 viel rumprobieren und fehlersuche
 *2.12 manueller modus und automaisch modus sind komplett fertig und funktionieren 1a
 *2.20 alles läuft soweit ich es feststellen kann
 *2.21 versuche schnelleinstellung für hohe zahlen einzubauen
 *2.30 schnelleinstellung für hohe zahlen fertig eingebaut. alles soweit funktionsfähig. fertig???????
 *2.31 bugs bei automaischen und autokochen beseitigt
 *2.32 gobetwino überarbeitet
 *2.34 NTCs genauer machen
 *2.40 Keine bugs mehr gefunden, soweit ist alles bereit für die generalprobe. lediglich die ausgangswerte müsste man noch anpassen.
 *2.41 Referenzwiderstand von tempT auf 7640 Ohm geändert (ja der ist laut multimeter wirklich so gering)
 *2.42 Gradientenberechnung der Heizungsregelung angepasst. Gradient ist jetzt wirklich K/min. Automaischen, Eingemaischt wird jetzt nach erreichen der temperatur und nicht nach ablauf des ersten timers.
 *2.43 ausprobieren einer pid regelung: http://playground.arduino.cc/Code/PIDLibrary, hat nicht funktioniert
 *2.50 alles funktioniert. Programmspeicher darf 85% nicht überschreiten, sonst funktioniert das relais nicht mehr
 *2.51 Maximaltemperatur auf 102°C erhöht, Referenzwiderstand des Temperatursensors auf 100°C kalibriert
 *2.52 Wenn Kochtemp >= 100 °C ist dann ist die Heizung ununterbrochen eingeschalten, hoch und runterpfeile in Statusmenüs entfernt, heizung ist jetzt mindestens für 5 sekunden an, Softwarenummer im statusmenue angezeigt
 *2.53 hoch und runterpfeile wieder im Statusmenü, temperatur auf eine nachkommastelle gerundet, fehler bei der anzeige behoben
 *2.54 Rührer aus wenn beim kochen hundert grad erreicht werden um den deckel zu wechseln, tempSollErreicht beim kochen für 99,5 grad festgelegt, eingabe max und min beim kochen angepasst, K wert auf 0,75 erhöht
 *2.55 Timer sollten die Laufzeit des Programms jetzt berücksichtigen und dadurch genauer sein, unnötige unsigned long Varibalen zu int gemacht, Gobetwino Ausgabe auf gerundete Temperaturwerte angepasst
 *2.56 timer änderungen von 2.55 wieder rückgängig gemacht, da der Timer so nur noch ungenauer wurde, pin belegung geändert, Referenzwiderstand von NTC der Elektronik geändert (= der des Topfes)
 *-
 *-
 *3.00 AB HIER FORK FÜR ESP32: anpassen der software um mit dem espressif esp32 zu funktionieren, wifi, ubidots.com, ds18b20 statt ntc
 *3.01 Thingspeak.com anstatt ubidots.com eingebaut
 *3.02 Option um die Daten im Thinkspeak.com Channel zu löschen eingebaut, Startwerte angepasst
 *3.02 Keypad Shield Werte und Tasten geändert, Button "Select" funktioniert mit esp32 nicht, temperaturmessung überarbeitet die frisst sehr viel rechenzeit
 *3.03 Temperatur wird noch alle halbe sec gemessen
 *3.04 Thermostat-Funktion eingebaut
 *3.05 Web Interface für Thermostat eingebaut
 *-
 *Konnte Display nicht am ESP32 zum laufen bringen, immer wieder darstellungsfehler, besonders bei selbst programmierten zeichen
 *-
 *4.00 AB HIER FORK FÜR ESP8266
 *4.01 auf die wenigen pins des boards reduziert, nur noch pins für 1x motor, heizung, kühlung und buzzer übrig gewesen
 *4.02 läuft soweit, fehlerhaftes durchheizen während einmaischen entfernt, regelungen überarbeitet 
 *4.03 wifimanager eingebaut,funktioniert aber auch weiterhin offline, gobetwino entfernt (Serielle ausgabe funkioniert ohnehin nicht mehr)
 *4.04 thingspeak daten können im wifimanager eingegeben werden
 *4.05 viele kleine Verbesserungen, ua. beim webserver und display
 *4.06 Rühren während des Kochens deaktiviert
 *4.07 Rühren komplett entfernt, dh der rührer wird nur noch manuell am schalter aus und ein geschalten
 *4.08 Timer korrigiert wegen ungenauigkeiten durch tempmessung, Temperaturmessung Genauigkeit auf 0,25 °C reduziert (messdauer 190ms statt 750ms), Automaischenmenue korrigiert
 *4.09 WebUpdate eingebaut
 *4.10 fehlerhaftes update
 *4.11 alles funktioniert soweit wieder
 *4.12 Die Modi über einen Webserver betreiben
 *4.13 Webinterface: manuell, thermostat, update, log und zugang löschen sind fertig
 *4.14 I2C LCD display statt keypad shield
 *4.15 Webinterface: automaischen ist fertig
 *4.16 Webinterface: alles ist fertig, muss noch unter realen bedingungen getestet werden.
 *4.17 thingspeak.com daten können im Webinterface eingesehen werden, thingspeak log kann über webinterface gelöscht werden, webinterface kosmetik
 *4.18 Gradientenberchnung verbessert, wird nur noch jede minute berechnet. gradient ist entsprechend höher, k angepasst
 *4.19 Box kann beim Startup auf offline geschalten werden
 *4.20 Kochtemperatur wird schon bei 98 Grad erreiht
 *4.21 heizung ist während des einmaischens aus
 *4.22 ntp und rtc für genauen timer eingefügt, hopfengaben alarm repariert
 *4.23 I2C pins geändert, D3 kann probleme machen (D2->D1, D3->D2)
 *4.24 filesystem überarbeitet, custom parameter sollten zuverlässiger geladen werden 
 *4.25 thingspeak überträgt jetzt auch immer einen Zeitstempel als status
 *4.26 thermostat hysterese überarbeitet, webinterface aktualisiert nur noch alle 5 min automatisch, ntp updateintervall entfernt
 *4.27 Fehlerhafter Build
 *4.27.2 string() statt dtostrf(), hopfengabe timer repariert, timer des manuellen modus wieder on the fly veränderbar, offline timer repariert
 *4.28 Fehlerhafter Build
 *4.29 Webinterface überarbeitet
 *4.30 Hopfentimer endgültig repariert
 */
 float software = 4.30;
 /*TODO:
 *  -Erledigt: Heizungsregelung einstellen dh. richtigen Kwert finden. 
 *  -Erledigt: Temperaturmessung kalibrieren dh. Referenzwiderstand richtig einstellen. mit 100°C kalibriert
 *  -Erledigt: Abkühlung druch das Malz mit einbeziehen, dh den timer vom zweiten automaischen Schritt gleich nach dem einmaischen zu starten und nicht erst wenn die temp erreicht wurde
 *  -Erledigt: min und max werte an reale werte anpassen 
 *  -Erledigt: gobetwino testen
 *  -Erledigt: standardmäßg rührer stufe 0 ist nervig, eigentlich muss immer gerührt werden
 *  -Erledigt (hat aber nicht funktioniert, messung war mit gemessenem Widerstand total falsch):genauen Referenzwiderstand messen
 *BUGS:
 *  -Behoben: Fehlerhafter timer im offline Betrieb
 *  -Behoben: Hopfentimer stimmt nicht, nach erstem timer wird der wert nicht richtig übergeben
 *  -Behoben: programmende macht probleme bei timer, zu sehen bei zwei mal hintereinander manuell
 *  -Behoben: timer manuell kann nicht mehr on the fly geändert werden
 *  -Behoben: Button Select funktioniert nicht --> stattdessen button right für ok verwenden
 *  -Behoben: tune funktioniert nicht (buzzer) -->passiven buzzer verwenden
 *  -Behoben: analog reference funktioniert nicht -->ds18b20 Temperatursensor verwenden
 *  -Behoben: ds18b20 frisst sehr viel Rechenzeit--> nor noch alle 1000ms eine Messung
 *  -Behoben: timer für die zweite rast wartet nicht auf die einmaischenfertig bestätigung sondern fängt gleich an
 *  -Behoben: nach einem durchlauf manuell lässt sich kein zweiter starten
 *  -Behoben: (rtc eingebaut) Timer ist ungenau, geht etwa fünf minuten pro stunde vor, wahrscheinlich wegen der Laufzeit des Programms
*/
//////////////////////////////
///////////////////////////////
///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// Bibliotheken
//
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#include <ESP8266WiFi.h> //für Wlan
#include <WiFiClient.h>
#include <ESP8266mDNS.h>
#include <ESP8266HTTPUpdateServer.h>
#include <OneWire.h> //für ds18b20 Temperatursensor
#include <DallasTemperature.h> //für ds18b20 Temperatursensor
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x3F,16,2);
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>          //https://github.com/tzapu/WiFiManager
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson

#include <RTClib.h>
RTC_DS1307 RTC;
#include <NTPClient.h>
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "europe.pool.ntp.org", 3600); //WiFiUPD, Offset in sec, Updateintervall in ms


ESP8266WebServer httpServer(80);
ESP8266HTTPUpdateServer httpUpdater;
const char* host = "bierbox";

bool online = true;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// Hardware Ports
//
const byte keypadPin = A0;  // Analog Keypad
const byte bmpSDA = D1;
const byte bmpSCL = D2;
//tempmessung
const byte oneWirePin = D5;
//relais (einfach über bsp "blinkwithoutdelay" bestimmen)
const byte heizungPin = D9;
const byte kuehlungPin = D10;
//buzzer
const byte buzzerPin = D7;
//leds (einfach über bsp "blinkwithoutdelay" bestimmen)
//////////////////////////////
///////////////////////////////
///////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////// Variablen
//////////////////////////////////////////////////////////////////////////////Eingabe
byte tempMin = 20; //minimal Temperatur
byte tempMax = 85;
byte tempKochenMin = 20;
byte tempKochenMax = 102;
byte timerMin = 1; //minimal Timer
byte timerMax = 180;
byte timerKochenMin = 1;
int zaehlerTakt = 150; //alle x ms steigt der zähler um eins
unsigned long zaehlerStart = 0;
byte button;
bool buttonPressed = false;
bool meldung = false;
bool automatikBestaetigung = false;
//
//////////////////////////////////////////////////////////////////////////////TemperaturSensoren
unsigned long sensorPreviousTime = 0;
int messIntervall = 1000;
float temperature;
OneWire oneWire(oneWirePin);
DallasTemperature sensors(&oneWire);
//////////////////////////////////////////////////////////////////////////////Temperaturregelung
float gradient = 0.0; 
float k = 0.5; //Gradientenfaktor
int heizungMesszyklus = 60000; //Abstand der Temperaturmessung für die Gradientenberechnung in ms
unsigned long heizungVorherigeZeit = 0; //Zeit bei t-1
float heizungVorherigeTemp = 0; //Temp bei t-1
unsigned long heizungSTimer = 0; //Wartezeitmessung Start
bool heizungOFF = false;
byte tempSoll = 50;
byte tempSensorDaempfung = 0; //dämpft überschwingen vom Temperatursensor
bool tempSollErreicht = false;
byte hysterese = 0;  //für den thermostaten
unsigned long kuehlungESVTimer = 0;
unsigned long kuehlungESDTimer = 0;
int kuehlungESV = 600000; //10 min
int kuehlungESD = 300000; //5 min
bool kuehlungStatus = false;
bool kuehlungJa = true;
unsigned long heizungESDTimer = 0;
bool heizungStatus = false;
bool thermostatStart = false;
bool heizungJa = true;
//
////////////////////////////////////////////////////////////////////////////Buzzer Takt
int buzzerFrequenz = 831; //buzzerFrequenz des Peips in Hz
byte buzzerDauer = 150; //Piep-Dauer in ms
byte buzzerIntervall = 200; //Pause zwischen Piep in ms
bool alarmOFF = false;
bool alarmON = false;
unsigned long buzzerPreviousMillis = 0;
bool buzzerON = false;
//
//////////////////////////////////////////////////////////////////////////////////////Timer
int timerIntervall = 860; //korrigierte Sekunde
bool datenUebergeben = false;
int timerMinuten;  //minutenanzeige
bool timerStart1 = false;
byte timerSekunden = 0; //sekundenanzeige
unsigned long timerSet = timerMin;
unsigned long timerPreviousMillis = 0;
bool timerEnde = false;
bool timerInt = false;
unsigned long timerGo;
int timer0;
//
///////////////////////////////////////////////////////////////////////////////////Hopfen Timer
unsigned long timerHopfenSet = timerMin;
unsigned long timerHopfenPreviousMillis = 0;
bool timerHopfenEnde = false;
int timerHopfenMinuten;
bool timerHopfenStart = false;
byte timerHopfenSekunden;
bool timerHopfenInt = false;
unsigned long timerHopfenGo;
int timerHopfen0;
//
//////////////////////////////////////////////////////////////////////////////////AutoMaischen Modus
bool einmaischenPause = false;
bool keinHopfenMehr = false;
byte autoMaischenStatus = 1; //für Statusanzeige auf display
bool autoMaischenStartDruecken = false;
bool autoMaischenStart = false;
bool autoMaischenEnde = false;
bool jetztLaeutern = false;
byte tempEinmaischen = 56;  //StartTemperaturen
byte tempStep1 = 54;   
byte tempStep2 = 64; 
byte tempStep3 = 72; 
byte tempStep4 = 78; 
byte tempStep5 = tempMin; 
byte tempStep6 = tempMin; 
int timerStep1 = 10;  //StartTimer Min
int timerStep2 = 60;
int timerStep3 = 20;
int timerStep4 = 5;
int timerStep5 = timerMin;
int timerStep6 = timerMin;
bool einmaischenJetzt = false;
bool einmaischenFertig = false;
byte currentStep = 1;
byte letzterStep;
byte tempStepMax; //maximaltemperatur während automatik
int timerStepMax;  //gesamtdauer der automatik
bool maximaMaischen = false;  //berechnet die drei vorherigen werte
//
//////////////////////////////////////////////////////////////////////////////////AutoKochen Modus
int timerKochen = 90;
byte tempKochen = 100;
byte autoKochenStatus = 1; //für Statusanzeige auf display
bool autoKochenStartDruecken = false;
int hopfenAlarm1;
int hopfenAlarm2;
int hopfenAlarm3;
int hopfenAlarm4;
int timerHopfen1 = 20;
int timerHopfen2 = 15;
int timerHopfen3 = 10;
int timerHopfen4 = timerMin;
int timerHopfen5 = timerMin;
bool maximaKochen = false;
bool autoKochenStart = false;
bool autoKochenEnde = false;
byte hopfen = 1;
byte letzterHopfen = 1;
byte autoKochenMenue = 1;
//
//////////////////////////////////////////////////////////////////////////////Manueller Modus
bool manuellStart = false;
byte manuellTemp = tempMin;
bool manuellEnde = false;
int timerManuell = 40;
//
/////////////////////////////////////////////////////////////////////////////Status Menü
byte statusMenue = 1;
//
/////////////////////////////////////////////////////////////////////////////Wifi
unsigned long reconnectPreviousMillis = 0;
//
///////////////////////////////////////////////////////////////////////////////Thingspeak.com
String str_thingspeakReadKey = "na";
String str_thingspeakUserKey = "na";
String str_thingspeakWriteKey = "na";
String str_thingspeakChannelID = "na";
char thingspeakWriteKey[40];
char thingspeakReadKey[40];
char thingspeakUserKey[40];
char thingspeakChannelID[10];
bool shouldSaveConfig = false;
bool wifiVerbunden = false;
const char* serverThingspeak = "api.thingspeak.com";
unsigned long thingspeakTime = 0;
char buffer[20];
unsigned long thingspeakPreviousMillis = 0;
bool thingspeakChannelClear = false; //Löscht die Daten wenn true
int thingspeakIntervall = 20000;
WiFiClient client;
//
////////////////////////////////////////////////////////////////////////////Webinterface
//
WiFiServer server(80);
bool einstellungen = false;
bool startUP = true;
bool webinterfaceStart = false;
bool webupdaterStart = false;
bool gotValue = false;
bool noUpdate = true;
bool noAlarm = false;
bool check = false;
byte graphTage = 1;
////////////////////////////////////////////////////////////////////////////Userinterface
unsigned long runtime;
byte hMenueMax = 5;
byte manuellMenue = 1;
byte thermostatMenue = 1;
byte autoMaischenMenue = 1;
bool hMenueON = true;
bool nMenueON = false;
byte hMenue = 1;
byte nMenue = 1;
byte meldungsNummer = 1;
byte ae[8] ={ //Buchstabe "ä"
  B01010,
  B00000,
  B01110,
  B00001,
  B01111,
  B10001,
  B01111,};
byte ue[8] ={ //Buchstabe "ü"
  B01010,
  B00000,
  B10001,
  B10001,
  B10001,
  B10011,
  B01101,};
byte grad[8] ={ //Grad Zeichen
  B00010,
  B00101,
  B00010,
  B00000,
  B00000,
  B00000,
  B00000,};
byte pfeil[8] = { //Größer Zeichen, OK
  B00000,
  B00100,
  B00010,
  B00001,
  B00010,
  B00100,
  B00000,};
byte up[8] = { //Pfeil Zeichen hoch
  B00000,
  B00000,
  B00100,
  B01010,
  B10001,
  B00000,
  B00000,};
byte down[8] = { //Pfeil Zeichen runter
  B00000,
  B00000,
  B00000,
  B10001,
  B01010,
  B00100,
  B00000,};
byte soll[8] = { //Pfeil rechts
  B00000,
  B00100,
  B00010,
  B11111,
  B00010,
  B00100,
  B00000,};
byte pm[8] = { //PlusMinus
  B00000,
  B00100,
  B01110,
  B00100,
  B00000,
  B01110,
  B00000,};
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////LCD Keypad Shield///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Button eingaben
//Keypad Shield
#define NONE       0  //kein Button wird gedrückt
#define RIGHT      1 //RIGHT (ungenutzt)
#define HOCH       2 //UP
#define RUNTER     3 //DOWN
#define OK         4 //LEFT
#define ZURUECK    5 //SELECT
int keypad() {
    int x;
    x = analogRead(keypadPin);
    if (x < 300) return NONE;
    else if (x < 450) return HOCH;
    else if (x < 600 ) return OK;
    else if (x < 850) return RUNTER;
    else if (x > 900) return ZURUECK;
    return NONE;
}
//Buttoneingabe verarbeiten
void eingabe() {
  button = keypad();
  switch(button) {
    //////////////////////
    ///////////////////////
    ////////////////////////
    /////////////////////////
    case ZURUECK: {  ///////////////Zurück
      if (!buttonPressed){
        buttonPressed = true;
        //////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////allgeimein
        if (alarmON) { /////////////////////////////////////////////schaltet alarm aus bei Meldungen, identisch mit dem code bei zurück
          alarmOFF = true;
          alarmON = false;
          if (meldungsNummer == 1) {  //bestätigt das das programm fertig ist "Programm beendet"
              autoKochenMenue = 1;
              manuellMenue = 1;
          hMenueON = true;//zurück ins Hauptmenü
          nMenueON = false;
          }
          if (meldungsNummer == 2 && einmaischenJetzt) { //bestätigt das eingemaischt wird "jetzt einmaischen"
            einmaischenJetzt = false;
            meldung = true;
          }
          if (meldungsNummer == 3) { //bestätigt das geläutert wird "jetzt läutern", das program ist dann fertig und geht zurück ins hauptmenue
            meldungsNummer = 1;
            autoMaischenMenue = 1;
            jetztLaeutern = false;
            hMenueON = true;//zurück ins Hauptmenü
            nMenueON = false;
          }
          break;        
        }
        if (meldung && automatikBestaetigung) { //bricht den start der Automatik ab
          meldung = false;
          automatikBestaetigung = false;
          break;
        }
        if (meldung && meldungsNummer == 6 || meldungsNummer == 5) { //daten werden nicht gelöscht
          meldungsNummer = 1;
          meldung = false;
          if (statusMenue > 1)statusMenue--;
          break;
        }
        //
        //////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////Menüsteuerung Status
        //
        //
        //
        //
        if (!hMenueON && nMenueON && hMenue == 5) {// gehtwieder ins Hauptmenü
          if (statusMenue > 1) statusMenue--;
          else {
            hMenueON = true;
            nMenueON = false;
          }
          break;
        }
        //////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////Menüsteuerung Manuell
        //
        //
        //
        if (!hMenueON && nMenueON && hMenue == 1) {
          if (meldung) {  //deaktiviert die meldung aber bleibt im manuellen Modus
              meldung = false;
              manuellMenue = 1;
              break;
          }
          //
          if (!meldung && !alarmON) {   //deaktiviert diesen part wenn alarm oder meldung ausgegeben wird
            if (!hMenueON && nMenueON && hMenue == 1) { //Verstellt das manuellMenü
              if (manuellMenue > 0) {
                if (manuellStart && manuellMenue == 4) {
                  manuellMenue = 2;
                  timerManuell = timerMinuten;
                }
                else manuellMenue--;
              }
              if (manuellStart && manuellMenue == 0) {
                manuellMenue = 1;
                meldung = true;  //warnt vor beenden des manuellen modus
              }
              if (manuellMenue == 0) {  // gehtwieder ins Hauptmenü
                manuellMenue = 1;
                hMenueON = true;
                nMenueON = false;
              }
            break;  
            }
          }
        }
        //
        ////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////MenüSteuerung Automaischen
        //
        //
        //
        if (!hMenueON && nMenueON && hMenue == 2){
          if (!autoMaischenStart && autoMaischenMenue > 1) {  //Verstellt das automaischmenü
            if (autoMaischenStartDruecken) autoMaischenStartDruecken = false;
            autoMaischenMenue--;
            break;
          }
          if(autoMaischenMenue == 1) { //zurück ins Hauptmenü
            hMenueON = true;
            nMenueON = false;
            break;
          }
          if (autoMaischenStart && !meldung) {  // warnt vor Beenden der Automatik
            meldung = true;
            break;
          }
          if (autoMaischenStart && meldung && meldungsNummer != 2) {  //schließt die meldung ohne Automaik zu beenden
            meldung = false;
            break;
          }
        }
       //
        ////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////MenüSteuerung AutoKochen
        //
        //
        //
        if (!hMenueON && nMenueON && hMenue == 3){
          if (!autoKochenStart && autoKochenMenue > 1) {  //Verstellt das automaischmenü
            if (autoKochenStartDruecken) autoKochenStartDruecken = false;
            autoKochenMenue--;
            break;
          }
          if(autoKochenMenue == 1) { //zurück ins Hauptmenü
            hMenueON = true;
            nMenueON = false;
            break;
          }
          if (autoKochenStart && !meldung) {  // warnt vor Beenden der Automatik
            meldung = true;
            break;
          }
          if (autoKochenStart && meldung) {  //schließt die meldung ohne Automaik zu beenden
            meldung = false;
            break;
          }
        }
        //////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////Menüsteuerung Thermostat
        //
        //
        //
        if (!hMenueON && nMenueON && hMenue == 4) {
          if (meldung) {  //deaktiviert die meldung aber bleibt im  Thermostat Modus
              meldung = false;
              thermostatMenue = 1;
              break;
          }
          //
          if (!meldung && !alarmON) {   //deaktiviert diesen part wenn alarm oder meldung ausgegeben wird
            if (!hMenueON && nMenueON && hMenue == 4) { //Verstellt das  ThermostatMenü
              if (thermostatMenue > 0) {
                if (thermostatStart && thermostatMenue == 6) thermostatMenue = 4;
                else thermostatMenue--;
              }
              if (thermostatStart && thermostatMenue == 0) {
                thermostatMenue = 1;
                meldung = true;  //warnt vor beenden des thermostat modus
              }
              if (thermostatMenue == 0) {  // gehtwieder ins Hauptmenü
                thermostatMenue = 1;
                hMenueON = true;
                nMenueON = false;
              }
            break;  
            }
          }
        }
      }
      break;
    }
    //////////////////////
    ///////////////////////
    ////////////////////////
    /////////////////////////
    case OK: {      ///////////////OK
      if (!buttonPressed){
        buttonPressed = true;
        ////////////////////////////////////////////////////////
        //////////////////////////////////////////////////////////////////////////////////////////////////////////allg. Menüsteuerung
        //
        //
        //
        if (meldung) { //schließt meldungen die mit Ok bestätigt werden müssen
          if ((autoMaischenStart || manuellStart || autoKochenStart || thermostatStart) && meldungsNummer != 2 && meldungsNummer != 3) { //bricht laufenden Modus ab und geht ins Hauptmenü, schaltet alles wieder auf anfang
            if (autoMaischenStart) autoMaischenEnde = true;                                                //"zum beenden ok drücken"
            autoMaischenMenue = 1;
            if (autoKochenStart) autoKochenEnde = true;
            autoKochenMenue = 1;
            if (manuellStart) manuellEnde = true;
            manuellMenue = 1;
            thermostatMenue = 1;
            timerEnde = false;
            timerStart1 = false;
            meldungsNummer = 1;
            hMenueON = true; //geht zurück ins Hauptmenue
            nMenueON = false;
            if (thermostatStart) {
              meldung = false;
              thermostatStart = false;
            }
            break;
          }
          if (automatikBestaetigung) { //übersicht vor automatik start, startet bei ok automatik
            meldung = false;
            automatikBestaetigung = false;
            if (hMenue == 2) {
              autoMaischenStart = true;
            }
            if (hMenue == 3) {
              autoKochenStart = true;
            }
            break;
          }
          if (autoMaischenStart && !einmaischenFertig) { //bestätigung das man mit dem einmaischen fertig ist
            meldung = false;
            einmaischenFertig = true;
            tempSollErreicht = true;
            meldungsNummer = 1;
            datenUebergeben = false;
            einmaischenPause = false;
            break;
          }
          if (meldungsNummer == 5) { //thingspeak.com daten werden gelöscht
            thingspeakChannelClear = true;
            meldungsNummer = 1;
            meldung = false;
            statusMenue = 1;
            hMenueON = true;
            nMenueON = false;
            break;
          }
          if (meldungsNummer == 6) { //wlan daten werden gelöscht
            WiFiManager wifiManager;
            wifiManager.resetSettings();
            delay(10);
            wifiManager.resetSettings();
            delay(10);
            SPIFFS.format();
            break;
          }
        }
        if (alarmON) { /////////////////////////////////////////////schaltet alarm aus bei Meldungen, identisch mit dem code bei zurück
          alarmOFF = true;
          alarmON = false;
          if (meldungsNummer == 1) {  //bestätigt das das programm fertig ist "Programm beendet"
              autoKochenMenue = 1;
              manuellMenue = 1;
          hMenueON = true;//zurück ins Hauptmenü
          nMenueON = false;
          }
          if (meldungsNummer == 2 && einmaischenJetzt) { //bestätigt das eingemaischt wird "jetzt einmaischen"
            einmaischenJetzt = false;
            meldung = true;
          }
          if (meldungsNummer == 3) { //bestätigt das geläutert wird "jetzt läutern", das program ist dann fertig und geht zurück ins hauptmenue
            autoMaischenMenue = 1;
            jetztLaeutern = false;
            meldungsNummer = 1;
            hMenueON = true;//zurück ins Hauptmenü
            nMenueON = false;
          }
          break;          
        }
        if (!meldung && !alarmON) { //deaktiviert diesen part wenn alarm oder meldung ausgegeben wird
          if (hMenueON && !nMenueON && !autoMaischenStart && !manuellStart && !autoKochenStart) {  //geht vom Hauptmenü ins Nebenmenü
            hMenueON = false;
            nMenueON = true;
            break;
          }
              //////////////////////////////////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////////////////Menüsteuerung Status
            //
            //
            //
            //
            if (!hMenueON && nMenueON && hMenue == 5) {// geht durchs Menü
              if (wifiVerbunden) {
                if (statusMenue < 5 ) {
                  statusMenue++;
                  break;
                }
              }
              else if (statusMenue < 4 ) statusMenue++;
              break;
            }
          ///////////////////////////////////////////////////////
          ////////////////////////////////////////////////////////////////////////////////////////////////////Menüsteuerung Manuell
          //
          //
          //
            if (!hMenueON && nMenueON && hMenue == 1) {  //Verstellt das manuellmenü
              if (manuellMenue < 3) {
                if (manuellStart && manuellMenue == 2) {
                  manuellMenue = manuellMenue + 2;
                  datenUebergeben = false;
                  timerInt = false;
                }
                else manuellMenue++;
                break;
              }
            }
            if (!hMenueON && nMenueON && hMenue == 1 && manuellMenue == 3 && !manuellEnde && !manuellStart) { //Startet den modus Manuell
              manuellStart = true;
              manuellMenue++;
              break;
            }
            ///////////////////////////////////////////////////////
          ////////////////////////////////////////////////////////////////////////////////////////////////////Menüsteuerung Thermostat
          //
          //
          //
            if (!hMenueON && nMenueON && hMenue == 4) { 
              if (thermostatMenue < 5) {
                if (thermostatStart && thermostatMenue == 4) thermostatMenue += 2;
                else thermostatMenue++;
                break;
              }
            }
            if (!hMenueON && nMenueON && hMenue == 4 && thermostatMenue == 5) { 
              thermostatStart = true;
              thermostatMenue++;
              break;
            }
          ///////////////////////////////////////////////////
          //////////////////////////////////////////////////////////////////////////////////////////////////Menüsteuerung AutoMaischen
          //
          //
          //
          if (!manuellStart && !autoKochenStart) {
            if (!hMenueON && nMenueON && hMenue == 2) {       
              if (!autoMaischenStartDruecken && autoMaischenMenue < 18 && !autoMaischenStart)   autoMaischenMenue++;  //überprüft ob Automaischen gestartet werden soll
              else if (!autoMaischenStart) {
                automatikBestaetigung = true;
                autoMaischenStartDruecken = false;
                maximaMaischen = true;
                meldung = true;
                }
              break;
            }
          }
          ///////////////////////////////////////////////////
          //////////////////////////////////////////////////////////////////////////////////////////////////Menüsteuerung AutoKochen
          //
          //
          //
          if (!manuellStart && !autoMaischenStart) {
            if (!hMenueON && nMenueON && hMenue == 3) {              
              if (!autoKochenStartDruecken && autoKochenMenue < 10 && !autoKochenStart)   autoKochenMenue++;  //überprüft ob Automaischen gestartet werden soll
              else if (!autoKochenStart) {
                automatikBestaetigung = true;
                autoKochenStartDruecken = false;
                maximaKochen = true;
                meldung = true;
                }
              break;
            }
          }
        }
        break;
      }
      break;
    }
    //////////////////////
    ///////////////////////
    
    ////////////////////////
    /////////////////////////
    case HOCH: {  ////////////////Hoch bzw +
        if (!meldung && !alarmON) { //deaktiviert diesen part wenn alarm oder meldung ausgegeben wird
          ////////////////////////////////////////////////
          ////////////////////////////////////////////////////////////////////////////////////////////////////Verstellt das Hauptmenü
          //
          //
          //
          if (!buttonPressed){//
          if (hMenueON && !nMenueON) {
            if (hMenue > 1) hMenue--;
            else hMenue = hMenueMax;
            buttonPressed = true;//
          }
          }
          ////////////////////////////////////////////////
          ///////////////////////////////////////////////////////////////////////////////////////verstellt Variablen im Moudus Manuell
          //
          //
          //
          if (!hMenueON && nMenueON && hMenue == 1) {
            if (manuellMenue == 1) { //Verstellt die Temperatur für Manuell+
              if (manuellTemp < tempKochenMax) {
                if (millis() - zaehlerStart >= zaehlerTakt) {
                  if (!buttonPressed) zaehlerStart = millis();
                  manuellTemp++;//
                }
              }
              else manuellTemp = tempMin;
            }
            if (manuellMenue == 2) { //Verstellt den Timer für Manuell+
              if (timerManuell < timerMax) {
                if (millis() - zaehlerStart >= zaehlerTakt){
                  if (!buttonPressed) zaehlerStart = millis();
                  timerManuell++;
                  //datenUebergeben = false;
                  //timerInt = false;
                }
              }
              else {
                timerManuell = timerMin;
               // datenUebergeben = false;
               // timerInt = false;
              }
            }
          }
          ////////////////////////////////////////////////
          ///////////////////////////////////////////////////////////////////////////////////////verstellt Variablen im Modus Thermostat
          //
          //
          //
          if (!hMenueON && nMenueON && hMenue == 4) {
            if (thermostatMenue == 1) { //Verstellt die Temperatur für thermostat+
              if (manuellTemp < tempKochenMax) {
                if (millis() - zaehlerStart >= zaehlerTakt) {
                  if (!buttonPressed) zaehlerStart = millis();
                  manuellTemp++;//
                }
              }
              else manuellTemp = 0;
            }
            
            if (thermostatMenue == 2) { //Verstellt die Kühlung
              if (!buttonPressed) {
              if (kuehlungJa) kuehlungJa = false;
              else kuehlungJa = true;
              buttonPressed = true;
              }
            }
            if (thermostatMenue == 3) { //Verstellt die heizung
              if (!buttonPressed) {
              if (heizungJa) heizungJa = false;
              else heizungJa = true;
              buttonPressed = true;
              }
            }
            if (thermostatMenue == 4) { //Verstellt die hysterese für thermostat+
              if (hysterese < 5) {
                if (millis() - zaehlerStart >= zaehlerTakt) {
                  if (!buttonPressed) zaehlerStart = millis();
                  hysterese++;//
                }
              }
              else hysterese = 0;
            }
          }
          ////////////////////////////////////////////////
          ///////////////////////////////////////////////////////////////////////////////////////verstellt Variablen im Modus AutoMaischen hoch
          //
          //
          //
          if (!hMenueON && nMenueON && hMenue == 2) {  //definition für AutoMaischen modus
            if (autoMaischenStart) {
              if (!buttonPressed) {
              if (autoMaischenStatus > 1) autoMaischenStatus--;
              else autoMaischenStatus = 2;
              buttonPressed = true;
              }
            }
            if (autoMaischenStartDruecken) {
              if (!buttonPressed) {
                autoMaischenStartDruecken = false;
                buttonPressed = true;
            }
            }
            else if (!autoMaischenStart){
              switch(autoMaischenMenue) {
                case 1: {  //hoch von tempEinmaischen
                  if (tempEinmaischen < tempMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempEinmaischen++;
                    }
                  }
                  else tempEinmaischen = tempMin;
                  break;
                }
                case 2: {  //hoch von tempStep////////////////////////////////////Step1
                  if (tempStep1 < tempMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep1++;
                    }
                  }
                  else tempStep1 = tempMin;
                  break;
                }
                case 3: { // hoch von timerstep
                  if (timerStep1 < timerMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep1++;
                    }
                  }
                  else timerStep1 = timerMin;
                  break;
                }
                case 4: {  //hoch von tempStep///////////////////////////////////Step2
                  if (tempStep2 < tempMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt){
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep2++;
                    }
                  }
                  else tempStep2 = tempMin;
                  break;
                }
                case 5: { // hoch von timerstep
                  if (timerStep2 < timerMax){
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep2++;
                    }
                  }
                  else timerStep2 = timerMin;
                  break;
                }
                case 6: { //wechselt auf start
                  if (!buttonPressed) {
                  autoMaischenStartDruecken = true;
                  buttonPressed = true;
                  }
                  break;
                }
                case 7: {  //hoch von tempStep///////////////////////////////Step3
                  if (tempStep3 < tempMax){
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep3++;
                    }
                  }
                  else tempStep3 = tempMin;
                  break;
                }
                case 8: { // hoch von timerstep
                  if (timerStep3 < timerMax){
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep3++;
                    }
                  }
                  else timerStep3 = timerMin;
                  break;
                }
                case 9: { //wechselt auf start
                  if (!buttonPressed) {
                  autoMaischenStartDruecken = true;
                  buttonPressed = true;
                  }
                  break;
                }
                case 10: {  //hoch von tempStep///////////////////////////Step4
                  if (tempStep4 < tempMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep4++;
                    }
                  }
                  else tempStep4 = tempMin;
                  break;
                }
                case 11: { // hoch von timerstep
                  if (timerStep4 < timerMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt){
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep4++;
                    }
                  }
                  else timerStep4 = timerMin;
                  break;
                }
                case 12: { //wechselt auf start
                  if (!buttonPressed) {
                  autoMaischenStartDruecken = true;
                  buttonPressed = true;
                  }
                  break;
                }
                case 13: {  //hoch von tempStep//////////////////////////////////Step5
                  if (tempStep5 < tempMax){
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep5++;
                    }
                  }
                  else tempStep5 = tempMin;
                  break;
                }
                case 14: { // hoch von timerstep
                  if (timerStep5 < timerMax){
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep5++;
                    }
                  }
                  else timerStep5 = timerMin;
                  break;
                }
                case 15: { //wechselt auf start
                  if (!buttonPressed) {
                  autoMaischenStartDruecken = true;
                  buttonPressed = true;
                  }
                  break;
                }
                case 16: {  //hoch von tempStep//////////////////////////////////Step6
                  if (tempStep6 < tempMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep6++;
                    }
                  }
                  else tempStep6 = tempMin;
                  break;
                }
                case 17: { // hoch von timerstep
                  if (timerStep6 < timerMax){
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep6++;
                    }
                  }
                  else timerStep6 = timerMin;
                  break;
                }//bei case 19 gibts nichts zu verstellen
              }
            }
          }
          ////////////////////////////////////////////////
          ///////////////////////////////////////////////////////////////////////////////////////verstellt Variablen im Modus AutoKochen hoch
          //
          //
          //
          if (!hMenueON && nMenueON && hMenue == 3) {  //definition für AutoKochen modus
            if (autoKochenStart) {
              if (!buttonPressed) {
              if (autoKochenStatus > 1) autoKochenStatus--;
              else autoKochenStatus = 2;
              buttonPressed = true;
              }
            }
            if (autoKochenStartDruecken) {
              if (!buttonPressed) {
              autoKochenStartDruecken = false;
              buttonPressed = true;
              }
            }
            else if (!autoKochenStart){  
              switch(autoKochenMenue) {
                case 1: {  //hoch von tempKochen////////////////////////////////////kochtemperatur
                  if (tempKochen < tempKochenMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempKochen++;
                    }
                  }
                  else tempKochen = tempKochenMin;
                  break;
                }
                case 2: { // hoch von kochdauer
                  if (timerKochen < timerMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerKochen++;
                    }
                  }
                  else timerKochen = timerKochenMin;
                  break;
                }
                case 3: { // hoch von timer hopfen1
                  if (timerHopfen1 < timerMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerHopfen1++;
                    }
                  }
                  else timerHopfen1 = timerMin;
                  break;
                }
                case 4: { //wechselt auf start
                  if (!buttonPressed) {
                  autoKochenStartDruecken = true;
                  buttonPressed = true;
                  }
                  break;
                }
                case 5: { // hoch von timer hopfen2
                  if (timerHopfen2 < timerMax){
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerHopfen2++;
                    }
                  }
                  else timerHopfen2 = timerMin;
                  break;
                }
                case 6: { //wechselt auf start
                  if (!buttonPressed) {
                  autoKochenStartDruecken = true;
                  buttonPressed = true;
                  }
                  break;
                }
                case 7: { // hoch von timerhopfen3
                  if (timerHopfen3 < timerMax){
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerHopfen3++;
                    }
                  }
                  else timerHopfen3 = timerMin;
                  break;
                }
                case 8: { //wechselt auf start
                  if (!buttonPressed) {
                  autoKochenStartDruecken = true;
                  buttonPressed = true;
                  }
                  break;
                }
                case 9: { // hoch von timerhopfen4
                  if (timerHopfen4 < timerMax) {
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerHopfen4++;
                    }
                  }
                  else timerHopfen4 = timerMin;
                  break;
                }//bei case 10 gibts nichts zu verstellen
              }
            //}
          }
        }
      }
      break;
    }
    //////////////////////
    ///////////////////////
    ////////////////////////
    /////////////////////////
    case RUNTER: {  ////////////////////////////////Runter bzw -
      //if (!buttonPressed){
        //buttonPressed = true;
        if (!meldung && !alarmON) { //deaktiviert diesen part wenn alarm oder meldung ausgegeben wird
          ///////////////////////////////////////////////////
          /////////////////////////////////////////////////////////////////////////////////////////////////////////allg Menüsteuerung
          //
          //
          //
          if (!buttonPressed){
          if (hMenueON && !nMenueON) {//Verstellt das Hauptmenü
            if (hMenue < hMenueMax) hMenue++;
            else hMenue = 1;
            buttonPressed = true;
          }
          }
          //////////////////////////////////////////////////////
          ///////////////////////////////////////////////////////////////////////////////////////////verstellt Variablen im Modus Manuell
          //
          //
          //
          if (!hMenueON && nMenueON && hMenue == 1) {  //Verstellt die Temperatur für Manuel-
            if (manuellMenue == 1){
            if (manuellTemp > tempMin) {
              if (millis() - zaehlerStart >= zaehlerTakt) {
                if (!buttonPressed) zaehlerStart = millis();
                manuellTemp--;
              }
            }
            else manuellTemp = tempKochenMax;
          }
            if (manuellMenue == 2){ //Verstellt den Timer für Manuell-
            if (timerManuell > timerMin) {
              if (millis() - zaehlerStart >= zaehlerTakt){
                  if (!buttonPressed) zaehlerStart = millis();
                  timerManuell--;
                  //datenUebergeben = false;
                  //timerInt = false;
              }
            }
            else {
              timerManuell = timerMax;
              //datenUebergeben = false;
              //timerInt = false;
            }

           }
          }
          ////////////////////////////////////////////////
          ///////////////////////////////////////////////////////////////////////////////////////verstellt Variablen im Modus Thermostat
          //
          //
          //
          if (!hMenueON && nMenueON && hMenue == 4) {
            if (thermostatMenue == 1) { //Verstellt die Temperatur für thermostat+
              if (manuellTemp > 0) {
              if (millis() - zaehlerStart >= zaehlerTakt) {
                if (!buttonPressed) zaehlerStart = millis();
                manuellTemp--;
                }
              }
              else manuellTemp = tempKochenMax;
            }
            if (thermostatMenue == 2) { //Verstellt die Kühlung
              if (!buttonPressed) {
              if (kuehlungJa) kuehlungJa = false;
              else kuehlungJa = true;
              buttonPressed = true;
              }
            }
            if (thermostatMenue == 3) { //Verstellt die heizung
              if (!buttonPressed) {
              if (heizungJa) heizungJa = false;
              else heizungJa = true;
              buttonPressed = true;
              }
            }
            if (thermostatMenue == 4) { //Verstellt die hysterese für thermostat+
              if (hysterese > 0) {
                if (millis() - zaehlerStart >= zaehlerTakt) {
                  if (!buttonPressed) zaehlerStart = millis();
                  hysterese--;//
                }
              }
              else hysterese = 5;
            }
          }
          ////////////////////////////////////////////////////
          //////////////////////////////////////////////////////////////////////////////////////////verstellt Variablen im Modus Automaischen
          //
          //
          //
          if (!hMenueON && nMenueON && hMenue == 2) {  //definition für AutoMaischen modus
            if (autoMaischenStart) {
              if (!buttonPressed) {
              if (autoMaischenStatus < 2) autoMaischenStatus++;
              else autoMaischenStatus = 0;
              buttonPressed = true;
              }
            }
            if (autoMaischenStartDruecken) {
              if (!buttonPressed) {
              autoMaischenStartDruecken = false;
              buttonPressed = true;
              }
            }
              else if (!autoMaischenStart) {
                switch(autoMaischenMenue) {
                case 1: {  // von tempEinmaischen
                  if (tempEinmaischen > tempMin) {
                    if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempEinmaischen--;
                    }
                  }
                  else tempEinmaischen = tempMax;
                  break;
                }
                  case 2: {  // von tempStep////////////////////////////////////Step1
                    if (tempStep1 > tempMin){
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep1--;
                      }
                    }
                    else tempStep1 = tempMax;
                    break;
                  }
                  case 3: { // von timerstep
                    if (timerStep1 > timerMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep1--;
                      }
                    }
                    else timerStep1 = timerMax;
                    break;
                  }
                  case 4: {  // von tempStep////////////////////////////////////Step2
                    if (tempStep2 > tempMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep2--;
                      }
                    }
                    else tempStep2 = tempMax;
                    break;
                  }
                  case 5: { // von timerstep
                    if (timerStep2 > timerMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep2--;
                      }
                    }
                    else timerStep2 = timerMax;
                    break;
                  }
                  case 6: { //wechselt auf start
                    if (!buttonPressed) {
                      autoMaischenStartDruecken = true;
                      buttonPressed = true;
                      }
                    break;
                  }
                  case 7: {  // von tempStep////////////////////////////////////Step3
                    if (tempStep3 > tempMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep3--;
                      }
                    }
                    else tempStep3 = tempMax;
                    break;
                  }
                  case 8: { // von timerstep
                    if (timerStep3 > timerMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep3--;
                      }
                    }
                    else timerStep3 = timerMax;
                    break;
                  }
                  case 9: { //wechselt auf start
                    if (!buttonPressed) {
                  autoMaischenStartDruecken = true;
                  buttonPressed = true;
                  }
                    break;
                  }
                  case 10: {  // von tempStep////////////////////////////////////Step4
                    if (tempStep4 > tempMin){
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep4--;
                      }
                    }
                    else tempStep4 = tempMax;
                    break;
                  }
                  case 11: { //  von timerstep
                    if (timerStep4 > timerMin){
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep4--;
                      }
                    }
                    else timerStep4 = timerMax;
                    break;
                  }
                  case 12: { //wechselt auf start
                   if (!buttonPressed) {
                  autoMaischenStartDruecken = true;
                  buttonPressed = true;
                  }
                    break;
                  }
                  case 13: {  //von tempStep////////////////////////////////////Step5
                    if (tempStep5 > tempMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep5--;
                      }
                    }
                    else tempStep5 = tempMax;
                    break;
                  }
                  case 14: { // von timerstep
                    if (timerStep5 > timerMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep5--;
                      }
                    }
                    else timerStep5 = timerMax;
                    break;
                  }
                  case 15: { //wechselt auf start
                    if (!buttonPressed) {
                  autoMaischenStartDruecken = true;
                  buttonPressed = true;
                  }
                    break;
                  }
                  case 16: {  //von tempStep////////////////////////////////////Step6
                    if (tempStep6 > tempMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempStep6--;
                      }
                    }
                    else tempStep6 = tempMax;
                    break;
                  }
                  case 17: { //  von timerstep
                    if (timerStep6 > timerMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerStep6--;
                      }
                    }
                    else timerStep6 = timerMax;
                    break;
                  }//in case 19 gibt es nichts zu verstellen
                }
              }
            
          }
          ////////////////////////////////////////////////////
          //////////////////////////////////////////////////////////////////////////////////////////verstellt Variablen im Modus AutoKochen
          //
          //
          //
          if (!hMenueON && nMenueON && hMenue == 3) {  //definition für AutoMaischen modus
            if (autoKochenStart) {
              if (!buttonPressed) {
              if (autoKochenStatus < 2) autoKochenStatus++;
              else autoKochenStatus = 0;
              buttonPressed = true;
              }
            }
            if (autoKochenStartDruecken) {
              if (!buttonPressed) {
              autoKochenStartDruecken = false;
              buttonPressed = true;
              }
            }
              else if (!autoKochenStart) {
                switch(autoKochenMenue) {
                  case 1: {  //hoch von tempKochen
                    if (tempKochen > tempKochenMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      tempKochen--;
                      }
                    }
                    else tempKochen = tempKochenMax;
                    break;
                  }
                  case 2: { // hoch von timerstep
                    if (timerKochen > timerKochenMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerKochen--;
                      }
                    }
                    else timerKochen = timerMax;
                    break;
                  }
                  case 3: { // hoch von timer
                    if (timerHopfen1 > timerMin){
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerHopfen1--;
                      }
                    }
                    else timerHopfen1 = timerMax;
                    break;
                  }
                  case 4: { //wechselt auf start
                    if (!buttonPressed) {
                  autoKochenStartDruecken = true;
                  buttonPressed = true;
                  }
                    break;
                  }
                  case 5: { // hoch von timerstep
                    if (timerHopfen2 > timerMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerHopfen2--;
                      }
                    }
                    else timerHopfen2 = timerMax;
                    break;
                  }
                  case 6: { //wechselt auf start
                    if (!buttonPressed) {
                  autoKochenStartDruecken = true;
                  buttonPressed = true;
                  }
                    break;
                  }
                  case 7: { // hoch von timerstep
                    if (timerHopfen3 > timerMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerHopfen3--;
                      }
                    }
                    else timerHopfen3 = timerMax;
                    break;
                  }
                  case 8: { //wechselt auf start
                    if (!buttonPressed) {
                  autoKochenStartDruecken = true;
                  buttonPressed = true;
                  }
                    break;
                  }
                  case 9: { // hoch von timerstep
                    if (timerHopfen4 > timerMin) {
                      if (millis() - zaehlerStart >= zaehlerTakt) {
                      if (!buttonPressed) zaehlerStart = millis();
                      timerHopfen4--;
                      }
                    }
                    else timerHopfen4 = timerMax;
                    break;
                  } //in case 10 gibt es nichts zu verstellen
                }
          }
        }
      }
      break;
    }
    delay(10);
    case NONE: {
      buttonPressed = false;
      break;
    }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////Temperaturmessungen///////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float tempT(){ //Temperatur am Topfboden
  if (millis() - sensorPreviousTime >= messIntervall) {
    sensors.requestTemperatures();
    float temperatureRead = sensors.getTempCByIndex(0);
    if (-30 < temperatureRead && temperatureRead < 110) temperature = temperatureRead;
    temperature *= 10.0;
    temperature = round(temperature);
    temperature /= 10.0;
    temperature += 1.0;
    sensorPreviousTime = millis();
  }
  return temperature;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////Heizungsregelung//////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool heizung() {
  if (autoMaischenStart || autoKochenStart || manuellStart) {
    int heizungSV = 10000;
      if (millis() - heizungVorherigeZeit >= heizungMesszyklus) {  //berechnet Gradienten in K/min
              gradient = ((tempT() - heizungVorherigeTemp) / (heizungMesszyklus / 60000.0)); //gradient in K/min
              if (gradient < 0.1) gradient = 0.1;
              heizungVorherigeZeit = millis();
              heizungVorherigeTemp = tempT();
      }
      if ((tempT() > (tempSoll - 0.5)) || (tempT() >= 98.0)) {
        tempSensorDaempfung++;
        if (tempSensorDaempfung > 20) tempSollErreicht = true;
      }
      if (tempSoll >= 100) heizungStatus = true; //Heizung ist dauerhaft ein wenn gekocht wird
      else if (millis() - heizungSTimer >= heizungSV){
        if (tempT() < tempSoll  - (gradient * k)){  //schaltet heizung ein
         heizungSTimer = millis();
         heizungStatus = true;
        }
        else {
          heizungSTimer = millis();
          heizungStatus = false;
        }
      }
  }
  else if (thermostatStart && heizungJa) {
    int heizungSV = 5 * 60000;
    tempSoll = manuellTemp;
    if (millis() - heizungSTimer >= heizungSV){
        if (tempT() < tempSoll  - hysterese && !heizungStatus){  //schaltet heizung ein
         heizungSTimer = millis();
         heizungStatus = true;
        }
        else if (tempT() >= tempSoll  + hysterese && heizungStatus) {
          heizungSTimer = millis();
          heizungStatus = false;
        }
      }
  }
  else heizungStatus = false;
  return heizungStatus;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////Kühlungsregelung//////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool kuehlung() {
  if (thermostatStart && kuehlungJa) {
    tempSoll = manuellTemp;
      if (millis() - kuehlungESVTimer >= kuehlungESV){  //schaltet kühlung ein
          if (tempT() > (tempSoll + hysterese) && !kuehlungStatus) {
            kuehlungESDTimer = millis();
            kuehlungStatus = true; 
          }
      }
      if (tempT() < tempSoll && millis() - kuehlungESDTimer >= kuehlungESD && kuehlungStatus) {  //schaltet die kühlung aus
            kuehlungESVTimer = millis();
            kuehlungStatus = false;
      }
  }
  else kuehlungStatus = false;
  return kuehlungStatus;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////Buzzer Takt///////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void alarmKILL(){
  if (alarmOFF) {
    alarmOFF = false;
    alarmON = false;
  }
}
bool buzzer() {
  alarmKILL();
  if (millis() - buzzerPreviousMillis >= buzzerIntervall) {
    buzzerPreviousMillis = millis();
    if (buzzerON) buzzerON = false;
      else buzzerON = true;
  }
  return buzzerON;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////Timer/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void timer(){
  timerMinuten = timerSet/60;
  if (timerStart1 && tempSollErreicht && !timerEnde) { //Verstellt den Timer wenn Automatik 
    DateTime now = RTC.now(); //holt zeitwert von der rtc
    int RTCsekunden = now.secondstime();
    if (RTCsekunden < 0) RTCsekunden *= -1;
    if (!timerInt) {
      timerGo = RTCsekunden;
      timer0 = timerSet;
      timerInt = true;
    }
    int timerDelta = RTCsekunden - timerGo;
    timerSet = timer0 - timerDelta;
    if (timerSet < 0) timerSet = 0;
    timerMinuten = timerSet/60;  //Minutenanzeige für den Status des Modus
    timerSekunden = timerSet - timerMinuten*60;   //Sekundenanzeige für den Status des Modus
  }
  if (timerSet <= 0){
    timerEnde = true;
    timerSet = timerMin;
  }
  if (timerEnde){  //alles wird auf anfang gesetzt
    timerInt = false;
    tempSollErreicht = false;
    tempSensorDaempfung = 0;
    timerStart1 = false;
    timerSekunden = 0;  //für die anzeige
    timerManuell = timerMin;
    timerSet = timerMin;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////Manueller Modus/////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void manuell() {
  if (manuellStart && !autoMaischenStart && !autoKochenStart) {
      if (!datenUebergeben) {
      timerSet = timerManuell*60;
      datenUebergeben = true;
      }
      heizungOFF = false;
      timerStart1 = true;
      tempSoll = manuellTemp;
      if (timerEnde){
        alarmON = true;
        meldungsNummer = 1;
        timerEnde = false;
        manuellEnde = true;
      }
      if (manuellEnde) {  //schaltet alles wieder auf Anfang
        manuellStart = false;
        manuellEnde = false;
        meldung = false;
        datenUebergeben = false;
        tempSollErreicht = false;
        tempSensorDaempfung = 0;
        timerInt = false;
        tempSollErreicht = false;
        tempSensorDaempfung = 0;
        timerStart1 = false;
        timerSekunden = 0;  //für die anzeige
        timerManuell = timerMin;
        timerSet = timerMin;
      }
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////AutoMaischen Modus////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void autoMaischen() {
  if (maximaMaischen) { //Bestimmt Rahmenwerte für die Automatik
    if (autoMaischenMenue == 6) letzterStep = 2;  //bestimmt letzten Step anhand der menüposition
    else if (autoMaischenMenue == 9) letzterStep = 3;
    else if (autoMaischenMenue == 12) letzterStep = 4;
    else if (autoMaischenMenue == 15) letzterStep = 5;
    else if (autoMaischenMenue == 18) letzterStep = 6;
    tempStepMax = tempStep1;
    timerStepMax = timerStep1;
    if (letzterStep == 2) { //Bestimmt den letzten Schritt der Automatik
      tempStepMax = tempStep2;  //für die zusammefassung vor dem automatikstart
      timerStepMax = timerStep1 + timerStep2;  //same
    }
    if (letzterStep == 3) {
      tempStepMax = tempStep3;
      timerStepMax = timerStep1 + timerStep2 + timerStep3;
    }
    if (letzterStep == 4) {
      tempStepMax = tempStep4;
      timerStepMax = timerStep1 + timerStep2 + timerStep3 + timerStep4;
    }
    if (letzterStep == 5) {
      tempStepMax = tempStep5;
      timerStepMax = timerStep1 + timerStep2 + timerStep3 + timerStep4 + timerStep5;
    }
    if (letzterStep == 6) {
      tempStepMax = tempStep6;
      timerStepMax = timerStep1 + timerStep2 + timerStep3 + timerStep4 + timerStep5 + timerStep6;
    }    
    maximaMaischen = false;  //bestimmt die Werte nur einmal
  }
  if (autoMaischenStart && !manuellStart && !autoKochenStart) {  //hier startet es erst richtig
      //////////////////////////////////////////////////Timen des Temperaturprofils
       if (!einmaischenPause) heizungOFF = false;  //schaltet zum start alles an
        if (!einmaischenFertig) {
          if (!datenUebergeben) {
            tempSoll = tempEinmaischen;
            timerSet = timerStep1*60;
            datenUebergeben = true;
          }
          if (tempSollErreicht && !einmaischenPause) {
                heizungOFF = true;
                alarmON = true;  //alarm zum einmaischen
                meldungsNummer = 2;
                einmaischenJetzt = true;  //pausiert die ganze automatik bis eingemaischt wurde, aka es bestätigt wurde
                einmaischenPause = true;
           }
        }
       if (einmaischenFertig) {
        heizungOFF = false;
        timerStart1 = true;
        switch(currentStep) {
          case 1: {  //automaisch schritt 1
            if (!datenUebergeben) {
              tempSoll = tempStep1;
              datenUebergeben = true;
            }
            break;
          }
          case 2: { //automaisch schritt 2
            if (!datenUebergeben) {  
              timerSet = timerStep2*60;
              tempSoll = tempStep2;
              datenUebergeben = true;
            }
            break;
          }
          case 3: {//automaisch schritt 3
            if (!datenUebergeben) {  
              timerSet = timerStep3*60;
              tempSoll = tempStep3;
              datenUebergeben = true;
            }
            break;
          }
          case 4: {//automaisch schritt 4
            if (!datenUebergeben) {  
              timerSet = timerStep4*60;
              tempSoll = tempStep4;
              datenUebergeben = true;
            }
            break;
          }
          case 5: {//automaisch schritt 5
            if (!datenUebergeben) {  
              timerSet = timerStep5*60;
              tempSoll = tempStep5;
              datenUebergeben = true;
            }
            break;
          }
          case 6: {//automaisch schritt 6
            if (!datenUebergeben) {  
              timerSet = timerStep6*60;
              tempSoll = tempStep6;
              datenUebergeben = true;
            }
            break;
          }
        }
        if (timerEnde) {  ///////////weiter zu nächsten Step
            timerEnde = false;
            if (currentStep < letzterStep) {
              currentStep++;
              datenUebergeben = false;
            }
            else autoMaischenEnde = true;
          }
        }
      }
    if (autoMaischenEnde){  //Schaltet alles auf Anfang wenn die Automatik vorbei ist, beendet sie und aktiviert den läuteralarm
      autoMaischenStatus = 1; //für Statusanzeige auf display
      autoMaischenStartDruecken = false;
      autoMaischenStart = false;
      autoMaischenEnde = false;
      if (!meldung && !noAlarm) {
        jetztLaeutern = true;  //alarm zum läutern
        alarmON = true;
        meldungsNummer = 3;
      }
      noAlarm = false;
      einmaischenJetzt = false; //wieder auf anfang
      einmaischenFertig = false;
      currentStep = 1;
      letzterStep = 1;
      tempStepMax = 0;
      timerStepMax = 0;
      maximaMaischen = false;
      heizungOFF = true;  //schaltet heizung usw. aus
      meldung = false;
      datenUebergeben = false;
      timerInt = false;
      tempSollErreicht = false;
      tempSensorDaempfung = 0;
      timerStart1 = false;
      timerSekunden = 0;  //für die anzeige
      timerManuell = timerMin;
      timerSet = timerMin;
    }
  
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////Hopfen Timer/////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void hopfentimer(){
  DateTime now = RTC.now(); //holt zeitwert von der rtc
  int RTCsekunden = now.secondstime();
  if (RTCsekunden < 0) RTCsekunden *= -1;
  timerHopfenMinuten = timerHopfenSet/60;
  if (timerHopfenStart && tempSollErreicht && !timerHopfenEnde) { //Verstellt den Timer wenn Automatik 
    if (!timerHopfenInt) {
      timerHopfenGo = RTCsekunden;
      timerHopfen0 = timerHopfenSet;
      timerHopfenInt = true;
    }
    int timerDelta = RTCsekunden - timerHopfenGo;
    timerHopfenSet = timerHopfen0 - timerDelta;
    if (timerHopfenSet < 0) timerHopfenSet = 0;
    timerHopfenMinuten = timerHopfenSet/60;  //Minutenanzeige für den Status des Modus
    timerHopfenSekunden = timerHopfenSet - timerHopfenMinuten*60;   //Sekundenanzeige für den Status des Modus
    if (timerHopfenSet == 0){
      timerHopfenEnde = true;
    }
  }
  if (timerHopfenEnde){  //alles wird auf anfang gesetzt
    timerHopfenStart = false;
    timerHopfenSekunden = 0;  //für die anzeige
    timerHopfenSet = 60;
    timerHopfenInt = false;
  }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////AutoKochen Modus////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void autoKochen() {
  if (maximaKochen) {  //bestimmt Rahmenbedingugen
    if (autoKochenMenue == 4) letzterHopfen = 1;  //bestimmt letzten hopfen anhand der menüposition
    else if (autoKochenMenue == 6) letzterHopfen = 2;
    else if (autoKochenMenue == 8) letzterHopfen = 3;
    else if (autoKochenMenue == 10) letzterHopfen = 4;
    hopfenAlarm1 = timerKochen - timerHopfen1;  //berechnet hopfenalarme aka. die abstände zwischen alarmen
    hopfenAlarm2 = timerKochen - timerHopfen2 - hopfenAlarm1;
    hopfenAlarm3 = timerKochen - timerHopfen3 - hopfenAlarm1 - hopfenAlarm2;
    hopfenAlarm4 = timerKochen - timerHopfen4 - hopfenAlarm1 - hopfenAlarm2 - hopfenAlarm3;
    timerSet = timerKochen * 60; //übergibt den timer für die gesamtzeit genau einmal
    maximaKochen = false;
  }
  if (autoKochenStart && !manuellStart && !autoMaischenStart) {  //programmanfang
      timerStart1 = true;  
      timerHopfenStart = true;
      heizungOFF = false; //schaltet alles an
      tempSoll = tempKochen; //neue temp für heizung
      if (hopfen <= letzterHopfen && !keinHopfenMehr) {
        if (timerHopfenEnde) {  ///////////Hopfengabe Alarm
                if (hopfen == letzterHopfen) keinHopfenMehr = true;
                timerHopfenEnde = false;
                datenUebergeben = false;
                alarmON = true;
                meldungsNummer = 4;
                if (!keinHopfenMehr) hopfen++;
          }
        switch(hopfen) {
            case 1: {
              if (!datenUebergeben) {
                timerHopfenSet = hopfenAlarm1 * 60;
                datenUebergeben = true;
              }
              break;
            }
            case 2: {  
              if (!datenUebergeben) {
                timerHopfenSet = hopfenAlarm2*60;
                datenUebergeben = true;
              }
              break;
            }
            case 3: {  //Einmaischen (automaischen schritt 1
              if (!datenUebergeben) {
                timerHopfenSet = hopfenAlarm3*60;
                datenUebergeben = true;
              }
              break;
            }
            case 4: {  //Einmaischen (automaischen schritt 1
              if (!datenUebergeben) {
                timerHopfenSet = hopfenAlarm4*60;
                datenUebergeben = true;
              }
              break;
            }
          }
        }
        if (timerEnde) {  //schaltet die automatik aus wenn der timer abgelaufen ist
              autoKochenEnde = true;
            }
        if (autoKochenEnde){  //Schaltet alles auf Anfang wenn die Automatik vorbei ist
          autoKochenEnde = false;
          datenUebergeben = false;
          timerEnde = false;
          timerStart1 = false;
          if (!meldung && !noAlarm){
            alarmON = true;
            meldungsNummer = 1;
          }
          noAlarm = false;
          autoKochenStart = false;
          heizungOFF = true;
          hopfen = 1;
          letzterHopfen = 1;
          meldung = false;
          datenUebergeben = false;
          maximaKochen = false;
          meldung = false;
          keinHopfenMehr = false;
          timerHopfenStart = false;
          timerHopfenSekunden = 0;  //für die anzeige
          timerHopfenSet = 60;
          timerHopfenInt = false;
          timerInt = false;
          tempSollErreicht = false;
          tempSensorDaempfung = 0;
          timerStart1 = false;
          timerSekunden = 0;  //für die anzeige
          timerManuell = timerMin;
          timerSet = timerMin;
        }
      }
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////User Interface//////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void userInterface() {
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////Meldungen die bestätigt werden müssen
 if (meldung && meldungsNummer != 2 && (manuellStart || autoMaischenStart || autoKochenStart || thermostatStart)) {
        lcd.home();
        lcd.print("  ZUM BEENDEN   ");  //meldung beim Abbrechen von Programmen
        lcd.setCursor(0,1);
        lcd.print("  OK DRUECKEN   ");
 }
 else if (alarmON && meldungsNummer == 1) {  //wenn programm fertig ist
       lcd.home();
       lcd.print("    PROGRAMM    ");
       lcd.setCursor(0,1);
       lcd.print("     FERTIG     ");
 }
 else if (alarmON && meldungsNummer == 2) {  //Meldung zum Einmaischen
       lcd.home();
       lcd.print("      JETZT     ");
       lcd.setCursor(0,1);
       lcd.print("   EINMAISCHEN  ");
 }
 else if (meldung && meldungsNummer == 2) {  
        lcd.home();
        lcd.print("NACH EINMAISCHEN");  //meldung beim Abbrechen von Programmen
        lcd.setCursor(0,1);
        lcd.print("  OK DRUECKEN   ");
 }
 else if (alarmON && meldungsNummer == 3) {  //Meldung zum Einmaischen
       lcd.home();
       lcd.print("     JETZT      ");
       lcd.setCursor(0,1);
       lcd.print("   ABMAISCHEN   ");
 }
 else if (alarmON && meldungsNummer == 4) {  //Meldung zur Hopfengabe
       lcd.home();
       lcd.print("      JETZT     ");
       lcd.setCursor(0,1);
       lcd.print("  ");
       if (!keinHopfenMehr) lcd.print(hopfen - 1);
       else lcd.print(hopfen);
       lcd.print(".HOPFENGABE  ");
 }
 else if (meldung && meldungsNummer == 2) {  
        lcd.home();
        lcd.print("NACH EINMAISCHEN");  //meldung beim Abbrechen von Programmen
        lcd.setCursor(0,1);
        lcd.print("  OK DRUECKEN   ");
 }
 else if (meldung && meldungsNummer == 5) {  //meldung beim Start ob Thingspeak.com Daten gelöscht werden sollen
       lcd.home();
       lcd.print("ZUM LOG LOESCHEN");
       lcd.setCursor(0,1);
       lcd.print("  OK DRUECKEN   ");
 }
 else if (meldung && meldungsNummer == 6) {  //meldung beim Start ob Thingspeak.com Daten gelöscht werden sollen
       lcd.home();
       lcd.print("ZUM WiFi LOESCH.");
       lcd.setCursor(0,1);
       lcd.print("  OK DRUECKEN   ");
 }
  else if (meldung && meldungsNummer == 7) {  //meldung zum webupdate
       lcd.home();
       lcd.print(" WEBUPDATE ODER ");
       lcd.setCursor(0,1);
       lcd.print("     RESET      ");
 }
 if (meldung && automatikBestaetigung) {  //übersicht bevor die automatik startet
      lcd.home();
      if (hMenue == 2) {
        lcd.print("AutoM ");
        lcd.print(tempStepMax);lcd.write(byte(1));
        lcd.print("C Tmax.");
        lcd.setCursor(0,1);
        lcd.print("insg.");
        if (timerStepMax < 10) lcd.print("  ");
        else if (timerStepMax < 100) lcd.print(" ");
        lcd.print(timerStepMax);lcd.print("Min");
      }
      if (hMenue == 3){
        lcd.print("AutoK ");
        lcd.print(tempKochen);lcd.write(byte(1));
        lcd.print("C Tkoch");
        lcd.setCursor(0,1);
        lcd.print("insg.");
        if (timerKochen < 10) lcd.print("  ");
        else if (timerKochen < 100) lcd.print(" ");
        lcd.print(timerKochen);lcd.print("Min");
      }
      lcd.setCursor(12,1);
      lcd.write(byte(2));lcd.print("OK?");
 }
 if (!meldung && !alarmON){ //menüs sind deaktiviert wenn meldungen an sind
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  ///////////////////////////////////////////////////////////////////////////////////////////////
  if (hMenueON && !nMenueON){///////////////////////////////////////////////////////////////////////////////Hauptmenü
    switch(hMenue) {
    case 1: {
      lcd.home();
      lcd.write(byte(2)); lcd.print("Manuell       ");lcd.write(byte(4));
      lcd.setCursor(0,1);
      lcd.print(" AutoMaischen  ");lcd.write(byte(5));
      break;
    }
    case 2: {
      lcd.home();
      lcd.print(" Manuell       ");lcd.write(byte(4));
      lcd.setCursor(0,1);
      lcd.write(byte(2)); lcd.print("AutoMaischen  ");lcd.write(byte(5));
      break;
    }
    case 3: {
      lcd.home();
      lcd.write(byte(2)); lcd.print("AutoKochen    ");lcd.write(byte(4));
      lcd.setCursor(0,1);
      lcd.print(" Thermostat    ");lcd.write(byte(5));
      break;
    }
    case 4: {
      lcd.home();
      lcd.print(" AutoKochen    ");lcd.write(byte(4));
      lcd.setCursor(0,1);
      lcd.write(byte(2)); lcd.print("Thermostat    ");lcd.write(byte(5));
      break;
    }
    case 5: {
      lcd.home();
      lcd.write(byte(2)); lcd.print("Status        ");lcd.write(byte(4));
      lcd.setCursor(0,1);
      lcd.print("               ");lcd.write(byte(5));
      break;
    }
  }
 }
  if (!hMenueON && nMenueON) {////////////////////////////////////////////////////////////////////////Nebenmenüs
    ////////////////////////////////
    /////////////////////////////////
    //////////////////////////////////
    /////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////Manueller Modus
    if (hMenue == 1) { 
      switch(manuellMenue) {
        case 1:  { //Temperatureinstellung
          lcd.home();
          lcd.print("Manuell         ");
          lcd.setCursor(0,1);
          lcd.print("   "); lcd.write(byte(2));  
          if (manuellTemp < 10) lcd.print("Temp.   ");//Stellt passend zur Temperatur die Anzeige ein
          else if (manuellTemp < 100) lcd.print("Temp.   ");
          else if (manuellTemp >= 100) lcd.print("Temp.  ");
          lcd.print(manuellTemp);
          lcd.write(byte(1));lcd.print("C");
          break;      
        }
        case 2: {  //Timereinstellung
          lcd.home();
          lcd.print("Manuell         ");
          lcd.setCursor(0,1);
          lcd.print("   "); lcd.write(byte(2)); 
          if (timerManuell < 10) lcd.print("Dauer   ");// stellt passend zur Zeit die Anzeige ein
          else if (timerManuell < 100) lcd.print("Dauer  ");
          else if (timerManuell >= 100) lcd.print("Dauer ");
          lcd.print(timerManuell);
          lcd.print("Min");
          break;
        }
        case 3: {  //Bestätigen für Start
          lcd.home();
          lcd.print("Manuell         ");
          lcd.setCursor(0,1);
          lcd.print("          ");lcd.write(byte(2));lcd.print("START");
          break;
        }
        case 4: {  //Übersicht während des Programms
          lcd.home();
          lcd.print("MANUELL  ");
          if (timerMinuten < 10) lcd.print("  ");
          else if (timerMinuten < 100) lcd.print(" ");
          lcd.print(timerMinuten);lcd.print("m");
          if (timerSekunden < 10) lcd.print(" ");
          lcd.print(timerSekunden);lcd.print("s");
          lcd.setCursor(0,1);
          lcd.print(" ");
          if (tempT() < 10) lcd.print("  ");
          else if (tempT() < 100) lcd.print(" ");
          lcd.print(tempT(),1);
          lcd.write(byte(1));lcd.print("C");
          lcd.write(byte(6));
          if (tempSoll < 100) lcd.print(" ");
          lcd.print(tempSoll);lcd.print(".0");
          lcd.write(byte(1));lcd.print("C");
          break;
        }
      }
    }
    /////////////////////
    ///////////////////////
    //////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    if (hMenue == 2) { ////////////////////////////////////////////////////////////////////////////////AutoMaischen Modus
      if (autoMaischenStartDruecken) { //Startanzeige in den Einstellungen
            lcd.home();
            if (autoMaischenMenue == 19) lcd.print("                 ");
            else {
              lcd.print(" n");lcd.write(byte(3));lcd.print("chste Rast   ");
            }
            lcd.setCursor(0,1);
            lcd.write(byte(2));
            lcd.print("START          ");
      }
      if (autoMaischenStart) {  //////////////////////////////////////////Die StatusDisplays während des Automaischens
        if (autoMaischenStatus == 1) {  //allgemeine Übersicht, istTemp, sollTemp, verbleibende Zeit
          lcd.home();
          lcd.print("NOW: ");
          lcd.print(currentStep);
          if (timerMinuten < 10) lcd.print(")   ");
          else if (timerMinuten < 100) lcd.print(")  ");
          else lcd.print(") ");
          lcd.print(timerMinuten); lcd.print("m");
          if (timerSekunden < 10) lcd.print(" ");
          lcd.print(timerSekunden);
          lcd.print("s");
          lcd.write(byte(4));
          lcd.setCursor(0,1);
          lcd.print("  ");
          if (tempT() < 10) lcd.print(" ");
          lcd.print(tempT(),1);lcd.write(byte(1));lcd.print("C");
          lcd.write(byte(6));
          lcd.print(tempSoll);lcd.print(".0");lcd.write(byte(1));lcd.print("C");
          lcd.write(byte(5));
        }
        else if (autoMaischenStatus  == 2) { // zeigt den nächsten step an, solltemp, dauer
              lcd.home();
              lcd.print("NXT: ");
              if (currentStep < letzterStep){
                lcd.print(currentStep+1);
                lcd.print(") ");
              switch(currentStep) {
                case 1:{
                  if (timerStep2 < 10) lcd.print("  ");
                  else if (timerStep2 < 100) lcd.print(" ");
                  lcd.print(timerStep2);
                  lcd.print("m 0s");
                  lcd.write(byte(4));
                  lcd.setCursor(0,1);
                  if (!einmaischenFertig) lcd.print("einmai. ");  //anzeige wenn als nächstes eingemaischt werden muss
                  else lcd.print("        ");
                  lcd.write(byte(6));lcd.print(tempStep2);lcd.print(".0");lcd.write(byte(1));lcd.print("C");
                  lcd.write(byte(5));
                  break;
                }
                case 2:{
                  if (timerStep3 < 10) lcd.print("  ");
                  else if (timerStep3 < 100) lcd.print(" ");
                  lcd.print(timerStep3);
                  lcd.print("m 0s");
                  lcd.write(byte(4));
                  lcd.setCursor(0,1);
                  lcd.print("        "); 
                  lcd.write(byte(6));lcd.print(tempStep3);lcd.print(".0");lcd.write(byte(1));lcd.print("C");
                  lcd.write(byte(5));
                  break;
                }
                case 3:{
                  if (timerStep4 < 10) lcd.print("  ");
                  else if (timerStep4 < 100) lcd.print(" ");
                  lcd.print(timerStep4);
                  lcd.print("m 0s");
                  lcd.write(byte(4));
                  lcd.setCursor(0,1);
                  lcd.print("        ");
                  lcd.write(byte(6));lcd.print(tempStep4);lcd.print(".0");lcd.write(byte(1));lcd.print("C");
                  lcd.write(byte(5));
                  break;
                }
                case 4:{
                  if (timerStep5 < 10) lcd.print("  ");
                  else if (timerStep5 < 100) lcd.print(" ");
                  lcd.print(timerStep5);
                  lcd.print("m 0s");
                  lcd.write(byte(4));
                  lcd.setCursor(0,1);
                  lcd.print("        "); 
                  lcd.write(byte(6));lcd.print(tempStep5);lcd.print(".0");lcd.write(byte(1));lcd.print("C");
                  lcd.write(byte(5));
                  break;
                }
                case 5:{
                  if (timerStep6 < 10) lcd.print("  ");
                  else if (timerStep6 < 100) lcd.print(" ");
                  lcd.print(timerStep6);
                  lcd.print("m 0s");
                  lcd.write(byte(4));
                  lcd.setCursor(0,1);
                  lcd.print("        ");
                  lcd.write(byte(6));lcd.print(tempStep6);lcd.print(".0");lcd.write(byte(1));lcd.print("C");
                  lcd.write(byte(5));
                  break;
                }
              }
              
            }
            else { //wenn als nächstes geläutert werden muss
                lcd.print("          ");
                lcd.write(byte(4));
                lcd.setCursor(0,1);
                lcd.print("abmai.         ");
                lcd.write(byte(5));
              }
          }
        }
      else if (!autoMaischenStartDruecken) {/////////////////////////////////////die Einstellungen für das Automaischen
        switch (autoMaischenMenue) {
        case 1: {  //EinmaischTemperatur einstellen
          lcd.home();
          lcd.print("Einmaischen bei ");
          lcd.setCursor(0,1);
          lcd.print("           ");
          lcd.write(byte(2));
          lcd.print(tempEinmaischen);
          lcd.write(byte(1));lcd.print("C");
          break;
        }
          case 2:  { //Temperatureinstellung/////////////////////////////////////////STEP1
            lcd.home();
            lcd.print("1) ");lcd.write(byte(2));
            lcd.print("Temp   ");
            lcd.print(tempStep1);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            if (timerStep1 < 10) lcd.print("    Dauer   ");
            else if (timerStep1 < 100) lcd.print("    Dauer  ");
            else lcd.print("    Dauer ");
            lcd.print(timerStep1);lcd.print("Min");
            break;
          }
          case 3:  { //Zeiteinstellung
            lcd.home();
            lcd.print("1)  Temp   ");
            lcd.print(tempStep1);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            lcd.print("   ");lcd.write(byte(2));
            if (timerStep1 < 10) lcd.print("Dauer   ");
            else if (timerStep1 < 100) lcd.print("Dauer  ");
            else lcd.print("Dauer ");
            lcd.print(timerStep1);lcd.print("Min");
            break;
          }
          case 4:  { //Temperatureinstellung////////////////////////////////////STEP2
            lcd.home();
            lcd.print("2) ");lcd.write(byte(2));lcd.print("Temp   ");
            lcd.print(tempStep2);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            if (timerStep2 < 10) lcd.print("    Dauer   ");
            else if (timerStep2 < 100) lcd.print("    Dauer  ");
            else lcd.print("    Dauer ");
            lcd.print(timerStep2);lcd.print("Min");
            break;
          }
          case 5:  { //Zeiteinstellung
            lcd.home();
            lcd.print("2)  Temp   ");
            lcd.print(tempStep2);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            lcd.print("   ");lcd.write(byte(2));
            if (timerStep2 < 10) lcd.print("Dauer   ");
            else if (timerStep2 < 100) lcd.print("Dauer  ");
            else lcd.print("Dauer ");
            lcd.print(timerStep2);lcd.print("Min");
            break;
          }
          case 6:  { //nächster schritt
            lcd.home();
            lcd.write(byte(2));
            lcd.print("n");lcd.write(byte(3));lcd.print("chste Rast   ");
            lcd.setCursor(0,1);
            lcd.print(" START          ");
            break;
          }
          case 7:  { //Temperatureinstellung////////////////////////////STEP3
            lcd.home();
            lcd.print("3) ");lcd.write(byte(2));
            lcd.print("Temp   ");
            lcd.print(tempStep3);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            if (timerStep3 < 10) lcd.print("    Dauer   ");
            else if (timerStep3 < 100) lcd.print("    Dauer  ");
            else lcd.print("    Dauer ");
            lcd.print(timerStep3);lcd.print("Min");
            break;
          }
          case 8:  { //Zeiteinstellung
            lcd.home();
            lcd.print("3)  Temp   ");
            lcd.print(tempStep3);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            lcd.print("   ");lcd.write(byte(2));
            if (timerStep3 < 10) lcd.print("Dauer   ");
            else if (timerStep3 < 100) lcd.print("Dauer  ");
            else lcd.print("Dauer ");
            lcd.print(timerStep3);lcd.print("Min");
            break;
          }
          case 9:  { //nächster schritt
            lcd.home();
            lcd.write(byte(2));
            lcd.print("n");lcd.write(byte(3));lcd.print("chste Rast   ");
            lcd.setCursor(0,1);
            lcd.print(" START          ");
            break;
          }
          case 10:  { //Temperatureinstellung////////////////////////////STEP4
            lcd.home();
            lcd.print("4) ");lcd.write(byte(2));
            lcd.print("Temp   ");
            lcd.print(tempStep4);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            if (timerStep4 < 10) lcd.print("    Dauer   ");
            else if (timerStep4 < 100) lcd.print("    Dauer  ");
            else lcd.print("    Dauer ");
            lcd.print(timerStep4);lcd.print("Min");
            break;
          }
          case 11:  { //Zeiteinstellung
            lcd.home();
            lcd.print("4)  Temp   ");
            lcd.print(tempStep4);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            lcd.print("   ");lcd.write(byte(2));
            if (timerStep4 < 10) lcd.print("Dauer   ");
            else if (timerStep4 < 100) lcd.print("Dauer  ");
            else lcd.print("Dauer ");
            lcd.print(timerStep4);lcd.print("Min");
            break;
          }
          case 12:  { //nächster schritt
            lcd.home();
            lcd.write(byte(2));
            lcd.print("n");lcd.write(byte(3));lcd.print("chste Rast   ");
            lcd.setCursor(0,1);
            lcd.print(" START          ");
            break;
          }
          case 13:  { //Temperatureinstellung////////////////////////////STEP5
            lcd.home();
            lcd.print("5) ");lcd.write(byte(2));
            lcd.print("Temp   ");
            lcd.print(tempStep5);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            if (timerStep5 < 10) lcd.print("    Dauer   ");
            else if (timerStep5 < 100) lcd.print("    Dauer  ");
            else lcd.print("    Dauer ");
            lcd.print(timerStep5);lcd.print("Min");
            break;
          }
          case 14:  { //Zeiteinstellung
            lcd.home();
            lcd.print("5)  Temp   ");
            lcd.print(tempStep5);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            lcd.print("   ");lcd.write(byte(2));
            if (timerStep5 < 10) lcd.print("Dauer   ");
            else if (timerStep5 < 100) lcd.print("Dauer  ");
            else lcd.print("Dauer ");
            lcd.print(timerStep5);lcd.print("Min");
            break;
          }
          case 15:  { //nächster schritt
            lcd.home();
            lcd.write(byte(2));
            lcd.print("n");lcd.write(byte(3));lcd.print("chste Rast   ");
            lcd.setCursor(0,1);
            lcd.print(" START          ");
            break;
          }
          case 16:  { //Temperatureinstellung////////////////////////////STEP6
            lcd.home();
            lcd.print("6) ");lcd.write(byte(2));lcd.print("Temp   ");
            lcd.print(tempStep6);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            if (timerStep6 < 10) lcd.print("    Dauer   ");
            else if (timerStep6 < 100) lcd.print("    Dauer  ");
            else lcd.print("    Dauer ");
            lcd.print(timerStep6);lcd.print("Min");
            break;
          }
          case 17:  { //Zeiteinstellung
            lcd.home();
            lcd.print("6)  Temp   ");
            lcd.print(tempStep6);
            lcd.print(" ");lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            lcd.print("   ");lcd.write(byte(2));
            if (timerStep6 < 10) lcd.print("Dauer   ");
            else if (timerStep6 < 100) lcd.print("Dauer  ");
            else lcd.print("Dauer ");
            lcd.print(timerStep6);lcd.print("Min");
            break;
          }
          case 18:  { //Start
            lcd.home();
            lcd.print("                ");
            lcd.setCursor(0,1);
            lcd.write(byte(2));
            lcd.print("START          ");
            break;
          }
        }
      }
    }
    /////////////////////
    ///////////////////////
    //////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    if (hMenue == 3) { //////////////////////////////////////////////////////////////////////////////////////AutoKochen Modus
      if (autoKochenStartDruecken) { //Startanzeige in den Einstellungen
            lcd.home();
            if (autoKochenMenue == 11) lcd.print("                 ");
            else {
              lcd.print(" n");lcd.write(byte(3));lcd.print("chster Hopfen");
            }
            lcd.setCursor(0,1);
            lcd.write(byte(2));
            lcd.print("START           ");
      }
      if (autoKochenStart) {  //////////////////////////////////////////Die StatusDisplays während des Autokochens
        if (autoKochenStatus == 1) {  //allgemeine Übersicht, istTemp, sollTemp, verbleibende Zeit
          lcd.home();
          if (hopfen <= letzterHopfen && !keinHopfenMehr){
            lcd.print("NXT: ");
            lcd.print(hopfen);
            lcd.print(") ");
            if (timerHopfenMinuten < 10) lcd.print("  ");
            else if (timerHopfenMinuten < 100) lcd.print(" ");
            lcd.print(timerHopfenMinuten); lcd.print("m");
            if (timerHopfenSekunden < 10) lcd.print(" ");
            lcd.print(timerHopfenSekunden);
            lcd.print("s");
          }
          else {
            lcd.print("Ende in ");
            if (timerMinuten < 10) lcd.print("  ");
            else if (timerMinuten < 100) lcd.print(" ");
            lcd.print(timerMinuten); lcd.print("m");
            if (timerSekunden < 10) lcd.print(" ");
            lcd.print(timerSekunden);
            lcd.print("s");
          }
          lcd.write(byte(4));
          lcd.setCursor(0,1);
          if (tempT() < 100) lcd.print(" ");
          lcd.print(tempT(),1);lcd.write(byte(1));lcd.print("C");
          lcd.write(byte(6));
          if (tempSoll < 100) lcd.print(" ");
          lcd.print(tempSoll);lcd.print(".0");lcd.write(byte(1));lcd.print("C");
          lcd.write(byte(5));
        }
        else if (autoKochenStatus  == 2) { // zeigt den nächsten step an, solltemp, dauer
              lcd.home();
              lcd.print("NXT: ");
              if (hopfen < letzterHopfen){
                lcd.print(hopfen + 1);
                lcd.print(")  Hopfen");
                lcd.write(byte(4));
                lcd.setCursor(0,1);
                lcd.print("     in ");
              switch(hopfen) {
                case 1:{
                  if (timerHopfenMinuten + hopfenAlarm2 < 10) lcd.print("  ");
                  else if (timerHopfenMinuten + hopfenAlarm2 < 100) lcd.print(" ");
                  lcd.print(timerHopfenMinuten + hopfenAlarm2);
                  lcd.print("m");
                  if (timerHopfenSekunden < 10) lcd.print(" ");
                  lcd.print(timerHopfenSekunden);
                  lcd.print("s");
                  lcd.write(byte(5));
                  break;
                }
                case 2:{
                  if (timerHopfenMinuten + hopfenAlarm3 < 10) lcd.print("  ");
                  else if (timerHopfenMinuten + hopfenAlarm3 < 100) lcd.print(" ");
                  lcd.print(timerHopfenMinuten + hopfenAlarm3);
                  lcd.print("m");
                  if (timerHopfenSekunden < 10) lcd.print(" ");
                  lcd.print(timerHopfenSekunden);
                  lcd.print("s");
                  lcd.write(byte(5));
                  break;
                }
                case 3:{
                  if (timerHopfenMinuten + hopfenAlarm3 < 10) lcd.print("  ");
                  else if (timerHopfenMinuten + hopfenAlarm3 < 100) lcd.print(" ");
                  lcd.print(timerHopfenMinuten + hopfenAlarm3);
                  lcd.print("m");
                  if (timerHopfenSekunden < 10) lcd.print(" ");
                  lcd.print(timerHopfenSekunden);
                  lcd.print("s");
                  lcd.write(byte(5));
                  break;
                }
                case 4:{
                  if (timerHopfenMinuten + hopfenAlarm4 < 10) lcd.print("  ");
                  else if (timerHopfenMinuten + hopfenAlarm4 < 100) lcd.print(" ");
                  lcd.print(timerHopfenMinuten + hopfenAlarm4);
                  lcd.print("m");
                  if (timerHopfenSekunden < 10) lcd.print(" ");
                  lcd.print(timerHopfenSekunden);
                  lcd.print("s");
                  lcd.write(byte(5));
                  break;
                }
              }
            }
            else if (hopfen == letzterHopfen) { //wenn das der letzte hopfen ist
                lcd.print("-)        ");
                lcd.write(byte(4));
                lcd.setCursor(0,1);
                lcd.print("               ");
                lcd.write(byte(5));
              }
          }
          else if (autoKochenStatus == 3) {  //zeigt verbleibende Kochdauer an
            lcd.home();
            lcd.print("Kochen fertig  ");lcd.write(byte(4));
            lcd.setCursor(0,1);
            lcd.print("     in ");
            if (timerMinuten < 10) lcd.print("  ");
            else if (timerMinuten < 100) lcd.print(" ");
            lcd.print(timerMinuten);lcd.print("m");
            if (timerSekunden < 10) lcd.print(" ");
            lcd.print(timerSekunden);
            lcd.print("s");
            lcd.write(byte(5));
          }
        }
      else if (!autoKochenStartDruecken) {//////////////////////////////////////////////////die Einstellungen für das Autokochen
        switch (autoKochenMenue) {
          case 1:  { //Temperatureinstellung/////////////////////////////////////////kochtemperatur
            lcd.home();
            lcd.write(byte(2));
            lcd.print("KochTemp  ");
            if (tempKochen < 100)lcd.print(" ");
            lcd.print(tempKochen);
            lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            lcd.print(" KochDauer");
            if (timerKochen < 10) lcd.print("  ");
            else if (timerKochen < 100) lcd.print(" ");
            lcd.print(timerKochen);lcd.print("Min");
            break;
          }
          case 2:  { //Zeiteinstellung
            lcd.home();
            lcd.print(" KochTemp  ");
            if (tempKochen < 100)lcd.print(" ");
            lcd.print(tempKochen);
            lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            lcd.write(byte(2));
            lcd.print("KochDauer");
            if (timerKochen < 10) lcd.print("  ");
            else if (timerKochen < 100) lcd.print(" ");
            lcd.print(timerKochen);lcd.print("Min");
            break;
          }
          case 3:  { //Zeiteinstellung erster hopfen
            lcd.home();
            lcd.print("1. Hopfen");
            if (timerHopfen1 < 10) lcd.print("   ");
            else if (timerHopfen1 < 100) lcd.print("  ");
            else lcd.print(" ");
            lcd.print(timerHopfen1);lcd.print("Min");
            lcd.setCursor(0,1);
            lcd.print("   vor Koch-Ende");
            break;
          }
          case 4:  { //nächster hopfen
            lcd.home();
            lcd.write(byte(2));
            lcd.print("n");lcd.write(byte(3));lcd.print("chster Hopfen");
            lcd.setCursor(0,1);
            lcd.print(" START          ");
            break;
          }
          case 5:  { //Zeiteinstellung 2. hopfen
            lcd.home();
            lcd.print("2. Hopfen");
            if (timerHopfen2 < 10) lcd.print("   ");
            else if (timerHopfen2 < 100) lcd.print("  ");
            else lcd.print(" ");
            lcd.print(timerHopfen2);lcd.print("Min");
            lcd.setCursor(0,1);
            lcd.print("   vor Koch-Ende");
            break;
          }
          case 6:  { //nächster hopfen
            lcd.home();
            lcd.write(byte(2));
            lcd.print("n");lcd.write(byte(3));lcd.print("chster Hopfen");
            lcd.setCursor(0,1);
            lcd.print(" START          ");
            break;
          }
          case 7:  { //Zeiteinstellung 3. hopfen
            lcd.home();
            lcd.print("3. Hopfen");
            if (timerHopfen3 < 10) lcd.print("   ");
            else if (timerHopfen3 < 100) lcd.print("  ");
            else lcd.print(" ");
            lcd.print(timerHopfen3);lcd.print("Min");
            lcd.setCursor(0,1);
            lcd.print("   vor Koch-Ende");
            break;
          }
          case 8:  { //nächster hopfen
            lcd.home();
            lcd.write(byte(2));
            lcd.print("n");lcd.write(byte(3));lcd.print("chster Hopfen");
            lcd.setCursor(0,1);
            lcd.print(" START          ");
            break;
          }
          case 9:  { //Zeiteinstellung 4. hopfen
            lcd.home();
            lcd.print("4. Hopfen");
            if (timerHopfen4 < 10) lcd.print("   ");
            else if (timerHopfen4 < 100) lcd.print("  ");
            else lcd.print(" ");
            lcd.print(timerHopfen4);lcd.print("Min");
            lcd.setCursor(0,1);
            lcd.print("   vor Koch-Ende");
            break;
          }
          case 10:  { //Start
            lcd.home();
            lcd.print("                ");
            lcd.setCursor(0,1);
            lcd.write(byte(2));
            lcd.print("START          ");
            break;
          }
        }
      }
    }
    if (hMenue == 5) { ////////////////////////////////////Statusmenü
      switch(statusMenue) {
        case 1:  {
          lcd.home();
            lcd.print("Temp     ");
            if (tempT() < 10) lcd.print("  ");
            else if (tempT() < 100) lcd.print(" ");
            lcd.print(tempT(),1);lcd.write(byte(1));lcd.print("C");
            lcd.setCursor(0,1);
            lcd.print("                ");
            break;
        }
        case 2: {
          lcd.home();
            lcd.print("Firmware    ");
            lcd.print(software);
            lcd.setCursor(0,1);
            lcd.print("                ");
            break;
        }
        case 3:  {
          lcd.home();
            lcd.print("IP Adresse:     ");
            lcd.setCursor(0,1);
            if (wifiVerbunden && WiFi.localIP() != IPAddress(0, 0, 0, 0)) {
              lcd.print(WiFi.localIP());
              lcd.print("  ");
            }
            else lcd.print("Kein wifi       ");
            break;
        }
        case 4:  {
          lcd.home();
            if (!online) {
              lcd.print("Offline Betrieb ");
              lcd.setCursor(0,1);
              lcd.print("                ");
            }
            else if (str_thingspeakChannelID != ""){
              lcd.print(str_thingspeakWriteKey);
              lcd.setCursor(0,1);
              lcd.print(str_thingspeakChannelID);
              lcd.print("        ");
            }
            else {
              lcd.print(" Wlan loeschen  ");
              lcd.setCursor(0,1);
              lcd.print("thingspeak eing.");
            }
            break;
        }
        case 5:  {
          meldung = true;
          meldungsNummer = 6;
          break;
        }
      }
    }
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////////////////////Thermostat Modus
    if (hMenue == 4) { 
      switch(thermostatMenue) {
        case 1: {  //Temperatureinstellung
          lcd.home();
          lcd.print("Thermostat      ");
          lcd.setCursor(0,1);
          lcd.print("   ");lcd.write(byte(2));
          if (manuellTemp < 10) lcd.print("Temp.    ");//Stellt passend zur Temperatur die Anzeige ein
          else if (manuellTemp < 100) lcd.print("Temp.   ");
          else if (manuellTemp >= 100) lcd.print("Temp.  ");
          lcd.print(manuellTemp);
          lcd.write(byte(1));lcd.print("C");
          break;
        }
        case 2:  { //Kühlung
          lcd.home();
          lcd.print(" K");lcd.write(byte(0));lcd.print("hlung   ");lcd.write(byte(2));
          if (kuehlungJa) lcd.print("Ja  ");
          else lcd.print("Nein");
          lcd.setCursor(0,1);
          lcd.print(" Heizung    ");
          if (heizungJa) lcd.print("Ja  ");
          else lcd.print("Nein");
          break;      
        }
        case 3: {  //Heizung
          lcd.home();
          lcd.print(" K");lcd.write(byte(0));lcd.print("hlung    ");
          if (kuehlungJa) lcd.print("Ja  ");
          else lcd.print("Nein");
          lcd.setCursor(0,1);
          lcd.print(" Heizung   ");lcd.write(byte(2));
          if (heizungJa) lcd.print("Ja  ");
          else lcd.print("Nein");
          break;
        }
        case 4: {  //Hysterese
          lcd.home();
          lcd.write(byte(2));lcd.print("Hysterese  ");
          if (hysterese < 10) lcd.print(" ");
          lcd.print(hysterese);
          lcd.write(byte(1));lcd.print("C");
          lcd.setCursor(0,1);
          lcd.print("           ");
          if (!thermostatStart) lcd.print("START");
          else lcd.print("      ");
          break;
        }
        case 5: {  //Hysterese
          lcd.home();
          lcd.print(" Hysterese  ");
          if (hysterese < 10) lcd.print(" ");
          lcd.print(hysterese);
          lcd.write(byte(1));lcd.print("C");
          lcd.setCursor(0,1);
          lcd.print("          ");lcd.write(byte(2));lcd.print("START");
          break;
        }
        case 6: {  //Übersicht während des Programms
          lcd.home();
          lcd.print("Thermostat ");
          if (hysterese < 10) lcd.print(" ");
          lcd.write(byte(7));lcd.print(hysterese);
          lcd.write(byte(1));lcd.print("C");
          lcd.setCursor(0,1);
          lcd.print(" ");
          if (tempT() < 10) lcd.print("  ");
          else if (tempT() < 100) lcd.print(" ");
          lcd.print(tempT(),1);
          lcd.write(byte(1));lcd.print("C");
          lcd.write(byte(6));
          if (tempSoll < 10) lcd.print("  ");
          else if (tempSoll < 100) lcd.print(" ");
          lcd.print(tempSoll);lcd.print(".0");
          lcd.write(byte(1));lcd.print("C");
          break;
        }
      }
    }
  }
 }
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////webInterface///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////überträgt daten an den Browser////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void webInterface () {
  webupdaterStart = false;
  httpServer.stop();
  if (!webinterfaceStart) {
    server.begin(); //startet server für Webinterface
    webinterfaceStart = true;
  }
  WiFiClient client = server.available();
  if (client) {                             // if you get a client,
    //Serial.println("New Client.");           // print a message out the Serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the Serial monitor
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            if (startUP) {
              meldungsNummer = 1;
              meldung = false;
              startUP = false;
            }
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println();
            client.print("<HTML><HEAD><TITLE>");
            client.print("BierBox");
            client.print("</TITLE>");
            if (manuellStart || autoMaischenStart || autoKochenStart) client.print("<meta http-equiv='refresh'content=\"30; URL=/\">"); //refresh während programme laufen
            if (!hMenueON && hMenue == 4) client.print("</TITLE><meta http-equiv='refresh'content=\"600; URL=/\">"); //refresh während thermostat läuft
            char buffer[20];
            client.print("</HEAD>");
            client.println("<style>");
            client.println(" .tooltip{position:relative;display:inline-block;}");
            client.println(" .tooltip .tooltiptext{visibility:hidden;width:200px;background-color:tomato;color:#fff;text-align:center;border-radius:6px;padding: 5px 0;position:absolute;z-index:1;top:0%;left:150%;opacity:0;transition:opacity 0.3s;}");
            client.println(" .tooltip:hover .tooltiptext{visibility:visible;opacity:1;}");
            client.println("</style>");
            client.print("<BODY>");
            if (hMenueON && !einstellungen) { //hauptmenü              
              client.print("<form> <button formaction=\"/zurueck\" disabled style=\"background-color:red\">Zurueck</button></form>");
              client.print("<h1>BierBox</h1>");
              client.print("<h3>Programme:</h3>");
              client.print("<form> <button formaction=\"/manuell\" style=\"width:150px\">Manuell</button></form>");
              client.print("<form> <button formaction=\"/automaischen\"style=\"width:150px\">AutoMaischen</button></form>");
              client.print("<form> <button formaction=\"/autokochen\"style=\"width:150px\">AutoKochen</button></form>");
              client.print("<form> <button formaction=\"/thermostat\"style=\"width:150px\">Thermostat</button></form>");
              client.print("<h3>&nbsp;</h3>");
              client.print("<form> <button formaction=\"/settings\"style=\"width:150px\">Einstellungen</button></form>");
            }
            else if (!hMenueON && hMenue == 1){ //manuell
              if (manuellStart) {
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">Zurueck</button></form>");
                client.print("<span class=\"tooltiptext\">Beendet das laufende Programm!");
                client.print("</span></div>"); 
              }
              else client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">Zurueck</button></form>");
              client.print("<h1>Manueller Modus</h1>");
              client.print("<table style=\"width:300px\">");
              client.print("<tr><td>IST-Temperatur:</td><td>");
              client.print(tempT());
              client.print(" &#xb0;C</td></tr>");
              client.print("<tr><td>Hysterese:</td><td>");
              client.print(gradient * k);
              client.print(" &#xb0;C</td></tr>");
              client.print("<tr><td>Status:</td><td>");
              if (heizung()) client.print("Heizung ON");
              else client.print("Heizung OFF");
              client.print("</td></tr>");
              client.print("<tr><td style=\"width:150px;\">&nbsp;</td><td style=\"width:150px;\">&nbsp;</td></tr>");
              client.print("<form action=\"/a\" method=\"get\">");
              client.print("<tr><td>SOLL-Temperatur:</td><td>");
              client.print(manuellTemp);
              client.print(" &#xb0;C</td></tr>");
              client.print("<tr><td></td><td>");
              client.print("<input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"a\" value=\"");
              client.print(manuellTemp);
              client.print("\" min=\""); client.print(tempMin); client.print("\" max=\""); client.print(tempKochenMax); client.print("\">");
              client.print(" &#xb0;C</td></tr>");
              client.print("<tr><td>&nbsp;</td></tr>");
              client.print("<tr><td>Timer:</td><td>");
              client.print(timerManuell);
              client.print(" min</td></tr><tr><td>&nbsp;</td><td>");
              client.print("<input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"b\" value=\"");
              client.print(timerManuell);
              client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
              client.print(" min</td</tr>");
              client.print("<input type=\"hidden\" name=\"z\" value=\"bier\"><br>");
              client.print("<tr><td><input type=\"submit\" value=\"Uebernehmen\"></td><td>&nbsp;</td></tr>");
              client.print("</form><br>");
              client.print("</table>");
              client.print("&nbsp;<br>"); 
              if (!manuellStart) {
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/start\" style=\"background-color:green\">START</button></form>");
                client.print("<span class=\"tooltiptext\">Vor dem Start auf Uebernehmen klicken!");
                client.print("</span></div>");          
              }
              else {
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/stop\" style=\"background-color:red\">STOP</button></form>");
                client.print("<span class=\"tooltiptext\">Beendet das laufende Programm!");
                client.print("</span></div>"); 
              }
              client.print("&nbsp;<br>"); 
              client.print("&nbsp;<br>"); 
              //Frame
              //https://thingspeak.com/channels/317449/charts/1?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&title=Temperatur&type=line
              client.print("<iframe width=\"450\" height=\"250\" style=\"border: 1px solid #cccccc;\" src=\"https://thingspeak.com/channels/");
              client.print(str_thingspeakChannelID);
              client.print("/charts/2?api_key=");
              client.print(str_thingspeakReadKey);
              client.print("&bgcolor=%23ffffff&color=%23d62020&dynamic=true&width=auto&height=auto&days=1&title=Temperaturverlauf&xaxis=&yaxis=Temperatur%20%5B%C2%B0C%5D&type=line\"></iframe>");
              client.print("&nbsp;<br>"); 
            }
            else if (!hMenueON && hMenue == 2) {  //automaischen
              if (!autoMaischenStart) { //automaischen einstellungen
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">Zurueck</button></form>");
                client.print("<span class=\"tooltiptext\">Beendet das laufende Programm!");
                client.print("</span></div>");     
                client.print("<h1>AutoMaischen Einstellungen</h1>");              
                client.print("<table style=\"width:450px\">");
                client.print("<form action=\"/a\" method=\"get\">");
                client.print("<tr><td>Einmaisch-Temperatur:</td><td>");
                client.print("<input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"a\" value=\"");
                client.print(tempEinmaischen);
                client.print("\" min=\""); client.print(tempMin); client.print("\" max=\""); client.print(tempMax); client.print("\">");
                client.print(" &#xb0;C</td></tr>");
                client.print("<tr><td style=\"width:180px;\">&nbsp;</td><td style=\"width:150px;\">&nbsp;</td><td style=\"width:120px;\">&nbsp;</td></tr>");
                client.print("<tr><td><h4>Anzahl Rasten:</h4></td><td>");
                client.print("<input type=\"radio\" name=\"o\" value=\"6\"");
                if (autoMaischenMenue == 1 || autoMaischenMenue == 6) client.print("checked");
                client.print(">2</td></tr>");
                client.print("<tr><td>&nbsp;</td><td><input type=\"radio\" name=\"o\" value=\"9\"");
                if (autoMaischenMenue == 9) client.print("checked");
                client.print(">3</td></tr>");
                client.print("<tr><td>&nbsp;</td><td><input type=\"radio\" name=\"o\" value=\"12\"");
                if (autoMaischenMenue == 12) client.print("checked");
                client.print(">4</td></tr>");
                client.print("<tr><td>&nbsp;</td><td><input type=\"radio\" name=\"o\" value=\"15\"");
                if (autoMaischenMenue == 15) client.print("checked");
                client.print(">5</td></tr>");
                client.print("<tr><td>&nbsp;</td><td><input type=\"radio\" name=\"o\" value=\"18\"");
                if (autoMaischenMenue == 18) client.print("checked");
                client.print(">6</td></tr>");
                client.print("<tr><td><h3>Maischplan</h3></td></tr>");
                client.print("<tr><td><h4>Rasten</h4></td> <td><h4>Temperatur</h4></td> <td><h4>Dauer</h4></td></tr>");
                client.print("<tr><td>1</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"b\" value=\"");
                client.print(tempStep1);
                client.print("\" min=\""); client.print(tempMin); client.print("\" max=\""); client.print(tempMax); client.print("\">");
                client.print(" &#xb0;C</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"c\" value=\"");
                client.print(timerStep1);
                client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min</td></tr>");
                client.print("<tr><td>2</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"b\" value=\"");
                client.print(tempStep2);
                client.print("\" min=\""); client.print(tempMin); client.print("\" max=\""); client.print(tempMax); client.print("\">");
                client.print(" &#xb0;C</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"c\" value=\"");
                client.print(timerStep2);
                client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min</td></tr>");
                client.print("<tr><td>3</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"b\" value=\"");
                client.print(tempStep3);
                client.print("\" min=\""); client.print(tempMin); client.print("\" max=\""); client.print(tempMax); client.print("\">");
                client.print(" &#xb0;C</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"c\" value=\"");
                client.print(timerStep3);
                client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min</td></tr>");
                client.print("<tr><td>4</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"b\" value=\"");
                client.print(tempStep4);
                client.print("\" min=\""); client.print(tempMin); client.print("\" max=\""); client.print(tempMax); client.print("\">");
                client.print(" &#xb0;C</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"c\" value=\"");
                client.print(timerStep4);
                client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min</td></tr>");
                client.print("<tr><td>5</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"b\" value=\"");
                client.print(tempStep5);
                client.print("\" min=\""); client.print(tempMin); client.print("\" max=\""); client.print(tempMax); client.print("\">");
                client.print(" &#xb0;C</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"c\" value=\"");
                client.print(timerStep5);
                client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min</td></tr>");
                client.print("<tr><td>6</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"b\" value=\"");
                client.print(tempStep6);
                client.print("\" min=\""); client.print(tempMin); client.print("\" max=\""); client.print(tempMax); client.print("\">");
                client.print(" &#xb0;C</td>");
                client.print("<td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"c\" value=\"");
                client.print(timerStep6);
                client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min</td></tr>");
                client.print("<input type=\"hidden\" name=\"z\" value=\"bier\"><br>");
                client.print("<tr><td>&nbsp;</td><td>&nbsp;</td></tr>");
                client.print("<tr><td><input type=\"submit\" value=\"Uebernehmen\"></td></tr>");
                client.print("</form><br></table>");
                if (!autoMaischenStart || autoMaischenEnde) {
                  client.print("<div class=\"tooltip\">");
                  client.print("<form> <button formaction=\"/start\" style=\"background-color:green\">START</button></form>");
                  client.print("<span class=\"tooltiptext\">Vor dem Start auf Uebernehmen klicken!");
                  client.print("</span></div>");   
                }
              }
              else if (autoMaischenStart) {
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">Zurueck</button></form>");
                client.print("<span class=\"tooltiptext\">Beendet das laufende Programm!");
                client.print("</span></div>"); 
                client.print("<h1>AutoMaischen Modus</h1>");
                client.print("<table style=\"width:450px;\">");
                client.print("<tr><td>IST-Temperatur:</td><td>");
                client.print(tempT());
                client.print(" &#xb0;C</td></tr>");
                client.print("<tr><td>SOLL-Temperatur:</td><td>");
                client.print(tempSoll);
                client.print(" &#xb0;C</td></tr>");
                client.print("<tr><td>Hysterese:</td><td>");
                client.print(gradient * k);
                client.print(" &#xb0;C</td></tr>");
                client.print("<tr><td>Status:</td><td>");
                if (heizung()) client.print("Heizung ON");
                else client.print("Heizung OFF");
                client.print("</td></tr>");
                client.print("<tr><td style=\"width:250px;\">&nbsp;</td><td style=\"width:200px;\">&nbsp;</td></tr>");
                client.print("<tr><td>");
                if (currentStep < letzterStep) {
                  client.print(currentStep+1);
                  client.print(". Rast in ");
                }
                else client.print(" Abmaischen in ");
                client.print(timerMinuten);
                client.print("m ");
                client.print(timerSekunden);
                client.print("s ");
                client.print("</td></tr></table>");
                client.print("&nbsp;<br>");
                client.print("<table style=\"width:450\"><tr><td><h3>Maischplan:</h3></td></tr>");
                client.print("<tr><td style=\"width:180px;\">&nbsp;</td><td style=\"width:150px;\">&nbsp;</td><td style=\"width:120px;\">&nbsp;</td></tr>");
                client.print("<tr><td><h4>Rasten</h4></td> <td><h4>Temperatur</h4></td><td><h4>Dauer</h4></td></tr>");
                client.print("<tr><td>1</td><td>");
                client.print(tempStep1);
                client.print(" &#xb0;C</td><td>");
                client.print(timerStep1);
                client.print(" min</td></tr>");
                client.print("<tr><td>2</td><td>");
                client.print(tempStep2);
                client.print(" &#xb0;C</td><td>");
                client.print(timerStep2);
                client.print(" min</td></tr>");
                if (letzterStep != 2) {
                  client.print("<tr><td>3</td><td>");
                  client.print(tempStep3);
                  client.print(" &#xb0;C</td><td>");
                  client.print(timerStep3);
                  client.print(" min</td></tr>");
                  if (letzterStep != 3) {
                    client.print("<tr><td>4</td><td>");
                    client.print(tempStep4);
                    client.print(" &#xb0;C</td><td>");
                    client.print(timerStep4);
                    client.print(" min</td></tr>");
                    if (letzterStep != 4) {
                      client.print("<tr><td>5</td><td>");
                      client.print(tempStep5);
                      client.print(" &#xb0;C</td><td>");
                      client.print(timerStep5);
                      client.print(" min</td></tr>");
                      if (letzterStep != 5) {
                        client.print("<tr><td>6</td><td>");
                      client.print(tempStep6);
                      client.print(" &#xb0;C</td><td>");
                      client.print(timerStep6);
                      client.print(" min</td></tr>");
                    
                      }
                    }
                  }
                }
                client.print("</table>");
                client.print("&nbsp;<br>");
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">STOP</button></form>");
                client.print("<span class=\"tooltiptext\">Beendet das laufende Programm!");
                client.print("</span></div>"); 
                client.print("&nbsp;<br>");
                client.print("&nbsp;<br>");
                //Frame
                //https://thingspeak.com/channels/317449/charts/1?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&title=Temperatur&type=line
                client.print("<iframe width=\"450\" height=\"250\" style=\"border: 1px solid #cccccc;\" src=\"https://thingspeak.com/channels/");
                client.print(str_thingspeakChannelID);
                client.print("/charts/2?api_key=");
                client.print(str_thingspeakReadKey);
                client.print("&bgcolor=%23ffffff&color=%23d62020&dynamic=true&width=auto&height=auto&days=1&title=Temperaturverlauf&xaxis=&yaxis=Temperatur%20%5B%C2%B0C%5D&type=line\"></iframe>");
              }
            }
            else if (!hMenueON && hMenue == 3) {  //autokochen
              if (!autoKochenStart) { //autokochen einstellungen
                client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">Zurueck</button></form>");
                client.print("<h1>AutoKochen Einstellungen</h1>"); 
                client.print("<table style=\"width=450px;\">");             
                client.print("<form action=\"/a\" method=\"get\">");
                client.print("<tr><td>Koch-Temperatur:</td>");
                client.print("<td><input style=\"width=50px;\" autocomplete=\"off\" type=\"number\" name=\"a\" value=\"");
                client.print(tempKochen);
                client.print("\" min=\""); client.print(tempKochenMin); client.print("\" max=\""); client.print(tempKochenMax); client.print("\">");
                client.print(" &#xb0;C</td>");
                client.print("<td>&nbsp;</td></tr>");
                client.print("<tr><td>Koch-Dauer:</td>");
                client.print("<td><input style=\"width:50px;\" autocomplete=\"off\" type=\"number\" name=\"b\" value=\"");
                client.print(timerKochen);
                client.print("\" min=\""); client.print(timerKochenMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min</td></tr>");
                client.print("<tr><td style=\"width:180px;\">&nbsp;</td><td style=\"width:150px;\">&nbsp;</td><td style=\"width:120px;\">&nbsp;</td></tr>");
                client.print("<tr><td><h4>Anzahl Hopfengaben:</h4></td>");
                client.print("<td><input type=\"radio\" name=\"o\" value=\"4\"");
                if (autoKochenMenue == 1 || autoKochenMenue == 4) client.print("checked");
                client.print(">1</td></tr>");
                client.print("<tr><td>&nbsp;</td><td><input type=\"radio\" name=\"o\" value=\"6\"");
                if (autoKochenMenue == 6) client.print("checked");
                client.print(">2</td></tr>");
                client.print("<tr><td>&nbsp;</td><td><input type=\"radio\" name=\"o\" value=\"8\"");
                if (autoKochenMenue == 8) client.print("checked");
                client.print(">3</td></tr>");
                client.print("<tr><td>&nbsp;</td><td><input type=\"radio\" name=\"o\" value=\"10\"");
                if (autoKochenMenue == 10) client.print("checked");
                client.print(">4</td></tr>");
                client.print("<tr><td><h3>Kochplan:</h3></td></tr>");
                client.print("<tr><td><h4>Hopfengaben</h4></td></tr>");
                client.print("<tr><td>1</td>");
                client.print("<td><input style=\"width:50px;\" autocomplete=\"off\" type=\"number\" name=\"c\" value=\"");
                client.print(timerHopfen1);
                client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min vor Kochende</td></tr>");
                client.print("<tr><td>2</td>");
                client.print("<td><input style=\"width:50px;\" autocomplete=\"off\" type=\"number\" name=\"c\" value=\"");
                client.print(timerHopfen2);
                client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min vor Kochende</td></tr>");
                client.print("<tr><td>3</td>");
                client.print("<td><input style=\"width:50px;\" autocomplete=\"off\" type=\"number\" name=\"c\" value=\"");
                client.print(timerHopfen3);
                client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min vor Kochende</td></tr>");
                client.print("<tr><td>4</td>");
                client.print("<td><input style=\"width:50px;\" autocomplete=\"off\" type=\"number\" name=\"c\" value=\"");
                client.print(timerHopfen4);
                client.print("\" min=\""); client.print(timerMin); client.print("\" max=\""); client.print(timerMax); client.print("\">");
                client.print(" min vor Kochende</td></tr>");
                client.print("<input type=\"hidden\" name=\"z\" value=\"bier\"><br>");
                client.print("<tr><td>&nbsp;</td><td>&nbsp;</td></tr>");
                client.print("<tr><td><input type=\"submit\" value=\"Uebernehmen\"></td></tr></table>");
                client.print("</form>");
                if (!autoKochenStart || autoKochenEnde) {
                  client.print("<div class=\"tooltip\">");
                  client.print("<form> <button formaction=\"/start\" style=\"background-color:green\">START</button></form>");
                  client.print("<span class=\"tooltiptext\">Vor dem Start auf Uebernehmen klicken!");
                  client.print("</span></div>");  
                }
              }
              else if (autoKochenStart) {
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">Zurueck</button></form>");
                client.print("<span class=\"tooltiptext\">Beendet das laufende Programm!");
                client.print("</span></div>"); 
                client.print("<h1>AutoKochen Modus</h1>");
                client.print("<table style=\"width:450px;\">");
                client.print("<tr><td>IST-Temperatur:</td><td>");
                client.print(tempT());
                client.print(" &#xb0;C</td></tr>");
                client.print("<tr><td>SOLL-Temperatur:</td><td>");
                client.print(tempSoll);
                client.print(" &#xb0;C</td></tr>");
                client.print("<tr><td>Status:</td><td>");
                if (heizung()) client.print("Heizung ON");
                else client.print("Heizung OFF");
                client.print("</td></tr>");
                client.print("<tr><td style=\"width:250px;\">&nbsp;</td><td style=\"width:200px;\">&nbsp;</td></tr>");
                client.print("<tr><td>");
                if (hopfen <= letzterHopfen && !keinHopfenMehr) {
                  client.print(hopfen);
                  client.print(". Hopfengabe in</td><td>");
                  client.print(timerHopfenMinuten);
                  client.print("m ");
                  client.print(timerHopfenSekunden);
                  client.print("s</td></tr>");
                }
                else client.print("</td></tr>");
                client.print("<tr><td>&nbsp;</td><td>&nbsp;</td></tr>");
                client.print("<tr><td>Kochende in</td><td>");
                client.print(timerMinuten);
                client.print("m ");
                client.print(timerSekunden);
                client.print("s</td></tr></table>");
                client.print("<table style=\"width:450px;\"><tr><td style=\"width:150px;\">&nbsp;</td><td style=\"width:100px;\">&nbsp;</td><td style=\"width:200px;\">&nbsp;</td></tr>");
                client.print("<tr><td><h4>Kochplan:</h4></td></tr>");
                client.print("<tr><td><h4>Hopfengaben</h4></td></tr>");
                client.print("<tr><td>1</td><td>");
                client.print(timerHopfen1);
                client.print("</td><td> min vor Kochende</td></tr>");
                if (autoKochenMenue >= 5) {
                  client.print("<tr><td>2</td><td>");
                  client.print(timerHopfen2);
                  client.print("</td><td> min vor Kochende</td></tr>");
                }
                if (autoKochenMenue >= 7) {
                  client.print("<tr><td>3</td><td>");
                  client.print(timerHopfen3);
                  client.print("</td><td> min vor Kochende</td></tr>");
                }
                if (autoKochenMenue >= 9) {
                  client.print("<tr><td>4</td><td>");
                  client.print(timerHopfen4);
                  client.print("</td><td> min vor Kochende</td></tr>");
                }
                client.print("</table>");
                client.print("&nbsp;<br>");
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">STOP</button></form>");
                client.print("<span class=\"tooltiptext\">Beendet das laufende Programm!");
                client.print("</span></div>"); 
                client.print("&nbsp;<br>");
                client.print("&nbsp;<br>");
                //Frame
                //https://thingspeak.com/channels/317449/charts/1?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&title=Temperatur&type=line
                client.print("<iframe width=\"450\" height=\"250\" style=\"border: 1px solid #cccccc;\" src=\"https://thingspeak.com/channels/");
                client.print(str_thingspeakChannelID);
                client.print("/charts/2?api_key=");
                client.print(str_thingspeakReadKey);
                client.print("&bgcolor=%23ffffff&color=%23d62020&dynamic=true&width=auto&height=auto&days=1&title=Temperaturverlauf&xaxis=&yaxis=Temperatur%20%5B%C2%B0C%5D&type=line\"></iframe>");
              }
            }
            else if (!hMenueON && hMenue == 4) { //thermostat
              if (thermostatStart) {
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">Zurueck</button></form>");
                client.print("<span class=\"tooltiptext\">Beendet das laufende Programm!");
                client.print("</span></div>"); 
              }
              else client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">Zurueck</button></form>");
              client.print("<h1>Thermostat Modus</h1>");
              client.print("<table style=\"width:300px\">");
              client.print("<tr><td>IST-Temperatur:</td><td>");
              client.print(tempT());
              client.print(" &#xb0;C</td></tr>");
              client.print("<tr><td>Status:</td><td>");
              if (heizung()) client.print("Heizung ON");
              else client.print("Heizung OFF");
              client.print("</td></tr><tr><td>&nbsp;</td><td>");
              if (kuehlung()) client.print("Kuehlung ON");
              else client.print("Kuehlung OFF");
              client.print("</td></tr>");
              client.print("<tr><td style=\"width:150px;\">&nbsp;</td><td style=\"width:150px;\">&nbsp;</td></tr>");
              client.print("<form action=\"/a\" method=\"get\">");
              client.print("<tr><td>SOLL-Temperatur:</td><td>");
              client.print(manuellTemp);
              client.print(" &#xb0;C</td></tr>");
              client.print("<tr><td>&nbsp;</td><td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"a\" value=\"");
              client.print(manuellTemp);
              client.print("\" min=\"0\" max=\"40\">");
              client.print(" &#xb0;C</td></tr>");
              client.print("<tr><td>Hysterese:</td><td>");
              client.print(hysterese);
              client.print(" &#xb0;C</td></tr>");
              client.print("<tr><td>&nbsp;</td><td><input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"b\" value=\"");
              client.print(hysterese);
              client.print("\" min=\"0\" max=\"5\">");
              client.print(" &#xb0;C</td></tr>");
              client.print("</table>");
              client.print("&nbsp;<br>");
              client.print("<input type=\"radio\" name=\"c\" value=\"1\"");
              if (kuehlungJa && !heizungJa) client.print("checked");
              client.print(">Kuehlung verwenden<br>");
              client.print("<input type=\"radio\" name=\"c\" value=\"2\"");
              if (!kuehlungJa && heizungJa) client.print("checked");
              client.print(">Heizung verwenden<br>");
              client.print("<input type=\"radio\" name=\"c\" value=\"3\"");
              if (kuehlungJa && heizungJa) client.print("checked");
              client.print(">Beides verwenden<br>");
              client.print("<input type=\"hidden\" name=\"z\" value=\"bier\"><br>");
              client.print("<input type=\"submit\" value=\"Uebernehmen\">");
              client.print("</form><br>");
              client.print("&nbsp;<br>");
              if (!thermostatStart) {
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/start\" style=\"background-color:green\">START</button></form>");
                client.print("<span class=\"tooltiptext\">Vor dem Start auf Uebernehmen klicken!");
                client.print("</span></div>");  
              }
              else {
                client.print("<div class=\"tooltip\">");
                client.print("<form> <button formaction=\"/stop\" style=\"background-color:red\">STOP</button></form>");
                client.print("<span class=\"tooltiptext\">Beendet das laufende Programm!");
                client.print("</span></div>"); 
              }
              client.print("&nbsp;<br>");
              client.print("&nbsp;<br>");
              //Frame
              //https://thingspeak.com/channels/317449/charts/1?bgcolor=%23ffffff&color=%23d62020&dynamic=true&results=60&title=Temperatur&type=line
              client.print("<iframe width=\"450\" height=\"250\" style=\"border: 1px solid #cccccc;\" src=\"https://thingspeak.com/channels/");
              client.print(str_thingspeakChannelID);
              client.print("/charts/2?api_key=");
              client.print(str_thingspeakReadKey);
              client.print("&bgcolor=%23ffffff&color=%23d62020&dynamic=true&width=auto&height=auto&days=");
              client.print(graphTage);
              client.print("&title=Temperaturverlauf%20der%20letzten%20");
              client.print(graphTage);
              client.print("%20Tage&xaxis=&yaxis=Temperatur%20%5B%C2%B0C%5D&type=line\"></iframe>");
              client.print("&nbsp;<br>");
              client.print("<form action=\"/a\" method=\"get\">");
              client.print("Anzeige der x-Achse fuer");
              client.print("<input style=\"width:50px\" autocomplete=\"off\" type=\"number\" name=\"a\" value=\"");
              client.print(graphTage);
              client.print("\" min=\"1\" max=\"31\">");
              client.print(" Tage ");
              client.print("<input type=\"hidden\" name=\"z\" value=\"graph\"><br>");
              client.print("<input type=\"submit\" value=\"Uebernehmen\">");
              client.print("</form><br>");
            }
            else if (hMenueON && einstellungen) {  //einstellungen
              client.print("<form> <button formaction=\"/zurueck\" style=\"background-color:red\">Zurueck</button></form>");
              client.print("<h1>Einstellungen</h1>");
              client.print("<h3>Firmware:</h3>");
              client.print("Version: ");
              client.print(software);client.print("<br>");
              client.print("&nbsp;<br>");
              client.print("<div class=\"tooltip\">");
              client.print("<form> <button formaction=\"/otaupdate\"style=\"width:150px;\">Update starten</button></form>");
              client.print("<span class=\"tooltiptext\">Wird kein Update hochgeladen so muss der Mikrocontroller resettet werden damit er wieder nutzbar ist.");
              client.print("</span></div>");
              client.print("&nbsp;<br>");
              client.print("&nbsp;<br>");
              client.print("Nach dem Klick auf \"Update starten\" bitte <a href=\"/update\">HIER</a> klicken.<br>");
              client.print("<h3>WLAN:</h3>");
              client.print("<table style=\"width:300px\">");
              client.print("<tr><td style=\"width:150px\">Verbindung:</td><td style=\"width:150px\">OK</td></tr>");
              client.print("<tr><td>Netzwerk:</td><td>");
              client.print(WiFi.SSID());
              client.print("</td></tr>");
              client.print("<tr><td>IP Adresse:</td><td>");
              client.print(WiFi.localIP());
              client.print("</td></tr>");
              client.print("<tr><td>MAC Adresse:</td><td>");
              client.print(WiFi.macAddress());
              client.print("</td></tr></table>");
              client.print("<h3>Thingspeak:</h3>");
              client.print("<a target=\"_blank\" href=\"https://www.thingspeak.com\">www.thingspeak.com</a><br>");
              client.print("<table style=\"width:300px\"><tr><td style=\"width:150px\">Channel ID:</td><td style=\"width:150px\">");
              client.print(str_thingspeakChannelID);
              client.print("</td></tr>");
              client.print("<tr><td>Write API Key:</td><td>");
              client.print(str_thingspeakWriteKey);
              client.print("</td></tr>");
              client.print("<tr><td>Read API Key:</td><td>");
              client.print(str_thingspeakReadKey);
              client.print("</td></tr>");
              client.print("<tr><td>User Key:</td><td>");
              client.print(str_thingspeakUserKey);
              client.print("</td></tr>");
              client.print("</table>");
              client.print("&nbsp;<br>");
              client.print("<div class=\"tooltip\">");
              client.print("<form> <button formaction=\"/logloeschen\"style=\"width:150px;\">Thingspeak Log loeschen</button></form>");
              client.print("<span class=\"tooltiptext\">Loescht die Log-Daten im hinterlegten Thingspeak Channel.");
              client.print("</span></div>");
              client.print("&nbsp;<br>");
              client.print("<div class=\"tooltip\">");
              client.print("<form> <button formaction=\"/zugangloeschen\"style=\"width:150px;\">Thingspeak Zugangsdaten loeschen</button></form>");
              client.print("<span class=\"tooltiptext\">Loescht zudem die WLAN Zugangsdaten.<br>");
              client.print("&nbsp;<br>");
              client.print("Die neuen Thingspeak Zugangsdaten muessen dann beim Einwaehlen ins WLAN eingegeben werden");
              client.print("</span></div>");
            }
            client.print("&nbsp;<br>");
            client.print("&nbsp;<br>");
            if (manuellStart || autoMaischenStart || autoKochenStart || thermostatStart) client.print("<a style =\"font-size:8pt\" href=\"/\">AKTUALISIEREN</a><br>");
            client.print("</BODY></HTML>");
            client.println();
            ////////////////
            //////klick verarbeitung
            // break out of the while loop:
            break;
          } else {    // if you got a newline, then clear currentLine:
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
        ////Serial.println(currentLine);
        // Check the request
        if (currentLine.endsWith("Referer")) gotValue = true;
        if (hMenueON && !einstellungen) { //hauptmenü
          if (currentLine.endsWith("GET /manuell")) {
          hMenue = 1;
          hMenueON = false;
          nMenueON = true;
          }
          if (currentLine.endsWith("GET /automaischen")) {
          hMenue = 2;
          hMenueON = false;
          nMenueON = true;
          }
          if (currentLine.endsWith("GET /autokochen")) {
          hMenue = 3;
          hMenueON = false;
          nMenueON = true;
          }
          if (currentLine.endsWith("GET /thermostat")) {
            hMenue = 4;
            hMenueON = false;
            nMenueON = true;
          }
          if (currentLine.endsWith("GET /settings")) {
            einstellungen = true;
          }
        }
        if (!hMenueON && hMenue == 1){ //manuell
          if (currentLine.endsWith("GET /zurueck") && !gotValue) {
          hMenueON = true;
          nMenueON = false;
          manuellEnde = true;
          manuellMenue = 1;
          }
          if (currentLine.endsWith("GET /start") && !gotValue) {
            manuellStart = true;
            manuellMenue = 4;
          }
          if (currentLine.endsWith("GET /stop") && !gotValue) {
            manuellEnde = true;
            manuellMenue = 1;
          }
          if (currentLine.endsWith("bier") && !gotValue) {
            //Serial.println(currentLine);    
            String value;
            value = getValue(currentLine, '=', 1);
            manuellTemp = value.toInt();
            //Serial.println(currentLine);
            //Serial.println("value: " + value);
            //Serial.println("mtemp");
            //Serial.println(manuellTemp);
            //Serial.println();
            value = "";
  
            value = getValue(currentLine, '=', 2);
            timerManuell = value.toInt();
            timerSet = timerManuell*60;
            //Serial.println(currentLine);
            //Serial.println("value: " + value);
            //Serial.println("timer");
            //Serial.println(timerManuell);
            //Serial.println();
            value = "";
          }
        }
        if (!hMenueON && hMenue == 2) { //automaischen
          if (!autoMaischenStart) { //automaischen einstellungen
            if (currentLine.endsWith("GET /zurueck") && !gotValue) {
            hMenueON = true;
            nMenueON = false;
            autoMaischenMenue = 1;
            }
            if (currentLine.endsWith("GET /start") && !gotValue) {
              maximaMaischen = true;
              autoMaischenStart = true;
            }
            //tem=59&ti=57&zi=10&tii=64&zii=50&tiii=72&ziii=30&tiv=78&ziv=20&tv=20&zv=1&tvi=20&zvi=1&input=6&z=bierbox
            if (currentLine.endsWith("bier") && !gotValue) {
              //Serial.println(currentLine);       
              String value;
              int i = 1;
              while (i < 15) {
                value = getValue(currentLine, '=', i);
                int x = value.toInt();
                //Serial.print("i: ");
                //Serial.println(i);
                //Serial.print("x: ");
                //Serial.println(x);
                //Serial.println();
                if (i == 1) tempEinmaischen = x;
                else if (i == 2) autoMaischenMenue = x;
                else if (i == 3) tempStep1 = x;
                else if (i == 4) timerStep1 = x;
                else if (i == 5) tempStep2 = x;
                else if (i == 6) timerStep2 = x;
                else if (i == 7) tempStep3 = x;
                else if (i == 8) timerStep3 = x;
                else if (i == 9) tempStep4 = x;
                else if (i == 10) timerStep4 = x;
                else if (i == 11) tempStep5 = x;
                else if (i == 12) timerStep5 = x;
                else if (i == 13) tempStep6 = x;
                else if (i == 14) timerStep6 = x;
                i++;
              }
            }
          }
          else if (autoMaischenStart) {
            if (currentLine.endsWith("GET /zurueck") && !gotValue) {
            hMenueON = true;
            nMenueON = false;
            autoMaischenEnde = true;
            noAlarm = true;
            autoMaischenMenue = 1;
            meldung = true;
            timerEnde = false;
            timerStart1 = false;
            meldungsNummer = 1;
            }
          }
        }
        if (!hMenueON && hMenue == 3) { //autokochen
          if (!autoKochenStart) { //automaischen einstellungen
            if (currentLine.endsWith("GET /zurueck") && !gotValue) {
            hMenueON = true;
            nMenueON = false;
            autoKochenMenue = 1;
            }
            if (currentLine.endsWith("GET /start") && !gotValue) {
              maximaKochen = true;
              autoKochenStart = true;
            }
            //a?a=25&b=20&o=10&c=19&c=18&c=17&c=16&z=bier
            if (currentLine.endsWith("bier") && !gotValue) {
              ////Serial.println(currentLine);       
              String value;
              int i = 1;
              while (i < 15) {
                value = getValue(currentLine, '=', i);
                int x = value.toInt();
                ////Serial.print("i: ");
                ////Serial.println(i);
                ////Serial.print("x: ");
                ////Serial.println(x);
                ////Serial.println();
                if (i == 1) tempKochen = x;
                else if (i == 2) timerKochen = x;
                else if (i == 3) autoKochenMenue = x;
                else if (i == 4) timerHopfen1 = x;
                else if (i == 5) timerHopfen2 = x;
                else if (i == 6) timerHopfen3 = x;
                else if (i == 7) timerHopfen4 = x;
                i++;
                delay(1);
              }
            }
          }
          else if (autoKochenStart) {
            if (currentLine.endsWith("GET /zurueck") && !gotValue) {
            hMenueON = true;
            nMenueON = false;
            autoKochenEnde = true;
            noAlarm = true;
            autoKochenMenue = 1;
            meldung = true;
            timerEnde = false;
            timerStart1 = false;
            meldungsNummer = 1;
            }
          }
        }
        if (!hMenueON && hMenue == 4) { //thermostat
          if (currentLine.endsWith("GET /zurueck") && !gotValue) {
          hMenueON = true;
          nMenueON = false;
          thermostatStart = false;
          thermostatMenue = 1;
          }
          if (currentLine.endsWith("GET /start") && !gotValue) {
            thermostatStart = true;
            thermostatMenue = 6;
          }
          if (currentLine.endsWith("GET /stop") && !gotValue) {
            thermostatStart = false;
            thermostatMenue = 1;
          }
          if (currentLine.endsWith("bier") && !gotValue) {
            ////Serial.println(currentLine);       
            String value;
            value = getValue(currentLine, '=', 1);
            manuellTemp = value.toInt();
            ////Serial.println(currentLine);
            ////Serial.println("value: " + value);
            ////Serial.println("mtemp");
            ////Serial.println(manuellTemp);
            ////Serial.println();
            value = "";
  
            value = getValue(currentLine, '=', 2);
            hysterese = value.toInt();
            ////Serial.println(currentLine);
            ////Serial.println("value: " + value);
            ////Serial.println("hysterese ");
            ////Serial.println(hysterese);
            ////Serial.println();
            value = "";
            
            value = getValue(currentLine, '=', 3);
            int x = value.toInt();
            if (x == 1) {
              kuehlungJa = true;
              heizungJa = false;
            }
            else if (x == 2) {
              kuehlungJa = false;
              heizungJa = true;
            }
            else if (x == 3) {
              kuehlungJa = true;
              heizungJa = true;
            }
            ////Serial.println(currentLine);
            ////Serial.println("value: " + value);
            ////Serial.println("x ");
            ////Serial.println(x);
            ////Serial.println();
            value = "";
          }
          if (currentLine.endsWith("graph") && !gotValue) {
            ////Serial.println(currentLine);       
            String value;
            value = getValue(currentLine, '=', 1);
            graphTage = value.toInt();
            ////Serial.println(currentLine);
            ////Serial.println("value: " + value);
            ////Serial.println("graphTage");
            ////Serial.println(graphTage);
            ////Serial.println();
            value = "";
          }
        }
        if (hMenueON && einstellungen) { //tingspeak zugangsdaten
          if (currentLine.endsWith("GET /zurueck") && !gotValue) {
          einstellungen = false;
          }
          if (currentLine.endsWith("GET /otaupdate")) {
            noUpdate = false;
            meldung = true;
            meldungsNummer = 7;
          }
          if (currentLine.endsWith("GET /logloeschen")) {
            thingspeakChannelClear = true;
          }

          if (currentLine.endsWith("GET /zugangloeschen")) {
            WiFiManager wifiManager;
            wifiManager.resetSettings();
            delay(10);
            wifiManager.resetSettings();
            delay(10);
            SPIFFS.format();
          }
        }
      }
      delay(1);
    }
  }
   client.stop();
   gotValue = false;
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////webinterface url zu Zahl///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
String getValue(String data, char separator, int index) {
    int found = 0;
    int strIndex[] = { 0, -1, -2 };
    int maxIndex = data.length() - 1;

    for (int i = 0; i <= maxIndex && found <= index; i++) {
        if (data.charAt(i) == separator || i == maxIndex) {
            found++;
            strIndex[0] = strIndex[1] + 1;
            strIndex[1] = (i == maxIndex) ? i+1 : i;
        }
    }
    return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////thingspeak.com///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////überträgt daten an einen thingspeak account////////////////////////////////////////////
/////////////////////////////////////////http://www.arduinesp.com/thingspeak /////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void thingspeakLog () {
    if (thermostatStart) {
      thingspeakIntervall = 600000;
    }
    else {
      thingspeakIntervall = 20000;
    }
    if (millis() - thingspeakPreviousMillis >= thingspeakIntervall && client.connect(serverThingspeak,80)) {
      thingspeakPreviousMillis = millis();
      String thingspeakHeader;
      String postStr;
      if (thingspeakChannelClear) {
         thingspeakHeader = "DELETE /channels/"+str_thingspeakChannelID+"/feeds HTTP/1.1";
         postStr = "api_key="+str_thingspeakUserKey;
      }
      else {
         thingspeakHeader = "POST /update HTTP/1.1";
         thingspeakTime += thingspeakIntervall/60000;
         String str_tempT = String(tempT(), 1);
         String str_thingspeakTime = String(thingspeakTime);
         /*
         char str_tempT[8];
         dtostrf(tempT(), 4, 2, str_tempT);
         String str_thingspeakTime = itoa(thingspeakTime, buffer, 10);
         */
         DateTime now = RTC.now();
         String statTag = String(now.day());
         String statMonat = String(now.month());
         String statJahr = String(now.year());
         String statStunde = String(now.hour());
         String statMinute = String(now.minute());
         String statSekunde = String(now.second());
         String str_thingspeakStatus = "Datum:"+statTag+"."+statMonat+"."+statJahr+"Uhrzeit:"+statStunde+":"+statMinute+":"+statSekunde;
         postStr = "api_key="+str_thingspeakWriteKey+"&field2="+str_tempT+"&field1="+str_thingspeakTime+"&status="+str_thingspeakStatus;
      }
      client.println(thingspeakHeader);
      client.println("Host: api.thingspeak.com");
      client.println("Connection: close");
      client.println("Content-Type: application/x-www-form-urlencoded");
      client.print("Content-Length: ");
      client.println(postStr.length());
      client.println();
      client.print(postStr);
      thingspeakChannelClear = false;
    }
    client.stop();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////wifimanager///////////////////////////////////////////////////////////////////
//////////////////////////////////////////////erlaubt eingabe von zugangsdaten und keys////////////////////////////////////////////
///////////////////////////////////////////https://github.com/tzapu/WiFiManager /////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void saveConfigCallback () {
  ////Serial.println("Should save config");
  shouldSaveConfig = true;
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////WebUpdater/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void webUpdate() {
  webinterfaceStart = false;
  server.stop();
  if (!webupdaterStart) {
    MDNS.begin(host);
    httpUpdater.setup(&httpServer);
    httpServer.begin();
    MDNS.addService("http", "tcp", 80);
    ////Serial.printf("HTTPUpdateServer ready! Open http://%s.local/update in your browser\n", host);
    webupdaterStart = true;
  }
  httpServer.handleClient();
  delay(1);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////SETUP und LOOP/////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  WiFi.hostname("BierBox");
  Serial.begin(115200); //Serielle Ausgabe wird angeschaltet
  ////////////////
  //Aktiviert Pins
  pinMode(heizungPin, OUTPUT);
  digitalWrite(heizungPin, HIGH);
  pinMode(kuehlungPin, OUTPUT);
  digitalWrite(kuehlungPin, HIGH);
  pinMode(buzzerPin, OUTPUT);
  pinMode(keypadPin, INPUT);
  Wire.begin(bmpSDA,bmpSCL); //([SDA], [SCL])
  ///////////////
  //startet RTC
  RTC.begin();
  if (! RTC.isrunning()) {
    ////Serial.println("RTC is NOT running!");
    // following line sets the RTC to the date & time this sketch was compiled
    RTC.adjust(DateTime(__DATE__, __TIME__));
  }
  else ////Serial.println("RTC is running");
  //////////////
  //startet ds18b20 temperatursensor
  sensors.begin();
  sensors.setResolution(10);
  /*Resol   Conversion  time
  9 bits    0.5°C       93.75 ms
  10 bits   0.25°C      187.5 ms
  11 bits   0.125°C     375 ms
  12 bits   0.0625°C    750 ms*/
  /////////////////
  //LCD Display Bibliothek wird gestartet (16 Spalten, 2 Zeilen)
  lcd.begin(16, 2); 
  lcd.init();
  lcd.backlight();
  lcd.createChar(0, ue);
  lcd.createChar(1, grad);
  lcd.createChar(2, pfeil);
  lcd.createChar(3, ae);
  lcd.createChar(4, up);
  lcd.createChar(5, down);
  lcd.createChar(6, soll);
  lcd.createChar(7, pm);
  ///////////////////
  //OFFLINE - ONLINE
  byte a = 0;
  unsigned long b = 0;
  byte acTime = 10;
  while (a < acTime && online) {
    if (millis() >= b + 1000) {
      a++;
      b = millis();
    }
    lcd.home();
    lcd.print("F. OFFLINE BETR.");
    lcd.setCursor(0,1);
    lcd.print(" OK DRUECKEN ");
    if (acTime - a < 10) lcd.print(" ");
    lcd.print(acTime - a);
    lcd.print("s");
    int z = analogRead(keypadPin);
    if (z < 600 && z > 480){
      online = false;
      lcd.home();
      lcd.print("    BierBox     ");
      lcd.setCursor(0,1);
      lcd.print("    Offline     ");
      delay(2000);
    }
    delay(1);
  }
  if (online) {
    ////////////////
    //WIFI
      //WiFi.mode(WIFI_AP_STA);
      lcd.home();
      lcd.print("    BierBox     ");
      lcd.setCursor(0,1);
      lcd.print("     Online     ");
      delay(2000);
      //clean FS, for testing
      //SPIFFS.format();
      //read configuration from FS json
      ////Serial.println("mounting FS...");
      if (SPIFFS.begin()) {
        ////Serial.println("mounted file system");
        if (SPIFFS.exists("/configBierBox.json")) {
          //file exists, reading and loading
          //Serial.println("reading config file");
          File configFile = SPIFFS.open("/configBierBox.json", "r");
          if (configFile) {
            //Serial.println("opened config file");
            size_t size = configFile.size();
            // Allocate a buffer to store contents of the file.
            std::unique_ptr<char[]> buf(new char[size]);
            configFile.readBytes(buf.get(), size);
            DynamicJsonBuffer jsonBuffer;
            JsonObject& json = jsonBuffer.parseObject(buf.get());
            json.printTo(Serial);
            if (json.success()) {
              //Serial.println("\nparsed json");
              strcpy(thingspeakWriteKey, json["thingspeakWriteKey"]);
              strcpy(thingspeakReadKey, json["thingspeakReadKey"]);
              strcpy(thingspeakUserKey, json["thingspeakUserKey"]);
              strcpy(thingspeakChannelID, json["thingspeakChannelID"]);
            } else {
              //Serial.println("failed to load json config");
            }
          }
        }
      } else {
        //Serial.println("failed to mount FS");
      }
      //end read
      // The extra parameters to be configured (can be either global or just in the setup)
      // After connecting, parameter.getValue() will get you the configured value
      // id/name placeholder/prompt default length
      WiFiManagerParameter custom_thingspeakWriteKey("writeKey", "API Write Key", thingspeakWriteKey, 40);
      WiFiManagerParameter custom_thingspeakReadKey("readKey", "API Read Key", thingspeakReadKey, 40);
      WiFiManagerParameter custom_thingspeakUserKey("userKey", "User Key", thingspeakUserKey, 40);
      WiFiManagerParameter custom_thingspeakChannelID("channelID", "Channel ID", thingspeakChannelID, 10);
      //WiFiManager
      //Local intialization. Once its business is done, there is no need to keep it around
      WiFiManager wifiManager;
      //set config save notify callback
      wifiManager.setSaveConfigCallback(saveConfigCallback);
      //set static ip
      //wifiManager.setSTAStaticIPConfig(IPAddress(10,0,1,99), IPAddress(10,0,1,1), IPAddress(255,255,255,0));
      //add all your parameters here
      wifiManager.addParameter(&custom_thingspeakWriteKey);
      wifiManager.addParameter(&custom_thingspeakReadKey);
      wifiManager.addParameter(&custom_thingspeakUserKey);
      wifiManager.addParameter(&custom_thingspeakChannelID);
      //reset settings - for testing
      //wifiManager.resetSettings();
      //set minimu quality of signal so it ignores AP's under that quality
      //defaults to 8%
      //wifiManager.setMinimumSignalQuality();
      //sets timeout until configuration portal gets turned off
      //useful to make it all retry or go to sleep
      //in seconds
      wifiManager.setTimeout(240);
      //and goes into a blocking loop awaiting configuration
      lcd.home();
      lcd.print("Verbindungsaufb. ");
      lcd.setCursor(0,1);
      lcd.print("4min f");lcd.write(byte(0));
      lcd.print("r connect");
      wifiManager.autoConnect("BierBox", "anno1516");
      if (WiFi.status() == WL_CONNECTED){
        wifiVerbunden = true;
        //if you get here you have connected to the WiFi
        //Serial.println("connected...yeey :)");
        lcd.home();
        lcd.print("   Verbindung   ");
        lcd.setCursor(0,1);
        lcd.print("   hergestellt  ");
      }
      else {
        //Serial.println("failed to connect and hit timeout");
        lcd.clear();
        lcd.home();
        lcd.print("keine Verbindung");
      }
      //read updated parameters
      strcpy(thingspeakWriteKey, custom_thingspeakWriteKey.getValue());
      strcpy(thingspeakReadKey, custom_thingspeakReadKey.getValue());
      strcpy(thingspeakUserKey, custom_thingspeakUserKey.getValue());
      strcpy(thingspeakChannelID, custom_thingspeakChannelID.getValue());
      //save the custom parameters to FS
      if (shouldSaveConfig) {
        //Serial.println("saving config");
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.createObject();
        json["thingspeakReadKey"] = thingspeakReadKey;
        json["thingspeakWriteKey"] = thingspeakWriteKey;
        json["thingspeakUserKey"] = thingspeakUserKey;
        json["thingspeakChannelID"] = thingspeakChannelID;
        File configFile = SPIFFS.open("/configBierBox.json", "w");
        if (!configFile) {
          //Serial.println("failed to open config file for writing");
        }
        json.printTo(Serial);
        json.printTo(configFile);
        configFile.close();
        //end save
      }
      str_thingspeakReadKey = String(thingspeakReadKey);
      str_thingspeakUserKey = String(thingspeakUserKey);
      str_thingspeakWriteKey = String(thingspeakWriteKey);
      str_thingspeakChannelID = String(thingspeakChannelID);
      //Serial.println("local ip");
      //Serial.println(WiFi.localIP());
      delay(2000);
    ////////////////
    //thingspeak.com
    if (wifiVerbunden) {
      meldung = true;
      meldungsNummer = 5;
    }
  timeClient.begin();
  timeClient.update();
  RTC.adjust(DateTime(timeClient.getEpochTime()));
  }
}
void loop() {
  unsigned long start = micros();
  //
  //schaltet programme ein
  eingabe();  //aktiviert die Eingabe durch buttons
  userInterface(); //Aktiviert das Userinterface
  autoMaischen(); //aktiviert Automaischen modus
  autoKochen();  //aktiviert Autokochen modus
  manuell(); //aktiviert den manuellen Modus
  timer();  //startet Timer für die gesamt Dauer
  hopfentimer(); //startet Timer für die einzellnen Hopfen
  //
  //
  //Schaltet die Heizung aus und ein
  if (heizung() == true && !heizungOFF){
    digitalWrite(heizungPin, LOW);  //schaltet Heizung ein
  }
    else {
      digitalWrite(heizungPin, HIGH);   //schaltet Heizung aus
    }
  //
  //
  //Schaltet die Kühlung aus und ein
  if (kuehlung() == true){
    digitalWrite(kuehlungPin, LOW);  //schaltet kühlung ein
  }
    else {
      digitalWrite(kuehlungPin, HIGH);   //schaltet kühlung aus
    }
  //
  //Schaltet den Buzzer aus und ein
  if (buzzer() == true  && alarmON) digitalWrite(buzzerPin, HIGH);
  else digitalWrite(buzzerPin, LOW);
  //
  //Schaltet thingspeak.com aus und ein
  if (autoMaischenStart || autoKochenStart || manuellStart || thermostatStart) {
    if (wifiVerbunden && online) thingspeakLog();
  }
  if (wifiVerbunden && online) { //steuert webinterface
    if (noUpdate) webInterface();
    else if (!noUpdate) webUpdate();
  }
  runtime = micros() - start;
}

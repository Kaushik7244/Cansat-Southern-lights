#include <Arduino.h>
#include <Wire.h>
#include <Arduino_BHY2.h>
#include <list>


void setup() {
  Serial.begin(9600); //baud rate

  //Her lager vi en overskrift over hvilke data som skrives i de ulike kolonnene.
  
  Serial.print("callsign"); //navn på CanSat'en
  Serial.print("; "); //delimiter - tegn som separerer individuelle data
  
  Serial.print("time (ms)"); //tid siden oppstart i millisekund
  Serial.print("; ");
  
  Serial.print("counter"); //teller som viser antall ganger løkken har blitt kjørt
  Serial.print("; ");
  
  Serial.print("ntc"); //temperatursensor
  Serial.print("; ");
  
  Serial.print("pressure"); //lufttrykk 
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print("Alt(m)"); //lufttrykk 
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print("Longatude"); //lufttrykk 
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print("Latitude"); //lufttrykk 
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print("Paretschute status"); //lufttrykk 
  Serial.println("; "); //NB - legg merke til linjeskift

}

void loop() {
Serial.print("Fallschirmjäger"); //navn skrives som tekst
  Serial.print("; "); //delimiter skrives som tekst
  
  Serial.print(ms); //funksjon som viser tid siden oppstart 
  Serial.print("; "); 
  
  Serial.print(counter++); //legg merke til ++, øker verdien på variablen med 1 for hver runde løkken har blitt kjørt
  Serial.print("; ");
  
  Serial.print(ntc); //verdien fra analog temperatursensor
  Serial.print("; ");

  Serial.print(pressure); //lufttrykk i pascal
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print(alt); //høyde
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print(longitude); //longitude kordinaten
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print(latitude); //latitude kordinaten
  Serial.print("; "); //NB - legg merke til linjeskift

  Serial.print(paratschute_deployed); //latitude kordinaten
  Serial.println("; "); //NB - legg merke til linjeskift
}


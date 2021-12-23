//Neuprogrammierung nach Totalverlust
//Thermometer mit DS18B20 Sensor und VFDisplay
//VFD angesteuert mittels 74HC595 und UDN2981

#include <Arduino.h>

//variables declarations
int toggleInterval = 200; //Zeit, die die LED an bzw aus ist in Millisekunden
int msgInterval = 1000;

bool keepFlag = 0;
unsigned long keepHmillis;

//functions declarations
void noDelayBlink(byte pin, int blinkZeit );
void serialMsg(int msgZeit);
void keepPinHighForTime(int keepHTime, int pin);

void pwmInit(void);
void pwmStop(void);
//////////////////////////////////////////////////////
// Setup fuer die LED und Serielle Ausgabe
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT); //LED_BUILTIN ist keyword fÃƒÂ¼r die LED auf dem Board, bei Arduino an Pin 13
  pinMode(12, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(8, INPUT_PULLUP);
}

//////////////////////////////////////////////////////
// the loop function runs over and over again forever
void loop() {
  noDelayBlink(LED_BUILTIN, toggleInterval);
  serialMsg(msgInterval);
  keepPinHighForTime(200, 12);
  if (digitalRead(8) == HIGH) {
    //Serial.println("8 ist HIGH");
    keepFlag = 0;
  }
}
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
void noDelayBlink(byte pin, int blinkZeit ) {
  static unsigned long blinkMillis;
  if (millis() - blinkMillis > blinkZeit) {
    blinkMillis = millis();
    digitalWrite(pin, !digitalRead(pin)); //toggeln eines Digitalausgangs
  }
}

void serialMsg(int msgZeit) {
  static unsigned long blinkMillis; //eigene Variable gleichen Namens, unabhaengig von der Variable in der "toggle-Funktion"
  if (millis() - blinkMillis > msgZeit) {
    blinkMillis = millis();
    Serial.println("Test Message");
    Serial.print("Vergangene Millisekunden seit Programmstart: ");
    Serial.println(millis());
    Serial.println("----- ----- ----- ----- -----");
  }
}

void keepPinHighForTime(int keepHTime, int pin) {
  static bool doneFlag;
  if (keepFlag == 0) {
    keepHmillis = millis();
    doneFlag = 0;
    keepFlag = 1;
    //Serial.println("SET");
  }
  if (millis() - keepHmillis < keepHTime && doneFlag == 0) {
    digitalWrite(pin, HIGH);
  }
  else {//if (millis() - keepHmillis > keepHTime ODER done Flag == 1)
    digitalWrite(pin, LOW);
    doneFlag = 1;
  }
}

void pwmInit(void) {
  //Timer2 setup
  //TCCR2A = 0; //Timer/Counter Control Register A
  TCCR2A |= (1<<COM2A1) | (1<<COM2A0) | (1<<COM2B1) | (0<<COM2B0);
  TCCR2A |= (1<<WGM21) | (1<<WGM20);
  //TCCR2B = 0; //Timer/Counter Control Register B
  TCCR2B |= (0<<CS22) | (1<<CS21) | (0<<CS20); //0 1 0 entspricht 1 zu 8
  TCCR2B |= (0<<WGM22);
  //TCNT2 = 0; // Timer/Counter Register: The Timer/Counter Register gives direct access, both for read and write operations, to theTimer/Counter unit 8-bit counte
  //OCR2A = 0; //Output Compare Register A
  OCR2A = 13;
  //OCR2B = 0; //Output Compare Register B
  OCR2B = 242;
  //TIMSK2 = 0; // Timer/Counter Interrupt Mask Registe
  //TIFR2 = 0; //  Timer/Counter 0 Interrupt Flag Registe
  //Ende Timer2 setup
}

void pwmStop(void) {
  TCCR2A &= (0<<COM2A1) | (0<<COM2A0) | (0<<COM2B1) | (0<<COM2B0);
  TCCR2B &= ~((1<<CS22)|(1<<CS21)|(1<<CS20));
}

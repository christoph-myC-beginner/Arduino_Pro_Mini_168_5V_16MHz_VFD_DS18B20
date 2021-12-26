//Neuprogrammierung nach Totalverlust
//Thermometer mit DS18B20 Sensor und VFDisplay
//VFD angesteuert mittels 74HC595 und UDN2981

#include <Arduino.h>

// Include the libraries we need
#include <OneWire.h>
#include <DallasTemperature.h>

OneWire oneWire(2); //OneWire an Pin 2
// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature DS18B20(&oneWire);

//variables declarations
int latchPin = 7;  // RCLK (Register Clock / Latch) Pin des 74HC595, der die Daten aus dem Storage an den Ausgang gibt
int clockPin = 8; // SRCLK (Shit Register Clock) Pin, mit dessen Flanke die Daten in das Register geschiftet werden
int dataPin = 5;  // SER (Serial input) Pin des 74HC595, der die Daten aufnimmt
int enablePin = 6; // EN (LOW active!) der die Ausgaenge des 74HC595 freischaltet

int toggleInterval = 800; //Zeit, die die LED an bzw aus ist in Millisekunden
int msgInterval = 2000;
int checkTempInt = 300;
int showTempInt = 800;
unsigned long showFunInt = 50000;
int scrollTime = 250;

int tempZehner;
int tempEiner;

bool keepFlag = 0;

unsigned long showFunMillis;

//Array fuer Shiftregister bzw 7-Segment definieren
//                   abcdefgGitter
byte ciphArr[19] = {0b00000000, // leer - 0
                    0b01100001, // 1
                    0b11011011, // 2
                    0b11110011, // 3
                    0b01100111, // 4
                    0b10110111, // 5
                    0b10111111, // 6
                    0b11100001, // 7
                    0b11111111, // 8
                    0b11110111, // 9
                    0b11111101, // 0 - 10
                    0b11101111, // A - 11
                    0b10011101, // C - 12
                    0b10011111, // E - 13
                    0b10001111, // F - 14
                    0b01101111, // H - 15
                    0b00011101, // L - 16
                    0b00101011, // n - 17
                    0b11000111  // ° - 18 //19!
                    };

//functions declarations
//void noDelayBlink(byte pin, int blinkZeit );
//void serialMsg(int msgZeit);
void checkTemp(int checkTime);
void showTemp(int showTime);
void putShiftRegister(int li, int re);
void funLena(void);
void pwmInit(void);
void pwmStop(void);
//////////////////////////////////////////////////////
// Setup fuer die LED und Serielle Ausgabe
void setup() {
  Serial.begin(9600);
  pinMode(LED_BUILTIN, OUTPUT); //LED_BUILTIN ist keyword fuerr die LED auf dem Board, bei Arduino an Pin 13
  digitalWrite(LED_BUILTIN, LOW);
  pinMode(11, OUTPUT); //PWM Kanal 1
  pinMode(3, OUTPUT); //PWM Kanal 2
  pwmStop();
  pwmInit();

  pinMode(2, OUTPUT); //OneWire BUS - DS18B20
  DS18B20.begin();

  pinMode(latchPin, OUTPUT);
  pinMode(dataPin, OUTPUT);  
  pinMode(clockPin, OUTPUT);
  pinMode(enablePin, OUTPUT);
  digitalWrite(enablePin, LOW); //LOW active!

}

//////////////////////////////////////////////////////
// the loop function runs over and over again forever
void loop() {
  //serialMsg(msgInterval);
  
  checkTemp(checkTempInt);
  if (millis() - showFunMillis > showFunInt) {
    showFunMillis = millis();
    funLena();
  }
  else {
    showTemp(showTempInt);
  }
}
//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
/*
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
*/

void putShiftRegister(int li, int re) {
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin, clockPin, MSBFIRST, li);
  shiftOut(dataPin, clockPin, MSBFIRST, re);
  digitalWrite(latchPin, HIGH);
}

void funLena(void) {
  putShiftRegister(ciphArr[0],ciphArr[0]);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[16]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(enablePin, LOW);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[13]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[17]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[11]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[0]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[0]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[tempZehner]);
  digitalWrite(latchPin, HIGH);
  delay(scrollTime);
  digitalWrite(latchPin, LOW);
  shiftOut(dataPin,clockPin,MSBFIRST,ciphArr[tempEiner]);
  digitalWrite(latchPin, HIGH);
  //delay(200);
}

void showTemp(int showTime) {
  static unsigned long checkMillis; 
  if (millis() - checkMillis > showTime) {
    checkMillis = millis();
    if (tempEiner == 0 && tempZehner >= 1) {
        tempEiner = 10;
    }
    putShiftRegister(ciphArr[tempZehner],ciphArr[tempEiner]);
  }
}

void checkTemp(int checkTime) {
  static unsigned long checkMillis; 
  if (millis() - checkMillis > checkTime) {
    checkMillis = millis();
    DS18B20.requestTemperatures(); // Send the command to get temperatures
    float tempC = DS18B20.getTempCByIndex(0);
    Serial.print("tempC = ");
    Serial.println(tempC);
    int ziffern = tempC - 1.2;
    tempEiner = ziffern % 10;
    tempZehner = ziffern / 10;
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

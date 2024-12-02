
//calibration factor will be the (reading)/(known weight)

//Libraries
#include <HX711.h>//https://github.com/bogde/HX711
//Parameters
//const long LOADCELL_OFFSET = 340884;
//const float LOADCELL_DIVIDER = 19218.1363904;
const int numberOfReadings = 10;
const int doutOPin = 36;
const int sckOPin = 37; 

const int doutYPin = 26;
const int sckYPin = 27;

const int doutPPin = 33;
const int sckPPin = 34;

const int doutGPin = 28;
const int sckGPin = 29;

const float inputWeight = 3.716; //x kg, m=(y-b)/x

//Variables
HX711 scale;
float weight = 0;

void setup() {
 //Init Serial USB
 Serial.begin(9600);
 
 Serial.println(F("Initialize System"));


  scale.begin(doutOPin, sckOPin);
// scale.begin(doutYPin, sckYPin);
// scale.begin(doutPPin, sckPPin);
// scale.begin(doutGPin, sckGPin);
 scale.tare();

//  long bO = scaleO.get_units(20);
// long bY = scaleY.get_units(20);
// long bP = scaleP.get_units(20);
// long bG = scaleG.get_units(20);
  long b = scale.get_units(20);


 Serial.println("Tare done. Enter 1 when weight is on scale");

 while (Serial.available() == 0) {
  }
  long y = scale.get_units(20);
  long m = (y-b)/inputWeight;
  scale.set_scale(m); 
  Serial.print("m: ");
  Serial.println(m);
  
  scale.set_offset(b);
  Serial.print("b: ");
  Serial.println(b);
}

void loop() {
 readScale(scale,"calibration"); //prints in kg because of input to m in kg
}

float readScale(HX711 scale, const char loadcell) { /* function readScale */
 float weight;
 //// Read button states from keypad
 if (scale.is_ready()) {
   weight = scale.get_units(numberOfReadings);
//   Serial.print(loadcell);
   Serial.print(" weight: ");
   Serial.println(weight);
   delay(100);
   return weight;
 } else {
//   Serial.println("HX711 not found.");
 }
 }

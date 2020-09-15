// OUTPUT PINS
// Solenoid output pins

// Color placeholder
// Orange / Orange-White = 12V key on (fused)
// Brown / Brown-White = 5V key on
// Blue = Speedo control voltage
// Blue-White = NA
// Green = Fuel pump control ground
// Green-White -> blue-white = TPS signal

#define TEENSY // we're running this on TEENSY


#define y3 36 // FMT3, orange<->brown/red // DOUT3, I need to use 16 as testing instead of 36.
#define y4 35 // orange <-> brown/grey // DOUT2
#define y5 8 //ex 14, orange <-> brown/black // DOUT1
#define mpc 38 // red <-> brown/pink // DOUT5 // FMT3
#define spc 37 // red <-> brown/yellow // DOUT4 // FMT3
#define tcc 29 // pink <-> brown/yellow/white // DOUT6
#define speedoCtrl 53 // default 7 // ex 7, blue <-> blue/green // DOUT7, 
#define rpmMeter 54 // default 30 // FMT2 missing // DOUT10
#define boostCtrl 6 // FMT1, green <-> green/white/yellow // DOUT8, 3?
#define fuelPumpCtrl 12 // missing // DOUT9, 13?
#define hornPin 30 // Horn // default 2
#define reversePin 7 // Reverse

// END OUTPUT PINS
#define injectionPin 36 // should be 16.


// INPUT PINS
// Stick input
#define whitepin 27 // 0.5kohm <-> yellow <-> grey-yellow-grey // DIN2 <-> blue
#define bluepin 34 // 0.5kohm <-> yellow <-> grey-green-grey // DIN4 <-> green
#define greenpin 26 // 0.5kohm <-> yellow <-> grey-white-grey // DIN1 <-> whiteblue
#define yellowpin 28     // 0.5kohm <-> yellow <-> grey-black-grey // DIN3 <-> whiteorange

// Switches
#define autoSwitch 33 // ex. 22, 0.5kohm <-> yellow <-> grey-pink-grey // DIN5

// KeyPad
#define keypadPin  27

#define gdownSwitch 23 // ex. 23 <-> NC // DIN6
#define gupSwitch 24 // <-> NC // DIN7
#define gupSwitchalt 31
#define fuelInPin 32

#define exhaustPresPin A12 // A12, A9 used in coupe.
#define exhaustTempPin 12

// Car sensor input pins, black
#define tpsPin A2 // voltage div 5/3 <-> black <-> blue-black-blue = 1kohm/1.8kohm div // ANAIN3, boost? A3->A11
#define atfPin A1 // voltage div 5/3 <-> black <-> pink = 1kohm/1.8kohm div // ANAIN2
#define boostPin A11 // voltage div 5/3 <-> black <-> blue-brown-blue = 1kohm/1.8kohm div // ANAIN4, tps? A2->A10
#define oilPin A0 // voltage div 12/3 <-> black <-> white-pink-white = 1kohm/380ohm div // ANAIN1
#define n2pin 18 // voltage div 5/3 <-> black <-> whiteredwhite = 1kohm/1.8kohm div // DIN14
#define n3pin 19 // voltage div 5/3 <-> black <-> brownredwhite = 1kohm/1.8kohm div // DIN15
#define speedPin 20 // voltage div 12/3 <-> black <-> blueyellowblue = 1kohm/380ohm div // DIN10
#define rpmPin 22 // voltage div 12/3 <-> black <-> whitebluewhite = 1kohm/380ohm div // DIN12
#define batteryPin A21 // car battery monitor
#define refPin A22 // sensor voltage ref ?
//#define refPin A13 // using this in exhaustTempPin for now.

#define exhaustTemperatureCS 9
#define displayCS 10
///
/// Program for sending E30 Signals to RealDash ///
/// Define global variables 
unsigned long Time;
byte LastBroadcast;

// Define global variables for Pins; r stands for read; bool is true or false (on/off in our case)
bool LBLr; // Left Blinker from Digital in on Arduino
bool RBLr; // Right Blinker from Digital in on Arduino
bool HBr; // High Beams from Digital in on Arduino
bool MALFr; // Check Engine Light from Digital in on Arduino
bool ParkBkr; // Park Brake From Digital in
bool LowFuelr; //Low Fuel from Digital in
bool BrkRearr; //Brake Lining Rear from Digital in
bool Fogr; //Fog Light from Digital in
bool BrkFrontr; // Brake Lining Front from Digtial in
bool RunLghtsr; //Running Lights from Digital in
bool AltFailr; //Alternator Fail from Digital in
bool BrakeFldr; //Brake Fluid Low from Digital in
bool OilPressr; //Oil Pressure Low from Analog

// Assign numbers/Pins to variables 
int ParkBkPin = 2; // Parking Brake on Pin D2
int LBLPin = 4; //Left Blinker on Pin D4
int HBPin = 5; // High Beams on Pin D5
int RBLPin = 6; // Right Blinker on Pin D6
int BrkRearPin = 7; // Brake Lining Rear on D7
int FogPin = 8; // Fog lights on D8
int BrkFrontPin = 9; // Brake Lining Front D9
int MALFPin = 10; // Check Engine (MALF) on Pin D10
int RunLghtsPin = 11; // Running Lights on pin D11
int AltFailPin = 12; // Alternator Fail on D12
int BrakeFldPin = 13; // Brake Fluid Low on D13
int OilPressPin = A0; // Low Oil Pressure on A0
int LowFuelPin = A2; // Low Fuel on Pin A2
int FuelLevelPin = A6; //Fuel level sensor on pin A6
int TempPin = A7; // Temp Sensor on pin A7
int FuelLevel; //define this as an integer
int TempVal;


//all of this is for speed calc 
int ReedSwitch = 3;  //Declaring pin for reed switch to be used for Speed Calculation. Digital Pin 2 or 3 must be used for the reed switch on the Nano
float start;
float elapsed;
const float circumference = 1.9812; //circumference of tire in meters 
float circMetric=(circumference); 
float circImperial; // using 1 kilometer = 0.621371192 miles
volatile byte Speed; // Keeping this as byte so it's smaller and transmitts every interrupt (not sure if required)
volatile byte Speed1;

//Fuel
const int numReadings = 500; //Size of array, larger the number the larger the array and the smoother the average value.
int readIndex = 0;
int readings[numReadings];
long total = 0;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  delay (3000); //3s startup delay 

//Speed Calc Stuff
  start= micros();
  circImperial=((circMetric*.62137)/8); // convert metric to imperial for MPH calculations. 1 kilometer = 0.621371192 miles. Divide by 8 beause 8 pulses = 1 revolution (i thought 9 before)
  
  Serial.begin(9600); //enable serial connection. Needs to be 9600 for bluetooth.
 // Serial.begin(115200); // 115200 for direct connection to comptuer 

  pinMode(LBLPin, INPUT_PULLUP);      // Left Blinker will activate with ground to pin
  pinMode(RBLPin, INPUT_PULLUP);      // Right Blinker will activate with ground to pin
  pinMode(HBPin, INPUT_PULLUP);       // High Beams will activate with ground to pin
  pinMode(MALFPin, INPUT_PULLUP);     // Check engine light will activate when pin is grounded
  pinMode(ParkBkPin, INPUT_PULLUP);   // Park Brake will activate when pin is grounded
  pinMode(LowFuelPin, INPUT_PULLUP);  // Low Fuel will activate when pin is grounded
  pinMode(BrkRearPin, INPUT_PULLUP);  // Activate when pin is grounded
  pinMode(FogPin, INPUT_PULLUP);      // Activate on ground to pin
  pinMode(BrkFrontPin, INPUT_PULLUP); // Activate when pin is grounded
  pinMode(RunLghtsPin, INPUT_PULLUP); // Activate on ground to pin
  pinMode(AltFailPin, INPUT_PULLUP);  // Activate when pin is grounded
  pinMode(BrakeFldPin, INPUT_PULLUP); // Activate when pin is grounded
  //pinMode(OilPressPin, INPUT_PULLUP); // Activate when pin is grounded
   
  attachInterrupt(digitalPinToInterrupt(ReedSwitch), CalculateSpeed, FALLING); //Reedswitch(D3) is detecting a change in voltage(FALLING) and running CalculateSpeed() at each interrupt. Inputpullup not needed before here based on the way I wired it (i think)
  
  //Fuel
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0; }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CalculateSpeed() //ISR for interrupt. (Run this each time the interrupt is called.)
{ 
    //calculate elapsed
      elapsed=micros()-start;

    //reset start
      start=micros();
  
        //calculate speed in mph
      Speed1=((3600*circImperial)/(elapsed))*1000; //*1000 because we are counting mircoseconds, need to convert to milliseconds. This comes out somewhat close to actual MPH. Value can to be changed in Realdash via gauge math compared to a known speed.      
      if (Speed1 >= 2){
        Speed = Speed1;
      }                          //if statements so MPH = 0 if slower than 2MPH
      if (Speed1 <= 2){
        Speed = 0;
      }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop() //Main functions that run continuously
{
Time = millis();

// Go read the ardiuno Pins
  ReadIOPins(); 
  
//Run the Calculate Fuel Level code
  CalculateFuelLevel();
  
//Check Temperature
  Temperature();
   
// Send Data to CanBus every 10 milliseconds: 23 ms orginally, not sure if 10ms is required for anything. Speed calc maybe? 
  if (Time - LastBroadcast > 23) 
    {
      LastBroadcast = millis();
      BuildCanFrames();
    }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void ReadIOPins() // Reads current status of each pin; r stands for read
{
  LBLr = !digitalRead(LBLPin);
  RBLr = !digitalRead(RBLPin);
  HBr = !digitalRead(HBPin);
  MALFr = !digitalRead(MALFPin);
  ParkBkr = !digitalRead(ParkBkPin);
  LowFuelr = !digitalRead(LowFuelPin);
  BrkRearr = !digitalRead(BrkRearPin);
  Fogr = !digitalRead(FogPin);
  BrkFrontr = !digitalRead(BrkFrontPin);
  RunLghtsr = !digitalRead(RunLghtsPin);
  AltFailr = !digitalRead(AltFailPin);
  BrakeFldr = !digitalRead(BrakeFldPin);
  OilPressr = !analogRead(OilPressPin);
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void CalculateFuelLevel()  //Math to calculate fuel level. 
{
  total -= readings[readIndex];
  total += analogRead(FuelLevelPin);
  readings[readIndex] = analogRead(FuelLevelPin);
  readIndex = (readIndex + 1) % numReadings;
  FuelLevel = 100 - (total/numReadings); //subtract from 100 because 100 = empty tank, 0 = full tank
  
 if (FuelLevel < 0)
 {                     // Apply rule if tank is reading less than empty, Prevents negative values.
    FuelLevel = 0;
 }
if (FuelLevel > 100)   // When the sensor bottoms out in the tank, the value shoots to +250, this should prevent that. 
 { 
   FuelLevel = 0; 
 }
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Temperature()
{
  TempVal = analogRead(TempPin);            //Read the temperature pin (A7)
  float mv = (TempVal/1024.0)*5000;         //Scale value
  float celcius = mv/10;                    //Convert to C
  float Temperature = (celcius*9)/5 + 32;   //Convert to F
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BuildCanFrames()
{
  byte CanBuffer[8];

  // build & send CAN frames to RealDash.
  // a CAN frame payload is always 8 bytes (0-7) containing data in a manner. 
  // described by the RealDash custom channel description XML file
  // all multibyte values are handled as little endian by default.
  // endianess of the values can be specified in XML file if it is required to use big endian values

  // build 1st CAN frame:              //This is what is transmitted to Realdash 
  CanBuffer[0] = HBr;
  CanBuffer[1] = MALFr;
  CanBuffer[2] = LBLr;
  CanBuffer[3] = RBLr;
  CanBuffer[4] = ParkBkr;
  CanBuffer[5] = LowFuelr;
  CanBuffer[6] = BrkRearr;
  CanBuffer[7] = Fogr;

  // write first CAN frame to serial:
  SendCANFrameToSerial(3200, CanBuffer); //1st CAN frame id is 3200 

  // build 2nd CAN frame:
  CanBuffer[0] = BrkFrontr;
  CanBuffer[1] = RunLghtsr;
  CanBuffer[2] = AltFailr;
  CanBuffer[3] = BrakeFldr;
  CanBuffer[4] = OilPressr;
  CanBuffer[5] = FuelLevel;
  CanBuffer[6] = Speed;
  CanBuffer[7] = Temperature;

  // write second CAN frame to serial:
  SendCANFrameToSerial(3201, CanBuffer); //2nd CAN frame id is 3201, 3rd would be 3202, etc. 
}

// -----------------------------------

// I don't understand anything after this but it works 
void SendCANFrameToSerial(unsigned long canFrameId, const byte* frameData)
{
  // the 4 byte identifier at the beginning of each CAN frame
  // this is required for RealDash to 'catch-up' on ongoing stream of CAN frames
  const byte serialBlockTag[4] = { 0x44, 0x33, 0x22, 0x11 };
  Serial.write(serialBlockTag, 4);

  // the CAN frame id number (as 32bit little endian value)
  Serial.write((const byte*)&canFrameId, 4);

  // CAN frame payload
  Serial.write(frameData, 8);
}

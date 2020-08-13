/* This example shows to to read all channels from the AS7341 and print out reported values, but allow loop() to run while waiting for the readings */
#include <Adafruit_AS7341.h>

Adafruit_AS7341 as7341;
//TwoWire myWire2(2);
uint16_t readings[12];

void setup() 
{
  Serial.begin(115200);

  // Wait for communication with the host computer serial monitor
  while (!Serial) 
  {
    Serial.println("Failed, looping forever");
    delay(1000);  
  }

//  myWire2.begin();
  Wire.begin();
  delay(20);
  
//  if (!as7341.begin(0x39, &myWire2)){ //Use TwoWire I2C port
  if (!as7341.begin()){ //Default address and I2C port
    Serial.println("Could not find AS7341");
    while (1) { delay(10); }
  }
  
  as7341.setATIME(35);
  as7341.setASTEP(10000); //This combination of ATIME and ASTEP gives an integration time of about 1sec, so with two integrations, that's 2 seconds for a complete set of readings
  as7341.setGain(AS7341_GAIN_256X);
  
  as7341.startReading();
}

void loop() 
{
  bool timeOutFlag = yourTimeOutCheck();
  if(as7341.checkReadingProgress() || timeOutFlag )
  {
    if(timeOutFlag)
    {} //Recover/restart/retc.
    
    Serial.println("\nAha, the reading we started a few cycles back is finished, here it is:");
    //IMPORTANT: make sure readings is a uint16_t array of size 12, otherwise strange things may happen
    as7341.getAllChannels(readings);  //Calling this any other time may give you old data
    printReadings();

    Serial.println("\nLet's try a reading using readAllChannels (inbuilt delay), waiting...");
    if (!as7341.readAllChannels(readings))
      Serial.println("Error reading all channels!");
    else
      printReadings();
    Serial.println("Guess we'll start another reading right away but do some work in the meantime\n");
    as7341.startReading();
  }

  //Do some work in the loop
  Serial.println("Working hard");
  delay(500); //Hardly working
}

bool yourTimeOutCheck()
{
  //Fill this in to prevent the possibility of getting stuck forever if you missed the result, or whatever
  return false;
}

void printReadings()
{
  Serial.print("ADC0/F1 415nm : ");
  Serial.println(readings[0]);
  Serial.print("ADC1/F2 445nm : ");
  Serial.println(readings[1]);
  Serial.print("ADC2/F3 480nm : ");
  Serial.println(readings[2]);
  Serial.print("ADC3/F4 515nm : ");
  Serial.println(readings[3]);
  Serial.print("ADC0/F5 555nm : ");

  /* 
  // we skip the first set of duplicate clear/NIR readings
  Serial.print("ADC4/Clear-");
  Serial.println(readings[4]);
  Serial.print("ADC5/NIR-");
  Serial.println(readings[5]);
  */
  
  Serial.println(readings[6]);
  Serial.print("ADC1/F6 590nm : ");
  Serial.println(readings[7]);
  Serial.print("ADC2/F7 630nm : ");
  Serial.println(readings[8]);
  Serial.print("ADC3/F8 680nm : ");
  Serial.println(readings[9]);
  Serial.print("ADC4/Clear    : ");
  Serial.println(readings[10]);
  Serial.print("ADC5/NIR      : ");
  Serial.println(readings[11]);
}

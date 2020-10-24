/* This example shows how to set up custom SMUX configurations for the AS7341 and take measurements with them
Written by Stephen Fordyce, 24/10/2020
*/

#include <Adafruit_AS7341.h>

Adafruit_AS7341 as7341;
//TwoWire myWire2(2);
uint16_t readings[12];
#define STANDARD 1
#define GENERIC 2

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
}

void loop() 
{
  Serial.println("\n\n******Start of tests*****");
  Serial.println("####Test #1: no option defaults to 2 integrations fixed low and fixed high channels");
  as7341.startReading();
  waitGetPrintReadings(STANDARD);


  Serial.println("\n\n####Test #2: fixed LOW channels only (1 integration only)");
  as7341.startReading(AS7341_CH_LOW);
  waitGetPrintReadings(STANDARD);

  Serial.println("\n####Test #2A: same channels, but setup using custom SMUX (1 integration only)");
  makeCustomSMUXLowConfig();
  as7341.startReading(AS7341_CUSTOM_SMUX);
  waitGetPrintReadings(GENERIC);

  Serial.println("\n####Test #2B: same channels, but setup using custom SMUX and config words (1 integration only)");
  makeConfigWordCustomSMUXLowConfig();
  as7341.startReading(AS7341_CUSTOM_SMUX);
  waitGetPrintReadings(GENERIC);
  
  Serial.println();
  Serial.println("For the 'Low' configuration, the SMUX was set (by different ways), with corresponding values:");
  as7341.printLocalSMUXConfig();
  Serial.println();
  printConfigWords();
  Serial.println("****Note that the 23 bits used in the config word correspond to (arbitrary) indexes, NOT pixels****");


  Serial.println("\n\n####Test #3: fixed HIGH channels only");
  as7341.startReading(AS7341_CH_HIGH);
  waitGetPrintReadings(STANDARD);

  Serial.println("\n####Test #3A: same channels, but setup using custom SMUX (1 integration only)");
  makeCustomSMUXHighConfig();
  as7341.startReading(AS7341_CUSTOM_SMUX);
  waitGetPrintReadings(GENERIC);

  Serial.println("\n####Test #3B: same channels, but setup using custom SMUX and config words (1 integration only)");
  makeConfigWordCustomSMUXHighConfig();
  as7341.startReading(AS7341_CUSTOM_SMUX);
  waitGetPrintReadings(GENERIC);

  Serial.println();
  Serial.println("For the 'High' configuration, the SMUX was set (by different ways), with corresponding values:");
  as7341.printLocalSMUXConfig();
  Serial.println();
  printConfigWords();
  Serial.println("****Note that the 23 bits used in the config word correspond to (arbitrary) indexes, NOT pixels****");


  Serial.println("\nEnd of tests (looping forever)");
  while(1)
    delay(50);
}  

void makeCustomSMUXLowConfig()
{  
  as7341.clearSMUXPixelRegistersLocal();  //Start with a fresh canvas
  as7341.updateSMUXEntryLocal(F1_LEFT, ADC0); //Connect photodiodes to ADC channels
  as7341.updateSMUXEntryLocal(F1_RIGHT, ADC0);
  as7341.updateSMUXEntryLocal(F2_LEFT, ADC1);
  as7341.updateSMUXEntryLocal(F2_RIGHT, ADC1);
  as7341.updateSMUXEntryLocal(F3_LEFT, ADC2);
  as7341.updateSMUXEntryLocal(F3_RIGHT, ADC2);
  as7341.updateSMUXEntryLocal(F4_LEFT, ADC3);
  as7341.updateSMUXEntryLocal(F4_RIGHT, ADC3);
  as7341.updateSMUXEntryLocal(FX_CLEAR_LEFT, ADC4);
  as7341.updateSMUXEntryLocal(FX_CLEAR_RIGHT, ADC4);
  as7341.updateSMUXEntryLocal(FX_NIR, ADC5);
  //No need to write to the device, this is taken care of when starting integration
}

void makeCustomSMUXHighConfig()
{  
  as7341.clearSMUXPixelRegistersLocal();  //Start with a fresh canvas
  as7341.updateSMUXEntryLocal(F5_LEFT, ADC0); //Connect photodiodes to ADC channels
  as7341.updateSMUXEntryLocal(F5_RIGHT, ADC0);
  as7341.updateSMUXEntryLocal(F6_LEFT, ADC1);
  as7341.updateSMUXEntryLocal(F6_RIGHT, ADC1);
  as7341.updateSMUXEntryLocal(F7_LEFT, ADC2);
  as7341.updateSMUXEntryLocal(F7_RIGHT, ADC2);
  as7341.updateSMUXEntryLocal(F8_LEFT, ADC3);
  as7341.updateSMUXEntryLocal(F8_RIGHT, ADC3);
  as7341.updateSMUXEntryLocal(FX_CLEAR_LEFT, ADC4);
  as7341.updateSMUXEntryLocal(FX_CLEAR_RIGHT, ADC4);
  as7341.updateSMUXEntryLocal(FX_NIR, ADC5);
  
  //No need to write to the device, this is taken care of when starting integration
}

void makeConfigWordCustomSMUXLowConfig()
{  
  //NOTE: must use "ADC0" macro instead of "0", they are different! 
  as7341.setSMUXADCFromConfigWord(ADC0, 0); //Set ADC0 to have no photodiodes
  as7341.setSMUXADCFromConfigWord(ADC1, CWORD_NONE);  //Set ADC1 to no photo diodes using pre-defined macro
  as7341.clearSMUXPixelRegistersLocal();  //Just blank all the ADCs

  //Set ADCx photodiodes according to config words generated from as7341.getSMUXConfigWordForADC(ADC0);
  as7341.setSMUXADCFromConfigWord(ADC0, 131074); //Same as CWORD_BOTH_F1
  as7341.setSMUXADCFromConfigWord(ADC1, 2080);  //Same as CWORD_BOTH_F2
  
  //Set ADCx photodiodes using pre-defined word macros
  as7341.setSMUXADCFromConfigWord(ADC2, CWORD_BOTH_F3); 
  as7341.setSMUXADCFromConfigWord(ADC3, CWORD_BOTH_F4);
  as7341.setSMUXADCFromConfigWord(ADC4, CWORD_BOTH_CLEAR);
  as7341.setSMUXADCFromConfigWord(ADC5, CWORD_NIR); 
  
  //No need to write to the device, this is taken care of when starting integration
}

void makeConfigWordCustomSMUXHighConfig()
{  
  //NOTE: must use "ADC0" macro instead of "0", they are different!
  as7341.setSMUXADCFromConfigWord(ADC0, 0); //Set ADC0 to have no photodiodes
  as7341.setSMUXADCFromConfigWord(ADC1, CWORD_NONE);  //Set ADC1 to no photo diodes using pre-defined macro
  as7341.clearSMUXPixelRegistersLocal();  //Just blank all the ADCs

  //Set ADCx photodiodes according to config words generated from as7341.getSMUXConfigWordForADC(ADC0);
  as7341.setSMUXADCFromConfigWord(ADC0, 576); //Same as CWORD_BOTH_F5
  as7341.setSMUXADCFromConfigWord(ADC1, 8200);  //Same as CWORD_BOTH_F6
  
  //Set ADCx photodiodes using pre-defined word macros
  as7341.setSMUXADCFromConfigWord(ADC2, CWORD_BOTH_F7); 
  as7341.setSMUXADCFromConfigWord(ADC3, CWORD_BOTH_F8);
  as7341.setSMUXADCFromConfigWord(ADC4, CWORD_BOTH_CLEAR);
  as7341.setSMUXADCFromConfigWord(ADC5, CWORD_NIR); 
  //No need to write to the device, this is taken care of when starting integration
}

void printConfigWords()
{  
  Serial.println("Generated SMUX Config words (uin32_t):");
  int thisADC = 0;
  uint32_t thisConfigWord = 0;
  for(int i=0; i<6; i++)
  {  
    switch(i)
    {
      case(0): {thisADC = ADC0; break;}
      case(1): {thisADC = ADC1; break;}
      case(2): {thisADC = ADC2; break;}
      case(3): {thisADC = ADC3; break;}
      case(4): {thisADC = ADC4; break;}
      case(5): {thisADC = ADC5; break;}
      default: {thisADC = ADC_DISABLED; break;}
    }
    thisConfigWord = as7341.getSMUXConfigWordForADC(thisADC);
    Serial.printf("ADC%d: %lu,\t0x%08X,\t0b", i, thisConfigWord, thisConfigWord);  
    as7341.printBits(thisConfigWord,32);  //Note this helper function will print anything that's fed to it, even though it belongs to the object
    Serial.println();
  }
}

void waitGetPrintReadings(int option)
{
  bool timeOutFlag = false;
  while(!as7341.checkReadingProgress() && !timeOutFlag ) //Wait for reading
  {
    timeOutFlag = yourTimeOutCheck();
    if(timeOutFlag)
    {} //Recover/restart/retc.
    delay(50);
  }    
  Serial.println("Here are the readings:");
  //IMPORTANT: make sure readings is a uint16_t array of size 12, otherwise strange things may happen
  as7341.getAllChannels(readings);  //Calling this any other time may give you old data
  if(option == STANDARD)
    printStandardReadings();
  if(option == GENERIC)
    printGenericReadings();
}

bool yourTimeOutCheck()
{
  //Fill this in to prevent the possibility of getting stuck forever if you missed the result, or whatever
  return false;
}

void printStandardReadings()
{
  Serial.print("Int#1, ADC0 (F1/415nm) : ");
  Serial.println(readings[0]);
  Serial.print("Int#1, ADC1 (F2/445nm) : ");
  Serial.println(readings[1]);
  Serial.print("Int#1, ADC2 (F3 480nm) : ");
  Serial.println(readings[2]);
  Serial.print("Int#1, ADC3 (F4 515nm) : ");
  Serial.println(readings[3]);
  // Consider skipping the first set of duplicate clear/NIR readings
  Serial.print("Int#1, ADC4 (Clear)    : ");
  Serial.println(readings[4]);
  Serial.print("Int#1, ADC5 (NIR)      : ");
  Serial.println(readings[5]);

  Serial.print("Int#2, ADC0 (F5 555nm) : ");
  Serial.println(readings[6]);
  Serial.print("Int#2, ADC1 (F6 590nm) : ");
  Serial.println(readings[7]);
  Serial.print("Int#2, ADC2 (F7 630nm) : ");
  Serial.println(readings[8]);
  Serial.print("Int#2, ADC3 (F8 680nm) : ");
  Serial.println(readings[9]);
  Serial.print("Int#2, ADC4 (Clear)    : ");
  Serial.println(readings[10]);
  Serial.print("Int#2, ADC5 (NIR)      : ");
  Serial.println(readings[11]);
}

void printGenericReadings()
{
  Serial.print("Int#1, ADC0 : ");
  Serial.println(readings[0]);
  Serial.print("Int#1, ADC1 : ");
  Serial.println(readings[1]);
  Serial.print("Int#1, ADC2 : ");
  Serial.println(readings[2]);
  Serial.print("Int#1, ADC3 : ");
  Serial.println(readings[3]);
  Serial.print("Int#1, ADC4 : ");
  Serial.println(readings[4]);
  Serial.print("Int#1, ADC5 : ");
  Serial.println(readings[5]);

  Serial.print("Int#2, ADC0 : ");
  Serial.println(readings[6]);
  Serial.print("Int#2, ADC1 : ");
  Serial.println(readings[7]);
  Serial.print("Int#2, ADC2 : ");
  Serial.println(readings[8]);
  Serial.print("Int#2, ADC3 : ");
  Serial.println(readings[9]);
  Serial.print("Int#2, ADC4 : ");
  Serial.println(readings[10]);
  Serial.print("Int#2, ADC5 : ");
  Serial.println(readings[11]);
}

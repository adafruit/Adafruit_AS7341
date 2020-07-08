
  /*

  The AS7341 device features three modes to perform a spectral measurement. The integration mode (INT_MODE) can be configured in register 0x70 (CONFIG)
  
  This Script shows the implementation of SYND Mode INT_MODE = 0x03
  
  Spectral measurement is based on Integration with external start and stop sync signal
  Integration is controlled via rising/falling edge on pin GPIO and register EDGE. If the number of edges on pin GPIO is reached, integration time is stopped. Actual integration time can be read out in register “ITIME”.

  
  Written by Sijo John @ ams AG, Application Support in November, 2018
  Development environment specifics: Arduino IDE 1.8.5

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
  */
  
  #include <Wire.h>
  // I2C device address - 0x39
  #define _i2cAddr (0x39)
  #define GPIO_OUT_PIN 5
  #define SYND_EDGES 10
  #define INT_PIN 6
  #define LOW_PULSE_DELAY 15
    void setup() 
    {

      // Initiate the Wire library and join the I2C bus as a master or slave
      Wire.begin();

      // communication with the host computer serial monitor
      Serial.begin(115200);
      while(!Serial){delay(1);}
      digitalWrite(GPIO_OUT_PIN, HIGH);
      pinMode(GPIO_OUT_PIN, OUTPUT);

      
    }

  
        void loop() 
        {

        //Function defined to read out channels with with SYND MODE and SMUX configration 1 for pixels - F1-F4, Clear, NIR   
        synD_Mode();
        // Sets the Spectral Gain in CFG1 Register (0xAA) in [4:0] bit
        setGAIN(byte (0x08));
        delay(100);
        }


        //<summary>
        // Function executes to read out channels with SYND MODE and SMUX configration 1 for pixels - F1-F4, Clear, NIR
        // the integration time is defined by the number of rising/falling edge between start and stop on pin GPIO Signal, which is set to register EDGE (0x72)
        //<summary>       
         void synD_Mode()
          {

            bool isEnabled = true;
            bool isDataReady = false;

            // Setting the PON bit in Enable register 0x80     
            PON();

                
            // Disable SP_EN bit in Enable register 0x80

            SpEn(false);
          
            // Write SMUX configuration from RAM to set SMUX chain registers (Write 0x10 to CFG6)
            SmuxConfigRAM();  

            
            // Write new configuration to all the 20 registers for reading channels from F5-F8, Clear and NIR         
            F1F4_Clear_NIR();
            
            
            
            // Start SMUX command: Enable the SMUXEN bit (bit 4) in register ENABLE
            SMUXEN();


            

            // Checking on the enabled SMUXEN bit whether back to zero- Poll the SMUXEN bit -> if it is 0 SMUX command is started           
            while(isEnabled)
            {
              isEnabled = getSmuxEnabled();
            }

            //Enabling the gpio_in_en (Bit 2) and gpio_out(Bit 1) in GPIO register 0xBE
            GPIO_MODE();
           

            
            
            //reg_bank bit(4) is set to '1' for setting the 0x00-0x7f regiater to reg_bank register (0xA9)
            RegBankConfig();
            
            

            // CONFIG (0x70) is used to set the INT_MODE (Bit 1:0) to SYND Mpde by  writing 0x03
            INT_MODE(0x03); 

            //Number of falling SYNC-edges between start and stop integration in SynD mode is set to register EDGE (0x72)
            
            Serial.print("syn SynEdge to "); Serial.println(SYND_EDGES);
            setSynEdge(SYND_EDGES); //4 edges

           // writing back the Reg_bank priorty to RAM bank select
            writeRegister(byte (0xA9), byte (0x00));    
                  
            SpEn(true);

                    
            uint16_t low_pulse_count = 0;
            // Reading and Polling the the AVALID bit in Status 2 Register 0xA3           
            while(!(isDataReady))
             {
            
              isDataReady = getIsDataReady();
              Serial.print("Pulsing low w/ delay ");Serial.println(LOW_PULSE_DELAY);
              digitalWrite(GPIO_OUT_PIN, LOW);
              delay(LOW_PULSE_DELAY);
              digitalWrite(GPIO_OUT_PIN, HIGH);
              low_pulse_count += 1;

             
             }
             Serial.println("Low pulses: "); Serial.println(low_pulse_count);
             // analog and digital saturation are read in Status2
             // Digital saturation - Indicates that the maximum counter value has been reached. Maximum counter value depends on integration time set in the ATIME register
             //Analog saturation - Indicates that the intensity of ambient light has exceeded the maximum integration level for the spectral analog circuit 
            Serial.print("Status2-");
            readRegisterPrint(0xA3);
            
            
            //reg_bank bit(4) is set to '1' for setting the 0x00-0x7f regiater to reg_bank register (0xA9)
            RegBankConfig();
           

            //Read all the registers from 0x60 to 0x6F in SYND MODE 
            readSynDRegisters();
         
         
          }
        

/*----- Register configuration  -----*/


       // <summary>  
       // Setting the PON (Power on) bit on the chip (bit0 at register ENABLE 0x80)
       // Attention: This function clears only the PON bit in ENABLE register and keeps the other bits
       // <summary>        
        void PON()
          {
   
            byte regVal = readRegister(byte(0x80));
            byte temp = regVal;
            regVal = regVal & 0xFE;
            regVal = regVal | 0x01;
            writeRegister(byte (0x80), byte (regVal));
    
          }



       // <summary>
       // Setting the SP_EN (spectral measurement enabled) bit on the chip (bit 1 in register ENABLE)
       // <summary>
       // <param name="isEnable">Enabling (true) or disabling (false) the SP_EN bit</param>          
         void SpEn(bool isEnable)
          {
          
             byte regVal = readRegister(byte(0x80));
             byte temp = regVal;
             regVal = regVal & 0xFD;
             
             if(isEnable == true)
               {
                regVal= regVal | 0x02;
               }
             else 
               {
                regVal = temp & 0xFD;
               }
            
             writeRegister(byte (0x80), byte (regVal));
             
         }

        // <summary>  
        // Write SMUX configration from RAM to set SMUX chain in CFG6 register 0xAF
        // <summary>        
          void SmuxConfigRAM()
          {
       
            
            writeRegister(byte (0xAF), byte (0x10));
          
          }


        // <summary>
        // Starting the SMUX command via enabling the SMUXEN bit (bit 4) in register ENABLE 0x80
        // The SMUXEN bit gets cleared automatically as soon as SMUX operation is finished
        // <summary>
         void SMUXEN()
        {

          byte regVal = readRegister(byte(0x80));
          byte temp = regVal;
          regVal = regVal & 0xEF;
          regVal = regVal | 0x10;
          writeRegister(byte (0x80), byte (regVal));
                           
        }


        // <summary>
        // Reading and Polling the the SMUX Enable bit in Enable Register 0x80
        // The SMUXEN bit gets cleared automatically as soon as SMUX operation is finished
        // <summary>
         bool getSmuxEnabled()
          {
          
            bool isEnabled = false;
            byte regVal = readRegister(byte(0x80));
      
            if( (regVal & 0x10) == 0x10)
              {
                return isEnabled = true;      
              }
                
            else
              {
                return isEnabled = false;
              }
              
           }


          // <summary>
          // Reading and Polling the the AVALID bit in Status 2 Register 0xA3,if the spectral measurement is ready or busy.
          // True indicates that a cycle is completed since the last readout of the Raw Data register
          // <summary>
         bool getIsDataReady()
            {
              bool isDataReady = false;
              byte regVal = readRegister(byte(0xA3));
    
              if( (regVal & 0x40) == 0x40){
    
              return isDataReady = true;
              }
              
              else
              {
              return isDataReady = false;
              }
            }

            

         // <summary>
         //Enabling the gpio_in_en (Bit 2) and gpio_out(Bit 1) in GPIO register 0xBE
         // <summary>
        void GPIO_MODE()
        {

          byte regVal = readRegister(byte(0xBE));
          byte temp = regVal;
          regVal = regVal & 0x0B;
          regVal = regVal | 0x06;
          writeRegister(byte (0xBE), byte (regVal));
                           
        }


         // <summary>
         //CONFIG (0x70) is used to set the INT_MODE (Bit 1:0) SYNSis set by writing 0x01 and SYND by  writing 0x03
         // <summary>
         void INT_MODE(byte mode)
         {

          byte regVal = readRegister(byte(0x70));
          byte temp = regVal;
          regVal = regVal & 0xFC;
          regVal = regVal | mode;
          writeRegister(byte (0x70), byte (regVal));
                           
        }



        // <summary>
        // select which Register bank to address in registers 0x00-0x7f has priority over ram_bank 
        //reg_bank bit(4) is set to '1' for setting the 0x00-0x7f regiater to reg_bank register. by default it is RAM register 
        // <summary>
        void RegBankConfig()
        {

          byte regVal = readRegister(byte(0xA9));
          byte temp = regVal;
          regVal = regVal & 0xEF;
          regVal = regVal | 0x10;
          writeRegister(byte (0xA9), byte (regVal));
                           
        }


        
/*----- Set Gain and integration time = iTime * 2.78µS -----*/


        //<summary>
        // Sets the Spectral Gain in CFG1 Register (0xAA) in [4:0] bit
        //<summary>
        // param name = "value"> integer value from 0 to 10 written to AGAIN register 0xAA
        void setGAIN(byte value)
          {
            writeRegister(byte (0xAA), value);
          }

          
         //<summary>
         // Number of falling SYNC-edges between start and stop integration in SynD mode is set to register EDGE (0x72)
         // integration time is determined by the integration_time = (syn_edge+1) * sync period
         //<summary>
          void setSynEdge(byte value)
          {
          writeRegister(byte (0x72), value);
          }



         
/*----- SMUX Configuration for F1,F2,F3,F4,CLEAR,NIR -----*/


        //<summary>
        // Mapping the individual Photo diodes to dedicated ADCs using SMUX Configuration for F1-F4,Clear,NIR
        //<summary>         
        void F1F4_Clear_NIR()
        {
          //SMUX Config for F1,F2,F3,F4,NIR,Clear
          writeRegister(byte (0x00), byte (0x30)); // F3 left set to ADC2
          writeRegister(byte (0x01), byte (0x01)); // F1 left set to ADC0
          writeRegister(byte (0x02), byte (0x00)); // Reserved or disabled
          writeRegister(byte (0x03), byte (0x00)); // F8 left disabled
          writeRegister(byte (0x04), byte (0x00)); // F6 left disabled
          writeRegister(byte (0x05), byte (0x42)); // F4 left connected to ADC3/f2 left connected to ADC1
          writeRegister(byte (0x06), byte (0x00)); // F5 left disbled
          writeRegister(byte (0x07), byte (0x00)); // F7 left disbled
          writeRegister(byte (0x08), byte (0x50)); // CLEAR connected to ADC4
          writeRegister(byte (0x09), byte (0x00)); // F5 right disabled
          writeRegister(byte (0x0A), byte (0x00)); // F7 right disabled
          writeRegister(byte (0x0B), byte (0x00)); // Reserved or disabled
          writeRegister(byte (0x0C), byte (0x20)); // F2 right connected to ADC1
          writeRegister(byte (0x0D), byte (0x04)); // F4 right connected to ADC3
          writeRegister(byte (0x0E), byte (0x00)); // F6/F7 right disabled
          writeRegister(byte (0x0F), byte (0x30)); // F3 right connected to AD2
          writeRegister(byte (0x10), byte (0x01)); // F1 right connected to AD0
          writeRegister(byte (0x11), byte (0x50)); // CLEAR right connected to AD4
          writeRegister(byte (0x12), byte (0x00)); // Reserved or disabled
          writeRegister(byte (0x13), byte (0x06)); // NIR connected to ADC5
        }


/*----- SMUX Configuration for F5,F6,F7,F8,CLEAR,NIR -----*/


        //<summary>
        // Mapping the individual Photo diodes to dedicated ADCs using SMUX Configuration for F5-F8,Clear,NIR
        //<summary>        
        void F5F8_Clear_NIR()
        {
          //SMUX Config for F5,F6,F7,F8,NIR,Clear
          writeRegister(byte (0x00), byte (0x00)); // F3 left disable
          writeRegister(byte (0x01), byte (0x00)); // F1 left disable
          writeRegister(byte (0x02), byte (0x00)); // reserved/disable
          writeRegister(byte (0x03), byte (0x40)); // F8 left connected to ADC3
          writeRegister(byte (0x04), byte (0x02)); // F6 left connected to ADC1
          writeRegister(byte (0x05), byte (0x00)); // F4/ F2 disabled
          writeRegister(byte (0x06), byte (0x10)); // F5 left connected to ADC0
          writeRegister(byte (0x07), byte (0x03)); // F7 left connected to ADC2
          writeRegister(byte (0x08), byte (0x50)); // CLEAR Connected to ADC4
          writeRegister(byte (0x09), byte (0x10)); // F5 right connected to ADC0
          writeRegister(byte (0x0A), byte (0x03)); // F7 right connected to ADC2
          writeRegister(byte (0x0B), byte (0x00)); // Reserved or disabled
          writeRegister(byte (0x0C), byte (0x00)); // F2 right disabled
          writeRegister(byte (0x0D), byte (0x00)); // F4 right disabled
          writeRegister(byte (0x0E), byte (0x24)); // F7 connected to ADC2/ F6 connected to ADC1
          writeRegister(byte (0x0F), byte (0x00)); // F3 right disabled
          writeRegister(byte (0x10), byte (0x00)); // F1 right disabled
          writeRegister(byte (0x11), byte (0x50)); // CLEAR right connected to AD4
          writeRegister(byte (0x12), byte (0x00)); // Reserved or disabled
          writeRegister(byte (0x13), byte (0x06)); // NIR connected to ADC5
        }

        //<summary> 
        // Read all the registers from 0x60 to 0x6F in SYND MODE 
        //<summary> 
          void readSynDRegisters()
            {

            uint8_t Astatus; uint16_t Channel0; uint32_t iTime; uint16_t Channel1; uint16_t Channel2; uint16_t Channel3; uint16_t Channel4; uint16_t Channel5; 
            
            Wire.beginTransmission(_i2cAddr);
            Wire.write(0x60);
            Wire.endTransmission(false);
        
            Wire.requestFrom(_i2cAddr,16, true);

            Astatus = Wire.read(); // Reading register 0x60 for Astatus 
            Serial.print("Astatus-");
            Serial.println(Astatus);

            
              Channel0 = Wire.read()| Wire.read()<<8; // reading register 0x61 XWING_ADATA0L & 0x62 XWING_ADATA0H channel 0 data
              Serial.print("Channel0-");
              Serial.println(Channel0);
              
            uint8_t iTimeL = Wire.read();  // register 0x63 XWING_ITIMEL
            uint16_t iTimeM = Wire.read(); // register 0x64 XWING_ITIMEM 
            uint32_t iTimeH = Wire.read(); // register 0x65 XWING_ITIMEH 
            iTime = (iTimeH << 16 | iTimeM << 8 | iTimeL); // iTime =  ITIME_H(bit 23:16) ITIME_M(15:8) ITIME_L(7:0)
            Serial.print("\tiTime-");
            Serial.println(iTime);

            float Tint = iTime * 2.78 * pow(10,-6);        
            Serial.print("\tTint-");
            Serial.println(Tint);


              
              Channel1 = Wire.read()|Wire.read()<<8;// reading register 0x66 XWING_ADATA1L & 0x67 XWING_ADATA1H channel 1 data
              Serial.print("\tChannel1-");
              Serial.println(Channel1);
              
              Channel2 = Wire.read()|Wire.read()<<8;// reading register 0x68 XWING_ADATA2L & 0x69 XWING_ADATA2H channel 2 data
              Serial.print("\tChannel2-");
              Serial.println(Channel2);

              Channel3 = Wire.read()|Wire.read()<<8;// reading register 0x6A XWING_ADATA3L & 0x6B XWING_ADATA3H channel 3 data
              Serial.print("\tChannel3-");
              Serial.println(Channel3);
              
              Channel4 = Wire.read()|Wire.read()<<8;// reading register 0x6C XWING_ADATA4L & 0x6D XWING_ADATA4H channel 4 data
              Serial.print("\tChannel4-");
              Serial.println(Channel4);
              
              Channel5 = Wire.read()|Wire.read()<<8;// reading register 0x6E XWING_ADATA5L & 0x6F XWING_ADATA5H channel 5 data
              Serial.print("\tChannel5-");
              Serial.println(Channel5);
              Serial.println("");
              delay(500);
                             
                  
          }



          
      /* ----- Read/Write to i2c register ----- */

      
      // <summary>  
      //Read a single i2c register
      // <summary>
      // param name = "addr">Register address of the the register to be read
      // param name = "_i2cAddr">Device address 0x39
      
      byte readRegister(byte addr)
       {
            Wire.beginTransmission(_i2cAddr);
            Wire.write(addr);
            Wire.endTransmission();
              
            Wire.requestFrom(_i2cAddr, 1);
            
            if (Wire.available()) 
                {
                  //Serial.println(Wire.read());
                  return (Wire.read());
                }
       
            else 
               {
                 Serial.println("I2C Error");
                 return (0xFF); //Error
               }
       }

      
      // <summary>  
      //Print out a single i2c register
      // <summary>
      // param name = "addr">Register address of the the register to be read
      // param name = "_i2cAddr">Device address 0x39
      
      void readRegisterPrint(byte addr)
       {
            Wire.beginTransmission(_i2cAddr);
            Wire.write(addr);
            Wire.endTransmission();
              
            Wire.requestFrom(_i2cAddr, 1);
            
            if (Wire.available()) 
                {
                  Serial.println(Wire.read());
                  
                }
       
            else 
               {
                 Serial.println("I2C Error");
                 
               }
       }

       
       // <summary>  
      // Read two consecutive i2c registers
      // <summary>
      // param name = "addr">First register address of two consecutive registers to be read
      // param name = "_i2cAddr">Device address 0x39
         
      uint16_t readTwoRegister1(byte addr)
          {
            uint8_t readingL; uint16_t readingH; uint16_t reading = 0; 
            Wire.beginTransmission(_i2cAddr);
            Wire.write(addr);
            Wire.endTransmission();
        
            Wire.requestFrom(_i2cAddr, 2);
          
            if (2<=Wire.available()) 
              {
              readingL = Wire.read();
              readingH = Wire.read();
              readingH = readingH << 8;
              reading = (readingH | readingL);
              return(reading);
              }
            else 
              {
              Serial.println("I2C Error");
              return (0xFFFF); //Error
              }
          }

        
      // <summary>  
      // Write a value to a single i2c register
      // <summary>
      // param name = "addr">Register address of the the register to the value to be written
      // param name = "val">The value written to the Register
      // param name = "_i2cAddr">Device address 0x39
    
      void writeRegister(byte addr, byte val)
       {
          Wire.beginTransmission(_i2cAddr);
          Wire.write(addr);
          Wire.write(val);
          Wire.endTransmission();
        }
        
   

/*  
 *  Created on: 06/02/2023
 *      Author: Andrea Sauviat
 *      E-mail: a-sauviat@laposte.net
 * 
 *  Description:  Ultra-low power 20 mm x 20 mm asset tracker consisting of :
 *                - CMWX1ZZABZ (SX1276 LoRa radio and STM32L082 host MCU)
 *                - MAX M8Q concurrent GNSS module
 *                - LIS2DW12 accelerometer for wake-on-motion/sleep-on-no-motion functionality.
 *        
 *  Buy Board     : https://www.tindie.com/products/tleracorp/gnat-loragnss-asset-tracker/
 *  Configuration : https://github.com/GrumpyOldPizza/ArduinoCore-stm32l0
 *  Master Code   : https://github.com/kriswiner/CMWX1ZZABZ/tree/master/Gnat
 * 
 */

#include <STM32L0.h>  // Management of the STM32L082CZ
#include "RTC.h"      // Use Real Time Clock features
#include "LIS2DW12.h" // Use accelerometer
#include "GNSS.h"     // Use GNSS
//#include "Flash_Page_L0.h"



/* >>> What to use ? <<< */
#define Use_Acc   false 


/* >>> STM32 <<< */
bool STM32_Sleeping = false ;
float STM32_Temperature_float ; // Updated by the function - void STM32_Temperature( bool Enable_SerialPrint_STM32 )

/* >>> BLUE LED <<< */
#define Blue_LED     10            // Blue led 

/* >>> RTC <<< */
bool RTC_Alarm_Flag = true ;
int RTC_Timer_count = 0 ;

/* >>> Sensor connection <<< */
#define I2C_BUS    Wire               // Define the I2C bus (Wire instance) you wish to use
I2Cdev             i2c_0(&I2C_BUS);   // Instantiate the I2Cdev object and point to the desired I2C bus

/* >>> EEPROM <<< */
uint16_t page_number = 0;     // set the page number for flash page write
uint8_t  sector_number = 0;   // set the sector number for sector write
uint8_t  flashPage[256];      // array to hold the data for flash page write
int EEPROM_address = 0 ;

  uint32_t address = 0x08021980; // adresse de départ de la mémoire flash
  uint8_t data_pushed[128]; // tableau pour stocker les données lues
  uint32_t count = 128; // nombre de données à lire

uint8_t data_pulled[128]; // tableau pour stocker les données lues

uint16_t Page_Number = 1075;     // set the page number for flash page write
uint8_t  Message_Pushed_Count = 0;   // set the sector number for sector write


/* >>> LIS2DW12 - Accelerometer <<< */
#if( Use_Acc == true )
  #define LIS2DW12_intPin1   A4    // interrupt1 pin definitions, wake-up from STANDBY pin
  #define LIS2DW12_intPin2    3    // interrupt2 pin definitions, data ready or sleep interrupt
  // Specify sensor parameters //
  LPMODE   lpMode = LIS2DW12_LP_MODE_1;      // choices are low power modes 1, 2, 3, or 4
  MODE     mode   = LIS2DW12_MODE_LOW_POWER; // choices are low power, high performance, and one shot modes
  ODR      odr    = LIS2DW12_ODR_12_5_1_6HZ; //  1.6 Hz in lpMode, max is 200 Hz in LpMode
  FS       fs     = LIS2DW12_FS_2G;          // choices are 2, 4, 8, or 16 g
  BW_FILT  bw     = LIS2DW12_BW_FILT_ODR2;   // choices are ODR divided by 2, 4, 10, or 20
  FIFOMODE fifoMode = BYPASS;                // capture 32 samples of data before wakeup event, about 2 secs at 25 Hz
  bool lowNoise = false;                     // low noise or lowest power
  float aRes = 0;         // Sensor data scale in mg/LSB
  int16_t accelCount[3];  // Stores the 16-bit signed accelerometer sensor output
  int16_t LIS2DWS12_Temp_Raw;      // temperature raw count output
  float   LIS2DWS12_Temperature;    // Stores the real internal chip temperature in degrees Celsius
  float ax, ay, az;       // variables to hold latest sensor data values 
  float offset[3];        // holds accel bias offsets
  float stress[3];        // holds results of the self test
  uint8_t status = 0, wakeSource = 0, FIFOstatus = 0, numFIFOSamples = 0;
  // Logic flags to keep track of device states
  volatile bool LIS2DW12_wake_flag = false;
  volatile bool LIS2DW12_sleep_flag = false;
  volatile bool InMotion = false;
  LIS2DW12 LIS2DW12(&i2c_0); // instantiate LIS2DW12 class
#endif



/* >>> Serial Println <<< */
bool Enable_SerialPrint_LED = false ;
bool Enable_SerialPrint_STM32 = false ;
bool Enable_SerialPrint_Acc = true ;


/* >>> Functions <<< */

void STM32_WakeUp( bool Enable_SerialPrint_STM32 ) ;
void STM32_StopMode( bool Enable_SerialPrint_STM32 ) ;
void STM32_Temperature( bool Enable_SerialPrint_STM32 ) ;
void STM32_Flash_Write( bool Enable_SerialPrint_STM32 ) ;

void BlueLED_Config( bool Enable_SerialPrint_LED )  ;
void BlueLED_ON( bool Enable_SerialPrint_LED ) ;
void BlueLED_OFF( bool Enable_SerialPrint_LED );

void RTC_Enable( bool Enable_SerialPrint );
void RTC_Disable( bool Enable_SerialPrint );
void RTC_Alarm_Fct_Wakeup() ;

void I2C_Config( ) ;

#if( Use_Acc == true )
  void Acc_Config( bool Enable_SerialPrint_Acc ) ;
  void Acc_Get_XYZ_Data( bool Enable_SerialPrint_Acc ) ;
  void Acc_Get_Temperature( bool Enable_SerialPrint_Acc ) ;
#endif


// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          SETUP()                                                                               //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void setup() {

  STM32L0.wakeup() ;
  
  // put your setup code here, to run once:
  Serial.begin(115200) ; 


  /* >>> BLUE LED <<< */
  BlueLED_Config( Enable_SerialPrint_LED ) ;


  /* >>> RTC <<< */
  RTC_Enable( true ) ;

  I2C_Config( ) ;




  /* >>> LIS2DW12 - Acc <<< */
  #if( Use_Acc == true )
    Acc_Config( Enable_SerialPrint_Acc ) ;
  #endif





  delay(5000) ;

}


// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          LOOP()                                                                                //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

void loop() {
  
// put your main code here, to run repeatedly:
  
//  BlueLED_ON( Enable_SerialPrint_LED ) ;
//  delay(500);
//  BlueLED_OFF( Enable_SerialPrint_LED ); 
//  delay(500);

//  Serial.println( (String)"STM32L0.resetCause() : " + STM32L0.resetCause() );

//  delay(500) ;

  Serial.println(".") ;
  Serial.println("Your program") ;
  STM32_Temperature( Enable_SerialPrint_STM32 ) ;

  #if( Use_Acc == true )
  Acc_Get_XYZ_Data( Enable_SerialPrint_Acc ) ;
  Acc_Get_Temperature( Enable_SerialPrint_Acc ) ;
  #endif

  /* >>> EEPROM <<< */
  /*
  unsigned long long value = 231232245943999999399999989919929939911 ;
  for (int i = 0; i < 8; i++) {
    EEPROM.write(EEPROM_address + i, (value >> (i * 8)) & 0xff);
  }
  EEPROM_address = EEPROM_address + 16 ; // 16 * 8 = 128

  Serial.println( (String)"EEPROM_address : " + EEPROM_address ); 

  unsigned long long value1_high = 9223372036854775807ULL; // Partie haute du premier nombre
  unsigned long long value1_low = 9223372036854775807ULL;  // Partie basse du premier nombre
  unsigned long long value2_high = 9223372036854775807ULL; // Partie haute du deuxième nombre
  unsigned long long value2_low = 9223372036854775807ULL;  // Partie basse du deuxième nombre

  // Addition des parties hautes et basses des deux nombres
  unsigned long long result_low = value1_low + value2_low;
  unsigned long long result_high = value1_high + value2_high + (result_low < value1_low);

  Serial.print("Result high: ");
  Serial.println(result_high);
  Serial.print("Result low: ");
  Serial.println(result_low);

  */

// https://www.st.com/resource/en/reference_manual/rm0376-ultralowpower-stm32l0x2-advanced-armbased-32bit-mcus-stmicroelectronics.pdf
// page 62

  int flash_Size = STM32L0.flashSize() ;

  Serial.println( (String)"stm32l0_flash_size : " + flash_Size + "b" ) ;

  Serial.print( "address : " ) ;
  Serial.println( address , HEX ) ;

  STM32L0.flashUnlock();

  char data_pushed[] = "Hello Hello Hello" ;
  uint32_t count_datapushed = sizeof(data_pushed);

  // Programme les données en mémoire flash
  if (STM32L0.flashProgram(address, data_pushed, count_datapushed)) {
    Serial.println("Données programmées en mémoire flash avec succès");
  } else {
    Serial.println("Echec de la programmation en mémoire flash");
  }

  // Verrouille la mémoire flash pour éviter tout autre accès
  STM32L0.flashLock();

  // STM32_Flash_Write( Enable_SerialPrint_STM32 ) ;


  // bool success = STM32L0.flashRead(address, data_pulled, sizeof(data_pulled) );

    // Serial.println("Données lues à partir de la mémoire flash : ");
    // for (int i = 0; i < sizeof(data_pulled); i++) {
    //     Serial.print(data_pulled[i], BIN);
    //     Serial.print(" ");
    // }
    // Serial.println();

//  address = address + 0x80 ; // One page = 128bytes so add 128 to change page



  Serial.println("Fin loop") ;

  // delay(2000) ;
  STM32_StopMode( Enable_SerialPrint_STM32 ) ;
  
}



// —————————————————————————————————————————————————————————————————————————————————————————————— //
//          FUNCTIONS                                                                             //
// —————————————————————————————————————————————————————————————————————————————————————————————— //

/* >>> STM32 <<< */

void STM32_WakeUp( bool Enable_SerialPrint_STM32 ){

  STM32L0.wakeup() ;

  STM32_Sleeping = false ; // Setting the flag
    
  if( Enable_SerialPrint_STM32 == true ){ Serial.println("STM32 : STM32_WakeUp"); } // WakeUp msg

}

void STM32_StopMode( bool Enable_SerialPrint_STM32 ){

  STM32_Sleeping = true ; // Setting the flag
    
  if( Enable_SerialPrint_STM32 == true ){ Serial.println("STM32 : STM32_StopMode"); } // Last msg

  STM32L0.stop() ; // Stop Mode

}

void STM32_Temperature( bool Enable_SerialPrint_STM32 ){

  STM32_Temperature_float = STM32L0.getTemperature() ;

  if( Enable_SerialPrint_STM32 == true ){ Serial.println( (String)"STM32 : STM32_Temperature : " + STM32_Temperature_float + "°" ); }

}

void STM32_Flash_Write( bool Enable_SerialPrint_STM32 ){

  uint32_t STM32_Flash_address = 0x8021980 ; 

  /*
   * Corresponds to the address on the 1075th page. We start writing from here. We will use 30% of the flash memory. Be careful not to write.
   *
   * This value corresponds to the address of the 1075th page. The flash memory of the STM32L082CZ is 196kbytes. 
   * It starts at address 0x80000000 and ends at address 0x8020FFFF.  There are 1535 pages, each offering 128bytes per page, i.e. 1024bits per page. 
   * So there is a total of 1568kBytes available on the flash. When the code is uploaded, it is stored on the flash and therefore takes up space. 
   * For example, it can take up 25% of the flash. So there is 75% unused space. This space can be used to store data.
   * 
   * In this code, we will use 30% of the flash, which is 460 pages or 58,944Bytes or 471,552bits. 
   * To make sure we don't write our data on the code, we start at address 0x8021980 which corresponds to page number 1075. 
   * 
   * The memory allocation is done as follows:
   * 
   * 0% –––––––––––––––––––––––––––> 69% | 70% ––––––––––––––––––––> 100%
   * .  Code implemented on the card     |     Memory space for data
   * 
  */

  if (Message_Pushed_Count < 2 && page_number < 1536) {                          // 32,768 256-byte pages in a 8 MByte flash

    data_pushed[ Message_Pushed_Count + 0 ] = 2302081615  ; // Date
    data_pushed[ Message_Pushed_Count + 1 ] = 43000000    ; // Latitude
    data_pushed[ Message_Pushed_Count + 2 ] = 3000000     ; // Longitude
    data_pushed[ Message_Pushed_Count + 3 ] = 10          ; // Nb of satellites
    data_pushed[ Message_Pushed_Count + 4 ] = 000         ; // Acc x
    data_pushed[ Message_Pushed_Count + 5 ] = 001         ; // Acc y 
    data_pushed[ Message_Pushed_Count + 6 ] = 002         ; // Acc z
    data_pushed[ Message_Pushed_Count + 7 ] = 234         ; // Temperature
    data_pushed[ Message_Pushed_Count + 8 ] = 11          ; // LoRa satus

    Message_Pushed_Count ++ ;

  }

  else if (Message_Pushed_Count == 2 && page_number < 1536) { // if 8 number of msg or nbr of pages available
      
    // Unlocks the flash memory so that it can be programmed
    STM32L0.flashUnlock();

    // Program the data into flash memory
    if( STM32L0.flashProgram(address, data_pushed, sizeof(data_pushed) ) ) { Serial.println("Données programmées en mémoire flash avec succès"); } 
    else { Serial.println("Echec de la programmation en mémoire flash") ; }

    // Verrouille la mémoire flash pour éviter tout autre accès
    STM32L0.flashLock();

    // Display a msg for each data wrote into the SPI flash
    Serial.println( "STM32 Flash: Data wrote." ) ;

    Message_Pushed_Count = 0 ; // Reset number of msg put into the page
    STM32_Flash_address = STM32_Flash_address + 0x80 ; // Increment the page number
      
  }
    
  else { Serial.println("Reached last page of flash memory !"); Serial.println("Data logging stopped!"); } // Max page reached


  // // Unlocks the flash memory so that it can be programmed
  // STM32L0.flashUnlock();

  // // Program the data into flash memory
  // if (STM32L0.flashProgram(address, data_pushed, sizeof(datapushed)) {
  //   Serial.println("Données programmées en mémoire flash avec succès");
  // } else {
  //   Serial.println("Echec de la programmation en mémoire flash");
  // }

  // // Verrouille la mémoire flash pour éviter tout autre accès
  // STM32L0.flashLock();

}


/* >>> BLUE LED <<< */

void BlueLED_Config( bool Enable_SerialPrint_LED ){
 
  pinMode(Blue_LED, OUTPUT);      
  BlueLED_ON( Enable_SerialPrint_LED ) ;

}

void BlueLED_ON( bool Enable_SerialPrint_LED ){
 
  digitalWrite(Blue_LED, LOW);

  if( Enable_SerialPrint_LED == true ){ Serial.println("LED: ON") ; }

}

void BlueLED_OFF( bool Enable_SerialPrint_LED ){
 
  digitalWrite(Blue_LED, HIGH);

  if( Enable_SerialPrint_LED == true ){ Serial.println("LED: OFF") ; }

}


/* >>> RTC Alarm <<< */

void RTC_Enable( bool Enable_SerialPrint ){
    // // --- Set the RTC time --- //
  RTC.setAlarmTime(12, 0, 0)                   ; // Setting alarm
  RTC.enableAlarm(RTC.MATCH_Every_10s)         ; // Alarm once per second
  //RTC.enableAlarm(RTC.MATCH_SS)            ; // Alarm once per minute
  RTC.attachInterrupt( RTC_Alarm_Fct_Wakeup ) ; // Alarm interrrupt
  if(Enable_SerialPrint == true ){ Serial.println("RTC enable.") ; };
}

void RTC_Disable( bool Enable_SerialPrint ){
  RTC.disableAlarm();
  if(Enable_SerialPrint == true ){ Serial.println("RTC disable.") ; };
}

void RTC_Alarm_Fct_Wakeup() {

  //if( STM32L0.resetCause() == 1 ){RTC_Timer_count = RTC_Timer_count + 1 ;}
  RTC_Timer_count = RTC_Timer_count + 1 ;

  // Serial.println( (String)"RTC_Timer_count = " + RTC_Timer_count ) ;

  if( STM32_Sleeping == true ){
    STM32_WakeUp( Enable_SerialPrint_STM32 ) ; 
  }
  RTC_Alarm_Flag = true ; // Just set flag when interrupt received, don't try reading data in an interrupt handler
  //Serial.println("RTC: Flag timer true");
}


void EEPROM_Size( bool Enable_SerialPrint_EEPROM ){

}

/* >>> Sensor connection <<< */

void I2C_Config( ){

    /* initialize two wire bus */
  I2C_BUS.begin();                // Set master mode, default on SDA/SCL for STM32L0
  I2C_BUS.setClock(400000);       // I2C frequency at 400 kHz
  delay(1000);

  Serial.println("Scan for I2C devices:");
  i2c_0.I2Cscan();                // should detect LIS2DW12 at 0x19
  delay(1000);
}


/* >>> LIS2DW12 - Acc <<< */
#if( Use_Acc == true )

  void Acc_Config( bool Enable_SerialPrint_Acc ){

  pinMode(LIS2DW12_intPin1, INPUT);  // define LIS2DW12 wake and sleep interrupt pins as L082 inputs
  pinMode(LIS2DW12_intPin2, INPUT);

  // Read the LIS2DW12 Chip ID register, this is a good test of communication
  Serial.println("LIS2DW12 accelerometer...");
  byte LIS2DW12_ID = LIS2DW12.getChipID();  // Read CHIP_ID register for LIS2DW12
  Serial.print("LIS2DW12 "); Serial.print("I AM "); Serial.print(LIS2DW12_ID, HEX); Serial.print(" I should be "); Serial.println(0x44, HEX);
  Serial.println(" ");
  delay(1000); 

  if(LIS2DW12_ID == 0x44) // check if all I2C sensors with WHO_AM_I have acknowledged
  {
   Serial.println("LIS2DW12 is online..."); Serial.println(" ");
   
   LIS2DW12.reset();                                                // software reset before initialization
   delay(100);      

   LIS2DW12.selfTest(stress);                                       // perform sensor self test
   Serial.print("x-axis self test = "); Serial.print(stress[0], 1); Serial.println("mg, should be between 70 and 1500 mg");
   Serial.print("y-axis self test = "); Serial.print(stress[1], 1); Serial.println("mg, should be between 70 and 1500 mg");
   Serial.print("z-axis self test = "); Serial.print(stress[2], 1); Serial.println("mg, should be between 70 and 1500 mg");
   delay(1000);                                                     // give some time to read the screen

   LIS2DW12.reset();                                                // software reset before initialization
   delay(100);                                                     

   aRes = 0.000244f * (1 << fs);                                    // scale resolutions per LSB for the sensor at 14-bit data 

   Serial.println("hold flat and motionless for bias calibration");
   delay(5000);
   LIS2DW12.Compensation(fs, odr, mode, lpMode, bw, lowNoise, offset); // quickly estimate offset bias in normal mode
   Serial.print("x-axis offset = "); Serial.print(offset[0]*1000.0f, 1); Serial.println(" mg");
   Serial.print("y-axis offset = "); Serial.print(offset[1]*1000.0f, 1); Serial.println(" mg");
   Serial.print("z-axis offset = "); Serial.print(offset[2]*1000.0f, 1); Serial.println(" mg");

   LIS2DW12.init(fs, odr, mode, lpMode, bw, lowNoise);               // Initialize sensor in desired mode for application                     
   LIS2DW12.configureFIFO(fifoMode, 0x1F); // 32 levels of data
   delay(1000); // let sensor settle
   }
  else 
  {
   if(LIS2DW12_ID != 0x44) Serial.println(" LIS2DW12 not functioning!");
  }

}

  void Acc_Get_XYZ_Data( bool Enable_SerialPrint_Acc ){

    LIS2DW12.readAccelData(accelCount); // get 14-bit signed accel data

    // Now we'll calculate the accleration value into actual g's
    ax = (float)accelCount[0]*aRes - offset[0];  // get actual g value, this depends on scale being set
    ay = (float)accelCount[1]*aRes - offset[1];   
    az = (float)accelCount[2]*aRes - offset[2]; 
     
    if( Enable_SerialPrint_Acc == true ){
      Serial.print( "Accelerometer : " ) ;
      Serial.print( "ax = "  ) ; Serial.print((int)1000*ax);  
      Serial.print( " ay = " ) ; Serial.print((int)1000*ay); 
      Serial.print( " az = " ) ; Serial.print((int)1000*az); 
      Serial.println(" mg");
    }
  }

  void Acc_Get_Temperature( bool Enable_SerialPrint_Acc ){

    LIS2DWS12_Temp_Raw = LIS2DW12.readTempData();  // Read the accel chip temperature adc values
    LIS2DWS12_Temperature =  ((float) LIS2DWS12_Temp_Raw) + 25.0f; // 8-bit accel chip temperature in degrees Centigrade
    // Print temperature in degrees Centigrade      
    Serial.print("Accel temperature is ");  Serial.print(LIS2DWS12_Temperature, 1);  Serial.println(" degrees C"); // Print T values to tenths of s degree C  

  }

#endif












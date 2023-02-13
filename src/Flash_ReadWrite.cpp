
#include "Arduino.h"
#include "STM32L0_Custom.h"
#include "Flash_ReadWrite.h"

uint32_t Flash_PushToMemory_Time( int Date_YYMMDDHHMM , uint32_t flashAddress , bool Enable_SerialPrint_Flash ){
  return Flash_Push_to_Memory( Date_YYMMDDHHMM , flashAddress ) ;
}

uint32_t Flash_PushToMemory_GPS( int GPS_Latitude, int GPS_Longitude , int GPS_NbSatellites , uint32_t flashAddress , bool Enable_SerialPrint_Flash ){
  Flash_Push_to_Memory( GPS_Latitude , flashAddress ) ;
  uint32_t GPS_LongitudeNbSatellites = Flash_Create32bWord_w_24b_8b( GPS_Longitude , GPS_NbSatellites ) ;
  return Flash_Push_to_Memory( GPS_LongitudeNbSatellites , flashAddress ) ;
}

uint32_t Flash_PushToMemory_Acc( int Acc_X , int Acc_Y , int Acc_Z , int Acc_Temp , uint32_t flashAddress , bool Enable_SerialPrint_Flash ){
  uint32_t Acc_XYZ_Temp = Flash_Create32bWord_w_8b_8b_8b_8b( Acc_X , Acc_Y , Acc_Z , Acc_Temp ) ; 
  return Flash_Push_to_Memory( Acc_XYZ_Temp , flashAddress ) ;
}

uint32_t Flash_PushToMemory_Vbat_LoRa( int Vbat , int LoRa_Busy , int LoRa_Joined , uint32_t flashAddress , bool Enable_SerialPrint_Flash ){
  uint32_t Vbat_LoRa = Flash_Create32bWord_w_16b_8b_8b( Vbat , LoRa_Busy , LoRa_Joined ) ;
  return Flash_Push_to_Memory( Vbat_LoRa , flashAddress ) ;
}


uint32_t Flash_Create32bWord_w_16b_16b( uint16_t Value_1 , uint16_t Value_2 ){
    
//  uint16_t a = 65534 ; // 11111111 11111111
//  uint16_t b = 0     ; // 00000000 00000000

  uint32_t Value_12 = (Value_1 << 16) | Value_2 ; // 11111111 11111111 00000000 00000000

  return Value_12 ;
}

uint32_t Flash_Create32bWord_w_8b_8b_8b_8b( uint8_t Value_1 , uint8_t Value_2 , uint8_t Value_3 , uint8_t Value_4 ){

//  uint8_t Value_1 = 129 ; // 1000 0001 
//  uint8_t Value_2 = 2   ; // 0000 0010
//  uint8_t Value_3 = 3   ; // 0000 0011
//  uint8_t Value_4 = 5   ; // 0000 0101

  uint16_t  Value_12   = ( Value_1   << 8 ) | Value_2 ; // 1000 0001 0000 0010
  uint32_t  Value_123  = ( Value_12  << 8 ) | Value_3 ; // 1000 0001 0000 0010 0000 0011
  uint32_t  Value_1234 = ( Value_123 << 8 ) | Value_4 ; // 1000 0001 0000 0010 0000 0011 0000 0101

  return Value_1234 ;
  
}

uint32_t Flash_Create32bWord_w_16b_8b_8b( uint16_t Value_1 , uint8_t Value_2 , uint8_t Value_3 ){

//  uint8_t Value_1 = 129 ; // 1000 0001 
//  uint8_t Value_2 = 2   ; // 0000 0010
//  uint8_t Value_3 = 3   ; // 0000 0011

  uint32_t  Value_12   = ( Value_1   << 8 ) | Value_2 ; // 1000 0001 0000 0010
  uint32_t  Value_123  = ( Value_12  << 8 ) | Value_3 ; // 1000 0001 0000 0010 0000 0011

  return Value_123 ;
  
}

uint32_t Flash_Create32bWord_w_24b_8b( int Value_1 , int Value_2 ){
  // Value 1 = xxxxxxxx xxxxxxxx xxxxxxx    Value 2 = yyyyyyyy
  return (Value_1 << 8) | Value_2 ; // xxxxxxxx xxxxxxxx xxxxxxx yyyyyyyy
}


uint32_t Flash_Push_to_Memory( uint32_t Data_to_Store , uint32_t flashAddress ){

  uint32_t Data_to_Store_BigEndian = htonl( Data_to_Store ) ;
  
  // Programme les données en mémoire flash
  if (STM32L0.flashProgram( flashAddress, &Data_to_Store_BigEndian, sizeof(uint32_t) )) {
    Serial.println("Data successfully written to flash memory");
  } else {
    Serial.println("Error writing data to flash memory");
  }

  uint32_t NewflashAddress = flashAddress + sizeof( Data_to_Store_BigEndian ) ;

  return NewflashAddress ;
}

uint32_t htonl(uint32_t hostLong){
    uint32_t networkLong = ((hostLong & 0x000000FF) << 24) |
                           ((hostLong & 0x0000FF00) << 8) |
                           ((hostLong & 0x00FF0000) >> 8) |
                           ((hostLong & 0xFF000000) >> 24);
    return networkLong;
}
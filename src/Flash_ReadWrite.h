
#include "Arduino.h"


uint32_t Flash_PushToMemory_Time( int Date_YYMMDDHHMM , uint32_t flashAddress , bool Enable_SerialPrint_Flash );
uint32_t Flash_PushToMemory_GPS( int GPS_Latitude, int GPS_Longitude , int GPS_NbSatellites , uint32_t flashAddress , bool Enable_SerialPrint_Flash );
uint32_t Flash_PushToMemory_Acc( int Acc_X , int Acc_Y , int Acc_Z , int Acc_Temp , uint32_t flashAddress , bool Enable_SerialPrint_Flash );
uint32_t Flash_PushToMemory_Vbat_LoRa( int Vbat , int LoRa_Busy , int LoRa_Joined , uint32_t flashAddress , bool Enable_SerialPrint_Flash );

uint32_t Flash_Create32bWord_w_16b_16b( uint16_t Value_1 , uint16_t Value_2 );
uint32_t Flash_Create32bWord_w_8b_8b_8b_8b( uint8_t Value_1 , uint8_t Value_2 , uint8_t Value_3 , uint8_t Value_4 );
uint32_t Flash_Create32bWord_w_16b_8b_8b( uint16_t Value_1 , uint8_t Value_2 , uint8_t Value_3 );
uint32_t Flash_Create32bWord_w_24b_8b( int Value_1 , int Value_2 );

uint32_t Flash_Push_to_Memory( uint32_t Data_to_Store , uint32_t flashAddress );
uint32_t htonl(uint32_t hostLong);
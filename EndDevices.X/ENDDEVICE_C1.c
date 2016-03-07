/* 
 * File:   main.c
 * Author: Lap
 *
 * Created on 7 de abril de 2015, 03:29 PM
 */



#include <p18cxxx.h>
#include "types.h"
#include "ConfigApp.h"
#include "gpios/gpios.h"
#include "spi/spi.h"
#include "adc/adc.h"
#include "st7032/st7032.h"
#include "uart1/uart1.h"
#include "uart1/uart2.h"
#include "printf/printf.h"
#include "delays/delays.h"
#include "system/system.h"
#include <pps.h>
#include <usart.h>
#include <timers.h>
#include "WirelessProtocols/NVM.h"
#include "MAC_EEProm.h"
#include "ConfigApp.h"
#include "WirelessProtocols/MCHP_API.h"
#include "WirelessProtocols/Console.h"

extern BYTE myLongAddress[];

#define C_CHANNEL            16

#define PanCoordToCoordChan  16
#define Coord1ToEndDevChan   17
#define Coord2ToEndDevChan   18
#define PanCoordToEndDevChan 19

#define NUM_TEMP_SAMPLES            4

const _U08 OutputStat  = 0x0B;

_U08 myNodeNumber = 2;

#if ADDITIONAL_NODE_ID_SIZE > 0
    BYTE AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {0x08};
#endif


BYTE myChannel = PanCoordToEndDevChan;
_U08 Tick,TickScaler;
_U08 SwTimer0,SwTimer1,SwTimer2,SwTimer3,SwTimer4;
_U08 BaseTimeToTx = 0;
_U08 MyTrackingNumber;
_U08 MyStat;
_U08 MyTemp;


_U08 EndDevStateMachine;
float LSB,AdcVal,Vout,Tc,Ta,Vc;
WORD VBGResult;
_U16 _i;



BYTE i, j;

//****************************************************************************//
//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//

/*********************************************************************
* Function:         WORD Read_VBGVoltage(void)
*
* PreCondition:     none
*
* Input:		    none
*
* Output:		    ADRES
*
* Side Effects:	    none
*
* Overview:		    Reads the band gap voltage and compares with reference voltage
*					to arrive at the current voltage level
*
* Note:
**********************************************************************/
WORD Read_VBGVoltage(void)
{
    //ADCON0 = 0x3D;				// Configures the channel as VBG
    //ADCON1 = 0xBD;				// Program the acquisition time
    ANCON1bits.VBGEN = 1;		// Enable Band gap reference voltage

    Delay10us(1000);			//Wait for the Band Gap Settling time

    PIR1bits.ADIF = 0;
    PIE1bits.ADIE = 0;			//Disable ADC interrupts
    							//This routine uses the polling based mechanism
    //ADCON0bits.GO = 1;		    //Start A/D conversion
    //while(ADCON0bits.DONE);

    //ADCON0bits.ADON = 0;	    // Turn ADC OFF
    ANCON1bits.VBGEN = 0;	    // Disable Bandgap

    return ADRES;
}
//----------------------------------------------------------------------------//

 BYTE TemperatureRead(void)
 {
    //double temp;
     float temp;
    WORD tempValue;
    float tempAverage = 0;
    BYTE tempArray[NUM_TEMP_SAMPLES];
    

    //VBGResult =  Read_VBGVoltage();

    //ANCON1bits.VBGEN = 1;		// Enable Band gap reference voltage
    //DelayMs(10);
    //VBGResult = Adc_u16Read(15);
    //ANCON1bits.VBGEN = 0;	    // Disable Bandgap




    //temp = (1.2/(float)VBGResult);

    //tempValue = Adc_u16Read(4);

    //temp = temp*(float)tempValue;
    //temp = (temp - 0.4)/0.0195;


    //tempValue = Adc_u16Read(4);
    //temp = (temp * tempValue);
    //temp = (temp - 400.0)/19.5;

    //(BYTE) temp;






    

    //ConsolePutROMString( (ROM char * const) "  \r\n VBGResult:" );
    //PrintChar(VBGResult>>8);
    //PrintChar(VBGResult);



    LSB =0.003069;

    for(i=0;i<NUM_TEMP_SAMPLES;i++)
    {
      AdcVal = Adc_u16Read(4);
      Vout = LSB*(float)AdcVal;
      Ta = (Vout-0.4)/(float)0.0195;
      tempArray[i] = (BYTE) Ta;
    }


    for(i = 0; i<NUM_TEMP_SAMPLES; i++)
    {
        tempAverage = (tempAverage + tempArray[i]);
    }

    tempAverage = (tempAverage/NUM_TEMP_SAMPLES);

    ConsolePutROMString( (ROM char * const) "  \r\n TEMP:" );

    PrintDec((BYTE) tempAverage);


    return (BYTE)tempAverage;


 }
//----------------------------------------------------------------------------//

void StartWirelessConnection(void)
 {
    Read_MAC_Address();

    LED_1 = 0;
    LED_2 = 0;

                    //ConsolePutROMString((ROM char *)"\r\n<MAC Addr:");
                    //PrintChar(myLongAddress[3]);
                    //PrintChar(myLongAddress[2]);
                    //PrintChar(myLongAddress[1]);
                    //PrintChar(myLongAddress[0]);
                    //ConsolePutROMString((ROM char *)"\r>");

                    //ConsolePutROMString((ROM char *)"\r\n Board role: ");



 if(myNodeNumber == 0) // if the node are PAN coordinator
 {
  MiApp_ProtocolInit(FALSE);

  if( MiApp_SetChannel(myChannel) == FALSE )
  {
      return;
  }

  MiApp_ConnectionMode(ENABLE_ALL_CONN);

  MiApp_ProtocolInit(FALSE);
  MiApp_StartConnection(START_CONN_DIRECT, 0, 0);

 }
 else
 {
  // if the node are Coordinator
  ConsolePutROMString((ROM char *)"COORDINATOR >> ");
  PrintChar(myNodeNumber);

  ConsolePutROMString((ROM char *)"\r\n<MAC Addr:");
  PrintChar(myLongAddress[3]);
  PrintChar(myLongAddress[2]);
  PrintChar(myLongAddress[1]);
  PrintChar(myLongAddress[0]);
  ConsolePutROMString((ROM char *)"\r>");

  MiApp_ProtocolInit(FALSE);

    // Force PANID & SHORT ADDRESS

  if( MiApp_SetChannel(myChannel) == FALSE )
  {
   return;
  }

  MiApp_ConnectionMode(ENABLE_ALL_CONN);

  myPANID.v[0] = 0x34;
  myPANID.v[1] = 0x12;

  myShortAddress.v[0] = 0x00;
  myShortAddress.v[1] = myNodeNumber;

  myChannel = PanCoordToEndDevChan;
  //role = ROLE_COORDINATOR;

  if( MiApp_SetChannel(myChannel) == FALSE )
  {
   return;
  }

 }
}
//********************************************************************//
 // TickScaler
 //
 // 0  == 10 tick +- 1 seg
 // 1     1          100 ms
 // 2     2          200 ms
 // 3     4          400 ms
 // 4     8          800 ms
 // 5     16         1.6 s
 // 6     32         3.2 s
 // 7     64         6.4 s
 // 8     128        12.8 s
 // 9     256        25.6 s
 // 10    512        51.2 s
 // 11    1024       102.4 s   1.7  min
 // 12    2048       204.8 s   3.4  min
 // 13    4096       409.6 s   6.8  min
 // 14    8192       819.2 s   13.6 min
 // 15    16384                27.3 min
 // 16    32768                54.6 min

 void Timer1Tick(void)
 {
   _U16 Scaler;

  if(PIR1bits.TMR1IF)
  {
    PIR1bits.TMR1IF = 0;
    WriteTimer1(9286);           // overflow every 150ms
    ++Tick;


  


    switch(TickScaler)
    {
        case 0:  Scaler = 10;    break;
        case 1:  Scaler = 1;     break;
        case 2:  Scaler = 2;     break;
        case 3:  Scaler = 4;     break;
        case 4:  Scaler = 8;     break;
        case 5:  Scaler = 16;    break;
        case 6:  Scaler = 32;    break;
        case 7:  Scaler = 64;    break;
        case 8:  Scaler = 128;   break;
        case 9:  Scaler = 256;   break;
        case 10: Scaler = 512;   break;
        case 11: Scaler = 1024;  break;
        case 12: Scaler = 2048;  break;
        case 13: Scaler = 4096;  break;
        case 14: Scaler = 8192;  break;
        case 15: Scaler = 16384; break;
        case 16: Scaler = 32768; break;
        default: Scaler = 8;     break;
    }



    if(Tick >= Scaler)
    {
     Tick =0;
     ++SwTimer0;
     ++SwTimer1;
     ++SwTimer2;
     ++SwTimer3;     // MIWIPRO STATUS TIMER
     ++SwTimer4;
    }
   }
 }

//----------------------------------------------------------------------------//
 void  TaskScheduler(void)
 {
     Timer1Tick();

     switch(EndDevStateMachine)
     {
         case 0:
         {
             if( MiApp_MessageAvailable())
             {
                if((rxMessage.Payload[1] == 0x00)&&((rxMessage.Payload[3] == OutputStat)))
                {
                    if(rxMessage.Payload[4] <= 16)
                    {
                        BaseTimeToTx = rxMessage.Payload[4];
                    }

                    SwTimer0 = 0;
                    LED_2 ^= 1;
                    EndDevStateMachine = 1;

                }

                 MiApp_DiscardMessage();
             }


         }
         break;

         case 1:
         {
             if(SwTimer0 >= (_U08)myNodeNumber)
             {

              MyTemp = TemperatureRead();

              MiApp_FlushTx();
              MiApp_WriteData(' ');
              MiApp_WriteData(myNodeNumber);
              MiApp_WriteData(MyTrackingNumber);
              MiApp_WriteData(OutputStat);
              MiApp_WriteData(MyStat);
              MiApp_WriteData(MyTemp);

              MiApp_BroadcastPacket(FALSE);

              EndDevStateMachine = 2;
             }
         }
         break;

         case 2:
         {
             LED_1 ^= 1;
             EndDevStateMachine = 0;

             //ConsolePutROMString( (ROM char * const) "  \r\n Temperature here:" );
             //PrintDec(MyTemp);

         }
         break;

         case 3:
         {

         }
         break;


         default:
         break;
     }

 }

//----------------------------------------------------------------------------//

void main(void)

{
     #define BAUDRG 77

    BYTE SecNum = 0;
    /*******************************************************************/
    // Initialize the system
    /*******************************************************************/

    ANCON0 = 0XFF;     /*desactiva entradas analogicas*/
    ANCON1 = 0XFF;     /*desactiva entradas analogicas*/



    PPSUnLock();

    PPSOutput(PPS_RP10, PPS_TX2CK2);    // TX2 RP17/RC6     icsp
    PPSInput(PPS_RX2DT2, PPS_RP9);     // RX2 RP18/RC7
    




    PPSOutput(PPS_RP23, PPS_SDO2);    // SDO2 RP23/RD6
    PPSInput(PPS_SDI2, PPS_RP24);     // SDI2 RP24/RD7
    PPSOutput(PPS_RP22, PPS_SCK2);    // SCK2 RP22/RD5

    PPSLock();

     System_PeripheralPinSelect( ExternalInterrupt3, 19);  /*external interrupt 3 B3*/

     BoardInit();
     ConsoleInit();

     Gpios_PinDirection(GPIOS_PORTC, 7, GPIOS_INPUT);  /*pin C0 como salida para SDI*/
     Gpios_PinDirection(GPIOS_PORTC, 6, GPIOS_OUTPUT); /*pin C1 como salida para SDO*/

     Open1USART(USART_TX_INT_OFF & USART_RX_INT_OFF & USART_EIGHT_BIT & USART_ASYNCH_MODE & USART_ADDEN_OFF, BAUDRG);
     baud1USART(BAUD_IDLE_TX_PIN_STATE_HIGH & BAUD_IDLE_RX_PIN_STATE_HIGH & BAUD_AUTO_OFF & BAUD_WAKEUP_OFF & BAUD_16_BIT_RATE & USART_RX_INT_OFF);


     Open2USART(USART_TX_INT_OFF & USART_RX_INT_OFF & USART_EIGHT_BIT & USART_ASYNCH_MODE & USART_ADDEN_OFF, BAUDRG);
     baud2USART(BAUD_IDLE_TX_PIN_STATE_HIGH & BAUD_IDLE_RX_PIN_STATE_HIGH & BAUD_AUTO_OFF & BAUD_WAKEUP_OFF & BAUD_16_BIT_RATE & USART_RX_INT_OFF);


     OpenTimer1( TIMER_INT_OFF &T1_16BIT_RW &T1_SOURCE_FOSC_4 & T1_PS_1_8 &T1_OSC1EN_OFF &T1_SYNC_EXT_OFF, TIMER_GATE_OFF  & TIMER_GATE_INT_OFF);



     Gpios_PinDirection(GPIOS_PORTD, 7, GPIOS_INPUT);  /*pin C0 como salida para SDI*/
     Gpios_PinDirection(GPIOS_PORTD, 6, GPIOS_OUTPUT); /*pin C1 como salida para SDO*/
     Gpios_PinDirection(GPIOS_PORTD, 5, GPIOS_OUTPUT); /*pin C2 como salida para SCK*/

     Spi_Init(SPI_PORT1, SPI_64DIV); /*Inicializamos SPI2*/
     Spi_Init(SPI_PORT2, SPI_64DIV); /*Inicializamos SPI2*/

     Adc_Init(ADC_10BITS);

     LED_1 = 1;
     LED_2 = 0;

     for(j=0;j<10;j++)
     {
         DelayMs(250);

         LED_1 ^= 1;
         LED_2 ^= 1;

     }

     LED_1 = 0;
     LED_2 = 0;


     StartWirelessConnection();
     TickScaler = 2;
     EndDevStateMachine =0;

     //.VBGOE = 0;
     //ANCON1bits.VBGEN = 1;		// Enable Band gap reference voltage
     //DelayMs(10);
     //VBGResult = Adc_u16Read(15);
     //ANCON1bits.VBGEN = 0;	    // Disable Bandgap

      //Adc_Init(ADC_10BITS);

	

     while(1)
     {
      TaskScheduler();
      
     }

}
/* 
 * File:   main.c
 * Author: Lap
 *V1.1. Se genera un pulso para monitorearse mediante watchdog externo
 * Created on 7 de abril de 2015, 03:29 PM
 */
#include <p18cxxx.h>
#include "types.h"
#include "ConfigApp.h"
#include "gpios/gpios.h"
#include "spi/spi.h"
//#include "adc/adc.h"
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

#include <adc.h>

extern BYTE myLongAddress[];

#define C_CHANNEL            16

#define PanCoordToCoordChan  16
#define Coord1ToEndDevChan   17
#define Coord2ToEndDevChan   18
#define PanCoordToEndDevChan 19

#define NUM_TEMP_SAMPLES            8
BYTE tempArray[NUM_TEMP_SAMPLES];
BYTE tempIndex = 0;

const _U08 OutputStat           = 0x0B;
const _U08 ListenDevices        = 0x0C;
const _U08 CalibrationProcedure = 0x0D;


//////////
_U08 PanCoordStateMachine = 0;
#define PCDevices           4
_U08 MyDevicesTemp[PCDevices];
_U08 ConnectedDevices[16];
_U08 myDevicesRequiredStatus[16];
_U08 myDevicesOutputStatus[16];
//////////

_U08 myNodeNumber = 128;


#if ADDITIONAL_NODE_ID_SIZE > 0
    BYTE AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {0x08};
#endif


BYTE myChannel = PanCoordToCoordChan;
_U08 Tick;
_U08 TickScaler = 4;

_U08 SwTimer0,SwTimer1,SwTimer2,SwTimer4;
_U16 SwTimer3;
_U08 BaseTimeToTx = 4;
_U08 MyTrackingNumber;
_U08 MyStat;
_U08 MyTemp;
_U08 TemperatureCalibrationValue;

_U08 EndDevStateMachine;
float LSB,AdcVal,Vout,Tc,Ta,Vc;
_U16 VBGResult,SensorResult;
_U16 _i;


BOOL EEPROMCHG =0;
BYTE i, j;
BYTE RestartCounter = 0;
BYTE BypassCounter;
BYTE RLY_STATUS=0;
BYTE SDW_RLY_STATUS=0;

BYTE IrStatus = 0;

#define DebounceButtons 20


_U08 Exit = 255;
_U08 MenuIn = 0;
_U08 Option = 0;
_U08 MenuIndex =0;
_U08 OffsetValue;

BOOL Starting = 1;
BOOL Button1Pressed = 0;
BOOL Button2Pressed = 0;
BOOL Button1Released = 1;
BOOL Button2Released = 1;
_U08 Debounce1ON = 0;
_U08 Debounce2ON = 0;
_U08 Debounce1OFF = 0;
_U08 Debounce2OFF = 0;

_U08 CycleButton = 0;


//****************************************************************************//
 void Rs485Put(_U08 c)
{
    while(!TXSTA1bits.TRMT);
    TXREG1 = c;
}


_U08 Rs485Get(void)
{
    _U16 timeout =0;

    while((!PIR1bits.RC1IF)&&(++timeout)<5000);
    //while((!PIR1bits.RC1IF));
    if(PIR1bits.RC1IF)
    {
        if(RCSTA1bits.OERR)
        {
            RCSTA1bits.CREN = 0;   // Disable UART receiver
            RCSTA1bits.CREN = 1;   // Enable UART receiver
        }

        return RCREG1;
    }
    else
    {
         return 0xFF;
    }
}



void Rs485GetString(_U08 *dest,  _U16 count)
{
 _U08 oldGIEH = INTCONbits.GIEH;
 INTCONbits.GIEH = 0;

 while( count > 0 )
 {
  *dest++ = Rs485Get();
  count--;
 }

  INTCONbits.GIEH = oldGIEH;

}
//----------------------------------------------------------------------------//
void IRCommand(BYTE Cmd)
{
    switch(Cmd)
    {
        case 1:
        {
          Rs485Put(0x23);
          Rs485Put(0x5E);
          Rs485Put(0x00); 
          Rs485Put(0x17);
          Rs485Put(0x00);       // comando 0 "encender aire"
          Rs485Put(0x3C);
          Rs485Put(0x3F);  
        }
        break;
        
        case 2:
        {
          Rs485Put(0x23);
          Rs485Put(0x5E);
          Rs485Put(0x00); 
          Rs485Put(0x17);
          Rs485Put(0x01);       // comando "apagar aire"
          Rs485Put(0x3C);
          Rs485Put(0x3F); 
        }
        break;
        
        default: break;
        
    }
}
//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//
//----------------------------------------------------------------------------//
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
    //ANCON1bits.VBGEN = 1;		// Enable Band gap reference voltage
    
    //Delay10us(100);			//Wait for the Band Gap Settling time
    ADCON0bits.CHS = 0x0F;  // VBG channel select
    DelayMs(1);
    
    //PIR1bits.ADIF = 0;
    //PIE1bits.ADIE = 0;			//Disable ADC interrupts
    							//This routine uses the polling based mechanism
    ADCON0bits.GO = 1;		    //Start A/D conversion
    while(ADCON0bits.DONE);

    //ADCON0bits.ADON = 0;	    // Turn ADC OFF
    //ANCON1bits.VBGEN = 0;	    // Disable Bandgap
    
    
    ConsolePutROMString( (ROM char * const) "  \r\n VBGResult:" );
    PrintChar(ADRESH);
    PrintChar(ADRESL);

    return ADRES;
}
//----------------------------------------------------------------------------//

 BYTE TemperatureRead(void)
 {
    double temp;
    BYTE tempHere;
    float tempAverage = 0;
    

    
    VBGResult = Read_VBGVoltage();
    
    
    ADCON0bits.CHS = 0x04;  // Temperature sensor channel select
    DelayMs(1);
    
    j=0;
    
    do
    {
    
      ADCON0bits.GO = 1;		    //Start A/D conversion
      while(ADCON0bits.DONE);
    
      SensorResult = ADRES;
      //ConsolePutROMString( (ROM char * const) "  \r\n TempResult:" );
      //PrintChar(ADRESH);
      //PrintChar(ADRESL);
    
    
      temp = (1200.0/VBGResult);
      temp = (temp * SensorResult);
      temp = (temp - 400.0)/19.5;
    
      tempArray[j] = (BYTE) temp;
      //tempIndex++;
    
      j++;
    }while(j < NUM_TEMP_SAMPLES);
    
    
    for(j = 0; j<NUM_TEMP_SAMPLES; j++)
    {
        tempAverage = (tempAverage + tempArray[j]);
    }
    tempAverage = (tempAverage/NUM_TEMP_SAMPLES);
    tempHere = (BYTE) tempAverage;
    tempAverage = (tempAverage - tempHere) * 10;
    
    if(tempAverage >= 5)
        tempHere = tempHere + 1;
        
    
    
    ConsolePutROMString( (ROM char * const) "  \r\n TempGrados:" );
    PrintDec((BYTE)tempHere);
    
    
    return (BYTE)tempHere;  


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

  myChannel = PanCoordToCoordChan;
  //role = ROLE_COORDINATOR;

  if( MiApp_SetChannel(myChannel) == FALSE )
  {
   return;
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
    //WriteTimer1(9286);           // overflow every 150ms
    //WriteTimer1(28036);          // overflow every 100 ms
    //WriteTimer1(28036);          // overflow every 50 ms
    WriteTimer1(56161);          // overflow every 25 ms
    
    ++Tick;
    ClrWdt();
   
    switch(TickScaler)
    {
        case 0:  Scaler = 10;    break;
        case 1:  Scaler = 1;     break;    // 25 ms
        case 2:  Scaler = 2;     break;    // 50 ms
        case 3:  Scaler = 4;     break;    // 100 ms
        case 4:  Scaler = 8;     break;    // 200 ms
        case 5:  Scaler = 16;    break;    // 400 ms
        case 6:  Scaler = 32;    break;    // 800 ms
        case 7:  Scaler = 64;    break;    // 
        case 8:  Scaler = 128;   break;    // 
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
    
     //************************************************************************
     //*********JL:Pulso para mantener el watchdog externo habilitado**********
     if(PORTBbits.RB4 == 0)
        {
        PORTBbits.RB4=1;
        }
    else if(PORTBbits.RB4 == 1)
        {
        PORTBbits.RB4=0;   
        }
     //*************************************************************************
    }
   }
 }

//----------------------------------------------------------------------------//
 void IRControl(void)
 {
     
     if(RLY_STATUS != IrStatus)   // si ha habido cambio en estado de relevador...
     {
         if(RLY_STATUS == 1)
         {
             IRCommand(1);
         }
         else
         {
             IRCommand(2);
         }
         
         IrStatus = RLY_STATUS;
         
     }
     
 }
 //---------------------------------------------------------------------------//
 
 //-----------------------------------------------------------------------------

 void WirelessTxRx(void)
 {
     Timer1Tick();

     switch(EndDevStateMachine)
     {
         case 0:                      // recibe estado para relevadores y latencia de comunicacion
         {
             if( MiApp_MessageAvailable())
             {
                if((rxMessage.Payload[1] == 0x00)&&((rxMessage.Payload[3] == OutputStat)))
                {
                    BypassCounter = 0;
                    if(rxMessage.Payload[4] <= 16)
                    {
                        //BaseTimeToTx = rxMessage.Payload[4];
                        TickScaler = rxMessage.Payload[4];
                    }


                    j = (myNodeNumber-1)/8;
                    j += 5;

                    i = rxMessage.Payload[j];

                    j =  (myNodeNumber-1)%8;

                    if(QUERY_8BIT(i,j))
                    {

                        RLY_STATUS = 1;
                    }
                    else
                    {

                        RLY_STATUS = 0;
                    }


                    ConsolePutROMString((ROM char *)"\r\n Req Stat:");

                    for(i=0;i<rxMessage.PayloadSize;i++)
                    {
                        PrintChar(rxMessage.Payload[i]);
                    }
                    

                    SwTimer0 = 0;
                    WriteTimer1(56161);           
                    Tick = 0;
                    
                    LED_2 ^= 1;
                    EndDevStateMachine = 1;
                    SwTimer3 = 0;     // clear MIWIPRO STATUS TIMER


                }
                else
                {
                    if((rxMessage.Payload[1] == 0x00)&&((rxMessage.Payload[3] == ListenDevices)))
                    {
                        ConsolePutROMString((ROM char *)"\r\n Listen Devices:");

                        for(i=0;i<rxMessage.PayloadSize;i++)
                        {
                         PrintChar(rxMessage.Payload[i]);
                        }
                        
                        j = (myNodeNumber-1)/8;
                        j += 4;

                        i = rxMessage.Payload[j];

                        j =  (myNodeNumber-1)%8;
                        
                        
                        if(QUERY_8BIT(i,j))
                        {
                         ConsolePutROMString((ROM char *)"\r\n Link stablished correctly");
                         RestartCounter = 0;
                        }
                        else
                        {
                         ConsolePutROMString((ROM char *)"\r\n NO Link stablished!!");
                         ++RestartCounter;
                        }
                        
                        
                        
                    }
                    else
                    {
                        if((rxMessage.Payload[1] == 0x00)&&((rxMessage.Payload[3] == CalibrationProcedure)))
                        {
                            
                            ConsolePutROMString((ROM char *)"\r\n Temperature sensor calibration:");
                            
                            //TemperatureCalibrationValue = rxMessage.Payload[5];
                            
                            
                            if((rxMessage.Payload[5] == MyTemp)||(rxMessage.Payload[5] == 0))
                            {
                                TemperatureCalibrationValue = 0;
                            }
                            else
                            {
                                if(rxMessage.Payload[5] > MyTemp)              // la temp de referencia es mayor a la leida
                                {
                                    //TemperatureCalibrationValue = (rxMessage.Payload[5] - MyTemp) & 0x0F;
                                    
                                }
                                else                                           // la temperatura leida es mayor a la temp de referencia
                                {
                                    //TemperatureCalibrationValue = (MyTemp - rxMessage.Payload[5]) & 0x0F;
                                    //SET_8BIT(TemperatureCalibrationValue,7);
                                }
                            }
                            
                            //EEPROMWRITE(TemperatureCalibrationValue,0,1);
                        }
                    }
                    
                }
                

                 MiApp_DiscardMessage();
             }
         }
         break;

//-----------------------------------------------------------------------------//

         case 1:                // envia estado y temperatura
         {
             if(SwTimer0 >= (_U08)myNodeNumber)
             {

              MyTemp = TemperatureRead();
                                                       
              MiApp_FlushTx();
              MiApp_WriteData(' ');
              MiApp_WriteData(myNodeNumber);
              MiApp_WriteData(MyTrackingNumber);
              MiApp_WriteData(OutputStat);
              MiApp_WriteData(RLY_STATUS);
              MiApp_WriteData(MyTemp);

              MiApp_BroadcastPacket(FALSE);
              

             /*              
              if(RLY_STATUS == 1)
              {
                  RLY_1 = 1;
              }
              else
              {
                  RLY_1 = 0;
              }
              */
              
              
              //SDW_RLY_STATUS
              
              if(RLY_STATUS == 1)
              {
                  if(SDW_RLY_STATUS == 1)
                  {
                      RLY_1 = 1;
                      
                      if(EEPROMCHG == 0)
                      {
                        i = 0x55;
                        EEPROMWRITE(&i,0,1); 
                        EEPROMWRITE(&i,0,1); 
                        EEPROMWRITE(&i,0,1); 
                        EEPROMWRITE(&i,0,1); 
                        EEPROMWRITE(&i,0,1); 
                          
                          
                       //for(j=0;j<16;j++)
                       //{
                        //myDevicesOutputStatus[j] = 0x55;
                       //}
                       //EEPROMWRITE(&myDevicesOutputStatus,0,16);
                       //EEPROMWRITE(&myDevicesOutputStatus,0,16);
                       //EEPROMWRITE(&myDevicesOutputStatus,0,16);
                       //EEPROMWRITE(&myDevicesOutputStatus,0,16);
                       //EEPROMWRITE(&myDevicesOutputStatus,0,16);
                        
                        ConsolePutROMString((ROM char *)"\r\n Relay status updated in eeprom 55");
                      }
                      
                      EEPROMCHG = 1;
                  }
                  else
                  {
                      SDW_RLY_STATUS = 1;
                  }
              }
              else
              {
                  if(SDW_RLY_STATUS == 0)
                  {
                      RLY_1 = 0;
                      
                      if(EEPROMCHG == 1)
                      {
                       i = 0xAA;
                       EEPROMWRITE(&i,0,1); 
                       EEPROMWRITE(&i,0,1); 
                       EEPROMWRITE(&i,0,1); 
                       EEPROMWRITE(&i,0,1); 
                       EEPROMWRITE(&i,0,1); 
                          
                       //for(j=0;j<16;j++)
                       //{
                        //myDevicesOutputStatus[j] = 0xAA;
                       //}
                       //EEPROMWRITE(&myDevicesOutputStatus,0,16);   
                       //EEPROMWRITE(&myDevicesOutputStatus,0,16);
                       //EEPROMWRITE(&myDevicesOutputStatus,0,16);
                       //EEPROMWRITE(&myDevicesOutputStatus,0,16);
                       //EEPROMWRITE(&myDevicesOutputStatus,0,16);
                          
                          
                       ConsolePutROMString((ROM char *)"\r\n Relay status updated in eeprom AA");
                      }
                      
                      EEPROMCHG = 0;
                      
                  }
                  else
                  {
                      SDW_RLY_STATUS = 0;
                  }
              }
              
              
              
              
              
              
              IRControl();
              

              EndDevStateMachine = 2;
             }
         }
         break;

//-----------------------------------------------------------------------------//

         case 2:
         {
             LED_1 ^= 1;
             EndDevStateMachine = 0;
         }
         break;

//-----------------------------------------------------------------------------//

         default:
         {
             EndDevStateMachine = 0;
         }
         break;


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
 
 void WirelesStatus(void)
 {
     if( SwTimer3 >= 150)           // if lost connection ...
     {
      ConsolePutROMString( (ROM char * const) "  \r\n Timer - Link Lost... Restarting..." );
      
      ++BypassCounter;
    
      Reset();
      //StartWirelessConnection();   //  Try to restart connection
     }
     
     
     
     if(RestartCounter >= 3)
     {
         ConsolePutROMString( (ROM char * const) "  \r\n Counter - Link Lost... Restarting..." );
         Reset();
     }
     
 }
 //----------------------------------------------------------------------------//
 void ReadButtons(void)
 {
  ++CycleButton;
  
  if(CycleButton >= 30)
  {
      CycleButton = 0;
      
     if(PUSH_BUTTON_1 == 0)
     {
       //ConsolePutROMString((ROM char *)"Button 1 Presed!!! "); 
         Debounce1ON++;
         Debounce1OFF = 0;
         
         if(Debounce1ON >= DebounceButtons)
         {
             Button1Pressed = 1;
         }
         
         
     }
     else
     {
         Debounce1OFF++;
         Debounce1ON = 0;
         
         if(Debounce1OFF >= DebounceButtons)
         {
             Button1Pressed = 0;
             Button1Released =1;
         }
     }
         
     if(PUSH_BUTTON_2 == 0)
     {
       //ConsolePutROMString((ROM char *)"Button 2 Presed!!! ");
         Debounce2ON++;
         Debounce2OFF = 0;
         
         if(Debounce2ON >= DebounceButtons)
         {
             Button2Pressed = 1;
         }
     }
     else
     {
         Debounce2OFF++;
         Debounce2ON = 0;
         
         if(Debounce2OFF >= DebounceButtons)
         {
             Button2Pressed = 0;
             Button2Released = 1;
         }
     }
     
     
     
     
  } 
 }
 //----------------------------------------------------------------------------//
 void Menu(void)
 {
      ReadButtons();
      
      
     if( Button1Pressed && Button2Pressed)   // si se han presionado ambos botones 
     {
      ++MenuIn;
     }
     else
     {
         MenuIn = 0;
     }
     
      if(MenuIn >=250)
      {
          MenuIndex = 1;
          MenuIn = 0;
      }
      else
      {
          MenuIndex = 0;
      }
     
      if(MenuIndex != 0)
      {
       switch(MenuIndex)
       {
         case 0:
         {
             
         }
         break;
         
         case 1:
         {
             LED_1 = 1;
             LED_2 = 1;
             
             ConsolePutROMString((ROM char *)"\r\n      MENU : Temperature Calibration ");
             
             
             while((Button1Pressed)&&(Button2Pressed))
             {
                 if( MiApp_MessageAvailable())
                 {
                     MiApp_DiscardMessage();
                 }
                 
                 ReadButtons();
                 ClrWdt();
             }
             
             ConsolePutROMString((ROM char *)"\r\n SW1: Increase offset value  || SW2 Decrease offset value");
             
             
             
             Button1Released = 1;
             Button2Released = 1;
             
             Exit = 0xff;
             OffsetValue = 100;
             
             while(Exit)
             {
                 ReadButtons();
                 
                 
                 
                 
                 
                 if( Button1Pressed && Button2Pressed)   // si se han presionado ambos botones
                 {
                     Exit = 0;
                     ConsolePutROMString((ROM char *)"\r\nExit");       
                 }
                 else
                 {
                     if( Button1Pressed && Button1Released)   // 
                   {
                     Button1Released = 0;
                     ++OffsetValue;
                     LED_1 ^= 1;
                     LED_2 ^= 1;
                     ConsolePutROMString((ROM char *)"\r\nIncreased offset value");
                   }
                 
                   if( Button2Pressed && Button2Released)   // 
                   {   
                     Button2Released = 0;
                     --OffsetValue;
                     LED_1 ^= 1;
                     LED_2 ^= 1;
                     ConsolePutROMString((ROM char *)"\r\nDecreased offset value");
                   }
                 }
                 
           
                 
                 
                 
                 if( MiApp_MessageAvailable())
                 {
                     MiApp_DiscardMessage();
                 }
                 
                 ClrWdt();
             }
             
             while((Button1Pressed)&&(Button2Pressed))
             {
              if( MiApp_MessageAvailable())
              {
                MiApp_DiscardMessage();
              }
                 
              ReadButtons();
              ClrWdt();
             }
             
             
             
             
         }
         break;
         
         case 2:
         {
             
         }
         break;
         
         case 3:
         {
             
         }
         break;
         
         
         default: break;
         
       }
 }
     
     
     
 }
 //----------------------------------------------------------------------------//
 void Bypass(void)
 {
    if(SwTimer4 >= 50 ) 
    {
     SwTimer4 = 0;
     
     if(BypassCounter >= 5)      
     {
         RLY_1 = 1;
         
         
         EEPROMRead(&myDevicesRequiredStatus, 0, 1);
         if(myDevicesRequiredStatus[0] == 0xAA)      // estado relevador apagado
         {
             i = 0x55;
             EEPROMWRITE(&i,0,1); 
             EEPROMWRITE(&i,0,1); 
             EEPROMWRITE(&i,0,1); 
             EEPROMWRITE(&i,0,1); 
             EEPROMWRITE(&i,0,1);
         }
         
         
         
         ConsolePutROMString((ROM char *)"\r\nBypass");
     }
    }
 }
void Repeater(void)
 {
     if( MiApp_MessageAvailable())
     {
            Printf("\r\n  ");
            if( rxMessage.flags.bits.secEn )
            {
                ConsolePutROMString((ROM char *)"Secured ");
            }

            if( rxMessage.flags.bits.broadcast )
            {
             ConsolePutROMString((ROM char *)"Broadcast Packet with RSSI ");
            }
            else
            {
             ConsolePutROMString((ROM char *)"Unicast Packet with RSSI ");
            }
            PrintChar(rxMessage.PacketRSSI);


           if( rxMessage.flags.bits.srcPrsnt )
            {
                ConsolePutROMString((ROM char *)" from ");
                if( rxMessage.flags.bits.altSrcAddr )
                {
                    PrintChar(rxMessage.SourceAddress[1]);
                    PrintChar(rxMessage.SourceAddress[0]);
                }
                else
                {
                    for(i = 0; i < MY_ADDRESS_LENGTH; i++)
                    {
                      PrintChar(rxMessage.SourceAddress[MY_ADDRESS_LENGTH-1-i]);

                    }
                }
            }

            ConsolePutROMString((ROM char *)": ");


            for(i = 0; i < rxMessage.PayloadSize; i++)
            {
              PrintChar(rxMessage.Payload[i]);
            }
         
       ClrWdt();  
       SwTimer3 = 0;  
         
       LED_2 ^= 1;
       MiApp_DiscardMessage();
       
     }
     
     
     if(SwTimer3 >= 500)           // if lost connection ... JL:Anteriormente 100, se cambio para
     {                              //lugares donde los mensajes llegan con mucho retardo entre uno y otro (area de pozo))
         Reset();
     }
 }
void main(void)
{
     #define BAUDRG 77

    BYTE SecNum = 0;

   BOOL Tx_Success = FALSE;
   BYTE Tx_Trials = 0, scanresult = 0;
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
     
     //Gpios_PinDirection(GPIOS_PORTD, 4, GPIOS_OUTPUT);  /*pin D4 como salida */

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

     //Adc_Init(ADC_10BITS);

     LED_1 = 1;
     LED_2 = 0;
     //RLY_1 = 0;
     RLY_2 = 0;

     ON_RADIO = 1;
     ON_MAC = 1;
     ON_TEMP = 1;
     
     StartWirelessConnection();
     
     
     
     //myDevicesRequiredStatus[0] = 0x55;
     
     
     EEPROMRead(&myDevicesRequiredStatus, 0, 1);
     
     if(myDevicesRequiredStatus[0] == 0x55)
     {
         RLY_1 = 1;
         EEPROMCHG = 1;
         ConsolePutROMString((ROM char *)"RELAY ON ");
     }
     
     if(myDevicesRequiredStatus[0] == 0xAA)
     {
         RLY_1 = 0;
         EEPROMCHG = 0;
         ConsolePutROMString((ROM char *)"RELAY OFF ");
     }
     
     
     
     
     

     for(j=0;j<10;j++)
     {
         DelayMs(50);

         LED_1 ^= 1;
         LED_2 ^= 1;
     }
     
     

     LED_1 = 0;
     LED_2 = 0;
     //RLY_1 = 0;
     RLY_2 = 0;
     
    
     
     TickScaler = 4;
     EndDevStateMachine =0;



/*
                                

                                while(!Tx_Success)
                                {
                                    if(myChannel < 8)
                                        scanresult = MiApp_SearchConnection(10, (0x00000001 << myChannel));
                                    else if(myChannel < 16)
                    				    scanresult = MiApp_SearchConnection(10, (0x00000100 << (myChannel-8)));
                                    else if(myChannel < 24)
                    				    scanresult = MiApp_SearchConnection(10, (0x00010000 << (myChannel-16)));
                                    else
                    				    scanresult = MiApp_SearchConnection(10, (0x01000000 << (myChannel-24)));
                                    if(scanresult == 0)
                                    {
                                        Tx_Trials++;
                                        if(Tx_Trials > 2) break;

                                    }
                                    else Tx_Success = TRUE;

                                }
                                if(Tx_Success)
                                {
                                    ConsolePutROMString((ROM char *)"RADIO OK ");
                                }
                                else
                                {
                                    ConsolePutROMString((ROM char *)"RADIO FAIL ");
                                }

*/


     //.VBGOE = 0;
     //ANCON1bits.VBGEN = 1;		// Enable Band gap reference voltage
     //DelayMs(10);
     //VBGResult = Adc_u16Read(15);
     //ANCON1bits.VBGEN = 0;	    // Disable Bandgap

      //Adc_Init(ADC_10BITS);
     
     
     ANCON0 = 0xFF;
     ANCON1 = 0x9F;
     ANCON1bits.VBGEN = 1;		// Enable Band gap reference voltage
     
     
     ADCON0bits.VCFG = 0;    // vreff VDD-VSS
     ADCON0bits.CHS = 0x0F;  // VBG channel select
     
     ADCON1 = 0xBE;
     ADCON0bits.ADON = 1;
     
     //for(j=0;j<16;j++)
     //{
        //myDevicesOutputStatus[j] = j;
     //}
    
     
     //EEPROMWRITE(myDevicesOutputStatus,0,16);
     
     
     //for(j=0;j<16;j++)
     //{
        //myDevicesOutputStatus[j] = 0;
     //}
     
     //DelayMs(500);
     
     EEPROMRead(&myDevicesOutputStatus, 0, 16);
     ConsolePutROMString((ROM char *)"EEPROM READ: ");
     //PrintChar(TemperatureCalibrationValue);
     
     
     for(j=0;j<1;j++)
     {
         PrintChar(myDevicesOutputStatus[j]);
     }
	

     SwTimer3 = 0;
     SwTimer4 = 0;
     
     TRISB&=0xEF;   //JL: Configuro el pin B4 como salida si modificar el estado de los demas pines
     
     while(1)
     {
     /*
         WirelessTxRx();
         WirelesStatus();
         Bypass();
         */
         
         
         //No se utilizaron
         //Menu();
         //Timer1Tick();
         //WirelessTxRxPANCOORD();
         //TaskScheduler();
         
         //JLEstas funciones deben habilitarse para trabajar como repetidora
         
         Timer1Tick();
         Repeater();
        
     }

}
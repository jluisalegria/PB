/* 
 * File:   main.c
 * Author: Lap
 *
 * Created on 7 de abril de 2015, 03:29 PM
 */


/*

 * dispositivos soportados:  128
 * rango:                    0 - 127

 * dispositivos Coordinador 1:   0  - 31  >> 32
 *
 * dispositivos PAN COORDINATOR: 32 - 95  >> 16
 *
 * dispositivos Coordinador 2:   96 - 127
 *
 */



#include <p18cxxx.h>
#include "types.h"
#include "ConfigApp.h"
#include "gpios/gpios.h"
#include "spi/spi.h"
#include "st7032/st7032.h"
#include "eeprom25xx/eeprom25xx.h"
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

//#define DevicesInstalled     48

#define PCDevices            48
#define C1Devices            1
#define C2Devices            1

#define CAL_LEN              46      // Bytes per Device
#define HRS_LEN              732     // Bytes per Device



const _U08 Resync      = 0x0A;
const _U08 OutputStat  = 0x0B;

_U08 myNodeNumber = 0;


_U08 myDevicesRequiredStatus[16];
_U08 myDevicesOutputStatus[16];
_U08 ConnectedDevices[16];
_U08 C1DevicesOutputStatus[4];
_U08 C2DevicesOutputStatus[4];

_U08 MyDevicesTemp[PCDevices];
_U08 C1DevicesTemp[C1Devices];
_U08 C2DevicesTemp[C2Devices];

_U08 myvar = 0;





#if ADDITIONAL_NODE_ID_SIZE > 0
    BYTE AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {0x08};
#endif


BYTE myChannel = C_CHANNEL;
_U08 Tick,TickScaler;
_U08 SwTimer0,SwTimer1,SwTimer2,SwTimer3,SwTimer4;
_U08 BaseTimeToTx = 0;
_U08 MyTrackingNumber;
_U08 ConnectedCoordinators;


_U08 PanCoordStateMachine = 0;

BYTE i, j;

typedef struct
{
    _U08 Byte;
    _U08 Bit;
}_ByteBit;

_ByteBit ByteBitVal;



void Return_ByteBit(_U08 dia, _U08 mes);
void QueryCalendars(void);

//********************************************************************//
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

  myChannel = C_CHANNEL;
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

    if(myNodeNumber == 0)
    {

    }
    else
    {

    }


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

//****************************************************************************//
//****************************************************************************//
//****************************************************************************//
 _U08 EthernetGetByte(void)
{
    _U08 timeout =0;
    _U08 RCVChar;

    CTS = 0;

    while((!PIR3bits.RC2IF)&&(++timeout)<250);
    CTS = 1;
    if(PIR3bits.RC2IF)
    {
        if(RCSTA2bits.OERR)
        {
            RCSTA2bits.CREN = 0;   // Disable UART receiver
            RCSTA2bits.CREN = 1;   // Enable UART receiver
        }

        RCVChar = RCREG2;

        //ConsolePut(RCVChar);

        return RCVChar;

    }
    else
    {

         return 0xFF;
    }
}
//****************************************************************************//

 void EthernetTask(void)
 {
     _U08 i,l,Cmd,Param,ByteCount;
     _U32 EEAdr;

     _U08 UartRxBuff[64];

  if(EthernetGetByte() == 0x23)
  {
    if(EthernetGetByte() == 0x5E)
    {
      switch(EthernetGetByte())     // if command
      {
       case 0x00:  {Cmd = 0; Param =0;}   break;             // ?
       case 0x01:  {Cmd = 1; Param =48;}  break;             // almacenar  calendario
       case 0x02:  {Cmd = 2; Param =1;}   break;             // Solicitud de estado de dispositivos (estado y temperatura)
       case 0x03:  {Cmd = 3; Param =1;}   break;             // almacenar horario
       case 0x04:  {Cmd = 4; Param =7;}   break;             // Actualizar fecha y hora
       case 0x05:  {Cmd = 5; Param =0;}   break;             // Descargar  fecha y hora
       case 0x06:  {Cmd = 6; Param =1;}   break;             // Descargar horario
       case 0x07:  {Cmd = 7; Param =1;}   break;             // Descargar Calendario


       case 0x08:  {Cmd = 8; Param =0;}   break;             // borrar calendarios
       case 0x09:  {Cmd = 9; Param =0;}   break;             // borrar horarios



       default:    {Cmd=0xff;Param =0;}   break;
      }





      if(Cmd == 3)
      {
          goto __skip;
      }

      for(i=0;i<Param;i++)
      {
       UartRxBuff[i] = EthernetGetByte();
       ++ByteCount;
      }

      if(EthernetGetByte() == 0x3C)
      {
        if(EthernetGetByte() == 0x3F)
        {

   __skip:
         switch(Cmd)
         {
           case 0:
           {
            LED_1 ^= 1;
            LED_2 ^= 1;

           }
           break;
//------------------------------------------------------------------------------
 //235E010001020304050607080910111213141516171819202122232425262728293031323334353637383940414243444546473C3F

           case 1:  //   Almacenar Calendario   ok
           {
               if(UartRxBuff[0]<128)   // si indice de calendario dentro del rango de dispositivos
               {
                 EEAdr = (_U32)UartRxBuff[0]*CAL_LEN;
            
                 
                 for(j=0;j<46;j++)                    // shift one byte to right the entire array
                 {
                     UartRxBuff[j] = UartRxBuff[j+1];
                 }

                 Eeprom25xx_Write(EEPROM25XX_MEMORY0, UartRxBuff,EEAdr,46);    // almacena calendario

                 ConsolePut(0x23);
                 ConsolePut(0x5E);
                 ConsolePut(0x01);
                 ConsolePut(0x01);
                 ConsolePut(0x3C);
                 ConsolePut(0x3F);

               } 
           }
           break;
//------------------------------------------------------------------------------
           case 2:  //Estado de dispositivos
           {
               if(UartRxBuff[0] == 6)
               {
                ConsolePut(0x23);
                ConsolePut(0x5E);
                ConsolePut(0x02);

                ConsolePut(0x45);  // envia identificador de comando "estado de dispositivos"
                    

                for(j=0;j<16;j++)
                {
                 ConsolePut(myDevicesOutputStatus[j]);
                }

                for(j=0;j<PCDevices;j++)
                {
                 ConsolePut(MyDevicesTemp[j]);
                }

                for(j=PCDevices;j<128;j++)
                {
                 ConsolePut(0);
                }

                ConsolePut(151);
                
                ConsolePut(0x3C);
                ConsolePut(0x3F);
               }
           }
           break;
//------------------------------------------------------------------------------
           case 3:    // recibe horario      ok
           {
                // Return_ByteBit(10, 0);

             UartRxBuff[0] = EthernetGetByte();

             if(UartRxBuff[0] < 128)
             {
               // calcula direccion de almacenamiento
              EEAdr = ((_U32)UartRxBuff[0]*(_U32)HRS_LEN)+((_U32)PCDevices*(_U32)CAL_LEN);

              for(i=0;i<11;i++)
              {

                for(j=0;j<64;j++)
                {
                 UartRxBuff[j] = EthernetGetByte();
                }


                Eeprom25xx_Write(EEPROM25XX_MEMORY0, UartRxBuff,EEAdr, 64);

                EEAdr += 64;
              }


              for(j=0;j<28;j++)
              {
               UartRxBuff[j] = EthernetGetByte();
              }

              Eeprom25xx_Write(EEPROM25XX_MEMORY0, UartRxBuff,EEAdr, 28);

             }


              ConsolePut(0x23);
              ConsolePut(0x5E);
              ConsolePut(0x03);
              ConsolePut(0x01);
              ConsolePut(0x3C);
              ConsolePut(0x3F);


           }
           break;
//------------------------------------------------------------------------------
           case 4:   // Actualizar fecha y hora   ok
           {
              EEAdr = 131064;
              Eeprom25xx_Write(EEPROM25XX_MEMORY0, UartRxBuff,EEAdr, 7);


              ConsolePut(0x23);
              ConsolePut(0x5E);
              ConsolePut(0x04);
              ConsolePut(0x01);
              ConsolePut(0x3C);
              ConsolePut(0x3F);
           }
           break;
//------------------------------------------------------------------------------
           case 5:      //  descargar fecha y hora ok
           {

               EEAdr = 131064;

               Eeprom25xx_Read(EEPROM25XX_MEMORY0, UartRxBuff,EEAdr,7);

               ConsolePut(0x23);
               ConsolePut(0x5E);
               ConsolePut(0x05);

               for(j=0;j<7;j++)
               {
                ConsolePut(UartRxBuff[j]);
               }

               ConsolePut(0x3C);
               ConsolePut(0x3F);


               /*
             if((UartRxBuff[0] <32)&&(UartRxBuff[1] < 12))
             {
                   Return_ByteBit(UartRxBuff[0], UartRxBuff[1]);

                   //Return_ByteBit(10, 0);

                   ConsolePut(ByteBitVal.Byte);
                   ConsolePut(ByteBitVal.Bit);
                   ConsolePut(0x3F);
              }
             */

           }
           break;
//------------------------------------------------------------------------------
           case 6:   // descargar horario    235E06XX3C3F    ok
           {
            if(UartRxBuff[0] <128)
            {
               // calcula direccion de lectura
              EEAdr = ((_U32)UartRxBuff[0]*(_U32)HRS_LEN)+((_U32)PCDevices*(_U32)CAL_LEN);

              ConsolePut(0x23);
              ConsolePut(0x5E);
              ConsolePut(0x06);

              for(i=0;i<11;i++)
              {
                Eeprom25xx_Read(EEPROM25XX_MEMORY0, UartRxBuff,EEAdr,64);
                EEAdr += 64;

                for(l=0;l<64;l++)
                {
                  ConsolePut(UartRxBuff[l]);
                }
              }

               Eeprom25xx_Read(EEPROM25XX_MEMORY0, UartRxBuff,EEAdr,28);

              for(j=0;j<28;j++)
              {
                  ConsolePut(UartRxBuff[j]);
              }

               ConsolePut(0x02);
               ConsolePut(0xE3);

               ConsolePut(0x3C);
               ConsolePut(0x3F);

            }
           }
           break;
//------------------------------------------------------------------------------

           case 7:   // Descargar calendario   235E07XX  3C3F   ok
           {

             if(UartRxBuff[0] <128)
             {
                  // calcula direccion de lectura
              EEAdr = ((_U32)UartRxBuff[0]*(_U32)CAL_LEN);

              ConsolePut(0x23);
              ConsolePut(0x5E);
              ConsolePut(0x07);

              Eeprom25xx_Read(EEPROM25XX_MEMORY0, UartRxBuff,EEAdr,46);

              for(j=0;j<46;j++)
              {
                  ConsolePut(UartRxBuff[j]);
              }

              ConsolePut(0x35);
              ConsolePut(0x3C);
              ConsolePut(0x3F);

             }

           }
           break;

//------------------------------------------------------------------------------

           case 8:  // Borrar todos los calendarios     235E083C3F   ok
           {

               EEAdr = 0;                 // 5888 bytes to erase

            for(i=0;i<64;i++)
            {
                UartRxBuff[i] = 0x00;
            }

            for(i=0;i<92;i++)
            {
                Eeprom25xx_Write(EEPROM25XX_MEMORY0, UartRxBuff,EEAdr, 64);
                EEAdr += 64;
            }


            ConsolePut(0x23);
            ConsolePut(0x5E);
            ConsolePut(0x08);
            ConsolePut(0x01);
            ConsolePut(0x3C);
            ConsolePut(0x3F);

           }
           break;

           //-------------------------------------------------------------------

           case 9:    
           {
             


           }
           break;
//------------------------------------------------------------------------------

             case 10:  // carga estado para enviar a dispositivos
             {
                 QueryCalendars();


             for(i=0;i<16;i++)
             {
               ConsolePut(myDevicesRequiredStatus[i]);
             }

             ConsolePut(0x3F);
             }
             break;

           //-------------------------------------------------------------------

           default:
           {

           }
           break;
          }

        }
      }

    }
  }
 }
//----------------------------------------------------------------------------//

void  WirelessTxRx(void)
 {
     Timer1Tick();

     switch(PanCoordStateMachine)
     {

         //  envia estados a todos los dispositivos (broadcast)

         case 0:
         {
             if( MiApp_MessageAvailable())
             {
                 MiApp_DiscardMessage();
             }

             ++MyTrackingNumber;


             if(myvar == 0)
             {
                 myvar = 1;

              for(j=0;j<16;j++)
              {
               myDevicesRequiredStatus[j] = 0;
              }


             }
             else
             {
                 myvar = 0;

                 for(j=0;j<16;j++)
                 {
                  myDevicesRequiredStatus[j] = 0xff;
                 }

             }





             MiApp_FlushTx();
             MiApp_WriteData(' ');                 // 0
             MiApp_WriteData(myNodeNumber);        // 1
             MiApp_WriteData(MyTrackingNumber);    // 2
             MiApp_WriteData(OutputStat);          // 3
             MiApp_WriteData(BaseTimeToTx);        // 4

             for(j=0;j<16;j++)
             {
              MiApp_WriteData(myDevicesRequiredStatus[j]);
             }

             MiApp_BroadcastPacket(FALSE);
             
             LED_1 ^= 1;
             SwTimer4 = 0;
             PanCoordStateMachine = 1;


             



         }
         break;
//------------------------------------------------------------------------------
         // espera respuesta de cada dispositivo

         case 1:
         {
             if( MiApp_MessageAvailable())
             {
              if((rxMessage.Payload[1] <= PCDevices)&&((rxMessage.Payload[3] == OutputStat))&&((rxMessage.Payload[4] <= 255))&&((rxMessage.Payload[5] <= 150)))
              {
                  MyDevicesTemp[rxMessage.Payload[1]] = rxMessage.Payload[5];   // get temperature value

                  // Dispositivo entre 0 y 7
                  if(rxMessage.Payload[1] <8)
                  {
                      SET_8BIT(ConnectedDevices[0],rxMessage.Payload[1]);

                      if(rxMessage.Payload[4] == 1)
                      {
                          SET_8BIT(myDevicesOutputStatus[0],rxMessage.Payload[1]);
                      }
                      else
                      {
                          if(rxMessage.Payload[4] == 0)
                         {
                          CLEAR_8BIT(myDevicesOutputStatus[0],rxMessage.Payload[1]);
                         }
                      }
                  }
                  else // dispositivo entre 8 y 127
                  {
                      // bit 0
                      if(rxMessage.Payload[1]%8 == 0)
                      {
                          j = rxMessage.Payload[1]/8;

                          SET_8BIT(ConnectedDevices[j],0);

                          if(rxMessage.Payload[4] == 1)
                          {
                              SET_8BIT(myDevicesOutputStatus[j],0);
                          }
                          else
                          {
                              if(rxMessage.Payload[4] == 0)
                              {
                                  CLEAR_8BIT(myDevicesOutputStatus[j],0);
                              }
                          }
                      }
                      else
                      {  // cualquier otro bit

                          j = rxMessage.Payload[1]/8;
                          i = rxMessage.Payload[1]%8;

                          SET_8BIT(ConnectedDevices[j],i);

                          if(rxMessage.Payload[4] == 1)
                          {
                              SET_8BIT(myDevicesOutputStatus[j],i);
                          }
                          else
                          {
                           if(rxMessage.Payload[4] == 0)
                           {
                             CLEAR_8BIT(myDevicesOutputStatus[j],i);
                           }
                          }
                      }
                  }
                  

                  //ConsolePutROMString( (ROM char * const) "  \r\n  Temperature NODE" );
                  //PrintDec(rxMessage.Payload[1]);
                  //ConsolePutROMString( (ROM char * const) ": " );
                  //PrintDec(rxMessage.Payload[5]);
                  //ConsolePutROMString( (ROM char * const) "C" );
                  //ConsolePutROMString( (ROM char * const) "  \r\n" );
                  
              }

              MiApp_DiscardMessage();
             }

             // if((ConnectedCoordinators == 3)||(SwTimer4 >= 10))
          if(SwTimer4 >= 20)
          {

           for(j=0;j<16;j++)
           {
            for(i=0;i<8;i++)
            {
             if(QUERY_8BIT(ConnectedDevices[j],i))
             {
              //ConsolePutROMString( (ROM char * const) "1" );
             }
             else
             {
              //ConsolePutROMString( (ROM char * const) "0" );
             }
            }
            //ConsolePutROMString( (ROM char * const) "-" );

           }


           for(j=0;j<16;j++)
           {
            ConnectedDevices[j]= 0;
           }

            PanCoordStateMachine = 0;
          }

         }
         break;
//------------------------------------------------------------------------------
         case 2:
         {

         }
         break;
//------------------------------------------------------------------------------
         case 3:
         {

         }
         break;
//------------------------------------------------------------------------------
         case 4:
         {

         }
         break;
//------------------------------------------------------------------------------
         default:
         {
          PanCoordStateMachine = 0;
         }
         break;
     }
 }

//********************************************************************//

 void  TaskScheduler(void)
 {
     Timer1Tick();

     switch(PanCoordStateMachine)
     {

         // Resincronizacion de coordinadores < Broadcast Resync packet>
         case 0:         
         {
             if( MiApp_MessageAvailable())
             {
                 MiApp_DiscardMessage();
             }


             ++MyTrackingNumber;

             MiApp_FlushTx();
             MiApp_WriteData(' ');
             MiApp_WriteData(myNodeNumber);
             MiApp_WriteData(MyTrackingNumber);
             MiApp_WriteData(Resync);
             MiApp_WriteData(BaseTimeToTx);

             MiApp_BroadcastPacket(FALSE);


             LED_1 ^= 1;

             PanCoordStateMachine = 1;
             ConnectedCoordinators = 0;
             SwTimer4 = 0;

         }
         break;

//------------------------------------------------------------------------------

         // Resincronizacion de coordinadores < espera respuesta de coordinadores>
         case 1:
         {
           if( MiApp_MessageAvailable())
           {
             if((rxMessage.Payload[3] == Resync)&&((rxMessage.Payload[2] == MyTrackingNumber))&&((rxMessage.Payload[1] <= 8)))
             {
                 SET_8BIT(ConnectedCoordinators,rxMessage.Payload[1]-1);
                 LED_2 ^= 1;
             }

             MiApp_DiscardMessage();
           }

         // if((ConnectedCoordinators == 3)||(SwTimer4 >= 10))
           if(SwTimer4 >= 15)
          {
            PanCoordStateMachine = 2;
          }
         }
         break;
//------------------------------------------------------------------------------
         // envia pkt de actualizacion a end devices pertenecientes al PANCOORD

         case 2:
         {
          
              MiApp_SetChannel(PanCoordToEndDevChan);    // switch channel to my End Devices

             MiApp_FlushTx();
             MiApp_WriteData(' ');
             MiApp_WriteData(myNodeNumber);
             MiApp_WriteData(MyTrackingNumber);
             MiApp_WriteData(OutputStat);
             MiApp_WriteData(BaseTimeToTx);

             for(j=0;j<8;j++)
             {
              MiApp_WriteData(myDevicesOutputStatus[j]);
             }

             MiApp_BroadcastPacket(FALSE);

             LED_1 ^= 1;
             SwTimer4 = 0;
             PanCoordStateMachine = 3;
         }
         break;

//------------------------------------------------------------------------------
         case 3:
         {
          if( MiApp_MessageAvailable())
          {
              //MiApp_WriteData(' ');
              //MiApp_WriteData(myNodeNumber);
              //MiApp_WriteData(MyTrackingNumber);
              //MiApp_WriteData(OutputStat);
              //MiApp_WriteData(MyStat);
              //MiApp_WriteData(MyTemp);

              if((rxMessage.Payload[1] <= PCDevices)&&((rxMessage.Payload[3] == OutputStat))&&((rxMessage.Payload[4] <= 255))&&((rxMessage.Payload[5] <= 150)))
              {
                  ConsolePutROMString( (ROM char * const) "  \r\n NODE" );
                  PrintDec(rxMessage.Payload[1]);
                  ConsolePutROMString( (ROM char * const) ": " );
                  PrintDec(rxMessage.Payload[5]); 
                  ConsolePutROMString( (ROM char * const) "C" );

              }
             
              MiApp_DiscardMessage();
          }

           if(SwTimer4 >= 8)
           {
             SwTimer4 = 0;
             PanCoordStateMachine = 4;
           }


         }
         break;
//------------------------------------------------------------------------------
         case 4:
         {
          if(SwTimer4 >= 2)
          {
            MiApp_SetChannel(PanCoordToCoordChan);    // switch channel to Coordinators Channel
            PanCoordStateMachine = 0;
          }
         }
         break;
//------------------------------------------------------------------------------

         default:
         {

         }
         break;
     }


 }

//****************************************************************************//

 // Devuelve byte y bit correspondiente al dia y mes que se de como parametros

  void Return_ByteBit(_U08 dia, _U08 mes)
  {
      _U08 DiasMes[12] = {31,29,31,30,31,30,31,31,30,31,30,31}; //
      _U16 Dias = 0;


      for(i=0;i<mes;i++)
      {
          Dias += DiasMes[i];
      }

      Dias += dia;

      if(Dias > 0)
      {

      if(Dias < 9)
      {
          ByteBitVal.Byte = 0;
          ByteBitVal.Bit  = Dias-1;
      }
      else
      {
          ByteBitVal.Byte = (Dias-1)/8;

         if(Dias%8 == 0)
         {
          ByteBitVal.Bit = 7;
        
         }
         else
         {
          ByteBitVal.Bit  = (Dias%8)-1;
         }
      }
  }
   else
   {
     ByteBitVal.Bit = 0xFF;
     ByteBitVal.Byte = 0xFF;
   }

  }
//****************************************************************************//


 // Toma de el calendario de cada dispositivo el bit correspondiente al dia de hoy y
 // lo pone en el arreglo myDevicesRequiredStatus
 void QueryCalendars(void)
 {
     _U08 k;
     _U08 Today = 0;
     _U16 EEAdr;
     _U08 _Bit  = 0;
     _U08 _Byte = 0;

     for(j=0;j<16;j++)
     {
        myDevicesRequiredStatus[j] = 0;
     }

     Return_ByteBit(1,0);


     for(k=0;k<PCDevices;k++)   // loop trought calendars (devices)
     {
        EEAdr = ((_U16)k*CAL_LEN)+ ByteBitVal.Byte;
        Eeprom25xx_Read(EEPROM25XX_MEMORY0, &Today,EEAdr,1);


        if(QUERY_8BIT(Today,ByteBitVal.Bit))
        {
            SET_8BIT(myDevicesRequiredStatus[_Byte],_Bit);
        }
        else
        {
            CLEAR_8BIT(myDevicesRequiredStatus[_Byte],_Bit);
        }

        if(_Bit < 7)
        {
            ++_Bit;
        }
        else
        {
            ++_Byte;
            _Bit = 0;
        }
     }

 }
//****************************************************************************//
 //Devuelve 1 si es tiempo de revisar los calendarios
 
 _U08 TimeToQueryCalendars(void)
 {
     if(SwTimer3 > 60)
     {
         SwTimer3 = 0;
         return 0;
     }
     else
     {
         SwTimer3 = 0;
         return 0;
     }
     
 }
//****************************************************************************//

 void CalendarTask(void)
 {
     if(TimeToQueryCalendars()==1)
     {
         QueryCalendars();
     }



 }

 //***************************************************************************//

void main(void)

{
     #define BAUDRG 77

    //BYTE SecNum = 0;
    /*******************************************************************/
    // Initialize the system
    /*******************************************************************/

    ANCON0 = 0XFF;     /*desactiva entradas analogicas*/
    ANCON1 = 0XFF;     /*desactiva entradas analogicas*/

    PPSUnLock();

    if(myNodeNumber == 0)
    {
       //PPSOutput(PPS_RP6, PPS_TX2CK2);    // TX2 RP17/RC6   ethernet
       //PSInput(PPS_RX2DT2, PPS_RP5);     // RX2 RP18/RC7

      PPSOutput(PPS_RP10, PPS_TX2CK2);    // TX2 RP17/RC6     icsp
      PPSInput(PPS_RX2DT2, PPS_RP9);     // RX2 RP18/RC7
    }
    else
    {
     PPSOutput(PPS_RP10, PPS_TX2CK2);    // TX2 RP17/RC6     icsp
     PPSInput(PPS_RX2DT2, PPS_RP9);     // RX2 RP18/RC7
    }

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



     LED_1 = 1;
     LED_2 = 0;

     RLY_1 = 0;
     RLY_2  = 0;

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

     for(j=0;j<16;j++)
     {
      ConnectedDevices[j]= 0;
      myDevicesOutputStatus[j] = 0;
     }

     for(j=0;j<PCDevices;j++)
     {
         MyDevicesTemp[j] = 0;
     }

     Eeprom25xx_Init(EEPROM25XX_MEMORY0, EEPROM25XX_1M);          // inicializa memoria 0, 128 kbits


     while(1)
     {

      //TaskScheduler();

      WirelessTxRx();
      //EthernetTask();
      CalendarTask();

      //DelayMs(250);
     }

}
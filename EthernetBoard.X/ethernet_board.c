/*********************************************************************
 *
 *                  PLAYA BONITA PIC32 ETHERNET BOARD
 * 
 * Tarjeta con micro de 32 bits (Firmware PlayaBonitaEthernetBoard), no funciona 
 * el radio MRF89XAM9A, por lo quese utiliza una wireless board pequeña de las 
 * que se utiliza en la UNISON, que lleva el firmware PlayaBonita_RadioBoard.
 * Se comunican a traves de un serial. La tarjeta ethernet utiliza un puerto 
 * expansión J2 y la tarjeta radio utiliza el puerto ICSP. con la siguente 
 * configuracion:
 * 
 * Ethernet J2      Radio ICSP
 * 1:VCC            2:VCC
 * 3:TXD            5:PGC
 * 4:RXD            4:PGD
 * 5:GND            3:GND
 *
 *********************************************************************
 * FileName:        ethernet_board.c
 * Dependencies:	p32xxxx.h
 *
 *
 * Processor:       PIC32
 *
 * Complier:        MPLAB C32
 *                  MPLAB IDE v8.0+
 * Company:         Microchip Technology Inc.
 *
/////////////////////////////////////////////////////////////////////////////////////
/*  Direccionamiento de memoria externa                                             */
/*                                                                                  */
/* 0 - 130047    espacio reservado para almacenar hasta 10000 eventos de reservacion*/
/* paginas 0 - 429                                                                  */
/* fin de pagina 429 = 110079*/
/*                                                                                  */
/* espacio reservado para calibraciones 256 bytes: pagina 508                       */
/*    inicio calibraciones =  130,048                                               */
/*    Fin calibraciones    =  130,303                                               */
/*                                                                                  */
/* espacio reservado para almacenar dispositivos controlados manualmente DCM        */
/*  inicio: 130,304                                                                 */
/*  fin:    130,320                                                                 */
/*                                                                                  */
/* espacio reservado para almacenar estado de disp contr manualmente EDCM           */
/*   inicio: 130,321                                                                */
/*   fin:    130,337                                                                */
/*                                                                                  */
/* espacio reservado para almacenar valores de tolerancia al encendido y al apagado */
/*   Inicio: 130338                                                                 */
/*   Fin:    130339                                                                 */
/*                                                                                  */
/* espacio reservado para almacenar dispositivos que tienen reservacion confirmada  */
/* Se almacena un bit en alto por cada dispositivo con reserv confirmada y al final */
/* se almacena fecha y hora de borrado de reservaciones                             */
/*   Inicio: 130340                                                                 */
/*   FIN:    130362                                                                 */
/*                                                                                  */
/* Espacio reservado para almacenar BaseTimeToTx                                    */
/*    DIRECCION:   130363 - 130364                                                  */
/*                                                                                  */
/*                                                                                  */
/*                                                                                  */
////////////////////////////////////////////////////////////////////////////////
 
/*    PENDIENTES DE AGREGAR O PULIR
**  
**    1.- Reducir el cuello de botella causado por la lectura de la memoria eeprom en busca de reservaciones activas
**    2.- Implementar mecanismo para impedir la confirmacion de las reservaciones que no se confirmaron en su tiempo 
**        de tolerancia.
**    3.- Implementar mecanismo para "recordar" las reservaciones en curso que esten confirmadas despues de hacer
**        una sincronizacion de reservaciones en el software 
** 
**
** Run this code on bridge board with PIC32MX.
** 
** 
*/

#include <p32xxxx.h>
#include <plib.h>  
#include <peripheral/pps.h>
//#include "ConfigApp.h"
//#include "MCHP_API.h"
//#include "NVM.h"
//#include "MAC_EEProm.h"

#include "Console.h"
#include"types.h"


//#include "BSP_MiWiDemoBd.h"

//#include <peripheral/ports.h>

// Config settings
// POSCMOD = HS, FNOSC = PRIPLL, FWDTEN = OFF
// PLLIDIV = DIV_2, PLLMUL = MUL_16
// PBDIV = 8 (default)
// Main clock = 8MHz /2 * 16    = 80MHz
// Peripheral clock = 80MHz /8  =  10MHz

// Configuration Bit settings
// SYSCLK = 80 MHz (8MHz Crystal/ FPLLIDIV * FPLLMUL / FPLLODIV)
// PBCLK = 10 MHz
// Primary Osc w/PLL (XT+,HS+,EC+PLL)
// WDT OFF
// Other options are don't care
//
#pragma config FPLLMUL = MUL_24, FPLLIDIV = DIV_3, FPLLODIV = DIV_8, FWDTEN = ON
#pragma config POSCMOD = HS, FNOSC = PRIPLL, FPBDIV = DIV_1, WDTPS = PS131072      

// #pragma config  PS1048576
// #pragma config  PS524288
// #pragma config  PS262144
// #pragma config  PS131072
// #pragma config  PS65536
// #pragma config  PS32768
// #pragma config  PS16384
// #pragma config  PS8192
// #pragma config  PS4096
// #pragma config  PS2048
// #pragma config  PS1024
// #pragma config  PS512


#define FOSC                    12E6   
#define TOGGLES_PER_SEC         40   
#define CORE_TICK_RATE         (FOSC/2/TOGGLES_PER_SEC)  

#define SYSCLK	(FOSC)
#define PBCLK  (SYSCLK)
#define Fsck	50000 
#define BRG_VAL 	((PBCLK/2/Fsck)-2)

//#define PPSUnLock() {SYSKEY=0x0;SYSKEY=0xAA996655;SYSKEY=0x556699AA;CFGCONbits.IOLOCK=0;}
//#define PPSLock() {SYSKEY=0x0;SYSKEY=0xAA996655;SYSKEY=0x556699AA;CFGCONbits.IOLOCK=1;}

#define ST_Bit         7   // start oscillator bit
#define TimeFormat_Bit 6
#define AmPm_Bit       5       
#define OSCRUN         5   // oscillator status bit
#define VBATEN         3   // VBAT enable bit
#define PWRFAIL        4   // power fail bit - always writes to 0
#define LPYR           5   // leap year bit

// time vars

                     //         Datos en BCD
typedef struct      //Definicion de Tipo Hora y Fecha
{
BYTE segundos;       //Dato de Segundo   00-59
BYTE minutos;        //Dato de Minuto    00-59
BYTE horas;          //Dato de Hora      00-23
BYTE dia_sem;        //Dato del Dia de la Semana 01-07
BYTE dia_mes;        //Dato del Día del Mes      01-31
BYTE mes;            //Dato del Numero de Mes    01-12
BYTE anio;           //Dato del Numero de Año    00-99
}HoraFecha;


HoraFecha Time;



#if ADDITIONAL_NODE_ID_SIZE > 0
    BYTE AdditionalNodeID[ADDITIONAL_NODE_ID_SIZE] = {0x08};
#endif

extern BYTE myLongAddress[];

#define C_CHANNEL                 16
#define PCDevices                 64
#define BroadcastNewDevicesStatus 1
#define BroadcastConnectedDevices 2

#define AdrFinReserv         129999
#define AdrDCM               130304        
#define AdrEDCM              130321

#define AdrBaseTimeToTx      130363

_U08 EnabledDevices[PCDevices/8];   // dispositivos a los que habra que revisar horario (dispositivos que no se encuentran controlados manualmente)
_U08 myDevicesRequiredStatus[16];   // estados a enviar a dispositivos
_U08 myDevicesOutputStatus[16];     // estados actuales de dispositivos (promediado)
_U08 ConnectedDevices[16];          // dispositivos que esta escuchando el PANCOORDINATOR (promediado)

_U08 myDevicesOutputStatusBuff0[16];     // estados actuales de dispositivos (Buffer 1)
_U08 ConnectedDevicesBuff0[16];          // dispositivos que esta escuchando el PANCOORDINATOR (Buffer 1)

_U08 myDevicesOutputStatusBuff1[16];     // estados actuales de dispositivos (Buffer 2)
_U08 ConnectedDevicesBuff1[16];          // dispositivos que esta escuchando el PANCOORDINATOR (Buffer 2)

_U08 ConfirmedDvices[PCDevices/8];
_U16 ConfirmedReserv[PCDevices];
_U16 U16Index;

_U08 EDCR[16];       // Dispositivos que deberian estar encendidos de acuerdo a las reservaciones activas en este momento 


_U08 MyDevicesTempBuff0[PCDevices];               // Temperatura enviada por cada dispositivo
_U08 MyDevicesTempBuff1[PCDevices];               // Temperatura enviada por cada dispositivo

_U08 OffsetMyDevicesTemp[PCDevices];         // ajuste de temperatura para cada dispositivo
_U08 OffsetAppliedMyDevicesTemp[PCDevices];  // temperatura con ajuste aplicado 

_U08 Tick,TickScaler;
_U08 SwTimer0,SwTimer1,SwTimer2,SwTimer3,SwTimer4;
_U16 EepromTimer;
_U08 BaseTimeToTx = 4;
_U08 PanCoordStateMachine;

const _U08 StartCmd              = 0x23; 
const _U08 EndCmd                = 0x3F;
const _U08 OutputStat            = 0x0B;
const _U32 CalibrationAddress    = 130048;
const _U32 ToleranceValueAddress = 130338;

_U08 Uart1RXState;
_U08 RcvNodeNumber,DevStatusTemp,RxCmdNum;

_U08 myChannel = C_CHANNEL;
_U08 i, j;

_U08  UartRxBuff[1024]; 
_U08  EepromBuff[1024];
_U08  ShortEepromBuff[16];
_U32  EepromSearchIndex;
_U16  UartRxBuffIndex;

_U16 _U16Var;
_U32 U32EEADR;
_U32 U32EEIndex;

_U08 BuffNumber;
_U08 DateTimeChars[5];
_U08 Tolerance;                   // suma de los minutos de tolerancia de encendido y apagado
_U32 NMinutes;     //hora fecha en este momento
_U32 SMinutes;     //hora fecha de inicio de reservacion
_U32 EMinutes;     //hora fecha de fin de reservacion


void EepromTask(void);
void Reservations_And_ManuallyControlledDevices(void);
_U08 AddMinutesToDateTime(_U08 Minutes, _U08 *DateTimeString, _U08 *NTimeString);

////////////////////////////////////////////////////////////////////////////////
ROM BYTE U1CharacterArray[]={'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

char Message[] = "Hello !!!";

////////////////////////////////////////////////////////////////////////////////
void Serial_print(char *buffer)
{
   while(*buffer != (char)0)
   {
      while(!UARTTransmitterIsReady(UART1));
      UARTSendDataByte(UART1, *buffer++);
   }
    while(!UARTTransmissionHasCompleted(UART1));
   UARTSendDataByte(UART1, '\r');
   UARTSendDataByte(UART1, '\n'); 
}


//------------------------------------------------------------------------------

void Uart1Put(BYTE c)
{
    while(U1STAbits.TRMT == 0);
    U1TXREG = c;
}

//------------------------------------------------------------------------------
void Uart1PrintChar(BYTE toPrint)
{
    BYTE PRINT_VAR;
    PRINT_VAR = toPrint;
    toPrint = (toPrint>>4)&0x0F;
    Uart1Put(U1CharacterArray[toPrint]);
    toPrint = (PRINT_VAR)&0x0F;
    Uart1Put(U1CharacterArray[toPrint]);
    return;
}

//------------------------------------------------------------------------------

void Uart1PrintDec(BYTE toPrint)
{
    Uart1Put(U1CharacterArray[toPrint/10]);
    Uart1Put(U1CharacterArray[toPrint%10]);
}
//------------------------------------------------------------------------------

void Uart1PutROMString(ROM char *str)
{
    BYTE c;

    while( (c = *str++) )
        Uart1Put(c);
}

////////////////////////////////////////////////////////////////////////////////

/*25AA1024 Driver functions*/

/**---------------------------------------------------------------------------------------------
      \def        _eEEPROM25XX_STATUS
      \brief      Mascaras que permiten leer los valores del registro de estatus interno de la memoria
    ----------------------------------------------------------------------------------------------*/
    typedef enum
    {
        EEPROM25XX_BUSY     = 0x01,
        EEPROM25XX_WREN     = 0x02
    }_eEEPROM25XX_STATUS;

#define READ            0x03  /*Read data from memory array beginning at selected address*/
#define WRITE           0x02  /*Write data to memory array beginning at selected address*/
#define WRDI            0x04  /*Reset the write enable latch (disable write operations)*/
#define WREN            0x06  /*Set the write enable latch (enable write operations)*/
#define RDSR            0x05  /*Read STATUS register*/
#define WRSR            0x01  /*Write STATUS register*/
#define DUMMY           0x55  /*Dummy byte for read operations*/
#define SE              0xD8  /*Sector Erase*/


/**-----------------------------------------------------------------------------------------------*/
void Eeprom25xx_Init(void)
{
  EE_nCS_TRIS     = 0;     /* pin ss como salida*/   
  EE_nCS          = 1;     /* ss en alto (deshabilitado)*/
}
/**-----------------------------------------------------------------------------------------------*/
_U08 Eeprom25xx_u8ReadStatus(void)
{
    BYTE u8Status;

    EE_nCS          = 0;
    SPIPut2(RDSR);
    u8Status = SPIGet2();
    EE_nCS          = 1;

    return u8Status;
}
/**-----------------------------------------------------------------------------------------------*/
void Eeprom25xx_Read(_U08 *pu8Buffer, _U32 u32Addr, _U16 u16Size)
{
    _U16 i;

    EE_nCS          = 0;
    SPIPut2(READ);
    SPIPut2((_U08)(u32Addr>>(_U32)16));         /*parte alta de la direccion de la memoria*/
    SPIPut2((_U08)(u32Addr>>(_U32)8));          /*parte alta de la direccion de la memoria*/
    SPIPut2((_U08)u32Addr);                     /*parte baja de la direccion de la memoria*/
  
    for(i=0;i<u16Size;i++)
    {
        *pu8Buffer++ = SPIGet2();        
    }
    EE_nCS          = 1;
}
/**-----------------------------------------------------------------------------------------------*/
void Eeprom25xx_Write(_U08 *pu8Buffer, _U32 u32Addr, _U16 u16Size)
{
    _U16 i;
    _U08 u08PageCounter = 0;

    i = u16Size; // cantidad de datos a escribir

    NEXT_PAGE:

    while((Eeprom25xx_u8ReadStatus() & EEPROM25XX_BUSY) == 1);

    EE_nCS = 0;
    SPIPut2(WREN);            /*mandamos comando de write enable*/
    EE_nCS = 1;
    
    DelayMs(1);

    EE_nCS = 0;
    SPIPut2(WRITE);           /*mandamos comando write*/
    SPIPut2((_U08)(u32Addr>>(_U32)16));         /*parte alta de la direccion de la memoria*/
    SPIPut2((_U08)(u32Addr>>(_U32)8));          /*parte alta de la direccion de la memoria*/
    SPIPut2((_U08)u32Addr);                     /*parte baja de la direccion de la memoria*/
   
    ////////////////////////////////////////////////////////////////////////////
    u08PageCounter = 0;
    while(i>0)
    {
        SPIPut2(*pu8Buffer);        /*escribimos un byte*/
        pu8Buffer++;
        i--;
        u08PageCounter++;
        if( ((u32Addr + u08PageCounter) & (255)) == 0 )
              {
               EE_nCS = 1;
               u32Addr += u08PageCounter;

               goto NEXT_PAGE;
              }
    }

   /////////////////////////////////////////////////////////////////////////////

    EE_nCS          = 1;
}
/**---------------------------------------------------------------------------*/

/*Inicio Sector 0:  00000h   FIN:  7FFFh */
/*Inicio Sector 1:   8000h   FIN: 0FFFFh */
/*Inicio Sector 2:  10000h   FIN: 17FFFh */
/*Inicio Sector 3:  18000h   FIN: 1FFFFh */

void Eeprom25xx_SectorErase(_U32 u32Addr)
{
    while((Eeprom25xx_u8ReadStatus()& EEPROM25XX_BUSY) == 1);
    
    EE_nCS = 0;
    SPIPut2(WREN);             /*mandamos comando de write enable*/
    EE_nCS = 1;

    EE_nCS = 0;
    SPIPut2(SE);
    
    SPIPut2((_U08)(u32Addr>>(_U32)16));        /*parte alta de la direccion de la memoria*/
    SPIPut2((_U08)(u32Addr>>(_U32)8));         /*parte alta de la direccion de la memoria*/
    SPIPut2((_U08)u32Addr);                    /*parte baja de la direccion de la memoria*/
    EE_nCS = 1;
}

/*------------------------------------------------------------------------------------------------*/

void SendCmdToRadio(_U08 Cmd)
{
    switch(Cmd)
    {
        
        case BroadcastNewDevicesStatus:
        {
            Uart1Put(0x23);
            Uart1Put(0x5E);
            Uart1Put(0x01);
            Uart1Put(BaseTimeToTx);
            
            for(j=0;j<16;j++)
            {
              Uart1Put(myDevicesRequiredStatus[j]);
            }
            
            Uart1Put(0x3C);
            Uart1Put(0x3F);
        }
        break;
        
        case BroadcastConnectedDevices:
        {
            Uart1Put(0x23);
            Uart1Put(0x5E);
            Uart1Put(0x02);
            
            for(j=0;j<16;j++)
            {
              Uart1Put(ConnectedDevices[j]);
            }
            
            Uart1Put(0x3C);
            Uart1Put(0x3F);
        }
        break;
        
        case 3:
        {
            Uart1Put(0x23);
            Uart1Put(0x5E);
            Uart1Put(0x03);
            Uart1Put(BaseTimeToTx);   
            Uart1Put(0x3C);
            Uart1Put(0x3F);
        }
        break;
        
        default: break;
    }
}
/*------------------------------------------------------------------------------------------------*/
/**-----------------------------------------------------------------------------------------------*/
/**-----------------------------------------------------------------------------------------------*/
//----------------------------------------------------------------------------//
void Write_rtcc(BYTE rtcc_reg, BYTE time_var)
{
 StartI2C1();                 // start I2C communication: SDA goes down while SCL remains high
 IdleI2C1();	              //Wait to complete
 MasterWriteI2C1(0xDE);       // send the RTCC's address for write = 0xde
 IdleI2C1();	              //Wait to complete
 MasterWriteI2C1(rtcc_reg) ;  // send the register's address
 IdleI2C1();	              //Wait to complete
 MasterWriteI2C1 (time_var) ; // send the data byte
 IdleI2C1();	              //Wait to complete
 StopI2C1();                  // stop I2C communication: SDA goes high while SCL remains high
 IdleI2C1();	              //Wait to complete
 //DelayMs(5);
}

 

BYTE Read_rtcc(BYTE rtcc_reg)
{
 BYTE data;
 StartI2C1();                 // start I2C communication: SDA goes down while SCL remains high
 IdleI2C1();	              //Wait to complete
 MasterWriteI2C1(0xDE);       // send the RTCC's address for write = 0xde
 IdleI2C1();	              //Wait to complete
 MasterWriteI2C1(rtcc_reg);   // send the register's address
 IdleI2C1();	              //Wait to complete
 StartI2C1();                 // switch to reads
 IdleI2C1();	              //Wait to complete
 MasterWriteI2C1(0xDF);       // send the RTCC's address for read = 0xdf
 IdleI2C1();	              //Wait to complete
 data = MasterReadI2C1();     // read the byte from the RTCC (register's content)
 IdleI2C1();	              //Wait to complete
 NotAckI2C1();
 IdleI2C1();	              //Wait to complete
 StopI2C1();                  // stop I2C communication: SDA goes high while SCL remains high
 IdleI2C1();	              //Wait to complete
 //DelayMs(1);
 return(data);
}
//----------------------------------------------------------------------------// 
//----------------------------------------------------------------------------// 
BOOL Set_Time(void)
{
  BOOL _ack = 0;
  // comprobacion de formato correcto
  if((((Time.segundos)&(0x7f))<= 0x60)&&(((Time.minutos)&(0x7f))<= 0x60)&&(((Time.horas)&(0x3f))<= 0x24)&&(((Time.dia_sem)&(0x07))<= 0x07)&&(((Time.dia_mes)&(0x3f))<= 0x31)&&(((Time.mes)&(0x1f))<= 0x12)&&(((Time.anio)&(0xff))<= 0x99))
  {
   CLEAR_8BIT(Time.segundos, ST_Bit);        // oscillator disabled 
   SET_8BIT(Time.dia_sem,VBATEN);            // habilita bateria de respaldo
   Write_rtcc(0,Time.segundos);
   CLEAR_8BIT(Time.horas,TimeFormat_Bit);    //formato 24 hrs
   
   Write_rtcc(0,Time.segundos);
   Write_rtcc(1,Time.minutos);
   Write_rtcc(2,Time.horas);
   Write_rtcc(3,Time.dia_sem);
   Write_rtcc(4,Time.dia_mes);
   Write_rtcc(5,Time.mes);
   Write_rtcc(6,Time.anio);
   
   SET_8BIT(Time.segundos, ST_Bit);        //  oscillator enabled
   Write_rtcc(0,Time.segundos);
   
   _ack = 1;
  }
  
  return _ack;
   
}


void Read_Time(void)
{
   Time.segundos=Read_rtcc(0);
   Time.minutos =Read_rtcc(1);
   Time.horas   =Read_rtcc(2);
   Time.dia_sem =Read_rtcc(3);
   Time.dia_mes =Read_rtcc(4);
   Time.mes     =Read_rtcc(5);
   Time.anio    =Read_rtcc(6);
   
   //CLEAR_8BIT(Time.segundos,7);
   CLEAR_8BIT(Time.horas,TimeFormat_Bit);
   CLEAR_8BIT(Time.horas,7);
   
  //JL:  Faltaba borrar el bit de año bisiesto:
   CLEAR_8BIT(Time.mes,LPYR);
   
   Time.dia_sem &= 0x07;
   Time.segundos &= 0x7F;
  
}
//----------------------------------------------------------------------------//
/*Esta funcion requiere que la cadena que contiene la fecha hora este en formato:
 *
 * DateTimeString[Offset]     = Minutos
 * DateTimeString[Offset+1]   = Horas 
 * DateTimeString[Offset+2]   = Dia
 * DateTimeString[Offset+3]   = Mes
 * DateTimeString[Offset+4]   = Anio
 * 
 * "Offset" indica apartir de que posicion de la cadena se encuentra el valor de minutos
 * Su principal utilidad es en la comparacion de dos fecha-hora
 */

_U32 DateTimeTo32BitDecimal(_U08 *DateTimeString, _U08 Offset)
{
 _U32 DecimalCodedDateTime;
    
      DecimalCodedDateTime = (DateTimeString[Offset+4]*(_U32)100000000)+(DateTimeString[Offset+3]*(_U32)1000000)+(DateTimeString[Offset+2]*(_U32)10000)+(DateTimeString[Offset+1]*(_U32)100)+(DateTimeString[Offset]);
 
 return DecimalCodedDateTime;
}
/*----------------------------------------------------------------------------*/


/*Esta funcion toma 5 bytes y convierte su valor en un decimal de 32 bits
 * que representa fecha y hora
 */


_U32 UnsortedDateTimeTo32BitDecimal(_U08 bAnio,_U08 bMes, _U08 bDia, _U08 bHora, _U08 bMinuto)
{
 _U32 DecimalCodedDateTime;
    
      DecimalCodedDateTime = (bAnio*(_U32)100000000)+(bMes*(_U32)1000000)+(bDia*(_U32)10000)+(bHora*(_U32)100)+(bMinuto);
 
 return DecimalCodedDateTime;
}

/*----------------------------------------------------------------------------*/

_U08 BCDTOHEX(_U08 BCD)
{
    _U08 Hex;
    
    Hex = (((BCD&0xF0) >>4)*10) + (BCD&0x0F);
    
    return Hex;
}
/*----------------------------------------------------------------------------*/
void HexCodedDateTime(void)
{
    DateTimeChars[0] = BCDTOHEX(Time.minutos);
    DateTimeChars[1] = BCDTOHEX(Time.horas);
    DateTimeChars[2] = BCDTOHEX(Time.dia_mes);
    DateTimeChars[3] = BCDTOHEX(Time.mes);
    DateTimeChars[4] = BCDTOHEX(Time.anio);
}
/*----------------------------------------------------------------------------*/

/*funcion que regresa la fecha y hora actual codificada en decimal en un entero de 32 bits*/
_U32 DateTimeDecimalCoded(_U08 NowTime)
{
    _U32 DTDC;
    
    if(NowTime == 1)
    {
     Read_Time();
    }
    
     HexCodedDateTime();
    
    DTDC = DateTimeTo32BitDecimal(DateTimeChars,0);
    
    return DTDC;
}
/*----------------------------------------------------------------------------*/

_U08 UartGetData(void)
{
    _U32 Timeout = 0;
    _U08 RX_data;
    
    while(DataRdyUART1() == 0)
    {
      ++Timeout;
      
      if(Timeout > 1000)
          break;
    }
    
    if(DataRdyUART1())
    {
        RX_data = ReadUART1();
        return RX_data;
    }
    else
    {
        return 0xff;
    }
}

//------------------------------------------------------------------------------
 _U08 EthernetGetByte(void)
{
    _U32 Timeout = 0;
    _U08 RX_data;
    
    CTS = 0;
    
    while((DataRdyUART2()==0)&&(++Timeout < 10000));
    CTS = 1;
    
    if(DataRdyUART2())
    {
        RX_data = ReadUART2();
        return RX_data;
    }
    else
    {
        return 0xff;
    }
    
    UART2ClearAllErrors();
}
/*-----------------------------------------------------------------------------*/
void ClockTask(void)
{   
    
    // peticion fecha-hora : 0x 235E013C3F
    // ajuste reloj        : 0x 235E02 |seg-min-hrs-diasem-diames-mes-anio| 3C3F
    
    
    if( (UartGetData()==0x23)&&(UartGetData()==0x5E))
    {
        switch(UartGetData())
        {
            case 0x01:
            {
                if( (UartGetData()==0x3C)&&(UartGetData()==0x3F))
                {
                 Read_Time();
        
                 Uart1Put(0x23);
                 Uart1Put(0x5E);
        
                 Uart1Put(Time.segundos); 
                 Uart1Put(Time.minutos);
                 Uart1Put(Time.horas);
         
                 Uart1Put(Time.dia_sem);
                 Uart1Put(Time.dia_mes); 
                 Uart1Put(Time.mes);
                 Uart1Put(Time.anio);
        
                 Uart1Put(0x3C);
                 Uart1Put(0x3F);
         
                 LED_1 ^= 1;
                }
            }
            break;
            
            case 0x02:
            {   
                Time.segundos = UartGetData();
                Time.minutos  = UartGetData();
                Time.horas    = UartGetData();
                Time.dia_sem  = UartGetData();
                Time.dia_mes  = UartGetData();
                Time.mes      = UartGetData();
                Time.anio     = UartGetData();
                
                if( (UartGetData()==0x3C)&&(UartGetData()==0x3F))
                {
                    Set_Time();
                }
                
            }
            break;
            
            default:
            {
                
            }
            break;
            
        }
        UART1ClearAllErrors();
    }
}

/*-----------------------------------------------------------------------------*/
void ApplyOffsetValues(void)
 {
     _U32 CalAdr = CalibrationAddress;
     
     Eeprom25xx_Read(OffsetMyDevicesTemp,CalAdr, PCDevices);      // lee offsets de temperatura
     
     for(j=0;j<PCDevices;j++)
     {
         /*
         if(QUERY_8BIT(i,7))      // lee signo de offset
         {
             OffsetMyDevicesTemp[j] = MyDevicesTemp[j] - i;
         }
         else
         {
             OffsetMyDevicesTemp[j] = MyDevicesTemp[j] + i;
         }
         */
         
         //OffsetAppliedMyDevicesTemp[j] = 0;
         
         
         
         if((MyDevicesTempBuff0[j] == MyDevicesTempBuff1[j])&&(MyDevicesTempBuff0[j] != 0)&&(MyDevicesTempBuff0[j]<70))
         {
             OffsetAppliedMyDevicesTemp[j] = MyDevicesTempBuff0[j] + OffsetMyDevicesTemp[j];
         }
         
         
         
         
         //if((MyDevicesTemp[j] != 0)&&(MyDevicesTemp[j]<70))
         //{
         //    OffsetAppliedMyDevicesTemp[j] = MyDevicesTemp[j] + OffsetMyDevicesTemp[j];
         //}
         
          
         
     } 
 }
/*-----------------------------------------------------------------------------*/

void RadioTask(void)
{
    switch(PanCoordStateMachine)
    {
        case 0:                     // envia nuevos estados de dispositivos 
        {                           
            SendCmdToRadio(BroadcastNewDevicesStatus); 
            SwTimer0 = 0; 
            PanCoordStateMachine = 1;
            
        }
        break;
        
        case 1:                              // espera respuestas de dispositivos
        {
            
          if(SwTimer0 >= PCDevices+5)     // Time Out
          {
            ApplyOffsetValues();
            
            SendCmdToRadio(BroadcastConnectedDevices);
            SwTimer0 = 0;
            PanCoordStateMachine = 2;
          }
          
        }
        break;
        
        case 2:
        {   
         /* Dentro de este caso no hay comunicaciones inalambricas de ningun tipo         */
            
         /* Se ejecuta la tarea de revision de reservaciones si ha transcurrido           */
         /* el tiempo estipulado para ello.                                               */
         /* la variable EepromTimer indica cuando se ejecuta la revision de reservaciones */   
         /* EepromTimer = 40 equivale aproximadamente a 1 segundo                         */
            
         if(EepromTimer >= 2400)     /*si ha transcurrido dos minutos...*/
         {
          EepromTimer = 0;
          
          EepromTask();
          Reservations_And_ManuallyControlledDevices();
         }
            
            
            
         if(SwTimer0 >= 5)
         {
             ClrWdt();
             // Se promedia estados y temperaturas  
             
            for(j=0;j<16;j++)
            {
              if(myDevicesOutputStatusBuff0[j] == myDevicesOutputStatusBuff1[j])
              {
                  myDevicesOutputStatus[j] = myDevicesOutputStatusBuff0[j];
              }
              else
              {
                  
              }
              
                
             //ConnectedDevices[j]= 0;
             //myDevicesOutputStatus[j] = 0;
            }

             
            /*cambia a siguiente buffer y lo inicializa*/
            if(BuffNumber == 0)
            {
              BuffNumber = 1;
              
              for(j=0;j<16;j++)
              {
               ConnectedDevicesBuff1[j]= 0;
               myDevicesOutputStatusBuff1[j] = 0;
              }
              
              for(j=0;j<PCDevices;j++)
              {
               MyDevicesTempBuff1[j] = 0;
              }
              
            }
            else
            {
              BuffNumber = 0;
              
              for(j=0;j<16;j++)
              {
               ConnectedDevicesBuff0[j]= 0;
               myDevicesOutputStatusBuff0[j] = 0;
              }
              
              for(j=0;j<PCDevices;j++)
              {
               MyDevicesTempBuff0[j] = 0;
              }
              
            }
             
            
           PanCoordStateMachine = 0;
         }
        }
        break;
        
        default: break;
    }
}
/**---------------------------------------------------------------------------*/
 ///////////////////////////////////////////////////////////////////////////////
 //devuelve el indice de 0 1 10 000 y compara si coinciden ID de dispositivos 
 
 _U16 SearchStoredID(_U08 Device,_U08 ID1,_U08 ID0)
 {
     U32EEADR = 0;
     i=0;
     _U08 SearchBuff[3];
     
     
     for(_U16Var = 0; _U16Var < 10000; _U16Var++)
     {
         U32EEADR = (_U32)_U16Var * (_U32)13;
         U32EEADR += 10;
         
         Eeprom25xx_Read(SearchBuff,U32EEADR,3);
         
         U32EEADR -= 10;
         
         if((SearchBuff[0] == Device)&&(SearchBuff[1] == ID1)&&((SearchBuff[2] == ID0)))
         {
             i=1;
             break;
         }
     }

     
     if(i == 1)
     {
         return _U16Var;
     }
     else
     {
         return 0xFFFF;
     }
 }
/*--------------------------------------------------------------------------------*/
 void EthernetAck(_U08 Cmd, _U08 Ack)
 {
     ConsolePut(0x23);
     ConsolePut(0x5E);
     ConsolePut(Cmd);
     ConsolePut(Ack);
     ConsolePut(0x3c);
     ConsolePut(0x3f);
 }
/*--------------------------------------------------------------------------------*/
 
void EthernetTask(void)
 {
     _U08 i,l,Cmd,Param,ByteCount;
     _U32 volatile EEAdr;
     _U08 EthernetRxBuff[64];
     //_U08 volatile AuxBuff[16];

  if(EthernetGetByte() == 0x23)
  {
    if(EthernetGetByte() == 0x5E)
    {
      switch(EthernetGetByte())     // if command
      {
       case 0x00:  {Cmd = 0; Param =0;}   break;             // free
       case 0x01:  {Cmd = 1; Param =0;}  break;             // free
       case 0x02:  {Cmd = 2; Param =1;}   break;             // Solicitud de estado de dispositivos (estado y temperatura)
       case 0x03:  {Cmd = 3; Param =1;}   break;             // almacenar horario
       case 0x04:  {Cmd = 4; Param =9;}   break;             // Actualizar fecha y hora
       case 0x05:  {Cmd = 5; Param =0;}   break;             // Descargar  fecha y hora
       case 0x06:  {Cmd = 6; Param =1;}   break;             // Descargar horario
       case 0x07:  {Cmd = 7; Param =1;}   break;             // Descargar Calendario
       case 0x08:  {Cmd = 8; Param =0;}   break;             // borrar condiciones de dispositivos controlados manualmente
       case 0x09:  {Cmd = 9; Param =1;}   break;             // borrar condicion de dispositivo controlado manualmente
       case 0x0A:  {Cmd = 10; Param =2;}  break;             // control manual de salidas
       case 0x0B:  {Cmd = 11; Param =14;} break;             // almacena evento de reservacion. Se agrego un byte de reservacion confirmada
       case 0x0C:  {Cmd = 12; Param = 0;} break;  
       case 0x0D:  {Cmd = 13; Param = 1;} break;             // Descargar eventos de reservacion
       case 0x0E:  {Cmd = 14; Param = 13;}break;             // Editar reservacion
       case 0x0F:  {Cmd = 15; Param = 3;} break;             // Borrar reservacion
       case 0x10:  {Cmd = 16; Param = 3;} break;             // confirmar reservacion
       case 0x11:  {Cmd = 17; Param = 2;} break;             // minutos de tolerancia
       
       
       case 0x20:  {Cmd = 32; Param =1;}  break;             // armar calendario
       case 0x21:  {Cmd = 33; Param =2;}  break;
       case 0x22:  {Cmd = 34; Param =1;}  break;             // control de todas las salidas para proposito de pruebas
       case 0x23:  {Cmd = 35; Param =1;}  break;             // base time to tx
       case 0x24:  {Cmd = 36; Param =2;}  break;             // valor de calibracion de temperatura individual
       case 0x25:  {Cmd = 37; Param =0;}  break;             // lee valores de calibracion de temperatura
       case 0x26:  {Cmd = 38; Param =0;}  break;             // reinicia valores de calibracion de temperatura
       case 0x27:  {Cmd = 39; Param =0;}  break;             // lee valores de tolerancia de encendido y apagado
       
       case 0X28:  {Cmd = 40; Param =0;}  break;            //JL Solicitar dispositivos que estan escuchandose para debug
       default:    {Cmd=0xff;Param =0;}   break;
      }





      if(Cmd == 3)
      {
          goto __skip;
      }

      for(i=0;i<Param;i++)
      {
       EthernetRxBuff[i] = EthernetGetByte();
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
            //LED_1 ^= 1;
            //LED_2 ^= 1;

           }
           break;
//------------------------------------------------------------------------------
           case 1:  
           {
             
           }
           break;
//------------------------------------------------------------------------------
           case 2:  //Estado de dispositivos  235E02063c3f
           {
               if(EthernetRxBuff[0] == 6)
               {
                ConsolePut(0x23);
                ConsolePut(0x5E);
                ConsolePut(0x02);
//              
                /*
                for(j=0;j<16;j++)
                {
                 ConsolePut(myDevicesOutputStatus[j]);
                }
                
                */
                
               // para facilitar las cosas al software el estado de cada dispositivo se envia en un byte cada uno
                
              
                for(j=0;j<16;j++)    // 
                {
                    
                    for(i=0;i<8;i++)
                    {
                        if(QUERY_8BIT(myDevicesOutputStatus[j],i))
                        {
                            ConsolePut(0x01);
                        }
                        else
                        {
                            ConsolePut(0x00);
                        }
                    }
                    
                    
                 //ConsolePut(myDevicesOutputStatus[j]);
                }
                

                for(j=0;j<PCDevices;j++)
                {
                 //ConsolePut(MyDevicesTemp[j]);
                 ConsolePut(OffsetAppliedMyDevicesTemp[j]);
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

           }
           break;
//------------------------------------------------------------------------------
           case 4:   // Actualizar fecha y hora   ok
           {
              for(l=0;l<7;l++)
              {
                  i = EthernetRxBuff[l];           //respalda valor
                  EthernetRxBuff[l] = ((i/10)<<4) | (i%10);
              }
              
              
              Time.segundos = EthernetRxBuff[0];
              Time.minutos  = EthernetRxBuff[1];
              Time.horas    = EthernetRxBuff[2];
              Time.dia_sem  = EthernetRxBuff[3];
              Time.dia_mes  = EthernetRxBuff[4];
              Time.mes      = EthernetRxBuff[5];
              Time.anio     = EthernetRxBuff[6];
              
              Set_Time();
              
              
              ConsolePut(0x23);
              ConsolePut(0x5E);
              ConsolePut(0x04);
              ConsolePut(0x01);
              ConsolePut(0x3C);
              ConsolePut(0x3F); 
              
              
           }
           break;
//------------------------------------------------------------------------------
           case 5:      //  descargar fecha y hora ok  235e053c3f     
           {   
               Read_Time();
               
               EthernetRxBuff[0] = (((Time.segundos&0xF0)>>4)*10) + (Time.segundos&0x0F);
               EthernetRxBuff[1] = (((Time.minutos&0xF0) >>4)*10) + (Time.minutos&0x0F);
               EthernetRxBuff[2] = (((Time.horas&0xF0)   >>4)*10) + (Time.horas&0x0F);
               EthernetRxBuff[3] = (((Time.dia_sem&0xF0) >>4)*10) + (Time.dia_sem&0x0F);
               EthernetRxBuff[4] = (((Time.dia_mes&0xF0) >>4)*10) + (Time.dia_mes&0x0F);
               EthernetRxBuff[5] = (((Time.mes&0xF0)     >>4)*10) + (Time.mes&0x0F);
               EthernetRxBuff[6] = (((Time.anio&0xF0)    >>4)*10) + (Time.anio&0x0F);
               

               ConsolePut(0x23);
               ConsolePut(0x5E);
               ConsolePut(0x05);
               
               for(i=0;i<7;i++)
               {
                ConsolePut(EthernetRxBuff[i]);
               }

               ConsolePut(0x3C);
               ConsolePut(0x3F);


           }
           break;
//------------------------------------------------------------------------------
           case 6:   // descargar horario    235E06XX3C3F    ok
           {

           }
           break;
//------------------------------------------------------------------------------

           case 7:   // Descargar calendario   235E07XX  3C3F   ok
           {

           }
           break;

//------------------------------------------------------------------------------
           /*Comando que en un solo paso pone en cero los bits DCM de tal modo que todos los dispositivos*/
           /*esten controlados por los estados dados por las reservaciones*/
           
           case 8:  /*Elimina todas las condiciones de dispositivos controlados manualmente*/
           {
              for(j=0;j<16;j++)
              {
                   EthernetRxBuff[j] = 0x00;
              }
                  
                  U32EEADR = AdrDCM;
                  Eeprom25xx_Write(EthernetRxBuff,U32EEADR, 16);  // todos los dispositivos controlados manualmente
                  
                  ConsolePut(0x23);
                  ConsolePut(0x5E);
                  ConsolePut(0x08);
                  ConsolePut(0x01);
                  ConsolePut(0x3C);
                  ConsolePut(0x3F);
           }
           break;

//------------------------------------------------------------------------------
           /*Este comando pone en cero el bit de Dispositivo Controlado Manualmente DCM*/
           /*Al hacer esto, el dispositivo es controlado unicamente por los estados dados por las reservaciones*/
           //hay que indicar el id del dispositivo
           case 9:    /* Eliminar condicion de dispositivo controlado manualmente*/
           {
               if(EthernetRxBuff[0] <=128)       // si ID de  dispositivo soportado
               {
                j = (EthernetRxBuff[0]-1)/8;    // calcula byte
                i = (EthernetRxBuff[0]-1)%8;    // calcula bit
                
                U32EEADR = AdrDCM;  
                Eeprom25xx_Read(EthernetRxBuff,U32EEADR,16);   // lee estados en eeprom
                //SET_8BIT(DCM[j],i);
                CLEAR_8BIT(EthernetRxBuff[j],i);                                     // actualiza estados en RAM
                Eeprom25xx_Write(EthernetRxBuff,U32EEADR,16);  // actualiza estados en eeprom
                
                ConsolePut(0x23);
                ConsolePut(0x5E);
                ConsolePut(0x09);
                ConsolePut(0x01);
                ConsolePut(0x3C);
                ConsolePut(0x3F);
                
                
                
               }
           }
           break;
//------------------------------------------------------------------------------

             case 10:  
                 // control manual de salidas
             {         // 235E0A XX YY 3C3F
                 
                 /* Al recibir una orden de control manual, antes de aplicar el estado, es necesario verificar que */
                 /* el estado solicitado sea distinto al actual*/
              
              //_U08 DCM[16];     Dispositivos controlados manualmente  "condicion forzada"
              //_U08 EDCM[16];    Estado de los dispositivos controlados manualmente  "Estado solicitado"
                                 
              if((EthernetRxBuff[0] <= 127)&&(EthernetRxBuff[1] <= 1))  
              {
                //_U16Var = ((_U16)UartRxBuff[0]<<8) + (_U16)UartRxBuff[1];
                  
                  l = EthernetRxBuff[1];
                
                j = (EthernetRxBuff[0]-1)/8;    // calcula byte
                i = (EthernetRxBuff[0]-1)%8;    // calcula bit
                
                U32EEADR = AdrDCM;  
                Eeprom25xx_Read(EthernetRxBuff,U32EEADR,16);   // lee estados en eeprom
                //SET_8BIT(DCM[j],i);
                SET_8BIT(EthernetRxBuff[j],i);                                     // actualiza estados en RAM
                Eeprom25xx_Write(EthernetRxBuff,U32EEADR,16);  // actualiza estados en eeprom
                
                U32EEADR = AdrEDCM;  
                Eeprom25xx_Read(EthernetRxBuff,U32EEADR,16);   // lee estados en eeprom
                
                  
                  if(l == 1)
                  {
                      SET_8BIT(myDevicesRequiredStatus[j],i);   
                      SET_8BIT(EthernetRxBuff[j],i);
                      //SET_8BIT(EDCM[j],i);
                      
                  }
                  
                  if(l == 0)
                  {
                      CLEAR_8BIT(myDevicesRequiredStatus[j],i);
                      CLEAR_8BIT(EthernetRxBuff[j],i);
                      //CLEAR_8BIT(EDCM[j],i);
                  }
                
                Eeprom25xx_Write(myDevicesRequiredStatus,U32EEADR,16);  // actualiza estados en eeprom
                  
                  ConsolePut(0x23);
                  ConsolePut(0x5E);
                  ConsolePut(0x0A);
                  ConsolePut(0x01);
                  ConsolePut(0x3C);
                  ConsolePut(0x3F);  
              } 
              
             }
             break;

           //-------------------------------------------------------------------
             case 11:       // almacenar evento de reservacion
             {
                 if((EthernetRxBuff[10] > 0)&&(EthernetRxBuff[10] <= 128))     // si el ID esta comprendido entre 1 y 128
                 {
                     // si los elementos de fecha y hora estan dentro de los rangos permitidos...
                     
                     _U16Var = ((_U16)EthernetRxBuff[11]<<8) + EthernetRxBuff[12];
                     
                     if((EthernetRxBuff[0] <= 99)&&(EthernetRxBuff[1] <= 12)&&(EthernetRxBuff[2] <= 31)&&(EthernetRxBuff[3] <= 23)&&(EthernetRxBuff[4] <= 59)
                      &&(EthernetRxBuff[5] <= 99)&&(EthernetRxBuff[6] <= 12)&&(EthernetRxBuff[7] <= 31)&&(EthernetRxBuff[8] <= 23)&&(EthernetRxBuff[9] <= 59)
                      &&(EthernetRxBuff[10] <= 128)&&(_U16Var <10000))
                     {
                        //Modificacion realizada para el caso en que se sincronizan reservaciones con el software
                        //Se agrego un byte al final que indica si la reservacion esta confirmada. en cuyo caso, guardo
                         //directamente la reservacion con el bit de confirmada.
                         if(EthernetRxBuff[13])
                         {
                             SET_8BIT(EthernetRxBuff[0],7);
                         }
                         
                       if(U32EEIndex < AdrFinReserv)       // si hay espacio para almacenar reservacion...
                       {
                           Eeprom25xx_Write(EthernetRxBuff,U32EEIndex, 13);
                           
                           U32EEIndex+=13;                 // incrementa indice a localidad para nueva reservacion... 
                           
                           EthernetRxBuff[0] = 0x2A;
                           EthernetRxBuff[1] = 0x46;
                           EthernetRxBuff[2] = 0x49;
                           EthernetRxBuff[3] = 0x4E;
                           
                           EthernetRxBuff[4] = _U16Var >> 8;    // parte alta de ultimo ID reservacion
                           EthernetRxBuff[5] = _U16Var;         // parte baja de ultimo ID reservacion
                           
                           EthernetRxBuff[6] = U32EEIndex >> 24;  // Direccion actual de escritura U32EEADR 
                           EthernetRxBuff[7] = U32EEIndex >> 16;
                           EthernetRxBuff[8] = U32EEIndex >> 8;
                           EthernetRxBuff[9] = U32EEIndex;
                           
                           EthernetRxBuff[10] = 0x55;  
                           EthernetRxBuff[11] = 0xAA;
                           EthernetRxBuff[12] = 0x55; 
                           
                           Eeprom25xx_Write(EthernetRxBuff,U32EEIndex, 13);
                           
                           
                       }
                       else                              // se ha agotado espacio => se reinicia indice de escritura
                       {
                           U32EEIndex = 0;
                       }

       
                      ConsolePut(0x23);
                      ConsolePut(0x5E);
                      ConsolePut(0x0B);
                      ConsolePut(0x01);
                      ConsolePut(0x3C);
                      ConsolePut(0x3F);
                         
                     }
                     else          // algun elemento del comando se encuentra fuera de rango
                     {
                      ConsolePut(0x23);
                      ConsolePut(0x5E);
                      ConsolePut(0x0B);
                      ConsolePut(0x00);
                      ConsolePut(0x3C);
                      ConsolePut(0x3F);
                     }
                 }
             }
             break;
            
           //-------------------------------------------------------------------
             case 12:              // borrar todas las reservaciones  235E0C3C3F
             {
                 
                 /*Inicio Sector 0:  00000h   FIN:  7FFFh          0 -   32,767 */     
                 /*Inicio Sector 1:   8000h   FIN: 0FFFFh     32,768 -   65,535 */
                 /*Inicio Sector 2:  10000h   FIN: 17FFFh     65,536 -   98,303 */
                 /*Inicio Sector 3:  18000h   FIN: 1FFFFh     98,304 -  131,071 */
                 
                 /* envia comando borrar sector 0*/
                 EEAdr = 0;
                 Eeprom25xx_SectorErase(EEAdr);
                 
                 
                 /* envia comando borrar sector 1*/
                 EEAdr = 0x00008000;
                 Eeprom25xx_SectorErase(EEAdr);
                 
                 /* envia comando borrar sector 2*/
                 EEAdr = 0x00010000;
                 Eeprom25xx_SectorErase(EEAdr);
                 
                 /* borra el resto de reservaciones escribiendo por paginas :)*/
                 
                 
                 
                 
               EEAdr = 0x00018000;         //98,304   

               for(i=0;i<64;i++)
               {
                 EthernetRxBuff[i] = 0xAA;
               }
               
               LED_1 =1;
               LED_2 =1;
               
               
                                                                              // 31,743  ==> 496 paginas
            for(_U16Var =0;_U16Var<496;_U16Var++)                             // borra 512 paginas de 64 bytes C/U = 32,768 bytes
            {                                                                 // dir final: 130047

                Eeprom25xx_Write(EthernetRxBuff,EEAdr, 64);
                EEAdr += 64;
                
                LED_1 ^=1;
                LED_2 ^=1;
            }
               
               
               
               
               
               
               U32EEADR = 0;
               U32EEIndex = 0;
               
               EthernetRxBuff[0] = 0x2A;
               EthernetRxBuff[1] = 0x46;
               EthernetRxBuff[2] = 0x49;
               EthernetRxBuff[3] = 0x4E;
                           
               EthernetRxBuff[4] = 0;    // parte alta de ultimo ID reservacion
               EthernetRxBuff[5] = 0;         // parte baja de ultimo ID reservacion
                           
               EthernetRxBuff[6] = U32EEADR >> 24;  // Direccion actual de escritura U32EEADR 
               EthernetRxBuff[7] = U32EEADR >> 16;
               EthernetRxBuff[8] = U32EEADR >> 8;
               EthernetRxBuff[9] = U32EEADR;
                           
               EthernetRxBuff[10] = 0x55;  
               EthernetRxBuff[11] = 0xAA;
               EthernetRxBuff[12] = 0x55; 
                           
               Eeprom25xx_Write(EthernetRxBuff,U32EEADR, 13);
                 

                 
                      ConsolePut(0x23);
                      ConsolePut(0x5E);
                      ConsolePut(0x0C);
                      ConsolePut(0x01);
                      ConsolePut(0x3C);
                      ConsolePut(0x3F);
             }
             break;
           //-------------------------------------------------------------------
             
             case 13:    // descargar reservaciones ok  235e0D003c3f
             {
                 if(EthernetRxBuff[0] < 100)       // si bloque de reservaciones existente
                 {
                     EEAdr = (_U16)EthernetRxBuff[0]*(_U16)1300;
                     
                     
                      ConsolePut(0x23);
                      ConsolePut(0x5E);
                      ConsolePut(0x0D);
                      
                      for(j=0;j<20;j++)
                      {
                       Eeprom25xx_Read(EthernetRxBuff,EEAdr,64);
                       
                       for(i=0;i<64;i++)
                       {
                           ConsolePut(EthernetRxBuff[i]);
                       }
                       EEAdr += 64;
                      }
                      
                      Eeprom25xx_Read(EthernetRxBuff,EEAdr,20);
                      
                      for(i=0;i<20;i++)
                      {
                        ConsolePut(EthernetRxBuff[i]);
                      }
                      
                      ConsolePut(0x3C);
                      ConsolePut(0x3F);
                 }
                 else
                 {
                      ConsolePut(0x23);
                      ConsolePut(0x5E);
                      ConsolePut(0x0D);
                      ConsolePut(0x00);
                      ConsolePut(0x3C);
                      ConsolePut(0x3F);
                 }
             }
             break;
           //-------------------------------------------------------------------
             case 14:    // Editar reservacion
             {
                 
                 _U16Var = SearchStoredID(EthernetRxBuff[10],EthernetRxBuff[11],EthernetRxBuff[12]);
                 
                 if( _U16Var != 0XFFFF)
                 {
                     
                    U32EEADR = (_U32)_U16Var * (_U32)13;
        
                    Eeprom25xx_Write(EthernetRxBuff,U32EEADR,10);
                     
                    
                     ConsolePut(0x23);
                     ConsolePut(0x5E);
                     ConsolePut(0x0E);
                     ConsolePut(0x01);
                     ConsolePut(0x3C);
                     ConsolePut(0x3F);
                 }
                 else
                 {
                     ConsolePut(0x23);
                     ConsolePut(0x5E);
                     ConsolePut(0x0E);
                     ConsolePut(0x00);
                     ConsolePut(0x3C);
                     ConsolePut(0x3F);
                 }
             }
             break;
           //-------------------------------------------------------------------
                     
             case 15:    // Eliminar reservacion
             {
                 _U16Var = SearchStoredID(EthernetRxBuff[0],EthernetRxBuff[1],EthernetRxBuff[2]);
                 
                 if( _U16Var != 0XFFFF)
                 {
                     
                     
                     for(j=0;j<13;j++)
                     {
                         EthernetRxBuff[j] = 0xff;
                     }
                     
                     U32EEADR = (_U32)_U16Var * (_U32)13;
        
                    Eeprom25xx_Write(EthernetRxBuff,U32EEADR,13);
                     
                     
                     ConsolePut(0x23);
                     ConsolePut(0x5E);
                     ConsolePut(0x0F);
                     ConsolePut(0x01);
                     ConsolePut(0x3C);
                     ConsolePut(0x3F);
                 }
                 else
                 {
                     ConsolePut(0x23);
                     ConsolePut(0x5E);
                     ConsolePut(0x0F);
                     ConsolePut(0x00);
                     ConsolePut(0x3C);
                     ConsolePut(0x3F);
                 }
                 
             }
             break;
           //-------------------------------------------------------------------
             case 16:    // Confirmar reservacion
             {
                  _U16Var = SearchStoredID(EthernetRxBuff[0],EthernetRxBuff[1],EthernetRxBuff[2]); //Busca si 
                  
                 if( _U16Var != 0XFFFF)
                 {
                      U32EEADR = ((_U32)_U16Var * (_U32)13);
                                                           
                      Eeprom25xx_Read(ShortEepromBuff,U32EEADR,13);       /*lee reservacion desde eeprom*/  
                      
                      j = (EthernetRxBuff[0]-1)/8;    // calcula byte dentro de ConfirmedDvices[]
                      i = (EthernetRxBuff[0]-1)%8;    // calcula bit de dispositivo involucrado
                      
                      SMinutes = UnsortedDateTimeTo32BitDecimal((ShortEepromBuff[0]&0x7F),ShortEepromBuff[1], ShortEepromBuff[2], ShortEepromBuff[3], ShortEepromBuff[4]);
                      EMinutes = UnsortedDateTimeTo32BitDecimal((ShortEepromBuff[5]&0x7F),ShortEepromBuff[6], ShortEepromBuff[7], ShortEepromBuff[8], ShortEepromBuff[9]);
                      NMinutes = DateTimeDecimalCoded(1);
                      
                      if((SMinutes <= NMinutes)&&(NMinutes < EMinutes))     /* Si fecha hora actual esta dentro de intervalo de reservacion...*/
                      {
                          //if(AddMinutesToDateTime(Tolerance,ShortEepromBuff, DateTimeChars) == 0)  /* si no ha finalizado tiempo de confirmacion*/
                          //{
                            Eeprom25xx_Read(EthernetRxBuff,U32EEADR,1);
                            SET_8BIT(EthernetRxBuff[0],7);  
                            Eeprom25xx_Write(EthernetRxBuff,U32EEADR,1);    /* escribe bit de confirmacion*/
                            
                            //No se utiliza tal instrucción
                            SET_8BIT(ConfirmedDvices[j],i);                 /* indica que dispositivo tiene confirmada la reservacion  activa ahora*/
                           
                            //no se itiliza la siguiente variable
                            ConfirmedReserv[EthernetRxBuff[0]] = ((_U16)EthernetRxBuff[1]<<8)+(_U16)EthernetRxBuff[2];
                            
                            EthernetAck(16,1);    /* Respuesta reservacion confirmada*/
                            
                          //}
                          //else
                          //{
                              //EthernetAck(16,2);    /* Respuesta reservacion NO confirmada,ya ha expirado tiempo de confirmacion*/
                          //EthernetAck(16,0); 
                          //}
                      }
                      else
                      {
                          //EthernetAck(16,3);    /* Respuesta reservacion NO confirmada, la reservacion no esta activa*/
                          EthernetAck(16,0);
                      }   
                      
                 }
                 else
                 {
                  //EthernetAck(16,4);    /*Respuesta No se encontro reservacion!*/
                  EthernetAck(16,0);    
                 }
             }
             break;
           //-------------------------------------------------------------------
             case 17:    // configurar minutos de tolerancia al encendido y al apagado
             {       
                 /*Este valor solamente es utilizado para apagar dispositivos que 
                  * no se haya confirmado su reservacion. 
                  * Los minutos de encendido antes de la reservacion es calculado
                  * por el software
                  */
                 
                 
                 if((EthernetRxBuff[0]<=60)&&(EthernetRxBuff[1]<=60))    // valores limitados a una hora 
                 {
                     
                     U32EEADR = ToleranceValueAddress;
                     Eeprom25xx_Write(EthernetRxBuff,U32EEADR,2);
                     
                     ConsolePut(0x23);
                     ConsolePut(0x5E);
                     ConsolePut(0x11);
                     ConsolePut(0x01);
                     ConsolePut(0x3C);
                     ConsolePut(0x3F);
                 }
                 else
                 {
                     ConsolePut(0x23);
                     ConsolePut(0x5E);
                     ConsolePut(0x11);
                     ConsolePut(0x00);
                     ConsolePut(0x3C);
                     ConsolePut(0x3F);
                 }
             }
             break;
           //-------------------------------------------------------------------
             
           case 0x20: 
           {
               
               
               
           }
           break;
           
           //-------------------------------------------------------------------
           case 0x21:
           {

           }
           break;
           //-------------------------------------------------------------------
           
             case 0x22:     // encendido/apagado de todos los dispositivos    <debug command>
             { 
                if(EthernetRxBuff[0] <= 1)
                {
                 if(EthernetRxBuff[0] == 1)         // si encender todas las salidas
                 {
                  for(j=0;j<16;j++)
                  {
                    myDevicesRequiredStatus[j] = 0xff;
                    EthernetRxBuff[j] = 0xff;
                  }
                  
                  U32EEADR = AdrDCM;
                  Eeprom25xx_Write(EthernetRxBuff,U32EEADR, 16);  // todos los dispositivos controlados manualmente
                  
                  U32EEADR = AdrEDCM; 
                  Eeprom25xx_Write(EthernetRxBuff,U32EEADR, 16);  // todos los dispositivos habilitados
                 }
                    
                    
                 if(EthernetRxBuff[0] == 0)    // si apagar todas las salidas
                 {
                  for(j=0;j<16;j++)
                  {
                   myDevicesRequiredStatus[j] = 0x00;
                   EthernetRxBuff[j] = 0xff;
                  }
                  
                  U32EEADR = AdrDCM;
                  Eeprom25xx_Write(EthernetRxBuff,U32EEADR, 16);  // todos los dispositivos controlados manualmente
                  
                  U32EEADR = AdrEDCM; 
                  Eeprom25xx_Write(myDevicesRequiredStatus,U32EEADR, 16);  // todos los dispositivos deshabilitados
                  
                 }
                    
                 ConsolePut(0x23);
                 ConsolePut(0x5E);
                 ConsolePut(0x22);
                 ConsolePut(0x01);
                 ConsolePut(0x3C);
                 ConsolePut(0x3F);
                   
                }
                         
             }
             break;
           
           //-------------------------------------------------------------------
             case 0x23:       // 235E23XX3c3f
             {
                 if(EthernetRxBuff[0] <= 16)
                 {
                   BaseTimeToTx = EthernetRxBuff[0];
                   
                   EthernetRxBuff[1] = ~EthernetRxBuff[0];
                   
                   U32EEADR = AdrBaseTimeToTx;
                   Eeprom25xx_Write(EthernetRxBuff,U32EEADR, 2);  // todos los dispositivos controlados manualmente
                   
                   
                   
                 
                   ConsolePut(0x23);
                   ConsolePut(0x5E);
                   ConsolePut(0x23);
                   ConsolePut(0x01);
                   
                   ConsolePut(EthernetRxBuff[0]);
                   ConsolePut(EthernetRxBuff[1]);
                   
                   ConsolePut(0x3C);
                   ConsolePut(0x3F);
                 }
             }
             break;
             
           //-------------------------------------------------------------------
             
             case 0x24:   // valor de calibracion de temperatura
             {
                 if((EthernetRxBuff[0] <= PCDevices) && EthernetRxBuff[0] != 0)
                 {
                     
                     if(EthernetRxBuff[1] <= 15)
                     {                        
                      --EthernetRxBuff[0];
                      EEAdr = ((_U32)EthernetRxBuff[0]) + (_U32)CalibrationAddress;
                      i = EthernetRxBuff[0];
                      l = EthernetRxBuff[1];
                      Eeprom25xx_Write(&l ,EEAdr, 1);  // Guarda valor de calibracion
                         
                      DelayMs(10);
                      
                      EEAdr = (_U32)CalibrationAddress;
                    
                      Eeprom25xx_Read(EthernetRxBuff,EEAdr,16);
                      
                      
                      if(EthernetRxBuff[i] == l)
                      {
                         ConsolePut(0x23);
                         ConsolePut(0x5E);
                         ConsolePut(0x24);
                         ConsolePut(0x01); 
                         ConsolePut(0x3C);
                         ConsolePut(0x3F);
                      }
                            
                     }  
                     
                 }
             }
             break;
             
             
             //-----------------------------------------------------------------
             case 0x25:   // lee valores de calibracion de temperatura   <debug command>
             {
                 
                 EEAdr = (_U32)CalibrationAddress;
                    
                         
                 Eeprom25xx_Read(EthernetRxBuff,EEAdr,PCDevices);
                 
                 ConsolePut(0x23);
                 ConsolePut(0x5E);
                 ConsolePut(0x25);
                         
                 for(j=0;j<PCDevices;j++)
                 {
                  ConsolePut(EthernetRxBuff[j]); 
                 }
                         
                         
                 ConsolePut(0x3C);
                 ConsolePut(0x3F);
             }
             break;
             
             //-----------------------------------------------------------------
             
             case 0x26:   // reiniciar valores de calibracion de temperatura  235E263C3F
             {
                 
                 EEAdr = (_U32)CalibrationAddress;
                 
                 for(j=0;j<PCDevices;j++)
                 {
                   EthernetRxBuff[j] = 0;
                 }
                 
                 Eeprom25xx_Write(EthernetRxBuff,EEAdr,PCDevices);
                 
                 DelayMs(20);
                         
                 Eeprom25xx_Read(EthernetRxBuff,EEAdr,PCDevices);
                 
                 ConsolePut(0x23);
                 ConsolePut(0x5E);
                 ConsolePut(0x26);
                         
                 for(j=0;j<PCDevices;j++)
                 {
                  ConsolePut(EthernetRxBuff[j]); 
                 }
                         
                         
                 ConsolePut(0x3C);
                 ConsolePut(0x3F);
             }
             break;
             
             //-----------------------------------------------------------------
           
             case 0x27:    // <debug command>  lee minutos de tolerancia configurados para el encendido antes de la reservacion
             {             // y apagado en caso de no haber confirmacion
                     U32EEADR = ToleranceValueAddress;
                     Eeprom25xx_Read(EthernetRxBuff,U32EEADR,2);
                     
                     ConsolePut(0x23);
                     ConsolePut(0x5E);
                     ConsolePut(0x27);
                     ConsolePut(EthernetRxBuff[0]);    // minutos encendido previo a reservacion
                     ConsolePut(EthernetRxBuff[1]);    // minutos apagado despues de reservacion al no haber confirmacion
                     ConsolePut(0x3C);
                     ConsolePut(0x3F);
             }
             break;
             //-----------------------------------------------------------------
              //-----------------------------------------------------------------
           //JL para poder implementar en software un request de los dispositivos comunicandose en la red inalambrica
             case 0x28:    // <debug command>  solicita los dispositivos que se estan comunicando dentro de la red.
             {             // 
                     
                     
                     ConsolePut(0x23);
                     ConsolePut(0x5E);
                     ConsolePut(0x28);
                     
                     for(j=0;j<16;j++)    // 
                    {
                    
                        for(i=0;i<8;i++)
                        {
                            if(QUERY_8BIT(ConnectedDevices[j],i))           
                            {
                            ConsolePut(0x01);
                            }
                            else
                            {
                            ConsolePut(0x00);
                            }
                        }

                    }
                    
                     ConsolePut(0x3C);
                     ConsolePut(0x3F);
             }
             break;
             //-----------------------------------------------------------------
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
/*----------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------*/
/*
 * Toma un valor entero que representan minutos y lo suma a una hora-fecha dada
 */
_U08 AddMinutesToDateTime(_U08 Minutes, _U08 *DateTimeString, _U08 *NTimeString)
{
   //_U08 DateTimeChars[5];
   //_U32 DateTime; 
   
   _U08 Minutos;
   _U08 Horas;
   _U32 CTime;
   _U32 NTime;
   
   Minutos = (Minutes + DateTimeString[4])%60;                               // obtiene el valor de minutos
   Horas = ( (DateTimeString[3]) + ((Minutes + DateTimeString[4])/60))%24;   // obtiene valor de horas
  
   CTime = (Horas*(_U32)100)+  Minutos;
   NTime = (NTimeString[1]*(_U32)100)+  NTimeString[0];
   
   /*
   Uart1PutROMString((ROM char *)"\r\n Add Minutes to Time:   ");
   
   Uart1PrintChar(Minutes);
   Uart1PrintChar(Horas);
   Uart1PrintChar(Minutos);
   
   
   
   Uart1PrintChar(CTime>>8);
   Uart1PrintChar(CTime);
   
   
   Uart1PrintChar(NTime>>8);
   Uart1PrintChar(NTime);
   */
   
   if(NTime > CTime)   // si ha pasado tiempo de tolerancia ...
   {
       //Uart1PutROMString((ROM char *)"\r\n se ha cancelado Reservacion!!");
       
      return 1;     
   }
   else
   {
       
       //Uart1PutROMString((ROM char *)"\r\n Reservacion en espera de confirmacion!!");
       return 0;
   }
   
   
   
   
   
  // if(((DateTimeString[3]) + ((Minutes + DateTimeString[4])/60)) > 23)    // si hay cambio de dia
  // {
       
  // }
 //  else    // Sigue estando en el mismo dia 
  // {
       
  // }

    
    //return DateTime;
}
/*-----------------------------------------------------------------------------*/
/*-----------------------------------------------------------------------------*/

/* lee todas las reservaciones almacenadas en eeprom y actualiza el estado que debe 
* tener cada dispositivo                                                           
*                                                                                  
* Esta tarea tiene un tiempo de ejecucion variable que depende de la cantidad de   
* reservaciones almacenadas en memoria. Es importante que el software mantenga     
* actualizado el registro de reservaciones, eliminando todas aquellas que ya hayan  
* expirado, de tal modo que el tiempo de busqueda de reservaciones sea lo mas corto
* posible.
*                                                                                  
* Esta tarea se ejecuta durante los intervalos en que no hay actividad de comunica-
* ciones inalambricas      
*                                                                               */
void EepromTask(void)
{
    //_U32 CMinutes;     //hora fecha  de confirmacion de reservacion
    _U32 Adr;
    _U08 ResBuff[15];
    _U32 k,l,n;
    _U08 m,_byte,_bit;
    _U16 _U16Var;
    
    Read_Time();                                         /* lee fecha y hora en este momento*/
/*   
    Uart1PutROMString((ROM char *)"\r\n Fecha Hora: ");
    
    Uart1PrintChar(Time.horas);
    Uart1PrintChar(Time.minutos);
    Uart1PrintChar(Time.dia_mes);
    Uart1PrintChar(Time.mes);
    Uart1PrintChar(Time.anio);
   
    /*convierte de formato BCD a Hexadecimal*/ 
    HexCodedDateTime();
    /*
    DateTimeChars[0] = (((Time.minutos&0xF0) >>4)*10) + (Time.minutos&0x0F);
    DateTimeChars[1] = (((Time.horas&0xF0)   >>4)*10) + (Time.horas&0x0F);
    DateTimeChars[2] = (((Time.dia_mes&0xF0) >>4)*10) + (Time.dia_mes&0x0F);
    DateTimeChars[3] = (((Time.mes&0xF0)     >>4)*10) + (Time.mes&0x0F);
    DateTimeChars[4] = (((Time.anio&0xF0)    >>4)*10) + (Time.anio&0x0F);       
    */
    /*agrupa fecha hora en un entero de 32bits con orden |Anio|Mes|Dia|Hora|Minutos| */
    //NMinutes = (DateTimeChars[4]*(_U32)100000000)+(DateTimeChars[3]*(_U32)1000000)+(DateTimeChars[2]*(_U32)10000)+(DateTimeChars[1]*(_U32)100)+(DateTimeChars[0]);
    //NMinutes = DateTimeTo32BitDecimal(DateTimeChars,0);
    NMinutes = DateTimeDecimalCoded(1);
    
    /*
    Uart1PutROMString((ROM char *)"\r\n NMinutes:");
    Uart1PrintDec(DateTimeChars[4]);
    Uart1PrintDec(DateTimeChars[3]);
    Uart1PrintDec(DateTimeChars[2]);
    Uart1PrintDec(DateTimeChars[1]);
    Uart1PrintDec(DateTimeChars[0]);
    */
   EepromSearchIndex = 0;
    
   for(m=0;m<16;m++)              /*inicializa a 0 dispositivos activos por reservaciones*/
   {
       EDCR[m] = 0;
   }
    
   U32EEADR = ToleranceValueAddress;
   Eeprom25xx_Read(EepromBuff,U32EEADR,2);              /*Lee minutos de tolerancia de encendido y apagado*/
   
   Tolerance = EepromBuff[0] + EepromBuff[1];           /*suma ambos valores */
   
   /*
   Uart1PutROMString((ROM char *)"\r\n Tolerance:");
   Uart1PrintChar(EepromBuff[0]);
   Uart1PrintChar(EepromBuff[1]);
   */
   m=0;       // indice de bloques leidos
   l=0;       // indice de lectura dentro de cada bloque
      
   //Uart1PutROMString((ROM char *)"\r\n Inicia revision de Reservaciones");
   
   while(m++<156)         // bucle que lee reservaciones en bloques de 64
   {
    //Uart1PutROMString((ROM char *)"\r\n [");
    Eeprom25xx_Read(EepromBuff,EepromSearchIndex,832);       // lee 64 posibles reservaciones 
    //Uart1PutROMString((ROM char *)"\r\n ]");
    
    /*------------------------------------------------------*/
    l=0;
    
    for(k=0;k<64;k++)                                        // analiza las 64 probables reservaciones 1 x 1
    {
        for(n=0;n<13;n++)                                    // lee los 13 bytes que componen una reservacion
        {
          ResBuff[n] = EepromBuff[l+n];
        }
        
        l += (_U32)13;  //para la siguiente vuelta guardar en ResBuff la siguiente reservacion
             
    /* inicio fin  de reservacion   bytes [0-9]   */
    /* ID de dispositivo            bytes [10]    */
    /* Indice consecutivo           bytes [11-12] */
        
        // valida que datos leidos sean congruentes con los que deberia tener una reservacion
         if(((ResBuff[0]&0x7F) < 100)&&(ResBuff[1]<13)&&(ResBuff[2]<32)&&(ResBuff[3]<24)&&(ResBuff[4]<60)
          &&((ResBuff[5]&0x7F) < 100)&&(ResBuff[6]<13)&&(ResBuff[7]<32)&&(ResBuff[8]<24)&&(ResBuff[9]<60)
          &&(ResBuff[10]<129))
         //&&(ResBuff[10]<129)&&(ResBuff[11]<0x28))
         {
             //Uart1PutROMString((ROM char *)"\r\n [");
             //Uart1PrintChar(ResBuff[11]);
             //Uart1PrintChar(ResBuff[12]);
             //Uart1PutROMString((ROM char *)"]");
             //DelayMs(200);
             
             /*agrupa fecha hora de inicio y de fin en un entero de 32bits, cada uno, con orden |Anio|Mes|Dia|Hora|Minutos| */
           //SMinutes = ((ResBuff[0]&0x7F)*(_U32)100000000)+(ResBuff[1]*(_U32)1000000)+(ResBuff[2]*(_U32)10000)+(ResBuff[3]*(_U32)100)+(ResBuff[4]);           
           //EMinutes = ((ResBuff[5]&0x7F)*(_U32)100000000)+(ResBuff[6]*(_U32)1000000)+(ResBuff[7]*(_U32)10000)+(ResBuff[8]*(_U32)100)+(ResBuff[9]);
         
           SMinutes = UnsortedDateTimeTo32BitDecimal((ResBuff[0]&0x7F),ResBuff[1], ResBuff[2], ResBuff[3], ResBuff[4]);
           EMinutes = UnsortedDateTimeTo32BitDecimal((ResBuff[5]&0x7F),ResBuff[6], ResBuff[7], ResBuff[8], ResBuff[9]);
           
           if((SMinutes <= NMinutes)&&(NMinutes < EMinutes))     /* Si fecha hora actual esta dentro de intervalo de reservacion...*/
           { 
           /*
           Uart1PutROMString((ROM char *)"\r\n SMinutes:");
           Uart1PrintDec(ResBuff[0]);
           Uart1PrintDec(ResBuff[1]);
           Uart1PrintDec(ResBuff[2]);
           Uart1PrintDec(ResBuff[3]);
           Uart1PrintDec(ResBuff[4]);
           Uart1PutROMString((ROM char *)"\r\n EMinutes:");
           Uart1PrintDec(ResBuff[5]);
           Uart1PrintDec(ResBuff[6]);
           Uart1PrintDec(ResBuff[7]);
           Uart1PrintDec(ResBuff[8]);
           Uart1PrintDec(ResBuff[9]);
           */
               
           //ConfirmedReserv[PCDevices];
               
                _byte = (ResBuff[10]-1)/8;    // calcula byte
                _bit  = (ResBuff[10]-1)%8;    // calcula bit
               
            /* Como la reservacion se encuentra activa se procede a verificar si se encuentra confirmada... */   
               if(QUERY_8BIT(ResBuff[0],7))         /* La reservacion esta confirmada*/
               {
                   /*activa bit de dispositivo correspondiente a reservacion*/
                   SET_8BIT(EDCR[_byte],_bit);
               }
               else  
               {    
                /*No esta confirmada por lo que se comprueba tiempo transcurrido desde que inicio      */
                /*en caso que el tiempo transcurrido sea mayor a la suma de los tiempos de tolerancia  */
                /*de encendido y apagado, se procede a cancelar la reservacion y apagar el dispositivo */     
                   
                   if(AddMinutesToDateTime(Tolerance,ResBuff,DateTimeChars)== 0)      /* no ha terminado tiempo de confirmacion*/
                   {
                        /*activa bit de dispositivo correspondiente a reservacion*/
                        SET_8BIT(EDCR[_byte],_bit);
                        
                   }
                   else /* ha terminado tiempo de confirmacion*/
                   {
                       /*El bit del dispositivo se deja apagado y se elimina la reservacion*/
                       
                       
                       
                      
                    
                    // _U16Var = SearchStoredID(ResBuff[10],ResBuff[11],ResBuff[12]);      // Busca Reservacion en eeprom y regresa indice
                /*
                     if( _U16Var != 0XFFFF)
                     {
                       for(j=0;j<13;j++)
                       {
                         _16EepromBuff[j] = 0xAA;
                       }
                     
                       U32EEADR = (_U32)_U16Var * (_U32)13;
                       Eeprom25xx_Write(_16EepromBuff,U32EEADR,13);
                     }
                     
                    */
                  
                   }
               }

               
     /*      
               Uart1PutROMString((ROM char *)"\r\n Reservacion Activa");
               
                     if(k<100)
                     {
                      Uart1PrintDec(k);
                     }
                     else
                     {
                      Uart1PrintDec(k/100);
                      Uart1PrintDec(k-100);
                     }
               
               Uart1PutROMString((ROM char *)"\r\n");
    
     */
           }
           else                                                   /* fecha hora actual no esta dentro de intervalo de reservacion*/
           {
               //Uart1PutROMString((ROM char *)"\r\nReservacion Inactiva");
           }
         
         }
         else  /*datos no corresponden a una reservacion                        */
         { 
           /*se pueden dar los siguientes casos:                                */
           /* 1.- que se encuentre indice de escritura de reservaciones         */
           /* 2.- Que el espacio este en blanco                                 */
           /**/
           /*verifica si datos corresponden a indice de escritura de ultima reservacion*/ 
             /*2A46494E 0024 0000 0111 55AA5*/
             
             if((ResBuff[0]== 0x2A)&&(ResBuff[1]== 0x46)&&(ResBuff[2]== 0x49)&&(ResBuff[3]== 0x4E)&&(ResBuff[10]== 0x55)&&(ResBuff[11]== 0xAA))
             {
                 //Uart1PutROMString((ROM char *)"\r\n  Se ha leido indice de escritura!!!");
                 m=156;       /* forza la salida de el bucle while*/
             }
             
         }
        
        
        
    }
  
    //DelayMs(10);
    EepromSearchIndex += (_U32)832;  // incrementa indice hasta siguiente bloque de reservaciones
   
    
    /*
    Uart1PutROMString((ROM char *)"\r\n");
    
    if(m<100)
    {
        Uart1PrintDec(m);
    }
    else
    {
        
        Uart1PrintDec(m/100);
        Uart1PrintDec(m-100);
    }
    */
    
    //Uart1PutROMString((ROM char *)"\r\n");
    
    //ConsolePut(EepromSearchIndex>>24);
    //ConsolePut(EepromSearchIndex>>16);
    //ConsolePut(EepromSearchIndex>>8);
    //ConsolePut(EepromSearchIndex);
    
   }
   
   /*
   Uart1PutROMString((ROM char *)"\r\n");
   
   for(m=0;m<16;m++)
   {
       Uart1PrintChar(EDCR[m]);
   }
   
   Uart1PutROMString((ROM char *)"\r\n Termina revision de Reservaciones");
   
    */
   //ConsolePut(0x3C);
   //ConsolePut(0x3F);
   
   
   
   
}
/*----------------------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
/*Esta funcion sigue las siguientes condiciones de entrada para dar una salida:           */
/*                                                                                        */
/*   1.- Si el bit DCM esta activo, entonces puede suceder que:                           */
/*    a) si los bits EDCM y EDCR son iguales, significa que ha terminado el intervalo     */
/*       de control manual del dispositivo, por lo que el bit DCM debera ponerce a cero y */
/*       el estado final del dispositivo vendra dado por el bit EDCR.                     */
/*                                                                                        */
/*    b) si los bits EDCM y EDCR son distintos, entonces el estado final del dispositivo  */
/*       estara dado por el bit EDCM.                                                     */
/*                                                                                        */
/*   2.- Si el bit DCM esta inactivo, entonces el estado final del dispositivo vendra dado*/
/*       unicamente por el bit EDCR                                                       */
/*                                                                                        */
/*   DCM  = Dispositivo Controlado Manualmente                                            */
/*   EDCM = Estado de Dispositivo Controlado Manualmente                                  */
/*   EDCR = Estado de Dispositivo Controlado por Reservacion                              */
/*                                                                                        */
void Reservations_And_ManuallyControlledDevices(void)
{
    _U08 XORBUFF[16];
    _U08 DCMBUFF[16];
    _U08 EDCMBUFF[16];
    
    
    U32EEADR = AdrDCM;
    Eeprom25xx_Read(DCMBUFF,U32EEADR, 16);  // lee dispositivos que estan controlados manualmente
    
    U32EEADR = AdrEDCM;
    Eeprom25xx_Read(EDCMBUFF,U32EEADR, 16);  // lee estado de dispositivos controlados manualmente
    
    /*
   Uart1PutROMString((ROM char *)"\r\n Dispositivos controlados manualmente DCM");
   
   for(j=0;j<16;j++)
   {
       Uart1PrintChar(DCMBUFF[j]);
   }
   
   Uart1PutROMString((ROM char *)"\r\n");
   
   
   Uart1PutROMString((ROM char *)"\r\n Estado Dispositivos controlados manualmente EDCM");
   
   for(j=0;j<16;j++)
   {
       Uart1PrintChar(EDCMBUFF[j]);
   }
   
   Uart1PutROMString((ROM char *)"\r\n");
    
    */
    
    for(i=0;i<16;i++)                         // bucle a travez de cada byte
    {
        /* aplica operacion XOR entre estados de reservacion y estados manuales para encontrar */
        /*que intervalos de control manual han finalizado*/
        
        XORBUFF[i] = EDCR[i] ^ EDCMBUFF[i]; 
        
        for(j=0;j<8;j++)     // bucle a travez de cada bit
        {
            /*pone a cero los bits de DCM que hayan finalizado su intervalo y que estaban activos anteriormente*/
            
            if(QUERY_8BIT(DCMBUFF[i],j))                         /* si dispositivo controlado manualmente*/
            {
                if(QUERY_8BIT(XORBUFF[i],j))                     /* si no ha finalizado intervalo de control manual*/
                {
                    if(QUERY_8BIT(EDCMBUFF[i],j))                /* estado final queda determinado por EDCM*/
                    {
                        SET_8BIT(myDevicesRequiredStatus[i],j);
                    }
                    else
                    {
                        CLEAR_8BIT(myDevicesRequiredStatus[i],j);
                    }
                }
                else                                             /*ha terminado intervalo de control manual*/
                {
                    CLEAR_8BIT(DCMBUFF[i],j);                    /* deshabilita control manual ya que ha finalizado intervalo*/
                    
                    U32EEADR = AdrDCM;
                    Eeprom25xx_Write(DCMBUFF,U32EEADR, 16);      /* actualiza dispositivos controlados manualmente */
                    
                    
                    if(QUERY_8BIT(EDCR[i],j))                    /* estado final queda determinado por reservaciones [EDCR]*/
                    {
                        SET_8BIT(myDevicesRequiredStatus[i],j);
                    }
                    else
                    {
                        CLEAR_8BIT(myDevicesRequiredStatus[i],j);
                    }
                    
                }
                
            }
            else  /* el dispositivo esta controlado unicamente por las reservaciones */
            {
                if(QUERY_8BIT(EDCR[i],j))                    /* estado final queda determinado por reservaciones [EDCR]*/
                {
                 SET_8BIT(myDevicesRequiredStatus[i],j);
                }
                else
                {
                 CLEAR_8BIT(myDevicesRequiredStatus[i],j);
                }
            }
        }
    }
   
   /*
   Uart1PutROMString((ROM char *)"\r\n Estados finales para dispositivos");
   
   for(j=0;j<16;j++)
   {
       Uart1PrintChar(myDevicesRequiredStatus[j]);
   }
   
   Uart1PutROMString((ROM char *)"\r\n");
   */
}
/*----------------------------------------------------------------------------*/
 /* Busca la direccion donde se ha almacenado ultimo evento de reservacion*/
 
 void SearchEepromIndex(void)
 {   
   U32EEIndex = 0; 
   _U08 IndexFound = 0;
   j=0;
   
  /*ClrWdt(); */
   
   for(_U16Var = 0; _U16Var < 10000; _U16Var++)
   {
       Eeprom25xx_Read(EepromBuff,U32EEIndex,13);               // lee localidad 
       
       if((EepromBuff[0]==0x2A)&&(EepromBuff[1]==0x46)&&(EepromBuff[2]==0x49)&&(EepromBuff[3]==0x4E))  // si se encuentra indice de lectura...
       {
           //ConsolePut(0x2A); 
           //ConsolePut(0x46); 
           //ConsolePut(0x3F); 
           
           IndexFound = 1;
           //Uart1PutROMString((ROM char *)"\r\n Indice de escritura encontrado!");
           
           break;
       }
       else
       {
           /*
           if(j==100)
           {
               j=0;
                      ConsolePut(U32EEADR>>24);
                      ConsolePut(U32EEADR>>16);
                      ConsolePut(U32EEADR>>8);
                      ConsolePut(U32EEADR);
                      ConsolePut(0x3F); 
           }
           
           j++;
                 */     
           
       }
      
      U32EEIndex += 13; 
      
   }
   
   if(IndexFound == 1)  /* Indice de escritura encontrado */
   {
       
   }
   else                 /* Accion a tomar al no encontrar el indice de escritura !!!*/
   {
       
   }
   
 }
/*----------------------------------------------------------------------------*/

void InitConfig(void)
{
    // disable JTAG port
    DDPCONbits.JTAGEN = 0;
    
    SYSTEMConfigPerformance(12000000);

    BoardInit();
    ConsoleInit();

    ANSELA = 0;
    ANSELB = 0;
    ANSELC = 0;
    
   mPORTBSetPinsDigitalOut(BIT_3);      // Set PB3(Tx) as output
   mPORTBSetPinsDigitalIn (BIT_13);     // Set PB13(Rx) as input
   mPORTCSetPinsDigitalOut(BIT_2);      // Set C2(Tx) as output
   mPORTASetPinsDigitalIn (BIT_1);      // Set A1(Rx) as input
   mPORTCSetPinsDigitalOut(BIT_1);      // Set C1(CTS) as output
   mPORTBSetPinsDigitalOut(BIT_6);      // Set PB6(SDO1) as output
   mPORTBSetPinsDigitalOut(BIT_14);     // Set PB14(SCK1) as output
   mPORTBSetPinsDigitalIn (BIT_5);      // Set PB5(SDI1) as input
   mPORTCSetPinsDigitalOut(BIT_8);      // Set PC8(SDO2) as output
   mPORTBSetPinsDigitalOut(BIT_15);     // Set PB15(SCK2) as output
   mPORTBSetPinsDigitalIn (BIT_2);      // Set PB2(SDI2) as input
   mPORTCSetPinsDigitalIn (BIT_7);      // Set PC7(INT4) as input       MRF-IRQ1
   mPORTCSetPinsDigitalOut(BIT_3);      // Set PC3(MRF_RESET) as output
   
   mPORTASetPinsDigitalOut(BIT_8);      /*LEDS*/
   mPORTASetPinsDigitalOut(BIT_9);

    PPSUnLock;                        // Allow PIN Mapping

    U1RXR = 0x03;  // RX1 --> B13
    RPB3R = 0x01;  // B3  --> U1TX
    
    U2RXR = 0x00;  // RX2 --> A1
    RPC2R = 0x02;  // C2  --> U2TX

    SDI1R = 0x01;  // SDI1 --> B5
    RPB6R = 0x03;   // B6   -->SDO1

    SDI2R = 0x04;  // SDI2 --> B2
    RPC8R = 0x04;   // C8   -->SDO2

    INT4R = 5;      // IRQ1 ---> C7 ---> INT4

    PPSLock;                        // Prevent Accidental Mapping

    CTS = 1;
    
   UARTConfigure(UART1, UART_ENABLE_PINS_TX_RX_ONLY);
   UARTSetLineControl(UART1, UART_DATA_SIZE_8_BITS | UART_PARITY_NONE | UART_STOP_BITS_1);
   UARTSetDataRate(UART1, 12000000, 38400);
   UARTEnable(UART1, UART_ENABLE_FLAGS(UART_PERIPHERAL | UART_RX | UART_TX));
   
   Eeprom25xx_Init();          // inicializa memoria 0, 128 kbits

   U1STAbits.URXISEL = 0;       // U1 interrupt on every char
   IPC8bits.U1IP = 2;
   IPC8bits.U1IS = 3;
   IEC1bits.U1RXIE = 1;         // enable U1RX interrupt
   
   //Uart1PutROMString((ROM char *)"\r\nPIC32 Bridge Board");
   
    OpenI2C1( I2C_EN, BRG_VAL );
    
            
    UartRxBuffIndex = 0;

    LED_1 = 1;
    LED_2 = 1;
    
    
    SearchEepromIndex();
    
    LED_1 = 1;
    LED_2 = 0;

     for(j=0;j<10;j++)
     {
         DelayMs(100);

         LED_1 ^= 1;
         LED_2 ^= 1;
     }

     LED_1 = 0;
     LED_2 = 0;
   
 
     Uart1RXState = 0;
     RxCmdNum = 0;
     
     for(j=0;j<16;j++)
     {
      ConnectedDevices[j]= 0;
      myDevicesOutputStatus[j] = 0;
      ConfirmedDvices[j] = 0;
     }

     for(j=0;j<PCDevices;j++)
     {
         MyDevicesTempBuff0[j] = 0;
         MyDevicesTempBuff1[j] = 0;
         OffsetAppliedMyDevicesTemp[j] = 0;
         ConfirmedReserv[j] = 0;
     }
     
    SwTimer0 = 0;
    PanCoordStateMachine = 0;
    EepromTimer = 0;
    
    
    U32EEIndex = AdrBaseTimeToTx;
    Eeprom25xx_Read(EepromBuff,U32EEIndex,2);               // lee localidad
    
    EepromBuff[1] = ~EepromBuff[1];
    
    
    if((EepromBuff[0]<17)&&(EepromBuff[0]==EepromBuff[1]))
    {
        BaseTimeToTx = EepromBuff[0];
    }
    else
    {
        BaseTimeToTx = 4;
    }
    
  
    
    OpenCoreTimer(CORE_TICK_RATE);
    mConfigIntCoreTimer((CT_INT_ON | CT_INT_PRIOR_2 | CT_INT_SUB_PRIOR_0));
}

////////////////////////////////////////////////////////////////////////////////

void main(void)
{
    
    // Pines Reubicados
    //SPI1 <transceiver>             SDI1 = B5  | SDO1 =  B6
    //SPI2  <EEPROM, MAC_EEPROM>     SDI2 = A4  | SDO2 =  C8
    //UART1 <DEBUG>                  TX1 = B3 | RX1 =  B13
    //UART2 <Console, Wiznet>        TX2 = C2 | RX2 =  A1
  
   
     InitConfig();
     
     EepromSearchIndex = 0;

     while(1)
     { 
         
      RadioTask();
      EthernetTask();   
      
      
     }
} 


/*-----------------------------------------------------------------------------*/
// UART 1 interrupt handler
// it is set at priority level 2
void __ISR(_UART1_VECTOR, ipl2) IntUart1Handler(void)
{
    _U08 data;
    // RX interrupt 
	if(IFS1bits.U1RXIF)
	{
	 IFS1bits.U1RXIF = 0; 
     data = (_U08)ReadUART1();
     
     switch(Uart1RXState)
     {
         case 0:
         {
             if(data == StartCmd)       
             {
                 Uart1RXState = 1;
             }
             else
             {
                 Uart1RXState = 0;
             }
         }
         break;
         
         
         case 1:        
         {
             if(data == OutputStat)
             {
                 Uart1RXState = 2;
             }
             else
             {
                 Uart1RXState = 0;
             }
         }
         break;
         
         case 2:
         {
             RcvNodeNumber = data;
             Uart1RXState = 3;
         }
         break;
         
         case 3:
         {
             DevStatusTemp = data;
             Uart1RXState = 4;
         }
         break;
         
         case 4:
         {
             if((data==EndCmd)&&(RcvNodeNumber<=PCDevices))                   // si se recibio comando <OutputStat> correcto y completo...
             {
                 Uart1RXState = 0;
                  
                  j = (RcvNodeNumber-1)/8;             // calcula byte
                  i = (RcvNodeNumber-1)%8;             // calcula bit
                  
                  /*guarda estados y temperatura en buffer en turno*/
                  
                  //MyDevicesTemp[RcvNodeNumber-1] = (DevStatusTemp&0x7F);   // get temperature value
                  SET_8BIT(ConnectedDevices[j],i);
                  
                  if(BuffNumber == 0)
                  {
                      MyDevicesTempBuff0[RcvNodeNumber-1] = (DevStatusTemp&0x7F);   // get temperature value
                      SET_8BIT(ConnectedDevicesBuff0[j],i);
                  
                      if(QUERY_8BIT(DevStatusTemp,7) == 1)
                      {
                        SET_8BIT(myDevicesOutputStatusBuff0[j],i);
                      }
                      else
                      {
                        CLEAR_8BIT(myDevicesOutputStatusBuff0[j],i);
                      }
                  }
                  else
                  {
                      MyDevicesTempBuff1[RcvNodeNumber-1] = (DevStatusTemp&0x7F);   // get temperature value
                      SET_8BIT(ConnectedDevicesBuff1[j],i);
                  
                      if(QUERY_8BIT(DevStatusTemp,7) == 1)
                      {
                        SET_8BIT(myDevicesOutputStatusBuff1[j],i);
                      }
                      else
                      {
                        CLEAR_8BIT(myDevicesOutputStatusBuff1[j],i);
                      }
                  }
                  
 
                 RxCmdNum++;
                 LED_1 ^= 1;
                 
             }
         }
         break;
         
         default:
         {
             Uart1RXState = 0;
         }
         break;
     }
     
     //UartRxBuff[UartRxBuffIndex] =  data;
     //++UartRxBuffIndex;
	}

	// TX interrupt
	if (IFS1bits.U1TXIF)
	{
		IFS1bits.U1TXIF = 0;
	}
}

/*----------------------------------------------------------------------------*/


void __ISR(_CORE_TIMER_VECTOR, ipl2) CoreTimerHandler(void)   
{   
    _U16 Scaler;
    
    // clear the interrupt flag   
    mCTClearIntFlag();   
   
    // .. things to do   
   
    // update the period   
    UpdateCoreTimer(CORE_TICK_RATE);  
    
    ++Tick;
    ++EepromTimer;

   
   TickScaler = BaseTimeToTx;
    switch(TickScaler)
    {
        case 0:  Scaler = 10;    break;
        case 1:  Scaler = 1;     break;     // 25 ms
        case 2:  Scaler = 2;     break;     // 50 ms
        case 3:  Scaler = 4;     break;     // 100 ms
        case 4:  Scaler = 8;     break;     // 200 ms
        case 5:  Scaler = 16;    break;     // 400 ms
        case 6:  Scaler = 32;    break;     // 800 ms
        case 7:  Scaler = 64;    break;     // 1.6 s
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
/*----------------------------------------------------------------------------*/
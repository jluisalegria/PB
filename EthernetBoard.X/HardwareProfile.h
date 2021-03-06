/********************************************************************
* FileName:		HardwareProfile.h
* Dependencies:    
* Processor:	PIC18, PIC24, PIC32, dsPIC30, dsPIC33
*               tested with 18F4620, dsPIC33FJ256GP710	
* Complier:     Microchip C18 v3.04 or higher
*				Microchip C30 v2.03 or higher	
*               Microchip C32 v1.02 or higher	
* Company:		Microchip Technology, Inc.
*
* Copyright and Disclaimer Notice
*
* Copyright � 2007-2010 Microchip Technology Inc.  All rights reserved.
*
* Microchip licenses to you the right to use, modify, copy and distribute 
* Software only when embedded on a Microchip microcontroller or digital 
* signal controller and used with a Microchip radio frequency transceiver, 
* which are integrated into your product or third party product (pursuant 
* to the terms in the accompanying license agreement).   
*
* You should refer to the license agreement accompanying this Software for 
* additional information regarding your rights and obligations.
*
* SOFTWARE AND DOCUMENTATION ARE PROVIDED �AS IS� WITHOUT WARRANTY OF ANY 
* KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY 
* WARRANTY OF MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A 
* PARTICULAR PURPOSE. IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE 
* LIABLE OR OBLIGATED UNDER CONTRACT, NEGLIGENCE, STRICT LIABILITY, 
* CONTRIBUTION, BREACH OF WARRANTY, OR OTHER LEGAL EQUITABLE THEORY ANY 
* DIRECT OR INDIRECT DAMAGES OR EXPENSES INCLUDING BUT NOT LIMITED TO 
* ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR CONSEQUENTIAL DAMAGES, 
* LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF SUBSTITUTE GOODS, 
* TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES (INCLUDING BUT 
* NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
*
*********************************************************************
* File Description:
*
*  This file defines functions used for demo board hardware
*
* Change History:
*  Rev   Date         Author    Description
*  1.0   2/15/2009    yfy       Initial revision
*  2.0   4/15/2009    yfy       MiMAC and MiApp revision
********************************************************************/

#ifndef _HARDWARE_PROFILE_H
    #define _HARDWARE_PROFILE_H
    
    #include "GenericTypeDefs.h"
    #include "ConfigApp.h"
    
    /*********************************************************************/
    // Choose one of the demo board that support the demo out of the box
    /*********************************************************************/
    //#define PICDEMZ
    //#define PIC18_EXPLORER
    #define EXPLORER16
    //#define EIGHT_BIT_WIRELESS_BOARD

    // there are three ways to use NVM to store data: External EPROM, Data EEPROM and 
    // programming space, with following definitions:
    //  #define USE_EXTERNAL_EEPROM
    //  #define USE_DATA_EEPROM
    //  #define USE_PROGRAMMING_SPACE  
    // Each demo board has defined the method of using NVM, as
    // required by Network Freezer feature.
    
    #if defined(EIGHT_BIT_WIRELESS_BOARD)
        #define CLOCK_FREQ          8000000
        #define USE_EXTERNAL_EEPROM
        
        // Define EEPROM_SHARE_SPI if external EEPROM shares the same 
        // SPI with transceiver
        #define EEPROM_SHARE_SPI
        
        // 8-bit wireless demo board can use the sensor port to
        // connect to either a LCD or a RS232 serial port
        #define SENSOR_PORT_LCD
        #define SENSOR_PORT_UART
        
        #if defined(MRF49XA)
            #define RFIF            INTCON3bits.INT3IF
            #define RFIE            INTCON3bits.INT3IE
            #define PHY_CS          LATCbits.LATC2
            #define PHY_CS_TRIS     TRISCbits.TRISC2
            #define RF_INT_PIN      PORTBbits.RB3
            #define RF_INT_TRIS     TRISBbits.TRISB3
            #define nFSEL           LATEbits.LATE1
            #define nFSEL_TRIS      TRISEbits.TRISE1
            #define FINT            PORTDbits.RD3
            #define FINT_TRIS       TRISDbits.TRISD3
			#define RF_EEnCS_TRIS	TRISEbits.TRISE2
			#define RF_EEnCS		LATEbits.LATE2
        #elif defined(MRF24J40)
            #define RFIF            INTCON3bits.INT1IF
            #define RFIE            INTCON3bits.INT1IE
            #define PHY_CS          LATBbits.LATB3
            #define PHY_CS_TRIS     TRISBbits.TRISB3
            #define RF_INT_PIN      PORTBbits.RB1
            #define RF_INT_TRIS     TRISBbits.TRISB1
            #define PHY_WAKE        LATDbits.LATD3
            #define PHY_WAKE_TRIS   TRISDbits.TRISD3
			#define RF_EEnCS_TRIS	TRISCbits.TRISC2
			#define RF_EEnCS		LATCbits.LATC2
		#elif defined(MRF89XA)
			#define IRQ1_INT_PIN	PORTBbits.RB3
			#define IRQ1_INT_TRIS	TRISBbits.TRISB3
			#define PHY_IRQ1        INTCON3bits.INT3IF
			#define PHY_IRQ1_En     INTCON3bits.INT3IE
			#define Config_nCS      LATEbits.LATE1
		    #define Config_nCS_TRIS TRISEbits.TRISE1
			#define Data_nCS        LATCbits.LATC2
			#define Data_nCS_TRIS   TRISCbits.TRISC2
			#define IRQ0_INT_PIN    PORTDbits.RD3
			#define IRQ0_INT_TRIS   TRISDbits.TRISD3
			#define RF_EEnCS_TRIS	TRISEbits.TRISE0
			#define RF_EEnCS		LATEbits.LATE0
			#define PHY_RESETn      LATDbits.LATB7
			#define PHY_RESETn_TRIS TRISDbits.TRISD7
			//If using both IRQ0 and IRQ1 then uncomment the following lines
			//Also define the PHY_IRQ0 connections
			//#define PHY_IRQ0		
			//#define PHY_IRQ0_En		
			//#define USE_IRQ0_AS_INTERRUPT in ConfigMRF89XA.h
        #endif
        
        #define SPI_SDI             PORTBbits.RB5               
        #define SDI_TRIS            TRISBbits.TRISB5
        #define SPI_SDO             LATCbits.LATC7               
        #define SDO_TRIS            TRISCbits.TRISC7
        #define SPI_SCK             LATBbits.LATB4               
        #define SCK_TRIS            TRISBbits.TRISB4
        
		#if defined(MRF24J40) || defined(MRF49XA)
        	#define PHY_RESETn          LATDbits.LATD7
        	#define PHY_RESETn_TRIS     TRISDbits.TRISD7
        #endif

        #define PUSH_BUTTON_1       PORTBbits.RB0
        #define BUTTON_1_TRIS       TRISBbits.TRISB0
        #define PUSH_BUTTON_2       PORTBbits.RB2
        #define BUTTON_2_TRIS       TRISBbits.TRISB2
        
        
        #define LED_1               LATAbits.LATA2
        #define LED_1_TRIS          TRISAbits.TRISA2
        #define LED_2               LATAbits.LATA3
        #define LED_2_TRIS          TRISAbits.TRISA3
    
        #define EE_nCS_TRIS         TRISDbits.TRISD5
        #define EE_nCS              LATDbits.LATD5
        #define TMRL                TMR0L   
               
    #endif
    
    #if defined(PIC18_EXPLORER) 
        #define CLOCK_FREQ          12000000
        #define USE_EXTERNAL_EEPROM
        //#define EEPROM_SHARE_SPI
         //#define USE_PROGRAMMING_SPACE

        #define SUPPORT_TWO_SPI
        
        // Transceiver Configuration
        #if defined(MRF49XA)
            #define RFIF            INTCON3bits.INT3IF
            #define RFIE            INTCON3bits.INT3IE
            #define PHY_CS          LATCbits.LATC2
            #define PHY_CS_TRIS     TRISCbits.TRISC2
            #define RF_INT_PIN      PORTBbits.RB3
            #define RF_INT_TRIS     TRISBbits.TRISB3
            #define nFSEL           LATEbits.LATE3             
            #define nFSEL_TRIS      TRISEbits.TRISE3
            #define FINT		    PORTBbits.RB4               
            #define FINT_TRIS       TRISBbits.TRISB4
        #elif defined(MRF24J40)
            #define RFIF            INTCON3bits.INT1IF
            #define RFIE            INTCON3bits.INT1IE
            #define PHY_CS          LATBbits.LATB3
            #define PHY_CS_TRIS     TRISBbits.TRISB3
            #define RF_INT_PIN      PORTBbits.RB1
            #define RF_INT_TRIS     TRISBbits.TRISB1

            #define PHY_WAKE        LATBbits.LATB4
            #define PHY_WAKE_TRIS   TRISBbits.TRISB4
		#elif defined(MRF89XA)


         #define IRQ1_INT_PIN    PORTDbits.RD2
         #define IRQ1_INT_TRIS   TRISDbits.TRISD2

         #define PHY_IRQ1        INTCON3bits.INT3IF
         #define PHY_IRQ1_En     INTCON3bits.INT3IE

         #define IRQ0_INT_TRIS   TRISDbits.TRISD3
         #define IRQ0_INT_PIN    PORTDbits.RD3

         #define Config_nCS      LATDbits.LATD1
          #define Config_nCS_TRIS TRISDbits.TRISD1

         #define Data_nCS        LATDbits.LATD0
         #define Data_nCS_TRIS   TRISDbits.TRISD0

         //If using both IRQ0 and IRQ1 then uncomment the following lines
         //Also define the PHY_IRQ0 connections
         //#define PHY_IRQ0
         //#define PHY_IRQ0_En
         //#define USE_IRQ0_AS_INTERRUPT //in ConfigMRF89XA.h



			
			//#define IRQ1_INT_PIN    PORTBbits.RB1
			//#define IRQ1_INT_TRIS	TRISBbits.TRISB1
			//#define PHY_IRQ1        INTCON3bits.INT1IF
			//#define PHY_IRQ1_En     INTCON3bits.INT1IE

			//#define IRQ0_INT_TRIS	TRISBbits.TRISB0
			//#define IRQ0_INT_PIN    PORTBbits.RB0

			//#define Config_nCS      LATBbits.LATB2
		    //#define Config_nCS_TRIS TRISBbits.TRISB2

			//#define Data_nCS        LATBbits.LATB3
			//#define Data_nCS_TRIS   TRISBbits.TRISB3

			//If using both IRQ0 and IRQ1 then uncomment the following lines
			//Also define the PHY_IRQ0 connections
			//#define PHY_IRQ0	INTCONbits.INT0IF
			//#define PHY_IRQ0_En	INTCONbits.INT0IE
			//#define USE_IRQ0_AS_INTERRUPT //in ConfigMRF89XA.h
        #endif
    
        #define SPI_SDI             PORTCbits.RC4
        #define SDI_TRIS            TRISCbits.TRISC4
        #define SPI_SDO             LATCbits.LATC5
        #define SDO_TRIS            TRISCbits.TRISC5

        #define SPI_SCK             LATCbits.LATC3
        #define SCK_TRIS            TRISCbits.TRISC3

        // SPI2 Pin Definitions
        #define SPI_SDI2            PORTDbits.RD7
        #define SDI2_TRIS           TRISDbits.TRISD7
        #define SPI_SDO2            LATDbits.LATD6
        #define SDO2_TRIS           TRISDbits.TRISD6
        #define SPI_SCK2            LATDbits.LATD5
        #define SCK2_TRIS           TRISDbits.TRISD5

        #define SPI2SSPIF           PIR3bits.SSP2IF
        #define SPI2WCOL            SSP2CON1bits.WCOL
        #define SPI2SSPBUF          SSP2BUF

        #define PHY_RESETn          LATCbits.LATC2
        #define PHY_RESETn_TRIS     TRISCbits.TRISC2
        
        #define PUSH_BUTTON_1       PORTAbits.RA2
        #define BUTTON_1_TRIS       TRISAbits.TRISA2
        #define PUSH_BUTTON_2       PORTAbits.RA3
        #define BUTTON_2_TRIS       TRISAbits.TRISA3
        
        
        #define LED_1               LATBbits.LATB2
        #define LED_1_TRIS          TRISBbits.TRISB2
        #define LED_2               LATBbits.LATB3
        #define LED_2_TRIS          TRISBbits.TRISB3

          // External EEPROM Pin Definitions
		//#define EE_nCS_TRIS         TRISDbits.TRISD5
        #define MAC_nCS             LATBbits.LATB1
        #define MAC_nCS_TRIS        TRISBbits.TRISB1

        //External SST Serial Flash Definitions
        #define EE_nCS              LATBbits.LATB0
        #define EE_nCS_TRIS         TRISBbits.TRISB0

        #define CTS                 LATEbits.LATE1
        #define CTS_TRIS            TRISEbits.TRISE1


        #define ON_TEMP             LATDbits.LATD4
        #define ON_TEMP_TRIS        TRISDbits.TRISD4

        #define ON_MAC              LATBbits.LATB5
        #define ON_MAC_TRIS         TRISBbits.TRISB5

        #define ON_EEPROM           LATAbits.LATA0
        #define ON_EEPROM_TRIS      TRISAbits.TRISA0

        #define ON_RADIO               LATEbits.LATE2
        #define ON_RADIO_TRIS          TRISEbits.TRISE2

        #define RLY_1               LATEbits.LATE0
        #define RLY_1_TRIS          TRISEbits.TRISE0

        #define RLY_2               LATEbits.LATE1
        #define RLY_2_TRIS          TRISEbits.TRISE1

        #define DRV_485             LATDbits.LATD4
        #define DRV_485_TRIS        TRISDbits.TRISD4

    





        #define TMRL                TMR0L
    #endif
    
    
    #if defined(PICDEMZ)
        #define CLOCK_FREQ          16000000
        #define USE_DATA_EEPROM
        
        // Transceiver Configuration
        #define RFIF                INTCONbits.INT0IF
        #define RFIE                INTCONbits.INT0IE
        #define PHY_CS              LATCbits.LATC0
        #define PHY_CS_TRIS         TRISCbits.TRISC0
        #define PHY_RESETn          LATCbits.LATC2
        #define PHY_RESETn_TRIS     TRISCbits.TRISC2
        #define PHY_WAKE            LATCbits.LATC1
        #define PHY_WAKE_TRIS       TRISCbits.TRISC1
        
        #define PUSH_BUTTON_1       PORTBbits.RB5
        #define PUSH_BUTTON_2       PORTBbits.RB4
        #define LED_1               LATAbits.LATA0
        #define LED_2               LATAbits.LATA1
        
        #define BUTTON_1_TRIS       TRISBbits.TRISB5
        #define BUTTON_2_TRIS       TRISBbits.TRISB4
        #define LED_1_TRIS          TRISAbits.TRISA0
        #define LED_2_TRIS          TRISAbits.TRISA1
        
        #define RF_INT_PIN          PORTBbits.RB0
        #define RF_INT_TRIS         TRISBbits.TRISB0
        
        #define SPI_SDI             PORTCbits.RC4               
        #define SDI_TRIS            TRISCbits.TRISC4
        #define SPI_SDO             LATCbits.LATC5               
        #define SDO_TRIS            TRISCbits.TRISC5
        #define SPI_SCK             LATCbits.LATC3               
        #define SCK_TRIS            TRISCbits.TRISC3
        
        #define TMRL                TMR0L
    #endif
    
    
    #if defined(EXPLORER16) 
    
        #if defined(__PIC32MX__)
            #if !defined(MRF89XA)
            	#define RFIF            IFS0bits.INT1IF
            	#define RFIE            IEC0bits.INT1IE
            #endif
            #define CLOCK_FREQ      12000000
            
        #else
            #if !defined(MRF89XA)
                #define RFIF            IFS1bits.INT1IF
                #define RFIE            IEC1bits.INT1IE
            #endif
            #define CLOCK_FREQ      8000000
        #endif
    
        #if defined(__PIC24F__) || defined(__PIC24FK__) || defined(__PIC32MX__)   
            #if defined(MRF89XA)
                #define IRQ1_INT_PIN      PORTCbits.RC7
                #define IRQ1_INT_TRIS     TRISCbits.TRISC7
                #define IRQ0_INT_PIN      PORTBbits.RB7
                #define IRQ0_INT_TRIS     TRISBbits.TRISB7

                //On Explorer 16 with PICtail plus connections we can use both IRQ0 and IRQ1 as interrupt lines
				//instead of using IRQ0 as port pin and IRQ1 as interrupt pin to the PIC
				//Enable USE_IRQ0_AS_INTERRUPT in ConfigMRF89XA.h
                
            #else
                #define RF_INT_PIN      PORTEbits.RE8   
                #define RF_INT_TRIS     TRISEbits.TRISE8
            #endif
            
        #elif defined(__dsPIC33F__) || defined(__PIC24H__)
            #define RF_INT_PIN      PORTAbits.RA12
            #define RF_INT_TRIS     TRISAbits.TRISA12
        #endif
        
        #define USE_EXTERNAL_EEPROM
        
        
        // Transceiver Configuration
		#if (defined(MRF24J40) || defined(MRF49XA))
        #define PHY_CS              LATBbits.LATB2
        #define PHY_CS_TRIS         TRISBbits.TRISB2
        #define PHY_RESETn          LATGbits.LATG2
        #define PHY_RESETn_TRIS     TRISGbits.TRISG2
        #endif

        #define SPI_SDI             PORTBbits.RB5
        #define SDI_TRIS            TRISBbits.TRISB5
        #define SPI_SDO             LATBbits.LATB6
        #define SDO_TRIS            TRISBbits.TRISB6
        #define SPI_SCK             LATBbits.LATB14
        #define SCK_TRIS            TRISBbits.TRISB14


        #define SPI_SDI2             PORTAbits.RA4
        #define SDI2_TRIS            TRISAbits.TRISA4
        #define SPI_SDO2             LATCbits.LATC8
        #define SDO2_TRIS            TRISCbits.TRISC8
        #define SPI_SCK2             LATBbits.LATB15
        #define SCK2_TRIS            TRISBbits.TRISB15


        
        #if defined(MRF49XA)
            #define nFSEL           LATBbits.LATB1              
            #define nFSEL_TRIS      TRISBbits.TRISB1
            #define FINT		    PORTEbits.RE9 
            #define FINT_TRIS       TRISEbits.TRISE9
        #elif defined(MRF24J40)
            #define PHY_WAKE        LATGbits.LATG3
            #define PHY_WAKE_TRIS   TRISGbits.TRISG3
		#elif defined(MRF89XA)
			#if defined(__PIC32MX__)
				//#define PHY_IRQ1        IFS0bits.INT2IF
				//#define PHY_IRQ1_En     IEC0bits.INT2IE

                #define PHY_IRQ1        IFS0bits.INT4IF
				#define PHY_IRQ1_En     IEC0bits.INT4IE
				#define PHY_IRQ0		IFS0bits.INT1IF
				#define PHY_IRQ0_En		IEC0bits.INT1IE
			#else
				#define PHY_IRQ1        IFS1bits.INT2IF
				#define PHY_IRQ1_En     IEC1bits.INT2IE
				#define PHY_IRQ0		IFS1bits.INT1IF
				#define PHY_IRQ0_En		IEC1bits.INT1IE
			#endif
			#define PHY_IRQ1_TRIS   TRISCbits.TRISC7
			#define Config_nCS      LATCbits.LATC4
		    #define Config_nCS_TRIS TRISCbits.TRISC4
			#define Data_nCS        LATCbits.LATC5
			#define Data_nCS_TRIS   TRISCbits.TRISC5
			#define PHY_RESETn      LATCbits.LATC3
			#define PHY_RESETn_TRIS TRISCbits.TRISC3
									                                
        #endif
        
        #define PUSH_BUTTON_1       PORTBbits.RB0
        #define PUSH_BUTTON_2       PORTBbits.RB1
        #define LED_1               LATAbits.LATA8
        #define LED_2               LATAbits.LATA9
        
        #define BUTTON_1_TRIS       TRISBbits.TRISB0
        #define BUTTON_2_TRIS       TRISBbits.TRISB1
        #define LED_1_TRIS          TRISAbits.TRISA8
        #define LED_2_TRIS          TRISAbits.TRISA9
        
        // Define SUPPORT_TWO_SPI if external EEPROM use the second SPI
        // port alone, not sharing SPI port with the transceiver
        #define SUPPORT_TWO_SPI
        
        // External EEPROM SPI chip select pin definition
        #define EE_nCS_TRIS         TRISBbits.TRISB10
        #define EE_nCS              LATBbits.LATB10

        #define CTS                 LATCbits.LATC1
        #define CTS_TRIS            TRISCbits.TRISC1

        #define MAC_nCS             LATBbits.LATB11
        #define MAC_nCS_TRIS        TRISBbits.TRISB11

        
        #define TMRL TMR2
    #endif
    

    #if defined(PICDEMZ)
        #define CLOCK_FREQ          16000000
        #define USE_DATA_EEPROM
        
        // Transceiver Configuration
        #define RFIF                INTCONbits.INT0IF
        #define RFIE                INTCONbits.INT0IE
        #define PHY_CS              LATCbits.LATC0
        #define PHY_CS_TRIS         TRISCbits.TRISC0
        #define PHY_RESETn          LATCbits.LATC2
        #define PHY_RESETn_TRIS     TRISCbits.TRISC2
        #define PHY_WAKE            LATCbits.LATC1
        #define PHY_WAKE_TRIS       TRISCbits.TRISC1
        
        #define PUSH_BUTTON_1       PORTBbits.RB5
        #define PUSH_BUTTON_2       PORTBbits.RB4
        #define LED_1               LATAbits.LATA0
        #define LED_2               LATAbits.LATA1
        
        #define BUTTON_1_TRIS       TRISBbits.TRISB5
        #define BUTTON_2_TRIS       TRISBbits.TRISB4
        #define LED_1_TRIS          TRISAbits.TRISA0
        #define LED_2_TRIS          TRISAbits.TRISA1
        
        #define RF_INT_PIN          PORTBbits.RB0
        #define RF_INT_TRIS         TRISBbits.TRISB0
        
        #define SPI_SDI             PORTCbits.RC4               
        #define SDI_TRIS            TRISCbits.TRISC4
        #define SPI_SDO             LATCbits.LATC5               
        #define SDO_TRIS            TRISCbits.TRISC5
        #define SPI_SCK             LATCbits.LATC3               
        #define SCK_TRIS            TRISCbits.TRISC3
        
        #define TMRL                TMR0L
    #endif        


    // Following definitions are used for LCD display on the demo board
    #if defined(EXPLORER16)
/*
    	#define LCD_DATA0_TRIS		(TRISEbits.TRISE0)		// Multiplexed with LED6
    	#define LCD_DATA0_IO		(LATEbits.LATE0)
    	#define LCD_DATA1_TRIS		(TRISEbits.TRISE1)
    	#define LCD_DATA1_IO		(LATEbits.LATE1)
    	#define LCD_DATA2_TRIS		(TRISEbits.TRISE2)
    	#define LCD_DATA2_IO		(LATEbits.LATE2)
    	#define LCD_DATA3_TRIS		(TRISEbits.TRISE3)		// Multiplexed with LED3
    	#define LCD_DATA3_IO		(LATEbits.LATE3)
    	#define LCD_DATA4_TRIS		(TRISEbits.TRISE4)		// Multiplexed with LED2
    	#define LCD_DATA4_IO		(LATEbits.LATE4)
    	#define LCD_DATA5_TRIS		(TRISEbits.TRISE5)
    	#define LCD_DATA5_IO		(LATEbits.LATE5)
    	#define LCD_DATA6_TRIS		(TRISEbits.TRISE6)
    	#define LCD_DATA6_IO		(LATEbits.LATE6)
    	#define LCD_DATA7_TRIS		(TRISEbits.TRISE7)
    	#define LCD_DATA7_IO		(LATEbits.LATE7)
    	#define LCD_RD_WR_TRIS		(TRISDbits.TRISD5)
    	#define LCD_RD_WR_IO		(LATDbits.LATD5)
    	#define LCD_RS_TRIS			(TRISBbits.TRISB15)
    	#define LCD_RS_IO			(LATBbits.LATB15)
    	#define LCD_E_TRIS			(TRISDbits.TRISD4)
    	#define LCD_E_IO			(LATDbits.LATD4)
*/
    
    #elif defined(PIC18_EXPLORER)
    
        //#define LCD_CS_TRIS         (TRISAbits.TRISA2)
        //#define LCD_CS              (LATAbits.LATA2)
        //#define LCD_RESET_TRIS      (TRISAbits.TRISA3)
        //#define LCD_RESET           (LATAbits.LATA3)
    
    #elif defined(EIGHT_BIT_WIRELESS_BOARD)
    
        #define SUPPORT_TWO_SPI
        #define SPInCS_TRIS         TRISCbits.TRISC6
        #define SPInCS_LAT	        LATCbits.LATC6
    	#define	SPISSPSTAT	        SSP2STAT
    	#define SPISSPCON	        SSP2CON1
    	#define SPISSPBUF	        SSP2BUF
    	#define SPIWCOL		        SSP2CON1bits.WCOL
    	#define SPISSPIE	        PIE3bits.SSP2IE
    	#define SPISSPIF	        PIR3bits.SSP2IF
    	
    #endif
    
    // Following definition is for delay functionality
    #if defined(__18CXX)
        #define GetInstructionClock()	(CLOCK_FREQ/4)
    #elif defined(__C30__) 
        #define GetInstructionClock()	(CLOCK_FREQ/2)
    #elif defined(__PIC32MX__)
        #define GetInstructionClock()	(CLOCK_FREQ)
    #endif

    #define SW1             1
    #define SW2             2

    BYTE ButtonPressed(void);
    void BoardInit(void);

    void LCDDisplay(char *, BYTE, BOOL);
    void LCDTRXCount(BYTE, BYTE);

    
#endif

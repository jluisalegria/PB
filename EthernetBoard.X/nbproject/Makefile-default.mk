#
# Generated Makefile - do not edit!
#
# Edit the Makefile in the project folder instead (../Makefile). Each target
# has a -pre and a -post target defined where you can add customized code.
#
# This makefile implements configuration specific macros and targets.


# Include project Makefile
ifeq "${IGNORE_LOCAL}" "TRUE"
# do not include local makefile. User is passing all local related variables already
else
include Makefile
# Include makefile containing local settings
ifeq "$(wildcard nbproject/Makefile-local-default.mk)" "nbproject/Makefile-local-default.mk"
include nbproject/Makefile-local-default.mk
endif
endif

# Environment
MKDIR=gnumkdir -p
RM=rm -f 
MV=mv 
CP=cp 

# Macros
CND_CONF=default
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
IMAGE_TYPE=debug
OUTPUT_SUFFIX=elf
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PlayaBonitaEthernetBoard.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/PlayaBonitaEthernetBoard.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=ethernet_board.c HardwareProfile.c MAC_EEProm.c TimeDelay.c MiWiPRO.c Console.c EEPROM.c MSPI.c NVM.c SymbolTime.c MRF89XA.c crc.c security.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/ethernet_board.o ${OBJECTDIR}/HardwareProfile.o ${OBJECTDIR}/MAC_EEProm.o ${OBJECTDIR}/TimeDelay.o ${OBJECTDIR}/MiWiPRO.o ${OBJECTDIR}/Console.o ${OBJECTDIR}/EEPROM.o ${OBJECTDIR}/MSPI.o ${OBJECTDIR}/NVM.o ${OBJECTDIR}/SymbolTime.o ${OBJECTDIR}/MRF89XA.o ${OBJECTDIR}/crc.o ${OBJECTDIR}/security.o
POSSIBLE_DEPFILES=${OBJECTDIR}/ethernet_board.o.d ${OBJECTDIR}/HardwareProfile.o.d ${OBJECTDIR}/MAC_EEProm.o.d ${OBJECTDIR}/TimeDelay.o.d ${OBJECTDIR}/MiWiPRO.o.d ${OBJECTDIR}/Console.o.d ${OBJECTDIR}/EEPROM.o.d ${OBJECTDIR}/MSPI.o.d ${OBJECTDIR}/NVM.o.d ${OBJECTDIR}/SymbolTime.o.d ${OBJECTDIR}/MRF89XA.o.d ${OBJECTDIR}/crc.o.d ${OBJECTDIR}/security.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/ethernet_board.o ${OBJECTDIR}/HardwareProfile.o ${OBJECTDIR}/MAC_EEProm.o ${OBJECTDIR}/TimeDelay.o ${OBJECTDIR}/MiWiPRO.o ${OBJECTDIR}/Console.o ${OBJECTDIR}/EEPROM.o ${OBJECTDIR}/MSPI.o ${OBJECTDIR}/NVM.o ${OBJECTDIR}/SymbolTime.o ${OBJECTDIR}/MRF89XA.o ${OBJECTDIR}/crc.o ${OBJECTDIR}/security.o

# Source Files
SOURCEFILES=ethernet_board.c HardwareProfile.c MAC_EEProm.c TimeDelay.c MiWiPRO.c Console.c EEPROM.c MSPI.c NVM.c SymbolTime.c MRF89XA.c crc.c security.c


CFLAGS=
ASFLAGS=
LDLIBSOPTIONS=

############# Tool locations ##########################################
# If you copy a project from one host to another, the path where the  #
# compiler is installed may be different.                             #
# If you open this project with MPLAB X in the new host, this         #
# makefile will be regenerated and the paths will be corrected.       #
#######################################################################
# fixDeps replaces a bunch of sed/cat/printf statements that slow down the build
FIXDEPS=fixDeps

.build-conf:  ${BUILD_SUBPROJECTS}
ifneq ($(INFORMATION_MESSAGE), )
	@echo $(INFORMATION_MESSAGE)
endif
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/PlayaBonitaEthernetBoard.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=32MX150F128D
MP_LINKER_FILE_OPTION=
# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assembleWithPreprocess
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/ethernet_board.o: ethernet_board.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ethernet_board.o.d 
	@${RM} ${OBJECTDIR}/ethernet_board.o 
	@${FIXDEPS} "${OBJECTDIR}/ethernet_board.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/ethernet_board.o.d" -o ${OBJECTDIR}/ethernet_board.o ethernet_board.c  
	
${OBJECTDIR}/HardwareProfile.o: HardwareProfile.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/HardwareProfile.o.d 
	@${RM} ${OBJECTDIR}/HardwareProfile.o 
	@${FIXDEPS} "${OBJECTDIR}/HardwareProfile.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/HardwareProfile.o.d" -o ${OBJECTDIR}/HardwareProfile.o HardwareProfile.c  
	
${OBJECTDIR}/MAC_EEProm.o: MAC_EEProm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MAC_EEProm.o.d 
	@${RM} ${OBJECTDIR}/MAC_EEProm.o 
	@${FIXDEPS} "${OBJECTDIR}/MAC_EEProm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/MAC_EEProm.o.d" -o ${OBJECTDIR}/MAC_EEProm.o MAC_EEProm.c  
	
${OBJECTDIR}/TimeDelay.o: TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/TimeDelay.o 
	@${FIXDEPS} "${OBJECTDIR}/TimeDelay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/TimeDelay.o.d" -o ${OBJECTDIR}/TimeDelay.o TimeDelay.c  
	
${OBJECTDIR}/MiWiPRO.o: MiWiPRO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MiWiPRO.o.d 
	@${RM} ${OBJECTDIR}/MiWiPRO.o 
	@${FIXDEPS} "${OBJECTDIR}/MiWiPRO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/MiWiPRO.o.d" -o ${OBJECTDIR}/MiWiPRO.o MiWiPRO.c  
	
${OBJECTDIR}/Console.o: Console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Console.o.d 
	@${RM} ${OBJECTDIR}/Console.o 
	@${FIXDEPS} "${OBJECTDIR}/Console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/Console.o.d" -o ${OBJECTDIR}/Console.o Console.c  
	
${OBJECTDIR}/EEPROM.o: EEPROM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/EEPROM.o.d 
	@${RM} ${OBJECTDIR}/EEPROM.o 
	@${FIXDEPS} "${OBJECTDIR}/EEPROM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/EEPROM.o.d" -o ${OBJECTDIR}/EEPROM.o EEPROM.c  
	
${OBJECTDIR}/MSPI.o: MSPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MSPI.o.d 
	@${RM} ${OBJECTDIR}/MSPI.o 
	@${FIXDEPS} "${OBJECTDIR}/MSPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/MSPI.o.d" -o ${OBJECTDIR}/MSPI.o MSPI.c  
	
${OBJECTDIR}/NVM.o: NVM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/NVM.o.d 
	@${RM} ${OBJECTDIR}/NVM.o 
	@${FIXDEPS} "${OBJECTDIR}/NVM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/NVM.o.d" -o ${OBJECTDIR}/NVM.o NVM.c  
	
${OBJECTDIR}/SymbolTime.o: SymbolTime.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SymbolTime.o.d 
	@${RM} ${OBJECTDIR}/SymbolTime.o 
	@${FIXDEPS} "${OBJECTDIR}/SymbolTime.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/SymbolTime.o.d" -o ${OBJECTDIR}/SymbolTime.o SymbolTime.c  
	
${OBJECTDIR}/MRF89XA.o: MRF89XA.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MRF89XA.o.d 
	@${RM} ${OBJECTDIR}/MRF89XA.o 
	@${FIXDEPS} "${OBJECTDIR}/MRF89XA.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/MRF89XA.o.d" -o ${OBJECTDIR}/MRF89XA.o MRF89XA.c  
	
${OBJECTDIR}/crc.o: crc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/crc.o.d 
	@${RM} ${OBJECTDIR}/crc.o 
	@${FIXDEPS} "${OBJECTDIR}/crc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/crc.o.d" -o ${OBJECTDIR}/crc.o crc.c  
	
${OBJECTDIR}/security.o: security.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/security.o.d 
	@${RM} ${OBJECTDIR}/security.o 
	@${FIXDEPS} "${OBJECTDIR}/security.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE) -g -D__DEBUG -D__MPLAB_DEBUGGER_PK3=1  -fframe-base-loclist  -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/security.o.d" -o ${OBJECTDIR}/security.o security.c  
	
else
${OBJECTDIR}/ethernet_board.o: ethernet_board.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/ethernet_board.o.d 
	@${RM} ${OBJECTDIR}/ethernet_board.o 
	@${FIXDEPS} "${OBJECTDIR}/ethernet_board.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/ethernet_board.o.d" -o ${OBJECTDIR}/ethernet_board.o ethernet_board.c  
	
${OBJECTDIR}/HardwareProfile.o: HardwareProfile.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/HardwareProfile.o.d 
	@${RM} ${OBJECTDIR}/HardwareProfile.o 
	@${FIXDEPS} "${OBJECTDIR}/HardwareProfile.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/HardwareProfile.o.d" -o ${OBJECTDIR}/HardwareProfile.o HardwareProfile.c  
	
${OBJECTDIR}/MAC_EEProm.o: MAC_EEProm.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MAC_EEProm.o.d 
	@${RM} ${OBJECTDIR}/MAC_EEProm.o 
	@${FIXDEPS} "${OBJECTDIR}/MAC_EEProm.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/MAC_EEProm.o.d" -o ${OBJECTDIR}/MAC_EEProm.o MAC_EEProm.c  
	
${OBJECTDIR}/TimeDelay.o: TimeDelay.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/TimeDelay.o.d 
	@${RM} ${OBJECTDIR}/TimeDelay.o 
	@${FIXDEPS} "${OBJECTDIR}/TimeDelay.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/TimeDelay.o.d" -o ${OBJECTDIR}/TimeDelay.o TimeDelay.c  
	
${OBJECTDIR}/MiWiPRO.o: MiWiPRO.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MiWiPRO.o.d 
	@${RM} ${OBJECTDIR}/MiWiPRO.o 
	@${FIXDEPS} "${OBJECTDIR}/MiWiPRO.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/MiWiPRO.o.d" -o ${OBJECTDIR}/MiWiPRO.o MiWiPRO.c  
	
${OBJECTDIR}/Console.o: Console.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/Console.o.d 
	@${RM} ${OBJECTDIR}/Console.o 
	@${FIXDEPS} "${OBJECTDIR}/Console.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/Console.o.d" -o ${OBJECTDIR}/Console.o Console.c  
	
${OBJECTDIR}/EEPROM.o: EEPROM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/EEPROM.o.d 
	@${RM} ${OBJECTDIR}/EEPROM.o 
	@${FIXDEPS} "${OBJECTDIR}/EEPROM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/EEPROM.o.d" -o ${OBJECTDIR}/EEPROM.o EEPROM.c  
	
${OBJECTDIR}/MSPI.o: MSPI.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MSPI.o.d 
	@${RM} ${OBJECTDIR}/MSPI.o 
	@${FIXDEPS} "${OBJECTDIR}/MSPI.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/MSPI.o.d" -o ${OBJECTDIR}/MSPI.o MSPI.c  
	
${OBJECTDIR}/NVM.o: NVM.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/NVM.o.d 
	@${RM} ${OBJECTDIR}/NVM.o 
	@${FIXDEPS} "${OBJECTDIR}/NVM.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/NVM.o.d" -o ${OBJECTDIR}/NVM.o NVM.c  
	
${OBJECTDIR}/SymbolTime.o: SymbolTime.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/SymbolTime.o.d 
	@${RM} ${OBJECTDIR}/SymbolTime.o 
	@${FIXDEPS} "${OBJECTDIR}/SymbolTime.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/SymbolTime.o.d" -o ${OBJECTDIR}/SymbolTime.o SymbolTime.c  
	
${OBJECTDIR}/MRF89XA.o: MRF89XA.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/MRF89XA.o.d 
	@${RM} ${OBJECTDIR}/MRF89XA.o 
	@${FIXDEPS} "${OBJECTDIR}/MRF89XA.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/MRF89XA.o.d" -o ${OBJECTDIR}/MRF89XA.o MRF89XA.c  
	
${OBJECTDIR}/crc.o: crc.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/crc.o.d 
	@${RM} ${OBJECTDIR}/crc.o 
	@${FIXDEPS} "${OBJECTDIR}/crc.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/crc.o.d" -o ${OBJECTDIR}/crc.o crc.c  
	
${OBJECTDIR}/security.o: security.c  nbproject/Makefile-${CND_CONF}.mk
	@${MKDIR} "${OBJECTDIR}" 
	@${RM} ${OBJECTDIR}/security.o.d 
	@${RM} ${OBJECTDIR}/security.o 
	@${FIXDEPS} "${OBJECTDIR}/security.o.d" $(SILENT) -rsi ${MP_CC_DIR}../ -c ${MP_CC} $(MP_EXTRA_CC_PRE)  -g -x c -c -mprocessor=$(MP_PROCESSOR_OPTION) -I"." -I"../../../../CodeBank8" -I"../../../../CodeBank8/bsp/Microchip" -I"../../../../CodeBank8/middleware" -I"../../../../CodeBank8/modules" -MMD -MF "${OBJECTDIR}/security.o.d" -o ${OBJECTDIR}/security.o security.c  
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/PlayaBonitaEthernetBoard.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mdebugger -D__MPLAB_DEBUGGER_PK3=1 -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PlayaBonitaEthernetBoard.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}       -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),--defsym=__ICD2RAM=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,--defsym=__MPLAB_DEBUGGER_PK3=1,-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map" 
else
dist/${CND_CONF}/${IMAGE_TYPE}/PlayaBonitaEthernetBoard.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -mprocessor=$(MP_PROCESSOR_OPTION)  -o dist/${CND_CONF}/${IMAGE_TYPE}/PlayaBonitaEthernetBoard.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} ${OBJECTFILES_QUOTED_IF_SPACED}       -Wl,--defsym=__MPLAB_BUILD=1$(MP_EXTRA_LD_POST)$(MP_LINKER_FILE_OPTION),-Map="${DISTDIR}/${PROJECTNAME}.${IMAGE_TYPE}.map"
	${MP_CC_DIR}\\pic32-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/PlayaBonitaEthernetBoard.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  
endif


# Subprojects
.build-subprojects:


# Subprojects
.clean-subprojects:

# Clean Targets
.clean-conf: ${CLEAN_SUBPROJECTS}
	${RM} -r build/default
	${RM} -r dist/default

# Enable dependency checking
.dep.inc: .depcheck-impl

DEPFILES=$(shell mplabwildcard ${POSSIBLE_DEPFILES})
ifneq (${DEPFILES},)
include ${DEPFILES}
endif

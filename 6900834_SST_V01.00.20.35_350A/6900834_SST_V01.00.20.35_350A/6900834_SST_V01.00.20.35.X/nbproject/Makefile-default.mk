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
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/6900834_SST_V01.00.20.35.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
else
IMAGE_TYPE=production
OUTPUT_SUFFIX=hex
DEBUGGABLE_SUFFIX=elf
FINAL_IMAGE=dist/${CND_CONF}/${IMAGE_TYPE}/6900834_SST_V01.00.20.35.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}
endif

ifeq ($(COMPARE_BUILD), true)
COMPARISON_BUILD=
else
COMPARISON_BUILD=
endif

ifdef SUB_IMAGE_ADDRESS

else
SUB_IMAGE_ADDRESS_COMMAND=
endif

# Object Directory
OBJECTDIR=build/${CND_CONF}/${IMAGE_TYPE}

# Distribution Directory
DISTDIR=dist/${CND_CONF}/${IMAGE_TYPE}

# Source Files Quoted if spaced
SOURCEFILES_QUOTED_IF_SPACED=../UART_Management.c ../6900840_SPI.c ../6900840_Start.c ../6900747_Int2.c ../6900747_StromRegel2.c ../6900747_Konfig.c ../6900747_Traps.c ../6900747_840_Haupt_P.c

# Object Files Quoted if spaced
OBJECTFILES_QUOTED_IF_SPACED=${OBJECTDIR}/_ext/1472/UART_Management.o ${OBJECTDIR}/_ext/1472/6900840_SPI.o ${OBJECTDIR}/_ext/1472/6900840_Start.o ${OBJECTDIR}/_ext/1472/6900747_Int2.o ${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o ${OBJECTDIR}/_ext/1472/6900747_Konfig.o ${OBJECTDIR}/_ext/1472/6900747_Traps.o ${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o
POSSIBLE_DEPFILES=${OBJECTDIR}/_ext/1472/UART_Management.o.d ${OBJECTDIR}/_ext/1472/6900840_SPI.o.d ${OBJECTDIR}/_ext/1472/6900840_Start.o.d ${OBJECTDIR}/_ext/1472/6900747_Int2.o.d ${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o.d ${OBJECTDIR}/_ext/1472/6900747_Konfig.o.d ${OBJECTDIR}/_ext/1472/6900747_Traps.o.d ${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o.d

# Object Files
OBJECTFILES=${OBJECTDIR}/_ext/1472/UART_Management.o ${OBJECTDIR}/_ext/1472/6900840_SPI.o ${OBJECTDIR}/_ext/1472/6900840_Start.o ${OBJECTDIR}/_ext/1472/6900747_Int2.o ${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o ${OBJECTDIR}/_ext/1472/6900747_Konfig.o ${OBJECTDIR}/_ext/1472/6900747_Traps.o ${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o

# Source Files
SOURCEFILES=../UART_Management.c ../6900840_SPI.c ../6900840_Start.c ../6900747_Int2.c ../6900747_StromRegel2.c ../6900747_Konfig.c ../6900747_Traps.c ../6900747_840_Haupt_P.c



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
	${MAKE}  -f nbproject/Makefile-default.mk dist/${CND_CONF}/${IMAGE_TYPE}/6900834_SST_V01.00.20.35.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}

MP_PROCESSOR_OPTION=33EP32GS502
MP_LINKER_FILE_OPTION=,--script="..\p33EP32GS502.gld"
# ------------------------------------------------------------------------------------
# Rules for buildStep: compile
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
${OBJECTDIR}/_ext/1472/UART_Management.o: ../UART_Management.c  .generated_files/8d48461a72c8796ee1b8d7dccb5ddb0a8fca397a.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/UART_Management.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/UART_Management.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../UART_Management.c  -o ${OBJECTDIR}/_ext/1472/UART_Management.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/UART_Management.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900840_SPI.o: ../6900840_SPI.c  .generated_files/bc5f6cd3234580c1721e5c7489039e01838aec29.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900840_SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900840_SPI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900840_SPI.c  -o ${OBJECTDIR}/_ext/1472/6900840_SPI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900840_SPI.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900840_Start.o: ../6900840_Start.c  .generated_files/4b2d94d0f54b8c2f4eb110955219d0a4164f813a.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900840_Start.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900840_Start.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900840_Start.c  -o ${OBJECTDIR}/_ext/1472/6900840_Start.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900840_Start.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900747_Int2.o: ../6900747_Int2.c  .generated_files/a3551c6bcaf85af885aeb0f2affa1917ff53bdf1.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Int2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Int2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900747_Int2.c  -o ${OBJECTDIR}/_ext/1472/6900747_Int2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900747_Int2.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o: ../6900747_StromRegel2.c  .generated_files/77dd3a2a4f3e3890912d369ecc50219156376a12.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900747_StromRegel2.c  -o ${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900747_Konfig.o: ../6900747_Konfig.c  .generated_files/93bcd5d94c873639e1df1f76723cb5a244ca30c1.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Konfig.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Konfig.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900747_Konfig.c  -o ${OBJECTDIR}/_ext/1472/6900747_Konfig.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900747_Konfig.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900747_Traps.o: ../6900747_Traps.c  .generated_files/3f7d9dabb240f4f9b3a0c99fae22e0d66121f578.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Traps.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900747_Traps.c  -o ${OBJECTDIR}/_ext/1472/6900747_Traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900747_Traps.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o: ../6900747_840_Haupt_P.c  .generated_files/6e56c57ffcfadd1752a56a56482ce554a2a6375.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900747_840_Haupt_P.c  -o ${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o.d"      -g -D__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -mno-eds-warn  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
else
${OBJECTDIR}/_ext/1472/UART_Management.o: ../UART_Management.c  .generated_files/5ee3d720d002d91ac2fddc0e290d78001374c3fa.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/UART_Management.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/UART_Management.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../UART_Management.c  -o ${OBJECTDIR}/_ext/1472/UART_Management.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/UART_Management.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900840_SPI.o: ../6900840_SPI.c  .generated_files/cc95dbcd13b8a0898c9fc0b6a80f1016e3a99cf3.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900840_SPI.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900840_SPI.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900840_SPI.c  -o ${OBJECTDIR}/_ext/1472/6900840_SPI.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900840_SPI.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900840_Start.o: ../6900840_Start.c  .generated_files/5bf4b987b17e67a492364514e75990ef4e19969e.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900840_Start.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900840_Start.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900840_Start.c  -o ${OBJECTDIR}/_ext/1472/6900840_Start.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900840_Start.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900747_Int2.o: ../6900747_Int2.c  .generated_files/42d69b2489ce1dc4a4f87a45decc39a1f4619394.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Int2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Int2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900747_Int2.c  -o ${OBJECTDIR}/_ext/1472/6900747_Int2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900747_Int2.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o: ../6900747_StromRegel2.c  .generated_files/650481f448ed8294e417d4f82a62d0bed19c2aec.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900747_StromRegel2.c  -o ${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900747_StromRegel2.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900747_Konfig.o: ../6900747_Konfig.c  .generated_files/bd2551d9f8e8a64b5178fbd3279d120cb9dea186.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Konfig.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Konfig.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900747_Konfig.c  -o ${OBJECTDIR}/_ext/1472/6900747_Konfig.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900747_Konfig.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900747_Traps.o: ../6900747_Traps.c  .generated_files/a8324840791f5ae4e03c1e112787f905ac6b7190.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Traps.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_Traps.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900747_Traps.c  -o ${OBJECTDIR}/_ext/1472/6900747_Traps.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900747_Traps.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o: ../6900747_840_Haupt_P.c  .generated_files/e089387d62c877c0abe2bafa9175fc5ec4064fe9.flag .generated_files/8d205901c45527d22743c792e408e393263604e4.flag
	@${MKDIR} "${OBJECTDIR}/_ext/1472" 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o.d 
	@${RM} ${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o 
	${MP_CC} $(MP_EXTRA_CC_PRE)  ../6900747_840_Haupt_P.c  -o ${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o  -c -mcpu=$(MP_PROCESSOR_OPTION)  -MP -MMD -MF "${OBJECTDIR}/_ext/1472/6900747_840_Haupt_P.o.d"      -mno-eds-warn  -g -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -O1 -I".." -I"." -msmart-io=1 -Wall -msfr-warn=off   
	
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemble
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: assemblePreproc
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
else
endif

# ------------------------------------------------------------------------------------
# Rules for buildStep: link
ifeq ($(TYPE_IMAGE), DEBUG_RUN)
dist/${CND_CONF}/${IMAGE_TYPE}/6900834_SST_V01.00.20.35.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk    ../p33EP32GS502.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/6900834_SST_V01.00.20.35.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -D__DEBUG=__DEBUG -D__MPLAB_DEBUGGER_ICD3=1  -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)   -mreserve=data@0x1000:0x101B -mreserve=data@0x101C:0x101D -mreserve=data@0x101E:0x101F -mreserve=data@0x1020:0x1021 -mreserve=data@0x1022:0x1023 -mreserve=data@0x1024:0x1027 -mreserve=data@0x1028:0x104F   -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,--defsym=__DEBUG=1,-D__DEBUG=__DEBUG,--defsym=__MPLAB_DEBUGGER_ICD3=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..",--library-path=".",--no-force-link,--smart-io,-Map="${DISTDIR}/6900747_840b_EP.X.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  
	
else
dist/${CND_CONF}/${IMAGE_TYPE}/6900834_SST_V01.00.20.35.X.${IMAGE_TYPE}.${OUTPUT_SUFFIX}: ${OBJECTFILES}  nbproject/Makefile-${CND_CONF}.mk   ../p33EP32GS502.gld
	@${MKDIR} dist/${CND_CONF}/${IMAGE_TYPE} 
	${MP_CC} $(MP_EXTRA_LD_PRE)  -o dist/${CND_CONF}/${IMAGE_TYPE}/6900834_SST_V01.00.20.35.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX}  ${OBJECTFILES_QUOTED_IF_SPACED}      -mcpu=$(MP_PROCESSOR_OPTION)        -omf=elf -DXPRJ_default=$(CND_CONF)  -no-legacy-libc  $(COMPARISON_BUILD)  -Wl,--local-stack,,--defsym=__MPLAB_BUILD=1,$(MP_LINKER_FILE_OPTION),--stack=16,--check-sections,--data-init,--pack-data,--handles,--isr,--no-gc-sections,--fill-upper=0,--stackguard=16,--library-path="..",--library-path=".",--no-force-link,--smart-io,-Map="${DISTDIR}/6900747_840b_EP.X.${IMAGE_TYPE}.map",--report-mem,--memorysummary,dist/${CND_CONF}/${IMAGE_TYPE}/memoryfile.xml$(MP_EXTRA_LD_POST)  
	${MP_CC_DIR}\\xc16-bin2hex dist/${CND_CONF}/${IMAGE_TYPE}/6900834_SST_V01.00.20.35.X.${IMAGE_TYPE}.${DEBUGGABLE_SUFFIX} -a  -omf=elf   
	
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

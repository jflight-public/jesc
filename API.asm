// Copyright 2019 Thorsten Laux

// This file defines constants to allow services to extend 8051 assembler based
// applications. It's dual licensed under GPL3 and MIT licenses.
    
// Service entry points for service detection, initialization, interrupts etc

SERVICE_CURRENT_VERSION EQU 03h
SERVICE_MAGIC         EQU 03e00h
SERVICE_VERSION       EQU 03e04h
SERVICE_START         EQU 0f900h
SERVICE_INIT          EQU (SERVICE_START + 00h)
SERVICE_NOTIFY_FRAME  EQU (SERVICE_START + 30h)
SERVICE_BEGIN_WAIT    EQU (SERVICE_START + 50h)
SERVICE_END_WAIT      EQU (SERVICE_START + 70h)
SERVICE_T4_INT        EQU (SERVICE_START + 90h)
SERVICE_T0_INT        EQU (SERVICE_START + 0b0h)
    
DSEG at 70h
Period_L:             DS  1       ; Low byte of commutation period
Period_H:             DS  1       ; High byte of commutation period
Rtx_Mask:             DS  1       ; RTX Pin Mask
DShot_Frame_Thresh:   DS  1       ; Dshot frame length threshold
MemPtr:               DS  1



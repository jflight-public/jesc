
// This file defines constants to allow services to extend 8051 assembler based
// applications. It's dual licensed under GPL3 and MIT licenses.
    
// Service entry points for service detection, initialization, interrupts etc

SERVICE_START         EQU 1300h
SERVICE_INIT          EQU (SERVICE_START + 80h)
SERVICE_BEGIN_WAIT    EQU (SERVICE_START + 100h)
SERVICE_END_WAIT      EQU (SERVICE_START + 180h)
SERVICE_NOTIFY_FRAME  EQU (SERVICE_START + 200h)
SERVICE_T0_INT        EQU (SERVICE_START + 280h)
SERVICE_T4_INT        EQU (SERVICE_START + 300h)
    
DSEG at 70h
Period_L:             DS  1       ; Low byte of commutation period
Period_H:             DS  1       ; High byte of commutation period
Rtx_Mask:             DS  1       ; RTX Pin Mask


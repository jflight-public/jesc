
// This file defines constants to allow services to extend 8051 assembler based
// applications. It's dual licensed under GPL3 and MIT licenses.
    
// Service entry points for service detection, initialization, interrupts etc

SERVICE_START         EQU 0f800h
SERVICE_INIT          EQU (SERVICE_START + 18h)
SERVICE_NOTIFY_FRAME  EQU (SERVICE_START + 40h)
SERVICE_BEGIN_WAIT    EQU (SERVICE_START + 60h)
SERVICE_END_WAIT      EQU (SERVICE_START + 80h)
SERVICE_T4_INT        EQU (SERVICE_START + 0a0h)
SERVICE_T0_INT        EQU (SERVICE_START + 0c0h)
    
DSEG at 70h
Period_L:             DS  1       ; Low byte of commutation period
Period_H:             DS  1       ; High byte of commutation period
Rtx_Mask:             DS  1       ; RTX Pin Mask


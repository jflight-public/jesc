$NOMOD51


;**** **** **** **** ****
;
; JEsc program for controlling brushless motors in multirotors
;
; Copyright 2019 Thorsten Laux
; Copyright 2011, 2012 Steffen Skaug
; This program is distributed under the terms of the GNU General Public License
;
; This file is part of JEsc.
;
; JEsc is free software: you can redistribute it and/or modify
; it under the terms of the GNU General Public License as published by
; the Free Software Foundation, either version 3 of the License, or
; (at your option) any later version.
;
; JEsc is distributed in the hope that it will be useful,
; but WITHOUT ANY WARRANTY; without even the implied warranty of
; MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
; GNU General Public License for more details.
;
; You should have received a copy of the GNU General Public License
; along with JEsc.  If not, see <http://www.gnu.org/licenses/>.
;
;**** **** **** **** ****
;
; This software was initially designed for use with Eflite mCP X, but is now adapted to copters/planes in general
;
; The software was inspired by and started from from Bernard Konze's BLMC: http://home.versanet.de/~bkonze/blc_6a/blc_6a.htm
; And also Simon Kirby's TGY: https://github.com/sim-/tgy
;
; This file is best viewed with tab width set to 5
;
; The code is designed for multirotor applications, running damped light mode
;
; The input signal can be Dshot 300 or Dshot 600 or Dshot 300 inverted.
;
; JEsc has eliminated the legacy protocols and added support for services enhancing ESC
; functionality. 
;
;**** **** **** **** ****
; Revision history:
; - Rev1.0 Started. Built upon rev 16.7 of base code
;
;**** **** **** **** ****
; Minimum 8K Bytes of In-System Self-Programmable Flash
; Minimum 512 Bytes Internal SRAM
;
;**** **** **** **** ****
; Master clock is internal 48MHz oscillator 
;
; JEsc uses Timer 0, Timer 1, Timer 2 and Timer 3. Timers 1-3 use interrupts, Timer 0 is only needed during dshot frame reception and
; does not need an interrupt. Therefore extension services can use Timer 0 when not receiving dshot and Timer 4 at all times.
;    
; - RC pulse measurement
; Timer 1 (41.67ns counts) always counts up and is used for
; - DShot frame sync detection
; Timer 2 (500ns counts) always counts up and is used for
; - RC pulse timeout counts and commutation times
; Timer 3 (500ns counts) always counts up and is used for
; - Commutation timeouts
; PCA0 (41.67ns counts) always counts up and is used for
; - Hardware PWM generation
;
;**** **** **** **** ****
; Interrupt handling
; The C8051 does not disable interrupts when entering an interrupt routine.
; Also some interrupt flags need to be cleared by software
; The code disables interrupts in some interrupt routines
; - Interrupts are disabled during beeps, to avoid audible interference from interrupts
;
;**** **** **** **** ****
; Motor control:
; - Brushless motor control with 6 states for each electrical 360 degrees
; - An advance timing of 0deg has zero cross 30deg after one commutation and 30deg before the next
; - Timing advance in this implementation is set to 15deg nominally
; - Motor pwm is always damped light (aka complementary pwm, regenerative braking)
; Motor sequence starting from zero crossing:
; - Timer wait: Wt_Comm         15deg   ; Time to wait from zero cross to actual commutation
; - Timer wait: Wt_Advance      15deg   ; Time to wait for timing advance. Nominal commutation point is after this
; - Timer wait: Wt_Zc_Scan      7.5deg  ; Time to wait before looking for zero cross
; - Scan for zero cross         22.5deg ; Nominal, with some motor variations
;
; Motor startup:
; There is a startup phase and an initial run phase, before normal bemf commutation run begins.
;
;**** **** **** **** ****
; List of enumerated supported ESCs
A_          EQU 1   ; X  X  RC X  MC MB MA CC    X  X  Cc Cp Bc Bp Ac Ap
B_          EQU 2   ; X  X  RC X  MC MB MA CC    X  X  Ap Ac Bp Bc Cp Cc
C_          EQU 3   ; Ac Ap MC MB MA CC X  RC    X  X  X  X  Cc Cp Bc Bp
D_          EQU 4   ; X  X  RC X  CC MA MC MB    X  X  Cc Cp Bc Bp Ac Ap    Com fets inverted
E_          EQU 5   ; L1 L0 RC X  MC MB MA CC    X  L2 Cc Cp Bc Bp Ac Ap    A with LEDs
F_          EQU 6   ; X  X  RC X  MA MB MC CC    X  X  Cc Cp Bc Bp Ac Ap
G_          EQU 7   ; X  X  RC X  CC MA MC MB    X  X  Cc Cp Bc Bp Ac Ap    Like D, but noninverted com fets
H_          EQU 8   ; RC X  X  X  MA MB CC MC    X  Ap Bp Cp X  Ac Bc Cc
I_          EQU 9   ; X  X  RC X  MC MB MA CC    X  X  Ac Bc Cc Ap Bp Cp
J_          EQU 10  ; L2 L1 L0 RC CC MB MC MA    X  X  Cc Bc Ac Cp Bp Ap    LEDs
K_          EQU 11  ; X  X  MC X  MB CC MA RC    X  X  Ap Bp Cp Cc Bc Ac    Com fets inverted
L_          EQU 12  ; X  X  RC X  CC MA MB MC    X  X  Ac Bc Cc Ap Bp Cp
M_          EQU 13  ; MA MC CC MB RC L0 X  X     X  Cc Bc Ac Cp Bp Ap X    LED
N_          EQU 14  ; X  X  RC X  MC MB MA CC    X  X  Cp Cc Bp Bc Ap Ac
O_          EQU 15  ; X  X  RC X  CC MA MC MB    X  X  Cc Cp Bc Bp Ac Ap    Like D, but low side pwm
P_          EQU 16  ; X  X  RC MA CC MB MC X     X  Cc Bc Ac Cp Bp Ap X
Q_          EQU 17  ; Cp Bp Ap L1 L0 X  RC X     X  MA MB MC CC Cc Bc Ac   LEDs
R_          EQU 18  ; X  X  RC X  MC MB MA CC    X  X  Ac Bc Cc Ap Bp Cp
S_              EQU 19      ; X  X  RC X  CC MA MC MB    X  X  Cc Cp Bc Bp Ac Ap   Like O, but com fets inverted
T_          EQU 20  ; RC X  MA X  MB CC MC X     X  X  Cp Bp Ap Ac Bc Cc
U_          EQU 21  ; MA MC CC MB RC L0 L1 L2    X  Cc Bc Ac Cp Bp Ap X Like M, but with 3 LEDs
V_          EQU 22  ; Cc X  RC X  MC CC MB MA    X  Ap Ac Bp X  X  Bc Cp
W_                      EQU 23  ; RC MC MB X  CC MA X X      X  Ap Bp Cp X  X  X  X     Tristate gate driver


;                     VBAT CUR L0 RC CC MB MC MA    Tlm Cc Bc Ac Cp Bp Ap

    
;**** **** **** **** ****
; Select the port mapping to use (or unselect all for use with external batch compile file)
;ESCNO EQU A_
;ESCNO EQU B_
;ESCNO EQU C_
;ESCNO EQU D_
;ESCNO EQU E_
;ESCNO EQU F_
;ESCNO EQU G_
;ESCNO EQU H_
;ESCNO EQU I_
;ESCNO EQU J_
;ESCNO EQU K_
;ESCNO EQU L_
;ESCNO EQU M_
;ESCNO EQU N_
;ESCNO EQU O_
;ESCNO EQU P_
;ESCNO EQU Q_
;ESCNO EQU R_
;ESCNO EQU S_
;ESCNO EQU T_
;ESCNO EQU U_
;ESCNO EQU V_
;ESCNO EQU W_

DEBUG_COMP EQU 0
DEBUG_COMP_DETAILED  EQU 0

;**** **** **** **** ****
; Select the MCU type (or unselect for use with external batch compile file)
;MCU_48MHZ EQU  0

;**** **** **** **** ****
; Select the fet deadtime (or unselect for use with external batch compile file)
;FETON_DELAY EQU 15 ; 20.4ns per step


$include (API.asm)
    
;**** **** **** **** ****
; ESC selection statements
IF ESCNO == A_
$include (A.inc)    ; Select pinout A
LAYOUT_TAG EQU "A"
ENDIF

IF ESCNO == B_
$include (B.inc)    ; Select pinout B
LAYOUT_TAG EQU "B"
ENDIF

IF ESCNO == C_
$include (C.inc)    ; Select pinout C
LAYOUT_TAG EQU "C"
ENDIF

IF ESCNO == D_
$include (D.inc)    ; Select pinout D
LAYOUT_TAG EQU "D"
ENDIF

IF ESCNO == E_
$include (E.inc)    ; Select pinout E
LAYOUT_TAG EQU "E"
ENDIF

IF ESCNO == F_
$include (F.inc)    ; Select pinout F
LAYOUT_TAG EQU "F"
ENDIF

IF ESCNO == G_
$include (G.inc)    ; Select pinout G
LAYOUT_TAG EQU "G"
ENDIF

IF ESCNO == H_
$include (H.inc)    ; Select pinout H
LAYOUT_TAG EQU "H"
ENDIF

IF ESCNO == I_
$include (I.inc)    ; Select pinout I
LAYOUT_TAG EQU "I"
ENDIF

IF ESCNO == J_
$include (J.inc)    ; Select pinout J
LAYOUT_TAG EQU "J"
ENDIF

IF ESCNO == K_
$include (K.inc)    ; Select pinout K
LAYOUT_TAG EQU "K"
ENDIF

IF ESCNO == L_
$include (L.inc)    ; Select pinout L
LAYOUT_TAG EQU "L"
ENDIF

IF ESCNO == M_
$include (M.inc)    ; Select pinout M
LAYOUT_TAG EQU "M"
ENDIF

IF ESCNO == N_
$include (N.inc)    ; Select pinout N
LAYOUT_TAG EQU "N"
ENDIF

IF ESCNO == O_
$include (O.inc)    ; Select pinout O
LAYOUT_TAG EQU "O"
ENDIF

IF ESCNO == P_
$include (P.inc)    ; Select pinout P
LAYOUT_TAG EQU "P"
ENDIF

IF ESCNO == Q_
$include (Q.inc)    ; Select pinout Q
LAYOUT_TAG EQU "Q"
ENDIF

IF ESCNO == R_
$include (R.inc)    ; Select pinout R
LAYOUT_TAG EQU "R"
ENDIF

IF ESCNO == S_
$include (S.inc)        ; Select pinout S
LAYOUT_TAG EQU "S"
ENDIF

IF ESCNO == T_
$include (T.inc)        ; Select pinout T
LAYOUT_TAG EQU "T"
ENDIF

IF ESCNO == U_
$include (U.inc)        ; Select pinout U
LAYOUT_TAG EQU "U"
ENDIF

IF ESCNO == V_
$include (V.inc)        ; Select pinout V
LAYOUT_TAG EQU "V"
ENDIF

IF ESCNO == W_
$include (W.inc)        ; Select pinout W
LAYOUT_TAG EQU "W"
ENDIF

IF FETON_DELAY != 0 && (ESCNO == B_ OR ESCNO == H_ OR ESCNO == K_ OR ESCNO == N_ OR ESCNO == S_ OR ESCNO == T_ OR ESCNO == V_ OR ESCNO == W_)
PCA_POWER_L EQU PCA0CPL1
PCA_POWER_H EQU PCA0CPH1
PCA_DAMP_L EQU PCA0CPL0
PCA_DAMP_H EQU PCA0CPH0
ELSE
PCA_POWER_L EQU PCA0CPL0
PCA_POWER_H EQU PCA0CPH0
PCA_DAMP_L EQU PCA0CPL1
PCA_DAMP_H EQU PCA0CPH1
ENDIF    

;**** **** **** **** ****
; Uses internal calibrated oscillator set to 24/48Mhz
;**** **** **** **** ****

;**** **** **** **** ****
; Constant definitions
;**** **** **** **** ****


IF MCU_48MHZ == 0
MCU_TAG EQU "L"
ELSE
MCU_TAG EQU "H"
ENDIF


IF FETON_DELAY == 120
DELAY_TAG2 EQU "0#"
ELSE
DELAY_TAG2 EQU "# "
ENDIF
    
IF FETON_DELAY == 0
DELAY_TAG EQU "00"
ELSEIF FETON_DELAY == 5
DELAY_TAG EQU "05"
ELSEIF FETON_DELAY == 10
DELAY_TAG EQU "10"
ELSEIF FETON_DELAY == 15
DELAY_TAG EQU "15"
ELSEIF FETON_DELAY == 20
DELAY_TAG EQU "20"
ELSEIF FETON_DELAY == 25
DELAY_TAG EQU "25"
ELSEIF FETON_DELAY == 30
DELAY_TAG EQU "30"
ELSEIF FETON_DELAY == 40
DELAY_TAG EQU "40"
ELSEIF FETON_DELAY == 50
DELAY_TAG EQU "50"
ELSEIF FETON_DELAY == 70
DELAY_TAG EQU "70"
ELSEIF FETON_DELAY == 90
DELAY_TAG EQU "90"
ELSEIF FETON_DELAY == 120
DELAY_TAG EQU "12"
ENDIF

IF PWM == 24
PWM_TAG EQU "24"    
ELSEIF PWM == 48
PWM_TAG EQU "48"    
ELSEIF PWM == 96
PWM_TAG EQU "96"    
ENDIF

CSEG AT 1A40h
Eep_ESC_Layout:
    DB "#"
    DB LAYOUT_TAG
    DB "_"
    DB MCU_TAG
    DB "_"
    DW DELAY_TAG
    DW DELAY_TAG2
    DB "        "	; ESC layout tag


IF MCU_48MHZ == 1
CSEG AT 1A50h
Eep_ESC_MCU:			DB	"#BLHELI$EFM8B21#"	; Project and MCU tag (16 Bytes)
ELSE
CSEG AT 1A50h
Eep_ESC_MCU:			DB	"#BLHELI$EFM8B10#"	; Project and MCU tag (16 Bytes)
ENDIF
    

;**** **** **** **** ****
; Programming defaults
;
DEFAULT_PGM_STARTUP_PWR                 EQU 9   ; 1=0.031 2=0.047 3=0.063 4=0.094 5=0.125 6=0.188   7=0.25  8=0.38  9=0.50  10=0.75 11=1.00 12=1.25 13=1.50
DEFAULT_PGM_COMM_TIMING             EQU 3   ; 1=Low         2=MediumLow     3=Medium        4=MediumHigh    5=High
DEFAULT_PGM_DEMAG_COMP              EQU 2   ; 1=Disabled    2=Low       3=High
DEFAULT_PGM_DIRECTION               EQU 1   ; 1=Normal  2=Reversed  3=Bidir     4=Bidir rev
DEFAULT_PGM_BEEP_STRENGTH           EQU 40  ; Beep strength
DEFAULT_PGM_BEACON_STRENGTH         EQU 80  ; Beacon strength
DEFAULT_PGM_BEACON_DELAY                EQU 4   ; 1=1m      2=2m            3=5m            4=10m       5=Infinite

; COMMON
DEFAULT_PGM_ENABLE_TX_PROGRAM       EQU 1   ; 1=Enabled     0=Disabled
DEFAULT_PGM_MIN_THROTTLE                EQU 37  ; 4*37+1000=1148
DEFAULT_PGM_MAX_THROTTLE                EQU 208 ; 4*208+1000=1832
DEFAULT_PGM_CENTER_THROTTLE         EQU 122 ; 4*122+1000=1488 (used in bidirectional mode)
DEFAULT_PGM_ENABLE_TEMP_PROT            EQU 7   ; 0=Disabled    1=80C   2=90C   3=100C  4=110C  5=120C  6=130C  7=140C
DEFAULT_PGM_ENABLE_POWER_PROT       EQU 1   ; 1=Enabled     0=Disabled
DEFAULT_PGM_BRAKE_ON_STOP           EQU 0   ; 1=Enabled     0=Disabled
DEFAULT_PGM_LED_CONTROL             EQU 0   ; Byte for LED control. 2bits per LED, 0=Off, 1=On

;**** **** **** **** ****
; Temporary register definitions

Temp1       EQU R0
Temp2       EQU R1
Temp3       EQU R2
Temp4       EQU R3
Temp5       EQU R4
Temp6       EQU R5
Temp7       EQU R6
Temp8       EQU R7

;**** **** **** **** ****
; Register definitions
DSEG AT 20h                 ; Variables segment 

Bit_Access:             DS  1       ; MUST BE AT THIS ADDRESS. Variable at bit accessible address (for non interrupt routines)
Bit_Access_Int:         DS  1       ; Variable at bit accessible address (for interrupts)

Rcp_Outside_Range_Cnt:  DS  1       ; RC pulse outside range counter (incrementing) 
Rcp_Timeout_Cntd:       DS  1       ; RC pulse timeout counter (decrementing) 


Flags0:                 DS  1       ; State flags. Reset upon init_start
DEMAG_DETECTED          EQU     0       ; Set when excessive demag time is detected
COMP_TIMED_OUT          EQU     1       ; Set when comparator reading timed out

Flags1:                 DS  1       ; State flags. Reset upon init_start 
STARTUP_PHASE           EQU     0       ; Set when in startup phase
INITIAL_RUN_PHASE       EQU     1       ; Set when in initial run phase, before synchronized run is achieved
MOTOR_STARTED           EQU     2       ; Set when motor is started
DIR_CHANGE_BRAKE        EQU     3       ; Set when braking before direction change
HIGH_RPM                EQU     4       ; Set when motor rpm is high (Comm_Period4x_H less than 2)
WAIT_ACTIVE             BIT Flags1.5    ; Set if set timeout hasn't triggered yet
TIMER1_XFER             BIT Flags1.6    
;RC_DURING_COMMUTATION   BIT Flags1.7
    
Flags2:                 DS  1       ; State flags. NOT reset upon init_start
RCP_UPDATED             EQU     0       ; New RC pulse length value available
RCP_DIR_REV             EQU     1       ; RC pulse direction in bidirectional mode
RCP_FULL_RANGE          EQU     2       ; When set full input signal range is used (1000-2000us) and stored calibration values are ignored
RCP_INVERTED            EQU     3       ; Is inverted dshot being used?

Flags3:                 DS  1       ; State flags. NOT reset upon init_start
PGM_DIR_REV             EQU     0       ; Programmed direction. 0=normal, 1=reversed
PGM_BIDIR_REV           EQU     1       ; Programmed bidirectional direction. 0=normal, 1=reversed
PGM_BIDIR               EQU     2       ; Programmed bidirectional operation. 0=normal, 1=bidirectional
SERVICE_DETECTED        BIT Flags3.3    ; Set when plugin is detected

;**** **** **** **** ****
; RAM definitions
DSEG AT 30h                     ; Ram data segment, direct addressing
Initial_Arm:                DS  1       ; Variable that is set during the first arm sequence after power on

Power_On_Wait_Cnt_L:        DS  1       ; Power on wait counter (lo byte)
Power_On_Wait_Cnt_H:        DS  1       ; Power on wait counter (hi byte)

Startup_Cnt:                DS  1       ; Startup phase commutations counter (incrementing)
Startup_Zc_Timeout_Cntd:    DS  1       ; Startup zero cross timeout counter (decrementing)
Initial_Run_Rot_Cntd:       DS  1       ; Initial run rotations counter (decrementing)
Stall_Cnt:                  DS  1       ; Counts start/run attempts that resulted in stall. Reset upon a proper stop
Demag_Detected_Metric:      DS  1       ; Metric used to gauge demag event frequency
Demag_Pwr_Off_Thresh:       DS  1       ; Metric threshold above which power is cut
Low_Rpm_Pwr_Slope:          DS  1       ; Sets the slope of power increase for low rpms

Timer2_X:                   DS  1       ; Timer 2 extended byte
Prev_Comm_L:                DS  1       ; Previous commutation timer 3 timestamp (lo byte)
Prev_Comm_H:                DS  1       ; Previous commutation timer 3 timestamp (hi byte)
Prev_Comm_X:                DS  1       ; Previous commutation timer 3 timestamp (ext byte)
Prev_Prev_Comm_L:           DS  1       ; Pre-previous commutation timer 3 timestamp (lo byte)
Prev_Prev_Comm_H:           DS  1       ; Pre-previous commutation timer 3 timestamp (hi byte)
Comm_Period1x_L:            DS  1       ; Timer 3 counts between the last 4 commutations (lo byte)
Comm_Period1x_H:            DS  1       ; Timer 3 counts between the last 4 commutations (hi byte)
Zc_Expected_L:              DS  1
Zc_Expected_H:              DS  1
Comparator_Read_Cnt:        DS  1       ; Number of comparator reads done

Wt_Adv_Start_L:             DS  1       ; Timer 3 start point for commutation advance timing (lo byte)
Wt_Adv_Start_H:             DS  1       ; Timer 3 start point for commutation advance timing (hi byte)
Wt_Zc_Scan_Start_L:         DS  1       ; Timer 3 start point from commutation to zero cross scan (lo byte)
Wt_Zc_Scan_Start_H:         DS  1       ; Timer 3 start point from commutation to zero cross scan (hi byte)
Wt_Zc_Tout_Start_L:         DS  1       ; Timer 3 start point for zero cross scan timeout (lo byte)
Wt_Zc_Tout_Start_H:         DS  1       ; Timer 3 start point for zero cross scan timeout (hi byte)
Wt_Comm_Start_L:            DS  1       ; Timer 3 start point from zero cross to commutation (lo byte)
Wt_Comm_Start_H:            DS  1       ; Timer 3 start point from zero cross to commutation (hi byte)

Dshot_Cmd:                  DS  1       ; Dshot command
Dshot_Cmd_Cnt:              DS  1       ; Dshot command count

New_Rcp:                    DS  1       ; New RC pulse value in pca counts
Rcp_Stop_Cnt:               DS  1       ; Counter for RC pulses below stop value

Power_Pwm_Reg_L:            DS  1       ; Power pwm register setting (lo byte)
Power_Pwm_Reg_H:            DS  1       ; Power pwm register setting (hi byte). 0x3F is minimum power
Damp_Pwm_Reg_L:             DS  1       ; Damping pwm register setting (lo byte)
Damp_Pwm_Reg_H:             DS  1       ; Damping pwm register setting (hi byte)
Current_Power_Pwm_Reg_H:    DS  1       ; Current power pwm register setting that is loaded in the PCA register (hi byte)

Pwm_Limit:                  DS  1       ; Maximum allowed pwm 
Pwm_Limit_By_Rpm:           DS  1       ; Maximum allowed pwm for low or high rpms
Pwm_Limit_Beg:              DS  1       ; Initial pwm limit

Adc_Conversion_Cnt:         DS  1       ; Adc conversion counter

Current_Average_Temp:       DS  1       ; Current average temperature (lo byte ADC reading, assuming hi byte is 1)

Beep_Strength:              DS  1       ; Strength of beeps

Skip_T2_Int:                DS  1       ; Set for 48MHz MCUs when timer 2 interrupt shall be ignored

Flash_Key_1:                DS  1       ; Flash key one
Flash_Key_2:                DS  1       ; Flash key two

Temp_Prot_Limit:            DS  1       ; Temperature protection limit

DShot_Pwm_Thr:              DS  1       ; DShot pulse width threshold value
DShot_Timer_Preset:         DS  1       ; DShot timer preset for frame sync detection
DShot_Frame_Start_L:        DS  1       ; DShot frame start timestamp (lo byte)
DShot_Frame_Start_H:        DS  1       ; DShot frame start timestamp (hi byte)
Dither_Cur_Bit:             DS  1
DShot_Frame_Length_Thr EQU DShot_Frame_Thresh  ; DShot frame length criteria (in units of 4 timer 2 ticks)
DShot_Byte:                 DS  1
    
    
; Indirect addressing data segment. The variables below must be in this sequence
ISEG AT 080h                    
_Pgm_Gov_P_Gain:            DS  1       ; Programmed governor P gain
_Pgm_Gov_I_Gain:            DS  1       ; Programmed governor I gain
_Pgm_Gov_Mode:              DS  1       ; Programmed governor mode
_Pgm_Low_Voltage_Lim:       DS  1       ; Programmed low voltage limit
_Pgm_Motor_Gain:            DS  1       ; Programmed motor gain
_Pgm_Motor_Idle:            DS  1       ; Programmed motor idle speed
Pgm_Startup_Pwr:            DS  1       ; Programmed startup power
_Pgm_Pwm_Freq:              DS  1       ; Programmed pwm frequency
Pgm_Direction:              DS  1       ; Programmed rotation direction
Pgm_Input_Pol:              DS  1       ; Programmed input pwm polarity
Initialized_L_Dummy:        DS  1       ; Place holder
Initialized_H_Dummy:        DS  1       ; Place holder
Pgm_Enable_TX_Program:      DS  1       ; Programmed enable/disable value for TX programming
_Pgm_Main_Rearm_Start:      DS  1       ; Programmed enable/disable re-arming main every start 
_Pgm_Gov_Setup_Target:      DS  1       ; Programmed main governor setup target
_Pgm_Startup_Rpm:           DS  1       ; Programmed startup rpm (unused - place holder)
_Pgm_Startup_Accel:         DS  1       ; Programmed startup acceleration (unused - place holder)
_Pgm_Volt_Comp:             DS  1       ; Place holder
Pgm_Comm_Timing:            DS  1       ; Programmed commutation timing
_Pgm_Damping_Force:         DS  1       ; Programmed damping force (unused - place holder)
_Pgm_Gov_Range:             DS  1       ; Programmed governor range
_Pgm_Startup_Method:        DS  1       ; Programmed startup method (unused - place holder)
Pgm_Min_Throttle:           DS  1       ; Programmed throttle minimum
Pgm_Max_Throttle:           DS  1       ; Programmed throttle maximum
Pgm_Beep_Strength:          DS  1       ; Programmed beep strength
Pgm_Beacon_Strength:        DS  1       ; Programmed beacon strength
Pgm_Beacon_Delay:           DS  1       ; Programmed beacon delay
_Pgm_Throttle_Rate:         DS  1       ; Programmed throttle rate (unused - place holder)
Pgm_Demag_Comp:             DS  1       ; Programmed demag compensation
_Pgm_BEC_Voltage_High:      DS  1       ; Programmed BEC voltage
Pgm_Center_Throttle:        DS  1       ; Programmed throttle center (in bidirectional mode)
_Pgm_Main_Spoolup_Time:     DS  1       ; Programmed main spoolup time
Pgm_Enable_Temp_Prot:       DS  1       ; Programmed temperature protection enable
Pgm_Enable_Power_Prot:      DS  1       ; Programmed low rpm power protection enable
_Pgm_Enable_Pwm_Input:      DS  1       ; Programmed PWM input signal enable
_Pgm_Pwm_Dither:            DS  1       ; Programmed output PWM dither
Pgm_Brake_On_Stop:          DS  1       ; Programmed braking when throttle is zero
Pgm_LED_Control:            DS  1       ; Programmed LED control

; The sequence of the variables below is no longer of importance
Pgm_Startup_Pwr_Decoded:        DS  1       ; Programmed startup power decoded


; Indirect addressing data segment
ISEG AT 0D0h                    
Temp_Storage:               DS  48      ; Temporary storage
     
                                ;**** **** **** **** ****
CSEG AT 1A00h            ; "Eeprom" segment
EEPROM_FW_MAIN_REVISION     EQU MAJOR      ; Main revision of the firmware
EEPROM_FW_SUB_REVISION      EQU MINOR      ; Sub revision of the firmware
EEPROM_LAYOUT_REVISION      EQU 33      ; Revision of the EEPROM layout

Eep_FW_Main_Revision:       DB  EEPROM_FW_MAIN_REVISION         ; EEPROM firmware main revision number
Eep_FW_Sub_Revision:        DB  EEPROM_FW_SUB_REVISION          ; EEPROM firmware sub revision number
Eep_Layout_Revision:        DB  EEPROM_LAYOUT_REVISION          ; EEPROM layout revision number

_Eep_Pgm_Gov_P_Gain:        DB  0FFh    
_Eep_Pgm_Gov_I_Gain:        DB  0FFh    
_Eep_Pgm_Gov_Mode:          DB  0FFh    
_Eep_Pgm_Low_Voltage_Lim:   DB  0FFh                            
_Eep_Pgm_Motor_Gain:        DB  0FFh    
_Eep_Pgm_Motor_Idle:        DB  0FFh                        
Eep_Pgm_Startup_Pwr:        DB  DEFAULT_PGM_STARTUP_PWR         ; EEPROM copy of programmed startup power
_Eep_Pgm_Pwm_Freq:          DB  0FFh    
Eep_Pgm_Direction:          DB  DEFAULT_PGM_DIRECTION           ; EEPROM copy of programmed rotation direction
_Eep_Pgm_Input_Pol:         DB  0FFh
Eep_Initialized_L:          DB  055h                            ; EEPROM initialized signature low byte
Eep_Initialized_H:          DB  0AAh                            ; EEPROM initialized signature high byte
Eep_Enable_TX_Program:      DB  DEFAULT_PGM_ENABLE_TX_PROGRAM       ; EEPROM TX programming enable
_Eep_Main_Rearm_Start:      DB  0FFh                            
_Eep_Pgm_Gov_Setup_Target:  DB  0FFh                            
_Eep_Pgm_Startup_Rpm:       DB  0FFh
_Eep_Pgm_Startup_Accel:     DB  0FFh
_Eep_Pgm_Volt_Comp:         DB  0FFh    
Eep_Pgm_Comm_Timing:        DB  DEFAULT_PGM_COMM_TIMING         ; EEPROM copy of programmed commutation timing
_Eep_Pgm_Damping_Force:     DB  0FFh
_Eep_Pgm_Gov_Range:         DB  0FFh    
_Eep_Pgm_Startup_Method:        DB  0FFh
Eep_Pgm_Min_Throttle:       DB  DEFAULT_PGM_MIN_THROTTLE            ; EEPROM copy of programmed minimum throttle
Eep_Pgm_Max_Throttle:       DB  DEFAULT_PGM_MAX_THROTTLE            ; EEPROM copy of programmed minimum throttle
Eep_Pgm_Beep_Strength:      DB  DEFAULT_PGM_BEEP_STRENGTH       ; EEPROM copy of programmed beep strength
Eep_Pgm_Beacon_Strength:        DB  DEFAULT_PGM_BEACON_STRENGTH     ; EEPROM copy of programmed beacon strength
Eep_Pgm_Beacon_Delay:       DB  DEFAULT_PGM_BEACON_DELAY            ; EEPROM copy of programmed beacon delay
_Eep_Pgm_Throttle_Rate:     DB  0FFh
Eep_Pgm_Demag_Comp:         DB  DEFAULT_PGM_DEMAG_COMP          ; EEPROM copy of programmed demag compensation
_Eep_Pgm_BEC_Voltage_High:  DB  0FFh    
Eep_Pgm_Center_Throttle:        DB  DEFAULT_PGM_CENTER_THROTTLE     ; EEPROM copy of programmed center throttle
_Eep_Pgm_Main_Spoolup_Time: DB  0FFh
Eep_Pgm_Temp_Prot_Enable:   DB  DEFAULT_PGM_ENABLE_TEMP_PROT        ; EEPROM copy of programmed temperature protection enable
Eep_Pgm_Enable_Power_Prot:  DB  DEFAULT_PGM_ENABLE_POWER_PROT       ; EEPROM copy of programmed low rpm power protection enable
_Eep_Pgm_Enable_Pwm_Input:  DB  0FFh    
_Eep_Pgm_Pwm_Dither:        DB  0FFh    
Eep_Pgm_Brake_On_Stop:      DB  DEFAULT_PGM_BRAKE_ON_STOP       ; EEPROM copy of programmed braking when throttle is zero
Eep_Pgm_LED_Control:        DB  DEFAULT_PGM_LED_CONTROL         ; EEPROM copy of programmed LED control

Eep_Dummy:              DB  0FFh                            ; EEPROM address for safety reason

Comm_Period4x_L    EQU Period_L
Comm_Period4x_H    EQU Period_H

CSEG AT 1A60h
Eep_Name:                   DB  "                "              ; Name tag (16 Bytes)

CSEG AT 0b0h
Jesc_Name:                   DB  "JESC01 #"
    DW PWM_TAG
    DB "      "            ; Name tag (16 Bytes)

End_Wait MACRO
local l1
    jnb SERVICE_DETECTED, l1
    jmp SERVICE_END_WAIT
l1: 
ENDM    

;**** **** **** **** ****
CSEG AT 0               ; Code segment start
IF MCU_48MHZ == 1
    jmp check_bootloader
ELSE
    jmp reset
ENDIF    
CSEG AT 03h         ; Int0 interrupt    
    mov DShot_Byte, TL0
    jmp int0_int    

CSEG AT 0Bh         ; Timer0 overflow interrupt
    jmp SERVICE_T0_INT

CSEG AT 13h         ; Int1 interrupt
    jmp int1_int

IF MCU_48MHZ == 0
t1_int_jmp:
    ajmp t1_int
CSEG AT 1Bh         ; Timer1 overflow interrupt
    jnb TIMER1_XFER, t1_int_jmp
    cpl RTX_PORT.RTX_PIN
    xch A, MemPtr
    xch A, R0
    mov TL1, @R0
    xch A, R0
    inc A
    xch A, MemPtr
    reti
ELSE
CSEG AT 1Bh         ; Timer1 overflow interrupt
    jmp t1_int
ENDIF    

CSEG AT 2Bh         ; Timer2 overflow interrupt
    jmp t2_int  
CSEG AT 3Bh         ; SMB irq
    jmp SERVICE_NOTIFY_FRAME 
CSEG AT 5Bh         ; Pca interrupt
    jmp pca_int
CSEG AT 73h         ; Timer3 overflow/compare interrupt

t3_int: ; Used for commutation timing
    clr IE_EA
    anl EIE1, #7Fh
    mov TMR3RLL, #0FAh      ; Set a short delay before next interrupt
    mov TMR3RLH, #0FFh
    setb IE_EA
    anl TMR3CN0, #07fh             ; clear irq flag
IF MCU_48MHZ == 1
    jnb WAIT_ACTIVE, t3_exit
    clr WAIT_ACTIVE 
    End_Wait
ELSE
    clr WAIT_ACTIVE 
ENDIF    
t3_exit:    
reti

IF MCU_48MHZ == 1
CSEG AT 9bh         ; Timer4 overflow/compare interrupt
    clr TMR4CN0_TF4H
    cpl RTX_PORT.RTX_PIN
    xch A, MemPtr
    xch A, R0
    mov TMR4H, @R0
    xch A, R0
    inc A
    xch A, MemPtr
reti

ENDIF

;**** **** **** **** ****

CSEG at 0c0h    
; Table definitions
STARTUP_POWER_TABLE:    DB  04h, 06h, 08h, 0Ch, 10h, 18h, 20h, 30h, 40h, 60h, 80h, 0A0h, 0C0h


IF PWM == 24
    D_BITS_PWM EQU 0
ELSEIF PWM == 48
    D_BITS_PWM EQU 1
ELSEIF PWM == 96
    D_BITS_PWM EQU 2
ENDIF
    
IF MCU_48MHZ == 1
    D_BITS_MCU EQU 0
    FDELAY_SHIFT EQU 0
ELSE    
    D_BITS_MCU EQU 1
    FDELAY_SHIFT EQU 1
ENDIF    

IF FETON_DELAY != 0
    D_BITS_FET EQU 1
ELSE
    D_BITS_FET EQU 0
ENDIF
    
    
D_BITS EQU (D_BITS_PWM + D_BITS_MCU + D_BITS_FET)
CYCLE_OVFL_BIT EQU (3 - D_BITS)


Set_MCU_Clk_24MHz MACRO
    mov CLKSEL, #13h        ; Set clock to 24MHz
    mov SFRPAGE, #10h
    mov PFE0CN, #00h        ; Set flash timing for 24MHz
    mov SFRPAGE, #00h
ENDM
Set_MCU_Clk_48MHz MACRO
    mov SFRPAGE, #10h
    mov PFE0CN, #30h        ; Set flash timing for 48MHz
    mov SFRPAGE, #00h
    mov CLKSEL, #03h        ; Set clock to 48MHz
ENDM

Initialize_PCA MACRO
	mov	PCA0CN0, #40h				; PCA enabled
	mov	PCA0MD, #08h				; PCA clock is system clock
    mov PCA0PWM, #(83h - D_BITS)
IF FETON_DELAY == 0
	mov	PCA0CENT, #00h				; Edge aligned pwm
ELSE
	mov	PCA0CENT, #03h				; Center aligned pwm
ENDIF
ENDM    

Decode_Dshot_2Msb MACRO
	movx	A, @DPTR
	mov	Temp6, A
	clr	C
	subb	A, Temp5					; Subtract previous timestamp
	clr	C
	subb	A, Temp1
	jc	t1_int_msb_fail			; Check that bit is longer than minimum

	subb	A, Temp1					; Check if bit is zero or one
	mov	A, Temp4					; Shift bit into data byte
	rlc	A
	mov	Temp4, A
	inc	DPL						; Next bit
	movx	A, @DPTR
	mov	Temp5, A
	clr	C
	subb	A, Temp6
	clr	C
	subb	A, Temp1
	jc	t1_int_msb_fail

	subb	A, Temp1
	mov	A, Temp4
	rlc	A
	mov	Temp4, A
	inc	DPL
ENDM
Decode_Dshot_2Lsb MACRO
	movx	A, @DPTR
	mov	Temp6, A
	clr	C
	subb	A, Temp5					; Subtract previous timestamp
	clr	C
	subb	A, Temp1
	jc	t1_int_lsb_fail			; Check that bit is longer than minimum

	subb	A, Temp1					; Check if bit is zero or one
	mov	A, Temp3					; Shift bit into data byte
	rlc	A
	mov	Temp3, A
	inc	DPL						; Next bit
	movx	A, @DPTR
	mov	Temp5, A
	clr	C
	subb	A, Temp6
	clr	C
	subb	A, Temp1
	jc	t1_int_lsb_fail

	subb	A, Temp1
	mov	A, Temp3
	rlc	A
	mov	Temp3, A
	inc	DPL
ENDM
    
IF MCU_48MHZ == 1
Wait_Pending MACRO
local l1, l2
    jnb SERVICE_DETECTED, l1
    anl EIE1, #07fh
    jnb WAIT_ACTIVE, l2
    call SERVICE_BEGIN_WAIT
IF MCU_48MHZ == 1
    mov SFRPAGE, #0
ENDIF    
l1:
    jb WAIT_ACTIVE, l1
l2:
ENDM    
ELSE
Wait_Pending MACRO
    jb WAIT_ACTIVE, $
ENDM
ENDIF    
    
IF MCU_48MHZ == 1
Notify_Frame MACRO  
    setb SMB0CN0_SI ; Indicate Frame available
ENDM
ELSE    
Notify_Frame MACRO  
ENDM
ENDIF
    
init_plugin:    
    clr IE_EA
    push DPL
    push DPH
    mov DPTR, #SERVICE_MAGIC
    clr A
    movc A, @A+DPTR
    cjne A, #'T', plugin_notfound
    inc DPTR
    clr A
    movc A, @A+DPTR
    cjne A, #'L', plugin_notfound
    inc DPTR
    clr A
    movc A, @A+DPTR
    cjne A, #'X', plugin_notfound
    inc DPTR
    inc DPTR
    inc DPTR
    clr A
    movc A, @A+DPTR
    cjne A, #SERVICE_CURRENT_VERSION, plugin_notfound
    lcall SERVICE_INIT
    setb SERVICE_DETECTED
    sjmp ($+4)
plugin_notfound:
    clr SERVICE_DETECTED
    pop DPH
    pop DPL
    setb IE_EA
ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Timer 1 interrupt routine
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
t1_int:
    clr     IE_EA
    clr IE_EX0          ; Disable int0 interrupts
    clr TMR2CN0_TR2     ; Timer 2 disabled
    clr TCON_TR1            ; Stop timer 1
    push    PSW
    setb    PSW.3           ; Select register bank 1 for this interrupt
    mov Temp1, TMR2L        ; Read timer value
    mov Temp2, TMR2H
    setb    TMR2CN0_TR2     ; Timer 2 enabled
    setb    IE_EA
    anl EIE1, #0EFh     ; Disable pca interrupts

    
    mov TL1, DShot_Timer_Preset ; Reset sync timer
    push    ACC
    push    B               ; Will be pop'ed by int0 exit

;    setb RC_DURING_COMMUTATION
    
    ; Reset timer 0
    mov TL0, #0
    ; Check frame time length
    clr C
    mov A, Temp1
    subb    A, DShot_Frame_Start_L
    mov Temp1, A
    mov A, Temp2
    subb    A, DShot_Frame_Start_H
    mov Temp2, A
    ; Divide by 2 (or 4 for 48MHz). Unit is then us
    clr C
    mov A, Temp2
    rrc A
    mov Temp2, A
    mov A, Temp1
    rrc A
    mov Temp1, A

IF MCU_48MHZ == 1
    clr C
    mov A, Temp2
    rrc A
    mov Temp2, A
    mov A, Temp1
    rrc A
    mov Temp1, A
ENDIF
    
    mov A, Temp2
    jnz t1_int_msb_fail ; Frame too long
    mov A, Temp1
    subb    A, DShot_Frame_Length_Thr
    jc  t1_int_msb_fail ; Frame too short
    subb    A, DShot_Frame_Length_Thr
    jnc t1_int_msb_fail ; Frame too long

    ; Check that correct number of pulses is received
    mov A, DPL          ; Read current pointer
    cjne    A, #16, t1_int_msb_fail

    ; Decode transmitted data
    mov Temp5, #0           ; Reset timestamp
    mov Temp4, #0           ; High byte of receive buffer
    mov Temp3, #0           ; Low byte of receive buffer
    mov Temp2, #8           ; Number of bits per byte
    mov DPTR, #0            ; Set pointer
    mov Temp1, DShot_Pwm_Thr; DShot pulse width criteria

IF MCU_48MHZ == 0
	clr	C
	mov	A, Temp1			; Scale pulse width criteria
	rrc	A
	mov	Temp1, A
ENDIF
    
t1_int_decode:
    ajmp    t1_int_decode_msb

t1_int_msb_fail:
    mov DPTR, #0            ; Set pointer to start
    setb    IE_EX0          ; Enable int0 interrupts
    setb    IE_EX1          ; Enable int1 interrupts
    ajmp int0_int_outside_range

t1_int_decode_msb:
    ; Decode DShot data Msb. Use more code space to save time (by not using loop)
    Decode_DShot_2Msb
    Decode_DShot_2Msb
    Decode_DShot_2Msb
    Decode_DShot_2Msb
    ajmp    t1_int_decode_lsb

t1_int_lsb_fail:
    mov DPTR, #0            ; Set pointer to start
    setb    IE_EX0          ; Enable int0 interrupts
    setb    IE_EX1          ; Enable int1 interrupts
    ajmp int0_int_outside_range

t1_int_decode_lsb:
    ; Decode DShot data Lsb
    Decode_DShot_2Lsb
    Decode_DShot_2Lsb
    Decode_DShot_2Lsb
    Decode_DShot_2Lsb
    ; XOR check (in inverted data, which is ok)
    mov A, Temp4
    swap    A
    xrl A, Temp4
    xrl A, Temp3
    anl A, #0F0h
    mov Temp2, A
    mov A, Temp3
    swap    A

    jnb Flags2.RCP_INVERTED, ($+4)
    cpl A
    
    anl A, #0F0h
    clr C
    subb    A, Temp2
    jz  t1_int_xor_ok       ; XOR check

    mov DPTR, #0            ; Set pointer to start
    setb    IE_EX0          ; Enable int0 interrupts
    setb    IE_EX1          ; Enable int1 interrupts
    ajmp int0_int_outside_range

t1_int_xor_ok:
;    call SERVICE_NOTIFY_FRAME 
    Notify_Frame
    
t1_int_no_notify:
     ; Swap to be LSB aligned to 12 bits (and invert)
    mov A, Temp4
    cpl A
    swap A
    anl A, #0F0h            ; Low nibble of high byte
    mov Temp2, A
    mov A, Temp3
    cpl A
    swap    A
    anl A, #0Fh         ; High nibble of low byte 
    orl A, Temp2
    mov Temp3, A
    mov A, Temp4            ; High nibble of high byte
    cpl A
    swap A
    anl A, #0Fh
    mov Temp4, A
    ; Subtract 96 (still 12 bits)
    clr C
    mov A, Temp3
    mov Temp2, A
    subb    A, #96
    mov Temp3, A
    mov A, Temp4
    subb    A, #0
    mov Temp4, A
    jnc     t1_normal_range

    clr C   
    mov A, Temp2        ; Check for 0 or dshot command
    mov Temp4, #0
    mov Temp3, #0
    mov Temp2, #0
    jz  t1_normal_range
    
    clr C               ; We are in the special dshot range
    rrc A               ; Divide by 2
    jnc     t1_dshot_set_cmd    ; Check for tlm bit set (if not telemetry, Temp2 will be zero and result in invalid command)

    mov     Temp2, A
    clr C
    subb A, Dshot_Cmd
    jz  t1_dshot_inc_cmd_cnt

t1_dshot_set_cmd:
    mov     A, Temp2
    mov Dshot_Cmd, A
    mov Dshot_Cmd_Cnt, #0
    mov Temp2, #0
    jmp     t1_normal_range
    
t1_dshot_inc_cmd_cnt:
    inc     Dshot_Cmd_Cnt
    
t1_normal_range:
    ; Check for bidirectional operation (0=stop, 96-2095->fwd, 2096-4095->rev)
    jnb Flags3.PGM_BIDIR, t1_int_not_bidir  ; If not bidirectional operation - branch

    ; Subtract 2000 (still 12 bits)
    clr C
    mov A, Temp3
    subb    A, #0D0h
    mov Temp1, A
    mov A, Temp4
    subb    A, #07h
    mov Temp2, A
    jc  t1_int_bidir_fwd                ; If result is negative - branch

    mov A, Temp1
    mov Temp3, A
    mov A, Temp2
    mov Temp4, A
    jb  Flags2.RCP_DIR_REV, t1_int_bidir_rev_chk    ; If same direction - branch

    setb    Flags2.RCP_DIR_REV
    ajmp    t1_int_bidir_rev_chk

t1_int_bidir_fwd:
    jnb Flags2.RCP_DIR_REV, t1_int_bidir_rev_chk    ; If same direction - branch

    clr Flags2.RCP_DIR_REV

t1_int_bidir_rev_chk:
    jb  Flags3.PGM_BIDIR_REV, ($+5)

    cpl Flags2.RCP_DIR_REV

    clr C                           ; Multiply throttle value by 2
    mov A, Temp3
    rlc A
    mov Temp3, A
    mov A, Temp4
    rlc A
    mov Temp4, A
t1_int_not_bidir:
    ; Generate 4/256
    mov A, Temp4
    add A, Temp4
    addc    A, Temp4
    addc    A, Temp4
    mov Temp2, A
    ; Align to 11 bits
    clr C
    mov A, Temp4
    rrc A
    mov Temp4, A
    mov A, Temp3
    rrc A
    mov Temp3, A
    ; Scale from 2000 to 2048
    mov A, Temp3
    add A, Temp2    ; Holds 4/128
    mov Temp3, A
    mov A, Temp4
    addc    A, #0
    mov Temp4, A
    jnb ACC.3, ($+7)

    mov Temp3, #0FFh
    mov Temp4, #0FFh

    ; Boost pwm during direct start
    mov A, Flags1
    anl A, #((1 SHL STARTUP_PHASE)+(1 SHL INITIAL_RUN_PHASE))
    jz  t1_int_startup_boosted

    jb  Flags1.MOTOR_STARTED, t1_int_startup_boosted    ; Do not boost when changing direction in bidirectional mode

    mov A, Pwm_Limit_Beg                ; Set 25% of max startup power as minimum power
    rlc A
    mov Temp2, A
    mov A, Temp4
    jnz t1_int_startup_boost_stall

    clr C
    mov A, Temp2
    subb    A, Temp3
    jc  t1_int_startup_boost_stall

    mov A, Temp2
    mov Temp3, A

t1_int_startup_boost_stall:
    mov A, Stall_Cnt                    ; Add an extra power boost during start
    swap    A
    rlc A
    add A, Temp3
    mov Temp3, A
    mov A, Temp4
    addc    A, #0
    mov Temp4, A

t1_int_startup_boosted:
    ; Set 8bit value
    clr C
    mov A, Temp3
    rlc A
    swap    A
    anl A, #0Fh
    mov Temp1, A
    mov A, Temp4
    rlc A
    swap    A
    anl A, #0F0h
    orl A, Temp1
    mov Temp1, A
    jnz t1_int_zero_rcp_checked ; New_Rcp (Temp1) is only zero if all 11 bits are zero

    mov A, Temp3
    jz  t1_int_zero_rcp_checked

    mov Temp1, #1

t1_int_zero_rcp_checked:
    mov DPTR, #0                    ; Set pointer to start
    ; Decrement outside range counter
    mov A, Rcp_Outside_Range_Cnt
    jz  ($+4)

    dec Rcp_Outside_Range_Cnt

    ajmp    int0_int_pulse_ready

t1_int_frame_fail:
    mov DPTR, #0                    ; Set pointer to start
    setb    IE_EX0                  ; Enable int0 interrupts
    setb    IE_EX1                  ; Enable int1 interrupts
    ajmp int0_int_outside_range


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Timer 2 interrupt routine
;
; No assumptions
; Requirements: Temp variables can NOT be used since PSW.x is not set
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
t2_int: ; Happens every 32ms
    push    PSW         ; Preserve registers through interrupt
    push    ACC
    clr TMR2CN0_TF2H                ; Clear interrupt flag
    inc Timer2_X

IF MCU_48MHZ == 1
    ; Check skip variable
    mov A, Skip_T2_Int
    jz  t2_int_start                ; Execute this interrupt

    mov Skip_T2_Int, #0
    ajmp    t2_int_exit

t2_int_start:
    mov Skip_T2_Int, #1         ; Skip next interrupt
ENDIF
    
    ; Update RC pulse timeout counter 
    mov A, Rcp_Timeout_Cntd         ; RC pulse timeout count zero?
    jz  ($+4)                   ; Yes - do not decrement

    dec Rcp_Timeout_Cntd            ; No decrement

    ; Check RC pulse against stop value
    clr C
    mov A, New_Rcp              ; Load new pulse value
    jz  t2_int_rcp_stop         ; Check if pulse is below stop value

    ; RC pulse higher than stop value, reset stop counter
    mov Rcp_Stop_Cnt, #0            ; Reset rcp stop counter
    ajmp    t2_int_exit

t2_int_rcp_stop:
    ; RC pulse less than stop value
    mov A, Rcp_Stop_Cnt         ; Increment stop counter
    add A, #1
    mov Rcp_Stop_Cnt, A
    jnc ($+5)                   ; Branch if counter has not wrapped

    mov Rcp_Stop_Cnt, #0FFh         ; Set stop counter to max

t2_int_exit:
    pop ACC         ; Restore preserved registers
    pop PSW
    reti


    
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Int0 interrupt routine
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
int0_int:   ; Used for RC pulse timing
    push    ACC
    mov A, DShot_Byte          ; Read pwm for DShot immediately

    mov TL1, DShot_Timer_Preset ; Reset sync timer
    movx    @DPTR, A            ; Store pwm
    inc DPTR
    pop ACC
    reti


int0_int_outside_range:
    inc Rcp_Outside_Range_Cnt
    mov A, Rcp_Outside_Range_Cnt
    jnz ($+4)

    dec Rcp_Outside_Range_Cnt

    clr C
    mov A, Rcp_Outside_Range_Cnt
    subb    A, #50                      ; Allow a given number of outside pulses
    jnc ($+4)
    ajmp    int0_int_set_timeout            ; If outside limits - ignore first pulses

    mov New_Rcp, #0                 ; Set pulse length to zero
    ajmp    int0_int_exit                   ; Exit without reseting timeout


int0_int_pulse_ready:
    mov New_Rcp, Temp1                  ; Store new pulse length
    setb    Flags2.RCP_UPDATED              ; Set updated flag
    ; Check if zero
    mov A, Temp1                        ; Load new pulse value
    jz  ($+5)                       ; Check if pulse is zero

    mov Rcp_Stop_Cnt, #0                ; Reset rcp stop counter

    ; Set pwm limit
    clr C
    mov A, Pwm_Limit                    ; Limit to the smallest
    mov Temp5, A                        ; Store limit in Temp5
    subb    A, Pwm_Limit_By_Rpm
    jc  ($+4)

    mov Temp5, Pwm_Limit_By_Rpm         

    ; Check against limit
    clr C
    mov A, Temp5
    subb    A, New_Rcp
    jnc int0_int_set_pwm_registers

    mov A, Temp5                        ; Multiply limit by 4 (8 for 48MHz MCUs)
    mov B, #8

    mul AB
    mov Temp3, A
    mov Temp4, B

int0_int_set_pwm_registers:
    
IF D_BITS == 0
    mov A, Temp4
    cpl A
    anl A, #7
    mov Temp2, A
    mov A, Temp3
    cpl A
    mov Temp1, A
ELSEIF D_BITS == 1
    mov A, Temp4
    cpl A
    rrc A
    anl A, #3
    mov Temp2, A
    mov A, Temp3
    cpl A
    mov Temp5, A
    rrc A
    mov Temp1, A

ELSEIF D_BITS == 2
    mov A, Temp4
    cpl A
    rrc A
    rrc A
    mov PSW_F0, C
    mov C, ACC.7
    anl A, #1
    mov Temp2, A
    mov A, Temp3
    cpl A
    mov Temp5, A
    rrc A
    mov C, PSW_F0
    rrc A
    mov Temp1, A

ELSEIF D_BITS == 3
    mov Temp2, #0
    mov A, Temp4
    cpl A
    swap A
    rl A
    anl A, #0e0h
    xch A, Temp3
    cpl A
    mov Temp5, A
    swap A
    rl A
    anl A, #1fh
    orl A, Temp3
    mov Temp1, A
ENDIF
   
IF D_BITS > 0
    mov A, Dither_Cur_Bit
    rl A
    mov Dither_Cur_Bit, A

    mov A, Temp5
    anl A, #((1 SHL D_BITS) - 1)
    add A, #(Dither_Table - int0_next)
    movc A, @A + PC
int0_next:  

    
    anl A, Dither_Cur_Bit
    jz int0_no_dither

    mov A, Temp1
    add A, #1
    mov A, Temp2
    addc A, #0

    jb ACC.CYCLE_OVFL_BIT, int0_no_dither

    inc Temp1
    mov Temp2, A
    
int0_no_dither:
ENDIF
    

IF FETON_DELAY != 0
    clr C
    mov A, Temp1                        ; Skew damping fet timing
    subb A, #(FETON_DELAY SHR FDELAY_SHIFT)
    mov Temp3, A
    mov A, Temp2
    subb    A, #0   
    mov Temp4, A
    jnc int0_int_set_pwm_damp_set

    mov Temp3, #0
    mov Temp4, #0

int0_int_set_pwm_damp_set:
ENDIF

    mov Power_Pwm_Reg_L, Temp1
    mov Power_Pwm_Reg_H, Temp2

IF NO_DAMPING == 1
    mov Damp_Pwm_Reg_L, #0h
    mov Damp_Pwm_Reg_H, #0h
ELSE    
    mov Damp_Pwm_Reg_L, Temp3
    mov Damp_Pwm_Reg_H, Temp4
ENDIF
    
    mov Rcp_Timeout_Cntd, #10           ; Set timeout count
    Clear_COVF_Interrupt
    Enable_COVF_Interrupt               ; Generate a pca interrupt
    jmp int0_int_schedule_pca

Dither_Table:
IF D_BITS == 1
DB 00h, 55h
ELSEIF D_BITS == 2    
DB 00h, 11h, 55h, 77h
ELSEIF D_BITS == 3    
DB 00h, 01h, 11h, 15h, 55h, 57h, 77h, 7fh
ENDIF


int0_int_set_timeout:
    mov Rcp_Timeout_Cntd, #10           ; Set timeout count
int0_int_exit:
int0_int_schedule_pca:  
IF MCU_48MHZ == 1
    mov A, EIE1
    mov C, ACC.0
    anl C, SMB0CN0_SI
    jc int0_int_delaypc0
ENDIF    
    pop B                           ; Restore preserved registers
    pop ACC
    pop PSW
    setb    IE_EX0                  ; Enable int0 interrupts
    setb    IE_EX1                  ; Enable int1 interrupts
    orl EIE1, #10h                  ; Enable pca interrupts
    reti
int0_int_delaypc0:
    pop B                           ; Restore preserved registers
    pop ACC
    pop PSW
    reti
    


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Int1 interrupt routine
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
int1_int:   ; Used for RC pulse timing
    clr IE_EX1          ; Disable int1 interrupts
    setb    TCON_TR1            ; Start timer 1
    clr TMR2CN0_TR2             ; Timer 2 disabled
    mov DShot_Frame_Start_L, TMR2L  ; Read timer value
    mov DShot_Frame_Start_H, TMR2H
    setb    TMR2CN0_TR2             ; Timer 2 enabled
reti

     
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; PCA interrupt routine
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
pca_int:    ; Used for setting pwm registers
    anl PCA0PWM, #0dfh ; clear covr irq
    push ACC
    clr IE_EA
    mov A, PCA0L
    mov A, PCA0H
    jnb ACC.CYCLE_OVFL_BIT, pca_no_update

IF D_BITS < 3
    mov PCA_POWER_L, Power_Pwm_Reg_L
    mov PCA_POWER_H, Power_Pwm_Reg_H
    
IF FETON_DELAY != 0
    mov PCA_DAMP_L, Damp_Pwm_Reg_L
    mov PCA_DAMP_H, Damp_Pwm_Reg_H
ENDIF
ELSE
    mov PCA_POWER_H, Power_Pwm_Reg_L
IF FETON_DELAY != 0
    mov PCA_DAMP_H, Damp_Pwm_Reg_L
ENDIF
ENDIF    

    Disable_COVF_Interrupt
pca_no_update:  
    setb IE_EA
    pop ACC
    reti

    
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait xms ~(x*4*250)  (Different entry points) 
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait1ms:    
    mov Temp2, #1
    jmp waitxms_i

wait3ms:    
    mov Temp2, #3
    jmp waitxms_i

wait10ms:   
    mov Temp2, #10
    jmp waitxms_i

wait15ms:   
    mov Temp2, #15
    jmp waitxms_i

wait30ms:   
    mov Temp2, #30
    jmp waitxms_i

wait100ms:  
    mov Temp2, #100
    jmp waitxms_i

wait200ms:  
    mov Temp2, #200
    jmp waitxms_i

waitxms_i:
IF MCU_48MHZ == 1
    Set_MCU_Clk_24MHz
ENDIF    
    

waitxms_o:  ; Outer loop
    mov Temp1, #20
    
waitxms_m:  ; Middle loop for 50us
    mov A, #240
waitnow:    
    nop
    nop
    nop
    ; 3 + 4 cycles at 24mhz, 3 + 7 at 48 mhz due to prefetch
    djnz    ACC, waitnow
    djnz    Temp1, waitxms_m
    djnz    Temp2, waitxms_o
IF MCU_48MHZ == 1
    Set_MCU_Clk_48MHz
ENDIF    
    ret

twait1ms:   
    mov Temp2, #1
    jmp twaitxms_i

twait3ms:   
    mov Temp2, #3
    jmp twaitxms_i

twait10ms:  
    mov Temp2, #10
    jmp twaitxms_i

twait30ms:  
    mov Temp2, #30
    jmp twaitxms_i

twait100ms: 
    mov Temp2, #100
    jmp twaitxms_i

twait200ms: 
    mov Temp2, #200
    jmp twaitxms_i

twaitxms_i: 

    mov TMR3CN0, #0             ; stop timer 3
IF MCU_48MHZ == 1
    mov TMR3L, #-208
    mov TMR3H, #-7
ELSE
    mov TMR3L, #-104
    mov TMR3H, #-4
ENDIF    
    mov TMR3CN0, #04h
    setb WAIT_ACTIVE
    orl EIE1, #80h  ; Enable timer 3 interrupts
    
    Wait_Pending
    djnz Temp2, twaitxms_i
ret



;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Set pwm limit low rpm
;
; No assumptions
;
; Sets power limit for low rpms and disables demag for low rpms
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
set_pwm_limit_low_rpm:
    ; Set pwm limit
    mov Temp1, #0FFh                    ; Default full power
    jb  Flags1.STARTUP_PHASE, set_pwm_limit_low_rpm_exit    ; Exit if startup phase set

    mov Temp2, #Pgm_Enable_Power_Prot       ; Check if low RPM power protection is enabled
    mov A, @Temp2
    jz  set_pwm_limit_low_rpm_exit      ; Exit if disabled

    mov A, Comm_Period4x_H
    jz  set_pwm_limit_low_rpm_exit      ; Avoid divide by zero

    mov A, #255                     ; Divide 255 by Comm_Period4x_H
    mov B, Comm_Period4x_H
    div AB
    mov B, Low_Rpm_Pwr_Slope            ; Multiply by slope
    jnb Flags1.INITIAL_RUN_PHASE, ($+6) ; More protection for initial run phase 
    mov B, #5
    mul AB
    mov Temp1, A                        ; Set new limit             
    xch A, B
    jz  ($+4)                       ; Limit to max
    
    mov Temp1, #0FFh                

    clr C
    mov A, Temp1                        ; Limit to min
    subb    A, Pwm_Limit_Beg
    jnc set_pwm_limit_low_rpm_exit

    mov Temp1, Pwm_Limit_Beg

set_pwm_limit_low_rpm_exit:
    mov Pwm_Limit_By_Rpm, Temp1             
    ret
    

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Set pwm limit high rpm
;
; No assumptions
;
; Sets power limit for high rpms
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
set_pwm_limit_high_rpm:
IF MCU_48MHZ == 1
    clr C
    mov A, Comm_Period4x_L
    subb    A, #0A0h                ; Limit Comm_Period to 160, which is 500k erpm
    mov A, Comm_Period4x_H
    subb    A, #00h
ELSE    
    clr	C
	mov	A, Comm_Period4x_L
	subb	A, #0E4h				; Limit Comm_Period to 228, which is 350k erpm
	mov	A, Comm_Period4x_H
	subb	A, #00h
ENDIF    
    mov A, Pwm_Limit_By_Rpm
    jnc set_pwm_limit_high_rpm_inc_limit
    
    dec A
    ajmp    set_pwm_limit_high_rpm_store
    
set_pwm_limit_high_rpm_inc_limit:
    inc A
set_pwm_limit_high_rpm_store:
    jz  ($+4)

    mov Pwm_Limit_By_Rpm, A

    ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Start ADC conversion
;
; No assumptions
;
; Start conversion used for measuring power supply voltage
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
start_adc_conversion:
    ; Start adc
    Start_Adc 
    ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Check temperature, power supply voltage and limit power
;
; No assumptions
;
; Used to limit main motor power in order to maintain the required voltage
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
check_temp_voltage_and_limit_power:
    inc Adc_Conversion_Cnt          ; Increment conversion counter
    clr C
    mov A, Adc_Conversion_Cnt       ; Is conversion count equal to temp rate?
    subb    A, #8
    jc  check_voltage_start         ; No - check voltage

    ; Wait for ADC conversion to complete
    jnb ADC0CN0_ADINT, check_temp_voltage_and_limit_power
    ; Read ADC result
    Read_Adc_Result
    ; Stop ADC
    Stop_Adc

    mov Adc_Conversion_Cnt, #0      ; Yes - temperature check. Reset counter
    mov A, Temp2                    ; Move ADC MSB to Temp3
    mov Temp3, A
    mov Temp2, #Pgm_Enable_Temp_Prot    ; Is temp protection enabled?
    mov A, @Temp2
    jz  temp_check_exit         ; No - branch
//  jmp temp_check_exit         ; No - branch

    mov A, Temp3                    ; Is temperature reading below 256?
    jnz temp_average_inc_dec        ; No - proceed

    mov A, Current_Average_Temp     ; Yes -  decrement average
    jz  temp_average_updated        ; Already zero - no change
    jmp temp_average_dec            ; Decrement 

temp_average_inc_dec:
    clr C
    mov A, Temp1                    ; Check if current temperature is above or below average
    subb    A, Current_Average_Temp
    jz  temp_average_updated_load_acc   ; Equal - no change

    mov A, Current_Average_Temp     ; Above - increment average
    jnc temp_average_inc                

    jz  temp_average_updated        ; Below - decrement average if average is not already zero
temp_average_dec:
    dec A                       ; Decrement average
    jmp temp_average_updated

temp_average_inc:
    inc A                       ; Increment average
    jz  temp_average_dec
    jmp temp_average_updated

temp_average_updated_load_acc:
    mov A, Current_Average_Temp
temp_average_updated:
    mov Current_Average_Temp, A
    clr C
    subb    A, Temp_Prot_Limit          ; Is temperature below first limit?
    jc  temp_check_exit         ; Yes - exit

    mov  Pwm_Limit, #192            ; No - limit pwm

    clr C
    subb    A, #(TEMP_LIMIT_STEP/2)     ; Is temperature below second limit
    jc  temp_check_exit         ; Yes - exit

    mov  Pwm_Limit, #128            ; No - limit pwm

    clr C
    subb    A, #(TEMP_LIMIT_STEP/2)     ; Is temperature below third limit
    jc  temp_check_exit         ; Yes - exit

    mov  Pwm_Limit, #64             ; No - limit pwm

    clr C
    subb    A, #(TEMP_LIMIT_STEP/2)     ; Is temperature below final limit
    jc  temp_check_exit         ; Yes - exit

    mov  Pwm_Limit, #0              ; No - limit pwm

temp_check_exit:
    ret

check_voltage_start:
    ; Increase pwm limit
    mov  A, Pwm_Limit
    add A, #16          
    jnc ($+4)                   ; If not max - branch

    mov A, #255

    mov Pwm_Limit, A                ; Increment limit 
    ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Set startup PWM routine
;
; Either the SETTLE_PHASE or the STEPPER_PHASE flag must be set
;
; Used for pwm control during startup
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
set_startup_pwm:    
    ; Adjust startup power
    mov A, #50                      ; Set power
    mov Temp2, #Pgm_Startup_Pwr_Decoded
    mov B, @Temp2
    mul AB
    xch A, B
    mov C, B.7                      ; Multiply result by 2 (unity gain is 128)
    rlc A
    mov Pwm_Limit_Beg, A                ; Set initial pwm limit
    ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Initialize timing routine
;
; No assumptions
;
; Part of initialization before motor start
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
initialize_timing: 
    mov Comm_Period4x_L, #00h               ; Set commutation period registers
    mov Comm_Period4x_H, #0F0h
    ret


store_comm_timing:      ; Entry point for run phase
    ; Read commutation time
    clr IE_EA
    clr TMR2CN0_TR2     ; Timer 2 disabled
    mov Temp1, TMR2L        ; Load timer value
    mov Temp2, TMR2H    
    mov Temp3, Timer2_X
    jnb TMR2CN0_TF2H, ($+4) ; Check if interrupt is pending
    inc Temp3           ; If it is pending, then timer has already wrapped
    setb    TMR2CN0_TR2     ; Timer 2 enabled
    setb    IE_EA
    clr C
    mov A, Temp3
    rrc A
    mov Temp3, A
    mov A, Temp2
    rrc A
    mov Temp2, A
    mov A, Temp1
    rrc A
    mov Temp1, A

    mov Prev_Prev_Comm_L, Prev_Comm_L
    mov Prev_Prev_Comm_H, Prev_Comm_H
    
                                ; Calculate this commutation time
    mov Prev_Comm_L, Temp1      ; Store timestamp as previous commutation
    mov Prev_Comm_H, Temp2
    mov Prev_Comm_X, Temp3      ; Store extended timestamp as previous commutation

ret

    
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Calculate next commutation timing routine
;
; No assumptions
;
; Called immediately after each commutation
; Also sets up timer 3 to wait advance timing
; Two entry points are used
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
calc_next_comm_timing:      ; Entry point for run phase
    ; Read commutation time
    clr IE_EA
    clr TMR2CN0_TR2     ; Timer 2 disabled
    mov Temp1, TMR2L        ; Load timer value
    mov Temp2, TMR2H    
    mov Temp3, Timer2_X
    jnb TMR2CN0_TF2H, ($+4) ; Check if interrupt is pending
    inc Temp3           ; If it is pending, then timer has already wrapped
    setb    TMR2CN0_TR2     ; Timer 2 enabled
    setb    IE_EA
IF MCU_48MHZ == 1
    clr C
    mov A, Temp3
    rrc A
    mov Temp3, A
    mov A, Temp2
    rrc A
    mov Temp2, A
    mov A, Temp1
    rrc A
    mov Temp1, A
ENDIF

                                ; Calculate this commutation time
    mov Temp4, Prev_Comm_L
    mov Temp5, Prev_Comm_H
    mov Prev_Comm_L, Temp1      ; Store timestamp as previous commutation
    mov Prev_Comm_H, Temp2

    clr C
    mov A, Temp1
    subb    A, Temp4                ; Calculate the new commutation time
    mov Temp1, A
    mov A, Temp2
    subb    A, Temp5

;    mov Comm_Period1x_L, Temp1
;    mov Comm_Period1x_H, A
    
    jb  Flags1.STARTUP_PHASE, calc_next_comm_startup

IF MCU_48MHZ == 1
    anl A, #7Fh
ENDIF    
    mov Temp2, A
    jnb Flags1.HIGH_RPM, ($+6)  ; Branch if high rpm
    ljmp    calc_next_comm_timing_fast
    sjmp    calc_next_comm_normal

calc_next_comm_startup:
    mov Temp6, Prev_Comm_X
    mov Prev_Comm_X, Temp3      ; Store extended timestamp as previous commutation
    mov Temp2, A
    mov A, Temp3
    subb    A, Temp6                ; Calculate the new extended commutation time
IF MCU_48MHZ == 1
    anl A, #7Fh
ENDIF    
    mov Temp3, A
    jz  calc_next_comm_startup_no_X

    mov Temp1, #0FFh
    mov Temp2, #0FFh
    sjmp    calc_next_comm_startup_average

calc_next_comm_startup_no_X:
    mov Temp7, Prev_Prev_Comm_L
    mov Temp8, Prev_Prev_Comm_H
    mov Prev_Prev_Comm_L, Temp4
    mov Prev_Prev_Comm_H, Temp5
    mov Temp1, Prev_Comm_L      ; Reload this commutation time  
    mov Temp2, Prev_Comm_H
    clr C
    mov A, Temp1
    subb    A, Temp7                ; Calculate the new commutation time based upon the two last commutations (to reduce sensitivity to offset)
    mov Temp1, A
    mov A, Temp2
    subb    A, Temp8
    mov Temp2, A

calc_next_comm_startup_average:
    clr C
    mov A, Comm_Period4x_H      ; Average with previous and save
    rrc A
    mov Temp4, A
    mov A, Comm_Period4x_L
    rrc A
    mov Temp3, A
    mov A, Temp1            
    add A, Temp3
    mov Comm_Period4x_L, A
    mov A, Temp2
    addc    A, Temp4
    mov Comm_Period4x_H, A
    jnc ($+8)

    mov Comm_Period4x_L, #0FFh
    mov Comm_Period4x_H, #0FFh

    ljmp    calc_new_wait_times_setup

calc_next_comm_normal:
    ; Calculate new commutation time 
    mov Temp3, Comm_Period4x_L  ; Comm_Period4x(-l-h) holds the time of 4 commutations
    mov Temp4, Comm_Period4x_H
    mov Temp5, Comm_Period4x_L  ; Copy variables
    mov Temp6, Comm_Period4x_H
    mov Temp7, #4               ; Divide Comm_Period4x 4 times as default
    mov Temp8, #2               ; Divide new commutation time 2 times as default
    clr C
    mov A, Temp4
    subb    A, #04h
    jc  calc_next_comm_avg_period_div

    dec Temp7               ; Reduce averaging time constant for low speeds
    dec Temp8

    clr C
    mov A, Temp4
    subb    A, #08h
    jc  calc_next_comm_avg_period_div

    jb  Flags1.INITIAL_RUN_PHASE, calc_next_comm_avg_period_div ; Do not average very fast during initial run

    dec Temp7               ; Reduce averaging time constant more for even lower speeds
    dec Temp8

calc_next_comm_avg_period_div:
    clr C
    mov A, Temp6                    
    rrc A                   ; Divide by 2
    mov Temp6, A    
    mov A, Temp5                
    rrc A
    mov Temp5, A
    djnz    Temp7, calc_next_comm_avg_period_div

    clr C
    mov A, Temp3
    subb    A, Temp5                ; Subtract a fraction
    mov Temp3, A
    mov A, Temp4
    subb    A, Temp6
    mov Temp4, A
    mov A, Temp8                ; Divide new time
    jz  calc_next_comm_new_period_div_done

calc_next_comm_new_period_div:
    clr C
    mov A, Temp2                    
    rrc A                   ; Divide by 2
    mov Temp2, A    
    mov A, Temp1                
    rrc A
    mov Temp1, A
    djnz    Temp8, calc_next_comm_new_period_div

calc_next_comm_new_period_div_done:
    mov A, Temp3
    add A, Temp1                ; Add the divided new time
    mov Temp3, A
    mov A, Temp4
    addc    A, Temp2
    mov Temp4, A
    mov Comm_Period4x_L, Temp3  ; Store Comm_Period4x_X
    mov Comm_Period4x_H, Temp4
    jnc calc_new_wait_times_setup; If period larger than 0xffff - go to slow case

    mov Temp4, #0FFh
    mov Comm_Period4x_L, Temp4  ; Set commutation period registers to very slow timing (0xffff)
    mov Comm_Period4x_H, Temp4

calc_new_wait_times_setup:  
    ; Set high rpm bit (if above 156k erpm)
    clr C
    mov A, Temp4
    subb    A, #2
    jnc ($+4)

    setb    Flags1.HIGH_RPM         ; Set high rpm bit
    
    ; Load programmed commutation timing
    jnb Flags1.STARTUP_PHASE, calc_new_wait_per_startup_done    ; Set dedicated timing during startup

    mov Temp8, #3
    ljmp    calc_new_wait_per_demag_done

calc_new_wait_per_startup_done:
    mov Temp1, #Pgm_Comm_Timing ; Load timing setting
    mov A, @Temp1               
    mov Temp8, A                ; Store in Temp8
    clr C
    mov A, Demag_Detected_Metric    ; Check demag metric
    subb    A, #130
    jc  calc_new_wait_per_demag_done

    inc Temp8               ; Increase timing

    clr C
    mov A, Demag_Detected_Metric
    subb    A, #160
    jc  ($+3)

    inc Temp8               ; Increase timing again

    clr C
    mov A, Temp8                ; Limit timing to max
    subb    A, #6
    jc  ($+4)

    mov Temp8, #5               ; Set timing to max

calc_new_wait_per_demag_done:
    ; Set timing reduction
    mov Temp7, #2
    ; Load current commutation timing
    mov A, Comm_Period4x_H      ; Divide 4 times
    swap    A
    anl A, #00Fh
    mov Temp2, A
    mov A, Comm_Period4x_H
    swap    A
    anl A, #0F0h
    mov Temp1, A
    mov A, Comm_Period4x_L
    swap    A
    anl A, #00Fh
    add A, Temp1
    mov Temp1, A

    clr C
    mov A, Temp1
    subb    A, Temp7
    mov Temp3, A
    mov A, Temp2                
    subb    A, #0
    mov Temp4, A
    jc  load_min_time           ; Check that result is still positive

    clr C
    mov A, Temp3
    subb    A, #1
    mov A, Temp4            
    subb    A, #0
    jnc calc_new_wait_times_exit    ; Check that result is still above minumum

load_min_time:
    mov Temp3, #1
    clr A
    mov Temp4, A 

calc_new_wait_times_exit:   
    ljmp    wait_advance_timing


; Fast calculation (Comm_Period4x_H less than 2)
calc_next_comm_timing_fast:         
    ; Calculate new commutation time
    mov Temp3, Comm_Period4x_L  ; Comm_Period4x(-l-h) holds the time of 4 commutations
    mov Temp4, Comm_Period4x_H
;    jb RC_DURING_COMMUTATION, calc_new_wait_times_fast_timing_reduction

    mov A, Temp4                ; Divide by 2 4 times
    swap    A
    mov Temp7, A
    mov A, Temp3
    swap A
    anl A, #0Fh
    orl A, Temp7
    mov Temp5, A
    clr C
    mov A, Temp3                ; Subtract a fraction
    subb    A, Temp5
    mov Temp3, A
    mov A, Temp4                
    subb    A, #0
    mov Temp4, A
    clr C
    mov A, Temp1
    rrc A                   ; Divide by 2 2 times
    clr C
    rrc A
    mov Temp1, A
    mov A, Temp3                ; Add the divided new time
    add A, Temp1
    mov Temp3, A
    mov A, Temp4
    addc    A, #0
    mov Temp4, A
    mov Comm_Period4x_L, Temp3  ; Store Comm_Period4x_X
    mov Comm_Period4x_H, Temp4
    clr C
    mov A, Temp4                ; If erpm below 156k - go to normal case
    subb    A, #2
    jc  ($+4)

    clr Flags1.HIGH_RPM         ; Clear high rpm bit

calc_new_wait_times_fast_timing_reduction:  
    ; Set timing reduction
    mov Temp1, #2
    mov A, Temp4                ; Divide by 2 4 times
    swap    A
    mov Temp7, A
    mov Temp4, #0
    mov A, Temp3
    swap A
    anl A, #0Fh
    orl A, Temp7
    mov Temp3, A
    clr C
    mov A, Temp3
    subb    A, Temp1
    mov Temp3, A
    jc  load_min_time_fast      ; Check that result is still positive

    clr C
    subb    A, #1
    jnc calc_new_wait_times_fast_done   ; Check that result is still above minumum

load_min_time_fast:
    mov Temp3, #1

calc_new_wait_times_fast_done:  
;    clr RC_DURING_COMMUTATION
    mov Temp1, #Pgm_Comm_Timing ; Load timing setting
    mov A, @Temp1               
    mov Temp8, A                ; Store in Temp8


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait advance timing routine
;
; No assumptions
; NOTE: Be VERY careful if using temp registers. They are passed over this routine
;
; Waits for the advance timing to elapse and sets up the next zero cross wait
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait_advance_timing:    
//  jnb WAIT_ACTIVE, wait_advance_timing_not_pending
    Wait_Pending
//  ajmp    wait_advance_timing


//wait_advance_timing_not_pending:    ; Setup next wait time
    mov TMR3RLL, Wt_ZC_Tout_Start_L
    mov TMR3RLH, Wt_ZC_Tout_Start_H
    setb    WAIT_ACTIVE
    orl EIE1, #80h  ; Enable timer 3 interrupts

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Calculate new wait times routine
;
; No assumptions
;
; Calculates new wait times
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
calc_new_wait_times:    
    clr C
    clr A
    subb    A, Temp3                ; Negate
    mov Temp1, A    
    clr A
    subb    A, Temp4                
    mov Temp2, A    
IF MCU_48MHZ == 1
    clr C
    mov A, Temp1                ; Multiply by 2
    rlc A
    mov Temp1, A
    mov A, Temp2
    rlc A
    mov Temp2, A
ENDIF    
    jnb Flags1.HIGH_RPM, ($+6)  ; Branch if high rpm
    ljmp    calc_new_wait_times_fast

    mov A, Temp1                ; Copy values
    mov Temp3, A
    mov A, Temp2
    mov Temp4, A
    setb    C                   ; Negative numbers - set carry
    mov A, Temp2                
    rrc A                   ; Divide by 2
    mov Temp6, A
    mov A, Temp1
    rrc A
    mov Temp5, A
    mov Wt_Zc_Tout_Start_L, Temp1; Set 15deg time for zero cross scan timeout
    mov Wt_Zc_Tout_Start_H, Temp2
    clr C
    mov A, Temp8                ; (Temp8 has Pgm_Comm_Timing)
    subb    A, #3               ; Is timing normal?
    jz  store_times_decrease    ; Yes - branch

    mov A, Temp8                
    jb  ACC.0, adjust_timing_two_steps  ; If an odd number - branch

    mov A, Temp1                ; Add 7.5deg and store in Temp1/2
    add A, Temp5
    mov Temp1, A
    mov A, Temp2
    addc    A, Temp6
    mov Temp2, A
    mov A, Temp5                ; Store 7.5deg in Temp3/4
    mov Temp3, A
    mov A, Temp6            
    mov Temp4, A
    jmp store_times_up_or_down

adjust_timing_two_steps:
    mov A, Temp1                ; Add 15deg and store in Temp1/2
    add A, Temp1
    mov Temp1, A
    mov A, Temp2
    addc    A, Temp2
    mov Temp2, A
    clr C
    mov A, Temp1
    add A, #1
    mov Temp1, A
    mov A, Temp2
    addc    A, #0
    mov Temp2, A
    mov Temp3, #-1              ; Store minimum time in Temp3/4
    mov Temp4, #0FFh

store_times_up_or_down:
    clr C
    mov A, Temp8                
    subb    A, #3                   ; Is timing higher than normal?
    jc  store_times_decrease        ; No - branch

store_times_increase:
    mov Wt_Comm_Start_L, Temp3      ; Now commutation time (~60deg) divided by 4 (~15deg nominal)
    mov Wt_Comm_Start_H, Temp4
    mov Wt_Adv_Start_L, Temp1       ; New commutation advance time (~15deg nominal)
    mov Wt_Adv_Start_H, Temp2
    mov Wt_Zc_Scan_Start_L, Temp5   ; Use this value for zero cross scan delay (7.5deg)
    mov Wt_Zc_Scan_Start_H, Temp6
    ljmp    wait_before_zc_scan

store_times_decrease:
    mov Wt_Comm_Start_L, Temp1      ; Now commutation time (~60deg) divided by 4 (~15deg nominal)
    mov Wt_Comm_Start_H, Temp2
    mov Wt_Adv_Start_L, Temp3       ; New commutation advance time (~15deg nominal)
    mov Wt_Adv_Start_H, Temp4
    mov Wt_Zc_Scan_Start_L, Temp5   ; Use this value for zero cross scan delay (7.5deg)
    mov Wt_Zc_Scan_Start_H, Temp6
    jnb Flags1.STARTUP_PHASE, store_times_exit

    mov Wt_Comm_Start_L, #0F0h      ; Set very short delays for all but advance time during startup, in order to widen zero cross capture range
    mov Wt_Comm_Start_H, #0FFh
    mov Wt_Zc_Scan_Start_L, #0F0h
    mov Wt_Zc_Scan_Start_H, #0FFh
    mov Wt_Zc_Tout_Start_L, #0F0h
    mov Wt_Zc_Tout_Start_H, #0FFh

store_times_exit:
    ljmp    wait_before_zc_scan


calc_new_wait_times_fast:   
    mov A, Temp1                ; Copy values
    mov Temp3, A
    setb    C                   ; Negative numbers - set carry
    mov A, Temp1                ; Divide by 2
    rrc A
    mov Temp5, A
    mov Wt_Zc_Tout_Start_L, Temp1; Set 15deg time for zero cross scan timeout
    clr C
    mov A, Temp8                ; (Temp8 has Pgm_Comm_Timing)
    subb    A, #3               ; Is timing normal?
    jz  store_times_decrease_fast; Yes - branch

    mov A, Temp8                
    jb  ACC.0, adjust_timing_two_steps_fast ; If an odd number - branch

    mov A, Temp1                ; Add 7.5deg and store in Temp1
    add A, Temp5
    mov Temp1, A
    mov A, Temp5                ; Store 7.5deg in Temp3
    mov Temp3, A
    ajmp    store_times_up_or_down_fast

adjust_timing_two_steps_fast:
    mov A, Temp1                ; Add 15deg and store in Temp1
    add A, Temp1
    add A, #1
    mov Temp1, A
    mov Temp3, #-1          ; Store minimum time in Temp3

store_times_up_or_down_fast:
    clr C
    mov A, Temp8                
    subb    A, #3               ; Is timing higher than normal?
    jc  store_times_decrease_fast; No - branch

store_times_increase_fast:
    mov Wt_Comm_Start_L, Temp3      ; Now commutation time (~60deg) divided by 4 (~15deg nominal)
    mov Wt_Adv_Start_L, Temp1       ; New commutation advance time (~15deg nominal)
    mov Wt_Zc_Scan_Start_L, Temp5   ; Use this value for zero cross scan delay (7.5deg)
    ljmp    wait_before_zc_scan

store_times_decrease_fast:
    mov Wt_Comm_Start_L, Temp1      ; Now commutation time (~60deg) divided by 4 (~15deg nominal)
    mov Wt_Adv_Start_L, Temp3       ; New commutation advance time (~15deg nominal)
    mov Wt_Zc_Scan_Start_L, Temp5   ; Use this value for zero cross scan delay (7.5deg)


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait before zero cross scan routine
;
; No assumptions
;
; Waits for the zero cross scan wait time to elapse
; Also sets up timer 3 for the zero cross scan timeout time
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait_before_zc_scan:    
    Wait_Pending
//  jnb WAIT_ACTIVE, ($+5) 
//  ajmp    wait_before_zc_scan

    mov Startup_Zc_Timeout_Cntd, #2
setup_zc_scan_timeout:
    setb    WAIT_ACTIVE
    orl EIE1, #80h          ; Enable timer 3 interrupts
    mov A, Flags1
    anl A, #((1 SHL STARTUP_PHASE)+(1 SHL INITIAL_RUN_PHASE))
    jz  wait_before_zc_scan_exit        

    mov Temp1, Comm_Period4x_L  ; Set long timeout when starting
    mov Temp2, Comm_Period4x_H
    clr C
    mov A, Temp2
    rrc A
    mov Temp2, A
    mov A, Temp1
    rrc A
    mov Temp1, A

IF MCU_48MHZ == 0
    clr C
    mov A, Temp2
    rrc A
    mov Temp2, A
    mov A, Temp1
    rrc A
    mov Temp1, A
ENDIF    
    jnb Flags1.STARTUP_PHASE, setup_zc_scan_timeout_startup_done

    mov A, Temp2
    add A, #40h             ; Increase timeout somewhat to avoid false wind up
    mov Temp2, A

setup_zc_scan_timeout_startup_done:
    clr IE_EA
    anl EIE1, #7Fh          ; Disable timer 3 interrupts
    mov TMR3CN0, #00h           ; Timer 3 disabled and interrupt flag cleared
    clr C
    clr A
    subb    A, Temp1                ; Set timeout
    mov TMR3L, A
    clr A
    subb    A, Temp2        
    mov TMR3H, A
    mov TMR3CN0, #04h           ; Timer 3 enabled and interrupt flag cleared
    setb    WAIT_ACTIVE
    orl EIE1, #80h          ; Enable timer 3 interrupts
    setb    IE_EA

wait_before_zc_scan_exit:          
    ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait for comparator to go low/high routines
;
; No assumptions
;
; Waits for the zero cross scan wait time to elapse
; Then scans for comparator going low/high
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait_for_comp_out_low:
    setb    Flags0.DEMAG_DETECTED       ; Set demag detected flag as default
    mov Comparator_Read_Cnt, #0     ; Reset number of comparator reads
    mov Bit_Access, #00h            ; Desired comparator output
    jnb Flags1.DIR_CHANGE_BRAKE, ($+6)
    mov Bit_Access, #40h        
    sjmp    wait_for_comp_out_start

wait_for_comp_out_high:
    setb    Flags0.DEMAG_DETECTED       ; Set demag detected flag as default
    mov Comparator_Read_Cnt, #0     ; Reset number of comparator reads
    mov Bit_Access, #40h            ; Desired comparator output
    jnb Flags1.DIR_CHANGE_BRAKE, ($+6)
    mov Bit_Access, #00h        

wait_for_comp_out_start:
IF DEBUG_COMP
    setb P2.DebugPin
ENDIF
    ; Set number of comparator readings
    mov Temp1, #1                   ; Number of OK readings required
    mov Temp2, #1                   ; Max number of readings required
    jb  Flags1.HIGH_RPM, comp_check_timeout //comp_scale_samples    ; Branch if high rpm

    mov A, Flags1                   ; Clear demag detected flag if start phases
    anl A, #((1 SHL STARTUP_PHASE)+(1 SHL INITIAL_RUN_PHASE))
    jz  ($+4)
        
    clr Flags0.DEMAG_DETECTED

       


IF  NK1306 == 0
    mov Temp2, #20              ; Too low value (~<15) causes rough running at pwm harmonics. Too high a value (~>35) causes the RCT4215 630 to run rough on full throttle
    mov     A, Comm_Period4x_H          ; Set number of readings higher for lower speeds
    clr C
    rrc A
    jnz ($+3)
    inc A
    mov Temp1, A
    clr C                       
    subb    A, #20
    jc  ($+4)

    mov Temp1, #20
ELSE  
    mov Temp2, #5               ; Too low value (~<15) causes rough running at pwm harmonics. Too high a value (~>35) causes the RCT4215 630 to run rough on full throttle
    mov     A, Comm_Period4x_H          ; Set number of readings higher for lower speeds
    clr C
    rrc A
    jnz ($+3)
    inc A
    clr C
    rrc A
    jnz ($+3)
    inc A
    clr C
    rrc A
    jnz ($+3)
    inc A
    mov Temp1, A
    clr C                       
    subb    A, #5
    jc  ($+4)
    mov Temp1, #5
ENDIF   
    jnb Flags1.STARTUP_PHASE, comp_scale_samples

    mov Temp1, #27              ; Set many samples during startup, approximately one pwm period
    mov Temp2, #27

comp_scale_samples:
IF MCU_48MHZ == 1
    clr C
    mov A, Temp1
    rlc A
    mov Temp1, A
    clr C
    mov A, Temp2
    rlc A
    mov Temp2, A
ENDIF
    
comp_check_timeout:
    jb  WAIT_ACTIVE, comp_check_timeout_not_timed_out       ; Has zero cross scan timeout elapsed?

    mov A, Comparator_Read_Cnt          ; Check that comparator has been read
    jz  comp_check_timeout_not_timed_out    ; If not read - branch

    jnb Flags1.STARTUP_PHASE, comp_check_timeout_timeout_extended   ; Extend timeout during startup

    djnz    Startup_Zc_Timeout_Cntd, comp_check_timeout_extend_timeout

comp_check_timeout_timeout_extended:
    setb    Flags0.COMP_TIMED_OUT
    ajmp    setup_comm_wait

comp_check_timeout_extend_timeout:
    call    setup_zc_scan_timeout
comp_check_timeout_not_timed_out:
    inc Comparator_Read_Cnt         ; Increment comparator read count
    Read_Comp_Out                   ; Read comparator output
    anl A, #40h
    cjne    A, Bit_Access, comp_read_wrong

IF DEBUG_COMP && DEBUG_COMP_DETAILED
    setb P2.DebugPin
ENDIF
    ajmp    comp_read_ok
    
comp_read_wrong:
IF DEBUG_COMP && DEBUG_COMP_DETAILED
    clr P2.DebugPin
ENDIF
    jnb Flags1.STARTUP_PHASE, comp_read_wrong_not_startup

    inc Temp1                   ; Increment number of OK readings required
    clr C
    mov A, Temp1
    subb    A, Temp2                    ; If above initial requirement - do not increment further
    jc  ($+3)
    dec Temp1

    ajmp    comp_check_timeout          ; Continue to look for good ones

comp_read_wrong_not_startup:
    jb  Flags0.DEMAG_DETECTED, comp_read_wrong_extend_timeout

    inc Temp1                   ; Increment number of OK readings required
    clr C
    mov A, Temp1
    subb    A, Temp2                    
    jc  ($+5)
    ljmp    wait_for_comp_out_start     ; If above initial requirement - go back and restart

    ajmp    comp_check_timeout          ; Otherwise - take another reading

comp_read_wrong_extend_timeout:
    clr Flags0.DEMAG_DETECTED       ; Clear demag detected flag
    anl EIE1, #7Fh              ; Disable timer 3 interrupts
    mov TMR3CN0, #00h               ; Timer 3 disabled and interrupt flag cleared
    jnb Flags1.HIGH_RPM, comp_read_wrong_low_rpm    ; Branch if not high rpm

    mov TMR3L, #00h             ; Set timeout to ~1ms
IF MCU_48MHZ == 1
	mov	TMR3H, #0F0h
ELSE
	mov	TMR3H, #0F8h
ENDIF
comp_read_wrong_timeout_set:
    mov TMR3CN0, #04h               ; Timer 3 enabled and interrupt flag cleared
    setb    WAIT_ACTIVE
    orl EIE1, #80h              ; Enable timer 3 interrupts
    ljmp    wait_for_comp_out_start     ; If comparator output is not correct - go back and restart

comp_read_wrong_low_rpm:
    mov A, Comm_Period4x_H          ; Set timeout to ~4x comm period 4x value
    mov Temp7, #0FFh                ; Default to long
IF MCU_48MHZ == 1
    clr C
    rlc A
    jc  comp_read_wrong_load_timeout
ENDIF
    
    clr C
    rlc A
    jc  comp_read_wrong_load_timeout

    clr C
    rlc A
    jc  comp_read_wrong_load_timeout

    mov Temp7, A

comp_read_wrong_load_timeout:
    clr C
    clr A
    subb    A, Temp7
    mov TMR3L, #0
    mov TMR3H, A
    ajmp    comp_read_wrong_timeout_set

comp_read_ok:
    clr C
    mov A, Startup_Cnt              ; Force a timeout for the first commutation
    subb    A, #1
    jnc ($+5)
    ljmp    wait_for_comp_out_start

    jnb Flags0.DEMAG_DETECTED, ($+6)    ; Do not accept correct comparator output if it is demag
    ljmp    wait_for_comp_out_start

    djnz    Temp1, comp_read_ok_jmp     ; Decrement readings counter - repeat comparator reading if not zero
    ajmp    ($+4)

comp_read_ok_jmp:
    ajmp    comp_check_timeout

    clr Flags0.COMP_TIMED_OUT


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Setup commutation timing routine
;
; No assumptions
;
; Sets up and starts wait from zc to commutation and from commutation to end of adv start
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
setup_comm_wait:
IF DEBUG_COMP    
    clr P2.DebugPin
ENDIF
    clr IE_EA
    anl EIE1, #7Fh      ; Disable timer 3 interrupts
    mov TMR3CN0, #00h       ; Timer 3 disabled and interrupt flag cleared
    mov TMR3L, Wt_Comm_Start_L
    mov TMR3H, Wt_Comm_Start_H
    mov TMR3CN0, #04h       ; Timer 3 enabled and interrupt flag cleared

setup_comm_wait_normal: 
    mov TMR3RLL, Wt_Adv_Start_L
    mov TMR3RLH, Wt_Adv_Start_H

setup_comm_wait_normal_continue:    
    setb    WAIT_ACTIVE
    orl EIE1, #80h      ; Enable timer 3 interrupts
    setb    IE_EA           ; Enable interrupts again


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Evaluate comparator integrity
;
; No assumptions
;
; Checks comparator signal behaviour versus expected behaviour
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
evaluate_comparator_integrity:
    mov A, Flags1
    anl A, #((1 SHL STARTUP_PHASE)+(1 SHL INITIAL_RUN_PHASE))
    jz  eval_comp_check_timeout

    jb  Flags1.INITIAL_RUN_PHASE, ($+5) ; Do not increment beyond startup phase
    inc Startup_Cnt                 ; Increment counter
    jmp eval_comp_exit

eval_comp_check_timeout:
    jnb Flags0.COMP_TIMED_OUT, eval_comp_exit   ; Has timeout elapsed?
    jb  Flags1.DIR_CHANGE_BRAKE, eval_comp_exit ; Do not exit run mode if it is braking
    jb  Flags0.DEMAG_DETECTED, eval_comp_exit   ; Do not exit run mode if it is a demag situation
    dec SP                              ; Routine exit without "ret" command
    dec SP
    ljmp    run_to_wait_for_power_on_fail           ; Yes - exit run mode

eval_comp_exit:
    ret



;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Wait for commutation routine
;
; No assumptions
;
; Waits from zero cross to commutation 
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
wait_for_comm: 
    ; Update demag metric
    mov Temp1, #0
    jnb Flags0.DEMAG_DETECTED, ($+5)

    mov Temp1, #1

    mov A, Demag_Detected_Metric    ; Sliding average of 8, 256 when demag and 0 when not. Limited to minimum 120
    mov B, #7
    mul AB                  ; Multiply by 7
    mov Temp2, A
    mov A, B                    ; Add new value for current demag status
    add A, Temp1                
    mov B, A
    mov A, Temp2
    mov C, B.0              ; Divide by 8
    rrc A                   
    mov C, B.1
    rrc A
    mov C, B.2
    rrc A
    mov Demag_Detected_Metric, A
    clr C
    subb    A, #120             ; Limit to minimum 120
    jnc ($+5)

    mov Demag_Detected_Metric, #120

    clr C
    mov A, Demag_Detected_Metric    ; Check demag metric
    subb    A, Demag_Pwr_Off_Thresh
    jc  wait_for_comm_wait      ; Cut power if many consecutive demags. This will help retain sync during hard accelerations

    All_pwmFETs_off
    Set_Pwms_Off

wait_for_comm_wait:
    Wait_Pending

//  jnb WAIT_ACTIVE, ($+5)          
//  ajmp    wait_for_comm_wait

    ; Setup next wait time
    mov TMR3RLL, Wt_Zc_Scan_Start_L
    mov TMR3RLH, Wt_Zc_Scan_Start_H
    setb    WAIT_ACTIVE
    orl EIE1, #80h          ; Enable timer 3 interrupts
    ret


setup_timing_zc_scan_after_comm:
    clr IE_EA
    anl EIE1, #7Fh          ; Disable timer 3 interrupts
    mov TMR3CN0, #00h           ; Timer 3 disabled and interrupt flag cleared

    mov TMR3L, Wt_Adv_Start_L
    mov TMR3H, Wt_Adv_Start_H
    mov TMR3RLL, Wt_Zc_Scan_Start_L
    mov TMR3RLH, Wt_Zc_Scan_Start_H
    mov TMR3CN0, #04h       ; Timer 3 enabled and interrupt flag cleared

    setb    WAIT_ACTIVE
    orl EIE1, #80h      ; Enable timer 3 interrupts
    setb    IE_EA           ; Enable interrupts again
ret    

;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Commutation routines
;
; No assumptions
;
; Performs commutation switching 
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
; Comm phase 1 to comm phase 2
comm1comm2: 
    Set_RPM_Out
    jb  Flags3.PGM_DIR_REV, comm12_rev

    clr     IE_EA               ; Disable all interrupts
    BcomFET_off                 ; Turn off comfet
    AcomFET_on              ; Turn on comfet
    Set_Pwm_C                   ; To reapply power after a demag cut
    setb    IE_EA
    Set_Comp_Phase_B            ; Set comparator phase
    jmp comm_exit

comm12_rev: 
    clr     IE_EA               ; Disable all interrupts
    BcomFET_off                 ; Turn off comfet
    CcomFET_on              ; Turn on comfet (reverse)
    Set_Pwm_A                   ; To reapply power after a demag cut
    setb    IE_EA
    Set_Comp_Phase_B            ; Set comparator phase
    jmp comm_exit


; Comm phase 2 to comm phase 3
comm2comm3: 
    Clear_RPM_Out
    jb  Flags3.PGM_DIR_REV, comm23_rev

    clr     IE_EA               ; Disable all interrupts
    CpwmFET_off             ; Turn off pwmfet
    Set_Pwm_B                   ; To reapply power after a demag cut
    AcomFET_on
    setb    IE_EA
    Set_Comp_Phase_C            ; Set comparator phase
    ajmp    comm_exit

comm23_rev:
    clr     IE_EA               ; Disable all interrupts
    ApwmFET_off             ; Turn off pwmfet (reverse)
    Set_Pwm_B                   ; To reapply power after a demag cut
    CcomFET_on
    setb    IE_EA
    Set_Comp_Phase_A            ; Set comparator phase (reverse)
    ajmp    comm_exit


; Comm phase 3 to comm phase 4
comm3comm4: 
    Set_RPM_Out
    jb  Flags3.PGM_DIR_REV, comm34_rev

    clr     IE_EA               ; Disable all interrupts
    AcomFET_off                 ; Turn off comfet
    CcomFET_on              ; Turn on comfet
    Set_Pwm_B                   ; To reapply power after a demag cut
    setb    IE_EA
    Set_Comp_Phase_A            ; Set comparator phase
    jmp comm_exit

comm34_rev: 
    clr     IE_EA               ; Disable all interrupts
    CcomFET_off                 ; Turn off comfet (reverse)
    AcomFET_on              ; Turn on comfet (reverse)
    Set_Pwm_B                   ; To reapply power after a demag cut
    setb    IE_EA
    Set_Comp_Phase_C            ; Set comparator phase (reverse)
    jmp comm_exit


; Comm phase 4 to comm phase 5
comm4comm5: 
    Clear_RPM_Out
    jb  Flags3.PGM_DIR_REV, comm45_rev

    clr     IE_EA               ; Disable all interrupts
    BpwmFET_off             ; Turn off pwmfet
    Set_Pwm_A                   ; To reapply power after a demag cut
    CcomFET_on
    setb    IE_EA
    Set_Comp_Phase_B            ; Set comparator phase
    jmp comm_exit

comm45_rev:
    clr     IE_EA               ; Disable all interrupts
    BpwmFET_off             ; Turn off pwmfet
    Set_Pwm_C
    AcomFET_on              ; To reapply power after a demag cut
    setb    IE_EA
    Set_Comp_Phase_B            ; Set comparator phase
    jmp comm_exit


; Comm phase 5 to comm phase 6
comm5comm6: 
    Set_RPM_Out
    jb  Flags3.PGM_DIR_REV, comm56_rev

    clr     IE_EA               ; Disable all interrupts
    CcomFET_off                 ; Turn off comfet
    BcomFET_on              ; Turn on comfet
    Set_Pwm_A                   ; To reapply power after a demag cut
    setb    IE_EA
    Set_Comp_Phase_C            ; Set comparator phase
    jmp comm_exit

comm56_rev:
    clr     IE_EA               ; Disable all interrupts
    AcomFET_off                 ; Turn off comfet (reverse)
    BcomFET_on              ; Turn on comfet
    Set_Pwm_C                   ; To reapply power after a demag cut
    setb    IE_EA
    Set_Comp_Phase_A            ; Set comparator phase (reverse)
    jmp comm_exit


; Comm phase 6 to comm phase 1
comm6comm1: 
    Clear_RPM_Out
    jb  Flags3.PGM_DIR_REV, comm61_rev

    clr     IE_EA               ; Disable all interrupts
    ApwmFET_off             ; Turn off pwmfet
    Set_Pwm_C
    BcomFET_on              ; To reapply power after a demag cut
    setb    IE_EA
    Set_Comp_Phase_A            ; Set comparator phase
    jmp comm_exit

comm61_rev:
    clr     IE_EA               ; Disable all interrupts
    CpwmFET_off             ; Turn off pwmfet (reverse)
    Set_Pwm_A
    BcomFET_on              ; To reapply power after a demag cut
    setb    IE_EA
    Set_Comp_Phase_C            ; Set comparator phase (reverse)

comm_exit:
    ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Beeper routines (4 different entry points) 
;
; No assumptions
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
beep_f1:    ; Entry point 1, load beeper frequency 1 settings
    mov Temp3, #28  ; Off wait loop length
    mov Temp4, #60  ; Number of beep pulses
    jmp beep

beep_f2:    ; Entry point 2, load beeper frequency 2 settings
    mov Temp3, #20
    mov Temp4, #75
    jmp beep

beep_f3:    ; Entry point 3, load beeper frequency 3 settings
    mov Temp3, #16
    mov Temp4, #90
    jmp beep

beep_f4:    ; Entry point 4, load beeper frequency 4 settings
    mov Temp3, #10
    mov Temp4, #120
    jmp beep

beep:   ; Beep loop start
    mov A, Beep_Strength
    djnz    ACC, beep_start
    ret

beep_start:
IF MCU_48MHZ == 1
    Set_MCU_Clk_24MHz
ENDIF    
    mov Temp2, #2
beep_onoff:
    clr A
    BcomFET_off     ; BcomFET off
    djnz    ACC, $      ; Allow some time after comfet is turned off
    BpwmFET_on      ; BpwmFET on (in order to charge the driver of the BcomFET)
    djnz    ACC, $      ; Let the pwmfet be turned on a while
    BpwmFET_off     ; BpwmFET off again
    djnz    ACC, $      ; Allow some time after pwmfet is turned off
    BcomFET_on      ; BcomFET on
    djnz    ACC, $      ; Allow some time after comfet is turned on
    ; Turn on pwmfet
    mov A, Temp2
    jb  ACC.0, beep_apwmfet_on
    ApwmFET_on      ; ApwmFET on
beep_apwmfet_on:
    jnb ACC.0, beep_cpwmfet_on
    CpwmFET_on      ; CpwmFET on
beep_cpwmfet_on:
    mov A, Beep_Strength
    djnz    ACC, $      
    ; Turn off pwmfet
    mov A, Temp2
    jb  ACC.0, beep_apwmfet_off
    ApwmFET_off     ; ApwmFET off
beep_apwmfet_off:
    jnb ACC.0, beep_cpwmfet_off
    CpwmFET_off     ; CpwmFET off
beep_cpwmfet_off:
    mov A, #150     ; 25�s off
    djnz    ACC, $      
    djnz    Temp2, beep_onoff
    ; Copy variable
    mov A, Temp3
    mov Temp1, A    
beep_off:       ; Fets off loop
    djnz    ACC, $
    djnz    Temp1,  beep_off
    djnz    Temp4,  beep
    BcomFET_off     ; BcomFET off   
IF MCU_48MHZ == 1
    Set_MCU_Clk_48MHz
ENDIF
    ret


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Switch power off routine
;
; No assumptions
;
; Switches all fets off 
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
switch_power_off:
    All_pwmFETs_Off     ; Turn off all pwm fets
    All_comFETs_Off     ; Turn off all commutation fets
    Set_Pwms_Off
    ret         


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Set default parameters
;
; No assumptions
;
; Sets default programming parameters
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
set_default_parameters:
    mov Temp1, #_Pgm_Gov_P_Gain
    mov @Temp1, #0FFh   ; Governor P gain
    inc Temp1
    mov @Temp1, #0FFh   ; Governor I gain
    inc Temp1
    mov @Temp1, #0FFh   ; Governor mode
    inc Temp1
    mov @Temp1, #0FFh   ; Low voltage limit
    inc Temp1
    mov @Temp1, #0FFh   ; Multi gain
    inc Temp1
    mov @Temp1, #0FFh   
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_STARTUP_PWR
    inc Temp1
    mov @Temp1, #0FFh   ; Pwm freq
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_DIRECTION

    mov Temp1, #Pgm_Enable_TX_Program
    mov @Temp1, #DEFAULT_PGM_ENABLE_TX_PROGRAM
    inc Temp1
    mov @Temp1, #0FFh   ; Main rearm start
    inc Temp1
    mov @Temp1, #0FFh   ; Governor setup target
    inc Temp1
    mov @Temp1, #0FFh   ; Startup rpm   
    inc Temp1
    mov @Temp1, #0FFh   ; Startup accel
    inc Temp1
    mov @Temp1, #0FFh   ; Voltage comp
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_COMM_TIMING
    inc Temp1
    mov @Temp1, #0FFh   ; Damping force 
    inc Temp1
    mov @Temp1, #0FFh   ; Governor range
    inc Temp1
    mov @Temp1, #0FFh   ; Startup method    
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_MIN_THROTTLE
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_MAX_THROTTLE
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_BEEP_STRENGTH
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_BEACON_STRENGTH
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_BEACON_DELAY
    inc Temp1
    mov @Temp1, #0FFh   ; Throttle rate 
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_DEMAG_COMP
    inc Temp1
    mov @Temp1, #0FFh   ; Bec voltage high
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_CENTER_THROTTLE
    inc Temp1
    mov @Temp1, #0FFh   
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_ENABLE_TEMP_PROT
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_ENABLE_POWER_PROT
    inc Temp1
    mov @Temp1, #0FFh   ; Enable pwm input
    inc Temp1
    mov @Temp1, #0FFh   ; Pwm dither
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_BRAKE_ON_STOP
    inc Temp1
    mov @Temp1, #DEFAULT_PGM_LED_CONTROL
    ret



;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Decode settings
;
; No assumptions
;
; Decodes various settings
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
decode_settings:
    ; Load programmed direction
    mov Temp1, #Pgm_Direction   
    mov A, @Temp1               
    clr C
    subb    A, #3
    setb    Flags3.PGM_BIDIR
    jnc ($+4)

    clr Flags3.PGM_BIDIR

    clr Flags3.PGM_DIR_REV
    mov A, @Temp1               
    jnb ACC.1, ($+5)
    setb    Flags3.PGM_DIR_REV
    mov C, Flags3.PGM_DIR_REV
    mov Flags3.PGM_BIDIR_REV, C
    ; Decode startup power
    mov Temp1, #Pgm_Startup_Pwr     
    mov A, @Temp1               
    dec A   
    mov DPTR, #STARTUP_POWER_TABLE
    movc A, @A+DPTR 
    mov Temp1, #Pgm_Startup_Pwr_Decoded
    mov @Temp1, A   
    ; Decode low rpm power slope
    mov Temp1, #Pgm_Startup_Pwr
    mov A, @Temp1
    mov Low_Rpm_Pwr_Slope, A
    clr C
    subb    A, #2
    jnc ($+5)
    mov Low_Rpm_Pwr_Slope, #2
    ; Decode demag compensation
    mov Temp1, #Pgm_Demag_Comp      
    mov A, @Temp1               
    mov Demag_Pwr_Off_Thresh, #255  ; Set default

    cjne    A, #2, decode_demag_high

    mov Demag_Pwr_Off_Thresh, #160  ; Settings for demag comp low

decode_demag_high:
    cjne    A, #3, decode_demag_done

    mov Demag_Pwr_Off_Thresh, #130  ; Settings for demag comp high

decode_demag_done:
    ; Decode temperature protection limit
    mov Temp1, #Pgm_Enable_Temp_Prot
    mov A, @Temp1
    mov Temp1, A
    jz  decode_temp_done

    mov A, #(TEMP_LIMIT-TEMP_LIMIT_STEP)
decode_temp_step:
    add A, #TEMP_LIMIT_STEP
    djnz    Temp1, decode_temp_step

decode_temp_done:
    mov Temp_Prot_Limit, A
    ret



;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; LED control
;
; No assumptions
;
; Controls LEDs
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
led_control:
    mov Temp1, #Pgm_LED_Control
    mov A, @Temp1
    mov Temp2, A
    anl A, #03h
    Set_LED_0
    jnz led_0_done
    Clear_LED_0
led_0_done:
    mov A, Temp2
    anl A, #0Ch
    Set_LED_1
    jnz led_1_done
    Clear_LED_1
led_1_done:
    mov A, Temp2
    anl A, #030h
    Set_LED_2
    jnz led_2_done
    Clear_LED_2
led_2_done:
    mov A, Temp2
    anl A, #0C0h
    Set_LED_3
    jnz led_3_done
    Clear_LED_3
led_3_done:
    ret


set_full_comm_timeout:
    ;; divide by two assuming 48mhz
    mov Temp3, Comm_Period4x_L
    mov Temp4, Comm_Period4x_H

    mov A, Temp4
    clr C
    rrc A
    cpl A
    mov Temp4, A
    mov A, Temp3
    rrc A
    cpl A
    mov Temp3, A
    
    mov TMR3RLL, Temp3
    mov TMR3RLH, Temp4 
    setb    WAIT_ACTIVE
    orl EIE1, #80h      ; Enable timer 3 interrupts
ret    

set_adv_scan_timeouts:  
    clr IE_EA
    anl EIE1, #7Fh      ; Disable timer 3 interrupts
    mov TMR3CN0, #00h       ; Timer 3 disabled and interrupt flag cleared
    mov TMR3L, Wt_Adv_Start_L
    mov TMR3H, Wt_Adv_Start_H
    mov TMR3CN0, #04h       ; Timer 3 enabled and interrupt flag cleared
  
    mov TMR3RLL, Wt_Zc_Scan_Start_L
    mov TMR3RLH, Wt_Zc_Scan_Start_H

    setb    WAIT_ACTIVE
    orl EIE1, #80h      ; Enable timer 3 interrupts
    setb    IE_EA           ; Enable interrupts again
ret    




;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Main program start
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;**** **** **** **** **** **** **** **** **** **** **** **** ****

pgm_start:
    ; Initialize flash keys to invalid values
    mov Flash_Key_1, #0
    mov Flash_Key_2, #0
    ; Disable the WDT.
    mov WDTCN, #0DEh        ; Disable watchdog
    mov WDTCN, #0ADh        
    ; Initialize stack
    mov SP, #0c0h           ; Stack = 64 upper bytes of RAM
    ; Initialize VDD monitor
    orl VDM0CN, #080h       ; Enable the VDD monitor
    mov     RSTSRC, #06h    ; Set missing clock and VDD monitor as a reset source if not 1S capable
    ; Set clock frequency
    mov CLKSEL, #00h        ; Set clock divider to 1
    mov CLKSEL, #00h        ; Set clock divider to 1
    
    ; Switch power off
    call    switch_power_off
    ; Ports initialization
    mov P0, #P0_INIT
    mov P0MDIN, #P0_DIGITAL
    mov P0MDOUT, #P0_PUSHPULL
    mov P0, #P0_INIT
    mov P0SKIP, #P0_SKIP                
    mov P1, #P1_INIT
    mov P1MDIN, #P1_DIGITAL
    mov P1MDOUT, #P1_PUSHPULL
    mov P1, #P1_INIT
    mov P1SKIP, #P1_SKIP                
    mov P2, #0
    mov P2MDOUT, #P2_PUSHPULL
IF MCU_48MHZ == 1
    mov SFRPAGE, # 20h
    mov P2SKIP, #P2_PUSHPULL
    mov SFRPAGE, #0
ENDIF
    
    ; Initialize the XBAR and related functionality
    Initialize_Xbar
    ; Switch power off again, after initializing ports
    call    switch_power_off
    ; Clear RAM
    clr A               ; Clear accumulator
    mov Temp1, A            ; Clear Temp1
    clear_ram:  
    mov @Temp1, A           ; Clear RAM
    djnz Temp1, clear_ram   ; Is A not zero? - jump
    ; Set default programmed parameters
    call    set_default_parameters
    ; Read all programmed parameters
    call read_all_eeprom_parameters
    ; Set beep strength
    mov Temp1, #Pgm_Beep_Strength
    mov Beep_Strength, @Temp1
    ; Set initial arm variable
    mov Initial_Arm, #1
    mov Dither_Cur_Bit, #1

IF MCU_48MHZ == 1
    Set_MCU_Clk_48MHz
ENDIF    
    
    ; Initializing beep
    clr IE_EA           ; Disable interrupts explicitly
    call wait200ms  
    call beep_f1
    call wait15ms
    call wait15ms
    call wait15ms
    call beep_f2
    call wait15ms
    call wait15ms
    call wait15ms
    call beep_f3
    call wait15ms
    call wait15ms
    call wait15ms
    call beep_f4
    call beep_f4
    call wait100ms
    call    led_control


;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; No signal entry point
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
init_no_signal:
    ; Disable interrupts explicitly
    clr IE_EA
    ; Switch off timer 4 and switch rtx to input
    ; orl XBR2, #80h              ;disable pullups

IF MCU_48MHZ == 1
    mov SFRPAGE, #10h 
    anl EIE2, #(not 10h) ; Disable timer 4 interrupts
    mov SFRPAGE, #00h
ENDIF
    
    anl  RTX_MDOUT, #not( 1 shl RTX_PIN)    ; RTX is open drain
    setb RTX_PORT.RTX_PIN    ; set to 1 -> no driver, input
    
IF MCU_48MHZ == 1
    Set_MCU_Clk_24MHz
ENDIF
    
    ; Initialize flash keys to invalid values
    mov Flash_Key_1, #0
    mov Flash_Key_2, #0
    ; Check if input signal is high for more than 15ms
    mov Temp3, IT01CF
    mov IT01CF, #(80h+(RTX_PIN SHL 4)+(RTX_PIN))    ; Route RCP input to INT0/1, with INT1 inverted
    mov Temp1, #250
input_high_check_1:
    mov Temp2, #250
input_high_check_2:
    jnb RTX_PORT.RTX_PIN, bootloader_done   ; Look for low
    djnz    Temp2, input_high_check_2
    djnz    Temp1, input_high_check_1

    ljmp    1C00h           ; Jump to bootloader
    mov IT01CF, Temp3

bootloader_done:
    clr IE_EA
    ; Decode settings
    call    decode_settings 
    ; Set beep strength
    mov Temp1, #Pgm_Beep_Strength
    mov Beep_Strength, @Temp1
    ; Switch power off
    call    switch_power_off

IF MCU_48MHZ == 1
    ; Set clock frequency
    Set_MCU_Clk_48MHz
ENDIF
    

    ; Setup timers 2 and 3
    mov TMR2CN0, #04h       ; Timer 2 enabled
    mov TMR3CN0, #04h       ; Timer 3 enabled
    
    clr WAIT_ACTIVE
    
    Initialize_PCA          ; Initialize PCA
    Set_Pwm_Polarity        ; Set pwm polarity
    Enable_Power_Pwm_Module ; Enable power pwm module
    Enable_Damp_Pwm_Module  ; Enable damping pwm module
    ; Enable interrupts
    mov IE, #21h            ; Enable timer 2 interrupts and INT0 interrupts

    //  mov EIE1, #90h      ; Enable timer 3 and PCA0 interrupts
    mov EIE1, #10h      ; PCA0  interrupts
    mov IP, #01h            ; High priority to INT0 interrupts
;    mov EIP1, #80h            ; High priority to Timer 3 interrupts
;    mov EIP1, #10h           ; High prio for PCA0 interrupts
    
    ; Initialize comparator
    Initialize_Comparator   ; Initialize comparator
    ; Initialize ADC
    Initialize_Adc          ; Initialize ADC operation
    call    wait1ms
    mov DPTR, #0 ; some of the pgm calls modify it
    mov TCON, #0 ; timer 0 and 1 and external irq off
    mov Rtx_Mask, #(1 shl RTX_PIN)
    call init_plugin
    
                                ; Reset stall count
    mov Stall_Cnt, #0
    ; Initialize RC pulse
    clr Flags2.RCP_UPDATED          ; Clear updated flag
    mov     Dshot_Cmd, #0                   ; Clear Dshot command
    mov     Dshot_Cmd_Cnt, #0               ; Clear Dshot command count

                                ; Test whether signal is regular pwm

    ; Setup timers for DShot
    mov IT01CF, #(08h+(RTX_PIN SHL 4)+(RTX_PIN))    ; Route RCP input to INT0/1, with INT1 inverted
    setb Flags2.RCP_INVERTED

    mov TCON, #51h      ; Timer 1 run and INT0 edge triggered
    mov TMOD, #0AAh     ; Timer 0/1 set to 8bits auto reload and gated by INT0
    mov TH0, #0         ; Auto reload value zero
    mov TH1, #0
    ; Setup interrupts for DShot
    clr IE_ET0          ; Disable timer 0 interrupts
    setb    IE_ET1          ; Enable timer 1 interrupts
    setb    IE_EX1          ; Enable int1 interrupts
    setb    IE_EA           ; Enable all interrupts

//    call twait200ms

IF MCU_48MHZ == 1
IF MCU_48MHZ == 0
	mov	CKCON0, #01h		; Timer 0/1 clock is system clock divided by 4 (for DShot150)
	; Setup variables for DSshot150
IF MCU_48MHZ == 1
	mov	DShot_Timer_Preset, #128			; Load DShot sync timer preset (for DShot150)
ELSE
	mov	DShot_Timer_Preset, #192
ENDIF
	mov	DShot_Pwm_Thr, #20				; Load DShot qualification pwm threshold (for DShot150)
	mov	DShot_Frame_Length_Thr, #80		; Load DShot frame length criteria
	; Test whether signal is DShot150
	mov	Rcp_Outside_Range_Cnt, #10		; Set out of range counter
	call twait100ms						; Wait for new RC pulse
	mov	DShot_Pwm_Thr, #16				; Load DShot regular pwm threshold
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check if pulses were accepted
	subb	A, #10
	mov 	Dshot_Cmd, #0
	mov 	Dshot_Cmd_Cnt, #0
	jc	validate_rcp_start
ENDIF
    
    ; Setup variables for DShot300
    mov CKCON0, #0Ch                    ; Timer 0/1 clock is system clock (for DShot300)
IF MCU_48MHZ == 1
	mov	DShot_Timer_Preset, #0			; Load DShot sync timer preset (for DShot300)
ELSE
	mov	DShot_Timer_Preset, #128
ENDIF
    mov DShot_Pwm_Thr, #40              ; Load DShot qualification pwm threshold (for DShot300)
    mov DShot_Frame_Length_Thr, #40     ; Load DShot frame length criteria
    ; Test whether signal is DShot300
    mov Rcp_Outside_Range_Cnt, #10      ; Set out of range counter
    call twait100ms                     ; Wait for new RC pulse
    mov DShot_Pwm_Thr, #32              ; Load DShot regular pwm threshold
    clr C
    mov A, Rcp_Outside_Range_Cnt            ; Check if pulses were accepted
    subb    A, #10
    mov     Dshot_Cmd, #0
    mov     Dshot_Cmd_Cnt, #0
    jc  validate_rcp_start

IF MCU_48MHZ == 1
    ; Setup variables for DShot600
    mov CKCON0, #0Ch                    ; Timer 0/1 clock is system clock (for DShot600)
IF MCU_48MHZ == 1
    mov DShot_Timer_Preset, #128            ; Load DShot sync timer preset (for DShot600)
ELSE
    mov	DShot_Timer_Preset, #192
ENDIF    
    mov DShot_Pwm_Thr, #20              ; Load DShot qualification pwm threshold (for DShot600)
    mov DShot_Frame_Length_Thr, #20     ; Load DShot frame length criteria
    ; Test whether signal is DShot600
    mov Rcp_Outside_Range_Cnt, #10      ; Set out of range counter
    call twait100ms                     ; Wait for new RC pulse
    mov DShot_Pwm_Thr, #16              ; Load DShot regular pwm threshold
    clr C
    mov A, Rcp_Outside_Range_Cnt            ; Check if pulses were accepted
    subb    A, #10
    mov     Dshot_Cmd, #0
    mov     Dshot_Cmd_Cnt, #0
    jc  validate_rcp_start
ENDIF    
ENDIF
    
    ; Non inverted
    mov IT01CF, #(80h+(RTX_PIN SHL 4)+(RTX_PIN))    ; Route RCP input to INT0/1, with INT1 inverted
    clr Flags2.RCP_INVERTED

IF MCU_48MHZ == 0
	mov	CKCON0, #01h		; Timer 0/1 clock is system clock divided by 4 (for DShot150)
	; Setup variables for DSshot150
IF MCU_48MHZ == 1
	mov	DShot_Timer_Preset, #128			; Load DShot sync timer preset (for DShot150)
ELSE
	mov	DShot_Timer_Preset, #192
ENDIF
	mov	DShot_Pwm_Thr, #20				; Load DShot qualification pwm threshold (for DShot150)
	mov	DShot_Frame_Length_Thr, #80		; Load DShot frame length criteria
	; Test whether signal is DShot150
	mov	Rcp_Outside_Range_Cnt, #10		; Set out of range counter
	call twait100ms						; Wait for new RC pulse
	mov	DShot_Pwm_Thr, #16				; Load DShot regular pwm threshold
	clr	C
	mov	A, Rcp_Outside_Range_Cnt			; Check if pulses were accepted
	subb	A, #10
	mov 	Dshot_Cmd, #0
	mov 	Dshot_Cmd_Cnt, #0
	jc	validate_rcp_start
ENDIF

    ; Setup variables for DShot300
    mov CKCON0, #0Ch                    ; Timer 0/1 clock is system clock (for DShot300) 
IF MCU_48MHZ == 1
	mov	DShot_Timer_Preset, #0			; Load DShot sync timer preset (for DShot300)
ELSE
	mov	DShot_Timer_Preset, #128
ENDIF
    mov DShot_Pwm_Thr, #40              ; Load DShot qualification pwm threshold (for DShot300)
    mov DShot_Frame_Length_Thr, #40     ; Load DShot frame length criteria
    ; Test whether signal is DShot300
    mov Rcp_Outside_Range_Cnt, #10      ; Set out of range counter
    call twait100ms                     ; Wait for new RC pulse
    mov DShot_Pwm_Thr, #32              ; Load DShot regular pwm threshold
    clr C
    mov A, Rcp_Outside_Range_Cnt            ; Check if pulses were accepted
    subb    A, #10
    mov     Dshot_Cmd, #0
    mov     Dshot_Cmd_Cnt, #0
    jc  validate_rcp_start

IF MCU_48MHZ == 1
    ; Setup variables for DShot600
    mov CKCON0, #0Ch                    ; Timer 0/1 clock is system clock (for DShot600)
IF MCU_48MHZ == 1
    mov DShot_Timer_Preset, #128            ; Load DShot sync timer preset (for DShot600)
ELSE
    mov	DShot_Timer_Preset, #192
ENDIF    
    mov DShot_Pwm_Thr, #20              ; Load DShot qualification pwm threshold (for DShot600)
    mov DShot_Frame_Length_Thr, #20     ; Load DShot frame length criteria
    ; Test whether signal is DShot600
    mov Rcp_Outside_Range_Cnt, #10      ; Set out of range counter
    call twait100ms                     ; Wait for new RC pulse
    mov DShot_Pwm_Thr, #16              ; Load DShot regular pwm threshold
    clr C
    mov A, Rcp_Outside_Range_Cnt            ; Check if pulses were accepted
    subb    A, #10
    mov     Dshot_Cmd, #0
    mov     Dshot_Cmd_Cnt, #0
    jc  validate_rcp_start
ENDIF
    ljmp    init_no_signal

validate_rcp_start:
    call init_plugin
    ; Validate RC pulse
    call twait3ms                       ; Wait for new RC pulse
    jb  Flags2.RCP_UPDATED, ($+6)       ; Is there an updated RC pulse available - proceed
    ljmp    init_no_signal                  ; Go back to detect input signal

    ; Beep arm sequence start signal
    clr     IE_EA                       ; Disable all interrupts
    mov C, Flags2.RCP_INVERTED
    anl C, SERVICE_DETECTED
    jc rcp_valid_inverted_beep

    call beep_f4
    call beep_f4
    sjmp arming_start

rcp_valid_inverted_beep:    
    call beep_f3                        ; Signal that RC pulse is ready
    call beep_f3
    
    ; Arming sequence start
arming_start:
    setb    IE_EA                       ; Enable all interrupts
    call twait200ms 
    call twait100ms             ; Wait for new throttle value

    clr C
    mov A, New_Rcp          ; Load new RC pulse value
    subb    A, #1               ; Below stop?
    jc  arm_end_beep            ; Yes - proceed

    jmp arming_start            ; No - start over

arm_end_beep:
    clr     IE_EA               ; Disable all interrupts
    mov C, Flags2.RCP_INVERTED
    anl C, SERVICE_DETECTED
    jc arm_inverted_beep
    call beep_f1                ; Signal that rcpulse is ready
    setb IE_EA
    call twait100ms
    clr IE_EA
    call beep_f1
    setb IE_EA
    call twait100ms
    clr IE_EA
    call beep_f1
    setb IE_EA
    call twait100ms
    clr IE_EA
    sjmp arm_beep_done
    
arm_inverted_beep:  
    call beep_f4                ; Signal that rcpulse is ready
    call beep_f4
    call beep_f4

arm_beep_done:  
    setb    IE_EA               ; Enable all interrupts
    call twait200ms
    
    ; Clear initial arm variable
    mov Initial_Arm, #0

    ; Armed and waiting for power on
wait_for_power_on:
    mov Comm_Period4x_L, #0h
    mov Comm_Period4x_H, #0h
    mov Comm_Period1x_L, #0h
    mov Comm_Period1x_H, #0h
    clr A
    mov Power_On_Wait_Cnt_L, A  ; Clear wait counter
    mov Power_On_Wait_Cnt_H, A  
    mov Bit_Access, #0
wait_for_power_on_loop:
    inc Bit_Access
    jnb Bit_Access.1, wait_for_power_on_no_beep
    mov Bit_Access, #0
    inc Power_On_Wait_Cnt_L     ; Increment low wait counter
    mov A, Power_On_Wait_Cnt_L
    cpl A
    jnz wait_for_power_on_no_beep; Counter wrapping (about 3 sec)
    inc Power_On_Wait_Cnt_H     ; Increment high wait counter

    mov Temp1, #Pgm_Beacon_Delay
    mov A, @Temp1
    mov Temp1, #25      ; Approximately 1 min
    dec A
    jz  beep_delay_set

    mov Temp1, #50      ; Approximately 2 min
    dec A
    jz  beep_delay_set

    mov Temp1, #125     ; Approximately 5 min
    dec A
    jz  beep_delay_set

    mov Temp1, #250     ; Approximately 10 min
    dec A
    jz  beep_delay_set

    mov Power_On_Wait_Cnt_H, #0     ; Reset counter for infinite delay

beep_delay_set:
    clr C
    mov A, Power_On_Wait_Cnt_H
    subb    A, Temp1                ; Check against chosen delay
    jc  wait_for_power_on_no_beep; Has delay elapsed?

    call    switch_power_off        ; Switch power off in case braking is set
    call    wait1ms
    dec Power_On_Wait_Cnt_H     ; Decrement high wait counter
    mov Power_On_Wait_Cnt_L, #0 ; Set low wait counter
    mov Temp1, #Pgm_Beacon_Strength
    mov Beep_Strength, @Temp1
    clr     IE_EA               ; Disable all interrupts
    call beep_f4                ; Signal that there is no signal
    setb    IE_EA               ; Enable all interrupts
    mov Temp1, #Pgm_Beep_Strength
    mov Beep_Strength, @Temp1
    call twait100ms             ; Wait for new RC pulse to be measured

wait_for_power_on_no_beep:
    call twait10ms
    mov A, Rcp_Timeout_Cntd         ; Load RC pulse timeout counter value
    jnz wait_for_power_on_not_missing   ; If it is not zero - proceed

    jmp init_no_signal              ; If pulses missing - go back to detect input signal

wait_for_power_on_not_missing:
    call twait1ms ; 
    jnb Flags2.RCP_UPDATED, wait_for_power_on_loop
    clr C
    mov A, New_Rcp          ; Load new RC pulse value
    subb    A, #1               ; Higher than stop
    jnc wait_for_power_on_nonzero   ; Yes - proceed

    clr C
    mov A, Dshot_Cmd
    subb    A, #1               ; 1 or higher
    jnc check_dshot_cmd     ; Check Dshot command

    ljmp    wait_for_power_on_loop  ; If not Dshot command - start over

wait_for_power_on_nonzero:
    lcall twait100ms            ; Wait to see if start pulse was only a glitch
    mov A, Rcp_Timeout_Cntd     ; Load RC pulse timeout counter value
    jnz ($+5)               ; If it is not zero - proceed
    ljmp    init_no_signal          ; If it is zero (pulses missing) - go back to detect input signal

    mov     Dshot_Cmd, #0
    mov     Dshot_Cmd_Cnt, #0
    ljmp init_start

check_dshot_cmd:
    clr C
    mov     A, Dshot_Cmd
    subb A, #1
    jnz     dshot_beep_2

    clr     IE_EA
    call    switch_power_off        ; Switch power off in case braking is set
    mov Temp1, #Pgm_Beacon_Strength
    mov Beep_Strength, @Temp1
    call beep_f1
    mov Temp1, #Pgm_Beep_Strength
    mov Beep_Strength, @Temp1
    setb    IE_EA   
    call twait100ms 
    jmp     clear_dshot_cmd

dshot_beep_2:   
    clr C
    mov     A, Dshot_Cmd
    subb A, #2
    jnz     dshot_beep_3

    clr     IE_EA
    call    switch_power_off        ; Switch power off in case braking is set
    mov Temp1, #Pgm_Beacon_Strength
    mov Beep_Strength, @Temp1
    call beep_f2
    mov Temp1, #Pgm_Beep_Strength
    mov Beep_Strength, @Temp1
    setb    IE_EA   
    call twait100ms 
    jmp     clear_dshot_cmd

dshot_beep_3:       
    clr C
    mov     A, Dshot_Cmd
    subb A, #3
    jnz     dshot_beep_4

    clr     IE_EA
    call    switch_power_off        ; Switch power off in case braking is set
    mov Temp1, #Pgm_Beacon_Strength
    mov Beep_Strength, @Temp1
    call beep_f3
    mov Temp1, #Pgm_Beep_Strength
    mov Beep_Strength, @Temp1
    setb    IE_EA   
    call twait100ms 
    jmp     clear_dshot_cmd

dshot_beep_4:
    clr C
    mov     A, Dshot_Cmd
    subb A, #4
    jnz     dshot_beep_5

    clr     IE_EA
    call    switch_power_off        ; Switch power off in case braking is set
    mov Temp1, #Pgm_Beacon_Strength
    mov Beep_Strength, @Temp1
    call beep_f4
    mov Temp1, #Pgm_Beep_Strength
    mov Beep_Strength, @Temp1
    setb    IE_EA   
    call twait100ms     
    jmp     clear_dshot_cmd

dshot_beep_5:
    clr C
    mov     A, Dshot_Cmd
    subb A, #5
    jnz     dshot_direction_1

    clr     IE_EA
    call    switch_power_off        ; Switch power off in case braking is set
    mov Temp1, #Pgm_Beacon_Strength
    mov Beep_Strength, @Temp1
    call beep_f4
    mov Temp1, #Pgm_Beep_Strength
    mov Beep_Strength, @Temp1
    setb    IE_EA   
    call twait100ms 
    jmp     clear_dshot_cmd

dshot_direction_1:
    clr C
    mov     A, Dshot_Cmd
    subb A, #7
    jnz     dshot_direction_2

    clr     C
    mov     A, Dshot_Cmd_Cnt
    subb A, #6                  ; Needs to receive it 6 times in a row
    jnc     ($+4)                   ; Same as "jc dont_clear_dshot_cmd"
    ljmp wait_for_power_on_not_missing

    mov A, #1
    jnb Flags3.PGM_BIDIR, ($+5)
    mov A, #3
    mov Temp1, #Pgm_Direction
    mov @Temp1, A
    clr     Flags3.PGM_DIR_REV
    clr     Flags3.PGM_BIDIR_REV
    jmp     clear_dshot_cmd

dshot_direction_2:
    clr C
    mov     A, Dshot_Cmd
    subb A, #8
    jnz     dshot_direction_bidir_off

    clr     C
    mov     A, Dshot_Cmd_Cnt
    subb A, #6                  ; Needs to receive it 6 times in a row
    jnc     ($+4)                   ; Same as "jc dont_clear_dshot_cmd"
    ljmp wait_for_power_on_not_missing

    mov A, #2
    jnb Flags3.PGM_BIDIR, ($+5)
    mov A, #4
    mov Temp1, #Pgm_Direction
    mov @Temp1, A
    setb Flags3.PGM_DIR_REV
    setb Flags3.PGM_BIDIR_REV
    jmp     clear_dshot_cmd

dshot_direction_bidir_off:
    clr C
    mov     A, Dshot_Cmd
    subb A, #9
    jnz     dshot_direction_bidir_on

    clr     C
    mov     A, Dshot_Cmd_Cnt
    subb A, #6                  ; Needs to receive it 6 times in a row
    jnc     ($+5)                   ; Same as "jc dont_clear_dshot_cmd"
    ljmp wait_for_power_on_not_missing

    jnb Flags3.PGM_BIDIR, dshot_direction_bidir_on

    clr C
    mov Temp1, #Pgm_Direction
    mov A, @Temp1
    subb    A, #2
    mov @Temp1, A
    clr     Flags3.PGM_BIDIR
    jmp     clear_dshot_cmd

dshot_direction_bidir_on:
    clr C
    mov     A, Dshot_Cmd
    subb A, #10
    jnz     dshot_direction_normal

    clr     C
    mov     A, Dshot_Cmd_Cnt
    subb A, #6                  ; Needs to receive it 6 times in a row
    jnc     ($+5)                   ; Same as "jc dont_clear_dshot_cmd"
    ljmp wait_for_power_on_not_missing

    jb  Flags3.PGM_BIDIR, dshot_direction_normal

    mov Temp1, #Pgm_Direction
    mov A, @Temp1
    add A, #2
    mov @Temp1, A
    setb    Flags3.PGM_BIDIR
    jmp     clear_dshot_cmd

dshot_direction_normal: 
    clr C
    mov     A, Dshot_Cmd
    subb A, #20
    jnz     dshot_direction_reverse

    clr     C
    mov     A, Dshot_Cmd_Cnt
    subb A, #6                  ; Needs to receive it 6 times in a row
    jnc     ($+5)                   ; Same as "jc dont_clear_dshot_cmd"
    ljmp wait_for_power_on_not_missing

    clr IE_EA                   ; DPTR used in interrupts
    mov DPTR, #Eep_Pgm_Direction        ; Read from flash
    mov A, #0
    movc    A, @A+DPTR
    setb    IE_EA
    mov Temp1, #Pgm_Direction
    mov @Temp1, A
    rrc A                       ; Lsb to carry
    clr     Flags3.PGM_DIR_REV
    clr     Flags3.PGM_BIDIR_REV
    jc  ($+4)
    setb    Flags3.PGM_DIR_REV
    jc  ($+4)
    setb    Flags3.PGM_BIDIR_REV
    jmp     clear_dshot_cmd

dshot_direction_reverse:            ; Temporary reverse
    clr C
    mov     A, Dshot_Cmd
    subb A, #21
    jnz     dshot_save_settings

    clr     C
    mov     A, Dshot_Cmd_Cnt
    subb A, #6                  ; Needs to receive it 6 times in a row
    jc  dont_clear_dshot_cmd
    
    clr IE_EA                   ; DPTR used in interrupts
    mov DPTR, #Eep_Pgm_Direction        ; Read from flash
    mov A, #0
    movc    A, @A+DPTR
    setb    IE_EA
    mov Temp1, A
    cjne    Temp1, #1, ($+5)
    mov A, #2
    cjne    Temp1, #2, ($+5)
    mov A, #1
    cjne    Temp1, #3, ($+5)
    mov A, #4
    cjne    Temp1, #4, ($+5)
    mov A, #3
    mov Temp1, #Pgm_Direction
    mov @Temp1, A
    rrc A                       ; Lsb to carry
    clr     Flags3.PGM_DIR_REV
    clr     Flags3.PGM_BIDIR_REV
    jc  ($+4)
    setb    Flags3.PGM_DIR_REV
    jc  ($+4)
    setb    Flags3.PGM_BIDIR_REV
    jmp     clear_dshot_cmd


dshot_save_settings:
    clr C
    mov     A, Dshot_Cmd
    subb A, #12
    jnz     clear_dshot_cmd

    mov Flash_Key_1, #0A5h          ; Initialize flash keys to valid values
    mov Flash_Key_2, #0F1h
    clr     C
    mov     A, Dshot_Cmd_Cnt
    subb A, #6                  ; Needs to receive it 6 times in a row
    jc  dont_clear_dshot_cmd

    call erase_and_store_all_in_eeprom
    setb    IE_EA
    
clear_dshot_cmd:
    mov     Dshot_Cmd_Cnt, #0
    mov     Dshot_Cmd, #0

dont_clear_dshot_cmd:
    mov Flash_Key_1, #0         ; Initialize flash keys to invalid values
    mov Flash_Key_2, #0
    clr Flags2.RCP_UPDATED
    jmp     wait_for_power_on_not_missing
    
;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Start entry point
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****
init_start:
    clr IE_EA
    call switch_power_off
    clr A
    setb    IE_EA
    clr A
    mov Adc_Conversion_Cnt, A
    mov Flags0, A               ; Clear flags0
    mov Flags1, A               ; Clear flags1
    mov Demag_Detected_Metric, A    ; Clear demag metric
    ;**** **** **** **** ****
    ; Motor start beginning
    ;**** **** **** **** **** 
    mov Adc_Conversion_Cnt, #8              ; Make sure a temp reading is done
    call twait1ms
    call start_adc_conversion
read_initial_temp:
    jnb ADC0CN0_ADINT, read_initial_temp
    Read_Adc_Result                     ; Read initial temperature
    mov A, Temp2
    jnz ($+3)                           ; Is reading below 256?

    mov Temp1, A                            ; Yes - set average temperature value to zero

    mov Current_Average_Temp, Temp1         ; Set initial average temperature
    call check_temp_voltage_and_limit_power
    mov Adc_Conversion_Cnt, #8              ; Make sure a temp reading is done next time
    ; Set up start operating conditions
    clr IE_EA               ; Disable interrupts
    call set_startup_pwm
    mov Pwm_Limit, Pwm_Limit_Beg
    mov Pwm_Limit_By_Rpm, Pwm_Limit_Beg
    setb    IE_EA
    ; Begin startup sequence

    jnb Flags3.PGM_BIDIR, init_start_bidir_done ; Check if bidirectional operation

    clr Flags3.PGM_DIR_REV          ; Set spinning direction. Default fwd
    jnb Flags2.RCP_DIR_REV, ($+5)   ; Check force direction
    setb    Flags3.PGM_DIR_REV          ; Set spinning direction

init_start_bidir_done:
    setb    Flags1.STARTUP_PHASE        ; Set startup phase flag
    mov Startup_Cnt, #0         ; Reset counter
    call comm5comm6             ; Initialize commutation
    call comm6comm1             
    call initialize_timing          ; Initialize timing
    call    calc_next_comm_timing       ; Set virtual commutation point
    call initialize_timing          ; Initialize timing
    call    calc_next_comm_timing       
    call    initialize_timing           ; Initialize timing



;**** **** **** **** **** **** **** **** **** **** **** **** ****
;
; Run entry point
;
;**** **** **** **** **** **** **** **** **** **** **** **** ****

run1:   
; Run 1 = B(p-on) + C(n-pwm) - comparator A evaluated
; Out_cA changes from low to high
    call wait_for_comp_out_high ; Wait for high
    call wait_for_comm          ; Wait from zero cross to commutation
    call comm1comm2         ; Commutate
    call calc_next_comm_timing  ; Calculate next timing and wait advance timing wait

; Run 2 = A(p-on) + C(n-pwm) - comparator B evaluated
; Out_cB changes from high to low

run2:   
    call wait_for_comp_out_low
    jb  Flags1.HIGH_RPM, ($+6)  ; Skip if high rpm
    lcall set_pwm_limit_low_rpm
    jnb Flags1.HIGH_RPM, ($+6)  ; Do if high rpm
    lcall set_pwm_limit_high_rpm
    call wait_for_comm
    call comm2comm3
    call calc_next_comm_timing

; Run 3 = A(p-on) + B(n-pwm) - comparator C evaluated
; Out_cC changes from low to high
run3:
    call wait_for_comp_out_high
    call wait_for_comm
    call comm3comm4
    call calc_next_comm_timing

; Run 4 = C(p-on) + B(n-pwm) - comparator A evaluated
; Out_cA changes from high to low
run4:
    call wait_for_comp_out_low
    call wait_for_comm
    call comm4comm5
    call calc_next_comm_timing

    
; Run 5 = C(p-on) + A(n-pwm) - comparator B evaluated
; Out_cB changes from low to high

run5:
    call wait_for_comp_out_high
    call wait_for_comm
    call comm5comm6
    call calc_next_comm_timing

; Run 6 = B(p-on) + A(n-pwm) - comparator C evaluated
; Out_cC changes from high to low
run6:
    call start_adc_conversion
    call wait_for_comp_out_low
    call wait_for_comm
    call comm6comm1
    call check_temp_voltage_and_limit_power
    call calc_next_comm_timing

    ; Check if it is direct startup
    jnb Flags1.STARTUP_PHASE, normal_run_checks

    ; Set spoolup power variables
    mov Pwm_Limit, Pwm_Limit_Beg        ; Set initial max power
    ; Check startup counter
    mov Temp2, #24              ; Set nominal startup parameters
    mov Temp3, #12
    clr C
    mov A, Startup_Cnt              ; Load counter
    subb    A, Temp2                    ; Is counter above requirement?
    jc  direct_start_check_rcp      ; No - proceed

    clr Flags1.STARTUP_PHASE        ; Clear startup phase flag
    setb    Flags1.INITIAL_RUN_PHASE        ; Set initial run phase flag
    mov Initial_Run_Rot_Cntd, Temp3 ; Set initial run rotation count
    mov Pwm_Limit, Pwm_Limit_Beg
    mov Pwm_Limit_By_Rpm, Pwm_Limit_Beg
    jmp normal_run_checks

direct_start_check_rcp:
    clr C
    mov A, New_Rcp              ; Load new pulse value
    subb    A, #1                   ; Check if pulse is below stop value
    jc  ($+5)

    ljmp    run1                        ; Continue to run 

    jmp run_to_wait_for_power_on


normal_run_checks:
    ; Check if it is initial run phase
    jnb Flags1.INITIAL_RUN_PHASE, initial_run_phase_done    ; If not initial run phase - branch
    jb  Flags1.DIR_CHANGE_BRAKE, initial_run_phase_done ; If a direction change - branch

    ; Decrement startup rotaton count
    mov A, Initial_Run_Rot_Cntd
    dec A
    ; Check number of initial rotations
    jnz     initial_run_check_startup_rot   ; Branch if counter is not zero

    clr Flags1.INITIAL_RUN_PHASE        ; Clear initial run phase flag
    setb    Flags1.MOTOR_STARTED        ; Set motor started
    jmp run1                        ; Continue with normal run

initial_run_check_startup_rot:
    mov Initial_Run_Rot_Cntd, A     ; Not zero - store counter

    jb  Flags3.PGM_BIDIR, initial_run_continue_run  ; Check if bidirectional operation

    clr C
    mov A, New_Rcp              ; Load new pulse value
    subb    A, #1                   ; Check if pulse is below stop value
    jc  ($+5)

initial_run_continue_run:
    ljmp    run1                        ; Continue to run 

    jmp run_to_wait_for_power_on

initial_run_phase_done:
    ; Reset stall count
    mov Stall_Cnt, #0
    ; Exit run loop after a given time
    jb  Flags3.PGM_BIDIR, run6_check_timeout    ; Check if bidirectional operation

    mov Temp1, #250
    mov Temp2, #Pgm_Brake_On_Stop
    mov A, @Temp2
    jz  ($+4)

    mov Temp1, #3                   ; About 100ms before stopping when brake is set

    clr C
    mov A, Rcp_Stop_Cnt         ; Load stop RC pulse counter low byte value
    subb    A, Temp1                    ; Is number of stop RC pulses above limit?
    jnc run_to_wait_for_power_on        ; Yes, go back to wait for poweron

run6_check_timeout:
    mov A, Rcp_Timeout_Cntd         ; Load RC pulse timeout counter value
    jz  run_to_wait_for_power_on        ; If it is zero - go back to wait for poweron

run6_check_dir:
    jnb Flags3.PGM_BIDIR, run6_check_speed      ; Check if bidirectional operation

    jb  Flags3.PGM_DIR_REV, run6_check_dir_rev      ; Check if actual rotation direction
    jb  Flags2.RCP_DIR_REV, run6_check_dir_change   ; Matches force direction
    jmp run6_check_speed

run6_check_dir_rev:
    jnb Flags2.RCP_DIR_REV, run6_check_dir_change
    jmp run6_check_speed

run6_check_dir_change:
    jb  Flags1.DIR_CHANGE_BRAKE, run6_check_speed

    setb    Flags1.DIR_CHANGE_BRAKE     ; Set brake flag
    mov Pwm_Limit, Pwm_Limit_Beg        ; Set max power while braking
    jmp run4                        ; Go back to run 4, thereby changing force direction

run6_check_speed:
    mov Temp1, #0F0h                ; Default minimum speed
    jnb Flags1.DIR_CHANGE_BRAKE, run6_brake_done; Is it a direction change?

    mov Pwm_Limit, Pwm_Limit_Beg    ; Set max power while braking
    mov Temp1, #20h                 ; Bidirectional braking termination speed

run6_brake_done:
    clr C
    mov A, Comm_Period4x_H          ; Is Comm_Period4x more than 32ms (~1220 eRPM)?
    subb    A, Temp1
    jnc ($+5)                   ; Yes - stop or turn direction 
    ljmp    run1                        ; No - go back to run 1

    jnb Flags1.DIR_CHANGE_BRAKE, run_to_wait_for_power_on   ; If it is not a direction change - stop

    clr Flags1.DIR_CHANGE_BRAKE     ; Clear brake flag
    clr Flags3.PGM_DIR_REV          ; Set spinning direction. Default fwd
    jnb Flags2.RCP_DIR_REV, ($+5)   ; Check force direction
    setb    Flags3.PGM_DIR_REV          ; Set spinning direction
    setb    Flags1.INITIAL_RUN_PHASE
    mov Initial_Run_Rot_Cntd, #18
    mov Pwm_Limit, Pwm_Limit_Beg        ; Set initial max power
    jmp run1                        ; Go back to run 1 

run_to_wait_for_power_on_fail:  
    inc Stall_Cnt                   ; Increment stall count
    mov A, New_Rcp              ; Check if RCP is zero, then it is a normal stop            
    jz  run_to_wait_for_power_on
    ajmp    run_to_wait_for_power_on_stall_done

run_to_wait_for_power_on:   
    mov Stall_Cnt, #0

run_to_wait_for_power_on_stall_done:
    clr IE_EA
    call switch_power_off
    mov Flags1, #0              ; Clear flags1

    setb IE_EA
    call    twait100ms                   ; Wait for pwm to be stopped
    call switch_power_off
    mov Flags0, #0              ; Clear flags0
    mov Temp1, #Pgm_Brake_On_Stop
    mov A, @Temp1
    jz  run_to_wait_for_power_on_brake_done

    AcomFET_on
    BcomFET_on
    CcomFET_on

run_to_wait_for_power_on_brake_done:
    clr C
    mov A, Stall_Cnt
    subb    A, #4
    jc  jmp_wait_for_power_on
    jmp init_no_signal

jmp_wait_for_power_on:
    jmp wait_for_power_on           ; Go back to wait for power on

;**** **** **** **** **** **** **** **** **** **** **** **** ****

$include (JEscPgm.inc)                ; Include source code for programming the ESC
$include (JEscBootLoad.inc)           ; Include source code for bootloader

;**** **** **** **** **** **** **** **** **** **** **** **** ****

CSEG AT 19E0h
check_bootloader:
	mov	WDTCN, #0DEh			; Disable watchdog
	mov	WDTCN, #0ADh		
    orl RSTSRC, #02h

    cjne A, #0a5h, check_bootloader_startup
check_bootloader_jump_reset:
    jmp reset
check_bootloader_startup:
    mov dptr, #0fbfdh
    clr a
    movc a, @a+dptr
    cjne a, #0a5h, check_bootloader_jump_reset
    jmp 0fbc0h
    

CSEG AT 19FDh                   ;
reset:
ljmp    pgm_start
    
END

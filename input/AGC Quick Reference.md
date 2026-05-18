# Standard Instructions

- Adr(12) is a 12 bit address in either fixed or erasable memory
- EAdr(10) is an erasable memory address with 10 bits
- EAdr(12) is an erasable memory address with 12 bits
- FAdr(12) is a fixed memory address with 12 bits
- Chan is a channel number with 9 bits

| Mnemonic | Arguments | Description                     | Comment                          |
| -------- | --------- | ------------------------------- | -------------------------------- |
|          |           | **Sequence Changing**           |                                  |
| TC       | Adr(12)   | Transfer Control                |                                  |
| TCF      | FAdr(12)  | Transfer Control to Fixed       |                                  |
| CCS      | EAdr(10)  | Count Compare and Skip          |                                  |
| BZF      | FAdr(12)  | Branch Zero to Fixed            |                                  |
| BZMF     | FAdr(12)  | Branch Zero or Minus to Fixed   |                                  |
|          |           | **Reading and Writing**         |                                  |
| CA       | Adr(12)   | Clear and Add                   |                                  |
| CS       | Adr(12)   | Clear and Subtract              |                                  |
| DCA      | Adr(12)   | Double Clear and Add            |                                  |
| DCS      | Adr(12)   | Double Clear and Subtract       |                                  |
| TS       | EAdr(10)  | Transfer to Storage             | Skips next instr. if A overflows |
| XCH      | EAdr(10)  | Exchange A and K                |                                  |
| LXCH     | EAdr(10)  | Exchange L and K                |                                  |
| QXCH     | EAdr(10)  | Exchange Q and K                |                                  |
| DXCH     | EAdr(10)  | Double Exchange                 |                                  |
|          |           | **Instruction Modification**    |                                  |
| INDEX    | EAdr(10)  | Index next Instruction          |                                  |
| INDEXE   | EAdr(12)  | Index next Instruction Extended |                                  |
|          |           | **Arithmetic and Logic**        |                                  |
| AD       | EAdr(12)  | Add                             |                                  |
| SU       | EAdr(10)  | Subtract                        |                                  |
| ADS      | EAdr(10)  | Add to Storage                  |                                  |
| MSU      | EAdr(10)  | Modular Subtract                |                                  |
| INCR     | EAdr(10)  | Increment                       |                                  |
| AUG      | EAdr(10)  | Augment                         |                                  |
| DIM      | EAdr(10)  | Diminish                        |                                  |
| DAS      | EAdr(10)  | Double Add to Storage           |                                  |
| MASK     | Adr(12)   | Mask A by K                     |                                  |
| MP       | Adr(12)   | Multiply                        |                                  |
| DV       | EAdr(10)  | Divide                          |                                  |
|          |           | **I/O Channel**                 |                                  |
| READ     | Chan      | Read KC                         |                                  |
| WRITE    | Chan      | Write Channel KC                |                                  |
| RAND     | Chan      | Read and Mask                   |                                  |
| WAND     | Chan      | Write and Mask                  |                                  |
| ROR      | Chan      | Read and Superimpose            |                                  |
| WOR      | Chan      | Write and Superimpose           |                                  |
| RXOR     | Chan      | Read and Invert                 |                                  |
|          |           | **Miscellaneous**               |                                  |
| RETURN   | none      | Return from Subroutine          |                                  |
| RELINT   | none      | Enable Interrupts               |                                  |
| INHINT   | none      | Inhibit Interrupts              |                                  |
| EXTEND   | none      | Set Extracode Flag              |                                  |
| EDRUPT   | none      | Ed Smally's Interrupt           |                                  |
| RESUME   | none      | Resume Interrupted Program      |                                  |
|          |           | **Involuntary**                 |                                  |
| RUPT     |           | Interrupt                       |                                  |
| PINC     | estor     | Pointer Increment               |                                  |
| MINC     | estor     | Pointer Decrement               |                                  |
| DINC     | estor     | Diminish absolute value by 1    |                                  |
| PCDU     | estor     | CDU Increment                   |                                  |
| MCDU     | estor     | CDU Decrement                   |                                  |
| SHINC    | estor     | Counter Shift                   |                                  |
| SHANC    | estor     | Counter Shift and Add 1         |                                  |

# Implied Address Instructions

| Assembled Instruction | Special code     | Description                                                         |
| --------------------- | ---------------- | ------------------------------------------------------------------- |
| COM                   | CS A             | Complement value of accumulator                                     |
| DCOM                  | DCS A            | Complement double precision value in accumulator and L register     |
| DDOUBL                | DAS A            | Double the double precision value in the accumulator and L register |
| DOUBLE                | AD A             | Double value in accumulator                                         |
| DTCB                  | DXCH Z and BBANK | Double Transfer Control Switching both Banks                        |
| DTCF                  | DXCH FB and Z    | Double Transfer Control Switchin the F Bank                         |
| NOOP                  | TCF I + 1        | No operation by branching to the next instruction in fixed storage  |
| NOOP                  | CA A             | No operation by clearing and reloading the accumulator              |
| OVSK                  | TS A             | Overflow Skip                                                       |
| RETURN                | TC Q             | Return to calling subrouting                                        |
| SQUARE                | MP A             | Square the value in the accumulator                                 |
| TCAA                  | TS Z             | Transfer Control to Address in A                                    |
| XLQ                   | TC L             | Execute using L and Q registers                                     |
| XXALQ                 | TC A             | Execute Extracode using A, L and Q registers                        |
| ZL                    | LXCH             | Zero the L register                                                 |
| ZQ                    | QXCH             | Zero the Q register                                                 |

# Pseudo Instructions

| Instruction name | Operand                 | Description                                                                |
| ---------------- | ----------------------- | -------------------------------------------------------------------------- |
| =                | Data                    | Link Data to the pseudo instruction's label                                |
| 1DNADR to 6DNADR | Label                   | Downlink data at Label                                                     |
| 2DEC             | Double precision number | Double precision constant                                                  |
| 2FCADR           | Double word constant    | For use by the DTCF operation.                                             |
| BANK             | BankNo (optional)       | Move the assembler LOC to the next free word in BankNo or the current bank |
| BNKSUM           | None                    | Print the number of words used in the current bank                         |
| CADR             | Address                 | Define an absolute fixed memory address                                    |
| COUNT, COUNT*    | BankNo/Label<br>$$      | Used to count memory usage.                                                |
| DEC              | Single precision number | Single precision constant                                                  |
| DNPTR            | Label                   | Downlink data pointed to by label                                          |
| EBANK=           | BankNo                  | Changes the erasable memory bank                                           |
| EQUALS           | Data                    | Similar to '='                                                             |
| ERASE            | Count(optional)         | Leaves one or more words empty in erasable memory, i.e. defines a variable |
| MEMORY           | Count(optional)         | Like ERASE                                                                 |
| OCT              | Octal number            | Octal constant                                                             |
| SBANK=           |                         | Indicate the use of the superbank                                          |
| SETLOC           | Address                 | Set the assembler LOC to the address                                       |
| SUBRO            | Name                    | Record a subroutine                                                        |

# Interrupt Vectors

| Interrupt name | Trigger Condition                      | Description                                                        |
| -------------- | -------------------------------------- | ------------------------------------------------------------------ |
| Startup        | AGC power on                           | Starting address after AGC power up                                |
| T6RUPT         | TIME6 decremented to 0                 | Timer for RCS jets, used by the digital autopilot                  |
| T5RUPT         | TIME5 timer overflow                   | Digital autopilot timer                                            |
| T4RUPT         | TIME4 timer overflow                   | DSKY monitoring and updating                                       |
| T3RUPT         | TIME3 timer overflow                   | WAITLIST task scheduler                                            |
| KEYRUPT1       | Keystroke received from DSKY           | Key code from main DSKY is available in channel 15                 |
| KEYRUPT2       | Keystroke received from secondary DSKY | Key code from Navigation DSKY is available in channel 16 (CM only) |
| UPRUPT         | Data ready in INLINK register          | Used for DSKY uplinks                                              |
| DOWNRUPT       | Downlink registers ready for more data | AGC downlink telemetry                                             |
| RADARUPT       | Data in RNRAD register is ready        | Data from rendezvous radar (LM only)                               |
| RUPT10         |                                        | LM P64 redesignations                                              |

# Registers

| Name                       | Address (oct) | Function                                                          |
| -------------------------- | ------------- | ----------------------------------------------------------------- |
| A                          | 00000         | Accumulator                                                       |
| L                          | 00001         | Low order product of the accumulator                              |
| Q                          | 00002         | TC instruction return address                                     |
| EBANK                      | 00003         | Erasable Memory Bank                                              |
| FBANK                      | 00004         | Fixed Memory Bank                                                 |
| Z                          | 00005         | 12 Bit Program Counter                                            |
| BBANK                      | 00006         | Both Bank Register                                                |
| ZERO                       | 00007         | Hardwired to value zero                                           |
| ARUPT                      | 00010         | Interrupted contents of A                                         |
| LRUPT                      | 00011         | Interrupted contents of L                                         |
| QRUPT                      | 00012         | Interrupted contents of Q                                         |
| SAMPTIME                   | 00013         | SAMPLED TIME 1                                                    |
| SAMPTIME                   | 00014         | SAMPLED TIME 2                                                    |
| ZRUPT                      | 00015         | Interrupted contents of Z (hardware)                              |
| BANKRUPT                   | 00016         | Interrupted contents of BBANK                                     |
| BRUPT                      | 00017         | Interrupted contents of B Internal Register                       |
| CYR                        | 00020         | Cycle Right                                                       |
| SR                         | 00021         | Shift Right                                                       |
| CYL                        | 00022         | Cycle Left                                                        |
| EDOP                       | 00023         | Edits Interpretive Operation Code Pairs                           |
| TIME2                      | 00024         | T2: Elapsed time                                                  |
| TIME1                      | 00025         | T1: Elapsed time                                                  |
| TIME3                      | 00026         | T3RUPT: Wait list                                                 |
| TIME4                      | 00027         | T4RUPT                                                            |
| TIME5                      | 00030         | T5RUPT: Digital Autopilot                                         |
| TIME6                      | 00031         | T6RUPT: Fine scale clocking                                       |
| CDUX                       | 00032         | Inner IMU Gimbal                                                  |
| CDUY                       | 00033         | Middle IMU Gimbal                                                 |
| CDUZ                       | 00034         | Outer IMU Gimbal                                                  |
| CDUT, OPTY                 | 00035         | Rendezvous Radar Trunnion or Optics Y axis                        |
| CDUS, OPTX                 | 00036         | Rendezvous Radar Shaft or Optics X axis                           |
| PIPAX                      | 00037         | Velocity measurement - X axis                                     |
| PIPAY                      | 00040         | Velocity measurement - Y axis                                     |
| PIPAZ                      | 00041         | Velocity measurement - Z axis                                     |
| BMAGX                      | 00042         | RHC input - Pitch                                                 |
| BMAGY                      | 00043         | RHC input - Yaw                                                   |
| BMAGZ                      | 00044         | RHC input - Roll                                                  |
| INLINK                     | 00045         | Telemetry Uplink                                                  |
| RNRAD                      | 00046         | Rendezvous and Landing Radar Data                                 |
| GYROCTR                    | 00047         | Outcounter for gyro                                               |
| CDUXCMD                    | 00050         | Outcounters for X CDU                                             |
| CDUYCMD                    | 00051         | Outcounters for Y CDU                                             |
| CDHZCMD                    | 00052         | Outcounters for Z CDU                                             |
| TVCYYAW, OPTYCMD           | 00053         | Outcounter for optics (Y) or radar, SPS Yaw command in TVC Mode   |
| CDUSCMD, TVCPITCH, OPTXCMD | 00054         | Outcounter for optics (X) or radar, SPS Pitch command in TVC Mode |
| THRUST                     | 00055         | LM DPS Thrust Command                                             |
| LEMONM                     | 00056         | Unused                                                            |
| OUTLINK                    | 00057         | Unused                                                            |
| ALTM                       | 00060         | Altitude Meter                                                    |

# Command Module I/O Channels

## Input Channel 3

Bits 1-14 contains the HIGH ORDER SCALER: 23.3 hours = maximum capacity in increments of 5.12 seconds

## Input Channel 4

Bits 1-14 contains the LOW ORDER SCALER: 5.12 seconds = maximum capacity in increments of 1/3200 seconds

## Output Channel 5

| Bit | Jet Designation<br>SM Quad No./CM Ring No | TRANS/ROT | SM Command<br>CM Command |
| --- | ----------------------------------------- | --------- | ------------------------ |
| 1   | C-3/1-3                                   | +X/+P     | +P                       |
| 2   | C-4/2-4                                   | -X/-P     | -P                       |
| 3   | A-3/2-3                                   | -X/+P     | +P                       |
| 4   | A-4/1-4                                   | +X/-P     | -P                       |
| 5   | D-3/2-5                                   | +X/+YW    | +YW                      |
| 6   | D-4/1-6                                   | -Y/-YW    | -YW                      |
| 7   | B-3/1-5                                   | -X/+YW    | +YW                      |
| 8   | B-4/2-6                                   | +X/-YW    | -YW                      |

## Output Channel 6

| Bit | Jet Designation<br>SM Quad No./CM Ring No | TRANS/ROT | SM Command<br>CM Command |
| --- | ----------------------------------------- | --------- | ------------------------ |
| 1   | B-1/1-1                                   | +Z/+R     | +R                       |
| 2   | B-2/1-2                                   | -Z/-R     | -R                       |
| 3   | D-1/2-1                                   | -Z/+R     | +R                       |
| 4   | D-2/2-2                                   | +Z/-R     | -R                       |
| 5   | A-1                                       | +Y/+R     |                          |
| 6   | A-2                                       | -Y/-R     |                          |
| 7   | C-1                                       | -Y/+R     |                          |
| 8   | C-2                                       | +Y/-R     |                          |

## Output Channel 11

| Bit | Description                  |
| --- | ---------------------------- |
| 1   | ISS Warning                  |
| 2   | Light Computer Activity Lamp |
| 3   | Light Uplink Activity Lamp   |
| 4   | Light Temp Caution Lamp      |
| 5   | Light Keyboard Release Lamp  |
| 6   | Flash Verb and Noun Lamps    |
| 7   | Light Operator Error Lamps   |
| 8   | Spare                        |
| 9   | Test Connector Outbit        |
| 10  | Caution Reset                |
| 11  | Spare                        |
| 12  | Spare                        |
| 13  | Engine On/Off (1-On, 0-Off)  |
| 14  | Spare                        |
| 15  | Spare                        |

## Output Channel 12

| Bit | Description                      |
| --- | -------------------------------- |
| 1   | Zero-Optics CDUs                 |
| 2   | Enable Optics CDU Error Counters |
| 3   | Not used                         |
| 4   | Coarse Align Enable              |
| 5   | Zero IMU CDUs                    |
| 6   | Enable IMU CDU Error Counters    |
| 7   | Spare                            |
| 8   | TVC Enable                       |
| 9   | Enable S-IVB Takeover            |
| 10  | Zero Optics                      |
| 11  | Disengage Optics DAC             |
| 12  | Spare                            |
| 13  | S-IVB Injection Sequence Start   |
| 14  | S-IVB Cutoff                     |
| 15  | ISS Turn-on Delay Complete       |

## Output Channel 13

| Bit | Description                  |
| --- | ---------------------------- |
| 1   | Range Unit Select c          |
| 2   | Range Unit Select b          |
| 3   | Range Unit Select a          |
| 4   | Range Unit Activity          |
| 5   | Not used                     |
| 6   | Block Inlink                 |
| 7   | Downlink Word Order Code Bit |
| 8   | Not used                     |
| 9   | Spare                        |
| 10  | Test Alarms                  |
| 11  | Enable Standby               |
| 12  | Reset Trap 31-A              |
| 13  | Reset Trap 31-B              |
| 14  | Reset Trap 32                |
| 15  | Enable T6RUPT                |

## Output Channel 14

| Bit | Description         |
| --- | ------------------- |
| 1   | Not used            |
| 2   | Spare               |
| 3   | Spare               |
| 4   | Spare               |
| 5   | Not used            |
| 6   | Gyro Enable         |
| 7   | Gyro Select b       |
| 8   | Gyro Select a       |
| 9   | Gyro Sign (1=minus) |
| 10  | Gyro Activity       |
| 11  | Drive CDU S         |
| 12  | Drive CDU T         |
| 13  | Drive CDU Z         |
| 14  | Drive CDU Y         |
| 15  | Drive CDU X         |

### Gyro Selection

| a   | b   | Gyro |
| --- | --- | ---- |
| 0   | 0   | -    |
| 0   | 1   | X    |
| 1   | 0   | Y    |
| 1   | 1   | Z    |

## Input Channel 15

| Bit  | Description              |
| ---- | ------------------------ |
| 1    | Key codes from Main DSKY |
| 2    | Key codes from Main DSKY |
| 3    | Key codes from Main DSKY |
| 4    | Key codes from Main DSKY |
| 5    | Key codes from Main DSKY |
| 6-15 | Spare                    |

## Input Channel 16

| Bit  | Description                    |
| ---- | ------------------------------ |
| 1    | Key codes from Navigation DSKY |
| 2    | Key codes from Navigation DSKY |
| 3    | Key codes from Navigation DSKY |
| 4    | Key codes from Navigation DSKY |
| 5    | Key codes from Navigation DSKY |
| 6    | Mark button                    |
| 7    | Mark reject button             |
| 8-15 | Spare                          |

## Input Channel 30

| Bit | Description                |
| --- | -------------------------- |
| 1   | Ullage Thrust Present      |
| 2   | CM/SM Separate             |
| 3   | SPS Ready                  |
| 4   | S-IVB Separate, Abort      |
| 5   | Liftoff                    |
| 6   | Guidance Reference Release |
| 7   | Optics CDU Fail            |
| 8   | Spare                      |
| 9   | IMU Operate                |
| 10  | S/C Control of Saturn      |
| 11  | IMU Cage                   |
| 12  | IMU CDU Fail               |
| 13  | IMU Fail                   |
| 14  | ISS Turn-On Request        |
| 15  | Temperature in Limits      |

## Input Channel 31

| Bit | Description             |
| --- | ----------------------- |
| 1   | + Pitch Manual Rotation |
| 2   | - Pitch Manual Rotation |
| 3   | + Yaw Manual Rotation   |
| 4   | - Yaw Manual Rotation   |
| 5   | + Roll Manual Rotation  |
| 6   | - Roll Manual Rotation  |
| 7   | +X Translation          |
| 8   | -X Translation          |
| 9   | +Y Translation          |
| 10  | -Y Translation          |
| 11  | +Z Translation          |
| 12  | -Z Translation          |
| 13  | Hold Function           |
| 14  | Free Function           |
| 15  | G&N Autopilot Control   |

## Input Channel 32

| Bit   | Description              |
| ----- | ------------------------ |
| 1     | + Pitch Minimum Impulse  |
| 2     | - Pitch Minimum Impulse  |
| 3     | + Yaw Minimum Impulse    |
| 4     | - Yaw Minimum Impulse    |
| 5     | + Roll Minimum Impulse   |
| 6     | - Roll Minimum Impulse   |
| 7-10  | Spare                    |
| 11    | LM Attached              |
| 12-13 | Spare                    |
| 14    | Proceed (Standby button) |
| 15    | Spare                    |

## Input Channel 33

| Bit | Description          |
| --- | -------------------- |
| 1   | Spare                |
| 2   | Rang Unit Data Good  |
| 3   | Spare                |
| 4   | Zero Optics          |
| 5   | CMC Control          |
| 6   | Not Used             |
| 7   | Not Used             |
| 8   | Spare                |
| 9   | Spare                |
| 10  | Block Uplink Input   |
| 11  | Uplink Too Fast      |
| 12  | Downlink Too Fast    |
| 13  | PIPA Fail            |
| 14  | AGC Warning          |
| 15  | AGC Oscillator Alarm |

## Output Channel 77 (Restart Monitor)

| Bit   | Description                   |
| ----- | ----------------------------- |
| 1     | Parity Fail (E or F memory)   |
| 2     | Parity Fail (E memory)        |
| 3     | TC Trap                       |
| 4     | RUPT Lock                     |
| 5     | Night Watchman                |
| 6     | Voltage Fail                  |
| 7     | Counter Fail                  |
| 8     | Scalar Fail                   |
| 9     | Scalar Double Frequency Alarm |
| 10-15 | Spare                         |

# Interpreter Instruction Set
Interpreter code is always enclosed in a call to the interpreter and the interpreter must always return to the real machine:
```
... machine code
		TC INTPRET
... interpreter code
		EXIT
... machine code
```
When a machine code subroutine needs to be called from interpreter code the `RTB`instruction must be used. When the machine code subroutine has finished it must return to the interpreter by executing this instruction:
```
		TC DANZIG
```
Note: DANZIG is the interpreter's instruction dispatch subroutine and as the interpreter implements a reversed polish notation the dispatch routine was the Gateway to Poland and named Danzig (not considering that Gdansk would have been more appropriate)
## Interpreter registers

| Name   | Description                                                                                                    |
| ------ | -------------------------------------------------------------------------------------------------------------- |
| MPAC   | Multi-purpose accumulator. Can contain single, double and triple precision values or double precision vectors. |
| OVFIND | Overflow indicator register                                                                                    |
| ADRLOC | The interpreter program counter                                                                                |
| QPRET  | Return address register                                                                                        |
| X1, X2 | Two index registers                                                                                            |
| S1, S2 | Two step registers, mostly used as loop counters                                                               |

The interpreter implements a stack with up to 38 values.
The interpreter supports 120 binary switches, numbered 0 - 119

## Store, Load and Push-down instructions

| Mnemonic | X/Y | Description                        |
| -------- | --- | ---------------------------------- |
| STORE    | X   | Store MPAC                         |
| STODL    | X   |                                    |
|          | Y   | Store MPAC and Reload in DP        |
| STOVL    | X   |                                    |
|          | Y   | Store MPAC and Reload a Vector     |
| STCALL   | X   |                                    |
|          | Y   | Store MPAC and Call a Routine      |
| DLOAD    | X   | Load MPAC in DP                    |
| TLOAD    | X   | Load MPAC in TP                    |
| VLOAD    | X   | Load MPAC in Vector                |
| SLOAD    | X   | Load MPAC in Single Precision      |
| PDDL     | X   | Push MPAC onto Stack and Reload DP |
| PDVL     | X   | Push MPAC onto Stack and Reload DP |
| PUSH     |     | Push MPAC onto Stack               |
| SETPD    | X   | Set Push-Down Stack Pointer        |

## Arithmetic Instructions

| Mnemonic | X/Y | Description           |
| -------- | --- | --------------------- |
| DAD      | X   | DP Add                |
| DSU      | X   | DP Subtract           |
| BDSU     | X   | DP Subtract From      |
| DMP      | X   | DP Multiply           |
| DMPR     | X   | DP Multiply and Round |
| DDV      | X   | DP Divide By          |
| BDDV     | X   | DP Divide Into        |
| SIGN     | X   | DP Sign Test          |
| TAD      | X   | TP Add                |

## Vector Arithmetic Instructions

| Mnemonic | X/Y | Description                          |
| -------- | --- | ------------------------------------ |
| VAD      | X   | Vector Add                           |
| VSU      | X   | Vector Subtract                      |
| BVSU     | X   | Vector Subtract From                 |
| DOT      | X   | Vector Dot Product                   |
| VXSC     | X   | Vector Times Scalar                  |
| V/SC     | X   | Vector Divided by Scalar             |
| VXV      | X   | Vector Cross Product                 |
| VPROJ    | X   | Vector Projection                    |
| VXM      | X   | Matrix Pre-Multiplication by Vector  |
| MXV      | X   | Matrix Post-Multiplication by Vector |

## Scalar Functions

| Mnemonic | Description       |
| -------- | ----------------- |
| SQRT     | DP Square Root    |
| SIN      | DP Sin            |
| COS      | DP Cosine         |
| ARCSIN   | DP Arcsin         |
| ARCCOS   | DP Arccosine      |
| DSQ      | DP Square         |
| ROUND    | Round to DP       |
| DCOMP    | TP Complement     |
| ABS      | TP Absolute Value |

## Vector Functions

| Mnemonic | Description             |
| -------- | ----------------------- |
| UNIT     | Unit Vector Function    |
| ABVAL    | Vector Length           |
| VSQ      | Square of Vector Length |
| VCOMP    | Vector Complement       |
| VDEF     | Vector Define           |

## Shift Instructions

| Mnemonic | Description                         |
| -------- | ----------------------------------- |
| SR1      | Scalar Shift Right 1 Bit            |
| SR2      | Scalar Shift Right 2 Bits           |
| SR3      | Scalar Shift Right 3 Bits           |
| SR4      | Scalar Shift RIght 4 Bits           |
| SL1      | Scalar Shift Left 1 Bit             |
| SL2      | Scalar Shift Left 2 Bits            |
| SL3      | Scalar Shift Left 3 Bits            |
| SL4      | Scalar Shift Left 4 Bits            |
| SR1R     | Scalar Shift Right 1 Bit and Round  |
| SR2R     | Scalar Shift Right 2 Bits and Round |
| SR3R     | Scalar Shift Right 3 Bits and Round |
| SR4R     | Scalar Shift RIght 4 Bits and Round |
| SL1R     | Scalar Shift Left 1 Bit and Round   |
| SL2R     | Scalar Shift Left 2 Bits and Round  |
| SL3R     | Scalar Shift Left 3 Bits and Round  |
| SL4R     | Scalar Shift Left 4 Bits and Round  |
| VSR1     | Vector Shift Right 1 Bit and Round  |
| VSR2     | Vector Shift RIght 2 Bits and Round |
| VSR3     | Vector Shift Right 3 Bits and Round |
| VSR4     | Vector Shift RIght 4 Bits and Round |
| VSR5     | Vector Shift Right 5 Bits and Round |
| VSR6     | Vector Shift Right 6 Bits and Round |
| VSR7     | Vector Shift Right 7 Bits and Round |
| VSR8     | Vector Shift Right 8 Bits and Round |
| VSL1     | Vector Shift Left 1 Bit and Round   |
| VSL2     | Vector Shift Left 2 Bits and Round  |
| VSL3     | Vector Shift Left 3 Bits and Round  |
| VSL4     | Vector Shift Left 4 Bits and Round  |
| VSL5     | Vector Shift Left 5 Bits and Round  |
| VSL6     | Vector Shift Left 6 Bits and Round  |
| VSL7     | Vector Shift Left 7 Bits and Round  |
| VSL8     | Vector Shift Left 8 Bits and Round  |

## General Shift Instructions

| Mnemonic | X/Y | Description                          |
| -------- | --- | ------------------------------------ |
| SL       | X   | General Scalar Shift Left            |
| SRR      | X   | General Scalar Shift RIght and Round |
| SLR      | X   | General Scalar Shift Left and Round  |
| VSR      | X   | General Vector Shift Right           |
| VSL      | X   | General Vector Shift Left            |

## Normalization

| Mnemonic | X/Y | Description      |
| -------- | --- | ---------------- |
| NORM     | X   | Scalar Normalize |

## Branching, Sequence Changing and Subroutine Linkage Instructions

| Mnemonic | X/Y | Description                             |
| -------- | --- | --------------------------------------- |
| GOTO     | X   | Go to                                   |
| CALL     | X   | Call Subroutine                         |
| CGOTO    | X   |                                         |
|          | Y   | Computed Go To                          |
| RVQ      |     | Return via QPRET                        |
| STQ      |     | Store QPRET                             |
| BPL      | X   | Branch on MPAC Plus                     |
| BZE      | X   | Branch on MPAC Zero                     |
| BMN      | X   | Branch on MPAC Minus                    |
| BHIZ     | X   | Branch on High Order Zero in MPAC       |
| BOV      | X   | Branch on MPAC Overflow                 |
| BOVB     | X   | Branch on Overflow to Basic Instruction |
| RTB      | X   | Return to Basic Instructions            |
| EXIT     | X   | Exit Interpreter                        |

## Switch Instructions

| Mnemonic | X/Y | Description             |
| -------- | --- | ----------------------- |
| SET      | X   | Set Switch              |
| CLEAR    | X   | Clear Switch            |
| INVERT   | X   | Invert Switch           |
| SETGO    | X   | Set Switch and Go To    |
| CLRGO    | X   | Clear Switch and Go To  |
| INVGO    | X   | Invert Switch and Go To |

## Switch Test Instructions

| Mnemonic | X/Y | Description                               |
| -------- | --- | ----------------------------------------- |
| BON      | X   |                                           |
|          | Y   | Branch if Switch is On                    |
| BOFF     | X   |                                           |
|          | Y   | Branch if Switch is Off                   |
| BONSET   | X   |                                           |
|          | Y   | Branch if Switch is On and Set Switch     |
| BOFFSET  | X   |                                           |
|          | Y   | Branch if Switch is Off and Set Switch    |
| BONCLR   | X   |                                           |
|          | Y   | Branch if Switch is On and Clear Switch   |
| BOFCLR   | X   |                                           |
|          | Y   | Branch if Switch is Off and Clear Switch  |
| BONINV   | X   |                                           |
|          | Y   | Branch if Switch is On and Invert Switch  |
| BOFINV   | X   |                                           |
|          | Y   | Branch if Switch is Off and Invert Switch |

## Index Register Instructions

| Mnemonic | X/Y | Description                           |
| -------- | --- | ------------------------------------- |
| AXT,1    |     |                                       |
| AXT,2    | X   | Address to Index True                 |
| AXC,1    |     |                                       |
| AXC,2    | X   | Address to Index Complemented         |
| LXA,1    |     |                                       |
| LXA,2    | X   | Load Index from Erasable              |
| LXC,1    |     |                                       |
| LXC,2    | X   | Load Index from Erasable Complemented |
| SXA,1    |     |                                       |
| SXA,2    | X   | Store Index into Erasable             |
| XCHX,1   |     |                                       |
| XCHX,2   | X   | Exhange Index with Erasable           |
| INCR,1   |     |                                       |
| INCR,2   | X   | Increment Index                       |
| XAD,1    |     |                                       |
| XAD,2    | X   | Index Register Add                    |
| XSU,1    |     |                                       |
| XSU,2    | X   | Index Register Subtract               |
| TIX,1    |     |                                       |
| TIX,2    | X   | Transfer on Index                     |

## Miscellaneous Instructions

| Mnemonic | X/Y | Description                 |
| -------- | --- | --------------------------- |
| SSP      | X   | Set Single Precision        |
| STADR    |     | Push Up Stack on Store Code |

# Command Module Programs (Major Modes)

| Group                       | Program number | Description                                                  |
| --------------------------- | -------------- | ------------------------------------------------------------ |
| Prelaunch and Service       | 00             | AGC Idling                                                   |
|                             | 01             | Prelaunch or Service: Initializing                           |
|                             | 02             | Prelaunch or Service: Gyrocompassing                         |
|                             | 03             | Prelaunch or Service: Optical Verification of Gyrocompassing |
|                             | 06             | AGC Power Down                                               |
|                             | 07             | System Test                                                  |
| Boost                       | 11             | Earth Orbit Insertion Monitor                                |
|                             | 15             | TLI Initiate/Cutoff                                          |
| Coast                       | 20             | Universal Tracking                                           |
|                             | 21             | Ground Track Determination                                   |
|                             | 22             | Orbital Navigation                                           |
|                             | 23             | Cislunar Midcourse Navigation                                |
|                             | 24             | Rate Aided Optics Tracking                                   |
|                             | 27             | AGC Update                                                   |
|                             | 29             | Time-of-Longitude                                            |
| Pre-Thrusting               | 30             | External Delta Y                                             |
|                             | 31             | CSM Height Adjustment Maneuver (HAM)                         |
|                             | 32             | CSM Coeliptic Sequence Initiation (CSI)                      |
|                             | 33             | CSM Constant Delta Altitude (CDA)                            |
|                             | 34             | CSM Transfer Phase Initiation (TPI) Targeting                |
|                             | 35             | CSM Transfer Phase Midcourse (TPM) Targeting                 |
|                             | 36             | CSM Plane Change (PC) Targeting                              |
|                             | 37             | Return to Earth                                              |
| Thrusting                   | 40             | SPS                                                          |
|                             | 41             | RCS                                                          |
|                             | 47             | Thrust Monitor                                               |
| Alignment                   | 51             | IMU Orientation Determination                                |
|                             | 52             | IMU Realign                                                  |
|                             | 53             | Backup IMU Orientation Determination                         |
|                             | 54             | Backup IMU Realign                                           |
| Entry                       | 61             | Entry-Preparation                                            |
|                             | 62             | Entry-CM/SM Separation and Preentry Manuever                 |
|                             | 63             | Entry-Initialization                                         |
|                             | 64             | Entry-Post 0.05G                                             |
|                             | 65             | Entry-Upcontrol                                              |
|                             | 66             | Entry-Balistic                                               |
|                             | 67             | Entry-Final Phase                                            |
| Pre-Thrusting Other Vehicle | 72             | LM Coelliptic Sequence Initiation (CSI)                      |
|                             | 73             | LM Constant Delta Attitude (CDA)                             |
|                             | 74             | LM Transfer Phase Initiation (TPI) Targeting                 |
|                             | 75             | LM Transfer Phase (Midcourse) Targeting                      |
|                             | 76             | LM Target Delta V                                            |
|                             | 77             | CSM Target Delta V                                           |
|                             | 78             | Rendezvous Final Phase                                       |

# Command Module Routines

| Routine | Routine Title                              |
| ------- | ------------------------------------------ |
| 00      | Final Automatic Request Terminate          |
| 01      | Erasable and Channel Modification Routine  |
| 02      | IMU Status Check                           |
| 03      | Digital Autopilot Data Load                |
| 05      | S-Band Antenna                             |
| 07      | MINKEY Controller                          |
| 21      | Rendezvous Tracking Sighting Mark          |
| 22      | Rendezvous Tracking Data Processing        |
| 23      | Backup Rendezvous Tracking Sighting Mark   |
| 30      | Orbit Parameter Display                    |
| 31      | Rendezvous Parameter Display Routine No. 1 |
| 33      | AGC/LGC Clock Synchronization              |
| 34      | Rendezvous Parameter Display Routine No. 2 |
| 35      | Lunar Landmark Selection                   |
| 36      | Rendezvous Out-Of-Plane Display Routine    |
| 40      | SPS Thrust Fail                            |
| 41      | State Vector Integration (MID to AVE)      |
| 50      | Coarse Align                               |
| 52      | Automatic Optics Positioning               |
| 53      | Sighting Mark                              |
| 54      | Sighting Mark Display                      |
| 55      | Gyro Torquing                              |
| 56      | Alternate LOS Sighting Mark                |
| 57      | Optics Calibration                         |
| 60      | Attitude Maneuver                          |
| 61      | Tracking Attitude                          |
| 62      | Crew-Defined Maneuver                      |
| 63      | Rendezvous Final Attitude                  |
| 67      | Universal Tracking Routing                 |

# Command Module Verbs

## Regular Verbs

| Verb | Description                                   |
| ---- | --------------------------------------------- |
| 01   | Display Octal Component 1 in R1               |
| 02   | Display Octal Component 2 in R2               |
| 03   | Display Octal Component 3 in R3               |
| 04   | Display Octal Components 1,2 in R1, R2        |
| 05   | Display Octal Components 1,2,3 in R1, R2, R3  |
| 06   | Display Decimal in R1 or R1, R2 or R1, R2, R3 |
| 07   | Display DP in R1 or R1, R2 or R1, R2, R3      |
| 08   | Spare                                         |
| 09   | Spare                                         |
| 10   | Spare                                         |
| 11   | Monitor Octal Component 1 in R1               |
| 12   | Monitor Octal Component 2 in R2               |
| 13   | Monitor Octal Component 3 in R3               |
| 14   | Monitor Octal Components 1,2 in R1, R2        |
| 15   | Monitor Octal Components 1,2,3 in R1, R2, R3  |
| 16   | Monitor Decimal in R1 or R1, R2 or R1, R2, R3 |
| 17   | Monitor DP in R1, R2 (test only)              |
| 18   | Spare                                         |
| 19   | Spare                                         |
| 20   | Spare                                         |
| 21   | Load Component 1 in R1                        |
| 22   | Load Component 2 in R2                        |
| 23   | Load Component 3 in R3                        |
| 24   | Load Components 1,2 in R1, R2                 |
| 25   | Load Components 1,2,3 in R1, R2, R3           |
| 26   | Spare                                         |
| 27   | Display Fixed Memory                          |
| 28   | Spare                                         |
| 29   | Spare                                         |
| 30   | Request EXECUTIVE                             |
| 31   | Request WAITLIST                              |
| 32   | Recycle program                               |
| 33   | Proceed without DSKY inputs                   |
| 34   | Terminate function                            |
| 35   | Test lights                                   |
| 36   | Request FRESH START                           |
| 37   | Change program (major mode)                   |
| 38   | Spare                                         |
| 39   | Spare                                         |

## Extended Verbs

| Verb | Description                                                             |
| ---- | ----------------------------------------------------------------------- |
| 40   | Zero ICDU                                                               |
| 41   | Coarse align CDUs (Specify N20 or N91)                                  |
| 42   | Pulse torque gyros                                                      |
| 43   | Load IMU attitude error needles (test only)                             |
| 44   | Set Surface flag                                                        |
| 45   | Reset Surface flag                                                      |
| 46   | Establish G & N autopilot control                                       |
| 47   | Move LM state vector into CM state vector                               |
| 48   | Request DAP Data Load routing (R03)                                     |
| 49   | Start automatic attitude maneuver                                       |
| 50   | Please perform                                                          |
| 51   | Please mark                                                             |
| 52   | Mark on offset landing site                                             |
| 53   | Please perform COAS mark                                                |
| 54   | Request rendezvous backup sighting mark routine (R23)                   |
| 55   | Increment AGC time (decimal)                                            |
| 56   | Terminate tracking (P20)                                                |
| 57   | Request display of full track flag (FULTKFLAG)                          |
| 58   | Request stick flag and Set V50 N18 flag                                 |
| 59   | Please calibrate                                                        |
| 60   | Set astronaut total attitude (N17) to present attitude                  |
| 61   | Display DAP following attitude errors (Mode 1)                          |
| 62   | Display total attitude errors with respect to Noun 22 (Mode 2)          |
| 63   | Display total astronaut attitude error with respect to Noun 17 (Mode 3) |
| 64   | Request S-Band Antenna routing (R05)                                    |
| 65   | Optical verification of prelaunch alignment                             |
| 66   | Vehicles are attached. Move this vehicle state vector to other vehicle  |
| 67   | Start W-matrix RMS error display                                        |
| 68   | Spare                                                                   |
| 69   | Cause RESTART                                                           |
| 70   | Update liftoff time (P27)                                               |
| 71   | Start AGC update; block address (P27)                                   |
| 72   | Start AGC update; single address (P27)                                  |
| 73   | Start AGC update; AGC time (P27)                                        |
| 74   | Initialize erasable dump via DOWNLINK                                   |
| 75   | Backup liftoff                                                          |
| 76   | Spare                                                                   |
| 77   | Spare                                                                   |
| 78   | Update prelaunch azimuth                                                |
| 79   | Spare                                                                   |
| 80   | Enable LM state vector update                                           |
| 81   | Enable CSM state vector update                                          |
| 82   | Request Orbit Parameter display (R30)                                   |
| 83   | Request Rendezvous Parameter display No. 1 (R31)                        |
| 80   | Spare                                                                   |
| 85   | Request Rendezvous Parameter display No. 2 (R34)                        |
| 86   | Reject Rendezvous Backup Sighting Mark                                  |
| 87   | Set VHF Range flag                                                      |
| 88   | Reset VHF Range flag                                                    |
| 89   | Request Rendezvous Final Attitude maneuver (R63)                        |
| 90   | Request Out of Place Rendezvous display (R36)                           |
| 91   | Display BANKSUM                                                         |
| 92   | Spare                                                                   |
| 93   | Enable W matrix initialization                                          |
| 94   | Perform Cislunar Attitude maneuver                                      |
| 95   | Spare                                                                   |
| 96   | Terminate integration and go to P00                                     |
| 97   | Please perform engine-fail (R40)                                        |
| 98   | Spare                                                                   |
| 99   | Please Enable Engine Ignition                                           |

# Command Module Nouns

| Number | Register Description                              | Data Format                 |
| ------ | ------------------------------------------------- | --------------------------- |
| 00     | Not in use                                        |                             |
| 01     | Specify address (fractional)                      | .XXXXX (fractional)         |
|        |                                                   | .XXXXX (fractional)         |
|        |                                                   | .XXXXX (fractional)         |
| 02     | Specify address (whole)                           | XXXXX. (integer)            |
|        |                                                   | XXXXX. (integer)            |
|        |                                                   | XXXXX. (integer)            |
| 03     | Specify address (degree)                          | XXX.XX (degree)             |
|        |                                                   | XXX.XX (degree)             |
|        |                                                   | XXX.XX (degree)             |
| 04     | Spare                                             |                             |
| 05     | Angular error/difference                          | XXX.XX (degree)             |
| 06     | Option code ID                                    | Octal                       |
|        | Option code                                       | Octal                       |
| 07     | FLAGWORD operator                                 |                             |
|        | ECADR                                             | Octal                       |
|        | BIT ID                                            | Octal                       |
|        | Action                                            | Octal                       |
| 08     | Alarm data                                        |                             |
|        | ADRES                                             | Octal                       |
|        | BBANK                                             | Octal                       |
|        | ERCOUNT                                           | Octal                       |
| 09     | Alarm codes                                       |                             |
|        | First                                             | Octal                       |
|        | Second                                            | Octal                       |
|        | Last                                              | Octal                       |
| 10     | Channel to be specified                           | Octal                       |
| 11     | TIG of CSI                                        | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 12     | Option code (extended verbs only)                 | Octal                       |
| 13     | TIG of CDH                                        | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 14     | Specified inertial velocity at TLI cutoff (V C/O) | XXXXX. ft/s                 |
| 15     | Increment address                                 | Octal                       |
| 16     | Time of event                                     | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 17     | Astronaut total attitude                          | R XXX.XX (degree)           |
|        |                                                   | P XXX.XX (degree)           |
|        |                                                   | Y XXX.XX (degree)           |
| 18     | Desired automaneuver FDAI ball angles             | R XXX.XX (degree)           |
|        |                                                   | P XXX.XX (degree)           |
|        |                                                   | Y XXX.XX (degree)           |
| 19     | Spare                                             |                             |
| 20     | Present ICDU angles                               | R (OG) XXX.XX (degree)      |
|        |                                                   | P (IG) XXX.XX (degree)      |
|        |                                                   | Y (MG) XXX.XX (degree)      |
| 21     | PIPAs                                             | X XXXXX. (pulses)           |
|        |                                                   | Y XXXXX. (pulses)           |
|        |                                                   | Z XXXXX. (pulses)           |
| 22     | Desired ICDU angles                               | R (OG) XXX.XX (degree)      |
|        |                                                   | P (IG) XXX.XX (degree)      |
|        |                                                   | Y (MG) XXX.XX (degree)      |
| 23     | Spare                                             |                             |
| 24     | Delta time for AGC clock                          | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 25     | CHECKLIST (used with V50)                         | XXXXX.                      |
| 26     | PRIO/DELAY, ADRES, BBCON                          | Octal                       |
|        |                                                   | Octal                       |
|        |                                                   | Octal                       |
| 27     | Self-test on/off switch                           | XXXXX.                      |
| 28     | Spare                                             |                             |
| 29     | XSM launch azimuth                                | XXX.XX (degree)             |
| 30     | Target codes                                      | XXXXX.                      |
|        |                                                   | XXXXX.                      |
|        |                                                   | XXXXX.                      |
| 31     | Time of W matrix initialization                   | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 32     | Time for perigee                                  | 00XXX.h                     |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 33     | Time of ignition (GETI)                           | 00XXX.h                     |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 34     | Time of event                                     | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 35     | Time from event                                   | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 36     | Time of AGC clock                                 | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 37     | Time of ignition (TPI)                            | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 38     | Time of state being integrated                    | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 39     | Delta time for transfer                           | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | 0XX.XX s                    |
| 40     | Time from ignition/cutoff (TFI/TFC)               | XXbXX min s                 |
|        | VG                                                | XXXX.X ft/s                 |
|        | Delta V (accumulated)                             | XXXX.X ft/s                 |
| 41     | Target                                            |                             |
|        | Azimuth                                           | XXX.XX (degree)             |
|        | Elevation                                         | XXX.XX (degree)             |
| 42     | Apocenter altitude                                | XXXX.X nmi                  |
|        | Pericenter altitude                               | XXXX.X nmi                  |
| 43     | Latitude                                          | XXX.XX (degree, + north)    |
|        | Longitude                                         | XXX.XX (degree, + east)     |
|        | Altitude                                          | XXXX.X nmi                  |
| 44     | Apocenter altitude                                | XXXX.X nmi                  |
|        | Pericenter altitude                               | XXXX.X nmi                  |
|        | TFF                                               | XXbXX min s                 |
| 45     | Marks (VHF/optics)                                | XXbXX marks                 |
|        | Time from ignition of next burn                   | XXbXX min s                 |
|        | Middle gimbal angle                               | XXX.XX (degree)             |
| 46     | DAP configuration                                 | Octal                       |
|        |                                                   | Octal                       |
| 47     | CSM weight                                        | XXXXX. lbs                  |
|        | LM weight                                         | XXXXX. lbs                  |
| 48     | Gimbal pitch trim                                 | XXX.XX (degree)             |
|        | Gimbal yaw trim                                   | XXX.XX (degree)             |
| 49     | Delta R                                           | XXXX.X nmi                  |
|        | Delta V                                           | XXXX.X ft/s                 |
|        | VHF or optics code                                | 0000X                       |
| 50     | Splash error                                      | XXXX.X nmi                  |
|        | Perigee                                           | XXXX.X nmi                  |
|        | TFF                                               | XXbXX min s                 |
| 51     | S-Band antenna angles                             |                             |
|        | RHO                                               | XXX.XX (degree)             |
|        | GAMMA                                             | XXX.XX (degree)             |
| 52     | Central angle of active vehicle                   | XXX.XX (degree)             |
| 53     | Range                                             | XXX.XX nmi                  |
|        | Range rate                                        | XXXX.X ft/s                 |
|        | Phi                                               | XXX.XX (degree)             |
| 54     | Range                                             | XXX.XX nmi                  |
|        | Range rate                                        | XXXX.X ft/s                 |
|        | Theta                                             | XXX.XX (degree)             |
| 55     | Perigee code                                      | 0000X.                      |
|        | Elevation angle                                   | XXX.XX (degree)             |
|        | Central angle of passive vehicle                  | XXX.XX (degree)             |
| 56     | Reentry angle                                     | XXX.XX (degree)             |
|        | Delta V                                           | XXXX.X ft/s                 |
| 57     | Spare                                             |                             |
| 58     | Pericenter altitude (post TPI)                    | XXXX.X nmi                  |
|        | Delta V (TPI)                                     | XXXX.X ft/s                 |
|        | Delta V (TPF)                                     | XXXX.X ft/s                 |
| 59     | Delta V (LOS1)                                    | XXXX.X ft/s                 |
|        | Delta V (LOS2)                                    | XXXX.X ft/s                 |
|        | Delta V (LOS3)                                    | XXXX.X ft/s                 |
| 60     | GMAX                                              | XXX.XX g                    |
|        | VPRED                                             | XXXXX. ft/s                 |
|        | GAMMA EI                                          | XXX.XX (degree + above)     |
| 61     | Impact                                            |                             |
|        | Latitude                                          | XXX.XX (degree + north)     |
|        | Longitude                                         | XXX.XX (degree + east)      |
|        | Heads up/down                                     | +/- 00001                   |
| 62     | Intertial velocity magnitude                      | XXXXX. ft/s                 |
|        | Altitude rate                                     | XXXXX. ft/s                 |
|        | Altitude above pad radius                         | XXXX.X nmi                  |
| 63     | Range from EI altitude to splash                  | XXXX.X nmi                  |
|        | Predicted inertial velocity                       | XXXXX. ft/s                 |
|        | Time from EI altitude                             | XXbXX min s                 |
| 64     | Drag aceleration                                  | XXX.XX g                    |
|        | Inertial velocity                                 | XXXXX. ft/s                 |
|        | Range to splash                                   | XXXX.X nmi (+ is overshoot) |
| 65     | Sampled AGC time (fetched in interrupt)           | 00XXX. h                    |
|        |                                                   | 000XX. m                    |
|        |                                                   | XXbXX. s                    |
| 66     | Commanded bank angle                              | XXX.XX (degree)             |
|        | Crossrange error                                  | XXXX.X nmi (+ south)        |
|        | Downrange error                                   | XXXX.X nmi (+ overshoot)    |
| 67     | Range to target                                   | XXXX.X nmi (+ overshoot)    |
|        | Present latitiude                                 | XXX.XX (degree + north)     |
|        | Present longitude                                 | XXX.XX (degree + east)      |
| 68     | Commanded bank angle                              | XXX.XX (degree)             |
|        | Intertial velocity                                | XXXXX. ft/s                 |
|        | Altitude rate                                     | XXXXX. ft/s                 |
| 69     | Commanded bank angle                              | XXX.XX (degree)             |
|        | Drag level                                        | XXX.XX g                    |
|        | Exit velocity                                     | XXXXX. ft/s                 |
| 70     | Celestial body code (before mark)                 | Octal                       |
|        | Landmark data                                     | Octal                       |
|        | Horizon data                                      | Octal                       |
| 71     | Celestial body code (after mark)                  | Octal                       |
|        | Landmark data                                     | Octal                       |
|        | Horizon data                                      | Octal                       |
| 72     | Spare                                             |                             |
| 73     | Altitude                                          | XXXXX. nmi                  |
|        | Velocity                                          | XXXXX. ft/s                 |
|        | Flight path angle                                 | XXX.XX (degree)             |
| 74     | Commanded bank angle                              | XXX.XX (degree)             |
|        | Inertial velocity                                 | XXXXX. ft/s                 |
|        | Drag acceleration                                 | XXX.XX g                    |
| 75     | Delta altitude (CDH)                              | XXXX.X nmi                  |
|        | Delta time (CDH - CSI or TPI - CDH)               | XXbXX m s                   |
|        | Delta time (TPI-CDH or TPI-NOMTPI)                | XXbXX m s                   |
| 76     | Spare                                             |                             |
| 77     | Spare                                             |                             |
| 78     | GAMMA                                             | XXX.XX (degree)             |
|        | RHO                                               | XXX.XX (degree)             |
|        | PHI                                               | XXX.XX (degree)             |
| 79     | P20 rotation rate                                 | X.XXXX deg/s                |
|        | P20 deadband                                      | XXX.XX (degree)             |
| 80     | Time from ignition/cutoff                         | XXbXX m s                   |
|        | Velocity to be gained                             | XXXXX. ft/s                 |
|        | Delta V (accumulated)                             | XXXXX. ft/s                 |
| 81     | Delta VX (LV)                                     | XXXX.X ft/s                 |
|        | Delta VY (LV)                                     | XXXX.X ft/s                 |
|        | Delta VZ (LV)                                     | XXXX.X ft/s                 |
| 82     | Delta VX (LV)                                     | XXXX.X ft/s                 |
|        | Delta VY (LV)                                     | XXXX.X ft/s                 |
|        | Delta VZ (LV)                                     | XXXX.X ft/s                 |
| 83     | Delta VX (body)                                   | XXXX.X ft/s                 |
|        | Delta VY (body)                                   | XXXX.X ft/s                 |
|        | Delta VZ (body)                                   | XXXX.X ft/s                 |
| 84     | Delta VX (LV of other vehicle)                    | XXXX.X ft/s                 |
|        | Delta VY (LV of other vehicle)                    | XXXX.X ft/s                 |
|        | Delta VZ (LV of other vehicle)                    | XXXX.X ft/s                 |
| 85     | VGX (body)                                        | XXXX.X ft/s                 |
|        | VGY (body)                                        | XXXX.X ft/s                 |
|        | VGZ (body)                                        | XXXX.X ft/s                 |
| 86     | Delta VX (LV)                                     | XXXX.X ft/s                 |
|        | Delta VY (LV)                                     | XXXX.X ft/s                 |
|        | Delta VZ (LV)                                     | XXXX.X ft/s                 |
| 87     | Mark data                                         |                             |
|        | Shaft angle                                       | XXX.XX (degree)             |
|        | Trunnion angle                                    | XXX.XX (degree)             |
| 88     | Celestial body unit vector                        |                             |
|        | X                                                 | .XXXXX                      |
|        | Y                                                 | .XXXXX                      |
|        | Z                                                 | .XXXXX                      |
| 89     | Landmark latitude                                 | XX.XXX (degree + north)     |
|        | Landmark longitude/2                              | XX.XXX (degree + east)      |
|        | Landmark altitude                                 | XXX.XX nmi                  |
| 90     | Rendezvous out of plane parameters                |                             |
|        | Y (Active)                                        | XXX.XX nmi                  |
|        | YDOT (Active)                                     | XXXX.X ft/s                 |
|        | YDOT (Passive)                                    | XXXX.X ft/s                 |
| 91     | Present optics angle                              |                             |
|        | Shaft                                             | XXX.XX (degree)             |
|        | Trunnion                                          | XXX.XX (degree)             |
| 92     | New optics angle                                  |                             |
|        | Shaft                                             | XXX.XX (degree)             |
|        | Trunnion                                          | XXX.XX (degree)             |
| 93     | Delta Gyro Angles                                 |                             |
|        | X                                                 | XX.XXX (degree)             |
|        | Y                                                 | XX.XXX (degree)             |
|        | Z                                                 | XX.XXX (degree)             |
| 94     | Alternate LOS                                     |                             |
|        | Shaft angle                                       | XXX.XX (degree)             |
|        | Trunnion angle                                    | XX.XXX (degree)             |
| 95     | Time from ignition/cutoff                         | XXbXX m s                   |
|        | VG (P15)                                          | XXXXX. ft/s                 |
|        | VI (P15)                                          | XXXXX. ft/s                 |
| 96     | Y (CSM)                                           | XXX.XX nmi                  |
|        | YDOT (CSM)                                        | XXXX.X ft/s                 |
|        | YDOT (LM)                                         | XXXX.X ft/s                 |
| 97     | System test inputs                                | XXXXX.                      |
|        |                                                   | XXXXX.                      |
|        |                                                   | XXXXX.                      |
| 98     | System test results and input                     | XXXXX.                      |
|        |                                                   | .XXXXX                      |
|        |                                                   | XXXXX.                      |
| 99     | RMS in position                                   | XXXXX. ft                   |
|        | RMS in velocity                                   | XXXX.X ft/s                 |
|        | RMS option code                                   | XXXXX.                      |

# Command Module Program Alarms

| Code  | Interpretation                                                       | Issued By                        |
| ----- | -------------------------------------------------------------------- | -------------------------------- |
| 00110 | No mark since last mark reject                                       | SXTMARK                          |
| 00113 | No inbits (Channel 16)                                               | SXTMARK                          |
| 00114 | Mark made but not desired                                            | SXTMARK                          |
| 00115 | Optics torque request with switch not at CMC                         | Extended verb optics CDU         |
| 00116 | Optics switch altered before 15 seconds ZERO time elapsed            | T4RUPT                           |
| 00117 | Optics torque request with Optics not available                      | Extended verb optics CDU         |
| 00120 | Optics torque request with Optics not ZEROED                         | T4RUPT                           |
| 00121 | Optics CDU no good at time of mark                                   | SXTMARK                          |
| 00205 | Bad PIPA reading                                                     | SERVICER                         |
| 00206 | Zero encode not allowed with coarse align + gimbal lock              | IMU mode switch                  |
| 00207 | ISS turn-on request not present for 90 seconds                       | T4RUPT                           |
| 00210 | IMU not operating                                                    | IMU mode switch, IMU-2, R05, P51 |
| 00211 | Coarse align error-drive > 2 degrees                                 | IMU mode switch                  |
| 00212 | PIPA fail but PIPA is not being used                                 | IMU mode switch, T4RUPT          |
| 00213 | IMU not operating with turn-on request                               | T4RUPT                           |
| 00214 | Program using IMU when turned off                                    | T4RUPT                           |
| 00217 | Bad return from Stall routines                                       | CURTAINS                         |
| 00220 | IMU not aligned (REFSMMAT)                                           | R02, P51                         |
| 00401 | Desired gimbal angles yield Gimbal Lock                              | Fine Align, IMU-2                |
| 00402 | Second MINKEY pulse torque must be done                              | P52                              |
| 00404 | Target out of view (trunnion angle > 90 degrees)                     | R52                              |
| 00405 | Two stars not available                                              | P52, P54                         |
| 00406 | Rendezvous navigation not operating                                  | R21, R23                         |
| 00421 | W-Matrix overflow                                                    | INTEGRV                          |
| 00600 | Imaginary roots on first iteration                                   | P32, P72                         |
| 00601 | Perigee altitude after CSI < 85 nmi earth orbit, 35000 ft moon orbit | P32, P72                         |
| 00602 | Perigee altitude after CDH < 85 nmi earth orbit, 35000 ft moon orbit | P32, P72                         |
| 00603 | CSI to CDH time < 10 minutes                                         | P32, P33, P72, P73               |
| 00604 | CDH to TPI time < 10 minutes                                         | P32, P72                         |
| 00605 | Number of iteration exceeds loop maximum                             | P32, P72, P37                    |
| 00606 | DV exceeds maximum                                                   | P32, P72                         |
| 00611 | No TIG for elevation angle                                           | P37, P74                         |
| 00612 | State vector on wrong sphere of influence                            | P37                              |
| 00613 | Reentry angle out of limits                                          | P37                              |
| 00777 | PIPA fail caused ISS warning                                         | T4RUPT                           |
| 01102 | AGC self test error                                                  | SELF-CHECK                       |
| 01105 | DOWNLINK too fast                                                    | T4RUPT                           |
| 01106 | UPLINK too fast                                                      | T4RUPT                           |
| 01107 | Phase table failure; assume erasable memory is destroyed             | RESTART                          |
| 01301 | ARCSIN-ARCCOS argument too large                                     | INTERPRETER                      |
| 01407 | VG increasing                                                        | S40.8                            |
| 01426 | IMU unsatisfactory                                                   | P61, P62                         |
| 01427 | IMU reversed                                                         | P61, P62                         |
| 01520 | V37 request not permitted at this time                               | V37                              |
| 01600 | Overflow in Drift Test                                               | Optical Prealignment Calibration |
| 01601 | Bad IMU torque                                                       | Optical Prealignment Calibration |
| 01703 | Insufficient time for integration                                    | R41                              |
| 03777 | ICDU fail caused the ISS warning                                     | T4RUPT                           |
| 04777 | ICDU, PIPA fails caused the ISS warning                              | T4RUPT                           |
| 07777 | IMU fail caused the ISS warning                                      | T4RUPT                           |
| 10777 | IMU, PIPA fails caused the ISS warning                               | T4RUPT                           |
| 13777 | IMU, ICDU fails cause the ISS warning                                | T4RUPT                           |
| 14777 | IMU, ICDU, PIPA fails cause the ISS warning                          | T4RUPT                           |
| 20430 | Integration abort due to subsurface state vector                     | All calls to integration         |
| 20607 | No solution for Time-Theta or Time-Radius                            | TIMETHET, TIMERAD                |
| 20610 | Lamda less than unity                                                | P37                              |
| 21204 | Negative or zero WAITLIST call                                       | WAITLIST                         |
| 21206 | Second job attempts to go to sleep via Keyboard or Display program   | PINBALL                          |
| 21210 | Two programs use device at the same time                             | IMU mode switch                  |
| 21302 | SORT called with negative argument                                   | INTERPRETER                      |
| 21501 | Keyboard and Display alarm during internal use (NVSUB)               | PINBALL                          |
| 21502 | Illegal flashing display                                             | GOPLAY                           |
| 21521 | P01 illegally selected                                               | P01                              |
| 31104 | Delay routine busy                                                   | EXECUTIVE                        |
| 31201 | Executive overflow-No VAC areas available                            | EXECUTIVE                        |
| 31202 | Executive overflow-No core sets available                            | EXECUTIVE                        |
| 31203 | WAITLIST overflow-too many tasks                                     | WAITLIST                         |
| 31211 | Illegal interrupt of Extended Verb                                   | SXTMARK, P23                     |

# Navigation Star Catalog

| Code | Star            | Vis Mag | Asc(h) | Asc(m) | Dec(d) | Dec(m) |
| ---- | --------------- | ------: | -----: | -----: | -----: | -----: |
| 01   | Alpheratz       |     2,1 |      0 |      6 |     28 |     53 |
| 02   | Diphda          |     2,2 |      0 |     42 |    -18 |     11 |
| 03   | Navi            |     2,2 |      0 |     54 |     60 |     27 |
| 04   | Achernar        |     0,6 |      1 |     36 |    -57 |     25 |
| 05   | Polaris         |     2,1 |      1 |     58 |     89 |      6 |
| 06   | Armour          |     3,4 |      2 |     57 |    -40 |     26 |
| 07   | Menkar          |     2,8 |      3 |      0 |      3 |     56 |
| 10   | Mirfak          |     1,9 |      3 |     22 |     49 |     44 |
| 11   | Aldebaran       |     1,1 |      4 |     34 |     16 |     26 |
| 12   | Rigel           |     0,3 |      5 |     12 |     -8 |     15 |
| 13   | Capella         |     0,2 |      5 |     13 |     45 |     57 |
| 14   | Canopus         |    -0,9 |      6 |     23 |    -52 |     40 |
| 15   | Sirius          |    -1,6 |      6 |     44 |    -16 |     40 |
| 16   | Proycon         |     0,5 |      7 |     37 |      5 |     19 |
| 17   | Regor           |     1,9 |      8 |      8 |    -47 |     14 |
| 20   | Donce           |     3,1 |      8 |     50 |     48 |     30 |
| 21   | Alphard         |     2,2 |      9 |     26 |     -8 |     30 |
| 22   | Regulus         |     1,3 |     10 |      6 |     12 |      9 |
| 23   | Denebola        |     2,2 |     11 |     47 |     14 |     46 |
| 24   | Gienah          |     2,8 |     12 |     13 |    -17 |     20 |
| 25   | Acrux           |     1,6 |     12 |     24 |    -62 |     49 |
| 26   | Spica           |     1,2 |     13 |     23 |    -10 |     58 |
| 27   | Alkaid          |     1,9 |     13 |     46 |     49 |     30 |
| 30   | Menkent         |     2,3 |     14 |      4 |    -36 |     11 |
| 31   | Arcturus        |     0,2 |     14 |     14 |     19 |     22 |
| 32   | Aphecca         |     2,3 |     15 |     33 |     26 |     50 |
| 33   | Antares         |     1,2 |     16 |     27 |    -26 |     21 |
| 34   | Atria Austrinus |     1,9 |     16 |     43 |    -68 |     56 |
| 35   | Resalhague      |     2,1 |     17 |     33 |     12 |     35 |
| 36   | Vega            |     0,1 |     18 |     36 |     38 |     45 |
| 37   | Nunki           |     2,1 |     18 |     53 |    -26 |     20 |
| 40   | Altair          |     0,9 |     19 |     49 |      8 |     46 |
| 41   | Dabih           |     3,2 |     20 |     19 |    -14 |     54 |
| 42   | Peacock         |     2,1 |     20 |     23 |    -56 |     51 |
| 43   | Deneb           |     1,3 |     20 |     40 |     45 |      9 |
| 44   | Enig            |     2,5 |     21 |     42 |      9 |     42 |
| 45   | Fomalhaut       |     1,3 |     22 |     56 |    -29 |     49 |

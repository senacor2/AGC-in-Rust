# Preface

This document is a write-up of
[the scanned document on ibiblio](https://www.ibiblio.org/apollo/Documents/SymbolicListingInformation.pdf)
which was semiautomatically converted into markdown to ease processing by AI agents.

The chapters "page layout" and "card layout" are less relevant today as the
[AGC source code on github](https://github.com/chrislgarry/Apollo-11) has a different layout.

*APOLLO GUIDANCE PROGRAM SYMBOLIC LISTING INFORMATION FOR BLOCK 2*

*WORKING PAPER*

# Abstract

The inrforrnation presented in this docrument on the mechanization
of current Block 2 Apollo guidance computer progralns is intended for
use only as an aid to the understarding of guidance program symbolic
listings for both the Command Modu1e and the Lunar Module. The material
which is included is oriented towards permitting a user to understand
the computations being performed by the program, and to follow the logic
associated with the control of a compplete guidance program assembly.

With the aid of the information in this docunent, it should be possible
to be come proficient in determining from a program symbolic listing just
what computations are being carried out. This document, however, is
insufficient to permit a user, without access to supplemental material,
to write a refiable program for the guidance computer.

This document supersedes the Revision I issue of 342O.5-27
(dated 72 June 1968), and includes the updating information that is
sunnarized in Appendix B. Major sections of the document are devoted
to computer hardware infomation, machine language instructions, the
format and features of the assembly program, the interpretive language,
and program performance control. To facilitate using the document for
reference, appendices contain a review of computer concepts, a summary
of computer inputs and outputs, ard alphabetical listings of machine
and interpretive operations, registers and tags, and terms.

This document was produced in order to fill the need for a compilation
of this material for use by those interested in reviewing a
guidance program symbolic listing for Block 2. A number of assumptions
had to be made concerning the operating characteristics of the hardware
and the intended application of the software, and therefore this document
must never be used as definitive information on the guidance computer
hardware or programs. If such information should be required, the G&N
contractor is the proper source for it, not this document. Since this
document has not been approved by NASA, it should never be cited as
a reference in official NASA docunentation.

# Introduction

Under the auspices of TRW Systems MICP task A-201 ("Support of
Apollo Guidance Document Review" J, Garman and J.E. Wi1liams, FS5,
task monitors), inforrmation on certain of the hardware and software
aspects of an Apo11o Block 2 primary guidance system program symbolic
listing has been assembled into this document. The purpose of this
effort was to facilitate the review of B1ock 2 guidance progran symbolic
Iistings by those unfamiliar with the Apollo guidance computer program
listing format. This document is Revision 2 of an earlier document on
the same subject ( originally published on 1O January 1967, with Revision
1 published on 27 June 1968), and completely supersedes these previous
documents, Appendix B summarizes some of the significant changes made
since the Rerision 1 issue.

Several different sources of information were used during the
preparation of this document, including:

1. A program assembl-y listing bearing the heading print:

```
GAP: ASSEMBI,E RXWSION 072 OF AGC PROGRAM COMANCHE
BY NASA 2021113-071 18:53 OCT. 17,1969
```

"COMANCHE" is the term used for the COLOSSUS (manned CSM earth/
lunar capability) 2x series of programs: this version is intended
for use on Apollo 13, and is also referred to as COLOSSUS 2D.

2. A program assembly listing bearing the heading print:

```
GAP: ASSEMBLE REVISION 130 of AGC PROGRAM LUMINARY
BY NASA 2021112-081 18:29 NOV. 4, 1969
```

"LUMINARY" is the term used for the manned LM earth/lunar
capability programs: this version is intended for use on
Apollo 13, and is also referred to as LUMINARY 1C.

3. Raytheon Apollo Guidance Computer Information Series publications,
used for much of the hardware information. Two separate
documents were employed, one identified as "Issue 10, Block II
Apollo Guidance Computer Subsystern, FR-2-130", updated 25
February 1966; the other was "Issue 32, Block II Machine
Instructions, FR-2-132", updated 25 March 1966.

4. MSC LM G&C Data Book, Revision 2, dated 15 July 1967.

5. "Apollo Operations Handbook for CSM, Volume 2 for Apollo 12, 
CSM 108 and Subsequent", dated 10 October 1969.

6. AGC4 Memo #9, "Block II Instructions", revised 1 June 1967

7. A GAP assembly program listing dated 17 May 1968.

8. A number of other miscellaneous sources of information, such as
other program listings, G&N contractor documentation, etc.

It should be clearly understood that the information in this document
was derived from program symbolic listings, and hence cannot be considered
to be an "independent source", particularly when matters such as signal
polarities and channel-bit assignments are considered. In addition,
changes to some of the material presented in this document may well
take place in the future (such as the addition of a "rate-sided optics"
capability to the CSM), so that where applicable information should be
checked against mission-peculiar data prepared based on a specific
program assembly.

The material on the following pages should _under no circumstances_
be employed as a definitive description of the guidance hardware or
portions of the guidance software. Such infomation, including that
necessary to write (as contrasted with read) guidance computer programs,
should be Drovided only by the G&N contractor through the appropriate
MSC channels.

## Notation

The notation employed in this documnent is intended to be consistent
with that employed in the previous two issues, as well as with documents
which have been produced on the AS-202, AS-204, LM-1, Sundisk, and
Colossus programmed guidance equations. For convenience, some of this
specialized notation is summarized below (specialized noiation is also
defined in individual sections to uhich it applies, such as the Section
for machine language order codes).

1. Unless otherwise specified, Information applies to both the
Command Module (CM) and Lunar Module (LM) computers. These
abbreviations are used when it is necessary to cite hardware
or softuare differences betv,reen the two systems.

2. Unless otherwise specified, material which is provided is
intended. to be consisient with sources #1 and #2 at the beginning of this document.
Material applicable to earlier programs, but no longer va1id,
is cited only when of historical or potential future application.

3. Bits are numbered from #15 (the sign bit) and #14 (the most
significant magnitude bit) down to bit #1 (the least significant
bit of the 15-bit conputer word).

4. A quantity in capital letters, unless an operation code,
generally means the contents of a cell with that tag. The
capital letter E, with subscripts of quantities in capital
letters, is reserved to mean "the contents of the cell or
cells whose tags are in the subscript." Hence E~TS~, for
example, would be the contents of the cell whose address is
stored in cell TS. TS alone, of course, would be the contents
of the cell with tag TS.

5. A quantity in quotation marks indicates that its address is
of interest rather than the contents of that address. No
quotation marks are used, however, unless the quantity corresponds
to a tagged program step: transfers in Section VIB to
other interpretive operations are indicated without quotation
marks.

6. Iogical branches in the software are indicated by "If"
statements, with subsequent equation information indented to
indicate the extent of the computations performed should the
"If" condition be satisfied.

7. Unless otherwise specified, numbers are given in decimal.
The subscript 8 signifies an octal nunber and the subscript
2 signifies a binary number. Where conventional reference
to quantities (such as channel numbers) considers them to be
octal, however, the subscript has been omitted.

8. The equation X = ABCD +2 means that X is set to the contents
of the address with tag "ABCD" plus 2; X. : ABCD+2, however,
omitting the space before the sign, means that the cell address
used is the ce11 two memory locations beyond "ABCD", If
variables are used as subscripts, horaever, the first meaning
always applies.

9. Perform "XXXX" means transfer to the routine starting at
"XXXX" and retain return address information (to permit
return after completion of the routine); Proceed to "XXXX",
however, merely means transfer to routine starting at "XXXX".

10. The scale factor of a quantity is the power of two by which
the number in the computer (considered as a fraction in the
range between -1 and +1) must be multiplied to obtain its true
va1ue. The scale factor is frequently shown as Bxx, to
signify binary as opposed to decimal exponent information.
See Appendix A for more details.

11. The equation "Set AA = BB and BB = AA" means to exchange the
contents of the cells with addresses "AA" and "BB".

12. The subscripts dp, tp, vc, x, y and z mean double precision,
triple precision, vector and vector x-z components respectively.

13. The equation B = (b~1, b~2~) means form a double precision number
B with most significant halb b~1~and least significant half b~2~.
The equation A = (a~1~, a~2~, a~3~) means form a vector from the
indicated components.

14. A task (short sequence of computations based upon some time or
event criterion) may be entered. into a list for subsequent
performance in yy seconds (see Section VIIA). The notation
for this is: Call "XXXX" in yy seconds, where "XXXX" is the
starting address of the waitlist task.

15. A job (computation that is not a task) may be entered into a
list for performance when its priority is high enough (see
Section VIIB). The notation for this is: Establish "XXXX".

16. A bit is "set" when its value is made a binary one, and is
"cleared" or "reset" when its value is made a binary zero.
"Set" is also used to mean "force bit value to be as specified".

17. In some cases an address may be determined from the combination
of a bank register and a quantity giving an address within the
bank (see Section IIB). In such cases, the bank setting is
sometimes shown explicitly and the address in the bark used as
described in #4 above.

18. The operation sgn X causes the quantity it affects to be
complemented if X is negative (same as multiplication by
the quantity X / |X|).

19. Where reasonably appearent from the context, explicit scaling
is not shown in the equations (and can be assumed to be proper).
For example, MPAC~tp~= MPAC~tp~ + MPAC+2 is frequently
used for rounding to double precision. MPAC+2 is added to
itself and carries propagated.

20. In some cases, loading of registers (such as memory-control
cells) that comprise fewer than the 15 bits of the nonnal
computer word lengih is indicated as if the loading were
accomplished by a mask-type procedure, in order to demonstrate
the function of indirvidual bits in a word. If hardware design
changes were to occur, of course, the indicated masking would
no longer be applicable.

# II Computer Hardware Information

## IIA General Data

The Apollo Block 2 primary guidance computer may be classified as
a general-purpose parallel operation binary computer. Various details
of its hardware, necessary for proper interpretation of its programs,
are given in Section II on the following pages. These details may be
summarized to be:

- Number System: Fractional binary, with negative numbers generally
in ones complement form. Numbers in arithmetic unit operated
on in parallel. Angle information is in twos complement form.

- Word Length: Sign and 14 magnitude bits of information. Words
stored in memory have a sixteenth bit for parity purposes;
in the arithmetic unit, the sixteenth bit is used for overflow
detection. A limited number of double precision operations
are included in the order code.

- Error Detection: Odd parity for all cells read from memory.

- Erasable Memory: Randim access coincident-current ferrite core,
destructive readout. Total capacity is 2040 cells, of which 12
are allocated to special functions and 29 to counters.
There are 760 cells which are uniquely addressable (including
the special function and counter cells); the remaining 1280
cells are addressed (in modules of 256) with erasable memory
bank register.

- Fixed Memory: Random access core rope, non-destructive readout,
Total capacity is 36,864 cells, of which 2048 are uniquely
addressable; 22,528 addressed (in modules of 1024) with
fixed memory bank register; and 12,288 addressed with both
fixed memory bank register and an additional register.

- Instruction Format: Three to six bits for operation code, remaining
bits for address.

- Hardware Registers: Total of 26 may be addressed, of which four
are associated with arithmetic unit; four with memory control;
ten with computer outputs (channels); and 8 with computer
inputs (channels).

- Operation Codes:

  - 15 regular machine-language
  - 19 "extended" machine language (requires 2 orders)
  - 4 "special" machine language (hardware functions)
  - 4 shift-register cells for bit shifts.
  - 7 "involuntary" for counter operations.
  - 2 "involuntary" for program interrupts (one can be programmed).
  - 5 "peripheral" for test equipment interface.

- Interrupts:

  - 29 for counter control.
  - 11 for program control.

- Speed:

  - 23.4 microseconds for single precision addition-type orders.
  - 46.9 microseconds for multiplication (net).
  - 82.0 microseconds for division (net).

- Hardware:

  - About 70 pounds weight.
  - About one cubic foot volume.
  - About 100 watts power in operation, 10 watts in standby.

The basic source of all timing for the computer is an oscillator
which operates at a frequency of 2.048 mc. The output is divided by two
to obtain the computer logic-control-pulse rate of 1.024 mc (or a period
of 0.9765625 microseconds). A set of computer logic-control-pulses
that occur simultaneously is termed an "action": 12 actions make up
(usually) a "subinstruction", which takes place in 11.71875 microseconds.
Since this is the time for complete memory cycle, the time interval is
called a "memory Cycle Time", or "MCT". All instructions take an
integral number of MCT's to perform.

The computer clock output of 1,024 mc is applied to a ring counter,
which gives an output after dividing by ten at 102.4 kc. This signal
is applied to a 33-stage binary counter, whose various output frequencies
(3300 pps, 100 pps, 0.78125 pps, etc.), both in phase and out of phase,
are used to provide various timing signals for computer functions. In
addition the most significant 28 stages of this counter are available
as input channels 03 and 04, and can be used to permit resoration of the
computer erasable memory record of time since launch after a period of
low power (standby) operation.

An odd parity bit, making the sum of binary ones in the memory cell,
including itself, an odd value, is included with all fixed memory cells,
and is generated when erasable memory cells are loaded. Readout of a
memory cell is accompanied by an automatic check for the validity of the
parity bit, and a hardware restart is generated if the parity bit is
determined to be inconsistent with the word. Because of this parity
check, the existence of an odd number of errors (1, 3, 5, etc.) in
the information read from memory can be detected, including cases where
all 16 bits are zero or one.

## IIB Address Allocation

The character of the address allocation problem in the computer can
be described by first considering the hardware which would be necessary
to be able to use any instruction with any address. Excluding the
special-purpose channel instructions, there are 27 instructions in the
computer machine language repertoire, would would require five bits
for representation (2^5^ = 32). There are also 38,912 addressable cells
in the computer (36,864 fixed remory, 2040 erasable nemory, and 8
non-channel hardware registers), requiring 16 bits for representation
2^16^ = 65,536). This would give a total requirement for 21 bits in
the instruction word length, or six bits more than the actual hardware
instruction word length of 15 bits (plus the 16th odd-parity bit).

In order to obtain the necessary hardware (and software) effect of
the "missing" bits at a monimum penalty, the following special design
features are found in the computer logic:

1. Instructions which are used comparatively infrequently (such as
multiply and divide) require two lines of coding, with the first
line setting an "extended-order" flip-flop (which is reset after
the order is performed),

2. Several instructions can be used with only one tyrpe of memory:
most transfer orders, for example, can refer only to addresses
in fixed memory, and instructions which load a memory cell can
refer only to addresses in erasable nemory. In addition, the
computer digital (as contrasted with analog-type pulse input/output)
input/output information is handled through "channels",
which can be addressed only by a special group of instructions
intended solely for that purpcse,

3. The erasable memory cells are divided into eight "banks" of
256 cells each, with banks O-2 ("unswitched erasable") addressable
directly (bank 0 includes the 8 non-channel hardware registers),
and the remaining banks, 3-7 , selected with the aid of a three-
bit "EBANK" register (cell OOO3~8~). These non-uniquely addressed
banks are referred to as "switched erasable": the cell within the
bank, of course, is determined by the address portion of the
instruction word.

4. The fixed memory cells are afso divided into "banks", but these
have a capacity of 1024 cells each. Two of the banks, O2 and
O3, are addressable directly, and hence are known as "fixed-fixed"
memory. Of the remaining 34 banks, 22 are selected with
the aid of a five-bit "FBANK" register (cell OOO4~8~), and
comprise banks OO, O1, and O4-27 (by convention, banks are
considered to be octal qualtities), and are "variable-fixed" memory.
The other 12 banks in fixed memory, banks 30-43, are considered
to be "superbanks", since their selection also depends on the
setting of channel 7 (SUPERBNK). The three bits of this channel
can be considered an, "extension" to the FBANK capacity (hence
the channel is sometimes referred to as "F EXT"), with a setting
of O-3 selecting superbank S3 (banks 30-37) and a setting of 4
selecting superbank S4 (banks 40-43). The other 4 banks for
superbank 4, plus those for superbanks 5-7, are not presently
included in the compuier hardware. The cell within the superbank,
of course, us determined by the FBANK setting and by the
address portion of the instruction word.

The logical design of the computer includes a twelve-bit memory
address register ("S-register") which, together with suitable EBANK or
FBANK and SUPERBNK information if necessary, is used to specify memory
cell locations within the computer. The S-register is not necessarily
loaded with bites 12-1 of the instructiuon word, however, since some
instructions use these bits to help determine the operation code.

### Computer Memory Address Allocation

| "True" Address | S-Register | EBANK | FBANK | SUPERBNK | Function |
| --- | --- | --- | --- | --- | --- |
| 00000-00007 | 0000-0007 | - | - | - | Non-channel hardware cells |
|             | 1400-1407 | 0 | - | 0 | |
| 00010-00060 | 0010-0060 | - | - | - | Special erasable cells |
|             | 1410-1460 | 0 | - | - | |
| 00061-00377 | 0061-0377 | - | - | - | Bank 0 of Unswitched Erasable |
|             | 1461-1777 | 0 | - | - | |
| 00400-00777 | 0400-0777 | - | - | - | Bank 1 of Unswitched Erasable |
|             | 1400-1777 | 1 | - | - | |
| 01000-01377 | 1000-1377 | - | - | - | Bank 2 of Unswitched Erasable |
|             | 1400-1777 | 2 | - | - | |
| 01400-01777 | 1400-1777 | 3 | - | - | Bank 3 of Switched Erasable |
| 02000-02377 | 1400-1777 | 4 | - | - | Bank 4 of Switched Erasable |
| 02400-03777 | 1400-1777 | 5-7 | - | - | Bank 5-7 of Switched Erasable |
| 04000-05777 | 4000-5777 | - | - | - | Bank 02 of Fixed-fixed Memory |
|             | 2000-3777 | - | 02 | - | |
| 06000-07777 | 6000-7777 | - | - | - | Bank 03 of Fixed-fixed Memory |
|             | 2000-3777 | - | 03 | - | |
| 10000-13777 | 2000-3777 | - | 00-01 | - | Banks 00-01 of Variable-fixed. (Cell conversion 00000-03777) |
| 20000-67777 | 2000-3777 | - | 04-27 | - | Banks 04-27 of Variable-fixed. (Cell conversion 10000-57777) |
| 70000-107777 | 2000-3777 | - | 30-37 | <= 3 | Superbank S3, banks 30-37. (Cell conversion 60000-77777) |
| 110000-117777 | 2000-3777 | - | 30-33 | 4 | Superbank S4, banks 40-43. (Cell conversion 100000-107777) |

The addresses in the computer are allocated as shown on the previous table
(all numerical quantities are given in octal). As can be seen from the table,
the following general rules apply for selection of cells within the computer.

1. If bits 12-11 of the S-register are both zero (S-register in the range 0000-1777),
then the erasable memory (or non-channel hardware cells) is read. If however, one
or both of these bits are one (S-register in range 2000-7777), then the fixed
memory is read.

2. The contents of EBANK influence the address which is selected
if bits 12-11 of the S-register are both zero and bits 10-9 are
both one (S-register in range 1400-1777, giving erasable memory
bank selection capability of 256 cells).

3. The contents of FBANK influence the address which is selected
if bit 12 of the S-register is zero and bit 11 is one (S-register
in range 2000-3777, giving fixed memory bank selection capability
of 1024 cells).

4. The contents of SUPERBNK influence the address which is
selected if the most significant two bits of FBANK are both
one (FBANK in range 30-37) and if S-register is in range
2000-3777. Note that values of SUPERBNK between 0 and 3 will
all select banks 30-37, contrary to analogous options
for EBANK and FBANK (which have non-redundant cell selections).

The quantity listed in the previous table as "true" address is
used for assembler purposes (to specify the absolute starting address
for a set of computations). In order to convert an erasable memory
"true" address to hardware cell contents, the following process can
be used:

```
S-register = 1400~8~+ bits 8-1 of "true" address
EBANK = bits 11-9 of "true" address
```

The "true" address is the one specified when external inputs that require
specification of absolute cell locations are required (such as for certain
uplink sequences and for address-to-be-specified inputs to the display
system). For programming convenience, the three bits of EBANK are
connected to bits 11-9 of the computer hardware accumulator.

In order to convert a fixed memory "true" address to hardware cell
contents, first subtract 10000~8~ if the "true" address is above that value:
the result of such a subtraction, identified as "cell conversion", is shown
in the Function column of the table. Starting with this "fixed" address,
the following process can then be used to determine address selection
parameters:

```
S-register = 2000~8~ + bits 10-1 of "fixed" address
SUPERBNK = bits 16-14 of "fixed" address (values of 0-3 the same for hardware)
FBANK = bits 15-11 of "fixed" address for SUPERBNK <= 3
FBANK = 30~8~+ bits 13-11 of "fixed" address for SUPERBNK > 3
```

For programming convenience, the five bits of FBANK are connected to
bits 15-11 of the computer hardware accumulator. In addition, the cell
BBANK (address 0006) has both FBANK (bits 15-11) and EBANK (in bits 3-1)
connected to it, and hence can be used to sample or load both bank registers,
provided the loading information is in the proper format. SUPERBNK is
connected to bits 7-5 of the computer hardware accumulator, but it must
be loaded by a channel order (note, however, that the bits for SUPERBNK
are compatible with the assigned bits in BBANK, permitting one 15-bit
computer word to have suitably formatted information for all three quantities).

## IIC Hardware Registers

There are eight non-channel hardware cells in the guidance computer,
with addresses 0000~8~- 0007~8~. These cells are described
below (see Section IIE for the computer channels).

| Address | Symbol | Function |
| --- | --- | --- |
| 0000~8~ | A | Accumulator. Most instructions refer to, or modify the contents of A. See Section IV. |
| 0001~8~ | L | L register or Low Order Accumulator. Use to contain the least significant half of double precision words for those operations which use, or generate, such words, and to contain the remainder after division. Cell also forms channel 01, permitting channel operations to be used for but manipulation purposes, when symbol conventionally is LCHAN. Cell frequently used for temporary storage purposes within a computation |
| 0002~8~ | Q | Q register, or Return Address Register. Loaded with the value of the program count (cell 0005~8~) of the step following a TC (transfer control, see Section IVB) instruction, thus retaining return address information. Cell also forms channel 02, permitting channel operations to be used for bit manipulation purposes, when symbol conventionally is QCHAN. Cell frequently used for temporary storage purposes within a computation. |
| 0003~8~ | EBANK | Erasable Memory Bank Selector, consisting only of bits 11-9 (which are also connected to bits 3-1 of cell 0006~8~). Contents used to specify which bank of 256 erasable memory cells (bitss 8-1 of S-register) is to be selected for S-register in the range 1400~8~ - 1777~8~. See Section IIB. |
| 0004~8~ | FBANK | Fixed Memory Bank Selector, consisting only of bits 15-11 (which are also connected to bits 15-11 of cell 0006~8~). Contents used to specify which bank of 1024 fixed memory cells (bits 10-1 of S-register) is to be selected for S-register in the range 2000~8~- 3777~8~. See Section IIB. |
| 0005~8~ | Z | Z register or Program Counter. Contains the address of the next step, and for most instructions it is incremented by 1 under hardware control. Can be loaded directly by program (pseudo operations DTCB and DTCF) in order to accomplish transfer of program control. Incrementing takes place as part of termination of previous instruction (so that direct loading of register 0002~8~from Z for a TC order achieves the desired effect). |
| 0006~8~ | BBANK | Both Banks, a cell which may be used if reference for reading or writing to both EBANK and FBANK is desired. The three bits (11-9) of EBANK are connected to bits 3-1 of BBANK while the five bits 15-11 of FBANK are connected to bits 15-11 of BBANK. The SUPERBNK bits (7-5) are not connected to BBANK, but instead must be processed by a separate channel instruction (referencing channel 7). |
| 0007~8~ | - | Adress which may be used as a source of 0000~8~ for clearing instructions. (such as the pseudo operations ZL and ZQ). No physical register corresponds to this address, so that "loading" of the address has no effect, and hence can be used to achieve desired program performance (such as modification of the A register via TS order, Section IVB). An attempt to read an unwired fixed memory cell to obtain 0 would give a hardware restart due to parity failure. |

## IID Special Erasable Cells

The first 41 cells of the erasable memory (locations 0010~8~- 0060~8)
are nominamlly allocated to special functions, with the last 29 (starting
at location 0024~8~) being used for counter purposes and the first 12 for
other specialized purposes (although some function as normal erasable
memory cells). In addition cell 0067~8~serves a special hardware function
in monitoring for a program loop and initiating a hardware restart if
one is detected. These cells and ther functions are described below.

| Address | Symbol | Function |
| --- | --- | --- |
| 0010~8~ | ARUPT | Normal erasable memory cell used by convention to contain the contents of the accumulator (A register) after program interrupts #1 - #10 (see Section IIH) acted upon, and used to restore these contents before resuming the interrupted computation |
| 0011~8~ | LRUPT | Normal erasable memory cell used by convention to contain the contents of the L register after program interrupts #1 - #10 acted upon, and used to restore these contents before resuming the interrupted computation |
| 0012~8~ | QRUPT | Normal erasable memory cell used by convention to contain the contents of the Q register (if the contents would be modified during the computations associated with the interrupt) after program interrupts #12 - #10 acted upon, and used to restore these contents before resuming the interrupted computation (if, of course QRUPT loaded). |
| 0013~8~ 0014~8~ | SAMPTIME | Normal erasable memory cells used to retain the value of cells 0024~8~ - 0025~8~ when certain program steps are performed (e.g. steps for program interrupts #5 - #7), for subsequent possible display. |
| 0015~8~ | ZRUPT | Cell used to contain the value of the program counter (Z register) when a program interrupt acted upon. It is usually loaded and restored to Z by hardware means, although it can also be sensed and stored by software |
| 0016~8~ | BANKRUPT | Normal erasable memory cell used by convention to contain the contents of BBANK (if these contens would be modified during the computations associated with the interrupt) after program interrupts #1 - #10 acted upon, and used to restore these contents before resuming the interrupted computation (if, of course BANKRUPT loaded). For those interrupts changing SUPERBNK, BANKRUPT also used to retain the SUPERBNK of the interrupted computation, thus requiring special restoration coding. |
| 0017~8~ | BRUPT | Cell used to contain the value of the nonaddressable "B-register" (buffer register, used to contain the next instruction) when a program interrupt is acted upon. It is noramlly loaded and restored to B by hardware means, although it can also be sensed and stored by software. Loading the cell with a certain program (Z register) count and then executing RESUME will cause program to start at indicated step, since the TC (transfer control) order has operation code = 0. |
| 0020~8~ | CYR | Cycle register. When the contents of the cell are written into, either as part of the original loading or as a result of most sensing operations (such as CA, Clear Add), they are shifted right one place in a cyclic fashion: bit 15 becomes bit 14, bit 14 becomes bit 13, ... bit 2 becomes bit 1, and bit 1 becomes bit 15. The unshifted value (except as it is shifted from a previous loading) is the one sensed. Shifting does not take place for the MASK, MP (Multiply), or DV (Divide) orders. See Section IV. |
| 0021~8~ | SR | Shift right register. When the contents of the cell are written into, either as part of the original loading or as a result of most sensing operations, they are shifted right one place in a non-cyclic fashion: bit 15 becomes bit 15 and bit 14, bit 14 becomes bit 13, ... bit 2 becomes bit 1, and bit 1 is lost. The unshifted value (except as it is shifted from a previous loading) is the one sensed. Shifting does not take place for the MASK, MP, or DV orders. See Section IV. |
| 0022~8~ | CYL | Cycle left register. When the contents of the cell are written into, either as part of the original loading or as a result of most sensing operations, they are shifted left one place in a cyclic fashion: bit 1 becomes bit 2, bit 2 becomes bit 3, ... bit 14 becomes bit 15, and bit 15 becomes bit 1. The unshifted value (except as it is shifted from a previous loading) is the one sensed. Shifting does not take place for the MASK, MP, or DV orders. See Section IV. The effect of a shift left in a non-cyclic fashion (except for bit 15) can be achied by addition of accumulator to itself the proper number of times. |
| 0023~8~ | EDOP | Edit operand register. When the contents of the cell are written into, either as part of the original loading or as a result of most sensing operations, bits 14-8 are loaded into bits 7-1 respectively, and bits 15 and 14-8 are set 0. The unshifted value (except as it is shifted from a previous loading) is the one sensed. Shifting does not take place for the MASK, MP, or DV orders. See Section IV. The right shift of 7 places for the selected bits is used for interpreter and verb/noun pattern editing operations |
| 0024~8~ | TIME2 | Cell used as the most significant half of the computer "clock", preset and sensed under program control. It is incremented by +1 when cell 0025~8~ overflows. TIME2 overflows every 745 hours (i.e. 31 days 1 hour), 39 minutes, 14.56 seconds, and is conventionally reset when liftoff is deduced so as to indicate mission time elapsed. |
| 0025~8~ | TIME1 | Cell used as the least significant half of the computer "clock", preset and sensed under program control. It is incremented by +1 each 0.01 second (i.e.e each centi-second). When the cell overflows (each 163.84 seconds), TIME2 is incremented by +1. See Section IIE for phasing with respect to Channel 04 time information. |
| 0026~8~ | TIME3 | Cell used to generate (when overflow takes place) program interrupt #3 (conventionally used for "waitlist" tasks, see Section IIH). Preset to appropriate value under program control (i.e. 2^14^ - required delay in centi-seconds), and incremented by +1 each 0.01 second. See Section VIIA for computations associated with determining proper settings. |
| 0027~8~ | TIME4 | Cell used to generate (when overflow takes place) program interrupt #4 (conventionally used for periodic "T4RUPT" input/output functions, see Section IIH). Preset to appropriate value under program control (i.e. 2^14^ - required delay in centi-seconds), and incremented by +1 each 0.01 second. Incrementing phased so as to take place 0.0075 second after the TIME3 increment. |
| 0030~8~ | TIME5 | Cell used to generate (when overflow takes place) program interrupt #2 (conventionally used for computations associated with the digital autopilots, see Section IIH). Preset to appropriate value under program control (i.e. 2^14^ - required delay in centi-seconds), and incremented by +1 each 0.01 second. |
| 0031~8~ | TIME6 | Cell used to generated (after has been decremented to -0) program interrupt #1 (conventionally used for timing of RCS jet commands in output channels 05 and 06 from the digital autopilots, see Section IIH). Preset to appropriate value under program control (i.e. required delay in units of 2^-4^ centi-seconds), and decremented by 1 each 0.000625 second (i.e. at a 1600 pps rate) provided bit 15 of channel 13 is 1. When the counter reaches a value of -0, the next DINC pulse causes bit 15 of channel 13 to be set 0 and the program interrupt to be generated. |
| 0032~8~ 0033~8~ 0034~8~ | CDUX CDUY CDUZ | Cells accumulating the output pusles from the three CDU's (Coupling Data Units) associated with the IMU (Inertial Measurement Unit), to provide information on the IMU gimbal angles (and hence on spacecraft attitude). Cells preset and sensed under program control, providing angle information in twos complement form, scale factor B-1 revolutions. One pulse, therefore, is 2^-15^ revolutions, equivalent to 39.55978125 arc sec (about 0.01098633°).
| 0035~8~ (CM) | CDUT | Cell accumulating the output pulses from the CM optics trunnion CDU, to provide information on optics trunnion angle. Cell preset are sensed under program control, with optics zeroing (bit 1 of channel 12) giving a setting of 61740~8~(about -19.7754^o^ in twos complement) for the cell. Scale factor B-3 revolutions, with data in tows complement. Zeroing points about 32° 31' 23.19" away from Z axis (towards X axis). |
| 0035~8~ (LM) | CDUT | Cell accumulating the output pusles from the LM rendezvous radar trunnion CDU, to provide information on rendezvous radar trunnion angle. Cell preset and sensed under program control, with radar CDU zeroing (bit 1 of channel 12) giving a setting of 0 for the cell (due to software), and also inhibiting cell increments. Scale factor B-1 revolutions, in twos complement. |
| 0036~8~ (CM) | CDUS | Cell accumulating the output pulses from the CM optics shaft CDU, to provide information on optics shaft angle. Cell preset and sensed under program control, with optics zeroing (bit 1 of channel 12) giving a setting of 0 for the cell. Scale factor B-1 revolutions, with data in twos complement. |
| 0036~8~ (LM) | CDUS | Cell accumulating the output pulses from the LM rendezvous radar shaft CDU, to provide information on rendezvous radar shaft angle. Cell preset and sensed under program control, with radar CDU zeroing (bit 1 of channel 12) giving a setting of 0 for the cell (due to software), and also inhibiting cell increments. Scale factor B-1 revolutions, in twos complement. |
| 0037~8~ 0040~8~ 0041~8~ | PIPAX PIPAY PIPAZ | Cells accumulating the output pulses from the three PIPA's (Pulsed Integrated Pendulous Accelerometers) associated with the IMU, to provide information on sensed velocity increments in IMU coordinates. Cells preset and sensed under program control, providing information with scale factor B14 counts. Nominal CM scale factor is 5.85cm/sec per count; nominal LM scale factor is 1.00 cm/sec per count. When the software resets the cells, the accelerometer electronics is not affected, so that fractional counts accumulated there would not be disturbed. |
| 0042~8~ 0043~8~ 0044~8~ (CM) | BMAGX BMAGY BMAGZ | Not used. Originally intended to provide an accumulation of angle increment data from the Gyro Display Coupler of the Spacecraft Control System BMAG's (Body Mounted Attitute Gyros), to serve as a backup source of attitude information in the event of IMU failure. Inputs to cells enabled if bit 8 of channel 13 is set. Cells preset and sensed under program control. |
| 0042~8~ 0043~8~ 0044~8~ (LM) | Q-RHCCTR P-RHCCTR R-RHCCTR | Cells accumulating the output pulses from the RHX (Rotational Hand Controller) pitch, yaw and roll axes respectively, used if the RHC is employed as a rate commanding device. If RHX used as a minimum impulse of landing point designator device, however, bits 6-1 of channel 31 used instead to determine the status of the controller. Inputs to counters enabled if bit 8 of channel 13 is set, and counters must be reset to 0 under program control. Bit 9 of channel 13 is used to cause a readout of the RHC analog-to-digital converters to be started, and then becomes reset. Separate sign and magnitude information is received from the converter, with magnitude provided by width of a dc pulse (which gates a 3200pps signal to the counter for digital conversion). Full-scale deflection (into soft stops) of RHC provides an input count of 42, scale factor B14 counts, with corresponding value of ratecommand determined by software. Software does not enable counting unless bit 15 of channel 31 indicates that RHC is out of detent (giving a minimum control capability of about 10% of full scale). Also known in LM as "ACA" (for Attitude Controller Assembly). |
| 0045~8~| INLINK | Cell into which serial binary data is shifted from the uplink receiver (after completion of the checks by the receiver for satisfactory message format) one bit at a time under hardware control. The overflow of this cell (implemented by having the first of the 16 bits sent to the computer to be a binary 1) causes program interrupt #7 (see Section IIH). Cell must be reset to 0 by software to permit the next word to be processed properly. Software performs additional checks on the 15-bit work read from the cell: bits 5-1 are checked to be the same as bits 15-11, and the same as the complement of bits 10-6, before processing the input further (using the five-bit codes listed in Section IIJ, the same as for DSKY inputs). If failure of the software check is encountered, all subsequent inputs are rejected by the software until an error reset pattern (22~8~) is received via the upling (not DSKY). No inputs to the cell are made if bit 6 of channel 13 is 1, nor if either of the spacecraft switches (the CM has two) are set to block uplink inputs (cf. bit 10 of channel 33). In addition an incoming bit is rejected if a 6400 pps signal has not occurred since the previous bit was accepted, and bit 11 of channel 33 (a flip-flop) is set to be sensed as a binary 0 to indicate such a rejection. Checks for too rapid an uplink rate made only if bit 5 of channel 13 is 0 and if switches set to accept uplink. Bit 5 of channel 13 can be set 1 to select output of "crosslink" hardware (cf. cell 0057~8~) instead of the uplink receiver for cell 0045~8~ input, but this capability is not used. The spacecraft switches cannot be set to block crosslink inputs from cell 0045~8~, although the same monitoring for too fast an input rate is made. |
| 0046~8~ | RNRAD | Cell into which VHF range data (for CM) or landing radar data (velocity and altitude) and rendezvous radar data (range and range rate) for LM is shifted unter control of bits 5-1 of channel 13. The rendezvous radar angle data is in cells 0035~8~ - 0036~8~. The source and type of data is selected by bits 3-1 of channel 13, and when bit4 of channel 13 becomes 1 the readout process is started, being terminated 90-100 ms later by the generation of program interrupt #9 and the resetting of bit 4 of channel 13. All 15 bits loaded are magnitude bits. When the first 100 pps signal after bit 4 of channel 13 becomes 1 occurs, a 3200 pps pulse train is generated on an appropriate computer output line (depending on the selection made by bits 3-1 of channel 13). This pulse train, which lasts for about 80 ms, is used by the radar to gate the selected data into a radar counter. About 5 ms after the termination of this pulse train, 15 sync pulses (on a separate line from the data gating pulses) are sent, again at 3200 pps rate, to shift data from the counter to cell 0046~8~. Improper shifting results if these sync pulses are not of the proper waveform (due to a channel 13 loading command, for example). After the last sync pulse (or 10 ms after the end of the measurement pulse train), program interrupt #9 is generated. For the CM the least increment on the quantity loaded into cell 0046~8~is about 0.01 nm (18.52 meters). For the LM, the landing radar measurement is made for about 80.001 ms with a 164.6 kc bias on rates (bias count of 12288.2), and least increments of about -0.6440, 1.212 and 0.8668 fps/bit vor x-z velocities. On the low range LR scale (cf. bit 9 of channel 33), least increment is about 1.079 feet/bit (high range 5.000 times bigger). For rendezvous radar, range on low scale (cf. bit 3 of channel 33) about 9.38 feet/bit; high range 8.000 times bigger. Range rate (counts for 80 ms) bias frequency is 212.5 kc (17000 bias count), and scale -0.6278 fps/bit. |
| 0047~8~ | GYROCMD | Cell which is loaded with the magnitude of the required IMU gyro torquing command, scale factor B14, units counts (one count is 2^-21^ revolution or about 0.61798096 arc seconds). Output pulses are generated at a 3200 pps rate, with power supply for them enabled by bit 6 of channel 14 and the sign and axis of the gyro to be torqued specified by bits 9-7 of channel 14. WHen bit 10 of channel 14 is set, the pulses are started (and GYROCMD decremented appropriately). When GYROCMD reaches zero, the pulses are terminated and bit 10 of channel 14 reset. |
| 0050~8~ 0051~8~ 0052~8~ | CDUXCMD CDUYCMD CDUZCMD | Cells loaded with values transmitted to IMU CDU error counters. Information gated out of the cells if bits 15-13 (respectively) of channel 14 are set, and error counters loaded if bit 6 of channel 12 is set. These "error counters" should be considered as being in large measure independent of the "CDU" information in cells 003~8~ - 0034~8~, and essentially serve the purpose of digital-to-analog converters. The error counters saturate at a count of 600~8~ (or 384 counts), and are incremented at a 3200 pps rate for a count determined by their respective CDUiCMD cell. If bit 4 of channel 12 is set, the error counter data is used for coarse align of the IMU (and the count in the error counter decremented in magniture as the IMU alignment proceeds). The error counters associated with all 3 cells are reset 0 if bit 6 of channel 12 is reset to 0. The scale factor of the cells for IMU coarse align is B1 revolutions (so that one pulse corresponds to 2^-13^ revolutions or about 158.2 arc seconds). See next paragraph for additional CM-only and LM-only uses. |
| 0050~8~ 0051~8~ 0052~8~ (CM) | | See previous paragraph for items common to CM and LM uses. If bit 9 of channel 12 is set, error counter output (converted to dc) is used for roll, pitch, and yaw control of the Saturn. Error counter output also used for roll, pitch, and yaw attitude error displays respectively on FDAI (Flight Director Attitude Indicator). Software loads cells with data scaled B1 revolutions (saturated error counter = 16.875°) except for roll during boost and entry, when scale factor is B3 revolutions (saturation = 67.5° for 384 counts into error counter). Actual display scale determined by spacecraft FDAI SCALE switch (which is not sensed by software): for ERR scale at "5", full scale is 5° for B1 rev. scaling for "51/15", full scale is 50° in roll for B3 scaling (12.5° for B1 scaling), and 15° in pitch/yaw (B1 scaling). |
| 0050~8~ 0051~8~ 0052~8~ (LM) | See paragraph before the previous paragraph for items common to CM and LM uses. Error counter output also used for LM P, Q, and R axes (yaw, pitch, and roll respectively) attitude "error" needles on FDAI: note that in LM vehicle rotation about thrust vector is "yaw" (in CSM it is "roll"). Software controls whether cells loaded with attitude error information (scaled B0 in units of 1800°) or vehicle rate information (scaled B0 in units of 450°/sec). For attitude error display, needles pin at about 5 1/16°; for rate display, they pin at about 1 17/64°. These figures correspond to 46 least increments in the error counters. |
| 0053~8~ 0054~8~ (CM) | CDUTCMD CDUSCMD | Cells loaded with values to be transmitted to optics CDU error counters. Information gated out of cells if bits 12-11 respectively of channel 14 are set, and error counters lodaded if bit 2 of channel 12 is set (counter set 0 if bit 2 = 0). Used durings optics position drive operations to drive the optics trunnion (scale B-1 rev.) and shaft (scale B1 rev.) respectively. Drive of optics inhibited if bit 11 of channel 12 is set 1. May also be used for rate drive of optics on subsequent flights (see mission documentation). The cells are also used for control of SPS engine (see next paragraph). |
| 0053~8~ 0054~8~ (CM) | TVCYAW TVCPITCH | Cells loaded with values to be transmitted to "optics" error counters for use in controlling the position of the SPS (Service Propulsion System) engine gimbals. Same cells used to drive optics (see previous paragraph), but the automatic optics drive can be disabled by setting bit 11 of channel 12 (although optics could still drift unless mode specified to be optics zeroing). Information is gated out of cells if bits 12-11 respectively of channel 14 are set, and error counters loaded if bit 2 of channel 12 is set (counters et 0 if bit 2 = 0). Output of error counters, converted to dc, is sent to SPS engine yaew and pitch servos if bit 8 of channel 12 is set (which also inhibits the position feedback to the error counters used when commanding optics positioning). The error counter saturates at 600~8~(384 counts), or about 9.1°, and is loaded at a 3200 pps rate. One count for SPS driving corresponds to 85,41 arc seconds (or 0.023725°), giving about 42.14963 pulses/degree or 388.7104° (about 1.07975111 revolutions) per 2^14^ pulses. |
| 0053~8~ 0054~8~ (LM) | CDUTCMD CDUSCMD | Cells loaded with values to be transmitted to rendezvous radar error counters for use in controlling the position of the rendezvous radar antenna when its position is being controlled by software (when antenna sufficiently close to proper direction, the radar system controls its position provided bit 14 of channel 12 is 1). Information gated out of cells if bits 12-11 respectively of channel 14 are set, and error counters loaded if bit 2 of channel 12 is set (error counter reset to 0 if bit 2 of channel 12 is reset to 0). Cells used to control radar trunnion and shaft drives respectively, with a saturated error count (384 pulses) corresponding to a dfrive rate of about 10°/second: position signal corrected by program for desired dynamic response. If bit 8 of channel 12 is set, the error counter outputs (converted to dc) are used to provide lateral and forward velocity information respectively to an anlog display, scaled about 0.5571 fps/bit. |
| 0055~8~ (CM) | | not used |
| 0055~8~ (LM) | THRUST | Cell used to provide engine throttle commands for the descent engine, giving output pulses at a 3200 pps rate when bit 4 of channel 14 is set, of a polarity determined by the polarity of 0055~8~. Cell decremented as pulses are sent, and bit 4 of channel 14 is reset 0 when cell contents have been reduced to 0. Actual throttle command to engine is sum of counter contents (counter incremented by outputs of cell 0055~8~) and the position of manual throttle. The counter driven by the pulses controlled by cell 0055~8~ is reset 0 when the descent engine is disarmed, and has a saturation level greater than the number of pulses required for full throttle setting. One pulse corresponds to roughly 2.8 pounds of thrust (see mission documentation for specific value). |
| 0056~8~ | | Not used. Originally intended to provide entry monitoring system velocity information for CM (tag EMSD) and LM monitor function (LEMONM). Cell gives output pulses at a 3200 pps rate when bit 5 of channel 14 is set 1, of a polarity determined by he polarity of cell 0056~8~. The cell is decremented in magnitude as pulses are sent, and bit 5 of channel 14 is reset 0 when the cell contents have been reduced to 0. |
| 0057~8~ (CM) | LOCALARM | Cell used for storage of alarm source information (using cell as a normal erasable memory cell, rather than employing the counter feature described for LM). See mission documentation for details. |
| 0058~8~ (LM) | OUTLINK | Not used. Originally intended for use to provide a "crosslink" capability for serial binary data to cell 0045~8~ of another computer (if bit 5 of channel 13 of that computer is set). After loading 0058~8~ with the proper data, setting of bit 1 of channel 14 to 1 causes the data to be sent at a 3200 pps rate: first a binary 1 is sent, then the 15 bits in cell 0057~8~(most significant bit first). Bit 1 of channel 14 is reset when the first biary 1 (satisfying the format requirement for program interrupt #7) is generated, which is (1/6.4) ms after the first 200 pps signal followed the setting of bit 1 of channel 14. |
| 0060~8~ (CM) | BANKALRM | Cell used for storage of alarm source information (using cell as normal erasable memory cell, rather than employing the counter feature described for the LM). See mission documentation for details. |
| 0060~8~ (LM) | ALTM | Cell used to provide altitude and altitude rate information to analog display. Data provided in serial binary form with bit 2 of channel 14 set to 1 if altitude rate information is supplied (scaled at 0.5 fps/bit), and reset to 0 if altitude information is supplied (scaled about 2.345 ft/bit). After loading 0060~8~ with the proper data, setting of bit 3 of channel 14 causes the data to be sent at a 3200 pps rate: first a binary 1 is sent, then the 15 bits of cell 0060~8~ (most significant bit first). Bit 3 reset with the same timing as bit 1 for cell 0057~8~. |
| 0061~8~ | NEWJOB | Cell used in control of job sequencing (see Section VIIB). Each time it is sampled, a flip-flop set by a signal with a 1.28-second period is reset. If the flip-flop is set when another 1.28-second period signal (0.64 out of phase with the first) occurs, a "night watchman" fault (see Section IIH), causing a hardware restart, is produced. Hence maximum allowable interval between samples ranges from 0.64 to 1.92 second. |

## IIE Input/Output Channels

Binary-level inputs and outputs from the computer, including control signals for portions of the computer hardware, are handed through interfacing hardware called "channels". Analog-type pulse input/output, on the other hand, is mechanized  through the special purpose erasable memory cells with their associated counter interrupts, as discussed in Section IID and IIH. One of these special purpose cells (0048~8~), for example, is used to contain the magnitude of the required gyro torquing pulse output, while appropriate bits in one of the output channels (bits 9-7 of channel 14) specify not only the sign of the required pulses, but also the gyro axis to which they are to be applied.

Of the twenty channels which are defined, three different types may be identified:

1. Ten output channels, numbered 05, 06, 07, 10, 11, 12, 13, 14, 34, and 35. The first 8 can be both loaded and sensed under program control, but channerls 34 and 35 can be loaded only (they are used to provide telemetry from the computer).
2. Eight input channels, numbered 03, 04, 15, 16, 30, 31, 32 and 33. All eight can be sensed under program control. Bits 15-11 of channel 33 are flip-flop inputs, which can be set to a binary 1 (logic 0) under program control by a "loading" type command (instructions WAND, WOR or WRITE in Section IVC).
3. Two computer registers, numbered 01 and 02, corresponding to the L register and Q register respectively of Section IIC. These registers are included as "channels" to permit use of the bit manipulation capabilities of the seven channel instructions (see Section IVC) in the computer order code.

Channels are conventionally referenced by their octal channel number (the number appearing in the address portion of the appropriate channel instruction). To permit references to each channel to be flagged by the assembly program (see Section III), however, the program coding generally uses a symbolic reference tag for each channel, as shown in the mnemonic column in the following table.

In order to sense and/or load the input/output channels, only the seven extended-order (see Section IVC) channel instructions may be used: RAND, READ, ROR, RXOR, WAND, WOR and WRITE. These instructions cannot be used with other computer registers (except, of course, L and Q which are also "defined" to be channels).

The bit assignments on the following table are those of the quantities associated with the hardware. Several (such as CM/SM separation) may not be actually sensed by the program for computation control (as contrasted with e.g. telemetry) purposes, and therefore reliable equation information should be consulted to determine which bits serve a purpose in a given computer program configuration.

As part of a hardware restart (signal produced by hardwware, see page IIH-9), all output channel bits (except those of channel 07) are reset zero. Consequently, the software must restore the appropriate bits (such as IMU control and engine-on information) as necessary. In addition, the channel loading commands (WAND, WOR and WRITE) zero all bits of the channel briefly (about 1/4 ms), and some spacecraft hardware is sensitive to this brief zeroing, such as the radar systems in the LM, so special software techniques are required  to avoid loading channel (#13) while shift pulses are being generated (otherwise, a single shift pulse could appear as two pulses).

| Channel | Mnemonic | Bits | Function |
| --- | --- | --- | --- |
| 01 | LCHAN | 15-1 | Computer L register (address 0001~8~ in Section IIC). |
| 02 | QCHAN | 15-1 | Computer Q register (address 0002~8~ in Section IIC). |
| 03 | HISCALAR | 14-1 | Most significant 14 bits from 33-stage binary counter driven by 102.4 kc signal from computer oscillator (see Section IIA). Counter keeps running when computer placed in low-power (standby) mode of operation, and hence data in counter can be used to restore the proper value of the computer clock (cells 0024~8~ - 0025~8~ in Section IID) after a period of standby operation. Scale factor for channel 03 data is B23 in units of centi-seconds, so least significant bit is 5.12 seconds and channel information overflows every 23 hours, 18 minutes, 6.08 seconds (about 23.3 hours). |
| 04 LOSCALAR | 14-1 | Next most significant 14 bits from 33-stage binary counter associated with channel 03. Scale factor for channel 04 is B9 in units of centi-seconds, so least significant bit is 1/3200 second and channel information overflows (and propagates to channel 03) every 5.12 seconds. Time information in channel 04 is 0.005 seconds out of phase (i.e. 1/2 centi-second) with cell 0025~8~ in Section IID, so that the least significant 5 bits of channel 04 are 20~8~ during the first 1/3200 second interval after cell 0025~8~ (TIME1) has been incremented by +1. TIME1, in turn, is 5ms out of phase with the 100 pps signal used for control of the radar (see cell 0046~8~in Section IID). |
| 05 | CHAN5 PYJETS | 8-1 | RCS (Reaction Control System) jet controls. See next two tables. |
| 06 | CHAN6 ROLLJETS | 8-1 | RCS (Reaction Control System) jet controls. See next two pages. |
| 07 | SUPERBNK | 7-5 | Superbank (sometimes called F EXT) register, used to select the appropriate fixed memory bank for FBANK values of 30~8~ or more. Channel not reset if get a hardware restart. See Section IIB. |
| 10 | OUTO | 15-1 | Register used to transmit latching-relay driving information to the display system (see Section IIJ). Bits 15-12 are set to the row number (01~8~ - 14~8~) of the relays to be changed, and bits 11-1 contain the required settings for the relays in the selected row. Since the relays are bi-stable devices, the OUTO setting need be left for only 0.02 second. After a period of 0.02 second in which the channel bits are all reset, a setting for another row can be specified (hence to change all 11 rows that control the DSKY digit(sign displays requires 0.44 seconds). Row 17~8~ has been used for mission programmer functions on unmanned flights (e.g. LM-1), when the OUTO setting was retained for 0.03 seconds. |
| 11 | DSALMOUT | | Register whose individual bits are used for engine on/off control and to drive individual indicators of the display system (see Section IIJ). |
| | | 15 | Not assigned |
| | | 14(CM) | Not assigned |
| | | 14(LM) | Eneine Off signal to engjne sequencer for descent and ascent engine. If bits 14-13 = OO~2~ the engine remains in its previous state (on or off), but if the vehicle stages with the bits equal to OO~2~ the ascent engine would not 1ight. If the descent engine sees the bits equal to 11~2~, it likewise remains in previous state; the ascent engine, however, turns on. The OO~2~ condition, however, should be avoided when the engine is armed. |
| | | 13(CM) | SPS (Service Propulsion System) engine turn-on signal (set to 0 to turn engine off). |
| | | 13(LM) | Engine On Signal to engine sequencer for descent and ascent engine. (See bit 14 of channel 11). |
| | | 12 | Not assigned |
| | | 11 | Not assigned. Used by LM-1 program for telemetry purposes as a status indicator of program performance |
| | | 10 | Caution reset signal. It resets the flip-flop holding the Restart light (See Section IIJ) of the display system in the energized state. |
| | | 9 | Test Connector Outbit (Connector A52 pin 813). Can be used as an indicator for hybrid simulator test purposes that Average-G (two second navigation cylce using accelerometer outputs) is running, if suitably set by software. |
| | | 8 | Not assigned |
| | | 7 | Bit that energizes the Operator Error (See Section IIJ) of the display system. It is set to 1 if an improper operator entry to the keyboard or uplink detected by software, and it causes the Operator Error light to be flashed. |
| | | 6 | Bit that energizes the Flash (See Section IIJ) of the display system, that causes the verb and noun register indicators to be flashed on and off (not noticable unless they are blank, of course). Used by the software to signify that an operator response or action is needed |
| | | 5 | Bit that energizes the Key Release (see Section IIJ) of the display system. It is set to 1 if the softuare of the internal display system users is inhibited from using the display system because of operator use. The bit causes the Key Release light to be flashed. |
| | | 4 | Bit that energizes the Temperature Caution Light (see Section IIJ) of the display system. This light is also connecied to bit 15 of channel 30. |
| | | 3 | Bit that energizes the Uplink Activity light (see Section IIJ) of the display system, set by software when program interrupt #7 (see Section ,IIH) is processed, and reset likewise by the software (at termination of uplink sequence, etc.). Can also be used for informing crew of other situations when uplink information not being received (such as, for CM, the need for an attitude maneuver): see equation documentation. |
| | | 2 | Bit that energizes the Computer Activity light (see Section IIJ) of the dlsplay system. It is set by the software if the executive system (Seetion VIIB) has an active job being performed (i.e. something besides the dummy job routine). The bit remains at its previous value uhen a task (such as the one initiated by program interrupt #8 for telemetry) is done. |
| | | 1 | Bit that energizes the ISS (inertial subsystem) Warning Iight, a red light on the caution and warning panel of the spacecraft, if IMU, IMU CDU, or PIPA fail indications are significant in terms of mission phase (as determined by the software). Bit can be used on unmanned flights to generate a PGNCS (primary guidance, navigation, and control system) failure indication. |
| 12 | CHAN12 | | Register whose individual bits are used to drive miscellaneous navigation and spacecraft hardware. |
| | | 15 | ISS turn-on delay complete. Bit set by software nominally 1O seconds after receipt of ISS power-on signal, bit 14 of channel 3O, and reset to zero nominally 10.24 seconds later. Used to delay the closing of the stabilization loops of the IMU gimbals (to permit the gyro wheels to reach operating speed) and also to delay torquing of the accelerometers. Bit energizes a latching relay which energizes the ISS turn-on relay, removing the signal from bit 14 of channel 30. Same effect achieved by IMU Cage button, bit 11 of channel 30. |
| | | 14(CM) | S4B Cutoff conrnand, Command provided via a relay in the DSKY to the Saturn Instrumentation Unit. The relay contact closure is not functional unless CMC control of Saturn is enabled (and hence software may close it unconditionally, see equation documentation). |
| | | 14(LM) | Enable rendezvous radar lock-on. Command provided via a relay in the DSKY to enable automatic angle tracking by the rendezvous radar when software determines that antenna position (from cells OO35~8~ - 0035~8~) is sufficiently close to the predicted position of the other vehicle. |
| | | 13(CM) | SA4 Injection Sequence start. Command provided via a relay in the DSKY to Saturn Instrumentation Unit if backup generation of the signal (which starts the Saturn "Time Base 6") is required. |
| | | 13(LM) | Landing radar position command. Command provided via a relay in the DSKY to change landing radar antenna position from position #1 (descent, see bit 6 of channel 33) to position #2 (hover, see bit 7 of chrannl 33). For the LGC command to have
an effect, the landing antenna switch must be in "AUTO" (its other positions are "DESC" and "HOVER"). |
| | | 12(CM) | Not assigned |
| | | 12(LM) | Descent engine negative roll gimbal trim. Nominal trim rate about O.2°/sec, and magnitude of trim determined by length of time that signal left at a binary 1, DPS engine trim gimbal actuator driven in such a way as to be rotated in a positive right hand sense about the positive roll (+Z) axis, for -R acceleration. |
| | | 11(CM) | Disengage optics DAC (digital to analog converter). Can be used to disconnect optics CDU DAC from shaft and trunnion motor drive amplifiers, if zeroing of optics desired by software with optics in computer mode. Can also be set by software (see equation docunentation) prior to use of the optics DAC for SPS gimbal drive purposes (see cells 0053~8~-OO54~8~ in Section IID). |
| | | 11(LM) | Descent engine positive roll gimbal trim. Nominal trim rate about 0.2°/sec, and magnitude of trim detemined by length of time that signal left at a binary 1. DPS engine trim gimbal actuator driven in such. a way as to be rotated in a negative right hand sense about the positive roll (+Z) axis, for +R acceleration. |
| | | 10(CM) | Zero optics. Function also performed by setting spacecraft "Optics Zero" switch to "ZERO". |
| | | 10(LM) | Descent engine negative pitch gimbal trim. Nominal trim rate about O.2°/sec, and magnitude of trim determined by length of time that signal left at a binary 1. DPS engine trim gimbal actuator driven in such a way as to be rotated in a positive right hand sense about the positive pitch (+Y) axis, for -Q acceleration. |
| | | 9(CM) | S4B takeover enable. Connects the dc output of the IMU CDU error counters (Ioaded from cells 0050~8~ - 0052~8~, see Section IID) to the Saturn Instrumentation Unit. Used to permit attitude control of Saturn through the guidance computer (which does not necessarily mean the engine sequencing and on/off control of bits 14-13 of this channel, of course ) |
| | | 9(LM) | Descent engine positive pitch gimbal trim. Nominal trim rate about O.2°/sec, and nagnitude of trim determined by length of time that signal left at a binary 1. DPS engine trim gimbal actuator driven in such a way as to be rotated in a negative right hand sense about the positive pitch (+Y) axis, for +Q acceleration. |
| | | 8(CM) | TVC (thrust vector control) enable. Connects the dc output of the optics CDU error counters (loaded from cells 0053~8~ e 0054~8~, see Section IID) to the SPS (service propulsion system) gimbal servo amplifiers. |
| | | 8(LM) | Display inertial data. Connects the dc output of the rendezvous radar CDU error counters (loaded from cells 0053~8~ -  0054~8~, see Section IID) to a spacecraft analog display to provide lateral and forward velocity information. Bit set by software (provided proper computations are being performed) when bit 6 of channel 30 indicates that such a display is desired. |
| | | 7 | Not assigned. Originally intended to be used as engine on command (done now by bits 14-13 of channel 11) |
| | | 6 | Enab1e IMU CDU error counters. The error counters are reset to O if this bit is O, and are loaded from cells OO50~8~ - 0052~8~ (see Section IID). |
| | | 5 | Zero IMU CDUs. Can be used to force the CDU hardware to a zero value, whereupon zeroing of cells OO32~8~ - 0034~8~ and then reset of this bit will permit these cel1s to reflect the IMU gimbal angle information. This bit alone does not cause movement of the stable member: this is done at IMU turn-on or by an IMU cage command (bit 11 of channel 30), or by coarse aligning. |
| | | 4 | Enable coarse align of IMU. Connects IMU CDU error counters to cause IMU coarse alignment (angle change information loaded into cells OO5O~8~ - OO52~8~, and bit 6 of this channel must be a 1). |
| | | 3 | Not used. In CM, assigned to enable star tracker (no longer in vehicle), and in LM assigned to indicate low scale for horizontal velocity output |
| | | 2(CM) | Enable optics CDU error counters. The error counters are reset to O if this bit is O, and are loaded from cells OO53~8~ - 0054~8~ (used also for yaw and pitch SPS control). |
| | | 2(LM) | Enable rendezvous radar CDU error counters. The error counters are reset io O if this bit is 0, and are loaded from cells OO53~8~ - OO54~8. |
| | | 1(CM) | Zero optics CDUs. Can be used to force the optics CDU hardware to a zero value, whereupon setting of cells 0035~8~ - 0036~8~ are then reset of this bit will permit these cells to reflect the optics angle information. |
| | | 1(LM) | Zero rendezvous radar CDUs. Simlar function to bit 1(CM), but for rendezvous radar rather than optics, To avoid an excessive number of counter interrupts which can occur if RR mode not set to LGC, software sets this bit 1 if the mode not LGC. |
| 13 | CHAN13 | | Regisier whose bits are used to control miscellaneous navigation system functions (some bits sensitive to write commands, see page IIE-2). |
| | | 15 | Bit set to 1 to permit cell 0031~8~ (TIME6) to be decremented by 1 each 0,000625 second (i.e. 1600 times a second). When cell has been reduced to -0, the next DINC pulse causes bit to be reset to O and program interrupt #1 to be produced. (see Section IIH) |
| | | 14 | Bit set to 1 to cause trap 32 to be reset. This input trap circuit is set when program interrupt #10 is generated in response to a signal fed to bits 12-7 of channel 31 (see Section IIH). |
| | | 13 | Bit set io 1 to cause trap 31B to be reset. This input trap circuit is set uhen program interrupt #10 is generated in response to a signal fed to bits l2-7 of channel 31 (see Section IIH). |
| | | 12 | Bit set to 1 to cause trap 31A to be reset. This input trap circuit is set when program interrupt #10 is generated in response to a signal fed to bits 6-1 of channel 31 (see Section IIH). |
| | | 11 | Bit set to 1 to permit relay in computer power supply to put computer in Standby (low-power) operation when the PRO key (formerly the _Standby_ key) on the DSKY is pressed. The bit is set by the software when preparations for standby operation completed, including retention of the conputer clock, and it is reset by the software after clock restored. |
| | | 10 | Bit set to 1 to tesi ihe DSKY lights and relays not othetwise accessible to the software. It energizes the Restart light, the Standby light, and the Computer Warning light (via a warning filter). |
| | | 9(CM) | Not assigend. |
| | | 9(LM) | Bit set to 1 to initiate readout of analog-to-digital converters associated with the displacement of the rotational hand controller when used as a rate commanding device. See cells 0042~8~- 0044~8~ in Section IID. |
| | | 8 | Bit set to 1 to enable inputs to cells 0042~8~- 0044~8~ (see Section IID) for LM rotational hand controller rate-comnand input and for (unused) CM BMAG input. |
| | | 7 | Bit used as the word order code bit (first bit in the 40-bit downlink sequence sent from computer for digital- data) for  teIemetry, in order to flag certain words in the list. |
| | | 6 | Bit, set to 1 to block all inputs to INLINK (cell 0045~8~, see Section IID). |
| | | 5 | Bit set to 1 to connect (unused) crosslink input instead of uplink receiver to cell 0045~8~ (see Section IID). |
| | | 4 | Bit set io I to initiate transmission of radar information to computer. Bit is reset to 0 when program interrupt #9 is generated (see Section IID for timing sequence associated with loading of cell 0046~8~.) |
| | | 3(CM) | Bits set to OO1~8~ in order to specify that range information from VHF range system is to be provided to computer in cell 0046~8~ (see Section IID) after bit 4 of channnel 13 is set 1. |
| | | 3(LM) | Bits assigned control functions for sampling of rendezvous radar (RR) if bit 3 is O and of landing radar (LR) if bit 3 is 1, to establish information fed to cell 0046~8~ when bit 4 of channel 13 is set. For RR bits 2-1 are set to 01~2~ for range data and 10~2~ for range rate data. For LR, bits 2-1 are set to 00~2~, 01~2~, and 10~2~ for x-z velocities respectively, and to 11~2~, for range (altitude) information. |
| 14 | CHAN14 | | Register whose bits are used to control the computer counter cells (CDU, gyro, and spacecraft functions) described jn Section IID. |
| | | 15 | Bit set to 1 to cause output pulses (at a 3200 pps rate) to be generated fron CDUXCMD, cell 0050~8~. When cell counted down to 0, bit is reset (at the next DINC, see Section IIH), thereby stopping the pulses. Error counter is loaded if bit 6 of channel 12 is 1. |
| | | 14 | Bit set to 1 to cause output pulses to be generated from CDUYCMD, cell 0051~8~: see bit 15 of channel 14. |
| | | 13 | Bit set to 1 to cause output pulses to be generated from CDUZCMD, ceII 0052~8~: see bit 15 of channel 14. |
| | | 12 | Bit set to 1 to cause output pulses (at a 3200 pps rate) to be generated from cell 0053~8~ (CDUTCMD or TVCYAW). When the cell has been counted down to 0, bit is reset (at the next DINC, see Section IIH), thereby stopping the pulses. Error counter is loaded if bit 2 of channel 12 is 1. |
| | | 11 | Bit set to 1 to cause output pulses to be generated from cell 0054~8~ (CDUSCMD or TVCPITCH): see bit 12 of channel 14. |
| | | 10 | Bit set to 1 to specify "gyro activity: it causes the pulse train whose magnitude is in cell 0047~8~, (GYROCMD) to be sent with polarity and destination specified by bits 9-7 of this channel, if bit 6 of this channel is 1. Bit reset 0 after proper pulses sent (cell reduced to O and the next DINC). |
| | | 9 | Bit set to 1 to specify a negative-polarity gyro torquing output from GYROCMD (cell 0047~8~). Other pulse-type outputs from the computer have the polarity indicated by the polarity of the information in the counter ceIL itself. |
| | | 8-7 | Bits used to specify the axis for gyro conpensation information from GIROCMD. Conventional output sequence is inner (Y), middle (Z), and outer (X) for torquing, with the following bit configurations: Bits 8-7 00~2~: Torque Output Non, 01~2~: X-axis Gyro, 10~2~: Y-axis Gyro, 11~2~: Z-axis Gyro |
| | | 6 | Bit set to 1 to enable the power supply that produces the torquing pulses used to torque gyros (in a manner determined by bits 7-10 of this channel and cell 0047~8~). Software generally leaves bit at 1 after the first gyro torquing is performed (reset to 0 when certain initialization functions performed). |
| | | 5 | Not used. Bit set to 1 to initiate commands from data in cell 0056~8~ (see Section IID). |
| | | 4(CM) | Not used (initiates commands from cell 0055~8~). |
| | | 4(LM) | Bit set to 1 to cause output pulses to be generated from cell 0055~8~ (THRUST) for use in controlling the position of the descent engine throttle (see Section IID). When cell has been reduced to -0, lhe nexl DINC pulse causes this bit to be reset to 0. |
| | | 3(CM) | Not used (initiates commands from cell 0060~8~). |
| | | 3(LM) | Bit set to 1 to initiate shifting of data from cell 0060~8~ (ALTM) to spacecraft indicator for altitude (bit 2 of this channel = 0) or altitude rate (bit 2 of this channel = 1) information. Bit reset to O just after start of data shift (see Section IID). |
| | | 2(CM) | Not used. |
| | | 2(LM) | Bit set to 1 to indicate that, altitude rate information is being shifted from cell 0060~8~; if is 0, altitude information is being shifted. |
| | | 1 | Not used. Bit set to 1 to initiate shifting of data from cell 0057~8~ (see Section IID). |
| 15 | MNKEYIN | 5-1 | Key code input from keyboard of DSKY (see Section IIJ), sensed by the program when program interrupt #5 (see Section IIH) is acted upon. For the CM (which has two DSKY's), channel 15 is associated with the DSKY located on the main display console. |
| 16 | NAVKEYIN | 5-1 | Optics mark information and lower equipnent bay (or "navigation panel") DSKY inputs for CM; optics mark information and rate-of-descent control for LM. Sensed by the program when program interrupt #6 (see Section IIH) is acted upon. |
| | | 7(CM) | Optics mark reject signal if 1. |
| | | 7(LM) | Bit set to 1 if an increase in the rate of descent is desired by crew (i.e. a lower thrust). Generated by moving rate-of-descent switch in the -X direction (i.e. towards engine), Effect of switch and scaling (e.g. 1 fps per "click") controlled by software: see equation documentation. |
| | | 6(CM) | Optics mark signal if 1 |
| | | 6(LM) | Bit set to 1 if a decrease in the rate of descent is desired by crew (i.e. a higher thrust). Generated by moving  rate-of-descent switch in +X direction (i.e. away from engine). Processed by software similarly to bit 7(LM). |
| | | 5-1(CM) | Lower equipurent bay (or "navigation panel") DSKY key code input (see Section IIJ). |
| | | 5(LM) | Optics mark reject signal if 1. |
| | | 4(LM) | Optics Y-axis nark signal for AOT (alignment optical telescope) if 1. |
| | | 3(LM) | Optics X-axis mark signal for AOT if 1. |
| | | 2-1(LM) | Not used |
| 17-27 | | | Channnels not assigned. Some tentatively allocated for control of additional memory capacity that has been considered  for CM (an Auxiliary Core Memory addressed with SUPRRBNK settings of 5 and 6). |
| 30 | CHAN30 | | Register whose bits are used to supply miscellaneous input information for the program. All bits are inverted as sensed by the program, so that a value of binary 0 means that the indicated signal is present. |
| | | 15 | Bit sensed as 0 if the stable member temperature is within its design limits . Software sets bit 4 of channel 11 to 1 if this bit becomes 1. The light controlled by bit 4 of channel 11 is also connected directly to this bit 15 of channel 30. |
| | | 14 | Bit sensed as O if the inertial subsystem has been turned on or commanded to be turned on. Bit 15 of channel 12 is set to 1 by the software about 90 seconds after this bit sensed as O (if checks passed), resetting this bit to 1. |
| | | 13 | Bit sensed as O if an IMU fail indication has been generated within the IMU hardware (due e.g. to exeessive servo errors or degradation of 3200 cps or 800 cps supply). Software controls setting of bit 1 (ISS Warning) of channe1 11 based on this input bit and the IMU mode. |
| | | 12 | Bit sensed as O if an IMU CDU fail indication has been generated within the IMU CDU hardware (due e,g, to excessive errors or 1ow voltage). Software controls setting of bit 1 (ISS Warning) of channel 11 based on this input bit and the IMU mode. |
| | | 11 | Bit sensed as 0 if the "IMU Cage" switch is set by crew to drive all the IMU gimbal angles to zero. The command is also sent directly to the IMU control hardware, and can be used as an emergency technique for recovering a tumbling IMU. The preferred method, however, is to remove power. |
| | | 10(CM) | Bit sensed as 0 if the 'Launch Vehicle Guidance' switch is set by crew to the "CMC" (as opposed to "IU") position, indicating that control of the Saturn vehicle has been given to the computer. |
| | | 10(LM) | Bit sensed as O if the 'Guidance Control' switch is set by crew to the "PGNS" (as opposed to "AGS", for Abort Guidance Section) position, indicating that control of the vehicle has been given to the computer. |
| | | 9 | Bit sensed as O if the IMU is turned on and operating with no malfunctions. |
| | | 8 | Not assigned |
| | | 7(CM) | Bit sensed as 0 if an optics CDU fail indication produced (due e.g. to excessive errors or low voltage). Software controls setting of bit 1 of row 14~8~ (TRACKER light, see Section IIJ) based on thls input bit and optics mode. |
| | | 7(LM) |  Bit sensed as 0 if a rendezvous radar CDU fail indication produced (due e.g. to excessive errors or low voltage). Software controls setting of bit 1 of row 14~8~ (TRACKER light, see Section IIJ) based on this input bit and radar selection. |
| | | 6(CM) | Bit sensed as 0 if GRR (guidance reference release) signal generated by S4B Instrumentation Unit, indicating that this action has occurred or has been commanded to occur. Software uses bit 5 rather than this bit to halt pre-launch computations (with backup of a DSKY verb). |
| | | 6(LM) | Bit sensed as 0 if a display of inertial data from the computer is desired by the crew, by setting the "Mode Select" switch to the "PGNS" position (as opposed to "LDG RADAR" or "AGS"). When the appropriate information has been loaded by the software, bit 8 of channel 12 is set to 1. |
| | | 5(CM) | Bit sensed as 0 if liftoff signa1 generated by S4B Instrumentation Unit, indicating that lift-off has taken place. Software uses this bit to halt pre-launch computations (with backup of a DSKY verb). |
| | | 5(LM) | Bit sensed as 0 if computer given control of descent engine throttle by the crew, by setting the "Throttle Control" switch to "AUTO" (as opposed to "MAN" ) position. In the "AUTO" position, computer throttle commands from cell 0055~8~ (THRUST, see Section IID) are.summed with the manual throttle commands; in the "MAN" position, with bit = 1, the computer commands no longer control the throttle setting. |
| | | 4(CM) | Bit sensed as 0 if the S4B separation (or abort) signal is received. Software does not use the bit. |
| | | 4(LM) | Bit sensed as 0 if the crew has produced an "Abort Stage" command (with a spacecraft pushbutton switch), indicating that an abort using the ascent engine is required (spacecraft hardware causes descent engine to be staged). |
| | | 3 | Bit sensed as 0 when preparations for use of the appropriate engine have been completed. Software does not use this bit. For CM, it indicates that a "Delta-V Thrust" switch has been set to "NORMAL"; for LM, that the "Engine Armed" switch has been set to "ASC" or "DSC". |
| | | 2(CM) | Bit sensed as 0 when CM/SM separation has taken place. The bit is generated by the CM/SM reaction jet control transfer unit, but is not used by the software |
| | | 2(LM) | Bit sensed as 0 to indicate that the descent stage is attached ("Stage Verify"): a value of 0 neans descent stage and a value of 1 means ascent stage only. Software does not use the bit. |
| | | 1(CM) | Bit sensed as 0 if "Ullage Thrust Present" signa1 received from S4B Instrunentation Unit. Bit not sensed by software. |
| | | 1(LM) |  Bit sensed as 0 if the crew has produced an "Abort" command (with a spacecraft pushbutton switch), indicating than an abort using the descent engine is required. When the descent engile propellants are nearly expended, the crew could then initiate the "Abort Stage" commnand (bit 4 or this channel). |
| 31 | CHAN31 | | Register whose bits are associated with the attitude controller, translational controller, and spacecraft attitude control. All bits are inverted as sensed by the program, so that a value of binary O neans that the indicated signal is present. |
| | | 15(CM) | Bit sensed as O if the computer is in control of the spacecraft. The bit becomes a binary 1 if the IMU is turned off, if the THC (translation hand controller) is twisted in the clockwise direction, or if the "Spacecraft Control" switch is placed in the "SCS" (spacecraft control system) as opposed to the "CMC" position. |
| | | 15(LM) | Bit sensed as O if ACA (attitude controller assembly) is out of detent. Control also referred to as RHC (rotational hand controller), see LM cells 0042~8~- 0044~8~ in Section IID. |
| | | 14(CM) | Bit sensed as 0 if the three-position "CMC Mode" switch is set by crew to "FREE". Software fires RCS jets only in response to controller inputs (as with other nanual inputs, bit ignored by software unless RCS DAP is running). |
| | | 14(LM) | Bit sensed es 0 if the "PGNS Mode Control" switch is set to "AUTO", indicating that the software has complete authority for control of spacecraft (if bit 10 of channel 30 = 0). |
| | | 13(CM) | Bit sensed as 0 if the three-position "CMC Mode" switch is set by crew to "HOLD", indicating that attitude hold is desired. If bits 14-13 = 11~2~, this means that the third position of the switch, "AUTO", is selected (softuare has complete authority for control of spacecraft if bits 15-13 = 011~2~). |
| | | 13(LM) | Bit sensed as 0 if the "IPGNS Mode Control" svitch is set to "ATT HOLD", indicating to the software that attitude hold is desired. If the switch is set to "OFF", then bits 14-13 : 11~2~. |
| | | 12 | Bit sensed as 0 if translation in the -Z direction commanded by THC (translation hand controller). In LM is "TTCA" (thrust/translation controller assembly). |
| | | 11 | Bit sensed as 0 if translation in +Z direction commanded by THC. |
| | | 10 | Bit sensed as 0 if transLation in -Y direction commanded by THC. |
| | | 9 | Bit sensed as 0 if translation in +Y direction commanded by THC. |
| | | 8 | Bit sensed as 0 if translation in -X direction commanded by THC. |
| | | 7 | Bit sensed as 0 if translation in +X direction commanded by THC. |
| | | 6(CM) | Bit sensed as 0 if rotation commanded in negative roll direction by RHC (rotational hand controller). |
| | | 6(LM) | Bit sensed as 0 if ACA (attitude controller assembly) is deflected in negative roll direction. If software set to use controller for minimum impulse purposes, then a rotation in the desired direction produced. Otherwise, controller used as a rate command device, with input to cells 0042~8~- 0044~8~. During portion of lunar descent, software senses bit for use as a landing point
designation change, giving a "negative azimuth" offset (new site is to left as viewed by crew). |
| | | 5(CM) | Bit sensed as 0 if rotation commanded in positive roll direction by RHC. |
| | | 5(LM) | Bit sensed as 0 if ACA deflected in positive roll direction (see bit 5(LM) discussion). For landing point designation, input gives a "positive azimuth" offset (new site is to right as viewed by crew). |
| | | 4(CM) | Bit sensed as 0 if rotation commanded in negative yaw direction by RHC. |
| | | 4(LM) | Bit sensed as 0 if ACA deflected in negative yaw direction (see bit 5(LM) discussion). |
| | | 3(CM) | Bit sensed as 0 if rotation commanded in positive yaw direction by RHC. |
| | | 3(LM) | Bit sensed as 0 if ACA deflected in positive yaw direction (see bit 6(LM) discussion). |
| | | 2(CM) | Bit sensed as 0 if rotation commanded in negative pitch direction by BHC. |
| | | 2(LM) | Bit sensed as 0 if ACA deflected in negative pitch direction (see bit 6(LM) discussion). For landing point designation, input gives a "negative elevation" offset (new site beyond the present site). |
| | | 1(CM) | Bit sensed as 0 if rotation commanded in positive pitch direction by RHC. |
| | | 1(LM) | Bit sensed as 0 if ACA deflected in positive pitch direction (see bit 6(LM) discussion). For landing point designation, input gives a "positive elevation" offset (new site short of the present site). |
| 32 | CHAN32 | | Register whose bits are used for miscellaneous inputs from the spacecraft. All bits are inverted as sensed by the program, so that a value of binary 0 means that the inndicated signal is present. |
| | | 15 | Not assigned |
| | | 14 | Bit sensed as 0 if the PRO (proceed) key on the DSKY is depressed (see Section IIJ). This key was formerly labeled "STBY" (and also serves that function if bit 11 of channel 13 is 1). Software can cause a logical "Proceed" function to be performed when a binary 1 to binary 0 transition of the bit is sensed by a check done every 0.12 sec. |
| | | 13 | Not assigned |
| | | 12 | Not assigned |
| | | 11(CM) | Bit sensed as 0 if the "DELTA VCG" switch set by crew to the "LM/CSM" (as opposed to "CSM") position. The software uses a DSKY input for vehicle status. |
| | | 11(LM) | Not assigned |
| | | 10(CM) | Not assigned |
| | | 10(LM) | Bit sensed as 0 if the descent engine gimbal failure monitor detects an apparent gimbal fail in the pitch or roll gimbal trim system. The software does not use the bit (but takes action based on bit 9 of this channel instead). |
| | | 9(CM) | Not assigned |
| | | 9(LM) | Bit, sensed as 0 if the "Engine Gimbal" switch set by crew to "OFF" (as opposed to "ENABLE") position, indicating that the descent engine gimbal drive system has been disabled. If bit 0, software does not attempt to use biLs 12-9 of channel 12 to control the position of the descent engine gimbal. |
| | | 8(CM) | Not assigned |
| | | 8(LM) | Bit sensed as 0 if System A Quad 2 RCS jets shut off (RCS jets 10 and 11). |
| | | 7(CM) | Not assigned |
| | | 7(LM) | Bit sensed as 0 if System B Quad 2 RCS jets shut off (RCS jets 9 and 12). |
| | | 6(CM) | Bit sensed as 0 if negative ro11 commanded by minimum impulse controller. |
| | | 6(LM) | Bit sensed as 0 if System A Quad 1 RCS jets shut off (RCS jets 13 and 15). |
| | | 5(CM) | Bit sensed as 0 if positive roll commanded by minimum impulse controller. |
| | | 5(LM) | Bit sensed as 0 if System B Quad 1 RCS jets shut off (RCS jets 14 and 16). |
| | | 4(CM) | Bit sensed as 0 if negati-ve yaw commanded by minimum impulse controller. |
| | | 4(LM) | Bit sensed as 0 if System B Quad 3 RCS jets shut off (RCS jets 6 and 7) |
| | | 3(CM) | Bit sensed as 0 if positive yaw commanded by minimum impulse controller. |
| | | 3(LM) | Bit sensed as 0 if System B Quad 4 RCS jets shut off (RCS jets 1 and 3). |
| | | 2(CM) | Bit sensed,as 0 if negative pitch commanded by minimum impulse controller. |
| | | 2(LM) | Bit sensed as 0 if System A Quad 3 RCS jets shut off (RCS jets 5 and 8). |
| | | 1(CM) | Bit sensed as 0 if positive pitch commanded by minimun impulse controller. |
| | | 1(LM) | Bit sensed as 0 if System A Quad d4 RCS jets shut off (RCS jets 2 and 4). |
| 33 | CHAN33 | | Register whose bits are used for various hardware status data. All bits are inverted as sensed by the program, so that a value of binary 0 means that the indicated signal is present. Bits 15-11 of this channel are flip-flop inputs, which retains a "set" state (binary 0 as sensed) until reset by a "loading" type command (orders WAND, WOR, or WRITE in Section IVC) or hardware restart. |
| | | 15 | Flip-flop input sensed as 0 if the computer oscillator has stopped. Can be reset by a channel loading command. |
| | | 14 | Flip-f1op input sensed as 0 if a computer warning indication produced (e.g. restart, counter fai1, voltage fail in standby, scaler double or fail, prime power fail, or alarm test, by bit 10 of channel 13). Can be reset by a channel loading command. |
| | | 13 | Flip-flop input sensed as 0 if a PIPA fail indication generated by PIPA (accelerometer) electronics due to improper pulses from a PIPA. Software controls setting of bit 1 (ISS Warning) of channel 11 based on this input bit and the use being made of PIPA outputs. Can be reset by a channel loading command. |
| | | 12 | Flip-flop input sensed as 0 if a telemetry end pulse occurs too soon after the previous pulse: these pulses cause program interrupt #8 to be generated, see Section IIH. The pulses are considered to be "too fast" if a 1OO pps pulse has not occured since the previous end pulse was received. Can be reset by a channel load. |
| | | 11 | Flip-flop input sensed as 0 if an input bit to cell 0045~8~ (INLINK, see Section IID) is rejected due io an excessive bit rate. Rejection takes place if a 6400 pps pulse has not occurred since thg previous input bit was received. Can be reset by a channel loading conmand. |
| | | 10(CM) | Bit sensed as 0 unless both the "CM Up Telemetry" switch (on the main display console) and the "Up Telemetry" switch (in the lower equipment bay) are each set to "ACCEPT" (as opposed to "BLOCK"), Bit not used by software, but it must be a binary 1 for inputs from the uplink receiver to be gated into cell 0045~8~ (INLINK, see Section IID). |
| | | 10(LM) | Not assigned (would be expected to read a binary 1). Similar blocking function to that, for CM could be obtained by setting spacecraft switch to use rf link for voice back-up (or by manually setting bit 6 of channel 13 to 1). |
| | | 9(CM) | Not assigned |
| | | 9(LM) | Bit sensed as 0 if landing radar range ("altitude") on low scale (controlled by landing radar system and changed at an altitude of about 2100 feet). Least increment of range information decreased by a factor of 5.000 when switch to low scale (see cell 0046~8~, RNRAD, in Section IID). |
| | | 8(CM) | Not assigned |
| | | 8(LM) | Bit sensed as 0 if all three landirg radar velocity trackers have locked on, a necessary criterion for landing radar velocities to be valid. |
| | | 7(CM) | Not assigned. Formerly used with star tracker to indicate star present |
| | | 7(LM) | Bit sensed as 0 if power applied to landing radar and the antenna is in "position 2" (used for hovering). Antenna can be commanded from position 1 to position 2 by bit 13 of channel 12. |
| | | 6(CM) | Not assigned. Formerly used with star tracker to indicate star tracker on. |
| | | 6(LM) | Bit sensed as 0 if power applied to landing radar and the antenna is in "position 1" (used for descent prior to hovering, see bit 7(LM) of this channel). |
| | | 5(CM) |  Bit sensed as 0 if the "Optics Mode" switch in the lower equipment bay is set to "CMC" and the "Optics Zero" switch there is set io "OFF". A binary 0 indicates that the optics can be driven via cells 00534~8~ - 0054~8~ (see Section IID) unless inhibited e.g. by setting bit 11 of channel 12 to 1. |
| | | 5(LM) | Bit sensed as 0 if the landing radar large tracker and two rear velocity-beam trackers have locked on, a necessary criterion for landing radar range ("altitude") data to be valid. |
| | | 4(CM) | Bit sensed as 0 if the "Optics Zero" switch in the lower equipment bay is set to "ZERO" (as opposed to "OFF"), regardless of the position of the "Optics Mode" switch there. If bits 5-4 = 11~2~, this indicates that the "Optics Zero" switch is set "OFF" and the "Optics Mode" switch is set to "MAN" (for manual positioning of optics). |
| | | 4(LM) | Bit sensed as 0 if the rendezvous radar range tracker and frequency tracker are locked on, a necessary criterion for rendezvous radar range and range rate data to be valid. |
| | | 3(CM) | Not assigned |
| | | 3(LM) | Bit sensed as 0 if rendezvous radar is on the low range scale. Internal range counter in radar is 18 bits in length, and if most significant 3 bits are 0 then bits 15-1 are sent to cell 0046~8~ and this bit is set O; otherwise, bits 18-4 are sent. Hence least increment decreased by a factor of 8.OOO when switch to low scale, and switch occurs at 9.38 (2^15^ - 1) feet, or at about 50.584 nm. |
| | | 2(CM) | Bit sensed as 0 if VHF range data considered OK. |
| | | 2(LM) | Bit sensed as 0 if rendezvous radar power is on and the rendezvous radar mode switch is in the "LGC" (as opposed to "SLEW" or "AUTO TRACK") position, meaning that CDUs driven from an LGC power supply and control of the antenna position can be accomplished via cells 0053~8~ - 0054~8~. If bit is 1, software can set bit 1 of channel 12 = 1 (see equation documentation) |
| | | 1 | Not assigned. |
| 34 | DNTM1 | 15-1 | Register used to contain the first word of a pair telemetered periodically, loading of a new pair is performed by software when program interrupt #8 (see Section IIH) is processed. Channel contents cannot be sensed by a channel-sensing instructlon (will give zero). See Section IIH for format of output. |
| 35 | DNTM2 | 15-1 | Register used to contain the second word of a pair telemetered periodically. Loaded by software when program interrupt #8 (see Section IIH) is processed: channel 34 is loaded also at this time. See Section IIH for format of output. Channel contents cannot be sensed by a channel sensing instruction (will give zero). |

### Channel 5 - Service Module RCS Jets

| Bit | Jet | Quad | Reaction |
| --- | --- | --- | --- |
| 8 | 6/B4 | B | +X -Yaw |
| 7 | 7/B3 | B | -X +Yaw |
| 6 | 8/D4 | D | -X -Yaw |
| 5 | 5/D4 | D | +X +Yaw |
| 4 | 2/A4 | A | +X -Pitch |
| 3 | 3/A3 | A | -X +Pitch |
| 2 | 4/C4 | C | -X -Pitch |
| 1 | 1/C3 | C | +X +Pitch |

### Channel 6 - Service Module RCS Jets

| Bit | Jet | Quad | Reaction |
| --- | --- | --- | --- |
| 8 | 14/C2 | C | +Y -Roll |
| 7 | 15/C1 | C | -Y +Roll |
| 6 | 16/A2 | A | -Y -Roll |
| 5 | 13/A1 | A | +Y +Roll |
| 4 | 10/D2 | D | +Z -Roll |
| 3 | 11/D1 | D | -Z +Roll |
| 2 | 12/B2 | B | -Z -Roll |
| 1 | 9/B1 | B | +Z +Roll |

### Channel 5 - Command Module RCS Jets

| Bit | Jet | Quad | Reaction |
| --- | --- | --- | --- |
| 8 | 6/26 | B | -Yaw |
| 7 | 7/25 | B | +Yaw |
| 6 | 8/16 | A | -Yaw |
| 5 | 5/15 | A | +Yaw |
| 4 | 2/14 | A | -Pitch |
| 3 | 3/23 | B | +Pitch |
| 2 | 4/24 | B | -Pitch |
| 1 | 1/13 | A | +Pitch |

### Channel 6 - Command Module RCS Jets

| Bit | Jet | Quad | Reaction |
| --- | --- | --- | --- |
| 4 | 10/22 | B | -Roll |
| 3 | 11/21 | B | +Roll |
| 2 | 12/12 | A | -Roll |
| 1 | 9/11 | A | +Roll |

- _Reaction_ means direction of spacecraft motion when jet fires.
- _Control_ means direction of spacecraft motion used in software for that jet.
- +X direction is same direction as SPS engine thrust (roll axis positive
about this axis in right-hand rule sense).
- Quads in order B, C, D, A starting at the +Y (pitch) axis and going
clockwise (looking forward, i.e. along +X), Control axes offset from
spacecraft axes by a rotation of -7.25° (measured from spacecraft to
control axes about +X. axis).
- See spacecraft hardware documentation for location of individual jets.

### Channel 5 - Lunar Module RCS Jets 

| Bit | Jet | Cluster | System | Translation | Rotation | Failure Bit (ch. 32) |
| --- | --- | --- | --- | --- | --- | --- |
| 8 | 14 | 1 D | B | +X | +Q,+R,+U | 5 |
| 7 | 13 | 1 U | A | -X | -Q,-R,-U | 6 |
| 6 | 10 | 2 D | A | +X | -Q,+R,+V | 8 |
| 5 | 9 | 2 U | B | -X | +Q,-R,-V | 7 |
| 4 | 6 | 3 D | B | +X | -Q,-R,-U | 4 |
| 3 | 5 | 3 U | A | -X | +Q,+R,+U | 2 |
| 2 | 2 | 4 D | A | +X | +Q,-R,-V | 1 |
| 1 | 1 | 4 U | B | -X | -Q,+R,+V | 3 |

### Channel 6 - Lunar Module RCS Jets

| Bit | Jet | Cluster | System | Translation | Rotation | Failure Bit (ch. 32) |
| --- | --- | --- | --- | --- | --- | --- |
| 8 | 16 | 1 S | B | +Y | -P | 5 |
| 7 | 4 | 4 S | A | -Y | +P | 1 |
| 6 | 8 | 3 S | A | -Y | -P | 2 |
| 5 | 12 | 2 S | B | +Y | +P | 7 |
| 4 | 11 | 2 F | A | +Z | -P | 8 |
| 3 | 15 | 1 F | A | -Z | +P | 6 |
| 2 | 3 | 4 F | B | -Z | -P | 3 |
| 1 | 7 | 3 F | B | +Z | +P | 4 |

- _Translation_ and _Rotation_ mean direction of spacecraft motion when
jets fires.
- +X. iss through the upper docking tunnel (+P rotations about this axis in
right-hand rule sense, for _yaw_), i.e. in dlrection of APS/DPS thrust.
- +Z is through the forward tunnel (+R rotations about this axis in _roll_).
- +Y completes right-hand set (+Q rotations about this axis in _pitch_).
- In ihe software, rotation control for channel 5 is oriented about the U, V
axis system, where +U is through cluster 4, between +Z and +Y, and +V is
through cluster 1, between +Z and -Y. The actual software outpuls are about
(non-orthogonat) U', V' axes (defined so as to avoid cross-coupling effects,
and restricted by software to be reasonably close (e.g. 15°) to U, V axes).
- Clusters are numbered clockwise starting at +Z (looking along +X), with jets
pointed up (U), down (D), foreward (F), or to the side (S).
See spacecraft hardware documentation for locations of individual jets.

## IIF Fixed Memory Mechanism

The fixed memory is implemented by a collection of 3072 magnetic
cores, each of which is suitably threaded or bypassed by 20B wires (192
sense lines, 14 inhibit lines, 1 set line, and 1 reset line). A given
core is used to determine the information from two addresses in each of
six consecutive banks, or a total of 12 addresses (12 x 3072 = 36,864).
Readout of the contents of a given address is accomplished by appropriate
hardware address decoding logic causing a particular core to be set
(magnetized in a certain direction) and then reset. The changing magnetic
field during the reset induces a voltage in the sense lines that are
threaded through the core (like an ordinary transformer), but not in the
sense lines that bypass the core. Additional hardware address decoding
logic selects the output of a set of 16 sense lines (called a strand: there
are 12 strands associated with each core, one for each address associated
with the core, giving ihe 15 x 12 = 192 sense lines with a core).
These 16 sense lines have the contents of the indicated address represented by
the presence (binary 1) or absence (binary 0) of an induced voltage: as
discussed in Section IIA, the 15 bits associated with a given address
reflect 15 tr "iformation" bits and an odd parity bit to make the total
number of binary ones in the 15-bit word an odd number.

Although a detailed knowledge of the logical design of the memory
is not required to review the program, some knowledge of its mechanization
is desirab1e for proper evaluation of the impact of program changes upon
the hardware. As discussed in Section IIB, the fixed memory is divided
into a collection of 36 banks, each of which conlains 1024 cells (giving
the fixed-memory capacity of 36 x 1024 == 36,864 cells). Banks 02 and 03
can be addressed independently of the FBANK register, and banks 00 - 27
are addressed independently of the contents of SUPERBNK ( channel 07).
Banks 30 - 37 are addressed for SUPERBNK contents of 3 or less (using
FBANK in range 30 - 37), and the remaininrg banhs 40 -43 are addressed
for SUPERBNK contents of 4 (using FBANK in range 30 - 33). It is
conventional to use the fixed memory bank number (in octal, of course)
to identify individual banks, and this convention is followed below,
without further reference to the method whereby the bank number is determined
from the contents of the S-register, FBANK, and SUPERBNK registers.

The allocation of the contents of individual banks to computer
hardware is reasonably straightforward, but can best be explained after
a digression to review mechanical design of fixed memory:

1. The fixed memory consists of three "rope assemblies", caIled
"R", "S", and "T". Each rope assembly in turn contains two
"modules": B1 and B2 are in rope assembly R; B3 and B4 are in
rope assembly S; and B5 and B6 are in rope assembly T.

2. Each module has two "sides", each of which is divided into two
"areas" (giving 4 areas per nodule). Each side has a conmon
"set" line.

3. An area contains 128 cores (hence a module has 4 x 128 = 512
cores, and the 6 modules total 6 x 512 = 3072 corees). Each
of the cores in an area is threaded by the same "reset" line.

4. Each core is associated with a set of "inhibit" lines and with
12 strands (which, as mentioned previously, consist of 16 wires
each for the 15 "information" bits and the odd parity bit).
There is a total of 14 inhibit lines associated with each core.

The selection of a particular word stored in fixed memory is
accomplished as described on the following pages. Conputer hardware
documentation should be consulted for details of timing etc. not
covered here.

1) The rope assembly and module in that assembly are selected by the
value of the bank number:

```
Banks 00 - 05 select rope R, module B1.
Banks 06 - 13 select rope R, module B2.
Banks 14 - 21 select rope S, module B3.
Banks 22 - 27 select rope S, module B4.
Banks 30 - 35 select rope T, module B5.
Banks 36 - 43 select rope T, module B6.
```

Hence each module has 6 banks, with the 6 modules giving the computer
capacity of 35 banks.

2) One core (out of 128) in each of the four areas of the selected
module is chosen by means of bits 7-1 of the S-register, whose one
and zero outputs are connected to a total of 14 inhibit lines so
threaded that all except one core in the area will receive an inhibit
current (note that 2^7^ = 128).

3) A current pulse through the set line associated with one side of the
selected module is produced. Because of the inhibit action of item 2,
only two cores (one in each of the two areas on the selected side of the
selected module) will become set. The side which is pulsed is selected
by bit 9 of the S-register, which drives "side A" if zero and "side B"
if one.

4) The strand (one out of 12) within ihe selected roodule is chosen by a
suitable combination of bank number and S-register information. The
strand (in range 1-12, a decimal number) is given by:

```
2 x (bank number modulo 6) + (bit 10 of S-register) + 1
```

The "modulo 6", operation is performed upon the decimal equivalent of the
bank number: it yields a result of 0 for the first bank in each module
and a result of 5 for the last bank (see #1 above), Address 07,3143
(FBANK = 07, S-register = 3143~8~) would be strand 4 (of module B2), since
the nodulo operation yields 1 and bit 10 of S-register = 1.

5) A current pulse through the reset line associated with one area of
the selected module is produced. The area (one of 4 in the nodule ) is
selected by bits 9-8 of the S-register, thus resetting one of the two
cores set in #3 above, and inducing a voltage into the sense lines that
are threaded through the core.

6) The output of the strand selected in #4 above is sensed to obtain
the required contents of the specified memory cell.

Another term associated with the fixed memory is "paragraph". The
paragraph is an octal, number giving a kind of "serial number" for the
infomation in fixed memory. Each paragraph consists of 256 words, and
the paragraph number is computed as follows:

```
4 x (bank number) + (bits 9-8 of S-register) + F
F = O for fiyed-fixed memory
F = 20~8~ for variable fixed memory.

Hence address 07,3143 would be in paragraph 4 x 7 + 10~2~ + 2O~8~ = 56~8~.

In addition to the check of the readout of each cell which is provided
by the parity bit, individual banks in the fixed memory can be checked
by means of a memory-cell summing routine which is included in the
computer self-check portion of the fixed memory. This routine sums the
contents of the addresses in each bank, halting when the last cell is
reached that has been wired, and either checks that the magnitude of
the sum is equal to the bank number or provides a display of the sum for
manual review, depending upon the original manual inputs that initiated
the check. The routine starts with the first cell in the bank and sums
successive cells until two consecutive cells with contents equal to their
addresses are found (or the last cell in the bank is read). If the cell
contents equal its address, this is a one-step loop: two such cells in sequence
would not serve a functional program purpose (one such cell, of course,
might be preceded by an index order so that transfer to a different
cell would actually be performed). The summing routine halts after
including in the sum the cell following the two consecutive cells with
contents equal to their addresses (or after the final cell in the bank
is reached).

The usual nethod for ending the wired cells jn a bank (for the
summing routines to work, no gaps in wired cells within a given bank can
be permitted) is with two transfer-control (TC) orders to the present
step (giving address contents equal to address, since the octal operation
code for TC is O), followed by a "checksum" (or "bugger word" as it is
called by the G&N contractor). This checksum is computed by the assembly
program, and it is formed so as to make the sum of the complete bank
(including that cell) equal in magnitude to the bank number. The assembler
operation BNKSUM is used to generate the required (up to 2) transfer-
control orders at the end of the bank (the octal bank number is in the
address field of the BNKSUM order), followed by the che ckecksum word. This
operation can be placed at any point in the assembly, and has the capability
of omitting the generation of the transfer-control orders (indicated by
"NO NEED" in the cell contents field) if the bank is full of functional
orders. In addiiion, the number of words left (computed as 1023 minus the
number of functional orders) in the bank is printed to the left of the
cell contents field: cell 1024, of course, must contain the check sum. If
no words are left, the statement "NO WORDS LEFT" is printed. A separate
fixed memory constant is used to specify within the summing routine what
the last bank entering the sum is to be, in the form of "BBCON" (an
operation which sets the octal cell contents to ihe proper value in
BBCON format), with blank address field.

The algorithm used to compute the sum of each bank consists of the
following machine language instructions, whose individual performances
are described in Section IVB. The symbol CELL represents the contents
of successive fixed-memory cells, read in sequence of increasing S-register
contents, and SUM is the value of the sum (set zero at the
start of each bank):

```
    CA CELL (clear add)
    AD SUM (add)
    TS SUM (store, skip next order if overflow and set A = 1 sgn SUM)
    CA ZERO
    AD SUM
    TS SUM
```

Considering the quantities to have scale factor B14, the algorithm
may be described as:

```
SUM = SUM+CELL
IF |SUM| >= 16384:
   SUM = SUM - 16383 sgn SUM

The check sum word is formed by the assembler in such a way as to give it
the smaller of its two possible magnitudes: if the sum of the cells prior
to the word is positive, for example, the word is formed so as to yield
the positive bank number. Banl< OO, of course, would have a sum of -O.

## IIG Arithmetic and Overflow

Although most of the mechanization details of the arithmetic unit
are not of interest from a programming viewpoint, some of its features
are instructive for analysis of program performance. The adding-type
arithmetic unit (ignoring some special-purpose provisions) makes use
of ones complement arithmetic when operating with most computer
instructions. Because of this, the quantity "zero" has two possible
representations : OOOOO~8~ and 77777~8~, which are designated as +0 and -0
respectively. Exceprt for some special cases involving two zero-
nagnitude operands (including (+0) + (+0) and (+0) - (-0)) , the
"zero" that results from addition or subtraction will be a negative zero.

Although most of the machine language orders (described in detail
in Section IV) make use of the computer hardware arithmetic registers
(A, L, or Q) for arithmetic manipulations, three instructions (AUG, DIM,
and INCR) are included for changing the contents of an erasable memory
cell (by +/- 1) without affecting the information in the arithmetic
registers. This feature is included in the computer logical design because
of the necessity for processing the counter interrupts described in
Section IIH without the execution time penalty that would be required to
save and then restore the arithnetie registers. To achieve this
capability, the adder in the arithmetic unit is not functionally composed
of addressable arithmetic registers: instead, a set of input gates is
used to proride binary levels corresponding to the operands, and output
leve1s corresponding to the answer nay be gated to the appropriate
destination as desired, Most data transfers in the computer hardware
take place by gating various registers to "write amplifier" inputs, and
the amplifier outputs are gated to the necessary destinations. Because
of this design, it is unnecessary, for example, to go through the
adder to load the A register (accumulator, see Section IlC).

In addition to the ones complement arithmetic operations included
in the order code, there is also a special instruction (MSU) which may
be used to form the ones complement difference of two twos complement
numbers: such numbers generally would be obtained from CDU angle data,
so that 2^15^, rather than (2^15^ - 1), different numbers can be represented,
a hardware convenience for representing points on a circle. The MSU
order is performed in the arithnetic unit by forcing an end-around carry
and by setting S~2~ = S~1~ (see below) at the completion of the operation.
If the second operand is 00000~8~, this process converts the first
operand from twos complenent to ones complenent; if the two operands are
equal, the result is +O (another exception to the rule that -0 results
from most computer arithmetic operations that yield a "zero" answer).

When a word is read out of the memory into one of the arithmetic
registers (A, L, or Q), bits 14-1 (the magnitude information) are
placed in their corresponding bit positions of the register. Bit 15 of
the memory word (the sign bit) is placed in both sign positions (ident-
fied for the A register as S~2~ and S~1~, where carries from bit 14
propagate to S~1~ and from S~1~ to S~2~) of the register. The S~2~ bit is
considered as "the" sign bit (for program control instructions sensing
the sign of A), and in general is the bit stored in memory for sign
information. The adder of the arithmetic unit, however, is connected
to S~1~ and S~2~ separately, so that arithmetic operations can effectively
make use of a 16-bit word. The full 15 bits can be used for transfers
of data between the A and Q registers, but the S~1~ bit is lost in transfers
between the L and A registers (such as XCH L with overflow in A).

Under normal conditions, the S~1~ and S~2~, bits will be equa1. After
an addition or subtraction operation in which overflow took p1ace, how-
ever, the bits wiIL be unequal. It shoul-d be errident that bit S, has
the overflow information that would have been propagated to the next
most significant magnitude bit (if the word length of the computer were
bigger), and advantage of this fact is taken in the TS (Transmit to
Storage, see Section IVB) order code instruction. To avoid improper
answers, S~1~ should egual S~2~ before division or multiplication operations
are performed; for addition and subtraction, however, the S~1~ bit is
effectively another magnitude bit and can be used as such: 1/4 + 1/2 + 1/2 - 1/2
(computed in that order) will give an answer of (3/4), provided of
course that the sum of the first three terms is not stored by a TS order.

Storage of the accumulator contents into memory causes the overflow
bit (in the quantity stored) to be lost, since the 16-bit memory word has
an odd parity bit instead of the overflow bit. If the TS instruction is
used, presence of an overflow (established by the fact that S~1~ != S~2~) will
cause the next instruction to be skipped and the least significant bit of
the accumulator to be set to +/-1, as described in Section IVB. A
similar setting is employed for the DAS instruction. Since S~1~ as
described previously, has the features of an additional nagnitude bit,
it is used in place of S~2~ for the storage of certain counter-inrcrementing
orders that require twos complement, arithmetic (and the MSU order), as
well as those counter orders requiririg the assembly of a serial stream
of input bits.

The computer order code includes four instructions that make use
directly cf double precision operands: DAS, DCA, DCS, and DXCH. The
interpretive language described in Section VI perrmits portions of the
computer program to be written almost as if the whole computer had
nothing but double precision operations, however. The double precision
machine language orders operate on the least significant half of the
double precision word first, using the computer L register. The address
associated with the order is then decremented by one and the most
significant half of the word processed. Hence DXCH L, for example, starts
by putting L in Q and Q in L, and then puts A in L and the (new) L in A,
giving the net effect of putting A in L, L in Q and Q in A.

There is no hardware requirement that sign agreement exist between
the two halves of the double precision words: they are treated essentially
as independent single precision quantities unless there is need to propagate
a carry (or borrow) from the least significant half. The DV (divide)
order employs a double precision dividend (in A,L) and forces sign agreement
by hardware means before initiating the division sequence.

The assembly program increments the address of the symbol provided
for a double precision order so as to read the least significant half
first (as described above). Consequently, the symbol provided with the
double precisicn order (either an absolute or a symbolic address) must
be that of the most significant half of the word, and naturally the last
cell in a switched memory bank cannot be considered the "most significant
half" for such orders.

A detailed description of the hardware algorithms employed for
multiplication and division can be found in the appropriate hardware
documentation, and therefore is not included here. To minimize
execution time, these algorithms are fairly elaborate. See Appendix A
for more details on addition and overflow.

## IIH Interrupts

There are two distinct types of interrrupts incorporated in the
computer logical design: counter interrupts and program interrupts.
Since they are quite different, separate sub-sections are devoted to
each below.

Since the counter interrupts represent one hardware approach
(others could have been selected, although probably with the need for
additional hardware) to the mechanization of computer inputs driven by
external signals, their existence for most programming purposes can be
ignored. Program interrupts, on the other hand, perform an integral
portion of the program control logic: consequently, it is conventional
that the term "interrupt", unless otherwise specified, refers to these
program interrupts.

### Counter Interrupts

The 29 counter interrupts in the computer are associated with the
29 erasable memory cells (0024~8~ - 0060~8~, see Section IID) that may contain
counter-type information. Seven "involuntary" (i.e. not under computer
program control) counter instructions are associated with these counters,
and can be performed when an appropriate counter interrupt is received.
In some cases, a counter interrupt can select different involuntary
instructions to be performed, depending on the nature of the external
signal (such as positive or negative changes to the value of a counter)
or the value of the quantity in the counter (positive or negative output
pulses). The seven involuntary instructions, and the cells to which
they apply, are given on the following pages.

1. DINC, applying to cell 0031~8~ (TIME6) and cells 0047~8~ - 0056~8~
(GIROCMD, CDU error counter drives, THRUST, and not used).
If the contents of the ce1I are positive non-zero, they are
decremented by 1 and positive output pulses are provided; if
the contents are negative non-zero, the contents are incremented
by 1 (i.e. nagnitude decreased by 1) and negative output pulses
are provided. Output pulses must be enabled from 0047~8~- 0056~8~ by bit
of channel 14 (10, 15-11, 4, and 5 respctively), which is reset
to O when the counter contents are equal to -0 and another
DINC pufse is generated (-0 is the result of a DINC to a cell
equal to +1 or -1). Consequently, zeroing of cells by program
means must load -0, not +0. Use of DINC with cell 0031~8~ is
enabled by bit 15 of channel 13, although the cell's output
pulses are not used: instead, its decrement to -0 causes
program interrupt #1 to be generated at the next DINC (and the
enabling bit to be reset).

2. MCDU, applying to cells 0032~8~- 0036~8~(input CDU angles from
IMU and optics/rendezvous radar). This instruction subtracts 1
(in twos complement) from the contents of the cell.

3. MINC, applying to cells 0037~8~ - 0044~8~ (accelerometer inputs and
RHC/unused BMAG analog inputs). This instruction subtracts 1
(in ones complement) from the contents of the cell.

4. PCDU, applying to cells 0032~8~ - 0036~8~ (input CDU angles from
IMU and optics/rendezvous radar). This instruction adds 1
(in twos complement) to the contents of the cell.

5. PINC, applying to cells 0024~8~ - 0030~8~ (TIMEi, i = 1-5) and cells
0037~8~ - 0044~8~ (accelerometer inputs and RHS/unused BMAG analog
inputs). This instruction adds 1 (in ones complement) to the
contents of the cell.

6. SHANC, applying to cells 0045~8~ - 0046~8~ (INLINK and RNRAD). This instruction shifts
the contents of the cell left one place, and then adds 1
(it is used for a binary one of a serial bit stream).

7. SHINC, applying to cells 0045~8~ - 0046~8~ (INLINK and RNRAD) and
to cells 0057~8~ - 0060~8~ (unused OUTLINK and ALTM). This
instruction shifts the contents of the cell left by one place
(it is used for a binary zero of a serial bit stream or to
generate a serial output bit stream from the cell overflow bit).

A counter interrupt request can be generated (in general) at any
time. AIl requests are retained by the hardware until the end of the
current computer instruction. At that time, provided that the next
instruction is not a special-purpose TC order (EXTEND, INHINT, or RELINT),
the request is honored. This means, for example, that a double precision
computer instruction (such as DCA) can be used to sample the values of
cells 0024~8~ - OO25~8~ (the computer clock) without concern that a counter
interrupt will cause the two halves to be inconsistent due to an overflow
of ce1l 0025~8~ (see Section IID).

Satisfaction of a counter interrupt takes one MCT (memory cycle
time of about 11.7 microseconds) per request (due to the need to read
the counter from memory, modify it, and store it back). Priority for
satisfaction of the requests is based upon the value of the counters
address (0024~8~ has the highest priority and 0060~8~ has the lowest), but
all requests are satisfied before the next program instruction is
started. See Section VII for the sequence with which the computer
hardware perfoms its various functions.

Counter interrupts are not under computer program control (once
the appropriate control bits, in some cases, have been set), cannot be
inhibited by the program, and in fact can only be determined by the
software to have occurred by sampling the cell in question. It is
sometimes necessary (such as when the accelerometer cells are sampled)
to sampie and reset a counter without losing any counts: the machine
language order XCH (Exchange) can be used for this purpose, since this
order exchanges the contents of the A register and the cell specified
by the address field of the order. In other instances, it is necessary
to change the value of an output-generating counter cell. (such as the
cell used to generate gyro torquing pulses) while it may be controlling
output pulse generation. In this case, the machine-language order
ADS (Add and Store) can be used.

### Program Interrupts

Eleven program interrupts are incorporated into the computer
design. Most interrupts (provided certain conditions are satisfied)
cause the performance of the program to be suspended, the contents of
certain registers to be saved (some by hardware means, some by software),
and the next instruction to be executed be the one at a special address
(different for each interrupt) in order to start the "task".

The interrupt is mecnahized through the involuntary instruction
RUPT, which takes 3 MCT to perform. If necessary, it can also be
programmed as EDRUPT, an extended order, using the following sequence:

```
     CA start address desired, in ADRES form
     TC bank 3 address (i.e. in fixed-fixed, form 7xxx~8~)
     -------
BNK3 EXTEND (BNK3 is address to which TC is done, form 7xxx~8~)
     EDRUPT BNK3
```

This sequence causes the hardware to initiate computations at the ADRES
address contained in the accumulator, with various hardware flip-flops
set as they would be for a "normal" hardware induced program interrupt
(FBANK setting is that from which the BNK3 step was entered). In either
case, resumption of the program is triggered by the special purpose
instruction RESUME (triggered by INDEX order for cell 0017~8~, see Section
VA), taking 2 MCT to perform. The mnemonic stems from the
phrase "Ed Smally's interrupt instruction".

The individual interrupts, with their titles, starting addresses,
causes and functions are:

1. T6RUPT, starting address 4004~8~, generated by the next DINC after
TIME6 (cell 0031~8~, see Section IID) has been reduced to -0.
Conventionally used to control the timing of RCS jet commands in
output channels 05 and 06 (by suitable software).

2. T5RUPT, starting address 4010~8~, generated by overflow of TIME5
(cell 0030~8~, see Section IID). Conventionally used to control
cycling of computations associated with the digital autopilots
(jet timing conventionally controlled by program interrupt #1).

3. T3RUPT, starting address 0014~8~, generated by overflow of TIME3
(cell 0026~8~, see Section IID). Conventionally used to control
performance of "waitlist" tasks (see Section VIIA).

4. T4RUPT, starting address 4020~8~, generated by overflow of TIME4
(cell 0027~8~, see Section IID). Conventionally used to control
cycling of periodic input/output functions (such as driving of
DSKY digits, see Section IIJ).

5. KEYRUPT1, starting address 4024~8, generated by depression of a
key on the DSKY keynoard (main panel DSKY for CM). Input trap
circuit reset when key is released. Used by software to
initiate processing of keyboard input from channel 15.

6. KEYRUPT2, starting address 4030~8~, generated for CM by depression
of a key on lower equipment bay (or "navigation panel"). DSKY or
depression of optics mark or mark reject button. For LM, it
is generated by depression of a mark or mark reject button or
by rate-of-descent switch offset. Input trap circuit reset when
key or button released, or rate-of-descent switch returned to
middle (neutral) position. Used by software to start channel 16
processing.

7. UPRUPT, starting address 4034~8~, generated by overflow of cell
0045~8~ (INLINK, see Section IID) due to shifting of the first
binary 1 (in the 16-bit word sent to the computer) out of the
cell. Used by software to start processing of information in
INLINK (including its reset). If the checks are passed, the
same computational job is established as that for program
interrupts #S and #6 if a DSKY input is involved.

8. DOWNRUPT, starting address 4040~8~, generated by an end pulse
from the telemetry system. The basic telemetry format consists
of eight-bit data words transmitted at a rate depending on the
setting of spacecraft switches. At the "high bit rate" (51.2
kbps), 5 of the 128 words in each frame are allocated to computer
digital data (giving 40 bits), thus permitting 50 of the 40-bit
computer words to be sent per second. Computer words are loaded
for downlink transmission in channels 34 and 35 (plus bit 7 of
channel 13 for "word order code" information). The 40 bits
are transmitted in the following sequence:
  a. Bit #1 is the word order code bit.
  b. Bits #2 - #16 are bits 15-1 (sign first) of channel 34.
  c. Bit #17 is an odd parity bit for channel 34 data.
  d. Bits #18 - #32 are bits 15-1 (sign first) of channel 35.
  e. Bit #33 is an odd parity bit for channel 35 data.
  f. Bits #34 - #40 are the same as bits #2 - #8 (i.e. bits
  16-9 of channel 34).
After the final bit, the end pulse from the telemetry system is
received, generating the interrupt (request). At the high bit
rate, the program has about 19.2 ms in which to respond to the
interrupt and load new data into channels 34-35 before the
transmission is started again. Garbled downlink data, of course,
would result if loading not accomplished (ground resynchronization
could be accomplished when the word order code bit flagged data).
The "low bit rate" in the CM is 1.6 kbps (200 eight bit words per
second), in which 50 of the 200 words are digital data (giving
an end-pulse rate of one every 0.1 second rather than the rate of
one every 0.02 second at the high bit rate.) In the LM no LGC data
is transmitted at low bit rate (hence AGC initialization, for example
must be accomplished at high bit rate). If bit 12 of channel 33 is a 
binary 0, this indicates that a telemetry end pulse was rejected.

9. RADAR RUPT, starting address 4044~8~, generated by completion of the
shifting of radar into cell 0046~8~(RNRAD). The time
delay between the setting of bit 4 of channel 13 and the
generation of the interrupt is 90-100ms (see Section IID). Used
by software to start processing of information in RNRAD.

10. HAND CONTROL RUPT, starting address 4050~8~, generated by the
setting of interrupt traps 31A, 31B or 32. These traps are
reset by bits 12-14 of channel 13 respectively, and are required
because of the duration of the input signals (which otherwise
could produce multiple program interrupts). Trap 31A is
associated with bits 6-1 of channel 31 (rotational hand controller
deflections); trap 31B is associated with bits 12-7 of channel
31 (translation hand controller interrupts); and trap 32 is
associated with bits 10-1 of channel 32 (CM minimum impulse
controller and LM; thruster fail and descent engine gimbal fail
inputs). A signal fed into indicated bit positions causes the
indicated trap to be set. In the CM software, this program
interrupt is not used, since sampling of the input signals
involved is done sufficiently often as a consequence of the
normal digital autopilot cycling. In the LM software, a similar
argument applies (the digital autopilot cycling and logic performs
functions equivalent to those originally intended by the hardware
design), so that only trap 31A is employed in order to monitor
for hand controller deflections associated with the landing point
designation (see bits 6,5,2, and 1 of channel 31, Section IIE).

11. GOPROG, starting address 4000~8~, caused by an internally generated
hardware signal in response to various hardware difficulties.
A "hardware restart" is produced, as described in more detail below.

Program interrupts #1. #10 have the following common features:

a. Their first few steps store A in ARUPT, L in LRUPT and transfer
control to a routine that performs the necessary computations
(after saving Q and/or BBANK and/or SUPERBNK if necessary).

b. They initiate the performance of a task, at the conclusion of
which (after restoration of A, L, and any other cells necessary)
the operation RESUME (see Section VA) causes the program to
start again from where it was interrupted, provided of course
that another program interrupt is not waiting to be processed.

c. Their priority for initiation is the order in which they were
listed above (#1 is the highest and #10 is the lowest). Once
a program interrupt has had its processing started, however,
it will continue on to completion: the "priority" is significant
only in determining which interrupt should be processed first.

d. They will not be acted upon (processed), but instead will be
retained for future action, if any of the following criteria
are satisfied:
  1. The current machine language order is not yet complete.
  2. An "extended" machine language order is about to be
  performed (see Section IV), since information retained
  when interrupt processing is started does not include
  the "extended order code" bit.
  3. An accumulator overflow (see Section IIG) condition
  exists, since information retained when interrupt
  processing is started does not include the overflow bit.
  Other overflows (e.g Q register) are not protected.
  4. The INHINT/RELINT flip-flop (see Section VA) is set
  to inhibit program interrupts, meaning that interrupts
  not desired by programmer (permitting flagword bits to
  be changed, downlink state vectors to be consistent,m etc.)
  5. A program interrupt (even one of lower priority) is
  already being processed.
  6. A special-purpose TC order (EXTEND, INHINT, or RELINT)
  is the next instruction to be executed.

For a summary of the sequence in which the computer hardware (and
software) performs its various functions, see Section VII.

Program interrupt #11 ( sometimes referred to as interrupt #0)
differs in a number of respects from the others. It does not result
in "normal" resumption of the program (instead, a "restart" is performed,
see section VIIC); it takes absolute prioirity over other program interrupts;
it cannot be inhibited; and it can even "interrupt an interrupt". As
part of its generation, a special involuntary interrupt instruction is
produced, causing a master clear signal ("GOJAM") to be generated by the
hardware. Program interrupt #11, which is also termed a "hardware
restart" (to distinguish it from similar functions that can be done solely
by software), can be triggered by the following:

1. Indication of power failure on the prime 28-volt supply (below
about 22.6 volts), the 14-volt supply (below 12.5 voltsor above 16 volts)
or the 4-volt supply (below 3.65 volts or above 4.4 volts).
2. Detection of a computer oscillator failure.
3. Detection of a large program loop ("night watchman"), revealed
by failure to address erasable memory cell 0067~8~ (NEWJOB, see
Section IID) in a period ranging from 0.64 to 1.92 seconds.
4. Detection of a transfer control failure ("TC trap"), revealed
by having a TC or TCF (see Section IV) order in effect for a
period of from 5 to 15 ms (or a counter interrupt), or if no
TC or TCD order is exeeuted in this same time interval. The
software can cause a hardware restart by this means, through
a TC order to the present step.
5. Detection of a parity failure on a uord read from fixed or
erasable memory (applies to all addresses of vatue 00l0~8~ or
above), A failure would be revealed if an even nunber of
binary ones were in the word (see Section IIA).
6. Detection of a program interrupt failure ("RUPT lock"), revealed
if a program interrupt is continuously in effect for a period of
from 140 ms to 3OO ms, or if no program interrupt takes place in
this same interval.
7. Recovery from "standby" operation. This is analogous to item
#2 above, since standby operation removes power from the monitor
circuit, causing the hardware to consider standby an "oscillator
fail" condition, even though the oscillator keeps on running
(so that time information can be obtained from channels 03 and
04, see Section IIE) .

### Peripheral Equipment Orders

There are five interrupt-type instructions which may be originated
from computer peripheral equipment, i.e. the CTS (Conputer Test Set)
or PAC (Progran Analyzer Console). Although these instructions are not
used when the computer is in a flight envirorment, they are listed
here in the interest of completeness:

1. FETCH 2. MCT Display content of specified address
2. INOTLD 1. MCT Load specified channel
3. INOTRD 1. MCT Read specified channel and display it.
4. STORE 2. MCT Store data in specified cell. Should not be
confused with interpretive language order having the same mnemonic.
5. TCSAJ 1. MCT Transfer control to specified address.

## IIJ Display System

Most of the outputs from the computer for display purposes are
transmitted through channel 10, which is assigned the mnemonic "OUTO".
Bits 15-12 of this register define a particular row of relays (which
are of the latching type) to be driven, while the remaining eleven bits
specify the new settings for these relays. Since the relays are bistable
devices, retaining either a binary 0 or a binary 1 state until changed,
register OUTO need retain the specification of the contents of a row
for onto 0.02 seconds (which also helps minimize power consumption and
heat buildup), following which the channel is zeroed for O.O2 seconds
before another row is specified. Under certain conditions (see equation
documentation for details) , the software allows a new row setting to be
specified every 0,04 seconds, permitting a complete change of the eleven
rows controlling the DSKY numerical and sign displays in less than
1 second.

Relay rows 1-11 (selected by having bits 15-12 of OUTO equal to
01~8~ -13~8~ respectively) are used to drive the digit and sign displays on
the DSKY (display ard keyboard assembly), while relay row 1-2 (bits
15-12 of OUTO equal to 14~8~) is used to drive some of the indicators on
DSKY (most of the other indicators are driven from channel 11 bits,
see Section IIE). The displays and indicators which are energized consist
of three banks (R1, R2, and R3 registers) of five digits and a sign; three
banks (noun, verb, and program or mode) of two digits each; 9(CM) or 11(LM)
indicators on the DSKY (two others are driven by separate hardware); and
a request for an operator action (FLASH, which causes the verb and noun
displays to blink on for O.64 seconds, then off for 0.64 seconds).

Numbers are spcified for display on the DSKY by a total of five
bits, while sign and indicator inforrnation require one bit each. The
individual bits of the first eleven rows have the following meanings:

| Row | Bit 11 | Bits 10-6 | Bits 5-1 |
| --- | --- | --- | --- |
| 01 | -R3S | R3D4 | R3D5 |
| 02 | +R3S | R3D2 | R3D3 |
| 03 | | R2D5 | R3D1 |
| 04 | -R2S | R2D3 | R2D4 |
| 05 | +R2S | R2D1 | R2D2 |
| 06 | -R1S | R1D4 | R1D5 |
| 07 | +R1S | R1D2 | R1D3 |
| 10 | | | R1D1 |
| 11 | | ND1 | ND2 |
| 12 | | VD1 | VD2 |
| 13 | | MD1 | MD2 |

In this tab1e, the row numbers are cited in octal (as loaded into bit
positions 15-12 of OUTO), and "D" means digit, with D1 the most significant
and D5 (or D2) the least significant. R1 - R3 refer to the three
display registers, each with sign ("S"), while "N" "V", and "M" refer
to the noun, verb, and mode (or program) two-digit registers respectively.

The pattern of the digit displayed on the DSKY panel is specified
by the five bits assigned to the character, according to the table on
the following page. In this tab1e, the five bits of the pattern are
identified as B5 through B1 respectively. The prime in the formula for
a particular display segment designates a complement. A numerical
entry in the table means that the display segment indicated at the left
is energized for the display of that digit.

| Display Segment | Formula | Blank | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| | | Pattern | 00 | 25 | 03 | 31 | 33 | 17 | 36 | 34 | 23 |
| Top | B5 | | 0 | | 2 | 3 | | 5 | 6 | 7 | 8 |
| Middle | B4 | | | | 2 | 3 | 4 | 5 | 6 | | 8 |
| Left Upper | B3 | | 0 | | | | 4 | 5 | 6 | | 8 |
| Right Upper | B2 | | 0 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 |
| Left Lower | B2' (B1+B5) | | 0 | | 2 | | | | 6 | | 8 |
| Right Lower | B2 + B2' B3 | | 0 | 1 | | 3 | 4 | 5 | 6 | 7 | 8 |
| Bottom | B5 (B3+B4) | | 0 | | 2 | 3 | | 5 | 6 | | 8 |

Most keys on the DSKY are used to generate a five-bit code which
appears in bits 5-1 of channel 15 (and, for CM, channel 16 in the case of
the DSKY in the lower equipment bay). The same codes are used for the
corresponding characters when transmitted via uplink means to cell 0045~8~
(INLINK, see Section IID). The individual key codes are:

| Key | Code | Function |
| --- | --- | --- |
| 0 | 20~8~ | Digit zero |
| 1-9 | 01~8~ - 11~8~ | Digits one to nine (code corresponds to the decimal value) |
| Verb | 21~8~ | Indicates that the next two digits specify verb code, indicating action desired |
| Noun | 37~8~ | Indicates that the next two digits specify noun code, indication action recepient |
| + | 32~8~ | Indicates that up to the next five digits specify a positive data number (loaded into Ri register) |
| - | 33~8~ | Indicates that up to the next five digits specify a negative data number (loaded into Ri register) |
| Clear (CLR) | 36~8~ | Causes Ri register to be cleared (if software checks passed satisfactorly) |
| Key Release (KEY REL) | 31~8~ | Indicates to software that operator is releasing display system for internal operator control of display |
| Enter (ENTR) | 34~8~ | Indicates to software that either execution of the verb/noun direction should be perfomed or that data that has been keyed into an Ri register is complete. |
| Error Reset (RSET) | 22~8~ | Indicates to software that various error indicators (and internal alarm-code cells) are to be reset. Turns off Restart light by hardware means. Uplink (but not DSKY input) also resets software bit that set after failure of check of
INLINK input (see cell 0045~8~ in Section IID). |

In addition to these key codes, the PRO button is also on the DSKY, and
causes bit 14 of channel 32 to be 0 when it is depressed (see Section
IIE). If bit 11 of channel 13 is set, this button also used to put
computer into standby (1ower-power consumption) mode if pressed for
0.64 - 1.92 seconds, and to return it to normal operation when pressed
again for the same interval.

The various bits of relay row 14~8~ have the following significance:

| Bit | Light | Function |
| --- | --- | --- |
| 11 | | Not assigned |
| 10 | | Not assigned |
| 9 | PROG | Set by program to indicate that a program check has failed |
| 8 | TRACKER | For CM, set by software to indicate an optics CDU fail; for LM, set by software to indicate a radar nalfunction. See mission documentation. |
| 7 | | Not assigned. (has been used for test purposes) |
| 6 | GIMBAL LOCK | Set by software to indicate approach of middle gimbal angle to a "lock" condition (such as an angle in excess of 70°). |
| 5 | ALT(LM-only) | Set by software (or flashed) to indicate data difficulty with landing radar altitude. Not connected in CM |
| 4 | NO ATT | Set by software to indicate that inertial subsystem not suitable as an attitute reference (because it is off, caged, or in coarse align) |
| 3 | VEL(LM-only) | Set by software (or flashed) to indicate data difficulty with landing radar velocity. Not connected in CM |
| 2 | | Not assigned |
| 1 | | Not assigned |

The STBY light is energized by the computer hardware if the
computer is nn the standby mode of operation, while the RESTART light
is energized if a computer restart (program interrupt #11, see Section
IIH) is encountered. For testing purposes, both lights can be energized
by bit 10 of channel 13. The RESTART light can be turned off by the
Error Reset key or by bit 10 of channel 11.

The various lights on the DSKY panel, and the source from which
lhey are driven, are summarized in schematic form on the next page.

| --- | --- | --- | --- | --- | --- | --- |
| UPLINK ACTY | ch.11 bit 3 | TEMP | ch. 11 bit 4 | COMP ACTY | ch. 11 bit 2 | prog |
| white | | yellow | | green | | MD1 MD2 |
| | | | | | | |
| NO ATT | row 14~8~ bit 5 | GIMBAL LOCK | row 14~8~ bit 6 | | | |
| white | | yellow | | | | |
| | | | | | | |
| STBY | hardware | PROG | row 14~8~ bit 9 | verb | |   noun |
| white | | yellow | | VD1 VD2 | | ND1 ND2 |
| | | | | | | |
| KEY REL | ch 11 bit 5 | RESTART | hardware | R1S | R1D1 R1D2 R1D3 R1D4 R1D5 |
| white (flash) | | yellow | | | | |
| | | | | | | |
| OPR ERR | ch. 11 bit 7 | TRACKER | row 14~8~ bit 8 | R2S | R2D1 R2D2 R2D3 R2D4 R2D5 |
| | | | | | | |
| | | ALT(LM) | row 14~8~ bit 5 | | | |
| | | yellow | | | | | |
| | | | | | | |
| | | VEL(LM) | row 14~8~ bit 3 | R3S | R3D1 R3D2 R3D3 R3D4 R3D5 |
| | | yellow | | | | | |

TEMP also connected to channel 30 bit 15 (so light comes on if computer
is in standby).

Verb and Noun flash: ch. 11 bit 6.

STBY and RESTART also energized by ch. 13 bit 10.

All digits on DSKY display (and sign) driven from rows 01~8~ - 13~8~, as
shown on page IIJ-2.

The COMP ACTY and the digit (and sign) display are electroluminescent
displays, while the remaining inrdicator lights are incandescent.

The keyboard layout is as follows:

| --- | --- | --- | --- | --- | --- | --- |
| | + | 7 | 8 | 9 | CLR | |
| VERB | | | | | | ENTR |
| | - | 4 | 5 | 6 | PRO | |
| NOUN | | | | | | RSET |
| | 0 | 1 | 2 | 3 | KEY REL | |

# Format of Guidance Program Symbolic Listing

This section describes the format of a program symbolic listing
as reflected by typical programs, based on the current1y used assembly
program. New features and capabilities can be expected to be added to
this program, however, so it should be realized that items may be encountered
in a listing which are not described below. In addition, in the past the
printer character sets used by MSC and the G&N contractor have had some
differences (the G&N contractor's "?", for example, has been printed as
"π" or "&" in MSC listings, and some G&N contractor symbols are not
printed at all in MSC listings, such as an apostrophe in line printer
outputs). Symbols may also appear differently (a colon as an apostrophe,
for example).

The assenbler (referred to also as the assembly program) is quite
flexible in its capabilities, and unusually tolerant in the variety of
formats, such as spaces between digits of a nunber, that it will accept.
A "symbol" consists of from one to eight characters (with certain
restrictions), and is equated to a unique octal cell address by one of
the following means:

a. Specification as the tag associated with a quantity stored in
that octal cell location.
b. Specification as a tag equated (by the EQUALS or = pseudo operations)
or some other quantity, which may be another tag or an absolute address.

A symbol must be eight characters or less in length, and cannot consist
of an integer, an integer preceded by a plus or minus sign, or an
integer followed by the letter D. Aside from these restrictions,
however, the symbols which may be selected are quite varied, as suggested
by the following symbols seleeted at random from a sample program.

| --- | --- | --- | --- | --- |
| 0.00167 | 16/32400 | A | BITS6&15 | DV-+,+ |
| 11DEC | 1SEC+1 | A+B | BUF+ | .166... |
| 11DEC. | +-ZERO | ACOS=0 | D--SC | -TAN22.5 |
| 13-11,1 | (1-K),QR | A(X) | DLOAD* | NXTT6=P |

For proper performance of the assembler, of course, symbol definitions
must be unique, so that only one octal cell location corresponds to the
given synbol. The assembier distinguishes between the letter O and the
number 0.

Quantities not satisfying the format restrictions for a "symbol"
are used for other purposes by the assembler. An unsigned integer in
the address field (frequently found in interpretive language coding)
without an operand indication (OCT, DEC, etc.) is considered as an octal
absolute address (equivalent to an octal integer for values below
10000~8~, as discussed in Section IIB). If the integer is followed by
the letter "D", it is treated. as a decimal number. Hence both "36"
and "30D" would be loaded in the memory as 00036~8~. The quantity
"20000" (with no operand indication) would be loaded in the memory as
10000~8~ in view of its definition as an address: for such numbers, it
is conventional (and desirable) to specify explicitly the operand
information. The "D" is optional with "8" or "9".

A blank in the address field is considered to be an the address of
the step itself, and a signed integer (such as "+2" or "-3") wou1d be
translated relative to the step's own address (two steps beyond or
three steps earlier respectively). As mentioned previously, signed
integers are not allowable symbols, so they sometimes are used in
association with the relative addresses as a form of program "remark":
lhey have no effect on the performance of the assembly program.

A symbol followed by a space and then a signed integer is treated
by the assembler as if the value of the integer modified the octal
instruction (operation code and address). If the integer is of
sufficient size, it will cause modification of the operation code, thus
giving compatibility with the hardware INDEX order described in Section
IV. As with other integers in ihe address fie1d, the signed integer
is considered to be octal unless followed by the letter "D" (for decimal).
It should be noted that a space must be left between the symbol and the
signed integer, or else the net combination would be considered as another
symbol (as indicated by the symbol "1SEC+1" given on the previous page).
If it is desired to have the assembled address information be negative (for
use with INDEX for example), this can be accomplished by the artifice
of using "0 - n" there, since merely "- n" would be considered as a
relative address.

The address-field form (symbol +/- integer) is the only type of
address-field modification conventionally allowed by the assembler.
The effect of adding two symbols (or subtracting them), however, can
be achieved by appropriate use of the address operations "=PLUS" and
"=MINUS", which are described in Section VC. Repeated use of these
operations, of course, can achieve the effect of multiplication of
a symbol's octal equivalent by an integer.

## Page Layout

Each page of the program iisting has 120 columns of available space
for the printing of program information, and a maximum of 56 output lines
per page (of which the first four, including two blanks, are for header data).
The location and infomation which is printed is established by the
nature of the original input to the assembler, as explained in more
detail below.

The top line on each page contains a program identification which
is specified when the run is made. The assembler identification (e.g.
"GAP") appears in print columns 1-4, followed by the assembler action
(such as "assembler") and the revision number, name, and "author" of
the program being assembled. On the right-hand side of the page, the
time when the run was made appears (hours:minutes) for identification,
followed by the date on which the run was made. Print columns 112-115
contain "PAGE" and columns 117-120 contain the master page number
(starts at 1, right justified with leading zeros suppressed) used
throughout the assenbly to identify locations in the listing. An
additional piece of information included in the top Iine (before the
PAGE print), if applicable, is the Subroutine name and revision (see
Infornation at Start of Listing below).

The second line is blank, and the third is used to supply the
"log" identification information. In order to permit different people
to work on different areas of the program while minimizing their need
for close synchronization during the developnent of these areas, the
assembler information is divided into a number of separate segments
("log sections"), each of which may be modified individually through
specification of the appropriate sequence nunber (line identification)
within that particular segment. Each segment is assigned a tit1e,
which appears on ihe left-hand side of this second line of printing,
The printing of the title of the 1og section generally starts in print
colunn 9, and in addition an "L" (for "log card") appears in print
column 1. On the right-hard side of this Line appears the "USER's
PAGE N0. xxx", where xxx is restarted at 1 at the beginning of each
log section (leading zeros again suppressed). This is followed
in print columrs 111-112 by Ei, giving the most recent erasable memory
bank specification. The Ei identification is nulled (printing E0) at
the start of each 1og section, and can be used by the assembler to check
for possibly i1lega1 memory references (see EBANK= below). The last
piece of information in this line is Si, which appears in print columns
114-115. This gives the most recent SUPERBNK setting specification
(see SBANK: betow), and can be used to generate BBCON values.

Next comes another blank 1ine, completing the four lines allocated
for header data, The remaining lines (up to 54) on the page contain the
program information. For each line of this information, print columns
2-7 contain a "sequence number", which is restarted at the beginning
of each 1og section and which increases monotonically (when left
justified) throughout that segnent. This number is normally incremented
by +1 in print column 5, and is used to specify the location of changes
when making modifications to a log section. For example, the numbers:

````
0009
0010
00r01
00103
001031
001032
00104
0011
````

could appear in sequence on successive lines of coding. An assembler
capabiliiy exits to cause the sequence nuabers within a log section to
be redefined so as to count up uniformly in the counting position (column
5), but this option is not necessarily employed when a new listing is made.

Print column 1 is blank lor most lines of coding. If it is blank,
card columns 1-8O are printed as print columns 49-120, and the renaining
print colunns are filled with address, address content, and symbol
reference information as described in more detail below. Print column
1 can also contain certairn letters, which result in assembler operation
as follows:

| Letter | Description |
| --- | --- |
| "A" | signifies an "aligned remark card", which does not produce any binary memory information. Card columns 9-80 are printed in the same print positions as for a normal card (i.e. 49-120), and are generally used to provide additional comments that could not be fitted onto the same card as the original program step being described. |
| "L" | signifies a "log card", used to specify the segment of the program (and appearing on the second printed line of the page, as described above). |
| "P" | signifies "page", and causes a printer page-eject signal before it is printed (making the "P" line the first line of program information, or fifth line on the page). Otherwise, the "P" is treated the same way as an "R" card. |
| "R" | signifies "remark" and does not produce any binary memory information. Card columns 9-80 are printed in print columns 9-80. Print columns 81-120 may be filled by the information on another card, if that card has a "9" punched in column 8 (and is in the proper sequence-number order). |

Print column 8 contains a flag (such as "*") if the card on that
print line was changed in the most recent modification of the Subroutine.

## Card Layout

Although the assignnent of functions to rhe individual columns of
the cards that are input to the assembler is not of direct concern unless
cards nust be punched, the card format serves as a methodical explanation
of some of ihe features of the assembler, and also can be useful in
reviewing lists of program changes that might be provided in the form
of a listing of input cards to the assembler.

- Column 1 is used for specification of the type of input: blank for
normal input, and otherwise A, L, P, or R as reviewed above.
If a change to a log section is provided, "=LOG" appears in
columns 1-4 to identify the subsequent information on the card
as a log identification.
- Columns 2-7 contain the left justified sequence number.
- Column 8 is used to contain printer control information, with
values of 0-7 providing the same number of line spacings after
the current line is printed (a blank is treated the same as a 1);
a value of 8 causing a page eject after the current
line is printed; and a value of 9 causing (with R in column 1)
the card information in columns 9-48 to be printed in print
columns 81-120 with n o space since the previous card. If a
"9" appears, of course, the sequence number for the card would
not appear in the final printout (although it would appear in
a list of card changes, naturally).
- Column 9-16 contain the tag of the cell, if any. The information
in the tag must observe the constraints, on allowable "symbols",
since the purpose of the tag is to permit reference to the
cell by symbolic means. As pointed out above, a "tag" such as
"#2" is essentially a comment, and is ignored by the assembler
if it appears in this card field, which is the "location field".
- Column 17 may contain a minus sign, in which case the memory
information resulting from the remainder of the card will be
completed before being stored.
- column 18-23 contain the operation code, making use of the
appropriate mnemonics assigned to machine-language or
interpretive-language orders (Sections IV and VI respectively),
or the appropriate assembler pseudo-operations (Section V).
In addition to these, however, the following asseembler control
operations (which do not generate binary memory information)
may also be used:
  - BANK: Set location counter (assembler counter used to
  determine the assignment of binary memory information
  to absolute machine address) equal to the first
  unassigned cell in the variable-fixed memory bank
  specified by the two-digit octal fixed bank number in
  the address fiewld. If the address field is blank,
  perform a similar function using the bank of the
  present location counter setting (generally follows
  a SETLOC instruction). Cells are assigned in ascending
  sequence starting from the beginning of each bank, but
  location counter changes to a fifferent bank must be
  by an explicit assembler control operation. BANK orders
  referencing a cell in S3 or S4 cause the Si printout
  (see SBANK= below) to be changed.
  - BLOCK: Same function as BANK, but conventionally used for
  fixed-fixed memory banks (02 and 03). "Blocks" 00 and 01
  are erasable memory, 04 is FBANK 00, etc. (cf. Section IIB).
  A blank BANK card can be used successfully with fixed-fixed
  memory banks, however.
  - COUNT: INitiate a count of the number of fixed memory cells,
  terminating when the next COUNT card is reached, for
  printout in a table at the end of the listing. The number
  of cells counted is associated with the tag in the
  address field (and the previous count, if any, and
  current total is provided on the printout). The operation
  COUNT*, if the tag is suitably flagged (e.g. "$$/xxx"),
  will replace the "$$" with the current fixed memory bank
  number (as if "$$" had originally been punched in that
  fashion). See Information at End of Listing.
  - EBANK=: Set the erasable memory bank portion of the following
  address pseudo-operation (BBCON, 2CADR, etc., See Section VC)
  to the erasable memory bank number of the tag in
  the address field (or to the number in the address field).
  If the EBANK= is not followed immediately by such an
  address pseudo-operation, an assembler cell is set to
  the same value, for use in monitoring machine language
  references to the erasable memory. This monitoring is
  reset at the beginning of each log section. The bank
  being monitored (if any) as ofthe last line on the
  previous page of the listing appears in print columns
  111-112 (e.g. "E3") of the third line (header log data).
  - EQUAL (or =): Translate the quantity in the tag field of
  the card in the same manner as the quantity in the address
  field of the card (which need not have preceded the EQUALS
  and which may be an absolute address as well as a symbol).
  If the address field is blank, the address corresponding
  to the present value of the location counter (e.g. one
  greater than the last filled address) is assigned to
  the tag in the tag fie1d. A distinction sometimes observed
  in the software is to use "EQUALS" to indicate either
  a relationship to a previous address ("chaining" of
  address assignments, useful for erasable memory) or a
  time-sharing of cells (between thrusting programs and
  entry guidance, for example); "=" on the other hand,
  indicates different tags for the same quantity.
  - ERASE: Allocate erasable memory cells in accordance with
  the material in the address field. If the address field
  is blank, one cell is allocated (and location counter
  advanced); if it is a signed integer (e.g. "+5"), then
  an additional number of cells (in this example, a total
  of six, sufficient for a double precision vector) are
  allocated as specified by this integer. If an unsigned
  (octal) integer is in the address field, on the other
  hand, then that absolute erasable memory cell (in ECADR
  format, see Section VC) is assigned to the tag in the
  tag field. Allocation of a set of cells can be
  accomplished in this fashion by ERASE xxx - yyy,
  where xxx is starting address and yyy the final address.
  - MEMORY: Allocate memory of the type indicated by location
  field (functions similarly to ERASE).
  - SBANK=: Set an aseembler control cell to indicate the
  use of the superbank (i.e. setting of SUPERBNK, channel
  07) given by the address field. This setting (in a
  manner similar to EBANK=, except it is not reset
  at the start of each log section) appears in print
  columns 114-115 (e.g. "S3") of the third line (header
  log data). Address constants such as BBCON and
  2CADR (see Section VC), if reference to a cell in
  53 or 54 is made (cf. Section IIB), will place the
  proper SUPERBNK bit setting in bits 7-5; if reference
  to bank nunbers of 27 or less is made, however, these
  bits will be set to either the most recent SBANK=
  statement or the last BANK pseudo-operation (whichever
  was the last to occur). The software is generally
  arranged so that reference to 53 is made wherever
  possible. The Si information on the third line, of
  course, is also changed by ihe BANK pseudo-operation.
  - SETLOC: Set location counter to value specified by address
  of card, which may be a True Address (see Section IIB)
  or a symbol. Frequently followed by a BANK card with a
  blank address field, to facilitate changes to memory
  bank allocations of the coding (see Information
  at Start of Listing). LOC means the same as SETLOC.
  - SUBRO: Include in the assembly the Subroutine identified
  by the symbol in the address field: see Information at
  Start of Listing below.
- Column 24 is blank.
- Columns 25-40 comprise the norEal address field. For machine-
language instructions, it may consist of a symbol or a symbol
+/- an integer (with a space before the sign). A blank means
the address of the present step, so that an address of "+2" would
mean an address two steps beyond the current step. For
interpretive-language orders, the address field contains
infornation as described in Section VI. Values of constants and
addresses, of course, appear in the address field too.
The information in the address field should end at or prior to
card column 40. If the required information is too lengthy to
complete in 16 card columns, the number of card columns
all-ocated to the address field may be increased by punching an
asterisk following the last character ( changing "2DEC" to
"2DEC*", for example) of the operation field and another
asterisk after the last character of the address information.
An asterisk may also be used to obtain special assembler
program performance (as mentioned above with COUNT and as also
mentioned in Section IIF), or to indicate indexing in the
interpretive language (see Section VI).
- Columns 41-80 (unless used with the address-field extension
technique described for columns 25-40) arc used for comments
information: the contents of these columns, of course, would
not affect the binary information generated by the assembler
for the computer memory.

## Symbol Reference Information

In the analysis of the performance of the software, it is
frequently valuable to be able to identify quickly and reliably
references to a given tag. Information permitting this to be done is
included in print columns 9-26 for those lines of coding with a tag in
the address field (for the operations such as 2CADR that generate two
lines of coding, the reference information is provided with the first
line). The symbol reference information, which is generated for the
various assembler control operations as well as for cards that generate
binary memory informnation, has the following print format:

- Columns 9-11 contain "REF" (for the serial number of the reference to the tag).
- Columns 13-15 contain the serial number of the reference to the
tag (starting from the beginning of the listing), with the
least significant digit in column 15 and with leading zeros
suppressed.
- Column 18-21 contain "LAST" (for the previous time in the
listing that the symbol was referenced), provided that
columns 13-15 do not contain 1 (if they do, meaning that
this is the first reference encountered, the printing of
"LAST" is suppressed).
- Column 23-26 contain the master page number (i.e. the one on
the top line of the page) where the previous reference (if any)
to the symbol was made).

It should be understood that the symbol reference information
applies to the symbol in the address field, not to the symbol in the
tag field. In order to identify references to the symbol in the tag
field, the information printed in the Symbol Table Listing at the end
of the program printout may be used (see Information at End of Listing below).

## Information at Start of Listing

The first 1og section of the listing is conventionalty titled
"Assembly and Operation Information". This 1og section generally
consists solely of remarks information, and hence no binary memory
loading information is generated from this segment. Therefore,
although the log section is intended to be a convenient source of
rapid reference information on the program, it should be clearly
understood that this information has no direct effect on the binary
memory information. Consequently, unless conscientious management
control procedures are enforced the material in this 1og section can
deviate from the actual performance of the software (an observation
that applies to all "comments" in the listing, of course). The
information generally included in this first 1og section includes:

- A table of Log Sections, giving the various Subroutines in the
software and the log sections that comprise them.
- A Verb List, giving the various verbs (see Section IIJ) in the
software and their numerical codes.
- A Noun List, giving the various nouns (see Section IIJ) in the
software and their numerical codes.
- An Alarm Code List, giving the patterns in the software and their
significance.
- Checklist and Option Codes, giving the patterns generated by the
software to request certain operation actions or decisions, an
the significance of each pattern.

It is emphasized once again that this log secion is made up solely of
remarks cards, and need no be consistent with the actual binary memory
information.

The second log section of the listing as conventionally titled
"Tags for Relative SETLOC and Blank BANK Cards". This log section is
used to assign various portions of the software to different fixed
memory banks. This is accomplished by having the software coding itself
written so as to specify the assembler location counter value by means
of a SETLOC card referencing a tag in this log section, followed by a
blank BANK card (see Card Layout above), causing the subsequent binary
memory information to be placed in the fixed memory bank dictated by
this second log section. The function of this log section, therefore,
is to associate a set of tags with appropriate fixed memory banks; it
allows absolute memory assignments in the software to be arranged
(for suitably fine-grained SETLOC and blank BANK cards) without changing
the log section in which the softuare itself appears. This technique
also allows some Subroutine information to be identical in different
programs, with necessary memory allocation differences handled in
this second log section rather than within the individual log sections
of the Subroutine. The only binary information generated by this
second log section of the listing is conventionally the memory check-
sum information, since the BNKSUM operands (see Section IIF) for the
different banks are conventionally placed here. In addition to fixed
memory bank assignments, sone fixed memory tag equivalences can appear
in this second log section, as well as some erasable memory bank
assignments and tag equivalences reflecting vehicle-peculiar computations.
Following this second log section may be additional log sections
for special purposes (such as bank-peculiar constants). The final
Iog section in the front of the program, however, is conventionally
titled "Subroutine Calls". During coding, it is convenient to have
the various elements of the software grouped into functions at a higher
level than the individual log section. This grouping is acconplished
by segmenting the software into groups called "Subroutines" (with a
capital "S" if the word appears with a lower-case "s", it has the
standard Websters 1965 definition of "specific instruction(s) whereby
a digital computer is guided to perfom a precisely defined mathematical
or logical operation"). Subroutines are assigned individual names
(which are not tags within the Subroutine itself), and software
modifications are made on a Subroutine basis (by, of course, specifying
log sections to be changed within the Subroutine). The listing flags
the last modification(s) made to the Subroutine as described earlier,
and an accounting is kept of the serial number of the Subroutine
revision (printed with the Subroutine name on line 1 of each page of
the Subroutine in the listing).

Subroutines are included in the assembly listing by means of
SUBRO cards (see Card Layout above), whose address field is the
name of the Subroutine. Each Subroutine, of course, must be compatible
with the others as far as memory usage, tag conflict, etc. are concerned
(there is no constraint on tag references between Subroutines, nor is
there any requirement for special assembler inputs to define such
tags). During the course of program development, the SUBRO log
section is the final one associated with the complete program,
and hence at the end of this log section there is a printout to
this effect (such as "*** END OF MAIN PROGRAM***"). The place for
the Subroutine name on line 1 of early pages of the assembly is
filled with "(MAIN)", indicating that no Subroutine is being printed
on this part of the listing.

After the program reaches a certain stage in its development,
however, it can be desirabte to restrict modifications to those which
are generated with reference to the complete program, rather than
merely to an individual Subroutine. This can be accomprished by suitable
assembler control cards, which cause the insertion of an "R" (for
Remark) in column 1 of each SUBRO card, thus retaining them in the
listing for reference. After this is accomplished, the place for the
Subroutine name in line 1 on all pages of the listing is filled with
"(MAIN)". This process is known as a "freeze" of Subroutines.

## Erasable Memory Information

The next log section in the listing (which can also comprise a
Subroutine) is conventionally titled "Erasable Assignment", and
gives most of the erasable memory and special register tag assigmnents
to absolute addresses (for convenience, the channel tag assignments
also are included). Many tags are assigned octal equivalent addresses
by the EQUALS or "=" assembler control operation (see Card Layout
above), and in these cases the corresponding S-register contents appear
in print columns 33-36. If EBANK is 3 or more for the address (see
Section IIB), the quantity "Ei" (where i is the EBANK number) appears
in print columns 30-32.

Other tags are assigned octal equivalent addresses by the ERASE
assembler control operation (see Card Layout above), and these have
the first address of the "block", (even if only one cell) in print
columns 30-36 and the second in print columns 39-45. As for the
assignment of tags by the EQUALS or "=" operation, the "Ei" is
suppressed for EBANK ralues of 0, 1, or 2 (so the address appears
in print columns 33-36 and 42-45 only).

The convention is sometimes followed that comments concerning the
erasable memory cell use are made in the comments field of the card,
such as "B(2)" if two cells are required for the quantity and it is
referenced in "basic" (i.e. machine language) coding so that its use
is EBANK sensitive; "I(6)" if six cells are required and it is referenced
only in "interpretive" coding (so use not EBANK sensitive); "PL(1)", if
the quantity is part of a "pad load", needing on1y 1 cell; etc. As is
true of all comments in the listing, however, there is no guarantee that
this information necessarily reflects the current status of the software.
Other aspects of the listing of erasable memory information (formats,
allowab1e symbols, symbol reference information, etc.) have already been
covered.

## Fixed Memory Information

Specification of the contents of the fixed memory is the major
purpose of the remairning 1og sections of the listing. The format of
the octal information (most of the other portions of this listilg have
already been described) is as follows:

1. An odd parity bit (to make the sum of the binary ones in the
15-bit memory word, including this bit, an odd value) is given
in print position 46 for all words to be loaded into the memory.
The onty allowable values, of course, that can appear in this
print column are O and 1.

2. For words loaded into the memory, print positions 33-36 give
the contents of the S-register. If the cell address is in
variable-fixed memory, the memory bank is in positions 30-31,
and a comma appears in print position 32. Words in fixed-fixed
memory (banks 02 and 03) have print positions 30-32 blank (and
S-register contents in range 4000~8~ - 7777~8~, cf. Section IIB).

3. For machine language instructions whose operation code is
specified completely by bits 15-13 (e.g. those operations which
can have addresses in both erasable and fixed memory, see
Section IV), the single octal digit of the operation code is in
print position 39 and the four octal digits of the operation
address are in positions 41-44.

4. For machine language instructions requiring portions of the
most significant digit of the (nominal) S-register portion for
their specification, and which reference the erasable memory
(or a channel), print positions 39-40 contain the two octal
digits of the operation and positions 42-44 contain the three
octal digits that remain for the address. If the most
significant bit of the allowed ten-bit address is a binary 1,
the operation code is an odd number (except for the channel
operations of Section IVC, only the most significant two bits
of the nominal S-register information are used for operation
information), and in addition an apostrophe (which nay appear
as some other character for different print chains) appears in
position 41 to emphasize the presence of a binary 1 from the
address information in the operation-code octal digits.

5. For address information, constants, and interpretive instructions
the five octal digits to be loaded into the memory are printed
in print positions 40-44.

6. Addresses generated by assembler functions (BANK, BLOCK, EBANK=,
EQUALS, "=", SBANK=, or SETLOC) appear in print positions 30-36
(the full address equivalent of the symbol is given, even)
though only a portion is functional). Since no binary memory
information is generated, no parity bit is printed.

## Information at End of Listing

After the final log section of the program, there are several
valuable reference tables which give useful information on the program.
The first of these is a "Symbol Table Listing", which gives all symbols
defined in the program as arranged in the order "sorted" by the assembler
(i.e. in order of increasing EBCDIC representation):

````
.
(
+
&
$
*
)
-
/
,
?
=
A-Z
0-9
````

Given after each symbol is the address (bank register, then S-register:
erasable banks of 3 or more are designated by Ei, while those less than
3 can be identified by their S-register contents, less than 1400~8~). To
the right of the address is given the "health" of the definition, which
is blank unless it is defined by EQUALS (or "="), in which case an "="
appears, or if some other difficulty was encountered such as poorly
or multiply defined symbols (suitably indicated per table at bottom of
each page). To the right of the "health" is given the page number on
which the symbol was defined, which of course is the "master" page number
appearing on the first line of each page. If the symbol is referenced
on several different pages of the program, the next three columns on
the page give the total number of references to the synbol, the page
number of the first reference, and the page nunber of the final
reference. If the symbol is only referenced on one page, the page number
of the "final" reference is blank, while if it is not referenced at all
these three columns are blank. Three symbol columns appear on each page.

If there were undefined symbols detected during the assembly, the
table following the "Symbol Table Listing" is the "Undefined Symbol
Table Listing", which gives the undefined symbols in the listing, with
their "health" (e.g. "UN" for undefined) and the same type of reference
information as for the previous table. No address or page number of
definition, of course, appears in this table. If there were no undefined
symbols, then the printing of this table is suppressed.

Next comes the "Unreferenced Symbol Listing", uhich lists only
those symbols from the "Symbol Table Listing" that are not referenced
in the program: this table repeats the information from the first 4
columns (symbol, address, health, and page of definition) of the "Symbol
Table Listing". Four symbol columns appear on each page.

Next comes an "Erasable & Equals Cross-Reference Table", which
lists all erasable memory tags in the order of increasing erasabl_e
memory address: the octal equivalent address is actually used, so that
flagword bits ard channel mnemonic assigmnents also appear. Tags
assigned to the same octal equivalent address are listed in the order
im which they were defined within the assembly (i.e. in order of increasing
page number), except those symbols which are equated to the same octal
value on the same page of the listing are provided instead in alphabetical
order for that page. At the end of the erasable memory tags, those
fixed memory tags which are defined by "=" (or EQUALS) assembler
operations are shown. Five columns of addresses are given on a page,
with each address followed by the page number and associated symbol.

The next table at the end of the listing provides a summary of
the addresses assigned ("Reserved") and spare ("Available"), in the
form of a "Memory Type & Availability Display", arranged with erasable
memory first, followed by fixed-fixed memory and then variable-fixed
memory. Figures deduced from this table would differ slightly from
numbers obtained from BNKSUM (see Section IIF), since this table includes
the two TC orders as "reserved". In addition, this table recognizes
erasable nemory as "reserved" only by the ERASE assembler operation,
so memory cell assignments by "chained" EQUALS cards are not reflected.

Following this table, there is a table which provides information
on the number of fixed memory cells that are expended for various
functions within the program. Information to make up this table is
provided by the COUNT and COUNT* (see Card Layout) cards within the
listing. The table lists in order the address-fieId information
associated with the COUNT and COUNT* cards (except that the COUNT*
"bank to be specified" information has instead the actual octal fixed
memory number inserted): these frequently take the form nn/XXXX,
where nn is the fixed memory bank number end XXXX is some convenient
mnnemonic (normal printing occurs, however, if a COUNT card specified
an nn different from that in which the steps involved actually are
located). With the address-field information is given the nunber of
references (including both COUNT and COUNt* that results in same "tag"),
the first and last pages of the final assembly accumulation of cell counts
for that "tag", with the number counted then ("LAST xxx TO yyy: zz"),
the total counted for that tag (the same as the final accumulation of
cell counts if REF = 1) and finally the cumulative count of cells used
since the beginning of the table: the final entry in this last column
in the table, therefore, would give the number of fixed memory cells
assigned in the complete program, since provision is made for a "blank"
count tag. If more than one reference to a given "tag" occurs,
the page nunber given for the first page of the final accumulation can
be checked: on that page will be found (to the left of the COUNT or
COUNT* print) the serial number of the reference to the "tag", the
previous "span" of counting for that "tag", the nunber found then,
and the number total to that point for the "tag". The count information
which is supplied, of course, is only as valid as the original placement
of the COUNT and COUNT* cards within the listing, and should be used
with caution as an indicator of how many steps would be "saved", for
example, if a function with a familiar mnemonic were to be deleted.

After this table comes a list of the "Paragraphs Generated for this
Assembly; Address Limits and the Manufacturing Location Code are shown
for Each." The hardware-oriented information presented in this table
is given in Secrion IIF.

Next comes an octal listing of the contents of each paragraph
(256 cells) in the program. Constants and interpretive operations
are flagged by "C:" and "I:" respectively before the cell contents
(which shows the odd parity bit to the right of the rest of the memory
word, separated by a space). The check sum word (the final cell that is
wired in the bank) is flagged by "CKSM" before the cell. The checksum
is computed by the assembler prior to printing each memory bank, using the
same algorithm as described in section IIF (including stopping when two
TC orders to the present step are found). Unwired cells in the memory
are flagged by the character "@" (which may appear differently due to
other printing hardware), while those cells whose contents were not
uniquely defined are suitably flagged.

After the octal listing comes a table which provides for each
assigned fixed memory cell the page number in the listing on which the
contents of that cell are specified (except for the check sum word
itself, see Section IIF, in each bank). This is followed by a list of
the Subroutines (if any) that are included in the program, along with
their revision numbers.

Finally, there is an indication of whether or not the assembly was
satisfactory. If it was, meaning that the assembler program detected no
deficiencies, an indication (e.g. "The assembly was good and manufacturable.
No lines were cussed.") is provided. If deficiencies ("cussed lines") were
detected, the number of these is provided, together with the page number
of the first page and last page where faults were noted. Within the
listing, each fault is accompanied by information on the reason for
flagging as a fault, its serial number, and the page number of the previous
fault (unless the previous one is on the same page). Pages III-25 to III-28
give a list of the fault messages appearing in one version of the assembly
program, arranged in order of increasing hexadecimal (base 15) serial
number of the message (printed to the far right of the line on which
the fault message appears in the listing). Also indicated is whether the
fault is considered "fatal" (if so, the assembly is considered 
"unmanufacturable").

## Program Changes

Program changes are specified by providing the modification
information segregated. by individual 1og sections which are to be
modified. The locations of the modifications are specified by the
sequence number punched on the card, as described earlier. Deletions
can be accomplished by ihe pseudo-operation DELETE, with the option
of adding "THRU yyyy" to delete the cards (i.e. lines) with sequence
numbers ranging from that of the DELETE through yyyy inclusive.

For an extensive insertion, the requirement for punching the sequence
number on every card may be avoided by the pseudo-operation INSERT
(with BEGIN in tag field, assigned a suitable sequence number); the
end of the insertion is again indicated by INSERT, this time with END
in the tag fie1d. Alternatively, "WITH nnnn", rather than a blank, can
be provided in the address field of the BEGIN INSERT, in which case the
first sequence number of the inserted coding will be that specified. In
either case, all subsequent sequence numbers (up through the end of the
1og section) will be modified so as to count up in the standard "counting
position" of these numbers (which allows four digits, i.e. print column 5).
If it is merely desired to modify the sequence numbers, this can be
accomplished by the CARDNS pseudo-operation.

A capability exists to print revision infornation by individual
Subroutine, reflecting the card inputs which were made to generate the
various versions. For such a listing, the first word in the top line
is "PRINT" (as opposed to "ASSEMBLE" for the normal program listing),
and instead of the overall program name, the name of the Subroutine is
included in the top line (if the overall program name is given, the
changes made to the material at the front of the listing, identified by
"(MAIN)" on the first line of the program, are supplied). The second
printed line for such listings gives the "author" and "date" (preceded
by the control characters "./"). 'A similar listing is generated when
the original modification is inserted, and has the first word in the
top line as "MODIIY" (or "CREATE" if a new Subroutine being gererated).
These have "GOOD UPDATE" printed at the botton if update successful.

### Fault Messages Generated by Assembler

| Serial | Fatal | Message |
| --- | --- | --- |
| Card Format |
| 01 | | Queer information in column 17 |
| 02 | | Queer information in column 24 |
| Erasable Problem |
| ß3 | x | EBANK/SBANK illegal except with BBCON & 2CADR |
| 04 | | EBANK conflict with one-shot declaration |
| Polish Opcode Problems |
| 05 | | Erased region should not cross E-banks |
| 06 | x | Polish words require blanks in columns 1, 17, & 24 |
| 07 | x | Previous Polish equation not concluded properly |
| 08 | x | Polish push-up requires negative word here |
| 09 | | Polish address expected here |
| 0A | x | Asterisk illegal on this opcode |
| 0B | x | Interpretive instruction not expected |
| 0C | x | Rt-opcode's mode-in disagrees with mode-out setting |
| 0D | x | Lft-opcode's mode-in disagrees with mode-out setting |
| 0E | | Address has no associated Polish opcode |
| 0F | x | Polish address(es) missing prior to this op pair |
| 10 | x | Location symbol improper on STADR'ed store word |
| 11 | x |Store opcode must be next after STADR |
| 12 | x | Push-up illegal before store opcode without "STADR" |
| 13 | | Address words cross over bank or VAR are boundary |
| 14 | x | Intrerpretive address word out of sequence |
| 15 | x | Address field should contain a Polish operator |
| 16 | x | First Polish operator illegally indexed |
| 17 | x | Interpreter opcode requires indexed address here |
| 18 | x | Interpreter opcode did not call for indexing |
| 19 | x | Second Polish operator illegally indexed |
| 1A | x | Can not handle neg addresses with indexing here |
| Numeric constant problems |
| 1B | | More than 14 octal digits in octal constant |
| 1C | | More than 10 digits in decimal constant |
| 1D | | Fractional part lost by truncation |
| 1E | x | Range error in constant field |
| 1F | | Inexact decimal to binary conversion |
| 20 | | Double precision constant should not cross banks |
| 21 | | No "D" in decimal number |
| Merge Control Problems |
| 22 | x | Subroutine name not recognized |
| 23 | | Multiple calls in one program or subroutine |
| 24 | x | Card ignored because it makes memory table too long |
| 25 | x | Card ignored because it's too late in the deck |
| 26 | x | Conflict with earlier head specification |
| 27 | | Card number out of sequence |
| 28 | x | No match found for second card number |
| 29 | x | First card number not less than second |
| 2A | x | No match found for card number or acceptor text |
| General Address Field Problems |
| 2B | | Blank address field expected |
| 2C | x | Blank address is undefined |
| 2D | x | Blank address was undefined in pass1 |
| 2E | | Blank address should be symbolic |
| 2F | x | Blank address was nearly defined by equals |
| 30 | x | Blank address was nearly defined by equals in pass1 |
| 31 | x | Blank address given multiple definitions |
| 32 | x | Blank address multiply defined including by equals |
| 33 | x | Blank address multiply defined including nearly by ='s |
| 34 | x | Blank address given oversize definition |
| 35 | x | Blank address associated with conflict |
| 36 | x | Blank address associated with multiple errors |
| 37 | x | Blank address associated with wrong memory type |
| 38 | x | Blank address is in miscellaneous trouble |
| 39 | | Address is inappropriate for opcode |
| 3A | x | Address is in bank 00 (filled in with bank number) |
| 3B | x | Address depends on unknown location |
| 3C | | Irregular but aacceptable address |
| 3D | x | Address field is meaningless |
| 3E | x | Addr. must be basic single-precision constant or inst |
| 3F | x | Range error in value of address |
| 40 | x | Indexing is illegal here |
| Opcode field problem |
| 41 | x | Illegal or mis-spelled operation field |
| 42 | | This instruction should be indexed |
| 43 | x | This instruction should be extended |
| 44 | x | This instruction should not be extended |
| Predefinition Problems |
| 45 | x | This instruction shouldn't have been predefined |
| 46 | x | Attempt to predefine location symbol failed |
| Location Field Problems |
| 47 | | Illegal location field format |
| 48 | | Location field should be blank |
| 49 | x | Location is in wrong type of memory |
| 4A | x | Numeric location field is illegal here |
| 4B | x | Oversized or ill-defined location |
| 4C | x | Conflict in use of this location |
| 4D | x | Conflict in won't fit in symbol table |
| 4E | x | No such bank or block number on this machine |
| 4F | x | This bank or block is full |
| Leftover Problems |
| 50 | x | This bank is indefinably leftover |
| 51 | x | Leftover won't fit in memory |
| 52 | x | Improper leftover location field format |
| More Cusses |
| 53 | | Queer information in column 1 |
| 54 | | Address field arithmetic not allowed here |
| 55 | | Address constant not expected here |
| 56 | | Address constant expected here |
| 57 | | Count table full. Address field ignored. |
| 58 | x | BBANK type constants require preceding EBANK= |
| 59 | | One shot SBANK= above was not needed |
| 5A | | Address 00,00 (filled with address) |
| 5B | | "STADR" unnecessary |
| 5C | | Assembler finds error but has no specific cuss for it |
| 5D | x | Address is in super bank 0 (filled with bank) |

# IV Machine Language Instructions

## IVA General Principles

There are 34 machine-language operation codes which may be performed
under program control (the operation EDRUPT is conventionally excluded
from the list of operation codes, and is discussed in Section IIH rather
than here). Use of special addresses with certain of these instructions
permits an additional four special functions EXTEND, INHINT, RELINT,
and RESUME, see Section VA) to be perforned, and use of addresses
0020~8~ - 0023~8~ permits shifting operations to be performed, as described
in Section IID. Of the 34 instructions, 15 may be classified as
"regular" orders and the remaining 19 as "extended" (or "extra code")
orders. The extended orders must be written as two lines of coding
(occupying two fixed memory cells, cf. Section IIB), with the first
line setting a special bit in the instruction register (by the EXTEND
operation: the bit is normally reset after the instruction is performed)
and the second line giving the order itself. Without the EXTEND, the
second line would be interpreted as a regular order.

Only a few of the instructions can be used with operands in both
erasable and fixed memory, since most of them achieve an effective
extension of the operation code bits by using the most significant two
bits of the l-2 bits nominally assigned to the S-register. The seven
channel instructions (see Section IIE), however, use the most significant
three bits of the nominal S-register information to determine the
operation to be performed.

The following two sections list the operation codes in alphabetical
sequence, with Section IVB givinig the regular orders and Section IVC
giving the extended orders, The symbols A, L, and Q refer to the
arithmetic registers defined in Section IIC, while quotation marks
around a symbol signify that the value of the address is of interest
rather than the information stored in that address. The fol1owing
special symbols are also used:

- E means an address in the range (S-register) 0000~8~ - 1777~8~, i.e.
a hardware register or an erasable memory.
- F means an address in the range (S-register) 2000~8~ - 7777~8~, i.e.
a cell in fixed memory.
- H means a channel (see Section IIE)
- K means an address in the range (S-register) 0000~8~ - 7777~8~, i.er.
a hardware register, an erasable memory cell, or a cell in fixed memory.
- N means the address of the step now being performed (i.e. the
one containing the operation code being described).

The term "erasable memory" is used in Sections IVB and IVC to
signify either an erasable memory cell or a hardware register.

The value shown in the "Operation" column is the operation code
that appears in the program listing: if two values appear, the choice
between them depends on the value of the address, as discussed in
Section III. A parenthetical 1 is used with the orders in Section IVC
to emphasize the need for having the extended-order flip-flop set by
the EXTEND operation.

The table on the following page summarizes the machine language
orders by operation value, separated into "regular orders" and the
"extended orders".

### Regular Orders

| Operation Value | Mnemonic |
| --- | --- |
| 0 | TC |
| | Address 3 = RELINT |
| | Address 4 = INHINT |
| | Address 6 = EXTEND |
| 10-11 | CCS |
| 12-17 | TCF |
| 20-21 | DAS |
| 22-23 | LXCH |
| 24-25 | INCR |
| 26-27 | ADS |
| 3 | CA |
| 4 | CS |
| 50-51 | INDEX |
| | Address 17 = RESUME |
| 52-53 | DXCH |
| 54-55 | TS |
| 56-57 | XCH |
| 6 | AD |
| 7 | MASK |

### Extended Orders

| Operation Value | Mnemonic |
| 00 | READ |
| 01 | WRITE |
| 02 | RAND |
| 03 | WAND |
| 04 | ROR |
| 05 | WOR |
| 06 | RXOR |
| 07 | EDRUPT |
| 10-11 | DV |
| 12-17 | BZF |
| 20-21 | MSU |
| 22-23 | QXCH |
| 24-25 | AUG |
| 26-27 | DIM |
| 3 | DCA |
| 4 | DCS |
| 5 | INDEX |
| 60-61 | SU |
| 62-67 | BZMF |
| 7 | MP |

## IVB Regular Orders

| Mnemonic | Operation | Performance |
| --- | --- | --- |
| AD "K" | 6 | Add. Two MCT (23.4 ms). Address can be in erasable or fixed memory. A = A+ K. If "K" = "A", this doubles the contents of the accumulator. If "K" = 20~8~ - 23~8~, K modified as described in Section IID. |
| ADS "E" | 26 27 | Add and Store. Two MCT (23.4 ms). Address can be in erasable memory only. A = A + E E = A If "E" = 20~8~ - 23~8~, the value stored in "E" modified as described in Section IID. |
| CA "K" | 3 | Clear and add. Two MCT (23.4 ms). Address can be in erasable or fixed memory. A = K If "K" = 20~8~ - 23~8~, K is modified as desribed in Section IID. |
| CCS "E" | 10 11 | Count, Compare and Skip. Two MCT (23.4 ms). Address can be in erasable memory only. Load A with |E| -1, limited >= +0 and skip 0 (E>0), 1 (E=+0), 2 (E<-0), or 3(E=-0) steps. Overflow bit can be sensed. If E is positive non-zero (in range 00001~8~ - 37777~8~): A = E-1 (if E=1, A=+0) Proceed to "N"+1 (the next step)

## IVC Extended Orders

## IVD Machine Language Examples

# Special Assembler Operations

## VA Equivalent Machine Language Instructions

## VB epresentation of Numbers

### Decimal Numbers

### Octal Numbers

## VC Representation of Addresses

# Interpretive Language

## VIA General Principles

## VIB Interpretive Language Operations

### Scalar Computation Operations

### Vector Computation Operations

### Shifting Operations

### Transmission Operations

### Control Operations

### Index Register Oriented Operations

### Logical Bit Operations

## VIC Addresses and Interpreter Control

### Overall Interpreter Control

### Interpreter Address Determination

### Interpreter Storage Orders

### Interpreter Transfer to Operation

## VID Relative Addresses, Push-down List and VAC Areas

## VIE Interpretive Language Examples

# Program Performance Control

## VIIA Waitlist System for Tasks

### Waitlist System Tables

## VIIB Executive System for Jobs

### Contents of Job Register Sets

## VIIC Mechanization of Restart Capability

## VIID Standard Program Subroutines

# App A: Review of Computer Concepts

## Number Systems

## Arithmetic and Overflow

## Order and Addressews

## Scaling

## Software Difficulties

# App B: Changes made for Revision 2

## Hardware

## Software

## Interpretive Language

# App C: Summary of Computer Inputs and Outputs

# App D: Alphabetical Listings

## Machine Language and Other Assembler Codes

## Interpretive Language Instructions

## Registers, Program Steps and Storage References

## Alphabetical Listings of Terms


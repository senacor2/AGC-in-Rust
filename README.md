# AGC-in-Rust

AI Agents port the Apollo Guidance Computer to Rust.

The goal is to transform the AGC code into readable and maintainable Rust code. This will require to re-create the abstractions that where lost, when the AGC assembler code was written.

## References

* [Apollo Guidance Computer source code](https://github.com/chrislgarry/Apollo-11): Digitized source code listings for the AGC, edited to run in the VirtualAGC environment.
* Frank O'Brien: The Apollo Guidance Computer - Architecture and Operation
* W. David Woods: How Apollo Flew To The Moon. Helpful to understand the underlying math used for navigation. 
* James E. Tomayko: Computers in Spaceflight. NASA Contractor Report 182505
* [Programmer's Manual - Block 2 AGC Assembly Language](https://www.ibiblio.org/apollo/assembly_language_manual.html) from the VirtualAGC project including machine, interpreter and pseudocode instruction descriptions.
* [Apollo Guidance Program Symbolic Listing Information for Block 2](https://www.ibiblio.org/apollo/Documents/SymbolicListingInformation.pdf) a formal description of the AGC machine.
* [User's Guide to The General Assembler Program (GAP)](https://www.ibiblio.org/apollo/Documents/Users_Guide_GAP_text.pdf)

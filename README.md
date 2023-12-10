
[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![UPRJ_CI](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml) [![Caravel Build](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml)

# J1Asic - a stack based CPU as open-source microchip

## Description
This is an ASIC version of the J1 stackbased CPU. The base reimplementation goes by the name J1Sc, was done by Steffen Reith and he wrote it in SpinalHDL. The J1Sc version for use as ASIC design in this repository is named J1Asic.  

## Code and License origin
The code repository and original Open-Source License (BSD 3) is available at:

https://github.com/SteffenReith/J1Sc

All the documentation about the J1 CPU itself resides inside that linked repository. There are plenty versions of this J1Sc for different FPGAs. The J1Sc is tested on most of these FPGA platforms. 

## Verilog for the J1Asic and restrictions
The verilog for this ASIC GF180 version was created from the branch ```TinyASIC``` and is a stripped down version of the J1Sc with the following restrictions in mind:

- No RAM macros, only DFF RAM is used.
- Two very much reduced main RAMs with only 256 Bytes each. Total FF ~5K
- Very much reduced stacksize (8 entries).
- Only the bare necessary periphials.

If you want to generate the Verilog yourself, follow the readme in the J1Sc repo inside the ```TinyASIC``` branch. And then copy the verilog file ```J1Asic.v``` over to this repository here. There are changes that were made by hand in the ```J1Asic.v``` before using it as a user_project for the caravel RISC-V. These changes are documented further below in the sections about oeb (output enable bar).  

## GF180 Open-Source PDK, OpenLANE and caravel
This design is a submission to the GF180 MPW-1 Shutte Run in Dec 2023. The Open-Source PDK from Global Foundry 180nm is used for production and the EDA tools from the Efabless OpenLANE repos are used to build and tapeout the designs. The J1Asic is embedded inside a managment core called caravel, also from Efabless. 

## Design workflow in a nutshell (chronologic order):
- The J1Asic.v goes into the user project area (around 10mm2) and gets hardened as a macro.
- This macro gets wrapped by a well defined and not changeable user_project_wrapper.v
- The user_project_wrapper gets hardened including the J1Asic macro.
- Both (J1Asic and user_project_wrapper) go inside the caravel RISC-V management core.
- Everything together (J1Asic, user_project_wrapper and caravel) gets hardened at the Tapeout process with Efabless. 
- Various checks are performed on the way.

There are a lot of files and configs that have to be worked on, that are not mention here. See the documentation for the PDK (GF180), the tools (OpenLane) and the caravel.

## The IOs, names and defines of the J1

| Name              | IO            | define        |
| ----------------- | ------------- | -----------   |
| reset             | wb_rst_i      | none, internal |
| boardClk          | wb_clk_i      | none, internal |
| boardClkLocked    | io_in[29]     | GPIO_MODE_USER_STD_INPUT_PULLDOWN |
| extInt	        | io_in[28:26]  | GPIO_MODE_USER_STD_INPUT_PULLDOWN |
| pmodA_read        | io_in[37:30]  | GPIO_MODE_USER_STD_BIDIRECTIONAL |
| pmodA_write       | io_out[37:30] | GPIO_MODE_USER_STD_BIDIRECTIONAL |
| pmodA_oeb     	| io_oeb[37:30] | GPIO_MODE_USER_STD_BIDIRECTIONAL |
| rx                | io_in[25]     | GPIO_MODE_USER_STD_INPUT_NOPULL |
| tx	            | io_out[21]    | GPIO_MODE_USER_STD_OUTPUT |
| tdi               | io_in[24]     | GPIO_MODE_USER_STD_INPUT_NOPULL |
| tdo               | io_out[20]    | GPIO_MODE_USER_STD_OUTPUT |
| tms               | io_in[23]     | GPIO_MODE_USER_STD_INPUT_NOPULL |
| tck               | io_in[22]     | GPIO_MODE_USER_STD_INPUT_NOPULL |
| io_oeb_high       | oi_oeb[29:22] | permanent set to high in J1Asic.v | 
| io_oeb_low        | oi_oeb[21:20] | permament set to low in J1Asic.v | 

All other IOs [19:5] are defined as GPIO_MODE_USER_STD_INPUT_PULLDOWN and are not used in the J1 design.

The IOs [4:0] are not touched.

#### A word about pmod_oeb
The GPIO implementation in spinalHDL sees the outputs as enabled with signal HIGH, namely the  ```pmodA_WriteEnable``` wires. The caravel RISC-V wants it the other way around (beeing input means HIGH), namely the ```io_oeb``` wires. And the user_project_wrapper does not allow anything but plain wires. The chosen place for the inversion is therefore inside the ```J1Asic.v``` verilog, near to the top.  

#### Another word about oeb
Even the ```only input and output wires``` of the caravel RISC-V need the oeb lines set (In = HIGH, Out = Low). And again this can't be done in the wrapper. And again it went straight into the ```J1Asic.v``` verilog, near to the top.


[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![UPRJ_CI](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/user_project_ci.yml) [![Caravel Build](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml/badge.svg)](https://github.com/efabless/caravel_project_example/actions/workflows/caravel_build.yml)

## Description
This is the reimplementation of a tiny stackbased CPU, named J1. The implementation was done by Steffen Reith, written in SpinalHDL.

## Code origin
The code repository and original Open-Source License (BSD 3) is available at:

https://github.com/SteffenReith/J1Sc

All the documentation about the J1 CPU itself resides inside that linked repository. There are plenty versions of this J1Sc for FPGAs and the CPU is well tested on FPGAs. 

## Verilog for the ASIC and restrictions
The verilog for this ASIC GF180 version was created from the branch ```TinyASIC``` and is a stripped down version of the J1Sc with the following restrictions in mind:

- No RAM macros, only DFF RAM is used.
- Two very much reduced main RAMs with only 256 Bytes each. Total FF ~5K
- Very much reduced stacksize (8 entries).
- Only the bare necessary periphials.

## GF180 Open-Source PDK
This is a submission to the GF180 MPW-1 Shutte Run in Dec 2023. The Open-Source PDK from Global Foundry 180nm is used for production and the EDA tools from the Efabless OpenLANE repos are used to build and tapeout the designs.

## The IOs, names and defines of the J1

| Name              | IO            | define        |
| ----------------- | ------------- | -----------   |
| reset             | wb_rst_i      | none, internal |
| boardClk          | wb_clk_i      | none, internal |
| boardClkLocked    | io_in[29]     | GPIO_MODE_USER_STD_INPUT_PULLDOWN |
| extInt	        | io_in[28:26]  | GPIO_MODE_USER_STD_INPUT_PULLDOWN |
| pmodA_read        | io_in[37:30]  | GPIO_MODE_USER_STD_BIDIRECTIONAL |
| pmodA_write       | io_out[37:30] | GPIO_MODE_USER_STD_BIDIRECTIONAL |
| pmodA_writeEnable	| io_oeb[37:30] | GPIO_MODE_USER_STD_BIDIRECTIONAL |
| rx                | io_in[25]     | GPIO_MODE_USER_STD_INPUT_NOPULL |
| tx	            | io_out[21]    | GPIO_MODE_USER_STD_OUTPUT |
| tdi               | io_in[24]     | GPIO_MODE_USER_STD_INPUT_NOPULL |
| tdo               | io_out[20]    | GPIO_MODE_USER_STD_OUTPUT |
| tms               | io_in[23]     | GPIO_MODE_USER_STD_INPUT_NOPULL |
| tck               | io_in[22]     | GPIO_MODE_USER_STD_INPUT_NOPULL |
| io_oeb_high       | oi_oeb[29:22] | permanent set to high in J1Asic.v | 
| io_oeb_low        | oi_oeb[21:20] | permament set to low in J1Asic.v | 

All other IOs [19:5] are defined as GPIO_MODE_USER_STD_INPUT_PULLDOWN and are not used in the J1 design.

The IOs below [4:0] are not touched.


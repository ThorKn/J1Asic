###############################################################################
# Created by write_sdc
# Sun Dec 10 15:21:30 2023
###############################################################################
current_design J1Asic
###############################################################################
# Timing Constraints
###############################################################################
create_clock -name boardClk -period 24.0000 [get_ports {boardClk}]
set_clock_transition 0.1500 [get_clocks {boardClk}]
set_clock_uncertainty 0.2500 boardClk
set_propagated_clock [get_clocks {boardClk}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {boardClkLocked}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {extInt[0]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {extInt[1]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {extInt[2]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_read[0]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_read[1]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_read[2]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_read[3]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_read[4]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_read[5]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_read[6]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_read[7]}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {reset}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {rx}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {tck}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {tdi}]
set_input_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {tms}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {io_oeb_high[0]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {io_oeb_high[1]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {io_oeb_high[2]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {io_oeb_high[3]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {io_oeb_high[4]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {io_oeb_high[5]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {io_oeb_high[6]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {io_oeb_high[7]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {io_oeb_low[0]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {io_oeb_low[1]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_oeb[0]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_oeb[1]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_oeb[2]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_oeb[3]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_oeb[4]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_oeb[5]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_oeb[6]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_oeb[7]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_write[0]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_write[1]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_write[2]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_write[3]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_write[4]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_write[5]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_write[6]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {pmodA_write[7]}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {tdo}]
set_output_delay 4.8000 -clock [get_clocks {boardClk}] -add_delay [get_ports {tx}]
###############################################################################
# Environment
###############################################################################
set_load -pin_load 0.0729 [get_ports {tdo}]
set_load -pin_load 0.0729 [get_ports {tx}]
set_load -pin_load 0.0729 [get_ports {io_oeb_high[7]}]
set_load -pin_load 0.0729 [get_ports {io_oeb_high[6]}]
set_load -pin_load 0.0729 [get_ports {io_oeb_high[5]}]
set_load -pin_load 0.0729 [get_ports {io_oeb_high[4]}]
set_load -pin_load 0.0729 [get_ports {io_oeb_high[3]}]
set_load -pin_load 0.0729 [get_ports {io_oeb_high[2]}]
set_load -pin_load 0.0729 [get_ports {io_oeb_high[1]}]
set_load -pin_load 0.0729 [get_ports {io_oeb_high[0]}]
set_load -pin_load 0.0729 [get_ports {io_oeb_low[1]}]
set_load -pin_load 0.0729 [get_ports {io_oeb_low[0]}]
set_load -pin_load 0.0729 [get_ports {pmodA_oeb[7]}]
set_load -pin_load 0.0729 [get_ports {pmodA_oeb[6]}]
set_load -pin_load 0.0729 [get_ports {pmodA_oeb[5]}]
set_load -pin_load 0.0729 [get_ports {pmodA_oeb[4]}]
set_load -pin_load 0.0729 [get_ports {pmodA_oeb[3]}]
set_load -pin_load 0.0729 [get_ports {pmodA_oeb[2]}]
set_load -pin_load 0.0729 [get_ports {pmodA_oeb[1]}]
set_load -pin_load 0.0729 [get_ports {pmodA_oeb[0]}]
set_load -pin_load 0.0729 [get_ports {pmodA_write[7]}]
set_load -pin_load 0.0729 [get_ports {pmodA_write[6]}]
set_load -pin_load 0.0729 [get_ports {pmodA_write[5]}]
set_load -pin_load 0.0729 [get_ports {pmodA_write[4]}]
set_load -pin_load 0.0729 [get_ports {pmodA_write[3]}]
set_load -pin_load 0.0729 [get_ports {pmodA_write[2]}]
set_load -pin_load 0.0729 [get_ports {pmodA_write[1]}]
set_load -pin_load 0.0729 [get_ports {pmodA_write[0]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_4 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {boardClk}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {boardClkLocked}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {reset}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {rx}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {tck}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {tdi}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {tms}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {extInt[2]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {extInt[1]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {extInt[0]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {pmodA_read[7]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {pmodA_read[6]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {pmodA_read[5]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {pmodA_read[4]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {pmodA_read[3]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {pmodA_read[2]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {pmodA_read[1]}]
set_driving_cell -lib_cell gf180mcu_fd_sc_mcu7t5v0__inv_1 -pin {ZN} -input_transition_rise 0.0000 -input_transition_fall 0.0000 [get_ports {pmodA_read[0]}]
set_timing_derate -early 0.9500
set_timing_derate -late 1.0500
###############################################################################
# Design Rules
###############################################################################
set_max_transition 3.0000 [current_design]
set_max_fanout 4.0000 [current_design]

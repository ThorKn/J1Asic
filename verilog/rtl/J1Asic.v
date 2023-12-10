// Generator : SpinalHDL v1.9.4    git head : 270018552577f3bb8e5339ee2583c9c22d324215
// Component : J1Asic
// Git hash  : 8e6e50b88477bc94cb11d6995a4638e0bed0b669

`timescale 1ns/1ps

module J1Asic (
  input  wire          reset,
  input  wire          boardClk,
  input  wire          boardClkLocked,
  input  wire [2:0]    extInt,
  input  wire [7:0]    pmodA_read,
  output wire [7:0]    pmodA_write,
  output wire [7:0]    pmodA_writeEnable,
  input  wire          rx,
  output wire          tx,
  input  wire          tdi,
  output wire          tdo,
  input  wire          tms,
  input  wire          tck,
  output wire [7:0]    io_oeb_high,
  output wire [1:0]    io_oeb_low
);
  localparam UartStopType_ONE = 1'd0;
  localparam UartStopType_TWO = 1'd1;
  localparam UartParityType_NONE = 2'd0;
  localparam UartParityType_EVEN = 2'd1;
  localparam UartParityType_ODD = 2'd2;

  assign io_oeb_high = 8'b11111111;
  assign io_oeb_low = 2'b0;

  wire                bufferCC_5_io_dataIn;
  wire       [7:0]    coreArea_cpu_intVec;
  wire       [7:0]    coreArea_pmodA_dirValue;
  wire       [7:0]    coreArea_pmodA_dataValue;
  wire       [15:0]   coreArea_timerA_cmpHigh;
  wire       [15:0]   coreArea_timerA_cmpLow;
  wire       [15:0]   coreArea_timerA_enable;
  wire       [15:0]   coreArea_timerB_cmpHigh;
  wire       [15:0]   coreArea_timerB_cmpLow;
  wire       [15:0]   coreArea_timerB_enable;
  wire                coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_flush;
  reg                 toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_ready;
  wire                toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_flush;
  reg        [3:0]    coreArea_intCtrl_irqReqs;
  reg        [3:0]    coreArea_intCtrl_enableWriteIrqVec;
  wire       [15:0]   coreArea_intCtrl_irqSetData;
  wire                jtagIface_jtagArea_jtag_tdo;
  wire                jtagIface_jtagArea_jtag_jtagDataFlow_valid;
  wire                jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagDataValid;
  wire                jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagStall;
  wire                jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCaptureMemory;
  wire       [7:0]    jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUAdr;
  wire       [15:0]   jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUWord;
  wire                jtagIface_jtagArea_jtag_jtagReset;
  wire                bufferCC_5_io_dataOut;
  wire                coreArea_cpu_cpuBus_enable;
  wire                coreArea_cpu_cpuBus_writeMode;
  wire       [15:0]   coreArea_cpu_cpuBus_address;
  wire       [15:0]   coreArea_cpu_cpuBus_writeData;
  wire                flowCCByToggle_1_io_output_valid;
  wire                flowCCByToggle_1_io_output_payload_jtagDataValid;
  wire                flowCCByToggle_1_io_output_payload_jtagStall;
  wire                flowCCByToggle_1_io_output_payload_jtagCaptureMemory;
  wire       [7:0]    flowCCByToggle_1_io_output_payload_jtagCPUAdr;
  wire       [15:0]   flowCCByToggle_1_io_output_payload_jtagCPUWord;
  wire       [7:0]    coreArea_pmodA_directions;
  wire       [7:0]    coreArea_pmodA_dataOut;
  wire       [15:0]   coreArea_timerA_enableState;
  wire       [15:0]   coreArea_timerA_highState;
  wire       [15:0]   coreArea_timerA_lowState;
  wire                coreArea_timerA_interrupt;
  wire       [15:0]   coreArea_timerB_enableState;
  wire       [15:0]   coreArea_timerB_highState;
  wire       [15:0]   coreArea_timerB_lowState;
  wire                coreArea_timerB_interrupt;
  wire                coreArea_uartCtrl_io_write_ready;
  wire                coreArea_uartCtrl_io_read_valid;
  wire       [7:0]    coreArea_uartCtrl_io_read_payload;
  wire                coreArea_uartCtrl_io_uart_txd;
  wire                coreArea_uartCtrl_io_readError;
  wire                coreArea_uartCtrl_io_readBreak;
  wire                coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready;
  wire                coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid;
  wire       [7:0]    coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload;
  wire       [3:0]    coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy;
  wire       [3:0]    coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_availability;
  wire                toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_push_ready;
  wire                toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_valid;
  wire       [7:0]    toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_payload;
  wire       [3:0]    toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_occupancy;
  wire       [3:0]    toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_availability;
  wire       [15:0]   coreArea_intCtrl_irqGetMask;
  wire       [15:0]   coreArea_intCtrl_irqVectors_0;
  wire       [15:0]   coreArea_intCtrl_irqVectors_1;
  wire       [15:0]   coreArea_intCtrl_irqVectors_2;
  wire       [15:0]   coreArea_intCtrl_irqVectors_3;
  wire       [15:0]   coreArea_intCtrl_intVec;
  wire                coreArea_intCtrl_irq;
  wire       [0:0]    _zz_coreArea_uartBridge_misc_readError;
  wire       [0:0]    _zz_coreArea_uartBridge_misc_readOverflowError;
  wire       [0:0]    _zz_coreArea_uartBridge_misc_breakDetected;
  wire       [0:0]    _zz_coreArea_uartBridge_misc_doBreak;
  wire       [0:0]    _zz_coreArea_uartBridge_misc_doBreak_1;
  wire       [3:0]    _zz_coreArea_peripheralBus_readData;
  wire                core_clk;
  wire                core_reset;
  wire                _zz_1;
  reg                 _zz_core_reset;
  reg                 toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagDataValid;
  reg                 toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagStall;
  reg                 toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagCaptureMemory;
  reg        [7:0]    toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUAdr;
  reg        [15:0]   toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUWord;
  wire                coreArea_peripheralBus_enable;
  wire                coreArea_peripheralBus_writeMode;
  wire       [15:0]   coreArea_peripheralBus_address;
  wire       [15:0]   coreArea_peripheralBus_writeData;
  reg        [15:0]   coreArea_peripheralBus_readData;
  reg        [15:0]   toplevel_coreArea_cpu_cpuBus_address_delay_1;
  reg                 toplevel_coreArea_cpu_cpuBus_enable_delay_1;
  reg                 toplevel_coreArea_cpu_cpuBus_writeMode_delay_1;
  reg        [15:0]   toplevel_coreArea_cpu_cpuBus_writeData_delay_1;
  wire                coreArea_peripheralBusCtrl_readErrorFlag;
  wire                coreArea_peripheralBusCtrl_writeErrorFlag;
  wire                coreArea_peripheralBusCtrl_askWrite;
  wire                coreArea_peripheralBusCtrl_askRead;
  reg                 _zz_dirEnable;
  reg                 _zz_dataEnable;
  reg                 _zz_loadLow;
  reg                 _zz_loadHigh;
  reg                 _zz_accessEnableWrite;
  reg                 _zz_loadLow_1;
  reg                 _zz_loadHigh_1;
  reg                 _zz_accessEnableWrite_1;
  wire                coreArea_uartBridge_busCtrlWrapped_readErrorFlag;
  wire                coreArea_uartBridge_busCtrlWrapped_writeErrorFlag;
  wire       [2:0]    coreArea_uartBridge_uartConfigReg_frame_dataLength;
  wire       [0:0]    coreArea_uartBridge_uartConfigReg_frame_stop;
  wire       [1:0]    coreArea_uartBridge_uartConfigReg_frame_parity;
  reg        [19:0]   coreArea_uartBridge_uartConfigReg_clockDivider;
  reg                 _zz_coreArea_uartBridge_write_streamUnbuffered_valid;
  wire                coreArea_uartBridge_write_streamUnbuffered_valid;
  wire                coreArea_uartBridge_write_streamUnbuffered_ready;
  wire       [7:0]    coreArea_uartBridge_write_streamUnbuffered_payload;
  reg                 coreArea_uartBridge_read_streamBreaked_valid;
  reg                 coreArea_uartBridge_read_streamBreaked_ready;
  wire       [7:0]    coreArea_uartBridge_read_streamBreaked_payload;
  reg                 coreArea_uartBridge_interruptCtrl_writeIntEnable;
  reg                 coreArea_uartBridge_interruptCtrl_readIntEnable;
  wire                coreArea_uartBridge_interruptCtrl_readInt;
  wire                coreArea_uartBridge_interruptCtrl_writeInt;
  wire                coreArea_uartBridge_interruptCtrl_interrupt;
  reg                 coreArea_uartBridge_misc_readError;
  reg                 when_BusSlaveFactory_l341;
  wire                when_BusSlaveFactory_l347;
  reg                 coreArea_uartBridge_misc_readOverflowError;
  reg                 when_BusSlaveFactory_l341_1;
  wire                when_BusSlaveFactory_l347_1;
  wire                toplevel_coreArea_uartCtrl_io_read_isStall;
  reg                 coreArea_uartBridge_misc_breakDetected;
  reg                 toplevel_coreArea_uartCtrl_io_readBreak_regNext;
  wire                when_UartCtrl_l155;
  reg                 when_BusSlaveFactory_l341_2;
  wire                when_BusSlaveFactory_l347_2;
  reg                 coreArea_uartBridge_misc_doBreak;
  reg                 when_BusSlaveFactory_l377;
  wire                when_BusSlaveFactory_l379;
  reg                 when_BusSlaveFactory_l341_3;
  wire                when_BusSlaveFactory_l347_3;
  reg                 _zz_enableWriteNewMask;
  reg                 _zz_enableWriteIrqVec;
  reg                 _zz_enableWriteIrqVec_1;
  reg                 _zz_enableWriteIrqVec_2;
  reg                 _zz_enableWriteIrqVec_3;
  `ifndef SYNTHESIS
  reg [23:0] coreArea_uartBridge_uartConfigReg_frame_stop_string;
  reg [31:0] coreArea_uartBridge_uartConfigReg_frame_parity_string;
  `endif

  function [19:0] zz_coreArea_uartBridge_uartConfigReg_clockDivider(input dummy);
    begin
      zz_coreArea_uartBridge_uartConfigReg_clockDivider = 20'h00000;
      zz_coreArea_uartBridge_uartConfigReg_clockDivider = 20'h0000c;
    end
  endfunction
  wire [19:0] _zz_2;

  assign _zz_coreArea_uartBridge_misc_readError = 1'b0;
  assign _zz_coreArea_uartBridge_misc_readOverflowError = 1'b0;
  assign _zz_coreArea_uartBridge_misc_breakDetected = 1'b0;
  assign _zz_coreArea_uartBridge_misc_doBreak = 1'b1;
  assign _zz_coreArea_uartBridge_misc_doBreak_1 = 1'b0;
  assign _zz_coreArea_peripheralBus_readData = (4'b1000 - coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy);
  J1Jtag jtagIface_jtagArea_jtag (
    .tdi                                    (tdi                                                           ), //i
    .tdo                                    (jtagIface_jtagArea_jtag_tdo                                   ), //o
    .tms                                    (tms                                                           ), //i
    .jtagDataFlow_valid                     (jtagIface_jtagArea_jtag_jtagDataFlow_valid                    ), //o
    .jtagDataFlow_payload_jtagDataValid     (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagDataValid    ), //o
    .jtagDataFlow_payload_jtagStall         (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagStall        ), //o
    .jtagDataFlow_payload_jtagCaptureMemory (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCaptureMemory), //o
    .jtagDataFlow_payload_jtagCPUAdr        (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUAdr[7:0]  ), //o
    .jtagDataFlow_payload_jtagCPUWord       (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUWord[15:0]), //o
    .jtagReset                              (jtagIface_jtagArea_jtag_jtagReset                             ), //o
    .tck                                    (tck                                                           ), //i
    .reset                                  (reset                                                         )  //i
  );
  BufferCC bufferCC_5 (
    .io_dataIn  (bufferCC_5_io_dataIn ), //i
    .io_dataOut (bufferCC_5_io_dataOut), //o
    .core_clk   (core_clk             ), //i
    ._zz_1      (_zz_1                )  //i
  );
  J1 coreArea_cpu (
    .stall            (toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagStall        ), //i
    .irq              (coreArea_intCtrl_irq                                                     ), //i
    .intVec           (coreArea_cpu_intVec[7:0]                                                 ), //i
    .captureMemory    (toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagCaptureMemory), //i
    .jtagMemAdr       (toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUAdr[7:0]  ), //i
    .jtagMemWord      (toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUWord[15:0]), //i
    .cpuBus_enable    (coreArea_cpu_cpuBus_enable                                               ), //o
    .cpuBus_writeMode (coreArea_cpu_cpuBus_writeMode                                            ), //o
    .cpuBus_address   (coreArea_cpu_cpuBus_address[15:0]                                        ), //o
    .cpuBus_writeData (coreArea_cpu_cpuBus_writeData[15:0]                                      ), //o
    .cpuBus_readData  (coreArea_peripheralBus_readData[15:0]                                    ), //i
    .core_reset       (core_reset                                                               ), //i
    .core_clk         (core_clk                                                                 )  //i
  );
  FlowCCByToggle flowCCByToggle_1 (
    .io_input_valid                      (jtagIface_jtagArea_jtag_jtagDataFlow_valid                    ), //i
    .io_input_payload_jtagDataValid      (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagDataValid    ), //i
    .io_input_payload_jtagStall          (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagStall        ), //i
    .io_input_payload_jtagCaptureMemory  (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCaptureMemory), //i
    .io_input_payload_jtagCPUAdr         (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUAdr[7:0]  ), //i
    .io_input_payload_jtagCPUWord        (jtagIface_jtagArea_jtag_jtagDataFlow_payload_jtagCPUWord[15:0]), //i
    .io_output_valid                     (flowCCByToggle_1_io_output_valid                              ), //o
    .io_output_payload_jtagDataValid     (flowCCByToggle_1_io_output_payload_jtagDataValid              ), //o
    .io_output_payload_jtagStall         (flowCCByToggle_1_io_output_payload_jtagStall                  ), //o
    .io_output_payload_jtagCaptureMemory (flowCCByToggle_1_io_output_payload_jtagCaptureMemory          ), //o
    .io_output_payload_jtagCPUAdr        (flowCCByToggle_1_io_output_payload_jtagCPUAdr[7:0]            ), //o
    .io_output_payload_jtagCPUWord       (flowCCByToggle_1_io_output_payload_jtagCPUWord[15:0]          ), //o
    .tck                                 (tck                                                           ), //i
    .reset                               (reset                                                         ), //i
    .core_clk                            (core_clk                                                      )  //i
  );
  GPIO coreArea_pmodA (
    .dirEnable  (_zz_dirEnable                 ), //i
    .dirValue   (coreArea_pmodA_dirValue[7:0]  ), //i
    .dataEnable (_zz_dataEnable                ), //i
    .dataValue  (coreArea_pmodA_dataValue[7:0] ), //i
    .directions (coreArea_pmodA_directions[7:0]), //o
    .dataIn     (pmodA_read[7:0]               ), //i
    .dataOut    (coreArea_pmodA_dataOut[7:0]   ), //o
    .core_clk   (core_clk                      ), //i
    .core_reset (core_reset                    )  //i
  );
  Timer coreArea_timerA (
    .loadHigh          (_zz_loadHigh                     ), //i
    .loadLow           (_zz_loadLow                      ), //i
    .cmpHigh           (coreArea_timerA_cmpHigh[15:0]    ), //i
    .cmpLow            (coreArea_timerA_cmpLow[15:0]     ), //i
    .enable            (coreArea_timerA_enable[15:0]     ), //i
    .accessEnableWrite (_zz_accessEnableWrite            ), //i
    .enableState       (coreArea_timerA_enableState[15:0]), //o
    .highState         (coreArea_timerA_highState[15:0]  ), //o
    .lowState          (coreArea_timerA_lowState[15:0]   ), //o
    .interrupt         (coreArea_timerA_interrupt        ), //o
    .core_clk          (core_clk                         ), //i
    .core_reset        (core_reset                       )  //i
  );
  Timer coreArea_timerB (
    .loadHigh          (_zz_loadHigh_1                   ), //i
    .loadLow           (_zz_loadLow_1                    ), //i
    .cmpHigh           (coreArea_timerB_cmpHigh[15:0]    ), //i
    .cmpLow            (coreArea_timerB_cmpLow[15:0]     ), //i
    .enable            (coreArea_timerB_enable[15:0]     ), //i
    .accessEnableWrite (_zz_accessEnableWrite_1          ), //i
    .enableState       (coreArea_timerB_enableState[15:0]), //o
    .highState         (coreArea_timerB_highState[15:0]  ), //o
    .lowState          (coreArea_timerB_lowState[15:0]   ), //o
    .interrupt         (coreArea_timerB_interrupt        ), //o
    .core_clk          (core_clk                         ), //i
    .core_reset        (core_reset                       )  //i
  );
  UartCtrl coreArea_uartCtrl (
    .io_config_frame_dataLength (coreArea_uartBridge_uartConfigReg_frame_dataLength[2:0]                          ), //i
    .io_config_frame_stop       (coreArea_uartBridge_uartConfigReg_frame_stop                                     ), //i
    .io_config_frame_parity     (coreArea_uartBridge_uartConfigReg_frame_parity[1:0]                              ), //i
    .io_config_clockDivider     (coreArea_uartBridge_uartConfigReg_clockDivider[19:0]                             ), //i
    .io_write_valid             (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid       ), //i
    .io_write_ready             (coreArea_uartCtrl_io_write_ready                                                 ), //o
    .io_write_payload           (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload[7:0]), //i
    .io_read_valid              (coreArea_uartCtrl_io_read_valid                                                  ), //o
    .io_read_ready              (toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_push_ready              ), //i
    .io_read_payload            (coreArea_uartCtrl_io_read_payload[7:0]                                           ), //o
    .io_uart_txd                (coreArea_uartCtrl_io_uart_txd                                                    ), //o
    .io_uart_rxd                (rx                                                                               ), //i
    .io_readError               (coreArea_uartCtrl_io_readError                                                   ), //o
    .io_writeBreak              (coreArea_uartBridge_misc_doBreak                                                 ), //i
    .io_readBreak               (coreArea_uartCtrl_io_readBreak                                                   ), //o
    .core_clk                   (core_clk                                                                         ), //i
    .core_reset                 (core_reset                                                                       )  //i
  );
  StreamFifo coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy (
    .io_push_valid   (coreArea_uartBridge_write_streamUnbuffered_valid                                  ), //i
    .io_push_ready   (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready       ), //o
    .io_push_payload (coreArea_uartBridge_write_streamUnbuffered_payload[7:0]                           ), //i
    .io_pop_valid    (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid        ), //o
    .io_pop_ready    (coreArea_uartCtrl_io_write_ready                                                  ), //i
    .io_pop_payload  (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_payload[7:0] ), //o
    .io_flush        (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_flush            ), //i
    .io_occupancy    (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_occupancy[3:0]   ), //o
    .io_availability (coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_availability[3:0]), //o
    .core_clk        (core_clk                                                                          ), //i
    .core_reset      (core_reset                                                                        )  //i
  );
  StreamFifo toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy (
    .io_push_valid   (coreArea_uartCtrl_io_read_valid                                           ), //i
    .io_push_ready   (toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_push_ready       ), //o
    .io_push_payload (coreArea_uartCtrl_io_read_payload[7:0]                                    ), //i
    .io_pop_valid    (toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_valid        ), //o
    .io_pop_ready    (toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_ready        ), //i
    .io_pop_payload  (toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_payload[7:0] ), //o
    .io_flush        (toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_flush            ), //i
    .io_occupancy    (toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_occupancy[3:0]   ), //o
    .io_availability (toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_availability[3:0]), //o
    .core_clk        (core_clk                                                                  ), //i
    .core_reset      (core_reset                                                                )  //i
  );
  InterruptCtrl coreArea_intCtrl (
    .irqReqs            (coreArea_intCtrl_irqReqs[3:0]          ), //i
    .enableWriteNewMask (_zz_enableWriteNewMask                 ), //i
    .enableWriteIrqVec  (coreArea_intCtrl_enableWriteIrqVec[3:0]), //i
    .irqSetData         (coreArea_intCtrl_irqSetData[15:0]      ), //i
    .irqGetMask         (coreArea_intCtrl_irqGetMask[15:0]      ), //o
    .irqVectors_0       (coreArea_intCtrl_irqVectors_0[15:0]    ), //o
    .irqVectors_1       (coreArea_intCtrl_irqVectors_1[15:0]    ), //o
    .irqVectors_2       (coreArea_intCtrl_irqVectors_2[15:0]    ), //o
    .irqVectors_3       (coreArea_intCtrl_irqVectors_3[15:0]    ), //o
    .intVec             (coreArea_intCtrl_intVec[15:0]          ), //o
    .irq                (coreArea_intCtrl_irq                   ), //o
    .core_clk           (core_clk                               ), //i
    .core_reset         (core_reset                             )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(coreArea_uartBridge_uartConfigReg_frame_stop)
      UartStopType_ONE : coreArea_uartBridge_uartConfigReg_frame_stop_string = "ONE";
      UartStopType_TWO : coreArea_uartBridge_uartConfigReg_frame_stop_string = "TWO";
      default : coreArea_uartBridge_uartConfigReg_frame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(coreArea_uartBridge_uartConfigReg_frame_parity)
      UartParityType_NONE : coreArea_uartBridge_uartConfigReg_frame_parity_string = "NONE";
      UartParityType_EVEN : coreArea_uartBridge_uartConfigReg_frame_parity_string = "EVEN";
      UartParityType_ODD : coreArea_uartBridge_uartConfigReg_frame_parity_string = "ODD ";
      default : coreArea_uartBridge_uartConfigReg_frame_parity_string = "????";
    endcase
  end
  `endif

  assign tdo = jtagIface_jtagArea_jtag_tdo;
  assign core_clk = boardClk;
  assign _zz_1 = ((reset || jtagIface_jtagArea_jtag_jtagReset) || (! boardClkLocked));
  assign bufferCC_5_io_dataIn = (1'b0 ^ 1'b0);
  assign core_reset = _zz_core_reset;
  assign coreArea_peripheralBus_address = toplevel_coreArea_cpu_cpuBus_address_delay_1;
  assign coreArea_peripheralBus_enable = toplevel_coreArea_cpu_cpuBus_enable_delay_1;
  assign coreArea_peripheralBus_writeMode = toplevel_coreArea_cpu_cpuBus_writeMode_delay_1;
  assign coreArea_peripheralBus_writeData = toplevel_coreArea_cpu_cpuBus_writeData_delay_1;
  assign coreArea_peripheralBusCtrl_readErrorFlag = 1'b0;
  assign coreArea_peripheralBusCtrl_writeErrorFlag = 1'b0;
  assign coreArea_peripheralBusCtrl_askWrite = (coreArea_peripheralBus_enable && coreArea_peripheralBus_writeMode);
  assign coreArea_peripheralBusCtrl_askRead = (coreArea_peripheralBus_enable && (! coreArea_peripheralBus_writeMode));
  always @(*) begin
    _zz_dirEnable = 1'b0;
    _zz_dataEnable = 1'b0;
    _zz_loadLow = 1'b0;
    _zz_loadHigh = 1'b0;
    _zz_accessEnableWrite = 1'b0;
    _zz_loadLow_1 = 1'b0;
    _zz_loadHigh_1 = 1'b0;
    _zz_accessEnableWrite_1 = 1'b0;
    _zz_coreArea_uartBridge_write_streamUnbuffered_valid = 1'b0;
    coreArea_uartBridge_read_streamBreaked_ready = 1'b0;
    when_BusSlaveFactory_l341 = 1'b0;
    when_BusSlaveFactory_l341_1 = 1'b0;
    when_BusSlaveFactory_l341_2 = 1'b0;
    when_BusSlaveFactory_l377 = 1'b0;
    when_BusSlaveFactory_l341_3 = 1'b0;
    _zz_enableWriteNewMask = 1'b0;
    _zz_enableWriteIrqVec = 1'b0;
    _zz_enableWriteIrqVec_1 = 1'b0;
    _zz_enableWriteIrqVec_2 = 1'b0;
    _zz_enableWriteIrqVec_3 = 1'b0;
    coreArea_peripheralBus_readData = 16'h0000;
    case(coreArea_peripheralBus_address)
      16'h0070 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_dirEnable = 1'b1;
        end
        coreArea_peripheralBus_readData[7 : 0] = coreArea_pmodA_directions;
      end
      16'h0074 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_dataEnable = 1'b1;
        end
        coreArea_peripheralBus_readData[7 : 0] = coreArea_pmodA_dataOut;
      end
      16'h00c0 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_loadLow = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerA_lowState;
      end
      16'h00c1 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_loadHigh = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerA_highState;
      end
      16'h00c2 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_accessEnableWrite = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerA_enableState;
      end
      16'h00d0 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_loadLow_1 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerB_lowState;
      end
      16'h00d1 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_loadHigh_1 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerB_highState;
      end
      16'h00d2 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_accessEnableWrite_1 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_timerB_enableState;
      end
      16'h00f0 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_coreArea_uartBridge_write_streamUnbuffered_valid = 1'b1;
        end
        if(coreArea_peripheralBusCtrl_askRead) begin
          coreArea_uartBridge_read_streamBreaked_ready = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 15] = (coreArea_uartBridge_read_streamBreaked_valid ^ 1'b0);
        coreArea_peripheralBus_readData[7 : 0] = coreArea_uartBridge_read_streamBreaked_payload;
      end
      16'h00f6 : begin
        coreArea_peripheralBus_readData[3 : 0] = _zz_coreArea_peripheralBus_readData;
        coreArea_peripheralBus_readData[11 : 8] = toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_occupancy;
      end
      16'h00f4 : begin
        coreArea_peripheralBus_readData[15 : 15] = coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid;
        coreArea_peripheralBus_readData[0 : 0] = coreArea_uartBridge_interruptCtrl_writeIntEnable;
        coreArea_peripheralBus_readData[1 : 1] = coreArea_uartBridge_interruptCtrl_readIntEnable;
        coreArea_peripheralBus_readData[8 : 8] = coreArea_uartBridge_interruptCtrl_writeInt;
        coreArea_peripheralBus_readData[9 : 9] = coreArea_uartBridge_interruptCtrl_readInt;
      end
      16'h0100 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          when_BusSlaveFactory_l341 = 1'b1;
          when_BusSlaveFactory_l341_1 = 1'b1;
          when_BusSlaveFactory_l341_2 = 1'b1;
          when_BusSlaveFactory_l377 = 1'b1;
          when_BusSlaveFactory_l341_3 = 1'b1;
        end
        coreArea_peripheralBus_readData[0 : 0] = coreArea_uartBridge_misc_readError;
        coreArea_peripheralBus_readData[1 : 1] = coreArea_uartBridge_misc_readOverflowError;
        coreArea_peripheralBus_readData[8 : 8] = coreArea_uartCtrl_io_readBreak;
        coreArea_peripheralBus_readData[9 : 9] = coreArea_uartBridge_misc_breakDetected;
      end
      16'h00e4 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_enableWriteNewMask = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_intCtrl_irqGetMask;
      end
      16'h00e0 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_enableWriteIrqVec = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_intCtrl_irqVectors_0;
      end
      16'h00e1 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_enableWriteIrqVec_1 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_intCtrl_irqVectors_1;
      end
      16'h00e2 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_enableWriteIrqVec_2 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_intCtrl_irqVectors_2;
      end
      16'h00e3 : begin
        if(coreArea_peripheralBusCtrl_askWrite) begin
          _zz_enableWriteIrqVec_3 = 1'b1;
        end
        coreArea_peripheralBus_readData[15 : 0] = coreArea_intCtrl_irqVectors_3;
      end
      default : begin
      end
    endcase
  end

  assign pmodA_write = coreArea_pmodA_dataOut;
  assign pmodA_writeEnable = coreArea_pmodA_directions;
  assign coreArea_uartBridge_busCtrlWrapped_readErrorFlag = 1'b0;
  assign coreArea_uartBridge_busCtrlWrapped_writeErrorFlag = 1'b0;
  assign _zz_2 = zz_coreArea_uartBridge_uartConfigReg_clockDivider(1'b0);
  always @(*) coreArea_uartBridge_uartConfigReg_clockDivider = _zz_2;
  assign coreArea_uartBridge_uartConfigReg_frame_dataLength = 3'b111;
  assign coreArea_uartBridge_uartConfigReg_frame_parity = UartParityType_NONE;
  assign coreArea_uartBridge_uartConfigReg_frame_stop = UartStopType_ONE;
  assign coreArea_uartBridge_write_streamUnbuffered_valid = _zz_coreArea_uartBridge_write_streamUnbuffered_valid;
  assign coreArea_uartBridge_write_streamUnbuffered_payload = coreArea_peripheralBus_writeData[7 : 0];
  assign coreArea_uartBridge_write_streamUnbuffered_ready = coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_push_ready;
  always @(*) begin
    coreArea_uartBridge_read_streamBreaked_valid = toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_valid;
    toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_ready = coreArea_uartBridge_read_streamBreaked_ready;
    if(coreArea_uartCtrl_io_readBreak) begin
      coreArea_uartBridge_read_streamBreaked_valid = 1'b0;
      toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_ready = 1'b1;
    end
  end

  assign coreArea_uartBridge_read_streamBreaked_payload = toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_pop_payload;
  assign coreArea_uartBridge_interruptCtrl_readInt = (coreArea_uartBridge_interruptCtrl_readIntEnable && coreArea_uartBridge_read_streamBreaked_valid);
  assign coreArea_uartBridge_interruptCtrl_writeInt = (coreArea_uartBridge_interruptCtrl_writeIntEnable && (! coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_pop_valid));
  assign coreArea_uartBridge_interruptCtrl_interrupt = (coreArea_uartBridge_interruptCtrl_readInt || coreArea_uartBridge_interruptCtrl_writeInt);
  assign when_BusSlaveFactory_l347 = coreArea_peripheralBus_writeData[0];
  assign when_BusSlaveFactory_l347_1 = coreArea_peripheralBus_writeData[1];
  assign toplevel_coreArea_uartCtrl_io_read_isStall = (coreArea_uartCtrl_io_read_valid && (! toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_push_ready));
  assign when_UartCtrl_l155 = (coreArea_uartCtrl_io_readBreak && (! toplevel_coreArea_uartCtrl_io_readBreak_regNext));
  assign when_BusSlaveFactory_l347_2 = coreArea_peripheralBus_writeData[9];
  assign when_BusSlaveFactory_l379 = coreArea_peripheralBus_writeData[10];
  assign when_BusSlaveFactory_l347_3 = coreArea_peripheralBus_writeData[11];
  assign tx = coreArea_uartCtrl_io_uart_txd;
  always @(*) begin
    coreArea_intCtrl_enableWriteIrqVec[0] = _zz_enableWriteIrqVec;
    coreArea_intCtrl_enableWriteIrqVec[1] = _zz_enableWriteIrqVec_1;
    coreArea_intCtrl_enableWriteIrqVec[2] = _zz_enableWriteIrqVec_2;
    coreArea_intCtrl_enableWriteIrqVec[3] = _zz_enableWriteIrqVec_3;
  end

  always @(*) begin
    coreArea_intCtrl_irqReqs[3 : 1] = extInt;
    coreArea_intCtrl_irqReqs[0] = coreArea_uartBridge_interruptCtrl_readInt;
    coreArea_intCtrl_irqReqs[1] = coreArea_timerA_interrupt;
    coreArea_intCtrl_irqReqs[2] = coreArea_timerB_interrupt;
  end

  assign coreArea_cpu_intVec = coreArea_intCtrl_intVec[7:0];
  assign coreArea_pmodA_dirValue = coreArea_peripheralBus_writeData[7 : 0];
  assign coreArea_pmodA_dataValue = coreArea_peripheralBus_writeData[7 : 0];
  assign coreArea_timerA_cmpLow = coreArea_peripheralBus_writeData[15 : 0];
  assign coreArea_timerA_cmpHigh = coreArea_peripheralBus_writeData[15 : 0];
  assign coreArea_timerA_enable = coreArea_peripheralBus_writeData[15 : 0];
  assign coreArea_timerB_cmpLow = coreArea_peripheralBus_writeData[15 : 0];
  assign coreArea_timerB_cmpHigh = coreArea_peripheralBus_writeData[15 : 0];
  assign coreArea_timerB_enable = coreArea_peripheralBus_writeData[15 : 0];
  assign coreArea_intCtrl_irqSetData = coreArea_peripheralBus_writeData[15 : 0];
  assign coreArea_uartBridge_write_streamUnbuffered_queueWithOccupancy_io_flush = 1'b0;
  assign toplevel_coreArea_uartCtrl_io_read_queueWithOccupancy_io_flush = 1'b0;
  always @(posedge core_clk) begin
    _zz_core_reset <= bufferCC_5_io_dataOut;
    if(flowCCByToggle_1_io_output_payload_jtagDataValid) begin
      toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagDataValid <= flowCCByToggle_1_io_output_payload_jtagDataValid;
      toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagStall <= flowCCByToggle_1_io_output_payload_jtagStall;
      toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagCaptureMemory <= flowCCByToggle_1_io_output_payload_jtagCaptureMemory;
      toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUAdr <= flowCCByToggle_1_io_output_payload_jtagCPUAdr;
      toplevel_flowCCByToggle_1_io_output_payload_regNextWhen_jtagCPUWord <= flowCCByToggle_1_io_output_payload_jtagCPUWord;
    end
    toplevel_coreArea_cpu_cpuBus_address_delay_1 <= coreArea_cpu_cpuBus_address;
    toplevel_coreArea_cpu_cpuBus_enable_delay_1 <= coreArea_cpu_cpuBus_enable;
    toplevel_coreArea_cpu_cpuBus_writeMode_delay_1 <= coreArea_cpu_cpuBus_writeMode;
    toplevel_coreArea_cpu_cpuBus_writeData_delay_1 <= coreArea_cpu_cpuBus_writeData;
    toplevel_coreArea_uartCtrl_io_readBreak_regNext <= coreArea_uartCtrl_io_readBreak;
  end

  always @(posedge core_clk) begin
    if(core_reset) begin
      coreArea_uartBridge_interruptCtrl_writeIntEnable <= 1'b0;
      coreArea_uartBridge_interruptCtrl_readIntEnable <= 1'b0;
      coreArea_uartBridge_misc_readError <= 1'b0;
      coreArea_uartBridge_misc_readOverflowError <= 1'b0;
      coreArea_uartBridge_misc_breakDetected <= 1'b0;
      coreArea_uartBridge_misc_doBreak <= 1'b0;
    end else begin
      if(when_BusSlaveFactory_l341) begin
        if(when_BusSlaveFactory_l347) begin
          coreArea_uartBridge_misc_readError <= _zz_coreArea_uartBridge_misc_readError[0];
        end
      end
      if(coreArea_uartCtrl_io_readError) begin
        coreArea_uartBridge_misc_readError <= 1'b1;
      end
      if(when_BusSlaveFactory_l341_1) begin
        if(when_BusSlaveFactory_l347_1) begin
          coreArea_uartBridge_misc_readOverflowError <= _zz_coreArea_uartBridge_misc_readOverflowError[0];
        end
      end
      if(toplevel_coreArea_uartCtrl_io_read_isStall) begin
        coreArea_uartBridge_misc_readOverflowError <= 1'b1;
      end
      if(when_UartCtrl_l155) begin
        coreArea_uartBridge_misc_breakDetected <= 1'b1;
      end
      if(when_BusSlaveFactory_l341_2) begin
        if(when_BusSlaveFactory_l347_2) begin
          coreArea_uartBridge_misc_breakDetected <= _zz_coreArea_uartBridge_misc_breakDetected[0];
        end
      end
      if(when_BusSlaveFactory_l377) begin
        if(when_BusSlaveFactory_l379) begin
          coreArea_uartBridge_misc_doBreak <= _zz_coreArea_uartBridge_misc_doBreak[0];
        end
      end
      if(when_BusSlaveFactory_l341_3) begin
        if(when_BusSlaveFactory_l347_3) begin
          coreArea_uartBridge_misc_doBreak <= _zz_coreArea_uartBridge_misc_doBreak_1[0];
        end
      end
      coreArea_uartBridge_interruptCtrl_readIntEnable <= 1'b1;
      case(coreArea_peripheralBus_address)
        16'h00f4 : begin
          if(coreArea_peripheralBusCtrl_askWrite) begin
            coreArea_uartBridge_interruptCtrl_writeIntEnable <= coreArea_peripheralBus_writeData[0];
            coreArea_uartBridge_interruptCtrl_readIntEnable <= coreArea_peripheralBus_writeData[1];
          end
        end
        default : begin
        end
      endcase
    end
  end


endmodule

module InterruptCtrl (
  input  wire [3:0]    irqReqs,
  input  wire          enableWriteNewMask,
  input  wire [3:0]    enableWriteIrqVec,
  input  wire [15:0]   irqSetData,
  output wire [15:0]   irqGetMask,
  output wire [15:0]   irqVectors_0,
  output wire [15:0]   irqVectors_1,
  output wire [15:0]   irqVectors_2,
  output wire [15:0]   irqVectors_3,
  output wire [15:0]   intVec,
  output wire          irq,
  input  wire          core_clk,
  input  wire          core_reset
);

  wire       [3:0]    irqReqs_buffercc_io_dataOut;
  wire       [14:0]   _zz_irqVectors_0_1;
  wire       [14:0]   _zz_irqVectors_1_1;
  wire       [14:0]   _zz_irqVectors_2_1;
  wire       [14:0]   _zz_irqVectors_3_1;
  wire       [3:0]    _zz_irqSync_ohFirst_masked;
  reg        [7:0]    _zz_intVec;
  reg        [3:0]    irqMask;
  wire       [3:0]    irqVecWriteEnable;
  wire                when_InterruptCtrl_l62;
  reg        [7:0]    irqVectors_0_1;
  wire                when_InterruptCtrl_l62_1;
  reg        [7:0]    irqVectors_1_1;
  wire                when_InterruptCtrl_l62_2;
  reg        [7:0]    irqVectors_2_1;
  wire                when_InterruptCtrl_l62_3;
  reg        [7:0]    irqVectors_3_1;
  wire       [3:0]    irqSync;
  wire       [3:0]    irqSync_ohFirst_input;
  wire       [3:0]    irqSync_ohFirst_masked;
  wire       [3:0]    irqSync_ohFirst_value;
  wire                _zz_intNo;
  wire                _zz_intNo_1;
  wire                _zz_intNo_2;
  wire       [1:0]    intNo;
  wire                _zz_irq;
  reg                 _zz_irq_1;

  assign _zz_irqVectors_0_1 = (irqSetData >>> 1'd1);
  assign _zz_irqVectors_1_1 = (irqSetData >>> 1'd1);
  assign _zz_irqVectors_2_1 = (irqSetData >>> 1'd1);
  assign _zz_irqVectors_3_1 = (irqSetData >>> 1'd1);
  assign _zz_irqSync_ohFirst_masked = (irqSync_ohFirst_input - 4'b0001);
  BufferCC_1 irqReqs_buffercc (
    .io_dataIn  (irqReqs[3:0]                    ), //i
    .io_dataOut (irqReqs_buffercc_io_dataOut[3:0]), //o
    .core_clk   (core_clk                        ), //i
    .core_reset (core_reset                      )  //i
  );
  always @(*) begin
    case(intNo)
      2'b00 : _zz_intVec = irqVectors_0_1;
      2'b01 : _zz_intVec = irqVectors_1_1;
      2'b10 : _zz_intVec = irqVectors_2_1;
      default : _zz_intVec = irqVectors_3_1;
    endcase
  end

  assign irqGetMask = {12'd0, irqMask};
  assign irqVecWriteEnable = enableWriteIrqVec;
  assign when_InterruptCtrl_l62 = irqVecWriteEnable[0];
  assign when_InterruptCtrl_l62_1 = irqVecWriteEnable[1];
  assign when_InterruptCtrl_l62_2 = irqVecWriteEnable[2];
  assign when_InterruptCtrl_l62_3 = irqVecWriteEnable[3];
  assign irqVectors_0 = {8'd0, irqVectors_0_1};
  assign irqVectors_1 = {8'd0, irqVectors_1_1};
  assign irqVectors_2 = {8'd0, irqVectors_2_1};
  assign irqVectors_3 = {8'd0, irqVectors_3_1};
  assign irqSync = irqReqs_buffercc_io_dataOut;
  assign irqSync_ohFirst_input = irqSync;
  assign irqSync_ohFirst_masked = (irqSync_ohFirst_input & (~ _zz_irqSync_ohFirst_masked));
  assign irqSync_ohFirst_value = irqSync_ohFirst_masked;
  assign _zz_intNo = irqSync_ohFirst_value[3];
  assign _zz_intNo_1 = (irqSync_ohFirst_value[1] || _zz_intNo);
  assign _zz_intNo_2 = (irqSync_ohFirst_value[2] || _zz_intNo);
  assign intNo = {_zz_intNo_2,_zz_intNo_1};
  assign intVec = {8'd0, _zz_intVec};
  assign _zz_irq = (|(irqSync & irqMask));
  assign irq = (_zz_irq && (! _zz_irq_1));
  always @(posedge core_clk) begin
    if(core_reset) begin
      irqMask <= 4'b0000;
      irqVectors_0_1 <= 8'h00;
      irqVectors_1_1 <= 8'h00;
      irqVectors_2_1 <= 8'h00;
      irqVectors_3_1 <= 8'h00;
      _zz_irq_1 <= 1'b0;
    end else begin
      if(enableWriteNewMask) begin
        irqMask <= irqSetData[3:0];
      end
      if(when_InterruptCtrl_l62) begin
        irqVectors_0_1 <= _zz_irqVectors_0_1[7:0];
      end
      if(when_InterruptCtrl_l62_1) begin
        irqVectors_1_1 <= _zz_irqVectors_1_1[7:0];
      end
      if(when_InterruptCtrl_l62_2) begin
        irqVectors_2_1 <= _zz_irqVectors_2_1[7:0];
      end
      if(when_InterruptCtrl_l62_3) begin
        irqVectors_3_1 <= _zz_irqVectors_3_1[7:0];
      end
      _zz_irq_1 <= _zz_irq;
    end
  end


endmodule

//StreamFifo_1 replaced by StreamFifo

module StreamFifo (
  input  wire          io_push_valid,
  output wire          io_push_ready,
  input  wire [7:0]    io_push_payload,
  output wire          io_pop_valid,
  input  wire          io_pop_ready,
  output wire [7:0]    io_pop_payload,
  input  wire          io_flush,
  output wire [3:0]    io_occupancy,
  output wire [3:0]    io_availability,
  input  wire          core_clk,
  input  wire          core_reset
);

  reg        [7:0]    _zz_logic_ram_port1;
  reg                 _zz_1;
  wire                logic_ptr_doPush;
  wire                logic_ptr_doPop;
  wire                logic_ptr_full;
  wire                logic_ptr_empty;
  reg        [3:0]    logic_ptr_push;
  reg        [3:0]    logic_ptr_pop;
  wire       [3:0]    logic_ptr_occupancy;
  wire       [3:0]    logic_ptr_popOnIo;
  wire                when_Stream_l1205;
  reg                 logic_ptr_wentUp;
  wire                io_push_fire;
  wire                logic_push_onRam_write_valid;
  wire       [2:0]    logic_push_onRam_write_payload_address;
  wire       [7:0]    logic_push_onRam_write_payload_data;
  wire                logic_pop_addressGen_valid;
  reg                 logic_pop_addressGen_ready;
  wire       [2:0]    logic_pop_addressGen_payload;
  wire                logic_pop_addressGen_fire;
  wire                logic_pop_sync_readArbitation_valid;
  wire                logic_pop_sync_readArbitation_ready;
  wire       [2:0]    logic_pop_sync_readArbitation_payload;
  reg                 logic_pop_addressGen_rValid;
  reg        [2:0]    logic_pop_addressGen_rData;
  wire                when_Stream_l369;
  wire                logic_pop_sync_readPort_cmd_valid;
  wire       [2:0]    logic_pop_sync_readPort_cmd_payload;
  wire       [7:0]    logic_pop_sync_readPort_rsp;
  wire                logic_pop_sync_readArbitation_translated_valid;
  wire                logic_pop_sync_readArbitation_translated_ready;
  wire       [7:0]    logic_pop_sync_readArbitation_translated_payload;
  wire                logic_pop_sync_readArbitation_fire;
  reg        [3:0]    logic_pop_sync_popReg;
  reg [7:0] logic_ram [0:7];

  always @(posedge core_clk) begin
    if(_zz_1) begin
      logic_ram[logic_push_onRam_write_payload_address] <= logic_push_onRam_write_payload_data;
    end
  end

  always @(posedge core_clk) begin
    if(logic_pop_sync_readPort_cmd_valid) begin
      _zz_logic_ram_port1 <= logic_ram[logic_pop_sync_readPort_cmd_payload];
    end
  end

  always @(*) begin
    _zz_1 = 1'b0;
    if(logic_push_onRam_write_valid) begin
      _zz_1 = 1'b1;
    end
  end

  assign when_Stream_l1205 = (logic_ptr_doPush != logic_ptr_doPop);
  assign logic_ptr_full = (((logic_ptr_push ^ logic_ptr_popOnIo) ^ 4'b1000) == 4'b0000);
  assign logic_ptr_empty = (logic_ptr_push == logic_ptr_pop);
  assign logic_ptr_occupancy = (logic_ptr_push - logic_ptr_popOnIo);
  assign io_push_ready = (! logic_ptr_full);
  assign io_push_fire = (io_push_valid && io_push_ready);
  assign logic_ptr_doPush = io_push_fire;
  assign logic_push_onRam_write_valid = io_push_fire;
  assign logic_push_onRam_write_payload_address = logic_ptr_push[2:0];
  assign logic_push_onRam_write_payload_data = io_push_payload;
  assign logic_pop_addressGen_valid = (! logic_ptr_empty);
  assign logic_pop_addressGen_payload = logic_ptr_pop[2:0];
  assign logic_pop_addressGen_fire = (logic_pop_addressGen_valid && logic_pop_addressGen_ready);
  assign logic_ptr_doPop = logic_pop_addressGen_fire;
  always @(*) begin
    logic_pop_addressGen_ready = logic_pop_sync_readArbitation_ready;
    if(when_Stream_l369) begin
      logic_pop_addressGen_ready = 1'b1;
    end
  end

  assign when_Stream_l369 = (! logic_pop_sync_readArbitation_valid);
  assign logic_pop_sync_readArbitation_valid = logic_pop_addressGen_rValid;
  assign logic_pop_sync_readArbitation_payload = logic_pop_addressGen_rData;
  assign logic_pop_sync_readPort_rsp = _zz_logic_ram_port1;
  assign logic_pop_sync_readPort_cmd_valid = logic_pop_addressGen_fire;
  assign logic_pop_sync_readPort_cmd_payload = logic_pop_addressGen_payload;
  assign logic_pop_sync_readArbitation_translated_valid = logic_pop_sync_readArbitation_valid;
  assign logic_pop_sync_readArbitation_ready = logic_pop_sync_readArbitation_translated_ready;
  assign logic_pop_sync_readArbitation_translated_payload = logic_pop_sync_readPort_rsp;
  assign io_pop_valid = logic_pop_sync_readArbitation_translated_valid;
  assign logic_pop_sync_readArbitation_translated_ready = io_pop_ready;
  assign io_pop_payload = logic_pop_sync_readArbitation_translated_payload;
  assign logic_pop_sync_readArbitation_fire = (logic_pop_sync_readArbitation_valid && logic_pop_sync_readArbitation_ready);
  assign logic_ptr_popOnIo = logic_pop_sync_popReg;
  assign io_occupancy = logic_ptr_occupancy;
  assign io_availability = (4'b1000 - logic_ptr_occupancy);
  always @(posedge core_clk) begin
    if(core_reset) begin
      logic_ptr_push <= 4'b0000;
      logic_ptr_pop <= 4'b0000;
      logic_ptr_wentUp <= 1'b0;
      logic_pop_addressGen_rValid <= 1'b0;
      logic_pop_sync_popReg <= 4'b0000;
    end else begin
      if(when_Stream_l1205) begin
        logic_ptr_wentUp <= logic_ptr_doPush;
      end
      if(io_flush) begin
        logic_ptr_wentUp <= 1'b0;
      end
      if(logic_ptr_doPush) begin
        logic_ptr_push <= (logic_ptr_push + 4'b0001);
      end
      if(logic_ptr_doPop) begin
        logic_ptr_pop <= (logic_ptr_pop + 4'b0001);
      end
      if(io_flush) begin
        logic_ptr_push <= 4'b0000;
        logic_ptr_pop <= 4'b0000;
      end
      if(logic_pop_addressGen_ready) begin
        logic_pop_addressGen_rValid <= logic_pop_addressGen_valid;
      end
      if(io_flush) begin
        logic_pop_addressGen_rValid <= 1'b0;
      end
      if(logic_pop_sync_readArbitation_fire) begin
        logic_pop_sync_popReg <= logic_ptr_pop;
      end
      if(io_flush) begin
        logic_pop_sync_popReg <= 4'b0000;
      end
    end
  end

  always @(posedge core_clk) begin
    if(logic_pop_addressGen_ready) begin
      logic_pop_addressGen_rData <= logic_pop_addressGen_payload;
    end
  end


endmodule

module UartCtrl (
  input  wire [2:0]    io_config_frame_dataLength,
  input  wire [0:0]    io_config_frame_stop,
  input  wire [1:0]    io_config_frame_parity,
  input  wire [19:0]   io_config_clockDivider,
  input  wire          io_write_valid,
  output reg           io_write_ready,
  input  wire [7:0]    io_write_payload,
  output wire          io_read_valid,
  input  wire          io_read_ready,
  output wire [7:0]    io_read_payload,
  output wire          io_uart_txd,
  input  wire          io_uart_rxd,
  output wire          io_readError,
  input  wire          io_writeBreak,
  output wire          io_readBreak,
  input  wire          core_clk,
  input  wire          core_reset
);
  localparam UartStopType_ONE = 1'd0;
  localparam UartStopType_TWO = 1'd1;
  localparam UartParityType_NONE = 2'd0;
  localparam UartParityType_EVEN = 2'd1;
  localparam UartParityType_ODD = 2'd2;

  wire                tx_io_cts;
  wire                tx_io_write_ready;
  wire                tx_io_txd;
  wire                rx_io_read_valid;
  wire       [7:0]    rx_io_read_payload;
  wire                rx_io_rts;
  wire                rx_io_error;
  wire                rx_io_break;
  reg        [19:0]   clockDivider_counter;
  wire                clockDivider_tick;
  reg                 clockDivider_tickReg;
  reg                 io_write_thrown_valid;
  wire                io_write_thrown_ready;
  wire       [7:0]    io_write_thrown_payload;
  `ifndef SYNTHESIS
  reg [23:0] io_config_frame_stop_string;
  reg [31:0] io_config_frame_parity_string;
  `endif


  UartCtrlTx tx (
    .io_configFrame_dataLength (io_config_frame_dataLength[2:0]), //i
    .io_configFrame_stop       (io_config_frame_stop           ), //i
    .io_configFrame_parity     (io_config_frame_parity[1:0]    ), //i
    .io_samplingTick           (clockDivider_tickReg           ), //i
    .io_write_valid            (io_write_thrown_valid          ), //i
    .io_write_ready            (tx_io_write_ready              ), //o
    .io_write_payload          (io_write_thrown_payload[7:0]   ), //i
    .io_cts                    (tx_io_cts                      ), //i
    .io_txd                    (tx_io_txd                      ), //o
    .io_break                  (io_writeBreak                  ), //i
    .core_clk                  (core_clk                       ), //i
    .core_reset                (core_reset                     )  //i
  );
  UartCtrlRx rx (
    .io_configFrame_dataLength (io_config_frame_dataLength[2:0]), //i
    .io_configFrame_stop       (io_config_frame_stop           ), //i
    .io_configFrame_parity     (io_config_frame_parity[1:0]    ), //i
    .io_samplingTick           (clockDivider_tickReg           ), //i
    .io_read_valid             (rx_io_read_valid               ), //o
    .io_read_ready             (io_read_ready                  ), //i
    .io_read_payload           (rx_io_read_payload[7:0]        ), //o
    .io_rxd                    (io_uart_rxd                    ), //i
    .io_rts                    (rx_io_rts                      ), //o
    .io_error                  (rx_io_error                    ), //o
    .io_break                  (rx_io_break                    ), //o
    .core_clk                  (core_clk                       ), //i
    .core_reset                (core_reset                     )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_config_frame_stop)
      UartStopType_ONE : io_config_frame_stop_string = "ONE";
      UartStopType_TWO : io_config_frame_stop_string = "TWO";
      default : io_config_frame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_config_frame_parity)
      UartParityType_NONE : io_config_frame_parity_string = "NONE";
      UartParityType_EVEN : io_config_frame_parity_string = "EVEN";
      UartParityType_ODD : io_config_frame_parity_string = "ODD ";
      default : io_config_frame_parity_string = "????";
    endcase
  end
  `endif

  assign clockDivider_tick = (clockDivider_counter == 20'h00000);
  always @(*) begin
    io_write_thrown_valid = io_write_valid;
    io_write_ready = io_write_thrown_ready;
    if(rx_io_break) begin
      io_write_thrown_valid = 1'b0;
      io_write_ready = 1'b1;
    end
  end

  assign io_write_thrown_payload = io_write_payload;
  assign io_write_thrown_ready = tx_io_write_ready;
  assign io_read_valid = rx_io_read_valid;
  assign io_read_payload = rx_io_read_payload;
  assign io_uart_txd = tx_io_txd;
  assign io_readError = rx_io_error;
  assign tx_io_cts = 1'b0;
  assign io_readBreak = rx_io_break;
  always @(posedge core_clk) begin
    if(core_reset) begin
      clockDivider_counter <= 20'h00000;
      clockDivider_tickReg <= 1'b0;
    end else begin
      clockDivider_tickReg <= clockDivider_tick;
      clockDivider_counter <= (clockDivider_counter - 20'h00001);
      if(clockDivider_tick) begin
        clockDivider_counter <= io_config_clockDivider;
      end
    end
  end


endmodule

//Timer_1 replaced by Timer

module Timer (
  input  wire          loadHigh,
  input  wire          loadLow,
  input  wire [15:0]   cmpHigh,
  input  wire [15:0]   cmpLow,
  input  wire [15:0]   enable,
  input  wire          accessEnableWrite,
  output wire [15:0]   enableState,
  output wire [15:0]   highState,
  output wire [15:0]   lowState,
  output wire          interrupt,
  input  wire          core_clk,
  input  wire          core_reset
);

  reg        [31:0]   cnt;
  reg        [31:0]   cmp;
  reg                 isEnabled;
  wire       [31:0]   maxCnt;
  wire                when_Timer_l73;
  wire                when_Timer_l76;

  assign highState = cmp[31 : 16];
  assign lowState = cmp[15 : 0];
  assign enableState = (isEnabled ? 16'hffff : 16'h0000);
  assign maxCnt = (cmp - 32'h00000001);
  assign when_Timer_l73 = (isEnabled && (! (loadHigh || loadLow)));
  assign when_Timer_l76 = (cnt < maxCnt);
  assign interrupt = ((isEnabled && (cnt == maxCnt)) && (! (loadHigh || loadLow)));
  always @(posedge core_clk) begin
    if(core_reset) begin
      cnt <= 32'h00000000;
      cmp <= 32'h00000000;
      isEnabled <= 1'b0;
    end else begin
      if(accessEnableWrite) begin
        isEnabled <= (|enable);
      end
      if(loadLow) begin
        cmp[15 : 0] <= cmpLow;
        isEnabled <= 1'b0;
        cnt <= 32'h00000000;
      end
      if(loadHigh) begin
        cmp[31 : 16] <= cmpHigh;
        isEnabled <= 1'b0;
        cnt <= 32'h00000000;
      end
      if(when_Timer_l73) begin
        if(when_Timer_l76) begin
          cnt <= (cnt + 32'h00000001);
        end else begin
          cnt <= 32'h00000000;
        end
      end
    end
  end


endmodule

module GPIO (
  input  wire          dirEnable,
  input  wire [7:0]    dirValue,
  input  wire          dataEnable,
  input  wire [7:0]    dataValue,
  output wire [7:0]    directions,
  input  wire [7:0]    dataIn,
  output wire [7:0]    dataOut,
  input  wire          core_clk,
  input  wire          core_reset
);

  reg        [7:0]    dirReg;
  reg        [7:0]    dataRegN;
  reg        [7:0]    dataReg;

  assign directions = dirReg;
  always @(*) begin
    if(dataEnable) begin
      dataRegN = ((dataIn & (~ directions)) | (dataValue & directions));
    end else begin
      dataRegN = (dataReg | (dataIn & (~ directions)));
    end
  end

  assign dataOut = dataReg;
  always @(posedge core_clk) begin
    if(core_reset) begin
      dirReg <= 8'h00;
      dataReg <= 8'h00;
    end else begin
      if(dirEnable) begin
        dirReg <= dirValue;
      end
      dataReg <= dataRegN;
    end
  end


endmodule

module FlowCCByToggle (
  input  wire          io_input_valid,
  input  wire          io_input_payload_jtagDataValid,
  input  wire          io_input_payload_jtagStall,
  input  wire          io_input_payload_jtagCaptureMemory,
  input  wire [7:0]    io_input_payload_jtagCPUAdr,
  input  wire [15:0]   io_input_payload_jtagCPUWord,
  output wire          io_output_valid,
  output wire          io_output_payload_jtagDataValid,
  output wire          io_output_payload_jtagStall,
  output wire          io_output_payload_jtagCaptureMemory,
  output wire [7:0]    io_output_payload_jtagCPUAdr,
  output wire [15:0]   io_output_payload_jtagCPUWord,
  input  wire          tck,
  input  wire          reset,
  input  wire          core_clk
);

  wire                bufferCC_5_io_dataIn;
  wire                bufferCC_5_io_dataOut;
  wire                inputArea_target_buffercc_io_dataOut;
  wire                flowCCByToggle_1_toplevel_reset_syncronized;
  reg                 inputArea_target;
  reg                 inputArea_data_jtagDataValid;
  reg                 inputArea_data_jtagStall;
  reg                 inputArea_data_jtagCaptureMemory;
  reg        [7:0]    inputArea_data_jtagCPUAdr;
  reg        [15:0]   inputArea_data_jtagCPUWord;
  wire                outputArea_target;
  reg                 outputArea_hit;
  wire                outputArea_flow_valid;
  wire                outputArea_flow_payload_jtagDataValid;
  wire                outputArea_flow_payload_jtagStall;
  wire                outputArea_flow_payload_jtagCaptureMemory;
  wire       [7:0]    outputArea_flow_payload_jtagCPUAdr;
  wire       [15:0]   outputArea_flow_payload_jtagCPUWord;
  reg                 outputArea_flow_m2sPipe_valid;
  (* async_reg = "true" *) reg                 outputArea_flow_m2sPipe_payload_jtagDataValid;
  (* async_reg = "true" *) reg                 outputArea_flow_m2sPipe_payload_jtagStall;
  (* async_reg = "true" *) reg                 outputArea_flow_m2sPipe_payload_jtagCaptureMemory;
  (* async_reg = "true" *) reg        [7:0]    outputArea_flow_m2sPipe_payload_jtagCPUAdr;
  (* async_reg = "true" *) reg        [15:0]   outputArea_flow_m2sPipe_payload_jtagCPUWord;

  BufferCC_2 bufferCC_5 (
    .io_dataIn  (bufferCC_5_io_dataIn ), //i
    .io_dataOut (bufferCC_5_io_dataOut), //o
    .core_clk   (core_clk             ), //i
    .reset      (reset                )  //i
  );
  BufferCC_3 inputArea_target_buffercc (
    .io_dataIn                                   (inputArea_target                           ), //i
    .io_dataOut                                  (inputArea_target_buffercc_io_dataOut       ), //o
    .core_clk                                    (core_clk                                   ), //i
    .flowCCByToggle_1_toplevel_reset_syncronized (flowCCByToggle_1_toplevel_reset_syncronized)  //i
  );
  assign bufferCC_5_io_dataIn = (1'b0 ^ 1'b0);
  assign flowCCByToggle_1_toplevel_reset_syncronized = bufferCC_5_io_dataOut;
  assign outputArea_target = inputArea_target_buffercc_io_dataOut;
  assign outputArea_flow_valid = (outputArea_target != outputArea_hit);
  assign outputArea_flow_payload_jtagDataValid = inputArea_data_jtagDataValid;
  assign outputArea_flow_payload_jtagStall = inputArea_data_jtagStall;
  assign outputArea_flow_payload_jtagCaptureMemory = inputArea_data_jtagCaptureMemory;
  assign outputArea_flow_payload_jtagCPUAdr = inputArea_data_jtagCPUAdr;
  assign outputArea_flow_payload_jtagCPUWord = inputArea_data_jtagCPUWord;
  assign io_output_valid = outputArea_flow_m2sPipe_valid;
  assign io_output_payload_jtagDataValid = outputArea_flow_m2sPipe_payload_jtagDataValid;
  assign io_output_payload_jtagStall = outputArea_flow_m2sPipe_payload_jtagStall;
  assign io_output_payload_jtagCaptureMemory = outputArea_flow_m2sPipe_payload_jtagCaptureMemory;
  assign io_output_payload_jtagCPUAdr = outputArea_flow_m2sPipe_payload_jtagCPUAdr;
  assign io_output_payload_jtagCPUWord = outputArea_flow_m2sPipe_payload_jtagCPUWord;
  always @(posedge tck or posedge reset) begin
    if(reset) begin
      inputArea_target <= 1'b0;
    end else begin
      if(io_input_valid) begin
        inputArea_target <= (! inputArea_target);
      end
    end
  end

  always @(posedge tck) begin
    if(io_input_valid) begin
      inputArea_data_jtagDataValid <= io_input_payload_jtagDataValid;
      inputArea_data_jtagStall <= io_input_payload_jtagStall;
      inputArea_data_jtagCaptureMemory <= io_input_payload_jtagCaptureMemory;
      inputArea_data_jtagCPUAdr <= io_input_payload_jtagCPUAdr;
      inputArea_data_jtagCPUWord <= io_input_payload_jtagCPUWord;
    end
  end

  always @(posedge core_clk) begin
    if(flowCCByToggle_1_toplevel_reset_syncronized) begin
      outputArea_flow_m2sPipe_valid <= 1'b0;
      outputArea_hit <= 1'b0;
    end else begin
      outputArea_hit <= outputArea_target;
      outputArea_flow_m2sPipe_valid <= outputArea_flow_valid;
    end
  end

  always @(posedge core_clk) begin
    if(outputArea_flow_valid) begin
      outputArea_flow_m2sPipe_payload_jtagDataValid <= outputArea_flow_payload_jtagDataValid;
      outputArea_flow_m2sPipe_payload_jtagStall <= outputArea_flow_payload_jtagStall;
      outputArea_flow_m2sPipe_payload_jtagCaptureMemory <= outputArea_flow_payload_jtagCaptureMemory;
      outputArea_flow_m2sPipe_payload_jtagCPUAdr <= outputArea_flow_payload_jtagCPUAdr;
      outputArea_flow_m2sPipe_payload_jtagCPUWord <= outputArea_flow_payload_jtagCPUWord;
    end
  end


endmodule

module J1 (
  input  wire          stall,
  input  wire          irq,
  input  wire [7:0]    intVec,
  input  wire          captureMemory,
  input  wire [7:0]    jtagMemAdr,
  input  wire [15:0]   jtagMemWord,
  output wire          cpuBus_enable,
  output wire          cpuBus_writeMode,
  output wire [15:0]   cpuBus_address,
  output wire [15:0]   cpuBus_writeData,
  input  wire [15:0]   cpuBus_readData,
  input  wire          core_reset,
  input  wire          core_clk
);

  reg                 mainMem_writeEnable;
  reg        [7:0]    mainMem_writeDataAdr;
  reg        [15:0]   mainMem_writeData;
  wire                coreJ1CPU_memWriteMode;
  wire                coreJ1CPU_ioWriteMode;
  wire                coreJ1CPU_ioReadMode;
  wire       [15:0]   coreJ1CPU_extAdr;
  wire       [15:0]   coreJ1CPU_extToWrite;
  wire       [7:0]    coreJ1CPU_nextInstrAdr;
  wire       [15:0]   mainMem_readData;
  wire       [15:0]   coreMemRead;
  reg        [15:0]   coreArea_cpu_coreJ1CPU_extAdr_delay_1;

  J1Core coreJ1CPU (
    .memWriteMode (coreJ1CPU_memWriteMode     ), //o
    .ioWriteMode  (coreJ1CPU_ioWriteMode      ), //o
    .ioReadMode   (coreJ1CPU_ioReadMode       ), //o
    .extAdr       (coreJ1CPU_extAdr[15:0]     ), //o
    .extToWrite   (coreJ1CPU_extToWrite[15:0] ), //o
    .toRead       (coreMemRead[15:0]          ), //i
    .stall        (stall                      ), //i
    .irq          (irq                        ), //i
    .intVec       (intVec[7:0]                ), //i
    .nextInstrAdr (coreJ1CPU_nextInstrAdr[7:0]), //o
    .memInstr     (mainMem_readData[15:0]     ), //i
    .clrActive    (core_reset                 ), //i
    .core_clk     (core_clk                   )  //i
  );
  MainMemory mainMem (
    .readDataAdr  (coreJ1CPU_nextInstrAdr[7:0]), //i
    .readData     (mainMem_readData[15:0]     ), //o
    .writeEnable  (mainMem_writeEnable        ), //i
    .writeDataAdr (mainMem_writeDataAdr[7:0]  ), //i
    .writeData    (mainMem_writeData[15:0]    ), //i
    .core_clk     (core_clk                   ), //i
    .core_reset   (core_reset                 )  //i
  );
  always @(*) begin
    if(stall) begin
      mainMem_writeEnable = captureMemory;
      mainMem_writeDataAdr = jtagMemAdr;
      mainMem_writeData = jtagMemWord;
    end else begin
      mainMem_writeEnable = coreJ1CPU_memWriteMode;
      mainMem_writeDataAdr = coreJ1CPU_extAdr[8 : 1];
      mainMem_writeData = coreJ1CPU_extToWrite;
    end
  end

  assign coreMemRead = (coreJ1CPU_ioReadMode ? cpuBus_readData : 16'h0000);
  assign cpuBus_enable = (coreJ1CPU_ioWriteMode || coreJ1CPU_ioReadMode);
  assign cpuBus_writeMode = coreJ1CPU_ioWriteMode;
  assign cpuBus_address = coreArea_cpu_coreJ1CPU_extAdr_delay_1;
  assign cpuBus_writeData = coreJ1CPU_extToWrite;
  always @(posedge core_clk) begin
    coreArea_cpu_coreJ1CPU_extAdr_delay_1 <= coreJ1CPU_extAdr;
  end


endmodule

module BufferCC (
  input  wire          io_dataIn,
  output wire          io_dataOut,
  input  wire          core_clk,
  input  wire          _zz_1
);

  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @(posedge core_clk or posedge _zz_1) begin
    if(_zz_1) begin
      buffers_0 <= 1'b1;
      buffers_1 <= 1'b1;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule

module J1Jtag (
  input  wire          tdi,
  output reg           tdo,
  input  wire          tms,
  output wire          jtagDataFlow_valid,
  output wire          jtagDataFlow_payload_jtagDataValid,
  output wire          jtagDataFlow_payload_jtagStall,
  output wire          jtagDataFlow_payload_jtagCaptureMemory,
  output wire [7:0]    jtagDataFlow_payload_jtagCPUAdr,
  output wire [15:0]   jtagDataFlow_payload_jtagCPUWord,
  output wire          jtagReset,
  input  wire          tck,
  input  wire          reset
);
  localparam jtagFSM_enumDef_BOOT = 5'd0;
  localparam jtagFSM_enumDef_testLogicReset = 5'd1;
  localparam jtagFSM_enumDef_runTestIdle = 5'd2;
  localparam jtagFSM_enumDef_selectDRScan = 5'd3;
  localparam jtagFSM_enumDef_captureDR = 5'd4;
  localparam jtagFSM_enumDef_shiftDR = 5'd5;
  localparam jtagFSM_enumDef_exit1DR = 5'd6;
  localparam jtagFSM_enumDef_pauseDR = 5'd7;
  localparam jtagFSM_enumDef_exit2DR = 5'd8;
  localparam jtagFSM_enumDef_updateDR = 5'd9;
  localparam jtagFSM_enumDef_selectIRScan = 5'd10;
  localparam jtagFSM_enumDef_captureIR = 5'd11;
  localparam jtagFSM_enumDef_shiftIR = 5'd12;
  localparam jtagFSM_enumDef_exit1IR = 5'd13;
  localparam jtagFSM_enumDef_pauseIR = 5'd14;
  localparam jtagFSM_enumDef_exit2IR = 5'd15;
  localparam jtagFSM_enumDef_updateIR = 5'd16;

  reg                 jtagDataValid;
  wire       [4:0]    _zz_when_J1Jtag_l203;
  wire       [4:0]    _zz_when_J1Jtag_l203_1;
  wire       [4:0]    _zz_when_J1Jtag_l203_2;
  wire       [4:0]    _zz_when_J1Jtag_l203_3;
  wire       [4:0]    _zz_when_J1Jtag_l203_4;
  wire       [4:0]    _zz_when_J1Jtag_l203_5;
  wire       [4:0]    _zz_dataHoldRegs_1;
  reg        [4:0]    instructionShiftReg;
  reg        [4:0]    instructionHoldReg;
  reg        [0:0]    dataHoldRegs_0;
  reg        [31:0]   dataHoldRegs_1;
  reg        [0:0]    dataHoldRegs_2;
  reg        [0:0]    dataHoldRegs_3;
  reg        [0:0]    dataHoldRegs_4;
  reg        [7:0]    dataHoldRegs_5;
  reg        [15:0]   dataHoldRegs_6;
  reg        [0:0]    dataShiftRegs_0;
  reg        [31:0]   dataShiftRegs_1;
  reg        [0:0]    dataShiftRegs_2;
  reg        [0:0]    dataShiftRegs_3;
  reg        [0:0]    dataShiftRegs_4;
  reg        [7:0]    dataShiftRegs_5;
  reg        [15:0]   dataShiftRegs_6;
  wire                jtagFSM_wantExit;
  reg                 jtagFSM_wantStart;
  wire                jtagFSM_wantKill;
  wire                when_J1Jtag_l203;
  wire                when_J1Jtag_l203_1;
  wire                when_J1Jtag_l203_2;
  wire                when_J1Jtag_l203_3;
  wire                when_J1Jtag_l203_4;
  wire                when_J1Jtag_l203_5;
  wire                when_J1Jtag_l203_6;
  wire                jtagDataBundle_jtagDataValid;
  wire                jtagDataBundle_jtagStall;
  wire                jtagDataBundle_jtagCaptureMemory;
  wire       [7:0]    jtagDataBundle_jtagCPUAdr;
  wire       [15:0]   jtagDataBundle_jtagCPUWord;
  reg        [4:0]    jtagFSM_stateReg;
  reg        [4:0]    jtagFSM_stateNext;
  wire                when_J1Jtag_l259;
  wire                when_J1Jtag_l272;
  wire                when_J1Jtag_l259_1;
  wire                when_J1Jtag_l259_2;
  wire                when_J1Jtag_l259_3;
  wire                when_J1Jtag_l295;
  wire                when_J1Jtag_l295_1;
  wire                when_J1Jtag_l295_2;
  wire                when_J1Jtag_l295_3;
  wire                when_J1Jtag_l295_4;
  wire                when_J1Jtag_l295_5;
  wire                when_J1Jtag_l295_6;
  wire                when_J1Jtag_l319;
  wire                when_J1Jtag_l319_1;
  wire                when_J1Jtag_l319_2;
  wire                when_J1Jtag_l319_3;
  wire                when_J1Jtag_l319_4;
  wire                when_J1Jtag_l319_5;
  wire                when_StateMachine_l237;
  wire                when_StateMachine_l237_1;
  `ifndef SYNTHESIS
  reg [111:0] jtagFSM_stateReg_string;
  reg [111:0] jtagFSM_stateNext_string;
  `endif


  initial begin
  `ifndef SYNTHESIS
    instructionShiftReg = {$urandom};
    instructionHoldReg = {$urandom};
  `endif
  end

  `ifndef SYNTHESIS
  always @(*) begin
    case(jtagFSM_stateReg)
      jtagFSM_enumDef_BOOT : jtagFSM_stateReg_string = "BOOT          ";
      jtagFSM_enumDef_testLogicReset : jtagFSM_stateReg_string = "testLogicReset";
      jtagFSM_enumDef_runTestIdle : jtagFSM_stateReg_string = "runTestIdle   ";
      jtagFSM_enumDef_selectDRScan : jtagFSM_stateReg_string = "selectDRScan  ";
      jtagFSM_enumDef_captureDR : jtagFSM_stateReg_string = "captureDR     ";
      jtagFSM_enumDef_shiftDR : jtagFSM_stateReg_string = "shiftDR       ";
      jtagFSM_enumDef_exit1DR : jtagFSM_stateReg_string = "exit1DR       ";
      jtagFSM_enumDef_pauseDR : jtagFSM_stateReg_string = "pauseDR       ";
      jtagFSM_enumDef_exit2DR : jtagFSM_stateReg_string = "exit2DR       ";
      jtagFSM_enumDef_updateDR : jtagFSM_stateReg_string = "updateDR      ";
      jtagFSM_enumDef_selectIRScan : jtagFSM_stateReg_string = "selectIRScan  ";
      jtagFSM_enumDef_captureIR : jtagFSM_stateReg_string = "captureIR     ";
      jtagFSM_enumDef_shiftIR : jtagFSM_stateReg_string = "shiftIR       ";
      jtagFSM_enumDef_exit1IR : jtagFSM_stateReg_string = "exit1IR       ";
      jtagFSM_enumDef_pauseIR : jtagFSM_stateReg_string = "pauseIR       ";
      jtagFSM_enumDef_exit2IR : jtagFSM_stateReg_string = "exit2IR       ";
      jtagFSM_enumDef_updateIR : jtagFSM_stateReg_string = "updateIR      ";
      default : jtagFSM_stateReg_string = "??????????????";
    endcase
  end
  always @(*) begin
    case(jtagFSM_stateNext)
      jtagFSM_enumDef_BOOT : jtagFSM_stateNext_string = "BOOT          ";
      jtagFSM_enumDef_testLogicReset : jtagFSM_stateNext_string = "testLogicReset";
      jtagFSM_enumDef_runTestIdle : jtagFSM_stateNext_string = "runTestIdle   ";
      jtagFSM_enumDef_selectDRScan : jtagFSM_stateNext_string = "selectDRScan  ";
      jtagFSM_enumDef_captureDR : jtagFSM_stateNext_string = "captureDR     ";
      jtagFSM_enumDef_shiftDR : jtagFSM_stateNext_string = "shiftDR       ";
      jtagFSM_enumDef_exit1DR : jtagFSM_stateNext_string = "exit1DR       ";
      jtagFSM_enumDef_pauseDR : jtagFSM_stateNext_string = "pauseDR       ";
      jtagFSM_enumDef_exit2DR : jtagFSM_stateNext_string = "exit2DR       ";
      jtagFSM_enumDef_updateDR : jtagFSM_stateNext_string = "updateDR      ";
      jtagFSM_enumDef_selectIRScan : jtagFSM_stateNext_string = "selectIRScan  ";
      jtagFSM_enumDef_captureIR : jtagFSM_stateNext_string = "captureIR     ";
      jtagFSM_enumDef_shiftIR : jtagFSM_stateNext_string = "shiftIR       ";
      jtagFSM_enumDef_exit1IR : jtagFSM_stateNext_string = "exit1IR       ";
      jtagFSM_enumDef_pauseIR : jtagFSM_stateNext_string = "pauseIR       ";
      jtagFSM_enumDef_exit2IR : jtagFSM_stateNext_string = "exit2IR       ";
      jtagFSM_enumDef_updateIR : jtagFSM_stateNext_string = "updateIR      ";
      default : jtagFSM_stateNext_string = "??????????????";
    endcase
  end
  `endif

  assign _zz_when_J1Jtag_l203 = 5'h1f;
  assign _zz_when_J1Jtag_l203_1 = 5'h05;
  assign _zz_when_J1Jtag_l203_2 = 5'h09;
  assign _zz_when_J1Jtag_l203_3 = 5'h0d;
  assign _zz_when_J1Jtag_l203_4 = 5'h11;
  assign _zz_when_J1Jtag_l203_5 = 5'h15;
  assign _zz_dataHoldRegs_1 = 5'h19;
  assign jtagFSM_wantExit = 1'b0;
  always @(*) begin
    jtagFSM_wantStart = 1'b0;
    jtagDataValid = 1'b0;
    tdo = dataShiftRegs_0[0];
    if(when_J1Jtag_l203) begin
      tdo = dataShiftRegs_0[0];
    end
    if(when_J1Jtag_l203_1) begin
      tdo = dataShiftRegs_1[0];
    end
    if(when_J1Jtag_l203_2) begin
      tdo = dataShiftRegs_2[0];
    end
    if(when_J1Jtag_l203_3) begin
      tdo = dataShiftRegs_3[0];
    end
    if(when_J1Jtag_l203_4) begin
      tdo = dataShiftRegs_4[0];
    end
    if(when_J1Jtag_l203_5) begin
      tdo = dataShiftRegs_5[0];
    end
    if(when_J1Jtag_l203_6) begin
      tdo = dataShiftRegs_6[0];
    end
    jtagFSM_stateNext = jtagFSM_stateReg;
    case(jtagFSM_stateReg)
      jtagFSM_enumDef_testLogicReset : begin
        jtagDataValid = 1'b1;
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_testLogicReset;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_runTestIdle;
        end
      end
      jtagFSM_enumDef_runTestIdle : begin
        jtagDataValid = 1'b1;
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_selectDRScan;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_runTestIdle;
        end
      end
      jtagFSM_enumDef_selectDRScan : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_selectIRScan;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_captureDR;
        end
      end
      jtagFSM_enumDef_captureDR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_exit1DR;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_shiftDR;
        end
      end
      jtagFSM_enumDef_shiftDR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_exit1DR;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_shiftDR;
        end
      end
      jtagFSM_enumDef_exit1DR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_updateDR;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_pauseDR;
        end
      end
      jtagFSM_enumDef_pauseDR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_exit2DR;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_pauseDR;
        end
      end
      jtagFSM_enumDef_exit2DR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_updateDR;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_shiftDR;
        end
      end
      jtagFSM_enumDef_updateDR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_selectDRScan;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_runTestIdle;
        end
      end
      jtagFSM_enumDef_selectIRScan : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_testLogicReset;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_captureIR;
        end
      end
      jtagFSM_enumDef_captureIR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_exit1IR;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_shiftIR;
        end
      end
      jtagFSM_enumDef_shiftIR : begin
        tdo = instructionShiftReg[0];
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_exit1IR;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_shiftIR;
        end
      end
      jtagFSM_enumDef_exit1IR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_updateIR;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_pauseIR;
        end
      end
      jtagFSM_enumDef_pauseIR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_exit2IR;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_pauseIR;
        end
      end
      jtagFSM_enumDef_exit2IR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_updateIR;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_shiftIR;
        end
      end
      jtagFSM_enumDef_updateIR : begin
        if(tms) begin
          jtagFSM_stateNext = jtagFSM_enumDef_selectDRScan;
        end else begin
          jtagFSM_stateNext = jtagFSM_enumDef_runTestIdle;
        end
      end
      default : begin
        jtagFSM_wantStart = 1'b1;
      end
    endcase
    if(when_StateMachine_l237) begin
      jtagDataValid = 1'b1;
    end
    if(when_StateMachine_l237_1) begin
      jtagDataValid = 1'b1;
    end
    if(jtagFSM_wantStart) begin
      jtagFSM_stateNext = jtagFSM_enumDef_testLogicReset;
    end
    if(jtagFSM_wantKill) begin
      jtagFSM_stateNext = jtagFSM_enumDef_BOOT;
    end
  end

  assign jtagFSM_wantKill = 1'b0;
  assign when_J1Jtag_l203 = (instructionHoldReg == _zz_when_J1Jtag_l203);
  assign when_J1Jtag_l203_1 = (instructionHoldReg == _zz_dataHoldRegs_1);
  assign when_J1Jtag_l203_2 = (instructionHoldReg == _zz_when_J1Jtag_l203_1);
  assign when_J1Jtag_l203_3 = (instructionHoldReg == _zz_when_J1Jtag_l203_2);
  assign when_J1Jtag_l203_4 = (instructionHoldReg == _zz_when_J1Jtag_l203_3);
  assign when_J1Jtag_l203_5 = (instructionHoldReg == _zz_when_J1Jtag_l203_4);
  assign when_J1Jtag_l203_6 = (instructionHoldReg == _zz_when_J1Jtag_l203_5);
  assign jtagDataBundle_jtagStall = dataHoldRegs_2[0];
  assign jtagReset = dataHoldRegs_3[0];
  assign jtagDataBundle_jtagDataValid = jtagDataValid;
  assign jtagDataBundle_jtagCaptureMemory = dataHoldRegs_4[0];
  assign jtagDataBundle_jtagCPUAdr = dataHoldRegs_5;
  assign jtagDataBundle_jtagCPUWord = dataHoldRegs_6;
  assign jtagDataFlow_payload_jtagDataValid = jtagDataBundle_jtagDataValid;
  assign jtagDataFlow_payload_jtagStall = jtagDataBundle_jtagStall;
  assign jtagDataFlow_payload_jtagCaptureMemory = jtagDataBundle_jtagCaptureMemory;
  assign jtagDataFlow_payload_jtagCPUAdr = jtagDataBundle_jtagCPUAdr;
  assign jtagDataFlow_payload_jtagCPUWord = jtagDataBundle_jtagCPUWord;
  assign jtagDataFlow_valid = jtagDataValid;
  assign when_J1Jtag_l259 = (instructionHoldReg == _zz_when_J1Jtag_l203);
  assign when_J1Jtag_l272 = (instructionHoldReg == _zz_dataHoldRegs_1);
  assign when_J1Jtag_l259_1 = (instructionHoldReg == _zz_when_J1Jtag_l203_1);
  assign when_J1Jtag_l259_2 = (instructionHoldReg == _zz_when_J1Jtag_l203_2);
  assign when_J1Jtag_l259_3 = (instructionHoldReg == _zz_when_J1Jtag_l203_3);
  assign when_J1Jtag_l295 = (instructionHoldReg == _zz_when_J1Jtag_l203);
  assign when_J1Jtag_l295_1 = (instructionHoldReg == _zz_dataHoldRegs_1);
  assign when_J1Jtag_l295_2 = (instructionHoldReg == _zz_when_J1Jtag_l203_1);
  assign when_J1Jtag_l295_3 = (instructionHoldReg == _zz_when_J1Jtag_l203_2);
  assign when_J1Jtag_l295_4 = (instructionHoldReg == _zz_when_J1Jtag_l203_3);
  assign when_J1Jtag_l295_5 = (instructionHoldReg == _zz_when_J1Jtag_l203_4);
  assign when_J1Jtag_l295_6 = (instructionHoldReg == _zz_when_J1Jtag_l203_5);
  assign when_J1Jtag_l319 = (instructionHoldReg == _zz_when_J1Jtag_l203);
  assign when_J1Jtag_l319_1 = (instructionHoldReg == _zz_when_J1Jtag_l203_1);
  assign when_J1Jtag_l319_2 = (instructionHoldReg == _zz_when_J1Jtag_l203_2);
  assign when_J1Jtag_l319_3 = (instructionHoldReg == _zz_when_J1Jtag_l203_3);
  assign when_J1Jtag_l319_4 = (instructionHoldReg == _zz_when_J1Jtag_l203_4);
  assign when_J1Jtag_l319_5 = (instructionHoldReg == _zz_when_J1Jtag_l203_5);
  assign when_StateMachine_l237 = ((jtagFSM_stateReg == jtagFSM_enumDef_updateDR) && (! (jtagFSM_stateNext == jtagFSM_enumDef_updateDR)));
  assign when_StateMachine_l237_1 = ((jtagFSM_stateReg == jtagFSM_enumDef_updateIR) && (! (jtagFSM_stateNext == jtagFSM_enumDef_updateIR)));
  always @(posedge tck or posedge reset) begin
    if(reset) begin
      dataHoldRegs_2 <= 1'b0;
      dataHoldRegs_3 <= 1'b0;
      dataHoldRegs_4 <= 1'b0;
      jtagFSM_stateReg <= jtagFSM_enumDef_BOOT;
    end else begin
      jtagFSM_stateReg <= jtagFSM_stateNext;
      case(jtagFSM_stateReg)
        jtagFSM_enumDef_testLogicReset : begin
        end
        jtagFSM_enumDef_runTestIdle : begin
        end
        jtagFSM_enumDef_selectDRScan : begin
        end
        jtagFSM_enumDef_captureDR : begin
        end
        jtagFSM_enumDef_shiftDR : begin
        end
        jtagFSM_enumDef_exit1DR : begin
        end
        jtagFSM_enumDef_pauseDR : begin
        end
        jtagFSM_enumDef_exit2DR : begin
        end
        jtagFSM_enumDef_updateDR : begin
          if(when_J1Jtag_l319_1) begin
            dataHoldRegs_2 <= dataShiftRegs_2;
          end
          if(when_J1Jtag_l319_2) begin
            dataHoldRegs_3 <= dataShiftRegs_3;
          end
          if(when_J1Jtag_l319_3) begin
            dataHoldRegs_4 <= dataShiftRegs_4;
          end
        end
        jtagFSM_enumDef_selectIRScan : begin
        end
        jtagFSM_enumDef_captureIR : begin
        end
        jtagFSM_enumDef_shiftIR : begin
        end
        jtagFSM_enumDef_exit1IR : begin
        end
        jtagFSM_enumDef_pauseIR : begin
        end
        jtagFSM_enumDef_exit2IR : begin
        end
        jtagFSM_enumDef_updateIR : begin
        end
        default : begin
        end
      endcase
    end
  end

  always @(posedge tck) begin
    case(jtagFSM_stateReg)
      jtagFSM_enumDef_testLogicReset : begin
        instructionHoldReg <= 5'h19;
        dataHoldRegs_1 <= {27'd0, _zz_dataHoldRegs_1};
      end
      jtagFSM_enumDef_runTestIdle : begin
      end
      jtagFSM_enumDef_selectDRScan : begin
      end
      jtagFSM_enumDef_captureDR : begin
        if(when_J1Jtag_l259) begin
          dataShiftRegs_0 <= dataHoldRegs_0;
        end
        if(when_J1Jtag_l272) begin
          dataShiftRegs_1 <= 32'h01234567;
        end
        if(when_J1Jtag_l259_1) begin
          dataShiftRegs_2 <= dataHoldRegs_2;
        end
        if(when_J1Jtag_l259_2) begin
          dataShiftRegs_3 <= dataHoldRegs_3;
        end
        if(when_J1Jtag_l259_3) begin
          dataShiftRegs_4 <= dataHoldRegs_4;
        end
      end
      jtagFSM_enumDef_shiftDR : begin
        if(when_J1Jtag_l295) begin
          dataShiftRegs_0 <= ({tdi,dataShiftRegs_0} >>> 1'd1);
        end
        if(when_J1Jtag_l295_1) begin
          dataShiftRegs_1 <= ({tdi,dataShiftRegs_1} >>> 1'd1);
        end
        if(when_J1Jtag_l295_2) begin
          dataShiftRegs_2 <= ({tdi,dataShiftRegs_2} >>> 1'd1);
        end
        if(when_J1Jtag_l295_3) begin
          dataShiftRegs_3 <= ({tdi,dataShiftRegs_3} >>> 1'd1);
        end
        if(when_J1Jtag_l295_4) begin
          dataShiftRegs_4 <= ({tdi,dataShiftRegs_4} >>> 1'd1);
        end
        if(when_J1Jtag_l295_5) begin
          dataShiftRegs_5 <= ({tdi,dataShiftRegs_5} >>> 1'd1);
        end
        if(when_J1Jtag_l295_6) begin
          dataShiftRegs_6 <= ({tdi,dataShiftRegs_6} >>> 1'd1);
        end
      end
      jtagFSM_enumDef_exit1DR : begin
      end
      jtagFSM_enumDef_pauseDR : begin
      end
      jtagFSM_enumDef_exit2DR : begin
      end
      jtagFSM_enumDef_updateDR : begin
        if(when_J1Jtag_l319) begin
          dataHoldRegs_0 <= dataShiftRegs_0;
        end
        if(when_J1Jtag_l319_4) begin
          dataHoldRegs_5 <= dataShiftRegs_5;
        end
        if(when_J1Jtag_l319_5) begin
          dataHoldRegs_6 <= dataShiftRegs_6;
        end
      end
      jtagFSM_enumDef_selectIRScan : begin
      end
      jtagFSM_enumDef_captureIR : begin
        instructionShiftReg <= 5'h1f;
        instructionShiftReg[0] <= 1'b1;
        instructionShiftReg[1] <= 1'b0;
      end
      jtagFSM_enumDef_shiftIR : begin
        instructionShiftReg <= ({tdi,instructionShiftReg} >>> 1'd1);
      end
      jtagFSM_enumDef_exit1IR : begin
      end
      jtagFSM_enumDef_pauseIR : begin
      end
      jtagFSM_enumDef_exit2IR : begin
      end
      jtagFSM_enumDef_updateIR : begin
        instructionHoldReg <= instructionShiftReg;
      end
      default : begin
      end
    endcase
  end


endmodule

module BufferCC_1 (
  input  wire [3:0]    io_dataIn,
  output wire [3:0]    io_dataOut,
  input  wire          core_clk,
  input  wire          core_reset
);

  (* async_reg = "true" *) reg        [3:0]    buffers_0;
  (* async_reg = "true" *) reg        [3:0]    buffers_1;
  (* async_reg = "true" *) reg        [3:0]    buffers_2;

  assign io_dataOut = buffers_2;
  always @(posedge core_clk) begin
    if(core_reset) begin
      buffers_0 <= 4'b0000;
      buffers_1 <= 4'b0000;
      buffers_2 <= 4'b0000;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
      buffers_2 <= buffers_1;
    end
  end


endmodule

module UartCtrlRx (
  input  wire [2:0]    io_configFrame_dataLength,
  input  wire [0:0]    io_configFrame_stop,
  input  wire [1:0]    io_configFrame_parity,
  input  wire          io_samplingTick,
  output wire          io_read_valid,
  input  wire          io_read_ready,
  output wire [7:0]    io_read_payload,
  input  wire          io_rxd,
  output wire          io_rts,
  output reg           io_error,
  output wire          io_break,
  input  wire          core_clk,
  input  wire          core_reset
);
  localparam UartStopType_ONE = 1'd0;
  localparam UartStopType_TWO = 1'd1;
  localparam UartParityType_NONE = 2'd0;
  localparam UartParityType_EVEN = 2'd1;
  localparam UartParityType_ODD = 2'd2;
  localparam UartCtrlRxState_IDLE = 3'd0;
  localparam UartCtrlRxState_START = 3'd1;
  localparam UartCtrlRxState_DATA = 3'd2;
  localparam UartCtrlRxState_PARITY = 3'd3;
  localparam UartCtrlRxState_STOP = 3'd4;

  wire                io_rxd_buffercc_io_dataOut;
  wire                _zz_sampler_value;
  wire                _zz_sampler_value_1;
  wire                _zz_sampler_value_2;
  wire                _zz_sampler_value_3;
  wire                _zz_sampler_value_4;
  wire                _zz_sampler_value_5;
  wire                _zz_sampler_value_6;
  wire       [2:0]    _zz_when_UartCtrlRx_l139;
  wire       [0:0]    _zz_when_UartCtrlRx_l139_1;
  reg                 _zz_io_rts;
  wire                sampler_synchroniser;
  wire                sampler_samples_0;
  reg                 sampler_samples_1;
  reg                 sampler_samples_2;
  reg                 sampler_samples_3;
  reg                 sampler_samples_4;
  reg                 sampler_value;
  reg                 sampler_tick;
  reg        [2:0]    bitTimer_counter;
  reg                 bitTimer_tick;
  wire                when_UartCtrlRx_l43;
  reg        [2:0]    bitCounter_value;
  reg        [6:0]    break_counter;
  wire                break_valid;
  wire                when_UartCtrlRx_l69;
  reg        [2:0]    stateMachine_state;
  reg                 stateMachine_parity;
  reg        [7:0]    stateMachine_shifter;
  reg                 stateMachine_validReg;
  wire                when_UartCtrlRx_l93;
  wire                when_UartCtrlRx_l103;
  wire                when_UartCtrlRx_l111;
  wire                when_UartCtrlRx_l113;
  wire                when_UartCtrlRx_l125;
  wire                when_UartCtrlRx_l136;
  wire                when_UartCtrlRx_l139;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  `endif


  assign _zz_when_UartCtrlRx_l139_1 = ((io_configFrame_stop == UartStopType_ONE) ? 1'b0 : 1'b1);
  assign _zz_when_UartCtrlRx_l139 = {2'd0, _zz_when_UartCtrlRx_l139_1};
  assign _zz_sampler_value = ((((1'b0 || ((_zz_sampler_value_1 && sampler_samples_1) && sampler_samples_2)) || (((_zz_sampler_value_2 && sampler_samples_0) && sampler_samples_1) && sampler_samples_3)) || (((1'b1 && sampler_samples_0) && sampler_samples_2) && sampler_samples_3)) || (((1'b1 && sampler_samples_1) && sampler_samples_2) && sampler_samples_3));
  assign _zz_sampler_value_3 = (((1'b1 && sampler_samples_0) && sampler_samples_1) && sampler_samples_4);
  assign _zz_sampler_value_4 = ((1'b1 && sampler_samples_0) && sampler_samples_2);
  assign _zz_sampler_value_5 = (1'b1 && sampler_samples_1);
  assign _zz_sampler_value_6 = 1'b1;
  assign _zz_sampler_value_1 = (1'b1 && sampler_samples_0);
  assign _zz_sampler_value_2 = 1'b1;
  BufferCC_4 io_rxd_buffercc (
    .io_dataIn  (io_rxd                    ), //i
    .io_dataOut (io_rxd_buffercc_io_dataOut), //o
    .core_clk   (core_clk                  ), //i
    .core_reset (core_reset                )  //i
  );
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      UartStopType_ONE : io_configFrame_stop_string = "ONE";
      UartStopType_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      UartParityType_NONE : io_configFrame_parity_string = "NONE";
      UartParityType_EVEN : io_configFrame_parity_string = "EVEN";
      UartParityType_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      UartCtrlRxState_IDLE : stateMachine_state_string = "IDLE  ";
      UartCtrlRxState_START : stateMachine_state_string = "START ";
      UartCtrlRxState_DATA : stateMachine_state_string = "DATA  ";
      UartCtrlRxState_PARITY : stateMachine_state_string = "PARITY";
      UartCtrlRxState_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  `endif

  always @(*) begin
    io_error = 1'b0;
    case(stateMachine_state)
      UartCtrlRxState_IDLE : begin
      end
      UartCtrlRxState_START : begin
      end
      UartCtrlRxState_DATA : begin
      end
      UartCtrlRxState_PARITY : begin
        if(bitTimer_tick) begin
          if(!when_UartCtrlRx_l125) begin
            io_error = 1'b1;
          end
        end
      end
      default : begin
        if(bitTimer_tick) begin
          if(when_UartCtrlRx_l136) begin
            io_error = 1'b1;
          end
        end
      end
    endcase
  end

  assign io_rts = _zz_io_rts;
  assign sampler_synchroniser = io_rxd_buffercc_io_dataOut;
  assign sampler_samples_0 = sampler_synchroniser;
  always @(*) begin
    bitTimer_tick = 1'b0;
    if(sampler_tick) begin
      if(when_UartCtrlRx_l43) begin
        bitTimer_tick = 1'b1;
      end
    end
  end

  assign when_UartCtrlRx_l43 = (bitTimer_counter == 3'b000);
  assign break_valid = (break_counter == 7'h68);
  assign when_UartCtrlRx_l69 = (io_samplingTick && (! break_valid));
  assign io_break = break_valid;
  assign io_read_valid = stateMachine_validReg;
  assign when_UartCtrlRx_l93 = ((sampler_tick && (! sampler_value)) && (! break_valid));
  assign when_UartCtrlRx_l103 = (sampler_value == 1'b1);
  assign when_UartCtrlRx_l111 = (bitCounter_value == io_configFrame_dataLength);
  assign when_UartCtrlRx_l113 = (io_configFrame_parity == UartParityType_NONE);
  assign when_UartCtrlRx_l125 = (stateMachine_parity == sampler_value);
  assign when_UartCtrlRx_l136 = (! sampler_value);
  assign when_UartCtrlRx_l139 = (bitCounter_value == _zz_when_UartCtrlRx_l139);
  assign io_read_payload = stateMachine_shifter;
  always @(posedge core_clk) begin
    if(core_reset) begin
      _zz_io_rts <= 1'b0;
      sampler_samples_1 <= 1'b1;
      sampler_samples_2 <= 1'b1;
      sampler_samples_3 <= 1'b1;
      sampler_samples_4 <= 1'b1;
      sampler_value <= 1'b1;
      sampler_tick <= 1'b0;
      break_counter <= 7'h00;
      stateMachine_state <= UartCtrlRxState_IDLE;
      stateMachine_validReg <= 1'b0;
    end else begin
      _zz_io_rts <= (! io_read_ready);
      if(io_samplingTick) begin
        sampler_samples_1 <= sampler_samples_0;
      end
      if(io_samplingTick) begin
        sampler_samples_2 <= sampler_samples_1;
      end
      if(io_samplingTick) begin
        sampler_samples_3 <= sampler_samples_2;
      end
      if(io_samplingTick) begin
        sampler_samples_4 <= sampler_samples_3;
      end
      sampler_value <= ((((((_zz_sampler_value || _zz_sampler_value_3) || (_zz_sampler_value_4 && sampler_samples_4)) || ((_zz_sampler_value_5 && sampler_samples_2) && sampler_samples_4)) || (((_zz_sampler_value_6 && sampler_samples_0) && sampler_samples_3) && sampler_samples_4)) || (((1'b1 && sampler_samples_1) && sampler_samples_3) && sampler_samples_4)) || (((1'b1 && sampler_samples_2) && sampler_samples_3) && sampler_samples_4));
      sampler_tick <= io_samplingTick;
      if(sampler_value) begin
        break_counter <= 7'h00;
      end else begin
        if(when_UartCtrlRx_l69) begin
          break_counter <= (break_counter + 7'h01);
        end
      end
      stateMachine_validReg <= 1'b0;
      case(stateMachine_state)
        UartCtrlRxState_IDLE : begin
          if(when_UartCtrlRx_l93) begin
            stateMachine_state <= UartCtrlRxState_START;
          end
        end
        UartCtrlRxState_START : begin
          if(bitTimer_tick) begin
            stateMachine_state <= UartCtrlRxState_DATA;
            if(when_UartCtrlRx_l103) begin
              stateMachine_state <= UartCtrlRxState_IDLE;
            end
          end
        end
        UartCtrlRxState_DATA : begin
          if(bitTimer_tick) begin
            if(when_UartCtrlRx_l111) begin
              if(when_UartCtrlRx_l113) begin
                stateMachine_state <= UartCtrlRxState_STOP;
                stateMachine_validReg <= 1'b1;
              end else begin
                stateMachine_state <= UartCtrlRxState_PARITY;
              end
            end
          end
        end
        UartCtrlRxState_PARITY : begin
          if(bitTimer_tick) begin
            if(when_UartCtrlRx_l125) begin
              stateMachine_state <= UartCtrlRxState_STOP;
              stateMachine_validReg <= 1'b1;
            end else begin
              stateMachine_state <= UartCtrlRxState_IDLE;
            end
          end
        end
        default : begin
          if(bitTimer_tick) begin
            if(when_UartCtrlRx_l136) begin
              stateMachine_state <= UartCtrlRxState_IDLE;
            end else begin
              if(when_UartCtrlRx_l139) begin
                stateMachine_state <= UartCtrlRxState_IDLE;
              end
            end
          end
        end
      endcase
    end
  end

  always @(posedge core_clk) begin
    if(sampler_tick) begin
      bitTimer_counter <= (bitTimer_counter - 3'b001);
    end
    if(bitTimer_tick) begin
      bitCounter_value <= (bitCounter_value + 3'b001);
    end
    if(bitTimer_tick) begin
      stateMachine_parity <= (stateMachine_parity ^ sampler_value);
    end
    case(stateMachine_state)
      UartCtrlRxState_IDLE : begin
        if(when_UartCtrlRx_l93) begin
          bitTimer_counter <= 3'b010;
        end
      end
      UartCtrlRxState_START : begin
        if(bitTimer_tick) begin
          bitCounter_value <= 3'b000;
          stateMachine_parity <= (io_configFrame_parity == UartParityType_ODD);
        end
      end
      UartCtrlRxState_DATA : begin
        if(bitTimer_tick) begin
          stateMachine_shifter[bitCounter_value] <= sampler_value;
          if(when_UartCtrlRx_l111) begin
            bitCounter_value <= 3'b000;
          end
        end
      end
      UartCtrlRxState_PARITY : begin
        if(bitTimer_tick) begin
          bitCounter_value <= 3'b000;
        end
      end
      default : begin
      end
    endcase
  end


endmodule

module UartCtrlTx (
  input  wire [2:0]    io_configFrame_dataLength,
  input  wire [0:0]    io_configFrame_stop,
  input  wire [1:0]    io_configFrame_parity,
  input  wire          io_samplingTick,
  input  wire          io_write_valid,
  output reg           io_write_ready,
  input  wire [7:0]    io_write_payload,
  input  wire          io_cts,
  output wire          io_txd,
  input  wire          io_break,
  input  wire          core_clk,
  input  wire          core_reset
);
  localparam UartStopType_ONE = 1'd0;
  localparam UartStopType_TWO = 1'd1;
  localparam UartParityType_NONE = 2'd0;
  localparam UartParityType_EVEN = 2'd1;
  localparam UartParityType_ODD = 2'd2;
  localparam UartCtrlTxState_IDLE = 3'd0;
  localparam UartCtrlTxState_START = 3'd1;
  localparam UartCtrlTxState_DATA = 3'd2;
  localparam UartCtrlTxState_PARITY = 3'd3;
  localparam UartCtrlTxState_STOP = 3'd4;

  wire       [2:0]    _zz_clockDivider_counter_valueNext;
  wire       [0:0]    _zz_clockDivider_counter_valueNext_1;
  wire       [2:0]    _zz_when_UartCtrlTx_l93;
  wire       [0:0]    _zz_when_UartCtrlTx_l93_1;
  reg                 clockDivider_counter_willIncrement;
  wire                clockDivider_counter_willClear;
  reg        [2:0]    clockDivider_counter_valueNext;
  reg        [2:0]    clockDivider_counter_value;
  wire                clockDivider_counter_willOverflowIfInc;
  wire                clockDivider_counter_willOverflow;
  reg        [2:0]    tickCounter_value;
  reg        [2:0]    stateMachine_state;
  reg                 stateMachine_parity;
  reg                 stateMachine_txd;
  wire                when_UartCtrlTx_l58;
  wire                when_UartCtrlTx_l73;
  wire                when_UartCtrlTx_l76;
  wire                when_UartCtrlTx_l93;
  wire       [2:0]    _zz_stateMachine_state;
  reg                 _zz_io_txd;
  `ifndef SYNTHESIS
  reg [23:0] io_configFrame_stop_string;
  reg [31:0] io_configFrame_parity_string;
  reg [47:0] stateMachine_state_string;
  reg [47:0] _zz_stateMachine_state_string;
  `endif


  assign _zz_clockDivider_counter_valueNext_1 = clockDivider_counter_willIncrement;
  assign _zz_clockDivider_counter_valueNext = {2'd0, _zz_clockDivider_counter_valueNext_1};
  assign _zz_when_UartCtrlTx_l93_1 = ((io_configFrame_stop == UartStopType_ONE) ? 1'b0 : 1'b1);
  assign _zz_when_UartCtrlTx_l93 = {2'd0, _zz_when_UartCtrlTx_l93_1};
  `ifndef SYNTHESIS
  always @(*) begin
    case(io_configFrame_stop)
      UartStopType_ONE : io_configFrame_stop_string = "ONE";
      UartStopType_TWO : io_configFrame_stop_string = "TWO";
      default : io_configFrame_stop_string = "???";
    endcase
  end
  always @(*) begin
    case(io_configFrame_parity)
      UartParityType_NONE : io_configFrame_parity_string = "NONE";
      UartParityType_EVEN : io_configFrame_parity_string = "EVEN";
      UartParityType_ODD : io_configFrame_parity_string = "ODD ";
      default : io_configFrame_parity_string = "????";
    endcase
  end
  always @(*) begin
    case(stateMachine_state)
      UartCtrlTxState_IDLE : stateMachine_state_string = "IDLE  ";
      UartCtrlTxState_START : stateMachine_state_string = "START ";
      UartCtrlTxState_DATA : stateMachine_state_string = "DATA  ";
      UartCtrlTxState_PARITY : stateMachine_state_string = "PARITY";
      UartCtrlTxState_STOP : stateMachine_state_string = "STOP  ";
      default : stateMachine_state_string = "??????";
    endcase
  end
  always @(*) begin
    case(_zz_stateMachine_state)
      UartCtrlTxState_IDLE : _zz_stateMachine_state_string = "IDLE  ";
      UartCtrlTxState_START : _zz_stateMachine_state_string = "START ";
      UartCtrlTxState_DATA : _zz_stateMachine_state_string = "DATA  ";
      UartCtrlTxState_PARITY : _zz_stateMachine_state_string = "PARITY";
      UartCtrlTxState_STOP : _zz_stateMachine_state_string = "STOP  ";
      default : _zz_stateMachine_state_string = "??????";
    endcase
  end
  `endif

  always @(*) begin
    clockDivider_counter_willIncrement = 1'b0;
    if(io_samplingTick) begin
      clockDivider_counter_willIncrement = 1'b1;
    end
  end

  assign clockDivider_counter_willClear = 1'b0;
  assign clockDivider_counter_willOverflowIfInc = (clockDivider_counter_value == 3'b111);
  assign clockDivider_counter_willOverflow = (clockDivider_counter_willOverflowIfInc && clockDivider_counter_willIncrement);
  always @(*) begin
    clockDivider_counter_valueNext = (clockDivider_counter_value + _zz_clockDivider_counter_valueNext);
    if(clockDivider_counter_willClear) begin
      clockDivider_counter_valueNext = 3'b000;
    end
  end

  always @(*) begin
    stateMachine_txd = 1'b1;
    io_write_ready = io_break;
    case(stateMachine_state)
      UartCtrlTxState_IDLE : begin
      end
      UartCtrlTxState_START : begin
        stateMachine_txd = 1'b0;
      end
      UartCtrlTxState_DATA : begin
        stateMachine_txd = io_write_payload[tickCounter_value];
        if(clockDivider_counter_willOverflow) begin
          if(when_UartCtrlTx_l73) begin
            io_write_ready = 1'b1;
          end
        end
      end
      UartCtrlTxState_PARITY : begin
        stateMachine_txd = stateMachine_parity;
      end
      default : begin
      end
    endcase
  end

  assign when_UartCtrlTx_l58 = ((io_write_valid && (! io_cts)) && clockDivider_counter_willOverflow);
  assign when_UartCtrlTx_l73 = (tickCounter_value == io_configFrame_dataLength);
  assign when_UartCtrlTx_l76 = (io_configFrame_parity == UartParityType_NONE);
  assign when_UartCtrlTx_l93 = (tickCounter_value == _zz_when_UartCtrlTx_l93);
  assign _zz_stateMachine_state = (io_write_valid ? UartCtrlTxState_START : UartCtrlTxState_IDLE);
  assign io_txd = _zz_io_txd;
  always @(posedge core_clk) begin
    if(core_reset) begin
      clockDivider_counter_value <= 3'b000;
      stateMachine_state <= UartCtrlTxState_IDLE;
      _zz_io_txd <= 1'b1;
    end else begin
      clockDivider_counter_value <= clockDivider_counter_valueNext;
      case(stateMachine_state)
        UartCtrlTxState_IDLE : begin
          if(when_UartCtrlTx_l58) begin
            stateMachine_state <= UartCtrlTxState_START;
          end
        end
        UartCtrlTxState_START : begin
          if(clockDivider_counter_willOverflow) begin
            stateMachine_state <= UartCtrlTxState_DATA;
          end
        end
        UartCtrlTxState_DATA : begin
          if(clockDivider_counter_willOverflow) begin
            if(when_UartCtrlTx_l73) begin
              if(when_UartCtrlTx_l76) begin
                stateMachine_state <= UartCtrlTxState_STOP;
              end else begin
                stateMachine_state <= UartCtrlTxState_PARITY;
              end
            end
          end
        end
        UartCtrlTxState_PARITY : begin
          if(clockDivider_counter_willOverflow) begin
            stateMachine_state <= UartCtrlTxState_STOP;
          end
        end
        default : begin
          if(clockDivider_counter_willOverflow) begin
            if(when_UartCtrlTx_l93) begin
              stateMachine_state <= _zz_stateMachine_state;
            end
          end
        end
      endcase
      _zz_io_txd <= (stateMachine_txd && (! io_break));
    end
  end

  always @(posedge core_clk) begin
    if(clockDivider_counter_willOverflow) begin
      tickCounter_value <= (tickCounter_value + 3'b001);
    end
    if(clockDivider_counter_willOverflow) begin
      stateMachine_parity <= (stateMachine_parity ^ stateMachine_txd);
    end
    case(stateMachine_state)
      UartCtrlTxState_IDLE : begin
      end
      UartCtrlTxState_START : begin
        if(clockDivider_counter_willOverflow) begin
          stateMachine_parity <= (io_configFrame_parity == UartParityType_ODD);
          tickCounter_value <= 3'b000;
        end
      end
      UartCtrlTxState_DATA : begin
        if(clockDivider_counter_willOverflow) begin
          if(when_UartCtrlTx_l73) begin
            tickCounter_value <= 3'b000;
          end
        end
      end
      UartCtrlTxState_PARITY : begin
        if(clockDivider_counter_willOverflow) begin
          tickCounter_value <= 3'b000;
        end
      end
      default : begin
      end
    endcase
  end


endmodule

module BufferCC_3 (
  input  wire          io_dataIn,
  output wire          io_dataOut,
  input  wire          core_clk,
  input  wire          flowCCByToggle_1_toplevel_reset_syncronized
);

  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @(posedge core_clk) begin
    if(flowCCByToggle_1_toplevel_reset_syncronized) begin
      buffers_0 <= 1'b0;
      buffers_1 <= 1'b0;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule

module BufferCC_2 (
  input  wire          io_dataIn,
  output wire          io_dataOut,
  input  wire          core_clk,
  input  wire          reset
);

  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @(posedge core_clk or posedge reset) begin
    if(reset) begin
      buffers_0 <= 1'b1;
      buffers_1 <= 1'b1;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule

module MainMemory (
  input  wire [7:0]    readDataAdr,
  output wire [15:0]   readData,
  input  wire          writeEnable,
  input  wire [7:0]    writeDataAdr,
  input  wire [15:0]   writeData,
  input  wire          core_clk,
  input  wire          core_reset
);

  reg        [15:0]   _zz_ramList_0_port1;
  reg        [15:0]   _zz_ramList_1_port1;
  wire       [6:0]    _zz_ramList_0_port;
  wire                _zz_ramList_0_port_1;
  wire                _zz_ramList_0_port_2;
  wire                _zz_rPortsVec_0_1;
  wire       [6:0]    _zz_ramList_1_port;
  wire                _zz_ramList_1_port_1;
  wire                _zz_ramList_1_port_2;
  wire                _zz_rPortsVec_1_1;
  reg        [15:0]   _zz_readData_1;
  wire       [6:0]    _zz_rPortsVec_0;
  wire       [15:0]   rPortsVec_0;
  wire       [6:0]    _zz_rPortsVec_1;
  wire       [15:0]   rPortsVec_1;
  reg        [0:0]    _zz_readData;
  reg [15:0] ramList_0 [0:127];
  reg [15:0] ramList_1 [0:127];

  assign _zz_ramList_0_port = writeDataAdr[6 : 0];
  assign _zz_ramList_0_port_1 = (writeEnable && (1'b0 == writeDataAdr[7 : 7]));
  assign _zz_rPortsVec_0_1 = 1'b1;
  assign _zz_ramList_1_port = writeDataAdr[6 : 0];
  assign _zz_ramList_1_port_1 = (writeEnable && (1'b1 == writeDataAdr[7 : 7]));
  assign _zz_rPortsVec_1_1 = 1'b1;
  initial begin
    $readmemb("J1Asic.v_toplevel_coreArea_cpu_mainMem_ramList_0.bin",ramList_0);
  end
  always @(posedge core_clk) begin
    if(_zz_ramList_0_port_1) begin
      ramList_0[_zz_ramList_0_port] <= writeData;
    end
  end

  always @(posedge core_clk) begin
    if(_zz_rPortsVec_0_1) begin
      _zz_ramList_0_port1 <= ramList_0[_zz_rPortsVec_0];
    end
  end

  initial begin
    $readmemb("J1Asic.v_toplevel_coreArea_cpu_mainMem_ramList_1.bin",ramList_1);
  end
  always @(posedge core_clk) begin
    if(_zz_ramList_1_port_1) begin
      ramList_1[_zz_ramList_1_port] <= writeData;
    end
  end

  always @(posedge core_clk) begin
    if(_zz_rPortsVec_1_1) begin
      _zz_ramList_1_port1 <= ramList_1[_zz_rPortsVec_1];
    end
  end

  always @(*) begin
    case(_zz_readData)
      1'b0 : _zz_readData_1 = rPortsVec_0;
      default : _zz_readData_1 = rPortsVec_1;
    endcase
  end

  assign _zz_rPortsVec_0 = readDataAdr[6 : 0];
  assign rPortsVec_0 = _zz_ramList_0_port1;
  assign _zz_rPortsVec_1 = readDataAdr[6 : 0];
  assign rPortsVec_1 = _zz_ramList_1_port1;
  assign readData = _zz_readData_1;
  always @(posedge core_clk) begin
    _zz_readData <= readDataAdr[7 : 7];
  end


endmodule

module J1Core (
  output wire          memWriteMode,
  output wire          ioWriteMode,
  output wire          ioReadMode,
  output wire [15:0]   extAdr,
  output wire [15:0]   extToWrite,
  input  wire [15:0]   toRead,
  input  wire          stall,
  input  wire          irq,
  input  wire [7:0]    intVec,
  output wire [7:0]    nextInstrAdr,
  input  wire [15:0]   memInstr,
  input  wire          clrActive,
  input  wire          core_clk
);

  wire       [15:0]   _zz__zz_1_port1;
  wire       [15:0]   _zz__zz_4_port1;
  wire       [12:0]   _zz_instr;
  wire                _zz__zz_1_port;
  wire       [15:0]   _zz_rtosN;
  wire       [9:0]    _zz_rtosN_1;
  wire                _zz__zz_4_port;
  wire       [16:0]   _zz__zz_aluResult;
  wire       [16:0]   _zz__zz_aluResult_1;
  wire       [16:0]   _zz__zz_aluResult_2;
  wire       [16:0]   _zz__zz_aluResult_3;
  wire       [15:0]   _zz_aluResult_1;
  wire       [15:0]   _zz_aluResult_2;
  wire       [15:0]   _zz_aluResult_3;
  wire       [14:0]   _zz__zz_dtosN;
  wire       [1:0]    _zz__zz_dStack_stackPtrN;
  wire       [2:0]    _zz_dStack_stackPtrN_1;
  wire       [2:0]    _zz_dStack_stackPtrN_2;
  wire       [1:0]    _zz__zz_rStack_stackPtrN;
  wire       [2:0]    _zz_rStack_stackPtrN_1;
  wire       [2:0]    _zz_rStack_stackPtrN_2;
  reg        [8:0]    pc;
  wire       [8:0]    _zz_returnPC;
  wire       [8:0]    pcN;
  wire                when_J1PC_l20;
  wire       [8:0]    returnPC;
  wire       [1:0]    stateSelect;
  reg        [15:0]   instr;
  reg                 dStack_stackWriteEnable;
  wire       [2:0]    dStack_stackPtrN;
  reg        [2:0]    dStack_stackPtr;
  wire       [15:0]   dtosN;
  wire                when_J1DStack_l18;
  reg        [15:0]   dtos;
  wire       [15:0]   dnos;
  wire       [15:0]   rtosN;
  reg                 rStack_stackWriteEnable;
  wire       [2:0]    rStack_stackPtrN;
  reg        [2:0]    rStack_stackPtr;
  wire                when_J1RStack_l16;
  wire       [15:0]   rtos;
  wire       [16:0]   _zz_aluResult;
  wire       [3:0]    switch_Misc_l232;
  reg        [15:0]   aluResult;
  reg        [15:0]   _zz_dtosN;
  wire       [3:0]    switch_J1Decoder_l18;
  wire                funcTtoN;
  wire                funcTtoR;
  wire                funcWriteMem;
  wire                funcWriteIO;
  wire                funcReadIO;
  wire                isALU;
  reg        [2:0]    _zz_dStack_stackPtrN;
  wire       [3:0]    switch_J1DStack_l41;
  reg        [2:0]    _zz_rStack_stackPtrN;
  wire       [3:0]    switch_J1RStack_l36;
  reg        [8:0]    _zz_pcN;
  wire       [7:0]    switch_J1PC_l42;
  (* ram_style = "distributed" *) reg [15:0] _zz_1 [0:7];
  (* ram_style = "distributed" *) reg [15:0] _zz_4 [0:7];

  assign _zz_instr = {5'd0, intVec};
  assign _zz_rtosN_1 = {returnPC,1'b0};
  assign _zz_rtosN = {6'd0, _zz_rtosN_1};
  assign _zz__zz_aluResult = _zz__zz_aluResult_1;
  assign _zz__zz_aluResult_1 = {1'd0, dnos};
  assign _zz__zz_aluResult_2 = _zz__zz_aluResult_3;
  assign _zz__zz_aluResult_3 = {1'd0, dtos};
  assign _zz_aluResult_1 = (dtos + dnos);
  assign _zz_aluResult_2 = _zz_aluResult[15:0];
  assign _zz_aluResult_3 = {13'd0, dStack_stackPtr};
  assign _zz__zz_dtosN = instr[14 : 0];
  assign _zz__zz_dStack_stackPtrN = instr[1 : 0];
  assign _zz_dStack_stackPtrN_1 = ($signed(_zz_dStack_stackPtrN_2) + $signed(_zz_dStack_stackPtrN));
  assign _zz_dStack_stackPtrN_2 = dStack_stackPtr;
  assign _zz__zz_rStack_stackPtrN = instr[3 : 2];
  assign _zz_rStack_stackPtrN_1 = ($signed(_zz_rStack_stackPtrN_2) + $signed(_zz_rStack_stackPtrN));
  assign _zz_rStack_stackPtrN_2 = rStack_stackPtr;
  assign _zz__zz_1_port = (dStack_stackWriteEnable && (! stall));
  assign _zz__zz_4_port = (rStack_stackWriteEnable && (! stall));
  always @(posedge core_clk) begin
    if(_zz__zz_1_port) begin
      _zz_1[dStack_stackPtrN] <= dtos;
    end
  end

  assign _zz__zz_1_port1 = _zz_1[dStack_stackPtr];
  always @(posedge core_clk) begin
    if(_zz__zz_4_port) begin
      _zz_4[rStack_stackPtrN] <= rtosN;
    end
  end

  assign _zz__zz_4_port1 = _zz_4[rStack_stackPtr];
  assign when_J1PC_l20 = (! clrActive);
  assign _zz_returnPC = (pc + 9'h001);
  assign returnPC = (irq ? pc : _zz_returnPC);
  assign stateSelect = {stall,irq};
  always @(*) begin
    case(stateSelect)
      2'b00 : begin
        instr = memInstr;
      end
      2'b01 : begin
        instr = {3'b010,_zz_instr};
      end
      2'b10 : begin
        instr = 16'h6000;
      end
      default : begin
        instr = 16'h6000;
      end
    endcase
  end

  assign when_J1DStack_l18 = (! stall);
  assign dnos = _zz__zz_1_port1;
  assign rtosN = ((! instr[13]) ? _zz_rtosN : dtos);
  assign when_J1RStack_l16 = (! stall);
  assign rtos = _zz__zz_4_port1;
  assign _zz_aluResult = ($signed(_zz__zz_aluResult) - $signed(_zz__zz_aluResult_2));
  assign switch_Misc_l232 = instr[11 : 8];
  always @(*) begin
    case(switch_Misc_l232)
      4'b0000 : begin
        aluResult = dtos;
      end
      4'b0001 : begin
        aluResult = dnos;
      end
      4'b0010 : begin
        aluResult = _zz_aluResult_1;
      end
      4'b1100 : begin
        aluResult = _zz_aluResult_2;
      end
      4'b0011 : begin
        aluResult = (dtos & dnos);
      end
      4'b0100 : begin
        aluResult = (dtos | dnos);
      end
      4'b0101 : begin
        aluResult = (dtos ^ dnos);
      end
      4'b0110 : begin
        aluResult = (~ dtos);
      end
      4'b1001 : begin
        aluResult = {dtos[15],dtos[15 : 1]};
      end
      4'b1010 : begin
        aluResult = {dtos[14 : 0],1'b0};
      end
      4'b1011 : begin
        aluResult = rtos;
      end
      4'b0111 : begin
        aluResult = (($signed(_zz_aluResult) == $signed(17'h00000)) ? 16'hffff : 16'h0000);
      end
      4'b1000 : begin
        aluResult = (((dtos[15] ^ dnos[15]) ? dnos[15] : _zz_aluResult[16]) ? 16'hffff : 16'h0000);
      end
      4'b1111 : begin
        aluResult = (_zz_aluResult[16] ? 16'hffff : 16'h0000);
      end
      4'b1101 : begin
        aluResult = toRead;
      end
      default : begin
        aluResult = _zz_aluResult_3;
      end
    endcase
  end

  assign switch_J1Decoder_l18 = {pc[8],instr[15 : 13]};
  always @(*) begin
    casez(switch_J1Decoder_l18)
      4'b1??? : begin
        _zz_dtosN = instr;
      end
      4'b01?? : begin
        _zz_dtosN = {1'd0, _zz__zz_dtosN};
      end
      4'b0000, 4'b0010 : begin
        _zz_dtosN = dtos;
      end
      4'b0001 : begin
        _zz_dtosN = dnos;
      end
      4'b0011 : begin
        _zz_dtosN = aluResult;
      end
      default : begin
        _zz_dtosN = 16'hffff;
      end
    endcase
  end

  assign dtosN = _zz_dtosN;
  assign funcTtoN = (instr[6 : 4] == 3'b001);
  assign funcTtoR = (instr[6 : 4] == 3'b010);
  assign funcWriteMem = (instr[6 : 4] == 3'b011);
  assign funcWriteIO = (instr[6 : 4] == 3'b100);
  assign funcReadIO = (instr[6 : 4] == 3'b101);
  assign isALU = ((! pc[8]) && (instr[15 : 13] == 3'b011));
  assign memWriteMode = (((! clrActive) && isALU) && funcWriteMem);
  assign ioWriteMode = (((! clrActive) && isALU) && funcWriteIO);
  assign ioReadMode = (((! clrActive) && isALU) && funcReadIO);
  assign extAdr = dtosN;
  assign extToWrite = dnos;
  assign switch_J1DStack_l41 = {pc[8],instr[15 : 13]};
  always @(*) begin
    casez(switch_J1DStack_l41)
      4'b1???, 4'b01?? : begin
        dStack_stackWriteEnable = 1'b1;
        _zz_dStack_stackPtrN = 3'b001;
      end
      4'b0001 : begin
        dStack_stackWriteEnable = 1'b0;
        _zz_dStack_stackPtrN = 3'b111;
      end
      4'b0011 : begin
        dStack_stackWriteEnable = funcTtoN;
        _zz_dStack_stackPtrN = {{1{_zz__zz_dStack_stackPtrN[1]}}, _zz__zz_dStack_stackPtrN};
      end
      default : begin
        dStack_stackWriteEnable = 1'b0;
        _zz_dStack_stackPtrN = 3'b000;
      end
    endcase
  end

  assign dStack_stackPtrN = _zz_dStack_stackPtrN_1;
  assign switch_J1RStack_l36 = {pc[8],instr[15 : 13]};
  always @(*) begin
    casez(switch_J1RStack_l36)
      4'b1??? : begin
        rStack_stackWriteEnable = 1'b0;
        _zz_rStack_stackPtrN = 3'b111;
      end
      4'b0010 : begin
        rStack_stackWriteEnable = 1'b1;
        _zz_rStack_stackPtrN = 3'b001;
      end
      4'b0011 : begin
        rStack_stackWriteEnable = funcTtoR;
        _zz_rStack_stackPtrN = {{1{_zz__zz_rStack_stackPtrN[1]}}, _zz__zz_rStack_stackPtrN};
      end
      default : begin
        rStack_stackWriteEnable = 1'b0;
        _zz_rStack_stackPtrN = 3'b000;
      end
    endcase
  end

  assign rStack_stackPtrN = _zz_rStack_stackPtrN_1;
  assign switch_J1PC_l42 = {{{{{stall,clrActive},pc[8]},instr[15 : 13]},instr[7]},(|dtos)};
  always @(*) begin
    casez(switch_J1PC_l42)
      8'b1??????? : begin
        _zz_pcN = pc;
      end
      8'b01?????? : begin
        _zz_pcN = 9'h000;
      end
      8'b000000??, 8'b000010??, 8'b000001?0 : begin
        _zz_pcN = instr[8 : 0];
      end
      8'b001?????, 8'b0000111? : begin
        _zz_pcN = rtos[9 : 1];
      end
      default : begin
        _zz_pcN = _zz_returnPC;
      end
    endcase
  end

  assign pcN = _zz_pcN;
  assign nextInstrAdr = pcN[7 : 0];
  always @(posedge core_clk) begin
    if(clrActive) begin
      pc <= 9'h000;
      dStack_stackPtr <= 3'b000;
      dtos <= 16'h0000;
      rStack_stackPtr <= 3'b000;
    end else begin
      if(when_J1PC_l20) begin
        pc <= pcN;
      end
      if(when_J1DStack_l18) begin
        dStack_stackPtr <= dStack_stackPtrN;
      end
      dtos <= dtosN;
      if(when_J1RStack_l16) begin
        rStack_stackPtr <= rStack_stackPtrN;
      end
    end
  end


endmodule

module BufferCC_4 (
  input  wire          io_dataIn,
  output wire          io_dataOut,
  input  wire          core_clk,
  input  wire          core_reset
);

  (* async_reg = "true" *) reg                 buffers_0;
  (* async_reg = "true" *) reg                 buffers_1;

  assign io_dataOut = buffers_1;
  always @(posedge core_clk) begin
    if(core_reset) begin
      buffers_0 <= 1'b0;
      buffers_1 <= 1'b0;
    end else begin
      buffers_0 <= io_dataIn;
      buffers_1 <= buffers_0;
    end
  end


endmodule

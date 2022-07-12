# Timing constraints
create_clock -period 10.0 -name main_clk [get_ports clk_i]

set_property PACKAGE_PIN H5 [get_ports {gpio_o[0]}]
set_property PACKAGE_PIN J5 [get_ports {gpio_o[1]}]
set_property PACKAGE_PIN T9 [get_ports {gpio_o[2]}]
set_property PACKAGE_PIN T10 [get_ports {gpio_o[3]}]
set_property PACKAGE_PIN D9 [get_ports {gpio_o[4]}]
set_property PACKAGE_PIN C9 [get_ports {gpio_o[5]}]
set_property PACKAGE_PIN B9 [get_ports {gpio_o[6]}]
set_property PACKAGE_PIN B8 [get_ports {gpio_o[7]}]
set_property PACKAGE_PIN E3 [get_ports clk_i]
set_property PACKAGE_PIN C2 [get_ports rstn_i]
set_property PACKAGE_PIN A9 [get_ports uart0_rxd_i]
set_property PACKAGE_PIN D10 [get_ports uart0_txd_o]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_o[7]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_o[6]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_o[5]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_o[4]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_o[3]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_o[2]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_o[1]}]
set_property IOSTANDARD LVCMOS33 [get_ports {gpio_o[0]}]
set_property IOSTANDARD LVCMOS33 [get_ports clk_i]
set_property IOSTANDARD LVCMOS33 [get_ports rstn_i]
set_property IOSTANDARD LVCMOS33 [get_ports uart0_rxd_i]
set_property IOSTANDARD LVCMOS33 [get_ports uart0_txd_o]

set_property CONFIG_MODE SPIx4 [current_design]

set_property OFFCHIP_TERM NONE [get_ports uart0_txd_o]
set_property OFFCHIP_TERM NONE [get_ports gpio_o[7]]
set_property OFFCHIP_TERM NONE [get_ports gpio_o[6]]
set_property OFFCHIP_TERM NONE [get_ports gpio_o[5]]
set_property OFFCHIP_TERM NONE [get_ports gpio_o[4]]
set_property OFFCHIP_TERM NONE [get_ports gpio_o[3]]
set_property OFFCHIP_TERM NONE [get_ports gpio_o[2]]
set_property OFFCHIP_TERM NONE [get_ports gpio_o[1]]
set_property OFFCHIP_TERM NONE [get_ports gpio_o[0]]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]

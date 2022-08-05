#
# user core constraints
#
# put your clock groups in here as well as any net assignments
#

set_clock_groups -asynchronous \
 -group { bridge_spiclk } \
 -group { clk_74a } \
 -group { clk_74b } \
 -group { ic|mp1|mf_pllbase_inst|altera_pll_i|general[0].gpll~PLL_OUTPUT_COUNTER|divclk } \
 -group { ic|mp1|mf_pllbase_inst|altera_pll_i|general[1].gpll~PLL_OUTPUT_COUNTER|divclk } \
 -group { ic|mp1|mf_pllbase_inst|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk } \
 -group { ic|mp1|mf_pllbase_inst|altera_pll_i|general[3].gpll~PLL_OUTPUT_COUNTER|divclk } 
# -group { ic|mp1|mf_pllbase_inst|altera_pll_i|general[4].gpll~PLL_OUTPUT_COUNTER|divclk } 
 

set dram_chip_clk "ic|mp1|mf_pllbase_inst|altera_pll_i|general[3].gpll~PLL_OUTPUT_COUNTER|divclk"
set dram_cont_clk "ic|mp1|mf_pllbase_inst|altera_pll_i|general[2].gpll~PLL_OUTPUT_COUNTER|divclk"

set_input_delay -clock $dram_chip_clk -reference_pin [get_ports {dram_clk}] -max 5.9 [get_ports dram_dq[*]]
set_input_delay -clock $dram_chip_clk -reference_pin [get_ports {dram_clk}] -min 0.9 [get_ports dram_dq[*]] 
#6.6
#3.5
#8.7
#6.0


set_output_delay -clock $dram_chip_clk -reference_pin [get_ports {dram_clk}] -max 2.0 [get_ports {dram_cke dram_a* dram_ba* dram_cas_n dram_ras_n dram_we_n}]
set_output_delay -clock $dram_chip_clk -reference_pin [get_ports {dram_clk}] -min -1.0 [get_ports {dram_cke dram_a* dram_ba* dram_cas_n dram_ras_n dram_we_n}]

set_multicycle_path -from $dram_chip_clk -to $dram_cont_clk -setup -end 2
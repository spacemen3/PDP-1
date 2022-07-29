set_global_assignment -entity "PLL_Core" -library "lib_PLL_Core" -name IP_TOOL_NAME "altera_pll"
set_global_assignment -entity "PLL_Core" -library "lib_PLL_Core" -name IP_TOOL_VERSION "20.1"
set_global_assignment -entity "PLL_Core" -library "lib_PLL_Core" -name IP_TOOL_ENV "mwpim"
set_global_assignment -library "lib_PLL_Core" -name SPD_FILE [file join $::quartus(sip_path) "PLL_Core.spd"]

set_global_assignment -library "lib_PLL_Core" -name MISC_FILE [file join $::quartus(sip_path) "PLL_Core_sim/PLL_Core.vo"]

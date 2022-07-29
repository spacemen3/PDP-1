set_global_assignment -entity "Multipler" -library "lib_Multipler" -name IP_TOOL_NAME "altera_mult_add"
set_global_assignment -entity "Multipler" -library "lib_Multipler" -name IP_TOOL_VERSION "20.1"
set_global_assignment -entity "Multipler" -library "lib_Multipler" -name IP_TOOL_ENV "mwpim"
set_global_assignment -library "lib_Multipler" -name SPD_FILE [file join $::quartus(sip_path) "Multipler.spd"]

set_global_assignment -library "lib_Multipler" -name MISC_FILE [file join $::quartus(sip_path) "Multipler_sim/Multipler.v"]

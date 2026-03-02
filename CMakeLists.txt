add_library(plugin_atc INTERFACE)

target_sources(plugin_atc INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/flexihal_atc.c
 ${CMAKE_CURRENT_LIST_DIR}/tooltable.c
 ${CMAKE_CURRENT_LIST_DIR}/atc_tool_change.c
)

target_include_directories(plugin_atc INTERFACE ${CMAKE_CURRENT_LIST_DIR})
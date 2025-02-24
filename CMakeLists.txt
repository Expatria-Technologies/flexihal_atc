add_library(plugin_atc INTERFACE)

target_sources(plugin_atc INTERFACE
 ${CMAKE_CURRENT_LIST_DIR}/flexihal_atc.c
)

target_include_directories(plugin_atc INTERFACE ${CMAKE_CURRENT_LIST_DIR})
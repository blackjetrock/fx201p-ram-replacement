# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# initalize pico_sdk from installed location
# (note this can come from environment, CMake cache etc)
set(PICO_SDK_PATH "/tree/downloaded_tools/pico/pico-sdk")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)
include(pico_extras_import.cmake)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g")
project(hd36106_replacement C CXX ASM)
set(PICO_COPY_TO_RAM 1)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# SD FAT LIB
#add_subdirectory(/tree/downloaded_tools/pico/ajm-no-OS-FatFS-SD-SPI-RPi-Pico/FatFs_SPI build)

# Add executable. Default name is the project name, version 0.1

add_executable(hd36106_replacement
hd36106_replacement.c
fx201p_execution.c
bid/bid_from_int.c
bid/bid_decimal_data.c
bid/bid_convert_data.c
bid/bid128_2_str.h
bid/bid128_2_str_macros.h
bid/bid128_2_str_tables.c
bid/bid128_acos.c
bid/bid128_acosh.c
bid/bid128_add.c
bid/bid128_asin.c
bid/bid128_asinh.c
bid/bid128_atan2.c
bid/bid128_atan.c
bid/bid128_atanh.c
bid/bid128.c
bid/bid128_cbrt.c
bid/bid128_compare.c
bid/bid128_cos.c
bid/bid128_cosh.c
bid/bid128_div.c
bid/bid128_erf.c
bid/bid128_erfc.c
bid/bid128_exp10.c
bid/bid128_exp2.c
bid/bid128_exp.c
bid/bid128_expm1.c
bid/bid128_fdimd.c
bid/bid128_fma.c
bid/bid128_fmod.c
bid/bid128_frexp.c
bid/bid128_hypot.c
bid/bid128_ldexp.c
bid/bid128_lgamma.c
bid/bid128_llquantexpd.c
bid/bid128_llrintd.c
bid/bid128_llround.c
bid/bid128_log10.c
bid/bid128_log1p.c
bid/bid128_log2.c
bid/bid128_logb.c
bid/bid128_logbd.c
bid/bid128_log.c
bid/bid128_lrintd.c
bid/bid128_lround.c
bid/bid128_minmax.c
bid/bid128_modf.c
bid/bid128_mul.c
bid/bid128_nearbyintd.c
bid/bid128_next.c
bid/bid128_nexttowardd.c
bid/bid128_noncomp.c
bid/bid128_pow.c
bid/bid128_quantexpd.c
bid/bid128_quantize.c
bid/bid128_quantumd.c
bid/bid128_rem.c
bid/bid128_round_integral.c
bid/bid128_scalb.c
bid/bid128_scalbl.c
bid/bid128_sin.c
bid/bid128_sinh.c
bid/bid128_sqrt.c
bid/bid128_string.c
bid/bid128_tan.c
bid/bid128_tanh.c
bid/bid128_tgamma.c
bid/bid128_to_int16.c
bid/bid128_to_int32.c
bid/bid128_to_int64.c
bid/bid128_to_int8.c
bid/bid128_to_uint16.c
bid/bid128_to_uint32.c
bid/bid128_to_uint64.c
bid/bid128_to_uint8.c
#bid/bid_binarydecimal.c
)

#add_library( gcc111libbid_hard UNKNOWN IMPORTED)
#SET_PROPERTY(TARGET gcc111libbid_hard  PROPERTY IMPORTED_LOCATION /tree/projects/github/fx201p-ram-replacement/firmware/hd36106_replacement/build/gcc111libbid_hard.a)

#pico_generate_pio_header(hd36106_replacement ${CMAKE_CURRENT_LIST_DIR}/picoputer.pio)

pico_set_program_name(hd36106_replacement "hd36106_replacement")
pico_set_program_version(hd36106_replacement "0.1")

#pico_enable_stdio_uart(hd36106_replacement 0)
pico_enable_stdio_usb(hd36106_replacement 1)

# Add the standard library to the build
target_link_libraries(hd36106_replacement pico_stdlib )

IF (NOT DEFINED N_SD_CARDS)
    SET(N_SD_CARDS 1)
ENDIF()
target_compile_definitions(hd36106_replacement PUBLIC N_SD_CARDS=${N_SD_CARDS})

# Add any user requested libraries
target_link_libraries(hd36106_replacement
 #gcc111libbid_hard
        #hardware_i2c
        hardware_pio
        hardware_clocks
	hardware_adc
	hardware_dma
#        pico_sd_card
        pico_multicore
#	FatFs_SPI
        )

pico_add_extra_outputs(hd36106_replacement)


add_executable(pcm1802_audio
	pcm1802_audio.c
)

target_compile_options(pcm1802_audio PRIVATE -Wall)

target_link_libraries(pcm1802_audio
	pico_stdlib
	pico_util
	hardware_pio
	hardware_dma
	libpico100basetx
)

pico_generate_pio_header(pcm1802_audio ${CMAKE_CURRENT_LIST_DIR}/pcm1802_fmt00.pio)

# enable usb output, disable uart output
pico_enable_stdio_usb(pcm1802_audio 1)

# create map/bin/hex file etc.
pico_add_extra_outputs(pcm1802_audio)

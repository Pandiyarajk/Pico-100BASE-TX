# Pico-100BASE-TX - Bit banged 100 MBit/s Ethernet

This library allows to stream out data with around 11 Mbyte/s from a RP2040 or RP2350 MCU using the PIO to bit-bang a 100 Mbit Fast Ethernet connection.
It is somewhat similar to [Pico-10BASE-T](https://github.com/kingyoPiyo/Pico-10BASE-T), which implemented TX-only 10 MBit Ethernet.

**Warning: Do not connect to any POE capable equipment!**

Ideally use a pulse transformer with proper matching circuitry. In my experiments I directly connected the two GPIOs to an old ethernet cable and it worked with all devices I've tested - only do that at your **own risk**. You also could use some old Ethernet switch and connect it in between the Pico and your machine to be safe.

Demo digitizing some WBFM IF signal with the internal ADC and streaming it out via Fast Ethernet:

https://github.com/user-attachments/assets/e150bc89-122c-4669-ae62-b9b03f8b2769

## Building

Make sure you have the latest version of the [pico-sdk](https://github.com/raspberrypi/pico-sdk) installed together with an appropriate compiler. You should be able to build the [pico-examples](https://github.com/raspberrypi/pico-examples).

To build Pico-100BASE-TX for a Pico2:

    git clone https://github.com/steve-m/Pico-100BASE-TX.git
    mkdir Pico-100BASE-TX/build
    cd Pico-100BASE-TX/build
    export PICO_SDK_PATH=/<path-to>/pico-sdk
    cmake -DPICO_PLATFORM=rp2350 -DPICO_BOARD=pico2 ../
    make -j 8

After the build succeeds you can copy the resulting *.uf2 file of the application you want to run to the board.

## Example applications

The repository contains a library - libpico100basetx - which implements the main functionality. It reads the data from a ringbuffer, and streams it out via the 100BASE-TX UDP frames.
In addition to that, the apps folder contains a couple of example applications:

### counter

This application uses the PIO to generate a 16-bit counter value which is written to a DMA ringbuffer, which is then streamed out via UDP datagrams.

### internal_adc

The data from the internal ADC is streamed out.

### pcm1802_audio

This app streams the audio data from a PCM1802 audio ADC board. The used pinout is the same as on this [adapter PCB](https://github.com/Sev5000/Pico2_12bitADC_PCMAudio), and the audio samplerate is 75 kHz.


## Credits

Pico-100BASE-TX is developed by Steve Markgraf, and is loosely based on [hsdaoh-rp2350](https://github.com/steve-m/hsdaoh-rp2350).



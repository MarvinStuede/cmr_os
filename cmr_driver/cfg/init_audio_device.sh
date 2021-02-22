#!/bin/bash

echo "=================="
echo "Setting ReSpeaker as default audio source and output device."
echo "Your user should be in the audio group"
pactl set-default-sink 'alsa_output.pci-0000_00_1f.3.analog-stereo'
#pactl set-default-sink 'alsa_output.pci-0000_00_1b.0.analog-stereo'
#pactl set-default-source 'alsa_input.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.multichannel-input'
#pactl set-default-sink 'alsa_output.usb-SEEED_ReSpeaker_4_Mic_Array__UAC1.0_-00.analog-stereo'
pactl set-sink-volume 0 100%
echo "=================="

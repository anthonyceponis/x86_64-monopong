%define FRAMEBUFFER "/dev/fb0"
%define KEYBOARD_DEVICE "/dev/input/event3" ; Your event number might be different. Gonna have to trial and error if that is the case. Check which ones are available in the "/dev/input" dir.

; Run "cat /sys/class/graphics/fb0/virtual_size" to obtain screen dimensions and replace numbers below.
SCREEN_X equ 1920
SCREEN_Y EQU 1200
COLOUR_DEPTH equ 4 ; In bytes.
FB_SIZE equ SCREEN_X * SCREEN_Y * COLOUR_DEPTH ; In bytes.

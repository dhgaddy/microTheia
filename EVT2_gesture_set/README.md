
# EVT2 Gesture Set
## Full temporal resolution EVT2 recordings, captured on STM32 board with GenX320 sensor.
Recordings are labeled as:

(gesture main type) _ (gesture sub type) _ (lighting condition) _ (collection name + file number).bin

ex) wave_up_sun_test1.bin is a wave type gesture, specifically waving up, that was recorded in the sun and is the first file in this gesture/lighting combination's test set.

## .elf Files for use with STM32F7 board with GenX320 sensor
Adapted from Prophesee's example code provided for use with the STM32/GenX320 Discovery kit.

Both versions maintain use of the LED screen visualizing the event stream that was present in Prophesee's example code.

x320_stm_usb.elf -> compressed live EVT2 streaming over USB_FS port. Drops nearly 2/3 of live events due to speed limitation.

fill_local_then_dump.elf -> fills local sdram on STM32 board with EVT2 stream immediately from boot-up, then streams out over USB once full. Limited to 1 - 3 seconds of recording, depending on scene activity level, but achieves full fidelity transfer.

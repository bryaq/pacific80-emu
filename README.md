# BUILD

Dependencies:

- SDL2

# USE

## Obtain ROM image

```
objcopy -I ihex -O binary --gap-fill=255 --pad-to=16384 27c128.hex 27c128.bin
```

## Obtain CF image

```
dd if=/dev/sdb of=cf.img bs=1M count=33
```

## Run

```
./pac80emu 27c128.bin cf.img
```

It will print pseudoterminal device name if you wish to connect to computer's serial port.

![pac80emu](pac80emu.png)

# TODO

- [x] interrupts
- [x] SN76489A
- [x] joystick
- [x] reset
- [x] XMODEM

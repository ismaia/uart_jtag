# uart_jtag

UART-JTAG  provides jtag-uart bridge to control OpenRISC via USB Cable and nios2-terminal

usb->serial adater is not needed anymore

This module is ready to use in orpsoc-cores

Works out of de box on DE0-Nano

How to use:
----------
  - fusesoc build de0_nano ; fusesoc pgm de0_nano
  - openocd -f de0_nano.cfg
  - or1k-elf-gdb  vmlinux --eval-command='target remote localhost:50001'
  - load linux image on gdb: (load)
  - start linux (spr npc 0x100)
  - CLOSE openocd
  - Open nios2-terminal (you should see linux boot, if not, close n2-terminal e re-open)


Troubleshooting
---------------
   nios2-terminal blocks frequently, CTRL-C and restart nios2-terminal.
   The linux console will be there


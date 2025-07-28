# Orin wifi 6E (6GHz) installation

Included here is the built driver (`ifx-backports`) for an Orin Nano or Orin AGX running Jetpack 6.2. You should be able to follow the instructions in `orin_wifi_install.txt`. If that doesn't work, or you hate yourself, you can use `orin_wifi_kernel_build.txt` to build the kernel and backports folder, and optionally flash the kernel onto a board (identical to the default Jetpack 6.2 kernel)

## Orin nano or Orin AGX flash/setup instructions

1. Flash your board with Jetpack 6.2 using the Nvidia SDK manager
    a. If your board is already flashed with Jetpack 6.2, you can install the driver onto it without reflashing (`orin_wifi_install.txt`)
    b. If your board is already flashed with NOT Jetpack 6.2, you will have to re-flash it with Jetpack 6.2. Start with your board powered off, then bridge the `FC_REC` and `GND` pins on the header underneath the heatsink, then power the board and connect it to your PC. You should see it appear in the SDK manager. Make sure to change the storage mode to NVME on the screen where you set the username and password
    c. If you have a clean board, you should just be able to install an NVME drive and power it on connected to your PC to get it into the SDK manager. Make sure to change the storage mode to NVME on the screen where you set the username and password
2. Boot up the board, log in, then follow the instructions in `orin_wifi_install.txt`

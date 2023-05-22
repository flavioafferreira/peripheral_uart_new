.. _peripheral_uart:

Bluetooth: Peripheral UART Modified 
##########################

This is a tracker project with alarm system.

Using a board Nordic nRF5340-DK PCA 10095.

I modified the Peripheral UART sample and added a lot of new functions I created.

In order to work, need to install on Network Core the RPC_Host. This demo was

created to exposes the Bluetooth to Application Core.

At the moment there is actived:

- 3 NTC 10k model MF58

- 4 buttons (from the develop kit)

- 4 leds    (from the develop kit)

- 1 uBlox NEO-6

- 1 Bluetooth connection with MTU of 65 bytes

- 1 External QSPI Memory

- 1 Lora transceiver SX1276

- 1 Infrared sensor

When you press the button, k_sem_take(&button_X,K_FOREVER); release the Thread to execute
some routines.

Button 1 - Send a protobuffer variable to Bluetooth

Button 2 - Save the memory

Button 3 - Print to Terminal all 24h data from HEAP

Button 4 - Send the word "ISADORA PENATI FERREIRA" to Bluetooth


Next Steps:

Install a LoraWan Module to send data to Helium gateway. (Done).

I will make the electrical design and the manual. At the moment the data is being sending and stored on google sheet.

On directory Case_3D you can find the 3d G-Code to print using a 3D Printer the base of nRF5340-DK.

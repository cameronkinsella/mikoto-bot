# WSL Setup for Embedded Development

By default, WSL does not have access to any USB devices.
You will need to use the following workaround if you want to run the program through WSL.

Take a look at the following Microsoft Article if you need additional help:
https://learn.microsoft.com/en-us/windows/wsl/connect-usb

The following instructions are for Ubuntu 20.04.

## Setup

1. Install the latest version of usbidp:
   https://github.com/dorssel/usbipd-win/releases

2. In WSL:
    ```bash
    sudo apt install linux-tools-virtual hwdata
    sudo update-alternatives --install /usr/local/bin/usbip usbip `ls /usr/lib/linux-tools/*/usbip | tail -n1` 20
    ```

3. Run: `usbipd.exe wsl list`. This will list all your USB devices. The STM32 will be labeled something with the word "ST-Link". Note its `BUSID` (something like `2-2`).

4. Replace `BUSID` with the value of `BUSID` found in the previous step
    ```bash
    usbipd.exe wsl attach --busid=BUSID
    ```

5. Run `usbipd.exe wsl list` again and confirm that the STM state is now attached to Ubuntu.

6. Run `lsusb | grep ST-LINK` to list the STM device. Confirm that the STM is showing up correctly.

7. Run the following to enable permission for non-root user to access the STM:
    ```bash
    sudo bash -c 'cat <<EOF > /etc/udev/rules.d/99-openocd.rules
    # STM32F3DISCOVERY - ST-LINK/V2.1
    ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE:="0666"
    EOF'
    ```

8. Reload the new rules: `sudo udevadm control --reload-rules`.
   If you get an error that "Failed to send reload request: No such file or directory", run `sudo service udev restart` then run it again.

9. Run: `usbipd.exe wsl detach --busid=BUSID` then `usbipd.exe wsl attach --busid=BUSID` (effectively unplugging then plugging the USB back in).

10. Done! You can now run the program from WSL.


## Subsequent Setups

If you ever unplug the STM from your computer, you will need to do a couple of commands before you can run the application.

1. Get the `BUSID`:
    ```bash
    usbipd.exe wsl list
    ```

2. Attach the usb to WSL:
    ```bash
    usbipd.exe wsl attach --busid=BUSID
    ```

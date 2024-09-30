# Usability improvements before Abu Dhabi trip

## 1. General improvements

The following things I did, but didnt have a big or even measurable effect on cold-to-wifi boot time

1. Disable dnsmasq, because another dnsmasq is provided within NetworkManager, it's a clusterfck
```
sudo systemctl disable dnsmasq
```
2. Switch from Hotspot to connecting to another Access Point (change the autostart option in `/etc/NetworkManager/something/something`
3. Set the 5 second UEFI timeout to 0 in the UEFI boot menu. This saved 5 seconds.
4. Disable the system console `sudo systemctl disable getty@tty1.service`
5. Disable the system console in the kernel commandline (remove `console=ttyTCU0,115200` from entries in /boot/extlinux/extlinux.conf)
	- this felt like it saved some time, but probably not much

Currently, cold-to-home-screen takes around 45 seconds, cold-to-wifi takes 100.


### Compile new Wifi-drivers.

This improved boot time by around 50 seconds.

I saw we have similar behaviour to `https://forums.developer.nvidia.com/t/jetpack-6-wifi-slow-startup-with-backport-iwlwifi-dkms/297967/8` with the same drivers. The solution in that thread was to backport a later version of either the kernel or the drivers. I dont know how to compile kernels for the jetson (even though that sounds like the cleaner solution), so drivers it is:

The steps verbatim from `dandelion1124`'s answer (although i chose `backports-5.15.153-1.tar.xz`, a newer backport):

> At first, I download backports-5.15.81-1.tar.xz from https://backports.wiki.kernel.org/index.php/Releases 28.
> I installed backports-iwlwifi from source code.

```
tar Jxfv backports-5.15.81-1.tar.xz
cd backports-5.15.81-1
make defconfig-iwlwifi
make -j8
sudo make install
```

> I add the following line to /etc/modprobe.d/iwlwifi.conf.

```
options iwlwifi 11n_disable=1
```

> And, I reboot OS.

**success**: boot time to wifi is low around 40 seconds


### Solve wifi ping issues

The issues were caused by power managed, as always... disable to according to chatGPT. 
This worked and now ping should be in the 1-3ms range.

In file `/etc/NetworkManager/conf.d/default-wifi-powersave-on.conf` change 3 to a 2 ("disable")
```
[connection]
wifi.powersave = 2
```


### Disable Bluetooth

For good measure...

In file `/etc/bluetooth/main.conf`
```
AutoEnable=false
```

Then reboot



### set fan to max on boot

1. create file `/usr/local/bin/set_fan_to_max.sh` with contents:
```
#!/bin/bash
jetson_clocks --fan
```
2. make executable `sudo chmod +x /usr/local/bin/set_fan_to_max.sh`
3. create a unit file, because thats fun. `/lib/systemd/system/set_fan_to_max.service`:
```
[Unit]
After=network.target

[Service]
ExecStart=/usr/local/bin/set_fan_to_max.sh

[Install]
WantedBy=default.target
```
4. enable it 
```
sudo systemctl daemon-reload
sudo systemctl enable set_fan_to_max.service
```
5. reboot and prepare to be annoyed. You can always disable `sudo sysstemctl disable set_fan_to_max` and reboot to turn this behaviour off




## 2. Access Flight Controller via Serial Port forwarding

Install tools:
```
sudo apt install ser2net
```

### UDEV mapping

We want the FC to have the same serial port device file everytime, so we forward the right one.
This is done via udev rules.

1. unplug FC
2. add a UDEV rule, so we know all flight controller serial ports are at `/dev/mapped/fc` at all times
   to do this, create the file /etc/udev/rules.d/99-map-fc.rules, with a single line:
```
ACTION=="add", SUBSYSTEM=="tty", ATTRS{idVendor}=="0483", SYMLINK+="mapped/fc"
```
3. create directory `/dev/mapped` used `sudo mkdir /dev/mapped`
4. reload the udev rules with `sudo udevadm control --reload-rules` and trigger then with `sudo udevadm trigger`. Alternatively, reboot
5. replug FC


### Configure ser2net

1. save default config `sudo cp /etc/ser2net.yaml /etc/ser2net.default.yaml`
2. modify `/etc/ser2net.yaml` to read
```
%YAML 1.1
---
# This is a ser2net configuration file, tailored to be rather
# simple.
#
# Find detailed documentation in ser2net.yaml(5)
# A fully featured configuration file is in
# /usr/share/doc/ser2net/examples/ser2net.yaml.gz
# 
# If you find your configuration more useful than this very simple
# one, please submit it as a bugreport

define: &banner \r\nser2net port \p device \d [\B] (Debian GNU/Linux)\r\n\r\n

connection: &con00
    accepter: tcp,5761
    enable: on
    options:
      kickolduser: true
      accepter-retry-time: 5
      connector-retry-time: 5
    connector: serialdev,/dev/mapped/fc,115200n81,local
```
3. Create unit file `/lib/systemd/system/ser2net.service` to start ser2net at boot:
```
[Unit]
Description=Serial port to network proxy
Documentation=man:ser2net(8)
After=network-online.target
Requires=network-online.target
 
[Service]
EnvironmentFile=-/etc/default/ser2net
ExecStart=/usr/sbin/ser2net -n -c $CONFFILE -P /run/ser2net.pid
# terrible hack, see https://github.com/cminyard/ser2net/issues/60#issuecomment-1077424534
ExecStartPost=-/bin/sh -c 'sleep 1;journalctl -u ser2net|tail -n 5|grep -E ^[A-Z]|grep -q "Invalid port name/number" || exit 0; sleep 5; systemctl restart ser2net'
Type=exec
#Restart=on-failure
Restart=always
 
[Install]
WantedBy=multi-user.target
```
4. start the service and enable it to be run at boot-time
```
sudo systemctl daemon-reload
sudo systemctl enable ser2net.service
sudo systemctl start ser2net.service
```



## 3. SWD

SWD is used to flash, reset, and debug the FC from the Orin (or rather, forwarded through the Orin).
Unfortunately, the Orin and carrier board doesn't expose the right interfaces for the job (GPIOs are hidden in a flat cable).
Instead, we will use a Raspberry Pi Pico (in the future the even smaller Raspberry Pi Debug Probe), which connects to the Orin via USB and is surprisingly fast and robust at providing the SWD connection.

### RPi Pico 1  or  Debug Probe setup

**Note**: steps 1. through 5. only have to be done on a Pico 1. The Debug Probe already comes with the correct softwrae and you dont have to solder.

1. Download debugprobe_on_pico.uf2 onto a laptop from https://github.com/raspberrypi/debugprobe/releases/tag/debugprobe-v2.0.1
2. Push the "bootsel" button on the pico while plugging it into the laptop
3. Drag-and-Drop the debugprobe_on_pico.uf2 into the raspberry-pi folder that comes up after plugging it in
4. disconnect the raspberry pi from usb
5. wire up the Raspberry Pi Pico
```
Ground to Ground on the FC
GP2 to SWDCLK (C), use white wire and MAXIMUM AWG24. And POSITIVELY NO DUPONT SHITTY WIRES
GP3 to SWDIO (D), use yellow wire and MAXIMUM AWG24. And POSITIVELY NO DUPONT SHITTY WIRES
```
6. plug the raspberry pi pico into the ORIN with a micro-USB to USBC cable



### Orin setup

1. on the orin, install `sudo apt install openocd`
2. `mkdir /opt/openocd`
3. create basic `openocd` configuration file in `/opt/openocd/openocd.cfg` (sorry for the unusual location, not gonna change it though). Most of these settings come from the raspberry pi pico manual, appendix A
```
source [find interface/cmsis-dap.cfg]
adapter speed 5000
transport select swd
set WORKAREASIZE 0x2000
source /opt/openocd/chip.cfg
reset_config none
```
4. create an additional config file  `/opt/openocd/openocd_debug.cfg` that starts a gdb debugging server:
```
source /opt/openocd/helpers.tcl
CDLiveWatchSetup
init
targets
reset halt # TODO: maybe not reset halt?
```
5. create the file `/opt/openocd/helpers.tcl` which enables the Cortex Live Watch stuff in VSCode
```
# Copyright 2017-2023 Marcel Ball
# see github.com/Marus/cortex-debug/blob/master/support/openocd-helpers.tcl

proc CDLiveWatchSetup {} {
    try {
        foreach tgt [target names] {
            set nConn [$tgt cget -gdb-max-connections]
            if { $nConn > 0 } {
                incr nConn
                $tgt configure -gdb-max-connections $nConn
                puts "[info script]: Info: Setting gdb-max-connections for target '$tgt' to $nConn"
            }
        }
    } on error {} {
        puts stderr "[info script]: Error: Failed to increase gdb-max-connections for current target. Live variables will not work"
    }
}
```
6. create a symbolic link to the correct chip config: `sudo ln -sf /usr/share/openocd/scripts/target/stm32h7x.cfg /opt/openocd/chip.cfg`
7. create a systemd service that allows us to easily start and restart the debugging. Create file `/lib/systemd/system/openocd.service`:
```
[Unit]
Description=OpenOCD daemon wrapper
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=/usr/bin/openocd -f /opt/openocd/openocd.cfg -f /opt/openocd/openocd_debug.cfg

[Install]
WantedBy=multi-user.target
```
8. `sudo systemctl daemon-reload`
9. `sudo reboot`
10. Test by installing gdb-multiarch on a local laptop. Run it and type `target extended-remote <IP_OF_THE_ORIN>:3333`. FC should halt, and gdb should warn that it has no executable to load. thats perfect! To get FC running again, reset it (see next section)

### Resetting the flight controller

To reset the FC with almost no questions asked, run

```
sudo service stop openocd
openocd -f /opt/openocd/openocd.cfg -c 'init; reset; exit'
```

Or, from a laptop connected to the same wifi

```
sshpass -p dr ssh droneracing@192.168.1.68 "sudo systemctl stop openocd; openocd -f /opt/openocd/openocd.cfg -c 'init; reset; exit'"
```



## 4. Log Downloading

We need to have absolute hygiene about mounting/unmounting stuff.

1. disable automounting when in ubuntu's desktop environment. Connected disks will still be shown in the file manager, but you have to click before they mount, which seems a good compromise.
```
gsettings set org.gnome.desktop.media-handling automount false
```
2. make it possible for the user to run sudo commands without providing a password. This is necessary for scripts that use `sshpass` to run commands on the Orin.
Add a file `/etc/sudoers.d/mounting` with contents:
```
droneracing ALL=(ALL:ALL) NOPASSWD: ALL
```
3. use the latest version of `indiflightSupport` and properly configure the new `remote.env` file. 


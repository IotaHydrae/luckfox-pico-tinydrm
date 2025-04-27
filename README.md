# TinyDRM drivers for luckfox pico

```shell
adb push output/image/boot.img /root
adb shell "dd if=/root/boot.img of=/dev/mmcblk1p4 bs=1M && reboot"
```

push ko
```shell
adb push st7567_tinydrm.ko /root/
adb shell "rmmod /root/st7567_tinydrm.ko || insmod /root/st7567_tinydrm.ko"
```

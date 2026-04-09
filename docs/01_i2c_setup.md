# Enabling I2C for the Waveshare Sense HAT B

## Orange Pi 5 (ArmbianOS)

On Orange Pi 5 with ArmbianOS, the physical pins 3 (SDA) and 5 (SCL) for the 26-pin header are actually connected to I2C-5 controller in the RK3588 SoC, and are configured in mux mode 3.

- Add user overlay of /boot/ArmbianEnv.txt: `user_overlays= rk3588-i2c5-m3`
- Run `sudo armbian-add-overlay ./hw/rk3588/rk3588-i2c5-m3.dts`
- Reboot and recheck if there are some entries with `sudo i2cdetect -y 5`.
- Another alternative to list all of the i2c-bus by running `ls /dev/i2c* | while read line; do id="$(echo $line | cut -d '-' -f 2)"; echo -e "\\n## Detecting i2c ID: $id"; sudo i2cdetect -y $id; done`

See other dts overlay files in https://github.com/orangepi-xunlong/linux-orangepi/tree/orange-pi-5.10-rk3588/arch/arm64/boot/dts/rockchip/overlay

## Raspberry Pi 4 (Ubuntu 24.04 Server)

On RaspberryPi with Ubuntu 24.04 Server, the sensor is connected to I2C-1. Follow these steps:

1. `sudo raspi-config`
2. Choose "3. Interface Options" -> "I5 I2C" -> select "yes".
3. Reboot and check that `sudo i2cdetect -y 1` returns device addresses on 0x29, 0x48, 0x5c, 0x68, and 0x70.

## Verifying I2C Detection

Overall `sudo i2cdetect -y 5` on Orange Pi 5 or `sudo i2cdetect -y 1` on Raspberry Pi 4 should return:

```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:                         -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- 29 -- -- -- -- -- --
30: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- 48 -- -- -- -- -- -- --
50: -- -- -- -- -- -- -- -- -- -- -- -- 5c -- -- --
60: -- -- -- -- -- -- -- -- 68 -- -- -- -- -- -- --
70: 70 -- -- -- -- -- -- --
```

Where

- 0x29 represents the Color recognition sensor TCS34725
- 0x48 represents the AD conversion ADS1015.
- 0x68 represents the IMU 9-axis sensor ICM-20948.
- 0x5C represents the Air pressure sensor LPS22HB
- 0x70 represents the Temperature and humidity sensor SHTC3

## I2C User Permissions

Add user to the group permission of i2c and dialout:

```
sudo usermod -aG i2c $USER
sudo usermod -aG dialout $USER
```

Log out and log back in for the group changes to take effect.
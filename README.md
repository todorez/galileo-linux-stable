galileo-linux-stable
====================

This is a kernel for Intel Galileo Gen1/2 which will be rebased on and will track Linux stable.

If your Galileo Gen1/2 project requires some of the latest kernel features then this might be the 
Linux kernel for you. 

How to build the kernel?

1.Build a cross-compiler toolchain with Yocto

http://git.yoctoproject.org/cgit/cgit.cgi/meta-intel-iot-devkit/

OR

2.Use a pre-built toolchain from IoT DevKit

https://software.intel.com/sites/landingpage/iotdk/linux-developement-kit.html


After you have built/downloaded the cross-compiler toolchain:

1.Include the cross-compilation tools in your PATH:

export PATH=path_to_sdk/sysroots/x86_64-pokysdk-linux/usr/bin/i586-poky-linux:$PATH


2.Cross-compile the kernel

ARCH=i386 LOCALVERSION= CROSS_COMPILE=i586-poky-linux- make -j8


3.Extract the kernel modules from the build to a target directory (e.g ../galileo-stable-install)

ARCH=i386 LOCALVERSION= INSTALL_MOD_PATH=../galileo-stable-install CROSS_COMPILE=i586-poky-linux- make modules_install


4.Extract the kernel image (bzImage) from the build to a target directory (e.g ../galileo-stable-install)

cp arch/x86/boot/bzImage ../galileo-stable-install/


5.Install the new kernel and modules from the target directory (e.g ../galileo-stable-install) to your micro SD card

5.1.Replace the bzImage found in the first partition (ESP) of your micro SD card with the one from your target directory (backup the bzImage on the micro SD card e.g. rename it to bzImage.old)

5.2.Copy the kernel modules from the target directory to the /lib/modules/ directory found in the second 
partition of your micro SD card (e.g /lib/modules/3.18.1-galileo-g1)

6.Reboot into your new kernel

NOTE: 
If you experience any issues with your custom-built kernel you can revert to you kernel backup from step 5.1

DISCLAIMER
This project is run on a best effort basis with no warranty and independent from my employer.
Hopefully it will be useful to someone and is provided "as is". The entire risk to the quality,functionality and 
performance is with you. If it breaks, you get to keep both pieces.


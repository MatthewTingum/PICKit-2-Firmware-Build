# Overview

This repository houses the latest publicly available PICKit II firmware source (v2.32.00)
along with a dockerfile and prebuilt Docker image to compile it.

The source was obtained from the [microchip website](http://ww1.microchip.com/downloads/en/DeviceDoc/FirmwareV2-32-00.zip).
It can also be obtained from the [wayback machine](https://web.archive.org/web/20240530101822/http://ww1.microchip.com/downloads/en/DeviceDoc/FirmwareV2-32-00.zip)
should Microchip ever stop hosting it.

I've made some small modifications to the firmware and bootloader source in order to make it
play nice with the toolchain used by this project.
The very first commit in this project imports the source as-is.
You can follow along with any modifications to the source in the git history.

# Building

The following sections describe how to build the firmware on `Ubuntu 24.04 LTS`.
The dockerized nature of the build environment should allow you to build the firmware
on almost any Linux distro or platform that supports Docker (Windows, Mac, etc.).

## Prerequisites

- python3
  - `sudo apt install python3`
  - You will need pip installed as well
- [scuba](https://pypi.org/project/scuba/)
  - `sudo pip3 install scuba`
- docker
  - `sudo apt install docker.io`
  - You might need to follow current year's [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/)

## Compiling and Linking

From the root directory of this repository, run the command:
```sh
scuba build
```

The bootloader and firmware binaries will be generated in the `dist` directory.

# Optional Reading

The following section(s) are optional reading.
The previous sections have already described how to build the firmware 

## Toolchain

If you `grep` the source for the word "Compiler", you will see a few different versions listed:
- C18 2.30.01+
- C18 3.00
- C18 3.11+

Looks like we'll need C18 3.11 or higher.
I do not know what version Microchip used to compile the firmware and bootloader.
There is a definition that was renamed in the firmware but not the bootloader.
This change makes the bootloader source uncompilable without the modifications made in this repo.
This indicates that the firmware has been built more recently than the bootloader or that
someone forgot to check something into version control.

This project utilizes "C18 3.40" because it runs on Linux and I was able to find a copy of it.
I did not find it in the Microchip archives.
At one point, a copy of the installer was hosted in a project on bitbucket.
That project has disappeared but the installer was still available on the
[Wayback Machine](https://web.archive.org/web/20150801132201/https://bbuseruploads.s3.amazonaws.com/simbuckone/simbuckbaseproject/downloads/mplabc18-v3.40-linux-full-installer.run?Signature=nMC54JLd0SfTdlc510ADndYnjW0%3D&Expires=1438437120&AWSAccessKeyId=0EMWEFSGA12Z1HF1TZ82&response-content-disposition=attachment%3B%20filename%3D%22mplabc18-v3.40-linux-full-installer.run%22).
I uploaded the installer to the Wayback Machine in the form of a direct download [here](https://archive.org/details/mplabc18-v3.40-linux-full-installer).
I would prefer a download direct from the Microchip website but this is what we have.
Please contact me if you find a direct download.
Security minded individuals, preservationists, and others dealing with compiler version-related bugs
should be aware that the compiler wasn't obtained directly from Microchip.

The source contains an `mcp` file.
This is a project file used by MPLAB which is an IDE.
This project compiles headless and does not use an IDE.
If you want to explore the real build experience of Microchip engineers, you'll probably be looking at an IDE prior to MPLABX.

If you would like to build the docker image containing MPLABC18 Version 3.40, simply build the `Dockerfile` in the `docker` directory.

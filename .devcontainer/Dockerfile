FROM archlinux:latest

RUN pacman -Syu --noconfirm \
    arm-none-eabi-binutils \
    arm-none-eabi-gcc \
    arm-none-eabi-gdb \
    arm-none-eabi-newlib \
    cmake make git \
    stlink openocd \
    libusb usbutils \
    minicom \
    && pacman -Scc --noconfirm

RUN pacman -Syu --noconfirm nano \
    && pacman -Scc --noconfirm

RUN groupadd -r plugdev && \
    useradd -m -G plugdev,uucp,lock -s /bin/bash devuser && \
    echo "devuser ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/devuser

USER devuser
WORKDIR /home/devuser

SHELL ["/bin/bash", "-c"]

ENTRYPOINT ["/bin/bash"]

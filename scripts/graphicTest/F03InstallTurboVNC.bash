#!/bin/bash
echo "Installing TurboVNC Version 1.1.95 (1.2rc)"
rm -rf turbovnc; mkdir -p turbovnc; cd turbovnc

# Figure out architecture and set the url accordingly
arch=`dpkg --print-architecture`
case "$arch" in
"amd64")
    echo "Using 64-bit version"
    pkg="$WORKSPACE/graphicTest/bin/turbovnc_1.1.95_amd64.deb"
    ;;
*)
    echo "Using 32-bit version"
    pkg="$WORKSPACE/graphicTest/bin/turbovnc_1.1.95_i386.deb"
    ;;
esac

# Install turbovnc dependencies
sudo apt-get install -y xauth
sudo apt-get install -y xfonts-base

# Install the package
sudo dpkg -i $pkg

# Copy passwd file
mkdir -p ~/.vnc
cp $WORKSPACE/graphicTest/vncpasswd ~/.vnc/passwd
cp $WORKSPACE/graphicTest/xstartup.turbovnc ~/.vnc/

# Update bashrc
echo "export PATH=/opt/TurboVNC/bin:\$PATH" >> ~/.bashrc

# Remove artifacts
cd .. && rm -rf turbovnc

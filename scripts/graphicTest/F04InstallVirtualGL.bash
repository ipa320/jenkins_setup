#!/bin/bash
echo "Installing TurboVNC Virtual GL 2.3.2"
rm -rf virtualgl; mkdir -p virtualgl; cd virtualgl

# Figure out architecture and set the url accordingly
arch=`dpkg --print-architecture`
case "$arch" in
"amd64")
    echo "Using 64-bit version"
    pkg="$WORKSPACE/graphicTest/bin/virtualgl_2.3.2_amd64.deb"
    ;;
*)
    echo "Using 32-bit version"
    pkg="$WORKSPACE/graphicTest/bin/virtualgl_2.3.2_i386.deb"
    ;;
esac

# Useful for some 3d-Applications
sudo apt-get install libxv1

# Download the package and install
sudo dpkg -i $pkg


# Update bashrc
echo "export PATH=/opt/VirtualGL/bin:\$PATH" >> ~/.bashrc

# Remove artifacts
cd .. && rm -rf virtualgl

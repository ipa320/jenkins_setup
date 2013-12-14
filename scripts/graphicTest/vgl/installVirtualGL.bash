#!/bin/bash
echo "Installing TurboVNC Virtual GL 2.3.2"

# Figure out architecture and set the url accordingly
case "`dpkg --print-architecture`" in
"amd64")
    echo "Using 64-bit version"
    pkg="$DIR/vgl/virtualgl_2.3.2_amd64.deb"
    ;;
*)
    echo "Using 32-bit version"
    pkg="$DIR/vgl/virtualgl_2.3.2_i386.deb"
    ;;
esac

# Required for some 3d-Applications
sudo apt-get install -y --force-yes libxv1

# Download the package and install
sudo dpkg -i $pkg

sudo /opt/VirtualGL/bin/vglserver_config -config +s +f -t

[ -f /opt/VirtualGL/bin/vglrun ]

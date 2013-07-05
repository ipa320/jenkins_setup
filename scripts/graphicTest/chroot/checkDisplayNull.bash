export DISPLAY=:0
type xdpyinfo 1>/dev/null 2>&1 || apt-get install x11-utils -y --force-yes
if [ ! -z "`xdpyinfo 2>&1 | grep -i 'unable to open'`" ]; then
    echo ''
    echo '------------------------------------'
    echo '     Could not open display :0      '
    echo 'Try running xhost + on remote server'
    echo '------------------------------------'
    echo ''
    exit 1
fi
exit 0

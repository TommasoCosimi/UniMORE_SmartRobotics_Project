#!/bin/bash
# Gather the VNC Password
vnc_password=$(cat /scripts/.vncpass)

# Allow remote connections to the X Server
sudo xhost +local:

# Remove old files
sudo rm -rf /tmp/.X11-unix/X*

# Start the VNC Server
echo -e "${vnc_password}\n${vnc_password}\nn\n" | tigervncserver -xstartup /usr/bin/xfce4-session -geometry 1280x720 -localhost no :1
websockify -D --web=/usr/share/novnc/ 6080 localhost:5901

# Keep the container running
sleep infinity
FROM debian:trixie

# Add a default non root user with the same UID and GID of the host user
ARG USERNAME=novnc_server
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Give the non root user sudo access
RUN apt-get update
RUN apt-get -y full-upgrade
RUN apt-get install -y sudo
RUN rm -rf /var/lib/apt/lists/*
RUN echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the non root user
USER $USERNAME

# Install a minimal XFCE Desktop Environment
RUN sudo apt-get update
RUN sudo apt-get -y install \
    dbus-x11 \
    xfce4 \
    xfce4-terminal
RUN sudo apt-get -y remove \
    xfce4-power-manager
RUN sudo rm -rf /var/lib/apt/lists/*

# Install the VNC Server
RUN sudo apt-get update
RUN sudo apt-get -y install \
    tigervnc-standalone-server
RUN sudo rm -rf /var/lib/apt/lists/*
    
# Install the noVNC Server
RUN sudo apt-get update
RUN sudo apt-get -y install \ 
    novnc \
    python3-websockify
RUN sudo rm -rf /var/lib/apt/lists/*
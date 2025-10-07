FROM python:3.13.7-trixie

# Install APT depedencies for Python Packages
RUN apt-get update 
RUN apt-get -y full-upgrade 
RUN apt-get install -y \
    ffmpeg \
    libsm6 \
    libxext6
RUN rm -rf /var/lib/apt/lists/*

# Add a default non root user with the same UID and GID of the host user
ARG USERNAME=python
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi

# Switch to the non root user
USER $USERNAME

COPY requirements.txt /requirements.txt

RUN pip install -r /requirements.txt
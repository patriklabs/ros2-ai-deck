FROM ros:jazzy-perception

ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Prevent stop building ubuntu at time zone selection.
ENV DEBIAN_FRONTEND=noninteractive

VOLUME /workspace

# Create the user
RUN apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update \
    && apt-get install -y python3-colcon-common-extensions \
    && apt install -y python3-virtualenv \
    && apt install -y python3-pip \
    && rm -rf /var/lib/apt/lists/*

COPY requirements.txt requirements.txt
RUN pip install --break-system-packages -r requirements.txt && rm -rf /var/lib/apt/lists/*

ENV DEBIAN_FRONTEND=dialog

ENV PATH="${PATH}:/home/$USERNAME/.local/bin"
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/$USERNAME/.bashrc
RUN echo "export ROS_DOMAIN_ID=43" >> /home/$USERNAME/.bashrc
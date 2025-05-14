FROM ros:jazzy
ARG USERNAME=selene
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
ENV SHELL=/bin/bash

# Install dependencies for NVM and Node.js
RUN apt-get install -y curl ca-certificates

# Set environment vars
ENV NVM_DIR=/home/$USERNAME/.nvm
ENV NODE_VERSION=20

# Install NVM and Node (inside one bash -c for proper variable expansion)
RUN mkdir -p /home/$USERNAME/.nvm && \
    bash -c "curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash && \
    source /home/$USERNAME/.nvm/nvm.sh && \
    nvm install $NODE_VERSION && \
    nvm use $NODE_VERSION && \
    nvm alias default $NODE_VERSION"

# Change ownership to the created user
RUN chown -R $USERNAME:$USERNAME /home/$USERNAME/.nvm

# Set up NVM environment for interactive shells
RUN echo "export NVM_DIR=$NVM_DIR" >> /home/$USERNAME/.bashrc && \
    echo '[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"' >> /home/$USERNAME/.bashrc && \
    echo '[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"' >> /home/$USERNAME/.bashrc && \
    echo 'source /opt/ros/jazzy/setup.bash' >> /home/$USERNAME/.bashrc && \
    echo 'source /home/$USERNAME/ROS2/install/setup.bash' >> /home/$USERNAME/.bashrc


# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]

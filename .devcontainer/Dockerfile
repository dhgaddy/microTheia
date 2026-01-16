FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu

ENV LANG C.UTF-8

# Update and install dependencies
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        wget \
        unzip \
        ca-certificates \
        curl \
        git \
        build-essential \
        python3 \
        python3-pip && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create tools directory
RUN mkdir -p /tools && chmod 755 /tools

# Install oss-cad-suite
RUN cd /tools && \
    wget --quiet https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-x64-20241121.tgz && \
    tar -xf oss-cad-suite-linux-x64-20241121.tgz && \
    rm oss-cad-suite-linux-x64-20241121.tgz && \
    chmod -R 755 /tools/oss-cad-suite

# Install sv2v
RUN cd /tools && \
    wget --quiet https://github.com/zachjs/sv2v/releases/download/v0.0.12/sv2v-Linux.zip && \
    unzip -q sv2v-Linux.zip && \
    rm sv2v-Linux.zip && \
    chmod 755 /tools/sv2v

# Add tools to PATH globally
ENV PATH="/tools/oss-cad-suite/bin:/tools:${PATH}"
RUN echo 'export PATH="/tools/oss-cad-suite/bin:/tools:$PATH"' >> /etc/bash.bashrc

# Switch to vscode user for Rust installation
USER vscode

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/home/vscode/.cargo/bin:${PATH}"
RUN echo 'export PATH="/home/vscode/.cargo/bin:/tools/oss-cad-suite/bin:/tools:$PATH"' >> /home/vscode/.bashrc

# Switch back to root
USER root

# Copy and make startup script executable
COPY startup.sh /usr/local/bin/startup.sh
RUN chmod +x /usr/local/bin/startup.sh && \
    chown vscode:vscode /usr/local/bin/startup.sh

FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu

ENV LANG C.UTF-8
ARG INSTALL_ZSH="true"

# Update package lists and install security updates
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y wget unzip ca-certificates && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create tools directory
RUN mkdir -p /tools && chmod 755 /tools

# Install oss-cad-suite with security best practices
RUN cd /tools && \
    wget --quiet https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-x64-20241121.tgz && \
    tar -xf oss-cad-suite-linux-x64-20241121.tgz && \
    rm oss-cad-suite-linux-x64-20241121.tgz && \
    chmod -R 755 /tools/oss-cad-suite

RUN echo 'export PATH="/tools/oss-cad-suite/bin:$PATH"' >> /etc/bash.bashrc

# Install sv2v
RUN cd /tools && \
    wget --quiet https://github.com/zachjs/sv2v/releases/download/v0.0.12/sv2v-Linux.zip && \
    unzip -q sv2v-Linux.zip && \
    rm sv2v-Linux.zip && \
    chmod 755 /tools/sv2v

# Install Rust as vscode user (non-root)
USER vscode
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && \
    chmod -R 755 /home/vscode/.cargo
ENV PATH="/home/vscode/.cargo/bin:${PATH}"


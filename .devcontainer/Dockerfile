FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu

ENV LANG C.UTF-8
ARG INSTALL_ZSH="true"
ARG TARGETARCH

# Update package lists and install security updates + dependencies
RUN apt-get update && \
    apt-get upgrade -y && \
    apt-get install -y \
        wget \
        unzip \
        ca-certificates \
        curl \
        git \
        build-essential && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Create tools directory
RUN mkdir -p /tools && chmod 755 /tools

# Install oss-cad-suite (architecture-aware)
RUN cd /tools && \
    if [ "$TARGETARCH" = "amd64" ]; then \
        wget --quiet https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-x64-20241121.tgz && \
        tar -xf oss-cad-suite-linux-x64-20241121.tgz && \
        rm oss-cad-suite-linux-x64-20241121.tgz; \
    elif [ "$TARGETARCH" = "arm64" ]; then \
        wget --quiet https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-arm64-20241121.tgz && \
        tar -xf oss-cad-suite-linux-arm64-20241121.tgz && \
        rm oss-cad-suite-linux-arm64-20241121.tgz; \
    else \
        echo "Unsupported architecture: $TARGETARCH" && exit 1; \
    fi && \
    chmod -R 755 /tools/oss-cad-suite

# Install sv2v (x64 only - no ARM build available)
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        cd /tools && \
        wget --quiet https://github.com/zachjs/sv2v/releases/download/v0.0.12/sv2v-Linux.zip && \
        unzip -q sv2v-Linux.zip && \
        rm sv2v-Linux.zip && \
        chmod 755 /tools/sv2v; \
    else \
        echo "WARNING: sv2v not available for $TARGETARCH, skipping..."; \
    fi

# Add tools to PATH for all users
RUN echo 'export PATH="/tools/oss-cad-suite/bin:/tools:$PATH"' >> /etc/bash.bashrc && \
    echo 'export PATH="/tools/oss-cad-suite/bin:/tools:$PATH"' >> /etc/profile

# Install Rust as vscode user (Rust supports both architectures)
USER vscode
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y && \
    chmod -R 755 /home/vscode/.cargo

# Add Rust and tools to vscode user's PATH
RUN echo 'export PATH="/home/vscode/.cargo/bin:/tools/oss-cad-suite/bin:/tools:$PATH"' >> /home/vscode/.bashrc

# Switch back to root for VS Code server
USER root

ENV PATH="/home/vscode/.cargo/bin:/tools/oss-cad-suite/bin:/tools:${PATH}"

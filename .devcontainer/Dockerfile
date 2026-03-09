
FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu

ENV LANG C.UTF-8
ARG INSTALL_ZSH="true" 
ARG TARGETARCH

RUN apt-get update
RUN apt-get -y install \
    wget \
    unzip \
    curl \
    git \
    build-essential \
    libgmp-dev \
    default-jre \
    gnupg \
    ca-certificates \
    python3 \
    python3-pip

#install oss-cad-suite
RUN cd /opt && \
    wget https://github.com/YosysHQ/oss-cad-suite-build/releases/download/2024-11-21/oss-cad-suite-linux-x64-20241121.tgz && \
    tar -xf oss-cad-suite-linux-x64-20241121.tgz && \
    rm oss-cad-suite-linux-x64-20241121.tgz
    
ENV PATH="/opt/oss-cad-suite/bin:${PATH}"

# install sv2v
RUN if [ "$TARGETARCH" = "amd64" ]; then \
        echo "Installing prebuilt sv2v (amd64)"; \
        cd /opt && \
        wget https://github.com/zachjs/sv2v/releases/download/v0.0.13/sv2v-Linux.zip && \
        unzip sv2v-Linux.zip && \
        rm sv2v-Linux.zip; \
    else \
        echo "Building sv2v from source (arm64)"; \
        curl -sSL https://get.haskellstack.org/ | sh && \
        cd /opt && \
        git clone https://github.com/zachjs/sv2v.git && \
        cd sv2v && \
        make; \
    fi

ENV PATH="/opt/sv2v-Linux:${PATH}"
ENV PATH="/opt/sv2v/bin:${PATH}"

RUN python3 -m pip install --no-cache-dir --break-system-packages numpy opencv-python matplotlib cocotb pyserial

USER vscode

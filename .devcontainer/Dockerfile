
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
    default-jre

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

# SBT
RUN apt-get update && apt-get install -y \
    curl gnupg ca-certificates && \
    echo "deb https://repo.scala-sbt.org/scalasbt/debian all main" \
        > /etc/apt/sources.list.d/sbt.list && \
    echo "deb https://repo.scala-sbt.org/scalasbt/debian /" \
        > /etc/apt/sources.list.d/sbt_old.list && \
    curl -fsSL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x2EE0EA64E40A89B84B2DF73499E82A75642AC823" \
        | gpg --dearmor \
        | tee /etc/apt/trusted.gpg.d/scalasbt-release.gpg > /dev/null && \
    apt-get update && \
    apt-get install -y sbt && \
    rm -rf /var/lib/apt/lists/*


USER vscode

#install Rust (So far no tools need rust, so do we need it?)
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/home/vscode/.cargo/bin:${PATH}"

RUN curl -sSLf https://scala-cli.virtuslab.org/get | sh

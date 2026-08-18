ARG BASE_IMAGE=registry-01-registry.ap-southeast-1.cr.aliyuncs.com/robot/nx-ubuntu2404-x86@sha256:bd313d470a6bdacee057a920a56bb698fd888bcbe6281b8a498f4677fefe8e4e
FROM ${BASE_IMAGE}

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends clang-format \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# SPDX-FileCopyrightText: (C) 2021 - 2025 Intel Corporation
# SPDX-License-Identifier: LicenseRef-Intel-Edge-Software
# This file is licensed under the Limited Edge Software Distribution License Agreement.

ARG RUNTIME_OS_IMAGE=ubuntu:24.04

# -------------- Base Stage Specific to Controller --------------
FROM ubuntu:24.04 AS scenescape-controller-base

# We use root for runtime init. The command in ENTRYPOINT will drop to an unprivileged user.
# hadolint ignore=DL3002
USER root
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# install common build dependencies
RUN : \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        # Keep package list in alphabetical order
        cmake \
        curl \
        g++ \
        git \
        libeigen3-dev \
        libgtest-dev \
        make \
        # needed by fast_geometry
        pkg-config \
        pybind11-dev \
        python3-full \
        python3-pip \
        # needed by fast_geometry
        python3-scipy \
        python-is-python3 \
    && rm -rf /var/lib/apt/lists/*

# create and set up Python virtual environment
ENV BUILD_ENV_DIR=/tmp/venv
RUN : \
    && mkdir -p ${BUILD_ENV_DIR} \
    && python3 -m venv ${BUILD_ENV_DIR} \
    && ${BUILD_ENV_DIR}/bin/pip3 install --upgrade --no-cache-dir pip \
    && ${BUILD_ENV_DIR}/bin/pip3 install --no-cache-dir wheel setuptools

ENV PATH="${BUILD_ENV_DIR}/bin:${PATH}"

# install common dependencies
COPY ./scene_common /tmp/scene_common
RUN : \
    && cd /tmp/scene_common \
    && mv src scene_common \
    && pip3 install --no-cache-dir pybind11 \
    && pip3 install  --use-pep517 --no-cache-dir . \
    && make -C scene_common/fast_geometry -j $(nproc) all install \
    && cd .. \
    && rm -rf scene_common

COPY ./tools/waitforbroker /tmp/tools/waitforbroker


# -------------- Controller Builder Stage --------------
FROM scenescape-controller-base AS scenescape-controller-builder

USER root
ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN : \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        libopencv-dev \
        python3-venv \
    && rm -rf /var/lib/apt/lists/*

ENV BUILD_ENV_DIR=/tmp/venv
ENV PATH="${BUILD_ENV_DIR}/bin:${PATH}"

# Build robot vision package
COPY ./controller/src/robot_vision /tmp/robot_vision
RUN export OpenCV_DIR="/usr/lib/x86_64-linux-gnu/cmake/opencv4" \
    && cd /tmp/robot_vision \
    && python3 setup.py bdist_wheel \
    && cd dist \
    && ${BUILD_ENV_DIR}/bin/pip3 install --no-cache-dir ./*.whl \
    && cd \
   && rm -rf /tmp/robot_vision

# Build main controller package
COPY ./controller/src/controller /tmp/controller/controller
COPY ./controller/src/setup.py /tmp/controller/setup.py
RUN cd /tmp/controller \
    && python3 setup.py bdist_wheel \
    && ${BUILD_ENV_DIR}/bin/pip3 install --no-cache-dir ./dist/*.whl -vvv \
    && cd \
    && rm -rf /tmp/controller

# -------------- Controller Runtime Stage --------------
FROM ${RUNTIME_OS_IMAGE} AS scenescape-controller-runtime

ARG USER_ID
ARG CERTDOMAIN=scenescape.intel.com
ARG PYTHON_VERSION=3.12

ENV PYTHON_VERSION=${PYTHON_VERSION}
ENV WSUSER=scenescape
ENV SCENESCAPE_HOME=/home/$WSUSER/SceneScape
ENV BUILD_ENV_DIR=/tmp/venv
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONPERFSUPPORT=1
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

USER root

RUN : \
    && apt-get update \
    && apt-get install -y --no-install-recommends \
        libgl1 \
        libopencv-contrib-dev \
        python3-opencv \
        netbase \
        python-is-python3 \
        python3-pip \
        python3.12-venv \
        sudo \
        # for perf
        linux-tools-$(uname -r) \
    && rm -rf /usr/lib/x86_64-linux-gnu/libLLVM-15.so.1 \
    && rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*

RUN : \
    && useradd -r -m -s /bin/bash $WSUSER \
    && usermod -a -G video,users $WSUSER \
    && eval WSHOME=~$WSUSER \
    && chmod a+rX "${WSHOME}"

# TODO: uncomment when issue with accessing the secrets is resolved
# USER $WSUSER

# create and set up Python virtual environment
ENV RUNTIME_ENV_DIR=/tmp/rt-venv
RUN : \
    && mkdir -p ${RUNTIME_ENV_DIR} \
    && python3 -m venv ${RUNTIME_ENV_DIR} \
    && ${RUNTIME_ENV_DIR}/bin/pip3 install --upgrade --no-cache-dir pip setuptools
ENV PATH="${RUNTIME_ENV_DIR}/bin:${PATH}"

# install only required runtime dependencies
COPY controller/requirements-runtime.txt /tmp
RUN : \
    && pip3 install --upgrade --no-cache-dir -r /tmp/requirements-runtime.txt \
    && rm -rf /tmp/requirements-runtime.txt

# copy installed scenescape packages from the previous stages
COPY --chown=$WSUSER:$WSUSER --from=scenescape-controller-base /usr/local/lib/python${PYTHON_VERSION}/dist-packages/fast_geometry ${RUNTIME_ENV_DIR}/lib/python${PYTHON_VERSION}/site-packages/fast_geometry
COPY --chown=$WSUSER:$WSUSER --from=scenescape-controller-base ${BUILD_ENV_DIR}/lib/python${PYTHON_VERSION}/site-packages/scene_common ${RUNTIME_ENV_DIR}/lib/python${PYTHON_VERSION}/site-packages/scene_common
COPY --chown=$WSUSER:$WSUSER --from=scenescape-controller-builder ${BUILD_ENV_DIR}/lib/python${PYTHON_VERSION}/site-packages/controller ${RUNTIME_ENV_DIR}/lib/python${PYTHON_VERSION}/site-packages/controller
COPY --chown=$WSUSER:$WSUSER --from=scenescape-controller-builder ${BUILD_ENV_DIR}/lib/python${PYTHON_VERSION}/site-packages/robot_vision ${RUNTIME_ENV_DIR}/lib/python${PYTHON_VERSION}/site-packages/robot_vision

COPY --chown=$WSUSER:$WSUSER ./controller/src/schema/metadata.schema.json $SCENESCAPE_HOME/schema/metadata.schema.json
COPY --chown=$WSUSER:$WSUSER ./controller/config/tracker-config.json $SCENESCAPE_HOME/tracker-config.json
COPY --chown=$WSUSER:$WSUSER --from=scenescape-controller-base /tmp/tools/waitforbroker $SCENESCAPE_HOME/tools/waitforbroker
COPY --chown=$WSUSER:$WSUSER ./controller/src/controller-cmd $SCENESCAPE_HOME/controller-cmd
COPY --chown=$WSUSER:$WSUSER ./controller/src/controller-init /usr/local/bin/

ENTRYPOINT ["/usr/local/bin/controller-init"]

# ---------- Controller Test Stage ------------------
# This stage is meant to be used for test execution (not for final runtime)
FROM scenescape-controller-runtime AS scenescape-controller-test
ENV DEBIAN_FRONTEND=noninteractive

# Install Python test dependencies
RUN pip3 install --upgrade --no-cache-dir coverage pytest

RUN : \
    ; eval WSHOME=~$WSUSER \
    ;

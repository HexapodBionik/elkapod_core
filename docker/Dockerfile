ARG BASE_IMAGE=ghcr.io/hexapodbionik/elkapod-ros-jazzy-base:latest

# --------------------------------------------------------------- manifests --
# Collect just the package.xml files. COPY --from hashes the content it copies,
# so the expensive rosdep layer below stays cached whenever only sources change.
FROM ${BASE_IMAGE} AS manifests
COPY . /tmp/repo
RUN mkdir -p /tmp/manifests && cd /tmp/repo && \
    find . -name package.xml -exec cp --parents {} /tmp/manifests/ \;

# -------------------------------------------------------------------- deps --
FROM ${BASE_IMAGE} AS deps
SHELL ["/bin/bash", "-lc"]
ARG REPO_NAME=elkapod_core
ARG ROSDEP_SKIP_KEYS=""
WORKDIR /ws

COPY repos.yaml /ws/repos.yaml
RUN mkdir -p src && vcs import src < /ws/repos.yaml

RUN --mount=type=cache,target=/home/ubuntu/.cache/pip,uid=1000,gid=1000 \
    pip install --index-url https://download.pytorch.org/whl/cpu torch "numpy<2"

COPY --chown=ubuntu:ubuntu --from=manifests /tmp/manifests /ws/src/${REPO_NAME}

RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt/lists,sharing=locked \
    sudo apt-get update && \
    rosdep install --from-paths src --ignore-src -y \
        --rosdistro "${ROS_DISTRO}" \
        ${ROSDEP_SKIP_KEYS:+--skip-keys "${ROSDEP_SKIP_KEYS}"}

# ------------------------------------------------------------------- build --
FROM deps AS build
SHELL ["/bin/bash", "-lc"]
ARG REPO_NAME=elkapod_core
ARG COLCON_PACKAGES=""
WORKDIR /ws

COPY --chown=ubuntu:ubuntu . /ws/src/${REPO_NAME}

RUN --mount=type=cache,target=/home/ubuntu/.ccache,uid=1000,gid=1000 \
    source "/opt/ros/${ROS_DISTRO}/setup.bash" && \
    colcon build \
        --event-handlers console_direct+ \
        --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        ${COLCON_PACKAGES:+--packages-up-to ${COLCON_PACKAGES}}

COPY --chmod=755 docker/entrypoint.sh /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]

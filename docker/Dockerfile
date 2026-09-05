# GPU runtime for g1-simulacrum. Source is bind-mounted; do not copy the repo in.
# Third-party Python is baked at /opt/venv so the container is ready on first start.
FROM ghcr.io/astral-sh/uv:0.8.22 AS uv

FROM nvidia/cuda:12.6.3-base-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PYTHONDONTWRITEBYTECODE=1 \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics \
    UV_LINK_MODE=copy \
    UV_HTTP_TIMEOUT=300 \
    VIRTUAL_ENV=/opt/venv \
    PATH=/opt/venv/bin:/usr/local/bin:/usr/bin:/bin \
    PYTHONPATH=/workspace:/opt \
    MUJOCO_GL=egl \
    PYOPENGL_PLATFORM=egl

# Ubuntu 22.04 already ships Python 3.10. GL/EGL/GLFW for MuJoCo (EGL offscreen,
# GLFW on host Intel X — do not force NVIDIA GLX).
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3.10 \
        python3.10-venv \
        python3.10-dev \
        python3-pip \
        build-essential \
        git \
        curl \
        ca-certificates \
        pkg-config \
        patchelf \
        libglvnd0 libgl1 libglx0 libegl1 libgles2 \
        libglib2.0-0 \
        libgomp1 \
        libosmesa6 \
        libglfw3 \
        libglew2.2 \
        libx11-6 \
        libxext6 \
        libsm6 \
        libxrender1 \
        libxrandr2 \
        libxi6 \
    && rm -rf /var/lib/apt/lists/* \
    && update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.10 1 \
    && update-alternatives --install /usr/bin/python python /usr/bin/python3.10 1

COPY --from=uv /uv /uvx /usr/local/bin/

RUN uv venv --python 3.10 /opt/venv

# Stub package so hatchling can resolve pyproject extras without copying source
# (bind-mount + PYTHONPATH=/workspace provides the real import at runtime).
WORKDIR /tmp/pkg
COPY pyproject.toml README.md ./
RUN mkdir -p g1_simulacrum \
    && printf '%s\n' '__version__ = "0.1.0"' > g1_simulacrum/__init__.py

RUN --mount=type=cache,target=/root/.cache/uv \
    UV_HTTP_TIMEOUT=900 UV_CONCURRENT_DOWNLOADS=2 \
    uv pip install --python /opt/venv/bin/python ".[dev]"

# Menagerie is not a PyPI package. Import path: mujoco_menagerie (PYTHONPATH=/opt).
RUN git clone --depth 1 https://github.com/google-deepmind/mujoco_menagerie.git \
        /opt/mujoco_menagerie \
    && touch /opt/mujoco_menagerie/__init__.py

RUN chmod -R a+rX /opt/venv /opt/mujoco_menagerie

# GL: libOpenGL.so.0 is GLVND dispatch. NVIDIA EGL ICD for offscreen pixels.
# A GLX ICD is written then removed so GLFW windows stay on Mesa (host X is Intel).
RUN apt-get update && apt-get install -y --no-install-recommends libopengl0 \
    && rm -rf /var/lib/apt/lists/* \
    && mkdir -p /usr/share/glvnd/egl_vendor.d /usr/share/glvnd/glx_vendor.d \
    && printf '%s\n' '{"file_format_version":"1.0.0","ICD":{"library_path":"libEGL_nvidia.so.0"}}' \
         > /usr/share/glvnd/egl_vendor.d/10_nvidia.json \
    && printf '%s\n' '{"file_format_version":"1.0.0","ICD":{"library_path":"libGLX_nvidia.so.0"}}' \
         > /usr/share/glvnd/glx_vendor.d/10_nvidia.json \
    && rm -f /usr/share/glvnd/glx_vendor.d/10_nvidia.json

COPY docker/entrypoint.sh /opt/g1/entrypoint.sh
RUN chmod +x /opt/g1/entrypoint.sh

WORKDIR /workspace

# NVIDIA Warp for mujoco-lidar. Own layer so gym/dev + menagerie stay cached.
RUN --mount=type=cache,target=/root/.cache/uv \
    UV_HTTP_TIMEOUT=900 \
    uv pip install --python /opt/venv/bin/python "mujoco-lidar[warp]" \
    && chmod -R a+rX /opt/venv

ENTRYPOINT ["/opt/g1/entrypoint.sh"]
CMD ["bash"]

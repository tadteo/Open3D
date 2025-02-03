#!/usr/bin/env bash
# Use: install_deps_fedora.sh [ assume-yes ] [ no-filament-deps ]

set -ev

SUDO=${SUDO:=sudo} # SUDO=command in docker (running as root, sudo not available)
options="$(echo "$@" | tr ' ' '|')"
DNF_CONFIRM="-y"
if [[ "assume-yes" =~ ^($options)$ ]]; then
    DNF_CONFIRM="-y"
fi
FILAMENT_DEPS="yes"
if [[ "no-filament-deps" =~ ^($options)$ ]]; then
    FILAMENT_DEPS=""
fi

deps=(
    git
    # Open3D
    xorg-x11-server-devel
    libxcb
    mesa-libGLU-devel
    python3-devel
    # filament linking
    libcxx-devel
    libcxxabi-devel
    SDL2-devel
    libXi-devel
    # ML
    tbb-devel
    # Headless rendering
    mesa-libOSMesa-devel
    # RealSense
    systemd-devel
    autoconf
    libtool
)

if [[ "$FILAMENT_DEPS" ]]; then     # Filament build-from-source
    deps+=(
        clang
        ninja-build
    )
fi

# Special case for ARM64
if [ "$(uname -m)" == "aarch64" ]; then
    # For compiling LAPACK in OpenBLAS
    deps+=("gcc-gfortran")
fi

echo "dnf install ${deps[*]}"
$SUDO dnf update -y
$SUDO dnf install ${DNF_CONFIRM} ${deps[*]}

#!/bin/bash

set -euo pipefail

# Run this script ON the Jetson.
# It installs SSH key material from docker/files/jetson_dot_ssh.tar.bz2
# and installs ROS 2 Humble + common build tooling locally.

SCRIPT_DIR="$(cd -P "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
SSH_ARCHIVE_PATH="$SCRIPT_DIR/../docker/files/jetson_dot_ssh.tar.bz2"
ROS_DISTRO="humble"

usage() {
    cat <<EOF
Usage: $(basename "$0") [options]

Options:
  -a, --archive <tar.bz2-path>    Path to jetson_dot_ssh.tar.bz2
  -h, --help                      Show this help
EOF
}

print_info() {
    echo "[INFO] $*"
}

print_warn() {
    echo "[WARN] $*"
}

print_error() {
    echo "[ERROR] $*" >&2
}

while [[ $# -gt 0 ]]; do
    case "$1" in
        -a|--archive)
            [[ $# -lt 2 ]] && { print_error "Missing value for $1"; usage; exit 1; }
            SSH_ARCHIVE_PATH="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            print_error "Unknown argument: $1"
            usage
            exit 1
            ;;
    esac
done

if [[ $EUID -eq 0 ]]; then
    print_error "Run this script as a normal user (with sudo access), not root."
    exit 1
fi

if ! command -v tar >/dev/null 2>&1; then
    print_error "tar is required."
    exit 1
fi

if [[ ! -f "$SSH_ARCHIVE_PATH" ]]; then
    print_error "SSH archive not found: $SSH_ARCHIVE_PATH"
    exit 1
fi

source /etc/os-release
if [[ "${ID}" != "ubuntu" ]]; then
    print_error "Expected Ubuntu. Found ID=${ID}."
    exit 1
fi

if [[ "${VERSION_CODENAME}" != "jammy" ]]; then
    print_error "ROS 2 Humble expects Ubuntu 22.04 (jammy). Found ${VERSION_CODENAME}."
    exit 1
fi

TMP_DIR="$(mktemp -d)"
cleanup() {
    [[ -d "$TMP_DIR" ]] && rm -rf "$TMP_DIR"
}
trap cleanup EXIT

print_info "Extracting SSH files from $SSH_ARCHIVE_PATH"
tar -xjf "$SSH_ARCHIVE_PATH" -C "$TMP_DIR"

mkdir -p "$HOME/.ssh"
chmod 700 "$HOME/.ssh"

for f in id_ed25519 id_ed25519.pub id_rsa id_rsa.pub config authorized_keys; do
    if [[ -f "$TMP_DIR/$f" ]]; then
        if [[ -f "$HOME/.ssh/$f" ]]; then
            cp "$HOME/.ssh/$f" "$HOME/.ssh/$f.bak.$(date +%s)"
            print_warn "Backed up existing ~/.ssh/$f"
        fi
        cp "$TMP_DIR/$f" "$HOME/.ssh/$f"
    fi
done

[[ -f "$HOME/.ssh/id_ed25519" ]] && chmod 600 "$HOME/.ssh/id_ed25519"
[[ -f "$HOME/.ssh/id_rsa" ]] && chmod 600 "$HOME/.ssh/id_rsa"
[[ -f "$HOME/.ssh/id_ed25519.pub" ]] && chmod 644 "$HOME/.ssh/id_ed25519.pub"
[[ -f "$HOME/.ssh/id_rsa.pub" ]] && chmod 644 "$HOME/.ssh/id_rsa.pub"
[[ -f "$HOME/.ssh/config" ]] && chmod 600 "$HOME/.ssh/config"
[[ -f "$HOME/.ssh/authorized_keys" ]] && chmod 600 "$HOME/.ssh/authorized_keys"

print_info "Installing ROS 2 $ROS_DISTRO prerequisites"
sudo apt update
sudo apt install -y locales software-properties-common curl gnupg lsb-release ca-certificates
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

sudo add-apt-repository universe -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | \
    sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null

sudo apt update
sudo apt install -y \
    ros-humble-ros-base \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    build-essential \
    terminator \
    git \
    python3-pip

if ! grep -q "source /opt/ros/humble/setup.bash" "$HOME/.bashrc"; then
    echo "source /opt/ros/humble/setup.bash" >> "$HOME/.bashrc"
fi

if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
    sudo rosdep init
fi
rosdep update

print_info "Local Jetson setup complete."

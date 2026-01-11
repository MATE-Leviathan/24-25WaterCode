# 24-25WaterCode & HoloOcean ROS Setup Guide

This guide covers the complete setup process for the 24-25WaterCode and holoocean-ros projects on Ubuntu with NVIDIA GPU support.

## Overview

This setup uses:
- **uv** - Modern Python package manager for 24-25WaterCode dependencies
- **Virtual Environment** - For both 24-25WaterCode and HoloOcean Python packages
- **Docker** - For running ROS 2 and simulation environments

## Prerequisites

Update your system:
```bash
sudo apt update
sudo apt upgrade
```

Install basic utilities:
```bash
sudo apt install ca-certificates curl gnupg2 git
```

## 1. SSH Key Setup for GitHub

Generate an SSH key for GitHub access:
```bash
ssh-keygen -t ed25519 -C "your-email@example.com"
```

Display your public key:
```bash
cat ~/.ssh/id_ed25519.pub
```

Copy the output and add it to your GitHub account at: https://github.com/settings/keys

## 2. Install Docker

### Add Docker's official GPG key:
```bash
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
```

### Add Docker repository:
```bash
sudo tee /etc/apt/sources.list.d/docker.sources <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF
```

### Install Docker:
```bash
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### Add your user to the docker group (to run docker without sudo):
```bash
sudo usermod -aG docker $USER
```

**Important:** Log out and log back in for group changes to take effect, or run:
```bash
newgrp docker
```

Verify Docker installation:
```bash
docker --version
groups  # Should show 'docker' in the list
```

## 3. Install NVIDIA Container Toolkit (for GPU support)

### Add NVIDIA Container Toolkit repository:
```bash
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

### Enable experimental packages (optional):
```bash
sudo sed -i -e '/experimental/ s/^#//g' /etc/apt/sources.list.d/nvidia-container-toolkit.list
```

### Install NVIDIA Container Toolkit:
```bash
sudo apt update
sudo apt install nvidia-container-toolkit
```

### Configure Docker to use NVIDIA runtime:
```bash
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### Verify NVIDIA Docker support:
```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

## 4. Install uv (Modern Python Package Manager)

Install `uv` for fast Python package management:
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

After installation, restart your terminal or run:
```bash
source $HOME/.cargo/env
```

Verify installation:
```bash
uv --version
```

## 5. Clone Repositories

Navigate to your home directory:
```bash
cd ~
```

Clone the repositories:
```bash
git clone git@github.com:MATE-Leviathan/24-25WaterCode.git
git clone git@github.com:MATE-Leviathan/holoocean-ros.git
```

## 6. Set Up 24-25WaterCode Virtual Environment

Navigate to the 24-25WaterCode directory and create the virtual environment:
```bash
cd ~/24-25WaterCode
uv sync
```

This will create a `.venv` directory with all Python dependencies.

## 7. Install HoloOcean (Required for Simulation)

**IMPORTANT:** HoloOcean must be installed in `~/HoloOcean` for the Docker containers to work properly.

### Prerequisites
Link your GitHub account with Unreal Engine (required by Unreal Engine EULA):
1. Go to https://www.unrealengine.com/en-US/ue-on-github
2. Link your GitHub account
3. Accept the Unreal Engine EULA

### Clone and Install HoloOcean
**Note:** Install HoloOcean inside the 24-25WaterCode virtual environment using `uv pip`.

```bash
cd ~
git clone https://github.com/byu-holoocean/HoloOcean.git

# Make sure you're in the 24-25WaterCode directory (uv automatically uses .venv)
cd ~/24-25WaterCode

# Install HoloOcean using uv pip
uv pip install -e ~/HoloOcean/client
```

**Note:** `uv` creates minimal venvs without pip by default. Use `uv pip` instead of `pip3` for all package installations.

### Install World Packages
After installing the client, install the Ocean world package:
```bash
cd ~/24-25WaterCode
uv run python -c "import holoocean; holoocean.install('Ocean')"
```

This will download simulation worlds and assets to `~/.local/share/holoocean/`.

**Note:** Use `uv run python` to run Python commands with the venv activated automatically.

For more details, see: https://byu-holoocean.github.io/holoocean-docs/v2.2.2/usage/installation.html

## 8. Quick Installation Script (Optional)

For a complete automated setup, you can run:
```bash
cd ~/24-25WaterCode/docker
./install.sh
```

This script will check and install all prerequisites including Docker, NVIDIA Container Toolkit, and HoloOcean.

**Note:** If you followed steps 1-7 manually, you can skip this and go directly to step 9.

## 9. Run Docker Setup Scripts

Navigate to the docker directory:
```bash
cd ~/24-25WaterCode/docker
```

### Option A: Run 24-25WaterCode only
```bash
./setup.sh
```

### Option B: Run 24-25WaterCode + HoloOcean with NVIDIA GPU
```bash
./setup.sh -H
```

### Option C: Run 24-25WaterCode + HoloOcean without NVIDIA
```bash
./setup.sh -N
```

### View help:
```bash
./setup.sh -h
```

## 10. Managing Docker Containers

### Enter a running container:

Enter the Leviathan container (24-25WaterCode):
```bash
./enter.sh
```

Enter the HoloOcean container (if running with -H option):
```bash
./enter.sh -H
```

### Stop containers:
```bash
./stop.sh
```

### View running containers:
```bash
docker ps
```

### View container logs:
```bash
docker compose logs -f
```

## Troubleshooting

### HoloOcean directory owned by root
If `holoocean.install('Ocean')` fails with permission errors, the HoloOcean directory may have been created by root:
```bash
# Check ownership
ls -la ~/.local/share/holoocean

# If owned by root, remove and recreate
sudo rm -rf ~/.local/share/holoocean
mkdir -p ~/.local/share/holoocean

# Run install again
cd ~/24-25WaterCode
uv run python -c "import holoocean; holoocean.install('Ocean')"
```

**Note:** This can happen if you previously ran HoloOcean commands with sudo. Always use regular user permissions.

### Docker permission denied
If you get permission errors, make sure you:
1. Added your user to the docker group: `sudo usermod -aG docker $USER`
2. Logged out and back in
3. Verify with: `groups` (should show 'docker')

### NVIDIA GPU not detected
1. Check if NVIDIA drivers are installed: `nvidia-smi`
2. Verify NVIDIA Docker runtime: `docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi`
3. Check Docker daemon configuration: `cat /etc/docker/daemon.json`

### Network timeout issues
If you experience timeout issues when downloading packages:
- Check your internet connection
- Try again later
- Use a different DNS server

## Quick Reference: Complete Setup Workflow

If this is your first time setting up, follow these steps in order:

```bash
# 1. Install system dependencies
sudo apt update && sudo apt upgrade
sudo apt install ca-certificates curl gnupg2 git python3-pip

# 2. Set up SSH key for GitHub
ssh-keygen -t ed25519 -C "your-email@example.com"
cat ~/.ssh/id_ed25519.pub  # Add this to GitHub

# 3. Install Docker
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc
# ... (follow Docker installation steps above)

# 4. Install NVIDIA Container Toolkit
sudo apt install nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker

# 5. Install uv
curl -LsSf https://astral.sh/uv/install.sh | sh
source $HOME/.cargo/env

# 6. Clone repositories
cd ~
git clone git@github.com:MATE-Leviathan/24-25WaterCode.git
git clone git@github.com:MATE-Leviathan/holoocean-ros.git
git clone https://github.com/byu-holoocean/HoloOcean.git

# 7. Set up 24-25WaterCode (creates .venv)
cd ~/24-25WaterCode
uv sync

# 8. Install HoloOcean (in the venv using uv pip)
uv pip install -e ~/HoloOcean/client
uv run python -c "import holoocean; holoocean.install('Ocean')"

# 9. Run Docker setup
cd ~/24-25WaterCode/docker
./setup.sh -H  # With GPU support

# 10. Enter container
./enter.sh -H  # For HoloOcean
```

## Additional Resources

- Docker Documentation: https://docs.docker.com/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/
- Project Repository: https://github.com/MATE-Leviathan/24-25WaterCode
- HoloOcean Documentation: https://byu-holoocean.github.io/holoocean-docs/
- uv Documentation: https://docs.astral.sh/uv/

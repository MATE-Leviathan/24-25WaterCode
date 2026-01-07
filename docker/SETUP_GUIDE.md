# 24-25WaterCode & HoloOcean ROS Setup Guide

This guide covers the complete setup process for the 24-25WaterCode and holoocean-ros projects on Ubuntu with NVIDIA GPU support.

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

## 4. Clone Repositories

Navigate to your code directory:
```bash
mkdir -p ~/code
cd ~/code
```

Clone the repositories:
```bash
git clone git@github.com:MATE-Leviathan/24-25WaterCode.git
git clone git@github.com:MATE-Leviathan/holoocean-ros.git
```

## 5. Run Setup Scripts

Navigate to the docker directory:
```bash
cd ~/code/24-25WaterCode/docker
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

## 6. Managing Docker Containers

### Enter a running container:
```bash
./enter.sh
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

## Additional Resources

- Docker Documentation: https://docs.docker.com/
- NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/
- Project Repository: https://github.com/MATE-Leviathan/24-25WaterCode

#!/bin/bash

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Helper functions
print_header() {
    echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
    echo -e "${BLUE}  $1${NC}"
    echo -e "${BLUE}═══════════════════════════════════════════════════════════${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# Check if running as root
if [ "$EUID" -eq 0 ]; then
    print_error "Please do not run this script as root or with sudo"
    exit 1
fi

print_header "24-25WaterCode & HoloOcean Installation Script"
echo ""

# ============================================================================
# 1. Check and install basic utilities
# ============================================================================
print_header "Step 1: Installing Basic Utilities"
echo ""

print_info "Updating package lists..."
sudo apt update -qq

PACKAGES="ca-certificates curl gnupg2 git python3-pip python3-venv"
print_info "Installing: $PACKAGES"
sudo apt install -y $PACKAGES > /dev/null 2>&1
print_success "Basic utilities installed"
echo ""

# ============================================================================
# 2. Check and install Docker
# ============================================================================
print_header "Step 2: Docker Installation"
echo ""

if command -v docker &> /dev/null; then
    print_success "Docker is already installed ($(docker --version))"
else
    print_info "Installing Docker..."

    # Add Docker's GPG key
    sudo install -m 0755 -d /etc/apt/keyrings
    sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add Docker repository
    sudo tee /etc/apt/sources.list.d/docker.sources <<EOF > /dev/null
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Signed-By: /etc/apt/keyrings/docker.asc
EOF

    # Install Docker
    sudo apt update -qq
    sudo apt install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin > /dev/null 2>&1

    print_success "Docker installed successfully"
fi

# Add user to docker group
if groups $USER | grep -q '\bdocker\b'; then
    print_success "User $USER is already in the docker group"
else
    print_info "Adding user $USER to docker group..."
    sudo usermod -aG docker $USER
    print_warning "You need to log out and log back in for docker group changes to take effect!"
    print_warning "After logging back in, run this script again to continue."
    exit 0
fi
echo ""

# ============================================================================
# 3. Check and install NVIDIA Container Toolkit (optional)
# ============================================================================
print_header "Step 3: NVIDIA Container Toolkit (Optional)"
echo ""

if command -v nvidia-smi &> /dev/null; then
    print_success "NVIDIA drivers detected"

    if command -v nvidia-ctk &> /dev/null; then
        print_success "NVIDIA Container Toolkit is already installed"
    else
        print_info "Installing NVIDIA Container Toolkit..."

        # Add NVIDIA repository
        curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
            sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

        curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
            sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
            sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list > /dev/null

        sudo apt update -qq
        sudo apt install -y nvidia-container-toolkit > /dev/null 2>&1

        # Configure Docker
        sudo nvidia-ctk runtime configure --runtime=docker > /dev/null 2>&1
        sudo systemctl restart docker

        print_success "NVIDIA Container Toolkit installed successfully"
    fi
else
    print_warning "NVIDIA drivers not detected. Skipping NVIDIA Container Toolkit installation."
    print_info "You can still run the containers without GPU support using the -N flag."
fi
echo ""

# ============================================================================
# 4. Clone repositories
# ============================================================================
print_header "Step 4: Cloning Repositories"
echo ""

CODE_DIR="$HOME"
mkdir -p "$CODE_DIR"

# Clone 24-25WaterCode
if [ -d "$CODE_DIR/24-25WaterCode" ]; then
    print_success "24-25WaterCode repository already exists"
else
    print_info "Cloning 24-25WaterCode..."
    git clone git@github.com:MATE-Leviathan/24-25WaterCode.git "$CODE_DIR/24-25WaterCode"
    print_success "24-25WaterCode cloned successfully"
fi

# Clone holoocean-ros
if [ -d "$CODE_DIR/holoocean-ros" ]; then
    print_success "holoocean-ros repository already exists"
else
    print_info "Cloning holoocean-ros..."
    git clone git@github.com:MATE-Leviathan/holoocean-ros.git "$CODE_DIR/holoocean-ros"
    print_success "holoocean-ros cloned successfully"
fi
echo ""

# ============================================================================
# 5. Install HoloOcean
# ============================================================================
print_header "Step 5: Installing HoloOcean"
echo ""

print_warning "IMPORTANT: You must link your GitHub account with Unreal Engine!"
print_info "1. Go to: https://www.unrealengine.com/en-US/ue-on-github"
print_info "2. Link your GitHub account"
print_info "3. Accept the Unreal Engine EULA"
echo ""
read -p "Have you completed the above steps? (y/n) " -n 1 -r
echo ""
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    print_error "Please complete the Unreal Engine GitHub linking before continuing."
    exit 1
fi

# Clone HoloOcean
if [ -d "$CODE_DIR/HoloOcean" ]; then
    print_success "HoloOcean repository already exists at $CODE_DIR/HoloOcean"
else
    print_info "Cloning HoloOcean..."
    cd "$CODE_DIR"
    git clone https://github.com/byu-holoocean/HoloOcean.git
    print_success "HoloOcean cloned successfully"
fi

# Install HoloOcean client
print_info "Installing HoloOcean Python client..."
cd "$CODE_DIR/HoloOcean/client"
pip3 install -e . > /dev/null 2>&1
print_success "HoloOcean client installed"

# Install Ocean world package
print_info "Installing Ocean world package (this may take a while)..."
python3 -c "import holoocean; holoocean.install('Ocean')"
print_success "Ocean world package installed to ~/.local/share/holoocean/"
echo ""

# ============================================================================
# Complete!
# ============================================================================
print_header "Installation Complete!"
echo ""
print_success "All components have been installed successfully!"
echo ""
print_info "Next steps:"
echo "  1. Navigate to the docker directory:"
echo "     cd $CODE_DIR/24-25WaterCode/docker"
echo ""
echo "  2. Run the setup script:"
echo "     ./setup.sh          # For 24-25WaterCode only"
echo "     ./setup.sh -H       # For 24-25WaterCode + HoloOcean with NVIDIA GPU"
echo "     ./setup.sh -N       # For 24-25WaterCode + HoloOcean without NVIDIA"
echo ""
print_info "For more information, see: SETUP_GUIDE.md"
echo ""

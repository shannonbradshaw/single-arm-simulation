#!/bin/bash
set -e
exec > /var/log/startup-script.log 2>&1

# Skip if already provisioned
if [ -f /opt/.provisioned ]; then exit 0; fi

echo "=== Installing NVIDIA drivers ==="
apt-get update
apt-get install -y linux-headers-$(uname -r)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed "s#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g" | \
  tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
apt-get update
apt-get install -y nvidia-driver-535 nvidia-container-toolkit

echo "=== Installing Docker ==="
curl -fsSL https://get.docker.com | sh
systemctl enable docker
systemctl start docker

echo "=== Configuring NVIDIA container runtime ==="
nvidia-ctk runtime configure --runtime=docker
systemctl restart docker

touch /opt/.provisioned
echo "=== Provisioning complete, rebooting ==="
reboot

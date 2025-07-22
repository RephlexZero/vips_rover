#!/bin/bash

# One-time system setup for rover hardware
echo "🛠️  Rover System Setup"
echo "===================="
echo ""
echo "This will configure your system for rover hardware operation:"
echo "  • CAN interface permissions (no sudo required)"
echo "  • Automatic CAN interface setup on boot"
echo "  • Install required utilities"
echo ""
echo "You'll need to log out and back in after this setup."
echo ""

read -p "Continue? (y/N): " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Setup cancelled."
    exit 1
fi

echo ""
echo "📋 Installing system dependencies..."
sudo apt update && sudo apt install -y can-utils

echo ""
echo "👥 Adding user to dialout group..."
sudo usermod -a -G dialout $USER

echo ""
echo "📁 Installing system configuration files..."

# Install udev rules
if [ ! -f /etc/udev/rules.d/99-can.rules ]; then
    echo "  • Installing CAN interface permissions..."
    sudo cp ~/ros2_ws/setup/99-can.rules /etc/udev/rules.d/
    sudo udevadm control --reload-rules
else
    echo "  • CAN interface permissions already configured"
fi

# Install systemd service
if [ ! -f /etc/systemd/system/can-setup.service ]; then
    echo "  • Installing CAN auto-setup service..."
    sudo cp ~/ros2_ws/setup/can-setup.service /etc/systemd/system/
    sudo systemctl enable can-setup.service
else
    echo "  • CAN auto-setup service already configured"
fi

echo ""
echo "✅ Setup complete!"
echo ""
echo "⚠️  IMPORTANT: Log out and log back in (or reboot) for changes to take effect."
echo ""
echo "After reboot, you can:"
echo "  • Run simulation: ./scripts/sim"
echo "  • Run real hardware: ./scripts/real"
echo "  • Check CAN status: ip link show can0"
echo ""

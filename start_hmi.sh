#!/bin/bash

##############################################################################
# start_motor_hmi.sh - Motor Control HMI Server
# 
# Starts the web HMI server only
##############################################################################

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

echo -e "${BLUE}🎮 Motor Control HMI Server${NC}"
echo ""

# Check if HMI server exists
if [ ! -f "./hmi_server.py" ]; then
    echo -e "${RED}❌ HMI server not found!${NC}"
    echo "Please ensure hmi_server.py is in the current directory."
    exit 1
fi

# Check Python dependencies
echo "Checking Python dependencies..."
python3 -c "import asyncio, websockets, json" 2>/dev/null
if [ $? -ne 0 ]; then
    echo -e "${YELLOW}⚠ Installing missing Python packages...${NC}"
    pip3 install websockets
fi

# Get network info
WLAN0_IP=$(ifconfig wlan0 2>/dev/null | grep -o 'inet [0-9.]*' | cut -d' ' -f2)
UAP0_IP=$(ifconfig uap0 2>/dev/null | grep -o 'inet [0-9.]*' | cut -d' ' -f2)
HMI_PORT=8080

echo -e "${GREEN}✓ System ready${NC}"
echo ""
echo "Network Information:"
if [ ! -z "$WLAN0_IP" ]; then
    echo "  wlan0 (Internet): $WLAN0_IP"
fi
if [ ! -z "$UAP0_IP" ]; then
    echo "  uap0 (Access Point): $UAP0_IP"
fi
echo "  HMI Web Port: $HMI_PORT"
echo "  WebSocket Port: $((HMI_PORT + 1))"
echo "  UDP Broadcast Port: 9999"
echo ""

# Start HMI server in background
echo -e "${BLUE}Starting HMI server...${NC}"
python3 ./hmi_server.py --port $HMI_PORT --udp-port 9999 --csv-path "/home/foxtrot/log/motor_data" &
HMI_PID=$!

# Wait a moment for server to start
sleep 2

# Check if HMI server started successfully
if kill -0 $HMI_PID 2>/dev/null; then
    echo -e "${GREEN}✓ HMI server started (PID: $HMI_PID)${NC}"
else
    echo -e "${RED}❌ Failed to start HMI server${NC}"
    exit 1
fi

echo ""
echo -e "${GREEN}🌐 Web HMI is ready!${NC}"
echo ""
echo "Access from any device on your network:"
if [ ! -z "$WLAN0_IP" ]; then
    echo "  � Internet Network (wlan0): http://$WLAN0_IP:$HMI_PORT"
fi
if [ ! -z "$UAP0_IP" ]; then
    echo "  � Access Point (uap0): http://$UAP0_IP:$HMI_PORT"
fi
echo "  🏠 Local: http://localhost:$HMI_PORT"
echo ""
echo -e "${BLUE}HMI server is running...${NC}"
echo "Waiting for motor control data on UDP port 9999"
echo "  - Listening on all network interfaces"
echo "  - Broadcasting supported on both wlan0 and uap0"
echo ""
echo "Press Ctrl+C to stop the server"
echo ""

# Function to cleanup on exit
cleanup() {
    echo ""
    echo -e "${YELLOW}Stopping HMI server...${NC}"
    
    # Kill HMI server
    if kill -0 $HMI_PID 2>/dev/null; then
        kill $HMI_PID
        echo "✓ HMI server stopped"
    fi
    
    exit 0
}

# Set up signal handling
trap cleanup SIGINT SIGTERM

# Wait for the HMI server process
wait $HMI_PID
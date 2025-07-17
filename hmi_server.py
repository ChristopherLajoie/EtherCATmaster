#!/usr/bin/env python3
"""
Motor Control Real-time HMI Server with CSV Export, Current Display, and Index Temperature

Receives UDP broadcasts from motor control system and serves
real-time data to web browsers via WebSockets, plus CSV file downloads.

Usage:
    python motor_hmi_server.py
    python motor_hmi_server.py --port 8080 --udp-port 9999 --csv-path /home/foxtrot/log/motor_data
"""

import asyncio
import websockets
import json
import socket
import threading
import time
import os
import glob
from datetime import datetime
from collections import deque
import argparse
from pathlib import Path
import http.server
import socketserver
import urllib.parse
import mimetypes


class MotorHMIServer:
    def __init__(self, web_port=8080, udp_port=9999, udp_ip="192.168.1.255", csv_path="/home/foxtrot/log/motor_data"):
        self.web_port = web_port
        self.udp_port = udp_port
        self.udp_ip = udp_ip
        self.csv_path = csv_path

        # Store connected WebSocket clients
        self.clients = set()

        # Data buffers (keep 1 minute at 10Hz = 600 points)
        self.max_history = 600
        self.data_history = deque(maxlen=self.max_history)

        # Latest data for new connections
        self.latest_data = None

        # UDP sockets for multiple interfaces
        self.udp_sockets = []
        self.running = False

        # Store the event loop for cross-thread communication
        self.loop = None

        # Get network interfaces
        self.network_interfaces = self._get_network_interfaces()

        print(f"Motor HMI Server with CSV Export, Current Display, and Index Temperature")
        print(f"Web interface: http://localhost:{web_port}")
        print(f"UDP listener: port {udp_port}")
        print(f"CSV files: {csv_path}")
        print(
            f"Network interfaces detected: {', '.join([f'{iface}({ip})' for iface, ip in self.network_interfaces.items()])}")

    def _get_network_interfaces(self):
        """Get available network interfaces and their IP addresses"""
        import subprocess
        import re

        interfaces = {}
        try:
            # Run ifconfig to get interface information
            result = subprocess.run(
                ['ifconfig'], capture_output=True, text=True)
            if result.returncode == 0:
                # Parse ifconfig output
                current_interface = None
                for line in result.stdout.split('\n'):
                    # Look for interface names
                    if re.match(r'^[a-zA-Z0-9]+:', line):
                        current_interface = line.split(':')[0]
                    # Look for inet addresses
                    elif current_interface and 'inet ' in line and '127.0.0.1' not in line:
                        match = re.search(r'inet\s+(\d+\.\d+\.\d+\.\d+)', line)
                        if match:
                            ip = match.group(1)
                            interfaces[current_interface] = ip

        except Exception as e:
            print(f"Warning: Could not detect network interfaces: {e}")

        return interfaces

    def start_udp_listener(self):
        """Start UDP listener in background thread for all interfaces"""
        try:
            # Create UDP socket that listens on all interfaces
            self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.udp_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.udp_socket.setsockopt(
                socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

            # Bind to all interfaces
            self.udp_socket.bind(('', self.udp_port))
            self.udp_socket.settimeout(1.0)

            print(
                f"✓ UDP listener started on port {self.udp_port} (all interfaces)")
            print(f"  Listening for broadcasts on:")
            for iface, ip in self.network_interfaces.items():
                print(f"    {iface}: {ip}")

            while self.running:
                try:
                    data, addr = self.udp_socket.recvfrom(1024)
                    print(f"Received UDP data from {addr[0]}:{addr[1]}")
                    self.handle_motor_data(data.decode('utf-8'))
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"UDP receive error: {e}")

        except Exception as e:
            print(f"Failed to start UDP listener: {e}")
        finally:
            if self.udp_socket:
                self.udp_socket.close()
                print("UDP listener stopped")

    def handle_motor_data(self, json_data):
        """Process incoming motor data"""
        try:
            data = json.loads(json_data)
            data['server_timestamp'] = datetime.now().isoformat()

            self.data_history.append(data)
            self.latest_data = data

            if self.clients and self.loop:
                message = json.dumps(data)
                asyncio.run_coroutine_threadsafe(
                    self.broadcast_to_clients(message),
                    self.loop
                )

        except json.JSONDecodeError as e:
            print(f"Invalid JSON received: {e}")
        except Exception as e:
            print(f"Error processing motor data: {e}")

    async def broadcast_to_clients(self, message):
        """Send message to all connected WebSocket clients"""
        if not self.clients:
            return

        disconnected = set()
        for client in self.clients.copy():
            try:
                await client.send(message)
            except websockets.exceptions.ConnectionClosed:
                disconnected.add(client)
            except Exception as e:
                print(f"Error sending to client: {e}")
                disconnected.add(client)

        self.clients -= disconnected

    async def websocket_handler(self, websocket):
        """Handle new WebSocket connection"""
        print(f"New client connected from {websocket.remote_address}")
        self.clients.add(websocket)

        try:
            if self.data_history:
                history_message = {
                    "type": "history",
                    "data": list(self.data_history)
                }
                await websocket.send(json.dumps(history_message))

            if self.latest_data:
                await websocket.send(json.dumps(self.latest_data))

            async for message in websocket:
                try:
                    client_data = json.loads(message)
                    print(f"Client message: {client_data}")
                except:
                    pass

        except websockets.exceptions.ConnectionClosed:
            pass
        except Exception as e:
            print(f"WebSocket error: {e}")
        finally:
            self.clients.discard(websocket)
            print(f"Client disconnected")

    def get_csv_files(self):
        """Get list of available CSV files"""
        if not os.path.exists(self.csv_path):
            return []

        csv_files = []
        pattern = os.path.join(self.csv_path, "motor_data_*.csv")

        for file_path in glob.glob(pattern):
            stat = os.stat(file_path)
            csv_files.append({
                'name': os.path.basename(file_path),
                'path': file_path,
                'size': stat.st_size,
                'modified': datetime.fromtimestamp(stat.st_mtime).strftime('%Y-%m-%d %H:%M:%S')
            })

        # Sort by modification time, newest first
        csv_files.sort(key=lambda x: x['modified'], reverse=True)
        return csv_files

    def create_html_page(self):
        """Create the enhanced HTML interface with current display and index temperature"""
        html_content = '''<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Motor Control HMI - With Current and Index Temperature Monitor</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/Chart.js/3.9.1/chart.min.js"></script>
    <script src="https://cdn.jsdelivr.net/npm/chartjs-adapter-date-fns@3.0.0/dist/chartjs-adapter-date-fns.bundle.min.js"></script>
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 0;
            background: linear-gradient(135deg, #2c3e50 0%, #34495e 100%);
            color: white;
            height: 100vh;
            overflow: hidden;
        }
        .container {
            max-width: 100%;
            height: 100vh;
            display: grid;
            grid-template-columns: 1fr 280px;
            gap: 15px;
            padding: 0;
        }
        .charts-section {
            display: grid;
            grid-template-columns: 1fr 1fr;
            grid-template-rows: 1fr 1fr 1fr;
            gap: 15px;
            height: 100vh;
            padding: 15px 0 15px 15px;
        }
        .info-section {
            display: flex;
            flex-direction: column;
            gap: 15px;
            height: 100vh;
            padding: 15px 15px 15px 0;
            overflow-y: auto;
        }
        .chart-container {
            background: rgba(236, 240, 241, 0.98);
            padding: 15px;
            border-radius: 8px;
            box-shadow: 0 8px 25px rgba(0, 0, 0, 0.15);
            border: 1px solid rgba(149, 165, 166, 0.2);
            display: flex;
            flex-direction: column;
            min-height: 0;
        }
        .chart-wrapper {
            flex: 1;
            position: relative;
            min-height: 0;
            display: flex;
            align-items: center;
            justify-content: center;
        }
        .status-card {
            background: rgba(44, 62, 80, 0.8);
            backdrop-filter: blur(10px);
            padding: 16px;
            border-radius: 8px;
            border: 1px solid rgba(149, 165, 166, 0.3);
            box-shadow: 0 4px 15px rgba(0, 0, 0, 0.2);
            text-align: center;
            margin-bottom: 10px;
        }
        .status-card.connection {
            font-weight: 600;
            font-size: 14px;
            letter-spacing: 0.5px;
        }
        .status-card.connection.connected {
            background: rgba(39, 174, 96, 0.9);
        }
        .status-card.connection.disconnected {
            background: rgba(231, 76, 60, 0.9);
        }
        .status-card div {
            font-size: 13px;
            font-weight: 500;
            color: #ecf0f1;
        }
        .status-label {
            font-size: 11px;
            color: #bdc3c7;
            margin-bottom: 6px;
            font-weight: 400;
            letter-spacing: 0.5px;
        }
        .status-value {
            font-size: 16px;
            font-weight: 600;
            color: #ecf0f1;
            font-family: 'Courier New', monospace;
        }
        .thermal-normal {
            background: rgba(39, 174, 96, 0.8);
        }
        .thermal-warning {
            background: rgba(243, 156, 18, 0.8);
        }
        .thermal-critical {
            background: rgba(231, 76, 60, 0.9);
            animation: pulse 1s infinite;
        }
        @keyframes pulse {
            0% { opacity: 0.8; }
            50% { opacity: 1; }
            100% { opacity: 0.8; }
        }
        
        /* CSV Download Section */
        .csv-section {
            background: rgba(44, 62, 80, 0.8);
            backdrop-filter: blur(10px);
            padding: 16px;
            border-radius: 8px;
            border: 1px solid rgba(149, 165, 166, 0.3);
            box-shadow: 0 4px 15px rgba(0, 0, 0, 0.2);
            margin-bottom: 10px;
        }
        .csv-header {
            font-size: 12px;
            font-weight: 600;
            color: #ecf0f1;
            margin-bottom: 10px;
            text-align: center;
            letter-spacing: 0.5px;
        }
        .csv-files {
            max-height: 150px;
            overflow-y: auto;
        }
        .csv-file {
            display: flex;
            justify-content: space-between;
            align-items: center;
            padding: 8px;
            margin: 4px 0;
            background: rgba(52, 73, 94, 0.6);
            border-radius: 4px;
            font-size: 10px;
        }
        .csv-file:hover {
            background: rgba(52, 73, 94, 0.8);
        }
        .csv-info {
            flex: 1;
            margin-right: 8px;
        }
        .csv-name {
            color: #ecf0f1;
            font-weight: 500;
            margin-bottom: 2px;
        }
        .csv-details {
            color: #bdc3c7;
            font-size: 9px;
        }
        .csv-download {
            background: #3498db;
            color: white;
            border: none;
            padding: 4px 8px;
            border-radius: 3px;
            font-size: 9px;
            cursor: pointer;
            font-weight: 500;
        }
        .csv-download:hover {
            background: #2980b9;
        }
        .csv-refresh {
            background: #95a5a6;
            color: white;
            border: none;
            padding: 6px 12px;
            border-radius: 4px;
            font-size: 10px;
            cursor: pointer;
            font-weight: 500;
            width: 100%;
            margin-top: 8px;
        }
        .csv-refresh:hover {
            background: #7f8c8d;
        }
        
        @media (max-width: 1200px) {
            .container {
                grid-template-columns: 1fr;
                grid-template-rows: 1fr auto;
                height: 100vh;
            }
            .charts-section {
                padding: 15px;
                height: 65vh;
                grid-template-rows: 1fr 1fr;
                grid-template-columns: 1fr 1fr 1fr;
            }
            .info-section {
                padding: 0 15px 15px 15px;
                height: 35vh;
                flex-direction: row;
                gap: 10px;
                overflow-x: auto;
            }
        }
    </style>
</head>
<body>    
    <div class="container">
        <div class="charts-section">
            <div class="chart-container">
                <div class="chart-wrapper">
                    <canvas id="velocityChart"></canvas>
                </div>
            </div>
            <div class="chart-container">
                <div class="chart-wrapper">
                    <canvas id="torqueChart"></canvas>
                </div>
            </div>
            <div class="chart-container">
                <div class="chart-wrapper">
                    <canvas id="currentChart"></canvas>
                </div>
            </div>
            <div class="chart-container">
                <div class="chart-wrapper">
                    <canvas id="i2tChart"></canvas>
                </div>
            </div>
            <div class="chart-container">
                <div class="chart-wrapper">
                    <canvas id="temperatureChart"></canvas>
                </div>
            </div>
        </div>
        
        <div class="info-section">
            <div class="status-card connection" id="connectionStatus">
                CONNECTING...
            </div>
            
            <div class="status-card" id="thermalCard">
                <div class="status-label">THERMAL STATUS</div>
                <div class="status-value" id="thermalStatus">--</div>
            </div>
            
            <div class="status-card">
                <div class="status-label">DATA RATE</div>
                <div class="status-value" id="dataRate">-- Hz</div>
            </div>
            
            <div class="status-card">
                <div class="status-label">LAST UPDATE</div>
                <div class="status-value" id="lastUpdate">--</div>
            </div>
            
            <div class="csv-section">
                <div class="csv-header">📁 CSV DATA FILES</div>
                <div class="csv-files" id="csvFiles">
                    <div style="text-align: center; color: #bdc3c7; font-size: 10px; padding: 20px;">
                        Loading files...
                    </div>
                </div>
                <button class="csv-refresh" onclick="refreshCSVFiles()">🔄 Refresh Files</button>
            </div>
        </div>
    </div>

    <script>
        // WebSocket connection  
        const wsUrl = `ws://192.168.1.100:8081`;
        let ws = null;
        let reconnectTimeout = null;
        
        // Data storage
        let dataHistory = [];
        let lastUpdateTime = null;
        let updateCount = 0;
        let startTime = Date.now();
        
        // Chart configuration
        const chartConfig = {
            type: 'line',
            options: {
                responsive: true,
                maintainAspectRatio: false,
                layout: {
                    padding: {
                        top: 10,
                        bottom: 10,
                        left: 10,
                        right: 10
                    }
                },
                scales: {
                    x: {
                        type: 'linear',
                        display: true,
                        grid: {
                            display: true,
                            color: 'rgba(149, 165, 166, 0.2)',
                            lineWidth: 1
                        },
                        ticks: {
                            display: false
                        },
                        title: {
                            display: false
                        },
                        min: 0,
                        max: 100
                    },
                    y: {
                        grid: {
                            display: true,
                            color: 'rgba(149, 165, 166, 0.2)',
                            lineWidth: 1
                        },
                        ticks: {
                            color: '#2c3e50',
                            font: {
                                family: 'Segoe UI',
                                size: 10
                            }
                        },
                        title: { 
                            display: false
                        }
                    }
                },
                plugins: {
                    legend: { 
                        position: 'top',
                        labels: {
                            color: '#2c3e50',
                            font: {
                                family: 'Segoe UI',
                                size: 11,
                                weight: 600
                            }
                        }
                    },
                    tooltip: { 
                        mode: 'index', 
                        intersect: false,
                        backgroundColor: 'rgba(44, 62, 80, 0.9)',
                        titleColor: '#ecf0f1',
                        bodyColor: '#ecf0f1'
                    }
                },
                interaction: { mode: 'nearest', axis: 'x', intersect: false },
                animation: { duration: 0 }
            }
        };
        
        // Initialize charts
        const velocityChart = new Chart(
            document.getElementById('velocityChart'), 
            {
                ...chartConfig,
                data: { datasets: [] },
                options: {
                    ...chartConfig.options,
                    scales: {
                        ...chartConfig.options.scales,
                        y: {
                            ...chartConfig.options.scales.y,
                            title: { 
                                display: true, 
                                text: 'VELOCITY (RPM)',
                                color: '#2c3e50',
                                font: { family: 'Segoe UI', size: 12, weight: 600 }
                            }
                        }
                    }
                }
            }
        );
        
        const torqueChart = new Chart(
            document.getElementById('torqueChart'), 
            {
                ...chartConfig,
                data: { datasets: [] },
                options: {
                    ...chartConfig.options,
                    scales: {
                        ...chartConfig.options.scales,
                        y: {
                            ...chartConfig.options.scales.y,
                            title: { 
                                display: true, 
                                text: 'TORQUE (mNm)',
                                color: '#2c3e50',
                                font: { family: 'Segoe UI', size: 12, weight: 600 }
                            }
                        }
                    }
                }
            }
        );
        
        const currentChart = new Chart(
            document.getElementById('currentChart'), 
            {
                ...chartConfig,
                data: { datasets: [] },
                options: {
                    ...chartConfig.options,
                    scales: {
                        ...chartConfig.options.scales,
                        y: {
                            ...chartConfig.options.scales.y,
                            title: { 
                                display: true, 
                                text: 'CURRENT (A)',
                                color: '#2c3e50',
                                font: { family: 'Segoe UI', size: 12, weight: 600 }
                            }
                        }
                    }
                }
            }
        );
        
        const i2tChart = new Chart(
            document.getElementById('i2tChart'), 
            {
                ...chartConfig,
                data: { datasets: [] },
                options: {
                    ...chartConfig.options,
                    scales: {
                        ...chartConfig.options.scales,
                        y: {
                            ...chartConfig.options.scales.y,
                            title: { 
                                display: true, 
                                text: 'THERMAL I²t (%)',
                                color: '#2c3e50',
                                font: { family: 'Segoe UI', size: 12, weight: 600 }
                            },
                            min: 0,
                            max: 120
                        }
                    }
                }
            }
        );
        
        const temperatureChart = new Chart(
            document.getElementById('temperatureChart'), 
            {
                ...chartConfig,
                data: { datasets: [] },
                options: {
                    ...chartConfig.options,
                    scales: {
                        ...chartConfig.options.scales,
                        y: {
                            ...chartConfig.options.scales.y,
                            title: { 
                                display: true, 
                                text: 'TEMPERATURE (°C)',
                                color: '#2c3e50',
                                font: { family: 'Segoe UI', size: 12, weight: 600 }
                            }
                        }
                    }
                }
            }
        );
        
        function connectWebSocket() {
            console.log('Connecting to:', wsUrl);
            ws = new WebSocket(wsUrl);
            
            ws.onopen = function() {
                console.log('WebSocket connected');
                updateConnectionStatus(true);
                if (reconnectTimeout) {
                    clearTimeout(reconnectTimeout);
                    reconnectTimeout = null;
                }
            };
            
            ws.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    if (data.type === 'history') {
                        handleHistoryData(data.data);
                    } else {
                        handleRealtimeData(data);
                    }
                } catch (e) {
                    console.error('Error parsing message:', e);
                }
            };
            
            ws.onclose = function() {
                console.log('WebSocket disconnected');
                updateConnectionStatus(false);
                reconnectTimeout = setTimeout(connectWebSocket, 2000);
            };
            
            ws.onerror = function(error) {
                console.error('WebSocket error:', error);
            };
        }
        
        function updateConnectionStatus(connected) {
            const status = document.getElementById('connectionStatus');
            if (connected) {
                status.textContent = 'CONNECTED';
                status.className = 'status-card connection connected';
            } else {
                status.textContent = 'DISCONNECTED';
                status.className = 'status-card connection disconnected';
            }
        }
        
        function handleHistoryData(history) {
            dataHistory = history;
            updateCharts();
            updateStats();
        }
        
        function handleRealtimeData(data) {
            dataHistory.push(data);
            
            if (dataHistory.length > 100) {
                dataHistory.shift();
            }
            
            updateCharts();
            updateThermalStatus(data);
            updateStats();
            
            updateCount++;
            lastUpdateTime = new Date();
        }
        
        function updateCharts() {
            if (dataHistory.length === 0) return;
            
            const isDualMotor = dataHistory[0].left_motor !== undefined;
            
            if (isDualMotor) {
                updateDualMotorCharts();
            } else {
                updateSingleMotorCharts();
            }
            
            updateThermalCharts(isDualMotor);
        }
        
        function updateSingleMotorCharts() {
            const velocityData = dataHistory.map((d, index) => ({
                x: index,
                y: d.motor.velocity
            }));
            
            const torqueData = dataHistory.map((d, index) => ({
                x: index,
                y: d.motor.torque
            }));
            
            const currentData = dataHistory.map((d, index) => ({
                x: index,
                y: d.motor.current || 0
            }));
            
            velocityChart.data.datasets = [{
                label: 'Motor Velocity',
                data: velocityData,
                borderColor: '#3498db',
                backgroundColor: 'rgba(52, 152, 219, 0.1)',
                fill: false,
                tension: 0.1,
                borderWidth: 2,
                pointRadius: 1,
                pointHoverRadius: 3
            }];
            
            torqueChart.data.datasets = [{
                label: 'Motor Torque',
                data: torqueData,
                borderColor: '#e74c3c',
                backgroundColor: 'rgba(231, 76, 60, 0.1)',
                fill: false,
                tension: 0.1,
                borderWidth: 2,
                pointRadius: 1,
                pointHoverRadius: 3
            }];
            
            currentChart.data.datasets = [{
                label: 'Motor Current',
                data: currentData,
                borderColor: '#f39c12',
                backgroundColor: 'rgba(243, 156, 18, 0.1)',
                fill: false,
                tension: 0.1,
                borderWidth: 2,
                pointRadius: 1,
                pointHoverRadius: 3
            }];
            
            updateChartRange(velocityChart);
            updateChartRange(torqueChart);
            updateChartRange(currentChart);
            
            velocityChart.update('none');
            torqueChart.update('none');
            currentChart.update('none');
        }
        
        function updateDualMotorCharts() {
            const leftVelocityData = dataHistory.map((d, index) => ({
                x: index,
                y: d.left_motor.velocity
            }));
            
            const rightVelocityData = dataHistory.map((d, index) => ({
                x: index,
                y: d.right_motor.velocity
            }));
            
            const leftTorqueData = dataHistory.map((d, index) => ({
                x: index,
                y: d.left_motor.torque
            }));
            
            const rightTorqueData = dataHistory.map((d, index) => ({
                x: index,
                y: d.right_motor.torque
            }));
            
            const leftCurrentData = dataHistory.map((d, index) => ({
                x: index,
                y: d.left_motor.current || 0
            }));
            
            const rightCurrentData = dataHistory.map((d, index) => ({
                x: index,
                y: d.right_motor.current || 0
            }));
            
            const totalCurrentData = dataHistory.map((d, index) => ({
                x: index,
                y: (d.left_motor.current || 0) + (d.right_motor.current || 0)
            }));
            
            velocityChart.data.datasets = [
                {
                    label: 'Left Motor',
                    data: leftVelocityData,
                    borderColor: '#3498db',
                    backgroundColor: 'rgba(52, 152, 219, 0.1)',
                    fill: false,
                    tension: 0.1,
                    borderWidth: 2,
                    pointRadius: 1,
                    pointHoverRadius: 3
                },
                {
                    label: 'Right Motor',
                    data: rightVelocityData,
                    borderColor: '#e74c3c',
                    backgroundColor: 'rgba(231, 76, 60, 0.1)',
                    fill: false,
                    tension: 0.1,
                    borderWidth: 2,
                    pointRadius: 1,
                    pointHoverRadius: 3
                }
            ];
            
            torqueChart.data.datasets = [
                {
                    label: 'Left Motor',
                    data: leftTorqueData,
                    borderColor: '#3498db',
                    backgroundColor: 'rgba(52, 152, 219, 0.1)',
                    fill: false,
                    tension: 0.1,
                    borderWidth: 2,
                    pointRadius: 1,
                    pointHoverRadius: 3
                },
                {
                    label: 'Right Motor',
                    data: rightTorqueData,
                    borderColor: '#e74c3c',
                    backgroundColor: 'rgba(231, 76, 60, 0.1)',
                    fill: false,
                    tension: 0.1,
                    borderWidth: 2,
                    pointRadius: 1,
                    pointHoverRadius: 3
                }
            ];
            
            currentChart.data.datasets = [
                {
                    label: 'Left Motor',
                    data: leftCurrentData,
                    borderColor: '#f39c12',
                    backgroundColor: 'rgba(243, 156, 18, 0.1)',
                    fill: false,
                    tension: 0.1,
                    borderWidth: 2,
                    pointRadius: 1,
                    pointHoverRadius: 3
                },
                {
                    label: 'Right Motor',
                    data: rightCurrentData,
                    borderColor: '#9b59b6',
                    backgroundColor: 'rgba(155, 89, 182, 0.1)',
                    fill: false,
                    tension: 0.1,
                    borderWidth: 2,
                    pointRadius: 1,
                    pointHoverRadius: 3
                },
                {
                    label: 'Total Current',
                    data: totalCurrentData,
                    borderColor: '#e74c3c',
                    backgroundColor: 'rgba(231, 76, 60, 0.1)',
                    fill: false,
                    tension: 0.1,
                    borderWidth: 3,
                    pointRadius: 1,
                    pointHoverRadius: 3
                }
            ];
            
            updateChartRange(velocityChart);
            updateChartRange(torqueChart);
            updateChartRange(currentChart);
            
            velocityChart.update('none');
            torqueChart.update('none');
            currentChart.update('none');
        }
        
        function updateThermalCharts(isDualMotor) {
            if (!dataHistory[0].thermal) return;
            
            if (isDualMotor) {
                const leftI2tData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal.left ? d.thermal.left.i2t : 0
                }));
                
                const rightI2tData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal.right ? d.thermal.right.i2t : 0
                }));
                
                const leftDriveTempData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal.left ? d.thermal.left.drive_temp : 0
                }));
                
                const rightDriveTempData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal.right ? d.thermal.right.drive_temp : 0
                }));
                
                const leftCoreTempData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal.left ? d.thermal.left.core_temp : 0
                }));
                
                const rightCoreTempData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal.right ? d.thermal.right.core_temp : 0
                }));
                
                // NEW: Add Index Temperature for dual motor
                const leftIndexTempData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal.left ? d.thermal.left.index_temp : 0
                }));
                
                const rightIndexTempData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal.right ? d.thermal.right.index_temp : 0
                }));
                
                i2tChart.data.datasets = [
                    {
                        label: 'Left Motor I²t',
                        data: leftI2tData,
                        borderColor: '#3498db',
                        backgroundColor: 'rgba(52, 152, 219, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    },
                    {
                        label: 'Right Motor I²t',
                        data: rightI2tData,
                        borderColor: '#e74c3c',
                        backgroundColor: 'rgba(231, 76, 60, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    }
                ];
                
                // UPDATED: Include Index Temperature in dual motor display
                temperatureChart.data.datasets = [
                    {
                        label: 'Left Drive Temp',
                        data: leftDriveTempData,
                        borderColor: '#3498db',
                        backgroundColor: 'rgba(52, 152, 219, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    },
                    {
                        label: 'Right Drive Temp',
                        data: rightDriveTempData,
                        borderColor: '#e74c3c',
                        backgroundColor: 'rgba(231, 76, 60, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    },
                    {
                        label: 'Left Core Temp',
                        data: leftCoreTempData,
                        borderColor: '#f39c12',
                        backgroundColor: 'rgba(243, 156, 18, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    },
                    {
                        label: 'Right Core Temp',
                        data: rightCoreTempData,
                        borderColor: '#9b59b6',
                        backgroundColor: 'rgba(155, 89, 182, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    },
                    {
                        label: 'Left Index Temp',
                        data: leftIndexTempData,
                        borderColor: '#27ae60',
                        backgroundColor: 'rgba(39, 174, 96, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    },
                    {
                        label: 'Right Index Temp',
                        data: rightIndexTempData,
                        borderColor: '#e67e22',
                        backgroundColor: 'rgba(230, 126, 34, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    }
                ];
            } else {
                const i2tData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal ? d.thermal.i2t : 0
                }));
                
                const driveTempData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal ? d.thermal.drive_temp : 0
                }));
                
                const coreTempData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal ? d.thermal.core_temp : 0
                }));
                
                // NEW: Add Index Temperature for single motor
                const indexTempData = dataHistory.map((d, index) => ({
                    x: index,
                    y: d.thermal ? d.thermal.index_temp : 0
                }));
                
                i2tChart.data.datasets = [{
                    label: 'Motor I²t',
                    data: i2tData,
                    borderColor: '#f39c12',
                    backgroundColor: 'rgba(243, 156, 18, 0.1)',
                    fill: false,
                    tension: 0.1,
                    borderWidth: 2,
                    pointRadius: 1,
                    pointHoverRadius: 3
                }];
                
                // UPDATED: Include Index Temperature in single motor display  
                temperatureChart.data.datasets = [
                    {
                        label: 'Drive Temperature',
                        data: driveTempData,
                        borderColor: '#e67e22',
                        backgroundColor: 'rgba(230, 126, 34, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    },
                    {
                        label: 'Core Temperature',
                        data: coreTempData,
                        borderColor: '#9b59b6',
                        backgroundColor: 'rgba(155, 89, 182, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    },
                    {
                        label: 'Index Temperature',
                        data: indexTempData,
                        borderColor: '#27ae60',
                        backgroundColor: 'rgba(39, 174, 96, 0.1)',
                        fill: false,
                        tension: 0.1,
                        borderWidth: 2,
                        pointRadius: 1,
                        pointHoverRadius: 3
                    }
                ];
            }
            
            updateChartRange(i2tChart);
            updateChartRange(temperatureChart);
            
            i2tChart.update('none');
            temperatureChart.update('none');
        }
        
        function updateChartRange(chart) {
            const maxX = Math.max(dataHistory.length - 1, 0);
            chart.options.scales.x.max = Math.max(maxX, 100);
        }
        
        function updateThermalStatus(data) {
            const statusElement = document.getElementById('thermalStatus');
            const cardElement = document.getElementById('thermalCard');
            
            if (!data.thermal || !data.thermal.valid) {
                statusElement.textContent = 'NO DATA';
                cardElement.className = 'status-card';
                return;
            }
            
            let maxI2t = 0;
            if (data.left_motor !== undefined) {
                maxI2t = Math.max(
                    data.thermal.left ? data.thermal.left.i2t : 0,
                    data.thermal.right ? data.thermal.right.i2t : 0
                );
            } else {
                maxI2t = data.thermal.i2t;
            }
            
            if (maxI2t > 95) {
                statusElement.textContent = 'CRITICAL';
                cardElement.className = 'status-card thermal-critical';
            } else if (maxI2t > 80) {
                statusElement.textContent = 'WARNING';
                cardElement.className = 'status-card thermal-warning';
            } else {
                statusElement.textContent = 'NORMAL';
                cardElement.className = 'status-card thermal-normal';
            }
        }
        
        function updateStats() {
            const now = Date.now();
            const elapsedSeconds = (now - startTime) / 1000;
            const dataRate = updateCount / elapsedSeconds;
            
            document.getElementById('dataRate').textContent = dataRate.toFixed(1) + ' Hz';
            
            if (lastUpdateTime) {
                const timeString = lastUpdateTime.toLocaleTimeString([], {
                    hour12: false,
                    hour: '2-digit',
                    minute: '2-digit',
                    second: '2-digit'
                });
                document.getElementById('lastUpdate').textContent = timeString;
            }
        }
        
        // CSV Functions
        function refreshCSVFiles() {
            fetch('/api/csv-files')
                .then(response => response.json())
                .then(files => {
                    displayCSVFiles(files);
                })
                .catch(error => {
                    console.error('Error fetching CSV files:', error);
                    document.getElementById('csvFiles').innerHTML = 
                        '<div style="text-align: center; color: #e74c3c; font-size: 10px; padding: 20px;">Error loading files</div>';
                });
        }
        
        function displayCSVFiles(files) {
            const container = document.getElementById('csvFiles');
            
            if (files.length === 0) {
                container.innerHTML = '<div style="text-align: center; color: #bdc3c7; font-size: 10px; padding: 20px;">No CSV files found</div>';
                return;
            }
            
            container.innerHTML = files.map(file => `
                <div class="csv-file">
                    <div class="csv-info">
                        <div class="csv-name">${file.name}</div>
                        <div class="csv-details">${file.modified} • ${formatFileSize(file.size)}</div>
                    </div>
                    <button class="csv-download" onclick="downloadFile('${file.name}')">
                        📥 Download
                    </button>
                </div>
            `).join('');
        }
        
        function formatFileSize(bytes) {
            if (bytes === 0) return '0 B';
            const k = 1024;
            const sizes = ['B', 'KB', 'MB'];
            const i = Math.floor(Math.log(bytes) / Math.log(k));
            return parseFloat((bytes / Math.pow(k, i)).toFixed(1)) + ' ' + sizes[i];
        }
        
        function downloadFile(filename) {
            const url = `/download/${encodeURIComponent(filename)}`;
            const link = document.createElement('a');
            link.href = url;
            link.download = filename;
            document.body.appendChild(link);
            link.click();
            document.body.removeChild(link);
        }
        
        // Initialize
        connectWebSocket();
        refreshCSVFiles();
        
        // Update stats every second
        setInterval(updateStats, 1000);
        
        // Refresh CSV files every 30 seconds
        setInterval(refreshCSVFiles, 30000);
    </script>
</body>
</html>'''
        return html_content

    async def start_servers(self):
        """Start both HTTP and WebSocket servers"""

        # HTTP Server Handler
        class MyHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=None, **kwargs)

            def do_GET(self):
                if self.path == '/' or self.path == '/index.html':
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html')
                    self.end_headers()
                    self.wfile.write(
                        server_instance.create_html_page().encode())

                elif self.path == '/api/csv-files':
                    # API endpoint to get CSV file list
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    files = server_instance.get_csv_files()
                    self.wfile.write(json.dumps(files).encode())

                elif self.path.startswith('/download/'):
                    # File download endpoint
                    filename = urllib.parse.unquote(
                        self.path[10:])  # Remove '/download/'
                    file_path = os.path.join(
                        server_instance.csv_path, filename)

                    if os.path.exists(file_path) and filename.startswith('motor_data_') and filename.endswith('.csv'):
                        self.send_response(200)
                        self.send_header('Content-type', 'text/csv')
                        self.send_header('Content-Disposition',
                                         f'attachment; filename="{filename}"')
                        self.end_headers()

                        with open(file_path, 'rb') as f:
                            self.wfile.write(f.read())
                    else:
                        self.send_error(404)
                else:
                    self.send_error(404)

        global server_instance
        server_instance = self

        # Start HTTP server in thread
        class ReuseTCPServer(socketserver.TCPServer):
            allow_reuse_address = True

        def start_http_server():
            try:
                httpd = ReuseTCPServer(
                    ("", self.web_port), MyHTTPRequestHandler)
                print(f"✓ HTTP server started on port {self.web_port}")
                httpd.serve_forever()
            except Exception as e:
                print(f"HTTP server error: {e}")

        http_thread = threading.Thread(target=start_http_server, daemon=True)
        http_thread.start()

        # Wait for HTTP server to start
        await asyncio.sleep(1)

        # Start WebSocket server
        print(f"✓ Starting WebSocket server on port {self.web_port + 1}")

        start_server = websockets.serve(
            self.websocket_handler,
            "0.0.0.0",
            self.web_port + 1
        )

        await start_server
        print(f"✓ WebSocket server started on port {self.web_port + 1}")
        print(f"✓ HMI server with CSV export, current monitoring, and index temperature ready!")
        print(f"  Access from any of these addresses:")
        for iface, ip in self.network_interfaces.items():
            print(f"    {iface}: http://{ip}:{self.web_port}")
        print(f"  Local access: http://localhost:{self.web_port}")
        print(f"  CSV downloads: Built into web interface")

    async def run(self):
        """Run the server"""
        self.running = True

        # Store the event loop for cross-thread communication
        self.loop = asyncio.get_event_loop()

        # Start UDP listener in background thread
        udp_thread = threading.Thread(
            target=self.start_udp_listener, daemon=True)
        udp_thread.start()

        # Start web servers
        await self.start_servers()

        # Keep running
        try:
            await asyncio.Future()  # Run forever
        except KeyboardInterrupt:
            print("\nShutting down...")
            self.running = False


def main():
    parser = argparse.ArgumentParser(
        description='Motor Control Real-time HMI Server with CSV Export, Current Display, and Index Temperature')
    parser.add_argument('--port', type=int, default=8080,
                        help='Web server port')
    parser.add_argument('--udp-port', type=int,
                        default=9999, help='UDP listener port')
    parser.add_argument('--udp-ip', default='192.168.1.255',
                        help='UDP broadcast IP')
    parser.add_argument('--csv-path', default='/home/foxtrot/log/motor_data',
                        help='Path to CSV files directory')

    args = parser.parse_args()

    server = MotorHMIServer(args.port, args.udp_port,
                            args.udp_ip, args.csv_path)

    try:
        asyncio.run(server.run())
    except KeyboardInterrupt:
        print("\nServer stopped")


if __name__ == "__main__":
    main()

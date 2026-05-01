ESP32-Based EV Safety & Diagnostics System with Real-Time Alerts
This is a smart IoT-based electric vehicle monitoring system using an ESP32 and multiple sensors. It continuously tracks vehicle parameters like battery, temperature, speed, and pressure.
In case of an accident, the system detects impact and sends instant alerts with GPS location via SMS and cloud dashboards.

Features
Real-time vehicle health monitoring
Accident detection using accelerometer
GPS location tracking with live coordinates
SMS alert system for emergencies
Live dashboard using Blynk
Cloud data logging via MQTT
Speed measurement using Hall sensor
Multi-sensor integration for accurate diagnostics

Components Required
ESP32 Microcontroller
SIM808 (GPS + GSM Module)
ADXL345 Accelerometer
INA219 Voltage & Current Sensor
HX710B Pressure Sensor
LM35 Temperature Sensor
Hall Effect Sensor (Speed)
Jumper Wires & Breadboard
Power Supply (Battery / Adapter)

How It Works
Sensors continuously collect vehicle data
ESP32 processes and uploads data to cloud
Accelerometer detects sudden impact (crash)
GPS module fetches location coordinates
SMS alert with Google Maps link is sent
Data is displayed on Blynk dashboard
Backend stores data via MQTT for analysis

Accident Detection

When high G-force is detected:
Crash is identified
GPS location is captured
Alert message is sent instantly
Cloud data is updated in real-time

Cloud & Connectivity
Wi-Fi (ESP32) → Sends data to cloud
Blynk App → Live monitoring
MQTT Protocol → Backend communication
Database → Stores vehicle data

Getting Started
Connect all components as per circuit diagram
Upload code to ESP32 using Arduino IDE
Configure:
Wi-Fi SSID & Password
Blynk Auth Token
MQTT Broker IP
Phone number for alerts
Power the system
Monitor live data and receive alerts

Data Monitored
Battery Voltage & Current
Temperature
Tire Pressure
Vehicle Speed
Acceleration (X, Y, Z)
GPS Location

Applications
Electric Vehicles
Fleet Monitoring Systems
Smart Mobility Solutions
Vehicle Safety Systems
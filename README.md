# URC Science Mission Operator Console

A PyQt5 GUI built as a ROS 2 node for the University Rover Challenge Science Mission.

## What it does
- Subscribes to /science_data (live sensor telemetry: temperature, humidity, pH, life-detection signal, GNSS, site ID)
- Subscribes to /sample_status (sample collection state: IDLE, DRILLING, ANALYZING, COMPLETE)
- Publishes operator commands to /sample_command (Begin Sample Collection, Abort)
- Tracks per-site documentation checklist per URC rule 1.b.iii
- Tracks a 5-minute cache removal countdown per URC rule 1.b.vii
- Shows live comms/link health and flags when the data feed goes stale

## Files
- `science_gui.py` — the operator console (run this)
- `fake_science_publisher.py` — simulated rover node standing in for real sensors/hardware

## Running it
Requires ROS 2 Jazzy and PyQt5 installed.

Terminal 1:
python3 fake_science_publisher.py

Terminal 2:
python3 science_gui.py

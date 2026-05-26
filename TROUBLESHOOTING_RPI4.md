# Troubleshooting Documentation: Raspberry Pi 4 ROS 2 Car Controller

## Overview

This document summarizes the troubleshooting steps, verified information, and remaining possible causes for the issue where the ROS 2 Car Controller (running on a Raspberry Pi 4 via Docker on Ubuntu 24.04 Server) fails to receive/execute movement commands from the Speedpilot mobile application (Flutter via WebSockets).

**Problem Statement:** The car moves correctly via commands when running on a Raspberry Pi 5. After migrating to a Raspberry Pi 4, the system compiles, boots, and the GPIO simulation/init logic passes, but commands sent from the phone (iOS/Android) result in no motor movement and no logging output indicating a received message.

---

## 1. System Architecture

- **OS:** Ubuntu 24.04 Server.
- **Networking Base:**
  - Static AP via `hostapd` and `dnsmasq` creating an isolated network (`192.168.4.1/24`) over `wlan0`.
  - IP Forwarding is active (`/proc/sys/net/ipv4/ip_forward` = 1).
- **Containerization:** Docker via `docker-compose`.
  - **Networking:** `network_mode: host` (bypassing isolated Docker NAT bridging).
  - **Privileges:** `privileged: true`, mounting `/dev/gpiochip*`.
- **ROS 2 Pipeline:**
  - `bridge_node.py`: Listens on `ws://0.0.0.0:9091` to receive JSON strings from the mobile app, and publishes to ROS topic `/vehicle_command`.
  - `controller_node.py`: Subscribes to `/vehicle_command` and directly manipulates GPIO duty cycles via `gpiod` v2 to drive the motors.

---

## 2. Issues Ruled Out

### [X] Hardware/Pin Discrepancies

- Analysis: BCM vs. Physical pin numbering mismatch.
- Conclusion: **Ruled Out**. Pins 23 (Steering), 24 (Forward), and 25 (Backward) are confirmed BCM allocations identically matched to physical wiring from the RPi 5 era.
- Logic Changes Made: RPi 4 uses `gpiochip0` (58 lines) while RPi 5 uses `gpiochip4` (28 lines). `controller_node.py`, `ultrasonic_node.py`, and `bridge_node.py` were dynamically rewritten to scan all `/dev/gpiochip*` for `num_lines >= 28` to ensure agnostic backwards & forwards hardware compatibility.

### [X] Silent GPIO Initialization Crashes

- Analysis: `gpiod.request_lines` throws unhandled `OSError` forcing nodes to die prior to logging.
- Conclusion: **Ruled Out**. Try/catch blocks were wrapped around all GPIO initializations. The nodes successfully fallback to "simulation" mode if hardware locks occur, yet they currently print `CarController node started.`, meaning they survive init without exceptions.

### [X] Internal Application Message Processing (ROS Level)

- Analysis: Messages arrive but ROS fails to dispatch them.
- Conclusion: **Ruled Out**. Running `ros2 topic echo /vehicle_command` directly on the Pi confirms nothing is being published. If messages arrived successfully at `bridge_node.py`, it would print `New WebSocket client connected`. This guarantees the pipeline breaks **before or during** the WebSocket handshake.

### [X] Docker / Ubuntu Basic Firewalls & Bindings

- Analysis: Port 9091 blocked by `ufw` or not bound by Python.
- Conclusion: **Ruled Out**.
  - `sudo ss -tulpn` verifies Python is actively listening on `0.0.0.0:9091`.
  - `iptables -L INPUT` verifies the default policy is `ACCEPT` and no strict drops apply.

---

## 3. Current Live Hypothesis: Network Demarcation / Protocol Drops

If the application connects, it should log a handshake or client connect. Because nothing logs, traffic is dying at the boundary.

### Hypothesis A: Handshake Rejection by `websocket-server`

- The Python Library `websocket-server` strictly enforces HTTP Upgrade headers.
- iOS/Android `web_socket_channel` might send headers that the server strictly rejects, causing a silent connection drop.
- **Mitigation currently in place:** The `safe_handshake` method in `bridge_node.py` was patched to explicitly print: `Handshake error intercepted: {e}`.
- **Next Test:** Connect the phone. If the terminal prints a Handshake Error, the library is explicitly rejecting the mobile client's formatting.

### Hypothesis B: Docker Host Network vs. WiFi AP Isolation

- `wlan0` provides `192.168.4.1/24`, but Docker attaches its Host namespace in a way where `iptables FORWARD` chains or `systemd-networkd` prevents the physical Wi-Fi AP sub-interface from accessing the loopback/host-listening daemon space.
- **Next Test:** Run a simple TCP Port scan against `192.168.4.1:9091` using a laptop connected to the hotspot, or via `nc -zv 192.168.4.1 9091`. If the scan timeouts/refuses, traffic routes are physically walled off at the Linux kernel level.

### Hypothesis C: Flutter Client Configuration (IP Mismatch)

- The mobile application code (`WebSocketManager.dart`) connects dynamically via `connect(String url)`.
- If the app is hardcoded or configured in-UI to attempt connecting to a prior IP (e.g., a local router IP rather than the Hotspot `192.168.4.1`), the traffic simply routes into the void.

## 4. Next Debugging Steps (For AI or Developer)

1. **Raw TCP Verification:** From a device connected to the `raspberry` hotspot, run:

   ```bash
   # Windows (PowerShell/CMD):
   Test-NetConnection -ComputerName 192.168.4.1 -Port 9091
   # macOS/Linux:
   nc -zv 192.168.4.1 9091
   ```

   **If True:** The port is open, the OS routes correctly, move to Step 2 (Handshake Inspection).
   **If False:** The OS kernel / `hostapd` / Docker network settings are blackholing the traffic.

2. **Monitor Docker Outputs:** Launch the mobile app and deliberately attempt to drive the car. Watch the speedpilot logs purely for the line `Handshake error intercepted`.

3. **Check App's Target URL:** Log or verify the UI string that gets passed to `WebSocketManager.connect(url)`. It must be `ws://192.168.4.1:9091`.

4. **Verify iOS Permissions (If resuming iOS testing):**
   Ensure `Info.plist` includes `NSAppTransportSecurity` / `NSAllowsArbitraryLoads` to permit plaintext `ws://` connections.

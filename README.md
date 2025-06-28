# ESP8266-WiFi-to-UART-Bridge
ESP8266 WiFi to UART Bridge for u360gts (https://github.com/raul-ortega/u360gts)

This sketch transforms your ESP8266 (NodeMCU) into a WiFi-to-UART bridge tailored for AAT u360gts or any other serial data over WiFi. It elegantly manages the UART interface depending on WiFi connectivity, ensuring the flight controller (or any other device that shares UART with USB) USB port is free for configuration when WiFi is down.

The original code: https://github.com/roboremo/ESP8266-WiFi-UART-Bridge


**Note!** *The code has been edited for my needs. ESP8266 in STA mode with UDP protocol on port 14550 that connects to my ELRS Backpack of my RC Radio running Mavlink-RC. ESP sends the telemetry via UART to SP Racing F3 Flight Controller which is the brain of u360gts Antenna Tracker. 
Code HAS NOT BEEN tested in AP mode and with TCP protocol, but should remain functional as the original.* 


## Key Features:
* WiFi Station Mode (STA): Connects to your existing WiFi network.

* UDP Communication: Sends/receives data packets via UDP.

* UART Bridge: Forwards data between WiFi and UART.

* Smart UART Management:

* Disables UART if WiFi connection fails or drops, freeing the FC USB for configuration.

* Re-enables UART automatically once WiFi reconnects — no power cycle needed.

* Visual Feedback: Onboard LED (GPIO2) indicates WiFi status — ON when connected, OFF when disconnected.

* Timeout Handling: Waits 15 seconds for WiFi connection at startup or reconnection attempt.

## How It Works:
**Startup & WiFi Connection:**

* The ESP boots, configures itself as a WiFi client, and attempts to connect to the specified SSID/password.

* If connected within 15 seconds, it turns the LED ON and starts the UDP server to communicate.

* UART Management Based on WiFi State:

**If WiFi connects:**

* The UART (Serial) interface starts at 115200 baud.

* Serial data received from WiFi is sent out over UART.

* If WiFi fails to connect or drops after being connected:

* The UART interface is cleanly shut down (Serial.end()), releasing the serial port so you can connect to the FC’s USB without conflict.

* The LED is turned OFF as a clear indicator WiFi is not connected.

**Automatic Reconnection & UART Restart:**

* While WiFi is disconnected and UART is disabled, the ESP periodically (every 5 seconds) tries to reconnect to WiFi.

* On successful reconnection, UART is re-initialized and the UDP server restarted.

* LED is switched ON again to indicate an active WiFi connection.

**Data Forwarding Loop:**

* When WiFi is connected, the code continuously listens for UDP packets and forwards them byte-wise to UART.

## Why This Matters for Flight Controllers:
Flight controllers (like SP Racing F3) often share UART lines with USB-to-serial converters. If the ESP keeps UART active without WiFi, it hijacks the port and blocks USB configuration access. This code smartly disables UART when WiFi is unavailable, letting you configure your FC via USB hassle-free — then automatically restores UART bridging once WiFi returns, all without needing to restart hardware.


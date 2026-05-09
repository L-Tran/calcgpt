CALCGPT is a custom firmware project that brings ChatGPT and Telegram messaging to a TI-84 Plus Silver Edition calculator. A Seeed Studio XIAO ESP32C3 is soldered directly inside the calculator, connecting to the link port and using the calculator's own batteries for power. The chip handles WiFi connectivity and API calls, acting as a bridge between the calculator and the internet. Features

ChatGPT — ask questions and get AI responses directly on your calculator screen
Conversation history — Can reply to previous chats
Paged responses — long answers are split into pages you can navigate
Telegram — send and receive Telegram messages from the calculator
Browser setup — connect to the CALCGPT WiFi hotspot and enter your credentials at 192.168.4.1, no reflashing required
Auto launcher — the chip pushes the TI-BASIC program directly to the calculator on first boot

Hardware

TI-84 Plus Silver Edition
Seeed Studio XIAO ESP32C3
Soldering equipment

See the wiring diagram in this repo for connection details.

Setup

1. Flash the firmware
Open calcgpt.ino in Arduino IDE with the XIAO ESP32C3 board selected. Export the compiled binary and flash it to the XIAO using esptool-js.
2. Configure credentials
Power on the calculator with the XIAO installed. On your phone or computer, connect to the WiFi network called CALCGPT (password: calcgpt123). Open a browser and go to 192.168.4.1. Enter your:

WiFi name and password
OpenAI API key
Telegram bot token and chat ID (optional)

3. Load the TI-BASIC program
On the calculator home screen, run:
5→C
Send(C
This tells the XIAO to push the CALCGPT program directly to the calculator.
4. Run CALCGPT
Press PRGM, find CALCGPT, and run it through Doors CS.

Dependencies

The following Arduino libraries are required:

ArTICL — TI link protocol
ArduinoJson
UrlEncode

Install ArTICL manually by downloading the ZIP and placing it in your Arduino libraries folder.
Notes

The XIAO is powered directly from the calculator's batteries via the VUSB pin
Doors CS must be installed on the calculator for the program to run
Responses from ChatGPT are limited to 300 tokens to fit the calculator's memory
The calculator screen is 16 characters wide, so responses are automatically formatted into pages

#include <Arduino.h>
#include <HardwareSerial.h>

// Define Serial1 for receiving data (PA10 = RX, PA9 = TX)
HardwareSerial Serial1(PA10, PA9);
#define MAX_TEXT_LENGTH 20 
// Define Serial2 for DWIN HMI communication (PA2 = TX, PA3 = RX)
HardwareSerial Serial2(PA3, PA2);

// Define Serial3 for debugging (PB11 = TX, PB10 = RX)
HardwareSerial Serial3(PB11, PB10);

// Variables to store sensor status
int Differential = 0;
int BuchholzRelay = 0;
int MasterTrip = 0;
int REF = 0;
int OverCurrent = 0;
int PRV = 0;
int RPRR = 0;

// Define VP addresses for LCD display (DWIN HMI)
const uint16_t VP_TEXT[7] = {0x400, 0x1210, 0x1220, 0x1230, 0x1240, 0x1250, 0x1260}; // Text display VP
const uint16_t VP_COLOR[7] = {0x5001, 0x5002, 0x5003, 0x5004, 0x5005, 0x5006, 0x5007}; // Color VP

// Define Labels for each sensor
const char* SENSOR_LABELS[7] = {
    "Differential",
    "BuchholzRelay",
    "MasterTrip",
    "REF",
    "OverCurrent",
    "PRV",
    "RPRR"
};

// Buffer for storing received data
String receivedString = "";

// Function to clear text before writing new data
void clearTextOnDWIN(uint16_t vpAddr) {
    const char emptyText[MAX_TEXT_LENGTH + 1] = "                    ";  // 20 spaces to erase old text
    uint8_t header[] = {0x5A, 0xA5};  // DWIN header
    uint8_t addr[] = {(uint8_t)(vpAddr >> 8), (uint8_t)(vpAddr & 0xFF)};  // VP address
    uint8_t len = strlen(emptyText) + 3;  // Length = 3 (cmd+VP addr) + text length
    uint8_t buffer[50];  // Buffer to hold command
    uint8_t checksum = 0;

    // Construct command frame
    buffer[0] = header[0];
    buffer[1] = header[1];
    buffer[2] = len;  // Length
    buffer[3] = 0x82; // Write text command
    buffer[4] = addr[0]; // VP High byte
    buffer[5] = addr[1]; // VP Low byte

    // Copy empty text into buffer
    for (uint8_t i = 0; i < strlen(emptyText); i++) {
        buffer[6 + i] = emptyText[i];
    }

    // Calculate checksum
    for (uint8_t i = 2; i < (6 + strlen(emptyText)); i++) {
        checksum += buffer[i];
    }
    buffer[6 + strlen(emptyText)] = checksum;  // Add checksum at the end

    // Send clear command to DWIN HMI
    Serial2.write(buffer, 7 + strlen(emptyText));

    // Debug output on Serial3
    Serial3.print("Cleared text at VP: 0x");
    Serial3.println(vpAddr, HEX);
    delay(150);  // Short delay to allow clearing effect
}

// Function to send text to DWIN HMI
void sendTextToDWIN(uint16_t vpAddr, const char* text) {
    clearTextOnDWIN(vpAddr); // First, clear the text field before writing new text

    uint8_t header[] = {0x5A, 0xA5};  // DWIN header
    uint8_t addr[] = { (uint8_t)(vpAddr >> 8), (uint8_t)(vpAddr & 0xFF) };  // VP address
    uint8_t len = strlen(text) + 3;  // Length = 3 (cmd+VP addr) + text length
    uint8_t buffer[50];  // Buffer to hold command
    uint8_t checksum = 0;

    // Construct command frame
    buffer[0] = header[0];
    buffer[1] = header[1];
    buffer[2] = len;  // Length
    buffer[3] = 0x82; // Write text command
    buffer[4] = addr[0]; // VP High byte
    buffer[5] = addr[1]; // VP Low byte

    // Copy text into buffer
    for (uint8_t i = 0; i < strlen(text); i++) {
        buffer[6 + i] = text[i];
    }

    // Calculate checksum
    for (uint8_t i = 2; i < (6 + strlen(text)); i++) {
        checksum += buffer[i];
    }
    buffer[6 + strlen(text)] = checksum;  // Add checksum at the end

    // Send data to DWIN HMI
    Serial2.write(buffer, 7 + strlen(text));

    // Debugging
    Serial3.print("Sent to DWIN VP ");
    Serial3.print(vpAddr, HEX);
    Serial3.print(": ");
    Serial3.println(text);
}

// Function to update text color (Green for Connected, Red for Disconnected)
void changeTextColor(uint16_t vpAddr, uint16_t color) {
    uint8_t header[] = {0x5A, 0xA5};  // DWIN header
    uint8_t len = 0x01;  // Data length
    uint8_t command = 0x82; // Write command
    uint8_t buffer[8];  // Buffer to hold command

    buffer[0] = header[0];
    buffer[1] = header[1];
    buffer[2] = len;  
    buffer[3] = command;
    buffer[4] = (vpAddr >> 8) & 0xFF;  // VP High byte
    buffer[5] = vpAddr & 0xFF;         // VP Low byte
    buffer[6] = (color >> 8) & 0xFF;   // Color High byte
    buffer[7] = color & 0xFF;          // Color Low byte

    // Send color update command to DWIN HMI
    Serial2.write(buffer, 8);

    // Debugging output
    Serial3.print("Updated text color at VP: 0x");
    Serial3.print(vpAddr, HEX);
    Serial3.print(" to color: 0x");
    Serial3.println(color, HEX);
}


void setup() {
    Serial2.begin(115200);  // Initialize Serial2 for DWIN HMI
    Serial3.begin(115200);  // Initialize Serial3 for Debugging
    Serial1.begin(115200);  // Initialize Serial1 for receiving sensor data

    Serial3.println("STM32 Ready to receive sensor data from Serial1...");

    changeTextColor(0X400, 0XF800);

}

void loop() {
    // Check if Serial1 has available data
    while (Serial1.available()) {
        char receivedChar = Serial1.read();  // Read one character at a time

        // End of transmission when newline ('\n') is received
        if (receivedChar == '\n') {
            receivedString.trim();  // Remove extra spaces and newlines

            Serial3.print("Received Data: ");
            Serial3.println(receivedString);

            // Process the received string
            processSensorStatus(receivedString);

            // Clear the buffer for the next message
            receivedString = "";
        } else {
            // Append the received character to the string
            receivedString += receivedChar;
        }
    }
}

// Function to process the received sensor status string
void processSensorStatus(String status) {
    if (status.length() < 7) {
        Serial3.println("Error: Incorrect data length!");
        return;
    }

    // Map received data to specific faults
    Differential = (status[0] == '1') ? 1 : 0;
    BuchholzRelay = (status[1] == '1') ? 1 : 0;
    MasterTrip = (status[2] == '1') ? 1 : 0;
    REF = (status[3] == '1') ? 1 : 0;
    OverCurrent = (status[4] == '1') ? 1 : 0;
    PRV = (status[5] == '1') ? 1 : 0;
    RPRR = (status[6] == '1') ? 1 : 0;
    
    // Send status to LCD
    for (int i = 0; i < 7; i++) {
        sendTextToDWIN(VP_TEXT[i], (status[i] == '1') ? "Normal" : "Disconnected");

       //(VP_COLOR[i], status[i] == '1');  // Update text color
    }
}

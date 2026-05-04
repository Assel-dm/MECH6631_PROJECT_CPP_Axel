// program.cpp — Hardware test program for manual robot control via Bluetooth
// Use arrow keys to drive the robot via Bluetooth HC-06 serial connection
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <cstdio>
#include <iostream>
#include <Windows.h>

using namespace std;

#include "Types.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

// ============================================================
// Bluetooth serial helpers
// ============================================================

static HANDLE openSerial(const char* port, DWORD baud)
{
    HANDLE h = CreateFileA(port, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                           OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
    if (h == INVALID_HANDLE_VALUE) {
        cerr << "ERROR: Could not open " << port << endl;
        return INVALID_HANDLE_VALUE;
    }
    DCB dcb = {};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(h, &dcb);
    dcb.BaudRate = baud;
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity   = NOPARITY;
    SetCommState(h, &dcb);

    COMMTIMEOUTS timeouts = {};
    timeouts.WriteTotalTimeoutConstant   = 20;
    timeouts.WriteTotalTimeoutMultiplier = 2;
    SetCommTimeouts(h, &timeouts);

    cout << "Bluetooth serial opened on " << port << " @ " << baud << " baud" << endl;
    return h;
}

// Sends a 6-byte motor command packet to the Arduino.
// Packet format: [0xFF][pw_l_hi][pw_l_lo][pw_r_hi][pw_r_lo][laser]
// Left: 1500µs neutral, right: 1480µs neutral (inverted - mirrored servos)
static void sendCommand(HANDLE hSerial, const Command& cmd)
{
    if (hSerial == INVALID_HANDLE_VALUE) return;

    int pw_l = vel_to_pw_left(cmd.left);
    int pw_r = vel_to_pw_right(cmd.right);

    uint8_t buf[6] = {
        0xFF,
        (uint8_t)(pw_l >> 8), (uint8_t)(pw_l & 0xFF),
        (uint8_t)(pw_r >> 8), (uint8_t)(pw_r & 0xFF),
        (uint8_t)(cmd.laser ? 1 : 0)
    };
    DWORD written;
    WriteFile(hSerial, buf, sizeof(buf), &written, NULL);
}

// ============================================================
// Main
// ============================================================
int main()
{
    SetConsoleOutputCP(CP_UTF8);

    cout << "\n========================================" << endl;
    cout << "   HARDWARE TEST — BLUETOOTH CONTROL" << endl;
    cout << "========================================" << endl;

    // ============================================================
    // Configuration
    // ============================================================
    const char*  BT_PORT    = "COM5";   // ← Your Bluetooth COM port
    const DWORD  BT_BAUD    = 9600;     // HC-06 baud rate
    const double SPEED      = 0.8;      // Motor speed (0.0 to 1.0)
    const double TURN_SPEED = 0.6;      // Turn speed (0.0 to 1.0)

    cout << "\nAttempting to connect on " << BT_PORT << "..." << endl;

    HANDLE hSerial = openSerial(BT_PORT, BT_BAUD);
    if (hSerial == INVALID_HANDLE_VALUE) {
        cout << "\nFailed to open Bluetooth port!" << endl;
        cout << "1. Check Device Manager for correct COM port" << endl;
        cout << "2. Ensure HC-06 is paired (PIN: 1234)" << endl;
        cout << "3. Disconnect HC-06 from pins 0/1 before uploading Arduino sketch" << endl;
        cout << "\nPress any key to exit..." << endl;
        cin.get();
        return 1;
    }

    // Wait for Arduino to boot
    cout << "\nWaiting for Arduino initialization..." << endl;
    Sleep(2000);

    cout << "\n✓ Connected!" << endl;
    cout << "\nPress SPACE to begin driving..." << endl;

    while (!KEY(VK_SPACE)) {
        Sleep(50);
    }

    cout << "\n=== CONTROLS ===" << endl;
    cout << "  UP ARROW    : Forward" << endl;
    cout << "  DOWN ARROW  : Backward" << endl;
    cout << "  LEFT ARROW  : Turn left" << endl;
    cout << "  RIGHT ARROW : Turn right" << endl;
    cout << "  SPACE       : Stop" << endl;
    cout << "  L           : Toggle laser" << endl;
    cout << "  X           : Exit" << endl;
    cout << "\nReady! Use arrow keys to control the robot." << endl;
    cout << "========================================\n" << endl;

    bool laser_on = false;
    Command cmd   = { 0.0, 0.0, false };

    // One-shot print flags — print direction once per key press
    bool up_was    = false;
    bool down_was  = false;
    bool left_was  = false;
    bool right_was = false;
    bool l_was     = false;

    while (true)
    {
        // Exit
        if (KEY('X')) {
            cout << "Exiting..." << endl;
            break;
        }

        // Laser toggle
        if (KEY('L')) {
            if (!l_was) {
                laser_on = !laser_on;
                cout << "Laser: " << (laser_on ? "ON" : "OFF") << endl;
                l_was = true;
            }
        } else {
            l_was = false;
        }

        // Reset command each loop
        cmd.left  = 0.0;
        cmd.right = 0.0;
        cmd.laser = laser_on;

        // Arrow key controls
        if (KEY(VK_UP)) {
            cmd.left  = SPEED;
            cmd.right = SPEED;
            if (!up_was) { cout << "Forward" << endl; up_was = true; }
            down_was = left_was = right_was = false;
        }
        else if (KEY(VK_DOWN)) {
            cmd.left  = -SPEED;
            cmd.right = -SPEED;
            if (!down_was) { cout << "Backward" << endl; down_was = true; }
            up_was = left_was = right_was = false;
        }
        else if (KEY(VK_LEFT)) {
            cmd.left  = -TURN_SPEED;
            cmd.right =  TURN_SPEED;
            if (!left_was) { cout << "Turn left" << endl; left_was = true; }
            up_was = down_was = right_was = false;
        }
        else if (KEY(VK_RIGHT)) {
            cmd.left  =  TURN_SPEED;
            cmd.right = -TURN_SPEED;
            if (!right_was) { cout << "Turn right" << endl; right_was = true; }
            up_was = down_was = left_was = false;
        }
        else if (KEY(VK_SPACE)) {
            cmd.left  = 0.0;
            cmd.right = 0.0;
            up_was = down_was = left_was = right_was = false;
        }
        else {
            up_was = down_was = left_was = right_was = false;
        }

        sendCommand(hSerial, cmd);
        Sleep(50);
    }

    // Stop robot and clean up
    cout << "Stopping robot..." << endl;
    sendCommand(hSerial, { 0.0, 0.0, false });
    Sleep(100);

    CloseHandle(hSerial);

    cout << "Done." << endl;
    cout << "Press any key to exit..." << endl;
    cin.get();
    return 0;
}
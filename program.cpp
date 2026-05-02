// program.cpp — Physical robot entry point
// All strategy computation runs on-PC; commands are sent to Arduino over Bluetooth serial.
#define NOMINMAX
#define _USE_MATH_DEFINES
#include <cstdio>
#include <iostream>
#include <cmath>
#include <Windows.h>

using namespace std;

#include "image_transfer.h"
#include "vision.h"
#include "timer.h"
#include "Types.h"
#include "StrategyEngine.h"
#include "ProgramVariant.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

// ============================================================
// Bluetooth serial helpers
// ============================================================

// Opens a Windows serial (COM) port for Bluetooth communication with the Arduino.
// Returns INVALID_HANDLE_VALUE on failure.
static HANDLE openSerial(const char* port, DWORD baud)
{
    HANDLE h = CreateFileA(port, GENERIC_WRITE, 0, NULL,
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
// Pulse widths are derived from normalized velocities in [-1, 1] via vel_to_pw().
static void sendCommand(HANDLE hSerial, const Command& cmd)
{
    if (hSerial == INVALID_HANDLE_VALUE) return;
    int pw_l = vel_to_pw(cmd.left);
    int pw_r = vel_to_pw(cmd.right);
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
    cout << "   PHYSICAL ROBOT — STRATEGY ENGINE" << endl;
    cout << "========================================" << endl;

    // ============================================================
    // Configuration — adjust these before each physical test run
    // ============================================================

    // PHYS_MODE 0: navigation/detection tuning only (no full strategy)
    // PHYS_MODE 1: full offense/defense strategy active
    const int    PHYS_MODE = PROGRAM_PHYS_MODE;

    const char*  BT_PORT   = "COM1";   // Windows COM port for Bluetooth (check Device Manager)
    const DWORD  BT_BAUD   = 9600;     // Must match Arduino baud rate
    const int    CAM_INDEX = 0;        // Camera index (try 1 or 2 if 0 fails)
    const int    CAM_W     = 1920;
    const int    CAM_H     = 1080;
    const double TEST_DURATION = 120.0; // Auto-stop after this many seconds (0 = disabled)

    // Navigation tuning target (pixels) — only used in PHYS_MODE 0
    const double NAV_GOAL_X = 960.0;   // Image centre X
    const double NAV_GOAL_Y = 540.0;   // Image centre Y

    // ============================================================
    // Open Bluetooth serial
    // ============================================================
    HANDLE hSerial = openSerial(BT_PORT, BT_BAUD);

    cout << "\nPress SPACE to begin..." << endl;
    pause();

    // ============================================================
    // Activate vision (webcam)
    // ============================================================
    activate_vision();

    int cam_err = activate_camera(CAM_INDEX, CAM_H, CAM_W);
    if (cam_err != 0) {
        cout << "ERROR: activate_camera failed (code " << cam_err << "). Try CAM_INDEX 1 or 2." << endl;
        deactivate_vision();
        pause();
        return 1;
    }
    cout << "Camera " << CAM_INDEX << " opened OK." << endl;

    // Allocate image buffer for the raw webcam frame
    image rgb;
    rgb.type   = RGB_IMAGE;
    rgb.width  = CAM_W;
    rgb.height = CAM_H;
    allocate_image(rgb);

    // ============================================================
    // Strategy engine — wraps all detection, planning, and fuzzy logic
    // ============================================================
    StrategyEngine engine;
    engine.setForcedMode(PROGRAM_MODE);
    engine.setKnownProfile(PROGRAM_MY_PROFILE);
    engine.setUseIDDance(PROGRAM_USE_ID_DANCE != 0);

    // ============================================================
    // Main loop
    // ============================================================
    double tc0 = high_resolution_time();
    double tc  = 0.0;
    int    frame_count = 0;
    bool   running = true;

    cout << "\n=== PHYSICAL TEST STARTED ===" << endl;
    cout << "Press 'X' to stop" << endl << endl;

    while (running)
    {
        if (KEY('X')) {
            cout << "\nStopped by user." << endl;
            break;
        }

        tc = high_resolution_time() - tc0;
        if (TEST_DURATION > 0.0 && tc > TEST_DURATION) {
            cout << "\nTest duration reached." << endl;
            break;
        }

        // Grab the latest frame from the webcam
        acquire_image(rgb, CAM_INDEX);

        // ---- Strategy update ----
        // The selected variant is controlled by ProgramVariant.h.
        // Known-profile programs skip ID dance; challenge programs identify our robot
        // internally using the profile-aware ID dance in StrategyEngine.
        Command cmd = engine.update(rgb, tc);
        sendCommand(hSerial, cmd);

        if (frame_count % 30 == 0) {
            const char* mode_txt = (PROGRAM_MODE == MODE_OFFENSE) ? "OFFENSE" :
                                   (PROGRAM_MODE == MODE_DEFENSE) ? "DEFENSE" : "AUTO";
            cout << mode_txt << " | t=" << (int)tc << "s | frame=" << frame_count
                 << " | L=" << cmd.left << " R=" << cmd.right
                 << " | laser=" << cmd.laser << endl;
        }

        // Display the live camera feed
        view_rgb_image(rgb);

        frame_count++;
        // No Sleep() — acquire_image() blocks until the next camera frame is ready.
        // Strategy runs at the native camera framerate.
    }

    // ============================================================
    // Stop robot and clean up on exit
    // ============================================================
    sendCommand(hSerial, { 0.0, 0.0, false });

    if (hSerial != INVALID_HANDLE_VALUE) CloseHandle(hSerial);
    stop_camera(CAM_INDEX);
    free_image(rgb);
    deactivate_vision();

    cout << "\nDuration: " << tc << "s | Frames: " << frame_count << endl;
    cout << "Press any key to exit..." << endl;
    pause();
    return 0;
}
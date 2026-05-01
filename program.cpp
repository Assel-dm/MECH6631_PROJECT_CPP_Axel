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
#include "IDDance.h"
#include "MarkerDetector.h"
#include "Tracking.h"
#include "RunLogger.h"

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

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

int main()
{
    SetConsoleOutputCP(CP_UTF8);

    cout << "\n========================================" << endl;
    cout << "   PHYSICAL ROBOT — STRATEGY ENGINE" << endl;
    cout << "========================================" << endl;

    const int    PHYS_MODE = 1;

    const char* BT_PORT = "COM1";   // Windows COM port for Bluetooth (check Device Manager)
    const DWORD  BT_BAUD = 9600;     // Must match Arduino baud rate
    const int    CAM_INDEX = 0;        // Camera index (try 1 or 2 if 0 fails)
    const int    CAM_W = 1280;
    const int    CAM_H = 960;
    const double TEST_DURATION = 120.0; // Auto-stop after this many seconds (0 = disabled)


    // Diagnostics output. After the run, execute:
    //   python plot_diagnostics.py run_diagnostics.csv
    const bool   ENABLE_LOGGING = true;
    const char*  LOG_FILE = "run_diagnostics.csv";

    const double NAV_GOAL_X = 960.0;
    const double NAV_GOAL_Y = 540.0;

    HANDLE hSerial = openSerial(BT_PORT, BT_BAUD);

    cout << "\nPress SPACE to begin..." << endl;
    pause();

    activate_vision();

    int cam_err = activate_camera(CAM_INDEX, CAM_H, CAM_W);
    if (cam_err != 0) {
        cout << "ERROR: activate_camera failed (code " << cam_err << "). Try CAM_INDEX 1 or 2." << endl;
        deactivate_vision();
        pause();
        return 1;
    }
    cout << "Camera " << CAM_INDEX << " opened OK." << endl;

    image rgb;
    rgb.type   = RGB_IMAGE;
    rgb.width  = CAM_W;
    rgb.height = CAM_H;
    allocate_image(rgb);

    StrategyEngine engine;

    RunLogger logger;
    if (ENABLE_LOGGING) {
        if (logger.open(LOG_FILE)) {
            cout << "Diagnostics logging enabled: " << LOG_FILE << endl;
        } else {
            cout << "WARNING: could not open diagnostics log file." << endl;
        }
    }

    IDDance   id_dance;
    MarkerDetector detector;
    Tracker   tracker;
    int       my_id = -1;
    std::vector<RobotTrack> tracks;

    double tc0 = high_resolution_time();
    double tc  = 0.0;
    double prev_loop_t = tc0;
    int    frame_count = 0;
    bool   running = true;

    cout << "\n=== PHYSICAL TEST STARTED ===" << endl;
    cout << "Press 'X' to stop" << endl << endl;

    while (running)
    {
        double loop_start_abs = high_resolution_time();

        if (KEY('X')) {
            cout << "\nStopped by user." << endl;
            break;
        }

        tc = loop_start_abs - tc0;
        if (TEST_DURATION > 0.0 && tc > TEST_DURATION) {
            cout << "\nTest duration reached." << endl;
            break;
        }

        acquire_image(rgb, CAM_INDEX);
        Command last_cmd{0.0, 0.0, false};

        if (my_id < 0) {
            my_id = id_dance.run(tc, rgb, detector, tracker);
            Command cmd = id_dance.currentCommand();
            sendCommand(hSerial, cmd);
            last_cmd = cmd;
            if (id_dance.done()) {
                engine.setID(my_id);
                cout << "ID dance complete — Robot ID: " << my_id << endl;
                if (PHYS_MODE == 0)
                    cout << "Mode: NAVIGATION TUNING — driving to (" << NAV_GOAL_X << ", " << NAV_GOAL_Y << ")" << endl;
                else
                    cout << "Mode: FULL STRATEGY" << endl;
            }
        }
        else if (PHYS_MODE == 0) {
            Command cmd = engine.update(rgb, tc);
            sendCommand(hSerial, cmd);
            last_cmd = cmd;

            if (frame_count % 30 == 0) {
                cout << "NAV | t=" << (int)tc << "s | frame=" << frame_count
                     << " | L=" << cmd.left << " R=" << cmd.right << endl;
            }
        }
        else {
            Command cmd = engine.update(rgb, tc);
            sendCommand(hSerial, cmd);
            last_cmd = cmd;

            if (frame_count % 30 == 0) {
                cout << "STRATEGY | t=" << (int)tc << "s | frame=" << frame_count
                     << " | L=" << cmd.left << " R=" << cmd.right
                     << " | laser=" << cmd.laser << endl;
            }
        }

        double loop_end_abs = high_resolution_time();
        double loop_ms = (loop_end_abs - loop_start_abs) * 1000.0;
        double dt = loop_start_abs - prev_loop_t;
        double fps = (dt > 1e-9) ? (1.0 / dt) : 0.0;
        prev_loop_t = loop_start_abs;
        if (logger.isOpen()) {
            logger.log(tc, frame_count, last_cmd, loop_ms, fps, engine.debugInfo());
        }

        view_rgb_image(rgb);
        frame_count++;
    }

    sendCommand(hSerial, { 0.0, 0.0, false });
    logger.close();

    if (hSerial != INVALID_HANDLE_VALUE) CloseHandle(hSerial);
    stop_camera(CAM_INDEX);
    free_image(rgb);
    deactivate_vision();

    cout << "\nDuration: " << tc << "s | Frames: " << frame_count << endl;
    if (ENABLE_LOGGING) {
        cout << "Diagnostics saved to: " << LOG_FILE << endl;
        cout << "To generate plots: python plot_diagnostics.py " << LOG_FILE << endl;
    }
    cout << "Press any key to exit..." << endl;
    pause();
    return 0;
}

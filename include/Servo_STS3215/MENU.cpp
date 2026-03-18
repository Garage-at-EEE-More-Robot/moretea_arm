#include <iostream>
#include <cstdlib>
#include <string>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include "SCServo.h"
#include "SMS_STS.h"
#include "SCSCL.h"
#include "SCSerial.h"
#include <vector>
#include <unistd.h>   // for usleep
#include <filesystem>
#include <algorithm>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>
using namespace std;
namespace fs = filesystem;
SMS_STS sm_st;  // reuse global servo object

// ---------- non-blocking keypress ----------
bool key_pressed()
{
    timeval tv {0, 0};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv);
    return FD_ISSET(STDIN_FILENO, &fds);
}

char get_char()
{
    char c;
    read(STDIN_FILENO, &c, 1);
    return c;
}

// ---------- RECORD TRAJECTORY ----------
int record_trajectory(const string& file_name)
{
    vector<int> servo_ids;

    // Detect servos (IDs 1..6)
    for (int id = 1; id <= 6; id++) {
        if (sm_st.Ping(id) != -1) {
            servo_ids.push_back(id);
            sm_st.EnableTorque(id, 0);  // free-move
        }
    }

    if (servo_ids.empty()) {
        cout << "No servos detected\n";
        return 1;
    }

    cout << "Capture mode started...\n";
    cout << "Detected servos: ";
    for (int id : servo_ids) cout << id << " ";
    cout << "\n";
    cout << "Press 'w' to capture positions (one waypoint)\n";
    cout << "Press 'q' to stop and save\n";

    ofstream file(file_name);
    if (!file) {
        cout << "Failed to open/create " << file_name << "\n";
        return 1;
    }

    file << "trajectory:\n";
    int step = 0;

    // Set terminal to raw mode
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (true) {
        if (key_pressed()) {
            char c = get_char();

            if (c == 'q' || c == 'Q') {
                break;
            }

            if (c == 'w' || c == 'W') {
                // Write ONE captured waypoint
                file << "  - i: " << step++ << "\n";
                file << "    servos:\n";

                cout << "\nCaptured #" << step << " -> ";

                for (int id : servo_ids) {
                    int pos = sm_st.ReadPos(id);
                    if (pos != -1) {
                        file << "      " << id << ": " << pos << "\n";
                        cout << id << ":" << pos << " ";
                    } else {
                        // optional: warn if a servo read failed
                        cout << id << ":(readfail) ";
                    }
                }

                cout << "\n";
                file.flush(); // ensure it writes immediately
            }
        }

        usleep(10000); // prevent busy loop
    }

    // Restore terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    file.close();
    cout << "Saved " << step << " waypoints to " << file_name << "\n";
    return 0;
}

string auto_detect_serial()
{
    const string base = "/dev/serial/by-id/";

    if (!fs::exists(base)) {
        return "";
    }

    for (const auto& entry : fs::directory_iterator(base)) {
        return entry.path().string(); // take first device
    }

    return "";
}
bool wait_until_reached(int id, int target)
{
    const int TOLERANCE = 50;        // ~4–5 degrees
    const int TIMEOUT_MS = 6000;
    const int POLL_US = 10000;

    int elapsed = 0;

    while (elapsed < TIMEOUT_MS * 1000) {
        int pos = sm_st.ReadPos(id);

        // If read failed, just wait and retry
        if (pos == -1) {
            usleep(POLL_US);
            elapsed += POLL_US;
            continue;
        }

        int err = abs(pos - target);

        if (err <= TOLERANCE) {
            return true;
        }

        usleep(POLL_US);
        elapsed += POLL_US;
    }

    return false; // timeout
}


int servo_move_id(int argc, char* argv[])
{
    const int POS_MIN = 0;
    const int POS_MAX = 4095;
    const int SERVO_SPEED_MAX = 100;
    const int ACC = 200;

    if (argc != 6) {
        cout << "Usage:\n";
        cout << "./servo_move servo_move_id <id> <pos1> <pos2> <speed>\n";
        return 1;
    }

    int id         = atoi(argv[2]);
    int pos1       = atoi(argv[3]);
    int pos2       = atoi(argv[4]);
    int user_speed = atoi(argv[5]);
    pos1 = clamp(pos1, 0, 4095);
    pos2 = clamp(pos2, 0, 4095);
    int speed = min(user_speed, SERVO_SPEED_MAX);

    // ---- FORCE POSITION MODE ----

    usleep(5000);


    sm_st.EnableTorque(id, 1);
    usleep(10000);


    int cur = sm_st.ReadPos(id);
    cout << "Servo " << id << " detected at position " << cur << endl;

    if (pos1 >= POS_MIN && pos1 <= POS_MAX &&
    pos2 >= POS_MIN && pos2 <= POS_MAX)
    {
        cout << "Moving to pos1: " << pos1 << endl;
        sm_st.WritePosEx(id, pos1, speed, ACC);
        wait_until_reached(id, pos1);

        cout << "Moving to pos2: " << pos2 << endl;
        sm_st.WritePosEx(id, pos2, speed, ACC);
        wait_until_reached(id, pos2);

        cout << "Servo " << id << " sequence done\n";
    }
    else
    {
        cout << "[SKIP] Unsafe positions: "
            << "pos1=" << pos1 << ", pos2=" << pos2 << endl;
    }
    return 0;
}


int play(int argc, char* argv[]){
    if (argc != 3) {
        cout << "Usage: ./servo_move play <file.yaml>\n";
        return 1;
    }

    const int IGNORE_ID = 6;   // ✅ ignore servo 6

    string file_name = argv[2];

    YAML::Node doc;
    try {
        doc = YAML::LoadFile(file_name);
    } catch (...) {
        cout << "Failed to load YAML file: " << file_name << "\n";
        return 1;
    }

    if (!doc["trajectory"] || !doc["trajectory"].IsSequence() || doc["trajectory"].size() == 0) {
        cout << "Invalid or empty trajectory file (missing trajectory: [ ... ])\n";
        return 1;
    }

    int step_us = 20000;
    if (doc["sample_us"]) {
        try {
            step_us = doc["sample_us"].as<int>();
            if (step_us <= 0) step_us = 20000;
        } catch (...) {
            step_us = 20000;
        }
    }

    const uint16_t ALIGN_SPEED = 150;
    const uint16_t ALIGN_ACC   = 200;
    const uint16_t PLAY_SPEED  = 290;
    const uint16_t PLAY_ACC    = 200;

    vector<int> active_ids;
    auto seen_id = [&](int id) {
        return find(active_ids.begin(), active_ids.end(), id) != active_ids.end();
    };

    // -------- GET FIRST POSE --------
    YAML::Node first_step = doc["trajectory"][0];
    if (!first_step["servos"] || !first_step["servos"].IsMap() || first_step["servos"].size() == 0) {
        cout << "First trajectory step missing 'servos' map\n";
        return 1;
    }
    YAML::Node first_pose = first_step["servos"];

    cout << "Aligning to initial pose...\n";

    // Enable torque + align to first pose (skip ID 6)
    for (const auto& kv : first_pose) {
        int id     = kv.first.as<int>();
        int target = kv.second.as<int>();

        if (id == IGNORE_ID) continue;  // ✅ skip

        if (!seen_id(id)) {
            active_ids.push_back(id);
            sm_st.EnableTorque(id, 1);
        }

        int current = sm_st.ReadPos(id);
        cout << "Servo " << id << " current=" << current << " target=" << target << "\n";

        sm_st.WritePosEx(id, target, ALIGN_SPEED, ALIGN_ACC);
    }

    // Wait for alignment to complete (skip ID 6)
    for (const auto& kv : first_pose) {
        int id     = kv.first.as<int>();
        int target = kv.second.as<int>();

        if (id == IGNORE_ID) continue;  // ✅ skip

        if (!wait_until_reached(id, target)) {
            cout << "[WARN] Align timeout on servo " << id << " target=" << target << "\n";
        }
    }

    cout << "Starting trajectory playback...\n";

    // -------- PLAYBACK LOOP --------
    for (size_t si = 0; si < doc["trajectory"].size(); si++) {
        const YAML::Node& step = doc["trajectory"][si];

        if (!step["servos"] || !step["servos"].IsMap()) {
            cout << "[WARN] Step " << si << " missing/invalid 'servos' map, skipping\n";
            continue;
        }

        const YAML::Node& servos = step["servos"];

        // Send commands first (skip ID 6)
        for (const auto& kv : servos) {
            int id  = kv.first.as<int>();
            int pos = kv.second.as<int>();

            if (id == IGNORE_ID) continue;  // ✅ skip

            if (pos < 0) pos = 0;
            if (pos > 4095) pos = 4095;

            if (!seen_id(id)) {
                active_ids.push_back(id);
                sm_st.EnableTorque(id, 1);
            }

            sm_st.WritePosEx(id, pos, PLAY_SPEED, PLAY_ACC);
        }

        // Then wait until reached (skip ID 6)
        for (const auto& kv : servos) {
            int id  = kv.first.as<int>();
            int pos = kv.second.as<int>();

            if (id == IGNORE_ID) continue;  // ✅ skip

            if (pos < 0) pos = 0;
            if (pos > 4095) pos = 4095;

            if (!wait_until_reached(id, pos)) {
                cout << "[WARN] Step " << si << " timeout on servo " << id << " target=" << pos << "\n";
            }
        }

        if (step_us > 0) usleep(step_us);
    }

    // Keep torque ON after playback (only for active_ids, which won't include 6)
    for (int id : active_ids) {
        sm_st.EnableTorque(id, 1);
    }

    cout << "Playback finished\n";
    return 0;
}


int ping_all_servos(int argc, char **argv)
{
    (void)argc;
    (void)argv;
    cout << "Scanning for servos...\n";
    bool found_any = false;

    for (int id = 1; id <= 6; id++) {
        int r = sm_st.Ping(id);
        if (r != -1) {
            cout << "Found servo ID: " << r << endl;
            found_any = true;
        }
    }

    if (!found_any) {
        cout << "No servos found.\n";
    }
    else {
        cout << "Scan complete.\n";
    }

    return 0;

}
int show_menu() {
    cout << "\n====== Servo Control Menu ======\n";
    cout << "1. Move Servo by ID\n";
    cout << "2. Record Trajectory (record <file>.yaml)\n";
    cout << "3. Play Trajectory\n";
    cout << "4. Ping All Servos\n";
    cout << "5. Detect Servo Angles\n";
    cout << "6. Detect specific servo torque load\n";
    cout << "7. Record Gripper\n";
    cout << "8. Play Gripper Files\n";
    cout << "9.TO BE UPDATED\n";
    cout << "0. Exit\n";
    cout << "Select option: ";

    int choice;
    cin >> choice;
    return choice;
}

int detect_all_servos(int argc, char **argv){
    cout << "Reading servo angles...\n";
    bool found_any = false;

    for (int id = 1; id <= 6; id++) {
        int pos = sm_st.ReadPos(id);
        if (pos != -1) 
        {
            float angle = pos * 360.0f / 4096.0f;
            cout << "\nServo ID: "<< id 
            << "\nRaw : " << pos
            << "\nAngle:" << angle << "deg\n";
            found_any = true;
        }
        usleep(3000); // 3 ms between reads

    }
    if (!found_any) {
        cout << "No servos detected.\n";
    }
    return 0;
}

int detectLT(int argc, char* argv[]){
    if (argc != 3) {
        cout << "Usage:\n";
        cout << "./servo_move detect_torq <id> \n";
        return 1;
    }

    int servo_id = atoi(argv[2]);
    cout << "Detecting Torque from servo"<< servo_id<<"(, Press 'Q' to quit)\n";
    sm_st.EnableTorque(servo_id, 1);

        // Set terminal to raw mode
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    while (true) {
        if (key_pressed()) {
            char c = get_char();
            if (c == 'q' || c == 'Q') {
                break;
            }

            cout <<endl;
        }
        int load = sm_st.ReadLoad(servo_id);
        cout << "\rServo: "<<servo_id <<" | Load:"<<load<<"    "<<flush;
        usleep(50000);
    }
    sm_st.EnableTorque(servo_id, 0);
    // Restore terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);


    return 0;

}

// ---------- RECORD GRIPPER ----------
// ---------- RECORD GRIPPER (press 'w' to capture positions) ----------
// ---------- RECORD GRIPPER (press 'w' to capture, YAML compatible with normal play()) ----------
// ---------- RECORD GRIPPER (press 'w' to capture, YAML compatible with normal play()) ----------
int record_gripper(const string& file_name)
{
    const int gripper_id = 6;

    // Free-move / hand guide
    sm_st.EnableTorque(gripper_id, 0);

    cout << "Capture started (gripper only)...\n";
    cout << "Press 'w' to capture current gripper position\n";
    cout << "Press 'q' to stop and save\n";

    ofstream file(file_name);
    if (!file) {
        cout << "Failed to open/create " << file_name << "\n";
        return 1;
    }

    // EXACT header & formatting like your example
    file << "trajectory:\n";

    // Set terminal to raw mode
    termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int step = 0;

    while (true) {
        if (key_pressed()) {
            char c = get_char();

            if (c == 'q' || c == 'Q') break;

            if (c == 'w' || c == 'W') {
                int pos = sm_st.ReadPos(gripper_id);
                if (pos == -1) {
                    cout << "\n[WARN] ReadPos failed, try again.\n";
                } else {
                    file << "  - i: " << step++ << "\n";
                    file << "    servos:\n";
                    file << "      " << gripper_id << ": " << pos << "\n";
                    file.flush();

                    cout << "\nCaptured i=" << (step - 1) << " pos=" << pos << "\n";
                }
            }
        }

        usleep(10000); // prevent busy loop
    }

    // Restore terminal
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    file.close();
    cout << "Recording saved to " << file_name << "\n";
    return 0;
}

bool wait_until_reached_or_load(int id, int target, int load_limit, int tol = 8, int timeout_ms = 4000)
{
    const int sleep_us = 10000;
    int elapsed_ms = 0;

    while (elapsed_ms < timeout_ms) {
        int pos  = sm_st.ReadPos(id);
        int load = sm_st.ReadLoad(id);

        if (abs(load) >= load_limit) {
            cout << "[GRIP] Load hit (" << load << "). Stop closing.\n";
            return true; // treat as "done gripping"
        }

        if (abs(pos - target) <= tol) {
            return true; // reached target normally
        }

        usleep(sleep_us);
        elapsed_ms += sleep_us / 1000;
    }

    return false; // timeout
}

// ---------- PLAY GRIPPER (simple + wait_until_reached) ----------
int play_gripper(int argc, char* argv[])
{

    if (argc != 3) {
        cout << "Usage: ./servo_move play_gripper <file.yaml>\n";
        return 1;
    }
    int current, current_now,hold_pos;
    const int GRIPPER_ID = 6;
    const uint16_t SPEED = 200;
    const uint16_t ACC   = 300;
    int load;
    string file_name = argv[2];

    YAML::Node doc;
    try {
        doc = YAML::LoadFile(file_name);
    } catch (...) {
        cout << "Failed to load YAML file: " << file_name << "\n";
        return 1;
    }

    if(!doc["trajectory"] || !doc["trajectory"].IsSequence()||doc["trajectory"].size() ==0){
        cout << "Invalid YAML: missing trajectory[]\n";
        return 1;
    }
    YAML::Node step0 = doc["trajectory"][0];
    if (!step0["servos"] || !step0["servos"].IsMap() || !step0["servos"][GRIPPER_ID]) {
        cout << "Invalid YAML: missing servos map or servo ID 6 in step 0\n";
        return 1;
    }
    int target = step0["servos"][GRIPPER_ID].as<int>();
    target = clamp(target,0,4095);

    sm_st.EnableTorque(GRIPPER_ID, 1);
    sm_st.WritePosEx(GRIPPER_ID, target, SPEED, ACC);

    if (!wait_until_reached_or_load(GRIPPER_ID, target, 100)) {
        cout << "[WARN] Timeout\n";
        return 1;
    }

    cout << "Latching for 5s...\n";
    usleep(5000000);
    sm_st.EnableTorque(GRIPPER_ID, 1); // or keep ON if you want it to keep holding

    return 0;
}


int main(int argc, char* argv[])
{
    // ================= MENU MODE =================
    if (argc == 1) {
        while (true) {
            int choice = show_menu();

            if (choice == 0) {
                cout << "Exiting menu.\n";
                break;
            }

            switch (choice) {
                case 1:
                    cout << "./servo_move servo_move_id <id> <pos1> <pos2> <speed>\n";
                    break;
                case 2:
                    cout << "./servo_move record <file_name> \n";
                    break;
                case 3:
                    cout << "./servo_move play <file_name>\n";
                    break;
                case 4:
                    cout << "./servo_move ping_all_servos\n";
                    break;
                case 5:
                    cout << "./servo_move detect_all_servos\n";
                    break;
                case 6:
                    cout << "./servo_move detect_torq <ID> \n";
                    break;
                case 7:
                    cout <<"./sero_move record_gripper file_name.yaml\n";
                case 8:
                    cout <<"./sero_move play_gripper file_name.yaml\n";
                default:
                    cout << "Invalid option\n";
            }
        }
        return 0;
    }

    // ================= ARGUMENT MODE =================
    string command = argv[1];

    // ---------- AUTO-DETECT SERIAL ----------
    string serial = auto_detect_serial();
    if (serial.empty()) {
        cout << "No serial device detected\n";
        return 1;
    }

    // ---------- OPEN BUS ONCE ----------
    if (!sm_st.begin(1000000, serial.c_str())) {
        cout << "Failed to open servo bus\n";
        return 1;
    }

    // ---------- DISPATCH ----------
    int rc = 0;

    if (command == "servo_move_id") {
        rc = servo_move_id(argc, argv);
    }
// If argument ends with .yaml → record trajectory
    else if (command == "record" && argc == 3) {
    string filename = argv[2];

        if (filename.size() > 5 &&
            filename.substr(filename.size() - 5) == ".yaml") 
        {
            rc = record_trajectory(filename);
        } 
        else 
        {
            cout << "File must end with .yaml\n";
            rc = 1;
            }
    }

    else if (command == "play") {
        rc = play(argc, argv);
    }
    else if (command == "ping_all_servos") {
        cout << "You have chosen to ping available servos.\n";
        rc = ping_all_servos(argc, argv);
    }
    else if (command == "detect_all_servos") {
        rc = detect_all_servos(argc, argv);
    }
    else if (command == "detect_torq") {
        rc = detectLT(argc, argv);
    }

    else if (command == "record_gripper" && argc == 3) {
        string filename = argv[2];

        if (filename.size() > 5 && filename.substr(filename.size() - 5) == ".yaml") 
        {
            rc = record_gripper(filename);
        } 
        else 
        {
            cout << "File must end with .yaml\n";
            rc = 1;
            }
    }
    else if (command == "play_gripper" && argc == 3) {
        rc = play_gripper(argc, argv);
}
    else {
        cout << "Unknown command: " << command << endl;
        rc = 1;
    }

    // ---------- CLOSE BUS ONCE ----------
    sm_st.end();
    return rc;
}

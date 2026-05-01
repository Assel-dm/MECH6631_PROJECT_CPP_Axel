#include "RunLogger.h"
#include <iomanip>

RunLogger::RunLogger() {}

bool RunLogger::open(const std::string& filename)
{
    file_.open(filename.c_str());
    if (!file_.is_open()) return false;
    file_ << "t,frame,pwm_left,pwm_right,laser,loop_ms,fps,"
          << "num_tracks,num_obstacles,my_id,enemy_id,offense_mode,boundary_guard,"
          << "my_x,my_y,my_theta,enemy_x,enemy_y,enemy_dist_px,enemy_bearing_deg,nearest_obstacle_dist_px\n";
    return true;
}

void RunLogger::log(double t, int frame, const Command& cmd,
                    double loop_ms, double fps, const StrategyDebugInfo& dbg)
{
    if (!file_.is_open()) return;
    file_ << std::fixed << std::setprecision(6)
          << t << ',' << frame << ','
          << cmd.left << ',' << cmd.right << ',' << (cmd.laser ? 1 : 0) << ','
          << loop_ms << ',' << fps << ','
          << dbg.num_tracks << ',' << dbg.num_obstacles << ','
          << dbg.my_id << ',' << dbg.enemy_id << ','
          << (dbg.offense_mode ? 1 : 0) << ',' << (dbg.boundary_guard ? 1 : 0) << ','
          << dbg.my_x << ',' << dbg.my_y << ',' << dbg.my_theta << ','
          << dbg.enemy_x << ',' << dbg.enemy_y << ','
          << dbg.enemy_dist_px << ',' << dbg.enemy_bearing_deg << ','
          << dbg.nearest_obstacle_dist_px << '\n';
}

void RunLogger::close() { if (file_.is_open()) file_.close(); }
bool RunLogger::isOpen() const { return file_.is_open(); }

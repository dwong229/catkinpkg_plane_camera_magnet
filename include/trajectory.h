#include <string>
#include <vector>
#include <plane_camera_magnet/PositionCommand.h>


typedef std::vector< std::vector< std::vector<double> > > traj_type;

// int loadTraj(const std::string &filename, traj_type &traj);

class Trajectory
{
  private:
    int error_code_;
    std::string filename_;
    ros::Time start_time_;
    traj_type traj_;
    int freq_;
    bool completed, loaded;
    double xoff, yoff;
    int trajidx;
    double dx; //how close you need to be to waypoint before you assign next waypoint.

  public:
    Trajectory();
    bool LoadTrajectory();
    void UpdateGoal(plane_camera_magnet::PositionCommand&);
    int UpdateGoaldx(plane_camera_magnet::PositionCommand&, plane_camera_magnet::PositionCommand&);

    void set_start_time() {start_time_ = ros::Time::now(); completed = false;}
    void set_start_time(ros::Time time) {start_time_ = time; completed = false;}
    void set_filename(const std::string fname) {filename_ = fname;}
    void setOffsets(double, double);

    traj_type get_traj() {return traj_;}
    int  get_error_code() {return error_code_;}
    bool isLoaded() {return loaded;}
    bool isCompleted() {return completed;}
    std::string get_filename() {return filename_;}
};
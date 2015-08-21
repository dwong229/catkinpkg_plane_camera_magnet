#include <ros/ros.h>
#include <plane_camera_magnet/xyFiltered.h>
#include <plane_camera_magnet/PositionCommand.h>
#include <trajectory.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>

using namespace std;

// Adapting trajectory.cpp from justinthomas https://github.com/justinthomas/hummingbird_demo/blob/master/src/trajectory.cpp

Trajectory::Trajectory() : completed(false), loaded(false), xoff(0), yoff(0), trajidx(0), dx(2) {}

void Trajectory::setOffsets(double x, double y) {
  xoff = x; yoff = y;
}

void Trajectory::UpdateGoal(plane_camera_magnet::PositionCommand &goal)
{
  ros::Duration delta_time = ros::Time::now() - start_time_;
  double traj_time = delta_time.toSec();
 
  unsigned long i = traj_time * freq_;

  if (i > traj_.size()-1)
  {
    i = traj_.size()-1;

    if (!completed)
    {
      ROS_INFO("Trajectory completed.");
      completed = true;
    }
  }

  //
  goal.position.x = traj_[i][0][0] + xoff;
  goal.position.y = traj_[i][1][0] + yoff;

  goal.velocity.x = traj_[i][0][1];
  goal.velocity.y = traj_[i][1][1];

  goal.acceleration.x = traj_[i][0][2];
  goal.acceleration.y = traj_[i][1][2];
  

  // gains
  /*
  goal.kx[0] = traj_[i][4][0];
  goal.kx[1] = traj_[i][4][1];
  goal.kx[2] = traj_[i][4][2];
  goal.kv[0] = traj_[i][4][3];
  goal.kv[1] = traj_[i][4][4];
  goal.kv[2] = traj_[i][4][5];
  */
}

//update traj depending on where robot is.
int Trajectory::UpdateGoaldx(plane_camera_magnet::PositionCommand &actual, plane_camera_magnet::PositionCommand &goal)
{
  // RETURN 0 if no update, return 1 if updated.
  int update = 0;
  //ros::Duration delta_time = ros::Time::now() - start_time_;
  //double traj_time = delta_time.toSec();
 
  //unsigned long i = traj_time * freq_;
  
  double distfromgoal = pow(pow(goal.position.x - actual.position.x,2) + pow(goal.position.y - actual.position.y,2),0.5);

  if (distfromgoal < dx)
  { 
    trajidx++;
    update = 1;
  }

  if (trajidx > traj_.size()-1)
  {
    trajidx = traj_.size()-1;

    if (!completed)
    {
      ROS_INFO("Trajectory completed.");
      completed = true;
    }
  }

  //
  goal.position.x = traj_[trajidx][0][0] + xoff;
  goal.position.y = traj_[trajidx][1][0] + yoff;

  goal.velocity.x = traj_[trajidx][0][1];
  goal.velocity.y = traj_[trajidx][1][1];

  goal.acceleration.x = traj_[trajidx][0][2];
  goal.acceleration.y = traj_[trajidx][1][2];
  

  // gains
  /*
  goal.kx[0] = traj_[i][4][0];
  goal.kx[1] = traj_[i][4][1];
  goal.kx[2] = traj_[i][4][2];
  goal.kv[0] = traj_[i][4][3];
  goal.kv[1] = traj_[i][4][4];
  goal.kv[2] = traj_[i][4][5];
  */
  return update;
}

bool Trajectory::LoadTrajectory()
{
  // Much of this is from http://www.cplusplus.com/forum/unices/112048/

  // dims[0] is the number of rows
  // dims[1] is the number of flat outputs
  // dims[2] is the number of derivatives
  // dims[3] is the number of additional columns that include things like gains, servo values, etc.
  // dims[4] is the frequency

  int dims[5];

  // Load the file
  std::ifstream file(filename_.c_str());

  // Define some variables
  std::string line;
  std::string val;

  // Read the first line and make sure it is good
  std::getline(file, line);
  if (!file.good())
  {
    error_code_ = 1;
    return false;
  }

  // Parse the line for commas
  // Note: we expect that the first line contains
  // # of time steps, # of flat outputs, # of derivatives
  // such that the product of the first line is the total number of elements
  std::stringstream iss(line);
  for(int idx=0; idx<5; idx++)
  {
    std::getline(iss, val, ',');
    if (iss.fail())
    {
      error_code_ = 2;
      return false;
    }

    std::stringstream convertor(val);
    convertor >> dims[idx];
  }

  cout << "Freq: " << dims[4] << ", Dimensions: {" << dims[0] << ", " << dims[1] << ", " << dims[2] << "}" << " + " << dims[3] << " additional columns" << endl;
  freq_ = dims[4];
  // Resize the trajectory vector of vectors
  //
  // Note: traj_[t_idx][flat_out][deriv]
  //
  traj_.resize(dims[0]);
  for (int i = 0; i < dims[0]; i++)
  {
    traj_[i].resize(dims[1]+1);
    for (int j = 0; j < dims[1]; j++)
    {
      traj_[i][j].resize(dims[2]);
    }
    traj_[i][dims[1]].resize(dims[3]);
  }

  // This dimension indexes the time
  for(int dim0 = 0; dim0 < dims[0]; dim0++)
  {
    // Get the entire line
    std::getline(file, line);
    if (file.fail())
    {
      cout << "Error reading line " << dim0+1 << endl;
      error_code_ = 3;
      return false;
    }

    // Create a stringstream for the line
    std::stringstream iss(line);

    // This dimension is the derivative
    for (int dim2 = 0; dim2 < dims[2]; dim2++)
    {
      // This dimension is the flat output
      for (int dim1 = 0; dim1 < dims[1]; dim1++)
      {
        std::getline(iss, val, ',');
        if (iss.fail())
        {
          error_code_ = 4;
          return false;
        }

        // The converter
        std::stringstream convertor(val);
        convertor >> traj_[dim0][dim1][dim2];
      }
    }

    // The additional columns
    for (int dim3 = 0; dim3 < dims[3]; dim3++)
    {
      std::getline(iss, val, ',');
      if (iss.fail())
      {
        error_code_ = 5;
        return false;
      }

      // The converter
      std::stringstream convertor(val);
      convertor >> traj_[dim0][dims[1]][dim3];
    }
  }
  cout << "Size of traj: " << traj_.size() << endl;
  cout << "Trajectory loaded" << endl;
  error_code_ = 0;
  loaded = true;
  return true;
}
double Trajectory::getdx()
{
  return dx;
}
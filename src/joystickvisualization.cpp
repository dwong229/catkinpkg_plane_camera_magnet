// Node to listen to controller for position and orientation 
// Plot in rviz

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <Eigen/Geometry>
#include <visualization_msgs/Marker.h> //for rvis visualization
#include <math.h>

typedef Eigen::Vector3d Vec3;

using namespace std;

static ros::Publisher joymarker_pub;



class MagnetJoy {
public:
    void publish_to_rviz() {
        
        joymarker_pub.publish(joyarrow);
    }

    void publish_marker_update(const sensor_msgs::Joy &msg) {
        //translate_with_joy(msg, diff);
        // update current position
        double joy_orientation;
        if(std::abs(msg.axes.at(axis_Lx))>zerocheck)
            magnet_position(0) -= msg.axes.at(axis_Lx);
        if(std::abs(msg.axes.at(axis_Ly))>zerocheck)
            magnet_position(1) += msg.axes.at(axis_Ly);
        joyarrow.pose.position.x = magnet_position(0);
        joyarrow.pose.position.y = magnet_position(1);
        if(std::abs(msg.axes.at(axis_Rx))>zerocheck ||std::abs(msg.axes.at(axis_Ry))>zerocheck){
            joy_orientation = atan2(msg.axes.at(axis_Ry),-msg.axes.at(axis_Rx));
            // determine direction of rotation: + CCW, - CW
            double a,b,c,d,diffang,angstepsize;
            a = cos(magnet_orientation); 
            b = -sin(magnet_orientation); //flip sign for flipped x in joy.
            c = cos(joy_orientation); //-msg.axes.at(axis_Rx)
            d = sin(joy_orientation); //msg.axes.at(axis_Ry)
            diffang = atan2(b*c + a*d,a*c-b*d);
            angstepsize = 0.0175*5;
            if(std::abs(diffang)>angstepsize){
            ROS_INFO_STREAM("joy_orientation: " << joy_orientation);
            ROS_INFO_STREAM("diffang: " << diffang);
            magnet_orientation += copysign(angstepsize,diffang); //1 deg per update)
            }
            else{
                magnet_orientation = joy_orientation;
            }
            //magnet_orientation += msg.axes.at(axis_Rx);

        }
                
        //ROS_INFO_STREAM("Magnet_orientation: " << magnet_orientation);


        // other Marker stuff:       
        joyarrow.header.seq = msg.header.seq;
        joyarrow.header.stamp = msg.header.stamp;
        joyarrow.header.frame_id = "camera_frame";
        joyarrow.type = visualization_msgs::Marker::ARROW;
        joyarrow.pose.orientation.x = 0.0;
        joyarrow.pose.orientation.y = 0.0;
        joyarrow.pose.orientation.z = 1.0 * sin(magnet_orientation/2);
        joyarrow.pose.orientation.w = cos(magnet_orientation/2);
        joyarrow.scale.x = 1;
        joyarrow.scale.y = .4;
        joyarrow.scale.z = .5;
        joyarrow.color.a = 1.0; // Don't forget to set the alpha!
        joyarrow.color.r = 0.0;
        joyarrow.color.g = 1.0;
        joyarrow.color.b = 0.0;
        joymarker_pub_.publish(joyarrow);
    }

    
    void update() {
    if (last_joy_.buttons.size() != 0) {
        publish_marker_update(last_joy_);
        }
    }
    void joyCB(const sensor_msgs::Joy &msg) {
        if (last_joy_.buttons.size() == 0)
            last_joy_ = msg;
        else {
            last_joy_.header = msg.header;
            last_joy_.axes = msg.axes;
        for (int i = 0; i < last_joy_.buttons.size(); i++)
            last_joy_.buttons.at(i) |= msg.buttons.at(i);
        }
    }

    MagnetJoy() {
        joy_sub_ = n_.subscribe("/joy", 20, &MagnetJoy::joyCB, this);
        joymarker_pub_ = n_.advertise<visualization_msgs::Marker>("visualization_joy", 1, this);
    }

private:
  ros::NodeHandle n_{"~"};
  ros::Subscriber joy_sub_;
  ros::Publisher joymarker_pub_;

  visualization_msgs::Marker joyarrow;

  double magnet_orientation{0.0};
  Vec3 magnet_position{Vec3::Zero()};
  double zerocheck = 0.1;
  sensor_msgs::Joy last_joy_;

  // button mappings
  const int axis_Lx{0};
  const int axis_Ly{1};
  const int axis_Rx{3};
  const int axis_Ry{4};
  const int axis_Lt{2};
  const int axis_Rt{5};

  const int button_a{0};
  const int button_b{1};
  const int button_x{2};
  const int button_y{3};
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "magnet_joy");
    ros::NodeHandle n("~");

    MagnetJoy mj;   

    ros::Rate r(10.0);
    while (n.ok()) {
        mj.update();
        r.sleep();
        ros::spinOnce();
    }
    return 0;

}
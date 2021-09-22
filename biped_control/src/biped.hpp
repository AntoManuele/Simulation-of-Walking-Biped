#ifndef BIPED_SENSOR_BIPED_HPP
#define BIPED_SENSOR_BIPED_HPP

#include 	<vector>
#include 	<string>
#include	"ros/ros.h"
#include    <std_msgs/Float64.h>
using namespace std;

/// Topic names
namespace Topics {
    /// left leg ///
    const static string BFZ_SX =  "biped_sensor/bacino_femore_sx_Z_position/command";
    const static string BFX_SX =  "biped_sensor/bacino_femore_sx_X_position/command";
    const static string BFY_SX =  "biped_sensor/bacino_femore_sx_Y_position/command";
    const static string G_SX   =  "biped_sensor/ginocchio_sx_position/command";
    const static string TPY_SX =  "biped_sensor/tibia_piede_sx_Y_position/command";
    const static string TPX_SX =  "biped_sensor/tibia_piede_sx_X_position/command";
    /// right leg ///
    const static string BFZ_DX =  "biped_sensor/bacino_femore_dx_Z_position/command";
    const static string BFX_DX =  "biped_sensor/bacino_femore_dx_X_position/command";
    const static string BFY_DX =  "biped_sensor/bacino_femore_dx_Y_position/command";
    const static string G_DX   =  "biped_sensor/ginocchio_dx_position/command";
    const static string TPY_DX =  "biped_sensor/tibia_piede_dx_Y_position/command";
    const static string TPX_DX =  "biped_sensor/tibia_piede_dx_X_position/command";
};


class Biped_Joint {
private:
    ros::NodeHandle node;
    ros::Publisher publisher;
    string topic;
    unsigned int queue_size;
public:
    Biped_Joint(ros::NodeHandle node, string topic, unsigned int queue_size){
        this->node = node;
        this->topic = topic;
        this->queue_size = queue_size;
        this->publisher = node.advertise<std_msgs::Float64>(topic, queue_size);
    }
    void send_Zero() {
        std_msgs::Float64 msg;
        msg.data = 0.0;
        publisher.publish(msg);
    }
    void send_Value(double value) {
        std_msgs::Float64 msg;
        msg.data = value;
        publisher.publish(msg);
    }
};

/// To solve exportation problems ///
const vector<int> signR = {1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1};
const vector<int> signL = {-1, -1, 1, 1, 1, -1, 1, 1, -1, -1, -1, 1};
const map<string, vector<int>>  sign  {{"R", signR}, {"L", signL}};


#endif //BIPED_SENSOR_BIPED_HPP

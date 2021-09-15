#include 	<iostream>
#include 	<vector>
#include 	<string>
#include 	<map>
#include	"ros/ros.h"
#include 	"std_msgs/String.h"
#include    <std_msgs/Float64.h>
#include 	<sstream>
using 	namespace std;

// 	parameters
#define 	param_num	 4
#define 	JOINTS 		 12


// 	global variables definition
int 				signR[JOINTS]	=  {1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1};
double              zeroQ[JOINTS]   =  {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


//	function prototypes
vector<string> 		generate_sequence (string init_foot, int num_steps, string directory);
//void                zeroposeFunc();


/* ---------------------------------   MAIN   ----------------------------*/

int main (int argc, char *argv[]) {

	int 	steps; 							// number of steps
	string 	foot;							// initial foot
	string 	csv_dir;						// directory, walk shape

	if (argc != param_num) {
		cout << "Inconsistent number of parameters" << endl;
		exit(EXIT_FAILURE);
	}
	
	else {

        foot 	= 	argv[1];
        steps 	= 	atoi(argv[2]);
        csv_dir = 	argv[3];

        // ROS
        ros::init(argc, argv, "walk");
        ros::NodeHandle n;

        ros::Publisher BFZ_SX = n.advertise<std_msgs::Float64>("biped_sensor/bacino_femore_sx_Z_position/command", 1000);
        ros::Publisher BFY_SX = n.advertise<std_msgs::Float64>("biped_sensor/bacino_femore_sx_Y_position/command", 1000);
        ros::Publisher BFX_SX = n.advertise<std_msgs::Float64>("biped_sensor/bacino_femore_sx_X_position/command", 1000);
        ros::Publisher G_SX   = n.advertise<std_msgs::Float64>("biped_sensor/ginocchio_sx_position/command", 1000);
        ros::Publisher TPX_SX = n.advertise<std_msgs::Float64>("biped_sensor/tibia_piede_sx_X_position/command", 1000);
        ros::Publisher TPY_SX = n.advertise<std_msgs::Float64>("biped_sensor/tibia_piede_sx_Y_position/command", 1000);
        ros::Publisher BFZ_DX = n.advertise<std_msgs::Float64>("biped_sensor/bacino_femore_dx_Z_position/command", 1000);
        ros::Publisher BFY_DX = n.advertise<std_msgs::Float64>("biped_sensor/bacino_femore_dx_Y_position/command", 1000);
        ros::Publisher BFX_DX = n.advertise<std_msgs::Float64>("biped_sensor/bacino_femore_dx_X_position/command", 1000);
        ros::Publisher G_DX   = n.advertise<std_msgs::Float64>("biped_sensor/ginocchio_dx_position/command", 1000);
        ros::Publisher TPX_DX = n.advertise<std_msgs::Float64>("biped_sensor/tibia_piede_dx_X_position/command", 1000);
        ros::Publisher TPY_DX = n.advertise<std_msgs::Float64>("biped_sensor/tibia_piede_dx_Y_position/command", 1000);

        vector<string> sequence 	= 	generate_sequence (foot, steps, csv_dir);

        ros::Rate rate(10);
        while (ros::ok())
        {

            std_msgs::Float64 msg;
            msg.data = 1.0;
            BFZ_SX.publish(msg);
            ros::spinOnce();
            rate.sleep();


            //ROS_INFO("%s", msg.data.c_str());
        }


	}
	
	
	
	
	
	
}



/*------------------------   FUNCTIONS    ---------------------------------*/

vector<string> 		generate_sequence (string init_foot, int num_steps, string directory) {
	
	vector<string> 		sequence;
	map<int, string> 	feet 	 {{0, "R"}, {1, "L"}};
	int 				current; 							//current foot
	
	if (!init_foot.compare("R"))
		current = 0;

	// Initial phase
	sequence.push_back("initial" + init_foot + ".csv");
	sequence.push_back("start" + init_foot + ".csv");
	sequence.push_back("change" + feet[!current] + "to" + feet[current] + ".csv");

	// Middle phase
	for (int i = 1; i < num_steps; i++) {
		current = !current;
		sequence.push_back("walk" + feet[current]);
		sequence.push_back("change" + feet[!current] + "to" + feet[current] + ".csv");
	}

	// Final phase
	sequence.push_back("finish" + feet[current]);
	sequence.push_back("reset" + feet[current]);

	// print the generated sequence 
	cout << "----------------------" << endl;
	cout << "Generated sequence: " << endl;
	for (int i = 0; i < sequence.size(); i++) {
		cout << sequence[i] << endl; 
	}
	cout << "----------------------" << endl;

	return sequence;
}

/* ----------------------------- */













#include 	<iostream>
#include 	<vector>
#include 	<string>
#include 	<map>
#include    <fstream>
#include	"ros/ros.h"
#include 	"std_msgs/String.h"
#include    <std_msgs/Float64.h>
#include 	<sstream>
using 	namespace std;

// 	parameters
#define 	param_num	 4
#define 	NUM_JOINTS 		 12
#define     publish_rate 10


class Topics {
public:
    const static inline vector<string> name = { "biped_sensor/bacino_femore_sx_Z_position/command",
                                                "biped_sensor/bacino_femore_sx_Y_position/command",
                                                "biped_sensor/bacino_femore_sx_X_position/command",
                                                "biped_sensor/ginocchio_sx_position/command",
                                                "biped_sensor/tibia_piede_sx_X_position/command",
                                                "biped_sensor/tibia_piede_sx_Y_position/command",
                                                "biped_sensor/bacino_femore_dx_Z_position/command",
                                                "biped_sensor/bacino_femore_dx_Y_position/command",
                                                "biped_sensor/bacino_femore_dx_X_position/command",
                                                "biped_sensor/ginocchio_dx_position/command",
                                                "biped_sensor/tibia_piede_dx_X_position/command",
                                                "biped_sensor/tibia_piede_dx_Y_position/command" };
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



//	function prototypes
vector<string> 		    generate_sequence (string init_foot, int num_steps);
vector<Biped_Joint>     Create_Joints(ros::NodeHandle n, unsigned int queue);
void                    init_Biped(vector<Biped_Joint> all);
void                    move_Biped(vector<Biped_Joint> all, vector<string> seq, string directory);


/* ---------------------------------   MAIN   ----------------------------*/

int main (int argc, char *argv[]) {

    vector<Biped_Joint>     allJoints;
	int 	steps; 							// number of steps
	string 	foot;							// initial foot
	string 	csv_dir;						// directory, walk shape
    int     queue_size = 16;

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
        ros::NodeHandle node;

        allJoints = Create_Joints(node, queue_size);
        vector<string> sequence 	= 	generate_sequence (foot, steps);

        ros::Rate rate(publish_rate);
        //while (ros::ok()) {

        // Send Zero, initial position
        init_Biped(allJoints);
        //move_Biped(allJoints, sequence, csv_dir);

        ros::spinOnce();
        //rate.sleep();


            //ROS_INFO("%s", msg.data.c_str());
      //  }


	}
	
	

	
}



/*------------------------   FUNCTIONS    ---------------------------------*/

vector<string> 		generate_sequence(string init_foot, int num_steps) {
	
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

/* --------------------------------------------------------------------------- */

vector<Biped_Joint>     Create_Joints(ros::NodeHandle n, unsigned int q) {

    vector<Biped_Joint> allJoints;
    Biped_Joint BFZ_SX(n, Topics::name[0], q); allJoints.push_back(BFZ_SX);
    Biped_Joint BFY_SX(n, Topics::name[1], q); allJoints.push_back(BFY_SX);
    Biped_Joint BFX_SX(n, Topics::name[2], q); allJoints.push_back(BFX_SX);
    Biped_Joint G_SX(n,   Topics::name[3], q); allJoints.push_back(G_SX);
    Biped_Joint TPX_SX(n, Topics::name[4], q); allJoints.push_back(TPX_SX);
    Biped_Joint TPY_SX(n, Topics::name[5], q); allJoints.push_back(TPY_SX);
    Biped_Joint BFZ_DX(n, Topics::name[6], q); allJoints.push_back(BFZ_DX);
    Biped_Joint BFY_DX(n, Topics::name[7], q); allJoints.push_back(BFY_DX);
    Biped_Joint BFX_DX(n, Topics::name[8], q); allJoints.push_back(BFX_DX);
    Biped_Joint G_DX(n,   Topics::name[9], q); allJoints.push_back(G_DX);
    Biped_Joint TPX_DX(n, Topics::name[10],q); allJoints.push_back(TPX_DX);
    Biped_Joint TPY_DX(n, Topics::name[11],q); allJoints.push_back(TPY_DX);

    return allJoints;
}

/* --------------------------------------------------------------------------- */

void  init_Biped(vector<Biped_Joint> all) {

    for (int i = 0; i < NUM_JOINTS; i++)
        all[i].send_Zero();
    cout << "ZERO POSE: done" << endl;
}

/* --------------------------------------------------------------------------- */

void  move_Biped(vector<Biped_Joint> all, vector<string> seq, string directory) {

    double q;
    for(int i = 0; i < seq.size(); i++) {
        ifstream csv_file("/" + directory + "/" + seq[i]);
        //csv_file.open("/" + directory + "/" + seq[i]);
        if (!csv_file.is_open()) {
            cerr << "Error: there is no file called: " << "/" << directory << "/" << seq[i] << endl;
            exit(EXIT_FAILURE);
        }
        while (csv_file >> q) {
            cout << q << endl;
        }
        csv_file.close();
    }

}


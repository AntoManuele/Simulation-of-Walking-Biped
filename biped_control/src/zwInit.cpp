#include 	<iostream>
#include 	<vector>
#include 	<string>
#include 	<map>
#include    <fstream>
#include	"ros/ros.h"
#include    <std_msgs/Float64.h>
#include 	<sstream>
#include    <unistd.h>
using 	namespace std;

// 	parameters
#define 	NUM_PARAM	    4
#define 	NUM_JOINTS 		12
#define     publish_rate    10
#define     QUEUE_SIZE      16

const vector<int> signR = {1, -1, 1, -1, -1, 1, -1, 1, -1, 1, 1, 1};
const vector<int> signL = {-1, 1, -1, 1, 1, -1, 1, -1, 1, -1, -1, -1};
const map<string, vector<int>>  sign  {{"R", signR}, {"L", signL}};

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
vector<string> 		    generate_sequence(string init_foot, int num_steps);
vector<Biped_Joint>     Create_Joints(ros::NodeHandle n, unsigned int queue);
void                    init_Biped(vector<Biped_Joint> all);
void                    move_Biped(vector<Biped_Joint> all, vector<string> seq, string directory);


/* ---------------------------------   MAIN   ----------------------------*/

int main (int argc, char *argv[]) {

    vector<Biped_Joint>     allJoints;
	int 	steps; 							// number of steps
	string 	foot;							// initial foot
	string 	csv_dir;						// directory, walk shape

	if (argc != NUM_PARAM) {
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
        ros::Rate rate(publish_rate);

        allJoints = Create_Joints(node, QUEUE_SIZE);
        vector<string> sequence = generate_sequence (foot, steps);

        while (ros::ok()) {

            init_Biped(allJoints);                                      // Send Zero, initial position
            move_Biped(allJoints, sequence, csv_dir);       // Send values read from files

            ros::spinOnce();
            rate.sleep();
            ROS_INFO("FINISH!");
            break;
        }

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
		sequence.push_back("walk" + feet[current] + ".csv");
		sequence.push_back("change" + feet[!current] + "to" + feet[current] + ".csv");
	}
	// Final phase
	sequence.push_back("finish" + feet[current] + ".csv");
	sequence.push_back("reset" + feet[current] + ".csv");
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
    ROS_INFO("ZERO POSE: done");
    ros::Duration(1.5).sleep();
}

/* --------------------------------------------------------------------------- */

void  move_Biped(vector<Biped_Joint> all, vector<string> seq, string directory) {

    double q;
    int k;
    string file_name, row, col, foot;

    for(int i = 0; i < seq.size(); i++) {
        file_name = directory + "/" + seq[i];
        ifstream csv_file(file_name);
        foot = file_name[file_name.size()-5];  //take the current foot by the name of the file
        if (!csv_file.is_open()) {
            cerr << "Error: there is no file called: " << seq[i] << endl;
            cerr << "Please control if you are in correct folder -> .../biped_control/src/" << endl;
            exit(EXIT_FAILURE);
        }
        else cout << "Opening file " << seq[i] << "..." << endl;

        while (getline(csv_file, row)) {
            istringstream iss{row};
            k = 0;
            do {                                          //for each joint
                getline(iss, col, ',');
                q = stod(col);
                all[k].send_Value(q*sign[foot][k]);
                k++;
            } while(k < NUM_JOINTS);

            ros::Duration(0.005).sleep();
        }

        ROS_INFO("%s: done", seq[i].c_str());
        ros::Duration(0.2).sleep();
        csv_file.close();
    }
}


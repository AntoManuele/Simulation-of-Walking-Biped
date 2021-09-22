#include 	<iostream>
#include 	<vector>
#include 	<string>
#include    <map>
#include    <fstream>
#include	"ros/ros.h"
#include    <std_msgs/Float64.h>
#include 	<sstream>
#include    <unistd.h>
#include    "biped.hpp"
using 	namespace std;

/// 	parameters
#define     NUM_PARAM		4
#define     NUM_JOINTS 		12
#define     publish_rate	10
#define     QUEUE_SIZE		16


///	function prototypes
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
        /// ROS
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
        sequence.push_back("juncW" + feet[current] + ".csv");
		sequence.push_back("change" + feet[!current] + "to" + feet[current] + ".csv");
	}
	// Final phase
    current = !current;
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

    vector<Biped_Joint>     allJoints;
    Biped_Joint BFZ_SX(n, Topics::BFZ_SX, q); allJoints.push_back(BFZ_SX);
    Biped_Joint BFX_SX(n, Topics::BFX_SX, q); allJoints.push_back(BFX_SX);
    Biped_Joint BFY_SX(n, Topics::BFY_SX, q); allJoints.push_back(BFY_SX);
    Biped_Joint G_SX(n,   Topics::G_SX, q); allJoints.push_back(G_SX);
    Biped_Joint TPY_SX(n, Topics::TPY_SX, q); allJoints.push_back(TPY_SX);
    Biped_Joint TPX_SX(n, Topics::TPX_SX, q); allJoints.push_back(TPX_SX);
    Biped_Joint BFZ_DX(n, Topics::BFZ_DX, q); allJoints.push_back(BFZ_DX);
    Biped_Joint BFX_DX(n, Topics::BFX_DX, q); allJoints.push_back(BFX_DX);
    Biped_Joint BFY_DX(n, Topics::BFY_DX, q); allJoints.push_back(BFY_DX);
    Biped_Joint G_DX(n,   Topics::G_DX, q); allJoints.push_back(G_DX);
    Biped_Joint TPY_DX(n, Topics::TPY_DX,q); allJoints.push_back(TPY_DX);
    Biped_Joint TPX_DX(n, Topics::TPX_DX,q); allJoints.push_back(TPX_DX);

    return allJoints;
}

/* --------------------------------------------------------------------------- */

void  init_Biped(vector<Biped_Joint> all) {

    for (int j = 0; j < 100; j++)
        for (int i = 0; i < NUM_JOINTS; i++) {
            all[i].send_Zero();
            ros::Duration(0.001).sleep();
        }
    ROS_INFO("ZERO POSE: done");
    ros::Duration(0.5).sleep();
}

/* --------------------------------------------------------------------------- */

void  move_Biped(vector<Biped_Joint> all, vector<string> seq, string directory) {

    double q;
    int k;
    string file_name, row, col, foot;

    for(int i = 0; i < seq.size(); i++) {
        file_name = directory + "/" + seq[i];
        ifstream csv_file(file_name);
        foot = file_name[file_name.size()-5];    //take the current foot by the name of the file
        if (!csv_file.is_open()) {
            cerr << "Error: there is no file called: " << seq[i] << endl;
            cerr << "Please control if you are in correct folder -> .../biped_control/src/" << endl;
            exit(EXIT_FAILURE);
        }
        else cout << "Opening file " << seq[i] << "..." << endl;

        while (getline(csv_file, row)) {
            istringstream iss{row};
            k = 0;
            while(k < NUM_JOINTS) {                     //for each joint
                getline(iss, col, ',');
                q = stod(col);
                auto it = sign.find(foot);
                all[k].send_Value(q * it->second[k]);
                k++;
            }
            ros::Duration(0.004).sleep();
        }
        ROS_INFO("%s: done", seq[i].c_str());
        ros::Duration(0.25).sleep();
        csv_file.close();
    }
}


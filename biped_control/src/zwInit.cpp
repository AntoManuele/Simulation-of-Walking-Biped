#include 	<iostream>
#include 	<vector>
#include 	<string>
#include 	<map>
//#include	"ros/ros.h"
//#include 	"std_msgs/String.h"
#include 	<sstream>
using 	namespace std;

// 	parameters
#define 	param_num	 4
#define 	JOINTS 		 12


// 	global variables definition
int 				signR[JOINTS]	=  {1, 1, -1, -1, -1, 1, -1, -1, 1, 1, 1, -1};


//	function prototypes
vector<string> 		generate_sequence (string init_foot, int num_steps, string directory);




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
	
		//ros::init(argc, argv, "talker");
		//ros::NodeHandle n;
		
		foot 	= 	argv[1];
		steps 	= 	atoi(argv[2]);
		csv_dir = 	argv[3];
	
		vector<string> sequence 	= 	generate_sequence (foot, steps, csv_dir);
	
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



























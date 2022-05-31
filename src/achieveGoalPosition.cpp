/**
* \file achieveGoalPosition.cpp
* \brief mode to set a goal and allow the robot to reach it on its own
* \author Samuele Pedrazzi
* \version 1.0.0
* \date 24/04/2022
*
*
* \details
*
* Subscribes to: <BR>
*  /move_base/status to get the status of the robot
*
* Publishes to: <BR>
*  /move_base/goal to publish te goal position
*  /move_base/cancel to cancel the goal
*
* Description : 
*
* This node's goal is to drive the robot into the correct location in the environment once a position has been specified.
* The node first asks the user for the goal's x and y coordinates, after which it generates and publishes a message 
* of type move_base_msgs/MoveBaseActionGoal in the /move_base/goal topic. 
* The node keeps track of each objective by assigning it an id that is generated at random within the node.
*
**/

#include "ros/ros.h"
#include "stdio.h"
#include "string.h"
#include "move_base_msgs/MoveBaseActionGoal.h"
#include "actionlib_msgs/GoalStatusArray.h"
#include "actionlib_msgs/GoalID.h"
#include "time.h"

#define NC "\033[0m"
#define ITALIC "\033[3m"
#define BOLD "\033[1m"
#define GREEN  "\033[1;32m"
#define RED "\033[1;31m"
#define BLUE "\033[1;34m"

ros::Publisher pub_goal; ///< Initializing the publisher for the goal
ros::Publisher pub_canc_goal; ///< Initializing the publisher for cancelling the goal

ros::Subscriber sub_feedback; ///< Initialize subscriber

// The messages needed to communicate with the move_base node are contained in the package move_base_msgs.
// The move_base package implements an action that, given a goal in the world, try to achieve it.
move_base_msgs::MoveBaseActionGoal msg_goal; ///< Variable for the goal

actionlib_msgs::GoalID msg_canc; ///< Create a variable to read the goal's fields and cancel it.


bool pending_goal = false; ///< Define a boolean if there is a pending goal or not


float x_goal; ///< Set the x position coordinate of the actual goal
float y_goal; ///< Set the y position coordinate of the actual goal

int goal_id; ///< Define a variable to save the goal id


void GetFeedback(const actionlib_msgs::GoalStatusArray::ConstPtr& fdb_msg); ///< Define the prototype of the function

/**
* \brief Function that simply show the actual goal coordinates. 
*
* \return void as this method cannot fail.
*
* This function simply show the actual goal coordinates, if there are any, otherwise it tells that there is none.
**/
void DisplayMenu()
{	
	if(pending_goal)
	{
		std::cout << BLUE << "Actual goal is: [" << x_goal << "], [" << y_goal << "].\n"<< NC;
	}
	else
	{
		std::cout << RED << "There is no goal set.\n" << NC;
	}
}

/**
* \brief Function to get the goal coordinates 
*
* \return void as this method cannot fail.
*
* This function receives the coordinates from the user and set the new goal with an id.
* Then it publish the goal to move_base_msgs::MoveBaseActionGoal and subscribe to the goal status /move_base/status.
**/
void SetTargetCoordinates()
{	
	
	system("clear");
	
	std::cout << BOLD << ITALIC << "Welcome to the interface dedicated to allowing you to reach a certain position.\n" << NC;
	
	std::string string_to_convert;
	
        std::cout << BOLD << ITALIC << "It's time to set the coordinates to reach. \n\n" << NC;
        std::cout << BLUE << BOLD << "Insert x coordinate: \n\n" << NC;
        std::cin >> string_to_convert;
        x_goal = atof(string_to_convert.c_str() );
        
        system("clear");
        std::cout << BLUE << BOLD << "Now insert y coordinate: \n\n" << NC;
        std::cin >> string_to_convert;
        y_goal = atof(string_to_convert.c_str() );
	
	msg_goal.goal.target_pose.header.frame_id = "map";
	msg_goal.goal.target_pose.pose.orientation.w = 1;
		    
	msg_goal.goal.target_pose.pose.position.x = x_goal;
	msg_goal.goal.target_pose.pose.position.y = y_goal;
	
	// Update the flag to true because there is a new goal
	pending_goal = true;
	
	// Assign an id to the new goal set
	goal_id = rand();
	msg_goal.goal_id.id = std::to_string(goal_id);
	
	
	system("clear");
	std::cout << GREEN << "A new goal has just been set.\n" << NC;
	
	// Publish new goal 
	pub_goal.publish(msg_goal);
	
	DisplayMenu();
	
	//subscribe to goal status
        ros::NodeHandle nh;
        sub_feedback = nh.subscribe("/move_base/status", 1, GetFeedback);  
}


/**
* \brief This function cancels the actual goal
* \return True if the goal has been cancelled correctly, False if there is no goal to cancel
*
* This Function that returns false if there isn't a goal to cancel
* otherwise, it cancels the goal by its id publishing a message on /move_base/cancel topic and return true.
**/
bool CancelGoal()
{
	//if there is not any pending goal, do nothing, otherwise delete the goal.
	if(!pending_goal)
		return false;
	msg_canc.id = std::to_string(goal_id); 
	pub_canc_goal.publish(msg_canc);
	pending_goal = false;
	
	sub_feedback.shutdown();
	
	return true;
}

/**
* \brief Function to check the feedback of the pending goal, delete the goal and let the user set another goal.
* \param msg contains the informations about the feedback provided by the robot scanner 
*
* \return void as this method cannot fail.
*
* The function checks the returning status from the ros message GoalStatusArray and
* select the correct action based on the feedback, which is also reported and shown to the user.
* Finally it asks the user if he want to set another goal.
**/
void GetFeedback(const actionlib_msgs::GoalStatusArray::ConstPtr& fdb_msg)
{	
	// Define a variable to save the goal's status
	int result = 0;
	
	//look for goals with correct id
        if (std::to_string(goal_id) == fdb_msg->status_list[0].goal_id.id) 
        	result = fdb_msg->status_list[0].status;
        	
        // switch statement to select the correct status code feedback 
        // checking the messages published into /move_base/status 
    	switch(result)
    	{
    	
    	//status SUCCEDED
    	case 3:
        	std::cout << BOLD << GREEN << "Goal achieved!\n" << NC;
        	CancelGoal();
        	break;
        
        //status ABORTED
    	case 4:
        	std::cout << BOLD << RED << "Goal aborted, it cannot be reached!\n" << NC;
        	CancelGoal();
        	break;
        	
        //status REJECTED
        case 5:
        	std::cout << BOLD << RED << "Goal rejected!\n" << NC;
        	CancelGoal();
        	break;
        	
        //status LOST
    	case 9:
        	std::cout << BOLD << RED << "Goal lost!\n" << NC;
        	CancelGoal();
        	break;

        	
        //Any other status
    	default:
        	break;
    	}
	
	// if there is no pending goal active, it is possible to set another goal.
	if(!pending_goal)
	{
		char c;
		std::cout << BOLD << ITALIC << "Would you want to set another goal? y/n\n\n" << NC;
   		std::cin >> c;
		
		if(c == 'y' || c == 'Y') 
		SetTargetCoordinates();
		
		else if(c == 'n' || c == 'N')
		{
		system("clear");
        	CancelGoal();
        	ros::shutdown();
        	exit(0);
		}
	}
	
}

/**
* \brief main
*
*
* \param  argc The command line arguments are counted in integers.
* \param  argv The command line arguments are represented as a vector.
* \return an integer 0 if it succeeded.
*
* The main function randomize the assignment of the id to a goal, then initialize the node and
* advertise the topics relative to publish the goals and cancel them. After that it calls the function 
* to set the goal's coordinates.
**/
int main(int argc, char **argv)
{
    srand(time(NULL));
    ros::init(argc, argv, "achieveGoalPosition");
    ros::NodeHandle nh;

    // the goal position is published in the "/move_base/goal" topic
    pub_goal = nh.advertise<move_base_msgs::MoveBaseActionGoal>("/move_base/goal", 1);
    
    // the message to cancel the goal is published on the "/move_base/cancel" topic
    pub_canc_goal = nh.advertise<actionlib_msgs::GoalID>("/move_base/cancel", 1);
   

    // Get the coordinates to try reach
    SetTargetCoordinates();
    ros::spin();

    return 0;
}


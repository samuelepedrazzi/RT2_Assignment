/**
* \file UI.cpp
* \brief Final assignment: user interface for a mobile robot using RViz and Gazebo
* \author Samuele Pedrazzi
* \version 1.0
* \date 24/04/2022
*
*
* Services : <BR>
*  /gazebo/reset_simulation used to reset the simulation in gazebo
*
* Description :
*
* This node allows the user to select the robot's movement modes:
* ° establishing the coordinates of a target to be accomplished, and the robot calculating a path to reach it dynamically
* ° controlling it via keyboard
* ° controlling it via keyboard with collision avoidance to avoid obstacles
*
* The robot is navigating through an unknown environment, but owing to laser scanner sensors, it can create a map and recognize barriers.
*
**/

#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include <termios.h>
#include <stdlib.h>

#define NC "\033[0m"
#define ITALIC "\033[3m"
#define BOLD "\033[1m"
#define GREEN  "\033[1;32m"
#define RED "\033[1;31m"

std_srvs::Empty reset;	///< Defining the Empty object reset which is needed to reset the robot position and the first value of the speed

std::string Menu = R"(
___________________________________________________________________

1 - Insert new coordinates to reach

2 - Drive the robot with the keyboard 

3 - Drive the robot with the keyboard with wall collision avoidance
	
4 - Quit the program

5 - Reset the simulation

)";	///< Define menu to select which activity the robot has to do

 
 
/**
* \brief Function that display the options and, depending on the user input, select the one chosen.
*
* \return an integer that determine the user choice, it will be used in the main to select the next node to be runned.
*
* This function print on the screen print a short message and the menu showing all the possible commands and
* the menu to choose the robot's movements modalities and returns the selected one in form of an integer.
**/
int BehaviourChoice()
{
    system("clear");
    std::cout << BOLD << ITALIC << "Welcome to the user interface, you can let the robot move based on these different behaviours:" << NC ;
    std::cout << GREEN << Menu << NC;
    int option ;
    option = getchar();
    
    return option;
    
}


/**
* \brief main
*
*
* \param  argc The command line arguments are counted in integers.
* \param  argv The command line arguments are represented as a vector.
* \return an integer 0 if it succeeded.
*
* The main function uses a while loop to correlate a certain keyboard input with a specific 
* modality, which is then executed with the system() function.
**/
int main(int argc, char **argv)
{
    // Initialize the node 
    ros::init(argc, argv, "UI");
    
    while (true)
    {

        switch (BehaviourChoice())
        {
        case '1':
            //launch achieveGoalPosition node
            system("rosrun RT2_Assignment achieveGoalPosition");
            break;
        case '2':
            //launch userDriveNotAssisted node
            system("rosrun RT2_Assignment userDriveNotAssisted");
            break;
            
        case '3':
            //launch userDriveAssisted node
            system("rosrun RT2_Assignment userDriveAssisted");
            break;
            
        case '4':
            //exit from main function
            system("clear");
            return 0;
            break;
            
        case '5':
            system("clear");        	
            //call gazebo/reset_simulation service and reset the simulation
            ros::service::call("/gazebo/reset_simulation", reset);
            break;
        default:
            break;
        }
    }

    return 0;
}

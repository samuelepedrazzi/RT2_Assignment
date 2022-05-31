/**
* \file userDriveAssisted.cpp
* \brief mode utilizing the keyboard to control the robot using obstacle avoidance
* \author Samuele Pedrazzi
* \version 1.0.0
* \date 24/04/2022
*
*
* \details
*
* Subscribes to: <BR>
*  /scan to detect obstacles
*
* Publishes to: <BR>
*  /cmd_vel to publish the velocity of the robot
*
* Description : 
*
* This node intends to give the user the ability to control the robot in the environment
* through the keyboard, but we also want to provide automatic obstacle avoidance in this situation.
* Because the purpose is similar to what was accomplished with the KeyboardDrive node, 
* some of the code has been reused.
*
**/

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <termios.h>

#define NC "\033[0m"
#define ITALIC "\033[3m"
#define BOLD "\033[1m"
#define GREEN "\033[1;32m"
#define RED "\033[1;31m"


ros::Publisher pub_vel; ///< Initializing the publisher 
geometry_msgs::Twist robot_vel; ///< Initializing the message of type geometry_msgs::Twist


float th_min = 0.8; ///< Define a global variable for the distance threshold
float vel = 0.5; ///< Define a global variable for the initial velocity
float twist_vel = 1; ///< Define a global initial angular velocity

int angle_dir = 0; ///< Define variable for teleop_key linear speed direction
int lin_dir = 0; ///< Define variable for teleop_key angular speed direction

std::string menuTeleop = R"(You can move the robot with the following commands:

   7    8    9
   4    5    6
   1    2    3
   
Press:
*/ /: increase/decrease all speeds by 10%
+/- : increase/decrease only linear speed by 10%
a/z : increase/decrease only angular speed by 10%
R/r : reset all speeds
e   : reset only linear speed
w   : reset only angular speed
CTRL-C to quit
Notice that any other input will stop the robot.

)";	///< Define the menu to show the possible commands

/**
* \brief Function to check the minimum value in a fixed range of an array
* \param angle_range[] the array 
* \param min_value the index from which we start searching
* \param max_value the index at which we stop the search
* 
* \return value the minimum float value in the range
*
* This function through a for loop compare the elements of the selected range,
* it calculates the minimum distance among array values and return the smallest.
**/
float checkDistance(float angle_range[], float min_value, float max_value)
{
    // set the max distance for the laser to not occure of errors
    float value = 100;

    // check every element of the array and
    for (int i = min_value; i < max_value; i++)
    {
        // if the value is less then the distance, update it
        if (angle_range[i] < value)
            value = angle_range[i];
    }
    return value;
}

/**
* \brief Function to check the closest wall and decide the next move to avoide crashing into it.
* \param msg contains the variables that the robot laser provides.
* 
* \return void as this method cannot fail.
*
* This function is used to control the robot movement and manage its speed.
* It checks the distance of the robot to a wall and if it is less than the threshold set before, it updates
* the speed in such a way that the robot cannot collide with circuit delimitations and it is able to continue driving avoiding walls.
* The parameter msg is the message published into base_scan topic.
**/
void CollisionAvoidance(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    // Initializing the array of the laser scans and the variables in which there will be the actual values of distance for the specific range
    float range_view[721];
    float right, left, front;

    // Filling the array for the CheckDistance() function
    for (int i = 0; i <= 721; i++)
        range_view[i] = msg->ranges[i];

    // Assigning the values that represent the front, left and right directions (ranges of view)
    right = checkDistance(range_view, 0, 120);
    left = checkDistance(range_view, 600, 720);
    front = checkDistance(range_view, 290, 410);

    // check if the distance of the robot to a wall is less than the threshold set before and update
    // the speed in such a way that the robot cannot collide with circuit delimitations,
    // it is able to continue driving avoiding walls.
    if (front < th_min)
    {
        if (robot_vel.linear.x > 0)
        {
            // Stop the robot, it can only turn now
            lin_dir = 0;
            std::cout << RED << "Be careful, wall on the front!\n"
                      << NC;
        }
    }
    
    if (right < th_min)
    {   
        if (robot_vel.angular.z < 0)
        {
            // Stop the twist, it can only go straight now
            angle_dir = 0;
            std::cout << RED << "Be careful, wall on the right!\n"
                      << NC;
        }
    }
    
    if (left < th_min)
    {
        if (robot_vel.angular.z > 0)
        {
            // Stop the twist, it can only go straight now
            angle_dir = 0;
            std::cout << RED << "Be careful, wall on the left!\n"
                      << NC;
        }
    }

    // Calculate new values of speed if the user increment/decrement linear/angular velocity
    robot_vel.linear.x = vel * lin_dir;
    robot_vel.angular.z = twist_vel * angle_dir;

    // Publish new value of speed
    pub_vel.publish(robot_vel);
}

// In order to use non-blocking inputs from the keyboard, let's take these function to select non-blocking mode and then restore to blocking mode.
// Functions are taken from the repository of github kbNonblock.c at https://gist.github.com/whyrusleeping/3983293

/**
* \brief Function to restore to blocking keyboard input
*
* \return void as this method cannot fail.
*
* This function is used to restore to blocking mode the old keyboard settings.
**/
void RestoreKeyboardBlocking(struct termios *initial_settings)
{
    // Reapply old settings
    tcsetattr(STDIN_FILENO, TCSANOW, initial_settings);
}


/**
* \brief Function to set non-blocking keyboard input
*
* \return void as this method cannot fail.
*
* This function is used to set to non-blocking mode the keyboard settings.
**/
void SetKeyboardNonBlock(struct termios *initial_settings)
{

    struct termios new_settings;

    // Store old settings, and copy to new settings
    tcgetattr(fileno(stdin), initial_settings);

    // Make required changes and apply the settings
    new_settings = *initial_settings;
    new_settings.c_lflag &= ~ICANON;
    new_settings.c_lflag &= ~ECHO;
    new_settings.c_lflag &= ~ISIG;
    new_settings.c_cc[VMIN] = 1;
    new_settings.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);
}

/**
* \brief Function to have non-blocking keyboard input
*
* \return an integer c given from the user input
*
* This function is used to avoid pressing enter after giving a keyboard input.
**/
int GetChar(void)
{
    struct termios term_settings;
    char c = 0;

    SetKeyboardNonBlock(&term_settings);

    c = getchar();

    // Not restoring the keyboard settings causes the input from the terminal to not work right
    RestoreKeyboardBlocking(&term_settings);

    return c;
}

/**
* \brief Function to associate the input from keyboard to the correct command
* \param an input char to associate with the correct command
*
* \return void as this method cannot fail.
*
* This function is used to control velocity strategically,
* the user's input is associated with a certain action via a switch and the
* speed and turn velocity are updated in the right way.
**/
void UpdateVelocity(char input)
{
    switch (input)
    {

   // change direction
    case '7':
        lin_dir = 1;
        angle_dir = 1;
        break;
    case '8':
        lin_dir = 1;
        angle_dir = 0;
        break;
    case '9':
        lin_dir = 1;
        angle_dir = -1;
        break;
    case '4':
        lin_dir = 0;
        angle_dir = 1;
        break;
    case '5':
        lin_dir = 0;
        angle_dir = 0;
        break;
    case '6':
        lin_dir = 0;
        angle_dir = -1;
        break;
    case '1':
        lin_dir = -1;
        angle_dir = -1;
        break;
    case '2':
        lin_dir = -1;
        angle_dir = 0;
        break;
    case '3':
        lin_dir = -1;
        angle_dir = 1;
        break;
    case '*':
        vel *= 1.1;
        twist_vel *= 1.1;
        break;
    case '/':
        vel *= 0.9;
        twist_vel *= 0.9;
        break;
    case '+':
        vel *= 1.1;
        break;
    case '-':
        vel *= 0.9;
        break;
    case 'a':
        twist_vel *= 1.1;
        break;
    case 'z':
        twist_vel *= 0.9;
        break;
    case 'R':
    case 'r':
        vel = 0.5;
        twist_vel = 1;
        break;
    case 'e':
        vel = 0.5;
        break;
    case 'w':
        twist_vel = 1;
        break;

        
    // CTRL+C
    case '\x03':

        // exit the node after have stopped the robot
        robot_vel.angular.z = 0;
        robot_vel.linear.x = 0;
        pub_vel.publish(robot_vel);
        system("clear");
        ros::shutdown();
        exit(0);
        break;
    default:
        // stop robot
        lin_dir = 0;
        angle_dir = 0;
        break;
    }
    
    return;
}

/**
* \brief Function that acts as UI of the current node, it is used to show the possible commands
*
* \return void as this method cannot fail.
*
* This function aim is to show to the user the possible commands for moving the robot in the environment.
* Then it updates the curent velocity and directon of the robot in response of the input.
**/
void TeleopCommands()
{
    std::cout << BOLD << ITALIC << "Welcome to the user interface, you can let the robot move based on these different behaviours:\n"
              << NC;
    std::cout << GREEN << menuTeleop << NC;

    // Updating and showing the changed value of speed
    printf("\nUpdated speed: @[%.2f,%.2f]\n", vel, twist_vel);
    
    char user_input;
    user_input = GetChar();

    // Call the function to decide what to do depending on which key the user has pressed
    UpdateVelocity(user_input);
    
    system("clear");

}

/**
* \brief main
*
*
* \param  argc The command line arguments are counted in integers.
* \param  argv The command line arguments are represented as a vector.
* \return an integer 0 if it succeeded.
*
* The main function initialize the node, advertise the topic to publish the velocity,
* subscribe to /scan topic and start the program with async spynner to create a multithread process
* in order to have a non-blocking read from the teleop keyboard inputs.
**/
int main(int argc, char **argv)
{
    system("clear");
    // Initialize the node, setup the NodeHandle for handling the communication with the ROS system
    ros::init(argc, argv, "userDriveAssisted");
    ros::NodeHandle nh;

    // Define subscribers to the ros topic (/base_scan)
    ros::Subscriber sub = nh.subscribe("/scan", 500, CollisionAvoidance);

    // Advertise, the node publishes updates to the topic /cmd_vel
    pub_vel = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // Start the program with async in order to read non blocking inputs and at the same time
    // receive info to avoide collision, so it become multithread.
    ros::AsyncSpinner spinner(2);
    spinner.start();

    while (true)
        TeleopCommands();
    spinner.stop();

    return 0;
}

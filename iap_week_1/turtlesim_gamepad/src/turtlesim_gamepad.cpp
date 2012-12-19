#include "ros/ros.h" // main ROS include
#include "sensor_msgs/Joy.h" // joystick message datatype
#include "turtlesim/Velocity.h" // turtlesim command message datatype

// simple class to contain the node's variables and code
class TurtlesimGamepadNode
{
	public:
		TurtlesimGamepadNode();

	private:
		ros::NodeHandle n_; // interface to this node
		ros::Subscriber joy_sub_; // interface to the joy topic subscription
		ros::Publisher cmd_vel_pub_; // interface to the turtlesim command velocity topic publication

		double linear_scale_;
		double angular_scale_;

		void joyCallback(sensor_msgs::Joy::ConstPtr const& reset);
};

// class constructor; subscribe to topics and advertise intent to publish
TurtlesimGamepadNode::TurtlesimGamepadNode() :
 	linear_scale_(2.0), angular_scale_(2.0)
{
	// print out a message for debugging
	ROS_INFO("In TurtlesimGamepadNode constructor.");

	// subscribe to the joy topic
	joy_sub_ = n_.subscribe("joy", 1, &TurtlesimGamepadNode::joyCallback, this);

	// advertise that we'll publish on the turtlesim command velocity topic
	cmd_vel_pub_ = n_.advertise<turtlesim::Velocity>("command_velocity", 1);
}
// the callback function for the joy topic subscription
void TurtlesimGamepadNode::joyCallback(sensor_msgs::Joy::ConstPtr const& joy_msg)
{
	// simple check for invalid joy_msg
	if (joy_msg->axes.size() < 2) {
		ROS_ERROR("joy_msg axes array length (%u) has less than expected length (2)",
		          (unsigned)joy_msg->axes.size());
		return;
	}

	// convert the joystick message to a velocity
	turtlesim::Velocity cmd_vel_msg;
	cmd_vel_msg.linear = joy_msg->axes[1]*linear_scale_;
	cmd_vel_msg.angular = joy_msg->axes[0]*angular_scale_;

	// publish the velocity command message
	cmd_vel_pub_.publish(cmd_vel_msg);
}

int main(int argc, char *argv[])
{
	// initialize the ROS client API, giving the default node name
	ros::init(argc, argv, "turtlesim_gamepad");

	TurtlesimGamepadNode node;

	// enter the ROS main loop
	ros::spin();

	return 0;
}

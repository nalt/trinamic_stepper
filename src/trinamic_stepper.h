#include "tmcl.h"
#include "ros/ros.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/JointState.h>
#include "tf/transform_broadcaster.h"
//#include <vector>
//#include <ctime>
//#include <cmath>


using namespace std;

class trinamic_stepper {
	public:
		trinamic_stepper(ros::NodeHandle &n);
		
		bool doInit();
		void onTimer();
		void onCommand(const sensor_msgs::JointState &command);
		
	private:
		ros::NodeHandle mN;
	    ros::Publisher mPub;
    	ros::Subscriber mSub;
		tf::TransformBroadcaster mTrSender;
    	
		TMCL mStepper;
		string mJointName;
		
		ros::Timer mTimer;
		uint mTimerCount;
		static const double mTimerInterval = 0.02;
};
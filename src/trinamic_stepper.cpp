#include "trinamic_stepper.h"


trinamic_stepper::trinamic_stepper(ros::NodeHandle &n) : mN(n)
{
	mN.param<std::string>("joint_name", mJointName, "stepper");

    mTimer = n.createTimer(ros::Duration(mTimerInterval), boost::bind(&trinamic_stepper::onTimer, this));
	mPub = n.advertise<sensor_msgs::JointState>("state", 1);
	mSub = n.subscribe("command", 1, &trinamic_stepper::onCommand, this);
	
	doInit();
}

bool trinamic_stepper::doInit()
{
	string serial_port;
	int baud_rate, ap_pulse_div=-1, ap_ramp_div=-1, ap_microsteps=6, ap_current_standby=20, ap_current_active=100, ap_freewheelingtime=0;
	
	mN.getParam("serial_port", serial_port);
	mN.getParam("baud_rate", baud_rate);
	mN.getParam("ap_pulse_div", ap_pulse_div);
	mN.getParam("ap_ramp_div", ap_ramp_div);
	mN.getParam("ap_microsteps", ap_microsteps);
	mN.getParam("ap_current_standby", ap_current_standby);
	mN.getParam("ap_current_active", ap_current_active);
	mN.getParam("ap_freewheelingtime", ap_freewheelingtime);
	
	mStepper.saveDivisors(ap_pulse_div, ap_ramp_div);
	mStepper.saveMicrosteps(ap_microsteps);
	mStepper.saveMotorCurrent(ap_current_active, ap_current_standby, -1, ap_freewheelingtime);
	bool ok2 = false;
	bool ok1 = mStepper.portOpen(serial_port.c_str(), baud_rate);
	if (ok1) ok2 = mStepper.init();
	ROS_INFO_STREAM("Opening " << serial_port << " with baud rate " << baud_rate << ": " << ok1 << ". Init: " << ok2);
	if (!ok1 || !ok2)
		ROS_ERROR_STREAM("Failed to open " << serial_port << " with baud rate " << baud_rate);

}

void trinamic_stepper::onTimer()
{
	double pos, vel;
	double position_offset = 0.0;
	mN.getParamCached("position_offset", position_offset);
	string tf_frame_base = mN.getNamespace()+"/base";
	string tf_frame_axis = mN.getNamespace()+"/axis";
	mN.getParamCached("tf_frame_axis", tf_frame_axis);
	mN.getParamCached("tf_frame_base", tf_frame_base);
	
	// Connection problem? Try to reconnect
	if (mStepper.numCommErrors() > 1 && (mTimerCount % (int)(3.0/mTimerInterval)) == 0) {
		bool ret = mStepper.reconnect();
		ROS_ERROR_STREAM("Reconnecting to stepper: " << ((ret)? "ok":"failed"));
	}
	//
	if (mStepper.getMotorPosition(&pos, &vel)) {
			pos *= -1.0; vel *= -1.0;
			pos += position_offset;
			// Publish JointState Message
			sensor_msgs::JointState js;
			js.name.resize(1); js.name[0] = mJointName;
			js.velocity.resize(1); js.velocity[0] = vel;
			js.position.resize(1); js.position[0] = pos;
			mPub.publish(js);
			// Also publish for transform broadcaster
			tf::Transform tr_axis;
			tr_axis.setOrigin(tf::Vector3(0,0,0));
			tr_axis.setRotation(tf::Quaternion(tf::Vector3(0,1,0), pos));
			mTrSender.sendTransform(tf::StampedTransform(tr_axis, ros::Time::now(), tf_frame_axis, tf_frame_base));
	}
	mTimerCount++;
}

void trinamic_stepper::onCommand(const sensor_msgs::JointState &command)
{
	double position_offset = 0.0;
	mN.getParamCached("position_offset", position_offset);
	
	for (int i=0; i<command.name.size(); i++)
		if (command.name[i] == mJointName) {
			double vel = NAN, pos = NAN, effort = NAN;
			if (command.position.size() > i) pos = command.position[i] - position_offset;
			if (command.velocity.size() > i) vel = command.velocity[i];
			if (command.effort.size() > i) effort = command.effort[i];
			pos *= -1.0; vel *= -1.0;
			double acc = 2.0*M_PI;
			mN.getParamCached("acceleration", acc);
			// Commanding modes

			/*if (!isnan(vel)) {
				//if (acc==0.0) acc = M_PI;
				ROS_DEBUG_STREAM("Setting velocity, acceleration: " << vel << ", " << acc);
				if (!mStepper.setVelocity(vel, acc))
					ROS_WARN_STREAM("setVelocity failed");
			}*/
			// Update motor position
			int ret_ump = mStepper.updateMotorPosition();
			if (ret_ump == 1)
				ROS_WARN_STREAM("Stepper position updated");
			else if (ret_ump == -2)
				ROS_DEBUG_STREAM("Problem during stepper position update");
			// Command position
			if (!isnan(pos)) {
				ROS_DEBUG_STREAM("goPosition: " << pos);
				if (!mStepper.goPosition(pos, fabs(vel), acc))
					ROS_ERROR_STREAM("goPosition failed");
			}
			// Command velocity
			if (isnan(pos) && !isnan(vel)) {
				ROS_DEBUG_STREAM("goRotate (with velocity " << vel << ")");
				// TODO: use speed here as parameter; set prescalers seperately
				if (!mStepper.goRotate(vel, acc))
					ROS_ERROR_STREAM("goRotate failed");
			}		
		}
}



int main(int argc, char** argv)
{
	// main_test();

    ros::init(argc, argv, "trinamic_stepper");
    ros::NodeHandle n("~");
    trinamic_stepper ts(n);
    ros::spin();
    
    return 0;
}

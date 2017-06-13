#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <strings.h>
#include <unistd.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <string>
#include <math.h>

using namespace std;

class TMCL {
	public:
		TMCL(void);
		~TMCL(void);
		bool reconnect();
	
		bool portOpen(const char* port, int baud);
		bool sendCmd(const char* cmd, int &ret_status, int &ret_value);
		bool init();
		bool setVelocity(double velocity, double accel = NAN);
		bool goPosition(double pos_rad, double speed = NAN, double accel = NAN);
		bool goPosition(int pos_step);
		bool goRotate(double speed, double accel = NAN);
		int  updateMotorPosition();
		bool getMotorPosition(double *pos, double *vel);
		bool setAxisParameter(int param_id, int value);
		bool getAxisParameter(int param_id, int &value);
		int  numCommErrors() { return m_comm_error; }
		void  saveDivisors(int pulse_div, int ramp_div);
		void  saveMicrosteps(int microsteps);
		void  saveMotorCurrent(int current_active, int current_standby=-1, int powerdowndelay=-1, int freewheelingtime=-1);
		
		static const int AP_TARGET_POS = 0;
		static const int AP_ACTUAL_POS = 1;
		static const int AP_ACTUAL_SPEED = 3;
		static const int AP_MAX_SPEED = 4;
		static const int AP_MAX_ACC = 5;
		static const int AP_MAX_CURRENT = 6;
		static const int AP_STANDBY_CURRENT = 7;
		static const int AP_FREEWHEELING_TIME = 204;
		static const int AP_POWERDOWNDELAY = 214;
		static const int AP_MICROSTEP_RES = 140;
		static const int AP_RAMP_DIVISOR = 153;
		static const int AP_PULSE_DIVISOR = 154;
		static const int AP_ENC_POS = 209;
		static const int AP_ENC_PRESCALE = 210;
		static const int AP_FULLSTEP_THRES = 211;
		static const int AP_MAX_ENC_DEVIATION = 212;
		static const int AP_ENC_RAW = 215;
	
	private:
		int calcVelocityParam(double speed, int pulse_div=-1, bool limits=true);
		
		int m_fd;
		double m_current_acc;
		int m_comm_error;
		string m_port;
		int m_baud;
		bool m_div_auto;
		/** Used to prevent sending MST continuously. Stepper makes strange noises if MST is sent continuously. */
		bool m_stopped;
		
		int mp_current, mp_current_standby, mp_powerdowndelay, mp_freewheelingtime;
		int mp_vel, mp_acc, mp_pulse_div, mp_ramp_div;
		int mp_module_address;
		int mp_mstep_exp, mp_stepsperround, mp_fclk;
		static const bool m_verbose = 0, mp_axis = 0;
};

int main_test();

#include "tmcl.h"

bool TMCL::portOpen(const char* port, int baud)
{
	// Serial port: http://www.easysw.com/~mike/serial/serial.html
	// open the serial port
	m_baud = baud;
	m_port = port;
	m_comm_error = 0;
	m_fd = -1;
	m_fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
	//printf("FD: %d\n", m_fd);
	if (m_fd<0) {
		fprintf(stderr, "Could not open serial device %s\n",port);
		return false;
	}
	fcntl(m_fd,F_SETFL, 0);

	struct termios oldtio, newtio;
	// save the current io settings
	tcgetattr(m_fd, &oldtio);
	// set up new settings
	memset(&newtio, 0, sizeof(newtio));
	newtio.c_cflag = /*ratemask |*/ CS8 | CLOCAL | CREAD; // | HUPCL;
	newtio.c_iflag = 0; // IGNPAR: ignore parity errors
	newtio.c_oflag = 0;	// Turn off output processing
	newtio.c_lflag = 0; //ICANON: Canonical input mode, allows line editing
	cfsetispeed(&newtio, baud);
	cfsetospeed(&newtio, baud);

	tcflush(m_fd, TCIFLUSH);
	tcsetattr(m_fd, TCSANOW, &newtio);
	return true;
}


TMCL::TMCL()
{
	// Parameters
	mp_vel = 200; 
	mp_acc = 200;
	mp_pulse_div = 6;
	mp_ramp_div  = 7;
	mp_module_address = 0;
	mp_mstep_exp = 6;
	mp_stepsperround = 200;
	mp_fclk = 16e6;
	m_current_acc = 2*M_PI;
	m_div_auto = false;
	mp_current = 100;
	mp_current_standby = 20;
	mp_powerdowndelay = 200;
	mp_freewheelingtime = 0;
	m_stopped = false;
	
	m_fd = -1;
}


TMCL::~TMCL()
{
	if (m_fd != -1) close(m_fd);
}

bool TMCL::reconnect()
{
	if (m_fd != -1) close(m_fd);
	if (!portOpen(m_port.c_str(), m_baud)) return false;
	return init();
}


void TMCL::saveDivisors(int pulse_div, int ramp_div)
{
	if (pulse_div == -1 || ramp_div == -1) m_div_auto = true; else m_div_auto = false;
	mp_pulse_div = pulse_div;
	mp_ramp_div = ramp_div;
}

void TMCL::saveMicrosteps(int microsteps)
{
	mp_mstep_exp = microsteps;
}

void TMCL::saveMotorCurrent(int current_active, int current_standby, int powerdowndelay, int freewheelingtime)
{
	if (powerdowndelay!=-1) mp_powerdowndelay = powerdowndelay;
	if (current_active!=-1) mp_current = current_active;
	if (current_standby!=-1) mp_current_standby = current_standby;
	if (current_standby!=-1) mp_freewheelingtime = freewheelingtime;
}

bool TMCL::sendCmd(const char *cmd, int &ret_status, int &ret_value)
{
	// Send Command
	char full_cmd[256], buf[256];
	//printf("Send: %s\n", cmd);
	sprintf(full_cmd, "%c%s\r", mp_module_address+'A', cmd);
	int nw = write(m_fd, full_cmd, strlen(full_cmd));
	ret_status = -1; ret_value = -1;
	// Read Answer
	int len = 0, ncr = 0, i;
	char *reply = &buf[0];
	for (i=0; i<100; i++) {
		usleep(1000);
		int nr = read (m_fd, &buf[len], 255-len);
		// Count \r chars; we expect 2 (echoing back & reply)
		for (int cc=0; cc<nr; cc++)
			if (buf[len+cc] == '\r') { ncr++; if (ncr==1) reply = &buf[len+cc]+1; }
		len += nr;
		if (nr <= 0 && i % 10 == 9) break;
		if (ncr >= 2) break;
	}
	if (ncr < 2)
		{ m_comm_error++; return false; }
	buf[len] = 0;
	// Debug Msgs
	/*
	for (int i=0; i<len; i++) if (buf[i] == 13) buf[i] = 10;
	printf("Full Answer: %s\nAnswer: %s\n", buf, reply); /**/
	// Check reply
	char ret_addr1, ret_addr2;
	int nscan = sscanf(reply, "%c%c%d%d", &ret_addr1, &ret_addr2, &ret_status, &ret_value);
	if (nscan != 4) { m_comm_error++; return false; }
	m_comm_error = 0;
	return true;
}

bool TMCL::init()
{
	char buf[256];
	// set to ascii mode
	char cmd_ascii[] = { 1, 139, 0, 0, 0, 0, 0, 0, 140 };
	write(m_fd, &cmd_ascii, 9);
	usleep(100000);
	read(m_fd, &buf, 255);
	// Send stop command
	int ret_status, ret_value;
	bool ok = false;
	// try to stop motor and get reply several times
	for (int i=0; i<5; i++) {
		if (sendCmd("MST", ret_status, ret_value))
			if (ret_status == 100)
				{ ok = true; break; }
		usleep(100000);
	}
	if (!ok)
		{ if (m_verbose) fprintf(stderr, "Failed to send stop command (MST)\n"); return ok; }
	
	if (m_verbose) printf("Waiting for stepper to stop ");
	do {
		bool ok = getAxisParameter(AP_ACTUAL_SPEED, ret_value);		// current speed
		if (!ok) return false;
		if (ret_value == 0) break;
		if (m_verbose) { printf("."); fflush (stdout); }
		usleep(300000);
	} while (ret_value != 0);
	if (m_verbose) printf("\n");
	
	// Initial settings
	ok = true;
	ok &= setAxisParameter(AP_MICROSTEP_RES, mp_mstep_exp);		// set microstepping; 6 = 64 steps
	ok &= setAxisParameter(AP_ENC_PRESCALE, pow(2, mp_mstep_exp)*mp_stepsperround*2);	// set enc prescaler
		// 8192 = equal to absoluste encoder position; 25600 = 12800 microsteps/round (=200*64)
	ok &= setAxisParameter(AP_RAMP_DIVISOR, mp_ramp_div);		// ramp divisor 0..13
	ok &= setAxisParameter(AP_PULSE_DIVISOR, mp_pulse_div);		// pulse divisor 0..13
	ok &= setAxisParameter(AP_MAX_SPEED, 500);					// max. speed
	ok &= setAxisParameter(AP_MAX_ACC, 500);					// max. accel
	ok &= setAxisParameter(AP_MAX_ENC_DEVIATION, 2*pow(2, mp_mstep_exp));		// max. deviation encoder pos (209) and actual pos (1)
	ok &= setAxisParameter(AP_MAX_CURRENT, mp_current);		// Default: 100
	ok &= setAxisParameter(AP_STANDBY_CURRENT, mp_current_standby);		// Default: 20
	ok &= setAxisParameter(AP_POWERDOWNDELAY, mp_powerdowndelay);
	ok &= setAxisParameter(AP_FREEWHEELING_TIME, mp_freewheelingtime);
	
	// Set position counter to absolute encoder value (so we always have 0 at the same position)
	usleep(100000);
	int abs_enc, abs_step;
	ok &= getAxisParameter(AP_ENC_RAW, abs_enc);
	abs_step = round((double)(pow(2, mp_mstep_exp)* mp_stepsperround) / (double)(4096) * (double)abs_enc);
	if (ok)
		ok &= setAxisParameter(AP_ENC_POS, abs_step);
	updateMotorPosition();
	
	m_comm_error = 0;
	return ok;
}


int TMCL::calcVelocityParam(double speed, int pulse_div, bool limits)
{
	if (pulse_div == -1) pulse_div = mp_pulse_div;
	int vel_param = round((speed / (2*M_PI)) * pow(2, pulse_div) * 2048.0 * 32.0 *
		pow(2, mp_mstep_exp) * (double)mp_stepsperround / (double)mp_fclk);
	if (limits) {
		if (vel_param < -2047) vel_param = -2047;
		if (vel_param > 2047) vel_param = 2047;
	}
	return vel_param;
}



bool TMCL::setVelocity(double velocity, double accel)
{
	// Calc parameters: velocity
	int vel_param;
	int pulse_div = mp_pulse_div; int new_pulse_div = mp_pulse_div;
	if (isnan(velocity)) vel_param = mp_vel;
	else do {
		pulse_div = new_pulse_div;
		vel_param = round((velocity / (2*M_PI)) * pow(2, pulse_div) * 2048.0 * 32.0 *
			pow(2, mp_mstep_exp) * (double)mp_stepsperround / (double)mp_fclk);
		// If vel_param is out of range, try to inc/dec the pulse_div parameter
		if (abs(vel_param) > 2000 && pulse_div > 0 && velocity != 0) new_pulse_div--;
		if (abs(vel_param) < 50 && pulse_div < 13 && velocity != 0) new_pulse_div++;
	} while (new_pulse_div != pulse_div && m_div_auto);
	
	// Calc parameters: acceleration
	int acc_param = 0;
	int ramp_div = mp_ramp_div; int new_ramp_div = mp_ramp_div;
	if (isnan(accel)) acc_param = mp_acc;
	else do {
		ramp_div = new_ramp_div;
		acc_param = abs(round(
			(pow(2.0, mp_mstep_exp) * (double)mp_stepsperround * accel / (2.0*M_PI) ) * 
			pow(2.0, pulse_div+ramp_div+29) / ((double)mp_fclk*(double)mp_fclk) ) );
		// If acc_param is out of range, try to inc/dec the ramp_div parameter
		if (acc_param > 2047 && ramp_div > 0 && abs(ramp_div - pulse_div) < 10) new_ramp_div--;
		if (acc_param < 50 && ramp_div < 13 && abs(ramp_div - pulse_div) < 10) new_ramp_div++;
	} while (new_ramp_div != ramp_div && m_div_auto);
	
	// Check limits
	if (m_verbose) printf("Velocity settings raw: v=%d, a=%d @div=%d/%d\n", vel_param, acc_param, pulse_div, ramp_div);
	// Acc limits as given in TMC428 DATASHEET
	if (acc_param < pow(2, ramp_div-pulse_div-1)) acc_param = pow(2, ramp_div-pulse_div-1);
	if (acc_param > pow(2, ramp_div-pulse_div+12)-1) acc_param = pow(2, ramp_div-pulse_div+12)-1;
	if (vel_param < -2047) vel_param = -2047;
	if (vel_param > 2047) vel_param = 2047;
	if (acc_param > 2047) acc_param = 2047;
	if (acc_param < 1) acc_param = 1;
	if (vel_param == 0) vel_param = 1;
	if (m_verbose) printf("Velocity settings use: v=%d, a=%d @div=%d/%d\n", vel_param, acc_param, pulse_div, ramp_div);
	
	// Send commands
	char cmd[256];
	int ret_status, ret_value;
	if (pulse_div != mp_pulse_div || ramp_div != mp_ramp_div) {
		if (!setAxisParameter(AP_PULSE_DIVISOR, pulse_div))
			return false;
		mp_pulse_div = pulse_div;
		if (setAxisParameter(AP_RAMP_DIVISOR, ramp_div))
			mp_ramp_div = ramp_div;
	}
	bool ok = setAxisParameter(AP_MAX_SPEED, vel_param);
	if (ok) mp_vel = vel_param;
	if (setAxisParameter(AP_MAX_ACC, acc_param))
		mp_acc = acc_param;
	return ok;
}


bool TMCL::getMotorPosition(double *pos, double *vel)
{
	bool ok = true;
	int ret_status;
	if (pos != NULL) {
		int enc_pos;
		sendCmd("GAP 209, 0", ret_status, enc_pos);
		ok &= (ret_status == 100);
		*pos = 2.0 * M_PI * (double)enc_pos / (pow(2, mp_mstep_exp) * (double)mp_stepsperround);
	}
	if (vel != NULL) {
		int cur_speed;
		sendCmd("GAP 3, 0", ret_status, cur_speed);
		ok &= (ret_status == 100);
		*vel = (2*M_PI) * (double)cur_speed * (double)mp_fclk / (pow(2, mp_pulse_div) * 2048.0 * 32.0 *
			pow(2, mp_mstep_exp) * (double)mp_stepsperround);
	}
	return ok;
}

/** Read the actual encoder counter (reg. 209) and write it to actual position (reg. 1) if motor is not moving
 *  \return		-2: problem, -1: motor moving, 0: not neccessary, 1: updated position*/ 
int TMCL::updateMotorPosition()
{
	int ret_status, ret_value, cur_speed, cur_pos_enc, cur_pos_cnt;
	if (!getAxisParameter(AP_ACTUAL_SPEED, cur_speed)) return -2;
	if (cur_speed != 0) return -1;
	if (!getAxisParameter(AP_ENC_POS, cur_pos_enc)) return -2;
	if (!getAxisParameter(AP_ACTUAL_POS, cur_pos_cnt)) return -2;
	if (abs(cur_pos_enc - cur_pos_cnt) < 100) return 0;
	
	sendCmd("MST 0", ret_status, ret_value);
	usleep(200000);
	if (!getAxisParameter(AP_ENC_POS, cur_pos_enc)) return -2;
	return (setAxisParameter(AP_ACTUAL_POS, cur_pos_enc))? 1 : -2;
}

bool TMCL::setAxisParameter(int param_id, int value)
{
	char cmd[256];
	int ret_status, ret_value;
	sprintf(cmd, "SAP %u, %u, %d", param_id, mp_axis, value);
	sendCmd(cmd, ret_status, ret_value);
	return ret_status == 100 && ret_value == value;
}


bool TMCL::getAxisParameter(int param_id, int &value)
{
	char cmd[256];
	int ret_status, ret_value;
	sprintf(cmd, "GAP %u, %u", param_id, mp_axis);
	sendCmd(cmd, ret_status, ret_value);
	value = ret_value;
	return ret_status == 100;
}


bool TMCL::goPosition(double pos_rad, double speed, double accel)
{
	setVelocity(speed, accel);
	int pos_step = round(pow(2, mp_mstep_exp) * (double)mp_stepsperround * pos_rad / (2.0 * M_PI));
	return goPosition(pos_step);
}


bool TMCL::goPosition(int pos_step)
{
	char cmd[256];
	int ret_status, ret_value;
	sprintf(cmd, "MVP ABS, 0, %d", pos_step);
	sendCmd(cmd, ret_status, ret_value);
	m_stopped = false;
	return (ret_status == 100 && ret_value == pos_step);
}

/*bool goStop()
{
	int ret_status, ret_value;
	sendCmd("MST 0", ret_status, ret_value);
	return (ret_status == 100);
}*/


bool TMCL::goRotate(double speed, double accel)
{
	char cmd[256];
	int ret_status, ret_value;
	setVelocity(speed, accel);

	int vel = calcVelocityParam(speed);
	if (vel != 0) {
		sprintf(cmd, "ROR 0, %d", vel);
		sendCmd(cmd, ret_status, ret_value);
		m_stopped = false;
		return (ret_status == 100 && ret_value == vel);
	} else if (!m_stopped) {
		sendCmd("MST 0", ret_status, ret_value);
		m_stopped = true;
		return (ret_status == 100);
	}
	else return true;
		
}


int main_test()
{
	int ret_ok, ret_status, ret_value;
	char cmd[256];
	
	TMCL stepper;
	stepper.setVelocity(0.1 * 2*M_PI);
	ret_ok = stepper.portOpen("/dev/ttyACM0", 9600);
	printf("OPEN:%u\n", ret_ok);
	if (!stepper.init())
		{ printf("Init Failed\n"); exit(1); }
	//
	strcpy(cmd, "MVP ABS, 0, 50000");
	ret_ok = stepper.sendCmd(cmd, ret_status, ret_value);
	printf("CMD: %s, Return(%u): %d, %d\n", cmd, ret_ok, ret_status, ret_value);
	int i = 0;
	while (1) {
		int abs_enc, enc_pos, enc_pre, pos, load;
		ret_ok = stepper.sendCmd("GAP 1, 0", ret_status, pos);
		ret_ok = stepper.sendCmd("GAP 215, 0", ret_status, abs_enc);
		ret_ok = stepper.sendCmd("GAP 209, 0", ret_status, enc_pos);
		ret_ok = stepper.sendCmd("GAP 210, 0", ret_status, enc_pre);
		ret_ok = stepper.sendCmd("GAP 206, 0", ret_status, load);	
		printf("ok:%u, POS:%6d, ENC:%6d, ABS:%5d, PRE:%5d, LOAD:%1d   \r", ret_ok, pos, enc_pos, abs_enc, enc_pre, load);
		fflush (stdout);
		usleep(100000);
		i++;
		if (i % 50 == 0) { strcpy(cmd, "ROR 0, 1000");
		ret_ok = stepper.sendCmd(cmd, ret_status, ret_value); }
	}
	//
	usleep(1000000);
	strcpy(cmd, "ROR 0, 1000");
	ret_ok = stepper.sendCmd(cmd, ret_status, ret_value);
	printf("CMD: %s, Return(%u): %d, %d\n", cmd, ret_ok, ret_status, ret_value);
	//
	usleep(1000000);
	strcpy(cmd, "MST");
	ret_ok = stepper.sendCmd(cmd, ret_status, ret_value);
	printf("CMD: %s, Return(%u): %d, %d\n", cmd, ret_ok, ret_status, ret_value);		
}
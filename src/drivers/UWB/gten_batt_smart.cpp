/****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file batt_smart.cpp
 *
 *
 * @author phil@tenx_eprom@sina.com
 */
#ifdef __PX4_NUTTX
#include <nuttx/clock.h>
#include <nuttx/arch.h>
#endif

#ifndef __PX4_QURT
#include <termios.h>
#include <poll.h>
#else
#include <sys/ioctl.h>
#include <dev_fs_lib_serial.h>
#endif

#include <px4_config.h>
#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sched.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <errno.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>
#include <ctype.h>
#include <nuttx/arch.h>
#include <nuttx/wqueue.h>
#include <nuttx/clock.h>
#include <board_config.h>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <systemlib/err.h>
#include <px4_workqueue.h>
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
#include <uORB/topics/smart_battery_status.h>
#include <uORB/topics/battery_status.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>
#include <float.h>
#include <drivers/drv_hrt.h>
#include <drivers/device/ringbuffer.h>
#include <ecl/geo/geo.h>
#include <px4_tasks.h>
#include <time.h>
#include <sys/ioctl.h>
#include <drivers/device/device.h>
#include <arch/board/board.h>
#include <parameters/param.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <dataman/dataman.h>
#include <matrix/math.hpp>


#define BATT_DEFAULT_UART_PORT   	"/dev/ttyS3"
#define BATT_DEFAULT_UART_BAUD   	115200

#define TIMEOUT_5HZ			20
#define BATT_WAIT_BEFORE_READ		2			//ms
#define CONNECT_TIMEOUT_MS		15000


#define BATT_SMBUS_PEC_POLYNOMIAL	0x07	///< Polynomial for calculating PEC


#define PEC_BUFFER_SIZE			200


enum batbus_status
{
	wait_head = 0,
	wait_command,
	wait_adrsn,
	wait_paramnum,
	wait_sot,
	wait_param,
	wait_pec,
	wait_eot,
};


#define BATT_BUS_HEAD			0xAA
#define BATT_BUS_SOT			0xBB
#define BATT_BUS_EOT			0xCC

#define BATTERY_DATA_CMD		1

class BATT_SMART
{
public:
	BATT_SMART(const char *uart_path);
	virtual ~BATT_SMART();

	int 		init();
	ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	int 		test();
	void		shut();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void		print_info();

protected:
	int		probe();

private:
	// internal variables
	bool			_task_should_exit;				///< flag to make the main worker task exit
	bool			_enabled;					///< true if we have successfully connected to battery
	volatile int		_task;						///< worker task
	uint64_t		_start_time;					///< system time we first attempt to communicate with battery

	int			_serial_fd;					///< serial interface to GPS
	char			_port[20];					///< device / serial port path

	int8_t 			_remaining;
	int8_t 			_remaining_time;
	int8_t 			_remaining_flight_time;
	int8_t 			_health;
	uint8_t 		_cycle_count;
	uint16_t 		_voltage_count1;
	uint16_t		_voltage_count2;
	uint16_t 		_voltage_count3;
	uint16_t 		_voltage_count4;
	int16_t 		_current;
	uint16_t 		_pec_bytes;
	uint8_t 		_pec_buffer[PEC_BUFFER_SIZE];
	uint16_t 		_param_bytes;
	uint8_t* 		_param_buffer;

	uint8_t 		_buf[PEC_BUFFER_SIZE+7+5];

	batbus_status   	_decode_state;
	uint8_t			_command;
	uint8_t			_id;
	uint8_t			_paramnum;


	uint16_t 		_frame_cnt;
	uint16_t 		_error_frame_cnt;
	int 			_remaining_cap;
	int 			_full_cap;
	int 			_volt;
	int			_full_volt;
	bool   			_unbalance;
	uint8_t 		_warning;

	int 			capacity_critical;

	bool   			_shut;    //for test connect lose

	struct smart_battery_status_s	_smart_battery_status;
	orb_advert_t			_smart_battery_status_pub;
	struct battery_status_s	 	_battery_status;
	orb_advert_t			_battery_status_pub;

	int				_global_pos_sub;
	int				_home_pos_sub;

	struct vehicle_global_position_s			_global_pos;
	struct home_position_s					_home_pos;

	param_t _param_curr_horizontal;
	param_t _param_curr_vertical;
	param_t _param_cap_safety;
	param_t _param_cap_safety_unbalance;
	param_t _param_mpc_xy_cruise;
	param_t _param_z_vel_max_dn;
	float 				_curr_horizontal;
	float 				_curr_vertical;
	float 				_cap_safety;
	float 				_cap_safety_unbalance;
	float 				_mpc_xy_cruise;
	float 				_z_vel_max_dn;

	orb_advert_t mavlink_log_pub;
	hrt_abstime last_time;
	uint8_t step;
	bool _home_updata;

	/**
	 * Initialise the automatic measurement state machine and start it.
	 *
	 * @note This function is called at open and error time.  It might make sense
	 *       to make it more aggressive about resetting the bus in case of errors.
	 */
	void				start();

	/**
	 * Stop the automatic measurement state machine.
	 */
	void				stop();

	/**
	 * Trampoline to the worker task
	 */
	static void task_main_trampoline(int argc, char *argv[]);

	/**
	 * Worker task: main GPS thread that configures the GPS and parses incoming data, always running
	 */
	void task_main(void);

	/**
	 * set the Baudrate
	 * @param baud
	 * @return 0 on success, <0 on error
	 */
	int setBaudrate(unsigned baud);

	/**
	 * This is an abstraction for the poll on serial used.
	 *
	 * @param buf: pointer to read buffer
	 * @param buf_length: size of read buffer
	 * @param timeout: timeout in ms
	 * @return: 0 for nothing read, or poll timed out
	 *	    < 0 for error
	 *	    > 0 number of bytes read
	 */
	int pollOrRead(uint8_t *buf, size_t buf_length, int timeout);

	int receive(unsigned timeout);
	int handleMessage(int len);
	void printStatus();
	int parseChar(uint8_t b);
	void publish();
	void poll_subscriptions();

	/**
	 * Calculate PEC for a read or write from the battery
	 * @param buff is the data that was read or will be written
	 */
	uint8_t			cal_PEC(const uint8_t buff[], uint8_t len) const;
	uint8_t 		cal_PEC2(uint8_t *data, uint8_t len);

	void			calculate_battery_warning(uint8_t* warning);

};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int gten_batt_smart_main(int argc, char *argv[]);

namespace
{
BATT_SMART	*g_dev = nullptr;
}


BATT_SMART::BATT_SMART(const char *uart_path):
	_task_should_exit(false),
	_enabled(false),
	_start_time(0),
	_serial_fd(-1),
	_remaining(-1),
	_remaining_time(-1),
	_remaining_flight_time(-1),
	_health(-1),
	_cycle_count(-1),
	_voltage_count1(0xFF),
	_voltage_count2(0xFF),
	_voltage_count3(0xFF),
	_voltage_count4(0xFF),
	_current(-1),
	_frame_cnt(0),
	_error_frame_cnt(0),
	_remaining_cap(0),
	_full_cap(0),
	_volt(0),
	_full_volt(0),
	_unbalance(false),
	_shut(false),
	_smart_battery_status_pub{nullptr},
	_battery_status_pub{nullptr},
	_global_pos_sub(-1),
	_home_pos_sub(-1),
	_global_pos{},
	_home_pos{},
	_curr_horizontal(11.0f),
	_curr_vertical(9.0f),
	_cap_safety(1000.0f),
	_cap_safety_unbalance(1800.0f),
	_mpc_xy_cruise(8.0f),
	_z_vel_max_dn(2.2f),
	mavlink_log_pub(0),
	last_time(0),
	step(0),
	_home_updata(false)
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	memset(&_smart_battery_status, 0, sizeof(_smart_battery_status));
	memset(&_battery_status, 0, sizeof(_battery_status));

	_param_curr_horizontal = param_find("BAT_H_CURR");
	param_get(_param_curr_horizontal, &_curr_horizontal);

	_param_curr_vertical = param_find("BAT_V_CURR");
	param_get(_param_curr_vertical, &_curr_vertical);

	_param_cap_safety = param_find("BAT_C_SAFE");
	param_get(_param_cap_safety, &_cap_safety);

	_param_cap_safety_unbalance = param_find("BAT_C_SAFE_UB");
	param_get(_param_cap_safety_unbalance, &_cap_safety_unbalance);

	_param_mpc_xy_cruise = param_find_no_notification("MPC_XY_CRUISE");
	param_get(_param_mpc_xy_cruise, &_mpc_xy_cruise);

	_param_z_vel_max_dn = param_find_no_notification("MPC_Z_VEL_MAX_DN");
	param_get(_param_z_vel_max_dn, &_z_vel_max_dn);

	// capture startup time
	_start_time = hrt_absolute_time();
}


BATT_SMART::~BATT_SMART()
{
	/* tell the task we want it to go away */
	_task_should_exit = true;

	/* spin waiting for the task to stop */
	for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
		/* give it another 100ms */
		usleep(100000);
	}

	/* well, kill it anyway, though this will probably crash */
	if (_task != -1) {
		px4_task_delete(_task);
	}

}


int
BATT_SMART::init()
{
	/* start the GPS driver worker task */
	_task = px4_task_spawn_cmd("batt_smart", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, 1600, (px4_main_t)&BATT_SMART::task_main_trampoline, NULL);

	if (_task < 0) {
		PX4_WARN("task start failed: %d", errno);
		_task = -1;
		return -errno;
	}

	return OK;
}

void BATT_SMART::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
}

void
BATT_SMART::task_main()
{
	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (_serial_fd < 0) {
		PX4_ERR("BATT_SMART: failed to open serial port: %s err: %d", _port, errno);

		/* tell the dtor that we are exiting, set error code */
		_task = -1;
		px4_task_exit(1);
	}
	else
	{
		printf("open port:%s success\n", _port);
	}

#ifndef __PX4_QURT
	// TODO: this call is not supported on Snapdragon just yet.
	// However it seems to be nonblocking anyway and working.
	int flags = fcntl(_serial_fd, F_GETFL, 0);
	fcntl(_serial_fd, F_SETFL, flags | O_NONBLOCK);
#endif

	setBaudrate(BATT_DEFAULT_UART_BAUD);

	int ret;
	uint64_t time_last_recv = hrt_absolute_time();

	//uint64_t uwb_t = 0;
	//uint64_t time_last_recv = 0;
	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		//updateParam();
		//poll_subscriptions();

		ret = receive(TIMEOUT_5HZ);

		if (ret > 0) {
			time_last_recv = hrt_absolute_time();
			//printf("publish\n");
		}
		else
		{
			if(time_last_recv - hrt_absolute_time() > 2000000)
			{
				printf("lost\n");
			}
		}

		receive(TIMEOUT_5HZ);

		publish();

		usleep(100000);


	}

	PX4_WARN("exiting");

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	px4_task_exit(0);
}

void BATT_SMART::publish()
{
	// if (_smart_battery_status_pub != nullptr) {
	// 	//printf("smbatt pub\n");
	// 	orb_publish(ORB_ID(smart_battery_status), _smart_battery_status_pub, &_smart_battery_status);
	// } else {
	// 	_smart_battery_status_pub = orb_advertise(ORB_ID(smart_battery_status), &_smart_battery_status);
	// }

}


void BATT_SMART::poll_subscriptions()
{
	// bool updated = false;

	// orb_check(_home_pos_sub, &updated);
	// if (updated) {
	// 	_home_updata = true;
	// 	orb_copy(ORB_ID(home_position), _home_pos_sub, &_home_pos);
	// }

	// orb_check(_global_pos_sub, &updated);
	// if (updated) {
	// 	orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
	// }
}


int BATT_SMART::setBaudrate(unsigned baud)
{

#if __PX4_QURT
	// TODO: currently QURT does not support configuration with termios.
	dspal_serial_ioctl_data_rate data_rate;

	switch (baud) {
	case 9600: data_rate.bit_rate = DSPAL_SIO_BITRATE_9600; break;

	case 19200: data_rate.bit_rate = DSPAL_SIO_BITRATE_19200; break;

	case 38400: data_rate.bit_rate = DSPAL_SIO_BITRATE_38400; break;

	case 57600: data_rate.bit_rate = DSPAL_SIO_BITRATE_57600; break;

	case 115200: data_rate.bit_rate = DSPAL_SIO_BITRATE_115200; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	int ret = ::ioctl(_serial_fd, SERIAL_IOCTL_SET_DATA_RATE, (void *)&data_rate);

	if (ret != 0) {

		return ret;
	}

#else
	/* process baud rate */
	int speed;
	printf("set baud:%d\n", baud);
	switch (baud) {
	case 9600:   speed = B9600;   break;

	case 19200:  speed = B19200;  break;

	case 38400:  speed = B38400;  break;

	case 57600:  speed = B57600;  break;

	case 115200: speed = B115200; break;

	default:
		PX4_ERR("ERR: unknown baudrate: %d", baud);
		return -EINVAL;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_serial_fd, &uart_config);

	/* properly configure the terminal (see also https://en.wikibooks.org/wiki/Serial_Programming/termios ) */

	//
	// Input flags - Turn off input processing
	//
	// convert break to null byte, no CR to NL translation,
	// no NL to CR translation, don't mark parity errors or breaks
	// no input parity check, don't strip high bit off,
	// no XON/XOFF software flow control
	//
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | ICRNL |
				 INLCR | PARMRK | INPCK | ISTRIP | IXON);
	//
	// Output flags - Turn off output processing
	//
	// no CR to NL translation, no NL to CR-NL translation,
	// no NL to CR translation, no column 0 CR suppression,
	// no Ctrl-D suppression, no fill characters, no case mapping,
	// no local output processing
	//
	// config.c_oflag &= ~(OCRNL | ONLCR | ONLRET |
	//                     ONOCR | ONOEOT| OFILL | OLCUC | OPOST);
	uart_config.c_oflag = 0;

	//
	// No line processing
	//
	// echo off, echo newline off, canonical mode off,
	// extended input processing off, signal chars off
	//
	uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);

	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetispeed)", termios_state);
		return -1;
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("ERR: %d (cfsetospeed)", termios_state);
		return -1;
	}

	if ((termios_state = tcsetattr(_serial_fd, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("ERR: %d (tcsetattr)", termios_state);
		return -1;
	}

#endif
	return 0;
}


int BATT_SMART::receive(unsigned timeout)
{
	int i = 0;
	int num_read = 0;
	
	//PX4_WARN("11");
	/* then poll or read for new data */
	num_read = pollOrRead(_buf, sizeof(_buf), timeout);
	//PX4_WARN("2");
	if (num_read < 0) {
		//PX4_WARN("22");
		/* something went wrong when polling */
		return -1;
	}

	/* pass received bytes to the packet decoder */
	while (i < num_read) {

		//printf("num read:%d i:%d\n", num_read, i);
		int len = 0;

		len = parseChar(_buf[i]);

		if (len > 0) {
			return 1;
		}

		i++;
	}

	/* everything is read */
	i = num_read = 0;

	return 1;
}

int BATT_SMART::parseChar(uint8_t b)
{
	int iRet = 0;
// 	printf("decode in:%x  ", b);
// 	switch (_decode_state) {
// 		case wait_head:
// 			if (b == BATT_BUS_HEAD) {
// 				_decode_state = wait_command;
// 				_param_bytes = 0;
// 				_pec_bytes = 0;
// 			}
// 			break;
// 		case wait_command:
// 			_pec_buffer[_pec_bytes++] = b;
// 			_command = b;
// 			//printf("get cmd %d\n", _command);
// 			_decode_state = wait_adrsn;
// 			break;
// 		case wait_adrsn:
// 			_pec_buffer[_pec_bytes++] = b;
// 			_id = b;
// 			//printf("get id %d\n", _id);
// 			_decode_state = wait_paramnum;
// 			break;
// 		case wait_paramnum:
// 			_pec_buffer[_pec_bytes++] = b;
// 			_paramnum = b;
// 			//printf("get param num %d\n", _paramnum);
// 			_decode_state = wait_sot;
// 			break;
// 		case wait_sot:
// 			if (b == BATT_BUS_SOT) {
// 				_pec_buffer[_pec_bytes++] = b;
// 				if(_paramnum > 0)
// 				{
// 					//printf("get sot\n param:");
// 					_decode_state = wait_param;
// 				}
// 				else
// 				{
// 					_decode_state = wait_pec;
// 					printf("get sot,but param=0\n");
// 				}
// 			}
// 			else
// 			{
// 				printf("get sot error\n");
// 				_decode_state = wait_head;
// 			}
// 			break;
// 		case wait_param:
// 			_pec_buffer[_pec_bytes++] = b;
// 			//_param_buffer[_param_bytes++] = b;
// 			_param_bytes++;
// 			//printf("%x ", b);
// 			if(_param_bytes >= _paramnum)
// 			{
// 				//printf("\n");
// 				_decode_state = wait_pec;
// 			}
// 			break;
// 		case wait_pec:
// 		{
// 			uint8_t pec = cal_PEC(_pec_buffer, _pec_bytes);
// 			//uint8_t pec = cal_PEC2(_pec_buffer, _pec_bytes);
// //			printf("\n****************\n");
// //			printf("pec: num:%d cal:%x get:%x \n", _pec_bytes, pec, b);
// //			for(int i=0; i<_pec_bytes; i++)
// //			{
// //				printf("%02x ", _pec_buffer[i]);
// //			}
// //			printf("\n****************\n\n");
// 			if(pec == b)
// 			{
// 				//printf("get pec right\n");
// 				_decode_state = wait_eot;
// 			}
// 			else
// 			{
// 				printf("pec:%x cal:%x\n", b, pec);
// 				_decode_state = wait_head;
// 			    _error_frame_cnt++;
// 			}
// 		    _frame_cnt++;
// 			//printf("total:%d  err:%d\n", _frame_cnt, _error_frame_cnt);
// 			break;
// 		}
// 		case wait_eot:
// 			if (b == BATT_BUS_EOT) {
// 				//printf("get eot\n");
// 				_decode_state = wait_head;
// 				iRet = _paramnum;
// 				_param_buffer = &_pec_buffer[4];
// 			}
// 			else
// 			{
// 				_decode_state = wait_head;
// 			}
// 			break;
// 		default:
// 			_decode_state = wait_head;
// 			break;
// 	}
//printf(" out:%d\n", _decode_state);
	return iRet;
}

int BATT_SMART::handleMessage(int len)
{

	return 1;
}


void BATT_SMART::calculate_battery_warning(uint8_t* warning)
{
}


void BATT_SMART::print_info()
{
	printf("total:%d  err:%d\n", _frame_cnt, _error_frame_cnt);
	printf("remaining:%d, ramaining time:%dmin, health:%d, cycle:%d, c1:%d.%dv, c2:%d.%dv, c3:%d.%dv, c4:%d.%dv, curr:%d.%da\n",
			_remaining,
			_remaining_time,
			_health,
			_cycle_count,
			_voltage_count1/10, _voltage_count1%10,
			_voltage_count2/10, _voltage_count2%10,
			_voltage_count3/10, _voltage_count3%10,
			_voltage_count4/10, _voltage_count4%10,
			_current/10, _current%10
			);
	printf("unbalance:%d\n", _unbalance);
	printf("ramaining flight time:%dmin\n", _remaining_flight_time);
	printf("warning:%d\n", _warning);

	printf("_remaining_cap:%dmah, _full_cap:%dmah ",_remaining_cap, _full_cap);
	if(_remaining_cap > _full_cap)
	{
		printf("cap error\n");
	}
	else
	{
		printf("\n");
	}

	printf("volt:%d.%d, full_volt:%d.%d ",_volt/1000, _volt%1000, _full_volt/1000, _full_volt%1000);
	if(_volt > _full_volt)
	{
		printf("volt error\n");
	}
	else
	{
		printf("\n");
	}

	printf("capacity_critical:%d _curr_horizontal:%f _mpc_xy_cruise:%f _curr_vertical:%f _z_vel_max_dn:%f\n",
			capacity_critical, (double)_curr_horizontal, (double)_mpc_xy_cruise, (double)_curr_vertical, (double)_z_vel_max_dn);

}


int BATT_SMART::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
#ifndef __PX4_QURT

	/* For non QURT, use the usual polling. */

	//Poll only for the serial data. In the same thread we also need to handle orb messages,
	//so ideally we would poll on both, the serial fd and orb subscription. Unfortunately the
	//two pollings use different underlying mechanisms (at least under posix), which makes this
	//impossible. Instead we limit the maximum polling interval and regularly check for new orb
	//messages.
	//FIXME: add a unified poll() API
	const int max_timeout = 100;

	pollfd fds[1];
	fds[0].fd = _serial_fd;
	fds[0].events = POLLIN;

	int ret = poll(fds, sizeof(fds) / sizeof(fds[0]), math::min(max_timeout, timeout));

	// /*test code*/
	 //printf("poll ret:%d\n", ret);
	
	if (ret > 0) {
		/* if we have new data from GPS, go handle it */
		if (fds[0].revents & POLLIN) {
			/*
			 * We are here because poll says there is some data, so this
			 * won't block even on a blocking device. But don't read immediately
			 * by 1-2 bytes, wait for some more data to save expensive read() calls.
			 * If more bytes are available, we'll go back to poll() again.
			 */
			usleep(BATT_WAIT_BEFORE_READ * 1000);
			ret = ::read(_serial_fd, buf, buf_length);

			// /*test code*/
			 //printf("read ret:%d\n", ret);
			if(ret > 0)
			{
				printf("\n****************\n");
				//printf("read: num:%d  \n", ret);
				for(int i=0; i<ret; i++)
				{
					printf("%02x ", buf[i]);
				}
				printf("\n****************\n\n");
			}
			/*end test*/

		} else {
			ret = -1;
		}
	}

	return ret;

#else
	/* For QURT, just use read for now, since this doesn't block, we need to slow it down
	 * just a bit. */
	usleep(10000);
	return ::read(_serial_fd, buf, buf_length);
#endif
}

int
BATT_SMART::test()
{
	int sm_sub = orb_subscribe(ORB_ID(smart_battery_status));
	bool sm_updated = false;
	struct smart_battery_status_s sm_status;
	int bs_sub = orb_subscribe(ORB_ID(battery_status));
	bool bs_updated = false;
	struct battery_status_s bs_status;
	uint64_t start_time = hrt_absolute_time();

	// loop for 5 seconds
	while ((hrt_absolute_time() - start_time) < 5000000) {

		// display new info that has arrived from the orb
		orb_check(sm_sub, &sm_updated);
		if (sm_updated) {
			if (orb_copy(ORB_ID(smart_battery_status), sm_sub, &sm_status) == OK) {

				warnx("remaining:%d, ramaining time:%dmin, health:%d, cycle:%d, c1:%d.%dv, c2:%d.%dv, c3:%d.%dv, c4:%d.%dv, curr:%f, volt:%f, cap:%f\n",
						sm_status.remaining,
						sm_status.remaining_time,
						sm_status.health,
						sm_status.cycle_count,
						sm_status.voltage_count1/10, sm_status.voltage_count1%10,
						sm_status.voltage_count2/10, sm_status.voltage_count2%10,
						sm_status.voltage_count3/10, sm_status.voltage_count3%10,
						sm_status.voltage_count4/10, sm_status.voltage_count4%10,
						//sm_status.current/10, sm_status.current%10
						(double)sm_status.current,
						(double)sm_status.volt,
						(double)sm_status.remaining_capacity
						);
			}
		}

		orb_check(bs_sub, &bs_updated);
		if (bs_updated) {
			if (orb_copy(ORB_ID(battery_status), bs_sub, &bs_status) == OK) {
				warnx("V=%4.2f C=%4.2f remaining=%f warning=%d\n",
						(double)bs_status.voltage_v,
						(double)bs_status.current_a,
						(double)bs_status.remaining,
						bs_status.warning);
			}
		}

		// sleep for 0.05 seconds
		usleep(50000);
	}

	return OK;
}

void
BATT_SMART::shut()
{
	_shut = !_shut;
}

uint8_t
BATT_SMART::cal_PEC(const uint8_t buff[], uint8_t len) const
{
	// exit immediately if no data
	if (len <= 0) {
		return 0;
	}

	/**
	 *  Note: The PEC is calculated on all the message bytes. See http://cache.freescale.com/files/32bit/doc/app_note/AN4471.pdf
	 *  and http://www.ti.com/lit/an/sloa132/sloa132.pdf for more details
	 */

	// prepare temp buffer for calculating crc

	uint8_t tmp_buff[len];

	memcpy(&tmp_buff[0], buff, len);

	// initialise crc to zero
	uint8_t crc = 0;
	uint8_t shift_reg = 0;
	bool do_invert;

	// for each byte in the stream
	for (uint8_t i = 0; i < sizeof(tmp_buff); i++) {
		// load next data byte into the shift register
		shift_reg = tmp_buff[i];

		// for each bit in the current byte
		for (uint8_t j = 0; j < 8; j++) {
			do_invert = (crc ^ shift_reg) & 0x80;
			crc <<= 1;
			shift_reg <<= 1;

			if (do_invert) {
				crc ^= BATT_SMBUS_PEC_POLYNOMIAL;
			}
		}
	}

	// return result
	return crc;
}


/****************************************
Pec
*****************************************/
uint8_t const Pectable[]=
{
	0x00,0x07,0x0E,0x09,0x1C,0x1B,0x12,0x15,0x38,0x3F,0x36,0x31,0x24,0x23,0x2A,0x2D,
	0x70,0x77,0x7E,0x79,0x6C,0x6B,0x62,0x65,0x48,0x4F,0x46,0x41,0x54,0x53,0x5A,0x5D,
	0xE0,0xE7,0xEE,0xE9,0xFC,0xFB,0xF2,0xF5,0xD8,0xDF,0xD6,0xD1,0xC4,0xC3,0xCA,0xCD,
	0x90,0x97,0x9E,0x99,0x8C,0x8B,0x82,0x85,0xA8,0xAF,0xA6,0xA1,0xB4,0xB3,0xBA,0xBD,
	0xC7,0xC0,0xC9,0xCE,0xDB,0xDC,0xD5,0xD2,0xFF,0xF8,0xF1,0xF6,0xE3,0xE4,0xED,0xEA,
	0xB7,0xB0,0xB9,0xBE,0xAB,0xAC,0xA5,0xA2,0x8F,0x88,0x81,0x86,0x93,0x94,0x9D,0x9A,
	0x27,0x20,0x29,0x2E,0x3B,0x3C,0x35,0x32,0x1F,0x18,0x11,0x16,0x03,0x04,0x0D,0x0A,
	0x57,0x50,0x59,0x5E,0x4B,0x4C,0x45,0x42,0x6F,0x68,0x61,0x66,0x73,0x74,0x7D,0x7A,
	0x89,0x8E,0x87,0x80,0x95,0x92,0x9B,0x9C,0xB1,0xB6,0xBF,0xB8,0xAD,0xAA,0xA3,0xA4,
	0xF9,0xFE,0xF7,0xF0,0xE5,0xE2,0xEB,0xEC,0xC1,0xC6,0xCF,0xC8,0xDD,0xDA,0xD3,0xD4,
	0x69,0x6E,0x67,0x60,0x75,0x72,0x7B,0x7C,0x51,0x56,0x5F,0x58,0x4D,0x4A,0x43,0x44,
	0x19,0x1E,0x17,0x10,0x05,0x02,0x0B,0x0C,0x21,0x26,0x2F,0x28,0x3D,0x3A,0x33,0x34,
	0x4E,0x49,0x40,0x47,0x52,0x55,0x5C,0x5B,0x76,0x71,0x78,0x7F,0x6A,0x6D,0x64,0x63,
	0x3E,0x39,0x30,0x37,0x22,0x25,0x2C,0x2B,0x06,0x01,0x08,0x0F,0x1A,0x1D,0x14,0x13,
	0xAE,0xA9,0xA0,0xA7,0xB2,0xB5,0xBC,0xBB,0x96,0x91,0x98,0x9F,0x8A,0x8D,0x84,0x83,
	0xDE,0xD9,0xD0,0xD7,0xC2,0xC5,0xCC,0xCB,0xE6,0xE1,0xE8,0xEF,0xFA,0xFD,0xF4,0xF3
};

uint8_t BATT_SMART::cal_PEC2(uint8_t *data, uint8_t len)
{
	uint8_t i;
	uint8_t ret=0x00;
	for(i=0;i<len;i++)
	{
		ret=Pectable[ret^data[i]];
	}
	return ret;
}


/**
 * Local functions in support of the shell command.
 */
namespace batt_smart
{
/* oddly, ERROR is not defined for c++ */
#ifdef ERROR
# undef ERROR
#endif
const int ERROR = -1;


void   	start(const char *uart_path);
void	stop();
void	test();
void	reset();
void	status();
void	usage();
void   	shut();

void start(const char *uart_path)
{
	if (g_dev != nullptr) {
		PX4_WARN("batt_smart already started");
		return;
	}

	/* create the driver */
	g_dev = new BATT_SMART(uart_path);

	if (!g_dev || OK != g_dev->init()) {
		if (g_dev != nullptr) {
			delete g_dev;
			g_dev = nullptr;
		}

		PX4_ERR("start of batt_smart failed");
	}
}


/**
 * Stop the driver.
 */
void
stop()
{
	delete g_dev;
	g_dev = nullptr;
}

void test()
{
	if (g_dev == nullptr) {
		PX4_WARN("batt_smart not started");
		return;
	}

	g_dev->test();
}

void reset()
{

}

void shut()
{
	if (g_dev == nullptr) {
		PX4_WARN("batt_smart not started");
		return;
	}

	g_dev->shut();
}

void status()
{
	if (g_dev == nullptr) {
		PX4_WARN("batt_smart not started");
		return;
	}
	g_dev->print_info();
}

void usage()
{
	fprintf(stderr,
		"usage: batt_smart start [-d <devicename>]\n"
		"       batt_smart stop\n"
		"       batt_smart status\n");
	exit(1);
}



} // namespace


int
gten_batt_smart_main(int argc, char *argv[])
{
	/* set to default */
	const char *device_name = BATT_DEFAULT_UART_PORT;

	if (argc < 2) {
		warnx("missing command");
		batt_smart::usage();
	}

	if (!strcmp(argv[1], "start")) {

		/* work around getopt unreliability */
		if (argc > 3) {
			if (!strcmp(argv[2], "-d")) {
				device_name = argv[3];

			} else {
				PX4_ERR("DID NOT GET -d");
				goto out;
			}
		}

		batt_smart::start(device_name);


	}

	if (!strcmp(argv[1], "stop")) {
		batt_smart::stop();
	}

	/*
	 * Test the driver/device.
	 */
	// if (!strcmp(argv[1], "test")) {
	// 	batt_smart::test();
	// }

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		batt_smart::reset();
	}

	/*
	 * shut data, for connect test.
	 */
	if (!strcmp(argv[1], "shut")) {
		batt_smart::shut();
	}


	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		batt_smart::status();
	}
	return 0;

out:
	PX4_ERR("unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status'\n [-d /dev/ttyS0-n]");
	return 1;
}

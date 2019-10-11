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
 * @file dist_lidar.cpp
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

#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>
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
#include <systemlib/mavlink_log.h>
#include <uORB/uORB.h>
 
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

#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/uwb_sensor.h>

#define DIST_DEFAULT_UART_PORT   	"/dev/ttyS3"
#define DIST_DEFAULT_UART_BAUD   	115200

#define TIMEOUT_5HZ			50
#define DIST_WAIT_BEFORE_READ		2			//ms

#define PEC_BUFFER_SIZE			200

#define LIDAR_MEASUREMENT_INTERVAL_US   40000


uint16_t temp_msg[7] = {};
uint8_t msg_count = 0;

enum NRA24_Decode{
    Wait_Head,
    Start_Sequence_1,
    Start_Sequence_2,
    // Message_ID_A,
    // Message_ID_B,
    // Message_ID_C,
    // Data_Payload,
    Check_Sum
}decode_state(Wait_Head);


class DIST_LIDAR
{
public:
	DIST_LIDAR(const char *uart_path);
	virtual ~DIST_LIDAR();

	int 		init();
	ssize_t		read(struct file *filp, char *buffer, size_t buflen);
	int 		test();
	void		shut();

	/**
	 * Diagnostics - print some basic information about the driver.
	 */
	void		print_info();

	int           init_tick();
	void 	       stop_tick();

protected:
	int		probe();

private:
	// internal variables
	bool			_task_should_exit;				///< flag to make the main worker task exit
	bool			_enabled;					///< true if we have successfully connected to lidar
	volatile int		_task;						///< worker task
	uint64_t		_start_time;					///< system time we first attempt to communicate with lidar

	int			_serial_fd;					///< serial interface to GPS
	char			_port[20];					///< device / serial port path

	uint8_t 		_buf[PEC_BUFFER_SIZE+7+5];

	bool   			_shut;    //for test connect lose
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

	static void	       cycle_trampoline(void *arg);
	void	       cycle();
	work_s			_work{};	///< work queue for scheduling reads

	orb_advert_t             _distance_sensor_pub;
	orb_advert_t			 _uwb_sensor_pub;


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
	int parseChar(uint8_t c);
	void publish();
	void poll_subscriptions();


	float _dist = 0.0f;

	int32_t pos_x, pos_y, pos_z, vel_x, vel_y, vel_z;

	struct distance_sensor_s dist_measure = {};

	struct uwb_sensor_s uwb_measure = {};
};


/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int dist_lidar_main(int argc, char *argv[]);

namespace
{
	DIST_LIDAR	*g_dev = nullptr;
}


DIST_LIDAR::DIST_LIDAR(const char *uart_path):
	_task_should_exit(false),
	_enabled(false),
	_start_time(0),
	_serial_fd(-1),
	_shut(false),
	_distance_sensor_pub(nullptr),
	_uwb_sensor_pub(nullptr)
{
	/* store port name */
	strncpy(_port, uart_path, sizeof(_port));
	/* enforce null termination */
	_port[sizeof(_port) - 1] = '\0';

	// capture startup time
	_start_time = hrt_absolute_time();

	//_distance_sensor_pub = orb_advertise(ORB_ID(distance_sensor), &dist_measure);
}


DIST_LIDAR::~DIST_LIDAR()
{
	// /* tell the task we want it to go away */
	// _task_should_exit = true;

	// /* spin waiting for the task to stop */
	// for (unsigned i = 0; (i < 10) && (_task != -1); i++) {
	// 	/* give it another 100ms */
	// 	usleep(100000);
	// }

	// /* well, kill it anyway, though this will probably crash */
	// if (_task != -1) {
	// 	px4_task_delete(_task);
	// }
	//stop_tick();
	memset(&_work, 0, sizeof(_work));

}

int
DIST_LIDAR::init_tick()
{
	int ret = ENOTTY;
		/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (_serial_fd < 0) {
		PX4_ERR("DIST_LIDAR: failed to open serial port: %s err: %d", _port, errno);
		return ret;
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

	setBaudrate(DIST_DEFAULT_UART_BAUD);

	ret = work_queue(LPWORK, &_work, (worker_t)&DIST_LIDAR::cycle_trampoline, this, USEC2TICK(LIDAR_MEASUREMENT_INTERVAL_US));

	return ret;
}

void
DIST_LIDAR::stop_tick()
{
	work_cancel(HPWORK, &_work);
}

void
DIST_LIDAR::cycle_trampoline(void *arg)
{
	//DIST_LIDAR *dev = (DIST_LIDAR *)arg;

	g_dev->cycle();
}

void
DIST_LIDAR::cycle()
{
	static uint64_t lidar_time = 0;

	float lidar_hz = 1e6/(hrt_absolute_time() - lidar_time);
	lidar_time = hrt_absolute_time();
	PX4_WARN("lidar_hz %.2f",(double)lidar_hz);

	static uint64_t time_last_recv = 0;
	static int ret = 0;

	poll_subscriptions();

	ret = receive(TIMEOUT_5HZ);

	if (ret > 0) {

		time_last_recv = hrt_absolute_time();
	}
	else
	{
		if(time_last_recv - hrt_absolute_time() > 20000000)
		{
			printf("lost\n");
		}
	}

	publish();

	// schedule a fresh cycle call when the measurement is done
	 work_queue(LPWORK, &_work, (worker_t)&DIST_LIDAR::cycle_trampoline, this,
		   USEC2TICK(LIDAR_MEASUREMENT_INTERVAL_US));

}

int
DIST_LIDAR::init()
{
	/* start the GPS driver worker task */
	_task = px4_task_spawn_cmd("dist_lidar", SCHED_DEFAULT,
				   SCHED_PRIORITY_SLOW_DRIVER, 1600, (px4_main_t)&DIST_LIDAR::task_main_trampoline, NULL);

	if (_task < 0) {
		PX4_WARN("task start failed: %d", errno);
		_task = -1;
		return -errno;
	}

	return OK;
}

void DIST_LIDAR::task_main_trampoline(int argc, char *argv[])
{
	g_dev->task_main();
}

void
DIST_LIDAR::task_main()
{
	/* open the serial port */
	_serial_fd = ::open(_port, O_RDWR | O_NOCTTY);

	if (_serial_fd < 0) {
		PX4_ERR("DIST_LIDAR: failed to open serial port: %s err: %d", _port, errno);

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

	setBaudrate(DIST_DEFAULT_UART_BAUD);

	int ret;
	uint64_t time_last_recv = hrt_absolute_time();

	//uint64_t lidar_time = 0;

	/* loop handling received serial bytes and also configuring in between */
	while (!_task_should_exit) {

		//updateParam();
		poll_subscriptions();

		//float lidar_hz = 1e6/(hrt_absolute_time() - lidar_time);
		//lidar_time = hrt_absolute_time();

		//PX4_WARN("lidar_hz %.2f",(double)lidar_hz);

		ret = receive(TIMEOUT_5HZ);

		if (ret > 0) {
			time_last_recv = hrt_absolute_time();
			//printf("publish\n");
		}
		else
		{
			if(time_last_recv - hrt_absolute_time() > 20000000)
			{
				//printf("lost\n");
			}
		}

		publish();

		usleep(100000);
	}

	PX4_WARN("exiting");

	::close(_serial_fd);

	/* tell the dtor that we are exiting */
	_task = -1;
	px4_task_exit(0);
}

void DIST_LIDAR::publish()
{
	// //struct distance_sensor_s dist_measure = {};

	// dist_measure.timestamp = hrt_absolute_time();

	// dist_measure.current_distance = _dist*0.01f;
	// //report.orientation = _rotation;
	// dist_measure.min_distance = 0.1f;
	// dist_measure.max_distance =100.0f;
	// dist_measure.covariance = 0.0f;
	// /* TODO: set proper ID */
	// dist_measure.id = 0;

	// //PX4_WARN(" dist: %.2f", (double)_distance_sensor_pub);

	// if(_distance_sensor_pub == nullptr){

	// 	_distance_sensor_pub = orb_advertise(ORB_ID(distance_sensor), &dist_measure);

	// }else{
	// 	//PX4_WARN(" dist: %.2f", (double)_dist);
	// 	orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub, &dist_measure);
	// }

	uwb_measure.timestamp = hrt_absolute_time();
	uwb_measure.posx = (float)pos_x/1000;
	uwb_measure.posy = (float)pos_y/1000;
	uwb_measure.posz = (float)pos_z/1000;

	uwb_measure.velx = (float)vel_x/1000;
	uwb_measure.vely = (float)vel_y/1000;
	uwb_measure.velz = (float)vel_z/1000;

	if(_uwb_sensor_pub == nullptr){

		_uwb_sensor_pub = orb_advertise(ORB_ID(uwb_sensor), &uwb_measure);

	}else{
		//PX4_WARN(" uwb: %.2f", (double)uwb_measure.velz);
		orb_publish(ORB_ID(uwb_sensor), _uwb_sensor_pub, &uwb_measure);
	}

}


void DIST_LIDAR::poll_subscriptions()
{

}


int DIST_LIDAR::setBaudrate(unsigned baud)
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


int DIST_LIDAR::receive(unsigned timeout)
{
	int i = 0;
	int num_read = 0;
	
	//PX4_WARN("11");
	/* then poll or read for new data */
	num_read = pollOrRead(_buf, sizeof(_buf), timeout);
	//PX4_WARN("2");
	PX4_WARN("num %.2f",(double)num_read);
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

uint8_t uwb_count = 0;
uint8_t uwb_buf[20];

int DIST_LIDAR::parseChar(uint8_t c)
{
	int iRet = 0;

	//static float dist = 0.0f;
	//PX4_WARN("state %.2f",(double)decode_state);
        switch(decode_state)
        {
          case Wait_Head:

            if(c == 0x55){
              decode_state = Start_Sequence_1;
            }
            break;
          case Start_Sequence_1:

            if(c == 0x01){
              decode_state = Start_Sequence_2;
              uwb_count = 0;
            }else{
              decode_state = Wait_Head;
            }
            break;
          case Start_Sequence_2:
          	//PX4_WARN("start...");
          	uwb_buf[uwb_count++] = c;
          	//uwb_count ++;
          	//printf("uwb_count = %x ", uwb_count);
            
          	if (uwb_count >= 18)
          	{
          		//PX4_WARN("start...");
          		pos_x = (uwb_buf[2] << 16) | (uwb_buf[1] << 8) | uwb_buf[0];
          		pos_y = (uwb_buf[5] << 16) | (uwb_buf[4] << 8) | uwb_buf[3];
          		pos_z = (uwb_buf[8] << 16) | (uwb_buf[7] << 8) | uwb_buf[6];

          		vel_x = (uwb_buf[11] << 16) | (uwb_buf[10] << 8) | uwb_buf[9];
          		vel_y = (uwb_buf[14] << 16) | (uwb_buf[13] << 8) | uwb_buf[12];
          		vel_z = (uwb_buf[8] << 17) | (uwb_buf[16] << 8) | uwb_buf[15];
          		// printf("uwb_buf = %x %x %x\n", uwb_buf[2],uwb_buf[1],uwb_buf[0]);
          		// printf("pos_x = %d ", pos_x);
          		// printf("pos_y = %d ", pos_y);
          		// printf("pos_z = %d\n", pos_z);
          		// printf("vel_x = %d ", vel_x);
          		// printf("vel_y = %d ", vel_y);
          		// printf("vel_z = %d\n", vel_z);
          		decode_state = Check_Sum;
          		uwb_count = 0;
          	}
          	break;
          case Check_Sum:
            uint16_t sum1 = temp_msg[0]+temp_msg[1]+temp_msg[2]+temp_msg[3]+temp_msg[4]+temp_msg[5]+temp_msg[6];
            uint16_t sum2 = sum1 & 0xFF;

            if(sum2 == c){
              _dist = temp_msg[2]*256 + temp_msg[3];
              //PX4_WARN(" dist: %.2f", (double)_dist);
              iRet = 1;
            }
            decode_state = Wait_Head;//End_Sequence;
            break;
        }
	return iRet;
}

int DIST_LIDAR::handleMessage(int len)
{
	return 1;
}



void DIST_LIDAR::print_info()
{

}


int DIST_LIDAR::pollOrRead(uint8_t *buf, size_t buf_length, int timeout)
{
#ifndef __PX4_QURT

	/* For non QURT, use the usual polling. */

	//Poll only for the serial data. In the same thread we also need to handle orb messages,
	//so ideally we would poll on both, the serial fd and orb subscription. Unfortunately the
	//two pollings use different underlying mechanisms (at least under posix), which makes this
	//impossible. Instead we limit the maximum polling interval and regularly check for new orb
	//messages.
	//FIXME: add a unified poll() API
	const int max_timeout = 50;

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
			usleep(DIST_WAIT_BEFORE_READ * 1000);
			ret = ::read(_serial_fd, buf, buf_length);

			/*test code*/
			// printf("read ret:%d\n", ret);
			// if(ret > 0)
			// {
			// 	printf("\n****************\n");
			// 	//printf("read: num:%d  \n", ret);
			// 	for(int i=0; i<ret; i++)
			// 	{
			// 		printf("%02x ", buf[i]);
			// 	}
			// 	printf("\n****************\n\n");
			// }
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
DIST_LIDAR::test()
{

	return OK;
}

void
DIST_LIDAR::shut()
{
	_shut = !_shut;
}


/**
 * Local functions in support of the shell command.
 */
namespace dist_lidar
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
		PX4_WARN("dist_lidar already started");
		return;
	}

	/* create the driver */
	g_dev = new DIST_LIDAR(uart_path);

	if (!g_dev || OK != g_dev->init()) {
		if (g_dev != nullptr) {
			delete g_dev;
			g_dev = nullptr;
		}

		PX4_ERR("start of dist_lidar failed");
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
		PX4_WARN("dist_lidar not started");
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
		PX4_WARN("dist_lidar not started");
		return;
	}

	g_dev->shut();
}

void status()
{
	if (g_dev == nullptr) {
		PX4_WARN("dist_lidar not started");
		return;
	}
	g_dev->print_info();
}

void usage()
{
	fprintf(stderr,
		"usage: dist_lidar start [-d <devicename>]\n"
		"       dist_lidar stop\n"
		"       dist_lidar status\n");
	exit(1);
}



} // namespace


int
dist_lidar_main(int argc, char *argv[])
{
	/* set to default */
	const char *device_name = DIST_DEFAULT_UART_PORT;

	if (argc < 2) {
		warnx("missing command");
		dist_lidar::usage();
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

		dist_lidar::start(device_name);


	}

	if (!strcmp(argv[1], "stop")) {
		dist_lidar::stop();
	}

	/*
	 * Test the driver/device.
	 */
	if (!strcmp(argv[1], "test")) {
		dist_lidar::test();
	}

	/*
	 * Reset the driver.
	 */
	if (!strcmp(argv[1], "reset")) {
		dist_lidar::reset();
	}

	/*
	 * shut data, for connect test.
	 */
	if (!strcmp(argv[1], "shut")) {
		dist_lidar::shut();
	}


	/*
	 * Print driver status.
	 */
	if (!strcmp(argv[1], "status")) {
		dist_lidar::status();
	}
	return 0;

out:
	PX4_ERR("unrecognized command, try 'start', 'stop', 'test', 'reset' or 'status'\n [-d /dev/ttyS0-n]");
	return 1;
}

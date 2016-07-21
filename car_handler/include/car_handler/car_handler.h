/*
 * car_handler.h
 *
 *      Authors: Sebastian Ehmes
 *				 Nicolas Acero
 *				 Huynh-Tan Truong
 *				 Li Zhao
 *
 *		Co-Author: Eugen Lutz
 */

#ifndef CARHANDLER_H_
#define CARHANDLER_H_

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <sys/signal.h>
#include <errno.h>
#include <termios.h>
#include <stdint.h>

union battery_data{
	unsigned int int_val;
	char byte[2];
};

union hall_data{
	unsigned int int_val;
	char byte[2];
};

union us_data{
	unsigned int int_val;
	char byte[2];
};

class car_handler{
	public:
	//Singleton pattern for CarHandler
	static car_handler* get_car_handler(){
		if(ch_instance != NULL) {
			return ch_instance;
		}
		else {
			ch_instance = new car_handler();
			struct termios term_attr;
			struct sigaction saio;
			fd = open("/dev/ttyUSB0", O_RDWR | O_NOCTTY | O_NDELAY);
			if (fd == -1)
			{
			   perror("Unable to open serial port ttyUSB0\n");
			   exit(1);
			}
			saio.sa_handler = &car_handler::signal_handler_IO;
			saio.sa_flags = 0;
			saio.sa_restorer = NULL;
			sigaction(SIGIO,&saio,NULL);

			fcntl(fd, F_SETFL, FNDELAY);
			fcntl(fd, F_SETOWN, getpid());
			fcntl(fd, F_SETFL,  O_ASYNC );

			tcgetattr(fd,&term_attr);
			cfsetispeed(&term_attr,B115200);
			cfsetospeed(&term_attr,B115200);
			term_attr.c_cflag &= ~PARENB;
			term_attr.c_cflag &= ~CSTOPB;
			term_attr.c_cflag &= ~CSIZE;
			term_attr.c_cflag |= CS8;
			term_attr.c_cflag |= (CLOCAL | CREAD);
			term_attr.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
			term_attr.c_iflag &= ~(IXON | IXOFF | IXANY);
			term_attr.c_oflag &= ~OPOST;
			tcsetattr(fd,TCSANOW,&term_attr);
			//printf("UART1 configured....\n");
			return ch_instance;
		}
	 };

	//call destructor
	void delete_car_handler(){
		if(ch_instance != NULL) {
			delete ch_instance;
		}
	};

	//returns current value of front ultra sonic sensor
	unsigned int get_front_us() {
		return us_distance_front.int_val;
	}

	//returns current value of right ultra sonic sensor
	unsigned int get_right_us() {
		return us_distance_right.int_val;
	}

	//returns current value of left ultra sonic sensor
	unsigned int get_left_us() {
		return us_distance_left.int_val;
	}

	//returns 1 if line detected, 0 otherwise
	char is_line_detected() {
		return line_detected;
	}

	//returns current value of travelled distance
	unsigned int get_hall_distance() {
		return hall_distance.int_val;
	};

	//returns the voltage of the car battery in volts. Should never be lower than 7.2 volts
	unsigned int get_battery_value(){
		return battery.int_val;
	};

	//motor level [-10,...,10]
	void set_motor_level(char level) {
		write_buffer = (1 << 7) + (level+10);
		write(fd, &write_buffer, 1);
	};

	//steering range [-50...,50]
	void set_steering_level(char steering){
		write_buffer = (0x7F & (steering+50));
		write(fd, &write_buffer, 1);
	};

	//handle incoming data over UART
	static void signal_handler_IO(int status) {
		char start_sequence;
		read(fd, &start_sequence, 1);
		if(start_sequence == 0x0){
			read(fd, &start_sequence, 1);
			if(start_sequence == 0xF) {
				char temp[2] = {0, 0};
				//clear screen
				printf("\033[2J");
				//read us data
				read(fd, &temp, 2);
				if( temp[0] != 0 || temp[1] != 0){
					us_distance_front.byte[0] = temp[0];
					us_distance_front.byte[1] = temp[1];
				}
				//printf("front distance 1: %u\n", us_distance_front.int_val);

				read(fd, &temp, 2);
				if( temp[0] != 0 || temp[1] != 0){
					us_distance_right.byte[0] = temp[0];
					us_distance_right.byte[1] = temp[1];
				}
				//printf("right distance 2: %u\n", us_distance_right.int_val);

				read(fd, &temp, 2);
				if( temp[0] != 0 || temp[1] != 0){
					us_distance_left.byte[0] = temp[0];
					us_distance_left.byte[1] = temp[1];
				}
				//printf("left distance 3: %u\n", us_distance_left.int_val);

				//read line sensor data
				read(fd, &line_detected, 1);
				//printf("line detected: %c\n", line_detected);

				//read hall sensor data
				read(fd, &temp, 2);
				hall_distance.byte[0] = temp[0];
				hall_distance.byte[1] = temp[1];

				//printf("hall distance: %u\n", hall_distance.int_val);

				//read battery data
				read(fd, &temp, 2);
				battery.byte[0] = temp[0];
				battery.byte[1] = temp[1];

				//printf("battery: %u\n", battery.int_val);
			}
		}
	};

	private:

	static car_handler* ch_instance;
	car_handler(){};
	~car_handler(){};
	static us_data us_distance_front, us_distance_right, us_distance_left;
	static hall_data hall_distance;
	//file descriptor for UART port
	static int fd;
	static battery_data battery;
	static char write_buffer;
	static char line_detected;
};
#endif

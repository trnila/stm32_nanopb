#include <errno.h>
#include <fcntl.h> 
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <stdio.h>
#include "pb.h"
#include <pb_encode.h>
#include "messages/reverse.pb.h"
#include <stdlib.h>

#define error_message printf

	int
set_interface_attribs (int fd, int speed, int parity)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		error_message ("error %d from tcgetattr", errno);
		return -1;
	}

	cfsetospeed (&tty, speed);
	cfsetispeed (&tty, speed);

	tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars
	tty.c_iflag &= ~IGNBRK;         // disable break processing
	tty.c_lflag = 0;                // no signaling chars, no echo,
	// no canonical processing
	tty.c_oflag = 0;                // no remapping, no delays
	tty.c_cc[VMIN]  = 0;            // read doesn't block
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl

	tty.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
	// enable reading
	tty.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	tty.c_cflag |= parity;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CRTSCTS;

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
	{
		error_message ("error %d from tcsetattr", errno);
		return -1;
	}
	return 0;
}

	void
set_blocking (int fd, int should_block)
{
	struct termios tty;
	memset (&tty, 0, sizeof tty);
	if (tcgetattr (fd, &tty) != 0)
	{
		error_message ("error %d from tggetattr", errno);
		return;
	}

	tty.c_cc[VMIN]  = should_block ? 1 : 0;
	tty.c_cc[VTIME] = 5;            // 0.5 seconds read timeout

	if (tcsetattr (fd, TCSANOW, &tty) != 0)
		error_message ("error %d setting term attributes", errno);
}

void send_data(int fd, int type, int size, void *data) {
	write(fd, &type, sizeof(type));
	write(fd, &size, sizeof(size));
	write(fd, data, size);
}

int main(int argc, char** argv) {
	char *portname = "/dev/ttyUSB0";

	if(argc != 3) {
		printf("Usage: %s string number\n", argv[0]);
		return 1;
	}

	int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
	if (fd < 0)
	{
		error_message ("error %d opening %s: %s", errno, portname, strerror (errno));
		return 0;
	}

	set_interface_attribs (fd, B9600, 0);  // set speed to 115,200 bps, 8n1 (no parity)
	set_blocking (fd, 0);                // set no blocking

	char buf[100];
	int len = 0;
	time_t start = time(NULL);
	int i = 0;


	ReverseMsg msg = ReverseMsg_init_zero;
	strcpy(msg.name, argv[1]);
	msg.num = atoi(argv[2]);


	uint8_t buffer[128];
	pb_ostream_t stream = pb_ostream_from_buffer(buffer, sizeof(buffer));
	if(!pb_encode(&stream, ReverseMsg_fields, &msg)) {
		printf("Failed");
	}

	for(;;) {
		send_data(fd, 1234678, stream.bytes_written, buffer);
	}
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <pthread.h>
#include <semaphore.h>

char *version = "1.2.0(Author:eason)";

/*
 * Define some parity flags, internal use only.
 */
#define PARITY_NONE     0
#define PARITY_EVEN     1
#define PARITY_ODD      2

/*
 * Default port settings.
 */
int hardware;
int software;
int parity = PARITY_NONE;
int databits = 8;
int twostopb;
unsigned int baud = 9600;
sem_t sem;
int rx_uart_stop_flag = 0;
char *devname;
char *filename;
char *capfile;
int gotdevice;
int cfd;

unsigned long int tx_len = 0;

unsigned long int rx_len = 0;

/*
 * Signal handling.
 */
struct sigaction    sact;

/*
 * Baud rate table for baud rate conversions.
 */
typedef struct baudmap {
    unsigned int    baud;
    unsigned int    flag;
} baudmap_t;

struct baudmap  baudtable[] = {
    { 0, B0 },
    { 50, B50 },
    { 75, B75 },
    { 110, B110 },
    { 134, B134 },
    { 150, B150 },
    { 200, B200 },
    { 300, B300 },
    { 600, B600 },
    { 1200, B1200 },
    { 1800, B1800 },
    { 2400, B2400 },
    { 4800, B4800 },
    { 9600, B9600 },
    { 19200, B19200 },
    { 38400, B38400 },
    { 57600, B57600 },
    { 115200, B115200 },
    { 230400, B230400 },
    { 460800, B460800 }
};

#define NRBAUDS     (sizeof(baudtable) / sizeof(struct baudmap))

/*****************************************************************************/

/*
 *      Verify that the supplied baud rate is valid.
 */
int baud2flag(unsigned int speed)
{
    int     i;

    for (i = 0; (i < NRBAUDS); i++) {
        if (speed == baudtable[i].baud)
            return(baudtable[i].flag);
    }
    return(-1);
}

/*****************************************************************************/

/*
 *      Set up remote port termio settings according to
 *      user specification.
 */

int setremotetermios(int fd)
{
    struct termios  tio;

    memset(&tio, 0, sizeof(tio));
    tio.c_cflag = CREAD | CLOCAL;

    cfsetispeed(&tio, baud2flag(baud));
    cfsetospeed(&tio, baud2flag(baud));

    switch (parity) {
    case PARITY_ODD:
        tio.c_cflag |= PARENB | PARODD;
        break;
    case PARITY_EVEN:
        tio.c_cflag |= PARENB;
        break;
    default:
        break;
    }

    switch (databits) {
    case 5:
        tio.c_cflag |= CS5;
        break;
    case 6:
        tio.c_cflag |= CS6;
        break;
    case 7:
        tio.c_cflag |= CS7;
        break;
    default:
        tio.c_cflag |= CS8;
        break;
    }

    if (twostopb)
        tio.c_cflag |= CSTOPB;

    if (software)
        tio.c_iflag |= IXON | IXOFF;
    if (hardware)
        tio.c_cflag |= CRTSCTS;

    //tio.c_lflag &= ~(ICANON | ECHO | ECHOE);
    //tio.c_oflag &= ~OPOST;

    tio.c_cc[VMIN] = 1;
    tio.c_cc[VTIME] = 1;

    tcflush(fd, TCIOFLUSH);

    if (tcsetattr(fd, TCSANOW, &tio) < 0) {
        fprintf(stderr, "ERROR: remote tcsetattr(TCSANOW) failed, "
            "errno=%d\n", errno);
        return(0);
    }

    return 0;
}

static int exec_cmd(char *str, char *buff, int buff_len)
{
    int rc = 0,re;
    FILE *fp;

    fp = popen(str , "r");
    if (NULL == fp) {
        return -1;
    }

    while (fgets(buff, buff_len, fp) != NULL ) {
        printf("%s", buff);
        ;
    }

    rc = pclose(fp);
    re = WEXITSTATUS(rc);
    //printf("re_return=%d\n",re);

    return re;
}

/*****************************************************************************/
void sighandler(int signal)
{
    //printf("\n\nGot signal %d!\n", signal);
    //printf("Cleaning up...");
    //printf("Done\n");
    exit(1);
}

void set_signal_handle(void)
{
    sact.sa_handler = sighandler;
    sact.sa_flags = 0;
    sigaction(SIGHUP, &sact, NULL);
    sigaction(SIGINT, &sact, NULL);
    sigaction(SIGQUIT, &sact, NULL);
    sigaction(SIGPIPE, &sact, NULL);
    sigaction(SIGTERM, &sact, NULL);

    return ;
}

/* set baud rate */
void setBaudrate(struct termios *pNewtio, int baudrate)
{
    switch (baudrate)
    {
    case B1200:
        cfsetispeed(pNewtio, B1200);
        cfsetospeed(pNewtio, B1200);
        break;
    case B2400:
        cfsetispeed(pNewtio, B2400);
        cfsetospeed(pNewtio, B2400);
        break;
    case B4800:
        cfsetispeed(pNewtio, B4800);
        cfsetospeed(pNewtio, B4800);
        break;
    case B9600:
        cfsetispeed(pNewtio, B9600);
        cfsetospeed(pNewtio, B9600);
        break;
    case B19200:
        cfsetispeed(pNewtio, B19200);
        cfsetospeed(pNewtio, B19200);
        break;
    case B38400:
        cfsetispeed(pNewtio, B38400);
        cfsetospeed(pNewtio, B38400);
        break;
    case B115200:
        cfsetispeed(pNewtio, B115200);
        cfsetospeed(pNewtio, B115200);
        break;
    case B230400:
        cfsetispeed(pNewtio, B230400);
        cfsetospeed(pNewtio, B230400);
        break;
    default:
        cfsetispeed(pNewtio, B9600);
        cfsetospeed(pNewtio, B9600);
        break;
    }
}

/* set data bit */
void setDataBit(struct termios *pNewtio, int databit)
{
    pNewtio->c_cflag &= ~CSIZE;
    switch (databit) {
    case 8:
        pNewtio->c_cflag |= CS8;
        break;
    case 7:
        pNewtio->c_cflag |= CS7;
        break;
    case 6:
        pNewtio->c_cflag |= CS6;
        break;
    case 5:
        pNewtio->c_cflag |= CS5;
        break;
    default:
        pNewtio->c_cflag |= CS8;
        break;
    }
}

/* Set check bit */
void serParity(struct termios *pNewtio, char parity)
{
    switch (parity) {
    case 'N':
        pNewtio->c_cflag &= ~PARENB;
        break;
    case 'E':
        pNewtio->c_cflag |= PARENB;
        pNewtio->c_cflag &= ~PARODD;
        break;
    case 'O':
        pNewtio->c_cflag |= PARENB;
        pNewtio->c_cflag |= ~PARODD;
        break;
    default:
        pNewtio->c_cflag &= ~PARENB;
        break;
    }
}

/* Set stop bit */
static void setStopbit(struct termios *pNewtio, const char *stopbit)
{
    if (0 == strcmp(stopbit, "1")) {
        pNewtio->c_cflag &= ~CSTOPB;
    }
    else if (0 == strcmp(stopbit, "1")) {
        pNewtio->c_cflag &= ~CSTOPB;
    }
    else if (0 == strcmp(stopbit, "2")) {
        pNewtio->c_cflag |= CSTOPB;
    }
    else {
        pNewtio->c_cflag &= ~CSTOPB;
    }
}

/* Serial port setting function */
void setTermios(struct termios *pNewtio, int baudrate, int databit, char parity, const char *stopbit)
{
    bzero(pNewtio, sizeof(struct termios));

    pNewtio->c_cflag = CS8 | CREAD | CLOCAL;
    setBaudrate(pNewtio, baudrate);
    setDataBit(pNewtio, databit);
    serParity(pNewtio, parity);
    setStopbit(pNewtio, stopbit);
    pNewtio->c_cc[VTIME] = 10;
    pNewtio->c_cc[VMIN] = 1;
}

int init_uart(char *devname)
{
    int fd = -1;

    struct termios oldtio, newtio;

    if ((fd = open(devname, O_RDWR | O_NOCTTY)) == -1) {
        printf("Failed to open serial port");
        return -1;
    }
    
    //fcntl(fd, F_SETFL, 0);

    tcgetattr(fd, &oldtio);
    setTermios(&newtio, B115200, 8, 'N', "1");
    tcflush(fd, TCIOFLUSH);
    tcsetattr(fd, TCSANOW, &newtio);

    return fd;
}

void uinit_uart(int fd)
{
    close(fd);

    return ;
}

void *thread_rx_function(void *arg)
{
    int len, fd, ret;
    unsigned char buf[512];
    unsigned char *bp;
    unsigned char data;
    fd_set rfds;
    
    fd = *((int *)arg);
    bp = &data;
    
    memset(buf, 0, sizeof(buf));
    while(1) {
        sem_wait(&sem);
        if (rx_uart_stop_flag) {
            sem_post(&sem);
            if (rx_len == tx_len) {
                //printf("rx total %ld bytes!\n", rx_len);
                break;
            }
        }
        sem_post(&sem);
        FD_ZERO(&rfds);
        FD_SET(fd, &rfds);
        ret = select(fd+1, &rfds, NULL, NULL, NULL);
        if(ret == -1) {
            fprintf(stderr, "select error.\n");
            break;
        } else if(ret) {
            if(FD_ISSET(fd, &rfds)) {
                ret = read(fd, bp, 1);
                rx_len++;

                if (cfd > 0)
                    write(cfd, bp, 1);
		else
                    write(1, bp, 1);
            }
        } else {
            fprintf(stderr, "select timeout.\n");
        }
    }
}

void *thread_tx_function(void *arg)
{
    int fd, sfd, n, rc;
    char *bp;
    unsigned char ibuf[512];
    int loop = 0;
    fd_set  outfds;
    
    fd = *((int *)arg);
    
    sfd = open(filename, O_RDONLY);
    if (sfd == -1) {
        fprintf(stderr, "ERROR: open(%s) failed, errno=%d\n", filename, errno);
        exit(-1);
    }
    
    while ((n = read(sfd, ibuf, sizeof(ibuf))) > 0) {
        bp = ibuf;
        //printf("tx will write %d bytes!\n", n);
        while(n > 0) {
            FD_ZERO(&outfds);
            FD_SET(fd, &outfds);
            if (select(fd + 1, NULL, &outfds, NULL, NULL) <= 0)
                break;
            
            if (FD_ISSET(fd, &outfds)) {
                rc = write(fd, bp, 1);
                if (rc <= 0) {
                    //printf("tx %d byte\n", rc);
                    break;
                }
                tx_len++;
                n -= rc;
                bp += rc;
            }
        }
    }
    
    close(sfd);
    //printf("tx thread end!tx len %ld bytes\n", tx_len);
    sem_wait(&sem);
    rx_uart_stop_flag = 1;
    sem_post(&sem);
}

void usage(FILE *fp, int rc)
{
    fprintf(fp, "Usage: uart_unit [-?vgheonxrwct125678] [-s speed] [-w file] "
        "[-d sfile] [device]\n\n"
        "\t-h?\tthis help\n"
        "\t-1\t1 stop bits (default)\n"
        "\t-2\t2 stop bits\n"
        "\t-5\t5 data bits\n"
        "\t-6\t6 data bits\n"
        "\t-7\t7 data bits\n"
        "\t-8\t8 data bits (default)\n"
        "\t-e\teven parity\n"
        "\t-o\todd parity\n"
        "\t-n\tno parity (default)\n"
        "\t-g\tgenerate 128KB file with 0x55\n"
        "\t-x\tuse software flow (xon/xoff)\n"
        "\t-r\tuse hardware flow (rts/cts)\n"
        "\t-f\tpass xon/xoff flow control to remote\n"
        "\t-s\tbaud rate (default 9600)\n"
        "\t-w\tcapture output to local file\n"
        "\t-d\tdownload file name\n");
    exit(rc);
}

int main(int argc, char *argv[])
{
    struct stat statbuf;
    int ret, fd;
    size_t len;
    int c;
    char *path = NULL;
    pthread_t thread_rx, thread_tx;
    char data[1024];
    int tfd;
    int i = 0;
    
    while ((c = getopt(argc, argv, "?vhgeonxrctf125678w:s:a:b:d:")) > 0) {
    switch (c) {
    case 'v':
        printf("%s: version %s\n", argv[0], version);
        exit(0);
    case 'g':
	memset(data, 0x55, sizeof(data));
	printf("generate 128KB file with 0x55.(128KB_file_0x55)\n");
        if ((tfd = open("128KB_file_0x55", (O_WRONLY | O_TRUNC | O_CREAT), 0666)) < 0) {
            fprintf(stderr, "ERROR: failed to open(%s), errno=%d\n",
                "128KB_file_0x55", errno);
            exit(0);
        }
	for(i = 0; i < 128; i++)
	    write(tfd, data, 1024);
	close(tfd);
	exit(0);
    case '1':
        twostopb = 0;
        break;
    case '2':
        twostopb = 1;
        break;
    case '5':
        databits = 5;
        break;
    case '6':
        databits = 6;
        break;
    case '7':
        databits = 7;
        break;
    case '8':
        databits = 8;
        break;
    case 'r':
        hardware++;
        break;
    case 'x':
        software++;
        break;
    case 'o':
        parity = PARITY_ODD;
        break;
    case 'e':
        parity = PARITY_EVEN;
        break;
    case 'n':
        parity = PARITY_NONE;
        break;
    case 's':
        baud = atoi(optarg);
        if (baud2flag(baud) < 0) {
            fprintf(stderr,
            "ERROR: baud speed specified %d\n",
            baud);
            exit(1);
        }
        break;
    case 'w':
        capfile = optarg;
        break;
    case 'd':
        filename = optarg;
        break;
    case 'a':
        gotdevice++;
        devname = optarg;
        break;
    case 'h':
    case '?':
        usage(stdout, 0);
        break;
    default:
        fprintf(stderr, "ERROR: unkown option '%c'\n", c);
        usage(stderr, 1);
        break;
    }
    }
    
    if ((optind < argc) && (gotdevice == 0)) {
        gotdevice++;
        devname = argv[optind++];
    }

    if (gotdevice == 0) {
        fprintf(stderr, "ERROR: no device specified\n");
        usage(stderr, 1);
    }
    if (optind < argc) {
        fprintf(stderr, "ERROR: too many arguments\n");
        usage(stderr, 1);
    }

    /* If a_devname does not exist as is, prepend '/dev/' */
    if (devname[0] != '/' && stat(devname, &statbuf) == -1) {
        len = strlen(devname) + strlen("/dev/") + 1;
        path = calloc(len, sizeof(*path));
        strncpy(path, "/dev/", len);
        strncat(path, devname, len);
    } else {
        path = strdup(devname);
    }
    if (path == NULL) {
        fprintf(stderr, "ERROR: failed to alloc() path, "
            "errno=%d\n", errno);
        exit(1);
    }
    
    #if 1
    if ((fd = open(path, (O_RDWR | O_NOCTTY))) < 0) {
    fprintf(stderr, "ERROR: failed to open() %s, "
        "errno=%d\n", path, errno);
    }
    setremotetermios(fd);
    #else
    fd = init_uart(path);
    #endif
    
    if (path != NULL) {
        free(path);
    }
    if (fd < 0) {
        exit(1);
    }
    
    sem_init(&sem, 0, 1);
    
    if (capfile != NULL) {
        if ((cfd = open(capfile, (O_WRONLY | O_TRUNC | O_CREAT), 0666)) < 0) {
            fprintf(stderr, "ERROR: failed to open(%s), errno=%d\n",
                capfile, errno);
            exit(0);
        }
    }
    
    set_signal_handle();
    
    ret = pthread_create(&thread_rx, NULL, thread_rx_function, (void *)(&fd));
    if(ret != 0){
        fprintf(stderr, "rx thread creation failed.");
        exit(1);
    }
    
    ret = pthread_create(&thread_tx, NULL, thread_tx_function, (void *)(&fd));
    if(ret != 0){
        fprintf(stderr, "tx thread creation failed.");
        exit(1);
    }

    pthread_join(thread_tx, NULL);
    pthread_join(thread_rx, NULL);

    uinit_uart(fd);
    if (cfd > 0)
        close(cfd);
    
    return 0;
}


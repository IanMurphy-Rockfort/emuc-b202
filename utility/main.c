/*
 * emucd.c - userspace daemon for Innodisk EMUC-B201 CAN interface driver
 *
 * Copyright (c) 2016 Innodisk Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Send feedback to <linux-can@vger.kernel.org>
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>
#include <pwd.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <termios.h>
#include <linux/tty.h>
#include <linux/sockios.h>
#include <linux/version.h>

#include "version.h"

/*=================================== EMUC library ===================================*/
#define  RS232_PORTNR  68
#define  VER_LEN       16

/*-------------------*/
enum
{
  EMUC_CAN_1 = 0,
  EMUC_CAN_2
};

/*-------------------*/
enum
{
  EMUC_INACTIVE = 0,
  EMUC_ACTIVE
};

/*-------------------*/
enum
{
  EMUC_NORMAL = 0,
  EMUC_LISTEN
};

/*-------------------*/
enum
{
  EMUC_DIS_ALL = 0,
  EMUC_EE_ERR,
  EMUC_BUS_ERR,
  EMUC_EN_ALL = 255
};

/*-------------------*/
typedef struct
{
  char   fw   [VER_LEN];
  char   api  [VER_LEN];
  char   model[VER_LEN];

} VER_INFO;


extern char *comports [RS232_PORTNR];

extern int EMUCOpenSocketCAN (int com_port);
extern int EMUCCloseDevice   (int com_port);
extern int EMUCClearFilter   (int com_port, int CAN_port);
extern int EMUCSetErrorType  (int com_port, int err_type);
extern int EMUCShowVer       (int com_port, VER_INFO *ver_info);
extern int EMUCInitCAN       (int com_port, int CAN1_sts,  int CAN2_sts);
extern int EMUCSetBaudRate   (int com_port, int CAN1_baud, int CAN2_baud);
extern int EMUCSetMode       (int com_port, int CAN1_mode, int CAN2_mode);
/*====================================================================================*/

/*
 * Ldisc number for emuc.
 * Beform 3.1.0, the ldisc number is private define
 * in kernel, usrspace application cannot use it.
 */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,1,0)
  #define N_EMUC (NR_LDISCS - 1)
#endif

#define   DAEMON_NAME      "emucd"
#define   TTYPATH_LENGTH   64

static int  emucd_running;
static int  exit_code;
static char ttypath [TTYPATH_LENGTH];

/* static function prototype */
static void reset_2_default (int com_port);
static void print_version (char *prg);
static void print_usage (char *prg);
static void child_handler (int signum);
static int look_up_emuc_comport (const char *tty);
static int check_can_speed_format (const char * speed);
static const char *look_up_can_speed (int speed);

/* v2.1: change some variable to global (for end process) */
int             port;
int             ldisc;
int             fd;
speed_t         old_ispeed;
speed_t         old_ospeed;
struct termios  tios;


/*------------------------------------------------------------------------------------*/
int main (int argc, char *argv[])
{
  int             sp;
  int             opt;
  int             channel;
  int             run_as_daemon = 1;
  char           *pch;
  char           *tty = NULL;
  char           *name[2];
  char           *speed = NULL;
  char            buf[IFNAMSIZ + 1];
  char const     *devprefix = "/dev/";


#ifdef N_EMUC
  ldisc = N_EMUC;
#else
  ldisc = 17; /* N_SLCAN */
#endif

  name[0] = NULL;
  name[1] = NULL;
  ttypath[0] = '\0';

  while ((opt = getopt(argc, argv, "s:v?hF")) != -1)
  {
    switch (opt)
    {
      case 's':
                speed = optarg;
                if (check_can_speed_format(speed) < 0)
                  print_usage(argv[0]);
                break;
      case 'F':
                run_as_daemon = 0;
                break;
      case 'v':
                if(argc == 3)
                {
                  print_version(argv[2]);                  
                }

      case 'h':
      case '?':
      default:
                print_usage(argv[0]);
                break;
    }
  }


  /* Initialize the logging interface */
  openlog(DAEMON_NAME, LOG_PID, LOG_LOCAL5);

  /* Parse serial device name and optional can interface name */
  tty = argv[optind];
  if (NULL == tty)
    print_usage(argv[0]);

  name[0] = argv[optind + 1];
  if (name[0])
    name[1] = argv[optind + 2];

  /* Prepare the tty device name string */
  pch = strstr(tty, devprefix);
  if (pch != tty)
    snprintf(ttypath, TTYPATH_LENGTH, "%s%s", devprefix, tty);
  else
    snprintf(ttypath, TTYPATH_LENGTH, "%s", tty);

  syslog(LOG_INFO, "starting on TTY device %s", ttypath);

  /* Set can spped by EMUC library */
  if(speed)
  {
    port = look_up_emuc_comport(ttypath);
    
    if(port < 0)
    {
      syslog(LOG_ERR, "unknown comport");
      exit(EXIT_FAILURE);
    }

    if (!EMUCOpenSocketCAN(port))
    {
      /* reset to default */
      reset_2_default(port);

      if(strlen(speed) == 2)
      {
        sp = atoi(speed);
        syslog(LOG_INFO, "Set can speed to %s on channel 1", look_up_can_speed(sp / 10));
        syslog(LOG_INFO, "Set can speed to %s on channel 2", look_up_can_speed(sp % 10));
        EMUCSetBaudRate(port, sp / 10, sp % 10);
      }
      else
      {
        syslog(LOG_INFO, "set can speed to %s on both channel", look_up_can_speed(atoi(speed)));
        EMUCSetBaudRate(port, atoi(speed), atoi(speed));
      }

      EMUCInitCAN (port, EMUC_ACTIVE, EMUC_ACTIVE);
      EMUCCloseDevice(port);
    }
    else
    {
      syslog(LOG_ERR, "fail to open comport");
      exit(EXIT_FAILURE);
    }
  }

  /* Daemonize */
  if (run_as_daemon)
  {
    if (daemon(0, 0))
    {
      syslog(LOG_ERR, "failed to daemonize");
      exit(EXIT_FAILURE);
    }
  }

  /* Trap signals that we expect to receive */
  /* End process */
  signal(SIGINT, child_handler);
  signal(SIGTERM, child_handler);

  emucd_running = 1;

  /* Now we are a daemon -- do the work for which we were paid */
  fd = open(ttypath, O_RDWR | O_NONBLOCK | O_NOCTTY);

  if (fd < 0)
  {
    syslog(LOG_NOTICE, "failed to open TTY device %s\n", ttypath);
    perror(ttypath);
    exit(EXIT_FAILURE);
  }

  /* Configure baud rate */
  memset(&tios, 0, sizeof(struct termios));
  if (tcgetattr(fd, &tios) < 0)
  {
    syslog(LOG_NOTICE, "failed to get attributes for TTY device %s: %s\n", ttypath, strerror(errno));
    exit(EXIT_FAILURE);
  }

  /* Get old values for later restore */
  old_ispeed = cfgetispeed(&tios);
  old_ospeed = cfgetospeed(&tios);

  /* Reset UART settings */
  cfmakeraw(&tios);
  tios.c_iflag &= ~IXOFF;
  tios.c_cflag &= ~CRTSCTS;

  /* Baud Rate */
  cfsetispeed(&tios, B9600);
  cfsetospeed(&tios, B9600);

  /* apply changes */
  if (tcsetattr(fd, TCSADRAIN, &tios) < 0)
    syslog(LOG_NOTICE, "Cannot set attributes for device \"%s\": %s!\n", ttypath, strerror(errno));

  /* set slcan like discipline on given tty */
  if (ioctl(fd, TIOCSETD, &ldisc) < 0)
  {
    perror("ioctl TIOCSETD");
    exit(EXIT_FAILURE);
  }
  
  for (channel = 0; channel < 2; channel++)
  {

    /* retrieve the name of the created CAN netdevice */
    if (ioctl(fd, SIOCGIFNAME, buf) < 0)
    {
      perror("ioctl SIOCGIFNAME");
      exit(EXIT_FAILURE);
    }

    syslog(LOG_NOTICE, "attached TTY %s channel %d to netdevice %s\n", ttypath, channel, buf);

    /* try to rename the created netdevice */
    if (name[channel])
    {
      struct ifreq ifr;
      int s = socket(PF_INET, SOCK_DGRAM, 0);

      if (s < 0)
        perror("socket for interface rename");
      else {
        strncpy(ifr.ifr_name, buf, IFNAMSIZ);
        strncpy(ifr.ifr_newname, name[channel], IFNAMSIZ);

        if (ioctl(s, SIOCSIFNAME, &ifr) < 0) {
          syslog(LOG_NOTICE, "netdevice %s rename to %s failed\n", buf, name[channel]);
          perror("ioctl SIOCSIFNAME rename");
          exit(EXIT_FAILURE);
        } else
          syslog(LOG_NOTICE, "netdevice %s renamed to %s\n", buf, name[channel]);

        close(s);
      }
    }
  }

  /* The Big Loop */
  while (emucd_running)
  {
    sleep(1); /* wait 1 second */
  }


  /* end process: must kill by pkill -2 emucd */
  /*--------------------------------------------------------------*/
  /* Reset line discipline */
  syslog(LOG_INFO, "stopping on TTY device %s", ttypath);
  ldisc = N_TTY;
  if (ioctl(fd, TIOCSETD, &ldisc) < 0)
  {
    perror("ioctl TIOCSETD");
    exit(EXIT_FAILURE);
  }

  /* Reset old rates */
  cfsetispeed(&tios, old_ispeed);
  cfsetospeed(&tios, old_ospeed);

  /* apply changes */
  if (tcsetattr(fd, TCSADRAIN, &tios) < 0)
    syslog(LOG_NOTICE, "Cannot set attributes for device \"%s\": %s!\n", ttypath, strerror(errno));

  /* Finish up */
  syslog(LOG_NOTICE, "terminated on %s", ttypath);
  closelog();

  EMUCInitCAN(port, EMUC_INACTIVE, EMUC_INACTIVE);
  EMUCCloseDevice(port);

  return exit_code;
  /*--------------------------------------------------------------*/

} /* END: main() */




/*------------------------------------------------------------------------------------*/
static void reset_2_default (int com_port)
{
  EMUCInitCAN     (com_port, EMUC_INACTIVE, EMUC_INACTIVE);
  EMUCClearFilter (com_port, EMUC_CAN_1);
  EMUCClearFilter (com_port, EMUC_CAN_2);
  EMUCSetErrorType(com_port, EMUC_DIS_ALL);
  EMUCSetMode     (com_port, EMUC_NORMAL, EMUC_NORMAL);

} /* END: reset_2_default() */



/*------------------------------------------------------------------------------------*/
static void print_version (char *prg)
{
  int          com_port;
  char        *pch;
  char const  *devprefix = "/dev/";
  VER_INFO     ver_info;

  pch = strstr(prg, devprefix);
  if (pch != prg)
  {
    snprintf(ttypath, TTYPATH_LENGTH, "%s%s", devprefix, prg);
  }
  else
  {
    snprintf(ttypath, TTYPATH_LENGTH, "%s", prg);
  }

  com_port = look_up_emuc_comport(ttypath);

  if(com_port != -1)
  {
    /* utility version */
    fprintf(stdout, "%s\n", EMUC_DAEMON_UTILITY_VERSION);
    fprintf(stdout, "%s\n", "==============");

    /* api version */
    EMUCOpenSocketCAN(com_port);
    EMUCInitCAN(com_port, EMUC_INACTIVE, EMUC_INACTIVE);
    EMUCShowVer(com_port, &ver_info);
    fprintf(stdout, "FW ver: %s\n",  ver_info.fw);
    fprintf(stdout, "LIB ver: %s\n", ver_info.api);
    fprintf(stdout, "Model: %s\n",   ver_info.model);

  }

  exit(EXIT_SUCCESS);

} /* END: print_version() */


/*------------------------------------------------------------------------------------*/
static void print_usage (char *prg)
{
  fprintf(stderr, "\nUsage: %s [options] <tty> [canif-name] [canif2-name]\n\n", prg);
  fprintf(stderr, "Options: -s <speed>[<speed>] (set CAN speed 3..7)\n");
  fprintf(stderr, "                4: 100 KBPS\n");
  fprintf(stderr, "                5: 125 KBPS\n");
  fprintf(stderr, "                6: 250 KBPS\n");
  fprintf(stderr, "                7: 500 KBPS\n");
  fprintf(stderr, "                8: 800 KPS\n");
  fprintf(stderr, "                9: 1 MBPS\n");
  fprintf(stderr, "         -F         (stay in foreground; no daemonize)\n");
  fprintf(stderr, "         -h         (show this help page)\n");
  fprintf(stderr, "         -v         (show version info)\n");
  fprintf(stderr, "\nExamples:\n");
  fprintf(stderr, "emucd_64 -v /dev/ttyACM0\n");
  fprintf(stderr, "emucd_64 -s7 /dev/ttyACM0\n");
  fprintf(stderr, "emucd_64 -s79 /dev/ttyACM0 can0 can1\n");
  fprintf(stderr, "(Note: 32bit OS will use emucd_32.)\n");
  fprintf(stderr, "\n");
  exit(EXIT_FAILURE);

} /* END: print_usage() */


/*------------------------------------------------------------------------------------*/
static void child_handler (int signum)
{
  switch (signum)
  {
    case SIGUSR1:
                  /* exit parent */
                  exit(EXIT_SUCCESS);
                  break;
    case SIGALRM:
    case SIGCHLD:
                  syslog(LOG_NOTICE, "received signal %i on %s", signum, ttypath);
                  exit_code = EXIT_FAILURE;
                  emucd_running = 0;
                  break;
    case SIGINT:
    case SIGTERM:
                  syslog(LOG_NOTICE, "received signal %i on %s", signum, ttypath);
                  exit_code = EXIT_SUCCESS;
                  emucd_running = 0;
                  break;
  }

} /* END: child_handler() */



/*------------------------------------------------------------------------------------*/
static int look_up_emuc_comport (const char *tty)
{
  int i;

  for(i=0; i<RS232_PORTNR; i++)
    if(strcmp(tty, comports[i]) == 0)
      return i;

  return -1; /* invalid port number for EMUC library */


} /* END: look_up_emuc_comport() */



/*------------------------------------------------------------------------------------*/
static int check_can_speed_format (const char * speed)
{
  int          len;
  const char  *p;

  len = strlen(speed);

  if(len < 1 || len > 2)
    return -1;

  for (p = speed; *p == '\0'; p++)
    if (*p < '3' || *p > '7')
      return -1;

  return 0;

} /* END: check_can_speed_format() */



/*------------------------------------------------------------------------------------*/
static const char *look_up_can_speed (int speed)
{
  switch (speed)
  {
    case 4:   return "100 KBPS";
    case 5:   return "125 KBPS";
    case 6:   return "250 KBPS";
    case 7:   return "500 KBPS";
    case 8:   return "800 KBPS";
    case 9:   return "1   MBPS";
    default:  return "unknown";
  }

} /* END: look_up_can_speed() */
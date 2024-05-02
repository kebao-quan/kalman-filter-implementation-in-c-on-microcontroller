/************************************************/
/*                                              */
/*  Real-time Task Signal Filtering Demo        */
/*                                              */
/*  This demo combines RT POSIX.4 signals and   */
/*  timers, and generates a periodic signal     */
/*  that is filtered using a digital filter     */
/*  with given coefficients.                    */
/*                                              */
/*  Author: Renato Mancuso (BU)                 */
/*  Class: CS454/654 Embedded Syst. Devel.      */
/*  Date: April 2020                            */
/*                                              */
/************************************************/


#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <math.h>

#include "kf.h"

#define SIG_SAMPLE SIGRTMIN
#define SIG_HZ 1
#define OUTFILE "signal.txt"
#define F_SAMPLE 50

#define USAGE_STR				\
	"Usage: %s [-s] [-n] [-f]\n"		\
	"\t -s: plot original signal\n"		\
	"\t -n: plot noisy signal\n"		\
	"\t -n: plot filtered signal\n"		\
	""

void handle_my_signal(int signo, siginfo_t * info, void * extra);
double get_butter(double cur, double * a, double * b);
void parse_cmdline(int argc, char ** argv);

/* Global flags reflecting the command line parameters */
int flag_signal = 0;
int flag_noise = 0;
int flag_filtered = 0;

/* Kalman filter parameters */
double F_[2 * 2] = {
	1, 1,
	0, 1
};
double P_[2 * 2] = {
	1000,    0,
	0,     1000
};
double R_[1] = {
	10
};
double Q_[2 * 2] = {
	0.0125, 0.025,
	0.025, 0.05
};
double x_[2] = {
	0,
	0
};
KalmanInput2D kf;


int main(int argc, char ** argv)
{
	createMatrix(2, 2, &kf.F, F_);
	createMatrix(2, 2, &kf.P, P_);
	createMatrix(1, 1, &kf.R, R_);
	createMatrix(2, 2, &kf.Q, Q_);
	createMatrix(2, 1, &kf.x, x_);


	printf("Starting kalman filtering demo\n");
	struct sigaction sa;
	sigset_t mask, wait_mask;
	struct sigevent ev;
	struct itimerspec it;
	timer_t timer;
	int outfile;
	FILE * outfd;
	
	int f_sample = F_SAMPLE; /* Frequency of sampling in Hz */
	double t_sample = (1.0/f_sample) * 1000 * 1000 * 1000; /* Sampling period in ns */

	parse_cmdline(argc, argv);
	outfile = open(OUTFILE, O_WRONLY | O_CREAT | O_TRUNC, 0666);
	outfd = fdopen(outfile, "w");
	
	printf("Output file: %s\n", OUTFILE);
	if (outfile < 0 || !outfd) {
		perror("Unable to open/create output file. Exiting.");
		return EXIT_FAILURE;
	}

	sa.sa_flags = SA_SIGINFO;
	
	/* Not using sa.sa_handler */
	sa.sa_sigaction = handle_my_signal;

	sigemptyset(&mask);
	sigemptyset(&wait_mask);
	sigemptyset(&sa.sa_mask);

	sigaddset(&sa.sa_mask, SIG_SAMPLE);
	sigaddset(&mask, SIG_SAMPLE);
	
	sigaddset(&wait_mask, SIGRTMAX);

	sigprocmask(SIG_BLOCK, &mask, NULL);

	sigaction(SIG_SAMPLE, &sa, NULL);
	
	/* Timer creation */
	memset(&ev, 0, sizeof(ev));
	ev.sigev_notify = SIGEV_SIGNAL;
	ev.sigev_signo = SIG_SAMPLE;
	ev.sigev_value.sival_ptr = (void *)outfd;
	
	if (timer_create(CLOCK_REALTIME, &ev, &timer) == -1) {
		perror("Unable to create timer. Exiting.");
		return EXIT_FAILURE;
	}

	memset(&it, 0, sizeof(it));
	it.it_value.tv_sec = 1;
	it.it_value.tv_nsec = 0;
	it.it_interval.tv_sec = 0;
	it.it_interval.tv_nsec = t_sample;
	
	
	if (timer_settime(timer, 0, &it, NULL)) {
		return EXIT_FAILURE;
	}
	
	while (1) {
	    sigsuspend(&wait_mask);	    
	}

	return EXIT_SUCCESS;
}

void handle_my_signal(int signo, siginfo_t * info, void * extra)
{
	static double glob_time = 0;
	FILE * outfd = (FILE *)info->si_value.sival_ptr;
	
	(void)extra;
	(void)info;
	(void)signo;
	
	// Seed the random number generator
    // srand(time(NULL));
    // Generate a random double between -1 and 1
    //double randomValue = -1 + (double)rand() / ((double)RAND_MAX / 2);
	//sig_val += randomValue;

	double sig_val = 1*sin(2*M_PI*SIG_HZ*glob_time);
	double sig_noise = sig_val + 0.5*cos(2*M_PI*10*glob_time);
	sig_noise += 0.1*cos(2*M_PI*4*glob_time);
	sig_noise += 0.5*cos(2*M_PI*12*glob_time);
	sig_noise += 0.3*cos(2*M_PI*15*glob_time);
	sig_noise += 0.2*cos(2*M_PI*18*glob_time);

	// if (glob_time > 5) {
	// 	sig_noise += glob_time;
	// }
	

	//double sig_filt = get_butter(sig_noise, a, b);
	predict(&kf);
	update(&kf, sig_noise);
	double sig_filt = M(&kf.x, 0, 0);

	fprintf(outfd, "%lf,", glob_time);

	if (flag_signal)
		fprintf(outfd, "%lf,", sig_val);
	else
		fprintf(outfd, ",");
	
	if (flag_noise)
		fprintf(outfd, "%lf,", sig_noise);
	else
		fprintf(outfd, ",");

	if (flag_filtered)
		fprintf(outfd, "%lf,", sig_filt);
	else
		fprintf(outfd, ",");

	fprintf(outfd, "\n");
	//fprintf(outfd, "%lf,%lf,%lf,%lf\n", glob_time, sig_val, sig_noise, sig_filt);
	fflush(outfd);

	glob_time += (1.0/F_SAMPLE); /* Sampling period in s */
	
}

void parse_cmdline(int argc, char ** argv)
{
	int opt;
	
	while ((opt = getopt(argc, argv, "snf")) != -1) {
		switch (opt) {
		case 's':
			flag_signal = 1;
			break;
		case 'n':
			flag_noise = 1;
			break;
		case 'f':
			flag_filtered = 1;
			break;
		default: /* '?' */
			fprintf(stderr, USAGE_STR, argv[0]);
			exit(EXIT_FAILURE);
		}
	}
	
	if ((flag_signal | flag_noise | flag_filtered) == 0)
	{
		flag_signal = flag_noise = flag_filtered = 1;
	}
}

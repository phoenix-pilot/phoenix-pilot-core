#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/time.h>

#include <rcbus.h>
#include <board_config.h>


struct {
	bool run;
} rclog_common;


static void rclog_rcbusHandler(const rcbus_msg_t *msg)
{
	const static uint16_t maxTriggerVal = MIN_CHANNEL_VALUE + ((95 * (MAX_CHANNEL_VALUE - MIN_CHANNEL_VALUE)) / 100);

	int i;
	time_t now;

	if (msg->channelsCnt < RC_CHANNELS_CNT) {
		fprintf(stderr, "rclog: rcbus supports insufficient number of channels\n");
		return;
	}

	gettime(&now, NULL);
	printf("%lli ", now / 1000);

	for (i = 0; i < RC_CHANNELS_CNT; i++) {
		printf("%d ", msg->channels[i]);
	}
	printf("\n");

	/* Abort: SWA/SWB/SWC/SWD == MAX */
	if (msg->channels[RC_SWA_CH] >= maxTriggerVal && msg->channels[RC_SWB_CH] >= maxTriggerVal
			&& msg->channels[RC_SWC_CH] >= maxTriggerVal && msg->channels[RC_SWD_CH] >= maxTriggerVal) {
		rclog_common.run = false;
	}
}


int main(int argc, char **argv)
{
	rclog_common.run = true;

	if (rcbus_init(PATH_DEV_RC_BUS, rc_typeIbus) < 0) {
		fprintf(stderr, "rc_log: cannot initialize rcbus using %s\n", PATH_DEV_RC_BUS);
		return EXIT_FAILURE;
	}

	if (rcbus_run(rclog_rcbusHandler, 500) < 0) {
		fprintf(stderr, "rc_log: cannot run rcbus\n");
		return EXIT_FAILURE;
	}

	while (rclog_common.run) {
		sleep(1);
	}

	rcbus_done();

	return EXIT_SUCCESS;
}

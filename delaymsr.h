#include "log.h"

#ifndef GPTP_DELAY_MSR_H
#define GPTP_DELAY_MSR_H

#define DM_STATE_INIT			0
#define DM_STATE_IDLE			1
#define DM_STATE_DELAY_RESP_WAIT	2
#define DM_STATE_DELAY_RESP_FLWUP_WAIT	3

void initDM(struct gPTPd* gPTPd);
void unintDM(struct gPTPd* gPTPd);
void dmHandleEvent(struct gPTPd* gPTPd, int evtId);

#ifdef DELAY_MSR_MODULE
static void dmHandleStateChange(struct gPTPd* gPTPd, int toState);
static void sendDelayReq(struct gPTPd* gPTPd);
static void sendDelayResp(struct gPTPd* gPTPd);
static void sendDelayRespFlwUp(struct gPTPd* gPTPd);
#endif

#endif

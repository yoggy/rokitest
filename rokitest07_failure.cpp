#include <iostream>
#include <roki/rk_fd.h>
#include <roki/rk_body.h>
#include <roki/rk_joint.h>
#include <roki/rk_motor.h>

#include <zeo/zeo.h>

extern "C" {
	rkFDCell* _rkFDCellPush(rkFD* fd, rkFDCell* lc);
}

#define DT 0.01
#define T  20

#define NUM 3

int main(int argc, char *argv[])
{
	char name_buf[256];

	rkFD fd;
	rkFDCreate(&fd);

	rkFDCell *lc;
	lc = zAlloc(rkFDCell, 1);

	rkChain  *chain = &(lc->data.chain);
	rkChainInit(chain);
	zNameSet(chain, "chain_name");

	zArrayAlloc(&(chain->link), rkLink, NUM);
	rkLink *rklink[NUM];
	rkJoint *joint[NUM];

	for (unsigned int i = 0; i < NUM; ++i) {
		rklink[i] = rkChainLink(chain, i);
		rkLinkInit(rklink[i]);

		snprintf(name_buf, 256, "link_name_%02d", i);
		zNameSet(rklink[i], name_buf);

		joint[i] = rkLinkJoint(rklink[i]);
		rkJointCreate(joint[i], RK_JOINT_FIXED);
	}

	rkChainSetMass(chain, 1.0); // dummy
	rkChainSetOffset(chain);
	rkChainUpdateFK(chain);
	rkChainUpdateID(chain);

	_rkFDCellPush(&fd, lc);

	// setup forward dynamics
	rkFDODE2Assign(&fd, Regular);
	rkFDODE2AssignRegular(&fd, RKG);
	rkFDSetDT(&fd, DT);

	rkFDSetSolver(&fd, Volume);
	rkFDUpdateInit(&fd);           // 追加したオブジェクトがすべてRK_JOINT_FIXEDだった場合、ここで落ちる

	// main loop
	while (rkFDTime(&fd) < T){
		rkFDUpdate(&fd);	
	}

	rkFDUpdateDestroy(&fd);
	rkFDDestroy(&fd);

	return 0;
}

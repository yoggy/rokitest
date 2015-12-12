#include <roki/rk_fd.h>
#include <roki/rk_body.h>
#include <roki/rk_joint.h>
#include <roki/rk_motor.h>

#include <zeo/zeo.h>

extern "C" {
	rkFDCell* _rkFDCellPush(rkFD* fd, rkFDCell* lc);
}

#define DT 0.01
#define T  0.02 

#define NUM 7

int main(int argc, char *argv[])
{
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

		// mass
		rkLinkSetMass(rklink[i], 1.0);
		zMat3DCreate(
				rkLinkInertia(rklink[i]),
				0.1,    0,    0,
				  0,  0.1,    0,
				  0,    0,  0.1);

		// frame 
		zVec3DCreate(rkLinkCOM(rklink[i]), 0, 0, 0);
		zVec3DCreate(rkLinkOrgPos(rklink[i]), 0, 0, 0);
		zMat3DCreate(
			rkLinkOrgAtt(rklink[i]),
			1,  0,  0,
			0,  1,  0,
			0,  0,  1);
		
		// joint
		joint[i] = rkLinkJoint(rklink[i]);
		switch(i) {
			case 0:
				rkJointCreate(joint[i], RK_JOINT_FIXED); // DOF=0, fixed
				break;
			case 1:
				rkJointCreate(joint[i], RK_JOINT_REVOL); // DOF=1, revolutional
				break;
			case 2:
				rkJointCreate(joint[i], RK_JOINT_PRISM); // DOF=1, prismatic
				break;
			case 3:
				rkJointCreate(joint[i], RK_JOINT_CYLIN); // DOF=2, cylindric
				break;
			case 4:
				rkJointCreate(joint[i], RK_JOINT_HOOKE); // DOF=2, universal, under construction error 
				break;
			case 5:
				rkJointCreate(joint[i], RK_JOINT_SPHER); // DOF=3, spherical, under construction error
				break;
			case 6:
				rkJointCreate(joint[i], RK_JOINT_FLOAT); // DOF=6, free-floating
				break;
		}
		if (i > 0) {
			rkLinkAddChild(rklink[i - 1], rklink[i]);
		}
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
	rkFDUpdateInit(&fd);

	// main loop
	while (rkFDTime(&fd) < T){
		rkFDUpdate(&fd);	

		for (unsigned int i = 0; i < NUM; ++i) {
			rkLinkPostureWrite(rklink[i]);
		}
	}

	rkFDUpdateDestroy(&fd);
	rkFDDestroy(&fd);

	return 0;
}

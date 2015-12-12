#include <iostream>
#include <roki/rk_fd.h>
#include <roki/rk_body.h>
#include <roki/rk_joint.h>
#include <roki/rk_motor.h>

#include <zeo/zeo.h>

#include <float.h>
#include <ncurses.h>
#include <unistd.h>

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

		// mass
		rkLinkSetMass(rklink[i], 1.0);
		zMat3DCreate(
				rkLinkInertia(rklink[i]),
				0.01,    0,    0,
				0,    0.01,    0,
				0,       0, 0.01);

		// frame 
		if (i == 0) {
			zVec3DCreate(rkLinkCOM(rklink[i]), 0, 0, 0);
			zVec3DCreate(rkLinkOrgPos(rklink[i]), 0, 0, 0);
			zMat3DCreate(
				rkLinkOrgAtt(rklink[i]),
				1,  0,  0,
				0,  0, -1,
				0,  1,  0);
		}
		else if (i == 1) {
			zVec3DCreate(rkLinkCOM(rklink[i]), 10, 0, 0);
			zVec3DCreate(rkLinkOrgPos(rklink[i]), 0, 0, 0);
			zMat3DCreate(
				rkLinkOrgAtt(rklink[i]),
				1,  0,  0,
				0,  1,  0,
				0,  0,  1);
		}
		else {
			zVec3DCreate(rkLinkCOM(rklink[i]), 10, 0, 0);
			zVec3DCreate(rkLinkOrgPos(rklink[i]), 10, 0, 0);
			zMat3DCreate(
				rkLinkOrgAtt(rklink[i]),
				1,  0,  0,
				0,  1,  0,
				0,  0,  1);
		}
		
		// joint
		joint[i] = rkLinkJoint(rklink[i]);
		if (i == 0) {
			rkJointCreate(joint[i], RK_JOINT_FIXED);
		}
		else {
			rkJointCreate(joint[i], RK_JOINT_REVOL); 
			rkLinkAddChild(rklink[i - 1], rklink[i]);

			// motor
			rkMotor* rkmotor;
			rkJointGetMotor(joint[1], &rkmotor);
			rkMotorCreate (rkmotor, RK_MOTOR_TRQ);
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

	// setup ncurses
	initscr();

	// main loop
	while (rkFDTime(&fd) < T){
		double e = 150;
		rkJointMotorSetInput(joint[1], &e);

		rkFDUpdate(&fd);	

		// draw
		erase();
		mvprintw(0, 0, "t=%f", rkFDTime(&fd)); 

		for (unsigned int i = 0; i < NUM; ++i) {
			//rkLinkPostureWrite(rklink[i]);
			zFrame3D *f   = rkLinkWldFrame(rklink[i]);
			//zVec3D *f_p   = &f->pos;
			//zMat3D *f_att = &f->att;

			zVec3D *p_com = rkLinkCOM(rklink[i]);
			zVec3D p_wld;
			zXfer3D(f, p_com, &p_wld);
			
			double x = zVec3DElem(&p_wld, 0);
			double y = zVec3DElem(&p_wld, 1);
			double z = zVec3DElem(&p_wld, 2);

			mvprintw(i, 15, "i=%d, pos=(%f, %f, %f)", i, x, y, z); 

			mvprintw((int)(5-z), x + 30, "*");  // cube
			
		}

		refresh();
		usleep((int)(DT * 1000 * 1000));
	}

	rkFDUpdateDestroy(&fd);
	rkFDDestroy(&fd);

	return 0;
}

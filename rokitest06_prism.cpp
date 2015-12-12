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

#define NUM 2

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
		zVec3DCreate(rkLinkCOM(rklink[i]), 0, 0, 0);
		zVec3DCreate(rkLinkOrgPos(rklink[i]), 0, 0, 0);
		if (i == 0) {
			zMat3DCreate(
				rkLinkOrgAtt(rklink[i]),
				0.7071, 0,  -0.7071,
				0,      1,  0,
				0.7071, 0,  0.7071);
		}
		else {
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

			// roki/rk_joint_prism.h
			//     typedef struct{
			//       double dis, vel, acc, trq;     // joint displacement, velocity, acceleration and torque
			//       double min, max;               // limiter
			//       double stiff, viscos, coulomb; // joint stiffness, viscosity and coulomb friction
			//       double tf;                     // friction
			//       double sf;                     // static friction
			//       rkMotor m;                     // motor
			//       rkJointRef _ref;               // for forward dynamics
			//     } rkJointPrpPrism;

			rkJointCreate(joint[i], RK_JOINT_PRISM); 
			((rkJointPrpPrism*)(joint[i]->prp))->max = 10;
			((rkJointPrpPrism*)(joint[i]->prp))->min =-10;
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

	// setup ncurses
	initscr();

	// main loop
	while (rkFDTime(&fd) < T){
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

			mvprintw((int)(5-z), x + 20, "*");  // cube
			
		}

		refresh();
		usleep((int)(DT * 1000 * 1000));
	}

	rkFDUpdateDestroy(&fd);
	rkFDDestroy(&fd);

	return 0;
}

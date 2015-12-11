
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
#define T  10

#define NUM 3

int main(int argc, char *argv[])
{
	char name_buf[256];

	rkFD fd;
	rkFDCreate(&fd);

	rkFDCell *lc[NUM];
	rkChain  *chain[NUM];
	rkLink   *rklink[NUM];
	rkJoint  *joint[NUM];

	// boxs
	for (unsigned int i = 0; i < NUM; ++i) {
		// cell
		lc[i] = zAlloc(rkFDCell, 1);

		// chain
		chain[i] = &(lc[i]->data.chain);
		rkChainInit(chain[i]);
		snprintf(name_buf, 256, "chain_name_%02d", i);
		zNameSet(chain[i], name_buf);

		zArrayAlloc(&(chain[i]->link), rkLink, 1);

		// link
		rklink[i] = rkChainLink(chain[i], 0);
		rkLinkInit(rklink[i]);
		snprintf(name_buf, 256, "link_name_%02d", i);
		zNameSet(rklink[i], name_buf);

		zVec3DCreate(rkLinkOrgPos(rklink[i]), i * 5, 0, 0);
		zMat3DCreate(
			rkLinkOrgAtt(rklink[i]),
			1,  0,  0,
			0,  1,  0,
			0,  0,  1);

		// mass
		rkLinkSetMass(rklink[i], 1.0);
		zVec3DCreate(rkLinkCOM(rklink[i]), 0, 0, 0);
		zMat3DCreate(
				rkLinkInertia(rklink[i]),
				0.01,    0,    0,
				0,    0.01,    0,
				0,       0, 0.01);
		
		// joint
		joint[i] = rkLinkJoint(rklink[i]);
		rkJointCreate(joint[i], RK_JOINT_FLOAT);

		// shape
		zShape3D* shape = zAlloc(zShape3D, 1);
		zShape3DInit(shape);
		zVec3D zp = {{ 0, 0, 0 }};
		zVec3D ax = {{ 1.0, 0.0, 0.0 }};
		zVec3D ay = {{ 0.0, 1.0, 0.0 }};
		zVec3D az = {{ 0.0, 0.0, 1.0 }};
		zShape3DCreateBox(shape, &zp, &ax, &ay, &az, 2.0, 2.0, 2.0);
		zBox3DInit(zShape3DBB(shape));
		rkLinkShapePush(rklink[i], shape);	

		rkChainSetMass(chain[i], 1.0); // dummy
		rkChainSetOffset(chain[i]);    // update link->offset value
		rkChainUpdateFK(chain[i]);
		rkChainUpdateID(chain[i]);

		_rkFDCellPush(&fd, lc[i]);
	
		// you must set joint displacements after _rkFDCellPush() calling....
		zVec dis = zVecAlloc(rkChainJointSize(chain[i]));
		zVecSetElem(dis, 0, i * 5);
		zVecSetElem(dis, 1, 0);
		zVecSetElem(dis, 2, 10);
		zVecSetElem(dis, 3, 0);
		zVecSetElem(dis, 4, 0);
		zVecSetElem(dis, 5, 0);
		rkFDChainSetDis(lc[i], dis);
		zVecFree(dis);
	}

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
			zVec dis;
			dis = zVecAlloc(rkChainJointSize(chain[i]));
			rkChainGetJointDisAll(chain[i], dis);
			double x = zVecElem(dis, 0);
			double y = zVecElem(dis, 1);
			double z = zVecElem(dis, 2);
	
			mvprintw(i, 15, "i=%d, pos=(%f, %f, %f)", i, x, y, z); 

			mvprintw((int)(11-z), x + 10, "*");  // cube
		}
		
		refresh();
		usleep((int)(DT * 1000 * 1000));
	}

	// teardown ncurses
	endwin();

	rkFDUpdateDestroy(&fd);
	rkFDDestroy(&fd);

	return 0;
}

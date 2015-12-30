
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

int main(int argc, char *argv[])
{
	rkFD fd;
	rkFDCreate(&fd);

	rkFDCell *lc;
	rkChain  *chain;
	rkLink   *rklink;
	rkJoint  *joint;


	// boxs
	lc = zAlloc(rkFDCell, 1);

	chain = &(lc->data.chain);
	rkChainInit(chain);

	// link
	zArrayAlloc(&(chain->link), rkLink, 1);
	rklink = rkChainLink(chain, 0);
	rkLinkInit(rklink);

	zVec3DCreate(rkLinkOrgPos(rklink), 0, 0, 0);
	zMat3DCreate(
			rkLinkOrgAtt(rklink),
			1,  0,  0,
			0,  1,  0,
			0,  0,  1);

	// mass
	rkLinkSetMass(rklink, 1.0);
	zVec3DCreate(rkLinkCOM(rklink), 0, 0, 0);
	zMat3DCreate(
			rkLinkInertia(rklink),
			0.1,    0,    0,
			0,    0.1,    0,
			0,       0, 0.1);
		
	// joint
	joint = rkLinkJoint(rklink);
	rkJointCreate(joint, RK_JOINT_FLOAT);

	// shape
	zShape3D* shape = zAlloc(zShape3D, 1);
	zShape3DInit(shape);
	zVec3D zp = {{ 0, 0, 0 }};
	zShape3DCreateSphere(shape, &zp, 1, 0);
	zBox3DInit(zShape3DBB(shape));

	rkChainSetMass(chain, 1.0); // dummy
	rkChainSetOffset(chain);    // update link->offset value
	rkChainUpdateFK(chain);
	rkChainUpdateID(chain);

	_rkFDCellPush(&fd, lc);
	
	// setup forward dynamics
	rkFDODE2Assign(&fd, Regular);
	rkFDODE2AssignRegular(&fd, RKG);

	rkFDSetDT(&fd, DT);

	rkFDSetSolver(&fd, Volume);
	rkFDUpdateInit(&fd);

	// src/rk_fd.c:
	//     void rkFDChainSetDis(rkFDCell *lc, zVec dis)
	//     { 
	//       zVecCopy( dis, &lc->data._dis );
	//       rkChainSetJointDisAll( &lc->data.chain, dis );
	//     }

	// you must set joint displacements after _rkFDCellPush() calling....
	zVec vel = zVecAlloc(rkChainJointSize(chain));
	zVecSetElem(vel, 0, 20);
	zVecSetElem(vel, 1, 0);
	zVecSetElem(vel, 2, 20);
	zVecSetElem(vel, 3, 0);
	zVecSetElem(vel, 4, 0);
	zVecSetElem(vel, 5, 0);
	zVecCopy(vel, &lc->data._vel);
	rkChainSetJointVelAll(chain, vel);
	zVecFree(vel);

	// setup ncurses
	initscr();

	// main loop
	while (rkFDTime(&fd) < T){
		rkFDUpdate(&fd);	

		// draw
		erase();

		mvprintw(0, 0, "t=%f", rkFDTime(&fd)); 

		zFrame3D *f   = rkLinkWldFrame(rklink);
		zVec3D   *f_p = &f->pos;

		double x = zVec3DElem(f_p, 0);
		double y = zVec3DElem(f_p, 1);
		double z = zVec3DElem(f_p, 2);
		
		mvprintw(0, 15, "pos=(%f, %f, %f)", x, y, z); 

		mvprintw((int)(20-z), x + 5, "*");  // sphere
		
		mvprintw(20, 0, "=========");  //plane
		refresh();
		usleep((int)(DT * 1000 * 1000));
	}

	// teardown ncurses
	endwin();

	rkFDUpdateDestroy(&fd);
	rkFDDestroy(&fd);

	return 0;
}

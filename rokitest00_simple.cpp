#include <roki/rk_fd.h>
#include <roki/rk_joint.h>

#include <zeo/zeo.h>

#include <float.h>

extern "C" {
	rkFDCell* _rkFDCellPush(rkFD* fd, rkFDCell* lc);
}

#define DT 0.1
#define T  5

int main(int argc, char *argv[])
{
	rkFD fd;
	rkFDCreate(&fd);

	rkFDCell *lc;
	rkChain  *chain;
	rkLink   *rklink;
	rkJoint  *joint;

	// rkFDCell
	lc = zAlloc(rkFDCell, 1);

	// rkChain
	chain = &(lc->data.chain);
	rkChainInit(chain);
	zArrayAlloc(&(chain->link), rkLink, 1);

	// link
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
			0.01,    0,    0,
			0,    0.01,    0,
			0,       0, 0.01);
	
	// joint
	joint = rkLinkJoint(rklink);
	rkJointCreate(joint, RK_JOINT_FLOAT);


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

	// main loop
	while (rkFDTime(&fd) < T){
		rkFDUpdate(&fd);	

		// フレームはlinkの中心位置、rkLinkCOMはフレーム内ローカルな重心位置
		zFrame3D *f   = rkLinkWldFrame(rklink);
		zVec3D *p_com = rkLinkCOM(rklink);

		zVec3D p_wld;
		zXfer3D(f, p_com, &p_wld);

		double x = zVec3DElem(&p_wld, 0);
		double y = zVec3DElem(&p_wld, 1);
		double z = zVec3DElem(&p_wld, 2);

		printf("t=%.3f, pos=(%f, %f, %f)\n", rkFDTime(&fd), x, y, z);
	}

	rkFDUpdateDestroy(&fd);
	rkFDDestroy(&fd);

	return 0;
}

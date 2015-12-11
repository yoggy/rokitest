
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

#define BOX_NUM 3

int main(int argc, char *argv[])
{
	char name_buf[256];

	rkFD fd;
	rkFDCreate(&fd);

	rkFDCell *lc[BOX_NUM + 1];
	rkChain  *chain[BOX_NUM + 1];
	rkLink   *rklink[BOX_NUM + 1];
	rkJoint  *joint[BOX_NUM + 1];

	// contact info
	zArrayAlloc(&(fd.ci), rkContactInfo, 2);

	// rkContactInfoRigidCreate(rkContactInfo *ci, double k, double l, double sf, double kf, char *stf1, char *stf2)
	// k:compensation, l:relaxation, sf:static friction, kf;kinetic friction
	rkContactInfo* ci;
	ci = zArrayElem(&fd.ci, 0);
	rkContactInfoRigidCreate(ci, 500, 1, 1.0, 0.2, (char*)"box", (char*)"plane");

	ci = zArrayElem(&fd.ci, 1);
	rkContactInfoRigidCreate(ci, 500, 1, 1.0, 0.2, (char*)"box", (char *)"box");


	// boxs
	for (unsigned int i = 0; i < BOX_NUM; ++i) {
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

		rkLinkSetStuff(rklink[i], "box"); // for collision

		zVec3DCreate(rkLinkOrgPos(rklink[i]), 0, 0, 0);
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
			0.1,    0,    0,
			0,    0.1,    0,
			0,       0, 0.1);
		
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
		zShape3DCreateBox(shape, &zp, &ax, &ay, &az, 3.0, 3.0, 3.0);
		zBox3DInit(zShape3DBB(shape));
		rkLinkShapePush(rklink[i], shape);	

		rkChainSetMass(chain[i], 1.0); // dummy
		rkChainSetOffset(chain[i]);    // update link->offset value
		rkChainUpdateFK(chain[i]);
		rkChainUpdateID(chain[i]);

		_rkFDCellPush(&fd, lc[i]);
	
		// you must set joint displacements after _rkFDCellPush() calling....
		zVec dis = zVecAlloc(rkChainJointSize(chain[i]));
		zVecSetElem(dis, 0, i * 2);
		zVecSetElem(dis, 1, 0);
		zVecSetElem(dis, 2, 10 + 4 * i);
		zVecSetElem(dis, 3, 0);
		zVecSetElem(dis, 4, 0);
		zVecSetElem(dis, 5, 0);
		rkFDChainSetDis(lc[i], dis);
		zVecFree(dis);
	}

	// plane (grand)
	{
		unsigned int i = BOX_NUM;

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

		rkLinkSetStuff(rklink[i], "plane"); // for collision

		zVec3DCreate(rkLinkOrgPos(rklink[i]), 0, 0, 0);
		zMat3DCreate(
			rkLinkOrgAtt(rklink[i]),
			1,  0,  0,
			0,  1,  0,
			0,  0,  1);

		// joint
		joint[i] = rkLinkJoint(rklink[i]);
		rkJointCreate(joint[i], RK_JOINT_FIXED);

		zShape3D *shape = zAlloc(zShape3D, 1);
		zShape3DInit(shape);
		zShape3DType(shape) = ZSHAPE_PH; // polyhedron

		shape->com = &zprim_ph3d_com;    // function valiable

		zPH3D* ph = (zPH3D*)&shape->body;
		zPH3DInit(ph); 
		zPH3DAlloc(ph, 4, 2);

		zVec3D* v;
		v = zPH3DVert(ph, 0);
		zVec3DSetElem( v, zX, -100.0);
		zVec3DSetElem( v, zY, -100.0);
		zVec3DSetElem( v, zZ,   0.0);

		v = zPH3DVert(ph, 1);
		zVec3DSetElem( v, zX,  100.0);
		zVec3DSetElem( v, zY, -100.0);
		zVec3DSetElem( v, zZ,   0.0);

		v = zPH3DVert(ph, 2);
		zVec3DSetElem( v, zX,  100.0);
		zVec3DSetElem( v, zY,  100.0);
		zVec3DSetElem( v, zZ,   0.0);

		v = zPH3DVert(ph, 3);
		zVec3DSetElem( v, zX, -100.0);
		zVec3DSetElem( v, zY,  100.0);
		zVec3DSetElem( v, zZ,   0.0);

		zTri3DCreate(zPH3DFace(ph, 0), zPH3DVert(ph, 0), zPH3DVert(ph, 1), zPH3DVert(ph, 2));
		zTri3DCreate(zPH3DFace(ph, 1), zPH3DVert(ph, 2), zPH3DVert(ph, 3), zPH3DVert(ph, 0));

		zAABox3D aabb;
		zAABB(&aabb, zShape3DVertBuf(shape), zShape3DVertNum(shape), NULL);
		zAABox3DToBox3D(&aabb, zShape3DBB(shape));

		rkLinkShapePush(rklink[i], shape); 

		rkChainSetMass(chain[i], 1.0); // dummy
		rkChainSetOffset(chain[i]);    // update link->offset value
		rkChainUpdateFK(chain[i]);
		rkChainUpdateID(chain[i]);

		_rkFDCellPush(&fd, lc[i]);
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

		for (unsigned int i = 0; i < BOX_NUM; ++i) {
			zVec dis;
			dis = zVecAlloc(rkChainJointSize(chain[i]));
			rkChainGetJointDisAll(chain[i], dis);
			double x = zVecElem(dis, 0);
			double y = zVecElem(dis, 1);
			double z = zVecElem(dis, 2);
	
			mvprintw(i, 15, "i=%d, pos=(%f, %f, %f)", i, x, y, z); 

			mvprintw((int)(20-z), x + 15, "*");  // cube
		}
		
		mvprintw(20, 0, "===========================================================");  //plane
		refresh();
		usleep((int)(DT * 1000 * 1000));
	}

	// teardown ncurses
	endwin();

	rkFDUpdateDestroy(&fd);
	rkFDDestroy(&fd);

	return 0;
}

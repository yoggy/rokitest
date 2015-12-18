#include <roki/rk_fd.h>
#include <roki/rk_body.h>
#include <roki/rk_joint.h>
#include <roki/rk_motor.h>

#include <zeo/zeo.h>

#include <float.h>
#include <ncurses.h>
#include <unistd.h>

#include <sstream>
#include <vector>

extern "C" {
	rkFDCell* _rkFDCellPush(rkFD* fd, rkFDCell* lc);
}

#define DT 0.01
#define T  10 

#define OBJECT_LIFE_SEC 3 

class TestObject {
public:
	TestObject(rkFD *fd, const char *name) : t(0) {
		this->name = name;

		// cell
		lc = zAlloc(rkFDCell, 1);

		// chain
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

		rkChainSetMass(chain, 1.0);
		rkChainSetOffset(chain);
		rkChainUpdateFK(chain);
		rkChainUpdateID(chain);
	}

	void set_pos(const double &x, const double &y, const double &z) {
		// you must set joint displacements after _rkFDCellPush() calling....
		zVec dis = zVecAlloc(rkChainJointSize(chain));
		zVecSetElem(dis, 0, x);
		zVecSetElem(dis, 1, y);
		zVecSetElem(dis, 2, z);
		zVecSetElem(dis, 3, 0);
		zVecSetElem(dis, 4, 0);
		zVecSetElem(dis, 5, 0);
		rkFDChainSetDis(lc, dis);
		zVecFree(dis);
	}

	double get_pos(const int &idx) const {
		zVec dis;
		dis = zVecAlloc(rkChainJointSize(chain));
		rkChainGetJointDisAll(chain, dis);
		double val = zVecElem(dis, idx);
		zVecFree(dis);
		return val;
	}

	double x() const {
		return get_pos(0);
	}

	double y() const {
		return get_pos(1);
	}

	double z() const {
		return get_pos(2);
	}
public:
	rkFDCell *lc;
	rkChain  *chain;
	rkLink   *rklink;
	rkJoint  *joint;

	std::string name;
	double t;
};

int main(int argc, char *argv[])
{
	rkFD fd;
	rkFDCreate(&fd);
	TestObject *dummy;
	std::vector<TestObject*> objs;

	// roki initialize
	dummy = new TestObject(&fd, "dummy");
	_rkFDCellPush(&fd, dummy->lc);

	rkFDODE2Assign(&fd, Regular);
	rkFDODE2AssignRegular(&fd, RKG);

	rkFDSetDT(&fd, DT);

	rkFDSetSolver(&fd, Volume);
	rkFDUpdateInit(&fd);


	// setup ncurses
	initscr();


	// main loop
	unsigned int append_count = 0;
	while (rkFDTime(&fd) < T){
		// append 
		if ((unsigned int)(rkFDTime(&fd)) > append_count) {
			std::stringstream ss;
			ss << "obj" << append_count;
			TestObject *obj = new TestObject(&fd,ss.str().c_str());
			_rkFDCellPush(&fd, obj->lc);
			obj->set_pos(append_count * 5, 0, 10);

			objs.push_back(obj); 
			append_count ++;

			rkFDUpdateInit(&fd);
		}

		// remove
		std::vector<TestObject*>::iterator it = objs.begin();
		while(it != objs.end()) {
			if ((*it)->t > OBJECT_LIFE_SEC) {
				delete *it;          // delete TestObject instance
				it = objs.erase(it); // erase pointer from vector
			}
			else {
				++it;
			}
		}

		// update forward dynamics
		rkFDUpdate(&fd);	
		for (unsigned int i = 0; i < objs.size(); ++i) {
			objs[i]->t += DT;
		}

		// draw
		erase();

		mvprintw(0, 0, "t=%f", rkFDTime(&fd)); 

		for (unsigned int i = 0; i < objs.size(); ++i) {
			double x = objs[i]->x();
			double y = objs[i]->y();
			double z = objs[i]->z();

			mvprintw(i, 15, "name=%s, pos=(%f, %f, %f)", objs[i]->name.c_str(), x, y, z);
			mvprintw((int)(11-z), x + 10, "*");  // cube
		}
		
		refresh();
		
		//refresh();
		usleep((int)(DT * 1000 * 1000));
	}

	// teardown ncurses
	endwin();

	rkFDUpdateDestroy(&fd);
	rkFDDestroy(&fd);

	for (unsigned int i = 0; i < objs.size(); ++i) {
		delete objs[i];
		objs[i] = NULL;
	}
	objs.clear();

	return 0;
}

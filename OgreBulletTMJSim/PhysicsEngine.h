#pragma once

#include <btBulletDynamicsCommon.h>
#include <btBulletCollisionCommon.h>
#include <BulletSoftBody\btSoftRigidDynamicsWorld.h>
#include <string>
#include "FiducialMark.h"
#include <BulletSoftBody\btSoftBodyHelpers.h>
#include "MuscleControl.h"
#include "LigamentControl.h"
#include "DiscControl.h"
#include "MassControl.h"
#include "SoftBodyControl.h"
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <thread>
#include <boost/thread/thread.hpp>
#include <boost\archive\text_oarchive.hpp>
#include <boost\archive\text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <boost\asio.hpp>
#include <boost\filesystem.hpp>

const btVector3 btStartPosition = btVector3(2.91061, -58.43966, 19.00534);

struct appliedForce
{
	std::string label;
	btScalar lenght = 0;
	btVector3 totalForce;
};

struct simulationParams
{
	float timeStep;
	int maxSubSteps;
	float fixedTimeStep;
	btScalar softBodyFactor;
	int solvers;
	bool loadDiscs;
	bool useTetras;
	bool useSoftLigaments;
	bool useHDdiscs;
	bool useGravity;
};

struct softBodyConfig
{
	int index;
	float mass;
	bool pose;
	btSoftBody::Config cfg;
};

namespace boost {
	namespace serialization {

		template<class Archive>
		void serialize(Archive & ar, simulationParams & s, const unsigned int version)
		{
			ar & s.timeStep;
			ar & s.maxSubSteps;
			ar & s.fixedTimeStep;
			ar & s.solvers;
			ar & s.loadDiscs;
			ar & s.useTetras;
			ar & s.useSoftLigaments;
			ar & s.useHDdiscs;
			ar & s.useGravity;
			ar & s.softBodyFactor;
		}

		template<class Archive>
		void serialize(Archive & ar, btSoftBody::Config & c, const unsigned int version)
		{
			ar & c.kVCF;
			ar & c.kDP;
			ar & c.kDG;
			ar & c.kLF;
			ar & c.kPR;
			ar & c.kVC;
			ar & c.kDF;
			ar & c.kMT;
			ar & c.kCHR;
			ar & c.kKHR;
			ar & c.kSHR;
			ar & c.kAHR;
			ar & c.kSRHR_CL;
			ar & c.kSKHR_CL;
			ar & c.kSSHR_CL;
		}

		template<class Archive>
		void serialize(Archive & ar, softBodyConfig & c, const unsigned int version)
		{
			ar & c.index;
			ar & c.mass;
			ar & c.pose;
			ar & c.cfg;
		}

		//template<class Archive>
		//void serialize(Archive & ar, std::vector<softBodyConfig> & cfg, const unsigned int version)
		//{
		//	//ar & cfg;
		//	ar & BOOST_SERIALIZATION_NVP(cfg);
		//}

	} // namespace serialization
} // namespace boost

struct Ligament
{
	Ligament()
	{
	}

	~Ligament()
	{
		std::cout << "Calling ligament destructor\n";
		mDebugDraw = 0;
	}

	int id; // Group
	int id2; // ID
	std::string label;
	btAlignedObjectArray<btTypedConstraint*> constraints;
	btTypedConstraint* c0;
	btTypedConstraint* c1;
	btIDebugDraw* mDebugDraw;
	btSoftBody* sbody;
};

struct ligamentConfig
{
	int group;
	int id;
	int solvers;
	float cfm;
	float erp;
};

struct cNode 
{

	btSoftBody::Node node;
	int index;
	float distance;
};

struct muscleMotor
{
	enum delayFlag
	{
		NO_SYNC = 0,
		PRE_SYNC,
		POST_SYNC,
		HOLD_POSITION
	};


	std::string desc;
	int group;
	int id;
	float minForce;
	float maxForce;
	float stepForce; 
	int timeStep; // ms
	int timeInRest; // ms
	int timeInActive; // ms
	int delayTime; //ms
	int delayFlag;
	bool active;
	bool addForce;
	//int numExec = 0;
	int maxExec = 0;
};

struct genericMuscle
{
	float multiply;
	float maxMultiply;
	float step;
	muscleMotor motor;
	btAlignedObjectArray<muscleMotor> motor2;
	bool motorsActive;
};

struct sbMuscle : public genericMuscle
{
	sbMuscle() : softbody(0) {}

	int group;
	int id;
	bool perNode = false;
	//float multiply;
	std::string label;
	btSoftBody* softbody;
	btAlignedObjectArray<btSoftBody::Node*> insertions;
	btAlignedObjectArray<btVector3> origins;
	btIDebugDraw* mDebugDraw;
	
};

struct rbMuscle 
{

	rbMuscle() : body0(0) {}

	int group;
	int id;
	std::string label;
	btRigidBody* body0;
	btVector3 orgin;
	btVector3 insertion;
	btVector3 relInsertion;
	float multiply;
	float maxMultiply;
	float step;
	muscleMotor motor;
	btAlignedObjectArray<muscleMotor> motor2;
	bool motorsActive;
	bool draw;

	btIDebugDraw* mDebugDraw;

	void drawForce()
	{

	}
};

struct RigidBodyDrawer
{
	int index;
	bool draw;
	btVector3 color;
	btRigidBody* body;
};

struct controlPoint
{
	std::string label;
	btVector3 point;
	float x;
	float y;
	float z;
	
	std::string getLogMessage()
	{
		std::string mlog = label + "," + std::to_string(x) + "," +
			std::to_string(y) + "," +
			std::to_string(z) + "\n";
		return mlog;
	}

};

struct simulationLog
{
	bool recording;
	btRigidBody* body;
	btAlignedObjectArray<controlPoint> pointsInLocal;
	std::string filename;
	int timeStep; // ms

	btAlignedObjectArray<controlPoint> getPointsInWorld()
	{
		btAlignedObjectArray<controlPoint> pointsInWorld;
		for (int i = 0; i < pointsInLocal.size(); i++)
		{
			controlPoint cp;
			cp.label = pointsInLocal[i].label;
			cp.point = body->getWorldTransform() * pointsInLocal[i].point;
			cp.x = cp.point.getX();
			cp.y = cp.point.getY();
			cp.z = cp.point.getZ();
			pointsInWorld.push_back(cp);
		}
		return pointsInWorld;
	}

	std::string getTransformationMatrix()
	{
		//btScalar* m = 0;
		//body->getWorldTransform().getOpenGLMatrix(m);
		btMatrix3x3 m = body->getWorldTransform().getBasis();
		btVector3 pos = body->getWorldTransform().getOrigin();
		btVector3 pos0 = pos;
		//pos = btStartPosition - pos;
		btVector3 pos1 = pos0 + btStartPosition;
		btVector3 pos2 = body->getWorldTransform() * btStartPosition;
		btMatrix3x3 swap(1, 0, 0, 
						 0, 0, 1, 
						 0, 1, 0);
		//m = m * swap;
		btScalar yaw, pitch, roll;
		m.getEulerYPR(yaw, pitch, roll);

		/*std::string matrix = "\n(" + std::to_string(m.getRow(0).getX()) + ", " + std::to_string(m.getRow(0).getY()) + ", " + std::to_string(m.getRow(0).getZ()) + ", " + std::to_string(pos.getX()) + ",\n" +
			" " + std::to_string(m.getRow(1).getX()) + ", " + std::to_string(m.getRow(1).getY()) + ", " + std::to_string(m.getRow(1).getZ()) + ", " + std::to_string(pos.getY()) + ",\n" +
			" " + std::to_string(m.getRow(2).getX()) + ", " + std::to_string(m.getRow(2).getY()) + ", " + std::to_string(m.getRow(3).getZ()) + ", " + std::to_string(pos.getZ()) + ",\n" +
			" " + std::to_string(0) + ", " + std::to_string(0) + ", " + std::to_string(0) + ", " + std::to_string(1) + ")\n" +
			"\nRotations, Yaw: " + std::to_string(yaw) + ", Pitch: " + std::to_string(pitch) + ", Roll: " + std::to_string(roll) + "\n" +
			"Posicao 0 Sem modificacao: " + std::to_string(pos0.getX()) + ", " + std::to_string(pos0.getY()) + ", " + std::to_string(pos0.getZ()) + "\n" +
			"Posicao com subtracao: " + std::to_string(pos.getX()) + ", " + std::to_string(pos.getY()) + ", " + std::to_string(pos.getZ()) + "\n" +
			"Posicao 1 com adicaoo: " + std::to_string(pos1.getX()) + ", " + std::to_string(pos1.getY()) + ", " + std::to_string(pos1.getZ()) + "\n" +
			"Posicao 2 com transformacao em relacao ao ponto de origem: " + std::to_string(pos2.getX()) + ", " + std::to_string(pos2.getY()) + ", " + std::to_string(pos2.getZ()) + "\n";*/

		/*std::string matrix = "\n" + std::to_string(m.getRow(0).getX()) + " " + std::to_string(m.getRow(0).getY()) + " " + std::to_string(m.getRow(0).getZ()) + " " + std::to_string(pos.getX()) + "\n" +
			std::to_string(m.getRow(1).getX()) + " " + std::to_string(m.getRow(1).getY()) + " " + std::to_string(m.getRow(1).getZ()) + " " + std::to_string(pos.getY()) + "\n" +
			std::to_string(m.getRow(2).getX()) + " " + std::to_string(m.getRow(2).getY()) + " " + std::to_string(m.getRow(2).getZ()) + " " + std::to_string(pos.getZ()) + "\n" +
			std::to_string(0) + " " + std::to_string(0) + " " + std::to_string(0) + " " + std::to_string(1) + "\n" +
			"\nRotations, Yaw: " + std::to_string(yaw) + ", Pitch: " + std::to_string(pitch) + ", Roll: " + std::to_string(roll) + "\n";
		return matrix;*/

		/*std::string matrix = "\n" + std::to_string(m.getRow(0).getX()) + " " + std::to_string(m.getRow(0).getY()) + " " + std::to_string(m.getRow(0).getZ()) + " " + std::to_string(pos.getX()) + "\n" +
			std::to_string(m.getRow(1).getX()) + " " + std::to_string(m.getRow(1).getY()) + " " + std::to_string(m.getRow(2).getY()) + " " + std::to_string(pos.getZ()) + "\n" +
			std::to_string(m.getRow(2).getX()) + " " + std::to_string(m.getRow(1).getZ()) + " " + std::to_string(m.getRow(2).getZ()) + " " + std::to_string(pos.getY()) + "\n" +
			std::to_string(0) + " " + std::to_string(0) + " " + std::to_string(0) + " " + std::to_string(1) + "\n" +
			"\nRotations, Yaw: " + std::to_string(yaw) + ", Pitch: " + std::to_string(pitch) + ", Roll: " + std::to_string(roll) + "\n";
		return matrix;*/

		std::string matrix = "\n" + std::to_string(m.getRow(0).getX()) + " " + std::to_string(m.getRow(2).getX()) + " " + std::to_string(m.getRow(0).getY()) + " " + std::to_string(pos.getX()) + "\n" +
			std::to_string(m.getRow(0).getZ()) + " " + std::to_string(m.getRow(2).getZ()) + " " + std::to_string(m.getRow(2).getY()) + " " + std::to_string(pos.getZ()) + "\n" +
			std::to_string(m.getRow(1).getX()) + " " + std::to_string(m.getRow(1).getZ()) + " " + std::to_string(m.getRow(1).getY()) + " " + std::to_string(pos.getY()) + "\n" +
			std::to_string(0) + " " + std::to_string(0) + " " + std::to_string(0) + " " + std::to_string(1) + "\n" +
			"\nRotations, Yaw: " + std::to_string(yaw) + ", Pitch: " + std::to_string(pitch) + ", Roll: " + std::to_string(roll) + "\n";
		return matrix;

	}

};



#define BIT(x) (1<<(x))
enum collisiontypes {
	COL_NOTHING = 0, //<Collide with nothing
	COL_MANDIBLE = BIT(0), //<Collide with ships
	COL_SKULL = BIT(1), //<Collide with walls
	COL_DISC = BIT(2) //<Collide with powerups
};



class PhysicsEngine
{
private:
	btDefaultCollisionConfiguration* mCollisionConfiguration;
	btCollisionDispatcher* mDispatcher;
	btCollisionDispatcher* mDispatcherMt; // for Multithread
	btBroadphaseInterface* mBroadohase;
	btSequentialImpulseConstraintSolver* mSolver;
	btConstraintSolver* mSolverMt; // for Multithread
	btDynamicsWorld* mDynamicsWorld;
	btSoftBodySolver* mSoftBodySolver;
	btSoftBodyWorldInfo mSoftBodyWorldInfo;
	btAlignedObjectArray<btCollisionShape*> mCollisionShapes;
	btAlignedObjectArray<btRigidBody*> mRigidBodies;
	btAlignedObjectArray<RigidBodyDrawer*> drawRigidBodies;
	btAlignedObjectArray<btSoftBody*> mSoftBodies;
	bool displayDebug;
	bool runPhysics;
	bool displaySoftBodyClusters;
	bool drawForces;
	bool displaySoftBodies;
	int drawSoftBodyFlags = /*fDrawFlags::Faces + fDrawFlags::Links + fDrawFlags::Normals + fDrawFlags::Contacts + fDrawFlags::Tetras;*/ fDrawFlags::Std;
	MuscleControl * mMuscleControl;
	LigamentControl * mLigamentControl;
	DiscControl *mDiscControl;
	MassControl* mMassControl;
	SoftBodyControl* mSoftBodyControl;
	int m_numThreads;
	

public:
	PhysicsEngine();
	virtual ~PhysicsEngine();
	//simulationLog* mSimLog;

	static btVector3 COLOR_RED;
	static btVector3 COLOR_GREEN;
	static btVector3 COLOR_BLUE;
	static btVector3 COLOR_YELLOW;
	static btVector3 COLOR_CYAN;
	static btVector3 COLOR_PINK;
	static btVector3 COLOR_BLACK;
	static btVector3 COLOR_WHITE;

	static float DEFAULT_MANDIBLE_MASS;
	static float DEFAULT_LIGAMENT_MASS;
	static float DEFAULT_MUSCLE_MASS;
	static float DEFAULT_DISC_MASS;


	// Collision Flags
	int mandibleCollidesWith = COL_SKULL | COL_DISC;
	int skullCollidesWith = COL_MANDIBLE | COL_DISC;
	int ligamentCollidesWith = COL_NOTHING;
	int muscleCollidesWith = COL_NOTHING;
	int discCollidesWith = COL_SKULL | COL_MANDIBLE;

	// Muscle Control
	

	void createSoftBodyWorld(btVector3 gravity);
	void deleteWorld();
	void createDefaultGround();
	btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, btMotionState *motionState, short group, short mask, bool addColShape = true);
	btRigidBody* createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, short group = -1, short mask = -1, bool addColShape = true);
	btRigidBody* createRigidBody(float mass, btVector3 origin, btCollisionShape* shape, short group = -1, short mask = -1, bool addColShape = true);
	btRigidBody* createRigidBodyFromObjFile(std::string filename, btMotionState * motionState, float mass, short group = -1, short mask = -1);
	btSoftBody* createFromTetGen(std::string ele, std::string face, std::string node, float mass = DEFAULT_DISC_MASS, short group = -1, short mask = -1);
	btSoftBody* createSoftBodyFromObjFile(std::string filename, float mass = DEFAULT_DISC_MASS);
	btSoftBody* createRetrodiscalTissue(std::string filename, btSoftBody* disc, int width, int height = 0, bool bendingConstrainst = true, float distance = 5.0f, int maxLinks = 3);
	btSoftBody* createAttachTissue(std::string filename, btSoftBody* disc, btRigidBody* body, int numVertices, float distance = 5.0f, int maxLinks = 5);
	btCompoundShape* loadCompoundShapeFromObjFile(std::string filename);
	btTypedConstraint* createP2PConstraint(btRigidBody* bodyA, btRigidBody* bodyB, btVector3 locationA, btVector3 locationB);
	std::vector<FiducialMark> PhysicsEngine::loadFiducial(std::string filename);
	btAlignedObjectArray<Ligament*> createLigamentsFromFiducial(btRigidBody* leftBody, btRigidBody* rightBody, btRigidBody* movableBody, std::string filename);
	btAlignedObjectArray<Ligament*> createLigamentsFromFiducial2(btRigidBody* leftBody, btRigidBody* rightBody, btRigidBody* movableBody, std::string filename);
	btAlignedObjectArray<Ligament*> createLigamentsFromFiducial3(btRigidBody* leftBody, btRigidBody* rightBody, btRigidBody* movableBody, std::string filename);
	btAlignedObjectArray<Ligament*> createLigamentsFromFiducial4(btRigidBody* leftBody, btRigidBody* rightBody, btRigidBody* movableBody, std::string filename);
	btAlignedObjectArray<rbMuscle*> createMusclesFromFiducial(btRigidBody* movableBody, std::string filename);
	btAlignedObjectArray<rbMuscle*> createMusclesFrom2Fiducial(btRigidBody* movableBody, std::string origin, std::string insertion);
	btAlignedObjectArray<rbMuscle*> createMusclesFrom2FiducialForSoftBody(btSoftBody* movableBody, std::string origin, std::string insertion);
	sbMuscle* createSBMuscleFromFiducial(btSoftBody* softBody, std::string label, std::string origin);
	sbMuscle* createSBMuscleFrom2Fiducial(btSoftBody* softBody, std::string origin, std::string insertion);
	sbMuscle* createSBMuscle(btSoftBody* softBody, std::string label);
	std::vector<cNode> getClosetNodes(btSoftBody* body, btVector3 point, float distance);
	void alignAxis(btRigidBody* bodyA, btRigidBody* bodyB, btTransform &bodyATransform, btTransform &bodyBTransform);

	void stepWorld(btScalar timeStep, int maxSubSteps = 1, btScalar fixedTimeStep = btScalar(1.) / btScalar(60.))
	{
		if (runPhysics)
		{
			mDynamicsWorld->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
			if (mMuscleControl) 
			{
				/*boost::thread* t =
					new boost::thread(&MuscleControl::applyAllForces, mMuscleControl);
				t->detach();*/
				mMuscleControl->applyAllForces();
			}
			//if (mSimLog.recording);
		}
		if (displayDebug) mDynamicsWorld->debugDrawWorld(); 
		else 
		{ 
			if (displaySoftBodies) for (int i = 0; i < mSoftBodies.size(); i++) btSoftBodyHelpers::Draw(mSoftBodies[i], mDynamicsWorld->getDebugDrawer(), drawSoftBodyFlags);
			
		}
		if (mLigamentControl) mLigamentControl->drawLigaments();

		if (!displayDebug)
		for (int i = 0; i < drawRigidBodies.size(); i++)
		{
			if (drawRigidBodies[i]->draw)
			{
				mDynamicsWorld->debugDrawObject(drawRigidBodies[i]->body->getWorldTransform(), drawRigidBodies[i]->body->getCollisionShape(), drawRigidBodies[i]->color);
				//std::cout << "Desenhando corpo rigido.." << std::endl;
			}
		}
	}

	simulationLog* loadLogPoints(std::string filename, btRigidBody* body, int timeStep = 500);

	btSoftBodyWorldInfo &getSoftBodyWorldInfo()
	{
		return mSoftBodyWorldInfo;
	}

	btSoftRigidDynamicsWorld* getSoftRigidDynamicsWorld()
	{
		return (btSoftRigidDynamicsWorld*)mDynamicsWorld;
	}


	btDynamicsWorld* getDynamicsWorld()
	{
		return mDynamicsWorld;
	}

	void setLigamentControl(LigamentControl* lcontrol)
	{
		mLigamentControl = lcontrol;
	}

	LigamentControl* getLigamentControl()
	{
		return mLigamentControl;
	}

	void setDebugDrawer(btIDebugDraw* debugDrawer)
	{
		mDynamicsWorld->setDebugDrawer(debugDrawer);
	}

	void setDisplayDebug(bool value)
	{
		displayDebug = value;
	}

	bool isDisplayDebug()
	{
		return displayDebug;
	}

	void setRunPhysics(bool value)
	{
		runPhysics = value;
		//if (mMuscleControl) mMuscleControl->activeMotors();
	}

	bool isRunPhysics()
	{
		return runPhysics;
	}

	void setMuscleControl(MuscleControl* mcontrol)
	{
		mMuscleControl = mcontrol;
	}

	void setMassControl(MassControl* mcontrol)
	{
		mMassControl = mcontrol;
	}

	void setDiscControl(DiscControl* mcontrol)
	{
		mDiscControl = mcontrol;
	}

	void setSoftBodyControl(SoftBodyControl* mcontrol)
	{
		mSoftBodyControl = mcontrol;
	}


	void toggleActiveMotors()
	{
		if (runPhysics) mMuscleControl->toggleActiveMotors();
	}

	void toggleDisplayDebug()
	{
		if (displayDebug) displayDebug = false; else displayDebug = true;
	}

	void toggleDisplaySoftBodies()
	{
		if (displaySoftBodies) displaySoftBodies = false; else displaySoftBodies = true;
	}


	void toggleRunPhysics()
	{
		if (runPhysics) runPhysics = false; else runPhysics = true;
		/*mMuscleControl->runPhysics = runPhysics;
		std::unique_lock<std::mutex> lck(mMuscleControl->mtx);
		mMuscleControl->cv.notify_all();*/
	}

	void toggleDisplayClusters()
	{
		if (displaySoftBodyClusters) 
		{
			drawSoftBodyFlags -= fDrawFlags::Clusters;
			displaySoftBodyClusters = false;
		}
		else
		{
			drawSoftBodyFlags += fDrawFlags::Clusters;
			displaySoftBodyClusters = true;
		}
		((btSoftRigidDynamicsWorld*)mDynamicsWorld)->setDrawFlags(drawSoftBodyFlags + fDrawFlags::Contacts);
	}

	void toggleDisplayForces()
	{
		mMuscleControl->toggleDrawForces();
	}

	void toggleAdaptiveForces()
	{
		mMuscleControl->toggleAdaptiveForceLines();
	}

	void toggleDisplayLigaments()
	{
		mLigamentControl->toggleDisplayLigaments();
	}

	void toggleMuscleControlVisibility()
	{
		mMuscleControl->toggleControlsVisibility();
	}

	void toggleLigamentsControlVisibility()
	{
		mLigamentControl->toggleControlsVisibility();
	}

	void toggleMassControlVisibility()
	{
		mMassControl->toggleControlsVisibility();
	}

	void addRigidBodyToDraw(btRigidBody* body, int index, btVector3 color)
	{
		RigidBodyDrawer* drawBody = new RigidBodyDrawer();
		drawBody->index = index;
		drawBody->body = body;
		drawBody->color = color;
		drawBody->draw = false;
		drawRigidBodies.push_back(drawBody);
	}

	void addSoftBodyToDraw(btSoftBody* body)
	{
		mSoftBodies.push_back(body);
	}

	void toggleDrawRigidBody(int index)
	{
		for (int i = 0; i < drawRigidBodies.size(); i++)
		{
			if (drawRigidBodies[i]->index == index) if (drawRigidBodies[i]->draw) drawRigidBodies[i]->draw = false; else drawRigidBodies[i]->draw = true;
		}
	}

	void startRecording(simulationLog* sim, std::string desc, bool position = false);

	void stopRecording(simulationLog* sim)
	{
		sim->recording = false;
	}

	void recordMovements(simulationLog* sim, std::string desc);

	void setSoftBodyFactor(btScalar value)
	{
		if (mMuscleControl)
		{
			mMuscleControl->setSoftBodyFactor(value);
		}
	}

	btScalar getSoftBodyFactor()
	{
		if (mMuscleControl) return mMuscleControl->getSoftBodyFactor();
		return 0;
	}

	void saveSoftBodyConfigs(std::string filename)
	{
		std::vector<softBodyConfig> configs = mSoftBodyControl->getSoftBodyConfigs();
		// save data to archive
		{
			std::ofstream ofs(filename);
			boost::archive::text_oarchive oa(ofs);
			oa & configs;
			ofs.close();
		}
	}

	bool loadSoftBodyConfigs(std::string filename)
	{
		std::ifstream ifs(filename, std::ios::binary);
		if (ifs.good())
			// load data to archive
		{
			std::cout << "Loading configs SB" << std::endl;
			std::vector<softBodyConfig> configs;
			{
				boost::archive::text_iarchive ia(ifs);
				ia & configs;
				ifs.close();
			}
			//for (int i = 0; i < configs.size(); i++)
			//{
			//	/*std::cout << " Index SB: " << configs[i].index << std::endl;
			//	std::cout << " kAHR SB: " << configs[i].cfg.kAHR << std::endl;*/
			//}
			mSoftBodyControl->setSoftBodyConfigs(configs);
			return true;

		}
		return false;
	}

	std::string checkExists(std::string filename)
	{
		std::string newFile = filename;
		int n = 0;
		while (boost::filesystem::exists(newFile))
		{
			size_t lastdot = newFile.find_last_of(".");
			std::string rawname = filename.substr(0, lastdot);
			rawname.append(std::to_string(n));
			rawname.append(".obj");
			newFile = rawname;
			n++;
		}
		return newFile;
	}

	void recordDiscShape(btSoftBody* softbody, std::string filename)
	{
		std::ofstream mFile3(checkExists(filename));
		btSoftBody *sb = softbody;

		// Record Vertices
		for (int k = 0; k < sb->m_nodes.size(); k++)
		{
			mFile3 << "v " << sb->m_nodes[k].m_x.getX() << " " << sb->m_nodes[k].m_x.getY() << " " << sb->m_nodes[k].m_x.getZ() << std::endl;

		}

		// Record faces
		for (int i = 0; i < sb->m_faces.size(); i++)
		{
			//std::cout << "f";
			mFile3 << "f";
			for (int j = 0; j < 3; j++)
			{

				btVector3 nLoc = sb->m_faces[i].m_n[j]->m_x;
				for (int k = 0; k < sb->m_nodes.size(); k++)
				{
					if (sb->m_nodes[k].m_x == nLoc)
					{
						int f = k + 1;
						//std::cout << " " << f;
						mFile3 << " " << f;
					}

				}

			}
			//std::cout << "\n";
			mFile3 << std::endl;
		}

		mFile3.close();
	}
	
};


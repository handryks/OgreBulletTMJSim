#include "PhysicsEngine.h"
#include <BulletSoftBody\btSoftBodyRigidBodyCollisionConfiguration.h>
#include "MultiThreadHelper.h"
#include "btDistanceConstraint.h"

#include "DebugDrawUtils.h"





// Constants for groud creation
static btVector3*	gGroundVertices = 0;
static int*	gGroundIndices = 0;
static float waveheight = 5.f;
const float TRIANGLE_SIZE = 8.f;


// Obj loader
#include <vector>
#include <assert.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <tiny_obj_loader.h>
#include <algorithm>






btVector3 PhysicsEngine::COLOR_RED = btVector3(255, 0, 0);
btVector3 PhysicsEngine::COLOR_GREEN = btVector3(0, 255, 0);
btVector3 PhysicsEngine::COLOR_BLUE = btVector3(0, 0, 255);
btVector3 PhysicsEngine::COLOR_YELLOW = btVector3(255, 255, 0);
btVector3 PhysicsEngine::COLOR_CYAN = btVector3(0, 255, 255);
btVector3 PhysicsEngine::COLOR_PINK = btVector3(255, 0, 255);
btVector3 PhysicsEngine::COLOR_BLACK = btVector3(0, 0, 0);
btVector3 PhysicsEngine::COLOR_WHITE = btVector3(255, 255, 255);

float PhysicsEngine::DEFAULT_MUSCLE_MASS = 1.0f;
float PhysicsEngine::DEFAULT_MANDIBLE_MASS = 75.0f;
float PhysicsEngine::DEFAULT_LIGAMENT_MASS = 75.0f;
float PhysicsEngine::DEFAULT_DISC_MASS = 10.0f;

PhysicsEngine::PhysicsEngine()
{
}


PhysicsEngine::~PhysicsEngine()
{
}


void PhysicsEngine::createSoftBodyWorld(btVector3 gravity)
{
	m_numThreads = setNumThreads(getMaxNumThreads());

	mDispatcher = 0;

	//btDefaultCollisionConstructionInfo cci;
	//// it isn't threadsafe to resize these pools while threads are using them
	//cci.m_defaultMaxPersistentManifoldPoolSize = 32768;
	//cci.m_defaultMaxCollisionAlgorithmPoolSize = 32768;

	mCollisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	mDispatcher = new btCollisionDispatcher(mCollisionConfiguration);

	btVector3 worldAabbMin(-1000, -1000, -1000);
	btVector3 worldAabbMax(1000, 1000, 1000);

	const int maxProxies = 32766;
	const int maxOverlap = 65535;

	//mBroadohase = new btAxisSweep3(worldAabbMin, worldAabbMax, maxProxies);
	//mBroadohase = new bt32BitAxisSweep3(worldAabbMin, worldAabbMax);
	mBroadohase = new btDbvtBroadphase();
	//mSolver = new btSequentialImpulseConstraintSolver();

	mDispatcherMt = new MyCollisionDispatcher(mCollisionConfiguration);
	mSolverMt = new MyConstraintSolverPool(m_numThreads);


	//mDispatcherMt->setDispatcherFlags(btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION);

	//mDynamicsWorld = new btSoftRigidDynamicsWorld(mDispatcher, mBroadohase, mSolver, mCollisionConfiguration, mSoftBodySolver);
	mDynamicsWorld = new btSoftRigidDynamicsWorld(mDispatcherMt, mBroadohase, mSolverMt, mCollisionConfiguration, mSoftBodySolver);
	//mDynamicsWorld = new MySoftRigidDynamicsWorld(mDispatcherMt, mBroadohase, mSolverMt, mCollisionConfiguration, mSoftBodySolver);

	//((MySoftRigidDynamicsWorld*)mDynamicsWorld)->getSimulationIslandManager()->setIslandDispatchFunction(parallelIslandDispatch);
	
	mDynamicsWorld->setGravity(gravity);
	mDynamicsWorld->setForceUpdateAllAabbs(false);
	mSoftBodyWorldInfo.m_dispatcher = mDispatcherMt;
	mSoftBodyWorldInfo.m_broadphase = mBroadohase;
	mSoftBodyWorldInfo.m_sparsesdf.Initialize();

	mMuscleControl = 0;
	mLigamentControl = 0;
	mDiscControl = 0;
	mMassControl = 0;
	mSoftBodyControl = 0;
	displaySoftBodies = true;
}

void PhysicsEngine::deleteWorld()
{
	std::cout << "Cleaning world\n";

	mMuscleControl->disableMotors();
	runPhysics = false;

	//mLigamentControl->saveConfig("teste.txt");

	delete mMuscleControl;
	delete mDiscControl;
	delete mLigamentControl;
	delete mMassControl;
	delete mSoftBodyControl;


	
	for (int i = 0; i < mDynamicsWorld->getNumConstraints(); i++) {
		delete mDynamicsWorld->getConstraint(i);
	}

	for (int i = 0; i < mRigidBodies.size(); i++)
	{
		mDynamicsWorld->removeRigidBody(mRigidBodies[i]);
		delete mRigidBodies[i]->getMotionState();
		delete mRigidBodies[i];
	}
	mRigidBodies.clear();

	for (int i = 0; i < mSoftBodies.size(); i++)
	{
		((btSoftRigidDynamicsWorld*)mDynamicsWorld)->removeSoftBody(mSoftBodies[i]);
		delete mSoftBodies[i];
	}
	mSoftBodies.clear();	

	/*for (int i = 0; i < mCollisionShapes.size(); i++)
	{
		delete mCollisionShapes[i];
	}
	mCollisionShapes.clear();*/

	for (int i = 0; i < drawRigidBodies.size(); i++)
	{
		delete drawRigidBodies[i];
	}
	drawRigidBodies.clear();

	if (mLigamentControl)
	{
		for (int i = 0; i < mLigamentControl->mLigaments.size(); i++)
		{
			delete mLigamentControl->mLigaments[i];
		}
	}

	
	delete mDynamicsWorld;
	delete mDispatcher;
	delete mBroadohase;
	delete mSolver;
	delete mCollisionConfiguration;
	delete mSolverMt;
	delete mDispatcherMt;
	//delete mSoftBodySolver;
	
}

void PhysicsEngine::createDefaultGround()
{

	btCollisionShape* groundShape = 0;
	{
		int i;
		int j;

		const int NUM_VERTS_X = 60;
		const int NUM_VERTS_Y = 60;
		const int totalVerts = NUM_VERTS_X*NUM_VERTS_Y;
		const int totalTriangles = 2 * (NUM_VERTS_X - 1)*(NUM_VERTS_Y - 1);

		gGroundVertices = new btVector3[totalVerts];
		gGroundIndices = new int[totalTriangles * 3];

		btScalar offset(-5);

		for (i = 0; i<NUM_VERTS_X; i++)
		{
			for (j = 0; j<NUM_VERTS_Y; j++)
			{
				gGroundVertices[i + j*NUM_VERTS_X].setValue((i - NUM_VERTS_X*0.5f)*TRIANGLE_SIZE,
					//0.f,
					waveheight*sinf((float)i)*cosf((float)j + (float)offset),
					(j - NUM_VERTS_Y*0.5f)*TRIANGLE_SIZE);
			}
		}

		int vertStride = sizeof(btVector3);
		int indexStride = 3 * sizeof(int);

		int index = 0;
		for (i = 0; i<NUM_VERTS_X - 1; i++)
		{
			for (int j = 0; j<NUM_VERTS_Y - 1; j++)
			{
				gGroundIndices[index++] = j*NUM_VERTS_X + i;
				gGroundIndices[index++] = j*NUM_VERTS_X + i + 1;
				gGroundIndices[index++] = (j + 1)*NUM_VERTS_X + i + 1;

				gGroundIndices[index++] = j*NUM_VERTS_X + i;
				gGroundIndices[index++] = (j + 1)*NUM_VERTS_X + i + 1;
				gGroundIndices[index++] = (j + 1)*NUM_VERTS_X + i;
			}
		}

		btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(totalTriangles,
			gGroundIndices,
			indexStride,
			totalVerts, (btScalar*)&gGroundVertices[0].x(), vertStride);

		bool useQuantizedAabbCompression = true;

		groundShape = new btBvhTriangleMeshShape(indexVertexArrays, useQuantizedAabbCompression);
		groundShape->setMargin(0.5);
	}


	// Create Physics Plane
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, -150, 0));

	btScalar groundMass(0.); //the mass is 0, because the ground is immovable (static)
	btVector3 localGroundInertia(0, 0, 0);

	btDefaultMotionState *groundMotionState = new btDefaultMotionState(groundTransform);

	btRigidBody::btRigidBodyConstructionInfo groundRBInfo(groundMass, groundMotionState, groundShape, localGroundInertia);
	btRigidBody *groundBody = new btRigidBody(groundRBInfo);
	mDynamicsWorld->addRigidBody(groundBody);
	mRigidBodies.push_back(groundBody);
	mCollisionShapes.push_back(groundShape);
}

btRigidBody* PhysicsEngine::createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, btMotionState *motionState, short group, short mask, bool addColShape)
{
	//rigidbody is dynamic if and only if mass is non zero, otherwise static
	bool isDynamic = (mass != 0.f);

	btVector3 localInertia(0, 0, 0);
	if (isDynamic)
		shape->calculateLocalInertia(mass, localInertia);

	//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects

	//btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	btRigidBody::btRigidBodyConstructionInfo cInfo(mass, motionState, shape, localInertia);

	btRigidBody* body = new btRigidBody(cInfo);

	if (group == -1)
	{
		mDynamicsWorld->addRigidBody(body);
	}
	else
	{
		mDynamicsWorld->addRigidBody(body, group, mask);
	}

	mRigidBodies.push_back(body);
	if (addColShape) mCollisionShapes.push_back(shape);
	return body;
}

btRigidBody* PhysicsEngine::createRigidBody(float mass, btVector3 origin, btCollisionShape* shape, short group, short mask, bool addColShape)
{
	btTransform startTransform = btTransform::getIdentity();
	startTransform.setOrigin(origin);
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	return createRigidBody(mass, startTransform, shape, myMotionState, group, mask, addColShape);
}


btRigidBody* PhysicsEngine::createRigidBody(float mass, const btTransform& startTransform, btCollisionShape* shape, short group, short mask, bool addColShape)
{
	btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);

	return createRigidBody(mass, startTransform, shape, myMotionState, group, mask, addColShape);
}



btCompoundShape* PhysicsEngine::loadCompoundShapeFromObjFile(std::string filename)
{
	std::string inputfile = filename;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::string err = tinyobj::LoadObj(shapes, materials, inputfile.c_str(), "data\\");

	//std::vector<btConvexShape*> colshapes;

	if (!err.empty()) {
		std::cerr << err << std::endl;
		std::cout << "ARQUIVO NAO ENCONTRADO\n";
		//return NULL;
		//exit(1);
	}

	bool sEnableSAT = false;

	btCompoundShape* shape = new btCompoundShape();

	for (size_t i = 0; i < shapes.size(); i++)
	{
		btConvexHullShape* mesh = new btConvexHullShape();

		//printf("shape[%ld].vertices: %ld\n", i, shapes[i].mesh.positions.size());
		assert((shapes[i].mesh.positions.size() % 3) == 0);
		for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
			mesh->addPoint(btVector3(
				shapes[i].mesh.positions[3 * v + 0],
				shapes[i].mesh.positions[3 * v + 1],
				shapes[i].mesh.positions[3 * v + 2]
				));
		}

		btTransform startTransform;
		startTransform.setIdentity();
		shape->addChildShape(startTransform, mesh);
	}

	return shape;
}

btRigidBody* PhysicsEngine::createRigidBodyFromObjFile(std::string filename, btMotionState * motionState, float mass, short group, short mask)
{
	btCollisionShape* shape = loadCompoundShapeFromObjFile(filename);
	return createRigidBody(mass, btTransform::getIdentity(), shape, motionState, group, mask);
}


std::vector<FiducialMark> PhysicsEngine::loadFiducial(std::string filename)
{
	std::ifstream fileStream(filename);

	std::string lineStream;
	int linenum = 0;

	std::vector<FiducialMark> fiducials;

	while (std::getline(fileStream, lineStream))
	{
		const char *token = lineStream.c_str();
		if (token[0] == '#') continue;

		FiducialMark fid;
		linenum++;
		//cout << "\nLine #" << linenum << ":" << endl;         
		std::istringstream linestream(lineStream);
		std::string item;
		int itemnum = 0;
		while (std::getline(linestream, item, ','))
		{
			itemnum++;
			//cout << "Item #" << itemnum << ": " << item << endl;
			// columns = id, x, y, z, ow, ox, oy, oz, vis, sel, lock, label, desc, associatedNodeID
			if (itemnum == 2) fid.x = -std::stof(item);
			if (itemnum == 3) fid.z = std::stof(item);
			if (itemnum == 4) fid.y = std::stof(item);
			if (itemnum == 10) fid.sel = std::stoi(item);
			if (itemnum == 12) fid.label = item;
			if (itemnum == 13) fid.desc = item;
		}
		fiducials.push_back(fid);
	}

	fileStream.close();

	//while (std::getline(fileStream, lineStream, ','))
	//{
	//	std::cout << "Comecou a ler o arquivo" << std::endl;

	//	const char *token = lineStream.c_str();
	//	if (token[0] == '#') continue;
	//	
	//	// columns = id, x, y, z, ow, ox, oy, oz, vis, sel, lock, label, desc, associatedNodeID
	//	std::string id, label, desc, associatedNodeID;
	//	int vis, sel, lock;
	//	float x, y, z, ow, ox, oy, oz;

	//	std::cout << lineStream << std::endl;
	//	
	//	//scanf(lineStream.c_str(), "%50[,]s,%f,%f,%f,%f,%f,%f,%f,%i,%i,%i,%s,%s,%s\n", &id, &x, &y, &z, &ow, &ox, &oy, &oz, &vis, &sel, &lock, &label, &desc, &associatedNodeID);
	//	scanf(lineStream.c_str(), "%[^,]", &id);
	//	
	//	std::cout << "Leou a linha com sucesso" << std::endl;


	//	/*Physics::FiducialMark fid;
	//	fid.id = id;
	//	fid.x = x;
	//	fid.y = y;
	//	fid.z = z;
	//	fid.ow = ow;
	//	fid.ox = ox;
	//	fid.oy = oy;
	//	fid.oz = oz;
	//	fid.vis = vis;
	//	fid.sel = sel;
	//	fid.lock = lock;
	//	fid.label = label;
	//	fid.desc = desc;
	//	fid.associatedNodeID = associatedNodeID;

	//	fiducials.push_back(fid);*/

	//}

	return fiducials;
}

btTypedConstraint* PhysicsEngine::createP2PConstraint(btRigidBody* bodyA, btRigidBody* bodyB, btVector3 locationA, btVector3 locationB)
{
	btTransform bodyATransform = btTransform::getIdentity();
	btVector3 pointA = bodyA->getCenterOfMassTransform().inverse() (locationA);

	btTransform	bodyBTransform = btTransform::getIdentity();
	btVector3 pointB = bodyB->getCenterOfMassTransform().inverse() (locationB);

	btPoint2PointConstraint* constraint = new btPoint2PointConstraint(*bodyA, *bodyB, pointA, pointB);
	mDynamicsWorld->addConstraint(constraint, true);
	return constraint;
}

btAlignedObjectArray<Ligament*> PhysicsEngine::createLigamentsFromFiducial(btRigidBody* leftBody, btRigidBody* rightBody, btRigidBody* movableBody, std::string filename)
{
	std::vector<FiducialMark> marks = loadFiducial(filename);
	btCollisionShape* shape = new btSphereShape(btScalar(1.0));

	mCollisionShapes.push_back(shape);

	btAlignedObjectArray<Ligament*> ligaments;

	btTypedConstraint* c0;
	btTypedConstraint* c1;
	for (int i = 0; i < marks.size(); i += 2)
	{
		btVector3 ori = marks[i].getVector3();
		btVector3 ins = marks[i + 1].getVector3();

		Ligament *l = new Ligament();
		l->label = marks[i].label;

		btTransform oriTrans = btTransform::getIdentity();
		oriTrans.setOrigin(ori);
		btRigidBody* b = createRigidBody(DEFAULT_LIGAMENT_MASS, oriTrans, shape, false);
		//b->setActivationState(DISABLE_DEACTIVATION);
		if (marks[i].label.at(0) == 'L')
		{
			c0 = createP2PConstraint(leftBody, b, ori, ori);
		}
		else
		{
			c0 = createP2PConstraint(rightBody, b, ori, ori);
		}
		c1 = createP2PConstraint(b, movableBody, ins, ins);
		l->c0 = c0;
		l->c1 = c1;
		l->mDebugDraw = mDynamicsWorld->getDebugDrawer();
		ligaments.push_back(l);
		/*c0->setOverrideNumSolverIterations(150);
		c1->setOverrideNumSolverIterations(150);*/
	}
	return ligaments;
}

btAlignedObjectArray<Ligament*> PhysicsEngine::createLigamentsFromFiducial4(btRigidBody* leftBody, btRigidBody* rightBody, btRigidBody* movableBody, std::string filename)
{
	std::vector<FiducialMark> marks = loadFiducial(filename);
	btCollisionShape* shape = new btSphereShape(btScalar(1.0f));

	mCollisionShapes.push_back(shape);

	btAlignedObjectArray<Ligament*> ligaments;
	btDistanceConstraint* constraint;

	for (int i = 0; i < marks.size(); i += 2)
	{
		btVector3 ori = marks[i].getVector3();
		btVector3 ins = marks[i + 1].getVector3();

		Ligament *l = new Ligament();
		l->label = marks[i].label;

		btAlignedObjectArray<btVector3> points;
		int numVertices = 2;

		btTransform oriTrans = btTransform::getIdentity();
		oriTrans.setOrigin(ori);


		for (int j = 0; j <= numVertices; j++)
		{
			btScalar n = btScalar(1.f / numVertices) * btScalar(j);
			btTransform trans = oriTrans;
			btVector3 p = trans.inverse() * ins;
			p = p * n;
			p = trans * p;
			points.push_back(p);
		}

		btAlignedObjectArray<btRigidBody*> bodies;

		btCollisionShape * mesh = new btSphereShape(1.f);

		float mass = 75.f;

		for (int j = 0; j < points.size(); j++)
		{
			/*btConvexHullShape* mesh = new btConvexHullShape();
			mesh->addPoint(points[j - 1]);
			mesh->addPoint(points[j]);*/
			btTransform t = btTransform::getIdentity();
			t.setOrigin(points[j]);
			btRigidBody* b = createRigidBody(mass, t, mesh, COL_NOTHING, ligamentCollidesWith, false);
			bodies.push_back(b);
			//mass -= 30.f;
		}

		btTypedConstraint* c0;

		for (int j = 1; j < bodies.size(); j++)
		{
			c0 = createP2PConstraint(bodies[j], bodies[j - 1], points[j], points[j]);
			c0->setOverrideNumSolverIterations(500);
			l->constraints.push_back(c0);
		}

		if (marks[i].label.at(0) == 'L')
		{
			c0 = (btPoint2PointConstraint*) createP2PConstraint(leftBody, bodies[0], ori, ori);
		}
		else
		{
			c0 = (btPoint2PointConstraint*) createP2PConstraint(rightBody, bodies[0], ori, ori);
		}
		l->constraints.push_back(c0);
		c0->setOverrideNumSolverIterations(500);

		c0 = createP2PConstraint(bodies[bodies.size() - 1], movableBody, ins, ins);
		l->constraints.push_back(c0);
		c0->setOverrideNumSolverIterations(500);

		l->mDebugDraw = mDynamicsWorld->getDebugDrawer();
		ligaments.push_back(l);

	}
	return ligaments;
}

btAlignedObjectArray<Ligament*> PhysicsEngine::createLigamentsFromFiducial3(btRigidBody* leftBody, btRigidBody* rightBody, btRigidBody* movableBody, std::string filename)
{
	std::vector<FiducialMark> marks = loadFiducial(filename);
	
	btCollisionShape* shape = new btSphereShape(1.0f);
	//mCollisionShapes.push_back(shape);

	btAlignedObjectArray<Ligament*> ligaments;

	btTypedConstraint* c0;
	btTypedConstraint* c1;
	for (int i = 0; i < marks.size(); i += 2)
	{
		btVector3 ori = marks[i].getVector3();
		btVector3 ins = marks[i + 1].getVector3();

		Ligament *l = new Ligament();
		l->label = marks[i].label;

		btTransform oriTrans = btTransform::getIdentity();
		oriTrans.setOrigin(ori);

		btRigidBody* b0 = createRigidBody(0.1f, oriTrans, shape, false);
		

		if (marks[i].label.at(0) == 'L')
		{
			c0 = createP2PConstraint(leftBody, b0, ori, ori);
		}
		else
		{
			c0 = createP2PConstraint(rightBody, b0, ori, ori);
		}

		btTransform t0 = btTransform::getIdentity();
		btTransform t1 = btTransform::getIdentity();

		t0.setOrigin(b0->getCenterOfMassTransform().inverse() * ori);
		t1.setOrigin(movableBody->getCenterOfMassTransform().inverse() * ins);

		btScalar limit = ori.distance(ins);

		btGeneric6DofSpring2Constraint* c6dof = new btGeneric6DofSpring2Constraint(*b0, *movableBody, t0, t1);
		//c6dof->setLinearUpperLimit(btVector3(0, -ori.distance(ins), 0));
		//c6dof->enableSpring(1, true);
		//c6dof->setStiffness(1, -10);
		c6dof->setLinearLowerLimit(btVector3(-limit * 0.25, -limit, -limit));
		c6dof->setLinearUpperLimit(btVector3(limit * 0.25, limit, limit * 0.8));
		c6dof->setDamping(0, 100);
		c6dof->setDamping(1, 100);
		c6dof->setDamping(2, 100);

		mDynamicsWorld->addConstraint(c6dof, true);
		c1 = (btTypedConstraint*)c6dof;
		
		l->c0 = c0;
		l->c1 = c1;
		l->mDebugDraw = mDynamicsWorld->getDebugDrawer();
		ligaments.push_back(l);
		//c1->setOverrideNumSolverIterations(150);
		
		/*c0->setOverrideNumSolverIterations(150);
		c1->setOverrideNumSolverIterations(150);*/
	}
	return ligaments;
}

btAlignedObjectArray<Ligament*> PhysicsEngine::createLigamentsFromFiducial2(btRigidBody* leftBody, btRigidBody* rightBody, btRigidBody* movableBody, std::string filename)
{
	std::vector<FiducialMark> marks = loadFiducial(filename);

	btAlignedObjectArray<Ligament*> ligaments;

	for (int i = 0; i < marks.size(); i += 2)
	{
		btVector3 ori = marks[i].getVector3();
		btVector3 ins = marks[i + 1].getVector3();

		Ligament *l = new Ligament();
		l->label = marks[i].label;

		btSoftBody* rope = btSoftBodyHelpers::CreateRope(mSoftBodyWorldInfo, ori, ins, 5, 1);
		rope->setTotalMass(DEFAULT_LIGAMENT_MASS);
		float mass = 15.0f;
		for (int j = rope->m_nodes.size() - 1; j > 0; j--)
		{
			rope->setMass(j, mass);
			mass += 10.f;
		}
		btSoftBody::Material*	pm = rope->appendMaterial();
		pm->m_kLST = .5f;
		pm->m_kAST = .5f;
		rope->generateBendingConstraints(2, pm);
		rope->randomizeConstraints();
		rope->setPose(false, true);
		rope->m_cfg.kMT = 0.05f;
		rope->setRestLengthScale(-0.5f);
		getSoftRigidDynamicsWorld()->addSoftBody(rope, COL_NOTHING, ligamentCollidesWith);
		//for (int i = 0; i < 10; i++) 
			rope->appendAnchor(rope->m_nodes.size() - 1, movableBody, true);	mSoftBodies.push_back(rope);
		l->sbody = rope;
		l->mDebugDraw = mDynamicsWorld->getDebugDrawer();
		ligaments.push_back(l);

	}
	return ligaments;
}


btAlignedObjectArray<rbMuscle*> PhysicsEngine::createMusclesFromFiducial(btRigidBody* movableBody, std::string filename)
{
	std::vector<FiducialMark> muscles = loadFiducial(filename);
	btAlignedObjectArray<btRigidBody*> bodies;
	btAlignedObjectArray<rbMuscle*> rbMuscles;

	btCollisionShape* sph = new btSphereShape(.2f);
	mCollisionShapes.push_back(sph);

	for (int i = 0; i < muscles.size(); i += 2)
	{
		if (muscles[i].sel == 0) continue;
		std::string label = muscles[i].label;
		btVector3 ori = muscles[i].getVector3();
		btVector3 ins = muscles[i + 1].getVector3();

		rbMuscle *mus = new rbMuscle();
		mus->label = label;
		mus->orgin = ori;
		mus->insertion = ins;
		mus->relInsertion = movableBody->getCenterOfMassTransform().inverse() (ins);
		mus->multiply = 0;
		mus->mDebugDraw = mDynamicsWorld->getDebugDrawer();

		/*muscleMotor motor;
		motor.addForce = true;
		motor.stepForce = 0.1f;
		motor.maxForce = 5;
		motor.timeInActive = 3000;
		motor.timeInRest = 3000;
		motor.timeStep = 200;*/

		bool exist = false;

		//if (bodies.size() > 0)
		//{
		for (int j = 0; j < bodies.size(); j++)
		{

			//std::cout << "Distancia entre os pontos: " << bodies[j]->getCenterOfMassTransform().getOrigin().distance(ins) << std::endl;
			if (bodies[j]->getCenterOfMassTransform().getOrigin().distance(ins) == 0)
			{
				exist = true;
				mus->body0 = bodies[j];
				//std::cout << "Ponto de insercao ja existe\n";
				break;
			}
		}
		//}

		if (!exist)
		{
			btRigidBody* rbi = createRigidBody(0.0001f, ins, sph, COL_NOTHING, muscleCollidesWith, false);
			bodies.push_back(rbi);
			//std::cout << "Adicionando corpo a array\n";

			btVector3 rbPos = movableBody->getCenterOfMassTransform().inverse() (ins);
			btVector3 rbiPos = rbi->getCenterOfMassTransform().inverse() (ins);

			btPoint2PointConstraint* p2pCon = new btPoint2PointConstraint(*movableBody, *rbi, rbPos, rbiPos);
			mDynamicsWorld->addConstraint(p2pCon, true);
			p2pCon->setOverrideNumSolverIterations(60);
			mus->body0 = rbi;
		}

		//mus->motor = motor;
		//musCon->m_rbmuscles.push_back(mus);
		btVector3 transIns = movableBody->getCenterOfMassTransform() (mus->relInsertion);
		rbMuscles.push_back(mus);
		/*std::cout << "Original insertion: " << cvt(ins) << std::endl;
		std::cout << "Relative insertion: " << cvt(mus->relInsertion) << std::endl;
		std::cout << "Transformed insertion: " << cvt(transIns) << std::endl;*/
	}
	return rbMuscles;
}

btAlignedObjectArray<rbMuscle*> PhysicsEngine::createMusclesFrom2Fiducial(btRigidBody* movableBody, std::string origin, std::string insertion)
{
	std::vector<FiducialMark> fidOrigin = loadFiducial(origin);
	std::vector<FiducialMark> fidInsertion = loadFiducial(insertion);
	btAlignedObjectArray<btRigidBody*> bodies;
	btAlignedObjectArray<rbMuscle*> rbMuscles;

	btCollisionShape* sph = new btSphereShape(0.2f);
	mCollisionShapes.push_back(sph);

	for (int i = 0; i < fidOrigin.size(); i ++)
	{
		if (fidOrigin[i].sel == 0) continue;
		std::string label = fidOrigin[i].label;
		btVector3 ori = fidOrigin[i].getVector3();
		btVector3 ins = fidInsertion[i].getVector3();

		rbMuscle *mus = new rbMuscle();
		mus->label = label;
		mus->orgin = ori;
		mus->insertion = ins;
		mus->relInsertion = movableBody->getCenterOfMassTransform().inverse() (ins);
		mus->multiply = 0;
		mus->mDebugDraw = mDynamicsWorld->getDebugDrawer();

		/*muscleMotor motor;
		motor.addForce = true;
		motor.stepForce = 0.1f;
		motor.maxForce = 5;
		motor.timeInActive = 3000;
		motor.timeInRest = 3000;
		motor.timeStep = 200;*/

		bool exist = false;

		//if (bodies.size() > 0)
		//{
		for (int j = 0; j < bodies.size(); j++)
		{

			//std::cout << "Distancia entre os pontos: " << bodies[j]->getCenterOfMassTransform().getOrigin().distance(ins) << std::endl;
			if (bodies[j]->getCenterOfMassTransform().getOrigin().distance(ins) == 0)
			{
				exist = true;
				mus->body0 = bodies[j];
				//std::cout << "Ponto de insercao ja existe\n";
				break;
			}
		}
		//}

		if (!exist)
		{
			btRigidBody* rbi = createRigidBody(DEFAULT_MUSCLE_MASS, ins, sph, COL_NOTHING, muscleCollidesWith, false);
			bodies.push_back(rbi);
			//std::cout << "Adicionando corpo a array\n";

			rbi->setDamping(0.5f, 0.5f);

			btVector3 rbPos = movableBody->getCenterOfMassTransform().inverse() (ins);
			btVector3 rbiPos = rbi->getCenterOfMassTransform().inverse() (ins);

			btPoint2PointConstraint* p2pCon = new btPoint2PointConstraint(*movableBody, *rbi, rbPos, rbiPos);
			mDynamicsWorld->addConstraint(p2pCon, true);
			//p2pCon->setOverrideNumSolverIterations(60);
			mus->body0 = rbi;
		}

		//mus->motor = motor;
		//musCon->m_rbmuscles.push_back(mus);
		btVector3 transIns = movableBody->getCenterOfMassTransform() (mus->relInsertion);
		rbMuscles.push_back(mus);
		/*std::cout << "Original insertion: " << cvt(ins) << std::endl;
		std::cout << "Relative insertion: " << cvt(mus->relInsertion) << std::endl;
		std::cout << "Transformed insertion: " << cvt(transIns) << std::endl;*/
	}
	return rbMuscles;
}


btAlignedObjectArray<rbMuscle*> PhysicsEngine::createMusclesFrom2FiducialForSoftBody(btSoftBody* movableBody, std::string origin, std::string insertion)
{
	std::vector<FiducialMark> fidOrigin = loadFiducial(origin);
	std::vector<FiducialMark> fidInsertion = loadFiducial(insertion);
	btAlignedObjectArray<btRigidBody*> bodies;
	btAlignedObjectArray<rbMuscle*> rbMuscles;

	btCollisionShape* sph = new btSphereShape(.2f);
	mCollisionShapes.push_back(sph);

	for (int i = 0; i < fidOrigin.size(); i++)
	{
		std::string label = fidOrigin[i].label;
		btVector3 ori = fidOrigin[i].getVector3();
		btVector3 ins = fidInsertion[i].getVector3();

		rbMuscle *mus = new rbMuscle();
		mus->label = label;
		mus->orgin = ori;
		mus->insertion = ins;
		//mus->relInsertion = movableBody->getCenterOfMassTransform().inverse() (ins);
		mus->multiply = 0;
		mus->mDebugDraw = mDynamicsWorld->getDebugDrawer();

		//btRigidBody* rbi = createRigidBody(0.000001f, ins, sph, COL_NOTHING, muscleCollidesWith, false);
		//bodies.push_back(rbi);

		//mus->body0 = rbi;
		//mus->body1 = movableBody;

		

		

		std::vector<cNode> node = getClosetNodes(movableBody, ins, 1.f);
		//mus->node = node[0].index;

		//std::cout << "Node: " << mus->node << std::endl;
		rbMuscles.push_back(mus);
		
	}
	return rbMuscles;
}

sbMuscle* PhysicsEngine::createSBMuscleFrom2Fiducial(btSoftBody* softBody, std::string origin, std::string insertion)
{
	std::vector<FiducialMark> fidOrigin = loadFiducial(origin);
	std::vector<FiducialMark> fidInsertion = loadFiducial(insertion);

	sbMuscle * muscle = new sbMuscle();
	muscle->softbody = softBody;
	muscle->label = fidOrigin[0].label;
	muscle->mDebugDraw = mDynamicsWorld->getDebugDrawer();

	for (int i = 0; i < fidOrigin.size(); i++)
	{
		btVector3 ori = fidOrigin[i].getVector3();
		btVector3 ins = fidInsertion[i].getVector3();

		std::vector<cNode> node = getClosetNodes(softBody, ins, 1.f);
		if (node.size() > 0)
			muscle->insertions.push_back(&softBody->m_nodes[node[0].index]);

		btTransform trans = btTransform::getIdentity();
		trans.setOrigin(ins);
		btVector3 ori2 = trans.inverse() (ori);

		muscle->origins.push_back(ori2);

	}
	return muscle;
}


sbMuscle* PhysicsEngine::createSBMuscleFromFiducial(btSoftBody* softBody, std::string label, std::string origin)
{
	std::vector<FiducialMark> fidOrigin = loadFiducial(origin);

	sbMuscle * muscle = new sbMuscle();
	muscle->softbody = softBody;
	muscle->label = label;
	muscle->mDebugDraw = mDynamicsWorld->getDebugDrawer();

	for (int i = 0; i < fidOrigin.size(); i++)
	{
		btVector3 ori = fidOrigin[i].getVector3();

		
		btTransform trans = btTransform::getIdentity();
		trans.setOrigin(softBody->m_nodes[0].m_x);
		ori = trans.inverse() (ori);

		muscle->origins.push_back(ori);

	}
	return muscle;
}



sbMuscle* PhysicsEngine::createSBMuscle(btSoftBody* softBody, std::string label)
{
	sbMuscle * muscle = new sbMuscle();
	muscle->softbody = softBody;
	muscle->label = label;
	muscle->mDebugDraw = mDynamicsWorld->getDebugDrawer();
	return muscle;
}

static btAlignedObjectArray<btVector3> loadTetGenNodesFile(std::string filename)
{
	std::ifstream inputStream(filename);
	std::string lineStream;
	btAlignedObjectArray<btVector3> pos;

	int size, dimension;

	std::getline(inputStream, lineStream);
	sscanf_s(lineStream.c_str(), "%i %i", &size, &dimension);
	pos.resize(size);

	for (int i = 0; i < pos.size(); i++)
	{
		int index = 0;
		float x, y, z;
		std::getline(inputStream, lineStream);
		sscanf_s(lineStream.c_str(), "%d %f %f %f", &index, &x, &y, &z);

		pos[index].setX(btScalar(x));
		pos[index].setY(btScalar(y));
		pos[index].setZ(btScalar(z));
	}

	inputStream.close();
	return pos;

}

static std::vector<std::vector<int>> loadTetGenElementsFile(std::string filename)
{
	std::ifstream inputStream(filename);
	std::string lineStream;
	std::vector<std::vector<int>> pos;

	int size, dimension;

	std::getline(inputStream, lineStream);
	sscanf_s(lineStream.c_str(), "%i %i", &size, &dimension);
	pos.resize(size);

	/*cout << "Size: " << size << endl;
	cout << "Dimension: " << dimension << endl;*/

	for (int i = 0; i < pos.size(); i++)
	{
		int index = 0;
		int a, b, c, d;
		std::getline(inputStream, lineStream);
		sscanf_s(lineStream.c_str(), "%i %i %i %i %i", &index, &a, &b, &c, &d);

		pos[index].resize(4);
		pos[index][0] = a;
		pos[index][1] = b;
		pos[index][2] = c;
		pos[index][3] = d;
	}

	inputStream.close();
	return pos;

}

static std::vector<std::vector<int>> loadTetGenFacesFile(std::string filename)
{
	std::ifstream inputStream(filename);
	std::string lineStream;
	std::vector<std::vector<int>> pos;

	int size, dimension;

	std::getline(inputStream, lineStream);
	sscanf_s(lineStream.c_str(), "%i %i", &size, &dimension);
	pos.resize(size);

	/*cout << "Size: " << size << endl;
	cout << "Dimension: " << dimension << endl;*/

	for (int i = 0; i < pos.size(); i++)
	{
		int index = 0;
		int a, b, c, d;
		std::getline(inputStream, lineStream);
		sscanf_s(lineStream.c_str(), "%i %i %i %i %i", &index, &a, &b, &c, &d);

		pos[index].resize(4);
		pos[index][0] = a;
		pos[index][1] = b;
		pos[index][2] = c;
		pos[index][3] = d;
	}

	inputStream.close();
	return pos;

}

btSoftBody* PhysicsEngine::createFromTetGen(std::string ele, std::string face, std::string node, float mass, short group, short mask)
{
	std::ifstream elements_file(ele);
	std::ifstream faces_file(face);
	std::ifstream nodes_file(node);
	std::string content_ele((std::istreambuf_iterator<char>(elements_file)), std::istreambuf_iterator<char>());
	std::string content_face((std::istreambuf_iterator<char>(faces_file)), std::istreambuf_iterator<char>());
	std::string content_node((std::istreambuf_iterator<char>(nodes_file)), std::istreambuf_iterator<char>());




	btSoftBody* psb = btSoftBodyHelpers::CreateFromTetGenData(mSoftBodyWorldInfo, content_ele.c_str(), content_face.c_str(), content_node.c_str(), true, true, true);

	faces_file.clear();
	faces_file.seekg(0);
	if (faces_file)
	{
		std::string lineStream;
		btAlignedObjectArray<btVector3> pos;

		std::getline(faces_file, lineStream);


		int								nface = 0;
		sscanf_s(lineStream.c_str(), "%d %d", &nface);

		std::cout << "Faces Relida: " << nface << std::endl;


		for (int i = 0; i<nface; ++i)
		{
			int			index = 0;
			int			bound = 0;
			int			ni[3];
			std::getline(faces_file, lineStream);
			sscanf_s(lineStream.c_str(), "%d %d %d %d", &index, &ni[0], &ni[1], &ni[2]);
			/*sf >> index;
			sf >> ni[0]; sf >> ni[1]; sf >> ni[2];
			sf >> bound;*/
			psb->appendFace(ni[0], ni[1], ni[2]);
			//std::cout << "Faces: " << psb->m_faces.size() << std::endl;
			/*if (btetralinks)
			{
				psb->appendLink(ni[0], ni[1], 0, true);
				psb->appendLink(ni[1], ni[2], 0, true);
				psb->appendLink(ni[2], ni[0], 0, true);
			}*/
		}
	}

	//psb->setVolumeMass(0.01);


	/////fix one vertex
	////psb->setMass(0,0);
	////psb->setMass(10,0);
	////psb->setMass(20,0);
	//psb->m_cfg.piterations = 2;
	////psb->generateClusters(128);
	//psb->generateClusters(16);
	//psb->getCollisionShape()->setMargin(0.5);

	//psb->getCollisionShape()->setMargin(0.01);
	//psb->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS
		//+ btSoftBody::fCollision::CL_SELF + btSoftBody::fCollision::RVSmask
	//	;
	//psb->m_materials[0]->m_kLST = 0.5;
	psb->setTotalMass(mass);

	((btSoftRigidDynamicsWorld*)mDynamicsWorld)->addSoftBody(psb, group, mask);
	mSoftBodies.push_back(psb);

	return psb;

}

bool sortByDistance(const cNode &lhs, const cNode &rhs) { return lhs.distance < rhs.distance; }

std::vector<cNode> PhysicsEngine::getClosetNodes(btSoftBody* body, btVector3 point, float dis)
{
	std::vector<cNode> nodes;

	for (int i = 0; i < body->m_nodes.size(); i++)
	{
		//std::cout << "Distance: " << referencePoint.distance(body->m_nodes[i].m_x) << std::endl;
		float distance = (float) point.distance(body->m_nodes[i].m_x);
		if (distance < dis)
		{
			cNode node;
			node.index = i;
			node.distance = distance;
			//node.node = ;
			nodes.push_back(node);
		}
	}

	std::sort(nodes.begin(), nodes.end(), sortByDistance);

	return nodes;

}

btSoftBody* PhysicsEngine::createSoftBodyFromObjFile(std::string filename, float mass)
{
	std::string inputfile = filename;
	std::vector<tinyobj::shape_t> shapes;
	std::vector<tinyobj::material_t> materials;

	std::string err = tinyobj::LoadObj(shapes, materials, inputfile.c_str(), "data\\");

	if (!err.empty()) {
		std::cerr << err << std::endl;
		std::cout << "ARQUIVO NAO ENCONTRADO\n";
	}

	std::cout << "Num of shapes: " << shapes.size() << std::endl;
	std::cout << "Num of indices: " << shapes[0].mesh.indices.size() << std::endl;
	std::cout << "Num of postitions: " << shapes[0].mesh.positions.size() << std::endl;
	std::cout << "Num of normals: " << shapes[0].mesh.normals.size() << std::endl;
	std::cout << "Num of texcoor: " << shapes[0].mesh.texcoords.size() << std::endl;

	btAlignedObjectArray<btVector3> vtx;

	/*for (int i = 0; i < shapes[0].mesh.indices.size() ; i += 3)
	{
		std::cout << "Indice A: " << shapes[0].mesh.indices[i] << ", Indice B: " << shapes[0].mesh.indices[i + 1] << ", Indice C: " << shapes[0].mesh.indices[i + 2] << std::endl;
	}*/
	for (int i = 0; i < shapes[0].mesh.positions.size() / 3; i++)
	{
		float x = shapes[0].mesh.positions[3 * i + 0];
		float y = shapes[0].mesh.positions[3 * i + 1];
		float z = shapes[0].mesh.positions[3 * i + 2];
		//std::cout << "X: " << shapes[0].mesh.positions[3 * i + 0] << ", Y: " << shapes[0].mesh.positions[3 * i + 1] << ", Z: " << shapes[0].mesh.positions[3 * i + 2] << std::endl;
		vtx.push_back(btVector3(x, y, z));
	}

	size_t index_count = shapes[0].mesh.indices.size() / 3;

	btSoftBody* psb = new btSoftBody(&mSoftBodyWorldInfo, vtx.size(), &vtx[0], 0);
	

	std::cout << "Nodes count: " << psb->m_nodes.size() << std::endl;

	for (int i = 0; i < shapes[0].mesh.indices.size(); i += 3)
	{
		int a = shapes[0].mesh.indices[i];
		int b = shapes[0].mesh.indices[i + 1];
		int c = shapes[0].mesh.indices[i + 2];
		psb->appendLink(a, b);
		psb->appendLink(b, c);
		psb->appendLink(c, a);
		psb->appendFace(a, b, c);
	}
	
	psb->setTotalMass(mass);
	btSoftBodyHelpers::ReoptimizeLinkOrder(psb);

	getSoftRigidDynamicsWorld()->addSoftBody(psb, COL_DISC, discCollidesWith);

	mSoftBodies.push_back(psb);

	return psb;

}


btSoftBody* PhysicsEngine::createRetrodiscalTissue(std::string filename, btSoftBody* disc, int width, int height, bool bendingConstrainst, float maxPointdistance, int maxLinks)
{
	if (height == 0) height = width;
	std::vector<FiducialMark> tissueMarks = loadFiducial(filename);
	btSoftBody* tissue = btSoftBodyHelpers::CreatePatch(mSoftBodyWorldInfo,
		tissueMarks[0].getVector3(),
		tissueMarks[1].getVector3(),
		tissueMarks[2].getVector3(),
		tissueMarks[3].getVector3(),
		width,
		height,
		0, true);
	/*rtiss->setUserIndex(3);
	rtiss->setTotalMass(PhysicsEngine::DEFAULT_DISC_MASS);
	rtiss->generateClusters(0);*/
	if (bendingConstrainst)
	{
		btSoftBody::Material* m = tissue->appendMaterial();
		m->m_flags -= btSoftBody::fMaterial::DebugDraw;
		tissue->generateBendingConstraints(2, m);
	}
	btSoftBodyHelpers::ReoptimizeLinkOrder(tissue);

	btTransform ori = btTransform::getIdentity();
	ori.setOrigin(tissueMarks[2].getVector3());
	
	
	btAlignedObjectArray<btVector3> points;

	//std::cout << cvt(rtissueMarks[0].getVector3()) << std::endl;

	btScalar distance = tissueMarks[2].getVector3().distance2(tissueMarks[3].getVector3());

	float multiply = 1 / width;

	//points.push_back(tissueMarks[2].getVector3());
	for (int i = 0; i <= width; i++)
	{
		
		btScalar n = btScalar(1.f / width) * btScalar(i);

		///std::cout << "Multiplicador quando em " << i << ": " << n << std::endl;
		btTransform trans = ori;
		btVector3 p = trans.inverse() * tissueMarks[3].getVector3();
		/*btVector3 p1(p.getX() * n,
			p.getY() * n,
			p.getZ() * n);*/
		p = p * n;
		p = ori * p;
		points.push_back(p);
	}

	//float dis = distance;
	//std::cout << "Quantidade de pontos encontrados: " << points.size() << std::endl;

	/*tissue->generateClusters(0);
	btSoftBody::AJoint::Specs s;
	s.axis = btVector3(0, 0, 1);
	tissue->appendAngularJoint(s, tissue->m_clusters[tissue->m_clusters.size() - 1], disc);*/
	

	for (int i = 0; i < points.size(); i++)
	{
		std::vector<cNode> nodeDisc = getClosetNodes(disc, points[i], maxPointdistance);
		std::vector<cNode> nodeTissue = getClosetNodes(tissue, points[i], maxPointdistance);
		int idt = 0;
		int idd = 0;

		if (nodeDisc.size() > 0 && nodeTissue.size() > 0)
		{
			int size = 0;
			if (nodeDisc.size() >= maxLinks) size = maxLinks; else size = nodeDisc.size();
			for (int j = 0; j < size; j++)
			{
				idt = nodeTissue[0].index;
				idd = nodeDisc[j].index;
				btSoftBody::Material* m = tissue->appendMaterial();
				m -= btSoftBody::fMaterial::DebugDraw;
				tissue->appendLink(&tissue->m_nodes[idt], &disc->m_nodes[idd]);
			}
			// Invert to connect the disc to muscles
			/*if (nodeTissue.size() >= maxLinks) size = maxLinks; else size = nodeTissue.size();
			for (int j = 0; j < size; j++)
			{
				idt = nodeTissue[j].index;
				idd = nodeDisc[0].index;
				disc->appendLink(&disc->m_nodes[idd], &tissue->m_nodes[idt]);
			}*/
		}

	}


	// Fixed points
	for (int i = 0; i < width; i++)
	{
		tissue->setMass(i, 0);
	}

	// Fixed points
	/*for (int i = numVertices + 1; i < numVertices * 2; i++)
	{
		tissue->setMass(i, 0);
	}*/


	return tissue;

}


btSoftBody* PhysicsEngine::createAttachTissue(std::string filename, btSoftBody* disc, btRigidBody* body, int numVertices, float maxPointdistance, int maxLinks)
{
	std::vector<FiducialMark> tissueMarks = loadFiducial(filename);
	btSoftBody* tissue = btSoftBodyHelpers::CreatePatch(mSoftBodyWorldInfo,
		tissueMarks[0].getVector3(),
		tissueMarks[1].getVector3(),
		tissueMarks[2].getVector3(),
		tissueMarks[3].getVector3(),
		numVertices,
		numVertices,
		0, true);

	btSoftBody::Material* m = tissue->appendMaterial();
	m->m_flags -= btSoftBody::fMaterial::DebugDraw;
	tissue->generateBendingConstraints(2, m);
	btSoftBodyHelpers::ReoptimizeLinkOrder(tissue);

	btTransform ori = btTransform::getIdentity();
	ori.setOrigin(tissueMarks[2].getVector3());


	btAlignedObjectArray<btVector3> points;

	btScalar distance = tissueMarks[2].getVector3().distance2(tissueMarks[3].getVector3());

	float multiply = 1 / numVertices;

	//points.push_back(tissueMarks[2].getVector3());
	for (int i = 0; i <= numVertices; i++)
	{
		btScalar n = btScalar(1.f / numVertices) * btScalar(i);
		btTransform trans = ori;
		btVector3 p = trans.inverse() * tissueMarks[3].getVector3();
		p = p * n;
		p = ori * p;
		points.push_back(p);
	}

	for (int i = 0; i < points.size(); i++)
	{
		std::vector<cNode> nodeDisc = getClosetNodes(disc, points[i], maxPointdistance);
		std::vector<cNode> nodeTissue = getClosetNodes(tissue, points[i], maxPointdistance);
		int idt = 0;
		int idd = 0;

		if (nodeDisc.size() > 0 && nodeTissue.size() > 0)
		{
			int size = 0;
			if (nodeDisc.size() >= maxLinks) size = maxLinks; else size = nodeDisc.size();
			for (int j = 0; j < size; j++)
			{
				idt = nodeTissue[0].index;
				idd = nodeDisc[j].index;
				tissue->appendLink(&tissue->m_nodes[idt], &disc->m_nodes[idd]);
			}
			// Invert to connect the disc to muscles
			if (nodeTissue.size() >= maxLinks) size = maxLinks; else size = nodeTissue.size();
			for (int j = 0; j < size; j++)
			{
				idt = nodeTissue[j].index;
				idd = nodeDisc[0].index;
				disc->appendLink(&disc->m_nodes[idd], &tissue->m_nodes[idt]);
			}
		}

	}


	// Fixed points
	for (int i = 0; i < numVertices; i++)
	{
		tissue->appendAnchor(i, body);
	}

	/*for (int i = 0; i < tissue->m_nodes.size(); i++)
	{
		std::cout << "mass of node " << i << ": " << tissue->m_nodes[i].m_im << std::endl;
		tissue->staticSolve(100);
		tissue->integrateMotion();
	}*/


	return tissue;

}

void PhysicsEngine::alignAxis(btRigidBody* bodyA, btRigidBody* bodyB, btTransform &bodyATransform, btTransform &bodyBTransform)
{
	// Get Body B position relative to Body A
	btVector3 vectorBinA = bodyA->getCenterOfMassTransform().inverse() (bodyB->getCenterOfMassTransform().getOrigin());
	double xB, yB, zB;
	xB = vectorBinA.getX(); // atan2 2st param
	yB = vectorBinA.getY();
	zB = vectorBinA.getZ(); // atan2 1st param

	//std::cout << "Relative Location around Y axis (1st param, 2st param): " << zB << ", " << xB << std::endl;
	double rotYA = std::atan2(zB, -xB);

	//std::cout << "Y Angle rotation: " << rotYA << std::endl;

	btQuaternion bodyARoration = bodyATransform.getRotation();


	// Get Body A position relative to Body B
	btVector3 vectorAinB = bodyB->getCenterOfMassTransform().inverse() (bodyA->getCenterOfMassTransform().getOrigin());
	double xA, yA, zA;
	xA = vectorAinB.getX(); // atan2 2st param
	yA = vectorAinB.getY();
	zA = vectorAinB.getZ(); // atan2 1st param

	//std::cout << "Relative Location around Y axis (1st param, 2st param): " << zA << ", " << xA << std::endl;
	std::cout << "Body A position relative to Body B: (" << xA << ", " << yA << ", " << zA << ")" << std::endl;
	double rotYB = -std::atan2(zA, xA);
	//double rotZB = -std::atan2(-yA, zA); // y < 0, z > 0
	//double rotZB = std::atan2(yA, -zA); // y > 0, z < 0
	double rotZB;

	if (yA < 0 && zA > 0)
	{
		rotZB = -std::atan2(-yA, zA); // y < 0, z > 0
	}
	else if (yA > 0 && zA < 0)
	{
		rotZB = std::atan2(yA, -zA); // y > 0, z < 0
	}
	else
	{
		//rotZB = -3.14/3.9; // y < 0, z < 0
		double xAAl = std::sqrt(yA*yA + zA*zA);
		rotZB = std::atan2(yA, xAAl);
		std::cout << "Z Angle rotation: " << rotZB << std::endl;
	}


	//std::cout << "Y Angle rotation: " << rotYA << std::endl;

	btQuaternion bodyBRoration = bodyBTransform.getRotation();


	bodyBRoration.setEuler(rotYB, 0, rotZB);
	bodyBTransform.setRotation(bodyBRoration);

	//std::cout << "Y Angle rotation: " << rotYA << std::endl;



	bodyARoration.setEuler(rotYA, 0, rotZB);
	bodyATransform.setRotation(bodyARoration);
}

simulationLog* PhysicsEngine::loadLogPoints(std::string filename, btRigidBody* body, int timeStep)
{
	simulationLog* mySimLog = new simulationLog();
	mySimLog->body = body;
	std::vector<FiducialMark> logMarks = loadFiducial(filename);
	//std::cout << "Pontos de controle encontrados: " << logMarks.size() << std::endl;
	for (int i = 0; i < logMarks.size(); i++)
	{
		btVector3 local = body->getWorldTransform().inverse() * logMarks[i].getVector3();
		//std::cout << "Local adicionado: " << cvt(local) << std::endl;
		controlPoint cp;
		cp.label = logMarks[i].label;
		cp.point = local;
		mySimLog->pointsInLocal.push_back(cp);
	}
	return mySimLog;
}


void PhysicsEngine::startRecording(simulationLog* sim, std::string desc, bool position)
{
	std::cout << "Logging position...\n";
	if (!sim->recording)
	{
		if (position)
		{
			std::ofstream mFile(sim->filename, std::ios::app);
			mFile << desc;
			btAlignedObjectArray<controlPoint> p = sim->getPointsInWorld();
			for (int i = 0; i < p.size(); i++)
			{
				mFile << p[i].getLogMessage();
			}
			mFile << sim->getTransformationMatrix();
			mFile.close();
			
			std::ofstream mFile2("data\\muscle_activation.txt", std::ios::app);
			std::vector<appliedForce> forces = mMuscleControl->getAppliedForces();
			for (int i = 0; i < forces.size(); i++)
			{
				mFile2 << desc;
				mFile2 << forces[i].label;
				if (i < forces.size() - 1) mFile2 << ","; else mFile2 << "\n";
			}
			for (int i = 0; i < forces.size(); i++)
			{
				//std::cout << "Muscle " << forces[i].label << ": " << forces[i].lenght / 10000. << std::endl;
				mFile2 << forces[i].lenght / 10000.;
				if (i < forces.size() - 1) mFile2 << ","; else mFile2 << "\n";
			}
			mFile2.close();
			if (mSoftBodyControl)
			{
				recordDiscShape(mSoftBodyControl->getSoftBody(1), "data\\right_disc_shape.obj");
				recordDiscShape(mSoftBodyControl->getSoftBody(2), "data\\left_disc_shape.obj");


				/*std::ofstream mFile3("data\\right_disc_state.obj");
				btSoftBody *sb = mSoftBodyControl->getSoftBody(1);

				for (int k = 0; k < sb->m_nodes.size(); k++)
				{
					mFile3 << "v " << sb->m_nodes[k].m_x.getX() << " " << sb->m_nodes[k].m_x.getY() << " " << sb->m_nodes[k].m_x.getZ() << std::endl;

				}



				for (int i = 0; i < sb->m_faces.size(); i++)
				{
					std::cout << "f";
					mFile3 << "f";
					for (int j = 0; j < 3; j++)
					{
						
						btVector3 nLoc = sb->m_faces[i].m_n[j]->m_x;
						for (int k = 0; k < sb->m_nodes.size(); k++)
						{
							if (sb->m_nodes[k].m_x == nLoc) 
							{
								int f = k + 1;
								std::cout << " " << f;
								mFile3 << " " << f;
							}
							
						}
						
					}
					std::cout << "\n";
					mFile3 << std::endl;
				}

				mFile3.close();*/
				
				/*std::cout << "Right Disc Volume: " << mSoftBodyControl->getSoftBody(1)->getVolume() << std::endl;
				std::cout << "Left Disc Volume: " << mSoftBodyControl->getSoftBody(2)->getVolume() << std::endl;
				std::cout << "Left Disc Location: " << mSoftBodyControl->getSoftBody(1)->getInterpolationWorldTransform().getOrigin().getZ() << std::endl;*/
			}
		}
		else
		{
			sim->recording = true;
			boost::thread* t =
				new boost::thread(&PhysicsEngine::recordMovements, this, sim, desc);
		}
	}
}


inline bool checkLabels(std::vector<std::string> labels, std::vector<appliedForce> forces)
{
	if (labels.size() != forces.size())
	{
		std::cout << "AS LABELS SAO COMPLETAMENTE DIFERENTES\n";
		return false;
	}
	for (int i = 0; i < labels.size(); i++)
	{
		if (labels[i].compare(forces[i].label) != 0)
		{
			std::cout << "AS LABELS SAO COMPLETAMENTE DIFERENTES\n";
			return false;
		}
	}
	return true;
}

inline std::vector<std::string> getLabels(std::vector<appliedForce> forces)
{
	std::vector<std::string> labels;
	for (int i = 0; i < forces.size(); i++)
	{
		labels.push_back(forces[i].label);
	}
	return labels;
}

void PhysicsEngine::recordMovements(simulationLog* sim, std::string desc)
{
	std::ofstream mFile(sim->filename, std::ios::app);
	mFile << desc;

	std::ofstream mFile2("data\\muscle_activation.txt", std::ios::app);
	mFile2 << desc;

	std::vector<std::string> labesl;

	std::cout << "Starting record...\n";
	int sequence = 0;
	while (sim->recording)
	{
		// Record disc positions
		sequence++;
		btAlignedObjectArray<controlPoint> p = sim->getPointsInWorld();
		std::vector<appliedForce> forces = mMuscleControl->getAppliedForces();
		for (int i = 0; i < p.size(); i++)
		{
			mFile << sequence << "," << p[i].getLogMessage();
		}
		boost::this_thread::sleep(boost::posix_time::milliseconds(sim->timeStep));


		// Record labels once
		if (forces.size() > 0)
		{
			if (!checkLabels(labesl, forces))
			{
				labesl = getLabels(forces);
				mFile2 << sequence << ",";
				for (int i = 0; i < labesl.size(); i++)
				{
					mFile2 << labesl[i];
					if (i < labesl.size() - 1) mFile2 << ","; else mFile2 << "\n";
				}
			}


			mFile2 << sequence << ",";
			for (int i = 0; i < forces.size(); i++)
			{
				mFile2 << "," << forces[i].lenght / 10000.;
				if (i < forces.size() - 1) mFile2 << ","; else mFile2 << "\n";
			}
		}

	}
	mFile.close();
	mFile2.close();
	std::cout << "Stoping record...\n";
	/*std::cout << "FUNCIONOU CABRA DA PESTE\n";
	std::ofstream mFile("data\\simulationLogs.txt", std::ios::app);
	while (mSimLog->recording)
	{
		controlPoint cp;
		btAlignedObjectArray<controlPoint> pointsInWorld;
		for (int i = 0; i < mSimLog->pointsInLocal.size(); i++)
		{
			
			cp.label = mSimLog->pointsInLocal[i].label;
			cp.point = mSimLog->body->getWorldTransform() * mSimLog->pointsInLocal[i].point;
			cp.x = cp.point.getX();
			cp.y = cp.point.getY();
			cp.z = cp.point.getZ();
			pointsInWorld.push_back(cp);
		}

		std::string mlog = cp.label + ": " + std::to_string(cp.x) + ", " +
			std::to_string(cp.y) + ", " +
			std::to_string(cp.z) + "\n";
		mFile << mlog;
		std::cout << "Local: " << cvt(mSimLog->pointsInLocal[0].point) << std::endl;
		btVector3 world = mSimLog->body->getWorldTransform() (mSimLog->pointsInLocal[0].point);
		std::cout << "World: " << cvt(world) << std::endl;
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
	}
	mFile.close();
	std::cout << "FINALIZANDO A THREAD *RRA\n";*/
}
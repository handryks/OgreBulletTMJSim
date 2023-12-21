/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "btBulletDynamicsCommon.h"
#include "LinearMath/btQuickprof.h"
#include "LinearMath/btIDebugDraw.h"

#include <stdio.h> //printf debugging
#include <algorithm>

class btCollisionShape;

#include "LinearMath/btAlignedObjectArray.h"
#include "btBulletCollisionCommon.h"
#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"  // for setSplitIslands()

#include "ParallelFor.h"

#include "BulletSoftBody\btSoftBodySolvers.h"
#include "BulletSoftBody\btDefaultSoftBodySolver.h"


#define USE_PARALLEL_NARROWPHASE 1  // detect collisions in parallel
#define USE_PARALLEL_ISLAND_SOLVER 1   // solve simulation islands in parallel
#define USE_PARALLEL_CREATE_PREDICTIVE_CONTACTS 1
#define USE_PARALLEL_INTEGRATE_TRANSFORMS 1
#define USE_PARALLEL_PREDICT_UNCONSTRAINED_MOTION 1

#if defined (_MSC_VER) && _MSC_VER >= 1600
// give us a compile error if any signatures of overriden methods is changed
#define BT_OVERRIDE override
#else
#define BT_OVERRIDE
#endif



#if USE_PARALLEL_NARROWPHASE

class MyCollisionDispatcher : public btCollisionDispatcher
{
public:
	MyCollisionDispatcher(btCollisionConfiguration* config) : btCollisionDispatcher(config)
	{
	}

	virtual ~MyCollisionDispatcher()
	{
	}

	struct Updater
	{
		btBroadphasePair* mPairArray;
		btNearCallback mCallback;
		btCollisionDispatcher* mDispatcher;
		const btDispatcherInfo* mInfo;

		Updater()
		{
			mPairArray = NULL;
			mCallback = NULL;
			mDispatcher = NULL;
			mInfo = NULL;
		}
		void forLoop(int iBegin, int iEnd) const
		{
			for (int i = iBegin; i < iEnd; ++i)
			{
				btBroadphasePair* pair = &mPairArray[i];
				mCallback(*pair, *mDispatcher, *mInfo);
			}
		}
	};

	virtual void dispatchAllCollisionPairs(btOverlappingPairCache* pairCache, const btDispatcherInfo& info, btDispatcher* dispatcher) BT_OVERRIDE
	{
		int grainSize = 40;  // iterations per task
		int pairCount = pairCache->getNumOverlappingPairs();
		Updater updater;
		updater.mCallback = getNearCallback();
		updater.mPairArray = pairCount > 0 ? pairCache->getOverlappingPairArrayPtr() : NULL;
		updater.mDispatcher = this;
		updater.mInfo = &info;

		btPushThreadsAreRunning();
		parallelFor(0, pairCount, grainSize, updater);
		btPopThreadsAreRunning();
	}
};

#endif


#if USE_PARALLEL_ISLAND_SOLVER
///
/// MyConstraintSolverPool - masquerades as a constraint solver, but really it is a threadsafe pool of them.
///
///  Each solver in the pool is protected by a mutex.  When solveGroup is called from a thread,
///  the pool looks for a solver that isn't being used by another thread, locks it, and dispatches the
///  call to the solver.
///  So long as there are at least as many solvers as there are hardware threads, it should never need to
///  spin wait.
///
class MyConstraintSolverPool : public btConstraintSolver
{
	const static size_t kCacheLineSize = 128;
	struct ThreadSolver
	{
		btConstraintSolver* solver;
		btMutex mutex;
		char _cachelinePadding[kCacheLineSize - sizeof(btMutex) - sizeof(void*)];  // keep mutexes from sharing a cache line
	};
	btAlignedObjectArray<ThreadSolver> m_solvers;
	btConstraintSolverType m_solverType;

	ThreadSolver* getAndLockThreadSolver()
	{
		while (true)
		{
			for (int i = 0; i < m_solvers.size(); ++i)
			{
				ThreadSolver& solver = m_solvers[i];
				if (btMutexTryLock(&solver.mutex))
				{
					return &solver;
				}
			}
		}
		return NULL;
	}
	void init(btConstraintSolver** solvers, int numSolvers)
	{
		m_solverType = BT_SEQUENTIAL_IMPULSE_SOLVER;
		m_solvers.resize(numSolvers);
		for (int i = 0; i < numSolvers; ++i)
		{
			m_solvers[i].solver = solvers[i];
		}
		if (numSolvers > 0)
		{
			m_solverType = solvers[0]->getSolverType();
		}
	}
public:
	// create the solvers for me
	explicit MyConstraintSolverPool(int numSolvers)
	{
		btAlignedObjectArray<btConstraintSolver*> solvers;
		solvers.reserve(numSolvers);
		for (int i = 0; i < numSolvers; ++i)
		{
			btConstraintSolver* solver = new btSequentialImpulseConstraintSolver();
			solvers.push_back(solver);
		}
		init(&solvers[0], numSolvers);
	}

	// pass in fully constructed solvers (destructor will delete them)
	MyConstraintSolverPool(btConstraintSolver** solvers, int numSolvers)
	{
		init(solvers, numSolvers);
	}
	virtual ~MyConstraintSolverPool()
	{
		// delete all solvers
		for (int i = 0; i < m_solvers.size(); ++i)
		{
			ThreadSolver& solver = m_solvers[i];
			delete solver.solver;
			solver.solver = NULL;
		}
	}

	//virtual void prepareSolve( int /* numBodies */, int /* numManifolds */ ) { ; } // does nothing

	///solve a group of constraints
	virtual btScalar solveGroup(btCollisionObject** bodies,
		int numBodies,
		btPersistentManifold** manifolds,
		int numManifolds,
		btTypedConstraint** constraints,
		int numConstraints,
		const btContactSolverInfo& info,
		btIDebugDraw* debugDrawer,
		btDispatcher* dispatcher
		)
	{
		ThreadSolver* solver = getAndLockThreadSolver();
		solver->solver->solveGroup(bodies, numBodies, manifolds, numManifolds, constraints, numConstraints, info, debugDrawer, dispatcher);
		btMutexUnlock(&solver->mutex);
		return 0.0f;
	}

	//virtual void allSolved( const btContactSolverInfo& /* info */, class btIDebugDraw* /* debugDrawer */ ) { ; } // does nothing

	///clear internal cached data and reset random seed
	virtual	void reset()
	{
		for (int i = 0; i < m_solvers.size(); ++i)
		{
			ThreadSolver& solver = m_solvers[i];
			btMutexLock(&solver.mutex);
			solver.solver->reset();
			btMutexUnlock(&solver.mutex);
		}
	}

	virtual btConstraintSolverType getSolverType() const
	{
		return m_solverType;
	}
};

struct UpdateIslandDispatcher
{
	btAlignedObjectArray<btSimulationIslandManager::Island*>* islandsPtr;
	btSimulationIslandManager::IslandCallback* callback;

	void forLoop(int iBegin, int iEnd) const
	{
		for (int i = iBegin; i < iEnd; ++i)
		{
			btSimulationIslandManager::Island* island = (*islandsPtr)[i];
			btPersistentManifold** manifolds = island->manifoldArray.size() ? &island->manifoldArray[0] : NULL;
			btTypedConstraint** constraintsPtr = island->constraintArray.size() ? &island->constraintArray[0] : NULL;
			callback->processIsland(&island->bodyArray[0],
				island->bodyArray.size(),
				manifolds,
				island->manifoldArray.size(),
				constraintsPtr,
				island->constraintArray.size(),
				island->id
				);
		}
	}
};

void parallelIslandDispatch(btAlignedObjectArray<btSimulationIslandManager::Island*>* islandsPtr, btSimulationIslandManager::IslandCallback* callback)
{
	int grainSize = 1;  // iterations per task
	UpdateIslandDispatcher dispatcher;
	dispatcher.islandsPtr = islandsPtr;
	dispatcher.callback = callback;
	btPushThreadsAreRunning();
	parallelFor(0, islandsPtr->size(), grainSize, dispatcher);
	btPopThreadsAreRunning();
}
#endif //#if USE_PARALLEL_ISLAND_SOLVER


///
/// MyDiscreteDynamicsWorld
///
///  Should function exactly like btDiscreteDynamicsWorld.
///  3 methods that iterate over all of the rigidbodies can run in parallel:
///     - predictUnconstraintMotion
///     - integrateTransforms
///     - createPredictiveContacts
///
ATTRIBUTE_ALIGNED16(class) MyDiscreteDynamicsWorld : public btDiscreteDynamicsWorld
{
	typedef btDiscreteDynamicsWorld ParentClass;

protected:
#if USE_PARALLEL_PREDICT_UNCONSTRAINED_MOTION
	struct UpdaterUnconstrainedMotion
	{
		btScalar timeStep;
		btRigidBody** rigidBodies;

		void forLoop(int iBegin, int iEnd) const
		{
			for (int i = iBegin; i < iEnd; ++i)
			{
				btRigidBody* body = rigidBodies[i];
				if (!body->isStaticOrKinematicObject())
				{
					//don't integrate/update velocities here, it happens in the constraint solver
					body->applyDamping(timeStep);
					body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
				}
			}
		}
	};

	virtual void predictUnconstraintMotion(btScalar timeStep) BT_OVERRIDE
	{
		BT_PROFILE("predictUnconstraintMotion");
		int grainSize = 50;  // num of iterations per task for TBB
		int bodyCount = m_nonStaticRigidBodies.size();
		UpdaterUnconstrainedMotion update;
		update.timeStep = timeStep;
		update.rigidBodies = bodyCount ? &m_nonStaticRigidBodies[0] : NULL;
		btPushThreadsAreRunning();
		parallelFor(0, bodyCount, grainSize, update);
		btPopThreadsAreRunning();
	}
#endif // #if USE_PARALLEL_PREDICT_UNCONSTRAINED_MOTION

#if USE_PARALLEL_CREATE_PREDICTIVE_CONTACTS
	struct UpdaterCreatePredictiveContacts
	{
		btScalar timeStep;
		btRigidBody** rigidBodies;
		MyDiscreteDynamicsWorld* world;

		void forLoop(int iBegin, int iEnd) const
		{
			world->createPredictiveContactsInternal(&rigidBodies[iBegin], iEnd - iBegin, timeStep);
		}
	};

	virtual void createPredictiveContacts(btScalar timeStep)
	{
		int grainSize = 50;  // num of iterations per task for TBB or OPENMP
		if (int bodyCount = m_nonStaticRigidBodies.size())
		{
			UpdaterCreatePredictiveContacts update;
			update.world = this;
			update.timeStep = timeStep;
			update.rigidBodies = &m_nonStaticRigidBodies[0];
			btPushThreadsAreRunning();
			parallelFor(0, bodyCount, grainSize, update);
			btPopThreadsAreRunning();
		}
	}
#endif // #if USE_PARALLEL_CREATE_PREDICTIVE_CONTACTS

#if USE_PARALLEL_INTEGRATE_TRANSFORMS
	struct UpdaterIntegrateTransforms
	{
		btScalar timeStep;
		btRigidBody** rigidBodies;
		const MyDiscreteDynamicsWorld* world;

		void forLoop(int iBegin, int iEnd) const
		{
			world->integrateTransformsInternal(&rigidBodies[iBegin], iEnd - iBegin, timeStep);
		}
	};

	virtual void integrateTransforms(btScalar timeStep) BT_OVERRIDE
	{
		BT_PROFILE("integrateTransforms");
		int grainSize = 50;  // num of iterations per task for TBB or OPENMP
		if (int bodyCount = m_nonStaticRigidBodies.size())
		{
			UpdaterIntegrateTransforms update;
			update.world = this;
			update.timeStep = timeStep;
			update.rigidBodies = &m_nonStaticRigidBodies[0];
			btPushThreadsAreRunning();
			parallelFor(0, bodyCount, grainSize, update);
			btPopThreadsAreRunning();
		}
	}
#endif // #if USE_PARALLEL_INTEGRATE_TRANSFORMS

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	MyDiscreteDynamicsWorld(btDispatcher* dispatcher,
		btBroadphaseInterface* pairCache,
		btConstraintSolver* constraintSolver,
		btCollisionConfiguration* collisionConfiguration
		) :
		btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration)
	{
	}

};


///
/// MyDiscreteDynamicsWorld
///
///  Should function exactly like btDiscreteDynamicsWorld.
///  3 methods that iterate over all of the rigidbodies can run in parallel:
///     - predictUnconstraintMotion
///     - integrateTransforms
///     - createPredictiveContacts
///
ATTRIBUTE_ALIGNED16(class) MySoftRigidDynamicsWorld : public btSoftRigidDynamicsWorld
{
	typedef btDiscreteDynamicsWorld ParentClass;

	/*btSoftBodyArray	m_softBodies;
	int				m_drawFlags;
	bool			m_drawNodeTree;
	bool			m_drawFaceTree;
	bool			m_drawClusterTree;
	btSoftBodyWorldInfo m_sbi;
	///Solver classes that encapsulate multiple soft bodies for solving
	btSoftBodySolver *m_softBodySolver;
	bool			m_ownsSolver;*/

protected:
#if USE_PARALLEL_PREDICT_UNCONSTRAINED_MOTION
	struct UpdaterUnconstrainedMotion
	{
		btScalar timeStep;
		btRigidBody** rigidBodies;

		void forLoop(int iBegin, int iEnd) const
		{
			for (int i = iBegin; i < iEnd; ++i)
			{
				btRigidBody* body = rigidBodies[i];
				if (!body->isStaticOrKinematicObject())
				{
					//don't integrate/update velocities here, it happens in the constraint solver
					body->applyDamping(timeStep);
					body->predictIntegratedTransform(timeStep, body->getInterpolationWorldTransform());
				}
			}
		}
	};

	virtual void predictUnconstraintMotion(btScalar timeStep) BT_OVERRIDE
	{
		BT_PROFILE("predictUnconstraintMotion");
		int grainSize = 50;  // num of iterations per task for TBB
		int bodyCount = m_nonStaticRigidBodies.size();
		UpdaterUnconstrainedMotion update;
		update.timeStep = timeStep;
		update.rigidBodies = bodyCount ? &m_nonStaticRigidBodies[0] : NULL;
		btPushThreadsAreRunning();
		parallelFor(0, bodyCount, grainSize, update);
		//m_softBodySolver->predictMotion(float(timeStep)); // Added for softbody
		btPopThreadsAreRunning();
	}
#endif // #if USE_PARALLEL_PREDICT_UNCONSTRAINED_MOTION

#if USE_PARALLEL_CREATE_PREDICTIVE_CONTACTS
	struct UpdaterCreatePredictiveContacts
	{
		btScalar timeStep;
		btRigidBody** rigidBodies;
		MySoftRigidDynamicsWorld* world;

		void forLoop(int iBegin, int iEnd) const
		{
			world->createPredictiveContactsInternal(&rigidBodies[iBegin], iEnd - iBegin, timeStep);
		}
	};

	virtual void createPredictiveContacts(btScalar timeStep)
	{
		int grainSize = 50;  // num of iterations per task for TBB or OPENMP
		if (int bodyCount = m_nonStaticRigidBodies.size())
		{
			UpdaterCreatePredictiveContacts update;
			update.world = this;
			update.timeStep = timeStep;
			update.rigidBodies = &m_nonStaticRigidBodies[0];
			btPushThreadsAreRunning();
			parallelFor(0, bodyCount, grainSize, update);
			btPopThreadsAreRunning();
		}
	}
#endif // #if USE_PARALLEL_CREATE_PREDICTIVE_CONTACTS

#if USE_PARALLEL_INTEGRATE_TRANSFORMS
	struct UpdaterIntegrateTransforms
	{
		btScalar timeStep;
		btRigidBody** rigidBodies;
		const MySoftRigidDynamicsWorld* world;

		void forLoop(int iBegin, int iEnd) const
		{
			world->integrateTransformsInternal(&rigidBodies[iBegin], iEnd - iBegin, timeStep);
		}
	};

	virtual void integrateTransforms(btScalar timeStep) BT_OVERRIDE
	{
		BT_PROFILE("integrateTransforms");
		int grainSize = 50;  // num of iterations per task for TBB or OPENMP
		if (int bodyCount = m_nonStaticRigidBodies.size())
		{
			UpdaterIntegrateTransforms update;
			update.world = this;
			update.timeStep = timeStep;
			update.rigidBodies = &m_nonStaticRigidBodies[0];
			btPushThreadsAreRunning();
			parallelFor(0, bodyCount, grainSize, update);
			btPopThreadsAreRunning();
		}
	}
#endif // #if USE_PARALLEL_INTEGRATE_TRANSFORMS

public:
	BT_DECLARE_ALIGNED_ALLOCATOR();

	MySoftRigidDynamicsWorld(btDispatcher* dispatcher,
		btBroadphaseInterface* pairCache,
		btConstraintSolver* constraintSolver,
		btCollisionConfiguration* collisionConfiguration,
		btSoftBodySolver *softBodySolver
		) :
		//btDiscreteDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration)
		btSoftRigidDynamicsWorld(dispatcher, pairCache, constraintSolver, collisionConfiguration, softBodySolver)
	{
		/*if (!m_softBodySolver)
		{
			void* ptr = btAlignedAlloc(sizeof(btDefaultSoftBodySolver), 16);
			m_softBodySolver = new(ptr)btDefaultSoftBodySolver();
			m_ownsSolver = true;
		}

		m_drawFlags = fDrawFlags::Std;
		m_drawNodeTree = true;
		m_drawFaceTree = false;
		m_drawClusterTree = false;
		m_sbi.m_broadphase = pairCache;
		m_sbi.m_dispatcher = dispatcher;
		m_sbi.m_sparsesdf.Initialize();
		m_sbi.m_sparsesdf.Reset();

		m_sbi.air_density = (btScalar)1.2;
		m_sbi.water_density = 0;
		m_sbi.water_offset = 0;
		m_sbi.water_normal = btVector3(0, 0, 0);
		m_sbi.m_gravity.setValue(0, -10, 0);

		m_sbi.m_sparsesdf.Initialize();*/
	}

};

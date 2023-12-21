#pragma once

#include <btBulletDynamicsCommon.h>
#include <CEGUI\CEGUI.h>
#include "DefaultControl.h"
#include <mutex>              // std::mutex, std::unique_lock
#include <condition_variable> // std::condition_variable
#include <boost/thread/thread.hpp>


struct rbMuscle;
struct sbMuscle;
struct genericMuscle;
struct muscleMotor;
struct appliedForce;

//struct LinkedScroll
//{
//	int groupID;
//	int scrollID1;
//	int scrollID2;
//	CEGUI::Scrollbar* scroll1;
//	CEGUI::Scrollbar* scroll2;
//	bool linked;
//};

class MuscleControl : public DefaultControl
{
private:
	bool allMotorsActive;
	bool activeRigid = true;
	bool activeSoft = true;
	bool drawForces;
	bool adaptiveForceLines;
	float softBodyFactor = 0.00025f;
	boost::thread_group gThreads;

public:
	std::mutex mtx;
	std::condition_variable cv;

	bool runPhysics;

	std::string MOTOR_ACTIVATED_MSG;
	std::string THREAD_INITIALIZED_MSG;
	std::string ADD_FORCE_CHANGE_MSG;
	std::string MOTOR_FINALIZED_MSG;

	std::vector<std::string> fileNames;

	const int size = 9;
	static std::string labels[]; 

	MuscleControl();
	~MuscleControl();

	btAlignedObjectArray<rbMuscle*> mRigidBodyMuscles;
	btAlignedObjectArray<sbMuscle*> mSoftBodyMuscles;
	void applyAllForces();
	void activateMotor(rbMuscle *muscle);
	void activateMotor2(rbMuscle *muscle);
	void activateMotor3(genericMuscle *muscle);
	void disableMotors();
	void deleteMuscles();
	void activeMotors();
	void changeActivationFactor(int group, int id, float value);

	void registerMotion(std::string file);

	void assignControls();
	void assignIds();
	bool getScrollEvent(const CEGUI::EventArgs &e);
	bool onDoubleVerticalScrollPositionChanged(const CEGUI::EventArgs &e);
	bool onDoubleHorizontalScrollPositionChanged(const CEGUI::EventArgs &e);
	bool onButtonClicked(const CEGUI::EventArgs &e);
	bool onEditBoxTextAccepted(const CEGUI::EventArgs &e);
	bool onCheckBoxChanged(const CEGUI::EventArgs &e);

	btScalar calculeAppliedForce(rbMuscle muscle);
	btVector3 calculeForceInWorldPosition(rbMuscle* muscle);
	btVector3 calculeSoftBodyForce(btVector3 currentNodePos, btVector3 nextNodePos, btScalar multiply);
	void addMuscles(btAlignedObjectArray<rbMuscle*> muscles);
	btAlignedObjectArray<muscleMotor> loadMotorsFromFile(std::string filename);

	void moveScrollUp(CEGUI::Scrollbar* bar);
	void moveScrollDown(CEGUI::Scrollbar* bar);

	void toggleActiveMotors()
	{
		if (allMotorsActive)
		{
			allMotorsActive = false;
			disableMotors();
		}
		else
		{
			allMotorsActive = true;
			activeMotors();
		}
	}

	void toggleDrawForces()
	{
		if (drawForces) drawForces = false; else drawForces = true;
	}

	void toggleAdaptiveForceLines()
	{
		if (adaptiveForceLines) adaptiveForceLines = false; else adaptiveForceLines = true;
	}

	void setSoftBodyFactor(btScalar value)
	{
		softBodyFactor = float(value);
	}

	btScalar getSoftBodyFactor()
	{
		return (btScalar)softBodyFactor;
	}

	std::vector<appliedForce> getAppliedForces();



};


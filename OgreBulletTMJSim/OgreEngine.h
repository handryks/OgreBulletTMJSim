#pragma once
#include <Ogre.h>
#include <OIS.h>
#include "PhysicsEngine.h"
#include "DebugDrawer.h"
#include <CEGUI\CEGUI.h>
#include "CEGUI\RendererModules\Ogre\Renderer.h"

class OgreEngine : public Ogre::WindowEventListener, public Ogre::FrameListener, public OIS::KeyListener, public OIS::MouseListener
{
private:
	// Ogre Base
	Ogre::Root* mRoot;
	Ogre::String mResourcesCfg;
	Ogre::String mPluginsCfg;
	Ogre::RenderWindow* mWindow;
	Ogre::SceneManager* mSceneMgr;
	Ogre::Camera* mCamera;
	Ogre::SceneNode* mCameraNode;

	// OIS
	OIS::InputManager* mInputManager;
	OIS::Mouse* mMouse;
	OIS::Keyboard* mKeyboard;
	float mRotSpd;

	// Physics
	simulationParams simParams;
	PhysicsEngine* mPhysicsEngine;
	DebugDrawer* mDebugDrawer;
	simulationLog* mSimulationLog;
	btRigidBody* mandible;
	bool useTetraHedrons = false;
	bool useSoftBodyLigaments = false;
	bool addDiscs = true;
	bool useHDdiscs = true;
	bool useGravity = false;
	bool forcesPerNode = false;
	btScalar gravY = -9800; // mm/s
	float timeStep = 1.f / 120.f;
	int maxSubSteps = 1;
	float fixedTimeStep = 1.f / 120.f;
	int solvers = 100;

	// Quit
	bool mShutDown;
	

protected:
	// CEGUI
	CEGUI::Renderer* mRenderer;

	// Ogre::FrameListener
	virtual bool frameStarted(const Ogre::FrameEvent& evt);
	virtual bool frameEnded(const Ogre::FrameEvent& evt);
	virtual bool frameRenderingQueued(const Ogre::FrameEvent& evt);

	// Ogre::WindowEventListener
	virtual void windowResized(Ogre::RenderWindow* rw);
	virtual void windowClosed(Ogre::RenderWindow* rw);

	// Load resources
	void loadResources(Ogre::String res);

	// Setup main camera
	void setupCamera(Ogre::Camera* cam, Ogre::SceneManager* sm, Ogre::RenderWindow* rw);

	// Create Ogre Scene
	void createScene();

	// Initial setup
	bool setup();

	// OIS
	void setupOIS();
	virtual bool keyPressed(const OIS::KeyEvent& ke);
	virtual bool keyReleased(const OIS::KeyEvent& ke);
	virtual bool mouseMoved(const OIS::MouseEvent& me);
	virtual bool mousePressed(const OIS::MouseEvent& me, OIS::MouseButtonID id);
	virtual bool mouseReleased(const OIS::MouseEvent& me, OIS::MouseButtonID id);

	// Setup CEGUI
	void loadCEGUI();

	// Load CEGUI Controls
	void loadCEGUIControls();

	// CEGUI Mouse Function
	CEGUI::MouseButton convertButton(OIS::MouseButtonID buttonID)
	{
		switch (buttonID)
		{
		case OIS::MB_Left:
			return CEGUI::LeftButton;

		case OIS::MB_Right:
			return CEGUI::RightButton;

		case OIS::MB_Middle:
			return CEGUI::MiddleButton;

		default:
			return CEGUI::LeftButton;
		}
	}

	// Physics
	

	void resetCamera()
	{
		mCameraNode->resetToInitialState();
		mCamera->setPosition(Ogre::Vector3(0, 0, 300));
		mCamera->lookAt(Ogre::Vector3(0, 0, 0));
	}

public:
	OgreEngine();
	~OgreEngine();

	void startPhysics()
	{
		mPhysicsEngine->setRunPhysics(true);
	}

	void stopPhysics()
	{
		mPhysicsEngine->setRunPhysics(false);
	}

	void restartPhysics(std::string dataFolder = "data\\")
	{
		mPhysicsEngine->deleteWorld();
		setupPhysics(dataFolder);
	}

	void setupPhysics(std::string dataFolder);

	PhysicsEngine* getPhysicsEngine()
	{
		return mPhysicsEngine;
	}
	
	
	simulationParams getSimulationParam()
	{
		simParams.timeStep = timeStep;
		simParams.fixedTimeStep = fixedTimeStep;
		simParams.maxSubSteps = maxSubSteps;
		simParams.loadDiscs = addDiscs;
		simParams.useTetras = useTetraHedrons;
		simParams.solvers = solvers;
		simParams.useSoftLigaments = useSoftBodyLigaments;
		simParams.useHDdiscs = useHDdiscs;
		simParams.useGravity = useGravity;
		simParams.softBodyFactor = mPhysicsEngine->getSoftBodyFactor();
		std::cout << "SoftBody Factor: " << simParams.softBodyFactor << std::endl;
		return simParams;
	}

	void setSimulationParams(simulationParams s)
	{
		simParams = s;
		timeStep = s.timeStep;
		fixedTimeStep = s.fixedTimeStep;
		maxSubSteps = s.maxSubSteps;
		addDiscs = s.loadDiscs;
		useTetraHedrons = s.useTetras;
		solvers = s.solvers;
		useSoftBodyLigaments = s.useSoftLigaments;
		useHDdiscs = s.useHDdiscs;
		useGravity = s.useGravity;
		if (mPhysicsEngine) mPhysicsEngine->setSoftBodyFactor(s.softBodyFactor);
	}

	void startRecording(bool position)
	{
		std::cout << "Rodando gravacao...\n";
		std::string caption = "RECORDING ";
		if (position) caption += "POSITION "; else caption += "MOVEMENT ";
		if (addDiscs) caption += " WITH DISC\n"; else caption += " WITHOUT DISC\n";
		mPhysicsEngine->startRecording(mSimulationLog, caption, position);
	}

	void stopRecording()
	{
		mSimulationLog->recording = false;
	}

	void toggleRecord()
	{
		if (mSimulationLog->recording) stopRecording(); else startRecording(false);
	}

	bool isRecording()
	{
		return mSimulationLog->recording;
	}

	void quit()
	{
		std::cout << "Changing quit value\n";
		mShutDown = true;
	}
	void run();

	void saveConfig(std::string filename);
	void loadConfig(std::string filename);

};


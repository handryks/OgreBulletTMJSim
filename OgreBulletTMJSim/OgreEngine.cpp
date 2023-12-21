#include "OgreEngine.h"
#include "ogreMotionState.h"
#include "DebugDrawUtils.h"
//#include "CollisionFlags.h"
#include "MenuControl.h"
//#include <stdio.h>
#include <chrono>
//#include <boost\archive\text_oarchive.hpp>
//#include <boost\archive\text_iarchive.hpp>
//#include <boost\asio.hpp>

const Ogre::Vector3 startPosition = Ogre::Vector3(2.91061, -58.43966, 19.00534);


OgreEngine::OgreEngine()
{
	mRotSpd = 0.1f;
}


OgreEngine::~OgreEngine()
{
	Ogre::WindowEventUtilities::removeWindowEventListener(mWindow, this);
	windowClosed(mWindow);
	saveConfig("simulation_params.cfg");
}

void OgreEngine::saveConfig(std::string filename)
{
	std::ofstream ofs(filename);
	// save data to archive
	{
		boost::archive::text_oarchive oa(ofs);
		// write class instance to archive
		oa << getSimulationParam();
		// archive and stream closed when destructors are called
	}
	ofs.close();
	if (addDiscs) if (mPhysicsEngine) mPhysicsEngine->saveSoftBodyConfigs("soft_configs.cfg");
}

void OgreEngine::loadConfig(std::string filename)
{
	simulationParams s;
	std::ifstream ifs(filename, std::ios::binary);
	if (ifs.good())
	// load data to archive
	{
		boost::archive::text_iarchive ia(ifs);
		// write class instance to archive
		ia >> s;
		// archive and stream closed when destructors are called
		ifs.close();
		setSimulationParams(s);
	}
}


bool OgreEngine::frameStarted(const Ogre::FrameEvent& evt)
{
	float fps = mWindow->getLastFPS();
	if (mPhysicsEngine)
	{
		mPhysicsEngine->stepWorld(timeStep, maxSubSteps, fixedTimeStep);
		//mPhysicsEngine->stepWorld(1.f / fps);
	}
	return true;
}

bool OgreEngine::frameEnded(const Ogre::FrameEvent& evt)
{
	return true;
}

bool OgreEngine::frameRenderingQueued(const Ogre::FrameEvent& evt)
{
	if (mWindow->isClosed()) return false;

	if (mShutDown) return false;

	mKeyboard->capture();
	mMouse->capture();

	//Need to inject timestamps to CEGUI System.
	CEGUI::System::getSingleton().injectTimePulse((float) evt.timeSinceLastFrame);
	return true;
}

void OgreEngine::windowResized(Ogre::RenderWindow* rw)
{
	unsigned int width, height, depth;
	int left, top;
	rw->getMetrics(width, height, depth, left, top);

	const OIS::MouseState &ms = mMouse->getMouseState();
	ms.width = width;
	ms.height = height;

	std::cout << "JANELA REDIMENSIONADA\n";

	CEGUI::System::getSingleton().notifyDisplaySizeChanged(
		CEGUI::Sizef(
		static_cast<float>(width),
		static_cast<float>(height)
		)
		);

	mCamera->setAspectRatio(Ogre::Real(width) / Ogre::Real(height));
}

void OgreEngine::windowClosed(Ogre::RenderWindow* rw)
{
	//Only close for window that created OIS (the main window in these demos)
	if (rw == mWindow)
	{
		if (mInputManager)
		{
			mInputManager->destroyInputObject(mMouse);
			mInputManager->destroyInputObject(mKeyboard);

			OIS::InputManager::destroyInputSystem(mInputManager);
			mInputManager = 0;
		}
	}
}

bool OgreEngine::keyPressed(const OIS::KeyEvent& ke)
{
	CEGUI::GUIContext& context = CEGUI::System::getSingleton().getDefaultGUIContext();
	context.injectKeyDown((CEGUI::Key::Scan)ke.key);
	context.injectChar((CEGUI::Key::Scan)ke.text);
	if (ke.key == OIS::KC_R)   // cycle polygon rendering mode
	{
		Ogre::String newVal;
		Ogre::PolygonMode pm;

		switch (mCamera->getPolygonMode())
		{
		case Ogre::PM_SOLID:
			newVal = "Wireframe";
			pm = Ogre::PM_WIREFRAME;
			break;
		case Ogre::PM_WIREFRAME:
			newVal = "Points";
			pm = Ogre::PM_POINTS;
			break;
		default:
			newVal = "Solid";
			pm = Ogre::PM_SOLID;
		}

		mCamera->setPolygonMode(pm);
	}
	else if (ke.key == OIS::KC_SPACE)
	{
		mPhysicsEngine->toggleRunPhysics();
	}
	else if (ke.key == OIS::KC_D)
	{
		mPhysicsEngine->toggleDisplayDebug();
	}
	else if (ke.key == OIS::KC_BACK && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		mPhysicsEngine->deleteWorld();
		setupPhysics("data\\");
	}
	else if (ke.key == OIS::KC_S)
	{
		mSceneMgr->getSceneNode("CranioNode")->flipVisibility(false);
	}
	else if (ke.key == OIS::KC_M)
	{
		mSceneMgr->getSceneNode("MandibulaNode")->flipVisibility(false);
	}
	else if (ke.key == OIS::KC_V)
	{
		mSceneMgr->getSceneNode("MandibulaVHACDNode")->flipVisibility();
	}
	else if (ke.key == OIS::KC_B)
	{
		mSceneMgr->getSceneNode("FossaDireitaNode")->flipVisibility();
	}
	else if (ke.key == OIS::KC_N)
	{
		mSceneMgr->getSceneNode("FossaEsquerdaNode")->flipVisibility();
	}
	else if (ke.key == OIS::KC_G)
	{
		mSceneMgr->getSceneNode("DentesNode")->flipVisibility();
	}
	else if (ke.key == OIS::KC_O)
	{
		startRecording(true);
	}
	else if (ke.key == OIS::KC_P)
	{
		toggleRecord();
	}
	else if (ke.key == OIS::KC_C)
	{
		mPhysicsEngine->toggleDisplayClusters();
	}
	else if (ke.key == OIS::KC_I)
	{
		mPhysicsEngine->toggleDisplaySoftBodies();
	}
	else if (ke.key == OIS::KC_F)
	{
		mPhysicsEngine->toggleDisplayForces();
	}
	else if (ke.key == OIS::KC_A)
	{
		mPhysicsEngine->toggleAdaptiveForces();
	}
	else if (ke.key == OIS::KC_L)
	{
		mPhysicsEngine->toggleDisplayLigaments();
	}
	else if (ke.key == OIS::KC_H)
	{
		mPhysicsEngine->toggleDrawRigidBody(1);
	}
	else if (ke.key == OIS::KC_J)
	{
		mPhysicsEngine->toggleDrawRigidBody(2);
	}
	else if (ke.key == OIS::KC_K)
	{
		mPhysicsEngine->toggleDrawRigidBody(3);
	}
	else if (ke.key == OIS::KC_Y)
	{
		mPhysicsEngine->toggleDrawRigidBody(4);
	}
	else if (ke.key == OIS::KC_Q) // Get applyed forces
	{
		btVector3 forces = mandible->getTotalForce();
		std::cout << "Forcas: " << cvt(forces) << std::endl;
	}
	else if (ke.key == OIS::KC_1 && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		resetCamera();
	}
	else if (ke.key == OIS::KC_2 && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		resetCamera();
		mCameraNode->yaw(Ogre::Angle(90));
	}
	else if (ke.key == OIS::KC_3 && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		resetCamera();
		mCameraNode->yaw(Ogre::Angle(-90));
	}
	else if (ke.key == OIS::KC_4 && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		resetCamera();
		mCameraNode->yaw(Ogre::Angle(-30));
		mCameraNode->pitch(Ogre::Angle(-15));
	}
	else if (ke.key == OIS::KC_5 && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		resetCamera();
		mCameraNode->yaw(Ogre::Angle(-30));
		mCameraNode->pitch(Ogre::Angle(30));
	}
	else if (ke.key == OIS::KC_6 && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		resetCamera();
		mCameraNode->yaw(Ogre::Angle(180));
		mCameraNode->pitch(Ogre::Angle(15));
	}
	else if (ke.key == OIS::KC_8 && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		mPhysicsEngine->toggleMassControlVisibility();
	}
	else if (ke.key == OIS::KC_9 && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		mPhysicsEngine->toggleMuscleControlVisibility();
	}
	else if (ke.key == OIS::KC_0 && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		mPhysicsEngine->toggleLigamentsControlVisibility();
	}
	else if (ke.key == OIS::KC_RETURN && mKeyboard->isKeyDown(OIS::KC_LCONTROL))
	{
		mPhysicsEngine->toggleActiveMotors();
	}
	return true;
}

bool OgreEngine::keyReleased(const OIS::KeyEvent& ke)
{
	CEGUI::System::getSingleton().getDefaultGUIContext().injectKeyUp((CEGUI::Key::Scan)ke.key);
	return true;
}

bool OgreEngine::mouseMoved(const OIS::MouseEvent& me)
{
	CEGUI::System &sys = CEGUI::System::getSingleton();
	// Inject mouse only if inside windows bounds
	if (me.state.X.abs > 0 && me.state.X.abs < (int)mWindow->getWidth()
		&& me.state.Y.abs > 0 && me.state.Y.abs < (int)mWindow->getHeight())
	{
		sys.getDefaultGUIContext().injectMousePosition((float)me.state.X.abs, (float)me.state.Y.abs);
	}


	if (me.state.Z.rel) {
		sys.getDefaultGUIContext().injectMouseWheelChange(me.state.Z.rel / 120.0f);
		if (mKeyboard->isKeyDown(OIS::KC_LCONTROL)) {
			mCamera->moveRelative(Ogre::Vector3(0, 0, -me.state.Z.rel * mRotSpd));
			//mCameraNode->translate(Ogre::Vector3(0, 0, -me.state.Z.rel * mRotSpd));
		}
	}
	else if (me.state.buttonDown(OIS::MB_Left))
	{
		if (mKeyboard->isKeyDown(OIS::KC_LCONTROL)) {
			mCameraNode->yaw(Ogre::Degree(-me.state.X.rel * mRotSpd));
			mCameraNode->pitch(Ogre::Degree(-me.state.Y.rel * mRotSpd));
		}
	}
	else if (me.state.buttonDown(OIS::MB_Right))
	{
		if (mKeyboard->isKeyDown(OIS::KC_LCONTROL)) {
			mCameraNode->translate(Ogre::Vector3(-me.state.X.rel * mRotSpd, me.state.Y.rel * mRotSpd, 0), Ogre::Node::TS_LOCAL);
			//mCameraNode->getUserObjectBindings().
			/*mCamera->moveRelative(Ogre::Vector3(-me.state.X.rel * mRotSpd, 0, 0));
			mCamera->moveRelative(Ogre::Vector3(0, me.state.Y.rel * mRotSpd, 0));*/
		}
	}
	else if (me.state.buttonDown(OIS::MB_Middle))
	{
		if (mKeyboard->isKeyDown(OIS::KC_LCONTROL)) {
			mCameraNode->roll(Ogre::Degree(-me.state.X.rel * mRotSpd));
			//mCamera->roll(Ogre::Degree(-me.state.X.rel * mRotSpd));
		}
	}
	return true;
}

bool OgreEngine::mousePressed(const OIS::MouseEvent& me, OIS::MouseButtonID id)
{
	CEGUI::System::getSingleton().getDefaultGUIContext().injectMouseButtonDown(convertButton(id));
	return true;
}

bool OgreEngine::mouseReleased(const OIS::MouseEvent& me, OIS::MouseButtonID id)
{
	CEGUI::System::getSingleton().getDefaultGUIContext().injectMouseButtonUp(convertButton(id));
	return true;
}

void OgreEngine::loadResources(Ogre::String res)
{
	Ogre::ConfigFile cf;
	cf.load(res);

	Ogre::String secName, name, locType;
	Ogre::ConfigFile::SectionIterator secIt = cf.getSectionIterator();

	while (secIt.hasMoreElements())
	{
		secName = secIt.peekNextKey();
		Ogre::ConfigFile::SettingsMultiMap* settings = secIt.getNext();
		Ogre::ConfigFile::SettingsMultiMap::iterator it;

		for (it = settings->begin(); it != settings->end(); ++it)
		{
			locType = it->first;
			name = it->second;

			std::cout << "Tipo: " << locType << "\nNome: " << name << std::endl;

			Ogre::ResourceGroupManager::getSingleton().addResourceLocation(name, locType, secName);
		}
	}

	Ogre::ResourceGroupManager::getSingleton().initialiseAllResourceGroups();
}


void OgreEngine::setupCamera(Ogre::Camera* cam, Ogre::SceneManager* sm, Ogre::RenderWindow* rw)
{
	mCamera = mSceneMgr->createCamera("MainCam");

	mCamera->setPosition(0, 0, 300);
	mCamera->lookAt(0, 0, 0);
	mCamera->setNearClipDistance(5);

	Ogre::Viewport* vp = mWindow->addViewport(mCamera);

	vp->setBackgroundColour(Ogre::ColourValue(200, 200, 200));

	//mCamera->setAutoAspectRatio(true);

	mCamera->setAspectRatio(
	Ogre::Real(vp->getActualWidth()) /
	Ogre::Real(vp->getActualHeight()));

	std::cout << "CURRENT WIDTH VP: " << vp->getActualWidth() << "\n";
	std::cout << "CURRENT HEIGHT VP: " << vp->getActualHeight() << "\n";

	mCameraNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	mCameraNode->attachObject(mCamera);
}

void OgreEngine::createScene()
{
	Ogre::Entity* mandibula = mSceneMgr->createEntity("Mandibula", "mandibula-dec.mesh");
	
	Ogre::SceneNode* mandibulaNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("MandibulaNode");
	mandibulaNode->attachObject(mandibula);
	mandibulaNode->translate(startPosition);

	Ogre::Entity* mandibula_vhacd = mSceneMgr->createEntity("MandibulaVHACD", "merge_mandibula.mesh");
	/*mandibula_vhacd->setCastShadows(false);
	mandibula_vhacd->setLightMask(Ogre::uint32(1));*/
	Ogre::SceneNode* mandibulaVHACDNode = mandibulaNode->createChildSceneNode("MandibulaVHACDNode");
	mandibulaVHACDNode->attachObject(mandibula_vhacd);
	mandibulaVHACDNode->setVisible(false);

	Ogre::Entity* cranio = mSceneMgr->createEntity("Cranio", "cranio-dec.mesh");
	Ogre::SceneNode* cranioNode = mSceneMgr->getRootSceneNode()->createChildSceneNode("CranioNode");
	cranioNode->attachObject(cranio);

	Ogre::Entity* fossaDireita = mSceneMgr->createEntity("FossaDireita", "merge_fossa_direita.mesh");
	Ogre::SceneNode* fossaDireitaNode = cranioNode->createChildSceneNode("FossaDireitaNode");
	fossaDireitaNode->attachObject(fossaDireita);
	fossaDireitaNode->setVisible(false);

	Ogre::Entity* fossaEsquerda = mSceneMgr->createEntity("FossaEsquerda", "merge_fossa_esquerda.mesh");
	Ogre::SceneNode* fossaEsquerdaNode = cranioNode->createChildSceneNode("FossaEsquerdaNode");
	fossaEsquerdaNode->attachObject(fossaEsquerda);
	fossaEsquerdaNode->setVisible(false);

	Ogre::Entity* dentes = mSceneMgr->createEntity("Dentes", "merge_dentes.mesh");
	Ogre::SceneNode* dentesNode = cranioNode->createChildSceneNode("DentesNode");
	dentesNode->attachObject(dentes);
	dentesNode->setVisible(false);

	//// Configura as luzes da cena
	mSceneMgr->setAmbientLight(Ogre::ColourValue(0.5, 0.5, 0.5));
	Ogre::Light* light = mSceneMgr->createLight("MainLight");
	light->setPosition(0, 150, 150);
	//light->setAttenuation(100, 1.0, 0.045, 0.0075);

	/*Ogre::Light* light0 = mSceneMgr->createLight("SecondLight");
	light0->setPosition(150, 80, 0);

	Ogre::Light* light1 = mSceneMgr->createLight("ThridLight");
	light1->setPosition(-150, 80, 0);*/
}

void OgreEngine::setupPhysics(std::string dataFolder)
{

	if (!mPhysicsEngine) mPhysicsEngine = new PhysicsEngine();
	btVector3 gravity;
	if (useGravity) gravity = btVector3(0, gravY, 0); else gravity = btVector3(0, 0, 0);
	mPhysicsEngine->createSoftBodyWorld(gravity);
	if (!mDebugDrawer) mDebugDrawer = new DebugDrawer(mSceneMgr);
	mDebugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawConstraints + btIDebugDraw::DBG_DrawConstraintLimits + btIDebugDraw::DBG_DrawContactPoints);
	mPhysicsEngine->setDebugDrawer(mDebugDrawer);
	mPhysicsEngine->createDefaultGround();

	btContactSolverInfo& info = mPhysicsEngine->getDynamicsWorld()->getSolverInfo();
	info.m_numIterations = solvers;
	info.m_solverMode = SOLVER_SIMD;

	Ogre::SceneNode* mandibulaNode = mSceneMgr->getSceneNode("MandibulaNode");
	mandibulaNode->resetToInitialState();
	mandibulaNode->setPosition(startPosition);
	
	try 
	{
		std::cout << "Loading control point\n";
		Ogre::SceneNode* openMouth = mSceneMgr->getSceneNode("OpenMouth");
		Ogre::SceneNode* closedMouth = mSceneMgr->getSceneNode("ClosedMouth");
	}
	catch (Ogre::Exception e)
	{
		std::cout << "Creating control point\n";
		std::vector<FiducialMark> openMouth = mPhysicsEngine->loadFiducial(dataFolder + "SIMULATION_LOG_POINTS_OPEN_MOUTH.fcsv");
		Ogre::SceneNode* openMouthNode = mSceneMgr->getSceneNode("CranioNode")->createChildSceneNode("OpenMouth");
		for (int i = 0; i < openMouth.size(); i++)
		{
			Ogre::Entity* point = mSceneMgr->createEntity("sphere.mesh");
			Ogre::SceneNode* pointNode = openMouthNode->createChildSceneNode();
			pointNode->attachObject(point);
			pointNode->setScale(Ogre::Vector3(0.01, 0.01, 0.01));
			pointNode->translate(cvt(openMouth[i].getVector3()));
			std::cout << openMouth[i].label << "," << openMouth[i].getVector3().getX() <<
				"," << openMouth[i].getVector3().getY() <<
				"," << openMouth[i].getVector3().getZ() << std::endl;
		}

		Ogre::SceneNode* closedMouthNode = mandibulaNode->createChildSceneNode("ClosedMouth");
		std::vector<FiducialMark> closedMouth = mPhysicsEngine->loadFiducial(dataFolder + "SIMULATION_LOG_POINTS_CLOSED_MOUTH.fcsv");
		for (int i = 0; i < closedMouth.size(); i++)
		{
			Ogre::Entity* point = mSceneMgr->createEntity("sphere.mesh");
			Ogre::MaterialPtr mat = point->getSubEntity(0)->getMaterial()->clone("Insertions");
			mat->setDiffuse(Ogre::ColourValue::Blue);
			mat->setAmbient(Ogre::ColourValue::Blue);
			point->setMaterial(mat);
			Ogre::SceneNode* pointNode = closedMouthNode->createChildSceneNode();
			pointNode->attachObject(point);
			pointNode->setScale(Ogre::Vector3(0.01, 0.01, 0.01));
			pointNode->translate(-startPosition);
			pointNode->translate(cvt(closedMouth[i].getVector3()));
		}
	}

	


	// Set all nodes visible
	mandibulaNode->setVisible(true, false);
	mSceneMgr->getSceneNode("CranioNode")->setVisible(true, false);
	

	ogreMotionState* mandibulaMotionState = new ogreMotionState();
	mandibulaMotionState->setNode(mandibulaNode);
	btRigidBody* rb = mPhysicsEngine->createRigidBodyFromObjFile(dataFolder + "mandibula-vhacd.obj", mandibulaMotionState, PhysicsEngine::DEFAULT_MANDIBLE_MASS, COL_MANDIBLE, mPhysicsEngine->mandibleCollidesWith);
	rb->setActivationState(DISABLE_DEACTIVATION);
	rb->translate(btStartPosition);
	/*rb->setFriction(0.f);
	rb->setRollingFriction(0.f);*/
	mPhysicsEngine->addRigidBodyToDraw(rb, 1, PhysicsEngine::COLOR_GREEN);

	mandible = rb;


	btCollisionShape* cranioDentesShape = mPhysicsEngine->loadCompoundShapeFromObjFile(dataFolder + "cranio_dentes-vhacd.obj");
	btRigidBody* cranioDentes = mPhysicsEngine->createRigidBody(0, btTransform::getIdentity(), cranioDentesShape);
	mPhysicsEngine->addRigidBodyToDraw(cranioDentes, 4, PhysicsEngine::COLOR_CYAN);

	btCollisionShape* fossaEsquerdaShape = mPhysicsEngine->loadCompoundShapeFromObjFile(dataFolder + "fossa_esquerda-vhacd.obj");
	btRigidBody* fossaEsquerda = mPhysicsEngine->createRigidBody(0, btTransform::getIdentity(), fossaEsquerdaShape);
	mPhysicsEngine->addRigidBodyToDraw(fossaEsquerda, 3, PhysicsEngine::COLOR_YELLOW);

	btCollisionShape* fossaDireitaShape = mPhysicsEngine->loadCompoundShapeFromObjFile(dataFolder + "fossa_direita-vhacd.obj");
	btRigidBody* fossaDireita = mPhysicsEngine->createRigidBody(0, btTransform::getIdentity(), fossaDireitaShape);
	mPhysicsEngine->addRigidBodyToDraw(fossaDireita, 2, PhysicsEngine::COLOR_YELLOW);

	/*cranioDentes->setFriction(0.f);
	fossaEsquerda->setFriction(0.f);
	fossaDireita->setFriction(0.f);*/


	// Configure Ligaments
	if (useSoftBodyLigaments)
	{
		btAlignedObjectArray<Ligament*> sligaments = mPhysicsEngine->createLigamentsFromFiducial2(fossaEsquerda, fossaDireita, rb, dataFolder + "Ligaments.fcsv");
		btAlignedObjectArray<Ligament*> sligaments2 = mPhysicsEngine->createLigamentsFromFiducial2(fossaEsquerda, fossaDireita, rb, dataFolder + "L_MASSETER__.fcsv");
	}
	else
	{
		btAlignedObjectArray<Ligament*> ligaments = mPhysicsEngine->createLigamentsFromFiducial(fossaEsquerda, fossaDireita, rb, dataFolder + "Ligaments.fcsv");
		//btAlignedObjectArray<Ligament*> ligaments2 = mPhysicsEngine->createLigamentsFromFiducial(fossaEsquerda, fossaDireita, rb, dataFolder + "CAPSULA_ARTICULAR.fcsv");
		LigamentControl* ligCon = new LigamentControl();
		ligCon->addLigament(ligaments);
		//ligCon->addLigament(ligaments2);
		ligCon->assignControls();
		ligCon->assignIDs();
		mPhysicsEngine->setLigamentControl(ligCon);
		float stifness = 272.4f;
		float dump = 0.49f;
		float step = timeStep;
		float erp = step * stifness / (step * stifness + dump);
		float cfm = 1 / (step * stifness + dump);
		std::cout << "ERP: " << erp << std::endl;
		std::cout << "CFM: " << cfm << std::endl;
		/*for (int i = 0; i < ligaments.size(); i++)
		{
			if (ligaments[i]->id == 1 || ligaments[i]->id == 4)
			{
				ligaments[i]->c1->setParam(BT_CONSTRAINT_ERP, erp);
				ligaments[i]->c1->setParam(BT_CONSTRAINT_CFM, cfm);
			}
		}*/
		//ligCon->loadValues();
		ligCon->loadConfig(dataFolder + "ligaments.cfg");
	}
	

	btSoftBody* discoDireito = 0;
	btSoftBody* discoEsquerdo = 0;

	MuscleControl* musCon = new MuscleControl();
	musCon->setSoftBodyFactor(simParams.softBodyFactor);

	int numOfLinks = 0;
	if (useHDdiscs)
	{
		numOfLinks = 100;
		if (useTetraHedrons) mPhysicsEngine->getSoftRigidDynamicsWorld()->setDrawFlags(mPhysicsEngine->getSoftRigidDynamicsWorld()->getDrawFlags() - fDrawFlags::Tetras);
	}
	else numOfLinks = 20;

	if (addDiscs)
	{
		if (useTetraHedrons) {
			if (useHDdiscs)
			{
				discoDireito = mPhysicsEngine->createFromTetGen(dataFolder + "tet_disco_direito_hd.1.ele",
					dataFolder + "tet_disco_direito_hd.1.smesh",
					dataFolder + "tet_disco_direito_hd.1.node",
					COL_DISC,
					mPhysicsEngine->discCollidesWith);

				discoEsquerdo = mPhysicsEngine->createFromTetGen(dataFolder + "tet_disco_esquerdo_hd.1.ele",
					dataFolder + "tet_disco_esquerdo_hd.1.smesh",
					dataFolder + "tet_disco_esquerdo_hd.1.node",
					COL_DISC,
					mPhysicsEngine->discCollidesWith);
			}
			else
			{
				discoDireito = mPhysicsEngine->createFromTetGen(dataFolder + "tet_disco_direito.1.ele",
					dataFolder + "tet_disco_direito.1.smesh",
					dataFolder + "tet_disco_direito.1.node",
					COL_DISC,
					mPhysicsEngine->discCollidesWith);

				discoEsquerdo = mPhysicsEngine->createFromTetGen(dataFolder + "tet_disco_esquerdo.1.ele",
					dataFolder + "tet_disco_esquerdo.1.smesh",
					dataFolder + "tet_disco_esquerdo.1.node",
					COL_DISC,
					mPhysicsEngine->discCollidesWith);
			}
		}
		else
		{
			if (useHDdiscs) 
			{
				discoDireito = mPhysicsEngine->createSoftBodyFromObjFile(dataFolder + "tet_disco_direito.obj");
				discoEsquerdo = mPhysicsEngine->createSoftBodyFromObjFile(dataFolder + "tet_disco_esquerdo.obj");
			}
			else
			{
				discoDireito = mPhysicsEngine->createSoftBodyFromObjFile(dataFolder + "tet_disco_direito_sd.obj");
				discoEsquerdo = mPhysicsEngine->createSoftBodyFromObjFile(dataFolder + "tet_disco_esquerdo_sd.obj");
			}
		}

		discoDireito->setUserIndex(1);
		discoEsquerdo->setUserIndex(2);
		//discoDireito->setUserPointer(this);
		//discoEsquerdo->setUserPointer(this);

		/*discoDireito->generateClusters(32);
		discoEsquerdo->generateClusters(32);*/
		btSoftBody::Material* m = discoDireito->appendMaterial();
		m->m_flags -= btSoftBody::fMaterial::DebugDraw;
		discoDireito->generateBendingConstraints(2, m);

		m = discoEsquerdo->appendMaterial();
		m->m_flags -= btSoftBody::fMaterial::DebugDraw;
		discoEsquerdo->generateBendingConstraints(2, m);

		/*btSoftBodyHelpers::ReoptimizeLinkOrder(discoDireito);
		btSoftBodyHelpers::ReoptimizeLinkOrder(discoEsquerdo);*/

		btAlignedObjectArray<btSoftBody*> sbodies;
		sbodies.push_back(discoDireito);
		sbodies.push_back(discoEsquerdo);

		SoftBodyControl* sbControl = new SoftBodyControl();
		mPhysicsEngine->setSoftBodyControl(sbControl);
		sbControl->mSoftBodies.push_back(discoDireito);
		sbControl->mSoftBodies.push_back(discoEsquerdo);

		// Create retrodiscal tissues
		{
			btSoftBody* rtiss = mPhysicsEngine->createRetrodiscalTissue(dataFolder + "R_RETRODISCAL_TISSUE.fcsv", discoDireito, 10, 10, true, 5.0f, numOfLinks);
			rtiss->setUserIndex(3);
			rtiss->setTotalMass(PhysicsEngine::DEFAULT_DISC_MASS);
			//rtiss->generateClusters(0);

			btSoftBody* ltiss = mPhysicsEngine->createRetrodiscalTissue(dataFolder + "L_RETRODISCAL_TISSUE.fcsv", discoEsquerdo, 10, 10, true, 5.0f, numOfLinks);
			ltiss->setUserIndex(4);
			ltiss->setTotalMass(PhysicsEngine::DEFAULT_DISC_MASS);
			//ltiss->generateClusters(0);

			sbodies.push_back(discoDireito);
			sbodies.push_back(discoEsquerdo);
			sbodies.push_back(rtiss);
			sbodies.push_back(ltiss);

			mPhysicsEngine->getSoftRigidDynamicsWorld()->addSoftBody(rtiss, COL_DISC, mPhysicsEngine->discCollidesWith);
			mPhysicsEngine->addSoftBodyToDraw(rtiss);

			mPhysicsEngine->getSoftRigidDynamicsWorld()->addSoftBody(ltiss, COL_DISC, mPhysicsEngine->discCollidesWith);
			mPhysicsEngine->addSoftBodyToDraw(ltiss);


			sbControl->mSoftBodies.push_back(rtiss);
			sbControl->mSoftBodies.push_back(ltiss);
		}

		// Create attachs for the right discs
		{
			if (forcesPerNode)
			{
				sbMuscle* rdsbm = mPhysicsEngine->createSBMuscleFromFiducial(discoDireito, "R_LAT_PTERYGOID_SUP", dataFolder + "R_DISC_MUSCLE.fcsv");
				rdsbm->perNode = true;
				musCon->mSoftBodyMuscles.push_back(rdsbm);
			}
			else
			{

				btSoftBody* rlatattach = mPhysicsEngine->createRetrodiscalTissue(dataFolder + "R_DISC_MUSCLE.fcsv", discoDireito, 10, 10, true, 10.f, numOfLinks);
				rlatattach->setUserIndex(5);
				rlatattach->setTotalMass(PhysicsEngine::DEFAULT_DISC_MASS);
				//rlatattach->generateClusters(0);
				sbodies.push_back(rlatattach);

				mPhysicsEngine->getSoftRigidDynamicsWorld()->addSoftBody(rlatattach, COL_DISC, mPhysicsEngine->discCollidesWith);
				mPhysicsEngine->addSoftBodyToDraw(rlatattach);


				sbMuscle* rdsbm = mPhysicsEngine->createSBMuscle(rlatattach, "R_LAT_PTERYGOID_SUP");
				musCon->mSoftBodyMuscles.push_back(rdsbm);
				sbControl->mSoftBodies.push_back(rlatattach);
			}

			btSoftBody* rantinfattach = mPhysicsEngine->createAttachTissue(dataFolder + "R_DISC_ANTERIOR_ATTACH_INF.fcsv", discoDireito, rb, 10, 1.5f, numOfLinks);
			rantinfattach->setUserIndex(6);
			rantinfattach->setTotalMass(PhysicsEngine::DEFAULT_DISC_MASS);
			//rantinfattach->generateClusters(0);
			

			mPhysicsEngine->getSoftRigidDynamicsWorld()->addSoftBody(rantinfattach, COL_DISC, mPhysicsEngine->discCollidesWith);
			mPhysicsEngine->addSoftBodyToDraw(rantinfattach);


			btSoftBody* rposinfattach = mPhysicsEngine->createAttachTissue(dataFolder + "R_DISC_POSTERIOR_ATTACH_INF.fcsv", discoDireito, rb, 10, 1.5f, numOfLinks);
			rposinfattach->setUserIndex(7);
			rposinfattach->setTotalMass(PhysicsEngine::DEFAULT_DISC_MASS);
			//rposinfattach->generateClusters(0);
			sbodies.push_back(rposinfattach);

			mPhysicsEngine->getSoftRigidDynamicsWorld()->addSoftBody(rposinfattach, COL_DISC, mPhysicsEngine->discCollidesWith);
			mPhysicsEngine->addSoftBodyToDraw(rposinfattach);

			
			sbControl->mSoftBodies.push_back(rantinfattach);
			sbControl->mSoftBodies.push_back(rposinfattach);
		}

		// Create attachs for the left discs
		{
			btSoftBody* llatattach = mPhysicsEngine->createRetrodiscalTissue(dataFolder + "L_DISC_MUSCLE.fcsv", discoEsquerdo, 10, 10, true, 10.f, numOfLinks);
			llatattach->setUserIndex(8);
			llatattach->setTotalMass(PhysicsEngine::DEFAULT_DISC_MASS);
			//llatattach->generateClusters(0);
			sbodies.push_back(llatattach);

			mPhysicsEngine->getSoftRigidDynamicsWorld()->addSoftBody(llatattach, COL_DISC, mPhysicsEngine->discCollidesWith);
			mPhysicsEngine->addSoftBodyToDraw(llatattach);

			sbMuscle* ldsbm = mPhysicsEngine->createSBMuscle(llatattach, "L_LAT_PTERYGOID_SUP");
			musCon->mSoftBodyMuscles.push_back(ldsbm);

			btSoftBody* lantinfattach = mPhysicsEngine->createAttachTissue(dataFolder + "L_DISC_ANTERIOR_ATTACH_INF.fcsv", discoEsquerdo, rb, 10, 1.5f, numOfLinks);
			lantinfattach->setUserIndex(9);
			lantinfattach->setTotalMass(PhysicsEngine::DEFAULT_DISC_MASS);
			//lantinfattach->generateClusters(0);
			sbodies.push_back(lantinfattach);

			mPhysicsEngine->getSoftRigidDynamicsWorld()->addSoftBody(lantinfattach, COL_DISC, mPhysicsEngine->discCollidesWith);
			mPhysicsEngine->addSoftBodyToDraw(lantinfattach);

			btSoftBody* lposinfattach = mPhysicsEngine->createAttachTissue(dataFolder + "L_DISC_POSTERIOR_ATTACH_INF.fcsv", discoEsquerdo, rb, 10, 1.5f, numOfLinks);
			lposinfattach->setUserIndex(10);
			lposinfattach->setTotalMass(PhysicsEngine::DEFAULT_DISC_MASS);
			//lposinfattach->generateClusters(0);
			sbodies.push_back(lposinfattach);

			mPhysicsEngine->getSoftRigidDynamicsWorld()->addSoftBody(lposinfattach, COL_DISC, mPhysicsEngine->discCollidesWith);
			mPhysicsEngine->addSoftBodyToDraw(lposinfattach);

			sbControl->mSoftBodies.push_back(llatattach);
			sbControl->mSoftBodies.push_back(lantinfattach);
			sbControl->mSoftBodies.push_back(lposinfattach);
		}

		

		


		

		//mPhysicsEngine->saveSoftBodyConfigs("soft_configs.cfg");
		if (!mPhysicsEngine->loadSoftBodyConfigs("soft_configs.cfg"))
		{
			for (int i = 0; i < sbodies.size(); i++)
			{
				if (sbodies[i]->getUserIndex() <= 2) // defaults for discs
				{
					sbodies[i]->setPose(false, true);
					sbodies[i]->m_cfg.kMT = 0.2f;
					sbodies[i]->m_cfg.kDP = 0.1;
					/*sbodies[i]->setCcdSweptSphereRadius(4.0f);
					sbodies[i]->setCcdMotionThreshold(0.5f);*/
					//sbodies[i]->m_cfg.kSK_SPLT_CL = 1.0f;
					//sbodies[i]->m_cfg.kSR_SPLT_CL = 1.0f;
					//sbodies[i]->m_cfg.kSS_SPLT_CL = 1.0f;
					/*sbodies[i]->m_cfg.citerations = 10;
					sbodies[i]->m_cfg.piterations = 10;
					sbodies[i]->m_cfg.viterations = 10;
					sbodies[i]->m_cfg.diterations = 10;*/
					//sbodies[i]->m_cfg.kVCF = 100;
					if (useTetraHedrons)
					{
						sbodies[i]->setVolumeMass(PhysicsEngine::DEFAULT_DISC_MASS);
						//sbodies[i]->generateClusters(16);
					}
					else
					{
						//sbodies[i]->generateClusters(256);
					}
				}
				if (sbodies[i]->getUserIndex() > 2) // Defaults for retrodiscal tissue
				{
					sbodies[i]->m_cfg.kAHR = .5f;
					sbodies[i]->m_cfg.kDP = 0.01;
					/*sbodies[i]->generateClusters(0);*/
				}
				
				sbodies[i]->m_cfg.kCHR = 1.f;
				sbodies[i]->m_cfg.kKHR = 1.f;
				sbodies[i]->m_cfg.kSHR = 1.f;
				sbodies[i]->m_cfg.kSRHR_CL = 1.f;
				sbodies[i]->m_cfg.kSKHR_CL = 1.f;
				sbodies[i]->m_cfg.kSSHR_CL = 1.f;
				sbodies[i]->randomizeConstraints();
			}
		}

		sbControl->assignControls();
		
	}

	// Configure Muscles
	
	for (int i = 0; i < musCon->fileNames.size(); i++)
	{
		btAlignedObjectArray<rbMuscle*> l = mPhysicsEngine->createMusclesFrom2Fiducial(rb,
			dataFolder + "L_" + musCon->fileNames[i] + "_ORI.fcsv",
			dataFolder + "L_" + musCon->fileNames[i] + "_INS.fcsv");
		btAlignedObjectArray<rbMuscle*> r = mPhysicsEngine->createMusclesFrom2Fiducial(rb,
			dataFolder + "R_" + musCon->fileNames[i] + "_ORI.fcsv",
			dataFolder + "R_" + musCon->fileNames[i] + "_INS.fcsv");

		musCon->addMuscles(l);
		musCon->addMuscles(r);
	}

	musCon->assignControls();
	musCon->assignIds();
	mPhysicsEngine->setMuscleControl(musCon);
	mPhysicsEngine->setSoftBodyFactor(getSimulationParam().softBodyFactor);

	btAlignedObjectArray<muscleMotor> motors = musCon->loadMotorsFromFile(dataFolder + "motors.cfg");


	for (int i = 0; i < musCon->mRigidBodyMuscles.size(); i++)
	{
		for (int j = 0; j < motors.size(); j++)
		{
			if (motors[j].group == musCon->mRigidBodyMuscles[i]->group)
			{
				musCon->mRigidBodyMuscles[i]->motor2.push_back(motors[j]);
			}
		}
	}
	for (int i = 0; i < musCon->mSoftBodyMuscles.size(); i++)
	{
		for (int j = 0; j < motors.size(); j++)
		{
			if (motors[j].group == musCon->mSoftBodyMuscles[i]->group)
			{
				std::cout << "Achou o motor para o musculo mode...\n";
				musCon->mSoftBodyMuscles[i]->motor2.push_back(motors[j]);
			}
		}
	}

	mSimulationLog = mPhysicsEngine->loadLogPoints(dataFolder + "SIMULATION_LOG_POINTS_CLOSED_MOUTH.fcsv", rb);
	mSimulationLog->filename = dataFolder + "simulationLogPosition.txt";
	mSimulationLog->timeStep = 16;
	mSimulationLog->recording = false;

	std::cout << "Total Muscles: " << musCon->mRigidBodyMuscles.size() << "\n";
	/*std::cout << "Total Ligaments: " << ligCon->mLigaments.size() << "\n";*/

	MassControl* massControl = new MassControl();
	massControl->assignControls();
	mPhysicsEngine->setMassControl(massControl);

}

void OgreEngine::setupOIS()
{
	Ogre::LogManager::getSingletonPtr()->logMessage("*** Initializing OIS ***");
	OIS::ParamList pl;
	size_t windowHnd = 0;
	std::ostringstream windowHndStr;

	mWindow->getCustomAttribute("WINDOW", &windowHnd);
	windowHndStr << windowHnd;
	pl.insert(std::make_pair(std::string("WINDOW"), windowHndStr.str()));
#if defined OIS_WIN32_PLATFORM
	pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_FOREGROUND")));
	pl.insert(std::make_pair(std::string("w32_mouse"), std::string("DISCL_NONEXCLUSIVE")));
	pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_FOREGROUND")));
	pl.insert(std::make_pair(std::string("w32_keyboard"), std::string("DISCL_NONEXCLUSIVE")));
#elif defined OIS_LINUX_PLATFORM
	pl.insert(std::make_pair(std::string("x11_mouse_grab"), std::string("false")));
	pl.insert(std::make_pair(std::string("x11_mouse_hide"), std::string("false")));
	pl.insert(std::make_pair(std::string("x11_keyboard_grab"), std::string("false")));
	pl.insert(std::make_pair(std::string("XAutoRepeatOn"), std::string("true")));
#endif
	//ShowCursor(0);

	mInputManager = OIS::InputManager::createInputSystem(pl);

	mKeyboard = static_cast<OIS::Keyboard*>(mInputManager->createInputObject(OIS::OISKeyboard, true));
	mMouse = static_cast<OIS::Mouse*>(mInputManager->createInputObject(OIS::OISMouse, true));

	mMouse->setEventCallback(this);
	mKeyboard->setEventCallback(this);

	// Set initial mouse clipping size
	windowResized(mWindow);

	// Register as a Window listener
	Ogre::WindowEventUtilities::addWindowEventListener(mWindow, this);

	mRoot->addFrameListener(this);
}

void OgreEngine::loadCEGUIControls()
{
	// Load main interface control
	CEGUI::Window *newWindow = CEGUI::WindowManager::getSingleton().loadLayoutFromFile("main.layout");
	CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->addChild(newWindow);
	 
	//std::cout << "Child Type: " << newWindow->getType() << std::endl;
	//CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow")->getChild("PopupMenu")->setVisible(true);
	//getScrolbar(newWindow);

	MenuControl* menuControl = new MenuControl();
	menuControl->assignControls();
	menuControl->mPhysicsEngine = mPhysicsEngine;
	menuControl->mOgreEngine = this;
}

void OgreEngine::loadCEGUI()
{
	//mRenderer = &CEGUI::OgreRenderer::bootstrapSystem();
	mRenderer = &CEGUI::OgreRenderer::bootstrapSystem(*mWindow);
	CEGUI::ImageManager::setImagesetDefaultResourceGroup("Imagesets");
	CEGUI::Font::setDefaultResourceGroup("Fonts");
	CEGUI::Scheme::setDefaultResourceGroup("Schemes");
	CEGUI::WidgetLookManager::setDefaultResourceGroup("LookNFeel");
	CEGUI::WindowManager::setDefaultResourceGroup("Layouts");

	// Setup Theme
	CEGUI::SchemeManager::getSingleton().createFromFile("OgreTray.scheme");
	CEGUI::SchemeManager::getSingleton().createFromFile("GWEN.scheme");
	CEGUI::SchemeManager::getSingleton().createFromFile("WindowsLook.scheme");
	CEGUI::SchemeManager::getSingleton().createFromFile("TaharezLook.scheme");
	CEGUI::SchemeManager::getSingleton().createFromFile("VanillaSkin.scheme");
	CEGUI::SchemeManager::getSingleton().createFromFile("VanillaCommonDialogs.scheme");
	CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().setDefaultImage("TaharezLook/MouseArrow");
	CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().hide();
	//CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().show();

	// Setup UI
	CEGUI::WindowManager &wmgr = CEGUI::WindowManager::getSingleton();
	CEGUI::Window *sheet = wmgr.createWindow("DefaultWindow", "_RootWindow");
	CEGUI::System::getSingleton().getDefaultGUIContext().setRootWindow(sheet);

}

bool OgreEngine::setup()
{
#ifdef _DEBUG
	mResourcesCfg = "resources_d.cfg";
	mPluginsCfg = "plugins_d.cfg";
#else
	mResourcesCfg = "resources.cfg";
	mPluginsCfg = "plugins.cfg";
#endif

	mRoot = new Ogre::Root(mPluginsCfg);

	// Load resources
	loadResources(mResourcesCfg);

	// Load config
	if (!(mRoot->restoreConfig() || mRoot->showConfigDialog()))
		return false;

	std::cout << "Render System Selected: " << mRoot->getRenderSystem()->getName() << "\n";

	// Create window and setup scene manager
	mWindow = mRoot->initialise(true, "Ogre Bullet Physics TMJ Simulator");
	mSceneMgr = mRoot->createSceneManager(Ogre::ST_GENERIC);

	// Setup main camera
	setupCamera(mCamera, mSceneMgr, mWindow);

	// Setup CEGUI
	loadCEGUI();

	// Create Scene
	createScene();

	// Load CEGUI controls
	loadCEGUIControls();

	// Load config
	loadConfig("simulation_params.cfg");

	// Create physics World
	setupPhysics("data\\");

	// Setup OIS
	setupOIS();



	return true;
}


void OgreEngine::run()
{
	if (!setup()) return;

	/*double frameTime = 0, lastFrame = 0;
	double frameFraction = (1. / 60.) * 1000.0;

	btClock clock;

	while (!mShutDown)
	{
		mRoot->renderOneFrame();
		Ogre::WindowEventUtilities::messagePump();
		frameTime = clock.getTimeMilliseconds() - lastFrame;
		if (frameTime < frameFraction)
			Sleep(frameFraction - frameTime);
		lastFrame = clock.getTimeMilliseconds();
	}*/

	mRoot->startRendering();

	/*using namespace std::chrono;

	const double frameFraction = (1. / 60.) * 1000.;
	high_resolution_clock::time_point frameTime;
	high_resolution_clock::time_point lastFrame;

	while (!mShutDown)
	{
		mRoot->renderOneFrame();
		Ogre::WindowEventUtilities::messagePump();
		frameTime = high_resolution_clock::now();
		duration<double> passedTime = frameTime - lastFrame;
		if (passedTime.count() < frameFraction)
		{
			auto timeToSleep = frameFraction - passedTime.count();
			boost::this_thread::sleep(boost::posix_time::milliseconds(timeToSleep));
		}
		lastFrame = high_resolution_clock::now();
	}*/
	
}

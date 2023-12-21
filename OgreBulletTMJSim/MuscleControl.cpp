#include "MuscleControl.h"
#include "PhysicsEngine.h"
#include <thread>
#include <ctime>
#include <chrono>
#include "DebugDrawUtils.h"
#include "ThreadPool.h"

MuscleControl::MuscleControl() : allMotorsActive(false), drawForces(false), adaptiveForceLines(true)
{
	fileNames = { "PLATYSMA", "LAT_PTERYGOID_SUP", "LAT_PTERYGOID_INF", "MEDIAL_PTERYGOID", "MASSETER_SUPERIOR", "MASSETER_DEEP",
		"TEMPORALIS_ANTERIOR", "TEMPORALIS_MEDIAL", "TEMPORALIS_POSTERIOR" };
}


std::string MuscleControl::labels[] = { "PLATYSMA", "LAT_PTERYGOID_SUP", "LAT_PTERYGOID_INF", "MEDIAL_PTERYGOID", "MASSETER_SUP", "MASSETER_DEEP",
"TEMPORALIS_ANT", "TEMPORALIS_MED", "TEMPORALIS_POS" };

MuscleControl::~MuscleControl()
{
	gThreads.interrupt_all();
	std::cout << "Calling destructor from muslce control\n";
	disableMotors();
	for (int i = 0; i < mRigidBodyMuscles.size(); i++)
	{
		delete mRigidBodyMuscles[i];
	}
	for (int i = 0; i < mSoftBodyMuscles.size(); i++)
	{
		delete mSoftBodyMuscles[i];
	}

}

bool MuscleControl::onButtonClicked(const CEGUI::EventArgs &e)
{
	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::PushButton* button = (CEGUI::PushButton*) args->window;
	CEGUI::Window* parent = getParentType(button, GWEN_GROUP_BOX);
	int group = parent->getID();
	if (button->getID() == 10)
	{
		if (allMotorsActive) 
		{
			disableMotors();
			button->setText("Active Motors");
		}
		else
		{
			activeMotors();
			button->setText("Deactive Motors");
		}
	}
	switch (button->getID())
	{
	case 1:
	{
		boost::thread* t =
			new boost::thread(&MuscleControl::moveScrollUp, this, getScrollBar(controlWindow, GWEN_HORIZONTAL_SCROLL, group, 1));
		gThreads.add_thread(t);
		//moveScroll(getScrollBar(controlWindow, GWEN_HORIZONTAL_SCROLL, group, 1));
		break;
	}
	case 2:
	{
		boost::thread* t =
			new boost::thread(&MuscleControl::moveScrollDown, this, getScrollBar(controlWindow, GWEN_HORIZONTAL_SCROLL, group, 2));
		gThreads.add_thread(t);
		break;
	}
	case 3:
	{
		switch (group)
		{
		case 11:
		{
			boost::thread* t =
				new boost::thread(&MuscleControl::moveScrollUp, this, getScrollBar(controlWindow, GWEN_HORIZONTAL_SCROLL, group, 1));
			gThreads.add_thread(t);
			boost::this_thread::sleep(boost::posix_time::milliseconds(20));
			boost::thread* t0 =
				new boost::thread(&MuscleControl::moveScrollDown, this, getScrollBar(controlWindow, GWEN_HORIZONTAL_SCROLL, 13, 2));
			gThreads.add_thread(t);
			break;
		}
		case 13:
		{
			boost::thread* t =
				new boost::thread(&MuscleControl::moveScrollUp, this, getScrollBar(controlWindow, GWEN_HORIZONTAL_SCROLL, group, 1));
			gThreads.add_thread(t);
			boost::this_thread::sleep(boost::posix_time::milliseconds(20));
			boost::thread* t0 =
				new boost::thread(&MuscleControl::moveScrollDown, this, getScrollBar(controlWindow, GWEN_HORIZONTAL_SCROLL, 11, 2));
			gThreads.add_thread(t);
			break;
		}
		default:
			break;
		}
		break;
	}
	default:
		break;
	}
	return true;
}


void MuscleControl::moveScrollUp(CEGUI::Scrollbar* bar)
{
	float step = 0.1f;
	controlWindow->hide();
	
	while (bar->getScrollPosition() < bar->getDocumentSize())
	{

		float atual = bar->getScrollPosition();
		bar->setScrollPosition(atual + step);
		//bar->render();
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}

	std::cout << "finalizing thread...\n";
	controlWindow->show();
}

void MuscleControl::moveScrollDown(CEGUI::Scrollbar* bar)
{
	float step = 0.1f;
	controlWindow->hide();
	while (bar->getScrollPosition() > 0)
	{

		float atual = bar->getScrollPosition();
		bar->setScrollPosition(atual - step);
		//bar->render();
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	}

	std::cout << "finalizing thread...\n";
	controlWindow->show();
}

bool MuscleControl::onEditBoxTextAccepted(const CEGUI::EventArgs &e)
{
	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::Editbox* editbox = (CEGUI::Editbox*) args->window;
	if (editbox->getID() == 11)
	{
		softBodyFactor = CEGUI::PropertyHelper<float>::fromString(editbox->getText());
		editbox->setText(std::to_string(softBodyFactor));
	}
	return true;
}

void MuscleControl::assignIds()
{
	/*const int size = 9;
	std::string labels[size] = { "PLATYSMA", "LAT_PTERYGOID_SUP", "LAT_PTERYGOID_INF", "MEDIAL_PTERYGOID", "MASSETER_SUP", "MASSETER_DEEP",
	"TEMPORALIS_ANT", "TEMPORALIS_MED", "TEMPORALIS_POS" };*/
	for (int i = 0; i < mRigidBodyMuscles.size(); i++)
	{
		for (int j = 0; j < size; j++)
		{
			size_t pos = mRigidBodyMuscles[i]->label.find(labels[j]);
			if (pos < labels[j].size()) {
				//std::cout << "Achou o valor de " << labels[j] << std::endl;
				mRigidBodyMuscles[i]->group = j + 1;
				if (mRigidBodyMuscles[i]->label.at(0) == 'R')
				{
					mRigidBodyMuscles[i]->id = 1;
					mRigidBodyMuscles[i]->maxMultiply = getScrollBar(controlWindow, GWEN_VERTICAL_SCROLL, j + 1, 1)->getDocumentSize();
				}
				else
				{
					mRigidBodyMuscles[i]->id = 2;
					mRigidBodyMuscles[i]->maxMultiply = getScrollBar(controlWindow, GWEN_VERTICAL_SCROLL, j + 1, 2)->getDocumentSize();
				}
				
			}
		}
	}
	for (int i = 0; i < mSoftBodyMuscles.size(); i++)
	{
		std::cout << "Label do softbody: " << mSoftBodyMuscles[i]->label << std::endl;
		for (int j = 0; j < size; j++)
		{
			size_t pos = mSoftBodyMuscles[i]->label.find(labels[j]);
			if (pos < labels[j].size()) {
				std::cout << "Achou o valor de " << labels[j] << " para o softbody" << std::endl;
				mSoftBodyMuscles[i]->group = j + 1;
				if (mSoftBodyMuscles[i]->label.at(0) == 'R')
				{
					mSoftBodyMuscles[i]->id = 1;
				}
				else
				{
					mSoftBodyMuscles[i]->id = 2;
				}

			}
		}
	}
}

void MuscleControl::addMuscles(btAlignedObjectArray<rbMuscle*> muscles)
{
	for (int i = 0; i < muscles.size(); i++)
	{
		mRigidBodyMuscles.push_back(muscles[i]);
	}
}

void MuscleControl::assignControls()
{
	/*CEGUI::Window *root = CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow();
	CEGUI::Window * controls = CEGUI::WindowManager::getSingleton().loadLayoutFromFile("forces.layout");
	root->addChild(controls);*/
	/*controlWindow = CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow/MusclesControl");
	setCloseEvent();
	getVerticalScrolbar(controlWindow);*/


	CEGUI::Window *newWindow = CEGUI::WindowManager::getSingleton().loadLayoutFromFile("muscles_control.layout");
	controlWindow = (CEGUI::FrameWindow*) newWindow->getChild(0);
	try {
		CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow")->addChild(controlWindow);
		setCloseEvent();
	} 
	catch (CEGUI::Exception e)
	{
		CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow/MuscleFrame")->destroy();
		CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow")->addChild(controlWindow);
		setCloseEvent();
	}

	setDoubleVerticalScrolbarEvent(controlWindow);
	setDoubleHorizontalScrolbarEvent(controlWindow);
	setButtonEvent(controlWindow);
	setEdiBoxEvent(controlWindow);
	setCheckBoxEvent(controlWindow);
	CEGUI::Editbox* editbox = (CEGUI::Editbox*) getChildByID(controlWindow, GWEN_EDITBOX, 11);
	float documentSize = ((CEGUI::Scrollbar*) getChildByGroupAndID(controlWindow, GWEN_GROUP_BOX, GWEN_VERTICAL_SCROLL, 2, 1))->getDocumentSize();
	//softBodyFactor = 0.0005f / documentSize;
	editbox->setText(std::to_string(softBodyFactor));
	getCheckBox(controlWindow, 10, 12)->setSelected(activeSoft);
	getCheckBox(controlWindow, 10, 13)->setSelected(activeRigid);
}

bool MuscleControl::onDoubleVerticalScrollPositionChanged(const CEGUI::EventArgs &e)
{
	DefaultControl::onDoubleVerticalScrollPositionChanged(e);

	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::Scrollbar* scroll = (CEGUI::Scrollbar*) args->window;
	int id = scroll->getID();
	int group = getParentType(scroll, GWEN_GROUP_BOX)->getID();

	changeActivationFactor(group, id, scroll->getScrollPosition());

	return true;
}

bool MuscleControl::onDoubleHorizontalScrollPositionChanged(const CEGUI::EventArgs &e)
{
	DefaultControl::onDoubleVerticalScrollPositionChanged(e);

	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::Scrollbar* scroll = (CEGUI::Scrollbar*) args->window;
	int id = scroll->getID();
	int group = getParentType(scroll, GWEN_GROUP_BOX)->getID();

	std::cout << "Scroll position: " << scroll->getScrollPosition() << std::endl;

	switch (group)
	{
	case 11:
	{
		int groups[2] = { 2, 3 };
		for (int i = 0; i < 2; i++)
		{
			changeActivationFactor(groups[i], id, scroll->getScrollPosition());
			getScrollBar(controlWindow, GWEN_VERTICAL_SCROLL, groups[i], id)->setScrollPosition(scroll->getScrollPosition());
		}

		break;
	}
	case 12:
	{
		int groups[4/*6*/] = { 4, 5, 6, 7/*, 8, 9 */};
		for (int i = 0; i < 4/*6*/; i++)
		{
			changeActivationFactor(groups[i], id, scroll->getScrollPosition());
			getScrollBar(controlWindow, GWEN_VERTICAL_SCROLL, groups[i], id)->setScrollPosition(scroll->getScrollPosition());
		}

		break;
	}
	case 13:
	{
		int groups[2] = { 8, 9 };
		for (int i = 0; i < 2; i++)
		{
			changeActivationFactor(groups[i], id, scroll->getScrollPosition());
			getScrollBar(controlWindow, GWEN_VERTICAL_SCROLL, groups[i], id)->setScrollPosition(scroll->getScrollPosition());
		}

		break;
	}
	default:
		break;
	}
	return true;
}

bool MuscleControl::getScrollEvent(const CEGUI::EventArgs &e)
{
	/*const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::Scrollbar* scroll = (CEGUI::Scrollbar*) args->window;
	CEGUI::ToggleButton* checkBox = (CEGUI::ToggleButton*) scroll->getParent()->getChild(3);*/

	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::Scrollbar* scroll = (CEGUI::Scrollbar*) args->window;
	CEGUI::ToggleButton* checkBox = (CEGUI::ToggleButton*) scroll->getParent()->getChild(3);
	if (checkBox->isSelected())
	{
		//std::cout << "CheckBox is pushed...\n";
		CEGUI::Scrollbar* scroll2 = (CEGUI::Scrollbar*) scroll->getUserData();
		scroll2->setScrollPosition(scroll->getScrollPosition());
	}
	else
	{
		//std::cout << "CheckBox is NOT pushed...\n";
	}

	std::cout << "Scrollbar value: " << scroll->getScrollPosition() << std::endl;

	int id = scroll->getID();
	int group = scroll->getParent()->getID();

	/*switch (id)
	{
	case 1:
		setSolvers(group, (int)scroll->getScrollPosition());
		break;
	case 2:
		setERP(group, scroll->getScrollPosition());
		break;
	case 3:
		setCFM(group, scroll->getScrollPosition());
		break;
	default:
		break;
	}*/

	//std::cout << scroll->getName() << ": " << scroll->getScrollPosition() << std::endl;
	changeActivationFactor(group, id, scroll->getScrollPosition());

	return true;

}

void MuscleControl::changeActivationFactor(int group, int id, float value)
{
	//std::cout << "Group: " << group << ", ID: " << id << ", Value: " << value << std::endl;
	for (int i = 0; i < mRigidBodyMuscles.size(); i++)
	{
		if (mRigidBodyMuscles[i]->group == group && mRigidBodyMuscles[i]->id == id)
		{
			//std::cout << "ID: " << mRigidBodyMuscles[i]->id << std::endl;
			mRigidBodyMuscles[i]->multiply = value;
		}
	}
	for (int i = 0; i < mSoftBodyMuscles.size(); i++)
	{
		/*std::cout << "SoftBody Group: " << mSoftBodyMuscles[i]->group << std::endl;
		std::cout << "SoftBody ID: " << mSoftBodyMuscles[i]->id << std::endl;*/
		if (mSoftBodyMuscles[i]->group == group && mSoftBodyMuscles[i]->id == id)
		{
			//std::cout << "ID: " << mSoftBodyMuscles[i]->id << std::endl;
			mSoftBodyMuscles[i]->multiply = value;
		}
	}
}

btVector3 MuscleControl::calculeSoftBodyForce(btVector3 currentNodePos, btVector3 nextNodePos, btScalar multiply)
{
	btTransform localTransform = btTransform::getIdentity();
	localTransform.setOrigin(currentNodePos);
	btVector3 nextPosInLocal = localTransform.inverse() (nextNodePos);
	nextPosInLocal = nextPosInLocal * multiply;
	btVector3 nextPosInWorld = localTransform(nextPosInLocal);
	if (currentNodePos.getZ() > nextNodePos.getZ())
		return currentNodePos;
	else
		return nextPosInWorld;

}

bool MuscleControl::onCheckBoxChanged(const CEGUI::EventArgs &e)
{
	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::ToggleButton* checkbox = (CEGUI::ToggleButton*) args->window;
	switch (checkbox->getID())
	{
	case 12:
	{
		if (checkbox->isSelected()) activeSoft = true; else activeSoft = false;
		break;
	}
	case 13:
	{
		if (checkbox->isSelected()) activeRigid = true; else activeRigid = false;
		break;
	}
	default:
		break;
	}
	return true;
}

btVector3 MuscleControl::calculeForceInWorldPosition(rbMuscle* muscle)
{
	btVector3 oriForce = btVector3(
		muscle->orgin.getX() * muscle->multiply,
		muscle->orgin.getY() * muscle->multiply,
		muscle->orgin.getZ() * muscle->multiply);

	btVector3 relOri = muscle->body0->getCenterOfMassTransform().inverse() (muscle->orgin);
	btVector3 force = btVector3(
		relOri.getX() * muscle->multiply,
		relOri.getY() * muscle->multiply,
		relOri.getZ() * muscle->multiply);
	btVector3 worldForce = muscle->body0->getCenterOfMassTransform() (force);
	/*std::cout << "Original Force " << muscle->label << ": " << cvt(oriForce) << std::endl;
	std::cout << "Modified Force " << muscle->label << ": " << cvt(worldForce) << std::endl;*/
	if (drawForces)
	{
		muscle->mDebugDraw->drawSphere(muscle->orgin, 0.2f, btVector3(0, 0, 255));
		muscle->mDebugDraw->drawSphere(muscle->body0->getCenterOfMassPosition(), 0.2f, btVector3(175, 0, 0));
		
		if (adaptiveForceLines)
		{
			btVector3 forceToDraw = force / muscle->maxMultiply;
			forceToDraw = muscle->body0->getCenterOfMassTransform() (forceToDraw);
			muscle->mDebugDraw->drawLine(muscle->body0->getCenterOfMassPosition(), forceToDraw, btVector3(50, 0, 0));
			muscle->mDebugDraw->drawLine(muscle->orgin, forceToDraw, btVector3(0, 255, 0));
		} 
		else if (muscle->multiply < 1)
		{
			muscle->mDebugDraw->drawLine(muscle->body0->getCenterOfMassPosition(), worldForce, btVector3(50, 0, 0));
			muscle->mDebugDraw->drawLine(muscle->orgin, worldForce, btVector3(0, 255, 0));
		}
		else
		{
			muscle->mDebugDraw->drawLine(muscle->body0->getCenterOfMassPosition(), worldForce, btVector3(255, 0, 255));
		}
	}
	return worldForce * 1000;
}

void MuscleControl::applyAllForces()
{
	//std::cout << "Applying forces...\n";
	if (activeRigid)
	{
		for (int i = 0; i < mRigidBodyMuscles.size(); i++)
		{
			if (mRigidBodyMuscles[i]->multiply > 0)
			{
				if (mRigidBodyMuscles[i]->body0) mRigidBodyMuscles[i]->body0->applyCentralForce(calculeForceInWorldPosition(mRigidBodyMuscles[i]));
			}
		}
	}
	if (activeSoft)
	{
		int softMuscleWidth = 10;
		for (int i = 0; i < mSoftBodyMuscles.size(); i++)
		{
			if (mSoftBodyMuscles[i]->multiply > 0)
			{
				// working
				if (mSoftBodyMuscles[i]->perNode)
				{
					btVector3 force = mSoftBodyMuscles[i]->origins[0] * mSoftBodyMuscles[i]->multiply * softBodyFactor * 1000;
					for (int j = 0; j < mSoftBodyMuscles[i]->softbody->m_nodes.size(); j++)
					{
					mSoftBodyMuscles[i]->softbody->m_nodes[j].m_f = force;
					}
				}
				else
				{
					btScalar multiply = btScalar(mSoftBodyMuscles[i]->multiply);
					for (int j = softMuscleWidth; j < mSoftBodyMuscles[i]->softbody->m_nodes.size(); j++)
					{
						btVector3 currentNodePos = mSoftBodyMuscles[i]->softbody->m_nodes[j].m_x;
						btVector3 nextNodePos = mSoftBodyMuscles[i]->softbody->m_nodes[j - softMuscleWidth].m_x;
						if (currentNodePos > nextNodePos) mSoftBodyMuscles[i]->softbody->setMass(j, 0); else
							mSoftBodyMuscles[i]->softbody->m_nodes[j].m_f = calculeSoftBodyForce(currentNodePos, nextNodePos, multiply) * softBodyFactor;
						//mSoftBodyMuscles[i]->softbody->m_nodes[j].m_f = calculeSoftBodyForce(currentNodePos, nextNodePos, multiply);
						//mSoftBodyMuscles[i]->softbody->m_nodes[j].m_x = calculeSoftBodyForce(currentNodePos, nextNodePos, multiply);
						if (softMuscleWidth > 10)
							if (j % 10 == 0)
								softMuscleWidth += 10;
					}
				}
				//mSoftBodyMuscles[i]->softbody->updateConstants();
			}
			/*else
			{
			for (int j = softMuscleWidth; j < mSoftBodyMuscles[i]->softbody->m_nodes.size(); j++)
			{
			mSoftBodyMuscles[i]->softbody->m_nodes[j].m_f = btVector3(0, 0, 0);
			}
			}*/
		}
	}
}

void MuscleControl::activeMotors()
{
	
	std::cout << "Activating motors...\n";
	if (activeRigid)
	{
		for (int i = 0; i < mRigidBodyMuscles.size(); i++)
		{
			// Original implementation
			/*mRigidBodyMuscles[i]->motor.active = true;
			std::thread t(&MuscleControl::activateMotor, this, mRigidBodyMuscles[i]);
			t.detach();*/

			// New implementation
			if (mRigidBodyMuscles[i]->motor2.size() > 0)
			{
				mRigidBodyMuscles[i]->motorsActive = true;
				/*std::thread t(&MuscleControl::activateMotor2, this, mRigidBodyMuscles[i]);
				t.detach();*/

				boost::thread* t =
					new boost::thread(&MuscleControl::activateMotor2, this, mRigidBodyMuscles[i]);
				gThreads.add_thread(t);
				// boost implementation
				/*tPool.enqueue([=]{
					activateMotor2(mRigidBodyMuscles[i]);
					});*/
			}
		}
	}
	if (activeSoft)
	{
		for (int i = 0; i < mSoftBodyMuscles.size(); i++)
		{
			// New implementation
			if (mSoftBodyMuscles[i]->motor2.size() > 0)
			{
				mSoftBodyMuscles[i]->motorsActive = true;
				boost::thread* t =
					new boost::thread(&MuscleControl::activateMotor3, this, mSoftBodyMuscles[i]);
				gThreads.add_thread(t);
			}
		}
	}
	allMotorsActive = true;
}

void MuscleControl::disableMotors()
{
	gThreads.interrupt_all();
	for (int i = 0; i < mRigidBodyMuscles.size(); i++)
	{
		mRigidBodyMuscles[i]->motorsActive = false;
		for (int j = 0; j < mRigidBodyMuscles[i]->motor2.size(); j++)
		{
			mRigidBodyMuscles[i]->motor2[j].active = false;
		}
		mRigidBodyMuscles[i]->multiply = 0;
	}
	allMotorsActive = false;
}

void MuscleControl::deleteMuscles()
{
	disableMotors();
	for (int i = 0; i < mRigidBodyMuscles.size(); i++)
	{
		delete mRigidBodyMuscles[i];
	}
	mRigidBodyMuscles.clear();
}

void MuscleControl::activateMotor(rbMuscle *muscle)
{
	std::cout << "Startin new thread...\n";
	while (muscle->motor.active)
	{
		std::cout << "Muscle multiply: " << muscle->multiply << std::endl;
		if (muscle->multiply <= 0)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(muscle->motor.timeInRest)); // Stay in rest position
			muscle->motor.addForce = true;
		}
		if (muscle->multiply >= muscle->motor.maxForce)
		{
			std::this_thread::sleep_for(std::chrono::milliseconds(muscle->motor.timeInActive)); // Stay in full active position
			muscle->motor.addForce = false;
		}
		if (muscle->motor.addForce)
		{
			if (muscle->multiply < muscle->motor.maxForce) muscle->multiply += muscle->motor.stepForce;
		}
		else
		{
			if (muscle->multiply > 0)muscle->multiply -= muscle->motor.stepForce;
		}
		btVector3 insertion = muscle->body0->getCenterOfMassTransform() (muscle->relInsertion);
		std::cout << "Insertion: " << cvt(insertion) << std::endl;
		std::this_thread::sleep_for(std::chrono::milliseconds(muscle->motor.timeStep)); // Time between next force change
	}
	std::cout << "Finalizating thread...\n";
}

void printMessage(std::string mensage, const std::string &global)
{
	if (mensage.compare(global) != 0)
	{
		std::cout << mensage << std::endl;
	}
}

void MuscleControl::activateMotor2(rbMuscle *muscle)
{
	bool delay = true;  // first execution of motor to sincronize movements
	while (muscle->motorsActive)
	{
		/*std::unique_lock<std::mutex> lck(mtx);
		while (!runPhysics) cv.wait(lck);*/
		for (int i = 0; i < muscle->motor2.size(); i++)
		{
			muscleMotor &motor = muscle->motor2[i];
			motor.active = true;
			std::cout << "Starting motor: " << motor.desc << std::endl;
			if (motor.delayTime > 0 /*&& delay == true*/ && motor.delayFlag == muscleMotor::PRE_SYNC) {
				delay = false;
				boost::this_thread::sleep(boost::posix_time::milliseconds(motor.delayTime));
			}
			//motor.numExec = 0;
			bool addForceInitState = motor.addForce;
			int maxForceChanges = 2 * motor.maxExec;
			int numForceChanges = 0;
			while (motor.active)
			{

				if (muscle->multiply <= motor.minForce)
				{
					
					
					if (!motor.addForce || numForceChanges == 0) {
						boost::this_thread::sleep(boost::posix_time::milliseconds(motor.timeInRest)); // boost implementation
						/*std::this_thread::sleep_for(std::chrono::milliseconds(motor.timeInRest));*/ // Stay in rest position
						numForceChanges++; // addForce is false, and we are changing it, so add to numForceChanges. It will be true only if the initial state was true    
					}
					motor.addForce = true;
				}

				if (muscle->multiply >= motor.maxForce)
				{
					if (motor.addForce) {
						boost::this_thread::sleep(boost::posix_time::milliseconds(motor.timeInActive)); // boost implementation
						/*std::this_thread::sleep_for(std::chrono::milliseconds(motor.timeInActive));*/ // Stay in full active position
						numForceChanges++; // addForce is true, and we are changing it, so add to numForceChanges. It will be false only if the initial state was false
					}
					motor.addForce = false;
				}

				// check if this motor have been fully executed
				if (numForceChanges > 0 && numForceChanges > maxForceChanges)
				{
					motor.active = false; // disable this motor
					std::cout << "Finalizing motor: " << motor.desc << std::endl;
					continue; // jump to the next motor
				}

				// Add or subtract from multiply
				if (motor.addForce)
				{
					if (muscle->multiply < motor.maxForce) muscle->multiply += motor.stepForce;
				}
				else
				{
					if (muscle->multiply > 0) muscle->multiply -= motor.stepForce;
				}
				//getScrollBar(controlWindow, GWEN_VERTICAL_SCROLL, motor.group, motor.id)->setScrollPosition(muscle->multiply);
				boost::this_thread::sleep(boost::posix_time::milliseconds(motor.timeStep)); // boost implementation
				/*std::this_thread::sleep_for(std::chrono::milliseconds(motor.timeStep));*/ // Time between next force change
			} // second while
			if (motor.delayTime > 0 && motor.delayFlag == muscleMotor::POST_SYNC)
			{
				std::cout << "Waiting for the next motor: " << motor.delayTime << std::endl;
				boost::this_thread::sleep(boost::posix_time::milliseconds(motor.delayTime));
			}
		} // loop
	} // first while
}



void MuscleControl::activateMotor3(genericMuscle *muscle)
{
	bool delay = true;  // first execution of motor to sincronize movements
	while (muscle->motorsActive)
	{
		/*std::unique_lock<std::mutex> lck(mtx);
		while (!runPhysics) cv.wait(lck);*/
		for (int i = 0; i < muscle->motor2.size(); i++)
		{
			muscleMotor &motor = muscle->motor2[i];
			motor.active = true;
			std::cout << "Starting motor: " << motor.desc << std::endl;
			if (motor.delayTime > 0 /*&& delay == true*/ && motor.delayFlag == muscleMotor::PRE_SYNC) {
				delay = false;
				boost::this_thread::sleep(boost::posix_time::milliseconds(motor.delayTime));
			}
			//motor.numExec = 0;
			bool addForceInitState = motor.addForce;
			int maxForceChanges = 2 * motor.maxExec;
			int numForceChanges = 0;
			while (motor.active)
			{

				if (muscle->multiply <= motor.minForce)
				{


					if (!motor.addForce || numForceChanges == 0) {
						boost::this_thread::sleep(boost::posix_time::milliseconds(motor.timeInRest)); // boost implementation
						/*std::this_thread::sleep_for(std::chrono::milliseconds(motor.timeInRest));*/ // Stay in rest position
						numForceChanges++; // addForce is false, and we are changing it, so add to numForceChanges. It will be true only if the initial state was true    
					}
					motor.addForce = true;
				}

				if (muscle->multiply >= motor.maxForce)
				{
					if (motor.addForce) {
						boost::this_thread::sleep(boost::posix_time::milliseconds(motor.timeInActive)); // boost implementation
						/*std::this_thread::sleep_for(std::chrono::milliseconds(motor.timeInActive));*/ // Stay in full active position
						numForceChanges++; // addForce is true, and we are changing it, so add to numForceChanges. It will be false only if the initial state was false
					}
					motor.addForce = false;
				}

				// check if this motor have been fully executed
				if (numForceChanges > 0 && numForceChanges > maxForceChanges)
				{
					motor.active = false; // disable this motor
					std::cout << "Finalizing motor: " << motor.desc << std::endl;
					continue; // jump to the next motor
				}

				// Add or subtract from multiply
				if (motor.addForce)
				{
					if (muscle->multiply < motor.maxForce) muscle->multiply += motor.stepForce;
				}
				else
				{
					if (muscle->multiply > 0) muscle->multiply -= motor.stepForce;
				}
				boost::this_thread::sleep(boost::posix_time::milliseconds(motor.timeStep)); // boost implementation
				/*std::this_thread::sleep_for(std::chrono::milliseconds(motor.timeStep));*/ // Time between next force change
			} // second while
			if (motor.delayTime > 0 && motor.delayFlag == muscleMotor::POST_SYNC)
			{
				std::cout << "Waiting for the next motor: " << motor.delayTime << std::endl;
				boost::this_thread::sleep(boost::posix_time::milliseconds(motor.delayTime));
			}
		} // loop
	} // first while
}

btAlignedObjectArray<muscleMotor> MuscleControl::loadMotorsFromFile(std::string filename)
{
	std::ifstream mFile(filename);
	std::string mLine;

	btAlignedObjectArray<muscleMotor> motors;

	while (std::getline(mFile, mLine))
	{
		const char *token = mLine.c_str();
		if (token[0] == '#') continue;

		std::istringstream linestream(mLine);
		std::string item;
		int itemnum = 0;

		// Motor values
		int group; // item 1
		int id; // item 2
		std::string desc; // item 3
		float minForce; // item 4
		float maxForce; // item 5
		float stepForce; // item 6
		int timeStep;  // item 7
		int timeInRest; // item 8
		int timeInActive; // item 9
		bool addForce; // item 10
		int maxExec = 0; // item 11
		int delayTime = 0;
		int delayFlag = 0;

		muscleMotor m;

		while (std::getline(linestream, item, ','))
		{
			itemnum++;
			//cout << "Item #" << itemnum << ": " << item << endl;
			// columns = id, x, y, z, ow, ox, oy, oz, vis, sel, lock, label, desc, associatedNodeID
			if (itemnum == 1) group = std::stoi(item);
			if (itemnum == 2) id = std::stoi(item);
			if (itemnum == 3) desc = item;
			if (itemnum == 4) minForce = std::stof(item);
			if (itemnum == 5) maxForce = std::stof(item);
			if (itemnum == 6) stepForce = std::stof(item);
			if (itemnum == 7) timeStep = std::stoi(item);
			if (itemnum == 8) timeInRest = std::stoi(item);
			if (itemnum == 9) timeInActive = std::stoi(item);
			if (itemnum == 10) 
			{
				//int v = std::stoi(item);
				if (std::stoi(item) == 0) addForce = false; else addForce = true;
			}

			if (itemnum == 11) maxExec = std::stoi(item);
			if (itemnum == 12) delayTime = std::stoi(item);
			if (itemnum == 13) delayFlag = std::stoi(item);
		}

		m.group = group;
		m.id = id;
		m.desc = desc;
		m.minForce = minForce;
		m.maxForce = maxForce;
		m.stepForce = stepForce;
		m.timeStep = timeStep;
		m.timeInRest = timeInRest;
		m.timeInActive = timeInActive;
		m.addForce = addForce;
		m.maxExec = maxExec;
		m.delayTime = delayTime;
		m.delayFlag = delayFlag;

		motors.push_back(m);

	}

	mFile.close();

	return motors;
}

std::vector<appliedForce> MuscleControl::getAppliedForces()
{
	std::vector<appliedForce> forces;
	for (int i = 0; i < size; i++)
	{
		appliedForce lforce;
		appliedForce rforce;
		lforce.label = "LEFT_" + labels[i];
		rforce.label = "RIGHT_" + labels[i];
		for (int j = 0; j < mRigidBodyMuscles.size(); j++)
		{
			if (mRigidBodyMuscles[j]->group == i + 1 && mRigidBodyMuscles[j]->multiply > 0)
			{
				if (mRigidBodyMuscles[j]->id == 1) 
				{
					rforce.totalForce += mRigidBodyMuscles[j]->body0->getTotalForce();
					rforce.lenght += mRigidBodyMuscles[j]->body0->getTotalForce().length();
				}
				else
				{
					lforce.totalForce += mRigidBodyMuscles[j]->body0->getTotalForce();
					lforce.lenght += mRigidBodyMuscles[j]->body0->getTotalForce().length();
				}
			}
		}
		if (lforce.lenght > 0) forces.push_back(lforce);
		if (rforce.lenght > 0) forces.push_back(rforce);
	}
	return forces;
}
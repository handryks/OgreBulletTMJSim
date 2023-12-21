#include "MenuControl.h"
#include "OgreEngine.h"

MenuControl::MenuControl()
{
}


MenuControl::~MenuControl()
{
}


void MenuControl::assignControls()
{
	controlWindow = CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow/Menubar");
	simConfigFrame = CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow/SimulationFrame");
	configureMenu(controlWindow, true);
	setCloseEvent(simConfigFrame);
	setButtonEvent(simConfigFrame);

}

bool MenuControl::onMenuItemClicked(const CEGUI::EventArgs& e)
{
	const CEGUI::WindowEventArgs& we = static_cast<const CEGUI::WindowEventArgs&>(e);
	/*setStatusText("Clicked " + we.window->getName());*/
	std::cout << "Item with id " << we.window->getID() << " clicked in child class\n";
	//if (we.window->getID() == 5) return false;

	switch (we.window->getID())
	{
	case 1:
	{
		mOgreEngine->startPhysics();
		break;
	}
	case 2:
	{
		mOgreEngine->stopPhysics();
		break;
	}
	case 3:
	{
		mOgreEngine->restartPhysics("data\\");
		break;
	}
	case 4:
	{
		if (simConfigFrame->isVisible()) simConfigFrame->activate();
		else
		{
			simConfigFrame->setVisible(true);
			simConfigFrame->activate();
		}
		simulationParams s = mOgreEngine->getSimulationParam();
		getEditBox(simConfigFrame, 0, 1)->setText(std::to_string(s.timeStep));
		getEditBox(simConfigFrame, 0, 2)->setText(std::to_string(s.maxSubSteps));
		getEditBox(simConfigFrame, 0, 3)->setText(std::to_string(s.fixedTimeStep));
		getEditBox(simConfigFrame, 0, 4)->setText(std::to_string(s.solvers));
		if (s.loadDiscs) getCheckBox(simConfigFrame, 0, 5)->setSelected(true); else getCheckBox(simConfigFrame, 0, 5)->setSelected(false);
		if (s.useTetras) getCheckBox(simConfigFrame, 0, 6)->setSelected(true); else getCheckBox(simConfigFrame, 0, 6)->setSelected(false);
		if (s.useSoftLigaments) getCheckBox(simConfigFrame, 0, 7)->setSelected(true); else getCheckBox(simConfigFrame, 0, 7)->setSelected(false);
		if (s.useHDdiscs) getCheckBox(simConfigFrame, 0, 8)->setSelected(true); else getCheckBox(simConfigFrame, 0, 8)->setSelected(false);
		if (s.useGravity) getCheckBox(simConfigFrame, 0, 9)->setSelected(true); else getCheckBox(simConfigFrame, 0, 9)->setSelected(false);
		break;
	}
	case 5:
	{
		mOgreEngine->saveConfig("simulation_params.cfg");
		break;
	}
	case 6:
	{
		CEGUI::Window* window = CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow/MuscleFrame");
		if (window->isVisible()) window->activate(); 
		else
		{
			window->setVisible(true); 
			window->activate();
		}
		break;
	}
	case 7:
	{
		CEGUI::Window* window = CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow/LigamentFrame");
		if (window->isVisible()) window->activate();
		else
		{
			window->setVisible(true);
			window->activate();
		}
		break;
	}
	case 8:
	{
		CEGUI::Window* window = CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow/SoftBodyFrame");
		if (window->isVisible()) window->activate();
		else
		{
			window->setVisible(true);
			window->activate();
		}
		break;
	}
	case 9:
	{
		// Masses Control
		mOgreEngine->startRecording(true);
		break;
	}
	case 10:
	{
		mOgreEngine->startRecording(true);
		break;
	}
	case 11:
	{
		mOgreEngine->toggleRecord();
		if (mOgreEngine->isRecording()) we.window->setText("Stop Recording"); else we.window->setText("Start Recording");
		break;
	}
	case 99:
	{
		//std::cout << "Runing quit button...\n";
		mOgreEngine->quit();
		break;
	}
	default:
		break;
	}
	return true;
}

bool MenuControl::onButtonClicked(const CEGUI::EventArgs &e)
{
	const CEGUI::WindowEventArgs& we = static_cast<const CEGUI::WindowEventArgs&>(e);
	if (we.window->getID() == 7)
	{
		simulationParams s = mOgreEngine->getSimulationParam();
		s.timeStep = CEGUI::PropertyHelper<float>::fromString(getEditBox(simConfigFrame, 0, 1)->getText());
		s.maxSubSteps = CEGUI::PropertyHelper<int>::fromString(getEditBox(simConfigFrame, 0, 2)->getText());
		s.fixedTimeStep = CEGUI::PropertyHelper<float>::fromString(getEditBox(simConfigFrame, 0, 3)->getText());
		s.solvers = CEGUI::PropertyHelper<int>::fromString(getEditBox(simConfigFrame, 0, 4)->getText());
		if (getCheckBox(simConfigFrame, 0, 5)->isSelected()) s.loadDiscs = true; else s.loadDiscs = false;
		if (getCheckBox(simConfigFrame, 0, 6)->isSelected()) s.useTetras = true; else s.useTetras = false;
		if (getCheckBox(simConfigFrame, 0, 7)->isSelected()) s.useSoftLigaments = true; else s.useSoftLigaments = false;
		if (getCheckBox(simConfigFrame, 0, 8)->isSelected()) s.useHDdiscs = true; else s.useHDdiscs = false;
		if (getCheckBox(simConfigFrame, 0, 9)->isSelected()) s.useGravity = true; else s.useGravity = false;
		mOgreEngine->setSimulationParams(s);
		mOgreEngine->restartPhysics();
		simConfigFrame->hide();
	}
	return true;
}
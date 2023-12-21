#include "MassControl.h"
#include <string>
#include <iostream>

MassControl::MassControl()
{
}


MassControl::~MassControl()
{
	std::cout << "Calling destructor from mass control\n";
}


void MassControl::assignControls()
{
	CEGUI::Window *newWindow = CEGUI::WindowManager::getSingleton().loadLayoutFromFile("masses_control.layout");
	controlWindow = (CEGUI::FrameWindow*) newWindow->getChild(0);
	CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow")->addChild(controlWindow);
	setCloseEvent();
}
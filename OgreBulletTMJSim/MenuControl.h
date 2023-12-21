#pragma once
#include "DefaultControl.h"
#include "PhysicsEngine.h"


class OgreEngine;

class MenuControl :
	public DefaultControl
{
protected:
	bool onMenuItemClicked(const CEGUI::EventArgs& e);
	CEGUI::Window* simConfigFrame;

	bool onButtonClicked(const CEGUI::EventArgs &e);

	bool onCloseClicked(const CEGUI::EventArgs &e)
	{
		simConfigFrame->setVisible(false);
		return true;
	}

	

public:
	MenuControl();
	~MenuControl();

	PhysicsEngine* mPhysicsEngine;
	OgreEngine* mOgreEngine;
	void assignControls();

};


#pragma once

#include <btBulletDynamicsCommon.h>
#include <CEGUI\CEGUI.h>
#include <string>
#include "DefaultControl.h"

struct Ligament;

struct ligamentConfig;

class LigamentControl : public DefaultControl
{
private:
	bool draw;

protected:
	//void getScrolbar(CEGUI::Window *window);

public:
	btAlignedObjectArray<Ligament*> mLigaments;
	LigamentControl();
	~LigamentControl();
	
	void assignControls();
	void assignIDs();
	//void assignGroupsAndIDs();
	bool getScrollEvent(const CEGUI::EventArgs &e);
	bool onDoubleHorizontalScrollPositionChanged(const CEGUI::EventArgs &e);
	void setSolvers(int group, int numSolvers);
	void setERP(int group, float erp);
	void setCFM(int group, float cfm);

	void setSolvers(int id, int id2, int numSolvers);
	void setERP(int id, int id2, float erp);
	void setCFM(int id, int id2, float cfm);


	void saveConfig(std::string filename);
	void loadConfig(std::string filename);

	btAlignedObjectArray<ligamentConfig> loadConfigFromFile(std::string filename);

	void drawLigaments();
	void toggleDisplayLigaments()
	{
		if (draw) draw = false; else draw = true;
	}

	void addLigament(btAlignedObjectArray<Ligament*> ligaments);

	void loadValues();

	/*CEGUI::Scrollbar* getScrollBar(CEGUI::Window *rootWindow, int group, int id);*/
};


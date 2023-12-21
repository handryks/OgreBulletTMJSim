#include "LigamentControl.h"
#include "PhysicsEngine.h"
#include <iostream>

LigamentControl::LigamentControl()
{
}


LigamentControl::~LigamentControl()
{
	//std::cout << "Destroing window " << controlWindow->getNamePath() << std::endl;
	std::cout << "Calling destructor from ligament control\n";
	for (int i = 0; i < mLigaments.size(); i++)
	{
		delete mLigaments[i];
	}
}

void LigamentControl::assignControls()
{
	//CEGUI::Window *newWindow = CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow/LigamentsControl");
	//getScrolbar(newWindow);
	/*controlWindow = CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow/LigamentsControl");
	getHorizontalScrolbar(controlWindow);*/
	CEGUI::Window *newWindow = CEGUI::WindowManager::getSingleton().loadLayoutFromFile("ligaments_tab.layout");
	controlWindow = (CEGUI::FrameWindow*) newWindow->getChild(0);
	CEGUI::TabControl* tabControl = (CEGUI::TabControl*) controlWindow->getChild(1);
	for (int i = 1; i < 5; i++)
	{
		CEGUI::Window *tab = newWindow->getChild(i);
		tab->setSize(CEGUI::USize(CEGUI::UDim(1.0f, 0.0f), CEGUI::UDim(1.0f, 0.0f))); // Size to 100% of its parent, the TabControl
		tab->setPosition(CEGUI::UVector2(CEGUI::UDim(0.0f, 0.0f), CEGUI::UDim(0.0f, 0.0f))); // Move to the upper left corner of its parent
		setDoubleHorizontalScrolbarEvent(tab);
		tabControl->addTab(tab);
	}


	CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow")->addChild(controlWindow);
	setCloseEvent();
	
}

void LigamentControl::assignIDs()
{
	std::string labels[4] = { "CAPSULA", "TEMPOROMANDIBULAR", "SPHENOMANDIBULAR", "STYLOMANDIBULAR" };
	for (int i = 0; i < mLigaments.size(); i++)
	{
		for (int j = 0; j < 4; j++)
		{
			size_t pos = mLigaments[i]->label.find(labels[j]);
			if (pos < labels[j].size()) {
				//std::cout << "Achou o valor de " << labels[j] << std::endl;
				mLigaments[i]->id = j + 1;
				if (mLigaments[i]->label.at(0) == 'R') mLigaments[i]->id2 = 1; else mLigaments[i]->id2 = 2;
			}
		}
	}
}

void LigamentControl::loadValues()
{
	inConfigMode = true;
	for (int i = 0; i < mLigaments.size(); i++)
	{

		int group = mLigaments[i]->id;
		int id = mLigaments[i]->id2;
		CEGUI::Window* tabPanel = getChildByID(controlWindow, GWEN_TAB_CONTENT_PANEL, group);

		std::cout << "Grupo: " << group << std::endl;
		std::cout << "ID: " << group << std::endl;

		std::cout << "CFM: " << mLigaments[i]->c1->getParam(BT_CONSTRAINT_CFM) << std::endl;
		std::cout << "ERP: " << mLigaments[i]->c1->getParam(BT_CONSTRAINT_ERP) << std::endl;

		setHorizontalScrollValue(tabPanel, 1, id, mLigaments[i]->c1->getParam(BT_CONSTRAINT_CFM));
		setHorizontalScrollValue(tabPanel, 2, id, mLigaments[i]->c1->getParam(BT_CONSTRAINT_ERP));
	}
	inConfigMode = false;
}


bool LigamentControl::getScrollEvent(const CEGUI::EventArgs &e)
{
	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::Scrollbar* scroll = (CEGUI::Scrollbar*) args->window;
	int id = scroll->getID();
	int group = scroll->getParent()->getID();

	switch (id)
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
	}

	//std::cout << scroll->getName() << ": " << scroll->getScrollPosition() << std::endl;

	return true;
}

bool LigamentControl::onDoubleHorizontalScrollPositionChanged(const CEGUI::EventArgs &e)
{
	if (!inConfigMode)
	{
		DefaultControl::onDoubleHorizontalScrollPositionChanged(e);

		const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
		CEGUI::Scrollbar* scroll = (CEGUI::Scrollbar*) args->window;
		int id = scroll->getID();
		int group = getParentType(scroll, GWEN_GROUP_BOX)->getID();
		int type = getParentType(scroll, GWEN_TAB_CONTENT_PANEL)->getID();

		std::cout << "Ligamento: " << type << ", Paramentro: " << group << ", Lado: " << id << std::endl;

		switch (group)
		{
		case 1:
			setCFM(type, id, scroll->getScrollPosition());
			break;
		case 2:
			setERP(type, id, scroll->getScrollPosition());
			break;
		case 3:
			setSolvers(type, id, (int)scroll->getScrollPosition());
			break;
		default:
			break;
		}
	}
	return true;
}

void LigamentControl::drawLigaments()
{
	if (draw)
	{
		for (int i = 0; i < mLigaments.size(); i++)
		{
			switch (mLigaments[i]->c1->getConstraintType())
			{
			case POINT2POINT_CONSTRAINT_TYPE:
			{
				btPoint2PointConstraint* p2pC = (btPoint2PointConstraint*)mLigaments[i]->c1;
				btVector3 origin = p2pC->getRigidBodyA().getCenterOfMassPosition();
				btVector3 insertion = p2pC->getPivotInA();
				insertion = p2pC->getRigidBodyA().getCenterOfMassTransform() * insertion;
				//origin = p2pC->getRigidBodyA().getCenterOfMassTransform() * origin;

				btVector3 strech = p2pC->getPivotInB();
				strech = p2pC->getRigidBodyB().getCenterOfMassTransform() * strech;
				
				//mLigaments[i]->mDebugDraw->drawLine(origin, insertion, btVector3(0, 255, 0));
				mLigaments[i]->mDebugDraw->drawLine(origin, insertion, btVector3(0, 255, 255));
				mLigaments[i]->mDebugDraw->drawLine(insertion, strech, btVector3(255, 0, 0));
			}
				break;
			default:
				break;
			}

		}
	}
}

void LigamentControl::addLigament(btAlignedObjectArray<Ligament*> ligaments)
{
	for (int i = 0; i < ligaments.size(); i++)
	{
		mLigaments.push_back(ligaments[i]);
	}
}

void LigamentControl::setSolvers(int group, int numSolvers)
{
	std::cout << "Setting solvers: " << numSolvers << std::endl;
	for (int i = 0; i < mLigaments.size(); i++)
	{
		if (mLigaments[i]->id == group)
		{
			//mLigaments[i]->c0->setOverrideNumSolverIterations(numSolvers);
			mLigaments[i]->c1->setOverrideNumSolverIterations(numSolvers -1);
		}
	}
}

void LigamentControl::setSolvers(int id, int id2, int numSolvers)
{
	std::cout << "Setting solvers: " << numSolvers << std::endl;
	for (int i = 0; i < mLigaments.size(); i++)
	{
		if (mLigaments[i]->id == id && mLigaments[i]->id2 == id2)
		{
			//mLigaments[i]->c0->setOverrideNumSolverIterations(numSolvers);
			mLigaments[i]->c1->setOverrideNumSolverIterations(numSolvers - 1);
		}
	}
}

void LigamentControl::setERP(int group, float erp)
{
	std::cout << "Setting ERP: " << erp << std::endl;
	for (int i = 0; i < mLigaments.size(); i++)
	{
		if (mLigaments[i]->id == group)
		{
			//mLigaments[i]->c0->setParam(BT_CONSTRAINT_ERP, btScalar(erp));
			mLigaments[i]->c1->setParam(BT_CONSTRAINT_ERP, btScalar(erp));
		}
	}
}

void LigamentControl::setERP(int id, int id2, float erp)
{
	std::cout << "Setting ERP: " << erp << std::endl;
	for (int i = 0; i < mLigaments.size(); i++)
	{
		if (mLigaments[i]->id == id && mLigaments[i]->id2 == id2)
		{
			//mLigaments[i]->c0->setParam(BT_CONSTRAINT_ERP, btScalar(erp));
			mLigaments[i]->c1->setParam(BT_CONSTRAINT_ERP, btScalar(erp));
		}
	}
}

void LigamentControl::setCFM(int group, float cfm)
{
	std::cout << "Setting CFM: " << cfm << std::endl;
	for (int i = 0; i < mLigaments.size(); i++)
	{
		if (mLigaments[i]->id == group)
		{
			//mLigaments[i]->c0->setParam(BT_CONSTRAINT_CFM, btScalar(cfm));
			mLigaments[i]->c1->setParam(BT_CONSTRAINT_CFM, btScalar(cfm));
		}
	}
}

void LigamentControl::setCFM(int id, int id2, float cfm)
{
	std::cout << "Setting CFM: " << cfm << std::endl;
	for (int i = 0; i < mLigaments.size(); i++)
	{
		if (mLigaments[i]->id == id && mLigaments[i]->id2 == id2)
		{
			//mLigaments[i]->c0->setParam(BT_CONSTRAINT_CFM, btScalar(cfm));
			mLigaments[i]->c1->setParam(BT_CONSTRAINT_CFM, btScalar(cfm));
		}
	}
}

void LigamentControl::saveConfig(std::string filename)
{
	std::ofstream mFile(filename);
	for (int i = 0; i < mLigaments.size(); i++)
	{
		mFile << mLigaments[i]->label << ","
			<< mLigaments[i]->id << ","
			//<< mLigaments[i]->c0->getOverrideNumSolverIterations() << " "
			<< mLigaments[i]->c1->getOverrideNumSolverIterations() << ","
			//<< mLigaments[i]->c0->getParam(BT_CONSTRAINT_ERP) << " "
			<< mLigaments[i]->c1->getParam(BT_CONSTRAINT_ERP) << ","
			//<< mLigaments[i]->c0->getParam(BT_CONSTRAINT_CFM) << " "
			<< mLigaments[i]->c1->getParam(BT_CONSTRAINT_CFM) << "\n";
	}
	mFile.close();
}

void LigamentControl::loadConfig(std::string filename)
{
	inConfigMode = true;
	btAlignedObjectArray<ligamentConfig> cfg = loadConfigFromFile(filename);

	for (int i = 0; i < mLigaments.size(); i++)
	{
		for (int j = 0; j < cfg.size(); j++) {
			if (cfg[j].group == mLigaments[i]->id && cfg[j].id == mLigaments[i]->id2)
			{
				std::cout << "Achou o ligamento para configurar...\n";
				int group = cfg[j].group;
				int id = cfg[j].id;
				CEGUI::Window* tabPanel = getChildByID(controlWindow, GWEN_TAB_CONTENT_PANEL, group);
				if (cfg[j].cfm > 0) {
					setCFM(cfg[j].group, cfg[j].id, cfg[j].cfm);
					setHorizontalScrollValue(tabPanel, 1, cfg[j].id, cfg[j].cfm);
				}
				if (cfg[j].erp > 0) {
					setERP(cfg[j].group, cfg[j].id, cfg[j].erp);
					setHorizontalScrollValue(tabPanel, 2, cfg[j].id, cfg[j].erp);
				}
				if (cfg[j].solvers > 0) {
					setSolvers(cfg[j].group, cfg[j].id, cfg[j].solvers);
					setHorizontalScrollValue(tabPanel, 3, cfg[j].id, cfg[j].solvers);
				}
			}
		}
	}
	inConfigMode = false;
}

//CEGUI::Scrollbar* LigamentControl::getScrollBar(CEGUI::Window *rootWindow, int group, int id)
//{
//	size_t numChilds = rootWindow->getChildCount();
//	for (int i = 0; i < numChilds; i++)
//	{
//		CEGUI::Window* w = rootWindow->getChildAtIdx(i);
//		//std::cout << "Widget Name: " << w->getName() << std::endl;
//		std::cout << "Buscando..." << std::endl;
//		if (w->getType().compare("Vanilla/HorizontalScrollbar") == 0)
//		{
//			if (w->getID() == id && w->getParent()->getID() == group) 
//			{
//				std::cout << "Achou..." << std::endl;
//				return (CEGUI::Scrollbar*) w;
//			}
//		}
//		else if ((w->getType().compare("Vanilla/FrameWindow") == 0 || w->getType().compare("Vanilla/FrameColourRect") == 0) && w->getID() == group)
//		{
//			return getScrollBar(w, group, id);
//		}
//		else
//		{
//			continue;
//		}
//	}
//}


btAlignedObjectArray<ligamentConfig> LigamentControl::loadConfigFromFile(std::string filename)
{
	std::ifstream mFile(filename);
	std::string mLine;

	btAlignedObjectArray<ligamentConfig> ligamentsCfg;

	while (std::getline(mFile, mLine))
	{
		const char *token = mLine.c_str();
		if (token[0] == '#') continue;

		std::istringstream linestream(mLine);
		std::string item;
		int itemnum = 0;

		int group = 0; // item 1
		int id = 0; // item 2
		int solvers = 0; // item 3
		float cfm; // item 4
		float erp; // item 5

		ligamentConfig l;

		while (std::getline(linestream, item, ','))
		{
			itemnum++;
			//cout << "Item #" << itemnum << ": " << item << endl;
			// columns = id, x, y, z, ow, ox, oy, oz, vis, sel, lock, label, desc, associatedNodeID
			if (itemnum == 1) group = std::stoi(item);
			if (itemnum == 2) id = std::stoi(item);
			if (itemnum == 3) solvers = std::stoi(item);
			if (itemnum == 4) cfm = std::stof(item);
			if (itemnum == 5) erp = std::stof(item);
		}

		l.group = group;
		l.id = id;
		l.solvers = solvers;
		l.cfm = cfm;
		l.erp = erp;

		ligamentsCfg.push_back(l);

	}

	mFile.close();

	return ligamentsCfg;

}
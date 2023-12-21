#include "SoftBodyControl.h"
#include <iostream>
#include "PhysicsEngine.h"

SoftBodyControl::SoftBodyControl()
{
}


SoftBodyControl::~SoftBodyControl()
{
	std::cout << "Calling destructor from softbody control\n";
}


std::string SoftBodyControl::labels[] = { "R Disc", "L Disc", "R Retrotissue", "L Retrotissue", "R Muscle", "R AntAttach", "R PosAttach", "L Muscle", "L AntAttach", "L PosAttach" };

void SoftBodyControl::assignControls()
{
	CEGUI::Window *window = CEGUI::WindowManager::getSingleton().loadLayoutFromFile("softbody_control.layout");
	controlWindow = (CEGUI::FrameWindow*) window->getChild("SoftBodyFrame");
	CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow")->addChild(controlWindow);
	CEGUI::TabControl* tabControl = (CEGUI::TabControl*) getChildByID(controlWindow, GWEN_TAB_CONTROL, 0);

	for (int i = 0; i < mSoftBodies.size(); i++)
	{
		std::cout << "Value of i:" << i << std::endl;

		CEGUI::Window* tabPane = new CEGUI::Window(GWEN_TAB_CONTENT_PANEL, "TabPanel" + std::to_string(i));
		tabPane->setID(mSoftBodies[i]->getUserIndex());
		/*std::string *name = static_cast<std::string*>(mSoftBodies[i]->getUserPointer());
		std::string n = *name;
		tabPane->setText(n);*/
		tabPane->setSize(CEGUI::USize(CEGUI::UDim(1.0f, 0.0f), CEGUI::UDim(1.0f, 0.0f)));
		tabPane->setPosition(CEGUI::UVector2(CEGUI::UDim(0.0f, 0.0f), CEGUI::UDim(0.0f, 0.0f)));
		/* to do */
		CEGUI::ScrollablePane* scrollablePane = (CEGUI::ScrollablePane*) window->getChild("ScrollablePane")->clone();
		scrollablePane->setSize(CEGUI::USize(CEGUI::UDim(1.0f, 0.0f), CEGUI::UDim(1.0f, 0.0f)));
		scrollablePane->setPosition(CEGUI::UVector2(CEGUI::UDim(0.0f, 0.0f), CEGUI::UDim(0.0f, 0.0f)));

		tabPane->addChild(scrollablePane);

		CEGUI::Window* tabContent = window->getChild("TabWindow")->clone();
		tabContent->setPosition(CEGUI::UVector2(CEGUI::UDim(0.0f, 0.0f), CEGUI::UDim(0.0f, 0.0f)));

		scrollablePane->addChild(tabContent);

		tabControl->addTab(tabPane);
		std::cout << "Label: " << labels[mSoftBodies[i]->getUserIndex() - 1] << std::endl;
		tabPane->setText(labels[mSoftBodies[i]->getUserIndex() - 1]);
	}
	//CEGUI::Window* tab = window->getChild("TabContentPane");
	//CEGUI::Window* content = window->getChild("TabWindow");
	//content->setPosition(CEGUI::UVector2(CEGUI::UDim(0.0f, 0.0f), CEGUI::UDim(0.0f, 0.0f))); // Move to the upper left corner of its parent
	//tab->getChild("ScrollablePane")->addChild(content);
	//tab->getChild("ScrollablePane")->setSize(CEGUI::USize(CEGUI::UDim(1.0f, 0.0f), CEGUI::UDim(1.0f, 0.0f))); // Size to 100% of its parent, the TabControl
	//tab->setText("Body 01");
	//tab->setSize(CEGUI::USize(CEGUI::UDim(1.0f, 0.0f), CEGUI::UDim(1.0f, 0.0f))); // Size to 100% of its parent, the TabControl
	//tab->setPosition(CEGUI::UVector2(CEGUI::UDim(0.0f, 0.0f), CEGUI::UDim(0.0f, 0.0f))); // Move to the upper left corner of its parent
	//tabControl->addTab(tab);
	//tabControl->setSize(CEGUI::USize(CEGUI::UDim(0.f, 420.0f), CEGUI::UDim(0.f, 800.0f)));
	setHorizontalScrollEditBoxEvent(tabControl);
	setCheckBoxEvent(tabControl);
	setCloseEvent();
	loadSoftBodyValues();
}

void SoftBodyControl::loadSoftBodyValues()
{
	for (int i = 0; i < mSoftBodies.size(); i++)
	{
		int group = mSoftBodies[i]->getUserIndex();
		CEGUI::Window * softBodyPaneConfig = getChildByID(controlWindow, GWEN_TAB_CONTENT_PANEL, group);
		if (mSoftBodies[i]->clusterCount() > 0)
		{
			CEGUI::ToggleButton* checkbox = (CEGUI::ToggleButton*) getChildByGroupAndID(softBodyPaneConfig, GWEN_GROUP_BOX, GWEN_CHECKBOX, 1, 5);
			checkbox->setSelected(true);
			CEGUI::Scrollbar* scroll = (CEGUI::Scrollbar*) getChildByGroupAndID(softBodyPaneConfig, GWEN_GROUP_BOX, GWEN_HORIZONTAL_SCROLL, 1, 2);
			scroll->setDocumentSize(mSoftBodies[i]->m_faces.size());
			scroll->setScrollPosition((float)mSoftBodies[i]->m_clusters.size());
			CEGUI::Editbox* editBox = (CEGUI::Editbox*) getChildByGroupAndID(softBodyPaneConfig, GWEN_GROUP_BOX, GWEN_EDITBOX, 1, 1);
			editBox->setText(std::to_string(mSoftBodies[i]->m_clusters.size()));

			getHorizontalScroll(softBodyPaneConfig, CONTACTS, RIGID_CONTACT_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kSRHR_CL);
			getHorizontalScroll(softBodyPaneConfig, CONTACTS, SOFT_CONTACT_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kSSHR_CL);
			getHorizontalScroll(softBodyPaneConfig, CONTACTS, KINECT_CONTACT_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kSKHR_CL);
		}
		else
		{
			getCheckBox(softBodyPaneConfig, CONSTRUCT, CLUSTER_CHECK)->setSelected(false);
			getHorizontalScroll(softBodyPaneConfig, CONSTRUCT, CLUSTER_SCROLL)->disable();
			getEditBox(softBodyPaneConfig, CONSTRUCT, CLUSTER_BOX)->disable();
			getHorizontalScroll(softBodyPaneConfig, CONSTRUCT, CLUSTER_SCROLL)->setDocumentSize(mSoftBodies[i]->m_nodes.size());
			getHorizontalScroll(softBodyPaneConfig, CONTACTS, RIGID_CONTACT_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kCHR);
			getHorizontalScroll(softBodyPaneConfig, CONTACTS, SOFT_CONTACT_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kKHR);
			getHorizontalScroll(softBodyPaneConfig, CONTACTS, KINECT_CONTACT_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kSHR);
		}

		getHorizontalScroll(softBodyPaneConfig, CONTACTS, ANCHOR_CONTACT_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kAHR);


		if (mSoftBodies[i]->m_pose.m_bframe) {
			getCheckBox(softBodyPaneConfig, POSE, SHAPE_MATCHING_CHECK)->setSelected(true);
			getHorizontalScroll(softBodyPaneConfig, POSE, SHAPE_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kMT);
		}
		else {
			getCheckBox(softBodyPaneConfig, POSE, SHAPE_MATCHING_CHECK)->setSelected(false);
			getHorizontalScroll(softBodyPaneConfig, POSE, SHAPE_SCROLL)->disable();
			getEditBox(softBodyPaneConfig, POSE, SHAPE_BOX)->disable();
		}

		if (mSoftBodies[i]->m_pose.m_bvolume) {
			getCheckBox(softBodyPaneConfig, POSE, VOLUME_MATCHING_CHECK)->setSelected(true);
			getHorizontalScroll(softBodyPaneConfig, POSE, VOLUME_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kVC);
			getHorizontalScroll(softBodyPaneConfig, POSE, VOLUME_RATIO_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.maxvolume);
		}
		else {
			getCheckBox(softBodyPaneConfig, POSE, VOLUME_MATCHING_CHECK)->setSelected(false);
			getHorizontalScroll(softBodyPaneConfig, POSE, VOLUME_SCROLL)->disable();
			getHorizontalScroll(softBodyPaneConfig, POSE, VOLUME_RATIO_SCROLL)->disable();
			getEditBox(softBodyPaneConfig, POSE, VOLUME_BOX)->disable();
			getEditBox(softBodyPaneConfig, POSE, VOLUME_RATIO_BOX)->disable();
		}


		std::cout << "Damping: " << mSoftBodies[i]->m_cfg.kDP << std::endl;

		getHorizontalScroll(softBodyPaneConfig, PROPERTIES, MASS_SCROLL)->setScrollPosition(mSoftBodies[i]->getTotalMass());
		getHorizontalScroll(softBodyPaneConfig, PROPERTIES, DAMPING_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kDP);
		getHorizontalScroll(softBodyPaneConfig, PROPERTIES, FRICTION_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kDF);
		getHorizontalScroll(softBodyPaneConfig, PROPERTIES, LINEAR_STIFNESS_SCROLL)->setScrollPosition(mSoftBodies[i]->m_materials[0]->m_kLST);
		getHorizontalScroll(softBodyPaneConfig, CONTACTS, ANCHOR_CONTACT_SCROLL)->setScrollPosition(mSoftBodies[i]->m_cfg.kAHR);


		getCheckBox(softBodyPaneConfig, CONSTRUCT, BENDING_CHECK)->setSelected(false);
		getHorizontalScroll(softBodyPaneConfig, CONSTRUCT, BENDING_SCROLL)->disable();
		getEditBox(softBodyPaneConfig, CONSTRUCT, BENDING_BOX)->disable();

	}
}

void SoftBodyControl::loadSoftBodyConfig(btSoftBody* softbody)
{
	int id = softbody->getUserIndex();
	CEGUI::Window * pane = getChildByID(controlWindow, GWEN_TAB_CONTENT_PANEL, id);
}

bool SoftBodyControl::onCheckBoxChanged(const CEGUI::EventArgs &e)
{
	std::cout << "CHECKBOX MUDADO....\n";

	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::ToggleButton* checkbox = (CEGUI::ToggleButton*) args->window;
	CEGUI::Window* parent = getParentType(checkbox, GWEN_TAB_CONTENT_PANEL);
	int sbID = parent->getID();
	btSoftBody* body = getSoftBody(sbID);
	switch (checkbox->getID())
	{
	case 5:
	{
		CEGUI::Scrollbar* bar = getHorizontalScroll(parent, CONSTRUCT, CLUSTER_SCROLL);
		CEGUI::Editbox* box = getEditBox(parent, CONSTRUCT, CLUSTER_BOX);
		if (checkbox->isSelected()) {
			bar->setEnabled(true);
			box->setEnabled(true);
			//bar->setScrollPosition(0.f);
			box->setText(std::to_string(0.f));
		}
		else
		{
			bar->setEnabled(false);
			box->setEnabled(false);
		}
		break;
	}
	case 6:
	{
		CEGUI::Scrollbar* bar = getHorizontalScroll(parent, CONSTRUCT, BENDING_SCROLL);
		CEGUI::Editbox* box = getEditBox(parent, CONSTRUCT, BENDING_BOX);
		if (checkbox->isSelected()) {
			bar->setEnabled(true);
			box->setEnabled(true);
			bar->setScrollPosition(0.f);
			box->setText(std::to_string(0.f));
		}
		else
		{
			bar->setEnabled(false);
			box->setEnabled(false);
		}
		break;
	}
	case 7: // Pose
	{
		CEGUI::Scrollbar* bar = getHorizontalScroll(parent, POSE, SHAPE_SCROLL);
		CEGUI::Editbox* box = getEditBox(parent, POSE, SHAPE_BOX);
		bool volume = getCheckBox(parent, POSE, VOLUME_MATCHING_CHECK)->isSelected();
		if (checkbox->isSelected()) {
			body->setPose(volume, true);
			float value = body->m_cfg.kMT;
			bar->setEnabled(true);
			box->setEnabled(true);
			bar->setScrollPosition(value);
			box->setText(std::to_string(value));
		}
		else
		{
			body->setPose(volume, false);
			bar->setEnabled(false);
			box->setEnabled(false);
		}
		break;
	}
	case 8: // Volume
	{
		CEGUI::Scrollbar* bar = getHorizontalScroll(parent, POSE, VOLUME_SCROLL);
		CEGUI::Editbox* box = getEditBox(parent, POSE, VOLUME_BOX);
		CEGUI::Scrollbar* bar2 = getHorizontalScroll(parent, POSE, VOLUME_RATIO_SCROLL);
		CEGUI::Editbox* box2 = getEditBox(parent, POSE, VOLUME_RATIO_BOX);
		bool shape = getCheckBox(parent, POSE, SHAPE_MATCHING_CHECK)->isSelected();
		if (checkbox->isSelected()) {
			body->setPose(shape, true);
			float value1 = body->m_cfg.kVC;
			float value2 = body->m_cfg.maxvolume;
			bar->setEnabled(true);
			bar->setScrollPosition(value1);
			box->setEnabled(true);
			box->setText(std::to_string(value1));
			bar2->setEnabled(true);
			bar2->setScrollPosition(value2);
			box2->setEnabled(true);
			box2->setText(std::to_string(value2));
		}
		else
		{
			body->setPose(shape, false);
			bar->setEnabled(false);
			box->setEnabled(false);
			bar2->setEnabled(false);
			box2->setEnabled(false);
		}
		break;
	}
	default:
		break;
	}
	return true;
}

bool SoftBodyControl::onHorizontalScrollEditBoxChange(const CEGUI::EventArgs &e)
{
	DefaultControl::onHorizontalScrollEditBoxChange(e);

	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	int id = args->window->getID();
	int group = getParentType(args->window, GWEN_GROUP_BOX)->getID();
	int softbody = getParentType(args->window, GWEN_TAB_CONTENT_PANEL)->getID();
	CEGUI::Scrollbar* scroll = 0;
	if (args->window->getType().compare(GWEN_HORIZONTAL_SCROLL) == 0) scroll = (CEGUI::Scrollbar*) args->window;

	

	switch (group)
	{
	case CONSTRUCT:
	{
		switch (id)
		{
		case CLUSTER_SCROLL:
		{
			int clusters = (int)scroll->getScrollPosition();
			if (clusters == 0)
			{
				getSoftBody(softbody)->releaseClusters();
			}
			else if (clusters == scroll->getDocumentSize())
			{
				getSoftBody(softbody)->generateClusters(0);
			}
			else
			{
				getSoftBody(softbody)->generateClusters(clusters);
			}
			std::cout << "Num of clusters: " << getSoftBody(softbody)->clusterCount() << std::endl;
			CEGUI::Editbox* editBox = (CEGUI::Editbox*) scroll->getUserData();
			editBox->setText(std::to_string(getSoftBody(softbody)->clusterCount()));
			break; // break for cluster scroll
		}
		case BENDING_SCROLL:
		{
			std::cout << "LINKS COUNT: " << getSoftBody(softbody)->m_links.size() << std::endl;
			getSoftBody(softbody)->generateBendingConstraints((int)scroll->getScrollPosition());
			std::cout << "LINKS COUNT: " << getSoftBody(softbody)->m_links.size() << std::endl;
			break; // break for bending scroll
		}
		default:
			break;
		}
		break; // break for construct
	}
	case PROPERTIES:
	{
		switch (id)
		{
		case LINEAR_STIFNESS_SCROLL:
		{
			getSoftBody(softbody)->m_materials[0]->m_kLST = scroll->getScrollPosition();
			break; 
		}
		case FRICTION_SCROLL:
		{
			getSoftBody(softbody)->m_cfg.kDF = scroll->getScrollPosition();
			break; 
		}
		case DAMPING_SCROLL:
		{
			getSoftBody(softbody)->m_cfg.kDP = scroll->getScrollPosition();
			break;
		}
		case MASS_SCROLL:
		{
			getSoftBody(softbody)->setTotalMass(scroll->getScrollPosition());
			break;
		}
		default:
			break;
		}
		break; // break for properties
	}
	case POSE:
	{
		switch (id)
		{
		case SHAPE_SCROLL:
		{
			getSoftBody(softbody)->m_cfg.kMT = scroll->getScrollPosition();
			break;
		}
		case VOLUME_SCROLL:
		{
			getSoftBody(softbody)->m_cfg.kVC = scroll->getScrollPosition();
			break;
		}
		case VOLUME_RATIO_SCROLL:
		{
			getSoftBody(softbody)->m_cfg.maxvolume = scroll->getScrollPosition();
			break;
		}
		default:
			break;
		}
		break; // break for pose
	}
	case CONTACTS:
	{
		float value = scroll->getScrollPosition();
		int clusters = getSoftBody(softbody)->clusterCount();
		switch (id)
		{
		case RIGID_CONTACT_SCROLL:
		{
			getSoftBody(softbody)->m_cfg.kCHR = value;
			if (clusters > 0) getSoftBody(softbody)->m_cfg.kSRHR_CL = value;
			break;
		}
		case KINECT_CONTACT_SCROLL:
		{
			getSoftBody(softbody)->m_cfg.kKHR = value;
			if (clusters > 0) getSoftBody(softbody)->m_cfg.kSKHR_CL = value;
			break;
		}
		case SOFT_CONTACT_SCROLL:
		{
			getSoftBody(softbody)->m_cfg.kSHR = value;
			if (clusters > 0) getSoftBody(softbody)->m_cfg.kSSHR_CL = value;
			break;
		}
		case ANCHOR_CONTACT_SCROLL:
		{
			getSoftBody(softbody)->m_cfg.kAHR = value;
			break;
		}
		default:
			break;
		}
		break; // break for contacts
	}
	default:
		break;
	}

	return true;
}

std::vector<softBodyConfig> SoftBodyControl::getSoftBodyConfigs()
{
	std::vector<softBodyConfig> configs;
	softBodyConfig c;
	for (int i = 0; i < mSoftBodies.size(); i++)
	{
		c.index = mSoftBodies[i]->getUserIndex();
		c.mass = mSoftBodies[i]->getTotalMass();
		c.pose = mSoftBodies[i]->m_pose.m_bframe;
		c.cfg.kVCF = mSoftBodies[i]->m_cfg.kVCF;
		c.cfg.kDP = mSoftBodies[i]->m_cfg.kDP;
		c.cfg.kDG = mSoftBodies[i]->m_cfg.kDG;
		c.cfg.kLF = mSoftBodies[i]->m_cfg.kLF;
		c.cfg.kPR = mSoftBodies[i]->m_cfg.kPR;
		c.cfg.kVC = mSoftBodies[i]->m_cfg.kVC;
		c.cfg.kDF = mSoftBodies[i]->m_cfg.kDF;
		c.cfg.kMT = mSoftBodies[i]->m_cfg.kMT;
		c.cfg.kCHR = mSoftBodies[i]->m_cfg.kCHR;
		c.cfg.kKHR = mSoftBodies[i]->m_cfg.kKHR;
		c.cfg.kSHR = mSoftBodies[i]->m_cfg.kSHR;
		c.cfg.kAHR = mSoftBodies[i]->m_cfg.kAHR;
		c.cfg.kSRHR_CL = mSoftBodies[i]->m_cfg.kSRHR_CL;
		c.cfg.kSKHR_CL = mSoftBodies[i]->m_cfg.kSKHR_CL;
		c.cfg.kSSHR_CL = mSoftBodies[i]->m_cfg.kSSHR_CL;
		configs.push_back(c);
	}
	return configs;
}

void SoftBodyControl::setSoftBodyConfigs(std::vector<softBodyConfig> configs)
{
	for (int i = 0; i < configs.size(); i++)
	{
		std::cout << " Index SB: " << configs[i].index << std::endl;
		std::cout << " kAHR SB: " << configs[i].cfg.kAHR << std::endl;
		btSoftBody* s = getSoftBody(configs[i].index);
		if (configs[i].pose) s->setPose(false, true);
		s->setTotalMass(configs[i].mass);
		s->m_cfg.kVCF = configs[i].cfg.kVCF;
		s->m_cfg.kDP = configs[i].cfg.kDP;
		s->m_cfg.kDG = configs[i].cfg.kDG;
		s->m_cfg.kLF = configs[i].cfg.kLF;
		s->m_cfg.kPR = configs[i].cfg.kPR;
		s->m_cfg.kVC = configs[i].cfg.kVC;
		s->m_cfg.kDF = configs[i].cfg.kDF;
		s->m_cfg.kMT = configs[i].cfg.kMT;
		s->m_cfg.kCHR = configs[i].cfg.kCHR;
		s->m_cfg.kKHR = configs[i].cfg.kKHR;
		s->m_cfg.kSHR = configs[i].cfg.kSHR;
		s->m_cfg.kAHR = configs[i].cfg.kAHR;
		s->m_cfg.kSRHR_CL = configs[i].cfg.kSRHR_CL;
		s->m_cfg.kSKHR_CL = configs[i].cfg.kSKHR_CL;
		s->m_cfg.kSSHR_CL = configs[i].cfg.kSSHR_CL;
	}
}
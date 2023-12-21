#pragma once
#include "CEGUI\CEGUI.h"

class DefaultControl
{
protected:
	CEGUI::Window* controlWindow;
	void getVerticalScrolbar(CEGUI::Window *window);
	void getHorizontalScrolbar(CEGUI::Window *window);
	void setDoubleHorizontalScrolbarEvent(CEGUI::Window *window);
	void setDoubleVerticalScrolbarEvent(CEGUI::Window *window);
	void setHorizontalScrollEditBoxEvent(CEGUI::Window *window);
	void setEdiBoxEvent(CEGUI::Window *window);
	void setButtonEvent(CEGUI::Window *window);
	void setCheckBoxEvent(CEGUI::Window *window);
	void configureMenu(CEGUI::Window* pParent, const bool& pMenubar);

	

	virtual bool getScrollEvent(const CEGUI::EventArgs &e);
	virtual bool onDoubleHorizontalScrollPositionChanged(const CEGUI::EventArgs &e);
	virtual bool onDoubleVerticalScrollPositionChanged(const CEGUI::EventArgs &e);
	virtual bool onHorizontalScrollEditBoxChange(const CEGUI::EventArgs &e);

	virtual bool onMouseEntersMenuItem(const CEGUI::EventArgs& e);
	virtual bool onMouseLeavesMenuItem(const CEGUI::EventArgs& e);
	virtual bool onMouseLeavesPopupMenuItem(const CEGUI::EventArgs& e);
	virtual bool onMenuItemClicked(const CEGUI::EventArgs& e);
	virtual bool onEditBoxTextAccepted(const CEGUI::EventArgs& e);
	virtual bool onButtonClicked(const CEGUI::EventArgs& e);
	virtual bool onCheckBoxChanged(const CEGUI::EventArgs& e);

	virtual bool onCloseClicked(const CEGUI::EventArgs &e)
	{
		controlWindow->setVisible(false);
		return true;
	}

	void setCloseEvent()
	{
		CEGUI::FrameWindow* frame = (CEGUI::FrameWindow*) controlWindow;
		frame->subscribeEvent(CEGUI::FrameWindow::EventCloseClicked, CEGUI::Event::Subscriber(&DefaultControl::onCloseClicked, this));
	}

	void setCloseEvent(CEGUI::Window* window)
	{
		CEGUI::FrameWindow* frame = (CEGUI::FrameWindow*) window;
		frame->subscribeEvent(CEGUI::FrameWindow::EventCloseClicked, CEGUI::Event::Subscriber(&DefaultControl::onCloseClicked, this));
	}

	bool inConfigMode;

public:
	std::string DEFAULT_WINDOW = "DefaultWindow";
	std::string GWEN_HORIZONTAL_SCROLL = "GWEN/HorizontalScrollbar";
	std::string GWEN_VERTICAL_SCROLL = "GWEN/VerticalScrollbar";
	std::string GWEN_TAB_CONTENT_PANEL = "GWEN/TabContentPane";
	std::string GWEN_TAB_CONTROL = "GWEN/TabControl";
	std::string GWEN_FRAME_WINDOW = "GWEN/FrameWindow";
	std::string GWEN_GROUP_BOX = "GWEN/GroupBox";
	std::string GWEN_SCROLLABLEPANE = "GWEN/ScrollablePane";
	std::string GWEN_CHECKBOX = "GWEN/Checkbox";
	std::string GWEN_EDITBOX = "GWEN/Editbox";
	std::string GWEN_MENU_ITEM = "GWEN/MenuItem";
	std::string GWEN_MENU_BAR = "GWEN/Menubar";
	std::string GWEN_POPUP_MENU = "GWEN/PopupMenu";
	std::string GWEN_BUTTON = "GWEN/Button";
	
	std::string HORIZONTAL_SCROLL = "Vanilla/HorizontalScrollbar";
	std::string VERTICAL_SCROLL = "Vanilla/VerticalScrollbar";
	std::string FRAME_WINDOW = "Vanilla/FrameWindow";
	std::string FRAME_RECT = "Vanilla/FrameColourRect";

	DefaultControl();
	virtual ~DefaultControl();

	CEGUI::Scrollbar* getScrollBar(CEGUI::Window *rootWindow, std::string TYPE, int group, int id);
	CEGUI::Window * getParentType(CEGUI::Window* window, std::string type);
	CEGUI::Window * getChildByID(CEGUI::Window* window, std::string TYPE, int id);
	CEGUI::Window * getChildByGroupAndID(CEGUI::Window* window, std::string GROUP_TYPE, std::string ID_TYPE, int group, int id);
	
	// Gwen specific functions
	CEGUI::Scrollbar * getHorizontalScroll(CEGUI::Window *window, int group, int id);
	CEGUI::ToggleButton * getCheckBox(CEGUI::Window *window, int group, int id);
	CEGUI::Editbox * getEditBox(CEGUI::Window *window, int group, int id);

	bool isParentType(CEGUI::Window* window, std::string type)
	{
		if (getParentType(window, type) != NULL) return true; else return false;
	}

	virtual void assignControls() = 0;

	void setVerticalScrollValue(CEGUI::Window* window, int group, int id, float value);
	void setHorizontalScrollValue(CEGUI::Window* window, int group, int id, float value);

	void toggleControlsVisibility()
	{
		//CEGUI::Window *controlWindow = CEGUI::System::getSingleton().getDefaultGUIContext().getRootWindow()->getChild("DefaultWindow/MusclesControl");
		bool isVisible = controlWindow->isVisible();
		if (isVisible) controlWindow->setVisible(false); else controlWindow->setVisible(true);
	}

};


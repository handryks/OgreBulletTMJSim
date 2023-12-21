#include "DefaultControl.h"
#include <iostream>

DefaultControl::DefaultControl()
{
}


DefaultControl::~DefaultControl()
{
	std::cout << "Calling destructor from base class\n";
	//std::cout << "Destroing window " << controlWindow->getNamePath() << std::endl;
	CEGUI::WindowManager::getSingleton().destroyWindow(controlWindow);
	//controlWindow->destroy();
}

void DefaultControl::getVerticalScrolbar(CEGUI::Window *window)
{
	size_t numChilds = window->getChildCount();
	if (numChilds == 0) return;
	for (int i = 0; i < numChilds; i++)
	{
		CEGUI::Window* w = window->getChildAtIdx(i);
		if (w->getType().compare(VERTICAL_SCROLL) == 0)
		{
			//std::cout << "Widget Name: " << w->getName() << std::endl;
			w->subscribeEvent(CEGUI::Scrollbar::EventScrollPositionChanged, CEGUI::Event::Subscriber(&DefaultControl::getScrollEvent, this));
			int id = w->getID();
			switch (id)
			{
			case 1:
				w->setUserData(w->getParent()->getChild(2));
				break;
			case 2:
				w->setUserData(w->getParent()->getChild(1));
			default:
				break;
			}

		}
		else if (w->getType().compare(FRAME_WINDOW) == 0 || w->getType().compare(FRAME_RECT) == 0)
		{
			getVerticalScrolbar(w);
		}
	}
}

void DefaultControl::getHorizontalScrolbar(CEGUI::Window *window)
{
	size_t numChilds = window->getChildCount();
	if (numChilds == 0) return;
	for (int i = 0; i < numChilds; i++)
	{
		CEGUI::Window* w = window->getChildAtIdx(i);
		if (w->getType().compare(HORIZONTAL_SCROLL) == 0)
		{
			//std::cout << "Widget Name: " << w->getName() << std::endl;
			w->subscribeEvent(CEGUI::Scrollbar::EventScrollPositionChanged, CEGUI::Event::Subscriber(&DefaultControl::getScrollEvent, this));
		}
		else if (w->getType().compare(FRAME_WINDOW) == 0 || w->getType().compare(FRAME_RECT) == 0)
		{
			getHorizontalScrolbar(w);
		}
	}
}

CEGUI::Window* DefaultControl::getParentType(CEGUI::Window* window, std::string type)
{
	CEGUI::Window* parent = window->getParent();
	if (parent == NULL) return parent;
	if (parent->getType().compare(type) == 0)
	{
		return parent;
	}
	else
	{
		return getParentType(parent, type);
	}
}

void DefaultControl::setDoubleHorizontalScrolbarEvent(CEGUI::Window *window)
{
	size_t numChilds = window->getChildCount();
	if (numChilds == 0) return;
	for (int i = 0; i < numChilds; i++)
	{
		CEGUI::Window* w = window->getChildAtIdx(i);
		if (w->getType().compare(GWEN_HORIZONTAL_SCROLL) == 0)
		{
			//std::cout << "Widget Name: " << w->getName() << std::endl;
			w->subscribeEvent(CEGUI::Scrollbar::EventScrollPositionChanged, CEGUI::Event::Subscriber(&DefaultControl::onDoubleHorizontalScrollPositionChanged, this));
			int id = w->getID();
			switch (id)
			{
			case 1:
				w->setUserData(w->getParent()->getChild(2));
				break;
			case 2:
				w->setUserData(w->getParent()->getChild(1));
			default:
				break;
			}

		}
		else /*if (w->getType().compare(DEFAULT_WINDOW) == 0 || 
			w->getType().compare(GWEN_FRAME_WINDOW) == 0 ||
			w->getType().compare(GWEN_TAB_CONTROL) == 0)*/
		{
			setDoubleHorizontalScrolbarEvent(w);
		}
	}
}

void DefaultControl::setDoubleVerticalScrolbarEvent(CEGUI::Window *window)
{
	size_t numChilds = window->getChildCount();
	if (numChilds == 0) return;
	for (int i = 0; i < numChilds; i++)
	{
		CEGUI::Window* w = window->getChildAtIdx(i);
		if (w->getType().compare(GWEN_VERTICAL_SCROLL) == 0)
		{
			//std::cout << "Widget Name: " << w->getName() << std::endl;
			w->subscribeEvent(CEGUI::Scrollbar::EventScrollPositionChanged, CEGUI::Event::Subscriber(&DefaultControl::onDoubleVerticalScrollPositionChanged, this));
			int id = w->getID();
			switch (id)
			{
			case 1:
				w->setUserData(w->getParent()->getChild(2));
				break;
			case 2:
				w->setUserData(w->getParent()->getChild(1));
			default:
				break;
			}

		}
		else /*if (w->getType().compare(DEFAULT_WINDOW) == 0 ||
			 w->getType().compare(GWEN_FRAME_WINDOW) == 0 ||
			 w->getType().compare(GWEN_TAB_CONTROL) == 0)*/
		{
			setDoubleVerticalScrolbarEvent(w);
		}
	}
}

bool DefaultControl::getScrollEvent(const CEGUI::EventArgs &e)
{
	return true;
}

void DefaultControl::setHorizontalScrollEditBoxEvent(CEGUI::Window *window)
{
	size_t numChilds = window->getChildCount();
	if (numChilds == 0) return;
	for (int i = 0; i < numChilds; i++)
	{
		CEGUI::Window* w = window->getChildAtIdx(i);
		if (w->getType().compare(GWEN_EDITBOX) == 0)
		{
			w->subscribeEvent(CEGUI::Editbox::EventTextAccepted, CEGUI::Event::Subscriber(&DefaultControl::onHorizontalScrollEditBoxChange, this));
			int id = w->getID() + 1;
			w->setUserData(w->getParent()->getChild(id));
		}
		else if (w->getType().compare(GWEN_HORIZONTAL_SCROLL) == 0 && isParentType(w, GWEN_GROUP_BOX))
		{
			w->subscribeEvent(CEGUI::Scrollbar::EventScrollPositionChanged, CEGUI::Event::Subscriber(&DefaultControl::onHorizontalScrollEditBoxChange, this));
			int id = w->getID() - 1;
			w->setUserData(w->getParent()->getChild(id));
			
		}
		else
		{
			setHorizontalScrollEditBoxEvent(w);
		}
	}
}

void DefaultControl::setEdiBoxEvent(CEGUI::Window *window)
{
	size_t numChilds = window->getChildCount();
	if (numChilds == 0) return;
	for (int i = 0; i < numChilds; i++)
	{
		CEGUI::Window* w = window->getChildAtIdx(i);
		if (w->getType().compare(GWEN_EDITBOX) == 0)
		{
			w->subscribeEvent(CEGUI::Editbox::EventTextAccepted, CEGUI::Event::Subscriber(&DefaultControl::onEditBoxTextAccepted, this));
			int id = w->getID() + 1;
		}
		else
		{
			setEdiBoxEvent(w);
		}
	}
}

void DefaultControl::setButtonEvent(CEGUI::Window *window)
{
	size_t numChilds = window->getChildCount();
	if (numChilds == 0) return;
	for (int i = 0; i < numChilds; i++)
	{
		CEGUI::Window* w = window->getChildAtIdx(i);
		if (w->getType().compare(GWEN_BUTTON) == 0)
		{
			w->subscribeEvent(CEGUI::PushButton::EventClicked, CEGUI::Event::Subscriber(&DefaultControl::onButtonClicked, this));
			//int id = w->getID() + 1;
		}
		else
		{
			setButtonEvent(w);
		}
	}
}

void DefaultControl::setCheckBoxEvent(CEGUI::Window *window)
{
	size_t numChilds = window->getChildCount();
	if (numChilds == 0) return;
	for (int i = 0; i < numChilds; i++)
	{
		CEGUI::Window* w = window->getChildAtIdx(i);
		if (w->getType().compare(GWEN_CHECKBOX) == 0)
		{
			w->subscribeEvent(CEGUI::ToggleButton::EventSelectStateChanged, CEGUI::Event::Subscriber(&DefaultControl::onCheckBoxChanged, this));
			//int id = w->getID() + 1;
		}
		else
		{
			setCheckBoxEvent(w);
		}
	}
}

bool DefaultControl::onDoubleHorizontalScrollPositionChanged(const CEGUI::EventArgs &e)
{
	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::Scrollbar* scroll = (CEGUI::Scrollbar*) args->window;
	CEGUI::ToggleButton* checkBox = (CEGUI::ToggleButton*) scroll->getParent()->getChild(3);
	if (!inConfigMode && checkBox->isSelected())
	{
		//std::cout << "CheckBox is pushed...\n";
		CEGUI::Scrollbar* scroll2 = (CEGUI::Scrollbar*) scroll->getUserData();
		scroll2->setScrollPosition(scroll->getScrollPosition());
	}
	else
	{
		//std::cout << "CheckBox is NOT pushed...\n";
	}
	return true;
}

bool DefaultControl::onDoubleVerticalScrollPositionChanged(const CEGUI::EventArgs &e)
{
	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::Scrollbar* scroll = (CEGUI::Scrollbar*) args->window;
	CEGUI::ToggleButton* checkBox = (CEGUI::ToggleButton*) scroll->getParent()->getChild(3);
	if (!inConfigMode && checkBox->isSelected())
	{
		//std::cout << "CheckBox is pushed...\n";
		CEGUI::Scrollbar* scroll2 = (CEGUI::Scrollbar*) scroll->getUserData();
		scroll2->setScrollPosition(scroll->getScrollPosition());
	}
	else
	{
		//std::cout << "CheckBox is NOT pushed...\n";
	}
	return true;
}

bool DefaultControl::onHorizontalScrollEditBoxChange(const CEGUI::EventArgs &e)
{
	const CEGUI::WindowEventArgs* args = static_cast<const CEGUI::WindowEventArgs*>(&e);
	CEGUI::Scrollbar* scroll;
	CEGUI::Editbox* box;
	int id = args->window->getID();
	if (id % 2 == 0)
	{
		scroll = (CEGUI::Scrollbar*) args->window;
		box = (CEGUI::Editbox*) args->window->getUserData();
		if (scroll->getStepSize() == 1.0f)
		{
			box->setText(std::to_string((int)scroll->getScrollPosition()));
		}
		else
		{
			box->setText(std::to_string(scroll->getScrollPosition()));
		}
	}
	else
	{
		box = (CEGUI::Editbox*) args->window;
		scroll = (CEGUI::Scrollbar*) args->window->getUserData();
		scroll->setScrollPosition(CEGUI::PropertyHelper<float>::fromString(box->getText()));
	}


	return true;
}


CEGUI::Window * DefaultControl::getChildByID(CEGUI::Window* window, std::string TYPE, int id)
{
	size_t numChilds = window->getChildCount();
	for (int i = 0; i < numChilds; i++)
	{
		
		CEGUI::Window* w = window->getChildAtIdx(i);
		//std::cout << "Procurando em: " << w->getType() << " com o ID: " << w->getID() << std::endl;
		if (w->getType().compare(TYPE) == 0 && w->getID() == id)
		{
			return w;
		}
		else if (w->getChildCount() > 0)
		{
			//std::cout << "Quantidade de filhos em " << w->getType() << ": " << w->getChildCount() << std::endl;
			CEGUI::Window * w1 = getChildByID(w, TYPE, id);
			if (w1) return w1; else continue;
			//return getChildByID(w, TYPE, id);
		}
		else
		{
			continue;
		}
	}
	return 0;
}


CEGUI::Scrollbar* DefaultControl::getScrollBar(CEGUI::Window *rootWindow, std::string TYPE, int group, int id)
{
	//CEGUI::Window* w0 = getChildByID(rootWindow, GWEN_GROUP_BOX, group);
	////std::cout << "Achou o grupo.... \n";
	//if (w0)
	//{
	//	CEGUI::Scrollbar* w1 = (CEGUI::Scrollbar*) getChildByID(w0, TYPE, id);
	//	return w1;
	//}
	//return 0;
	CEGUI::Scrollbar* w = (CEGUI::Scrollbar*) getChildByGroupAndID(rootWindow, GWEN_GROUP_BOX, TYPE, group, id);
	return w;
}


CEGUI::Window* DefaultControl::getChildByGroupAndID(CEGUI::Window* window, std::string GROUP_TYPE, std::string ID_TYPE, int group, int id)
{
	CEGUI::Window* w0 = getChildByID(window, GROUP_TYPE, group);
	//std::cout << "Achou o grupo.... \n";
	if (w0)
	{
		CEGUI::Scrollbar* w1 = (CEGUI::Scrollbar*) getChildByID(w0, ID_TYPE, id);
		return w1;
	}
	return 0;
}

void DefaultControl::configureMenu(CEGUI::Window* pParent, const bool& pMenubar)
{
	size_t childCount = pParent->getChildCount();
	for (size_t childIdx = 0; childIdx < childCount; childIdx++)
	{
		if (pParent->getChildAtIdx(childIdx)->getType().compare(GWEN_MENU_ITEM) == 0
			|| pParent->getChildAtIdx(childIdx)->getType().compare(GWEN_POPUP_MENU) == 0)
		{
			pParent->getChildAtIdx(childIdx)->subscribeEvent(CEGUI::MenuItem::EventMouseEntersSurface, CEGUI::Event::Subscriber(&DefaultControl::onMouseEntersMenuItem, this));
			pParent->getChildAtIdx(childIdx)->subscribeEvent(CEGUI::MenuItem::EventMouseLeavesSurface, pMenubar ? CEGUI::Event::Subscriber(&DefaultControl::onMouseLeavesMenuItem, this)
				: CEGUI::Event::Subscriber(&DefaultControl::onMouseLeavesPopupMenuItem, this));
			if (pParent->getChildAtIdx(childIdx)->getType().compare(GWEN_MENU_ITEM) == 0)
			{
				pParent->getChildAtIdx(childIdx)->subscribeEvent(CEGUI::MenuItem::EventClicked, CEGUI::Event::Subscriber(&DefaultControl::onMenuItemClicked, this));
			}
		}
		configureMenu(pParent->getChildAtIdx(childIdx), pMenubar);
	}

}


//-------------------------------------------------------------------
bool DefaultControl::onMouseEntersMenuItem(const CEGUI::EventArgs& e)
{
	// Open or close a submenu
	const CEGUI::WindowEventArgs& we = static_cast<const CEGUI::WindowEventArgs&>(e);
	CEGUI::MenuItem* menuItem = static_cast<CEGUI::MenuItem*>(we.window);
	if (menuItem)
	{
		if (menuItem->getPopupMenu())
		{
			if (menuItem->isOpened())
			{
				if (menuItem->getType().compare(GWEN_MENU_ITEM) == 0)
				{
					menuItem->closePopupMenu();
				}
			}
			else
			{
				menuItem->openPopupMenu();
			}
		}
	}
	else
	{
	}
	return true;
}

//-------------------------------------------------------------------
bool DefaultControl::onMouseLeavesMenuItem(const CEGUI::EventArgs& e)
{
	// Close a menu
	const CEGUI::WindowEventArgs& we = static_cast<const CEGUI::WindowEventArgs&>(e);
	//setStatusText("");

	CEGUI::Window* menubar = we.window;
	while (menubar)
	{
		if (menubar->getType().compare(GWEN_MENU_BAR) == 0)
		{
			// We found the root; a menu bar
			CEGUI::Vector2f childPosition = CEGUI::System::getSingleton().getDefaultGUIContext().getMouseCursor().getPosition();
			CEGUI::Window* windowUnderTheCursor = menubar->getTargetChildAtPosition(childPosition);
			if (!windowUnderTheCursor)
			{
				CEGUI::MenuItem* popupMenu = static_cast<CEGUI::Menubar*>(menubar)->getPopupMenuItem();
				if (popupMenu)
				{
					// This does not close sub-popup menus, only the current one
					popupMenu->closePopupMenu();
				}
			}
			break;
		}
		menubar = menubar->getParent();
	}

	return true;
}

//-------------------------------------------------------------------
bool DefaultControl::onMouseLeavesPopupMenuItem(const CEGUI::EventArgs& e)
{
	// Close a popup menu
	const CEGUI::WindowEventArgs& we = static_cast<const CEGUI::WindowEventArgs&>(e);
	//setStatusText("");

	//CEGUI::Window* menubar = we.window;
	//CEGUI::Window* popupParent;
	//while (menubar)
	//{
	//	popupParent = menubar->getParent();
	//	if (popupParent
	//		&& !popupParent->testClassName("Menubar")
	//		&& !popupParent->testClassName("PopupMenu")
	//		&& !popupParent->testClassName("MenuItem")
	//		)
	//	{
	//		// We found the root; a popup menu
	//		CEGUI::Window* popupMenu = menubar;
	//		menubar = menubar->getParent();
	//		CEGUI::UVector2 childPosition = CEGUI::MouseCursor::getSingleton().getPosition();
	//		CEGUI::Window* windowUnderTheCursor = menubar->getTargetChildAtPosition(childPosition);
	//		if (!windowUnderTheCursor)
	//		{
	//			popupMenu->hide();
	//		}
	//		break;
	//	}
	//	menubar = menubar->getParent();
	//}

	return true;
}

bool DefaultControl::onEditBoxTextAccepted(const CEGUI::EventArgs& e)
{
	return true;
}

bool DefaultControl::onButtonClicked(const CEGUI::EventArgs& e)
{
	return true;
}

bool DefaultControl::onCheckBoxChanged(const CEGUI::EventArgs& e)
{
	return true;
}

//-------------------------------------------------------------------
bool DefaultControl::onMenuItemClicked(const CEGUI::EventArgs& e)
{
	const CEGUI::WindowEventArgs& we = static_cast<const CEGUI::WindowEventArgs&>(e);
	/*setStatusText("Clicked " + we.window->getName());*/
	std::cout << "Item clicked\n";
	return true;
}




void DefaultControl::setHorizontalScrollValue(CEGUI::Window* window, int group, int id, float value)
{
	CEGUI::Scrollbar* scroll = getScrollBar(window, GWEN_HORIZONTAL_SCROLL, group, id);
	scroll->setScrollPosition(value);
}

CEGUI::Scrollbar * DefaultControl::getHorizontalScroll(CEGUI::Window *window, int group, int id)
{
	CEGUI::Scrollbar* widget = (CEGUI::Scrollbar*) getChildByGroupAndID(window, GWEN_GROUP_BOX, GWEN_HORIZONTAL_SCROLL, group, id);
	return widget;
}

CEGUI::ToggleButton * DefaultControl::getCheckBox(CEGUI::Window *window, int group, int id)
{
	CEGUI::ToggleButton* widget = (CEGUI::ToggleButton*) getChildByGroupAndID(window, GWEN_GROUP_BOX, GWEN_CHECKBOX, group, id);
	return widget;
}

CEGUI::Editbox * DefaultControl::getEditBox(CEGUI::Window *window, int group, int id)
{
	CEGUI::Editbox* widget = (CEGUI::Editbox*) getChildByGroupAndID(window, GWEN_GROUP_BOX, GWEN_EDITBOX, group, id);
	return widget;
}
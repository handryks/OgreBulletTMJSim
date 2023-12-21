#pragma once

#include "DefaultControl.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody\btSoftBody.h"

class DiscControl : public DefaultControl
{
public:
	DiscControl();
	virtual ~DiscControl();

	btAlignedObjectArray<btSoftBody*> mSoftBodies;
	void setDefaultParams();

	void assignControls();

};


#pragma once
#include "DefaultControl.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody\btSoftBody.h"

struct softBodyConfig;

class SoftBodyControl :
	public DefaultControl
{
public:
	SoftBodyControl();
	~SoftBodyControl();

	std::vector<softBodyConfig> cfg;

	enum groups 
	{
		CONSTRUCT = 1,
		PROPERTIES,
		POSE,
		CONTACTS
	};

	enum construct
	{
		CLUSTER_BOX = 1,
		CLUSTER_SCROLL,
		BENDING_BOX,
		BENDING_SCROLL,
		CLUSTER_CHECK,
		BENDING_CHECK
	};

	enum properties 
	{
		LINEAR_STIFNESS_BOX = 1,
		LINEAR_STIFNESS_SCROLL,
		FRICTION_BOX,
		FRICTION_SCROLL,
		DAMPING_BOX,
		DAMPING_SCROLL,
		MASS_BOX,
		MASS_SCROLL
	};

	enum pose
	{
		SHAPE_BOX = 1,
		SHAPE_SCROLL,
		VOLUME_BOX,
		VOLUME_SCROLL,
		VOLUME_RATIO_BOX,
		VOLUME_RATIO_SCROLL,
		SHAPE_MATCHING_CHECK,
		VOLUME_MATCHING_CHECK
	};

	enum contacts
	{
		RIGID_CONTACT_BOX = 1,
		RIGID_CONTACT_SCROLL,
		KINECT_CONTACT_BOX,
		KINECT_CONTACT_SCROLL,
		SOFT_CONTACT_BOX,
		SOFT_CONTACT_SCROLL,
		ANCHOR_CONTACT_BOX,
		ANCHOR_CONTACT_SCROLL
	};

	enum labels
	{
		RIGHT_DISC,
		LEFT_DISC,
		RIGHT_RETRODISCAL_TISSUE,
		LEFT_RETRODISCAL_TISSUE
	};

	static std::string labels[];

	btAlignedObjectArray<btSoftBody*> mSoftBodies;

	void assignControls();
	void loadSoftBodyValues();
	void loadSoftBodyConfig(btSoftBody* softbody);
	bool onHorizontalScrollEditBoxChange(const CEGUI::EventArgs &e);
	bool onCheckBoxChanged(const CEGUI::EventArgs &e);

	std::vector<softBodyConfig> getSoftBodyConfigs();
	void setSoftBodyConfigs(std::vector<softBodyConfig> configs);

	btSoftBody* getSoftBody(int group)
	{
		for (int i = 0; i < mSoftBodies.size(); i++)
		{
			if (mSoftBodies[i]->getUserIndex() == group) return mSoftBodies[i];
		}
		return 0;
	}

};


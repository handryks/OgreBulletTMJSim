#pragma once

#include <string>
#include <vector>
#include <iostream>
#include <btBulletDynamicsCommon.h>

struct FiducialMark
{

	FiducialMark()
	{
	}
	// columns = id, x, y, z, ow, ox, oy, oz, vis, sel, lock, label, desc, associatedNodeID
	std::string id, label, desc, associatedNodeID;
	int vis, sel, lock;
	float x, y, z, ow, ox, oy, oz;

	btTransform getTransform()
	{
		btTransform transform = btTransform::getIdentity();
		transform.setOrigin(btVector3(x, y, z));
		return transform;
	}

	btVector3 getVector3()
	{
		return btVector3(x, y, z);
	}

};


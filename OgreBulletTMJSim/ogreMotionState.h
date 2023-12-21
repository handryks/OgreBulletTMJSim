#pragma once

#include "LinearMath\btMotionState.h"
#include "Ogre.h"

ATTRIBUTE_ALIGNED16(struct) ogreMotionState : public btMotionState
{
	Ogre::SceneNode* mSceneNode;
	btTransform mInitialPosition;

	ogreMotionState(const btTransform& startTrans = btTransform::getIdentity()) : mInitialPosition(startTrans)
	{
	}

	void setNode(Ogre::SceneNode *node)
	{
		mSceneNode = node;
	}

	Ogre::SceneNode *getNode()
	{
		return mSceneNode;
	}

	virtual void getWorldTransform(btTransform &worldTrans) const
	{
		worldTrans = mInitialPosition;
	}

	virtual void setWorldTransform(const btTransform &worldTrans)
	{
		if (mSceneNode == nullptr)
			return; // silently return before we set a node

		btQuaternion rot = worldTrans.getRotation();
		mSceneNode->setOrientation(rot.w(), rot.x(), rot.y(), rot.z());
		btVector3 pos = worldTrans.getOrigin();
		mSceneNode->setPosition(pos.x(), pos.y(), pos.z());
	}
};
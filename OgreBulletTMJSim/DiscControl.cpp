#include "DiscControl.h"
#include <iostream>
#include "DebugDrawUtils.h"


DiscControl::DiscControl()
{
}


DiscControl::~DiscControl()
{
}

void DiscControl::setDefaultParams()
{
	for (int i = 0; i < mSoftBodies.size(); i++)
	{
		//mSoftBodies[i]->generateBendingConstraints(1);
		mSoftBodies[i]->m_cfg.collisions = btSoftBody::fCollision::CL_SS + btSoftBody::fCollision::CL_RS;

		btSoftBody::Material*	pm = mSoftBodies[i]->appendMaterial();
		/*pm->m_kLST = 0.5;
		pm->m_flags -= btSoftBody::fMaterial::DebugDraw;*/
		//mSoftBodies[i]->generateBendingConstraints(1, pm);
		mSoftBodies[i]->m_cfg.piterations = 2;
		mSoftBodies[i]->m_cfg.citerations = 3;
		mSoftBodies[i]->m_cfg.kDF = 0.5;
		mSoftBodies[i]->m_cfg.kDP = 0.5;
		mSoftBodies[i]->m_cfg.kSHR = 1;
		mSoftBodies[i]->m_cfg.kSRHR_CL = 1;
		mSoftBodies[i]->randomizeConstraints();
		//mSoftBodies[i]->scale(btVector3(6, 6, 6));
		//mSoftBodies[i]->setTotalMass(1.0f, true);
		mSoftBodies[i]->setPose(false, true);
		mSoftBodies[i]->m_cfg.kMT = 0.5;
		mSoftBodies[i]->generateClusters(256);
		mSoftBodies[i]->m_cfg.kAHR = 1;

		//mSoftBodies[i]->setVolumeMass(0.01f);
		//mSoftBodies[i]->setTotalDensity(0.001);
		//mSoftBodies[i]->m_cfg.piterations = 2;
		//mSoftBodies[i]->m_cfg.kVC = 10000;
		//mSoftBodies[i]->generateClusters(16);
		//mSoftBodies[i]->m_materials[0]->m_kLST = 0.8;
		//mSoftBodies[i]->m_materials[0]->m_kVST = 0.7;
		//mSoftBodies[i]->setPose(false, true);
		//mSoftBodies[i]->m_cfg.kMT = 0.6;
		////mSoftBodies[i]->m_cfg.kDP = 0.07;
		////mSoftBodies[i]->m_cfg.kSRHR_CL = 1;
		//std::cout << "Vector Node: " << cvt(mSoftBodies[i]->m_nodes[0].m_x) << "\n";
	}
}


void DiscControl::assignControls()
{

}
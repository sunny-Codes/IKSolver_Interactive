#ifndef SKELETON_H
#define SKELETON_H

#include "BodyNode.h"
#include <vector>

class Skeleton
{
public:
	Skeleton();
	BodyNode* getRootBodyNode()					{return rootBodyNode;}
	void setRootBodyNode(BodyNode* rootBodyNode);
	BodyNode* getBodyNode(std::string name);
	BodyNode* getBodyNode(int index)				{return BodyNodes[index];}
	std::vector<BodyNode*> getBodyNodes()			{return BodyNodes;}
	void addBodyNode(BodyNode* newNode);
	int getNumBodyNodes()							{return BodyNodes.size();}
	int getNumDofs()							{return BodyNodes.size()*3+3;}
	void setPositions(Eigen::VectorXd pos);
	void setPosition(int index, double value);
	Eigen::VectorXd getPositions();
	Eigen::MatrixXd getJacobian(BodyNode* bodyNode, Eigen::Vector3d offset);

private:
	BodyNode* rootBodyNode;
	std::vector<BodyNode*> BodyNodes;
};
// typedef std::shared_ptr<Skeleton> SkeletonPtr;
Eigen::Vector3d QuaternionToAngleAxis(Eigen::Quaterniond qt);
Eigen::Quaterniond AngleAxisToQuaternion(Eigen::Vector3d angleAxis);


#endif	//SKELETON_H


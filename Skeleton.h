#ifndef SKELETON_H
#define SKELETON_H

#include "BodyNode.h"
#include <vector>

class Skeleton
{
public:
	Skeleton();
	BodyNodePtr getRootBodyNode()					{return rootBodyNode;}
	void setRootBodyNode(BodyNodePtr rootBodyNode);
	BodyNodePtr getBodyNode(std::string name);
	BodyNodePtr getBodyNode(int index)				{return BodyNodes[index];}
	std::vector<BodyNodePtr> getBodyNodes()			{return BodyNodes;}
	void addBodyNode(BodyNodePtr newNode);
	int getNumBodyNodes()							{return BodyNodes.size();}
	int getNumDofs()							{return BodyNodes.size()*3+3;}
	void setPositions(Eigen::VectorXd pos);
	void setPosition(int index, double value);
	Eigen::VectorXd getPositions();
	Eigen::MatrixXd getJacobian(BodyNodePtr bodyNode, Eigen::Vector3d offset);

private:
	BodyNodePtr rootBodyNode;
	std::vector<BodyNodePtr> BodyNodes;
};
typedef std::shared_ptr<Skeleton> SkeletonPtr;


#endif	//SKELETON_H


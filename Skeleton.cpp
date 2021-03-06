#include "Skeleton.h"
#include <iostream>
using namespace std;

Skeleton::Skeleton()
{
}

BodyNode* Skeleton::getBodyNode(std::string name)
{
	for(auto bodyNode : BodyNodes)
	{
		if(bodyNode->getName() == name)
			return bodyNode;
	}
	std::cout<<"There is no body named \'"<<name<<"\'"<<endl;
	return NULL;
}

void Skeleton::addBodyNode(BodyNode* newNode)
{
	BodyNodes.push_back(newNode);
}

void Skeleton::setRootBodyNode(BodyNode* rootBodyNode)
{
	this->rootBodyNode = rootBodyNode;
	addBodyNode(rootBodyNode);
}

/// convert quaternion -> log(quaternion)
/// moved to "BasicMath.cpp"
/*
Eigen::Vector3d QuaternionToAngleAxis(Eigen::Quaterniond qt)
{
	double angle = atan2(qt.vec().norm(), qt.w());
	return angle * qt.vec().normalized();
}

/// convert log(quaternion) -> quaternion
Eigen::Quaterniond AngleAxisToQuaternion(Eigen::Vector3d angleAxis)
{
	Eigen::Quaterniond qt;
	qt.vec() = angleAxis.normalized() * sin(angleAxis.norm());
	qt.w() = cos(angleAxis.norm());
	return qt;
}
*/

Eigen::MatrixXd Skeleton::getJacobian(BodyNode* bodyNode, Eigen::Vector3d offset)
{
	Eigen::MatrixXd jacobian(3, getNumDofs());
	jacobian.setZero();
	BodyNode* body = getBodyNodes()[0];
	//Translation of root joint
	jacobian.block<3,3>(0,0) = Eigen::Matrix3d::Identity();
	double h = 1E-3;	//Add h and get the difference -> numerical gradient
	Eigen::Quaterniond qt, qt_h, qt_diff;
	
	qt = body->getTransform().linear();
	Eigen::Vector3d angleAxis_qt, angleAxis_qt_h;
	angleAxis_qt = QuaternionToAngleAxis(qt);

	//Rotation of root joint
	for(int i =0;i<3;i++)
	{
		angleAxis_qt_h = angleAxis_qt;
		angleAxis_qt_h[i] += h;
		qt_h = AngleAxisToQuaternion(angleAxis_qt_h);
		qt_diff = qt_h * qt.inverse();
		jacobian.block<3,1>(0,3+i) = qt_diff.vec().normalized().cross(bodyNode->getTransform()*offset-body->getTransform().translation());
	}


	
	int index = 6;
	for(int i=1;i<getNumBodyNodes();i++)
	{
		body = getBodyNodes()[i];
		// every joint except for root joint is ball joint
		int dof = 3;
		qt = body->getTransform().linear();
		angleAxis_qt = QuaternionToAngleAxis(qt);
		BodyNode* targetBody = bodyNode;
		bool isTargetParent = false;
		while(1)
		{
			if(body->getName()==targetBody->getName())
			{
				isTargetParent = true;
				break;
			}
			if(!targetBody->haveParent())
				break;
			targetBody = targetBody->getParent();
		}
		if(isTargetParent)
		{
			for(int j =0;j<3;j++)
			{
				angleAxis_qt_h = angleAxis_qt;
				angleAxis_qt_h[j] += h;
				qt_h = AngleAxisToQuaternion(angleAxis_qt_h);
				qt_diff = qt_h * qt.inverse();
				jacobian.block<3,1>(0,index+j) 
				= qt_diff.vec().normalized().cross(
					bodyNode->getWorldTransform()*offset-body->getWorldTransform().translation())
					* atan2(qt_diff.vec().norm(), qt_diff.w())/h;
			}
		}
		index += dof;
	}
	return jacobian;
}

// dofs are consist of log(quaternion)
void Skeleton::setDofs(Eigen::VectorXd pos)
{
	if(pos.size()!=getNumDofs())
		cout<<"error in setDofs"<<endl;
	BodyNodes[0]->setWorldTranslation(pos.segment(0,3));
	BodyNodes[0]->setWorldRotation(AngleAxisToQuaternion(pos.segment(3,3)).toRotationMatrix());
	int numBodies = getNumBodyNodes();
	for(int i=1;i<numBodies;i++)
	{
		BodyNodes[i]->setRotation(AngleAxisToQuaternion(pos.segment(3+3*i,3)).toRotationMatrix());
	}
}

void Skeleton::setDof(int index, double value)
{
	Eigen::VectorXd pos = getDofs();
	pos[index] = value;
	setDofs(pos);
}
Eigen::VectorXd Skeleton::getDofs()
{
	Eigen::VectorXd pos(getNumDofs());
	pos.segment(0,3) = BodyNodes[0]->getWorldTransform().translation();
	pos.segment(3,3) = QuaternionToAngleAxis(Eigen::Quaterniond(BodyNodes[0]->getWorldTransform().linear()));
	int numBodies = getNumBodyNodes();
	for(int i=1;i<numBodies;i++)
	{
		pos.segment(3+3*i,3) = QuaternionToAngleAxis(Eigen::Quaterniond(BodyNodes[i]->getTransform().linear()));
	}

	return pos;
}

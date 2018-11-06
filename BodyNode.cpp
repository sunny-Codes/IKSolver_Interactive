#include "BodyNode.h"

BodyNode::BodyNode(std::string name, std::shared_ptr<BodyNode> parentNode)
: name(name), parentNode(parentNode)
{
	transform.setIdentity();
	worldTransform.setIdentity();
	haveParent_ = true;
	joint.parentBodyToJoint.setIdentity();
	joint.childBodyToJoint.setIdentity();
}	

BodyNode::BodyNode(std::string name)
: name(name), parentNode(NULL)
{
	transform.setIdentity();
	worldTransform.setIdentity();
	haveParent_ = false;
	joint.parentBodyToJoint.setIdentity();
	joint.childBodyToJoint.setIdentity();
}

void BodyNode::setShape(float width, float height, float depth)
{
	boxShape.width = width;
	boxShape.height = height;
	boxShape.depth = depth;
}
// We will render links according to world transform.
void BodyNode::updateWorldTransform()
{
	if(haveParent())
	{
		worldTransform = parentNode->getWorldTransform()*joint.parentBodyToJoint * transform;
	}
}
// We should decide 2 parameter for each joint 
// to precisely deterimine the position of joint and the child joint 
// Since every joint except for root joint is ball joint, we don't need to change translation.
// To change the position of child link, we should change the pbtj and cbtj.
void BodyNode::setJoint(Eigen::Isometry3d pbtj, Eigen::Isometry3d cbtj)
{
	joint.parentBodyToJoint = pbtj;
	joint.childBodyToJoint = cbtj;
	transform = cbtj.inverse();
	updateWorldTransform();
}

// Parent have no local transform. So return worldTransform.
Eigen::Isometry3d BodyNode::getTransform()
{
	if(haveParent())
		return transform;
	else
		return worldTransform;
}

void BodyNode::setWorldTranslation(Eigen::Vector3d translation)	//only for root.
{
	worldTransform.translation() = translation;

}

void BodyNode::setWorldRotation(Eigen::Matrix3d rotation)	//only for root
{
	worldTransform.linear() = rotation;
}

void BodyNode::setRotation(Eigen::Matrix3d rotation)
{
	if(haveParent())
	{
		transform.linear() = rotation;
		transform.translation() = -rotation * joint.childBodyToJoint.translation();
		updateWorldTransform();
	}
	else
		setWorldRotation(rotation);

}
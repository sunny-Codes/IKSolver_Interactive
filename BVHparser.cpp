#include <Eigen/Geometry>
#include "BVHparser.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <algorithm>
#define M_PI 3.14159265358979323846

using namespace std;

JointNode::JointNode()
{
	name = "";
	axisOrder = "";
	isRoot = false;
	isEnd = false;
	parent = nullptr;
	next = nullptr;
	childs = vector<JointNode*>();
	channelNum = 0;
}
JointNode* JointNode::getParent()
{
	return parent;
}
vector<JointNode*> JointNode::getChilds()
{
	return childs;
}
void JointNode::setParent(JointNode* pnode)
{
	parent = pnode;
	pnode->addChild(this);
}
void JointNode::addChild(JointNode* cnode)
{
	childs.push_back(cnode);
}
void JointNode::setRoot()
{
	isRoot = true;
}
void JointNode::setEnd()
{
	isEnd = true;
}
void JointNode::setName(string mname)
{
	name = mname;
}
void JointNode::setAxisOrder(string maxisOrder)
{
	axisOrder = maxisOrder;
}
void JointNode::setOffset(float x, float y, float z)
{
	offset[0] = x;
	offset[1] = y;
	offset[2] = z;
}
void JointNode::setNext(JointNode *nextNode)
{
	next = nextNode;
}
void JointNode::setChannelNum(int mchannelNum)
{
	channelNum = mchannelNum;
}
bool JointNode::checkEnd()
{
	return isEnd;
}
string JointNode::getName()
{
	return name;
}
string JointNode::getAxisOrder()
{
	return axisOrder;
}
void JointNode::getOffset(float* outOffset)
{
	for(int i=0;i<3;i++)
	{
		outOffset[i] = offset[i];
	}
}
float JointNode::getOffset(int index)
{
	return offset[index];
}
int JointNode::getChannelNum()
{
	return channelNum;
}
JointNode* JointNode::getNextNode()
{
	return next;
}
void JointNode::initData(int frames)
{
	data = new float*[frames];
	for(int i = 0; i < frames; i++)
	{
		data[i] = new float[channelNum];
	}
}

BVHparser::BVHparser(const char* path)
{
	int lineNum = 0;
	int channelNum = 0;
	string channels[6];
	float offx, offy, offz;
	mPath = path;
	ifstream in;
	in.open(path, ios::in);
	if(!in)
	{
		cerr << "Cannot open "<<path<<endl; exit(1);
	}
	string line;
	getline(in, line);										//HIERARCHY
	lineNum++;

	if(line.compare(0,9,"HIERARCHY")!=0)
	{ 
		cout << "Check the file format. The line number "<<lineNum<<" is not fit to the format"<<endl;
		cout<<line.compare("HIERARCHY")<<endl;
		cout<<line.length()<<endl;
		cout << line<<endl;
		exit(-1);
	}
	getline(in, line);										//ROOT Hips
	lineNum++;
	rootNode = new JointNode();
	rootNode->setRoot();
	
    allNodes= vector<JointNode*>();
    //cout<<"before puting rootNode/ allNodes size: "<<allNodes.size()<<endl;
    allNodes.push_back(rootNode);
    
    istringstream s(line);
	string bvh_keyword;
	string bvh_nodeName;
	s >> bvh_keyword; s >> bvh_nodeName;
	if(bvh_keyword.compare("ROOT")!=0)
	{
		cout << "Check the file format. The line number "<<lineNum<<" is not fit to the format"<<endl;
		cout << line<<endl;
		exit(-1);
	}
	rootNode->setName(bvh_nodeName);
	getline(in, line);										//{

	getline(in, line);										//	OFFSET 0.00 0.00 0.00
	s.str("");
	s = istringstream(line);
	s >> bvh_keyword; s >> offx; s >> offy; s >> offz;
	rootNode->setOffset(offx, offy -90.0, offz);


	getline(in, line);										//	CHANNELS 6 Xposition Yposition Zposition Xrotation Yrotation Zrotation
	s.str("");
	s = istringstream(line);
	s >> bvh_keyword; s >> channelNum;
	string newAxisOrder = "";
	for(int i = 0; i < channelNum; i++)
	{
		s >> channels[i];
		if(channels[i].substr(1) == "rotation")
		{
			transform(channels[i].begin(), channels[i].end(), channels[i].begin(), ::tolower);
			newAxisOrder += channels[i][0];
		}
	}
	rootNode->setAxisOrder(newAxisOrder);
	rootNode->setChannelNum(channelNum);
	JointNode* prevNode = rootNode;
	JointNode* prevNode4NextNode = rootNode;
	getline(in, line);
	while(line.compare(0, 6, "MOTION") != 0)						
	{
		//cout<<endl;
		s.str("");
		s = istringstream(line);
		s >> bvh_keyword; s >> bvh_nodeName;
		if(bvh_keyword=="JOINT")							//	JOINT LeftUpLeg
		{
			JointNode *newNode = new JointNode();
			newNode->setName(bvh_nodeName);
            allNodes.push_back(newNode);
            //cout<<"allNodes:"<<allNodes.size()<<endl;
			//cout << bvh_nodeName <<endl;	//print to check

			getline(in, line);								//	{
			getline(in, line);								//		OFFSET 3.64953 0.00000 0.00000
			s.str("");
			s = istringstream(line);
			s >> bvh_keyword; s >> offx; s >> offy; s >> offz;
			newNode->setOffset(offx, offy, offz);

			getline(in, line);								//		CHANNELS 3 Xrotation Yrotation Zrotation
			s.str("");
			s = istringstream(line);
			s >> bvh_keyword; s >> channelNum;

			newAxisOrder = "";
			for(int i = 0;i < channelNum;i++)
			{
				s >> channels[i];
				if(channels[i].substr(1) == "rotation")
				{
					transform(channels[i].begin(), channels[i].end(), channels[i].begin(), ::tolower);
					newAxisOrder += channels[i][0];
				}
			}
			newNode->setParent(prevNode);
			newNode->setChannelNum(channelNum);
			newNode->setAxisOrder(newAxisOrder);
			prevNode4NextNode->setNext(newNode);
			prevNode = newNode;
			prevNode4NextNode = newNode;	
        }
		else if(bvh_keyword =="End")						//	End Site
		{
			JointNode *newNode = new JointNode();
			allNodes.push_back(newNode);
            //cout<<"allNodes:"<<allNodes.size()<<endl;
            newNode->setName(prevNode->getName()+bvh_nodeName);
			newNode->setEnd();
			getline(in, line);								//	{
			getline(in, line);								//		OFFSET 3.64953 0.00000 0.00000
			s.str("");
			s = istringstream(line);
			s >> bvh_keyword; s >> offx; s >> offy; s >> offz;
			newNode->setOffset(offx, offy, offz);

			newNode->setParent(prevNode);

			getline(in, line);								//	}
		}
		else if(bvh_keyword =="}")
		{
			prevNode = prevNode->getParent();
		}
		else
		{
			cout << "Check the file format. "<< bvh_keyword <<" is right?" <<endl;
			exit(-1);
		}
		getline(in, line);
	}

       // cout<<"after parsing basic structure | allNodes: "<<allNodes.size()<<endl;
// 	Start JointNode										//MOTION
	string str1, str2;	//to get the string for format
	float fvalue;
	getline(in, line);										//Frames: 4692
	s.str("");
	s = istringstream(line);
	s >> str1; s >> fvalue;
	frames = fvalue;

	getline(in, line);										//Frame Time: 0.008333
	s.str("");
	s = istringstream(line);
	s >> str1; s >> str2; s >> fvalue;
	frameTime = fvalue;

	// cout << "frames : " << frames <<", frame time : " << frameTime << endl;

	float f[6];
	JointNode *curNode;
	curNode = rootNode;
	while(curNode != nullptr)
	{
		curNode->initData(frames);
		curNode = curNode->getNextNode();
	}

	for(int i = 0; i < frames; i++)
	{
		curNode = rootNode;
		getline(in, line);
		s.str("");
		s = istringstream(line);
		while(curNode != nullptr)
		{
			for(int j = 0; j < curNode->getChannelNum(); j++)
			{
				s >> f[j];
				curNode->data[i][j] = f[j];
			}
			curNode = curNode->getNextNode();
		}

	}
	Eigen::Quaterniond q;
	Eigen::Vector3d euler;
	curNode = rootNode;
	for(int i = 0; i < frames; i++)
	{
		q = Eigen::Quaterniond::Identity();
		//q = q * Eigen::AngleAxisd(M_PI/2.0, Eigen::Vector3d::UnitY());
		for(int j = 0; j < 3; j++)
		{
			if(curNode->getAxisOrder().substr(j,1).compare("x") == 0)
			{
				q = q * Eigen::AngleAxisd(curNode->data[i][j+3]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitX());
			}
			else if(curNode->getAxisOrder().substr(j,1).compare("y") == 0)
			{
				q = q * Eigen::AngleAxisd(curNode->data[i][j+3]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitY());
			}
			else
			{
				q = q * Eigen::AngleAxisd(curNode->data[i][j+3]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitZ());
			}
		}

		//euler = q.toRotationMatrix().eulerAngles(0, 1, 2); 

		//cout<<"frame : "<< i <<" -> "<<euler[0]<<", "<<euler[1]<<", "<<euler[2]<<", "<<endl;
		//cout << q.w() <<" "<< q.x() <<" "<< q.y() <<" "<< q.z()<<endl;
		/*
		for(int j = 0; j < 3; j++)
		{
			curNode->data[i][j+3] = euler[j];
		}
		*/
		float theta;
		theta = atan2(sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z()), q.w());
		//cout<< q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z() <<endl;
		Eigen::Vector3d root_axis = Eigen::Vector3d(q.x(), q.y(), q.z());
		root_axis.normalize();
		curNode->data[i][3] = theta * root_axis.x();
		curNode->data[i][4] = theta * root_axis.y();
		curNode->data[i][5] = theta * root_axis.z();

		for(int j = 0; j < 3; j++)
		{
			//curNode->data[i][j] -= 50.0f;
		}
	}
	curNode = curNode->getNextNode();
	while(curNode != nullptr)
	{
		for(int i = 0; i < frames; i++)
		{
			q = Eigen::Quaterniond::Identity();
			for(int j = 0; j < 3; j++)
			{
				if(curNode->getAxisOrder().substr(j,1).compare("x") == 0)
				{
					q = q * Eigen::AngleAxisd(curNode->data[i][j]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitX());
				}
				else if(curNode->getAxisOrder().substr(j,1).compare("y") == 0)
				{
					q = q * Eigen::AngleAxisd(curNode->data[i][j]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitY());
				}
				else
				{
					q = q * Eigen::AngleAxisd(curNode->data[i][j]/360.0 * 2.0 * M_PI, Eigen::Vector3d::UnitZ());
				}
			}


			//euler = q.toRotationMatrix().eulerAngles(0, 1, 2);		//#######changed######
			float theta;
			theta = atan2(sqrt(q.x()*q.x() + q.y()*q.y() + q.z()*q.z()), q.w());
			//cout<< q.w()*q.w() + q.x()*q.x() + q.y()*q.y() + q.z()*q.z() <<endl;
			Eigen::Vector3d root_axis = Eigen::Vector3d(q.x(), q.y(), q.z());
			root_axis.normalize();
			curNode->data[i][0] = theta * root_axis.x();
			curNode->data[i][1] = theta * root_axis.y();
			curNode->data[i][2] = theta * root_axis.z();

			// for(int j = 0; j < 3; j++)
			// {
			// 	curNode->data[i][j] = euler[j];
			// }
			//curNode->data[i][2] = - euler[2];
		}
		curNode = curNode->getNextNode();
	}

    cout<<"after parsing all | "<<path<<" | allNodes size= "<<allNodes.size()<<endl;
}

JointNode* BVHparser::getRootNode()
{
	return rootNode;
}

JointNode* BVHparser::getNode(int nodeNum)
{
    return allNodes[nodeNum];
}


const char* BVHparser::getPath()
{
	return mPath;
}

string bvhpath_2_skelpath_(const char* path)
{
	string pathString; pathString = path;
	if(pathString.find(".bvh") != std::string::npos)
	{
		pathString.erase(pathString.find(".bvh"), 5);
	}
	return pathString;
}

string getFileName_(const char* path)
{
	string pathString; pathString = path;
	int cur = 0;
	while(pathString.find("/") != std::string::npos)
	{
		pathString.erase(0, pathString.find("/")+1);
	}

	return pathString.substr(0, pathString.find("."));
}
float max_3(float a, float b, float c)
{
	float maximum = a;
	if(b>= maximum)
		maximum = b;
	if(c>= maximum)
		maximum = c;
	return maximum;
}

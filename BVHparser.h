#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <vector>
#include <string>
using namespace std;
#ifndef JointNode_H
#define JointNode_H

class JointNode
{
	string name;
	bool isRoot;
	bool isEnd;
	JointNode* parent;
	vector<JointNode*> childs;
	float offset[3];
	string axisOrder;
	int channelNum;
	JointNode* next;
public :
	float **data;
	JointNode();
	JointNode* getParent();
	vector<JointNode*> getChilds();
	void setParent(JointNode* pnode);
	void addChild(JointNode* cnode);
	void setRoot();
	void setEnd();
	void setName(string mname);
	void setAxisOrder(string maxisOrder);
	void setOffset(float x, float y, float z);
	void setNext(JointNode *nextNode);
	void setChannelNum(int mchannelNum);
	bool checkEnd();
	string getName();
	string getAxisOrder();
	void getOffset(float* outOffset);
	float getOffset(int index);
	int getChannelNum();
	JointNode* getNextNode();
	void initData(int frames);
};
#endif
#ifndef BVHparser_H
#define BVHparser_H
class BVHparser
{
	const char* mPath;
	JointNode* rootJoint;
	vector<JointNode*> allJoints;
	bool lowerBodyOnly;
	bool upperBodyOnly;
public :
	BVHparser(const char* path);
	int frames;
	float frameTime;
	JointNode* getRootJoint();
	JointNode* getJoint(int nodeNum);
	const char* getPath();
	void writeSkelFile();
	void useLowerBodyOnly();
	void useUpperBodyOnly();
	bool isLowerBodyOnly();
	bool isUpperBodyOnly();
    vector<JointNode*> getAllJoints(){return allJoints;}
    int get_allJoints_size(){return allJoints.size();}
};
#endif



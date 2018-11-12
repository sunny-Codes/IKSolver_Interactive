#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <vector>
#include <string>
using namespace std;
#ifndef MotionNode_H
#define MotionNode_H

class MotionNode
{
	string name;
	bool isRoot;
	bool isEnd;
	MotionNode* parent;
	vector<MotionNode*> childs;
	float offset[3];
	string axisOrder;
	int channelNum;
	MotionNode* next;
public :
	float **data;
	MotionNode();
	MotionNode* getParent();
	vector<MotionNode*> getChilds();
	void setParent(MotionNode* pnode);
	void addChild(MotionNode* cnode);
	void setRoot();
	void setEnd();
	void setName(string mname);
	void setAxisOrder(string maxisOrder);
	void setOffset(float x, float y, float z);
	void setNext(MotionNode *nextNode);
	void setChannelNum(int mchannelNum);
	bool checkEnd();
	string getName();
	string getAxisOrder();
	void getOffset(float* outOffset);
	float getOffset(int index);
	int getChannelNum();
	MotionNode* getNextNode();
	void initData(int frames);
};
#endif
#ifndef BVHparser_H
#define BVHparser_H
class BVHparser
{
	const char* mPath;
	MotionNode* rootNode;
	bool lowerBodyOnly;
	bool upperBodyOnly;
public :
	BVHparser(const char* path);
	int frames;
	float frameTime;
	MotionNode* getRootNode();
	const char* getPath();
	void writeSkelFile();
	void useLowerBodyOnly();
	void useUpperBodyOnly();
	bool isLowerBodyOnly();
	bool isUpperBodyOnly();
};
#endif
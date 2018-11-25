#include "BVHmanager.h"
#include "BVHparser.h"
#include <string.h>
#define MOTION_STATE_STOP 10
#define MOTION_STATE_LEFT_FOOT 11
#define MOTION_STATE_RIGHT_FOOT 12

MotionSegment::MotionSegment(BVHparser * _bvhParser, const char* _motion_name, int _start, int _end, int _start_state, int _end_state): bvhParser(_bvhParser), motion_name(_motion_name), start(_start), end(_end), start_state(_start_state), end_state(_end_state){}

//get functions
Vector3d MotionSegment::get_current_position(int nodeNum, int frameTime){
    double x= bvhParser->getNode(nodeNum)->data[frameTime][0];
    double y= bvhParser->getNode(nodeNum)->data[frameTime][1];
    double z= bvhParser->getNode(nodeNum)->data[frameTime][2];
    return Vector3d(x,y,z);
}
Vector3d MotionSegment::get_current_rotation(int nodeNum, int frameTime){
    double x= bvhParser->getNode(nodeNum)->data[frameTime][3];
    double y= bvhParser->getNode(nodeNum)->data[frameTime][4];
    double z= bvhParser->getNode(nodeNum)->data[frameTime][5];
    return Vector3d(x,y,z);
}

Vector3d MotionSegment::get_start_position(int nodeNum){
    return get_current_position(nodeNum, start);
}

Vector3d MotionSegment::get_start_rotation(int nodeNum){
    return get_current_rotation(nodeNum, start);
}

Vector3d MotionSegment::get_end_position(int nodeNum){
    return get_current_position(nodeNum, end);
}
Vector3d MotionSegment::get_end_rotation(int nodeNum){
    return get_current_rotation(nodeNum, end);
}

Vector3d MotionSegment::get_current_position_displacement(int nodeNum, int frameTime){
	return get_current_position(nodeNum, frameTime)- get_current_position(nodeNum, start);
}


int MotionSegment::get_start(){return start;}
int MotionSegment::get_end(){return end;}
int MotionSegment::get_start_state(){return start_state;}
int MotionSegment::get_end_state(){return end_state;}
const char*  MotionSegment::get_motion_name(){return motion_name;}

VectorXd MotionSegment::get_Skeleton_positions(int frameTime){
	VectorXd positions;

	JointNode* curNode= bvhParser->getRootNode();
	int i=0;

	Vector3d root_position = Vector3d(curNode->data[frameTime][0], curNode->data[frameTime][1], curNode->data[frameTime][2]);
    positions.segment(i,3)= root_position;
    i+= 3;
	Vector3d root_rotation = Vector3d(curNode->data[frameTime][3], curNode->data[frameTime][4], curNode->data[frameTime][5]);
	positions.segment(i,3)= root_rotation;
	i+= 3;

	curNode= curNode->getNextNode();
	while(curNode!=nullptr)
	{
		if(curNode->checkEnd())
		{
			curNode = curNode->getNextNode();
			continue;
		}
		Vector3d rotation = Vector3d(curNode->data[frameTime][0], curNode->data[frameTime][1], curNode->data[frameTime][2]);
		positions.segment(i,3)= rotation; // angle-axis
		curNode = curNode->getNextNode();
        i+= 3;
    }
	return positions;
}

VectorXd MotionSegment::get_Skeleton_end_positions(){
	return get_Skeleton_positions(end);
}
void MotionSegment::set_Skeleton_positions(int frameTime, Skeleton * skel){
	JointNode* curNode= bvhParser->getRootNode();
	int i=0;

	Vector3d root_position = Vector3d(curNode->data[frameTime][0], curNode->data[frameTime][1], curNode->data[frameTime][2]);
	skel->getRootBodyNode()->setWorldTranslation(root_position);

	Vector3d root_rotation = Vector3d(curNode->data[frameTime][3], curNode->data[frameTime][4], curNode->data[frameTime][5]);
    skel->getRootBodyNode()->setWorldRotation_v(root_rotation);

    curNode= curNode->getNextNode();

	while(curNode!=nullptr)
	{
		if(curNode->checkEnd())
		{
			curNode = curNode->getNextNode();
			continue;
		}
		Vector3d rotation = Vector3d(curNode->data[frameTime][0], curNode->data[frameTime][1], curNode->data[frameTime][2]);
		skel->getBodyNode(curNode->getName())->setRotation_v(rotation);
		curNode = curNode->getNextNode();
	}

}


BVHmanager::BVHmanager()
{
	BVHparser* bvhParser = newBVHparser("../MotionData2/mrl/walk_fast_stright.bvh");
    newMotionSegment(bvhParser, "walk_fast", 107, 137, MOTION_STATE_RIGHT_FOOT, MOTION_STATE_RIGHT_FOOT);

    bvhParser = newBVHparser("../MotionData2/mrl/walk_normal_stright.bvh");
    newMotionSegment(bvhParser, "walk_normal", 100, 139, MOTION_STATE_RIGHT_FOOT, MOTION_STATE_RIGHT_FOOT);
    newMotionSegment(bvhParser, "walk_start", 66, 107, MOTION_STATE_STOP, MOTION_STATE_RIGHT_FOOT);
    newMotionSegment(bvhParser, "walk_stop_left", 588, 641, MOTION_STATE_LEFT_FOOT, MOTION_STATE_STOP);
    newMotionSegment(bvhParser, "walk_stop_right", 84, 897, MOTION_STATE_RIGHT_FOOT, MOTION_STATE_STOP);
    newMotionSegment(bvhParser, "stop", 641, 641, MOTION_STATE_STOP, MOTION_STATE_STOP);
    newMotionSegment(bvhParser, "walk_left_to_right", 119, 139, MOTION_STATE_LEFT_FOOT, MOTION_STATE_RIGHT_FOOT);
    newMotionSegment(bvhParser, "walk_right_to_left", 100, 119, MOTION_STATE_RIGHT_FOOT, MOTION_STATE_LEFT_FOOT);

    bvhParser = newBVHparser("../MotionData2/mrl/walk_slow_stright.bvh");
	newMotionSegment(bvhParser, "walk_slow", 170, 221, MOTION_STATE_RIGHT_FOOT, MOTION_STATE_RIGHT_FOOT);
   
    bvhParser = newBVHparser("../MotionData2/mrl/walk_normal_left_45.bvh");
	newMotionSegment(bvhParser, "left_45", 119, 180, MOTION_STATE_LEFT_FOOT, MOTION_STATE_RIGHT_FOOT);
   
    bvhParser = newBVHparser("../MotionData2/mrl/walk_normal_left_90.bvh");
	newMotionSegment(bvhParser, "left_90", 96, 159, MOTION_STATE_LEFT_FOOT, MOTION_STATE_RIGHT_FOOT);
   
    bvhParser = newBVHparser("../MotionData2/mrl/walk_normal_left_135.bvh");
	newMotionSegment(bvhParser, "left_135", 100, 173, MOTION_STATE_LEFT_FOOT, MOTION_STATE_RIGHT_FOOT);

    bvhParser = newBVHparser("../MotionData2/mrl/walk_normal_right_45.bvh");
	newMotionSegment(bvhParser, "right_45", 84, 138, MOTION_STATE_LEFT_FOOT, MOTION_STATE_RIGHT_FOOT);

    bvhParser = newBVHparser("../MotionData2/mrl/walk_normal_right_90.bvh");
    newMotionSegment(bvhParser, "right_90", 91, 149, MOTION_STATE_LEFT_FOOT, MOTION_STATE_RIGHT_FOOT);

    bvhParser = newBVHparser("../MotionData2/mrl/walk_normal_right_135.bvh");
	newMotionSegment(bvhParser, "right_135", 96, 156, MOTION_STATE_LEFT_FOOT, MOTION_STATE_RIGHT_FOOT);
    cout<<"BVHmanager constructor end"<<endl;
    cout<<"bvhParser_list: "<<bvhParser_list.size()<<endl;

    cout<<"motionSegment_list: "<<motionSegment_list.size()<<endl;


}

BVHparser* BVHmanager::newBVHparser(const char* action){
    BVHparser* bvhParser= new BVHparser(action);
    bvhParser_list.push_back(*bvhParser);
    return bvhParser;
}

void BVHmanager::newMotionSegment(BVHparser* bvhparser, const char* motion_name, int start, int end, int start_state, int end_state){
    cout<<"newMotionSegment/ motion_name: "<<motion_name<<endl;
     cout<<"bvhparser: "<<bvhparser->getPath()<<endl;
   MotionSegment* motionSegment= new MotionSegment(bvhparser, motion_name, start, end, start_state, end_state);
    motionSegment_list.push_back(*motionSegment);
    cout<<"motionSegment, all nodes;"<<motionSegment->get_all_nodes_size()<<endl;
    cout<<motion_name<<" constructed using "<<bvhparser->getPath()<<" , allNodes:"<<bvhparser->get_all_nodes_size()<<endl;
}

MotionSegment* BVHmanager::getMotionSegment(const char*action){
	for(int i=0;i<motionSegment_list.size();i++)
	{
		if(strcmp(motionSegment_list[i].get_motion_name(),action)==0)
            return &motionSegment_list[i];
	}
	cout<<"getMotionSegment : no such action "<<action<<endl;
	return NULL;
}
const char* bvhNameConverter(const char* action)
{
	if(strcmp(action, "walk_start") == 0)
		return "../MotionData2/mrl/walk_normal_stright.bvh";
	else if(strcmp(action, "walk_stop_left") == 0)	//left foot contact start, go to stop
		return "../MotionData2/mrl/walk_normal_stright.bvh";
	else if(strcmp(action, "walk_stop_right") == 0)	//right foot contact start, go to stop
		return "../MotionData2/mrl/walk_normal_stright.bvh";
	else if(strcmp(action, "stop") == 0)	//right foot contact start, go to stop
		return "../MotionData2/mrl/walk_normal_stright.bvh";
	else if(strcmp(action, "walk_left_to_right") == 0)		//if current foot is left foot loop, change to right foot loop
		return "../MotionData2/mrl/walk_normal_stright.bvh";
	else if(strcmp(action, "walk_right_to_left") == 0)		//if current foot is right foot loop, change to left foot loop
		return "../MotionData2/mrl/walk_normal_stright.bvh";
	else if(strcmp(action, "walk_fast") == 0)
		return "../MotionData2/mrl/walk_fast_stright.bvh";
	else if(strcmp(action, "walk_normal") == 0)
		return "../MotionData2/mrl/walk_normal_stright.bvh";
	else if(strcmp(action, "walk_slow") == 0)
		return "../MotionData2/mrl/walk_slow_stright.bvh";
	else if(strcmp(action, "left_45") == 0)
		return "../MotionData2/mrl/walk_normal_left_45.bvh";
	else if(strcmp(action, "left_90") == 0)
		return "../MotionData2/mrl/walk_normal_left_90.bvh";
	else if(strcmp(action, "left_135") == 0)
		return "../MotionData2/mrl/walk_normal_left_135.bvh";
	else if(strcmp(action, "right_45") == 0)
		return "../MotionData2/mrl/walk_normal_right_45.bvh";
	else if(strcmp(action, "right_90") == 0)
		return "../MotionData2/mrl/walk_normal_right_90.bvh";
	else if(strcmp(action, "right_135") == 0)
		return "../MotionData2/mrl/walk_normal_right_135.bvh";

	cout<<"no actions named "<<action<<endl;
	return "";
}

BVHparser*
BVHmanager::
getBVHparser(const char* action)
{
	for(int i=0;i<bvhParser_list.size();i++)
	{
		if(strcmp(bvhParser_list[i].getPath(), bvhNameConverter(action))==0)
			return &bvhParser_list[i];
	}
	cout<<"getBVHparser : no such action "<<action<<endl;
	return NULL;
}

void
BVHmanager::
getStartEndFrame(const char* action, int* start, int* end, int* start_motion_state, int* end_motion_state)
{
	if(strcmp(action, "walk_start") == 0)
	{
		*start = 66;
		*end = 107;
		*start_motion_state = MOTION_STATE_STOP;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "stop") == 0)
	{
		*start = 641;
		*end =  641;
		*start_motion_state = MOTION_STATE_STOP;
		*end_motion_state = MOTION_STATE_STOP;
	}
	else if(strcmp(action, "walk_stop_left") == 0)
	{
		*start = 588;
		*end = 641;
		*start_motion_state = MOTION_STATE_LEFT_FOOT;
		*end_motion_state = MOTION_STATE_STOP;
	}
	else if(strcmp(action, "walk_stop_right") == 0)
	{
		*start = 824;
		*end = 879;
		*start_motion_state = MOTION_STATE_RIGHT_FOOT;
		*end_motion_state = MOTION_STATE_STOP;
	}
	else if(strcmp(action, "walk_left_to_right") == 0)
	{
		*start = 119;
		*end = 139;
		*start_motion_state = MOTION_STATE_LEFT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "walk_right_to_left") == 0)
	{
		*start = 100;
		*end = 119;
		*start_motion_state = MOTION_STATE_RIGHT_FOOT;
		*end_motion_state = MOTION_STATE_LEFT_FOOT;
	}
	else if(strcmp(action, "walk_fast") == 0)
	{
		*start = 107;
		*end = 137;
		*start_motion_state = MOTION_STATE_RIGHT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "walk_normal") == 0)
	{
		*start = 100;
		*end = 139;
		*start_motion_state = MOTION_STATE_RIGHT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "walk_slow") == 0)
	{
		*start = 170;
		*end = 221;
		*start_motion_state = MOTION_STATE_RIGHT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "left_45") == 0)
	{
		*start = 119;
		*end = 180;
		*start_motion_state = MOTION_STATE_LEFT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "left_90") == 0)
	{
		*start = 96;
		*end = 159;
		*start_motion_state = MOTION_STATE_LEFT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "left_135") == 0)
	{
		*start = 100;
		*end = 173;
		*start_motion_state = MOTION_STATE_LEFT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "right_45") == 0)
	{
		*start = 84;
		*end = 138;
		*start_motion_state = MOTION_STATE_LEFT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "right_90") == 0)
	{
		*start = 91;
		*end = 149;
		*start_motion_state = MOTION_STATE_LEFT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "right_135") == 0)
	{
		*start = 96;
		*end = 156;
		*start_motion_state = MOTION_STATE_LEFT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else
	{
		cout<<"Not valid action "<<action<<endl;
		exit(1);
	}
}

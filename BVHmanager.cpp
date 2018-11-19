#include "BVHmanager.h"
#include "BVHparser.h"
#include <string.h>
#define MOTION_STATE_STOP 10
#define MOTION_STATE_LEFT_FOOT 11
#define MOTION_STATE_RIGHT_FOOT 12

BVHmanager::BVHmanager()
{
	BVHparser* bvhParser = new BVHparser("../MotionData2/mrl/walk_fast_stright.bvh");
	bvhParser_list.push_back(bvhParser);
	bvhParser = new BVHparser("../MotionData2/mrl/walk_normal_stright.bvh");
	bvhParser_list.push_back(bvhParser);
	bvhParser = new BVHparser("../MotionData2/mrl/walk_normal_stright.bvh");
	bvhParser_list.push_back(bvhParser);
	bvhParser = new BVHparser("../MotionData2/mrl/walk_slow_stright.bvh");
	bvhParser_list.push_back(bvhParser);
	bvhParser = new BVHparser("../MotionData2/mrl/walk_normal_left_45.bvh");
	bvhParser_list.push_back(bvhParser);
	bvhParser = new BVHparser("../MotionData2/mrl/walk_normal_left_90.bvh");
	bvhParser_list.push_back(bvhParser);
	bvhParser = new BVHparser("../MotionData2/mrl/walk_normal_left_135.bvh");
	bvhParser_list.push_back(bvhParser);
	bvhParser = new BVHparser("../MotionData2/mrl/walk_normal_right_45.bvh");
	bvhParser_list.push_back(bvhParser);
	bvhParser = new BVHparser("../MotionData2/mrl/walk_normal_right_90.bvh");
	bvhParser_list.push_back(bvhParser);
	bvhParser = new BVHparser("../MotionData2/mrl/walk_normal_right_135.bvh");
	bvhParser_list.push_back(bvhParser);
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
		if(strcmp(bvhParser_list[i]->getPath(), bvhNameConverter(action))==0)
			return bvhParser_list[i];
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
		*start = 62;
		*end = 99;
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
		*end = 135;
		*start_motion_state = MOTION_STATE_RIGHT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "walk_normal") == 0)
	{
		*start = 100;
		*end = 137;
		*start_motion_state = MOTION_STATE_RIGHT_FOOT;
		*end_motion_state = MOTION_STATE_RIGHT_FOOT;
	}
	else if(strcmp(action, "walk_slow") == 0)
	{
		*start = 170;
		*end = 219;
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
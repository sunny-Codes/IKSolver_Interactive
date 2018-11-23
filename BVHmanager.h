#ifndef BVHmanager_H
#define BVHmanager_H
#include "BVHparser.h"

class BVHmanager
{
public:
    //constructor
	BVHmanager();

    //get functions
	BVHparser* getBVHparser(const char* action);
	void getStartEndFrame(const char* action, int* start, int* end, int* start_motion_state, int* end_motion_state);


    //member variables
    std::vector<BVHparser*> bvhParser_list;
    std::vector<MotionSegment*> motionSegment_list;
};

#endif

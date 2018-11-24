#include "BVHparser.h"
#include <Eigen/Dense>
using namespace Eigen;

#ifndef MotionSegment_H
#define MotionSegment_H
class MotionSegment{
    private:
        const char * motion_name;
        BVHparser* bvhParser;

        int start;
        int end;
        int start_state;
        int end_state;
    public:
        //constuctor
        MotionSegment(BVHparser * _bvhParser, const char* _motion_name, int start, int end, int start_state, int end_state);

        //get functions
        Vector3d get_current_position(int nodeNum, int frameTime);
        Vector3d get_current_rotation(int nodeNum, int frameTime);
    
        Vector3d get_start_position(int nodeNum);
        Vector3d get_start_rotation(int nodeNum);
 
        Vector3d get_end_position(int nodeNum);
        Vector3d get_end_rotation(int nodeNum);

        int get_start();
        int get_end();
        int get_start_state();
        int get_end_state();

        const char* get_motion_name();


}
#endif

#ifndef BVHmanager_H
#define BVHmanager_H
class BVHmanager
{
public:
    //constructor
	BVHmanager();

    //get functions
	BVHparser* getBVHparser(const char* action);
	MotionSegment* getMotionSegment(const char* action);
    void getStartEndFrame(const char* action, int* start, int* end, int* start_motion_state, int* end_motion_state);

    //add functions
    void newBVHparser(const char* action);
    void newMotionSegment(BVHparser* bvhparser, const char* motion_name, int start, int end, int start_state, int end_state);

    //member variables
    std::vector<BVHparser*> bvhParser_list;
    std::vector<MotionSegment*> motionSegment_list;
};

#endif

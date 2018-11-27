#include "BVHparser.h"
#include "Skeleton.h"
#include <Eigen/Dense>
using namespace Eigen;

#ifndef MotionFrame_H
#define MotionFrame_H
class MotionFrame{
    public:
    Vector3d root_position;
    vector<Vector3d> rotations;
    MotionFrame();
    MotionFrame(Vector3d root_position, vector<Vector3d>rotations);
    MotionFrame(VectorXd dofs);
    void set_root_position(Vector3d root_position);
    void add_rotation(Vector3d rotation);
    VectorXd to_VectorXd();
};
#endif

#ifndef MotionSegment_H
#define MotionSegment_H

class MotionSegment{
    private:
        const char * motion_name;
        //BVHparser* bvhParser;
        //vector<JointNode*> allJoints;
        vector<MotionFrame> motionFrames;
        vector<string> rotation_dof_order;
        /*
        int start;
        int end;
        */
        int start_state;
        int end_state;
        vector<int> knots;

    public:
        //constuctor
        MotionSegment(BVHparser * _bvhParser, const char* _motion_name, int start, int end, int start_state, int end_state);

        MotionSegment(vector<VectorXd> dofs, vector<string> _dof_joint_order);

        //get functions
        Vector3d get_current_root_position(int frameTime);
        Vector3d get_start_root_position();
        Vector3d get_end_root_position();

        Vector3d get_current_rotation(int jointNum, int frameTime);
        Vector3d get_start_rotation(int jointNum);
        Vector3d get_end_rotation(int jointNum);

        Vector3d get_current_root_position_displacement(int frameTime);

        /*
        int get_start();
        int get_end();
        */
        int get_start_state();
        int get_end_state();
        
        const char* get_motion_name();
        int get_knots_size(){return knots.size(); }
        int get_frame_length(){return motionFrames.size();}

        //calculate
 
        MotionFrame get_motionFrame(int frameTime);
        VectorXd get_Skeleton_dofs(int frameTime);
        VectorXd get_Skeleton_end_dofs();

        void set_Skeleton_dofs(VectorXd dofs, float scale, Skeleton * skel);
        void set_Skeleton_dofs_except_root(VectorXd dofs, float scale, Skeleton * skel);
 
        void set_Skeleton_bodyNode(int frameTime, float scale, Skeleton * skel);
        void set_Skeleton_bodyNode(Vector3d root_position, Vector3d root_rotation,int frmaeTime, Skeleton * skel);
    
    //int get_allJoints_size(){return allJoints.size(); }
    MotionSegment* blend(MotionSegment* anotherMS, float t);
    MotionSegment* dynamicTimeWarping(MotionSegment* anotherMS, float t);
 
};
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
    
    int get_bvh_parser_list_size(){return bvhParser_list.size();}
    int get_motion_segment_list_size(){return motionSegment_list.size();}


    void getStartEndFrame(const char* action, int* start, int* end, int* start_motion_state, int* end_motion_state);

    //add functions
    BVHparser* newBVHparser(const char* action);
    void newMotionSegment(BVHparser* bvhparser, const char* motion_name, int start, int end, int start_state, int end_state);
    void next();

    //member variables
	MotionSegment* curMotionSegment;
	MotionSegment* prevMotionSegment;
	Skeleton* worldSkel;
	int mFrame = 0;
    std::vector<BVHparser> bvhParser_list;
    std::vector<MotionSegment> motionSegment_list;

    Vector3d prev_action_end_position;
	VectorXd prev_action_end_frame_Skeleton_dofs;
	int MOTION_STATE;
    int CONTROL_front_stack = 0;
    int CONTROL_leftTurn_stack = 0;
	Vector3d interMotion_root_rotation_displacement;
};

#endif

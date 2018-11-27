#include "BVHmanager.h"
#include "BVHparser.h"
#include <string.h>
#define MOTION_STATE_STOP 10
#define MOTION_STATE_LEFT_FOOT 11
#define MOTION_STATE_RIGHT_FOOT 12

/* MotionFrame */

MotionFrame::MotionFrame(){
}
MotionFrame::MotionFrame(Vector3d _root_position, vector<Vector3d> _rotations){
    root_position= _root_position;
    rotations= _rotations;
}
MotionFrame::MotionFrame(VectorXd dofs){
    set_root_position(dofs.segment(0,3));
    for(int j=3; j<dofs.size(); j=j+3){
        add_rotation(dofs.segment(j,3));
    }
}
void MotionFrame::set_root_position(Vector3d _root_position){
    root_position= _root_position;
}
void MotionFrame::add_rotation(Vector3d _rotation){
    rotations.push_back(_rotation);
}
VectorXd MotionFrame::to_VectorXd(){
    VectorXd vec(rotations.size()*3+3);
    vec.segment(0,3)= root_position;
    for(int i=0; i<rotations.size(); i++ ){
        vec.segment(i*3+3,3)= rotations[i];
    }
    return vec;
}

/* MotionSegment */
MotionSegment::MotionSegment(BVHparser * bvhParser, const char* _motion_name, int start, int end, int _start_state, int _end_state): 
    motion_name(_motion_name), start_state(_start_state), end_state(_end_state){
        for(int frameTime= start; frameTime<= end; frameTime++){
            JointNode* curJoint= bvhParser->getRootJoint();
            MotionFrame mf= MotionFrame();

            Vector3d root_position = Vector3d(curJoint->data[frameTime][0], curJoint->data[frameTime][1], curJoint->data[frameTime][2]);
            mf.set_root_position(root_position);
            
            Vector3d root_rotation = Vector3d(curJoint->data[frameTime][3], curJoint->data[frameTime][4], curJoint->data[frameTime][5]);
            mf.add_rotation(root_rotation);

            rotation_dof_order.push_back(curJoint->getName());
            curJoint= curJoint->getNextNode();
            while(curJoint!=nullptr)
            {
                if(curJoint->checkEnd())
                {
                    curJoint = curJoint->getNextNode();
                    continue;
                }
                Vector3d rotation = Vector3d(curJoint->data[frameTime][0], curJoint->data[frameTime][1], curJoint->data[frameTime][2]);
                rotation_dof_order.push_back(curJoint->getName());
                mf.add_rotation(rotation);
               
                curJoint = curJoint->getNextNode();
            }
            motionFrames.push_back(mf);
        }
        knots.push_back(0);
        knots.push_back(end-start);
    }

MotionSegment::MotionSegment(vector<VectorXd> dofs, vector<string> _rotation_dof_order){
    for(int i=0; i<dofs.size(); i++){
        MotionFrame mf(dofs[i]);
        motionFrames.push_back(mf);
    }
    rotation_dof_order= _rotation_dof_order;
}
//get functions
Vector3d MotionSegment::get_current_root_position(int frameTime){
    return motionFrames[frameTime].root_position; 
}
Vector3d MotionSegment::get_start_root_position(){
    return get_current_root_position(0);
}
Vector3d MotionSegment::get_end_root_position(){
    return get_current_root_position(motionFrames.size()-1);
}

Vector3d MotionSegment::get_current_rotation(int jointNum, int frameTime){
    return motionFrames[frameTime].rotations[jointNum];
}

Vector3d MotionSegment::get_start_rotation(int jointNum){
    return get_current_rotation(jointNum, 0);
}

Vector3d MotionSegment::get_end_rotation(int jointNum){
    return get_current_rotation(jointNum, motionFrames.size()-1);
}

Vector3d MotionSegment::get_current_root_position_displacement(int frameTime){
    return get_current_root_position(frameTime)- get_current_root_position(0);
}

/*
int MotionSegment::get_start(){return start;}
int MotionSegment::get_end(){return end;}
*/
int MotionSegment::get_start_state(){return start_state;}
int MotionSegment::get_end_state(){return end_state;}
const char*  MotionSegment::get_motion_name(){return motion_name;}


MotionFrame MotionSegment::get_motionFrame(int frameTime){
    return motionFrames[frameTime];
}

VectorXd MotionSegment::get_Skeleton_dofs(int frameTime){
    return get_motionFrame(frameTime).to_VectorXd();
}

VectorXd MotionSegment::get_Skeleton_end_dofs(){
    return get_Skeleton_dofs(get_frame_length()-1);
}

void MotionSegment::set_Skeleton_bodyNode(int frameTime, float scale, Skeleton * skel){
    //JointNode* curJoint= allJoints[0]; //bvhParser->getRootJoint();

    Vector3d root_position = get_current_root_position(frameTime);
    skel->getRootBodyNode()->setWorldTranslation(scale*root_position);

    Vector3d root_rotation = get_current_rotation(0, frameTime);
    skel->getRootBodyNode()->setWorldRotation_v(root_rotation);

    for(int i=1; i<rotation_dof_order.size(); i++){
        Vector3d joint_rotation = get_current_rotation(i, frameTime);
        skel->getBodyNode(rotation_dof_order[i])->setRotation_v(joint_rotation);
    }
    
}

void MotionSegment::set_Skeleton_bodyNode(Vector3d root_position, Vector3d root_rotation, int frameTime, Skeleton * skel){
    skel->getRootBodyNode()->setWorldTranslation(root_position); //assume: already scaled
    skel->getRootBodyNode()->setWorldRotation_v(root_rotation);

    for(int i=1; i<rotation_dof_order.size(); i++){
        Vector3d joint_rotation = get_current_rotation(i, frameTime);
        skel->getBodyNode(rotation_dof_order[i])->setRotation_v(joint_rotation);
    }

}

MotionSegment* MotionSegment::blend(MotionSegment* anotherMS, float t){
    assert(get_knots_size()== anotherMS->get_knots_size());
    MotionSegment* shorter= anotherMS;
    MotionSegment* longer= this;
    if(anotherMS->get_frame_length() > get_frame_length()) {
        shorter= this;
        longer= anotherMS;
    }
    // for(int i=0; i<shorter->)
}

VectorXd dofSlerp(VectorXd dof, VectorXd dof2, float t){
    assert(dof.size()== dof2.size());
    VectorXd slerped(dof.size());
    slerped.setZero();
    slerped.segment(0,3)= t*dof.segment(0,3)+(1-t)*dof2.segment(0,3);
    for(int i=3; i<dof.size(); i=i+3){
        Quaterniond quat= AngleAxisToQuaternion(dof.segment(i,3));
        Quaterniond quat2= AngleAxisToQuaternion(dof2.segment(i,3));
        slerped.segment(i,3)= QuaternionToAngleAxis(quat.slerp(t, quat2));
    }
    return slerped;
}

float dofDistance(VectorXd dof, VectorXd dof2){
    assert(dof.size()== dof2.size());

    VectorXd displacement= dof-dof2;
    displacement.segment(0,3)=Vector3d(0,0,0);
    return displacement.norm();
}

MotionSegment* MotionSegment::dynamicTimeWarping(MotionSegment* anotherMS, float t){
    int length= get_frame_length();
    int length2= anotherMS->get_frame_length();
    int dmt[length2][length];
   
    vector<vector<int>> pairs;
    int i= 0;
    int j= 0;
    vector<int> pair= vector<int>();
    pair.push_back(j);

    while(pair.size()< length && j<length2){
        float right= dofDistance(get_Skeleton_dofs(i), anotherMS->get_Skeleton_dofs(j+1));
        float down= dofDistance(get_Skeleton_dofs(i+1), anotherMS->get_Skeleton_dofs(j));
        float diagonal= dofDistance(get_Skeleton_dofs(i+1), anotherMS->get_Skeleton_dofs(j+1));

        if(right<down && right<diagonal){
            j++;
            pair.push_back(j);
        }else if(down<diagonal){
            pairs.push_back(pair);
            //make new pair
            pair= vector<int>();
            pair.push_back(j);
        }else {
            pairs.push_back(pair);
            pair= vector<int>();
            j++;
            pair.push_back(j);
        }
    }
    if(pairs.size()== length){
        for(;j<length2;j++){
            pair.push_back(j);
        }
        pairs.push_back(pair);
    }
    else if(j== length2){
        while(pairs.size()<length){
            pair=vector<int>();
            pair.push_back(j-1);
            pairs.push_back(pair);
        }
    }
    
    //i= start;
    i=0;
    vector<VectorXd> slerped_dofs;
    for(vector<vector<int>>::iterator vit= pairs.begin(); vit!=pairs.end(); vit++){
        vector<int> pair= *vit;
        VectorXd dof= get_Skeleton_dofs(i);
        VectorXd another_dof= anotherMS->get_Skeleton_dofs(pair[0]);
        VectorXd slerped= dofSlerp(dof, another_dof,t);
        slerped_dofs.push_back(slerped);
        i++;
    }
    return new MotionSegment(slerped_dofs, rotation_dof_order);
}
 


/* BVHmanager */
BVHmanager::BVHmanager()
{
    BVHparser* bvhParser = newBVHparser("../MotionData2/mrl/walk_fast_stright.bvh");
    newMotionSegment(bvhParser, "walk_fast", 107, 135, MOTION_STATE_RIGHT_FOOT, MOTION_STATE_RIGHT_FOOT);

    bvhParser = newBVHparser("../MotionData2/mrl/walk_normal_stright.bvh");
    newMotionSegment(bvhParser, "walk_normal", 100, 137, MOTION_STATE_RIGHT_FOOT, MOTION_STATE_RIGHT_FOOT);
    newMotionSegment(bvhParser, "walk_start", 62, 99, MOTION_STATE_STOP, MOTION_STATE_RIGHT_FOOT);
    newMotionSegment(bvhParser, "walk_stop_left", 588, 641, MOTION_STATE_LEFT_FOOT, MOTION_STATE_STOP);
    newMotionSegment(bvhParser, "walk_stop_right", 824, 879, MOTION_STATE_RIGHT_FOOT, MOTION_STATE_STOP);
    newMotionSegment(bvhParser, "stop", 641, 641, MOTION_STATE_STOP, MOTION_STATE_STOP);
    newMotionSegment(bvhParser, "walk_left_to_right", 119, 139, MOTION_STATE_LEFT_FOOT, MOTION_STATE_RIGHT_FOOT);
    newMotionSegment(bvhParser, "walk_right_to_left", 100, 119, MOTION_STATE_RIGHT_FOOT, MOTION_STATE_LEFT_FOOT);

    bvhParser = newBVHparser("../MotionData2/mrl/walk_slow_stright.bvh");
    newMotionSegment(bvhParser, "walk_slow", 170, 219, MOTION_STATE_RIGHT_FOOT, MOTION_STATE_RIGHT_FOOT);

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

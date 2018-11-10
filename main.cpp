#include <GL/glut.h>
#include <iostream>
#include <math.h>
#include <float.h>
#include <Eigen/Dense>
#include "Skeleton.h"
#include "BVHparser.h"
#define M_PI 3.14159265358979323846
using namespace std;

float fovy = 50.0f;
float aspectRatio = 1260.0f / 760.0f;
Eigen::Vector3d eye(0.0f, 8.0f, -10.0f);
Eigen::Vector3d viewCenter(0.0f, 0.0f, 0.0f);
Eigen::Vector3d viewUp(0.0f, 1.0f, 0.0f);

float px, py;
bool mouseViewRotate_ON = false;
bool mouseViewTranslate_ON = false;
bool mouseObjectRotate_ON = false;
bool mouseObjectTranslate_ON = false;

Eigen::Vector3d prev_eye = eye;
Eigen::Vector3d prev_viewUp = viewUp;
Eigen::Vector3d prev_viewCenter = viewCenter;
Eigen::Vector3d axis;
Eigen::Vector3d actual_cur_point, actual_prev_point;

Eigen::VectorXd prev_skel_positions;
Eigen::Isometry3d prev_skel_transform;
SkeletonPtr worldSkel;
BVHparser* bvhParser;
float scale = 1.0/20.0;
int selectedObject = -1;
int selectingObject = -1;

Eigen::Isometry3d obj_trackBall_transform = Eigen::Isometry3d::Identity();
Eigen::Isometry3d prev_obj_trackBall_transform = Eigen::Isometry3d::Identity();
double trackball_radius = 1.0;

int mFrame;
int mDisplayTimeout;

using namespace std;

void hideObjTrackball()
{
	obj_trackBall_transform.translation() = eye * 2.0;
}
void renderCube(float width, float height, float depth) {

	glBegin(GL_QUADS);
	glNormal3f(0.0, 0.0, 1.0);
	glVertex3f(width, -height, depth);
	glVertex3f(width, height, depth);
	glVertex3f(-width, height, depth);
	glVertex3f(-width, -height, depth);

	glNormal3f(1.0, 0.0, 0.0);
	glVertex3f(width, -height, -depth);
	glVertex3f(width, height, -depth);
	glVertex3f(width, height, depth);
	glVertex3f(width, -height, depth);

	glNormal3f(0.0, 0.0, -1.0);
	glVertex3f(-width, -height, -depth);
	glVertex3f(-width, height, -depth);
	glVertex3f(width, height, -depth);
	glVertex3f(width, -height, -depth);

	glNormal3f(-1.0, 0.0, 0.0);
	glVertex3f(-width, -height, depth);
	glVertex3f(-width, height, depth);
	glVertex3f(-width, height, -depth);
	glVertex3f(-width, -height, -depth);

	glNormal3f(0.0, 1.0, 0.0);
	glVertex3f(width, height, depth);
	glVertex3f(width, height, -depth);
	glVertex3f(-width, height, -depth);
	glVertex3f(-width, height, depth);

	glNormal3f(0.0, -1.0, 0.0);
	glVertex3f(width, -height, depth);
	glVertex3f(-width, -height, depth);
	glVertex3f(-width, -height, -depth);
	glVertex3f(width, -height, -depth);

	glEnd();
}
/// Show XYZ basis at (0, 0, 0)
void showXYZaxis()
{
	glColor3f(1.0, 0.0, 0.0);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(1.0, 0.0, 0.0);
	glEnd();

	glColor3f(0.0, 1.0, 0.0);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 1.0, 0.0);
	glEnd();

	glColor3f(0.0, 0.0, 1.0);
	glLineWidth(2.0);
	glBegin(GL_LINES);
	glVertex3f(0.0, 0.0, 0.0);
	glVertex3f(0.0, 0.0, 1.0);
	glEnd();

}

/// Draw skeleton 'wordSkel'
void drawSkeleton()
{
	glInitNames();
	for(int i=0;i<worldSkel->getNumBodyNodes();i++)
	{
		glPushName(i);
		glPushMatrix();
		glMultMatrixd(worldSkel->getBodyNode(i)->getWorldTransform().data());
		if(i == selectedObject)
			glColor3f(1.0, 0.0, 0.0);
		else if(i == selectingObject)
			glColor3f(0.0, 0.0, 1.0);
		else
			glColor3f(0.2, 0.2, 0.2);
		renderCube(worldSkel->getBodyNode(i)->getShape().width,
			worldSkel->getBodyNode(i)->getShape().height,
			worldSkel->getBodyNode(i)->getShape().depth);

		glPopMatrix();
		glPopName();
	}

}

/// Draw trackball which is for IK rotation
void drawTrackball()
{

	glPushMatrix();

	glMultMatrixd(obj_trackBall_transform.data());
	glutWireSphere(trackball_radius, 10, 10);
	glPopMatrix();

}

/// Picking. return the content(integer) of glPushName(int index)
int selectObject(GLint x, GLint y)
{
	GLuint selectBuff[64];
	GLint hits, viewport[4];

	glSelectBuffer(63, selectBuff);
	glGetIntegerv(GL_VIEWPORT, viewport);

	glMatrixMode(GL_PROJECTION);
	glRenderMode(GL_SELECT);
	glLoadIdentity();
	gluPickMatrix(x, viewport[3]-y, 2, 2, viewport);
	gluPerspective(fovy, 
		aspectRatio,
		0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW); 
	glLoadIdentity();
 	gluLookAt(eye[0], eye[1], eye[2],
		viewCenter[0], viewCenter[1], viewCenter[2],
		viewUp[0], viewUp[1], viewUp[2]);
 	glPushMatrix();
 	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
 	drawSkeleton();
 	hits = glRenderMode(GL_RENDER);
 	// cout<<"hits : "<<hits<<endl;
 	// if(hits>0)
 	// {
 	// 	cout<<"picking! "<<endl;
 	// 	cout<<worldSkel->getBodyNode(selectBuff[3])->getName()<<endl;
 	// }
 	glPopMatrix();
 	glMatrixMode(GL_MODELVIEW);
 	if(hits == 0)
 		return -1;
 	return selectBuff[3];
}

/// Render Scene.
void renderScene(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	// glClearColor(0.3, 0.3, 0.8, 1.0);
	glEnable(GL_DEPTH_TEST);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fovy, 
		aspectRatio,
		0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW); 
	glLoadIdentity();
 	gluLookAt(eye[0], eye[1], eye[2],
		viewCenter[0], viewCenter[1], viewCenter[2],
		viewUp[0], viewUp[1], viewUp[2]);
 	glPushMatrix();
 	
	GLfloat ambientLight[] = { 0.8f, 0.8f, 0.8f, 0.0f };     // <1>
	GLfloat diffuseLight[] = { 0.8f, 0.8f, 0.8f, 0.0f };       // <2>
	GLfloat lightPos[] = { 0.0f, 10.0f, -10.0f, 0.0f };    // <3>sss
	   
	glEnable(GL_LIGHTING);                                     // <4>
	glEnable(GL_LIGHT0);                                       // <8>
	glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight);            // <5>
	glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight);            // <6>
	glLightfv(GL_LIGHT0, GL_POSITION, lightPos);               // <7>
	glEnable(GL_COLOR_MATERIAL);
 	drawSkeleton();
 	// drawTrackball();
 	// renderCube(0.4, 0.4, 0.4);
 	// glutSolidTeapot(0.5f);
 	glDisable(GL_LIGHTING);
 	showXYZaxis();
 	//showViewUp();
 	//showRotAxis();
 	glPopMatrix();

 	glutSwapBuffers();
}

/// called every frameTime sec.
void Timer(int value)
{
	MotionNode* curNode = bvhParser->getRootNode();
	Eigen::Vector3d rotation;
	rotation = Eigen::Vector3d(curNode->data[mFrame][3], curNode->data[mFrame][4], curNode->data[mFrame][5]);
	worldSkel->getRootBodyNode()
	->setWorldTranslation(Eigen::Vector3d(curNode->data[mFrame][0]*scale, curNode->data[mFrame][1]*scale, curNode->data[mFrame][2]*scale));
	worldSkel->getRootBodyNode()
	->setWorldRotation(Eigen::AngleAxisd(rotation.norm(), rotation.normalized()).toRotationMatrix());

	curNode= curNode->getNextNode();
	while(curNode!=nullptr)
	{
		if(curNode->checkEnd())
		{
			cout<<curNode->getName()<<endl;
			curNode = curNode->getNextNode();
			continue;
		}
		rotation = Eigen::Vector3d(curNode->data[mFrame][0], curNode->data[mFrame][1], curNode->data[mFrame][2]);
		worldSkel->getBodyNode(curNode->getName())->setRotation(Eigen::AngleAxisd(rotation.norm(), rotation.normalized()).toRotationMatrix());
		curNode = curNode->getNextNode();
	}
	renderScene();
	mFrame++;
	mFrame %= bvhParser->frames;
 	glutTimerFunc(mDisplayTimeout, Timer,1);
}
void passiveProcessMouse(int x, int y)
{
	selectingObject = selectObject(x, y);
	renderScene();
}

void processMouse(int button, int state, int x, int y) 
{
	if(button == GLUT_LEFT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			px = x;
			py = y;
			// if(glutGetModifiers() == GLUT_ACTIVE_CTRL)
			// {
			// 	selectedObject = selectObject(x, y);
			// 	if(selectedObject >= 0)
			// 	{
			// 		cout<<worldSkel->getBodyNode(selectedObject)->getName()<<endl;
			// 		prev_skel_transform = worldSkel->getBodyNode(selectedObject)->getTransform();
			// 		prev_skel_positions= worldSkel->getPositions();
			// 		prev_obj_trackBall_transform = obj_trackBall_transform;
			// 		mouseObjectRotate_ON = true;
			// 	}
			// 	else{
			// 		mouseObjectRotate_ON = false;
			// 	}
			// }
			prev_eye = eye;
			prev_viewUp = viewUp;
			mouseViewRotate_ON = true;
		}
		else
		{
			mouseViewRotate_ON = false;
			// mouseObjectRotate_ON = false;
			// hideObjTrackball();
			// selectedObject = -1;
		}
	}
	else if(button == GLUT_RIGHT_BUTTON)
	{
		if (state == GLUT_DOWN)
		{
			px = x;
			py = y;
			// if(glutGetModifiers() == GLUT_ACTIVE_CTRL)
			// {
			// 	selectedObject = selectObject(x, y);
			// 	if(selectedObject >= 0)
			// 	{
			// 		mouseObjectTranslate_ON = true;
			// 		cout<<worldSkel->getBodyNode(selectedObject)->getName()<<endl;

			// 		prev_skel_positions= worldSkel->getPositions();
			// 	}
			// 	else{
			// 		mouseObjectTranslate_ON = false;
			// 	}
					

			// }
			prev_eye = eye;
			prev_viewCenter = viewCenter;
			mouseViewTranslate_ON = true;
		}
		else
		{
			mouseViewTranslate_ON = false;
			// mouseObjectTranslate_ON = false;
			// hideObjTrackball();
			// selectedObject = -1;
		}
	}
	else if(button == 3 && state == GLUT_DOWN)
	{
		eye += (viewCenter - eye).normalized() * 0.5;
	}
	else if(button == 4 && state == GLUT_DOWN)
	{
		eye -= (viewCenter - eye).normalized() * 0.5;
	}
	renderScene();
}

 void resize(int width, int height) {
 	if (((float)width) / height < aspectRatio) {
		glViewport(0, 0, height * aspectRatio, height);
		glutReshapeWindow(int(height * aspectRatio), height);
	}
	else {
		glViewport(0, 0, width, width / aspectRatio);
		glutReshapeWindow(width, int(width / aspectRatio));

	}
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(fovy,
		aspectRatio,
		0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(eye[0], eye[1], eye[2],
		viewCenter[0], viewCenter[1], viewCenter[2],
		viewUp[0], viewUp[1], viewUp[2]);
 	glutSwapBuffers();
 }

void rotateView(int x, int y)
{
	float width = glutGet(GLUT_WINDOW_WIDTH);
	float height = glutGet(GLUT_WINDOW_HEIGHT);
	float pixel_radius = min(width, height) / 2.0;
	Eigen::Vector3d normed_cur_point, normed_prev_point;

	normed_cur_point.setZero();
	normed_cur_point[0] = -(width / 2.0f - x) / pixel_radius;
	normed_cur_point[1] = (height / 2.0f - y) / pixel_radius;
	if(normed_cur_point.norm()>1)
		normed_cur_point.normalize();
	else
	{
		normed_cur_point[2] 
		= sqrt(1.0f - normed_cur_point[0]*normed_cur_point[0]-normed_cur_point[1]*normed_cur_point[1]);
	}
	
	normed_prev_point.setZero();
	normed_prev_point[0] = -(width / 2.0f - px) / pixel_radius;
	normed_prev_point[1] = (height / 2.0f - py) / pixel_radius;
	if(normed_prev_point.norm()>1)
		normed_prev_point.normalize();
	else
	{
		normed_prev_point[2] 
		= sqrt(1.0f - normed_prev_point[0]*normed_prev_point[0]-normed_prev_point[1]*normed_prev_point[1]);
	}

	Eigen::Vector3d y_cord = prev_viewUp;
	Eigen::Vector3d z_cord = (prev_eye-viewCenter).normalized();
	Eigen::Vector3d x_cord = y_cord.cross(z_cord).normalized();
	Eigen::Matrix3d transform;

	transform.block<3,1>(0,0) = x_cord;
	transform.block<3,1>(0,1) = y_cord;
	transform.block<3,1>(0,2) = z_cord;

	actual_prev_point = transform * normed_prev_point;
	actual_cur_point = transform * normed_cur_point;

	axis = (actual_prev_point.cross(actual_cur_point)).normalized();

	float angle = -atan2(actual_prev_point.cross(actual_cur_point).norm(), actual_prev_point.dot(actual_cur_point));
	Eigen::Vector3d trackball_center = viewCenter;
	eye = prev_eye - trackball_center;
	eye = Eigen::AngleAxisd(angle, axis) * eye;
	eye = eye + trackball_center;

	viewUp = Eigen::AngleAxisd(angle, axis) * prev_viewUp;
}
void translateView(int x, int y)
{
	float width = glutGet(GLUT_WINDOW_WIDTH);
	float height = glutGet(GLUT_WINDOW_HEIGHT);
	float pixel_radius = min(width, height) / 2.0;
	Eigen::Vector2d normed_cur_point, normed_prev_point, normed_diff;
	Eigen::Vector2d real_diff;

	normed_cur_point[0] = (width / 2.0f - x) / (width/2.0);
	normed_cur_point[1] = (height / 2.0f - y) / (height/2.0);
	normed_prev_point[0] = (width / 2.0f - px) / (width/2.0);
	normed_prev_point[1] = (height / 2.0f - py) / (height/2.0);

	normed_diff = normed_cur_point - normed_prev_point;

	double viewDistance = (viewCenter - eye).norm();
	real_diff[1] = normed_diff[1] * viewDistance * tan(fovy*M_PI/180.0/2.0);
	real_diff[0] = normed_diff[0] * viewDistance * tan(fovy*M_PI/180.0/2.0) * aspectRatio;


	Eigen::Vector3d viewLeft;
	viewLeft = (viewUp).normalized().cross(viewCenter - eye);
	// The norm of viewRight is already 1. For safe implement, we will normailze.
	viewLeft.normalize();

	viewCenter = prev_viewCenter - (real_diff[0] * viewLeft + real_diff[1] * viewUp);
	eye = prev_eye - (real_diff[0] * viewLeft + real_diff[1] * viewUp);
}

double lineSearch(SkeletonPtr skel, 
				Eigen::VectorXd pos, 
				Eigen::VectorXd gradient, 
				double stepSize,
				int sel_ob,
				Eigen::Vector3d offset,
				Eigen::Vector3d targetPosition)
{
	Eigen::VectorXd prev_pos = pos;
	Eigen::Vector3d prev_vec = targetPosition - skel->getBodyNode(sel_ob)->getWorldTransform()*offset;
	prev_vec.normalize();
	pos += gradient * stepSize;
	skel->setPositions(pos);
	Eigen::Vector3d cur_vec = targetPosition - skel->getBodyNode(sel_ob)->getWorldTransform()*offset;
	cur_vec.normalize();
	skel->setPositions(prev_pos);
	if(prev_vec.dot(cur_vec) < cos(M_PI/2.0 * 0.2))
	{
		return lineSearch(skel, pos, gradient, stepSize/2.0, sel_ob, offset, targetPosition);
	}
	return stepSize;
}

void solveJacobianIK_translation(int x, int y)
{
	float width = glutGet(GLUT_WINDOW_WIDTH);
	float height = glutGet(GLUT_WINDOW_HEIGHT);
	float pixel_radius = min(width, height) / 2.0;
	/// normed point -> point on the screen x : -1.0 ~ 1.0, y : -1.0 ~ 1.0
	Eigen::Vector2d normed_cur_point, normed_prev_point, normed_diff;
	/// real_diff -> point on the picked plane
	Eigen::Vector2d real_diff;

	normed_cur_point[0] = (width / 2.0f - x) / (width/2.0);
	normed_cur_point[1] = (height / 2.0f - y) / (height/2.0);
	normed_prev_point[0] = (width / 2.0f - px) / (width/2.0);
	normed_prev_point[1] = (height / 2.0f - py) / (height/2.0);

	normed_diff = normed_cur_point - normed_prev_point;

	Eigen::Vector3d obj_center = worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation();
	double viewDistance = (obj_center- eye).normalized().dot(worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation()- eye);
	real_diff[1] = normed_diff[1] * viewDistance * tan(fovy*M_PI/180.0/2.0) ;
	real_diff[0] = normed_diff[0] * viewDistance * tan(fovy*M_PI/180.0/2.0) * aspectRatio;


	Eigen::Vector3d viewLeft;
	viewLeft = (viewUp).normalized().cross((viewCenter - eye).normalized());
	// The norm of viewRight is already 1. For safe implement, we will normailze.
	viewLeft.normalize();
	Eigen::Vector3d real_diff_3d;
	real_diff_3d = prev_viewCenter + real_diff[0] * viewLeft + real_diff[1] * (viewUp);

	worldSkel->setPositions(prev_skel_positions);
	Eigen::Vector3d targetPosition = worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation() + real_diff_3d;

	int steps = 0;
	double stepSize = 0.1;
	Eigen::VectorXd prev_pos, gradient;

	while((targetPosition - worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation()).norm()>1E-4)
	{
		stepSize = 0.1;
		Eigen::VectorXd pos = worldSkel->getPositions();
		Eigen::MatrixXd jacobian = worldSkel->getJacobian(worldSkel->getBodyNode(selectedObject), Eigen::Vector3d::Zero());
		Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
		gradient =svd.solve(targetPosition - worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation());
		stepSize = lineSearch(worldSkel, pos, gradient, stepSize, selectedObject, Eigen::Vector3d::Zero(), targetPosition);
		pos += gradient * stepSize;
		worldSkel->setPositions(pos);
		steps++;
	}
}

void solveJacobianIK_rotation(int x, int y)
{
	obj_trackBall_transform.translation() = worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation();
	
	float width = glutGet(GLUT_WINDOW_WIDTH);
	float height = glutGet(GLUT_WINDOW_HEIGHT);
	Eigen::Vector2d normed_cur_point, normed_prev_point, normed_diff;
	Eigen::Vector2d real_diff;

	normed_cur_point[0] = (width / 2.0f - x) / (width/2.0);
	normed_cur_point[1] = (height / 2.0f - y) / (height/2.0);
	normed_prev_point[0] = (width / 2.0f - px) / (width/2.0);
	normed_prev_point[1] = (height / 2.0f - py) / (height/2.0);
	
	Eigen::Vector3d obj_center = worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation();


	double viewDistance = (obj_center- eye).normalized().dot(worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation()- eye);
	
	Eigen::Vector3d real_cur_point, real_prev_point;

	Eigen::Vector3d viewLeft;
	viewLeft = (viewUp).normalized().cross((viewCenter - eye).normalized());
	// The norm of viewRight is already 1. For safe implement, we will normailze.
	viewLeft.normalize();

	real_cur_point = viewLeft * normed_cur_point[0] * viewDistance * tan(fovy*M_PI/180.0/2.0) * aspectRatio
		+ viewUp * normed_cur_point[1] * viewDistance * tan(fovy*M_PI/180.0/2.0)
		+ eye + (viewCenter- eye).normalized()*viewDistance;
	real_prev_point = viewLeft * normed_prev_point[0] * viewDistance * tan(fovy*M_PI/180.0/2.0) * aspectRatio
		+ viewUp * normed_prev_point[1] * viewDistance * tan(fovy*M_PI/180.0/2.0)
		+ eye + (viewCenter- eye).normalized()*viewDistance;


	Eigen::Vector3d viewZ = (eye-obj_center).normalized();

	if((real_prev_point -obj_center).norm()>trackball_radius)
	{
		real_prev_point = obj_center + (real_prev_point-obj_center).normalized();
	}
	if((real_cur_point -obj_center).norm()>trackball_radius)
	{
		real_cur_point = obj_center + (real_cur_point-obj_center).normalized();
	}


	double prev_point_viewZ = sqrt(trackball_radius*trackball_radius 
		- (real_prev_point-obj_center).norm()*(real_prev_point-obj_center).norm());
	double cur_point_viewZ = sqrt(trackball_radius*trackball_radius  
		- (real_cur_point-obj_center).norm()*(real_cur_point-obj_center).norm());

	Eigen::Vector3d real_prev_sphere_point = real_prev_point + prev_point_viewZ*viewZ;
	Eigen::Vector3d real_cur_sphere_point = real_cur_point + cur_point_viewZ*viewZ;

	Eigen::Vector3d cur_vec = (real_cur_sphere_point - obj_center).normalized();
	Eigen::Vector3d prev_vec = (real_prev_sphere_point - obj_center).normalized();

	Eigen::Vector3d axis = prev_vec.cross(cur_vec);
	double angle = atan2(axis.norm(), prev_vec.dot(cur_vec));
	axis.normalize();

	worldSkel->getBodyNode(selectedObject)->setRotation(
			prev_skel_transform.rotation() * Eigen::AngleAxisd(angle, axis).toRotationMatrix());

	obj_trackBall_transform.linear()=
		prev_obj_trackBall_transform.rotation() * Eigen::AngleAxisd(angle, axis).toRotationMatrix();

	Eigen::VectorXd curPos = worldSkel->getPositions();
	worldSkel->setPositions(prev_skel_positions);
	Eigen::Vector3d targetPosition = worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation();
	worldSkel->setPositions(curPos);

	int steps = 0;
	double stepSize = 0.1;
	Eigen::VectorXd prev_pos, gradient;
	while((targetPosition - worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation()).norm()>1E-4)
	{
		stepSize = 0.1;
		Eigen::VectorXd pos = worldSkel->getPositions();
		Eigen::MatrixXd jacobian = worldSkel->getJacobian(worldSkel->getBodyNode(selectedObject), Eigen::Vector3d::Zero());
		jacobian.block<3,3>(0, 3+3*selectedObject) = Eigen::Matrix3d::Zero();

		Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
		gradient =svd.solve(targetPosition - worldSkel->getBodyNode(selectedObject)->getWorldTransform().translation());
		stepSize = lineSearch(worldSkel, pos, gradient, stepSize, selectedObject, Eigen::Vector3d::Zero(), targetPosition);
		pos += gradient * stepSize;
		worldSkel->setPositions(pos);
		steps++;
	}
}

void pressedProcessMouse(int x, int y) {
	if(mouseViewRotate_ON)
	{
		rotateView(x, y);
	}
	if(mouseViewTranslate_ON)
	{
		translateView(x, y);
	}
	// if(mouseObjectTranslate_ON)
	// {
	// 	solveJacobianIK_translation(x,y);
	// }
	// if(mouseObjectRotate_ON)
	// {
	// 	solveJacobianIK_rotation(x,y);
	// }
	renderScene();
}

void keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
		case 27:
		exit(0);
		break;
		default:
		break;
	}
}

SkeletonPtr createBVHSkeleton(const char* path)
{
	SkeletonPtr skel = std::shared_ptr<Skeleton>(new Skeleton());
	bvhParser = new BVHparser(path);
	MotionNode *curNode;
	curNode = bvhParser->getRootNode();
	BodyNodePtr rootBody = std::shared_ptr<BodyNode>(new BodyNode(curNode->getName()));
	rootBody->setShape(0.15, 0.15, 0.15);
	skel->setRootBodyNode(rootBody);
	curNode = curNode->getNextNode();
	while(curNode != nullptr)
	{
		if(curNode->checkEnd())
		{
			curNode = curNode->getNextNode();
			continue;
		}
		BodyNodePtr newBody 
		= std::shared_ptr<BodyNode>(new BodyNode(curNode->getName(),skel->getBodyNode(curNode->getParent()->getName())));
		newBody->setShape(
			fmax(0.15, fabs(curNode->getChilds()[0]->getOffset(0)*scale/2.0)), 
			fmax(0.15, fabs(curNode->getChilds()[0]->getOffset(1)*scale/2.0)), 
			fmax(0.15, fabs(curNode->getChilds()[0]->getOffset(2)*scale/2.0)));
		Eigen::Isometry3d pbtj, cbtj;
		pbtj.setIdentity();
		cbtj.setIdentity();
		pbtj.translation() = newBody->getParent()->getJoint().childBodyToJoint.translation()
		+ Eigen::Vector3d(
			(curNode->getOffset(0))*scale, 
			(curNode->getOffset(1))*scale, 
			(curNode->getOffset(2))*scale);
		cbtj.translation() = -Eigen::Vector3d(
			(curNode->getChilds()[0]->getOffset(0))*scale/2.0, 
			(curNode->getChilds()[0]->getOffset(1))*scale/2.0, 
			(curNode->getChilds()[0]->getOffset(2))*scale/2.0);
		newBody->setJoint(pbtj, cbtj);
		
		skel->addBodyNode(newBody);
		curNode = curNode->getNextNode();
	}
	mDisplayTimeout = 1000.0*bvhParser->frameTime;
	return skel;
}

/// View Up should match to our eye, viewCener
void initViewUP()
{
	Eigen::Vector3d viewRight = (viewCenter - eye).normalized().cross(viewUp);
	viewRight.normalize();
	viewUp = viewRight.cross((viewCenter - eye).normalized());
}


int main(int argc, char **argv) {

	// init GLUT and create Window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DEPTH | GLUT_SINGLE | GLUT_RGBA);
	glutInitWindowPosition(200, 200);
	glutInitWindowSize(1260, 760);
	glutCreateWindow("Advanced Animation hw 3");
	initViewUP();
	mFrame = 0;
	worldSkel = createBVHSkeleton("../data/Trial001.bvh");
	// hideObjTrackball();

	// register callbacks
	glutDisplayFunc(renderScene);
	glutReshapeFunc(resize);
	glutMouseFunc(processMouse);
	glutMotionFunc(pressedProcessMouse);
	//glutPassiveMotionFunc(passiveProcessMouse);
	glutKeyboardFunc(keyboard);
	glutTimerFunc(mDisplayTimeout, Timer, 0);

	// enter GLUT event processing cycle
	glutMainLoop();
	return 0;
}

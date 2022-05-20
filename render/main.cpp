#include "Window.h"
#include "Environment.h"
#include "DARTHelper.h"
#include "Character.h"
#include "BVH.h"
#include "Muscle.h"
namespace p = boost::python;
namespace np = boost::python::numpy;

#include "dart/collision/bullet/bullet.hpp"
#include "dart/constraint/ContactConstraint.hpp"
using namespace dart;
using namespace dart::simulation;
using namespace dart::dynamics;
using namespace MASS;

int main(int argc,char** argv)
{
	MASS::Environment* env = new MASS::Environment();

	if(argc==1)
	{
		std::cout<<"Provide Metadata.txt"<<std::endl;
		return 0;
	}
	env->Initialize(std::string(argv[1]),true);
	// if(argc==3)
	// 	env->SetUseMuscle(true);
	// else
	// 	env->SetUseMuscle(false);
	// env->SetControlHz(30);
	// env->SetSimulationHz(600);

	// MASS::Character* character = new MASS::Character();
	// character->LoadSkeleton(std::string(MASS_ROOT_DIR)+std::string("/data/human.xml"),true);
	// if(env->GetUseMuscle())
	// 	character->LoadMuscles(std::string(MASS_ROOT_DIR)+std::string("/data/muscle.xml"));
	// character->LoadBVH(std::string(MASS_ROOT_DIR)+std::string("/data/motion/walk.bvh"),true);
	
	// double kp = 300.0;
	// character->SetPDParameters(kp,sqrt(2*kp));
	// env->SetCharacter(character);
	// env->SetGround(MASS::BuildFromFile(std::string(MASS_ROOT_DIR)+std::string("/data/ground.xml")));

	// env->Initialize();

	Py_Initialize();
	np::initialize();
	glutInit(&argc, argv);

	auto mCharacter = env->GetCharacter();
	int t = 10;
	int mControlHz = 30;

	//let's print out all joints and index info as below
	// for (int i = 0; i < mCharacter->GetSkeleton()->getNumJoints(); ++i) {
	// 	auto jnt = mCharacter->GetSkeleton()->getJoint(i);
	// 	std::cout << "\n";
	// 	std::cout << "Joint " << jnt->getName() << std::endl;
	// 	std::cout << "Joint ID: " << i << std::endl;
	// 	if(jnt->getParentBodyNode())
	// 		std::cout << "Parent-Child Bodies: (" << jnt->getParentBodyNode()->getName();
	// 	else 
	// 		std::cout << "Parent-Child Bodies: ( world ";
	// 	std::cout << ", " << jnt->getChildBodyNode()->getName() << ")\n";
		
	// 	std::cout << "DOFs:\n";
	// 	for (std::size_t j = 0; j < jnt->getNumDofs(); j++) {
	// 		std::cout << jnt->getDof(j)->getName() << "\t, id: " << jnt->getDof(j)->getIndexInSkeleton() << ", value: " << jnt->getDof(j)->getPosition() << std::endl; //the same as jnt->getIndexInSkeleton(j);
	// 	}
	// }

	// auto pelvis_y_pos = mCharacter->GetSkeleton()->getJoint(0)->getDof(4)->getPosition();
	// mCharacter->GetSkeleton()->getJoint(0)->getDof(4)->setPosition(pelvis_y_pos-0.0403); //talus y translation from human.xml

	// mCharacter->GetSkeleton()->getJoint(12)->getDof(0)->setPosition(0.5); //torso rot_x
	// auto initial_COM = mCharacter->GetSkeleton()->getCOM();
	// auto initial_COM_vel = mCharacter->GetSkeleton()->getCOMLinearVelocity();
	// mCharacter->GetSkeleton()->getJoint(3)->getDof(0)->setPosition(-0.5);
	// mCharacter->GetSkeleton()->getJoint(8)->getDof(0)->setPosition(-0.5);

		

	// std::pair<Eigen::VectorXd,Eigen::VectorXd> pv = mCharacter->GetTargetPosAndVel(t,1.0/mControlHz);
	// Eigen::VectorXd mTargetPositions = pv.first; Eigen::VectorXd mTargetVelocities = pv.second;
	// Eigen::VectorXd cpos = mCharacter->GetSkeleton()->getPositions(); Eigen::VectorXd cvel = mCharacter->GetSkeleton()->getVelocities();

	// std::cout << "Current positions: " << cpos << std::endl;
	// std::cout << "Current velocities: " << cvel << std::endl; 

 //dart weld constraint

	// BodyNode* bn1 = mCharacter->GetSkeleton()->getBodyNode("world");
 	// BodyNode* bn2 = mCharacter->GetSkeleton()->getBodyNode("TalusL");
	// auto mWeldJoint = std::make_shared<constraint::WeldJointConstraint>(bn1, bn2);
	// Eigen::Isometry3d t1 = bn1->getTransform();
 	// Eigen::Isometry3d t2 = bn2->getTransform();
	// // t1.translation() = t1*std::get<2>("world");
	// // t2.translation() = t2*std::get<3>("TalusL");
	// mWeldJoint->setRelativeTransform(t2.inverse()*t1);
	// auto mWorld(std::make_shared<World>());
	// mWorld->getConstraintSolver()->addConstraint(mWeldJoint);
	// index for ankle double ; cpos[ankleIndex_l] = ankle_angle; cpos[ankleIndex_r] = ankle_angle;
	// std::cout<<mTargetPositions<<std::endl;
	// std::cout<<mTargetVelocities<<std::endl;


 //Eigen::VectorXd mTargetZeroPositions = 0 * 
 //mCharacter->GetSkeleton()->setPositions(mTargetPositions);
 //mCharacter->GetSkeleton()->setVelocities(mTargetVelocities);
 //mCharacter->GetSkeleton()->computeForwardKinematics(true,false,false);

	MASS::Window* window;
	if(argc == 2)
	{
		window = new MASS::Window(env);
	}
	else
	{
		if(env->GetUseMuscle())
		{
			if(argc!=4){
				std::cout<<"Please provide two networks"<<std::endl;
				return 0;
			}
			window = new MASS::Window(env,argv[2],argv[3]);
		}
		else
		{
			if(argc!=3)
			{
				std::cout<<"Please provide the network"<<std::endl;
				return 0;
			}
			window = new MASS::Window(env,argv[2]);
		}
	}
	// if(argc==1)
	// 	window = new MASS::Window(env);
	// else if (argc==2)
	// 	window = new MASS::Window(env,argv[1]);
	// else if (argc==3)
	// 	window = new MASS::Window(env,argv[1],argv[2]);
	
	window->initWindow(1920,1080,"gui");
	glutMainLoop();
}

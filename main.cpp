#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/utils/utils.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <dart/gui/gui.hpp>
#include "HRP4WorldNode.hpp"
#include "HRP4EventHandler.hpp"
#include "HRP4Widget.hpp"
#include <dart/common/LocalResourceRetriever.hpp>



int main(int argc, char* argv[])
{
  // Create a world  
  dart::simulation::WorldPtr world(new dart::simulation::World);

  // Load ground and Nao robot and add them to the world
  dart::utils::DartLoader urdfLoader;
  auto ground = urdfLoader.parseSkeleton(
	"/home/filippo/DART codes/HRP-4 IS-MPC Basic/urdf/ground.urdf");
  auto hrp4 = urdfLoader.parseSkeleton(
        "/home/filippo/DART codes/HRP-4 IS-MPC Basic/urdf/hrp4.urdf"); 
  world->addSkeleton(ground);
  world->addSkeleton(hrp4);

  // set joint actuator type
  std::vector<size_t> indices;
  std::vector<dart::dynamics::Joint::ActuatorType> types;
  double sufficient_force = 100;// 20;

  for (size_t i = 0; i < hrp4->getNumJoints(); i++) {
	  size_t  dim   = hrp4->getJoint(i)->getNumDofs();
	  if(dim==6){
		  hrp4->getJoint(i)->setActuatorType(dart::dynamics::Joint::PASSIVE);
	  }
	  if(dim==1){
		  hrp4->getJoint(i)->setActuatorType(dart::dynamics::Joint::SERVO);
		  hrp4->getJoint(i)->setForceUpperLimit(0, sufficient_force);
		  hrp4->getJoint(i)->setForceLowerLimit(0, -sufficient_force);
		  hrp4->getJoint(i)->setPositionLimitEnforced(true);
	  }
  }


  // Set gravity of the world
  world->setGravity(Eigen::Vector3d(0.0, 0.0, -9.81));
  world->setTimeStep(1.0/100.0);


  // Wrap a WorldNode around it
  osg::ref_ptr<NaoWorldNode> node
      = new NaoWorldNode(world, hrp4);
  node->setNumStepsPerCycle(1);

  // Create a Viewer and set it up with the WorldNode
  dart::gui::osg::ImGuiViewer viewer;
  viewer.addWorldNode(node);

  // Add control widget for nao
  viewer.getImGuiHandler()->addWidget(
        std::make_shared<NaoWidget>(&viewer, node.get(), world));

  // Pass in the custom event handler
  viewer.addEventHandler(new NaoEventHandler(node));

  // Set the dimensions for the window
  viewer.setUpViewInWindow(0, 0, 1280, 960);

  // Set the window name
  viewer.realize();
  osgViewer::Viewer::Windows windows;
  viewer.getWindows(windows);
  windows.front()->setWindowName("HRP-4 MPC");

  // Adjust the viewpoint of the Viewer
/*
  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 5.14,  3.28, 6.28)*0.7,
        ::osg::Vec3d( 0.50,  -1.00, 0.00),
        ::osg::Vec3d( 0.00,  0.00, 0.1));/**/
  viewer.getCameraManipulator()->setHomePosition(
        ::osg::Vec3d( 5.14,  3.28, 6.28)*0.7,
        ::osg::Vec3d( 0.50,  -1.00, 0.00),
        ::osg::Vec3d( 0.00,  0.00, 0.1));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();
}

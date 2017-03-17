/*
	Bullet Tutorial Hello World: http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World
*/
#include <iostream>
#include "btBulletDynamicsCommon.h"
#include <BulletCollision\Gimpact\btGImpactCollisionAlgorithm.h>

int main() {
	/*
		We need to specify what Broadphase algorithm we want to use. Choosing the broadphase is important 
		if the world will have a lot of rigid bodies in it, since it has to somehow check every pair which 
		when implemented naively is an O(n^2) problem.
		Additional Information For Broadphase: http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Broadphase

		Build our Broadphase
	*/
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();

	/*
		The broadphase is an excellent place for eliminating object pairs that should not collide. 
		This can be for performance or gameplay reasons. You can use the collision dispatcher to register a 
		callback that filters overlapping broadphase proxies so that the collisions are not processed by the rest of the system. 
		More information in Collision Things : http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Collision_Things.
		The collision configuration allows you to fine tune the algorithms used for the full (not broadphase) collision detection. Here be dragons!

		Set up the collision configuration and dispatcher
	*/

	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

	/*
		If you introduce different types of collision object later (eg meshes using btGImpactMeshShape) 
		then you may need to register a collision algorithm to get collisions recognised:

		The actual physics solver
	*/
	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

	/*
		We also need a "solver". This is what causes the objects to interact properly, taking into account gravity, 
		game logic supplied forces, collisions, and hinge constraints. It does a good job as long as you don't push it to extremes, 
		and is one of the bottlenecks in any high performance simulation. There are parallel versions available for some threading models

		The world.
	*/

	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	/*
		Now, we can finally instantiate the dynamics world:
	*/

	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

	/*
		One last (rather obvious) line sets the gravity. We have chosen the Y axis to be "up".
	*/
	dynamicsWorld->setGravity(btVector3(0, -10, 0));

	// Do_everything_else_here

	/*
		Collision Shapes
		We will place a ground plane running through the origin.
		For pedantic purposes, we will define the collision shape with an offset of 1 unit from the origin.
	*/
	btCollisionShape* groundShape = new btStaticPlaneShape(btVector3(0, 1, 0), 1);

	/*
		The shape that we will let fall from the sky is a sphere with a radius of 1 metre.
	*/
	btCollisionShape* fallShape = new btSphereShape(1);

	/*
		Rigid Bodies
		Now we can add the collision shapes into our scene, positioning them with rigid body instances.
		Let's first instantiate the ground. Its orientation is the identity, Bullet quaternions are specified in x,y,z,w form. 
		The position is 1 metre below the ground, which compensates the 1m offset we had to put into the shape itself. 
		Motionstates are covered in detail on a page dedicated to them: MotionStates: http://www.bulletphysics.org/mediawiki-1.5.8/index.php/MotionStates
	*/
	btDefaultMotionState* groundMotionState =
		new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, -1, 0)));

	/*
		The first and last parameters of the following constructor are the mass and inertia of the ground. 
		Since the ground is static, we represent this by filling these values with zeros. Bullet considers passing a mass of zero 
		equivalent to making a body with infinite mass - it is immovable.
	*/
	btRigidBody::btRigidBodyConstructionInfo
		groundRigidBodyCI(0, groundMotionState, groundShape, btVector3(0, 0, 0));
	btRigidBody* groundRigidBody = new btRigidBody(groundRigidBodyCI);

	/*
		Finally, we add the ground to the world:
	*/
	dynamicsWorld->addRigidBody(groundRigidBody);

	/*
		Adding the falling sphere is very similar. We will place it 50m above the ground.
	*/
	btDefaultMotionState* fallMotionState =
		new btDefaultMotionState(btTransform(btQuaternion(0, 0, 0, 1), btVector3(0, 50, 0)));

	/*
		Since it's dynamic we will give it a mass of 1kg. 
		I can't remember how to calculate the inertia of a sphere, but that doesn't matter because Bullet provides a utility function:
	*/

	btScalar mass = 1;
	btVector3 fallInertia(0, 0, 0);
	fallShape->calculateLocalInertia(mass, fallInertia);

	/*
		Now we can construct the rigid body just like before, and add it to the world:
	*/
	btRigidBody::btRigidBodyConstructionInfo fallRigidBodyCI(mass, fallMotionState, fallShape, fallInertia);
	btRigidBody* fallRigidBody = new btRigidBody(fallRigidBodyCI);
	dynamicsWorld->addRigidBody(fallRigidBody);

	/*
		Stepping the simulation
		This is where the fun begins. We will step the simulation 300 times, at an interval of 60hz. 
		This will give the sphere enough time to hit the ground under the influence of gravity. Each step, we will print out its height above the ground.

		The stepSimulation function does what you'd expect, but its interface is fairly complicated. Read Stepping The World for more details.
		http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Stepping_The_World
	*/
	for (int i = 0; i < 300; i++) {

		dynamicsWorld->stepSimulation(1 / 60.f, 10);

		btTransform trans;
		fallRigidBody->getMotionState()->getWorldTransform(trans);

		std::cout << "sphere height: " << trans.getOrigin().getY() << std::endl;
	}

	std::cin.get();
	// End of Do_everything_else_here

	/*
		Bullet has a policy of "whoever allocates, also deletes" memory, so all of these structures must be deleted at the end of main().
		We now have prepared the first few lines that are common to *any* Bullet application. The code up to this point looks like this:

		Clean up behind ourselves like good little programmers
	*/

	dynamicsWorld->removeRigidBody(fallRigidBody);
	delete fallRigidBody->getMotionState();
	delete fallRigidBody;

	dynamicsWorld->removeRigidBody(groundRigidBody);
	delete groundRigidBody->getMotionState();
	delete groundRigidBody;

	delete groundShape;

	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;
	delete fallShape;

	return 0;
}
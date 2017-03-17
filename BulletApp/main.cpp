/*
	Bullet Tutorial Hello World: http://www.bulletphysics.org/mediawiki-1.5.8/index.php/Hello_World
*/

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

	// End of Do_everything_else_here

	/*
		Bullet has a policy of "whoever allocates, also deletes" memory, so all of these structures must be deleted at the end of main().
		We now have prepared the first few lines that are common to *any* Bullet application. The code up to this point looks like this:

		Clean up behind ourselves like good little programmers
	*/
	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;

	return 0;
}
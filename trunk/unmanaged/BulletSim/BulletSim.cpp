/*
 * Copyright (c) Contributors, http://opensimulator.org/
 * See CONTRIBUTORS.TXT for a full list of copyright holders.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyrightD
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the OpenSimulator Project nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE DEVELOPERS ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE CONTRIBUTORS BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "BulletSim.h"
#include "Util.h"

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "BulletCollision/CollisionShapes/btTriangleShape.h"
#include "LinearMath/btGeometryUtil.h"

#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"

#if defined(USEBULLETHACD)
#if defined(__linux__) || defined(__APPLE__) 
#include "HACD/hacdHACD.h"
#elif defined(_WIN32) || defined(_WIN64)
#include "HACD/hacdHACD.h"
#else
#error "Platform type not understood."
#endif
#endif

#if defined(USEVHACD)
#include "VHACD.h"
using namespace VHACD;
#endif

// Linkages to debugging dump routines
extern "C" void DumpPhysicsStatistics2(BulletSim* sim);
extern "C" void DumpActivationInfo2(BulletSim* sim);

// Bullet has some parameters that are just global variables
extern ContactAddedCallback gContactAddedCallback;
extern btScalar gContactBreakingThreshold;

BulletSim::BulletSim(btScalar maxX, btScalar maxY, btScalar maxZ)
{
	bsDebug_Initialize();

	// Make sure structures that will be created in initPhysics are marked as not created
	m_worldData.dynamicsWorld = NULL;

	m_worldData.sim = this;

	m_worldData.MinPosition = btVector3(0, 0, 0);
	m_worldData.MaxPosition = btVector3(maxX, maxY, maxZ);
}

// Called when a collision point is being added to the manifold.
// This is used to modify the collision normal to make meshes collidable on only one side.
// Based on rule: "Ignore collisions if dot product of hit normal and a vector pointing to the center of the the object
//     is less than zero."
// Code based on example code given in: http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=3052&start=15#p12308
// Code used under Creative Commons, ShareAlike, Attribution as per Bullet forums.
static void SingleSidedMeshCheck(btManifoldPoint& cp, const btCollisionObjectWrapper* colObj, int partId, int index)
{
        const btCollisionShape* shape = colObj->getCollisionShape();

		// TODO: compound shapes don't have this proxy type.  How to get vector pointing to object middle?
        if (shape->getShapeType() != TRIANGLE_SHAPE_PROXYTYPE) return;
        const btTriangleShape* tshape = static_cast<const btTriangleShape*>(colObj->getCollisionShape());

        btVector3 v1 = tshape->m_vertices1[0];
        btVector3 v2 = tshape->m_vertices1[1];
        btVector3 v3 = tshape->m_vertices1[2];

		// Create a normal pointing to the center of the mesh based on the first triangle
        btVector3 normal = (v2-v1).cross(v3-v1);

		// Since the collision points are in local coordinates, create a transform for the collided
		//    object like it was at <0, 0, 0>.
        btTransform orient = colObj->getWorldTransform();
        orient.setOrigin( btVector3(0.0, 0.0, 0.0) );

		// Rotate that normal to world coordinates and normalize
        normal = orient * normal;
        normal.normalize();

		// Dot the normal to the center and the collision normal to see if the collision normal is pointing in or out.
        btScalar dot = normal.dot(cp.m_normalWorldOnB);
        btScalar magnitude = cp.m_normalWorldOnB.length();
        normal *= dot > 0 ? magnitude : -magnitude;

        cp.m_normalWorldOnB = normal;
}

// Check the collision point and modify the collision direction normal to only point "out" of the mesh
static bool SingleSidedMeshCheckCallback(btManifoldPoint& cp, 
							const btCollisionObjectWrapper* colObj0, int partId0, int index0,
							const btCollisionObjectWrapper* colObj1, int partId1, int index1)
{
        SingleSidedMeshCheck(cp, colObj0, partId0, index0);
        SingleSidedMeshCheck(cp, colObj1, partId1, index1);
        return true;
}

// After each sub-step, this routine is called.
// Collisions need to be checked in the substep because, in the full simulation step, something
//    could bounce off another object thus making no collision when the simulatin step is complete.
static void SubstepCollisionCallback(btDynamicsWorld *world, btScalar timeStep) {
	BulletSim* bulletSim = (BulletSim*)world->getWorldUserInfo();

	int numManifolds = world->getDispatcher()->getNumManifolds();
	for (int j = 0; j < numManifolds; j++)
	{
		btPersistentManifold* contactManifold = world->getDispatcher()->getManifoldByIndexInternal(j);
		int numContacts = contactManifold->getNumContacts();
		if (numContacts == 0)
			continue;

		const btCollisionObject* objA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
		const btCollisionObject* objB = static_cast<const btCollisionObject*>(contactManifold->getBody1());

		// When two objects collide, we only report one contact point
		const btManifoldPoint& manifoldPoint = contactManifold->getContactPoint(0);
		const btVector3& contactPoint = manifoldPoint.getPositionWorldOnB();
		const btVector3 contactNormal = -manifoldPoint.m_normalWorldOnB;	// make relative to A
		const float penetration = manifoldPoint.getDistance();

		bulletSim->RecordCollision(objA, objB, contactPoint, contactNormal, penetration);

		if (bulletSim->collisionsThisFrame >= bulletSim->maxCollisionsPerFrame) 
			break;
	}

	// Any ghost objects must be relieved of their collisions.
	WorldData::SpecialCollisionObjectMapType::iterator it = bulletSim->getWorldData()->specialCollisionObjects.begin();
	for (; it != bulletSim->getWorldData()->specialCollisionObjects.end(); it++)
	{
		if (bulletSim->collisionsThisFrame >= bulletSim->maxCollisionsPerFrame) 
			break;

		btCollisionObject* collObj = it->second;
		btPairCachingGhostObject* obj = (btPairCachingGhostObject*)btGhostObject::upcast(collObj);
		if (obj)
		{
			bulletSim->RecordGhostCollisions(obj);
		}
	}
}

void BulletSim::initPhysics2(ParamBlock* parms, 
							int maxCollisions, CollisionDesc* collisionArray, 
							int maxUpdates, EntityProperties* updateArray)
{
	// Tell the world we're initializing and output size of types so we can
	//    debug mis-alignments when changing architecture.
	m_worldData.BSLog("InitPhysics: sizeof(int)=%d, sizeof(long)=%d, sizeof(long long)=%d, sizeof(float)=%d",
		sizeof(int), sizeof(long), sizeof(long long), sizeof(float));

	// remember the pointers to pinned memory for returning collisions and property updates
	maxCollisionsPerFrame = maxCollisions;
	m_collidersThisFrameArray = collisionArray;
	m_maxUpdatesPerFrame = maxUpdates;
	m_updatesThisFrameArray = updateArray;

	// Parameters are in a block of pinned memory
	m_worldData.params = parms;

	// create the functional parts of the physics simulation
	btDefaultCollisionConstructionInfo cci;
	// if you are setting a pool size, you should disable dynamic allocation
	if (m_worldData.params->maxPersistantManifoldPoolSize > 0)
	{
		cci.m_defaultMaxPersistentManifoldPoolSize = (int)m_worldData.params->maxPersistantManifoldPoolSize;
		m_worldData.BSLog("initPhysics2: setting defaultMaxPersistentManifoldPoolSize = %f", m_worldData.params->maxPersistantManifoldPoolSize);
	}
	if (m_worldData.params->maxCollisionAlgorithmPoolSize > 0)
	{
		cci.m_defaultMaxCollisionAlgorithmPoolSize = (int)m_worldData.params->maxCollisionAlgorithmPoolSize;
		m_worldData.BSLog("initPhysics2: setting defaultMaxCollisionAlgorithmPoolSize = %f", m_worldData.params->maxCollisionAlgorithmPoolSize);
	}
	
	m_collisionConfiguration = new btDefaultCollisionConfiguration(cci);
	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	// optional but not a good idea
	if (m_worldData.params->shouldDisableContactPoolDynamicAllocation != ParamFalse)
	{
		m_dispatcher->setDispatcherFlags(
				btCollisionDispatcher::CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION | m_dispatcher->getDispatcherFlags());
		m_worldData.BSLog("initPhysics2: adding CD_DISABLE_CONTACTPOOL_DYNAMIC_ALLOCATION to dispatcherFlags");
	}

	m_broadphase = new btDbvtBroadphase();

	// the following is needed to enable GhostObjects
	m_broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
	
	m_solver = new btSequentialImpulseConstraintSolver();

	// Create the world
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher, m_broadphase, m_solver, m_collisionConfiguration);
	m_worldData.dynamicsWorld = dynamicsWorld;

	// Register callback for sub-step collisons
	dynamicsWorld->setInternalTickCallback(SubstepCollisionCallback, (void*) this);

	// Register GImpact collsions since that type can be created
	btGImpactCollisionAlgorithm::registerAlgorithm((btCollisionDispatcher*)dynamicsWorld->getDispatcher());
	
	// disable or enable the continuious recalculation of the static AABBs
	// http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=4991
	// Note that if disabled, movement or changes to a static object will not update the AABB. Must do it explicitly.
	dynamicsWorld->setForceUpdateAllAabbs(m_worldData.params->shouldForceUpdateAllAabbs != ParamFalse);
	m_worldData.BSLog("initPhysics2: setForceUpdateAllAabbs = %d", (m_worldData.params->shouldForceUpdateAllAabbs != ParamFalse));
	
	// Randomizing the solver order makes object stacking more stable at a slight performance cost
	if (m_worldData.params->shouldRandomizeSolverOrder != ParamFalse)
	{
		dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_RANDMIZE_ORDER;
		m_worldData.BSLog("initPhysics2: setting SOLVER_RANMIZE_ORDER");

	}

	// Change the breaking threshold if specified.
	if (m_worldData.params->globalContactBreakingThreshold != 0)
	{
		gContactBreakingThreshold = m_worldData.params->globalContactBreakingThreshold;
		m_worldData.BSLog("initPhysics2: setting gContactBreakingThreshold = %f", m_worldData.params->globalContactBreakingThreshold);
	}

	// setting to false means the islands are not reordered and split up for individual processing
	if (m_worldData.params->shouldSplitSimulationIslands != ParamFalse)
	{
		dynamicsWorld->getSimulationIslandManager()->setSplitIslands(true);
		m_worldData.BSLog("initPhysics2: setting setSplitIslands => true");
	}
	else
	{
		dynamicsWorld->getSimulationIslandManager()->setSplitIslands(false);
		m_worldData.BSLog("initPhysics2: setting setSplitIslands => false");
	}

	if (m_worldData.params->useSingleSidedMeshes != ParamFalse)
	{
		gContactAddedCallback = SingleSidedMeshCheckCallback;
		m_worldData.BSLog("initPhysics2: enabling SingleSidedMeshCheckCallback");
	}

	/*
	// Performance speedup: http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?p=14367
	// Actually a NOOP unless Bullet is compiled with USE_SEPDISTANCE_UTIL2 set.
	dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = btScalar(0.01);
	*/

	// Performance speedup: from BenchmarkDemo.cpp, ln 381
	if (m_worldData.params->shouldEnableFrictionCaching != ParamFalse)
	{
		m_worldData.dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_ENABLE_FRICTION_DIRECTION_CACHING; //don't recalculate friction values each frame
		m_worldData.BSLog("initPhysics2: enabling SOLVER_ENABLE_FRICTION_DIRECTION_CACHING");
	}

	// Increasing solver interations can increase stability.
	if (m_worldData.params->numberOfSolverIterations > 0)
	{
		m_worldData.dynamicsWorld->getSolverInfo().m_numIterations = (int)m_worldData.params->numberOfSolverIterations;
		m_worldData.BSLog("initPhysics2: setting solver iterations = %f", m_worldData.params->numberOfSolverIterations);
	}

	// Earth-like gravity
	dynamicsWorld->setGravity(btVector3(0.f, 0.f, m_worldData.params->gravity));

	m_dumpStatsCount = 0;
	if (m_worldData.debugLogCallback != NULL)
	{
		m_dumpStatsCount = (int)m_worldData.params->physicsLoggingFrames;
		if (m_dumpStatsCount != 0)
			m_worldData.BSLog("Logging detailed physics stats every %d frames", m_dumpStatsCount);
	}

	// Information on creating a custom collision computation routine and a pointer to the computation
	// of friction and restitution at:
	// http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?f=9&t=7922

	// foreach body that you want the callback, enable it with:
	// body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);

}

void BulletSim::exitPhysics2()
{
	if (m_worldData.dynamicsWorld == NULL)
		return;

	// Delete solver
	if (m_solver != NULL)
	{
		delete m_solver;
		m_solver = NULL;
	}

	// Delete broadphase
	if (m_broadphase != NULL)
	{
		delete m_broadphase;
		m_broadphase = NULL;
	}

	// Delete dispatcher
	if (m_dispatcher != NULL)
	{
		delete m_dispatcher;
		m_dispatcher = NULL;
	}

	// Delete collision config
	if (m_collisionConfiguration != NULL)
	{
		delete m_collisionConfiguration;
		m_collisionConfiguration = NULL;
	}
}

// Step the simulation forward by one full step and potentially some number of substeps
int BulletSim::PhysicsStep2(btScalar timeStep, int maxSubSteps, btScalar fixedTimeStep, int* updatedEntityCount, int* collidersCount)
{
	int numSimSteps = 0;

	if (m_worldData.dynamicsWorld)
	{

		// All collisions are recorded by the substep callback which populate m_collidersThisFrame
		m_collidersThisFrame.clear();
		collisionsThisFrame = 0;

		// The simulation calls the SimMotionState to put object updates into updatesThisFrame.
		// m_worldData.BSLog("Before step");
		numSimSteps = m_worldData.dynamicsWorld->stepSimulation(timeStep, maxSubSteps, fixedTimeStep);
		// m_worldData.BSLog("After step. Steps=%d,updates=%d", numSimSteps, m_worldData.updatesThisFrame.size());

		if (m_dumpStatsCount != 0)
		{
			if (--m_dumpStatsCount <= 0)
			{
				m_dumpStatsCount = (int)m_worldData.params->physicsLoggingFrames;
				// DumpPhysicsStatistics2(this);
				DumpActivationInfo2(this);
			}
		}

		// OBJECT UPDATES =================================================================
		// Put all of the updates this frame into m_updatesThisFrameArray
		int updates = 0;
		if (m_worldData.updatesThisFrame.size() > 0)
		{
			WorldData::UpdatesThisFrameMapType::const_iterator it = m_worldData.updatesThisFrame.begin(); 
			for (; it != m_worldData.updatesThisFrame.end(); it++)
			{
				m_updatesThisFrameArray[updates] = *(it->second);
				updates++;
				if (updates >= m_maxUpdatesPerFrame) 
					break;
			}
			m_worldData.updatesThisFrame.clear();
		}

		// Update the values passed by reference into this function
		*updatedEntityCount = updates;

		*collidersCount = collisionsThisFrame;
	}

	return numSimSteps;
}

void BulletSim::RecordCollision(const btCollisionObject* objA, const btCollisionObject* objB, 
					const btVector3& contact, const btVector3& norm, const float penetration)
{
	btVector3 contactNormal = norm;

	// One of the objects has to want to hear about collisions
	if ((objA->getCollisionFlags() & BS_WANTS_COLLISIONS) == 0
			&& (objB->getCollisionFlags() & BS_WANTS_COLLISIONS) == 0)
	{
		return;
	}

	// Get the IDs of colliding objects (stored in the one user definable field)
	IDTYPE idA = CONVLOCALID(objA->getUserPointer());
	IDTYPE idB = CONVLOCALID(objB->getUserPointer());

	// Make sure idA is the lower ID so we don't record both 'A hit B' and 'B hit A'
	if (idA > idB)
	{
		IDTYPE temp = idA;
		idA = idB;
		idB = temp;
		contactNormal = -contactNormal;
	}

	// m_worldData.BSLog("Collision: idA=%d, idB=%d, contact=<%f,%f,%f>", idA, idB, contact.getX(), contact.getY(), contact.getZ());

	// Create a unique ID for this collision from the two colliding object IDs
	// We check for duplicate collisions between the two objects because
	//    there may be multiple hulls involved and thus multiple collisions.
	// TODO: decide if this is really a problem -- can this checking be removed?
	//    How many duplicate manifolds are there?
	// Also, using obj->getCollisionFlags() we can pass up only the collisions
	//    for one object if it's the only one requesting. Wouldn't have to do
	//    the "Collide(a,b);Collide(b,a)" in BSScene.
	COLLIDERKEYTYPE collisionID = ((COLLIDERKEYTYPE)idA << 32) | idB;

	// If this collision has not been seen yet, record it
	if (m_collidersThisFrame.find(collisionID) == m_collidersThisFrame.end())
	{
		m_collidersThisFrame.insert(collisionID);

		CollisionDesc cDesc;
		cDesc.aID = idA;
		cDesc.bID = idB;
		cDesc.point = contact;
		cDesc.normal = contactNormal;
		cDesc.penetration = penetration;
		m_collidersThisFrameArray[collisionsThisFrame] = cDesc;
		collisionsThisFrame++;
	}
}

void BulletSim::RecordGhostCollisions(btPairCachingGhostObject* obj)
{
	btManifoldArray   manifoldArray;
	btBroadphasePairArray& pairArray = obj->getOverlappingPairCache()->getOverlappingPairArray();
	int numPairs = pairArray.size();

	// For all the pairs of sets of contact points
	for (int i=0; i < numPairs; i++)
	{
		if (collisionsThisFrame >= maxCollisionsPerFrame) 
			break;

		manifoldArray.clear();
		const btBroadphasePair& pair = pairArray[i];

		// The real representation is over in the world pair cache
		btBroadphasePair* collisionPair = m_worldData.dynamicsWorld->getPairCache()->findPair(pair.m_pProxy0,pair.m_pProxy1);
		if (!collisionPair)
			continue;

		if (collisionPair->m_algorithm)
			collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);

		// The collision pair has sets of collision points (manifolds)
		for (int j=0; j < manifoldArray.size(); j++)
		{
			btPersistentManifold* contactManifold = manifoldArray[j];
			int numContacts = contactManifold->getNumContacts();

			const btCollisionObject* objA = static_cast<const btCollisionObject*>(contactManifold->getBody0());
			const btCollisionObject* objB = static_cast<const btCollisionObject*>(contactManifold->getBody1());

			// TODO: this is a more thurough check than the regular collision code --
			//     here we find the penetrating contact in the manifold but for regular
			//     collisions we assume the first point in the manifold is good enough.
			//     Decide of this extra checking is required or if first point is good enough.
			for (int p=0; p < numContacts; p++)
			{
				const btManifoldPoint& pt = contactManifold->getContactPoint(p);
				// If a penetrating contact, this is a hit
				if (pt.getDistance()<0.f)
				{
					const btVector3& contactPoint = pt.getPositionWorldOnA();
					const btVector3& normalOnA = -pt.m_normalWorldOnB;
					RecordCollision(objA, objB, contactPoint, normalOnA, pt.getDistance());
					// Only one contact point for each set of colliding objects
					break;
				}
			}
		}
	}
}

btCollisionShape* BulletSim::CreateMeshShape2(int indicesCount, int* indices, int verticesCount, float* vertices)
{
	// We must copy the indices and vertices since the passed memory is released when this call returns.
	btIndexedMesh indexedMesh;
	int* copiedIndices = new int[indicesCount];
	__wrap_memcpy(copiedIndices, indices, indicesCount * sizeof(int));
	int numVertices = verticesCount * 3;
	float* copiedVertices = new float[numVertices];
	__wrap_memcpy(copiedVertices, vertices, numVertices * sizeof(float));

	indexedMesh.m_indexType = PHY_INTEGER;
	indexedMesh.m_triangleIndexBase = (const unsigned char*)copiedIndices;
	indexedMesh.m_triangleIndexStride = sizeof(int) * 3;
	indexedMesh.m_numTriangles = indicesCount / 3;
	indexedMesh.m_vertexType = PHY_FLOAT;
	indexedMesh.m_numVertices = verticesCount;
	indexedMesh.m_vertexBase = (const unsigned char*)copiedVertices;
	indexedMesh.m_vertexStride = sizeof(float) * 3;

	btTriangleIndexVertexArray* vertexArray = new btTriangleIndexVertexArray();
	vertexArray->addIndexedMesh(indexedMesh, PHY_INTEGER);

	bool useQuantizedAabbCompression = true;
	bool buildBvh = true;
	btBvhTriangleMeshShape* meshShape = new btBvhTriangleMeshShape(vertexArray, useQuantizedAabbCompression, buildBvh);

	meshShape->setMargin(m_worldData.params->collisionMargin);

	return meshShape;
}

btCollisionShape* BulletSim::CreateGImpactShape2(int indicesCount, int* indices, int verticesCount, float* vertices)
{
	// We must copy the indices and vertices since the passed memory is released when this call returns.
	btIndexedMesh indexedMesh;
	int* copiedIndices = new int[indicesCount];
	__wrap_memcpy(copiedIndices, indices, indicesCount * sizeof(int));
	int numVertices = verticesCount * 3;
	float* copiedVertices = new float[numVertices];
	__wrap_memcpy(copiedVertices, vertices, numVertices * sizeof(float));

	indexedMesh.m_indexType = PHY_INTEGER;
	indexedMesh.m_triangleIndexBase = (const unsigned char*)copiedIndices;
	indexedMesh.m_triangleIndexStride = sizeof(int) * 3;
	indexedMesh.m_numTriangles = indicesCount / 3;
	indexedMesh.m_vertexType = PHY_FLOAT;
	indexedMesh.m_numVertices = verticesCount;
	indexedMesh.m_vertexBase = (const unsigned char*)copiedVertices;
	indexedMesh.m_vertexStride = sizeof(float) * 3;

	btTriangleIndexVertexArray* vertexArray = new btTriangleIndexVertexArray();
	vertexArray->addIndexedMesh(indexedMesh, PHY_INTEGER);

	btGImpactMeshShape* meshShape = new btGImpactMeshShape(vertexArray);
	m_worldData.BSLog("GreateGImpactShape2: ind=%d, vert=%d", indicesCount, verticesCount);

	meshShape->setMargin(m_worldData.params->collisionMargin);

	// The gimpact shape needs some help to create its AABBs
	meshShape->updateBound();

	return meshShape;
}

btCollisionShape* BulletSim::CreateHullShape2(int hullCount, float* hulls )
{
	// Create a compound shape that will wrap the set of convex hulls
	btCompoundShape* compoundShape = new btCompoundShape(false);

	btTransform childTrans;
	childTrans.setIdentity();

	compoundShape->setMargin(m_worldData.params->collisionMargin);
	
	// Loop through all of the convex hulls and add them to our compound shape
	int ii = 1;
	for (int i = 0; i < hullCount; i++)
	{
		int vertexCount = (int)hulls[ii];

		// Offset this child hull by its calculated centroid
		btVector3 centroid = btVector3((btScalar)hulls[ii+1], (btScalar)hulls[ii+2], (btScalar)hulls[ii+3]);
		childTrans.setOrigin(centroid);
		// m_worldData.BSLog("CreateHullShape2: %d Centroid = <%f,%f,%f>", i, centroid.getX(), &centroid.getY(), &centroid.getZ());	// DEBUG DEBUG
		// Create the child hull and add it to our compound shape
		btScalar* hullVertices = (btScalar*)&hulls[ii+4];
		btConvexHullShape* convexShape = new btConvexHullShape(hullVertices, vertexCount, sizeof(Vector3));
		// for (int j = 0; j < vertexCount; j += 3)	// DEBUG DEBUG
		// {	// DEBUG DEBUG
		// 	m_worldData.BSLog("CreateHullShape2: %d %d <%f,%f,%f>", i, j, 	// DEBUG DEBUG
		// 		hullVertices[j] + 0,	// DEBUG DEBUG
		// 		hullVertices[j] + 1,	// DEBUG DEBUG
		// 		hullVertices[j] + 2	// DEBUG DEBUG
		// 		);	// DEBUG DEBUG
		// }	// DEBUG DEBUG
		convexShape->setMargin(m_worldData.params->collisionMargin);
		convexShape->optimizeConvexHull();
		compoundShape->addChildShape(childTrans, convexShape);

		ii += (vertexCount * 3 + 4);
	}

	return compoundShape;
}

// If using Bullet' convex hull code, refer to following link for parameter setting
// http://kmamou.blogspot.com/2011/11/hacd-parameters.html
// Another useful reference for ConvexDecomp
// http://www.bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=7159

// From a previously created mesh shape, create a convex hull using the Bullet
//   HACD hull creation code. The created hull will go into the hull collection
//   so remember to delete it later.
// Returns the created collision shape or NULL if couldn't create
btCollisionShape* BulletSim::BuildHullShapeFromMesh2(btCollisionShape* mesh, HACDParams* parms)
{
#if defined(USEBULLETHACD)
	// Get the triangle mesh data out of the passed mesh shape
	int shapeType = mesh->getShapeType();
	if (shapeType != TRIANGLE_MESH_SHAPE_PROXYTYPE)
	{
		// If the passed shape doesn't have a triangle mesh, we cannot hullify it.
		m_worldData.BSLog("HACD: passed mesh not TRIANGLE_MESH_SHAPE");	// DEBUG DEBUG
		return NULL;
	}
	btStridingMeshInterface* meshInfo = ((btTriangleMeshShape*)mesh)->getMeshInterface();
	const unsigned char* vertexBase;
	int numVerts;
	PHY_ScalarType vertexType;
	int vertexStride;
	const unsigned char* indexBase;
	int indexStride;
	int numFaces;
	PHY_ScalarType indicesType;
	meshInfo->getLockedReadOnlyVertexIndexBase(&vertexBase, numVerts, vertexType, vertexStride, &indexBase, indexStride, numFaces, indicesType);

	if (vertexType != PHY_FLOAT || indicesType != PHY_INTEGER)
	{
		// If an odd data structure, we cannot hullify
		m_worldData.BSLog("HACD: triangle mesh not of right types");	// DEBUG DEBUG
		return NULL;
	}

	// Create pointers to the vertices and indices as the PHY types that they are
	float* tVertex = (float*)vertexBase;
	int tVertexStride = vertexStride / sizeof(float);
	int* tIndices = (int*) indexBase;
	int tIndicesStride = indexStride / sizeof(int);
	m_worldData.BSLog("HACD: nVertices=%d, nIndices=%d", numVerts, numFaces*3);	// DEBUG DEBUG

	// Copy the vertices/indices into the HACD data structures
	std::vector< HACD::Vec3<HACD::Real> > points;
	std::vector< HACD::Vec3<long> > triangles;
	for (int ii=0; ii < (numVerts * tVertexStride); ii += tVertexStride)
	{
		HACD::Vec3<HACD::Real> vertex(tVertex[ii], tVertex[ii+1],tVertex[ii+2]);
		points.push_back(vertex);
	}
	for(int ii=0; ii < (numFaces * tIndicesStride); ii += tIndicesStride ) 
	{
		HACD::Vec3<long> vertex( tIndices[ii],  tIndices[ii+1], tIndices[ii+2]);
		triangles.push_back(vertex);
	}

	meshInfo->unLockReadOnlyVertexBase(0);
	m_worldData.BSLog("HACD: structures copied");	// DEBUG DEBUG

	// Setup HACD parameters
	HACD::HACD myHACD;
	myHACD.SetPoints(&points[0]);
	myHACD.SetNPoints(points.size());
	myHACD.SetTriangles(&triangles[0]);
	myHACD.SetNTriangles(triangles.size());

	myHACD.SetCompacityWeight((double)parms->compacityWeight);
	myHACD.SetVolumeWeight((double)parms->volumeWeight);
	myHACD.SetNClusters((size_t)parms->minClusters);
	myHACD.SetNVerticesPerCH((size_t)parms->maxVerticesPerHull);
	myHACD.SetConcavity((double)parms->concavity);
	myHACD.SetAddExtraDistPoints(parms->addExtraDistPoints == ParamTrue ? true : false);   
	myHACD.SetAddNeighboursDistPoints(parms->addNeighboursDistPoints == ParamTrue ? true : false);   
	myHACD.SetAddFacesPoints(parms->addFacesPoints == ParamTrue ? true : false); 

	m_worldData.BSLog("HACD: Before compute. nPoints=%d, nTriangles=%d, minClusters=%f, maxVerts=%f", 
		points.size(), triangles.size(), parms->minClusters, parms->maxVerticesPerHull);	// DEBUG DEBUG

	// Hullify the mesh
	myHACD.Compute();
	int nHulls = (int)myHACD.GetNClusters();	
	m_worldData.BSLog("HACD: After compute. nHulls=%d", nHulls);	// DEBUG DEBUG

	// Create the compound shape all the hulls will be added to
	btCompoundShape* compoundShape = new btCompoundShape(true);
	compoundShape->setMargin(m_worldData.params->collisionMargin);

	// Convert each of the built hulls into btConvexHullShape objects and add to the compoundShape
	for (int hul=0; hul < nHulls; hul++)
	{
		size_t nPoints = myHACD.GetNPointsCH(hul);
		size_t nTriangles = myHACD.GetNTrianglesCH(hul);
		m_worldData.BSLog("HACD: Add hull %d. nPoints=%d, nTriangles=%d", hul, nPoints, nTriangles);	// DEBUG DEBUG

		// Get the vertices and indices for one hull
		HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
		HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
		myHACD.GetCH(hul, pointsCH, trianglesCH);

		// Average the location of all the vertices to create a centriod for the hull.
		btAlignedObjectArray<btVector3> vertices;
		btVector3 centroid;
		centroid.setValue(0,0,0);
		for (int ii=0; ii < (int)nTriangles; ii++)
		{
			long tri = trianglesCH[ii].X();
			btVector3 corner1(pointsCH[tri].X(), pointsCH[tri].Y(), pointsCH[tri].Z() );
			vertices.push_back(corner1);
			centroid += corner1;
			tri = trianglesCH[ii].Y();
			btVector3 corner2(pointsCH[tri].X(), pointsCH[tri].Y(), pointsCH[tri].Z() );
			vertices.push_back(corner2);
			centroid += corner2;
			tri = trianglesCH[ii].Z();
			btVector3 corner3(pointsCH[tri].X(), pointsCH[tri].Y(), pointsCH[tri].Z() );
			vertices.push_back(corner3);
			centroid += corner3;
		}
		centroid *= 1.f/((float)(nTriangles * 3));

		for (int ii=0; ii < vertices.size(); ii++)
		{
			vertices[ii] -= centroid;
		}

		delete [] pointsCH;
		delete [] trianglesCH;

		btConvexHullShape* convexShape;
		// Optionally compress the hull a little bit to account for the collision margin.
		if (parms->shouldAdjustCollisionMargin == ParamTrue)
		{
			float collisionMargin = 0.01f;
			
			btAlignedObjectArray<btVector3> planeEquations;
			btGeometryUtil::getPlaneEquationsFromVertices(vertices, planeEquations);

			btAlignedObjectArray<btVector3> shiftedPlaneEquations;
			for (int p=0; p<planeEquations.size(); p++)
			{
				btVector3 plane = planeEquations[p];
				plane[3] += collisionMargin;
				shiftedPlaneEquations.push_back(plane);
			}
			btAlignedObjectArray<btVector3> shiftedVertices;
			btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations,shiftedVertices);
			
			convexShape = new btConvexHullShape(&(shiftedVertices[0].getX()),shiftedVertices.size());
			convexShape->optimizeConvexHull();

		}
		else
		{
			convexShape = new btConvexHullShape(&(vertices[0].getX()),vertices.size());
			convexShape->optimizeConvexHull();
		}
		convexShape->setMargin(m_worldData.params->collisionMargin);

		// Add the hull shape to the compound shape
		btTransform childTrans;
		childTrans.setIdentity();
		childTrans.setOrigin(centroid);
		m_worldData.BSLog("HACD: Add child shape %d", hul);	// DEBUG DEBUG
		compoundShape->addChildShape(childTrans, convexShape);
	}

	return compoundShape;
#else
	return NULL;
#endif
}

#if defined(USEVHACD)
// Instance that is called by VHACD to report hulling progress
class VHACDProgressLog : public IVHACD::IUserCallback
{
	WorldData* m_WorldData;
public:
	VHACDProgressLog(WorldData* wd) {m_WorldData = wd; }
	~VHACDProgressLog() {}
	void Update(const double          overallProgress,
                const double          stageProgress,
                const double          operationProgress,
                const char * const    stage,
                const char * const    operation) 
	 {
		 m_WorldData->BSLog("VHACD: progress=%f, stageProg=%f, opProgress=%f, state=%s, op=%s",
			 overallProgress, stageProgress, operationProgress, stage, operation);
	 }
};
#endif

btCollisionShape* BulletSim::BuildVHACDHullShapeFromMesh2(btCollisionShape* mesh, HACDParams* parms)
{
#if defined(USEVHACD)

	int* triangles;	// array of indesex
	float* points;	// array of coordinates

	// copy the mesh into the structures
	btStridingMeshInterface* meshInfo = ((btTriangleMeshShape*)mesh)->getMeshInterface();

	const unsigned char* vertexBase;	// base of the vertice array
	int numVerts;						// the num of vertices
	PHY_ScalarType vertexType;			// the data type representing the vertices
	int vertexStride;					// bytes between each vertex
	const unsigned char* indexBase;		// base of the index array
	int indexStride;					// bytes between the index values
	int numFaces;						// the number of triangles specified by the indexes
	PHY_ScalarType indicesType;			// the data type of the indexes
	meshInfo->getLockedReadOnlyVertexIndexBase(&vertexBase, numVerts, vertexType, vertexStride, &indexBase, indexStride, numFaces, indicesType);

	if (vertexType != PHY_FLOAT || indicesType != PHY_INTEGER)
	{
		// If an odd data structure, we cannot hullify
		m_worldData.BSLog("VHACD: triangle mesh not of right types");	// DEBUG DEBUG
		return NULL;
	}

	// Create pointers to the vertices and indices as the PHY types that they are
	float* tVertex = (float*)vertexBase;
	int tVertexStride = vertexStride / sizeof(float);
	int* tIndices = (int*) indexBase;
	int tIndicesStride = indexStride / sizeof(int);
	m_worldData.BSLog("VHACD: nVertices=%d, nIndices=%d", numVerts, numFaces*3);	// DEBUG DEBUG

	// Copy the vertices/indices into the HACD data structures
	points = new float[numVerts * 3];
	triangles = new int[numFaces * 3];

	int pp = 0;
	for (int ii=0; ii < (numVerts * tVertexStride); ii += tVertexStride)
	{
		points[pp+0] = tVertex[ii+0];
		points[pp+1] = tVertex[ii+1];
		points[pp+2] = tVertex[ii+2];
		pp += 3;
	}
	pp = 0;
	for(int ii=0; ii < (numFaces * tIndicesStride); ii += tIndicesStride ) 
	{
		triangles[pp+0] = tIndices[ii+0];
		triangles[pp+1] = tIndices[ii+1];
		triangles[pp+2] = tIndices[ii+2];
		pp += 3;
	}

	meshInfo->unLockReadOnlyVertexBase(0);
	m_worldData.BSLog("VHACD: structures copied");	// DEBUG DEBUG

	IVHACD::Parameters vParams;
	vParams.m_resolution              = (unsigned int)parms->vHACDresolution;
    vParams.m_depth                   = (int)parms->vHACDdepth;
    vParams.m_concavity               = (double)parms->vHACDconcavity;
    vParams.m_planeDownsampling       = (int)parms->vHACDplaneDownsampling;
    vParams.m_convexhullDownsampling  = (int)parms->vHACDconvexHullDownsampling;
    vParams.m_alpha                   = (double)parms->vHACDalpha;
    vParams.m_beta                    = (double)parms->vHACDbeta;
	vParams.m_delta                   = (double)parms->vHACDdelta;
    vParams.m_gamma                   = (double)parms->vHACDgamma;
    vParams.m_pca                     = (int)parms->vHACDpca;
    vParams.m_mode                    = (int)parms->vHACDmode;
    vParams.m_maxNumVerticesPerCH     = (unsigned int)parms->vHACDmaxNumVerticesPerCH;
    vParams.m_minVolumePerCH          = (double)parms->vHACDminVolumePerCH;
	vParams.m_callback                = new VHACDProgressLog(&m_worldData);
    // vParams.m_logger                  =
    vParams.m_convexhullApproximation = (parms->vHACDconvexHullApprox == ParamTrue);
    vParams.m_oclAcceleration         = (parms->vHACDoclAcceleration == ParamTrue);

	IVHACD* interfaceVHACD = CreateVHACD();

	bool res = interfaceVHACD->Compute(points, 1, numFaces * 3, triangles, 3, numVerts, vParams);

	unsigned int nConvexHulls = interfaceVHACD->GetNConvexHulls();
	m_worldData.BSLog("VHACD: After compute. nHulls=%d", nConvexHulls);	// DEBUG DEBUG

	// Create the compound shape all the hulls will be added to
	btCompoundShape* compoundShape = new btCompoundShape(true);
	compoundShape->setMargin(m_worldData.params->collisionMargin);

	// Convert each of the built hulls into btConvexHullShape objects and add to the compoundShape
	IVHACD::ConvexHull ch;
	for (unsigned int hul = 0; hul < nConvexHulls; hul++)
	{
		interfaceVHACD->GetConvexHull(hul, ch);
		size_t nPoints = ch.m_nPoints;
		size_t nTriangles = ch.m_nTriangles;
		m_worldData.BSLog("VHACD: Add hull %d. nPoints=%d, nTriangles=%d", hul, nPoints, nTriangles);	// DEBUG DEBUG

		// Average the location of all the vertices to create a centriod for the hull.
		btAlignedObjectArray<btVector3> vertices;
		vertices.reserve(nTriangles);
		btVector3 centroid;
		centroid.setValue(0,0,0);

		/*
		int pp = 0;
		for (int ii=0; ii < nPoints; ii++)
		{
			btVector3 vertex(ch.m_points[pp + 0], ch.m_points[pp + 1], ch.m_points[pp + 2] );
			vertices.push_back(vertex);
			centroid += vertex;
			pp += 3;
			m_worldData.BSLog("VHACD: Hull %d, vertex %d:<%f,%f,%f>", hul, ii, vertex.getX(), vertex.getY(), vertex.getZ());	// DEBUG DEBUG
		}
		// centroid *= 1.f/((float)(nPoints));

		// Move the vertices to have the common centroid
		for (int ii=0; ii < nPoints; ii++)
		{
			vertices[ii] -= centroid;
		}
		*/

		for (int ii=0; ii < nTriangles; ii++)
		{
			int tri = ch.m_triangles[ii] * 3;
			btVector3 vertex(ch.m_points[tri+0], ch.m_points[tri+1], ch.m_points[tri+2]);
			vertices.push_back(vertex);
		}

		btConvexHullShape* convexShape;
		// Optionally compress the hull a little bit to account for the collision margin.
		if (parms->shouldAdjustCollisionMargin == ParamTrue)
		{
			float collisionMargin = 0.01f;
			
			btAlignedObjectArray<btVector3> planeEquations;
			btGeometryUtil::getPlaneEquationsFromVertices(vertices, planeEquations);

			btAlignedObjectArray<btVector3> shiftedPlaneEquations;
			for (int p=0; p<planeEquations.size(); p++)
			{
				btVector3 plane = planeEquations[p];
				plane[3] += collisionMargin;
				shiftedPlaneEquations.push_back(plane);
			}
			btAlignedObjectArray<btVector3> shiftedVertices;
			btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations,shiftedVertices);
			
			convexShape = new btConvexHullShape(shiftedVertices[0], shiftedVertices.size(), sizeof(btVector3));
		}
		else
		{
			convexShape = new btConvexHullShape(vertices[0], vertices.size(), sizeof(btVector3));
		}
		convexShape->setMargin(m_worldData.params->collisionMargin);

		// Add the hull shape to the compound shape
		btTransform childTrans;
		childTrans.setIdentity();
		childTrans.setOrigin(centroid);
		m_worldData.BSLog("HACD: Add child shape %d", hul);	// DEBUG DEBUG
		compoundShape->addChildShape(childTrans, convexShape);
	}

	// The params structure doesn't have a destructor to get rid of logging and progress callbacks
	if (vParams.m_callback)
	{
		delete vParams.m_callback;
		vParams.m_callback = 0;
	}

	delete [] points;
	delete [] triangles;

	interfaceVHACD->Clean();
	interfaceVHACD->Release();

	return compoundShape;
#else
	return NULL;
#endif
}

// Return a btConvexHullShape constructed from the passed btCollisonShape.
// Used to create the separate hulls if using the C# HACD algorithm.
btCollisionShape* BulletSim::BuildConvexHullShapeFromMesh2(btCollisionShape* mesh)
{
	btConvexHullShape* hullShape = new btConvexHullShape();

	// Get the triangle mesh data out of the passed mesh shape
	int shapeType = mesh->getShapeType();
	if (shapeType != TRIANGLE_MESH_SHAPE_PROXYTYPE)
	{
		// If the passed shape doesn't have a triangle mesh, we cannot hullify it.
		m_worldData.BSLog("BuildConvexHullShapeFromMesh2: passed mesh not TRIANGLE_MESH_SHAPE");	// DEBUG DEBUG
		return NULL;
	}
	btStridingMeshInterface* meshInfo = ((btTriangleMeshShape*)mesh)->getMeshInterface();
	const unsigned char* vertexBase;
	int numVerts;
	PHY_ScalarType vertexType;
	int vertexStride;
	const unsigned char* indexBase;
	int indexStride;
	int numFaces;
	PHY_ScalarType indicesType;
	meshInfo->getLockedReadOnlyVertexIndexBase(&vertexBase, numVerts, vertexType, vertexStride, &indexBase, indexStride, numFaces, indicesType);

	if (vertexType != PHY_FLOAT || indicesType != PHY_INTEGER)
	{
		// If an odd data structure, we cannot hullify
		m_worldData.BSLog("BuildConvexHullShapeFromMesh2: triangle mesh not of right types");	// DEBUG DEBUG
		return NULL;
	}

	// Create pointers to the vertices and indices as the PHY types that they are
	float* tVertex = (float*)vertexBase;
	int tVertexStride = vertexStride / sizeof(float);
	int* tIndices = (int*) indexBase;
	int tIndicesStride = indexStride / sizeof(int);
	m_worldData.BSLog("BuildConvexHullShapeFromMesh2: nVertices=%d, nIndices=%d", numVerts, numFaces*3);	// DEBUG DEBUG

	// Add points to the hull shape
	for(int ii=0; ii < (numFaces * tIndicesStride); ii += tIndicesStride ) 
	{
		int point1Index = tIndices[ii + 0] * tVertexStride;
		btVector3 point1 = btVector3(tVertex[point1Index + 0], tVertex[point1Index + 1], tVertex[point1Index + 2] );
		hullShape->addPoint(point1);

		int point2Index = tIndices[ii + 1] * tVertexStride;
		btVector3 point2 = btVector3(tVertex[point2Index + 0], tVertex[point2Index + 1], tVertex[point2Index + 2] );
		hullShape->addPoint(point2);

		int point3Index = tIndices[ii + 2] * tVertexStride;
		btVector3 point3 = btVector3(tVertex[point3Index + 0], tVertex[point3Index + 1], tVertex[point3Index + 2] );
		hullShape->addPoint(point3);
	}

	meshInfo->unLockReadOnlyVertexBase(0);

	return hullShape;
}

btCollisionShape* BulletSim::CreateConvexHullShape2(int indicesCount, int* indices, int verticesCount, float* vertices)
{
	btConvexHullShape* hullShape = new btConvexHullShape();

	for (int ii = 0; ii < indicesCount; ii += 3)
	{
		int point1Index = indices[ii + 0] * 3;
		btVector3 point1 = btVector3(vertices[point1Index + 0], vertices[point1Index + 1], vertices[point1Index + 2] );
		hullShape->addPoint(point1);

		int point2Index = indices[ii + 1] * 3;
		btVector3 point2 = btVector3(vertices[point2Index + 0], vertices[point2Index + 1], vertices[point2Index + 2] );
		hullShape->addPoint(point2);

		int point3Index = indices[ii + 2] * 3;
		btVector3 point3 = btVector3(vertices[point3Index + 0], vertices[point3Index + 1], vertices[point3Index + 2] );
		hullShape->addPoint(point3);

	}
	return hullShape;
}

// TODO: get this code working
SweepHit BulletSim::ConvexSweepTest(btCollisionShape* shape, btVector3& fromPos, btVector3& targetPos, btScalar extraMargin)
{
	return SweepHit();
	/*
	SweepHit hit;
	hit.ID = ID_INVALID_HIT;

	btCollisionObject* castingObject = NULL;

	// Look for a character
	CharactersMapType::iterator cit = m_characters.find(id);
	if (cit != m_characters.end())
		castingObject = cit->second;

	if (!castingObject)
	{
		// Look for a rigid body
		BodiesMapType::iterator bit = m_bodies.find(id);
		if (bit != m_bodies.end())
			castingObject = bit->second;
	}

	if (castingObject)
	{
		btCollisionShape* shape = castingObject->getCollisionShape();

		// Convex sweep test only works with convex objects
		if (shape->isConvex())
		{
			btConvexShape* convex = static_cast<btConvexShape*>(shape);

			// Create transforms to sweep from and to
			btTransform from;
			from.setIdentity();
			from.setOrigin(fromPos);

			btTransform to;
			to.setIdentity();
			to.setOrigin(targetPos);

			btScalar originalMargin = convex->getMargin();
			convex->setMargin(originalMargin + extraMargin);

			// Create a callback for the test
			ClosestNotMeConvexResultCallback callback(castingObject);

			// Do the sweep test
			m_worldData.dynamicsWorld->convexSweepTest(convex, from, to, callback, m_worldData.dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration);

			if (callback.hasHit())
			{
				hit.ID = CONVLOCALID(callback.m_hitCollisionObject->getCollisionShape()->getUserPointer());
				hit.Fraction = callback.m_closestHitFraction;
				hit.Normal = callback.m_hitNormalWorld;
				hit.Point = callback.m_hitPointWorld;
			}

			convex->setMargin(originalMargin);
		}
	}

	return hit;
	*/
}

RaycastHit BulletSim::RayTest(btVector3& from, btVector3& to, short filterGroup, short filterMask)
{
	RaycastHit hit;
	btCollisionWorld::ClosestRayResultCallback hitResult(from, to);
	hitResult.m_collisionFilterGroup = filterGroup;
	hitResult.m_collisionFilterMask = filterMask;

	m_worldData.dynamicsWorld->rayTest(from, to, hitResult);
	if (hitResult.hasHit())
	{
		hit.ID = CONVLOCALID(hitResult.m_collisionObject->getUserPointer());
		hit.Fraction = hitResult.m_closestHitFraction;
		hit.Normal = hitResult.m_hitNormalWorld;
		hit.Point = hitResult.m_hitPointWorld;
	}

	return hit;
}

// TODO: get this code working
const btVector3 BulletSim::RecoverFromPenetration(IDTYPE id)
{
	/*
	// Look for a character
	CharactersMapType::iterator cit = m_characters.find(id);
	if (cit != m_characters.end())
	{
		btCollisionObject* character = cit->second;

		ContactSensorCallback contactCallback(character);
		m_worldData.dynamicsWorld->contactTest(character, contactCallback);

		return contactCallback.mOffset;
	}
	*/
	return btVector3(0.0, 0.0, 0.0);
}

bool BulletSim::UpdateParameter2(IDTYPE localID, const char* parm, float val)
{
	btScalar btVal = btScalar(val);
	btVector3 btZeroVector3 = btVector3(0, 0, 0);

	// changes to the environment
	if (strcmp(parm, "gravity") == 0)
	{
		m_worldData.dynamicsWorld->setGravity(btVector3(0.f, 0.f, val));
		return true;
	}
	return false;
}

// #include "LinearMath/btQuickprof.h"
// Call Bullet to dump its performance stats
// Bullet must be patched to make this work. See BulletDetailLogging.patch
void BulletSim::DumpPhysicsStats()
{
	// CProfileManager::dumpAll();
	return;
}

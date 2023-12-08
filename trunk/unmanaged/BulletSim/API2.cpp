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
#include <stdarg.h>

#include "BulletCollision/CollisionDispatch/btSimulationIslandManager.h"
#include "BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h"
#include "BulletCollision/BroadphaseCollision/btBroadphaseProxy.h"

#include <map>

#if defined(_WIN32) || defined(_WIN64)
    #define DLL_EXPORT __declspec( dllexport )
    #define DLL_IMPORT __declspec( dllimport )
#else
    #define DLL_EXPORT
    #define DLL_IMPORT
#endif

#ifdef __cplusplus
    #define EXTERN_C extern "C"
#else
    #define EXTERN_C extern
#endif

EXTERN_C DLL_EXPORT void DumpConstraint2(BulletSim* sim, btTypedConstraint* constrain);

#pragma warning( disable: 4190 ) // Warning about returning Vector3 that we can safely ignore

// The minimum thickness for terrain. If less than this, we correct
#define TERRAIN_MIN_THICKNESS (0.2)

/**
 * Returns a pointer to a string that identifies the version of the BulletSim.dll
 * @return pointer to zero terminated static string of format BULLETENGINEVERSION,BULLETSIMVERSION ("3.25,1.3")
 */
EXTERN_C DLL_EXPORT const char* GetVersion2()
{
	return BulletSimVersionString.c_str();
}

// DEBUG DEBUG DEBUG =========================================================================================
// USE ONLY FOR VITAL DEBUGGING!!!!
// These are really, really dangerous as they rely on static settings which will only work if there
//     is only one instance of BulletSim.
//     Put in the log messages and, when done, take them out for release.
static int lastNumberOverlappingPairs;
static BulletSim* staticSim;

static void InitCheckOverlappingPairs(BulletSim* pSim)
{
	staticSim = pSim;
	lastNumberOverlappingPairs = staticSim->getDynamicsWorld()->getPairCache()->getNumOverlappingPairs();
}
static void CheckOverlappingPairs(char* pReason)
{
	int thisOverlapping = staticSim->getDynamicsWorld()->getPairCache()->getNumOverlappingPairs();
	if (thisOverlapping != lastNumberOverlappingPairs)
	{
		btBroadphasePairArray& pairArray = staticSim->getDynamicsWorld()->getPairCache()->getOverlappingPairArray();
		int ii = thisOverlapping -1;
		staticSim->getWorldData()->BSLog("Pair cache change. old=%d, new=%d, from=%s. Last added id0=%u, id1=%u",
											lastNumberOverlappingPairs, thisOverlapping, pReason,
											((btCollisionObject*)pairArray[ii].m_pProxy0->m_clientObject)->getUserPointer(),
											((btCollisionObject*)pairArray[ii].m_pProxy1->m_clientObject)->getUserPointer());
		lastNumberOverlappingPairs = thisOverlapping;
	}
}
/*
void __cdecl StaticBSLog(const char* msg, ...)
{
	if (staticSim->getWorldData()->debugLogCallback != NULL) {
		va_list args;
		va_start(args, msg);
		staticSim->getWorldData()->BSLog2(msg, args);
		va_end(args);
	}
}
*/
// END DEBUG DEBUG DEBUG =========================================================================================

/**
 * Initializes the physical simulation.
 * @param maxPosition Top north-east corner of the simulation, with Z being up. The bottom south-west corner is 0,0,0.
 * @param maxCollisions maximum number of collisions that can be reported each tick
 * @param updateArray pointer to pinned memory to return the collision info
 * @param maxUpdates maximum number of property updates that can be reported each tick
 * @param maxCollisions pointer to pinned memory to return the update information
 * @return pointer to the created simulator
 */
EXTERN_C DLL_EXPORT BulletSim* Initialize2(Vector3 maxPosition, ParamBlock* parms,
											int maxCollisions, CollisionDesc* collisionArray,
											int maxUpdates, EntityProperties* updateArray,
											DebugLogCallback* debugLog)
{
	bsDebug_Initialize();

	BulletSim* sim = new BulletSim(maxPosition.X, maxPosition.Y, maxPosition.Z);
	sim->getWorldData()->debugLogCallback = debugLog;
	sim->initPhysics2(parms, maxCollisions, collisionArray, maxUpdates, updateArray);

	return sim;
}

/**
 * Update the internal value of a parameter. Some parameters require changing world state.
 * @param worldID ID of the world to change the paramter in
 * @param localID ID of the object to change the paramter on or special values for NONE or ALL
 * @param parm the name of the parameter to change (must be passed in as lower case)
 * @param value the value to change the parameter to
 */
EXTERN_C DLL_EXPORT bool UpdateParameter2(BulletSim* sim, unsigned int localID, const char* parm, float value)
{
	return sim->UpdateParameter2(localID, parm, value);
}

/**
 * Shuts down the physical simulation.
 * @param worldID ID of the world to shut down.
 */
EXTERN_C DLL_EXPORT void Shutdown2(BulletSim* sim)
{
	sim->exitPhysics2();
	bsDebug_AllDone();
	delete sim;
}

// Very low level reset of collision proxy pool
EXTERN_C DLL_EXPORT void ResetBroadphasePool(BulletSim* sim)
{
	sim->getDynamicsWorld()->getBroadphase()->resetPool(sim->getDynamicsWorld()->getDispatcher());
}
// Very low level reset of the constraint solver
EXTERN_C DLL_EXPORT void ResetConstraintSolver(BulletSim* sim)
{
	sim->getDynamicsWorld()->getConstraintSolver()->reset();
}

/**
 * Steps the simulation forward a given amount of time and retrieves any physics updates.
 * @param worldID ID of the world to step.
 * @param timeStep Length of time in seconds to move the simulation forward.
 * @param maxSubSteps Clamps the maximum number of fixed duration sub steps taken this step.
 * @param fixedTimeStep Length in seconds of the sub steps Bullet actually uses for simulation. Example: 1.0 / TARGET_FPS.
 * @param updatedEntityCount Pointer to the number of EntityProperties generated this call.
 * @param updatedEntities Pointer to an array of pointers to EntityProperties containing physics updates generated this call.
 * @param collidersCount Pointer to the number of colliders detected this call.
 * @param colliders Pointer to an array of colliding object IDs (in pairs of two).
 * @return Number of sub steps that were taken this call.
 */
EXTERN_C DLL_EXPORT int PhysicsStep2(BulletSim* sim, float timeStep, int maxSubSteps, float fixedTimeStep, 
										int* updatedEntityCount, int* collidersCount)
{
	return sim->PhysicsStep2(timeStep, maxSubSteps, fixedTimeStep, updatedEntityCount, collidersCount);
}

// Cause a position update to happen next physics step.
// This works by placing an entry for this object in the SimMotionState's
//    update event array.
EXTERN_C DLL_EXPORT bool PushUpdate2(btCollisionObject* obj)
{
	bsDebug_AssertIsKnownCollisionObject(obj, "PushUpdate2: not a known body");
	bool ret = false;
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb != NULL)
	{
		SimMotionState* sms = (SimMotionState*)rb->getMotionState();
		if (sms != NULL)
		{
			btTransform wt;
			sms->getWorldTransform(wt);
			sms->setWorldTransform(wt, true);
			ret = true;
		}
	}
	return ret;
}

// =====================================================================
// Mesh, hull, shape and body creation helper routines
EXTERN_C DLL_EXPORT btCollisionShape* CreateMeshShape2(BulletSim* sim, 
						int indicesCount, int* indices, int verticesCount, float* vertices )
{
	btCollisionShape* shape = sim->CreateMeshShape2(indicesCount, indices, verticesCount, vertices);
	bsDebug_RememberCollisionShape(shape);
	return shape;
}

EXTERN_C DLL_EXPORT btCollisionShape* CreateGImpactShape2(BulletSim* sim, 
						int indicesCount, int* indices, int verticesCount, float* vertices )
{
	btCollisionShape* shape = sim->CreateGImpactShape2(indicesCount, indices, verticesCount, vertices);
	bsDebug_RememberCollisionShape(shape);
	return shape;
}

EXTERN_C DLL_EXPORT btCollisionShape* CreateHullShape2(BulletSim* sim, 
						int hullCount, float* hulls )
{
	btCollisionShape* shape = sim->CreateHullShape2(hullCount, hulls);
	bsDebug_RememberCollisionShape(shape);
	return shape;
}

EXTERN_C DLL_EXPORT btCollisionShape* BuildHullShapeFromMesh2(BulletSim* sim, btCollisionShape* mesh, HACDParams* parms) {
	btCollisionShape* shape;
	bsDebug_AssertIsKnownCollisionShape(mesh, "BuildHullShapeFromMesh2: unknown shape passed for conversion");
	if (parms->whichHACD)
		shape = sim->BuildVHACDHullShapeFromMesh2(mesh, parms);
	else
		shape = sim->BuildHullShapeFromMesh2(mesh, parms);
	bsDebug_RememberCollisionShape(shape);
	return shape;
}

EXTERN_C DLL_EXPORT btCollisionShape* BuildConvexHullShapeFromMesh2(BulletSim* sim, btCollisionShape* mesh) {
	bsDebug_AssertIsKnownCollisionShape(mesh, "BuildConvexHullShapeFromMesh2: unknown shape passed for conversion");
	btCollisionShape* shape = sim->BuildConvexHullShapeFromMesh2(mesh);
	bsDebug_RememberCollisionShape(shape);
	return shape;
}

EXTERN_C DLL_EXPORT btCollisionShape* CreateConvexHullShape2(BulletSim* sim, 
						int indicesCount, int* indices, int verticesCount, float* vertices )
{
	btCollisionShape* shape = sim->CreateConvexHullShape2(indicesCount, indices, verticesCount, vertices);
	bsDebug_RememberCollisionShape(shape);
	return shape;
}

EXTERN_C DLL_EXPORT btCollisionShape* CreateCompoundShape2(BulletSim* sim, bool enableDynamicAabbTree)
{
	btCompoundShape* cShape = new btCompoundShape(enableDynamicAabbTree);
	bsDebug_RememberCollisionShape(cShape);
	return cShape;
}

EXTERN_C DLL_EXPORT int GetNumberOfCompoundChildren2(btCompoundShape* cShape)
{
	return cShape->getNumChildShapes();
}

EXTERN_C DLL_EXPORT void AddChildShapeToCompoundShape2(btCompoundShape* cShape, 
				btCollisionShape* addShape, Vector3 relativePosition, Quaternion relativeRotation)
{
	btTransform relativeTransform(relativeRotation.GetBtQuaternion(), relativePosition.GetBtVector3());

	cShape->addChildShape(relativeTransform, addShape);
}

EXTERN_C DLL_EXPORT btCollisionShape* GetChildShapeFromCompoundShapeIndex2(btCompoundShape* cShape, int ii)
{
	return cShape->getChildShape(ii);
}

EXTERN_C DLL_EXPORT void RemoveChildShapeFromCompoundShape2(btCompoundShape* cShape, btCollisionShape* removeShape)
{
	cShape->removeChildShape(removeShape);
}

EXTERN_C DLL_EXPORT btCollisionShape* RemoveChildShapeFromCompoundShapeIndex2(btCompoundShape* cShape, int ii)
{
	btCollisionShape* ret = cShape->getChildShape(ii);
	cShape->removeChildShapeByIndex(ii);
	return ret;
}

EXTERN_C DLL_EXPORT void RecalculateCompoundShapeLocalAabb2(btCompoundShape* cShape)
{
	cShape->recalculateLocalAabb();
}

EXTERN_C DLL_EXPORT void UpdateChildTransform2(btCompoundShape* cShape, int childIndex, Vector3 pos, Quaternion rot, bool shouldRecalculateLocalAabb)
{
	btTransform newTrans(rot.GetBtQuaternion(), pos.GetBtVector3());
	cShape->updateChildTransform(childIndex, newTrans, shouldRecalculateLocalAabb);
}

EXTERN_C DLL_EXPORT Vector3 GetCompoundChildPosition2(btCompoundShape* cShape, int childIndex)
{
	btTransform childTrans = cShape->getChildTransform(childIndex);
	return childTrans.getOrigin();
}

EXTERN_C DLL_EXPORT Quaternion GetCompoundChildOrientation2(btCompoundShape* cShape, int childIndex)
{
	btTransform childTrans = cShape->getChildTransform(childIndex);
	return childTrans.getRotation();
}


EXTERN_C DLL_EXPORT btCollisionShape* BuildNativeShape2(BulletSim* sim, ShapeData shapeData)
{
	btCollisionShape* shape = NULL;
	switch ((int)shapeData.Type)
	{
		case ShapeData::SHAPE_BOX:
			// btBoxShape subtracts the collision margin from the half extents, so no 
			// fiddling with scale necessary
			// boxes are defined by their half extents
			shape = new btBoxShape(btVector3(0.5, 0.5, 0.5));	// this is really a unit box
			break;
		case ShapeData::SHAPE_CONE:	// TODO:
			shape = new btConeShapeZ(0.5, 1.0);
			break;
		case ShapeData::SHAPE_CYLINDER:	// TODO:
			shape = new btCylinderShapeZ(btVector3(0.5f, 0.5f, 0.5f));
			break;
		case ShapeData::SHAPE_SPHERE:
			shape = new btSphereShape(0.5);		// this is really a unit sphere
			break;
	}
	if (shape != NULL)
	{
		shape->setMargin(btScalar(sim->getWorldData()->params->collisionMargin));
		shape->setLocalScaling(shapeData.Scale.GetBtVector3());
		bsDebug_RememberCollisionShape(shape);
	}

	return shape;
}

// Return 'true' if this shape is a Bullet implemented native shape
EXTERN_C DLL_EXPORT bool IsNativeShape2(btCollisionShape* shape)
{
	bool ret = false;
	bsDebug_AssertIsKnownCollisionShape(shape, "IsNativeShape2: not known shape");
	switch (shape->getShapeType())
	{
		case BOX_SHAPE_PROXYTYPE:
		case CONE_SHAPE_PROXYTYPE:
		case SPHERE_SHAPE_PROXYTYPE:
		case CYLINDER_SHAPE_PROXYTYPE:
			ret = true;
			break;
		default:
			ret = false;
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT void SetShapeCollisionMargin(btCollisionShape* shape, float margin)
{
	bsDebug_AssertIsKnownCollisionShape(obj, "SetShapeCollisonMargin: unknown collisionShape");
	shape->setMargin(btScalar(margin));
}

EXTERN_C DLL_EXPORT btCollisionShape* BuildCapsuleShape2(BulletSim* sim, float radius, float height, Vector3 scale)
{
	btCollisionShape* shape = new btCapsuleShapeZ(btScalar(radius), btScalar(height));
	if (shape)
	{
		shape->setMargin(sim->getWorldData()->params->collisionMargin);
		shape->setLocalScaling(scale.GetBtVector3());
		bsDebug_RememberCollisionShape(shape);
	}
	return shape;
}

// Note: this does not do a deep deletion.
EXTERN_C DLL_EXPORT bool DeleteCollisionShape2(BulletSim* sim, btCollisionShape* shape)
{
	bsDebug_AssertIsKnownCollisionShape(shape, "DeleteCollisionShape2: not known shape");
	bsDebug_ForgetCollisionShape(shape);
	delete shape;
	return true;
}

EXTERN_C DLL_EXPORT btCollisionShape* DuplicateCollisionShape2(BulletSim* sim, btCollisionShape* src, unsigned int id)
{
	btCollisionShape* newShape = NULL;
	bsDebug_AssertIsKnownCollisionShape(shape, "DuplicateCollisionShape2: not known shape");

	int shapeType = src->getShapeType();
	switch (shapeType)
	{
		case TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			btBvhTriangleMeshShape* srcTriShape = (btBvhTriangleMeshShape*)src;
			newShape = new btBvhTriangleMeshShape(srcTriShape->getMeshInterface(), true, true);
			break;
		}
		/*
		case SCALED_TRIANGLE_MESH_SHAPE_PROXYTYPE:
		{
			btScaledBvhTriangleMeshShape* srcTriShape = (btScaledBvhTriangleMeshShape*)src;
			newShape = new btScaledBvhTriangleMeshShape(srcTriShape, src->getLocalScaling());
			break;
		}
		*/
		case COMPOUND_SHAPE_PROXYTYPE:
		{
			btCompoundShape* srcCompShape = (btCompoundShape*)src;

			btCompoundShape* newCompoundShape = new btCompoundShape(false);
			int childCount = srcCompShape->getNumChildShapes();
			btCompoundShapeChild* children = srcCompShape->getChildList();

			for (int i = 0; i < childCount; i++)
			{
				btCollisionShape* childShape = children[i].m_childShape;
				btTransform childTransform = children[i].m_transform;

				newCompoundShape->addChildShape(childTransform, childShape);
			}
			newShape = newCompoundShape;
			break;
		}
		default:
			break;
	}
	if (newShape != NULL)
	{
		newShape->setUserPointer(PACKLOCALID(id));
		bsDebug_RememberCollisionShape(newShape);
	}
	return newShape;
}

// Returns a btCollisionObject::CollisionObjectTypes
EXTERN_C DLL_EXPORT int GetBodyType2(btCollisionObject* obj)
{
	bsDebug_AssertIsKnownCollisionObject(obj, "GetBodyType2: not known collisionObject");
	return obj->getInternalType();
}

// ========================================================================
// Create aa btRigidBody with our MotionState structure so we can track updates to this body.
EXTERN_C DLL_EXPORT btCollisionObject* CreateBodyFromShape2(BulletSim* sim, btCollisionShape* shape, 
						IDTYPE id, Vector3 pos, Quaternion rot)
{
	bsDebug_AssertIsKnownCollisionShape(shape, "CreateBodyFromShape2: unknown collision shape");
	btTransform bodyTransform(rot.GetBtQuaternion(), pos.GetBtVector3());

	// Use the BulletSim motion state so motion updates will be sent up
	SimMotionState* motionState = new SimMotionState(id, bodyTransform, &(sim->getWorldData()->updatesThisFrame));
	btRigidBody::btRigidBodyConstructionInfo cInfo(0.0, motionState, shape);
	btRigidBody* body = new btRigidBody(cInfo);
	motionState->RigidBody = body;

	body->setUserPointer(PACKLOCALID(id));
	bsDebug_RememberCollisionObject(obj);

	return body;
}

// Create a btRigidBody with the default MotionState. We will not get any movement updates from this body.
EXTERN_C DLL_EXPORT btCollisionObject* CreateBodyWithDefaultMotionState2(btCollisionShape* shape, 
						IDTYPE id, Vector3 pos, Quaternion rot)
{
	bsDebug_AssertIsKnownCollisionShape(shape, "CreateBodyWithDefaultMotionState2: unknown collision shape");
	btTransform heightfieldTr(rot.GetBtQuaternion(), pos.GetBtVector3());

	// Use the default motion state since we are not interested in these
	//   objects reporting collisions. Other objects will report their
	//   collisions with the terrain.
	btDefaultMotionState* motionState = new btDefaultMotionState(heightfieldTr);
	btRigidBody::btRigidBodyConstructionInfo cInfo(0.0, motionState, shape);
	btRigidBody* body = new btRigidBody(cInfo);

	body->setUserPointer(PACKLOCALID(id));
	bsDebug_RememberCollisionObject(body);

	return body;
}

// Create a btGhostObject with the passed shape
EXTERN_C DLL_EXPORT btCollisionObject* CreateGhostFromShape2(BulletSim* sim, btCollisionShape* shape, 
						IDTYPE id, Vector3 pos, Quaternion rot)
{
	bsDebug_AssertIsKnownCollisionShape(shape, "CreateGhostFromShape2: unknown collision shape");
	btTransform bodyTransform(rot.GetBtQuaternion(), pos.GetBtVector3());

	btGhostObject* gObj = new btPairCachingGhostObject();
	gObj->setWorldTransform(bodyTransform);
	gObj->setCollisionShape(shape);

	gObj->setUserPointer(PACKLOCALID(id));
	bsDebug_RememberCollisionObject(gObj);

	sim->getWorldData()->specialCollisionObjects[id] = gObj;
	
	return gObj;
}

/*
// Create a RigidBody from passed shape and construction info.
// NOTE: it is presumed that a previous RigidBody was saved into the construction info
//     and that, in particular, the motionState is a SimMotionState from the saved RigidBody.
//     This WILL NOT WORK for terrain bodies.
// This does not restore collisionFlags.
EXTERN_C DLL_EXPORT btCollisionObject* CreateBodyFromShapeAndInfo2(BulletSim* sim, btCollisionShape* shape, 
						IDTYPE id, btRigidBody::btRigidBodyConstructionInfo* consInfo)
{
	bsDebug_AssertIsKnownCollisionShape(shape, "CreateBodyFromShapeAndInfo2: unknown collision shape");
	consInfo->m_collisionShape = shape;
	btRigidBody* body = new btRigidBody(*consInfo);

	// The saved motion state was the SimMotionState saved from before.
	((SimMotionState*)consInfo->m_motionState)->RigidBody = body;
	body->setUserPointer(PACKLOCALID(id));
	bsDebug_RememberCollisionObject(body);

	return body;
}

// Build a RigidBody construction info from an existing RigidBody.
// Can be used later to recreate the rigid body.
EXTERN_C DLL_EXPORT btRigidBody::btRigidBodyConstructionInfo* AllocateBodyInfo2(btCollisionObject* obj)
{

	btRigidBody::btRigidBodyConstructionInfo* consInfo = NULL;

	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb)
	{
		consInfo = new btRigidBody::btRigidBodyConstructionInfo(
			1.0 / rb->getInvMass(), rb->getMotionState(), rb->getCollisionShape() );
		consInfo->m_localInertia = btVector3(0.0, 0.0, 0.0);
		consInfo->m_linearDamping = rb->getLinearDamping();
		consInfo->m_angularDamping = rb->getAngularDamping();
		consInfo->m_friction = rb->getFriction();
		consInfo->m_restitution = rb->getRestitution();
		consInfo->m_linearSleepingThreshold = rb->getLinearSleepingThreshold();
		consInfo->m_angularSleepingThreshold = rb->getAngularSleepingThreshold();
		// The following are unaccessable but they are usually the default anyway.
		// If these are used, they must be later set on the restored RigidBody.
		// consInfo->m_additionalDamping = rb->m_additionalDamping;
		// consInfo->m_additionalDampingFactor = rb->m_additionalDampingFactor;
		// consInfo->m_additionalLinearDampingThresholdSqr = rb->m_additionalLinearDampingThresholdSqr;
		// consInfo->m_additionalAngularDampingThresholdSqr = rb->m_additionalAngularDampingThresholdSqr;
		// consInfo->m_additionalAngularDampingFactor = rb->m_additionalAngularDampingFactor;
	}
	return consInfo;
}

// Release the btRigidBodyConstructionInfo created in the above routine.
EXTERN_C DLL_EXPORT void ReleaseBodyInfo2(btRigidBody::btRigidBodyConstructionInfo* consInfo)
{
	delete consInfo;
	return;
}
*/

/**
 * Free all memory allocated to an object. The caller must have previously removed
 * the object from the dynamicsWorld.
 * @param worldID ID of the world to modify.
 * @param id Object ID.
 * @return True on success, false if the object was not found.
 */
EXTERN_C DLL_EXPORT void DestroyObject2(BulletSim* sim, btCollisionObject* obj)
{

	bsDebug_AssertIsKnownCollisionObject(obj, "DestroyObject2: unknown collisionObject");

	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb)
	{
		// If we added a motionState to the object, delete that
		btMotionState* motionState = rb->getMotionState();
		if (motionState)
			delete motionState;
	}
	
	// Delete the rest of the memory allocated to this object
	btCollisionShape* shape = obj->getCollisionShape();
	if (shape) 
	{
		bsDebug_AssertIsKnownCollisionShape(shape, "DestroyObject2: unknown collisionShape");
		bsDebug_ForgetCollisionShape(shape);
		delete shape;
	}

	// Remove from special collision objects. A NOOP if not in the list.
	IDTYPE id = CONVLOCALID(obj->getUserPointer());
	sim->getWorldData()->specialCollisionObjects.erase(id);

	// finally make the object itself go away
	bsDebug_ForgetCollisionObject(obj);
	delete obj;
}

// =====================================================================
// Terrain creation and helper routines

EXTERN_C DLL_EXPORT btCollisionShape* CreateTerrainShape2(IDTYPE id, Vector3 size, float minHeight, float maxHeight, float* heightMap, 
								float scaleFactor, float collisionMargin)
{
	const int upAxis = 2;
	btHeightfieldTerrainShape* terrainShape = new btHeightfieldTerrainShape(
										(int)size.X, (int)size.Y, heightMap, (btScalar)scaleFactor, 
										(btScalar)minHeight, (btScalar)maxHeight, upAxis, PHY_FLOAT, false);

	terrainShape->setMargin(btScalar(collisionMargin));
	terrainShape->setUseDiamondSubdivision(true);

	// Add the localID to the object so we know about collisions
	terrainShape->setUserPointer(PACKLOCALID(id));
	bsDebug_RememberCollisionShape(terrainShape);

	return terrainShape;
}

EXTERN_C DLL_EXPORT btCollisionShape* CreateGroundPlaneShape2(
	IDTYPE id,
	float height,	// usually 1
	float collisionMargin)
{
	// Initialize the ground plane
	btVector3 groundPlaneNormal = btVector3(0, 0, 1);	// Z up
	btStaticPlaneShape* m_planeShape = new btStaticPlaneShape(groundPlaneNormal, (btScalar)height);
	m_planeShape->setMargin(collisionMargin);

	m_planeShape->setUserPointer(PACKLOCALID(id));
	bsDebug_RememberCollisionShape(m_planeShape);

	return m_planeShape;
}

// =====================================================================
// Constraint creation and helper routines
/**
 * Add a generic 6 degree of freedom constraint between two previously created objects
 * @param sim pointer to BulletSim instance this creation is to be in
 * @param id1 first object to constrain
 * @param id2 other object to constrain
 * @param lowLinear low bounds of linear constraint
 * @param hiLinear hi bounds of linear constraint
 * @param lowAngular low bounds of angular constraint
 * @param hiAngular hi bounds of angular constraint
 * @param 'true' if to use FrameA as reference for the constraint action
 * @param 'true' if disable collisions between the constrained objects
 */
EXTERN_C DLL_EXPORT btTypedConstraint* Create6DofConstraint2(BulletSim* sim, 
				btCollisionObject* obj1, btCollisionObject* obj2,
				Vector3 frame1loc, Quaternion frame1rot,
				Vector3 frame2loc, Quaternion frame2rot,
				bool useLinearReferenceFrameA, bool disableCollisionsBetweenLinkedBodies)
{
	bsDebug_AssertIsKnownCollisionObject(obj1, "Create6DofConstraint2: obj1 unknown CollisionObject");
	bsDebug_AssertIsKnownCollisionObject(obj2, "Create6DofConstraint2: obj2 unknown CollisionObject");
	bsDebug_AssertNoExistingConstraint(obj1, obj2, "Create6DofConstraint2: constraint exists");

	btRigidBody* rb1 = btRigidBody::upcast(obj1);
	btRigidBody* rb2 = btRigidBody::upcast(obj2);

	btGeneric6DofConstraint* constrain = NULL;
	if (rb1 != NULL && rb2 != NULL)
	{
		btTransform frame1t(frame1rot.GetBtQuaternion(), frame1loc.GetBtVector3());
		btTransform frame2t(frame2rot.GetBtQuaternion(), frame2loc.GetBtVector3());

		constrain = new btGeneric6DofConstraint(*rb1, *rb2, frame1t, frame2t, useLinearReferenceFrameA);

		constrain->calculateTransforms();
		sim->getDynamicsWorld()->addConstraint(constrain, disableCollisionsBetweenLinkedBodies);

		bsDebug_RememberConstraint(constrain);
		// sim->getWorldData()->BSLog("CreateConstraint2: loc=%x, body1=%u, body2=%u", constrain,
		// 					CONVLOCALID(obj1->getCollisionShape()->getUserPointer()),
		// 					CONVLOCALID(obj2->getCollisionShape()->getUserPointer()));
		// sim->getWorldData()->BSLog("          f1=<%f,%f,%f>, f1r=<%f,%f,%f,%f>, f2=<%f,%f,%f>, f2r=<%f,%f,%f,%f>",
		// 					frame1loc.X, frame1loc.Y, frame1loc.Z, frame1rot.X, frame1rot.Y, frame1rot.Z, frame1rot.W,
		// 					frame2loc.X, frame2loc.Y, frame2loc.Z, frame2rot.X, frame2rot.Y, frame2rot.Z, frame2rot.W);
	}
	return constrain;
}

// Create a 6Dof constraint between two objects and around the given world point.
EXTERN_C DLL_EXPORT btTypedConstraint* Create6DofConstraintToPoint2(BulletSim* sim, 
				btCollisionObject* obj1, btCollisionObject* obj2,
				Vector3 joinPoint,
				bool useLinearReferenceFrameA, bool disableCollisionsBetweenLinkedBodies)
{
	bsDebug_AssertIsKnownCollisionObject(obj1, "Create6DofConstraint2: obj1 unknown CollisionObject");
	bsDebug_AssertIsKnownCollisionObject(obj2, "Create6DofConstraint2: obj2 unknown CollisionObject");
	bsDebug_AssertNoExistingConstraint(obj1, obj2, "Create6DofConstraint2: constraint exists");
	btGeneric6DofConstraint* constrain = NULL;

	btRigidBody* rb1 = btRigidBody::upcast(obj1);
	btRigidBody* rb2 = btRigidBody::upcast(obj2);

	if (rb1 != NULL && rb2 != NULL)
	{
		// following example at http://bulletphysics.org/Bullet/phpBB3/viewtopic.php?t=5805
		btTransform joinPointt, frame1t, frame2t;
		joinPointt.setIdentity();
		joinPointt.setOrigin(joinPoint.GetBtVector3());
		frame1t = rb1->getWorldTransform().inverse() * joinPointt;
		frame2t = rb2->getWorldTransform().inverse() * joinPointt;

		constrain = new btGeneric6DofConstraint(*rb1, *rb2, frame1t, frame2t, useLinearReferenceFrameA);

		sim->getDynamicsWorld()->addConstraint(constrain, disableCollisionsBetweenLinkedBodies);

		bsDebug_RememberConstraint(constrain);
	}

	return constrain;
}

// Create a 6Dof constraint tied to a temporary, static world object
EXTERN_C DLL_EXPORT btTypedConstraint* Create6DofConstraintFixed2(BulletSim* sim, 
				btCollisionObject* obj1, Vector3 frameInBloc, Quaternion frameInBrot,
				bool useLinearReferenceFrameB, bool disableCollisionsBetweenLinkedBodies)
{
	bsDebug_AssertIsKnownCollisionObject(obj1, "Create6DofConstraintFixed2: obj1 unknown CollisionObject");

	btGeneric6DofConstraint* constrain = NULL;

	btRigidBody* rb1 = btRigidBody::upcast(obj1);

	if (rb1 != NULL)
	{
		btTransform frameInB(frameInBrot.GetBtQuaternion(), frameInBloc.GetBtVector3());

		constrain = new btGeneric6DofConstraint(*rb1, frameInB, useLinearReferenceFrameB);

		sim->getDynamicsWorld()->addConstraint(constrain, disableCollisionsBetweenLinkedBodies);

		bsDebug_RememberConstraint(constrain);
	}

	return constrain;
}

EXTERN_C DLL_EXPORT btTypedConstraint* Create6DofSpringConstraint2(BulletSim* sim, 
				btCollisionObject* obj1, btCollisionObject* obj2,
				Vector3 frame1loc, Quaternion frame1rot,
				Vector3 frame2loc, Quaternion frame2rot,
				bool useLinearReferenceFrameA, bool disableCollisionsBetweenLinkedBodies)
{
	bsDebug_AssertIsKnownCollisionObject(obj1, "Create6DofSpringConstraint2: obj1 unknown CollisionObject");
	bsDebug_AssertIsKnownCollisionObject(obj2, "Create6DofSpringConstraint2: obj2 unknown CollisionObject");
	bsDebug_AssertNoExistingConstraint(obj1, obj2, "Create6DofSpringConstraint2: constraint exists");

	btRigidBody* rb1 = btRigidBody::upcast(obj1);
	btRigidBody* rb2 = btRigidBody::upcast(obj2);

	btGeneric6DofSpringConstraint* constrain = NULL;
	if (rb1 != NULL && rb2 != NULL)
	{
		btTransform frame1t(frame1rot.GetBtQuaternion(), frame1loc.GetBtVector3());
		btTransform frame2t(frame2rot.GetBtQuaternion(), frame2loc.GetBtVector3());

		constrain = new btGeneric6DofSpringConstraint(*rb1, *rb2, frame1t, frame2t, useLinearReferenceFrameA);

		sim->getWorldData()->BSLog("Create6DofSpringConstraint2 ++++++++++++");
		DumpConstraint2(sim, constrain);

		constrain->calculateTransforms();
		sim->getDynamicsWorld()->addConstraint(constrain, disableCollisionsBetweenLinkedBodies);

		bsDebug_RememberConstraint(constrain);
	}
	return constrain;
}


EXTERN_C DLL_EXPORT btTypedConstraint* CreateHingeConstraint2(BulletSim* sim,
						btCollisionObject* obj1, btCollisionObject* obj2,
						Vector3 pivotInA, Vector3 pivotInB,
						Vector3 axisInA, Vector3 axisInB,
						bool useReferenceFrameA,
						bool disableCollisionsBetweenLinkedBodies
						)
{
	bsDebug_AssertIsKnownCollisionObject(obj1, "CreateHingeConstraint2: obj1 unknown CollisionObject");
	bsDebug_AssertIsKnownCollisionObject(obj2, "CreateHingeConstraint2: obj2 unknown CollisionObject");
	bsDebug_AssertNoExistingConstraint(obj1, obj2, "CreateHingeConstraint2: constraint exists");
	btHingeConstraint* constrain = NULL;

	btRigidBody* rb1 = btRigidBody::upcast(obj1);
	btRigidBody* rb2 = btRigidBody::upcast(obj2);

	if (rb1 != NULL && rb2 != NULL)
	{
		constrain = new btHingeConstraint(*rb1, *rb2, 
									pivotInA.GetBtVector3(), pivotInB.GetBtVector3(), 
									axisInA.GetBtVector3(), axisInB.GetBtVector3(), 
									useReferenceFrameA);

		sim->getDynamicsWorld()->addConstraint(constrain, disableCollisionsBetweenLinkedBodies);
		bsDebug_RememberConstraint(constrain);
	}

	return constrain;
}

EXTERN_C DLL_EXPORT btTypedConstraint* CreateSliderConstraint2(BulletSim* sim, 
				btCollisionObject* obj1, btCollisionObject* obj2,
				Vector3 frame1loc, Quaternion frame1rot,
				Vector3 frame2loc, Quaternion frame2rot,
				bool useLinearReferenceFrameA, bool disableCollisionsBetweenLinkedBodies)
{
	bsDebug_AssertIsKnownCollisionObject(obj1, "CreateSliderConstraint2: obj1 unknown CollisionObject");
	bsDebug_AssertIsKnownCollisionObject(obj2, "CreateSliderConstraint2: obj2 unknown CollisionObject");
	bsDebug_AssertNoExistingConstraint(obj1, obj2, "CreateSliderConstraint2: constraint exists");

	btRigidBody* rb1 = btRigidBody::upcast(obj1);
	btRigidBody* rb2 = btRigidBody::upcast(obj2);

	btSliderConstraint* constrain = NULL;
	if (rb1 != NULL && rb2 != NULL)
	{
		btTransform frame1t(frame1rot.GetBtQuaternion(), frame1loc.GetBtVector3());
		btTransform frame2t(frame2rot.GetBtQuaternion(), frame2loc.GetBtVector3());

		constrain = new btSliderConstraint(*rb1, *rb2, frame1t, frame2t, useLinearReferenceFrameA);

		sim->getDynamicsWorld()->addConstraint(constrain, disableCollisionsBetweenLinkedBodies);

		bsDebug_RememberConstraint(constrain);
	}
	return constrain;
}

EXTERN_C DLL_EXPORT btTypedConstraint* CreateConeTwistConstraint2(BulletSim* sim, 
				btCollisionObject* obj1, btCollisionObject* obj2,
				Vector3 frame1loc, Quaternion frame1rot,
				Vector3 frame2loc, Quaternion frame2rot,
				bool disableCollisionsBetweenLinkedBodies)
{
	bsDebug_AssertIsKnownCollisionObject(obj1, "CreateConeTwistConstraint2: obj1 unknown CollisionObject");
	bsDebug_AssertIsKnownCollisionObject(obj2, "CreateConeTwistConstraint2: obj2 unknown CollisionObject");
	bsDebug_AssertNoExistingConstraint(obj1, obj2, "CreateConeTwistConstraint2: constraint exists");

	btRigidBody* rb1 = btRigidBody::upcast(obj1);
	btRigidBody* rb2 = btRigidBody::upcast(obj2);

	btConeTwistConstraint* constrain = NULL;
	if (rb1 != NULL && rb2 != NULL)
	{
		btTransform frame1t(frame1rot.GetBtQuaternion(), frame1loc.GetBtVector3());
		btTransform frame2t(frame2rot.GetBtQuaternion(), frame2loc.GetBtVector3());

		constrain = new btConeTwistConstraint(*rb1, *rb2, frame1t, frame2t);

		sim->getDynamicsWorld()->addConstraint(constrain, disableCollisionsBetweenLinkedBodies);

		bsDebug_RememberConstraint(constrain);
	}
	return constrain;
}

EXTERN_C DLL_EXPORT btTypedConstraint* CreateGearConstraint2(BulletSim* sim, 
				btCollisionObject* obj1, btCollisionObject* obj2,
				Vector3 axisInA, Vector3 axisInB,
				Vector3 frame2loc, Quaternion frame2rot,
				float ratio, bool disableCollisionsBetweenLinkedBodies)
{
	bsDebug_AssertIsKnownCollisionObject(obj1, "CreateGearConstraint2: obj1 unknown CollisionObject");
	bsDebug_AssertIsKnownCollisionObject(obj2, "CreateGearConstraint2: obj2 unknown CollisionObject");
	bsDebug_AssertNoExistingConstraint(obj1, obj2, "CreateGearConstraint2: constraint exists");

	btRigidBody* rb1 = btRigidBody::upcast(obj1);
	btRigidBody* rb2 = btRigidBody::upcast(obj2);

	btGearConstraint* constrain = NULL;
	if (rb1 != NULL && rb2 != NULL)
	{
		constrain = new btGearConstraint(*rb1, *rb2, axisInA.GetBtVector3(), axisInB.GetBtVector3(), ratio);

		sim->getDynamicsWorld()->addConstraint(constrain, disableCollisionsBetweenLinkedBodies);

		bsDebug_RememberConstraint(constrain);
	}
	return constrain;
}

EXTERN_C DLL_EXPORT btTypedConstraint* CreatePoint2PointConstraint2(BulletSim* sim, 
				btCollisionObject* obj1, btCollisionObject* obj2,
				Vector3 pivotInA, Vector3 pivotInB,
				bool disableCollisionsBetweenLinkedBodies)
{
	bsDebug_AssertIsKnownCollisionObject(obj1, "CreatePoint2PointConstraint2: obj1 unknown CollisionObject");
	bsDebug_AssertIsKnownCollisionObject(obj2, "CreatePoint2PointConstraint2: obj2 unknown CollisionObject");
	bsDebug_AssertNoExistingConstraint(obj1, obj2, "CreatePoint2PointConstraint2: constraint exists");

	btRigidBody* rb1 = btRigidBody::upcast(obj1);
	btRigidBody* rb2 = btRigidBody::upcast(obj2);

	btPoint2PointConstraint* constrain = NULL;
	if (rb1 != NULL && rb2 != NULL)
	{
		constrain = new btPoint2PointConstraint(*rb1, *rb2, pivotInA.GetBtVector3(), pivotInB.GetBtVector3());

		sim->getDynamicsWorld()->addConstraint(constrain, disableCollisionsBetweenLinkedBodies);

		bsDebug_RememberConstraint(constrain);
	}
	return constrain;
}

EXTERN_C DLL_EXPORT bool SetFrames2(btTypedConstraint* constrain, 
			Vector3 frameA, Quaternion frameArot, Vector3 frameB, Quaternion frameBrot)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "SetFrame2: unknown constraint");

	btTransform transA(frameArot.GetBtQuaternion(), frameA.GetBtVector3());
	btTransform transB(frameBrot.GetBtQuaternion(), frameB.GetBtVector3());
	switch (constrain->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			break;
		case HINGE_CONSTRAINT_TYPE:
		{
			btHingeConstraint* cc = (btHingeConstraint*)constrain;
			cc->setFrames(transA, transB);
			ret = true;
			break;
		}
		case CONETWIST_CONSTRAINT_TYPE:
		{
			btConeTwistConstraint* cc = (btConeTwistConstraint*)constrain;
			cc->setFrames(transA, transB);
			ret = true;
			break;
		}
		case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* cc = (btGeneric6DofConstraint*)constrain;
			cc->setFrames(transA, transB);
			ret = true;
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
		{
			btSliderConstraint* cc = (btSliderConstraint*)constrain;
			cc->setFrames(transA, transB);
			ret = true;
			break;
		}
		case CONTACT_CONSTRAINT_TYPE:
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->setFrames(transA, transB);
			ret = true;
			break;
		}
		case GEAR_CONSTRAINT_TYPE:
			break;
		default:
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT void SetConstraintEnable2(btTypedConstraint* constrain, float trueFalse)
{
	bsDebug_AssertIsKnownConstraint(constrain, "SetConstraintEnable2: unknown constraint");
	constrain->setEnabled(trueFalse == ParamTrue ? true : false);
}

EXTERN_C DLL_EXPORT void SetConstraintNumSolverIterations2(btTypedConstraint* constrain, float iterations)
{
	bsDebug_AssertIsKnownConstraint(constrain, "SetConstraintNumSolverIterations2: unknown constraint");
	constrain->setOverrideNumSolverIterations((int)iterations);
}

EXTERN_C DLL_EXPORT bool SetLinearLimits2(btTypedConstraint* constrain, Vector3 low, Vector3 high)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "SetLinearLimits2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			break;
		case HINGE_CONSTRAINT_TYPE:
			break;
		case CONETWIST_CONSTRAINT_TYPE:
			break;
		case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* cc = (btGeneric6DofConstraint*)constrain;
			cc->setLinearLowerLimit(low.GetBtVector3());
			cc->setLinearUpperLimit(high.GetBtVector3());
			ret = true;
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
			break;
		case CONTACT_CONSTRAINT_TYPE:
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->setLinearLowerLimit(low.GetBtVector3());
			cc->setLinearUpperLimit(high.GetBtVector3());
			ret = true;
			break;
		}
		case GEAR_CONSTRAINT_TYPE:
			break;
		default:
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT bool SetAngularLimits2(btTypedConstraint* constrain, Vector3 low, Vector3 high)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "SetAngularLimits2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			break;
		case HINGE_CONSTRAINT_TYPE:
			break;
		case CONETWIST_CONSTRAINT_TYPE:
			break;
		case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* cc = (btGeneric6DofConstraint*)constrain;
			cc->setAngularLowerLimit(low.GetBtVector3());
			cc->setAngularUpperLimit(high.GetBtVector3());
			ret = true;
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
			break;
		case CONTACT_CONSTRAINT_TYPE:
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->setAngularLowerLimit(low.GetBtVector3());
			cc->setAngularUpperLimit(high.GetBtVector3());
			ret = true;
			break;
		}
		case GEAR_CONSTRAINT_TYPE:
			break;
		default:
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT bool UseFrameOffset2(btTypedConstraint* constrain, float enable)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "UseFrameOffset2: unknown constraint");
	bool onOff = (enable == ParamTrue);
	switch (constrain->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			break;
		case HINGE_CONSTRAINT_TYPE:
		{
			btHingeConstraint* hc = (btHingeConstraint*)constrain;
			hc->setUseFrameOffset(onOff);
			ret = true;
			break;
		}
		case CONETWIST_CONSTRAINT_TYPE:
			break;
		case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* cc = (btGeneric6DofConstraint*)constrain;
			cc->setUseFrameOffset(onOff);
			ret = true;
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
			break;
		case CONTACT_CONSTRAINT_TYPE:
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->setUseFrameOffset(onOff);
			ret = true;
			break;
		}
		case GEAR_CONSTRAINT_TYPE:
			break;
		default:
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT bool TranslationalLimitMotor2(btTypedConstraint* constrain, 
				float enable, float targetVelocity, float maxMotorForce)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "TranslationalLimitMotor2: unknown constraint");
	bool onOff = (enable == ParamTrue);
	switch (constrain->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			break;
		case HINGE_CONSTRAINT_TYPE:
			break;
		case CONETWIST_CONSTRAINT_TYPE:
		{
			btConeTwistConstraint* cc = (btConeTwistConstraint*)constrain;
			cc->enableMotor(onOff);
			cc->setMaxMotorImpulse(maxMotorForce);
			ret = true;
			break;
		}
		case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* cc = (btGeneric6DofConstraint*)constrain;
			cc->getTranslationalLimitMotor()->m_enableMotor[0] = onOff;
			cc->getTranslationalLimitMotor()->m_targetVelocity[0] = targetVelocity;
			cc->getTranslationalLimitMotor()->m_maxMotorForce[0] = maxMotorForce;
			ret = true;
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
			break;
		case CONTACT_CONSTRAINT_TYPE:
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->getTranslationalLimitMotor()->m_enableMotor[0] = onOff;
			cc->getTranslationalLimitMotor()->m_targetVelocity[0] = targetVelocity;
			cc->getTranslationalLimitMotor()->m_maxMotorForce[0] = maxMotorForce;
			ret = true;
			break;
		}
		case GEAR_CONSTRAINT_TYPE:
			break;
		default:
			break;
	}

	return ret;
}

EXTERN_C DLL_EXPORT bool SetBreakingImpulseThreshold2(btTypedConstraint* constrain, float thresh)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "SetBreakingImpulseThreshold2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			break;
		case HINGE_CONSTRAINT_TYPE:
			break;
		case CONETWIST_CONSTRAINT_TYPE:
			break;
		case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* cc = (btGeneric6DofConstraint*)constrain;
			cc->setBreakingImpulseThreshold(btScalar(thresh));
			ret = true;
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
			break;
		case CONTACT_CONSTRAINT_TYPE:
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->setBreakingImpulseThreshold(btScalar(thresh));
			ret = true;
			break;
		}
		case GEAR_CONSTRAINT_TYPE:
			break;
		default:
			break;
	}

	return ret;
}

EXTERN_C DLL_EXPORT bool ConstraintSetAxis2(btTypedConstraint* constrain, Vector3 axisA, Vector3 axisB)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "SetConstraintAxis2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			break;
		case HINGE_CONSTRAINT_TYPE:
		{
			btHingeConstraint* cc = (btHingeConstraint*)constrain;
			btVector3 hingeAxis = axisA.GetBtVector3();
			cc->setAxis(hingeAxis);
			ret = true;
			break;
		}
		case CONETWIST_CONSTRAINT_TYPE:
			break;
		case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* cc = (btGeneric6DofConstraint*)constrain;
			cc->setAxis(axisA.GetBtVector3(), axisB.GetBtVector3());
			ret = true;
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
			break;
		case CONTACT_CONSTRAINT_TYPE:
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->setAxis(axisA.GetBtVector3(), axisB.GetBtVector3());
			ret = true;
			break;
		}
		case GEAR_CONSTRAINT_TYPE:
			break;
		default:
			break;
	}
	return ret;
}

#define HINGE_NOT_SPECIFIED (-1.0)
EXTERN_C DLL_EXPORT bool ConstraintHingeSetLimit2(btTypedConstraint* constrain, float low, float high, float softness, float bias, float relaxation)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "ConstraintHingeSetLimits2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case HINGE_CONSTRAINT_TYPE:
		{
			btHingeConstraint* cc = (btHingeConstraint*)constrain;
			if (softness == HINGE_NOT_SPECIFIED)
				cc->setLimit(low, high);
			else if (bias == HINGE_NOT_SPECIFIED)
				cc->setLimit(low, high, softness);
			else if (relaxation == HINGE_NOT_SPECIFIED)
				cc->setLimit(low, high, softness, bias);
			else
				cc->setLimit(low, high, softness, bias, relaxation);
			ret = true;
			break;
		}
		default:
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT bool ConstraintSpringEnable2(btTypedConstraint* constrain, int index, bool onOff)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "ConstraintSpringEnable2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->enableSpring(index, onOff);
			ret = true;
			break;
		}
		default:
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT bool ConstraintSpringSetEquilibriumPoint2(btTypedConstraint* constrain, int index, float eqPoint)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "ConstraintSpringEnable2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			if (index == CONSTRAINT_NOT_SPECIFIED)
			{
				cc->setEquilibriumPoint();
			}
			else
			{
				if (eqPoint == CONSTRAINT_NOT_SPECIFIEDF)
				{
					cc->setEquilibriumPoint(index);
				}
				else
				{
					cc->setEquilibriumPoint(index, eqPoint);
				}
			}
			ret = true;
			break;
		}
		default:
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT bool ConstraintSpringSetStiffness2(btTypedConstraint* constrain, int index, float stiffness)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "ConstraintSpringSetStiffness2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->setStiffness(index, stiffness);
			ret = true;
			break;
		}
		default:
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT bool ConstraintSpringSetDamping2(btTypedConstraint* constrain, int index, float damping)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "ConstraintSpringSetDamping2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->setDamping(index, damping);
			ret = true;
			break;
		}
		default:
			break;
	}
	return ret;
}

#define SLIDER_LOWER_LIMIT 0
#define SLIDER_UPPER_LIMIT 1
#define SLIDER_LINEAR 2
#define SLIDER_ANGULAR 3
EXTERN_C DLL_EXPORT bool ConstraintSliderSetLimits2(btTypedConstraint* constrain, int upperLower, int linAng, float val)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "ConstraintSlider2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case SLIDER_CONSTRAINT_TYPE:
		{
			btSliderConstraint* cc = (btSliderConstraint*)constrain;
			switch (upperLower)
			{
				case SLIDER_LOWER_LIMIT:
					switch (linAng)
					{
						case SLIDER_LINEAR:
							cc->setLowerLinLimit(val);
							break;
						case SLIDER_ANGULAR:
							cc->setLowerAngLimit(val);
							break;
					}
					break;
				case SLIDER_UPPER_LIMIT:
					switch (linAng)
					{
						case SLIDER_LINEAR:
							cc->setUpperLinLimit(val);
							break;
						case SLIDER_ANGULAR:
							cc->setUpperAngLimit(val);
							break;
					}
					break;
			}
			ret = true;
			break;
		}
		default:
			break;
	}
	return ret;
}

#define SLIDER_SET_SOFTNESS 4
#define SLIDER_SET_RESTITUTION 5
#define SLIDER_SET_DAMPING 6
#define SLIDER_SET_DIRECTION 7
#define SLIDER_SET_LIMIT 8
#define SLIDER_SET_ORTHO 9
EXTERN_C DLL_EXPORT bool ConstraintSliderSet2(btTypedConstraint* constrain, int softRestDamp, int dirLimOrtho, int linAng, float val)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "ConstraintSliderSet2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case SLIDER_CONSTRAINT_TYPE:
		{
			btSliderConstraint* cc = (btSliderConstraint*)constrain;
			switch (softRestDamp)
			{
				case SLIDER_SET_SOFTNESS:
					switch (dirLimOrtho)
					{
						case SLIDER_SET_DIRECTION:
							switch (linAng)
							{
								case SLIDER_LINEAR: cc->setSoftnessDirLin(val); break;
								case SLIDER_ANGULAR: cc->setSoftnessDirAng(val); break;
							}
							break;
						case SLIDER_SET_LIMIT:
							switch (linAng)
							{
								case SLIDER_LINEAR: cc->setSoftnessLimLin(val); break;
								case SLIDER_ANGULAR: cc->setSoftnessLimAng(val); break;
							}
							break;
						case SLIDER_SET_ORTHO:
							switch (linAng)
							{
								case SLIDER_LINEAR: cc->setSoftnessOrthoLin(val); break;
								case SLIDER_ANGULAR: cc->setSoftnessOrthoAng(val); break;
							}
							break;
					}
					break;
				case SLIDER_SET_RESTITUTION:
					switch (dirLimOrtho)
					{
						case SLIDER_SET_DIRECTION:
							switch (linAng)
							{
								case SLIDER_LINEAR: cc->setRestitutionDirLin(val); break;
								case SLIDER_ANGULAR: cc->setRestitutionDirAng(val); break;
							}
							break;
						case SLIDER_SET_LIMIT:
							switch (linAng)
							{
								case SLIDER_LINEAR: cc->setRestitutionLimLin(val); break;
								case SLIDER_ANGULAR: cc->setRestitutionLimAng(val); break;
							}
							break;
						case SLIDER_SET_ORTHO:
							switch (linAng)
							{
								case SLIDER_LINEAR: cc->setRestitutionOrthoLin(val); break;
								case SLIDER_ANGULAR: cc->setRestitutionOrthoAng(val); break;
							}
							break;
					}
					break;
				case SLIDER_SET_DAMPING:
					switch (dirLimOrtho)
					{
						case SLIDER_SET_DIRECTION:
							switch (linAng)
							{
								case SLIDER_LINEAR: cc->setDampingDirLin(val); break;
								case SLIDER_ANGULAR: cc->setDampingDirAng(val); break;
							}
							break;
						case SLIDER_SET_LIMIT:
							switch (linAng)
							{
								case SLIDER_LINEAR: cc->setDampingLimLin(val); break;
								case SLIDER_ANGULAR: cc->setDampingLimAng(val); break;
							}
							break;
						case SLIDER_SET_ORTHO:
							switch (linAng)
							{
								case SLIDER_LINEAR: cc->setDampingOrthoLin(val); break;
								case SLIDER_ANGULAR: cc->setDampingOrthoAng(val); break;
							}
							break;
					}
					break;
			}
			ret = true;
			break;
		}
		default:
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT bool ConstraintSliderMotorEnable2(btTypedConstraint* constrain, int linAng, float numericTrueFalse)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "ConstraintSliderMotorEnable2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case SLIDER_CONSTRAINT_TYPE:
		{
			btSliderConstraint* cc = (btSliderConstraint*)constrain;
			switch (linAng)
			{
				case SLIDER_LINEAR:
					cc->setPoweredLinMotor(numericTrueFalse == 0.0 ? false : true);
					break;
				case SLIDER_ANGULAR:
					cc->setPoweredAngMotor(numericTrueFalse == 0.0 ? false : true);
					break;
			}
			ret = true;
			break;
		}
		default:
			break;
	}
	return ret;
}

#define SLIDER_MOTOR_VELOCITY 10
#define SLIDER_MAX_MOTOR_FORCE 11
EXTERN_C DLL_EXPORT bool ConstraintSliderMotor2(btTypedConstraint* constrain, int forceVel, int linAng, float val)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "ConstraintSlider2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case SLIDER_CONSTRAINT_TYPE:
		{
			btSliderConstraint* cc = (btSliderConstraint*)constrain;
			switch (forceVel)
			{
				case SLIDER_MOTOR_VELOCITY:
					switch (linAng)
					{
						case SLIDER_LINEAR:
							cc->setTargetLinMotorVelocity(val);
							break;
						case SLIDER_ANGULAR:
							cc->setTargetAngMotorVelocity(val);
							break;
					}
					break;
				case SLIDER_MAX_MOTOR_FORCE:
					switch (linAng)
					{
						case SLIDER_LINEAR:
							cc->setMaxLinMotorForce(val);
							break;
						case SLIDER_ANGULAR:
							cc->setMaxAngMotorForce(val);
							break;
					}
					break;
			}
			ret = true;
			break;
		}
		default:
			break;
	}
	return ret;
}

EXTERN_C DLL_EXPORT bool CalculateTransforms2(btTypedConstraint* constrain)
{
	bool ret = false;
	bsDebug_AssertIsKnownConstraint(constrain, "CalculateTransforms2: unknown constraint");
	switch (constrain->getConstraintType())
	{
		case POINT2POINT_CONSTRAINT_TYPE:
			break;
		case HINGE_CONSTRAINT_TYPE:
			break;
		case CONETWIST_CONSTRAINT_TYPE:
			break;
		case D6_CONSTRAINT_TYPE:
		{
			btGeneric6DofConstraint* cc = (btGeneric6DofConstraint*)constrain;
			cc->calculateTransforms();
			ret = true;
			break;
		}
		case SLIDER_CONSTRAINT_TYPE:
		{
			btSliderConstraint* cc = (btSliderConstraint*)constrain;
			cc->calculateTransforms(cc->getCalculatedTransformA(), cc->getCalculatedTransformB());
			ret = true;
			break;
		}
		case CONTACT_CONSTRAINT_TYPE:
			break;
		case D6_SPRING_CONSTRAINT_TYPE:
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			cc->calculateTransforms();
			ret = true;
			break;
		}
		case GEAR_CONSTRAINT_TYPE:
			break;
		default:
			break;
	}
	return ret;
}
/*
Each of the constraint types have many specialized methods specific to their type.
		POINT2POINT_CONSTRAINT_TYPE:
			setPivotA(btVector3)
			setPivotB(btVector3)
			btVector3 getPivotInA()
			btVector3 getPivotInB()
			m_setting.m_tau	= 0.3
			m_setting.m_damping = 1
			m_setting.m_impulseClamp = 0
		HINGE_CONSTRAINT_TYPE:
			btVector3 getAnchor()
			btVector3 getAnchor2()
			btVector3 getAxis1()
			btVector3 getAxis2()
			btVector3 getAngle1()
			btVector3 getAngle2()
			setLowerLimit(btScalar);
			setUpperLimit(btScalar);
		CONETWIST_CONSTRAINT_TYPE:
			rtRigidBody& getRigidBodyA()
			rtRigidBody& getRigidBodyB()
			setAngularOnly(bool)
			setLimit(int index, btScalar value)
			setLimit(btScalar swingSpan1, btScalar swingSpan2, btScalar twistSpan, btScalar softness, btScalar biasFactor, btScalar relaxationFactor)
			int getSolveTwistLimit()
			int getSolveSwingLimit()
			btScalar getSolveTwistLimitSign()
			btScalar getSwingSpan1()
			btScalar getSwingSpan2()
			btScalar getTwistSpan()
			btScalar getTwistAngle()
			bool isPastSwingLimit()
			setDamping(btScalar)
			enableMotor(bool)
			setMaxMotorImpulse(btScalar)
			setMaxMotorImpulseNormalized(btScalar)
			btScalar getFixThresh()
			setFixThresh(btScalar)
			setMotorTarget(btQuaternion)
			setMotorTargetInConstraintSpace(btQuaternion)
			btVector3 GetPointForAngle(btScalar fAngleInRadian, btScalar len)
		D6_CONSTRAINT_TYPE:
			btTansform getFrameOffsetA()
			btTansform getFrameOffsetB()
			btVector3 getAxis(int axisIndex)
			btScalar getAngle(int axisIndex)
			btScalar getRelativePivotPosition(int axisIndex)
			setFrames(btTransform frameA, btTransform frameB)
			getLinearLowerLimit(btVector3&)
			getLinearUpperLimit(btVector3&)
			setAngularLowerLimit(btVector3 angLower)
			getAngularLowerLimit(btVector3 angLower)
			setAngularUpperLimit(btVector3 angLower)
			getAngularUpperLimit(btVector3 angLower)
			getRotationalLimitMorot(int axisIndex)
				btScalar m_loLimit
				btScalar m_hiLimit
				btScalar m_targetVelocity
				btScalar m_maxMotorForce
				btScalar m_maxLimitForce
				btScalar m_damping
				btScalar m_limitSoftness
				btScalar m_normalCFM
				btScalar m_stopERP
				btScalar m_stopCFM
				btScalar m_bounce
				bool m_enableMotor
			getTranslationalLimitMorot(int axisIndex)
				btVector3 m_lowerLimit
				btVector3 m_upperLimit
				btScalar m_limitSoftness
				btScalar m_damping
				btScalar m_restitution
				btVector3 m_normalCFM
				btVector3 m_stopERP
				btVector3 m_stopCFM
				btVector3 m_enableMotor[3]
				btVector3 m_targetVelocity
				btVector3 m_maxMotorForce
			setLimit(int axisIndex, btScalar low, btScalar hi)
			bool getUseFrameOffset()
			setUserFrameOffset(bool)
			setAxis(btVector3 axis1, btVector3 axis2)
		SLIDER_CONSTRAINT_TYPE:
		    btRigidBody& getRigidBodyA()
		    btRigidBody& getRigidBodyB()
		    btTransform& getCalculatedTransformA()
		    btTransform& getCalculatedTransformB()
		    btTransform& getFrameOffsetA()
		    btTransform& getFrameOffsetB()
		    btTransform& getFrameOffsetA()
		    btTransform& getFrameOffsetB()
		    btScalar getLowerLinLimit()
		    void setLowerLinLimit(btScalar lowerLimit)
		    btScalar getUpperLinLimit()
		    void setUpperLinLimit(btScalar upperLimit)
		    btScalar getLowerAngLimit()
		    void setLowerAngLimit(btScalar lowerLimit)
		    btScalar getUpperAngLimit()
		    void setUpperAngLimit(btScalar upperLimit)
			bool getUseLinearReferenceFrameA()
			btScalar getSoftnessDirLin()
			btScalar getRestitutionDirLin()
			btScalar getDampingDirLin()
			btScalar getSoftnessDirAng()
			btScalar getRestitutionDirAng()
			btScalar getDampingDirAng()
			btScalar getSoftnessLimLin()
			btScalar getRestitutionLimLin()
			btScalar getDampingLimLin()
			btScalar getSoftnessLimAng()
			btScalar getRestitutionLimAng()
			btScalar getDampingLimAng()
			btScalar getSoftnessOrthoLin()
			btScalar getRestitutionOrthoLin()
			btScalar getDampingOrthoLin()
			btScalar getSoftnessOrthoAng()
			btScalar getRestitutionOrthoAng()
			btScalar getDampingOrthoAng()
			setSoftnessDirLin(btScalar softnessDirLin)
			setRestitutionDirLin(btScalar restitutionDirLin)
			setDampingDirLin(btScalar dampingDirLin)
			setSoftnessDirAng(btScalar softnessDirAng)
			setRestitutionDirAng(btScalar restitutionDirAng)
			setDampingDirAng(btScalar dampingDirAng)
			setSoftnessLimLin(btScalar softnessLimLin)
			setRestitutionLimLin(btScalar restitutionLimLin)
			setDampingLimLin(btScalar dampingLimLin)
			setSoftnessLimAng(btScalar softnessLimAng)
			setRestitutionLimAng(btScalar restitutionLimAng)
			setDampingLimAng(btScalar dampingLimAng)
			setSoftnessOrthoLin(btScalar softnessOrthoLin)
			setRestitutionOrthoLin(btScalar restitutionOrthoLin)
			setDampingOrthoLin(btScalar dampingOrthoLin)
			setSoftnessOrthoAng(btScalar softnessOrthoAng)
			setRestitutionOrthoAng(btScalar restitutionOrthoAng)
			setDampingOrthoAng(btScalar dampingOrthoAng)
			setPoweredLinMotor(bool onOff)
			bool getPoweredLinMotor()
			void setTargetLinMotorVelocity(btScalar targetLinMotorVelocity)
			btScalar getTargetLinMotorVelocity()
			void setMaxLinMotorForce(btScalar maxLinMotorForce)
			btScalar getMaxLinMotorForce()
			void setPoweredAngMotor(bool onOff)
			bool getPoweredAngMotor()
			void setTargetAngMotorVelocity(btScalar targetAngMotorVelocity)
			btScalar getTargetAngMotorVelocity()
			void setMaxAngMotorForce(btScalar maxAngMotorForce)
			btScalar getMaxAngMotorForce()
			btScalar getLinearPos()
			btScalar getAngularPos()
			setFrames(btTransform frameA, btTransform frameB)
		CONTACT_CONSTRAINT_TYPE:
		D6_SPRING_CONSTRAINT_TYPE:
			everything in D6_CONSTRAINT plus
			enableSpring(int index, bool onOff)
			setStiffness(int index, btScalar stiffness)
			setDamping(int index, btScalar damping)
			setEquilibriumPoint();
			setEquilibriumPoint(int index);
			setEquilibriumPoint(int index, btScalar val);
			setAxis(btVector3& axis1, btVector3& axis2)
		GEAR_CONSTRAINT_TYPE:
*/

EXTERN_C DLL_EXPORT bool SetConstraintParam2(btTypedConstraint* constrain, int paramIndex, float value, int axis)
{
	bsDebug_AssertIsKnownConstraint(constrain, "SetConstraintParam2: unknown constraint");
	if (axis == COLLISION_AXIS_LINEAR_ALL || axis == COLLISION_AXIS_ALL)
	{
		constrain->setParam(paramIndex, btScalar(value), 0);
		constrain->setParam(paramIndex, btScalar(value), 1);
		constrain->setParam(paramIndex, btScalar(value), 2);
	}
	if (axis == COLLISION_AXIS_ANGULAR_ALL || axis == COLLISION_AXIS_ALL)
	{
		constrain->setParam(paramIndex, btScalar(value), 3);
		constrain->setParam(paramIndex, btScalar(value), 4);
		constrain->setParam(paramIndex, btScalar(value), 5);
	}
	if (axis < COLLISION_AXIS_LINEAR_ALL)
	{
		constrain->setParam(paramIndex, btScalar(value), axis);
	}
	return true;
}

EXTERN_C DLL_EXPORT bool DestroyConstraint2(BulletSim* sim, btTypedConstraint* constrain)
{
	bsDebug_AssertIsKnownConstraint(constrain, "DestroyConstraint2: unknown constraint");
	sim->getDynamicsWorld()->removeConstraint(constrain);
	bsDebug_ForgetConstraint(constrain);
	delete constrain;
	return true;
}

// =====================================================================
// btCollisionWorld entries
EXTERN_C DLL_EXPORT void UpdateSingleAabb2(BulletSim* world, btCollisionObject* obj)
{
	bsDebug_AssertIsKnownCollisionObject(obj, "updateSingleAabb2: unknown collisionObject");
	world->getDynamicsWorld()->updateSingleAabb(obj);
}

EXTERN_C DLL_EXPORT void UpdateAabbs2(BulletSim* world)
{
	world->getDynamicsWorld()->updateAabbs();
}

EXTERN_C DLL_EXPORT bool GetForceUpdateAllAabbs2(BulletSim* world)
{
	return world->getDynamicsWorld()->getForceUpdateAllAabbs();
}

EXTERN_C DLL_EXPORT void SetForceUpdateAllAabbs2(BulletSim* world, bool forceUpdateAllAabbs)
{
	world->getDynamicsWorld()->setForceUpdateAllAabbs(forceUpdateAllAabbs);
}

// =====================================================================
// btDynamicWorld entries
// TODO: Remember to restore any constraints
EXTERN_C DLL_EXPORT bool AddObjectToWorld2(BulletSim* sim, btCollisionObject* obj)
{
	bsDebug_AssertIsKnownCollisionObject(obj, "AddObjectToWorld2: unknown collisionObject");
	bsDebug_AssertCollisionObjectIsNotInWorld(sim, obj, "AddObjectToWorld2: collisionObject already in world");
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb)
		sim->getDynamicsWorld()->addRigidBody(rb);
	else
		sim->getDynamicsWorld()->addCollisionObject(obj);
	return true;
}

// Remember to remove any constraints
EXTERN_C DLL_EXPORT bool RemoveObjectFromWorld2(BulletSim* sim, btCollisionObject* obj)
{
	bsDebug_AssertIsKnownCollisionObject(obj, "RemoveObjectFromWorld2: unknown collisionObject");
	bsDebug_AssertCollisionObjectIsInWorld(sim, obj, "RemoveObjectToWorld2: collisionObject not in world");
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb)
		sim->getDynamicsWorld()->removeRigidBody(rb);
	else
		sim->getDynamicsWorld()->removeCollisionObject(obj);
	return true;
}

EXTERN_C DLL_EXPORT bool ClearCollisionProxyCache2(BulletSim* sim, btCollisionObject* obj)
{
	bsDebug_AssertIsKnownCollisionObject(obj, "RemoveObjectFromWorld2: unknown collisionObject");
	bsDebug_AssertCollisionObjectIsInWorld(sim, obj, "RemoveObjectToWorld2: collisionObject not in world");
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb && rb->getBroadphaseHandle())
	{
		// A cheap way of clearing the collision proxy cache is to remove and add the object
		short collisionGroup = obj->getBroadphaseHandle()->m_collisionFilterGroup;
		short collisionMask = obj->getBroadphaseHandle()->m_collisionFilterMask;
		sim->getDynamicsWorld()->removeCollisionObject(obj);
		sim->getDynamicsWorld()->addCollisionObject(obj, collisionGroup, collisionMask);
		// sim->getDynamicsWorld()->getBroadphase()->getOverlappingPairCache()->cleanProxyFromPairs(rb->getBroadphaseHandle(), sim->getDynamicsWorld()->getDispatcher());
		// sim->getDynamicsWorld()->getBroadphase()->destroyProxy(rb->getBroadphaseHandle(), sim->getDynamicsWorld()->getDispatcher());
		// rb->setBroadphaseHandle(0);
	}
	return true;
}

EXTERN_C DLL_EXPORT bool AddConstraintToWorld2(BulletSim* sim, btTypedConstraint* constrain, bool disableCollisionsBetweenLinkedBodies)
{
	bsDebug_AssertIsKnownConstraint(constrain, "AddConstraintToWorld2: unknown constraint");
	bsDebug_AssertConstraintIsNotInWorld(sim, constrain, "AddConstraintToWorld2: constraint already in world");
	sim->getDynamicsWorld()->addConstraint(constrain, disableCollisionsBetweenLinkedBodies);
	sim->getWorldData()->BSLog("AddConstraintToWorld2 ++++++++++++");
	DumpConstraint2(sim, constrain);
	return true;
}

EXTERN_C DLL_EXPORT bool RemoveConstraintFromWorld2(BulletSim* sim, btTypedConstraint* constrain)
{
	bsDebug_AssertIsKnownConstraint(constrain, "RemoveConstraintToWorld2: unknown constraint");
	bsDebug_AssertConstraintIsInWorld(sim, constrain, "RemoveConstraintToWorld2: constraint not in world");
	sim->getWorldData()->BSLog("RemoveConstraintFromWorld2 ++++++++++++");
	DumpConstraint2(sim, constrain);
	sim->getDynamicsWorld()->removeConstraint(constrain);
	return true;
}

// =====================================================================
// btCollisionObject entries and helpers
// These are in the order they are defined in btCollisionObject.h.
EXTERN_C DLL_EXPORT Vector3 GetAnisotropicFriction2(btCollisionObject* obj)
{
	bsDebug_AssertIsKnownCollisionObject(obj, "GetAnisotropicFriction2: unknown collisionObject");
	btVector3 aFrict = obj->getAnisotropicFriction();
	return Vector3(aFrict.getX(), aFrict.getY(), aFrict.getZ());
}

EXTERN_C DLL_EXPORT void SetAnisotropicFriction2(btCollisionObject* obj, Vector3 aFrict)
{
	obj->setAnisotropicFriction(aFrict.GetBtVector3());
}

EXTERN_C DLL_EXPORT bool HasAnisotropicFriction2(btCollisionObject* obj)
{
	return obj->hasAnisotropicFriction();
}

EXTERN_C DLL_EXPORT void SetContactProcessingThreshold2(btCollisionObject* obj, float threshold)
{
	obj->setContactProcessingThreshold(btScalar(threshold));
}

EXTERN_C DLL_EXPORT float GetContactProcessingThreshold2(btCollisionObject* obj)
{
	return obj->getContactProcessingThreshold();
}

EXTERN_C DLL_EXPORT bool IsStaticObject2(btCollisionObject* obj)
{
	return obj->isStaticObject();
}

EXTERN_C DLL_EXPORT bool IsKinematicObject2(btCollisionObject* obj)
{
	return obj->isKinematicObject();
}

EXTERN_C DLL_EXPORT bool IsStaticOrKinematicObject2(btCollisionObject* obj)
{
	return obj->isStaticOrKinematicObject();
}

EXTERN_C DLL_EXPORT bool HasContactResponse2(btCollisionObject* obj)
{
	return obj->hasContactResponse();
}

// Given a previously allocated collision object and a new collision shape,
//    replace the shape on the collision object with the new shape.
EXTERN_C DLL_EXPORT void SetCollisionShape2(BulletSim* sim, btCollisionObject* obj, btCollisionShape* shape)
{
	bsDebug_AssertIsKnownCollisionObject(obj, "SetCollisionShape2: unknown collisionObject");
	bsDebug_AssertIsKnownCollisionShape(obj, "SetCollisionShape2: unknown collisionShape");
	bsDebug_AssertCollisionObjectIsNotInWorld(sim, obj, "SetCollisionShape2: collision object is in world");

	obj->setCollisionShape(shape);

	// test
	// An attempt to make Bullet accept the new shape that has been stuffed into the RigidBody.
	// NOTE: This does not work here because the collisionObject is not in the world
	// btOverlappingPairCache* opp = sim->getDynamicsWorld()->getBroadphase()->getOverlappingPairCache();
	// opp->cleanProxyFromPairs(obj->getBroadphaseHandle(), sim->getDynamicsWorld()->getDispatcher());

	// Don't free the old shape here since it could be shared with other
	//    bodies. The managed code should be keeping track of the shapes
	//    and their use counts and creating and deleting them as needed.
}

EXTERN_C DLL_EXPORT btCollisionShape* GetCollisionShape2(btCollisionObject* obj)
{
	bsDebug_AssertIsKnownCollisionObject(obj, "GetCollisionShape: unknown collisionObject");
	btCollisionShape* shape = obj->getCollisionShape();
	bsDebug_AssertIsKnownCollisionShape(obj, "GetCollisionShape2: unknown collisionShape");
	return shape;
}

EXTERN_C DLL_EXPORT int GetActivationState2(btCollisionObject* obj)
{
	return obj->getActivationState();
}

EXTERN_C DLL_EXPORT void SetActivationState2(btCollisionObject* obj, int state)
{
	obj->setActivationState(state);
}

EXTERN_C DLL_EXPORT void SetDeactivationTime2(btCollisionObject* obj, float dtime)
{
	obj->setDeactivationTime(btScalar(dtime));
}

EXTERN_C DLL_EXPORT float GetDeactivationTime2(btCollisionObject* obj)
{
	return obj->getDeactivationTime();
}

EXTERN_C DLL_EXPORT void ForceActivationState2(btCollisionObject* obj, int newState)
{
	obj->forceActivationState(newState);
}

EXTERN_C DLL_EXPORT void Activate2(btCollisionObject* obj, bool forceActivation)
{
	obj->activate(forceActivation);
}

EXTERN_C DLL_EXPORT bool IsActive2(btCollisionObject* obj)
{
	return obj->isActive();
}

EXTERN_C DLL_EXPORT void SetRestitution2(btCollisionObject* obj, float val)
{
	obj->setRestitution(btScalar(val));
}

EXTERN_C DLL_EXPORT float GetRestitution2(btCollisionObject* obj)
{
	return obj->getRestitution();
}

EXTERN_C DLL_EXPORT void SetFriction2(btCollisionObject* obj, float val)
{
	obj->setFriction(btScalar(val));
}

EXTERN_C DLL_EXPORT float GetFriction2(btCollisionObject* obj)
{
	return obj->getFriction();
}

EXTERN_C DLL_EXPORT void SetWorldTransform2(btCollisionObject* obj, Transform& trans)
{
	obj->setWorldTransform(trans.GetBtTransform());
}

EXTERN_C DLL_EXPORT Transform GetWorldTransform2(btCollisionObject* obj)
{
	btTransform xform;

	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb)
		xform = rb->getWorldTransform();
	else
		xform = obj->getWorldTransform();

	return xform;
}

// Helper function to get the position from the world transform.
EXTERN_C DLL_EXPORT Vector3 GetPosition2(btCollisionObject* obj)
{
	btTransform xform;

	// getWorldTransform() on a collisionObject adds interpolation to the position.
	// Getting the transform directly from the rigidBody gets the real value.
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb)
		xform = rb->getWorldTransform();
	else
		xform = obj->getWorldTransform();

	btVector3 p = xform.getOrigin();
	return Vector3(p.getX(), p.getY(), p.getZ());
}

// Helper function to get the rotation from the world transform.
EXTERN_C DLL_EXPORT Quaternion GetOrientation2(btCollisionObject* obj)
{
	Quaternion ret = Quaternion();

	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) 
		ret = rb->getOrientation();
	else
		ret = obj->getWorldTransform().getRotation();

	return ret;
}

// Helper routine that sets the world transform based on the passed position and rotation.
EXTERN_C DLL_EXPORT void SetTranslation2(btCollisionObject* obj, Vector3 position, Quaternion rotation)
{
	btVector3 pos = position.GetBtVector3();
	btQuaternion rot = rotation.GetBtQuaternion();
	// Build a transform containing the new position and rotation
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(pos);
	transform.setRotation(rot);

	// Set the new transform for the rigid body and the motion state
	obj->setWorldTransform(transform);

	// If this is also a rigid body, push the object movement so it will be recoreded as an update.
	// The setWorldTransform() above only sets the transform variable in the object. In order to
	//      have the movement show up as a property update, it must be pushed through the motionState.
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb)
		if (!rb->isStaticOrKinematicObject())
			if (rb->getMotionState())
				rb->getMotionState()->setWorldTransform(transform);

	return;
}

EXTERN_C DLL_EXPORT btBroadphaseProxy* GetBroadphaseHandle2(btCollisionObject* obj)
{
	return obj->getBroadphaseHandle();
}

EXTERN_C DLL_EXPORT void SetBroadphaseHandle2(btCollisionObject* obj, btBroadphaseProxy* proxy)
{
	obj->setBroadphaseHandle(proxy);
}

EXTERN_C DLL_EXPORT Transform GetInterpolationWorldTransform2(btCollisionObject* obj)
{
	Transform ret = obj->getInterpolationWorldTransform();
	return ret;
}

EXTERN_C DLL_EXPORT void SetInterpolationWorldTransform2(btCollisionObject* obj, Transform trans)
{
	obj->setInterpolationWorldTransform(trans.GetBtTransform());
}

EXTERN_C DLL_EXPORT void SetInterpolationLinearVelocity2(btCollisionObject* obj, Vector3 vel)
{
	obj->setInterpolationLinearVelocity(vel.GetBtVector3());
}

EXTERN_C DLL_EXPORT void SetInterpolationAngularVelocity2(btCollisionObject* obj, Vector3 ang)
{
	obj->setInterpolationAngularVelocity(ang.GetBtVector3());
}

// Helper function that sets both linear and angular interpolation velocity
EXTERN_C DLL_EXPORT void SetInterpolationVelocity2(btCollisionObject* obj, Vector3 lin, Vector3 ang)
{
	obj->setInterpolationLinearVelocity(lin.GetBtVector3());
	obj->setInterpolationAngularVelocity(ang.GetBtVector3());
}

EXTERN_C DLL_EXPORT Vector3 GetInterpolationLinearVelocity2(btCollisionObject* obj)
{
	return Vector3(obj->getInterpolationLinearVelocity());
}

EXTERN_C DLL_EXPORT Vector3 GetInterpolationAngularVelocity2(btCollisionObject* obj)
{
	return Vector3(obj->getInterpolationLinearVelocity());
}

EXTERN_C DLL_EXPORT float GetHitFraction2(btCollisionObject* obj)
{
	return obj->getHitFraction();
}

EXTERN_C DLL_EXPORT void SetHitFraction2(btCollisionObject* obj, float val)
{
	obj->setHitFraction(btScalar(val));
}

EXTERN_C DLL_EXPORT int GetCollisionFlags2(btCollisionObject* obj)
{
	return obj->getCollisionFlags();
}

EXTERN_C DLL_EXPORT uint32_t SetCollisionFlags2(btCollisionObject* obj, uint32_t flags)
{
	obj->setCollisionFlags(flags);
	return obj->getCollisionFlags();
}

EXTERN_C DLL_EXPORT uint32_t AddToCollisionFlags2(btCollisionObject* obj, uint32_t flags)
{
	obj->setCollisionFlags(obj->getCollisionFlags() | flags);
	return obj->getCollisionFlags();
}

EXTERN_C DLL_EXPORT uint32_t RemoveFromCollisionFlags2(btCollisionObject* obj, uint32_t flags)
{
	obj->setCollisionFlags(obj->getCollisionFlags() & ~flags);
	return obj->getCollisionFlags();
}

EXTERN_C DLL_EXPORT float GetCcdSweptSphereRadius2(btCollisionObject* obj)
{
	return obj->getCcdSweptSphereRadius();
}

EXTERN_C DLL_EXPORT void SetCcdSweptSphereRadius2(btCollisionObject* obj, float val)
{
	obj->setCcdSweptSphereRadius(btScalar(val));
}

EXTERN_C DLL_EXPORT float GetCcdMotionThreshold2(btCollisionObject* obj)
{
	return obj->getCcdMotionThreshold();
}

EXTERN_C DLL_EXPORT float GetSquareCcdMotionThreshold2(btCollisionObject* obj)
{
	return obj->getCcdSquareMotionThreshold();
}

EXTERN_C DLL_EXPORT void SetCcdMotionThreshold2(btCollisionObject* obj, float val)
{
	obj->setCcdMotionThreshold(btScalar(val));
}

EXTERN_C DLL_EXPORT void* GetUserPointer2(btCollisionObject* obj)
{
	return obj->getUserPointer();
}

EXTERN_C DLL_EXPORT void SetUserPointer2(btCollisionObject* obj, void* ptr)
{
	obj->setUserPointer(ptr);
}

// ==================================================================================
// btRigidBody methods
// These are in the order found in btRigidBody.h

EXTERN_C DLL_EXPORT void ApplyGravity2(btCollisionObject* obj)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->applyGravity();
}
EXTERN_C DLL_EXPORT void SetGravity2(btCollisionObject* obj, Vector3 grav)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setGravity(grav.GetBtVector3());
}
EXTERN_C DLL_EXPORT Vector3 GetGravity2(btCollisionObject* obj)
{
	Vector3 ret = Vector3();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getGravity();
	return ret;
}

EXTERN_C DLL_EXPORT void SetDamping2(btCollisionObject* obj, float lin_damping, float ang_damping)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setDamping(btScalar(lin_damping), btScalar(ang_damping));
}

EXTERN_C DLL_EXPORT void SetLinearDamping2(btCollisionObject* obj, float lin_damping)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setDamping(btScalar(lin_damping), rb->getAngularDamping());
}

EXTERN_C DLL_EXPORT void SetAngularDamping2(btCollisionObject* obj, float ang_damping)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setDamping(rb->getLinearDamping(), btScalar(ang_damping));
}

EXTERN_C DLL_EXPORT float GetLinearDamping2(btCollisionObject* obj)
{
	float ret = 0.0;
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getLinearDamping();
	return ret;
}

EXTERN_C DLL_EXPORT float GetAngularDamping2(btCollisionObject* obj)
{
	float ret = 0.0;
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getAngularDamping();
	return ret;
}

EXTERN_C DLL_EXPORT float GetLinearSleepingThreshold2(btCollisionObject* obj)
{
	float ret = 0.0;
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getLinearSleepingThreshold();
	return ret;
}

EXTERN_C DLL_EXPORT float GetAngularSleepingThreshold2(btCollisionObject* obj)
{
	float ret = 0.0;
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getAngularSleepingThreshold();
	return ret;
}

EXTERN_C DLL_EXPORT void ApplyDamping2(btCollisionObject* obj, float timeStep)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->applyDamping(btScalar(timeStep));
}

EXTERN_C DLL_EXPORT void SetMassProps2(btCollisionObject* obj, float mass, Vector3 inertia)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setMassProps(btScalar(mass), inertia.GetBtVector3());
}

EXTERN_C DLL_EXPORT Vector3 GetLinearFactor2(btCollisionObject* obj)
{
	Vector3 ret = Vector3();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getLinearFactor();
	return ret;
}

EXTERN_C DLL_EXPORT void SetLinearFactor2(btCollisionObject* obj, Vector3 fact)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setLinearFactor(fact.GetBtVector3());
}

EXTERN_C DLL_EXPORT void SetCenterOfMassTransform2(btCollisionObject* obj, Transform trans)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setCenterOfMassTransform(trans.GetBtTransform());
}

EXTERN_C DLL_EXPORT void SetCenterOfMassByPosRot2(btCollisionObject* obj, Vector3 pos, Quaternion rot)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb)
	{
		btTransform trans;
		trans.setIdentity();
		trans.setOrigin(pos.GetBtVector3());
		trans.setRotation(rot.GetBtQuaternion());

		rb->setCenterOfMassTransform(trans);
	}
}

EXTERN_C DLL_EXPORT void ApplyCentralForce2(btCollisionObject* obj, Vector3 force)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->applyCentralForce(force.GetBtVector3());
}

EXTERN_C DLL_EXPORT void SetObjectForce2(btCollisionObject* obj, Vector3 force)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	// Oddly, Bullet doesn't have a way to directly set the force so this
	//    subtracts the total force (making force zero) and then adds our new force.
	if (rb) rb->applyCentralForce(force.GetBtVector3() - rb->getTotalForce());
}

EXTERN_C DLL_EXPORT Vector3 GetTotalForce2(btCollisionObject* obj)
{
	Vector3 ret = Vector3();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getTotalForce();
	return ret;
}

EXTERN_C DLL_EXPORT Vector3 GetTotalTorque2(btCollisionObject* obj)
{
	Vector3 ret = Vector3();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getTotalTorque();
	return ret;
}

EXTERN_C DLL_EXPORT Vector3 GetInvInertiaDiagLocal2(btCollisionObject* obj)
{
	Vector3 ret = Vector3();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getInvInertiaDiagLocal();
	return ret;
}

EXTERN_C DLL_EXPORT void SetInvInertiaDiagLocal2(btCollisionObject* obj, Vector3 inert)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setInvInertiaDiagLocal(inert.GetBtVector3());
}

EXTERN_C DLL_EXPORT void SetSleepingThresholds2(btCollisionObject* obj, float lin_threshold, float ang_threshold)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setSleepingThresholds(btScalar(lin_threshold), btScalar(ang_threshold));
}

EXTERN_C DLL_EXPORT void ApplyTorque2(btCollisionObject* obj, Vector3 force)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->applyTorque(force.GetBtVector3());
}

EXTERN_C DLL_EXPORT void ApplyForce2(btCollisionObject* obj, Vector3 force, Vector3 pos)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->applyForce(force.GetBtVector3(), pos.GetBtVector3());
}

EXTERN_C DLL_EXPORT void ApplyCentralImpulse2(btCollisionObject* obj, Vector3 force)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->applyCentralImpulse(force.GetBtVector3());
}

EXTERN_C DLL_EXPORT void ApplyTorqueImpulse2(btCollisionObject* obj, Vector3 force)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->applyTorqueImpulse(force.GetBtVector3());
}

EXTERN_C DLL_EXPORT void ApplyImpulse2(btCollisionObject* obj, Vector3 force, Vector3 pos)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->applyImpulse(force.GetBtVector3(), pos.GetBtVector3());
}

EXTERN_C DLL_EXPORT void ClearForces2(btCollisionObject* obj)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->clearForces();
}

// Zero out all forces and bring the object to a dead stop
EXTERN_C DLL_EXPORT void ClearAllForces2(btCollisionObject* obj)
{
	btVector3 zeroVector = btVector3(0.0, 0.0, 0.0);

	obj->setInterpolationLinearVelocity(zeroVector);
	obj->setInterpolationAngularVelocity(zeroVector);
	obj->setInterpolationWorldTransform(obj->getWorldTransform());

	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb)
	{
		rb->setLinearVelocity(zeroVector);
		rb->setAngularVelocity(zeroVector);
		rb->clearForces();
	}
}

EXTERN_C DLL_EXPORT void UpdateInertiaTensor2(btCollisionObject* obj)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->updateInertiaTensor();
}

EXTERN_C DLL_EXPORT Vector3 GetCenterOfMassPosition2(btCollisionObject* obj)
{
	Vector3 ret = Vector3();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getCenterOfMassPosition();
	return ret;
}

/* A helper function is above that works generally for btCollisionObject's
EXTERN_C DLL_EXPORT Quaternion GetOrientation2(btCollisionObject* obj)
{
	Quaternion ret = Quaternion();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getOrientation();
	return ret;
}
*/

EXTERN_C DLL_EXPORT Transform GetCenterOfMassTransform2(btCollisionObject* obj)
{
	Transform ret = Transform();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getCenterOfMassTransform();
	return ret;
}

EXTERN_C DLL_EXPORT Vector3 GetLinearVelocity2(btCollisionObject* obj)
{
	Vector3 ret = Vector3();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getLinearVelocity();
	return ret;
}

EXTERN_C DLL_EXPORT Vector3 GetAngularVelocity2(btCollisionObject* obj)
{
	Vector3 ret = Vector3();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getAngularVelocity();
	return ret;
}

EXTERN_C DLL_EXPORT void SetLinearVelocity2(btCollisionObject* obj, Vector3 velocity)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setLinearVelocity(velocity.GetBtVector3());
}

EXTERN_C DLL_EXPORT void SetAngularVelocity2(btCollisionObject* obj, Vector3 angularVelocity)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setAngularVelocity(angularVelocity.GetBtVector3());
}

EXTERN_C DLL_EXPORT Vector3 GetVelocityInLocalPoint2(btCollisionObject* obj, Vector3 pos)
{
	Vector3 ret = Vector3();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getVelocityInLocalPoint(pos.GetBtVector3());
	return ret;
}

EXTERN_C DLL_EXPORT void Translate2(btCollisionObject* obj, Vector3 trans)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->translate(trans.GetBtVector3());
}

EXTERN_C DLL_EXPORT void UpdateDeactivation2(btCollisionObject* obj, float timeStep)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->updateDeactivation(btScalar(timeStep));
}

EXTERN_C DLL_EXPORT bool WantsSleeping2(btCollisionObject* obj)
{
	bool ret = false;
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->wantsSleeping();
	return ret;
}

EXTERN_C DLL_EXPORT void SetAngularFactor2(btCollisionObject* obj, float fact)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setAngularFactor(btScalar(fact));
}

EXTERN_C DLL_EXPORT void SetAngularFactorV2(btCollisionObject* obj, Vector3 fact)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->setAngularFactor(fact.GetBtVector3());
}

EXTERN_C DLL_EXPORT Vector3 GetAngularFactor2(btCollisionObject* obj)
{
	Vector3 ret = Vector3();
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getAngularFactor();
	return ret;
}

EXTERN_C DLL_EXPORT bool IsInWorld2(btCollisionObject* obj)
{
	bool ret = false;
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->isInWorld();
	return ret;
}

EXTERN_C DLL_EXPORT void AddConstraintRef2(btCollisionObject* obj, btTypedConstraint* constrain)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->addConstraintRef(constrain);
}

EXTERN_C DLL_EXPORT void RemoveConstraintRef2(btCollisionObject* obj, btTypedConstraint* constrain)
{
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) rb->removeConstraintRef(constrain);
}

EXTERN_C DLL_EXPORT btTypedConstraint* GetConstraintRef2(btCollisionObject* obj, int index)
{
	btTypedConstraint* ret = 0;
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getConstraintRef(index);
	return ret;
}

EXTERN_C DLL_EXPORT int GetNumConstraintRefs2(btCollisionObject* obj)
{
	int ret = 0;
	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb) ret = rb->getNumConstraintRefs();
	return ret;
}

// ========================================
// btCollisionShape methods and related

// Would be nice to have these functions but have not decided how to return the values
// EXTERN_C DLL_EXPORT void GetAabb2(btCollisionShape* shape)
// EXTERN_C DLL_EXPORT void GetBoundingSphere2(btCollisionShape* shape)
// EXTERN_C DLL_EXPORT void CalculateTemporalAabb2(btCollisionShape* shape)

EXTERN_C DLL_EXPORT float GetAngularMotionDisc2(btCollisionShape* shape)
{
	return shape->getAngularMotionDisc();
}

EXTERN_C DLL_EXPORT float GetContactBreakingThreshold2(btCollisionShape* shape, float defaultFactor)
{
	return shape->getContactBreakingThreshold(btScalar(defaultFactor));
}

EXTERN_C DLL_EXPORT bool IsPloyhedral2(btCollisionShape* shape)
{
	return shape->isPolyhedral();
}

EXTERN_C DLL_EXPORT bool IsConvex2d2(btCollisionShape* shape)
{
	return shape->isConvex2d();
}

EXTERN_C DLL_EXPORT bool IsConvex2(btCollisionShape* shape)
{
	return shape->isConvex();
}

EXTERN_C DLL_EXPORT bool IsNonMoving2(btCollisionShape* shape)
{
	return shape->isNonMoving();
}

EXTERN_C DLL_EXPORT bool IsConcave2(btCollisionShape* shape)
{
	return shape->isConcave();
}

EXTERN_C DLL_EXPORT bool IsCompound2(btCollisionShape* shape)
{
	return shape->isCompound();
}

EXTERN_C DLL_EXPORT bool IsSoftBody2(btCollisionShape* shape)
{
	return shape->isSoftBody();
}

EXTERN_C DLL_EXPORT bool IsInfinite2(btCollisionShape* shape)
{
	return shape->isInfinite();
}

EXTERN_C DLL_EXPORT void SetLocalScaling2(btCollisionShape* shape, Vector3 scale)
{
	shape->setLocalScaling(scale.GetBtVector3());
}

EXTERN_C DLL_EXPORT Vector3 GetLocalScaling2(btCollisionShape* shape)
{
	return shape->getLocalScaling();
}

EXTERN_C DLL_EXPORT Vector3 CalculateLocalInertia2(btCollisionShape* shape, float mass)
{
	btVector3 btInertia;
	shape->calculateLocalInertia(btScalar(mass), btInertia);
	return Vector3(btInertia);
}

EXTERN_C DLL_EXPORT int GetShapeType2(btCollisionShape* shape)
{
	return shape->getShapeType();
}

EXTERN_C DLL_EXPORT void SetMargin2(btCollisionShape* shape, float val)
{
	shape->setMargin(btScalar(val));
}

EXTERN_C DLL_EXPORT float GetMargin2(btCollisionShape* shape)
{
	return shape->getMargin();
}

EXTERN_C DLL_EXPORT bool SetCollisionGroupMask2(btCollisionObject* obj, unsigned int group, unsigned int mask)
{
	bool ret = false;
	btBroadphaseProxy* proxy = obj->getBroadphaseHandle();
	// If the object is not in the world, there won't be a proxy.
	if (proxy)
	{
		// staticSim->getWorldData()->BSLog("SetCollisionGroupMask. ogroup=%x, omask=%x, ngroup=%x, nmask=%x",
		// 				(int)proxy->m_collisionFilterGroup, (int)proxy->m_collisionFilterMask, group, mask);
		proxy->m_collisionFilterGroup = (short)group;
		proxy->m_collisionFilterMask = (short)mask;
		ret = true;
	}
	// else
	// {
	// 	staticSim->getWorldData()->BSLog("SetCollisionGroupMask did not find a proxy");
	// }
	return ret;
}

// =====================================================================


// =====================================================================
// =====================================================================
// =====================================================================
// Raycasting.
/**
 * Perform a sweep test by moving a convex shape through space and testing for collisions. 
 * Starting and ending rotations are not currently supported since this was designed for
 * character sweep tests, which use capsules.
 * @param worldID ID of the world to access.
 * @param id Object ID of a convex object.
 * @param from Starting position of the sweep.
 * @param to Destination position of the sweep.
 * @param extraMargin Extra collision margin to add to the convex shape during the sweep.
 * @return Sweep results. If there were no collisions, SweepHit.ID will be ID_INVALID_HIT (0xFFFFFFFF)
 */
EXTERN_C DLL_EXPORT SweepHit ConvexSweepTest2(BulletSim* world, btCollisionShape* obj, Vector3 from, Vector3 to, float extraMargin)
{
	btVector3 f = from.GetBtVector3();
	btVector3 t = to.GetBtVector3();
	return world->ConvexSweepTest(obj, f, t, extraMargin);
}

/**
 * Perform a raycast test by drawing a line from a and testing for collisions.
 * @param worldID ID of the world to access.
 * @param id Object ID to ignore during the raycast.
 * @param from Start of the ray.
 * @param to End of the ray.
 * @return Raycast results. If there were no collisions, RaycastHit.ID will be ID_INVALID_HIT (0xFFFFFFFF)
 */
EXTERN_C DLL_EXPORT RaycastHit RayTest2(BulletSim* world, Vector3 from, Vector3 to, unsigned int filterGroup, unsigned int filterMask)
{
	btVector3 f = from.GetBtVector3();
	btVector3 t = to.GetBtVector3();
	return world->RayTest(f, t, (short)filterGroup, (short)filterMask);
}

/**
 * Returns the position offset required to bring a character out of a penetrating collision.
 * @param worldID ID of the world to access.
 * @param id Character ID.
 * @return A position offset to apply to the character to resolve a penetration.
 */
EXTERN_C DLL_EXPORT Vector3 RecoverFromPenetration2(BulletSim* world, unsigned int id)
{
	btVector3 v = world->RecoverFromPenetration(id);
	return Vector3(v.getX(), v.getY(), v.getZ());
}

// =====================================================================
// Debugging
// Dump a btCollisionObject and even more if it's a btRigidBody.
EXTERN_C DLL_EXPORT void DumpRigidBody2(BulletSim* sim, btCollisionObject* obj)
{
	sim->getWorldData()->BSLog("DumpRigidBody: id=%u, loc=%x, pos=<%f,%f,%f>, orient=<%f,%f,%f,%f>",
				CONVLOCALID(obj->getUserPointer()),
				obj,
				(float)obj->getWorldTransform().getOrigin().getX(),
				(float)obj->getWorldTransform().getOrigin().getY(),
				(float)obj->getWorldTransform().getOrigin().getZ(),
				(float)obj->getWorldTransform().getRotation().getX(),
				(float)obj->getWorldTransform().getRotation().getY(),
				(float)obj->getWorldTransform().getRotation().getZ(),
				(float)obj->getWorldTransform().getRotation().getW()
		);

	sim->getWorldData()->BSLog("DumpRigidBody: actState=%d, active=%s, static=%s, mergesIslnd=%s, contactResp=%s, cFlag=%d, deactTime=%f",
				obj->getActivationState(),
				obj->isActive() ? "true" : "false",
				obj->isStaticObject() ? "true" : "false",
				obj->mergesSimulationIslands() ? "true" : "false",
				obj->hasContactResponse() ? "true" : "false",
				obj->getCollisionFlags(),
				(float)obj->getDeactivationTime()
		);

	sim->getWorldData()->BSLog("DumpRigidBody: ccdTrsh=%f, ccdSweep=%f, contProc=%f, frict=%f, hitFract=%f, restit=%f, internTyp=%f",
				(float)obj->getCcdMotionThreshold(),
				(float)obj->getCcdSweptSphereRadius(),
				(float)obj->getContactProcessingThreshold(),
				(float)obj->getFriction(),
				(float)obj->getHitFraction(),
				(float)obj->getRestitution(),
				(float)obj->getInternalType()
		);

	btBroadphaseProxy* proxy = obj->getBroadphaseHandle();
	if (proxy)
	{
		sim->getWorldData()->BSLog("DumpRigidBody: collisionFilterGroup=%X, mask=%X",
									proxy->m_collisionFilterGroup,
									proxy->m_collisionFilterMask);
	}

	btTransform interpTrans = obj->getInterpolationWorldTransform();
	btVector3 interpPos = interpTrans.getOrigin();
	btQuaternion interpRot = interpTrans.getRotation();
	sim->getWorldData()->BSLog("DumpRigidBody: interpPos=<%f,%f,%f>, interpRot=<%f,%f,%f,%f>, interpLVel=<%f,%f,%f>, interpAVel=<%f,%f,%f>",
				(float)interpPos.getX(),
				(float)interpPos.getY(),
				(float)interpPos.getZ(),
				(float)interpRot.getX(),
				(float)interpRot.getY(),
				(float)interpRot.getZ(),
				(float)interpRot.getW(),
				(float)obj->getInterpolationLinearVelocity().getX(),
				(float)obj->getInterpolationLinearVelocity().getY(),
				(float)obj->getInterpolationLinearVelocity().getZ(),
				(float)obj->getInterpolationAngularVelocity().getX(),
				(float)obj->getInterpolationAngularVelocity().getY(),
				(float)obj->getInterpolationAngularVelocity().getZ()
		);



	btRigidBody* rb = btRigidBody::upcast(obj);
	if (rb)
	{
		sim->getWorldData()->BSLog("DumpRigidBody: lVel=<%f,%f,%f>, lFactor=<%f,%f,%f>, aVel=<%f,%f,%f>, aFactor=<%f,%f,%f> sleepThresh=%f, aDamp=%f",
					(float)rb->getLinearVelocity().getX(),
					(float)rb->getLinearVelocity().getY(),
					(float)rb->getLinearVelocity().getZ(),
					(float)rb->getLinearFactor().getX(),
					(float)rb->getLinearFactor().getY(),
					(float)rb->getLinearFactor().getZ(),
					(float)rb->getAngularVelocity().getX(),
					(float)rb->getAngularVelocity().getY(),
					(float)rb->getAngularVelocity().getZ(),
					(float)rb->getAngularFactor().getX(),
					(float)rb->getAngularFactor().getY(),
					(float)rb->getAngularFactor().getZ(),
					(float)rb->getAngularSleepingThreshold(),
					(float)rb->getAngularDamping()
			);

		sim->getWorldData()->BSLog("DumpRigidBody: totForce=<%f,%f,%f>, totTorque=<%f,%f,%f>",
					(float)rb->getTotalForce().getX(),
					(float)rb->getTotalForce().getY(),
					(float)rb->getTotalForce().getZ(),
					(float)rb->getTotalTorque().getX(),
					(float)rb->getTotalTorque().getY(),
					(float)rb->getTotalTorque().getZ()
			);

		float invMass = (float)rb->getInvMass();
		btTransform COMtransform = rb->getCenterOfMassTransform();
		btVector3 COMPosition = COMtransform.getOrigin();
		btQuaternion COMRotation = COMtransform.getRotation();
		sim->getWorldData()->BSLog("DumpRigidBody: grav=<%f,%f,%f>, COMPos=<%f,%f,%f>, COMRot=<%f,%f,%f,%f>,invMass=%f, mass=%f",
					(float)rb->getGravity().getX(),
					(float)rb->getGravity().getY(),
					(float)rb->getGravity().getZ(),
					(float)COMPosition.getX(),
					(float)COMPosition.getY(),
					(float)COMPosition.getZ(),
					(float)COMRotation.getX(),
					(float)COMRotation.getY(),
					(float)COMRotation.getZ(),
					(float)COMRotation.getW(),
					invMass,
					invMass == 0.0 ? 0.0 : (1.0 / invMass)
			);

		btScalar inertiaTensorYaw, inertiaTensorPitch, inertiaTensorRoll;
		rb->getInvInertiaTensorWorld().getEulerYPR(inertiaTensorYaw, inertiaTensorPitch, inertiaTensorRoll);
		sim->getWorldData()->BSLog("DumpRigidBody: invInertDiag=<%f,%f,%f>, invInertiaTensorW: yaw=%f, pitch=%f, roll=%f",
					(float)rb->getInvInertiaDiagLocal().getX(),
					(float)rb->getInvInertiaDiagLocal().getY(),
					(float)rb->getInvInertiaDiagLocal().getZ(),
					(float)inertiaTensorYaw,
					(float)inertiaTensorPitch,
					(float)inertiaTensorRoll
			);
	}
}

EXTERN_C DLL_EXPORT void DumpCollisionShape2(BulletSim* sim, btCollisionShape* shape)
{
	int shapeType = shape->getShapeType();
	const char* shapeTypeName;
	switch (shapeType)
	{
		case BOX_SHAPE_PROXYTYPE: shapeTypeName = "boxShape"; break;
		case TRIANGLE_SHAPE_PROXYTYPE: shapeTypeName = "triangleShape"; break;
		case SPHERE_SHAPE_PROXYTYPE: shapeTypeName = "sphereShape"; break;
		case CAPSULE_SHAPE_PROXYTYPE: shapeTypeName = "capsuleShape"; break;
		case TRIANGLE_MESH_SHAPE_PROXYTYPE: shapeTypeName = "triangleMeshShape"; break;
		case COMPOUND_SHAPE_PROXYTYPE: shapeTypeName = "compoundShape"; break;
		default: shapeTypeName = "unknown"; break;
	}
	sim->getWorldData()->BSLog("DumpCollisionShape: type=%s, id=%u, loc=%x, margin=%f, isMoving=%s, isConvex=%s",
			shapeTypeName,
			CONVLOCALID(shape->getUserPointer()),
			shape,
			(float)shape->getMargin(),
			shape->isNonMoving() ? "true" : "false",
			shape->isConvex() ? "true" : "false"
		);
	sim->getWorldData()->BSLog("DumpCollisionShape:   localScaling=<%f,%f,%f>",
			(float)shape->getLocalScaling().getX(),
			(float)shape->getLocalScaling().getY(),
			(float)shape->getLocalScaling().getZ()
		);
}

EXTERN_C DLL_EXPORT void DumpFrameInfo(BulletSim* sim, const char* type, btTransform& frameInA, btTransform& frameInB)
{
	btVector3 frameInALoc = frameInA.getOrigin();
	btQuaternion frameInARot = frameInA.getRotation();
	btVector3  frameInBLoc = frameInB.getOrigin();
	btQuaternion frameInBRot = frameInB.getRotation();
	sim->getWorldData()->BSLog("DumpConstraint: %s: frameInALoc=<%f,%f,%f>, frameInARot=<%f,%f,%f,%f>", type,
				frameInALoc.getX(), frameInALoc.getY(), frameInALoc.getZ(),
				frameInARot.getX(), frameInARot.getY(), frameInARot.getZ(), frameInARot.getW() );
	sim->getWorldData()->BSLog("DumpConstraint: %s: frameInBLoc=<%f,%f,%f>, frameInBRot=<%f,%f,%f,%f>", type,
				frameInBLoc.getX(), frameInBLoc.getY(), frameInBLoc.getZ(),
				frameInBRot.getX(), frameInBRot.getY(), frameInBRot.getZ(), frameInBRot.getW() );
}

// Several of the constraints are based on the 6Dof constraint.
// Print common info.
EXTERN_C DLL_EXPORT void Dump6DofInfo(BulletSim* sim, const char* type, btGeneric6DofConstraint* constrain)
{
	btTransform frameInA = constrain->getFrameOffsetA();
	btTransform frameInB = constrain->getFrameOffsetB();
	DumpFrameInfo(sim, type, frameInA, frameInB);

	btVector3 linLower, linUpper;
	btVector3 angLower, angUpper;
	constrain->getLinearLowerLimit(linLower);
	constrain->getLinearUpperLimit(linUpper);
	constrain->getAngularLowerLimit(angLower);
	constrain->getAngularUpperLimit(angUpper);
	sim->getWorldData()->BSLog("DumpConstraint: %s: linLow=<%f,%f,%f>, linUp=<%f,%f,%f>", type,
				linLower.getX(), linLower.getY(), linLower.getZ(),
				linUpper.getX(), linUpper.getY(), linUpper.getZ() );
	sim->getWorldData()->BSLog("DumpConstraint: %s: angLow=<%f,%f,%f>, angUp=<%f,%f,%f>,appliedImpulse=%f", type,
				angLower.getX(), angLower.getY(), angLower.getZ(),
				angUpper.getX(), angUpper.getY(), angUpper.getZ(),
				constrain->getAppliedImpulse() );
}

// Outputs constraint information
EXTERN_C DLL_EXPORT void DumpConstraint2(BulletSim* sim, btTypedConstraint* constrain)
{
		sim->getWorldData()->BSLog("DumpConstraint: obj1=%x, obj2=%x, enabled=%s",
			&(constrain->getRigidBodyA()),
			&(constrain->getRigidBodyB()),
			constrain->isEnabled() ? "true" : "false");
		if (constrain->getConstraintType() == D6_CONSTRAINT_TYPE)
		{
			btGeneric6DofConstraint* cc = (btGeneric6DofConstraint*)constrain;
			Dump6DofInfo(sim, "6DOF", cc);
		}
		if (constrain->getConstraintType() == D6_SPRING_CONSTRAINT_TYPE)
		{
			btGeneric6DofSpringConstraint* cc = (btGeneric6DofSpringConstraint*)constrain;
			Dump6DofInfo(sim, "Spring", cc);
		}
		if (constrain->getConstraintType() == HINGE_CONSTRAINT_TYPE)
		{
			btHinge2Constraint* cc = (btHinge2Constraint*)constrain;
			// Dump6DofInfo(sim, "Hinge", cc);
			btVector3 anchor1, anchor2, axis1, axis2;
			anchor1 = cc->getAnchor();
			anchor2 = cc->getAnchor2();
			axis1 = cc->getAxis1();
			axis2 = cc->getAxis2();
			sim->getWorldData()->BSLog("DumpConstraint: Hinge: anchor1=<%f,%f,%f>, anchor2=<%f,%f,%f>, axis1=<%f,%f,%f>, axis2=<%f,%f,%f>",
						anchor1.getX(), anchor1.getY(), anchor1.getZ(),
						anchor2.getX(), anchor2.getY(), anchor2.getZ(),
						axis1.getX(), axis1.getY(), axis1.getZ(),
						axis2.getX(), axis2.getY(), axis2.getZ() );
			float angle1, angle2;
			angle1 = cc->getAngle1();
			angle2 = cc->getAngle2();
			sim->getWorldData()->BSLog("DumpConstraint: Hinge: angle1=%f, angle2==%f", angle1, angle2);
		}
		if (constrain->getConstraintType() == SLIDER_CONSTRAINT_TYPE)
		{
			btSliderConstraint* cc = (btSliderConstraint*)constrain;
			btTransform frameInA = cc->getFrameOffsetA();
		    btTransform frameInB = cc->getFrameOffsetB();
			DumpFrameInfo(sim, "Slider", frameInA, frameInB);

		    btScalar lowerLinLimit = cc->getLowerLinLimit();
		    btScalar upperLinLimit = cc->getUpperLinLimit();
		    btScalar lowerAngLimit = cc->getLowerAngLimit();
		    btScalar upperAngLimit = cc->getUpperAngLimit();
			bool useLinearReferenceFrameA = cc->getUseLinearReferenceFrameA();
			sim->getWorldData()->BSLog("DumpConstraint: Slider: lowLinLim=%f, upperLinLim=%f, lowAngLim=%f, upperAngLim=%f, useRefFrameA=%d", 
						lowerLinLimit, lowerLinLimit, upperAngLimit, upperAngLimit, useLinearReferenceFrameA );

			btScalar softnessDirLin = cc->getSoftnessDirLin();
			btScalar restitutionDirLin = cc->getRestitutionDirLin();
			btScalar dampingDirLin = cc->getDampingDirLin();
			btScalar softnessDirAng = cc->getSoftnessDirAng();
			btScalar restitutionDirAng = cc->getRestitutionDirAng();
			btScalar dampingDirAng = cc->getDampingDirAng();
			sim->getWorldData()->BSLog("DumpConstraint: Slider: DirLin: soft=%f, rest=%f, damp=%f. DirAng: soft=%f, rest=%f, damp=%f",
										softnessDirLin, restitutionDirLin, dampingDirLin,
										softnessDirAng, restitutionDirAng, dampingDirAng);
			btScalar softnessLimLin = cc->getSoftnessLimLin();
			btScalar restitutionLimLin = cc->getRestitutionLimLin();
			btScalar dampingLimLin = cc->getDampingLimLin();
			btScalar softnessLimAng = cc->getSoftnessLimAng();
			btScalar restitutionLimAng = cc->getRestitutionLimAng();
			btScalar dampingLimAng = cc->getDampingLimAng();
			sim->getWorldData()->BSLog("DumpConstraint: Slider: LimLin: soft=%f, rest=%f, damp=%f. LimAng: soft=%f, rest=%f, damp=%f",
										softnessLimLin, restitutionLimLin, dampingLimLin,
										softnessLimAng, restitutionLimAng, dampingLimAng);
			btScalar softnessOrthoLin = cc->getSoftnessOrthoLin();
			btScalar restitutionOrthoLin = cc->getRestitutionOrthoLin();
			btScalar dampingOrthoLin = cc->getDampingOrthoLin();
			btScalar softnessOrthoAng = cc->getSoftnessOrthoAng();
			btScalar restitutionOrthoAng = cc->getRestitutionOrthoAng();
			btScalar dampingOrthoAng = cc->getDampingOrthoAng();
			sim->getWorldData()->BSLog("DumpConstraint: Slider: OrthoLin: soft=%f, rest=%f, damp=%f. OrthoAng: soft=%f, rest=%f, damp=%f",
										softnessOrthoLin, restitutionOrthoLin, dampingOrthoLin,
										softnessOrthoAng, restitutionOrthoAng, dampingOrthoAng );
		}
		if (constrain->getConstraintType() == CONETWIST_CONSTRAINT_TYPE)
		{
			btConeTwistConstraint* cc = (btConeTwistConstraint*)constrain;
			btTransform frameInA = cc->getAFrame();
		    btTransform frameInB = cc->getBFrame();
			DumpFrameInfo(sim, "ConeTwist", frameInA, frameInB);
		}
}

extern float gDeactivationTime;
EXTERN_C DLL_EXPORT void DumpAllInfo2(BulletSim* sim)
{
	btDynamicsWorld* world = sim->getDynamicsWorld();

	sim->getWorldData()->BSLog("gDisableDeactivation=%d, gDeactivationTime=%f, splitIslands=%d",
		gDisableDeactivation, gDeactivationTime, ((btDiscreteDynamicsWorld*)world)->getSimulationIslandManager()->getSplitIslands());

	btCollisionObjectArray& collisionObjects = world->getCollisionObjectArray();
	int numCollisionObjects = collisionObjects.size();
	for (int ii=0; ii < numCollisionObjects; ii++)
	{
		btCollisionObject* obj = collisionObjects[ii];
		// If there is an object and it probably is not terrain, dump.
		if (obj && (CONVLOCALID(obj->getUserPointer()) > 100))
		{
			sim->getWorldData()->BSLog("===========================================");
			DumpRigidBody2(sim, obj);
			btCollisionShape* shape = obj->getCollisionShape();
			if (shape)
			{
				DumpCollisionShape2(sim, shape);
			}
		}
	}

	sim->getWorldData()->BSLog("=CONSTRAINTS==========================================");
	int numConstraints = world->getNumConstraints();
	for (int jj=0; jj < numConstraints; jj++)
	{
		DumpConstraint2(sim, world->getConstraint(jj));
	}
	sim->getWorldData()->BSLog("=END==========================================");
}

// Dump info about the number of objects and their activation state
EXTERN_C DLL_EXPORT void DumpActivationInfo2(BulletSim* sim)
{
	btDynamicsWorld* world = sim->getDynamicsWorld();
	btCollisionObjectArray& collisionObjects = world->getCollisionObjectArray();
	int numRigidBodies = 0;
	int* activeStates = new int[10];
	for (int ii=0; ii<10; ii++) activeStates[ii] = 0;

	int numCollisionObjects = collisionObjects.size();
	for (int ii=0; ii < numCollisionObjects; ii++)
	{
		btCollisionObject* obj = collisionObjects[ii];
		int activeState = obj->getActivationState();
		activeStates[activeState]++;

		btRigidBody* rb = btRigidBody::upcast(obj);
		if (rb)
		{
			numRigidBodies++;
		}
	}
	sim->getWorldData()->BSLog("     num CollisionObject = %d", numCollisionObjects);
	sim->getWorldData()->BSLog("         num RigidBodies = %d", numRigidBodies);
	sim->getWorldData()->BSLog("          num ACTIVE_TAG = %d", activeStates[ACTIVE_TAG]);
	sim->getWorldData()->BSLog("     num ISLAND_SLEEPING = %d", activeStates[ISLAND_SLEEPING]);
	sim->getWorldData()->BSLog("  num WANTS_DEACTIVATION = %d", activeStates[WANTS_DEACTIVATION]);
	sim->getWorldData()->BSLog("num DISABLE_DEACTIVATION = %d", activeStates[DISABLE_DEACTIVATION]);
	sim->getWorldData()->BSLog("  num DISABLE_SIMULATION = %d", activeStates[DISABLE_SIMULATION]);
	sim->getWorldData()->BSLog("    num overlappingPairs = %d", world->getPairCache()->getNumOverlappingPairs());

	/* Code for displaying some of the info in the overlapping pairs cache
	btBroadphasePairArray& pairArray = world->getPairCache()->getOverlappingPairArray();
	int numPairs = pairArray.size();

	for (int ii=0; ii < numPairs; ii += 10000)
	{
		sim->getWorldData()->BSLog("pairArray[%d], id0=%u, id1=%u", ii,
					((btCollisionObject*)pairArray[ii].m_pProxy0->m_clientObject)->getUserPointer(),
					((btCollisionObject*)pairArray[ii].m_pProxy1->m_clientObject)->getUserPointer());
	}
	*/

}


// Version of log printer that takes the simulator as a parameter.
// Used for statistics logging from Bullet.
// Bullet must be patched to enable this functionality -- by default it does a printf.
EXTERN_C DLL_EXPORT void DebugLogger2(void* xxx, const char* msg, ...)
{
	BulletSim* sim = (BulletSim*)xxx;
	va_list args;
	va_start(args, msg);
	sim->getWorldData()->BSLog2(msg, args);
	va_end(args);
	return;
}

// Causes detailed physics performance statistics to be logged.
EXTERN_C DLL_EXPORT void DumpPhysicsStatistics2(BulletSim* sim)
{
	if (sim->getWorldData()->debugLogCallback)
	{
		// Uncomment the next line to enable timing logging from Bullet.
		// The Bullet library MUST be patched to create this entry point.
		sim->getWorldData()->dumpAll();
	}
	return;
}

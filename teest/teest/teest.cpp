#include "includes.h"
#include "btBulletDynamicsCommon.h"


class MyCallback :public osg::NodeCallback
{
public:
	MyCallback()
	{}
	MyCallback(osg::MatrixTransform* cow, btDiscreteDynamicsWorld* dynamicsWorld)
		:m_cow(cow), m_dynamicsWorld(dynamicsWorld)
	{}

	virtual void operator()(osg::Node* node, osg::NodeVisitor* nv)
	{
		cout << 16 << endl;

		//for (i=0;i<100;i++)
//{
		m_dynamicsWorld->stepSimulation(1.f / 60.f, 10);// 每秒60帧

		//print positions of all objects
		for (int j = m_dynamicsWorld->getNumCollisionObjects() - 1; j >= 0; j--)
		{
			btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[j];
			btRigidBody* body = btRigidBody::upcast(obj);
			if (body && body->getMotionState())
			{
				btTransform trans;
				body->getMotionState()->getWorldTransform(trans);
				printf("world pos = %f\t%f\t%f\t(%d)\n", float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()), j);

				if (1 == j)
				{
					m_cow->setMatrix(osg::Matrix::translate(osg::Vec3(float(trans.getOrigin().getX()), float(trans.getOrigin().getY()), float(trans.getOrigin().getZ()))));
				}
			}
		}
		//}

			//继续遍历
		traverse(node, nv);
	}

private:
	osg::MatrixTransform* m_cow;
	btDiscreteDynamicsWorld* m_dynamicsWorld;
};


int main()
{
	osgViewer::Viewer* viewer1 = new osgViewer::Viewer;
	osg::Node* cow = osgDB::readNodeFile("cow.osg");
	osg::Group* gScene = new osg::Group;

	// 把牛移上天：
	osg::MatrixTransform* mtCow = new osg::MatrixTransform;
	mtCow->addChild(cow);
	mtCow->setMatrix(osg::Matrix::translate(osg::Vec3(0, 0, 100)));
	gScene->addChild(mtCow);// 加入场景

	// 建地面：
	osg::Geometry* geometry1 = new osg::Geometry;
	osg::Vec3Array* arrVertex = new osg::Vec3Array;
	geometry1->setVertexArray(arrVertex);
	arrVertex->push_back(osg::Vec3(-100, 100, 0));
	arrVertex->push_back(osg::Vec3(100, 100, 0));
	arrVertex->push_back(osg::Vec3(100, -100, 0));
	arrVertex->push_back(osg::Vec3(-100, -100, 0));
	geometry1->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::QUADS, 0, 4));
	//osg::Geode *geode1 = new osg::Geode;
	//geode1->addDrawable(geometry1);

	// 加入场景:
	gScene->addChild(geometry1);

	// 物理引擎：
	///collision configuration contains default setup for memory, collision setup. Advanced users can create their own configuration.
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	///use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);
	///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
	///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, 0, -9.8));


	///create a few basic rigid bodies 地面
	btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(50.), btScalar(50.), btScalar(0.1)));
	//keep track of the shapes, we release memory at exit.
	//make sure to re-use collision shapes among rigid bodies whenever possible!
	btAlignedObjectArray<btCollisionShape*> collisionShapes;// 物理单位
	collisionShapes.push_back(groundShape);
	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(0, 0, 0));

	{
		btScalar mass(0.);// 地板没有质量， 不受重力影响

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0, 0, 0);
		if (isDynamic)
		{
			groundShape->calculateLocalInertia(mass, localInertia);
			cout << "0. != 0.f" << endl;
		}

		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(groundTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, groundShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		//add the body to the dynamics world
		dynamicsWorld->addRigidBody(body);
	}

	{
		//create a dynamic rigidbody

		//btCollisionShape* colShape = new btBoxShape(btVector3(1,1,1));
		btCollisionShape* colShape = new btSphereShape(btScalar(1.));
		collisionShapes.push_back(colShape);

		/// Create Dynamic Objects
		btTransform startTransform;
		startTransform.setIdentity();

		btScalar	mass(1.f);

		//rigidbody is dynamic if and only if mass is non zero, otherwise static
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(100, 0, 0);
		if (isDynamic)
		{
			colShape->calculateLocalInertia(mass, localInertia);
		}
		startTransform.setOrigin(btVector3(0, 0, 100));// 初始高度
		//using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		dynamicsWorld->addRigidBody(body);
	}

	///-----stepsimulation_start-----
	//int i;
	//for (i=0;i<100;i++)
	//{
	//	dynamicsWorld->stepSimulation(1.f/60.f,10);// 每秒60帧
	//	
	//	//print positions of all objects
	//	for (int j=dynamicsWorld->getNumCollisionObjects()-1; j>=0 ;j--)
	//	{
	//		btCollisionObject* obj = dynamicsWorld->getCollisionObjectArray()[j];
	//		btRigidBody* body = btRigidBody::upcast(obj);
	//		if (body && body->getMotionState())
	//		{
	//			btTransform trans;
	//			body->getMotionState()->getWorldTransform(trans);
	//			printf("world pos = %f\t%f\t%f\t(%d)\n",float(trans.getOrigin().getX()),float(trans.getOrigin().getY()),float(trans.getOrigin().getZ()),j);
	//		}
	//	}
	//}

	MyCallback* cb1 = new MyCallback(mtCow, dynamicsWorld);
	gScene->addUpdateCallback(cb1);

	viewer1->setSceneData(gScene);
	viewer1->run();
	return 0;
}
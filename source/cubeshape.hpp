#ifndef CUBESHAPE_HPP
#define CUBESHAPE_HPP

#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <btBulletDynamicsCommon.h>

#include "vertexmanage.hpp"

extern btDiscreteDynamicsWorld *dynamicsWorld;

class cubeshapeObject{
	public:
	int id;
	btDiscreteDynamicsWorld *dynamicsWorld;
	btRigidBody* body;
	glm::vec3 size;
	cubeshapeObject(int id, btRigidBody* body, glm::vec3 size, btDiscreteDynamicsWorld *dynamicsWorld);
	void destroy();
	void changeID(int newID);
	void loadMotionState();
};

namespace cubeshape{

	extern GLuint indexBufferObject;
	extern GLuint instanceMatrixBuffer;

	extern GLuint indexBufferArray[14];

	extern std::vector<glm::mat4> instanceMatrixArray;


	extern void init();
	extern cubeshapeObject* create(glm::vec3 position, glm::vec3 size, glm::quat quat, btScalar mass, btDiscreteDynamicsWorld *dynamicsWorld);
	extern void destroy(int id);
	extern void render();
}

extern cubeshapeObject* cubeshape_create(float x, float y, float z, float w, float h, float d, float qw, float qx, float qy, float qz, float g);

#endif
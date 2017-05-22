#ifndef FLOORSHAPE_HPP
#define FLOORSHAPE_HPP

#include <vector>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <btBulletDynamicsCommon.h>

#include "vertexmanage.hpp"

class floorshapeObject;

namespace floorshape{
	extern GLuint floor_indexBufferObject;
	extern GLuint floor_instanceMatrixBuffer;
	extern GLuint floor_indexBufferArray[4];

	/*
	GLuint floor_indexBufferArray[4] = {
		8, 9, 10, 11
	};
	*/

	extern glm::mat4 floor_instanceMatrixArray[1];



	extern void init();
	extern floorshapeObject* create(glm::vec3 position, glm::vec3 face, glm::quat quat, btDiscreteDynamicsWorld *dynamicsWorld);
	extern void destroy(int id);
	extern void render();


}


#endif
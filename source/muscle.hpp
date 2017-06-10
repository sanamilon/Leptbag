#ifndef MUSCLE_HPP
#define MUSCLE_HPP

#include <iostream>
#include <vector>
#include <random>

#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <btBulletDynamicsCommon.h>

#include "vertexmanage.hpp"
#include "shader.hpp"
#include "cubeshape.hpp"
#include "floorshape.hpp"
#include "constraints.hpp"

class contractileElement{
public:
    int id;
    cubeshapeObject* cubeA;
    cubeshapeObject* cubeB;
    btVector3 attachInA;
    btVector3 attachInB;
    float angle;
    float max_force;


    contractileElement(cubeshapeObject* cubeA, cubeshapeObject* cubeB, btVector3 attachInA, btVector3 attachInB, float length, float angle);
    virtual float getTorque(float u);
    virtual float activationDynamics(float u);
    virtual float contractionDynamics(float act);
    virtual float updateJointMoments(float force);
    virtual btVector3 localToWorld(cubeshapeObject* cube, btVector3 inLocal);
};

#endif

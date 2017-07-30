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

#include "elementNode.hpp"
#include "utilities/utilities.hpp"

class contractileElement{
    public:
    contractileElement();
    contractileElement(elementNode* elemA, elementNode* elemB, vec3 attachInA, vec3 attachInB, float angle, float max_force);
    elementNode* elemA;
    elementNode* elemB;
    vec3 attachInA;
    vec3 attachInB;
    float angle;
    float max_force;
    virtual float getTorque(float u);
    virtual float activationDynamics(float u);
    virtual float contractionDynamics(float act);
    virtual float updateJointMoments(float force);
    virtual btVector3 localToWorld(elementNode* elem, btVector3 inLocal);
};

extern "C" contractileElement* contractileElement_create(elementNode* elemA, elementNode* elemB, vec3 &attachInA, vec3 &attachInB, float &angle, float &max_force);

#endif

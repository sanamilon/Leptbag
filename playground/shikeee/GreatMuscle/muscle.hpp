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

    float length;       // 現在の長さ
    float opt_length;   // 最適長
    float rest_length;  // たるんだ時の長さ(最大長)
    float force;
    float preForce;
    float max_force;
    float velocity;
    float max_velocity;
    float act;

    float w; // fL曲線の鐘の幅
    float N; // エンハンス
    float K; // 曲率

    contractileElement(cubeshapeObject* cubeA, cubeshapeObject* cubeB, float ax, float ay, float az, float bx, float by, float bz, float length);
    virtual void contract(float act);
    virtual float fL();
    virtual float fV();
    virtual float getForce();
    virtual btVector3 localToWorld(cubeshapeObject* cube, btVector3 inLocal);
};

#endif

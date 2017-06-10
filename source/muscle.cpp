#include "muscle.hpp"

#include <iostream>
#include <cmath>

contractileElement::contractileElement(cubeshapeObject* cubeA, cubeshapeObject* cubeB, btVector3 attachInA, btVector3 attachInB, float angle, float max_force){
	//this->id = id;
	this->cubeA = cubeA;
	this->cubeB = cubeB;
	this->attachInA = attachInA;
    this->attachInB = attachInB;
	this->angle = angle;
	this->max_force  = max_force;
}

// このメソッド、今のところ使いみちないけど便利そうだからとっておく。
btVector3 contractileElement::localToWorld(cubeshapeObject* cube, btVector3 inLocal){
	btVector3 axis  = cube->body->getOrientation().getAxis();
	btScalar angle = btScalar(cube->body->getOrientation().getAngle());

	btTransform transform;
	cube->body->getMotionState()->getWorldTransform(transform);
	btVector3 pos = transform.getOrigin();

	// 回転
	btVector3 inWorld = inLocal.rotate(axis, angle);
	// 平行移動
	inWorld += pos;

	return(inWorld);
}

float contractileElement::getTorque(float u){
	// これが筋肉モデルのすべて
	float act    = activationDynamics(u);
	float force  = contractionDynamics(act);
	float torque = updateJointMoments(force);
	return torque;
}

// 以下のメソッドを徐々に拡充させていく. 今はちょっとテキトー。
float contractileElement::activationDynamics(float u){
	// 励起信号から筋肉活性率を求める。
	 float act = u;
	 return act;
}

float contractileElement::contractionDynamics(float act){
	// 力の大きさを計算
	float force = act * max_force;
	return force;
}

float contractileElement::updateJointMoments(float force){
	// モーメントアームと垂直成分を求める
	float r = attachInB.getY();
	float f = force * sin(angle);

	// トルクを出力
	return r * f;
}

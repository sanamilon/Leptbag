#include "muscle.hpp"

#include <iostream>
#include <cmath>

contractileElement::contractileElement(){
}

contractileElement::contractileElement(elementNode* elemA, elementNode* elemB, vec3 attachInA, vec3 attachInB, float angle, float max_force){
	this->elemA = elemA;
	this->elemB = elemB;
	this->attachInA = attachInA;
    this->attachInB = attachInB;
	this->angle = angle;
	this->max_force  = max_force;
}

// このメソッド、今のところ使いみちないけど便利そうだからとっておく。
btVector3 contractileElement::localToWorld(elementNode* elem, btVector3 inLocal){
	btVector3 axis  = elemA->getBody()->getOrientation().getAxis();
	btScalar angle = btScalar(elemA->getBody()->getOrientation().getAngle());

	btTransform transform;
	elemA->getBody()->getMotionState()->getWorldTransform(transform);
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
	float r = attachInB.y;
	float f = force * sin(angle);

	// トルクを出力
	return r * f;
}

contractileElement* contractileElement_create(elementNode* elemA, elementNode* elemB, vec3 &attachInA, vec3 &attachInB, float &angle, float &max_force){
	return new contractileElement(elemA, elemB, attachInA, attachInB, angle, max_force);
}

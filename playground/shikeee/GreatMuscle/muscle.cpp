#include "muscle.hpp"

#include <iostream>
#include <cmath>

contractileElement::contractileElement(cubeshapeObject* cubeA, cubeshapeObject* cubeB, float ax, float ay, float az, float bx, float by, float bz, float length){
	//this->id = id;
	this->cubeA = cubeA;
	this->cubeB = cubeB;
	this->attachInA = btVector3(ax, ay, az);
    this->attachInB = btVector3(bx, by, bz);
	this->length = length;
	this->rest_length = length;

	this->force = 0.0;
	this->opt_length = 1.0;
	this->max_force  = 40;
	this->max_velocity = -1.0;
	this->act = 0.01;
	this->w = 0.4;
    this->N = 1.5;
    this->K = 5;
}
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
void contractileElement::contract(float act){
	/* ✝無視された計算式の墓場✝

	velocity = act * max_velocity;
	float f;
	if(velocity < max_velocity) velocity = max_velocity;
	length += velocity * (1 / 60.f);

	if(length <= rest_length){
		force =  act * max_force * fL() * fV();
		//std::cout << force << std::endl;
	} else {
		force = 0;
	}
	*/

	// 加える力の方向を求める
	btVector3 aInWorld = localToWorld(cubeA, attachInA);
	btVector3 bInWorld = localToWorld(cubeB, attachInB);

	btVector3 dir = (aInWorld - bInWorld).normalize();

	// 目的の方向に力を加える
	btVector3 forceV = dir * act * max_force;
	cubeA->body->applyForce(-forceV, attachInA);
	std::cout << forceV.getX() <<", "<< forceV.getY() <<", "<< forceV.getZ() << std::endl;;
	cubeB->body->applyForce(forceV, attachInB);
}

float contractileElement::fL(){
	float Q = exp(log(0.05) * pow(std::abs((length - opt_length)/(opt_length * w)), 3));
	return Q;
}

float contractileElement::fV(){
	float Q;
	if(velocity >= 0){
		Q = N + (N-1)*((max_velocity + velocity)/(7.56 * K * velocity - max_velocity));
	} else {
		Q = (max_velocity - velocity)/(max_velocity + K * velocity);
	}
	std::cout << Q << std::endl;
	return Q;
}

float contractileElement::getForce(){
	return force;
}

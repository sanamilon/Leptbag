import core.stdc.stdio;
import std.stdio;
import core.runtime;
import std.random;
import std.math;
import std.algorithm;


import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;
import params;
import loadJson;

kame myKame;
agentBodyParameter info;

const float bodyMass = 5.0f;
int wait = 200;

class kame{

	elementNode[string] parts;
	hingeConstraint[string] hinges;
	generic6DofConstraint[string] g6dofs;
	agentBodyParameter bodyInfo;


	this(float x, float y, float z, agentBodyParameter info){

		spawn(Vector3f(x, y, z), info);

	}

	void spawn(Vector3f position, agentBodyParameter info){

		this.bodyInfo = info;

		foreach(string name, params; bodyInfo.partParams){
				bodyInfo.partsGenerator[name] = new elementManager(bodyInfo.partParams[name].vertices, &createConvexHullShapeBody);
		}

		Vector3f zeroVec3 = Vector3f(0.0, 0.0, 0.0);


		foreach(string s, partsGen; bodyInfo.partsGenerator){
			switch(s){
				case "shell":
				parts[s] = partsGen.generate(parameterPack(
					param("position", bodyInfo.partParams[s].position + position),
					param("scale",    bodyInfo.partParams[s].scale),
					param("rotation", bodyInfo.partParams[s].rotation),
					param("model",    bodyInfo.partParams[s].vertices),
					param("mass",     0.0f)));
					break;
				default:
				parts[s] = partsGen.generate(parameterPack(
					param("position", bodyInfo.partParams[s].position + position),
					param("scale",    bodyInfo.partParams[s].scale),
					param("rotation", bodyInfo.partParams[s].rotation),
					param("model",    bodyInfo.partParams[s].vertices),
					param("mass",     bodyInfo.partParams[s].mass * bodyMass)));
					break;
			}

		}

		writeln(bodyInfo);
		foreach(string s, param; bodyInfo.g6dofParams){
			elementNode obj1, obj2;
			Vector3f pos1, pos2;
			Quaternionf rotation;

			obj1 = parts[bodyInfo.g6dofParams[s].object1Name];
			obj2 = parts[bodyInfo.g6dofParams[s].object2Name];
			pos1 = bodyInfo.g6dofParams[s].object1Position;
			pos2 = bodyInfo.g6dofParams[s].object2Position;
			rotation = bodyInfo.g6dofParams[s].rotation;

			g6dofs[s] = new generic6DofConstraint(obj1, obj2, pos1, pos2, rotation);

			switch(s){
				case "leftback-shell":
				g6dofs[s].setAngularLimit(zeroVec3, zeroVec3);
				g6dofs[s].setLinearLimit (bodyInfo.g6dofParams[s].linLimitLower, bodyInfo.g6dofParams[s].linLimitUpper);
				/*
				g6dofs[s].setRotationalMotor(0);
				g6dofs[s].setRotationalMotor(1);
				g6dofs[s].setRotationalMotor(2);
				g6dofs[s].setMaxRotationalMotorForce(0, 10000);
				g6dofs[s].setMaxRotationalMotorForce(1, 10000);
				g6dofs[s].setMaxRotationalMotorForce(2, 10000);
				break;
				*/
				break;
				default:
				g6dofs[s].setAngularLimit(Vector3f(0.0,0.0,0.0), Vector3f(0.01,0.01,0.01));
				g6dofs[s].setLinearLimit (Vector3f(0.0,0.0,0.0), Vector3f(0.01,0.01,0.01));
				break;
			}

		}



	}

	void move(){
		if(wait-->0) return;
		foreach(string s, param; bodyInfo.g6dofParams){
			switch(s){
				case "Constraint.002", "Constraint.005", "Constraint.007", "Constraint.010":
					g6dofs[s].setMaxRotationalMotorForce(0, 10000);
					g6dofs[s].setMaxRotationalMotorForce(1, 10000);
					g6dofs[s].setMaxRotationalMotorForce(2, 10000);
					g6dofs[s].setRotationalTargetVelocity(Vector3f(0, 0, 100));
				default:
			}
		}
	}

	void despawn(){
		foreach(part; parts) part.destroy();
		foreach(hinge; hinges) hinge.destroy();
		foreach(dofs; g6dofs) dofs.destroy();
	}


}


//ApplicationInterface----------------------

extern (C) void init(){
	rt_init();
	writeln("learningKame.d loaded");

	//jsonからload
	loadMesh(info.partParams);
	loadG6dof(info.g6dofParams);

	myKame = new kame(0, 2, -1, info);
}


extern (C) void tick(){
	myKame.move();
}



//------------------------------------------
import core.stdc.stdio;
import std.stdio;
import core.runtime;
import std.random;
import std.math;
import std.algorithm;


import japariSDK.japarilib;
import params;
import loadJson;

kame myKame;
agentBodyParameter info;

const float bodyMass = 5.0f;

class kame{

	elementNode[string] parts;
	hingeConstraint[string] hinges;
	generic6DofConstraint[string] g6dofs;
	agentBodyParameter bodyInfo;


	this(float x, float y, float z, agentBodyParameter info){

		spawn(createVec3(x, y, z), info);

	}

	void spawn(vec3 position, agentBodyParameter info){

		this.bodyInfo = info;

		foreach(string name, params;bodyInfo.partParams){
				bodyInfo.partsGenerator[name] = createElementManager(bodyInfo.partParams[name].vertices, &createConvexHullShapeBody);
		}


		foreach(string s, partsGen; bodyInfo.partsGenerator){

			parts[s] = partsGen.generate(paramWrap(
				param("position", addVec(bodyInfo.partParams[s].position, position)),
				param("scale",    bodyInfo.partParams[s].scale),
				param("rotation", bodyInfo.partParams[s].rotation),
				param("model",    bodyInfo.partParams[s].vertices),
				param("mass",
				//0.0f)));
				bodyInfo.partParams[s].mass * bodyMass)));

			}

		foreach(string s, param; bodyInfo.hingeParams){
			hinges[s] = hingeConstraint_create(parts[bodyInfo.hingeParams[s].object1Name], parts[bodyInfo.hingeParams[s].object2Name],
					bodyInfo.hingeParams[s].object1Position, bodyInfo.hingeParams[s].object2Position,
					bodyInfo.hingeParams[s].axis1, bodyInfo.hingeParams[s].axis2);
			hinges[s].setLimit( bodyInfo.hingeParams[s].limitLower, bodyInfo.hingeParams[s].limitLower );
			if( bodyInfo.hingeParams[s].enabled ){
				hinges[s].enableMotor(true);
				hinges[s].setMaxMotorImpulse(5);
			}
		}

		foreach(string s, param; bodyInfo.g6dofParams){
			g6dofs[s] = generic6DofConstraint_create(parts[bodyInfo.g6dofParams[s].object1Name], parts[bodyInfo.g6dofParams[s].object2Name],
					bodyInfo.g6dofParams[s].object1Position, bodyInfo.g6dofParams[s].object2Position,
					bodyInfo.g6dofParams[s].rotation);
		}



	}

	void move(int sequence){

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
	loadHinge(info.hingeParams);
	loadG6dof(info.g6dofParams);

	myKame = new kame(0, 1, -1, info);
}


extern (C) void tick(){

}



//------------------------------------------

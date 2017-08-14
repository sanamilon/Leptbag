import core.stdc.stdio;
import std.stdio;
import core.runtime;
import std.random;
import std.math;
import std.algorithm;


import japariSDK.japarilib;

kame myKame;

class kame{

	elementNode[string] parts;
	hingeConstraint[string] hinges;
	generic6DofConstraint[string] g6dofs;
	float neko = 0;

	float[string][20] dna;

	this(float x, float y, float z, bool initialDNA){

		spawn(createVec3(x, y, z));

		if(initialDNA == true){
			foreach(string s, hinge; hinges){
				if(hingeParams[s].enabled){
					for(int row = 0; row < 20; row++){
						dna[row][s] = uniform(-PI/2, PI/2, rnd);
					}
				}
			}
		}

	}

	void spawn(vec3 position){


		foreach(string s, elementManager partsGen; partsGenerator){

			parts[s] = partsGen.generate(paramWrap(
						param("position", addVec(partParams[s].position, position)),
						param("scale",    partParams[s].scale),
						param("rotation", partParams[s].rotation),
						param("model",    partParams[s].vertices),
						param("mass",
							//0.0f)));
							partParams[s].mass * bodyMass)));

		}

		foreach(string s, param; hingeParams){
			hinges[s] = hingeConstraint_create(parts[hingeParams[s].object1Name], parts[hingeParams[s].object2Name],
					hingeParams[s].object1Position, hingeParams[s].object2Position,
					hingeParams[s].axis1, hingeParams[s].axis2);
			hinges[s].setLimit( hingeParams[s].limitLower, hingeParams[s].limitLower );
			if( hingeParams[s].enabled ){
				hinges[s].enableMotor(true);
				hinges[s].setMaxMotorImpulse(5);
			}
		}

		foreach(string s, param; g6dofParams){
			g6dofs[s] = generic6DofConstraint_create(parts[g6dofParams[s].object1Name], parts[g6dofParams[s].object2Name],
					g6dofParams[s].object1Position, g6dofParams[s].object2Position,
					g6dofParams[s].rotation);

			for(int i=0; i<3; i++){
				if(g6dofParams[s].useAngLimit[i]) g6dofs[s].setRotationalMotor(i);
				if(g6dofParams[s].useLinLimit[i]) g6dofs[s].setLinearMotor(i);
			}

			vec3 zeroVec3 = createVec3( 0.0, 0.0, 0.0 ); //セッターに同じvec3を入れるとロック
			g6dofs[s].setAngularLimit( g6dofParams[s].angLimitLower, g6dofParams[s].angLimitUpper );
			g6dofs[s].setLinearLimit( zeroVec3, zeroVec3 );


　
			//最大出力．index ; (x, y, z)=(0, 1, 2)(たぶん？)
			g6dofs[s].setMaxRotationalMotorForce( 0, 5.0);
			g6dofs[s].setMaxRotationalMotorForce( 1, 5.0);
			g6dofs[s].setMaxRotationalMotorForce( 2, 5.0);
			g6dofs[s].setMaxLinearMotorForce( zeroVec3 );
		}



	}

	void move(int sequence){
		if(hinges.length!=0) foreach(string s, hinge; hinges){
			if(hingeParams[s].enabled){
				float target = abs(hingeParams[s].limitLower-hingeParams[s].limitUpper) * dna[sequence][s] * 2.0/PI;
				hinge.setMotorTarget(target, 0.5);
			}
		}
		g6dofs["Constraint.003"].setRotationalTargetVelocity(createVec3(0.0f,1.0*sin(neko), 0.0f));
		g6dofs["Constraint.001"].setRotationalTargetVelocity(createVec3(0.0f,1.0*sin(neko-PI/2.0), 0.0f));
		g6dofs["Constraint.002"].setRotationalTargetVelocity(createVec3(0.0f,1.0*sin(neko-PI), 0.0f));
		g6dofs["Constraint.004"].setRotationalTargetVelocity(createVec3(0.0f,1.0*sin(neko-PI*3.0/2.0), 0.0f));

		neko += 0.3f;
		if(neko>=2.0*3.14f) neko -= 2.0*3.14f;

			/*
						uniform(g6dofParams[s].angLimitLower.getx(), g6dofParams[s].angLimitUpper.getx(), rnd),
						uniform(g6dofParams[s].angLimitLower.gety(), g6dofParams[s].angLimitUpper.gety(), rnd),
						uniform(g6dofParams[s].angLimitLower.getz(), g6dofParams[s].angLimitUpper.getz(), rnd)));
						*/
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
	myKame = new kame(0, 1, -1);
}


extern (C) void tick(){

}



//------------------------------------------

import core.stdc.stdio;
import std.stdio;
import core.time;
import core.runtime;
import std.random;
import std.math;
import std.datetime;
import std.algorithm;
import std.conv;
import std.array;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;

SysTime previousTime;
bool timeFlag = true;
dog[] doglist;
Random rnd;
int count = 0;

const int numofdog = 100;

struct muscle{
	const float fCE_max = 500;
	const float vCE_max = 0.2;
	const float l_opt = 0.3;
	const float w = 0.4 * 0.2;
	const float K = 5.0;
	const float N = 1.5;
	const float c = log(0.05);
	
	elementNode elemA;
	elementNode elemB;
	
	Vector3f posInA = Vector3f(0, 0, 0);
	Vector3f posInB = Vector3f(0, 0, 0);
	Vector3f jointInA = Vector3f(0, 0, 0);
	
	float massOfArm = 0;
	float lengthOfArm = 0;
	
	float dt = 0.05;
	
	float fM = 0;
	float act = 0;
	float fCE = 0;
	float fPE = 0;
	float vCE = 0;
	float lCE = 0.4;
	
	void setParams(elementNode eA, elementNode eB, Vector3f pA, Vector3f pB, Vector3f jA, float m, float l){
		elemA = eA;
		elemB = eB;
		posInA = pA;
		posInB = pB;
		jointInA = jA;
		massOfArm = m;
		lengthOfArm = l;
	}	
	
	float musculoskeletalModel(float u){
		// muscle actication dynamics
		act = muscleActivationDynamics(u);
		//if(posInA == Vector3f(0.4, -0.5, -0.4)) writeln(act);
		
		// contraction dynamics
		fM = muscleContractionDynamics(act);
		//if(posInA == Vector3f(0.4, -0.5, -0.4)) writeln(vCE);
		//if(posInA == Vector3f(0.4, -0.5, -0.4)) writeln(fCE);
		
		float targetVelocity = updateJointMoments();
		//writeln(posInA, ", ", targetVelocity);
		return targetVelocity;
	}
	
	float fCE_func(float a, float l, float v){
		//if(posInA == Vector3f(0.4, -0.5, -0.4)) writeln(fCE, ", a = ", a, ", fL(l) = ", fL(l), ", fV(v) = ", fV(v));
		return a*fCE_max*fL(l)*fV(v);
	}
	
	float fL(float l){
		float wow = exp(c*abs(pow(((l-l_opt)/(l_opt-w)),3)));
		return wow;
	}
	
	float fV(float v){
		float wow;
		if(v < 0){
			wow = ((vCE_max - v)/(vCE_max + K*v));
		} else {
			wow = (N + (N-1)*((vCE_max + v)/7.56*K*v - vCE_max));
		}
		return wow;
	}
	
	float fV_reverse(float x){
		float r;
		if(x < 0){
			r = ((x-1)*vCE_max)/(x*K + 1);
		} else {
			r = ((x-1)*vCE_max)/(7.56*K*(x-N)-N+1);
		}
		return r;
	}
	
	float fPE_func(float l){
		if(l < l_opt){
			return fCE_max*pow(((l-l_opt)/l_opt*w), 2);
		} else {
			return 0;
		}
	}
	
	float muscleActivationDynamics(float u){
		/* ここでdtを算出する. この後の計算でも同じステップ内では同じdtを用いる.
		if(timeFlag){
			previousTime = Clock.currTime();
			dt = 0;
			timeFlag = false;
		} else {
			auto currentTime = Clock.currTime();
			auto dur = currentTime - previousTime;
			dt = dur.total!"usecs";
			dt = dt / 1000;
			previousTime = currentTime;
		}
		*/

		return ((u - act)*dt + act);
	}
	
	float muscleContractionDynamics(float a){
		// fVの逆関数を用いてvCEを計算
		if(fL(lCE) == 0){
			vCE = fV_reverse(0);
		}else{
			vCE = fV_reverse(fCE/(a * fCE_max * fL(lCE)));
		}
		
		// vCEを用いてlCEを計算
		this.lCE -= vCE*dt;
		
		// fCEを計算
		fCE = fCE_func(a, lCE, vCE);
		
		// fPEを計算
		fPE = fPE_func(lCE);
		
		// ゲラゲラポ
		return fCE + fPE;
	}
	
	// update joint moments "&" TRANSFORM IT TO TARGET VELOCITY FOR BULLET PHYSICS
	float updateJointMoments(){
		// 一様な棒回りの慣性モーメント
		float i = massOfArm*pow(lengthOfArm,2);
		Vector3f s = (elemB.getPos() + posInB) - (elemA.getPos() + posInA);
		Vector3f tmp1 = posInA - jointInA;
		Vector3f tmp2 = s / s.length();
		Vector3f r;
		r = Vector3f(tmp1.y*tmp2.z - tmp1.z*tmp2.y, tmp1.z*tmp2.x - tmp1.x,tmp2.z, tmp1.x*tmp2.y - tmp1.y*tmp2.x);
		float torque;
		
		// なぜかここy, しかもy>0なんだよなぁ、不思議すぎる
		if(r.y > 0){
			torque = -fM * r.length();
		} else {
			torque = fM * r.length();
		}
		
		float a = torque / i;
		// target velocity = a * dt
		return a * dt;
	}
	

}


class dog{
	float[5][5] w12;
	float[6] w23;
	
	float b1 = 0.1;
	float b2 = 0.1;
	
	float[8] u;
	
	muscle[8] myMuscle;

	elementNode chest;
	elementNode head;
	elementNode muzzle;
	elementNode earLeft;
	elementNode earRight;
	elementNode legFrontLeft;
	elementNode legFrontRight;
	elementNode legBackLeft;
	elementNode legBackRight;
	elementNode tail;

	generic6DofConstraint hinge_body_head;
	generic6DofConstraint hinge_head_muzzle;
	generic6DofConstraint hinge_earLeft_head;
	generic6DofConstraint hinge_earRight_head;
	generic6DofConstraint hinge_body_legFrontLeft;
	generic6DofConstraint hinge_body_legFrontRight;
	generic6DofConstraint hinge_body_legBackLeft;
	generic6DofConstraint hinge_body_legBackRight;
	generic6DofConstraint hinge_body_tail;


	this(float x, float y, float z, bool initialDNA) {
		if(initialDNA == true){
			
			for(int col = 0; col < 5; col++){
				for(int row = 0; row < 5; row++){
					//dna[col][row] = uniform(-PI/2, PI/2, rnd);
					w12[col][row] = uniform(0.0, 0.3, rnd);
				}
			}
			
			for(int col = 0; col < 6; col++){
				w23[col] = uniform(0.0, 0.3, rnd);
			}
		}
		
		for(int col = 0; col < 4; col++){
			u[col] = 0.2;
		}
		for(int col = 4; col < 8; col++){
			u[col] = 0.5;
		}

		spawn(x, y, z);
	}
	
	
	float trynet(float[4] inputs){
			
			//中間層計算
			float[5] a = [0,0,0,0,0];
			for(int i = 0; i < 5 ; i++) {
				for(int j = 0; j < 5; j++){
					if(j == 0){
						a[i] += b1;
					} else {
						a[i] += inputs[j-1] * w12[i][j];
					}
				}
			}
			
			//出力層計算
			float o = 0;
			for(int i = 0; i < 6 ; i++) {
				if(i == 0){
					o += b2;
				} else {
					o += a[i-1] * w23[i];
				}
			}
			
			//活性化関数
			writeln(o);
			if(o > 1){
				return 1;
			} else if(o < 0){
				return 0;
			} else {
				return o;
			}

		}
		



	void spawn(float x, float y, float z){
		//犬の体の構造を定義している
		//キューブで肉体を作る cubeshape::create(位置, 大きさ, 傾き, 重さ, 追加先物理世界);

		chest = getCubeShape().generate(parameterPack(
					param("position", Vector3f(    x,     y,     z)),
					param("scale",    Vector3f(  1, 0.5, 0.5)),
					param("rotation", Quaternionf(0, 0, 0, 1)),
					param("mass", 2.0f)));

		head = getCubeShape().generate(parameterPack(
					param("position", Vector3f(x+1.4,     y,     z)),
					param("scale", Vector3f(0.4, 0.4, 0.4)),
					param("rotation", Quaternionf(0, 0, 0, 1)),
					param("mass", 0.5f)));

		muzzle = getCubeShape().generate(parameterPack(
					param("position", Vector3f(x+2.1, y-0.2,     z)),
					param("scale", Vector3f(0.3, 0.2, 0.2)),
					param("rotation", Quaternionf(0, 0, 0, 1)),
					param("mass", 0.1f)));

		earLeft = getCubeShape().generate(parameterPack(
					param("position", Vector3f(x+1.4, y+0.5, z-0.2)),
					param("scale", Vector3f(0.1, 0.1, 0.1)),
					param("rotation", Quaternionf(0, 0, 0, 1)),
					param("mass", 0.05f)));

		earRight = getCubeShape().generate(parameterPack(
					param("position", Vector3f(x+1.4, y+0.5, z+0.2)),
					param("scale", Vector3f(0.1, 0.1, 0.1)),
					param("rotation", Quaternionf(0, 0, 0, 1)),
					param("mass", 0.05f)));

		legFrontLeft = getCubeShape().generate(parameterPack(
					param("position", Vector3f(x+0.5,   y-1, z-0.4)),
					param("scale", Vector3f(0.1, 0.5, 0.1)),
					param("rotation", Quaternionf(0, 0, 0, 1)),
					param("mass", 0.3f)));

		legFrontRight = getCubeShape().generate(parameterPack(
					param("position", Vector3f(x+0.5,   y-1, z+0.4)),
					param("scale", Vector3f(0.1, 0.5, 0.1)),
					param("rotation", Quaternionf(0, 0, 0, 1)),
					param("mass", 0.3f)));

		legBackLeft = getCubeShape().generate(parameterPack(
					param("position", Vector3f(x-0.5,   y-1, z-0.4)),
					param("scale", Vector3f(0.1, 0.5, 0.1)),
					param("rotation", Quaternionf(0, 0, 0, 1)),
					param("mass", 0.3f)));

		legBackRight = getCubeShape().generate(parameterPack(
					param("position", Vector3f(x-0.5,   y-1, z+0.4)),
					param("scale", Vector3f(0.1, 0.5, 0.1)),
					param("rotation", Quaternionf(0, 0, 0, 1)),
					param("mass", 0.3f)));

		tail = getCubeShape().generate(parameterPack(
					param("position", Vector3f(x-1.5, y+0.4,     z)),
					param("scale", Vector3f(0.5, 0.1, 0.1)),
					param("rotation", Quaternionf(0, 0, 0, 1)),
					param("mass", 0.2f)));
					
		
		myMuscle[0].setParams(chest, legFrontLeft,  Vector3f( 0.4, -0.5, -0.4), Vector3f(-0.1, -0.5, 0), Vector3f( 0.5, -0.5, -0.4), 0.4, 1.0);
		myMuscle[1].setParams(chest, legFrontRight, Vector3f( 0.4, -0.5,  0.4), Vector3f(-0.1, -0.5, 0), Vector3f( 0.5, -0.5,  0.4), 0.4, 1.0);
		myMuscle[2].setParams(chest, legBackLeft,   Vector3f(-0.6, -0.5, -0.4), Vector3f(-0.1, -0.5, 0), Vector3f(-0.5, -0.5, -0.4), 0.4, 1.0);
		myMuscle[3].setParams(chest, legBackRight,  Vector3f(-0.6, -0.5,  0.4), Vector3f(-0.1, -0.5, 0), Vector3f(-0.5, -0.5,  0.4), 0.4, 1.0);
		
		myMuscle[4].setParams(chest, legFrontLeft,  Vector3f( 0.6, -0.5, -0.4), Vector3f( 0.1, -0.5, 0), Vector3f( 0.5, -0.5, -0.4), 0.4, 1.0);
		myMuscle[5].setParams(chest, legFrontRight, Vector3f( 0.6, -0.5,  0.4), Vector3f( 0.1, -0.5, 0), Vector3f( 0.5, -0.5,  0.4), 0.4, 1.0);
		myMuscle[6].setParams(chest, legBackLeft,   Vector3f(-0.4, -0.5, -0.4), Vector3f( 0.1, -0.5, 0), Vector3f(-0.5, -0.5, -0.4), 0.4, 1.0);
		myMuscle[7].setParams(chest, legBackRight,  Vector3f(-0.4, -0.5,  0.4), Vector3f( 0.1, -0.5, 0), Vector3f(-0.5, -0.5,  0.4), 0.4, 1.0);


		hinge_body_head			= new generic6DofConstraint(chest   , head         , Vector3f(   1,    0,    0), Vector3f(-0.4,   0,    0), Quaternionf(0, 0, 0, 1));
		hinge_head_muzzle		= new generic6DofConstraint(head    , muzzle       , Vector3f( 0.4, -0.2,    0), Vector3f(-0.3,   0,    0), Quaternionf(0, 0, 0, 1));
		hinge_earLeft_head		= new generic6DofConstraint(earLeft , head         , Vector3f(   0, -0.1,    0), Vector3f(   0, 0.4, -0.2), Quaternionf(0, 0, 0, 1));
		hinge_earRight_head		= new generic6DofConstraint(earRight, head         , Vector3f(   0, -0.1,    0), Vector3f(   0, 0.4,  0.2), Quaternionf(0, 0, 0, 1));
		hinge_body_legFrontLeft = new generic6DofConstraint(chest   , legFrontLeft , Vector3f( 0.5, -0.5, -0.4), Vector3f(   0, 0.5,  0.0), Quaternionf(0, 0, 0, 1));
		hinge_body_legFrontRight= new generic6DofConstraint(chest   , legFrontRight, Vector3f( 0.5, -0.5,  0.4), Vector3f(   0, 0.5,  0.0), Quaternionf(0, 0, 0, 1));
		hinge_body_legBackLeft	= new generic6DofConstraint(chest   , legBackLeft  , Vector3f(-0.5, -0.5, -0.4), Vector3f(   0, 0.5,  0.0), Quaternionf(0, 0, 0, 1));
		hinge_body_legBackRight	= new generic6DofConstraint(chest   , legBackRight , Vector3f(-0.5, -0.5,  0.4), Vector3f(   0, 0.5,  0.0), Quaternionf(0, 0, 0, 1));
		hinge_body_tail			= new generic6DofConstraint(chest   , tail         , Vector3f(  -1,  0.4,    0), Vector3f( 0.5,   0,  0.0), Quaternionf(0, 0, 0, 1));

		hinge_body_head.setAngularLimit(Vector3f(0, -PI/6, 0), Vector3f(0, PI/6, 0));
		hinge_head_muzzle.setAngularLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_earLeft_head.setAngularLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_earRight_head.setAngularLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_body_legFrontLeft.setAngularLimit(Vector3f(0, -PI/2, 0), Vector3f(0, PI/2, 0));
		hinge_body_legFrontRight.setAngularLimit(Vector3f(0, -PI/2, 0), Vector3f(0, PI/2, 0));
		hinge_body_legBackLeft.setAngularLimit(Vector3f(0, -PI/2, 0), Vector3f(0, PI/2, 0));
		hinge_body_legBackRight.setAngularLimit(Vector3f(0, -PI/2, 0), Vector3f(0, PI/2, 0));
		hinge_body_tail.setAngularLimit(Vector3f(0, -PI/3, 0), Vector3f(0, PI/3, 0));
		
		hinge_body_head.setLinearLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_head_muzzle.setLinearLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_earLeft_head.setLinearLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_earRight_head.setLinearLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_body_legFrontLeft.setLinearLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_body_legFrontRight.setLinearLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_body_legBackLeft.setLinearLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_body_legBackRight.setLinearLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		hinge_body_tail.setLinearLimit(Vector3f(0, 0, 0), Vector3f(0, 0, 0));
		
		for (int i = 0; i<3; i++) {
			hinge_body_legFrontLeft.setRotationalMotor(i);
			hinge_body_legFrontLeft.setMaxRotationalMotorForce(i, 1);
			hinge_body_legFrontRight.setRotationalMotor(i);
			hinge_body_legFrontRight.setMaxRotationalMotorForce(i, 1);
			hinge_body_legBackLeft.setRotationalMotor(i);
			hinge_body_legBackLeft.setMaxRotationalMotorForce(i, 1);
			hinge_body_legBackRight.setRotationalMotor(i);
			hinge_body_legBackRight.setMaxRotationalMotorForce(i, 1);
			
			hinge_body_legFrontLeft.setLinearMotor(i);
			hinge_body_legFrontLeft.setMaxLinearMotorForce(Vector3f(0, 0, 0));
			hinge_body_legFrontRight.setLinearMotor(i);
			hinge_body_legFrontRight.setMaxLinearMotorForce(Vector3f(0, 0, 0));
			hinge_body_legBackLeft.setLinearMotor(i);
			hinge_body_legBackLeft.setMaxLinearMotorForce(Vector3f(0, 0, 0));
			hinge_body_legBackRight.setLinearMotor(i);
			hinge_body_legBackRight.setMaxLinearMotorForce(Vector3f(0, 0, 0));
		}

		hinge_body_legFrontLeft.setRotationalTargetVelocity(Vector3f(0, 0, 0));
		hinge_body_legFrontRight.setRotationalTargetVelocity(Vector3f(0, 0, 0));
		hinge_body_legBackLeft.setRotationalTargetVelocity(Vector3f(0, 0, 0));
		hinge_body_legBackRight.setRotationalTargetVelocity(Vector3f(0, 0, 0));
		
		hinge_body_legFrontLeft.setLinearTargetVelocity(Vector3f(0, 0, 0));
		hinge_body_legFrontRight.setLinearTargetVelocity(Vector3f(0, 0, 0));
		hinge_body_legBackLeft.setLinearTargetVelocity(Vector3f(0, 0, 0));
		hinge_body_legBackRight.setLinearTargetVelocity(Vector3f(0, 0, 0));
	}

	void move(int sequence){
		/*
		hinge_body_legFrontLeft.setRotationalTargetVelocity(dna[sequence][0], 0.3);
		hinge_body_legFrontRight.setRotationalTargetVelocity(dna[sequence][1], 0.3);
		hinge_body_legBackLeft.setRotationalTargetVelocity(dna[sequence][2], 0.3);
		hinge_body_legBackRight.setRotationalTargetVelocity(dna[sequence][3], 0.3);
		
		hinge_body_legFrontLeft.setRotationalTargetVelocity(Vector3f(0, (dna[sequence][0]-hinge_body_legFrontLeft.getAngle(1))*2, 0));
		hinge_body_legFrontRight.setRotationalTargetVelocity(Vector3f(0, (dna[sequence][1]-hinge_body_legFrontRight.getAngle(1))*2, 0));
		hinge_body_legBackLeft.setRotationalTargetVelocity(Vector3f(0, (dna[sequence][2]-hinge_body_legBackLeft.getAngle(1))*2, 0));
		hinge_body_legBackRight.setRotationalTargetVelocity(Vector3f(0, (dna[sequence][3]-hinge_body_legBackRight.getAngle(1))*2, 0));
		
		*/
		
		
		u[0] = trynet([u[0], u[4], hinge_body_legFrontLeft.getAngle(1), 1]);
		u[1] = trynet([u[1], u[5], hinge_body_legFrontRight.getAngle(1), 1]);
		u[2] = trynet([u[2], u[6], hinge_body_legBackLeft.getAngle(1), 1]);
		u[3] = trynet([u[3], u[7], hinge_body_legBackRight.getAngle(1), 1]);
		u[4] = trynet([u[4], u[0], hinge_body_legFrontLeft.getAngle(1), 1]);
		u[5] = trynet([u[5], u[1], hinge_body_legFrontRight.getAngle(1), 1]);
		u[6] = trynet([u[6], u[2], hinge_body_legBackLeft.getAngle(1), 1]);
		u[7] = trynet([u[7], u[3], hinge_body_legBackRight.getAngle(1), 1]);
		
		//for(int i=0; i<8; i++) writeln(u[i]);
		
		hinge_body_legFrontLeft.setRotationalTargetVelocity(Vector3f(0, (myMuscle[0].musculoskeletalModel(u[0]) + myMuscle[4].musculoskeletalModel(u[4])  - hinge_body_legFrontLeft.getAngle(1)*2), 0));
		hinge_body_legFrontRight.setRotationalTargetVelocity(Vector3f(0, (myMuscle[1].musculoskeletalModel(u[1]) + myMuscle[5].musculoskeletalModel(u[5]) - hinge_body_legFrontRight.getAngle(1)*2), 0));
		hinge_body_legBackLeft.setRotationalTargetVelocity(Vector3f(0, (myMuscle[2].musculoskeletalModel(u[2]) + myMuscle[6].musculoskeletalModel(u[6])   - hinge_body_legBackLeft.getAngle(1)*2), 0));
		hinge_body_legBackRight.setRotationalTargetVelocity(Vector3f(0, (myMuscle[3].musculoskeletalModel(u[3]) + myMuscle[7].musculoskeletalModel(u[7])  - hinge_body_legBackRight.getAngle(1)*2), 0));
	}


	void despawn(){

		hinge_body_head.destroy();
		hinge_head_muzzle.destroy();
		hinge_earLeft_head.destroy();
		hinge_earRight_head.destroy();
		hinge_body_legFrontLeft.destroy();
		hinge_body_legFrontRight.destroy();
		hinge_body_legBackLeft.destroy();
		hinge_body_legBackRight.destroy();
		hinge_body_tail.destroy();

		chest.destroy();
		head.destroy();
		muzzle.destroy();
		earLeft.destroy();
		earRight.destroy();
		legFrontLeft.destroy();
		legFrontRight.destroy();
		legBackLeft.destroy();
		legBackRight.destroy();
		tail.destroy();
	}
}


//ApplicationInterface----------------------


extern (C) void init(){
	rt_init();
try{
	Random(unpredictableSeed);

	for(int i = 0; i < numofdog; i++){
		doglist ~= new dog(0, 1.5, -5*i, true);
	}
}
catch (Exception ex){
	writeln(ex.toString);
}
}


float topRecord = 0;
int timerDivisor = 0;
int time = 0;
int generation = 0;
int sequence = 0;
int maxSequence = 20;

extern (C) void tick(){
	if(timerDivisor++ == 6){
		
		sequence = (sequence+1)%20;
		timerDivisor = 0;

		foreach(elem; doglist){
			elem.move(sequence);
		}

		time++;

	}

	//世代終わり
	if(time == 30 + generation*2){
		timeFlag = true;

		foreach(ref elem; doglist){
			
			if(isNaN(elem.muzzle.getPos().x)){
				writeln("nan...だと...");
			}
			
			/*
			write(elem.muzzle.getPos().x);
			write(",");
			*/
		}
		writeln();

		//成績順にソート
		
		dog best = doglist[0];
		dog second = doglist[1];
		foreach(i; 0..numofdog-1){
			foreach(j; 1..numofdog-i){
				if(doglist[j].muzzle.getPos().x >= doglist[j-1].muzzle.getPos().x){
					dog temp = doglist[j];
					doglist[j] = doglist[j-1];
					doglist[j-1] = temp;
				}
			}
		}
		//doglist.sort!("a.muzzle.getPos().x >= b.muzzle.getPos().x");

		//優秀なDNAをコピー
		float[5][5] firstW12 = doglist[0].w12;
		float[5][5] secondW12 = doglist[1].w12;
		
		float[6] firstW23 = doglist[0].w23;
		float[6] secondW23 = doglist[1].w23;

		

		//新記録を更新したDNAを表示
		if(topRecord < doglist[0].muzzle.getPos().x){
			topRecord = doglist[0].muzzle.getPos().x;
			writeln("New Record!: " ~ to!string(topRecord));
			/*
			writeln("dna:");
			foreach(float[8] elem; doglist[0].dna){
				writeln(to!string(elem[0]) ~ ", " ~ to!string(elem[1]) ~ ", " ~ to!string(elem[2]) ~ ", " ~ to!string(elem[3]));
			}
			*/
		}

		//犬のリセット
		foreach(int i, ref elem; doglist){
			elem.despawn();
			elem = new dog(0, 1.5, -5*i, false);
		}
		
		


		//最初の2個体はさっきの優秀個体をそのまま動かす
		doglist[0].w12 = firstW12;
		doglist[1].w12 = secondW12;
		
		doglist[0].w23 = firstW23;
		doglist[1].w23 = secondW23;

		//残りの個体
		foreach(i; 2..numofdog){
			//交配
			foreach(uint j, ref elem; doglist[i].w12[]){
				if(uniform(0, 2, rnd) == 0){
					elem = firstW12[j];
				}else{
					elem = secondW12[j];
				}
			}
			
			for(int k=0; k<6; k++){
				if(uniform(0, 2, rnd) == 0){
					doglist[i].w23[k] = firstW23[k];
					}else{
						doglist[i].w23[k] = secondW23[k];		
					}
				}

			//突然変異
			int numOfAttack = uniform(0, 10, rnd);
				
			for(int j = 0; j < numOfAttack; j++){
				doglist[i].w12[uniform(0, 4, rnd)][uniform(0, 4, rnd)] = uniform(0.0, 0.3, rnd);
				doglist[i].w23[uniform(0, 5, rnd)] = uniform(0.0, 0.3, rnd);
			}

		}

		generation++;
		time = 0;

		writeln("generation: " ~ to!string(generation));
	}
}



//------------------------------------------

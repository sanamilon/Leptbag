import core.stdc.stdio;
import std.stdio;
import core.runtime;
import std.random;
import std.math;
import std.algorithm;
import std.conv;
import std.array;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;

dog[] doglist;
Random rnd;

const int numofdog = 100;
const int dnacol = 20;
const int dnarow = 4;

class dog{

	float[4][20] dna;

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
			for(int col = 0; col < dnacol; col++){
				for(int row = 0; row < dnarow; row++){
					dna[col][row] = uniform(-PI/2, PI/2, rnd);
				}
			}
		}

		spawn(x, y, z);
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


		hinge_body_legFrontLeft.setRotationalMotor(1);
		hinge_body_legFrontLeft.setMaxRotationalMotorForce(1, 1);
		hinge_body_legFrontRight.setRotationalMotor(1);
		hinge_body_legFrontRight.setMaxRotationalMotorForce(1, 1);
		hinge_body_legBackLeft.setRotationalMotor(1);
		hinge_body_legBackLeft.setMaxRotationalMotorForce(1, 1);
		hinge_body_legBackRight.setRotationalMotor(1);
		hinge_body_legBackRight.setMaxRotationalMotorForce(1, 1);

		hinge_body_legFrontLeft.setRotationalTargetVelocity(Vector3f(0, 0, 0));
		hinge_body_legFrontRight.setRotationalTargetVelocity(Vector3f(0, 0, 0));
		hinge_body_legBackLeft.setRotationalTargetVelocity(Vector3f(0, 0, 0));
		hinge_body_legBackRight.setRotationalTargetVelocity(Vector3f(0, 0, 0));
	}

	void move(int sequence){
		/*
		hinge_body_legFrontLeft.setRotationalTargetVelocity(dna[sequence][0], 0.3);
		hinge_body_legFrontRight.setRotationalTargetVelocity(dna[sequence][1], 0.3);
		hinge_body_legBackLeft.setRotationalTargetVelocity(dna[sequence][2], 0.3);
		hinge_body_legBackRight.setRotationalTargetVelocity(dna[sequence][3], 0.3);
		*/
		hinge_body_legFrontLeft.setRotationalTargetVelocity(Vector3f(0, dna[sequence][0], 0));
		hinge_body_legFrontRight.setRotationalTargetVelocity(Vector3f(0, dna[sequence][1], 0));
		hinge_body_legBackLeft.setRotationalTargetVelocity(Vector3f(0, dna[sequence][2], 0));
		hinge_body_legBackRight.setRotationalTargetVelocity(Vector3f(0, dna[sequence][3], 0));
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

		foreach(ref elem; doglist){
			if(isNaN(elem.muzzle.getPos().x)){
				writeln("nan...だと...");
			}
		}
		writeln();

		//成績順にソート
		doglist.sort!("a.muzzle.getPos().x >= b.muzzle.getPos().x");

		//優秀なDNAをコピー
		float[4][20] firstDNA = doglist[0].dna;
		float[4][20] secondDNA = doglist[1].dna;

		//新記録を更新したDNAを表示
		if(topRecord < doglist[0].muzzle.getPos().x){
			topRecord = doglist[0].muzzle.getPos().x;
			writeln("New Record!: " ~ to!string(topRecord));
			writeln("dna:");
			foreach(float[4] elem; doglist[0].dna){
				writeln(to!string(elem[0]) ~ ", " ~ to!string(elem[1]) ~ ", " ~ to!string(elem[2]) ~ ", " ~ to!string(elem[3]));
			}
		}

		//犬のリセット
		foreach(int i, ref elem; doglist){
			elem.despawn();
			elem = new dog(0, 1.5, -5*i, false);
		}


		//最初の2個体はさっきの優秀個体をそのまま動かす
		doglist[0].dna = firstDNA;
		doglist[1].dna = secondDNA;

		//残りの個体
		foreach(i; 2..numofdog){
			//交配
			foreach(uint j, ref elem; doglist[i].dna[]){
				if(uniform(0, 2, rnd) == 0){
					elem = firstDNA[j];
				}else{
					elem = secondDNA[j];
				}
			}

			//突然変異
			int numOfAttack = uniform(0, 10, rnd);
				
			for(int j = 0; j < numOfAttack; j++){
				doglist[i].dna[uniform(0, dnacol, rnd)][uniform(0, dnarow, rnd)] = uniform(-PI/2, PI/2, rnd);
			}

		}

		generation++;
		time = 0;

		writeln("generation: " ~ to!string(generation));
	}
}



//------------------------------------------

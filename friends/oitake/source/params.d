import core.runtime;
import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;

//import Oscillator;


struct agentBodyParameter{
	elementManager[string] partsGenerator;
	partParam[string] partParams; //身体パーツのパラメータ
	g6dofParam[string] g6dofParams; //g6dofのパラメータ
}

//身体パーツパラメータ
struct partParam{

	vertexManager vertices;
	Vector3f position;
	Vector3f scale;
	Quaternionf rotation;
	float mass; //総体重に対する百分率
	float friction; //摩擦係数

}



//ヒンジパラメータ
struct hingeParam{

	string name;
	Vector3f position;
	Vector3f axis1;
	Vector3f axis2;
	string object1Name;
	string object2Name;
	Vector3f object1Position;
	Vector3f object2Position;
	bool enabled;
	bool useLimit;
	float limitLower;
	float limitUpper;


}

//g6dofパラメータ
struct g6dofParam{

	string name;
	bool enabled;
	Vector3f position;
	Quaternionf rotation;
	string object1Name;
	string object2Name;
	Vector3f object1Position;
	Vector3f object2Position;
	bool[3] useAngLimit; //(x, y, z) : ( 0, 1, 2 )
	Vector3f angLimitLower;
	Vector3f angLimitUpper;
	bool[3] useLinLimit;
	Vector3f linLimitLower;
	Vector3f linLimitUpper;

}


//遺伝させるパラメータ
struct serialOrderGene{

	static uint lengthOfSet = 8;
	Vector3f[string][] tracks;
	bool[][] wavelengthOfOrder;
	float friction;
	float maxRotationalMotorForce;
	Vector3f[string][] maxVelocity;

	void init(){

		auto rnd = Random(unpredictableSeed);

		tracks.length = lengthOfSet;
		maxVelocity.length = lengthOfSet;

		this.initFriction();
		this.initMaxRotationalMotorForce();


		wavelengthOfOrder.length = lengthOfSet;
		for(int i=0; i<wavelengthOfOrder.length; i++){

			wavelengthOfOrder[i].length = uniform(1, 10, rnd);

			for(int j=0; j<wavelengthOfOrder[i].length; j++){
				if(uniform(0.0f, 1.0f, rnd) < 0.5f) wavelengthOfOrder[i][j] = true;
				else wavelengthOfOrder[i][j] = false;
			}

		}

	}


	void init(string s, Vector3f lowerLimit, Vector3f upperLimit){

		auto rnd = Random(unpredictableSeed);

		for(int i=0; i<lengthOfSet; i++){

			float x, y, z;
			if(lowerLimit.x<upperLimit.x){
				x = uniform(lowerLimit.x, upperLimit.x, rnd);
			}else{
				x = 0.0f;
			}

			if(lowerLimit.y<upperLimit.y){
				y = uniform(lowerLimit.y, upperLimit.y, rnd);
			}else{
				y = 0.0f;
			}

			if(lowerLimit.z<upperLimit.z){
				z = uniform(lowerLimit.z, upperLimit.z, rnd);
			}else{
				z = 0.0f;
			}

			tracks[i][s] = Vector3f(x, y, z);
			//write(s, ":", i, "(", tracks[i][s].x, ", ", tracks[i][s].y, ")");
		}

		for(int i=0; i<lengthOfSet; i++){

			float x, y, z;
			if(lowerLimit.x<upperLimit.x){
				x = uniform(0.0f, 20.0f, rnd);
			}else{
				x = 0.0f;
			}

			if(lowerLimit.y<upperLimit.y){
				y = uniform(0.0f, 20.0f, rnd);
			}else{
				y = 0.0f;
			}

			if(lowerLimit.z<upperLimit.z){
				z = uniform(0.0f, 20.0f, rnd);
			}else{
				z = 0.0f;
			}

			maxVelocity[i][s] = Vector3f(x, y, z);
			//write(s, ":", i, "(", tracks[i][s].x, ", ", tracks[i][s].y, ")");
		}

	}

	void init(int i, string s, Vector3f lowerLimit, Vector3f upperLimit){
		auto rnd = Random(unpredictableSeed);

			float x, y, z;
			if(lowerLimit.x<upperLimit.x){
				x = uniform(lowerLimit.x, upperLimit.x, rnd);
			}else{
				x = 0.0f;
			}

			if(lowerLimit.y<upperLimit.y){
				y = uniform(lowerLimit.y, upperLimit.y, rnd);
			}else{
				y = 0.0f;
			}

			if(lowerLimit.z<upperLimit.z){
				z = uniform(lowerLimit.z, upperLimit.z, rnd);
			}else{
				z = 0.0f;
			}

			tracks[i][s] = Vector3f(x, y, z);

	}

	void initFriction(){
		auto rnd = Random(unpredictableSeed);
		this.friction = uniform(0.1f, 6.0f, rnd);
	}

	void initMaxRotationalMotorForce(){
		auto rnd = Random(unpredictableSeed);
		maxRotationalMotorForce = uniform(0.0f, 20.0f, rnd);
	}

	void copytracks(serialOrderGene u){
		foreach(int i, elem1; this.tracks){
			foreach(string s, elem2; elem1){
				this.tracks[i][s] = u.tracks[i][s];
			}
		}
	}

	void copytracks(serialOrderGene u,int i,string s){
		this.tracks[i][s] = u.tracks[i][s];
	}


}



//========未整備==========
/+
struct oscillator2Gene{

	Vector3f[string] angLimitLower;
	Vector3f[string] angLimitUpper;
	float friction;
	Vector3f[string] maxForce; //最大出力．いくらmaxVeloを大きくしてもこれ以上の力では駆動しない．
	Vector3f[string] maxVelo; //g6dofを動かす最高速
	oscillator2 oscil; //振動子モデル．1個体に1つ．
	int degree; //振動子モデルの近似精度(sin(nx), cos(nx)のn)

	//関節間で共通するパラメータの初期化
	void init(){
		degree = 5;
		oscil = new oscillator2(degree);
		auto rnd = Random(unpredictableSeed);
		friction = 2.0f;//uniform(0.0f, 5.0f, rnd);
	}


	//各関節で異なるパラメータの初期化
	//blenderで定義した関節制限角度を用いない場合
	void init(string s){
		auto rnd = Random(unpredictableSeed);

		maxForce[s] = Vector3f( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), 0.0f );
		maxVelo[s] = Vector3f( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd) );

		oscil.init(s);

		angLimitUpper[s] = Vector3f( uniform(0.0f, 1.57f, rnd), uniform(0.0f, 1.57f, rnd), 0.0f );
		angLimitLower[s] = Vector3f( uniform(-1.57f, 0.0f, rnd), uniform(-1.57f, 0.0f, rnd), 0.0f );

	}

	//各関節で異なるパラメータの初期化
	//関節角度をblenderから読込む場合
	void init(string s, g6dofParam dofParam){

		auto rnd = Random(unpredictableSeed);
		maxForce[s] = Vector3f( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), 0.0f );
		maxVelo[s] = Vector3f( uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd), uniform(0.0f, 10.0f, rnd) );

		oscil.init(s);

		angLimitLower[s] = dofParam.angLimitLower;
		angLimitUpper[s] = dofParam.angLimitUpper;

	}

	void rehash(){
		angLimitLower.rehash;
		angLimitUpper.rehash;
		maxForce.rehash;
		maxVelo.rehash;
	}

	//表示関数．oscilのtoString()は実装してない．
	void toString(){

		write("angLimitLower : ");
		foreach(string s, elem; angLimitLower){
			write("[ \"", s, "\": ", elem.x, ", ", elem.y, ", ", elem.z, " ], ");
		}
		writeln("");

		write("angLimitUpper : ");
		foreach(string s, elem; angLimitUpper){
			write("[ \"", s, "\": ", elem.x, ", ", elem.y, ", ", elem.z, " ], ");
		}
		writeln("");

		writeln("friction = ", friction);

		write("maxForce : ");
		foreach(string s, elem; maxForce){
			write("[ \"", s, "\": ", elem.x, ", ", elem.y, ", ", elem.z, " ], ");
		}
		writeln("");

		write("maxVelo : ");
		foreach(string s, elem; maxVelo){
			write("[ \"", s, "\": ", elem.x, ", ", elem.y, ", ", elem.z, " ], ");
		}
		writeln("");

		//oscil.toString();
		writeln("degree = ", degree);


	}


}
+/

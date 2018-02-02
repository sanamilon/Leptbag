import core.runtime;
import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;
import std.array;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;

import agent;
import DEforSOG;
import DEforPOG;
import params;
import loadJson;


const int agentNum = 50;
const int averageOf = 3; //一世代averageOf回の試行を行いその平均をスコアとする
const float bodyMass = 30.0f; //動物の総体重．blender側では各パーツに百分率で質量を付与．
const float personalSpace = 5.0f; //動物を並べる間隔
const string measuredPart = "head"; //この名前のパーツの移動距離を測る

agent[] agents; //メインとする群
agent[] evaluateds; //DEにおける突然変異個体

agentBodyParameter info;

//ApplicationInterface----------------------


//initialize--------------------------------

extern (C) void init(){
	rt_init();
	Random(unpredictableSeed);
	writeln("start initialization model:oitake");

	//jsonからload
	loadMesh(info.partParams);
	loadG6dof(info.g6dofParams);
	writeln("loaded data from .json");
	info.partParams = info.partParams.rehash;
	info.g6dofParams = info.g6dofParams.rehash;

	//agents生成
	agents.length = agentNum*averageOf;
	agent.registerParameter(info);
	prepareAgentsGroup(agents, info);
	writeln("made main groups of ", averageOf, " (", agentNum, " agents in each group)");

	agent.shareGeneAmongGroup(agents, agentNum, averageOf);
	writeln("shared gene among main groups");

	writeln("start simulation");
}

void prepareAgentsGroup(agent[] group, agentBodyParameter information){
	//group.length = agentNum*averageOf;


	for(int i=0; i<averageOf; i++){
		for(int j=0; j<agentNum; j++){
			group[j + i*agentNum]
				= new agent(to!float(j)*personalSpace, 0.0f, -10.0f + to!float(i)*personalSpace, measuredPart);
		}
	}

}


//毎ステップ実行される--------------------

float topScore = -1000.0f; //動物たちは-z方向に歩いていることに注意
//そのステップ内で行うべき処理を決定するための変数
int time = 0; //時計
int generation = 0; //世代を記録する
bool evaluation = false; //trueならDEの突然変異体評価フェイズ

const int generationStroke = 0; //一世代毎にgenerationStrokeだけ長い時間の試行を行うようになる
const int trialSpan = 500; //一試行の長さ


float Cr = 0.9f; //Crの確率で親の遺伝子を引き継ぐ
float coinForRandomMutation = 0.1f; //遺伝子要素がランダムに突然変異．

int clock = 0;
extern (C) void tick(){


	time++;

	/+
	if(time%50==0){
		if(!evaluation){
			write(agents[0].parts["head"].getRotationAngle(), ", ");
			write(agents[0].parts["head"].getRotation(), ", ");
			write(agents[0].gravityDirection);
			writeln(", ", agents[0].eyeDirection);
		}else{
			write(evaluateds[0].parts["head"].getRotationAngle(), ", ");
			write(evaluateds[0].parts["head"].getRotation(), ", ");
			write(evaluateds[0].parts["head"].getRotation().conjugate().rotate(evaluateds[0].initialGravityDirection).normalized());
			writeln(", ", evaluateds[0].parts["head"].getRotation().rotate(evaluateds[0].initialEyeDirection).normalized());
		}

	}
	+/



	if(time%2==0){
		//運動する
		updateAgentsClock();
		//writeln("clock : ", agents[0].biologicalClock);
		//writeln("sequence : ", agents[0].sequenceOfOrder);
	}

	if(time%12==0){
		if(!evaluation){

		}
		//運動する
		moveAgents();
	}

	//一世代終了
	if( time == (trialSpan + generation*generationStroke) ){

		writeln();

		time = 0;
		terminateTrial();
	}

}


//----------------------------------------

void updateAgentsClock(){

	//writeln(seq);
	if(!evaluation){
		foreach(elem; agents){
			elem.updateBiologicalClock();
		}
	}else{
		foreach(elem; evaluateds){
			elem.updateBiologicalClock();
		}
	}

}

void moveAgents(){

	//writeln(seq);
	if(!evaluation){
		foreach(elem; agents){
			//elem.moveWithSerialOrder();
			elem.moveWithPhaseOscillator();
		}
	}else{
		foreach(elem; evaluateds){
			//elem.moveWithSerialOrder();
			elem.moveWithPhaseOscillator();
		}
	}

}


//一試行が終わるたびに実行する処理
void terminateTrial(){

	Vector3f preTopScoreTmp = Vector3f(0.0f, 0.0f, 10000.0f);
	float proScoreTmp = -1000.0f; //この世代の最高移動距離
	float[] averageScore;
	averageScore.length = averageOf;
	averageScore[] = 0.0f;


	if(!evaluation){ //各個体の移動距離を測るフェイズ

		foreach(int i, ref elem; agents){

			agents[i].addCurrentPosition(measuredPart);
			agents[i].absScore(true, false, false);

			proScoreTmp = max( -1.0*agents[i].score.z, proScoreTmp );

			if(agents[i].score.z < preTopScoreTmp.z){
				preTopScoreTmp = agents[i].score;
			}

		}

		displayGenerationResult(agents, preTopScoreTmp);

	}else{ //突然変異体評価フェイズ

		//evaluateds[0].gene.toString();

		foreach(int i, ref elem; evaluateds){

			
			evaluateds[i].addCurrentPosition(measuredPart);
			evaluateds[i].absScore(true, false, false);

			proScoreTmp = max( -1.0*evaluateds[i].score.z, proScoreTmp );

			if(evaluateds[i].score.z < preTopScoreTmp.z){
				preTopScoreTmp = evaluateds[i].score;
			}

		}

		displayGenerationResult(evaluateds, preTopScoreTmp);


	}



	terminateGeneration();

}

//今世代の結果を表示
void displayGenerationResult(agent[] group, Vector3f pretopscoretmp){

	//今回の世代の最高記録
	writeln("	top proceeding of this generation : ", pretopscoretmp);

	writeln("\taverage scores at each trial");
	writeln("\t", culculateAverage(agents, agentNum, averageOf) );

	//最高記録が出たら記録，表示
	if(-1.0*pretopscoretmp.z>topScore){
		topScore = -1.0*pretopscoretmp.z;
		writeln("!	top proceeding ever! : ", topScore);
	}


	for(int i=0; i<0; i++){
		writeln("agents[", i, "].score.z : ", agents[i].score.z);
	}

	writeln();

	/+
	for(int i=0; i<averageOf; i++){
		write("group", i, "[ ");
		for(int j=0; j<agentNum; j++){
			write(agents[j+i*agentNum].score.z, ", ");
		}
		writeln("]");
	}
	+/

}


//世代の終了時処理
void terminateGeneration(){


	if(!evaluation){ //評価フェイズ
		writeln("	start evaluation ", generation, ":");
	}else{
		writeln("start generation ", ++generation, ": ---------------------------------");
	}


	if(!evaluation){ //各個体の移動距離を測るフェイズ


		Vector3f[] scores = agent.sumScoreOnIndividual(agents, agentNum, averageOf);

		//agentsは一旦退場
		foreach(int i,ref elem; agents){
			elem.despawn();
		}

		float[] value = agent.culculateValueOnProceed(scores);
		int[] bests = agent.chooseBest(value);

		if(generation==0){ //最初に評価用の犬たちevaluatedsをつくる

			evaluateds.length = agentNum*averageOf;

			prepareAgentsGroup(evaluateds, info);

			foreach(int i, ref elem; evaluateds){
				elem.copyGene(agents[i]);
			}

			writeln("made groups for evaluation to mutation");
			writeln("copied gene of main group to group for evaluation");

		}else{ //1世代以降

			auto rnd = Random(unpredictableSeed);
			//evaluatedsをpop
			for(int i=0; i<averageOf; i++){
				for(int j=0; j<agentNum; j++){
					Vector3f spawnPosition = Vector3f(to!float(j)*personalSpace , 0.0f, -10.0f + to!float(i)*personalSpace);
					Vector3f blur = Vector3f( uniform(-0.1f, 0.1f, rnd), uniform(-0.1f, 0.1f, rnd), uniform(-0.1f, 0.1f, rnd) );
					spawnPosition = spawnPosition + blur;
					evaluateds[j + i*agentNum].spawn(spawnPosition, measuredPart);
				}
			}

		}

		//DEに用いるパラメータ
		auto rnd = Random(unpredictableSeed);
		float ditherF = uniform(0.0f, 0.5f, rnd);
		//突然変異
		//evolveSOG(agentNum, evaluateds[0..agentNum], agents[0..agentNum], coinForRandomMutation, Cr, ditherF, bests);
		evolvePOG(agentNum, evaluateds[0..agentNum], agents[0..agentNum], coinForRandomMutation, Cr, ditherF, bests);
		//evolvePOG(agentNum, evaluateds[0..agentNum], agents[0..agentNum], coinForRandomMutation, Cr, ditherF);

		agent.shareGeneAmongGroup(evaluateds, agentNum, averageOf);

		evaluation = true; //次は突然変異体評価

	}else{ //突然変異体を評価する

		agent.evaluateEvolutionOnProceed(agents, evaluateds, agentNum, averageOf);

		auto rnd = Random(unpredictableSeed);
		//突然変異体は一旦退場
		foreach(int i, ref elem; evaluateds) elem.despawn();
			for(int i=0; i<averageOf; i++){
				for(int j=0; j<agentNum; j++){
					Vector3f spawnPosition = Vector3f(to!float(j)*personalSpace , 0.0f, -10.0f + to!float(i)*personalSpace);
					Vector3f blur = Vector3f( uniform(-0.1f, 0.1f, rnd), uniform(-0.1f, 0.1f, rnd), uniform(-0.1f, 0.1f, rnd) );
					spawnPosition = spawnPosition + blur;
					agents[j + i*agentNum].spawn(spawnPosition, measuredPart);
				}
			}

		evaluation = false; //次は採用した突然変異体を混ぜて性能評価

	}


}

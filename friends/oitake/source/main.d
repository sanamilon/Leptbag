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
import params;
import loadJson;


const int agentNum = 60;
const int averageOf = 3; //一世代averageOf回の試行を行いその平均をスコアとする
const float bodyMass = 10.0f; //動物の総体重．blender側では各パーツに百分率で質量を付与．
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

	prepareAgentsGroup(agents, info);

	writeln("made main groups of ", averageOf, " (", agentNum, " agents in each group)");


	agent.shareGeneAmongGroup(agents, agentNum, averageOf);

	writeln("shared gene among main groups");

	writeln("start simulation");

}

void prepareAgentsGroup(agent[] group, agentBodyParameter information){
	//group.length = agentNum*averageOf;

	agent.registerParameter(information);

	foreach(int i, ref elem; group){
		group[i] = new agent(to!float(i)*personalSpace, 0.0f, -1.0f, measuredPart);
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
float coinForRandomMutation = 0.1f; //(1.0-Cr)*coinForRandomMutationの確率で遺伝子要素がランダムに突然変異．

extern (C) void tick(){

	time++;

	if(time%2==0){
		//運動する
		updateAgentsClock();
		//writeln("clock : ", agents[0].biologicalClock);
		//writeln("sequence : ", agents[0].sequenceOfOrder);
	}

	if(time%12==0){
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
			elem.moveWithSerialOrder();
		}
	}else{
		foreach(elem; evaluateds){
			elem.moveWithSerialOrder();
		}
	}

}


//一試行が終わるたびに実行する処理
void terminateTrial(){


	float proScoreTmp = -1000.0f; //この世代の最高移動距離
	float[] averageScore;
	averageScore.length = averageOf;
	averageScore[] = 0.0f;


	if(!evaluation){ //各個体の移動距離を測るフェイズ

		foreach(int i, ref elem; agents){

			agents[i].addCurrentPosition(measuredPart);
			agents[i].absScore(true, false, false);

			proScoreTmp = max( -1.0*agents[i].score.z, proScoreTmp );

		}

		displayGenerationResult(agents, proScoreTmp);

	}else{ //突然変異体評価フェイズ

		//evaluateds[0].gene.toString();

		foreach(int i, ref elem; evaluateds){

			
			evaluateds[i].addCurrentPosition(measuredPart);
			evaluateds[i].absScore(true, false, false);

			proScoreTmp = max( -1.0*evaluateds[i].score.z, proScoreTmp );

		}

		displayGenerationResult(evaluateds, proScoreTmp);


	}

	terminateGeneration();

}

//今世代の結果を表示
void displayGenerationResult(agent[] group, float proscoretmp){

	//今回の世代の最高記録
	writeln("	top proceeding of this generation : ", proscoretmp);

	writeln("average scores at each trial");
	writeln( culculateAverage(agents, agentNum, averageOf) );

	//最高記録が出たら記録，表示
	if(proscoretmp>topScore){
		topScore = proscoretmp;
		writeln("!	top proceeding ever! : ", topScore);
	}


	for(int i=0; i<5; i++){
		writeln("agents[", i, "].score.z : ", agents[i].score.z);
	}

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
		foreach(int i, elem; agents){
			elem.despawn();
		}

		float[] value = culculateValue(scores);
		int[] bests = chooseBest(value);

		if(generation==0){ //最初に評価用の犬たちevaluatedsをつくる

			evaluateds.length = agentNum*averageOf;
			foreach(int i, ref elem; evaluateds){
				elem = new agent(to!float(i)*personalSpace, 0.0f, 0.0f, measuredPart);
				elem.copyGene(agents[i]);
			}

			writeln("made groups for evaluation to mutation");
			writeln("copied gene of main group to group for evaluation");

		}else{ //1世代以降

			//evaluatedsをpop
			foreach(int i, ref elem; evaluateds){
				elem.spawn(Vector3f(to!float(i)*personalSpace, 0.0f, 0.0f), measuredPart);
			}

		}

		//DEに用いるパラメータ
		auto rnd = Random(unpredictableSeed);
		float ditherF = uniform(0.0f, 0.5f, rnd);
		//突然変異
		evolveSOG(agentNum, evaluateds[0..agentNum], agents[0..agentNum], coinForRandomMutation, Cr, ditherF, bests);

		agent.shareGeneAmongGroup(evaluateds, agentNum, averageOf);

		evaluation = true; //次は突然変異体評価フェイズ

	}else{ //突然変異体を評価するフェイズ

		agent.evaluateEvolution(agents, evaluateds, agentNum, averageOf);

		//突然変異体は一旦退場
		foreach(int i, ref elem; evaluateds) elem.despawn();
		foreach(int i, ref elem; agents) elem.spawn(Vector3f(to!float(i)*personalSpace, 0.0f, 0.0f), measuredPart);

		evaluation = false; //次は採用した突然変異体を混ぜて性能評価

	}


}

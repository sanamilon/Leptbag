import std.stdio;
import std.math;
import std.json;
import std.random;
import std.conv;
import std.algorithm;

import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;

import agent;
//import DEforOscillator2;
//import Oscillator;
import params;

Random rnd;

void simpleSOG(agent[] children, agent[] parents, float Cr, float F, int[] bests){

	auto rnd = Random(unpredictableSeed);

	int k = bests[0];
	int l = bests[1];
	int m = bests[2];

	foreach(int j, child; children){

		foreach(string s, dof; child.g6dofs){
			for(uint i=0; i<child.SOG.tracks.length; i++){

				float coin = uniform(0.0f, 1.0f, rnd);
				//if((j==0)&&(s=="Constraint")) writeln("no. ", i , " : coin is ", coin);
				if(0.4 > coin){
					//if((j==0)&&(s=="Constraint")) writeln("use first gene");
					child.SOG.tracks[i][s] = parents[k].SOG.tracks[i][s];
				}else if(0.8 > coin){
					//if((j==0)&&(s=="Constraint")) writeln("use second gene");
					child.SOG.tracks[i][s] = parents[l].SOG.tracks[i][s];
				}else{
					/+
					if((j==0)&&(s=="Constraint")) writeln("before mutation");
					if((j==0)&&(s=="Constraint")) writeln(child.SOG.tracks[i][s].getx());
					+/

					child.SOG.init(s, child.bodyInformation.g6dofParams[s].angLimitLower, child.bodyInformation.g6dofParams[s].angLimitUpper);
					/+
					if((j==0)&&(s=="Constraint")) writeln("after mutation");
					if((j==0)&&(s=="Constraint")) writeln(child.SOG.tracks[i][s].getx());
					+/
				}


			}
		}
	}

	/+
	writeln("children[0]");
	children[0].checkSOG();
	writeln("best parents");
	parents[bests[0]].checkSOG();
	+/


}


//rand
void evolveSOG(int agentNum, agent[] children, agent[] parents, float coin, float Cr, float F){

	auto rnd = Random(unpredictableSeed);

	foreach(int j, child; children){

		int k = uniform(0, agentNum, rnd);
		int l = uniform(0, agentNum, rnd);
		int m = uniform(0, agentNum, rnd);

		//tracks(命令セット)
		foreach(string s, dof; child.g6dofs){
			for(uint i=0; i<child.SOG.tracks.length; i++){

				if(Cr > uniform(0.0f, 1.0f, rnd)){
					child.SOG.tracks[i][s] = parents[k].SOG.tracks[i][s] + F * ( parents[m].SOG.tracks[i][s] - parents[l].SOG.tracks[i][s] );
				}else{

					if(coin > uniform(0.0f, 1.0f, rnd)){
						child.SOG.init(i, s, child.bodyInformation.g6dofParams[s].angLimitLower, child.bodyInformation.g6dofParams[s].angLimitUpper);
					}else{
						child.SOG.tracks[i][s] = parents[j].SOG.tracks[i][s];
					}

				}

			}


		}


		//friction
		if(Cr > uniform(0.0f, 1.0f, rnd)){
			child.SOG.friction = parents[k].SOG.friction + F * ( parents[m].SOG.friction - parents[l].SOG.friction );
		}else{

			if(coin > uniform(0.0f, 1.0f, rnd)){
				child.SOG.initFriction();
			}else{
				child.SOG.friction = parents[j].SOG.friction;
			}

		}

		//maxRotationalMotorForce
		if(Cr > uniform(0.0f, 1.0f, rnd)){
			child.SOG.maxRotationalMotorForce = parents[k].SOG.maxRotationalMotorForce + F * ( parents[m].SOG.maxRotationalMotorForce - parents[l].SOG.maxRotationalMotorForce );
		}else{

			if(coin > uniform(0.0f, 1.0f, rnd)){
				child.SOG.initMaxRotationalMotorForce();

			}else{
				child.SOG.maxRotationalMotorForce = parents[j].SOG.maxRotationalMotorForce;
			}

		}

	}



}


//best
void evolveSOG(int agentNum, agent[] children, agent[] parents, float coin, float Cr, float F, int[] bests){

	auto rnd = Random(unpredictableSeed);

	int k = bests[0];
	int l = bests[1];
	int m = bests[2];

	foreach(int j, child; children){

		//tracks(命令セット)
		for(uint i=0; i<child.SOG.tracks.length; i++){

			if(Cr > uniform(0.0f, 1.0f, rnd)){
				foreach(string s, dof; child.g6dofs){
					child.SOG.tracks[i][s] = parents[k].SOG.tracks[i][s] + F * ( parents[m].SOG.tracks[i][s] - parents[l].SOG.tracks[i][s] );
				}
			}else{
				if(coin > uniform(0.0f, 1.0f, rnd)){
					foreach(string s, dof; child.g6dofs){
						child.SOG.init(s, child.bodyInformation.g6dofParams[s].angLimitLower, child.bodyInformation.g6dofParams[s].angLimitUpper);
					}
				}else{
					foreach(string s, dof; child.g6dofs){
						child.SOG.tracks[i][s] = parents[j].SOG.tracks[i][s];
					}
				}

			}

			if(0.3333f > uniform(0.0f, 1.0f, rnd)){
				child.SOG.wavelengthOfOrder[i].length = parents[k].SOG.wavelengthOfOrder[i].length;
				child.SOG.wavelengthOfOrder[i] = parents[k].SOG.wavelengthOfOrder[i];
			}else if(0.66666f > uniform(0.0f, 1.0f, rnd)){
				child.SOG.wavelengthOfOrder[i].length = parents[l].SOG.wavelengthOfOrder[i].length;
				child.SOG.wavelengthOfOrder[i] = parents[l].SOG.wavelengthOfOrder[i];
			}else{
				child.SOG.wavelengthOfOrder[i].length = parents[m].SOG.wavelengthOfOrder[i].length;
				child.SOG.wavelengthOfOrder[i] = parents[m].SOG.wavelengthOfOrder[i];
			}

		}





		//friction
		if(Cr > uniform(0.0f, 1.0f, rnd)){
			child.SOG.friction = parents[k].SOG.friction + F * ( parents[m].SOG.friction - parents[l].SOG.friction );
		}else{

			if(coin > uniform(0.0f, 1.0f, rnd)){
				child.SOG.initFriction();
			}else{
				child.SOG.friction = parents[j].SOG.friction;
			}

		}

		//maxRotationalMotorForce
		if(Cr > uniform(0.0f, 1.0f, rnd)){
			child.SOG.maxRotationalMotorForce = parents[k].SOG.maxRotationalMotorForce + F * ( parents[m].SOG.maxRotationalMotorForce - parents[l].SOG.maxRotationalMotorForce );
		}else{

			if(coin > uniform(0.0f, 1.0f, rnd)){
				child.SOG.initMaxRotationalMotorForce();

			}else{
				child.SOG.maxRotationalMotorForce = parents[j].SOG.maxRotationalMotorForce;
			}

		}




	}


}

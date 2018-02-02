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
import params;

Random rnd;

//best
void evolvePOG(int agentNum, agent[] children, agent[] parents, float coin, float Cr, float F, int[] bests){

	auto rnd = Random(unpredictableSeed);

	int k = bests[0];
	int l = bests[1];
	int m = bests[2];


	foreach(int h, child; children){
		foreach(string s1, dof; child.g6dofs){
			for(uint i=0; i<3; i++){


				if(Cr >= uniform(0.0f, 1.0f, rnd)){
					child.POG[i].omega[s1] = parents[k].POG[i].omega[s1] + F*( parents[m].POG[i].omega[s1] - parents[l].POG[i].omega[s1] );
				}else{
					if(coin >= uniform(0.0f, 1.0f, rnd)){
						child.POG[i].initOmega(s1);
					}
				}

				foreach(string s2, alp; child.POG[i].alpha){

					if(s1!=s2){
						for(uint j=0; j<child.POG[i].degreeOfFourier; j++){

							if(Cr >= uniform(0.0f, 1.0f, rnd)){
								child.POG[i].alpha[s1][s2][j] =
									parents[k].POG[i].alpha[s1][s2][j]
									+ F*( parents[m].POG[i].alpha[s1][s2][j] - parents[l].POG[i].alpha[s1][s2][j] );
							}else{
								if(coin >= uniform(0.0f, 1.0f, rnd)){
									child.POG[i].initAlpha(s1, s2, j);
								}
							}

							if(Cr >= uniform(0.0f, 1.0f, rnd)){
								child.POG[i].beta[s1][s2][j] =
									parents[k].POG[i].beta[s1][s2][j]
									+ F*( parents[m].POG[i].beta[s1][s2][j] - parents[l].POG[i].beta[s1][s2][j] );
							}else{
								if(coin >= uniform(0.0f, 1.0f, rnd)){
									child.POG[i].initBeta(s1, s2, j);
								}
							}

						}
					}

				}


			}
		}
	}


}


//rand
void evolvePOG(int agentNum, agent[] children, agent[] parents, float coin, float Cr, float F){

	auto rnd = Random(unpredictableSeed);

	foreach(int h, child; children){
		foreach(string s1, dof; child.g6dofs){
			for(uint i=0; i<3; i++){

				int k = uniform(0, agentNum, rnd);
				int l = uniform(0, agentNum, rnd);
				int m = uniform(0, agentNum, rnd);

				if(Cr >= uniform(0.0f, 1.0f, rnd)){
					child.POG[i].omega[s1] = parents[k].POG[i].omega[s1] + F*( parents[m].POG[i].omega[s1] - parents[l].POG[i].omega[s1] );
				}else{
					/+
						child.POG[i].omega[s1] = parents[h].POG[i].omega[s1];
					+/
				}

				foreach(string s2, alp; child.POG[i].alpha){

					if(s1!=s2){
						for(uint j=0; j<child.POG[i].degreeOfFourier; j++){

							if(Cr >= uniform(0.0f, 1.0f, rnd)){
								child.POG[i].alpha[s1][s2][j] =
									parents[k].POG[i].alpha[s1][s2][j]
									+ F*( parents[m].POG[i].alpha[s1][s2][j] - parents[l].POG[i].alpha[s1][s2][j] );
							}else{
								/+
									child.POG[i].alpha[s1][s2][j] = parents[h].POG[i].alpha[s1][s2][j];
								+/
							}

							if(Cr >= uniform(0.0f, 1.0f, rnd)){
								child.POG[i].beta[s1][s2][j] =
									parents[k].POG[i].beta[s1][s2][j]
									+ F*( parents[m].POG[i].beta[s1][s2][j] - parents[l].POG[i].beta[s1][s2][j] );
							}else{
								/+
									child.POG[i].beta[s1][s2][j] = parents[h].POG[i].beta[s1][s2][j];
								+/
							}

						}
					}

				}


			}
		}
	}


}






//
void simplePOG(agent[] children, agent[] parents, float Cr, float F, int[] bests){

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



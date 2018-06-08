import std.stdio;
import std.math;
import std.random;
import std.conv;
import std.algorithm;

import dlib.math.vector;
import dlib.math.quaternion;


class sepCMAES(int dim, int nsample){

	float function(float[] x) func;
	Vector!(float, dim) mean;
	float sigma;
	int N;
	float[][] arx;
	Vector!(float, nsample) arf;
	Vector!(float, nsample) weights;

	//for CSA
	Vector!(float, dim) ps;
	float cs;
	float ds;
	float chiN;
	float mueff;

	this(float function(float[] x) func, float[] init_mean, float init_sigma, int nsample){
		this.func = func;
		this.mean = init_mean;
		this.sigma = init_sigma;
		this.N = to!int(this.mean.length);
		weights.length = nsample;
		for(int i=0; i<nsample; i++){
			this.weights[i] = 0.0;
		}
		weights[0..to!int(nsample/4)] = 1/to!int(nsample/4);
	}

	void test(float[] x){
		writeln(func(x));
	}

};

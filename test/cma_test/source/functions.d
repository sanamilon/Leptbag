import std.stdio;
import std.math;
import std.random;
import std.conv;
import std.algorithm;

import dlib.math.vector;
import dlib.math.quaternion;

float sphere(T, int size)(Vector!(T, size) x){
	return x.lengthsqr();
}

float ellipsoid(T, int size)(Vector!(T, size) x){
	int condition = 2;
	float value = 0;
	
	for(int i=0; i<size; i++){
		value += x[i]*x[i]*pow(10, condition*float(i)/float(size-1));
	}

	return sqrt(value);
}

float ellipsoid(T, int size)(Vector!(T, size) x, int condition){
	float value = 0;
	
	for(int i=0; i<size; i++){
		value += x[i]*x[i]*pow(10, condition*float(i)/float(size-1));
	}

	return sqrt(value);
}

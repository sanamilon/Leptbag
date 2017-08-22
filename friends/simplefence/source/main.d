import core.stdc.stdio;
import std.stdio;
import core.runtime;
import std.random;
import std.math;
import std.algorithm;


import japariSDK.japarilib;
import dlib.math.vector;
import dlib.math.quaternion;


//ApplicationInterface----------------------

extern (C) void init(){
	rt_init();

	getCubeshape().generate(paramWrap(
				param("position", Vector3f(50.25, 0.25, 0)),
				param("scale", Vector3f(0.25, 0.25, 50)),
				param("rotation", Quaternionf(0, 0, 0, 1)),
				param("mass", 100.0f)));

	getCubeshape().generate(paramWrap(
				param("position", Vector3f(-50.25, 0.25, 0)),
				param("scale", Vector3f(0.25, 0.25, 50)),
				param("rotation", Quaternionf(0, 0, 0, 1)),
				param("mass", 100.0f)));

	getCubeshape().generate(paramWrap(
				param("position", Vector3f(0, 0.25, 50.25)),
				param("scale", Vector3f(50, 0.25, 0.25)),
				param("rotation", Quaternionf(0, 0, 0, 1)),
				param("mass", 100.0f)));

	getCubeshape().generate(paramWrap(
				param("position", Vector3f(0, 0.25, -50.25)),
				param("scale", Vector3f(50, 0.25, 0.25)),
				param("rotation", Quaternionf(0, 0, 0, 1)),
				param("mass", 100.0f)));

	getCubeshape().generate(paramWrap(
				param("position", Vector3f(50.25, 0.25, 50.25)),
				param("scale", Vector3f(0.25, 0.25, 0.25)),
				param("rotation", Quaternionf(0, 0, 0, 1)),
				param("mass", 100.0f)));

	getCubeshape().generate(paramWrap(
				param("position", Vector3f(50.25, 0.25, -50.25)),
				param("scale", Vector3f(0.25, 0.25, 0.25)),
				param("rotation", Quaternionf(0, 0, 0, 1)),
				param("mass", 100.0f)));

	getCubeshape().generate(paramWrap(
				param("position", Vector3f(-50.25, 0.25, 50.25)),
				param("scale", Vector3f(0.25, 0.25, 0.25)),
				param("rotation", Quaternionf(0, 0, 0, 1)),
				param("mass", 100.0f)));

	getCubeshape().generate(paramWrap(
				param("position", Vector3f(-50.25, 0.25, -50.25)),
				param("scale", Vector3f(0.25, 0.25, 0.25)),
				param("rotation", Quaternionf(0, 0, 0, 1)),
				param("mass", 100.0f)));

}


extern (C) void tick(){
}



//------------------------------------------
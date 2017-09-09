#version 330 core

// 入力頂点データ。このシェーダのすべての実行で異なる
layout(location = 0) in vec3 vertexPosition_modelspace;

// メッシュ全体で固定した値
uniform mat4 depthMVP;

void main(){
	gl_Position =  depthMVP * vec4(vertexPosition_modelspace,1);
}

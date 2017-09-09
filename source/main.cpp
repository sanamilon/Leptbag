#include <iostream>
#include <vector>
#include <random>
#include <string>
#include <sstream>
#include <cstdlib>

#include <stdio.h>
#include <stdlib.h>
#include <dlfcn.h>
#include <dirent.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <bullet/btBulletDynamicsCommon.h>


#include "vertexmanager.hpp"
#include "shader.hpp"
#include "constraints.hpp"
#include "utilities/utilities.hpp"
#include "elementNode.hpp"
#include "elementManager.hpp"
#include "bodyGenerator.hpp"
#include "primitiveShape.hpp"


GLFWwindow* window;

//ウィンドウの大きさ
GLint windowWidth  = 1000;
GLint windowHeight = 800;

//半分の大きさを定義しておく。ポインタを固定する位置に使う。
GLint midWindowX = windowWidth  / 2;
GLint midWindowY = windowHeight / 2;


glm::mat4 ViewMatrix;
glm::mat4 ProjectionMatrix;

GLuint uniform_viewMatrix;
GLuint uniform_projectionMatrix;
GLuint uniform_LightColor;
GLuint uniform_LightPower;
GLuint uniform_LightDirection;
GLuint uniform_depthBiasVP;




btDiscreteDynamicsWorld* dynamicsWorld;


typedef void (*pluginfunc_t)();
std::vector<pluginfunc_t> pluginInitVector;
std::vector<pluginfunc_t> pluginTickVector;


//カメラの位置など
glm::vec3 position = glm::vec3( 0, 0, 0 ); 
double horizontalAngle = 3.14f;
double verticalAngle = 0.0f;

float initialFoV = 45.0f;

float speed = 0.1f;
float mouseSpeed = 0.001f;

glm::vec3 lightColor = glm::vec3(1, 1, 1);
float lightPower = 1.0f;
glm::vec3 lightDirection = glm::vec3(-1, 1, 0);



// Hoding any keys down?
bool holdingForward     = false;
bool holdingBackward    = false;
bool holdingLeftStrafe  = false;
bool holdingRightStrafe = false;

bool holdingSneek = false;
bool holdingSpace = false;

std::vector<std::string> split(const std::string &str, char sep){
	std::vector<std::string> v;
	std::stringstream ss(str);
	std::string buffer;
	while( std::getline(ss, buffer, sep) ) {
		v.push_back(buffer);
	}
	return v;
}


void computeMatricesFromInputs(){


	//カメラの向きを計算する
	glm::vec3 direction(
			cos(verticalAngle) * sin(horizontalAngle), 
			sin(verticalAngle),
			cos(verticalAngle) * cos(horizontalAngle)
			);


	//カメラ移動
	if (holdingForward == true){
		position[0] += sin(horizontalAngle)* speed;
		position[2] += cos(horizontalAngle)* speed;
	}

	if (holdingBackward == true){
		position[0] += sin(horizontalAngle+3.14)* speed;
		position[2] += cos(horizontalAngle+3.14)* speed;
	}

	if (holdingRightStrafe == true){
		position[0] += sin(horizontalAngle-(3.14/2))* speed;
		position[2] += cos(horizontalAngle-(3.14/2))* speed;
	}

	if (holdingLeftStrafe == true){
		position[0] += sin(horizontalAngle+(3.14/2)) * speed;
		position[2] += cos(horizontalAngle+(3.14/2)) * speed;
	}

	if (holdingSpace == true){
		position[1] += speed;
	}

	if (holdingSneek == true){
		position[1] -= speed;
	}


	float FoV = initialFoV;

	// Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	ProjectionMatrix = glm::perspective(FoV, (float)windowWidth/(float)windowHeight, 0.1f, 300.0f);

	// Camera matrix
	ViewMatrix = glm::lookAt(
			position,           // Camera is here
			position+direction, // and looks here : at the same position, plus "direction"
			glm::vec3(0,1,0)    // Head is up (set to 0,-1,0 to look upside-down)
			);

}



void handleMouseMove(GLFWwindow* window, double xpos, double ypos){

	//カメラが一回転したら強制的に2PI回すことで無限に回れるようにする
	if(horizontalAngle + mouseSpeed * float(midWindowX - xpos) > 3.14){
		horizontalAngle = (horizontalAngle + mouseSpeed * float(midWindowX - xpos)) - (3.14*2);
	}else if(horizontalAngle + mouseSpeed * float(midWindowX - xpos) < -3.14){
		horizontalAngle = (horizontalAngle + mouseSpeed * float(midWindowX - xpos)) + (3.14*2);
	}else{
		horizontalAngle += mouseSpeed * float(midWindowX - xpos );
	}

	//カメラは真下から真上までの範囲しか動かない。頭は縦に一回転しない。
	if(verticalAngle + mouseSpeed * float(midWindowY - ypos ) > 3.14/2){
		verticalAngle = 3.14/2;
	}else if(verticalAngle + mouseSpeed * float(midWindowY - ypos ) < -3.14/2){
		verticalAngle = -3.14/2;
	}else{
		verticalAngle   += mouseSpeed * float(midWindowY - ypos );
	}

	//マウスを強制的に真ん中に戻す
	glfwSetCursorPos(window, midWindowX, midWindowY);
}


void handleKeypress(GLFWwindow* window, int key, int scancode, int action, int mods){

	if (action == GLFW_PRESS){
		switch(key) {
			case 'W':
				holdingForward = true;
				break;

			case 'S':
				holdingBackward = true;
				break;

			case 'A':
				holdingLeftStrafe = true;
				break;

			case 'D':
				holdingRightStrafe = true;
				break;

			case GLFW_KEY_LEFT_SHIFT:
				holdingSneek = true;
				break;

			case GLFW_KEY_SPACE:
				holdingSpace = true;
				break;

			case GLFW_KEY_ESCAPE:
				glfwSetCursorPosCallback(window, NULL);
				glfwSetKeyCallback(window, NULL);
				glfwSetCursorPos(window, midWindowX, midWindowY);
				glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);
				break;

			default:
				break;
		}
	}else if(action == GLFW_RELEASE){
		switch(key) {
			case 'W':
				holdingForward = false;
				break;

			case 'S':
				holdingBackward = false;
				break;

			case 'A':
				holdingLeftStrafe = false;
				break;

			case 'D':
				holdingRightStrafe = false;
				break;

			case GLFW_KEY_LEFT_SHIFT :
				holdingSneek = false;
				break;

			case GLFW_KEY_SPACE:
				holdingSpace = false;
				break;

			default:
				break;
		}

	}
}

void handleMouseButton(GLFWwindow* window, int button, int action, int mods){
	if(action == GLFW_PRESS){
		switch(button){
			case GLFW_MOUSE_BUTTON_1:
				glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
				glfwSetCursorPos(window, midWindowX, midWindowY);
				glfwSetCursorPosCallback(window, handleMouseMove);
				glfwSetKeyCallback(window, handleKeypress);
				break;
		}
	}
}

void handleWindowResize(GLFWwindow* window, int width, int height){
	windowWidth  = width;
	windowHeight = height;
	midWindowX = windowWidth  / 2;
	midWindowY = windowHeight / 2;
}


//オイラー角から４次元数を計算する。opengl-math用とbullet用で2つある。
glm::quat createq(double RotationAngle, double RotationAxisX, double RotationAxisY, double RotationAxisZ){
	double x = RotationAxisX * sin(RotationAngle / 2);
	double y = RotationAxisY * sin(RotationAngle / 2);
	double z = RotationAxisZ * sin(RotationAngle / 2);
	double w = cos(RotationAngle / 2);
	return glm::quat(w, x, y, z);
}

btQuaternion btcreateq(double RotationAngle, double RotationAxisX, double RotationAxisY, double RotationAxisZ){
	double x = RotationAxisX * sin(RotationAngle / 2);
	double y = RotationAxisY * sin(RotationAngle / 2);
	double z = RotationAxisZ * sin(RotationAngle / 2);
	double w = cos(RotationAngle / 2);
	return btQuaternion(x, y, z, w);
}

int main(){

	if (!glfwInit()){
		std::cout << "glfw init failed...." << std::endl;
	}

	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	window = glfwCreateWindow(windowWidth, windowHeight, "Japari", NULL, NULL);
	if (!window){
		std::cout << "cannot open OpenGL window" << std::endl;
	}
	glfwMakeContextCurrent(window);

	glewExperimental = GL_TRUE;
	if(glewInit () != GLEW_OK){
		std::cout << "glew init failed...." << std::endl;
	}

	glClearColor(0.5f, 0.5f, 1.0f, 0.0f);


	glEnable(GL_DEPTH_TEST); //隠面消去
	glDepthFunc(GL_LESS);    //近いものを表示
	glEnable(GL_CULL_FACE);  //ウラは表示しない


	//VAOを作る
	GLuint VertexArrayID;
	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);


	// Create and compile our GLSL program from the shaders
	GLuint programID = LoadShaders( "main.vert", "main.frag" );
	// Get a handle for our "VP" uniform
	uniform_viewMatrix = glGetUniformLocation(programID, "V");
	uniform_projectionMatrix = glGetUniformLocation(programID, "P");
	uniform_LightColor = glGetUniformLocation(programID, "LightColor");
	uniform_LightPower = glGetUniformLocation(programID, "LightPower");
	uniform_LightDirection = glGetUniformLocation(programID, "LightDirection");
	uniform_depthBiasVP = glGetUniformLocation(programID, "DepthBiasVP");


	//----- 影関連 -----//

	// Create and compile our GLSL program from the shaders
	GLuint depthProgramID = LoadShaders( "depthBuffer.vert", "depthBuffer.frag");
	// Get a handle for our "VP" uniform
	GLuint depthMatrixID = glGetUniformLocation(depthProgramID, "depthMV");

	// The framebuffer, which regroups 0, 1, or more textures, and 0 or 1 depth buffer.
	GLuint FramebufferName;
	glGenFramebuffers(1, &FramebufferName);

	glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);

	// Depth texture. Slower than a depth buffer, but you can sample it later in your shader
	GLuint depthTexture;
	glGenTextures(1, &depthTexture);
	glBindTexture(GL_TEXTURE_2D, depthTexture);
	glTexImage2D(GL_TEXTURE_2D, 0,GL_DEPTH_COMPONENT16, 1024, 1024, 0,GL_DEPTH_COMPONENT, GL_FLOAT, 0);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR); 
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_R_TO_TEXTURE);

	glFramebufferTexture(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, depthTexture, 0);

	// No color output in the bound framebuffer, only depth.
	glDrawBuffer(GL_NONE);

	// Always check that our framebuffer is ok
	if(glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE){
		return false;
	}

	//----- 影関連 -----//

	glBindFramebuffer(GL_FRAMEBUFFER, 0);


	//入力のコールバック・カーソルタイプの設定
	glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);
	glfwSetKeyCallback(window, handleKeypress);
	glfwSetMouseButtonCallback(window, handleMouseButton);
	glfwSetCursorPosCallback(window, handleMouseMove);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
	glfwSetWindowSizeCallback(window, handleWindowResize);


	//物理ワールドの生成
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -10, 0));


	//頂点バッファオブジェクトを作る
	initVBO();


	initPrimitives();


	void *lh;
	const char* path = "./friends/";
	DIR *dp;       // ディレクトリへのポインタ
	dirent* entry; // readdir() で返されるエントリーポイント

	dp = opendir(path);
	if (dp==NULL) exit(1);
	entry = readdir(dp);
	while (entry != NULL){
		std::string filename(entry->d_name);
		if(split(filename,'.').size() >= 2 && split(filename, '.')[1] == "friends"){

			lh = dlopen((path + filename).c_str(), RTLD_LAZY);
			if (!lh) {
				fprintf(stderr, "dlopen error: %s\n", dlerror());
				exit(1);
			}

			void (*pluginInit)() = (void (*)())dlsym(lh, "init");
			char *error = dlerror();
			if (error) {
				fprintf(stderr, "dlsym error: %s\n", error);
				exit(1);
			}
			pluginInitVector.push_back(*pluginInit);

			void (*pluginTick)() = (void (*)())dlsym(lh, "tick");
			error = dlerror();
			if (error) {
				fprintf(stderr, "dlsym error: %s\n", error);
				exit(1);
			}
			pluginTickVector.push_back(*pluginTick);



		}

		entry = readdir(dp);
	}


	for(auto elem: pluginInitVector){
		(elem)();
	}


	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glEnableVertexAttribArray(3);
	glEnableVertexAttribArray(4);
	glEnableVertexAttribArray(5);
	glEnableVertexAttribArray(6);



	//毎フレームごとにこの中が実装される。
	while (glfwWindowShouldClose(window) == GL_FALSE){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//カメラ位置等を計算する
		computeMatricesFromInputs();

		for(auto elem: pluginTickVector){
			(elem)();
		}

		//物理演算1ステップ進める
		dynamicsWorld->stepSimulation(1 / 60.f, 10);



		// :: OpenGL描画 ::

		// まずはデプスバッファを作る

		// Render to our framebuffer
		glBindFramebuffer(GL_FRAMEBUFFER, FramebufferName);
		glViewport(0,0,1024,1024); // Render on the whole framebuffer, complete from the lower left corner to the upper right

		// We don't use bias in the shader, but instead we draw back faces, 
		// which are already separated from the front faces by a small distance 
		// (if your geometry is made this way)
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK); // Cull back-facing triangles -> draw only front-facing triangles

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		// Use our shader
		glUseProgram(depthProgramID);

		glm::vec3 lightInvDir = glm::vec3(0.5f,2,2);

		// Compute the VP matrix from the light's point of view
		glm::mat4 depthProjectionMatrix = glm::ortho<float>(-50,50,-50,50,-10,20);
		glm::mat4 depthViewMatrix = glm::lookAt(lightInvDir, glm::vec3(0,0,0), glm::vec3(0,1,0));

		glm::mat4 depthVP = depthProjectionMatrix * depthViewMatrix;

		// Send our transformation to the currently bound shader,
		// in the "VP" uniform
		glUniformMatrix4fv(depthMatrixID, 1, GL_FALSE, &depthVP[0][0]);


		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)(sizeof(GLfloat)*3));
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)(sizeof(GLfloat)*6));


		for(auto elem: elementManager::elementManagerList){ //TODO 最適化できるはず(下にも同じコード)
			elem->render();
		}


		// 通常描画
		// Render to the screen
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		glViewport(0, 0, windowWidth*2, windowHeight*2); // Render on the whole framebuffer, complete from the lower left corner to the upper right

		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK); // Cull back-facing triangles -> draw only front-facing triangles

		// Clear the screen
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glUseProgram(programID);


		glm::mat4 biasMatrix(
			0.5, 0.0, 0.0, 0.0,
			0.0, 0.5, 0.0, 0.0,
			0.0, 0.0, 0.5, 0.0,
			0.5, 0.5, 0.5, 1.0
		);
		glm::mat4 depthBiasVP = biasMatrix*depthVP;

		glUniformMatrix4fv(uniform_viewMatrix,       1, GL_FALSE, &ViewMatrix[0][0]);
		glUniformMatrix4fv(uniform_projectionMatrix, 1, GL_FALSE, &ProjectionMatrix[0][0]);
		glUniformMatrix4fv(uniform_depthBiasVP,      1, GL_FALSE, &depthBiasVP[0][0]);
		glUniform3fv      (uniform_LightColor,       1, &lightColor[0]);
		glUniform1fv      (uniform_LightPower,       1, &lightPower);
		glUniform3fv      (uniform_LightDirection,   1, &lightDirection[0]);

		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)(sizeof(GLfloat)*3));
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)(sizeof(GLfloat)*6));


		for(auto elem: elementManager::elementManagerList){
			elem->render();
		}

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	std::cout << "stopping program..." << std::endl;

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(3);
	glDisableVertexAttribArray(4);
	glDisableVertexAttribArray(5);
	glDisableVertexAttribArray(6);

	printf("unloading libdll.so\n");
	dlclose(lh);


	while(elementManager::elementManagerList.empty() == false){
		delete elementManager::elementManagerList.back();
		elementManager::elementManagerList.pop_back();
	}

	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;

	glDeleteVertexArrays(1, &VertexArrayID);
	glDeleteProgram(programID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();



	std::cout << "paramWrapper: " << paramWrapper::count << std::endl;
	std::cout << "parameterPack: " << parameterPack::count << std::endl;
	std::cout << "univStr: " << univStr::count << std::endl;
	std::cout << "vec3: " << vec3::count << std::endl;
	std::cout << "quat: " << quat::count << std::endl;
	std::cout << "vertex: " << vertex::count << std::endl;
	std::cout << "vertexManager: " << vertexManager::count << std::endl;
	std::cout << "hingeConstraint: " << hingeConstraint::count << std::endl;
	std::cout << "generic6DofConstraint: " << generic6DofConstraint::count << std::endl;
	std::cout << "elementManager: " << elementManager::count << std::endl;
	std::cout << "elementNode: " << elementNode::count << std::endl;








	return 0;
}

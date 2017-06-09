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
#include <cmath>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <btBulletDynamicsCommon.h>


#include "vertexmanage.hpp"
#include "shader.hpp"
#include "cubeshape.hpp"
#include "floorshape.hpp"
#include "constraints.hpp"
#include "commonshape.hpp"
#include "muscle.hpp"

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
	ProjectionMatrix = glm::perspective(FoV, 4.0f / 3.0f, 0.1f, 300.0f);

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

class dog{

	public:

	float dna[20][4] = {};
	btDiscreteDynamicsWorld* dynamicsWorld;


	cubeshapeObject* body;
	cubeshapeObject* head;
	cubeshapeObject* muzzle;
	cubeshapeObject* earLeft;
	cubeshapeObject* earRight;
	cubeshapeObject* legFrontLeft;
	cubeshapeObject* legFrontRight;
	cubeshapeObject* legBackLeft;
	cubeshapeObject* legBackRight;
	cubeshapeObject* tail;

	btHingeConstraint* hinge_body_head;
	btHingeConstraint* hinge_head_muzzle;
	btHingeConstraint* hinge_earLeft_head;
	btHingeConstraint* hinge_earRight_head;
	btHingeConstraint* hinge_body_tail;
	btGeneric6DofConstraint* hinge_body_legFrontLeft;
	btGeneric6DofConstraint* hinge_body_legFrontRight;
	btGeneric6DofConstraint* hinge_body_legBackLeft;
	btGeneric6DofConstraint* hinge_body_legBackRight;


	dog(btDiscreteDynamicsWorld* dynamicsWorld, float x, float y, float z, bool initialDNA){

		this->dynamicsWorld = dynamicsWorld;


		//DNAをランダムで初期化する
		if(initialDNA == true){
			std::random_device rd;
			std::mt19937 mt(rd());
			std::uniform_real_distribution<double> score(-1.57,1.57);

			for(auto elem: dna){
				elem[0] = score(mt);
				elem[1] = score(mt);
				elem[2] = score(mt);
				elem[3] = score(mt);
			}
		}

		spawn(x, y, z);

	}

	void spawn(float x, float y, float z){
		//犬の体の構造を定義している
		//キューブで肉体を作る cubeshape::create(位置, 大きさ, 傾き, 重さ, 追加先物理世界);
		body			= cubeshape::create(vec3(x,     y,     z),		vec3(2, 1, 1),			quat(1, 0, 0, 0), 2,		dynamicsWorld);
		head			= cubeshape::create(vec3(x+1.4, y,     z),		vec3(0.8, 0.8, 0.8),	quat(1, 0, 0, 0), 0.5,		dynamicsWorld);
		muzzle			= cubeshape::create(vec3(x+2.1, y-0.2, z),		vec3(0.6, 0.4, 0.4),	quat(1, 0, 0, 0), 0.1,		dynamicsWorld);
		earLeft			= cubeshape::create(vec3(x+1.4, y+0.5, z-0.2),	vec3(0.2, 0.2, 0.2),	quat(1, 0, 0, 0), 0.05,	dynamicsWorld);
		earRight		= cubeshape::create(vec3(x+1.4, y+0.5, z+0.2),	vec3(0.2, 0.2, 0.2),	quat(1, 0, 0, 0), 0.05,	dynamicsWorld);
		legFrontLeft	= cubeshape::create(vec3(x+0.5, y-1,   z-0.4),	vec3(0.2, 1, 0.2),		quat(1, 0, 0, 0), 0.3,		dynamicsWorld);
		legFrontRight	= cubeshape::create(vec3(x+0.5, y-1,   z+0.4),	vec3(0.2, 1, 0.2),		quat(1, 0, 0, 0), 0.3,		dynamicsWorld);
		legBackLeft		= cubeshape::create(vec3(x-0.5, y-1,   z-0.4),	vec3(0.2, 1, 0.2),		quat(1, 0, 0, 0), 0.3,		dynamicsWorld);
		legBackRight	= cubeshape::create(vec3(x-0.5, y-1,   z+0.4),	vec3(0.2, 1, 0.2),		quat(1, 0, 0, 0), 0.3,		dynamicsWorld);
		tail			= cubeshape::create(vec3(x-1.5, y+0.4, z),		vec3(1, 0.2, 0.2),		quat(1, 0, 0, 0), 0.2,		dynamicsWorld);

		//肉体同士を関節で接続する	btHingeConstraint(物体A, 物体B, 物体A上の位置, 物体B上の位置, ヒンジの軸の方向);
		hinge_body_head = new btHingeConstraint(*(body->body), *(head->body), btVector3(1, 0, 0), btVector3(-0.4, 0, 0), btVector3(0, 0, 1), btVector3(0, 0, 1));
		hinge_body_head->setLimit(-3.14/6, 3.14/6);
		dynamicsWorld->addConstraint(hinge_body_head, true);

		hinge_head_muzzle = new btHingeConstraint(*(head->body), *(muzzle->body), btVector3(0.4, -0.2, 0), btVector3(-0.3, 0, 0), btVector3(1, 0, 0), btVector3(1, 0, 0));
		hinge_head_muzzle->setLimit(0, 0);
		dynamicsWorld->addConstraint(hinge_head_muzzle, true);

		hinge_earLeft_head = new btHingeConstraint(*(earLeft->body), *(head->body), btVector3(0, -0.1, 0), btVector3(0, 0.4, -0.2), btVector3(1, 0, 0), btVector3(1, 0, 0));
		hinge_earLeft_head->setLimit(0, 0);
		dynamicsWorld->addConstraint(hinge_earLeft_head, true);

		hinge_earRight_head = new btHingeConstraint(*(earRight->body), *(head->body), btVector3(0, -0.1, 0), btVector3(0, 0.4, 0.2), btVector3(1, 0, 0), btVector3(1, 0, 0));
		hinge_earRight_head->setLimit(0, 0);
		dynamicsWorld->addConstraint(hinge_earRight_head, true);


		hinge_body_legFrontLeft  = make6dof(body, legFrontLeft,  btVector3(0.5,  -0.5, -0.4), btVector3(0, 0.5, 0.0));
		dynamicsWorld->addConstraint(hinge_body_legFrontLeft);

		hinge_body_legFrontRight = make6dof(body, legFrontRight, btVector3(0.5,  -0.5,  0.4), btVector3(0, 0.5, 0.0));
		dynamicsWorld->addConstraint(hinge_body_legFrontRight);

		hinge_body_legBackLeft   = make6dof(body, legBackLeft,   btVector3(-0.5, -0.5, -0.4), btVector3(0, 0.5, 0.0));
		dynamicsWorld->addConstraint(hinge_body_legBackLeft);

		hinge_body_legBackRight  = make6dof(body, legBackRight,  btVector3(-0.5, -0.5,  0.4), btVector3(0, 0.5, 0.0));
		dynamicsWorld->addConstraint(hinge_body_legBackRight);

		hinge_body_tail = new btHingeConstraint(*(body->body), *(tail->body), btVector3(-1, 0.4, 0), btVector3(0.5, 0, 0.0), btVector3(0, 0, 1), btVector3(0, 0, 1));
		hinge_body_tail->setLimit(-3.14/3, 3.14/3);
		dynamicsWorld->addConstraint(hinge_body_tail, true);
	}


	//シーケンス番号に対応するDNAに記録されている角度まで足を動かす
	void move(int sequence){

	}

	void destroy(){
		dynamicsWorld->removeConstraint(hinge_body_head);
		dynamicsWorld->removeConstraint(hinge_body_head);
		dynamicsWorld->removeConstraint(hinge_head_muzzle);
		dynamicsWorld->removeConstraint(hinge_earLeft_head);
		dynamicsWorld->removeConstraint(hinge_earRight_head);
		dynamicsWorld->removeConstraint(hinge_body_legFrontLeft);
		dynamicsWorld->removeConstraint(hinge_body_legFrontRight);
		dynamicsWorld->removeConstraint(hinge_body_legBackLeft);
		dynamicsWorld->removeConstraint(hinge_body_legBackRight);
		dynamicsWorld->removeConstraint(hinge_body_tail);

		body->destroy();
		head->destroy();
		muzzle->destroy();
		earLeft->destroy();
		earRight->destroy();
		legFrontLeft->destroy();
		legFrontRight->destroy();
		legBackLeft->destroy();
		legBackRight->destroy();
		tail->destroy();
	}

	btGeneric6DofConstraint* make6dof(cubeshapeObject* cubeA, cubeshapeObject* cubeB, btVector3 pivotInA, btVector3 pivotInB){
		btTransform frameInA, frameInB;
		frameInA = cubeA->body->getCenterOfMassTransform();
		frameInB = cubeB->body->getCenterOfMassTransform();
		frameInA.setOrigin(pivotInA);
		frameInB.setOrigin(pivotInB);

		btGeneric6DofConstraint* pGen6Dof = new btGeneric6DofConstraint(*(cubeA->body), *(cubeB->body), frameInA, frameInB, false );

		pGen6Dof->setAngularLowerLimit(btVector3(0,0,-M_PI));
		pGen6Dof->setAngularUpperLimit(btVector3(0,0,M_PI));
		pGen6Dof->setLinearLowerLimit(btVector3(0,0,0));
		pGen6Dof->setLinearUpperLimit(btVector3(0,0,0));
		pGen6Dof->getTranslationalLimitMotor()->m_enableMotor[5] = true;

		return pGen6Dof;
	}


};


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
	GLuint programID = LoadShaders( "TransformVertexShader.vertexshader", "ColorFragmentShader.fragmentshader" );
	// Get a handle for our "MVP" uniform
	uniform_viewMatrix = glGetUniformLocation(programID, "V");
	uniform_projectionMatrix = glGetUniformLocation(programID, "P");
	uniform_LightColor = glGetUniformLocation(programID, "LightColor");
	uniform_LightPower = glGetUniformLocation(programID, "LightPower");
	uniform_LightDirection = glGetUniformLocation(programID, "LightDirection");
	// Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	glm::mat4 Projection = glm::perspective(45.0f, 4.0f / 3.0f, 0.1f, 100.0f);

	//入力のコールバック・カーソルタイプの設定
	glfwSetInputMode(window, GLFW_STICKY_KEYS, 1);
	glfwSetKeyCallback(window, handleKeypress);
	glfwSetCursorPosCallback(window, handleMouseMove);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);


	//物理ワールドの生成
	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	dynamicsWorld->setGravity(btVector3(0, -10, 0));


	//頂点バッファオブジェクトを作る
	initVBO();

	//使う図形についてinit
	cubeshape::init();
	floorshape::init();


	void *lh;
	const char* path = "./friends/";
	DIR *dp;       // ディレクトリへのポインタ
	dirent* entry; // readdir() で返されるエントリーポイント

	std::string hoge;

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

	// 筋肉を作ってます
	cubeshapeObject* cubeA = cubeshape::create(vec3(0,   5, -5), vec3(0.2, 1, 0.2), quat(1, 0, 0, 0), 0, dynamicsWorld);
	cubeshapeObject* cubeB = cubeshape::create(vec3(0, 3.6, -5), vec3(0.2, 1, 0.2), quat(1, 0, 0, 0), 1, dynamicsWorld);

	contractileElement* muscle     = new contractileElement(cubeA, cubeB,  0.2,  0.9, 0.,  0.2, 1.0, 0., 2);
	contractileElement* ant_muscle = new contractileElement(cubeA, cubeB, -0.2, -1.0, 0., -0.2, 1.0, 0., 2);

	btTransform frameInA, frameInB;
	frameInA = cubeA->body->getCenterOfMassTransform();
	frameInB = cubeB->body->getCenterOfMassTransform();
	frameInA.setOrigin(btVector3(btScalar(0), btScalar(-1.2), btScalar(0.)));
	frameInB.setOrigin(btVector3(btScalar(0), btScalar( 1.2), btScalar(0.)));

	btGeneric6DofConstraint* pGen6Dof = new btGeneric6DofConstraint(*(cubeA->body), *(cubeB->body), frameInA, frameInB, false );
	dynamicsWorld->addConstraint(pGen6Dof);

	pGen6Dof->setAngularLowerLimit(btVector3(0,0,-M_PI));
	pGen6Dof->setAngularUpperLimit(btVector3(0,0,M_PI));
	pGen6Dof->setLinearLowerLimit(btVector3(0,0,0));
	pGen6Dof->setLinearUpperLimit(btVector3(0,0,0));

	//pGen6Dof->getTranslationalLimitMotor()->m_enableMotor[0] = true;
	//pGen6Dof->getTranslationalLimitMotor()->m_targetVelocity[0] = 5.0f;
	//pGen6Dof->getTranslationalLimitMotor()->m_maxMotorForce[0] = 10.0f;


	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);
	glEnableVertexAttribArray(3);
	glEnableVertexAttribArray(4);
	glEnableVertexAttribArray(5);
	glEnableVertexAttribArray(6);

	int counter = 0;
	bool wait = true;


	//毎フレームごとにこの中が実装される。
	while (glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS && glfwWindowShouldClose(window) == GL_FALSE){
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//カメラ位置等を計算する
		computeMatricesFromInputs();

		for(auto elem: pluginTickVector){
			(elem)();
		}



		//物理演算1ステップ進める
		dynamicsWorld->stepSimulation(1 / 60.f, 10);

		// 収縮
		if(!wait){
			muscle->contract(0.89);
			ant_muscle->contract(muscle->antRate());
		}

		if(counter < 100)
			counter++;
		else{
			wait = false;
			counter = 0;
		}

		//OpenGL描画
		glUseProgram(programID);

		glUniformMatrix4fv(uniform_viewMatrix,       1, GL_FALSE, &ViewMatrix[0][0]);
		glUniformMatrix4fv(uniform_projectionMatrix, 1, GL_FALSE, &ProjectionMatrix[0][0]);
		glUniform3fv(uniform_LightColor, 1, &lightColor[0]);
		glUniform1fv(uniform_LightPower, 1, &lightPower);
		glUniform3fv(uniform_LightDirection, 1, &lightDirection[0]);

		glBindBuffer(GL_ARRAY_BUFFER, vertexBufferObject);
		glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)0);
		glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)(sizeof(GLfloat)*3));
		glVertexAttribPointer(2, 3, GL_FLOAT, GL_FALSE, sizeof(vertex), (void*)(sizeof(GLfloat)*6));

		cubeshape::render();
		floorshape::render();

		for(auto elem: commonshapeList){
			elem->render();
		}

		glfwSwapBuffers(window);
		glfwPollEvents();
	}

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glDisableVertexAttribArray(3);
	glDisableVertexAttribArray(4);
	glDisableVertexAttribArray(5);
	glDisableVertexAttribArray(6);

	printf("unloading libdll.so\n");
	dlclose(lh);


	glDeleteVertexArrays(1, &VertexArrayID);
	glDeleteProgram(programID);

	// Close OpenGL window and terminate GLFW
	glfwTerminate();


	return 0;
}

/**
* @file main.cpp
* @brief Main file of project
* \mainpage Description
* Project which visualizes joint tracking using ORBBEC Astra 3D camera with OpenGL
*/


#include "stdafx.h"

#include <GL\glew.h>
#include <GL\freeglut.h>
#include <iostream>
#include <thread>
#include <stdlib.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <astra\astra.hpp>

#include "CStreamListener.h"
#include "CAstraStream.h"
#include "SocketTCP.hpp"

#include <algorithm>
#include <fstream>
#include <vector>

#define OFFSET 1.15
#define bufSize 7//640*480*7//512
#define nBodyparts 19
// Currently stream, sensor and reader variables are global because of OpenGL which
// doesn't function well when programming objective oriented

float camX = 1.5, camY = 0;

/// Class which implements astra::on_frame_ready, serves as interrupt when frame is recieved
AstraStream stream;

/// Sensor and reader are pretty self-explainatory I'd say.
/// They are used to enable particular stream, capture frame and stuff like that.
astra::StreamSet sensor;
astra::StreamReader reader;
using namespace std;
using namespace cv;

/// GLUT window dimensions.
double w;
double h;
char strPoint[bufSize];
bool hasNewData = false, doOnce = false;
astra::MaskedColorFrame lastFrame = NULL;

struct Points
{
	float x;
	float y;
	float z;
};

class MaskedColorFrameListener : public astra::FrameListener
{
	int cnt = 0;
public:
	MaskedColorFrameListener(int maxFramesToProcess)
		: maxFramesToProcess_(maxFramesToProcess)
	{}

	bool is_finished() const { return isFinished_; }

private:
	void on_frame_ready(astra::StreamReader& reader,astra::Frame& frame) override
	{
		astra::MaskedColorFrame maskedColorFrame = frame.get<astra::MaskedColorFrame>();

		if (maskedColorFrame.is_valid())
		{
			//print_depth_frame(maskedColorFrame);
			++framesProcessed_;
			//printf("Frame: %d", framesProcessed_);
			//std::cout << "Pozvan" << maskedColorFrame.height() <<std::endl;
			lastFrame = maskedColorFrame;
			if (true)
			{
				/*Mat image;
				image = depthframe2mat(maskedColorFrame);
				imshow("MyWindow", image);*/

				/*int width = lastFrame.width();
				int height = lastFrame.height();
				int grayval = 0;
				Mat image(height, width, CV_8UC4);
				
				for (int y = 0; y < height; y++)
				{
					for (int x = 0; x < width; x++)
					{
						//grayval = lastframe.data()[x + y * width];
						image.at<astra_rgba_pixel_t>(y, x) = lastFrame.data()[x + y * width];
					}
				}*/
				
				Mat my_frame = Mat(lastFrame.height(), lastFrame.width(), CV_8UC4, (void *)lastFrame.data());
				//		cvtColor(lastFrame.data, lastFrame.data, CV_BGR2RGB); //this will put colors right
			//	memcpy((void *)lastFrame.data(), imageBuffer, 4 * lastFrame.height()*lastFrame.width() * sizeof(uint8_t));
				char imgName[100];
				sprintf(imgName, "some%d.png", cnt);
				//printf("%s", imgName);
				if(cnt % 100 == 0)
				{	
					cv::cvtColor(my_frame, my_frame, cv::COLOR_BGRA2RGBA);
					cv::imwrite(imgName, my_frame);
				}
				
				//imshow("MyWindow", my_frame); 
				//waitKey(1);
				doOnce = true;
				cnt++;
			}
			//Mat image(480, 640, CV_8UC4 , (void *)lastFrame.data());
			//imwrite("astra.png", (void *)maskedColorFrame.data());
			//String windowName = "My HelloWorld Window"; //Name of the window

			//namedWindow(windowName); // Create a window

		//	imshow(windowName, image); // Show our image inside the created window.
			/*std::cout << "RGB: " << maskedColorFrame.handle()->metadata.pixelFormat<< std::endl;
			for (auto it = 0; it != 500; ++it)
			{
				std::cout << "Frame index: "<< maskedColorFrame.frame_index() <<"Index:" << it << std::endl;
				std::cout << "Alpha." << static_cast< int >(maskedColorFrame.data()[it].alpha) << std::endl;
				std::cout << "R" << static_cast< int >(maskedColorFrame.data()[it].r) << std::endl;
				std::cout << "G" << static_cast< int >(maskedColorFrame.data()[it].g) << std::endl;
				std::cout << "B" << static_cast< int >(maskedColorFrame.data()[it].b) << std::endl;

			}*/
			
			//for(int i = 0; i < sizeof(maskedColorFrame.copy_to()
		}
			
		isFinished_ = framesProcessed_ >= maxFramesToProcess_;
	}

	/*Mat depthframe2mat(astra::MaskedColorFrame lastframe)
	{
		int width = lastFrame.width();
		int height = lastFrame.height();
		int grayval = 0;
		Mat image(height, width, CV_8UC4);

		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				//grayval = lastframe.data()[x + y * width];
				image.at<astra_rgba_pixel_t>(y, x) = lastframe.data()[x + y * width];
			}
		}
		return image;
	}*/

	void print_depth_frame(const astra::MaskedColorFrame& depthFrame) const
	{
		const int frameIndex = depthFrame.frame_index();
		const short middleValue = get_middle_value(depthFrame);

		std::printf("Depth frameIndex: %d value: %d \n", frameIndex, middleValue);
		std::cout << "Depth frameIndex: " << frameIndex << "value: \n" << middleValue << std::endl;
	}

	short get_middle_value(const astra::MaskedColorFrame& depthFrame) const
	{
		const int width = depthFrame.width();
		const int height = depthFrame.height();

		const size_t middleIndex = ((width * (height / 2.f)) + (width / 2.f));

	//	const short* frameData = depthFrame.data();
	//	const short middleValue = frameData[middleIndex];
		const short middleValue = 2;

		return middleValue;
	}

	bool isFinished_{ false };
	int framesProcessed_{ 0 };
	int maxFramesToProcess_{ 0 };

};


MaskedColorFrameListener listener(100);

/*class MaskedColorFrameListener : public astra::FrameListener
{
	virtual void on_frame_ready(astra::StreamReader& reader, astra::Frame& frame) override
	{
		const astra::MaskedColorFrame maskedColorFrame = frame.get<astra::MaskedColorFrame>();

		if (maskedColorFrame.is_valid())
		{
			// do all the things
			std::cout << "Evo framea";
		}
	}
};*/

struct Points points[nBodyparts][3];
/// Control variable - user must choose '1' from menu to start the stream
int gameRunning = 0;

void sendData();
string dec2Hex(int value);
bool sendPoint(SOCKET sock, char* points);
int sendCnt = 0;

#pragma region initsForOpenGl

/// Draws line in OpenGL window
/// @param x1 first x coordinate
/// @param y1 first y coordinate
/// @param z1 first z coordinate - currently drawing is 2D so this is commented out
/// @param x2 second x coordinate
/// @param y2 second y coordinate
/// @param z2 second z coordinate - currently drawing is 2D so this is commented out
/// @returns nothing
void drawLine(float x1, float y1, /*float z1, */float x2, float y2/*, float z2*/);

/// OpenGL redraw function - draws joint points and lines between them
/// @returns nothing
void Draw();

/// OpenGL timer function - set to repaint canvas every x ms (in this case 20) if control variable is 1
/// @param iUnused unused
/// @returns nothing
void Timer(int iUnused);

/// Starts the camera, all the streams, adds frame listener
/// @returns nothing
void astraInit();

/// Initializes canvas (clears it and sets projection)
/// @return nothing
void Initialize();

/// Used for 3D view, currently not used
/// @param w OpenGL window width
/// @param h OpenGL window height
void resize(int w, int h);

/// OpenGL window hotkeys
/// @param key pressed key
/// @param int unused (?)
/// @param int unused (?)
/// @returns nothing
void keyboard(unsigned char key, int, int);

/* Initialize OpenGL Graphics */
void initGL() {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set background color to black and opaque
	glClearDepth(1.0f);                   // Set background depth to farthest
	glEnable(GL_DEPTH_TEST);   // Enable depth testing for z-culling
	glDepthFunc(GL_LEQUAL);    // Set the type of depth-test
	glShadeModel(GL_SMOOTH);   // Enable smooth shading
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);  // Nice perspective corrections
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
	glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

									// Render a color-cube consisting of 6 quads with different colors
	glLoadIdentity();                 // Reset the model-view matrix
	glTranslatef(camX, camY, -1500);  // Move right and into the screen

	glBegin(GL_QUADS);                // Begin drawing the color cube with 6 quads
									  // Top face (y = 1.0f)
									  // Define vertices in counter-clockwise (CCW) order with normal pointing out
	glColor3f(0.0f, 100.0f, 0.0f);     // Green
	glVertex3f(100.0f, 100.0f, -100.0f);
	glVertex3f(-100.0f, 100.0f, -100.0f);
	glVertex3f(-100.0f, 100.0f, 100.0f);
	glVertex3f(100.0f, 100.0f, 100.0f);

	// Bottom face (y = -1.0f)
	glColor3f(1.0f, 0.5f, 0.0f);     // Orange
	glVertex3f(1.0f, -1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);

	// Front face  (z = 1.0f)
	glColor3f(1.0f, 0.0f, 0.0f);     // Red
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);

	// Back face (z = -1.0f)
	glColor3f(1.0f, 1.0f, 0.0f);     // Yellow
	glVertex3f(1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, -1.0f);

	// Left face (x = -1.0f)
	glColor3f(0.0f, 0.0f, 1.0f);     // Blue
	glVertex3f(-1.0f, 1.0f, 1.0f);
	glVertex3f(-1.0f, 1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glVertex3f(-1.0f, -1.0f, 1.0f);

	// Right face (x = 1.0f)
	glColor3f(1.0f, 0.0f, 1.0f);     // Magenta
	glVertex3f(1.0f, 1.0f, -1.0f);
	glVertex3f(1.0f, 1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, 1.0f);
	glVertex3f(1.0f, -1.0f, -1.0f);
	glEnd();  // End of drawing color-cube

			  // Render a pyramid consists of 4 triangles
	glLoadIdentity();                  // Reset the model-view matrix
	glTranslatef(-1.5f, 0.0f, -6.0f);  // Move left and into the screen

	glBegin(GL_TRIANGLES);           // Begin drawing the pyramid with 4 triangles
									 // Front
	glColor3f(1.0f, 0.0f, 0.0f);     // Red
	glVertex3f(0.0f, 1.0f, 0.0f);
	glColor3f(0.0f, 1.0f, 0.0f);     // Green
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glColor3f(0.0f, 0.0f, 1.0f);     // Blue
	glVertex3f(1.0f, -1.0f, 1.0f);

	// Right
	glColor3f(1.0f, 0.0f, 0.0f);     // Red
	glVertex3f(0.0f, 1.0f, 0.0f);
	glColor3f(0.0f, 0.0f, 1.0f);     // Blue
	glVertex3f(1.0f, -1.0f, 1.0f);
	glColor3f(0.0f, 1.0f, 0.0f);     // Green
	glVertex3f(1.0f, -1.0f, -1.0f);

	// Back
	glColor3f(1.0f, 0.0f, 0.0f);     // Red
	glVertex3f(0.0f, 1.0f, 0.0f);
	glColor3f(0.0f, 1.0f, 0.0f);     // Green
	glVertex3f(1.0f, -1.0f, -1.0f);
	glColor3f(0.0f, 0.0f, 1.0f);     // Blue
	glVertex3f(-1.0f, -1.0f, -1.0f);

	// Left
	glColor3f(1.0f, 0.0f, 0.0f);       // Red
	glVertex3f(0.0f, 1.0f, 0.0f);
	glColor3f(0.0f, 0.0f, 1.0f);       // Blue
	glVertex3f(-1.0f, -1.0f, -1.0f);
	glColor3f(0.0f, 1.0f, 0.0f);       // Green
	glVertex3f(-1.0f, -1.0f, 1.0f);
	glEnd();   // Done drawing the pyramid

	glutSwapBuffers();  // Swap the front and back frame buffers (double buffering)
}

void reshape(GLsizei width, GLsizei height) {  // GLsizei for non-negative integer
											   // Compute aspect ratio of the new window
	if (height == 0) height = 1;                // To prevent divide by 0
	GLfloat aspect = (GLfloat)width / (GLfloat)height;

	// Set the viewport to cover the new window
	glViewport(0, 0, width, height);

	// Set the aspect ratio of the clipping volume to match the viewport
	glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
	glLoadIdentity();             // Reset
								  // Enable perspective projection with fovy, aspect, zNear and zFar
	gluPerspective(45.0f, aspect, 0.1f, 10000.0f);
}

/// Initializes GLUT window, display functions, and other variables needed for canvas (OpenGL) to work
/// @param iArgc first parameter of main function
/// @param cppArgv second parameter of main function
/// @returns nothing


#pragma endregion
void setDrawing(int *iArgc, char **cppArgv)
{
	glutInit(iArgc, cppArgv);            // Initialize GLUT
	glutInitDisplayMode(GLUT_DOUBLE); // Enable double buffered mode
	glutInitWindowSize(640, 480);   // Set the window's initial width & height
	glutInitWindowPosition(50, 50); // Position the window's initial top-left corner
	glutCreateWindow("Skeleton Tracking - Orbbec Astra");          // Create window with the given title
	glutDisplayFunc(Draw);       // Register callback handler for window re-paint event
	glutReshapeFunc(reshape);       // Register callback handler for window re-size event
	initGL();                       // Our own OpenGL initialization
	glutKeyboardFunc(keyboard);
	Timer(0);
	glutMainLoop();
}


/// Main function of the program
int main(int iArgc, char **cppArgv)
{
	/*int var = 0;*/ int var = 1;



	Mat image = imread("rocco.png", CV_8UC4);

	std::vector<uchar> buffer;
	#define MB 1024*1024
	buffer.resize(200 * MB);
	cv::imencode(".png", image, buffer);

	Mat img = imdecode(buffer, -1);
	imwrite("rocco2.png", img);

	/*int size = image.total() * image.elemSize();
	BYTE *bytes = new BYTE[size];  // you will have to delete[] that later
	std::memcpy(bytes, image.data, size * sizeof(BYTE));

	Mat img = Mat(640, 480, CV_8UC4, bytes).clone(); // make a copy
	imwrite("rocco2.png", img);
	/*vector<BYTE> v_char;
	v_char.reserve(image.rows * image.cols);
	for (int i = 0; i < image.rows; i++) {
		int start = *image.data + i * image.step;
		v_char.insert(v_char.end(), start, start + image.cols);
	}

	Mat img = imdecode(*v_char.data(), 0);
	cv::imwrite("rocco2.png", img);*/

	/*while (1)
	{
	std::cout << "Press 1 to start the camera and game, 2 to leave\n";
	std::cin >> var;
	*/

	/*Mat image = imread("horse.jpg");

	if (image.empty()) // Check for failure
	{
		cout << "Could not open or find the image" << endl;
		system("pause"); //wait for any key press
		return -1;
	}


	String windowName = "My HelloWorld Window"; //Name of the window

	namedWindow(windowName); // Create a window

	imshow(windowName, image); // Show our image inside the created window.
	waitKey(1);//NE BRISI
	*/

	if (var == 1)
	{
		gameRunning = 1;

		// astraInit() will set gameRunning to 1 if successfully initialized
		// TODO: implement boolean instead of void
		astraInit();

		if (gameRunning)
		{
			// Start new thread for OpenGL drawing, in this thread update camera
			std::thread t1(setDrawing, &iArgc, cppArgv);
			std::thread t2(sendData);

			// Currently stream is always running
			// TODO: Combine stream.streamRunning with gameRunning, useless to use both variables
			while (stream.streamRunning)
			{
				astra_update();
				//std::cout << "Ovo je tip: " << maskColorFrame.stream_type << std::endl;
			}

			// After stream is not running, set control variable and properly terminate camera
			gameRunning = 0;

			astra::terminate();

			// Wait for thread to finish
			t1.join();
		}
		var = 0;
		/*}
		else if (var == 2)
		{
		break;
		}*/
	}
	return 0;
}

void drawLine(float x1, float y1, /*float z1, */float x2, float y2/*, float z2*/)
{

	/*x1 += 320.0;
	x2 += 320.0;
	y1 += 240.0;
	y2 += 240.0;*/

	glBegin(GL_LINES);
	glVertex2f(x1, h - y1/*, -z1*/);
	glVertex2f(x2, h - y2/*, -z2*/);
	glEnd();
}

void Draw()
{
	/*
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);//brise ekran
	glMatrixMode(GL_MODELVIEW);
	light();
	glLoadIdentity();
	gluLookAt(0.0, 0.0, 1500.0, //gdje je kamera (eye)
	0.0, 0.0, 1.0, //gdje gledamo (center)
	0.0f, 1.0f, 0.0f); //up-vektor (up)
	*/
	//std::cout << "Ovo je tip: " << maskColorFrame.stream_type << std::endl;

	//do {
	//astra_update();
	//} while (!listener.is_finished());



	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); // Clear color and depth buffers
	glMatrixMode(GL_MODELVIEW);     // To operate on model-view matrix

	glLoadIdentity();                 // Reset the model-view matrix

	auto bodies = stream.getBodies();
	auto nearestPoint = stream.m_nearestPoint;

	//glClear(GL_COLOR_BUFFER_BIT); // clear display window

	//glMatrixMode(GL_MODELVIEW);
	//glLoadIdentity();

	glPointSize(5.0f);

	/*glBegin(GL_POINTS);
	glColor3f(1.0, 1.0, 1.0);
	glVertex2i(nearestPoint->x, nearestPoint->y);
	glEnd();*/

	//std::cout << stream.m_pointFrame.length() << std::endl;

	for (auto body : bodies)
	{
		auto joints = body.joints();
		// ovo je samo referenca da se kasnije mogu postaviti na nešto, jer auto x ne mozes definirati bez dodjele vrijednosti
		auto mostLeftJoint = joints[0];
		auto mostRightJoint = joints[0];
		auto mostTopJoint = joints[0];
		auto mostBottomJoint = joints[0];
		auto mostFrontJoint = joints[0];
		auto mostBackJoint = joints[0];
		for (/*auto joint : joints*/int i = 0; i < joints.size() - 1; i++)
		{

			const astra::Matrix3x3* rotationX = &joints[i].orientation();
			points[i][1].x = rotationX->m01();
			points[i][1].y = rotationX->m11();
			points[i][1].z = rotationX->m21();
			points[i][2].x = rotationX->m02();
			points[i][2].y = rotationX->m12();
			points[i][2].z = rotationX->m22();

			const astra::Vector3f* data = stream.m_pointFrame.data();

			if (joints[i].status() == astra::JointStatus::NotTracked)
			{
				continue;
			}

			if (joints[i].world_position().x < mostLeftJoint.world_position().x)
			{
				mostLeftJoint = joints[i];
			}
			if (joints[i].world_position().x > mostRightJoint.world_position().x)
			{
				mostRightJoint = joints[i];
			}
			if (joints[i].world_position().y > mostTopJoint.world_position().y)
			{
				mostTopJoint = joints[i];
			}
			if (joints[i].world_position().y < mostBottomJoint.world_position().y)
			{
				mostBottomJoint = joints[i];
			}
			if (joints[i].world_position().z > mostFrontJoint.world_position().z)
			{
				mostFrontJoint = joints[i];
			}
			if (joints[i].world_position().z < mostBackJoint.world_position().z)
			{
				mostBackJoint = joints[i];
			}

			auto xi = joints[i].world_position().x / 100;
			auto yi = joints[i].world_position().y / 100;
			auto zi = -joints[i].world_position().z / 100;

			points[i][0].x = joints[i].world_position().x;
			points[i][0].y = joints[i].world_position().y;
			points[i][0].z = joints[i].world_position().z;
			hasNewData = true;

			glBegin(GL_POINTS);
			glColor3f(1.0, 0.0, 0.0);
			glVertex3f(xi, yi, zi);
			glEnd();

		}
		glLineWidth(5.0);
		glBegin(GL_LINES);
		glColor3f(1.0, 1.0, 0.0);
		glVertex3f(mostLeftJoint.world_position().x, mostBottomJoint.world_position().y, mostBackJoint.world_position().z);
		/*glVertex3f(mostRightJoint.world_position().x, mostBottomJoint.world_position().y, mostBackJoint.world_position().z);
		glVertex3f(mostLeftJoint.world_position().x, mostBottomJoint.world_position().y, mostFrontJoint.world_position().z);
		glVertex3f(mostRightJoint.world_position().x, mostBottomJoint.world_position().y, mostFrontJoint.world_position().z);
		glVertex3f(mostLeftJoint.world_position().x, mostTopJoint.world_position().y, mostBackJoint.world_position().z);
		glVertex3f(mostRightJoint.world_position().x, mostTopJoint.world_position().y, mostBackJoint.world_position().z);*/
		glVertex3f(mostLeftJoint.world_position().x, mostTopJoint.world_position().y, mostFrontJoint.world_position().z);
		/*glVertex3f(mostRightJoint.world_position().x, mostTopJoint.world_position().y, mostFrontJoint.world_position().z);*/
		glEnd();

		auto xiLeftHand = joints[(int)astra::JointType::LeftHand].world_position().x / 100;
		auto yiLeftHand = joints[(int)astra::JointType::LeftHand].world_position().y / 100;
		auto ziLeftHand = -joints[(int)astra::JointType::LeftHand].world_position().z / 100;

		auto xiLeftWrist = joints[(int)astra::JointType::LeftWrist].world_position().x / 100;
		auto yiLeftWrist = joints[(int)astra::JointType::LeftWrist].world_position().y / 100;
		auto ziLeftWrist = -joints[(int)astra::JointType::LeftWrist].world_position().z / 100;


		const astra::Vector3f* data = stream.m_pointFrame.data();

		for (int i = 0; i < stream.m_pointFrame.length(); i += 2)
		{
			const astra::Vector3f* dataPoint = data + i;
			if (dataPoint->x == 0) continue;

			//ako je data->x manji od jointpoint->x + neki_radius onda je unutar trazenog prostora
			if ((dataPoint->x / 100 > mostLeftJoint.world_position().x / 100 - OFFSET)
				&& (dataPoint->x / 100 < mostRightJoint.world_position().x / 100 + OFFSET)
				&& (dataPoint->y / 100 < mostTopJoint.world_position().y / 100 + OFFSET)
				&& (dataPoint->y / 100 > mostBottomJoint.world_position().y / 100 - OFFSET)
				&& (dataPoint->z / 100 > mostBackJoint.world_position().z / 100 - OFFSET - 1)
				&& (dataPoint->z / 100 < mostFrontJoint.world_position().z / 100 + OFFSET))
			{
				//Then I am in range
				glPointSize(1.0);
				glBegin(GL_POINTS);
				glColor3f(0.0, 1.0, 1.0);
				glVertex3f(dataPoint->x / 100, dataPoint->y / 100, -dataPoint->z / 100);
				glEnd();
			}
		}
	}

	glutSwapBuffers();
}
#pragma region meniNepotrebneFunc

void Timer(int iUnused)
{
	if (gameRunning)
	{
		glutPostRedisplay();
		glutTimerFunc(20, Timer, 0);
	}
	else
	{
		glutLeaveMainLoop();
	}
}

void Initialize()
{
	glClearColor(0.0, 0.0, 0.0, 0.0);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	w = glutGet(GLUT_WINDOW_WIDTH);
	h = glutGet(GLUT_WINDOW_HEIGHT);

	gluOrtho2D(0.0, w, 0.0, h);
}

void resize(int w, int h)
{
	glMatrixMode(GL_PROJECTION);
	glViewport(0, 0, w, h);
	glLoadIdentity();
	gluPerspective(45, w / h, 1, 10000); // view angle u y, aspect, near, far
}

void keyboard(unsigned char key, int, int)
{
	if (key == 'r') {
		//reset();
	}
	else if (key == 'q') {
		exit(0);
	}
	else if (key == 'a')
	{
		camX--;
	}
	else if (key == 's')
	{
		camY--;
	}
	else if (key == 'd')
	{
		camX++;
	}
	else if (key == 'w')
	{
		camY++;
	}
}

void astraInit()
{
	//Status is always returning SUCCESS, don't know why
	astra_status_t status = astra::initialize();
	const char* licenseString = "Okk/ujeQpYTstUwg8FPWhWztPS/nR31ZNAOMioFZMXpPegCjscSpAse7zvsEOoL9H1CafIzWEd++wyiyzU5jCk5hbWU9S3Jpc3RpamFuIExlbmFjfE9yZz1Vbml2ZXJzaXR5IG9mIFJpamVrYXxDb21tZW50PXxFeHBpcmF0aW9uPTk5OTk5OTk5OTk=";
	orbbec_body_tracking_set_license(licenseString);


	// Currently there is no license key so if we uncomment this line, astra won't work due to invalid license key
	//const char* licenseString = "<INSERT LICENSE KEY HERE>";
	//orbbec_body_tracking_set_license(licenseString);

	reader = sensor.create_reader();

	auto depthStream = reader.stream<astra::DepthStream>();
	depthStream.start();

	auto colorStream = reader.stream<astra::ColorStream>();

	auto maskedColorViewer = reader.stream<astra::MaskedColorStream>();
	maskedColorViewer.start();
	
	astra_usb_info_t info = colorStream.usb_info();
	std::cout << "pid: " << info.pid << ", vid: " << info.vid << std::endl;

	if (!reader.get_latest_frame())
	{
		std::cout << "Camera not initialized properly\n";

		gameRunning = 0;

		return;
	}


	auto bodyStream = reader.stream<astra::BodyStream>();
	bodyStream.start();
	//auto features = bodyStream.get_body_features(bodyStream.id);

	auto pointStream = reader.stream<astra::PointStream>();
	pointStream.start();

	//novi reader za rgb sliku korisnika
	reader.stream<astra::MaskedColorStream>().start();
	const int maxFramesToProcess = 100;	
	reader.add_listener(listener);

	//auto handStream = reader.stream<astra::HandStream>();
	//handStream.start();

	reader.add_listener(stream);
}


#pragma endregion

//Send points through socket
bool sendPoint(SOCKET sock, char* buffer, int len) // ili char len = 640 480 *4
{

	int sendResult = send(sock, buffer, len, 0);
	if (sendResult == SOCKET_ERROR)
	{
		return false;
	}
	_sleep(1);
	//printf("Broj: %d", ++sendCnt);
	return true;
}



void sendData()
{
	//wait for tracking to kick in
	/*Sleep(5000);*/
	
	string ipAddress = "127.0.0.1";     // IP Address of the server (localhost)
	int port = 54000;           // Listening port # on the server
	auto bodies = stream.getBodies();
	auto joints = bodies[0].joints();

	for (auto body : bodies)
	{
		auto joints = body.joints();
	}


	// Initialize WinSock
	WSADATA data;
	WORD ver = MAKEWORD(2, 2);
	int wsResult = WSAStartup(ver, &data);
	if (wsResult != 0)
	{
		cerr << "Can't start Winsock, Err # " << wsResult << endl;
		return;
	}

	// Create socket
	SOCKET sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == INVALID_SOCKET)
	{
		cerr << "Can't create socket, Err # " << WSAGetLastError() << endl;
		return;
	}

	// Fill in a hint structure
	sockaddr_in hint;
	hint.sin_family = AF_INET;
	hint.sin_port = htons(port);
	inet_pton(AF_INET, ipAddress.c_str(), &hint.sin_addr);

	// Connect to server
	int connResult = connect(sock, (sockaddr*)&hint, sizeof(hint));
	while (connResult == SOCKET_ERROR)
	{
		cerr << "Can't connect to server, gre[ka # " << WSAGetLastError() << endl;
		//closesocket(sock);
		//WSACleanup();
		Sleep(2000);
		connResult = connect(sock, (sockaddr*)&hint, sizeof(hint));
		//return;
	}


	int lastFrameIndex = 1;
	do
	{
		Sleep(20);		// 1 message each 33ms is close to 30FPS
		//ZeroMemory(strPoint, bufSize);
		if (hasNewData)
		{
			//lastFrame.acquire()

			//ZeroMemory(strPoint, bufSize);
			/*for (int i = 0; i < lastFrame.height()*lastFrame.width() ; i++)
			{ 
				ZeroMemory(strPoint, bufSize);
				for (int j = 0; j < slova_po_redu; j++)
				{*/
					//sprintf(strPoint + strlen(strPoint), "%d;%.0f;%.0f;%.0f;%.3f;%.3f;%.3f;%.3f;%.3f;%.3f ", i, points[i][0].x, points[i][0].y, points[i][0].z, points[i][1].x, points[i][1].y, points[i][1].z, points[i][2].x, points[i][2].y, points[i][2].z);
					/*ZeroMemory(strPoint, bufSize);
					char red[2];
					strcpy(red, dec2Hex((int)lastFrame.data()[i].r).c_str());
					char green[2];
					strcpy(green, dec2Hex((int)lastFrame.data()[i].g).c_str());
					char blue[2];
					strcpy(blue, dec2Hex((int)lastFrame.data()[i].b).c_str());
					char alpha[2];
					strcpy(alpha, dec2Hex((int)lastFrame.data()[i].alpha).c_str());

					sprintf(strPoint + strlen(strPoint), "%c%c%c%c%c%c", red[0],red[1], green[0], green[1], blue[0], blue[1]/*, alpha[0], alpha[1]);
					if (lastFrameIndex == lastFrame.frame_index())
					{
						sprintf(strPoint + strlen(strPoint), ";");
					}
					else
					{
						sprintf(strPoint + strlen(strPoint), "#");
						lastFrameIndex = lastFrame.frame_index();
					}
					*/
					Mat imageToSend = imread("horse.jpg");

					if (imageToSend.empty()) // Check for failure
					{
						cout << "Could not open or find the image" << endl;
						//system("pause"); //wait for any key press
						//return -1;
					}

				
					/*cv::Mat imageWithData = cv::Mat(sizeof(v_char), 1, CV_8UC4, v_char.data()).clone();
					Mat reshapedImage = imageWithData.reshape(0, 480);*/


					/*std::ifstream ifs("rocco.png");
					
					if (ifs)
					{
						std::vector<byte> dataToSend = std::vector<byte>(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());

						//If you really need it in a string you can initialize it the same way as the vector
						//std::string data2 = std::string(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());

						//std::for_each(dataToSend.begin(), dataToSend.end(), [](char c) { std::cout << c; });

						//std::cin.get();
						if (!sendPoint(sock, dataToSend.data(), dataToSend.size()))
						{
							//printf("Could not send point");
							std::cout << "Could not send point";
							break;
						}
						else {
							std::cout << dataToSend.data();
							//printf("poslano");
						}
					}*/
					


					hasNewData = false;		// For checking if the tracking gave out some new data, so it stops sending if the tracking isn't being used}

					
					//strcpy(strPoint, lastFrame.data);

					//string s = 
					//int_hex(7);
					//cout << dec2Hex(7);
					//printf("%s", s);
				//}

				//polje koje ima duzinu visina*širna*broj px, u tom polju for petlju
				//
				//	astra::RgbaPixel *buffer = new astra::RgbaPixel[640 * 480 * 4]();
				//	lastFrame.copy_to(buffer);

					/*for (int i = 0; i < 640; i++)
					{
						/*strPoint[i * 4] = (int)lastFrame.data()[i].r;
						strPoint[i * 4 + 1] = (int)lastFrame.data()[i].g;
						strPoint[i * 4 + 2] = (int)lastFrame.data()[i].b;
						strPoint[i * 4 + 3] = (int)lastFrame.data()[i].alpha;
						auto dataToSend = *lastFrame.data();
						std::cout << "datatoSend" << (int)dataToSend.r << std::endl;
						printf("%d%d%d%d", (int)lastFrame.data()[i].r, (int)lastFrame.data()[i].g, (int)lastFrame.data()[i].b, (int)lastFrame.data()[i].alpha);
						sprintf(strPoint + strlen(strPoint), "%d%d%d%d", (int)lastFrame.data()[i].r, (int)lastFrame.data()[i].g, (int)lastFrame.data()[i].b, (int)lastFrame.data()[i].alpha);
					}*/
					//std::cout << "Pozvan" << strPoint[9] << std::endl;
					//strcpy(strPoint, buffer);

				//std::cout << strPoint << "\n";
			//}
			
		}
		
	} while (gameRunning);
	// Gracefully close down everything
	closesocket(sock);
	WSACleanup();
}

string dec2Hex(int dec)
{
	/*string s = "";
	if (value == 0)
		return s;

	int rem = value % 16;
	value /= 16;
	dec2Hex(value); //first execute recurency and print next value

					  //after return print the less significant digit
	if (rem > 9)
		//cout << (char)(rem - 10 + 'A');
		s += (char)(rem - 10 + 'A');
	else
		//cout << rem;
		s += rem;*/
	string s = "";
	int rem;
	while (dec > 0)   // Do this whilst the quotient is greater than 0.
	{
		rem = dec % 16; // Get the remainder.
		if (rem > 9)
		{
			// Map the character given that the remainder is greater than 9.
			switch (rem)
			{
			case 10: s = "A" + s; break;
			case 11: s = "B" + s; break;
			case 12: s = "C" + s; break;
			case 13: s = "D" + s; break;
			case 14: s = "E" + s; break;
			case 15: s = "F" + s; break;
			}
		}
		else
		{
			s = char(rem + 48) + s; // Converts integer (0-9) to ASCII code.
									// x + 48 is the ASCII code for x digit (if 0 <= x <= 9)
		}
		dec = dec / 16;
	}
	if (s == "") // if the number was 0, the string will remain empty
		//cout << "0";
		return s;
	else if (dec < 16)
		return "0" + s;
	else
		//cout << s;
		return s;
	
}


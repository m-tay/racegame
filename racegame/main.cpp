#include <glew.h> // include the GLEW header file  
#include <glut.h> // include the GLUT header file  
#include <SOIL.h> // include the SOIL header file (for loading images)
#include <iostream>
#include <cmath>

// arrays to store all possible keystates
bool* keyStates = new bool[256]();
bool* keySpecialStates = new bool[256]();

// stores all the textures
GLuint texture[2];

// global variables
float carLength = 1.0f;
float carWidth = 0.5f;
float decelRate = 0.00009f;
float maxSpeed = 0.01f;

// camera positions
float cam_x = 0;
float cam_y = 0;



class Car {
public:
	float pos_x;
	float pos_y;
	float rot;
	float acceleration;
	float vel_x;
	float vel_y;

	// constructor sets car's default position
	Car(float x, float y) {
		pos_x = x;
		pos_y = y;
		rot = 0.0f;
		acceleration = 0.0f;
	}
};

// initialise objects for game
Car playerCar = Car(0, 0);


// processes key presses
void keyOperations(void) {
	if (keyStates[27]) // escape
		exit(0);

	if (keyStates['w'] && playerCar.acceleration < maxSpeed)
		playerCar.acceleration += 0.00009f;

	if (keyStates['s'])
		playerCar.acceleration -= 0.00009f;

	if (keyStates['a'])
		playerCar.rot += 0.05f;

	if (keyStates['d'])
		playerCar.rot -= 0.05f;
		
}



// processes special key presses
void keySpecialOperations(void) {
	// special keys
}

// texture loader
int loadTextures()
{
	// loads image directly as texture
	texture[0] = SOIL_load_OGL_texture
	(
		"textures/grass.jpg",
		SOIL_LOAD_AUTO,
		SOIL_CREATE_NEW_ID,
		SOIL_FLAG_MIPMAPS
	);

	texture[1] = SOIL_load_OGL_texture
	(
		"textures/car1.png",
		SOIL_LOAD_AUTO,
		SOIL_CREATE_NEW_ID,
		SOIL_FLAG_MIPMAPS
	);

	// if doesn't load properly, return false
	if (texture[0] == 0 || texture[1] == 0)
		return false;


	// bind and generate texture
	glBindTexture(GL_TEXTURE_2D, texture[0]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	glBindTexture(GL_TEXTURE_2D, texture[1]);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);

	// enable blending on the alpha channel
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	return true;
}

// draws background
void renderBackground(void) {
	// enable and bind texture
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture[0]); 
	
	// draw vertices
	glBegin(GL_QUADS);
	glTexCoord2d(0.0, 0.0);
	glVertex3f(-5000.0f, -5000.0f, -1.0f);	// bottom left

	glTexCoord2d(0.0, 5000.0);
	glVertex3f(-5000.0f, 5000.0f, -1.0f);	// top left

	glTexCoord2d(5000.0, 5000.0);
	glVertex3f(5000.0f, 5000.0f, -1.0f);	// top right

	glTexCoord2d(5000.0, 0.0);
	glVertex3f(5000.0f, -5000.0f, -1.0f);	// bottom right
	
	glEnd();
	glDisable(GL_TEXTURE_2D); // disable texture drawing
}

void renderCars(void) {
	// translate to centre of player car, rotate, then translate back
	glTranslatef(playerCar.pos_x, playerCar.pos_y, 0.0f);
	glRotatef(playerCar.rot, 0.0, 0.0, 1.0);
	glTranslatef(-playerCar.pos_x, -playerCar.pos_y, 0.0f);

	// apply velocity vector
	playerCar.vel_x = -sin(playerCar.rot * (3.14159265359 / 180));	// get x velocity after converting to rads
	playerCar.vel_y = cos(playerCar.rot * (3.14159265359 / 180));	// get y velocity after converting to rads

	playerCar.pos_x += playerCar.acceleration * playerCar.vel_x;
	playerCar.pos_y += playerCar.acceleration * playerCar.vel_y;

	// enable and bind texture
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture[1]);

	// draw car
	glBegin(GL_QUADS);
	glTexCoord2d(0.0, 0.0);
	glVertex3f(playerCar.pos_x - (carWidth / 2), playerCar.pos_y - (carLength / 2), 0.0f); // bottom left

	glTexCoord2d(0.0, 1.0);
	glVertex3f(playerCar.pos_x - (carWidth / 2), playerCar.pos_y + (carLength / 2), 0.0f); // top left

	glTexCoord2d(1.0, 1.0);
	glVertex3f(playerCar.pos_x + (carWidth / 2), playerCar.pos_y + (carLength / 2), 0.0f); // top right
	
	glTexCoord2d(1.0, 0.0);
	glVertex3f(playerCar.pos_x + (carWidth / 2), playerCar.pos_y - (carLength / 2), 0.0f); // bottom right


	glEnd();

	glDisable(GL_TEXTURE_2D);

}

void camera(void) {
	glTranslatef(-cam_x, -cam_y, 0.0f);
}

void display(void) {
	// process key operations
	keyOperations();
	keySpecialOperations();

	// clear background to a colour
	glClearColor(0.0f, 0.5f, 0.5f, 1.0f);

	// clear the colour buffer
	glClear(GL_COLOR_BUFFER_BIT);

	// load the identity matrix to reset drawing locations
	glLoadIdentity();

	// update camera
	cam_x = playerCar.pos_x;
	cam_y = playerCar.pos_y;
	camera();

	// push back everything 5 units on the z axis, so we can see it
	glTranslatef(0.0f, 0.0f, -5.0f);

	renderBackground();
	renderCars();

	// reset rotation
	if (playerCar.rot > 360)
		playerCar.rot = 0;

	if (playerCar.rot < -360) 
		playerCar.rot = 0;

	
	//std::cout << "playerCar pos_x = " << playerCar.pos_x << "\n";
	//std::cout << "playerCar pos_y = " << playerCar.pos_y << "\n";
	//std::cout << "playerCar vel_x = " << playerCar.vel_x << "\n";
	//std::cout << "playerCar vel_y = " << playerCar.vel_y << "\n";
	//std::cout << "playerCar rot  = " << playerCar.rot << "\n";
	//std::cout << "playerCar acc  = " << playerCar.acceleration << "\n";
	

	// displays newly drawn buffer
	glutSwapBuffers();


}

void timer(int t) {
	playerCar.pos_y += playerCar.acceleration;
	
	if (playerCar.acceleration > 0)
		playerCar.acceleration -= decelRate;
	
	if (playerCar.acceleration < 0)
		playerCar.acceleration = 0;

	glutTimerFunc(50, timer, 0);
}

// method to reshape windows
void reshape(int width, int height) {

	// set viewport to size of window
	glViewport(0, 0, (GLsizei)width, (GLsizei)height);

	// switch to projection matrix
	glMatrixMode(GL_PROJECTION);

	// reset the projection matrix to the identity matrix (cleaning up)
	glLoadIdentity();

	// set up view: fov, aspect ratio, near + far plane
	gluPerspective(60, (GLfloat)width / (GLfloat)height, 1.0, 100.0);

	// set back to model view matrix
	glMatrixMode(GL_MODELVIEW);
}

void keyPressed(unsigned char key, int x, int y) {
	keyStates[key] = true;
}

void keyUp(unsigned char key, int x, int y) {
	keyStates[key] = false;
}

void keySpecial(int key, int x, int y) {
	keySpecialStates[key] = true;
}

void keySpecialUp(int key, int x, int y) {
	keySpecialStates[key] = false;
}

int main(int argc, char **argv) {

	// initialise GLUT
	glutInit(&argc, argv);

	// set the display mode
	glutInitDisplayMode(GLUT_DOUBLE);

	// set the size and position of the GLUT window
	glutInitWindowSize(600, 600);
	glutInitWindowPosition(100, 100);

	// create the window, with a title
	glutCreateWindow("Matt's OpenGL car thing");

	// use the display() function for displaying
	glutDisplayFunc(display);

	// set the timer function
	glutTimerFunc(50, timer, 0);

	// set the display() method to be an idle method
	glutIdleFunc(display);

	// use the reshape() function for reshaping
	glutReshapeFunc(reshape);

	// set the keypress/keyup functions
	glutKeyboardFunc(keyPressed);
	glutKeyboardUpFunc(keyUp);

	// set the special keys/keyup functions
	glutSpecialFunc(keySpecial);
	glutSpecialUpFunc(keySpecialUp);

	// load textures, quit if failed for any reason
	if (!loadTextures())
		return false;


	// enter GLUTs main loop
	glutMainLoop();


}
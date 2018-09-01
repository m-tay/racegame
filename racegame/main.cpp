#include <glew.h> // include the GLEW header file  
#include <glut.h> // include the GLUT header file  
#include <SOIL.h> // include the SOIL header file (for loading images)
#include <iostream>
#include <cmath>
#include <vector>

// arrays to store all possible keystates
bool* keyStates = new bool[256]();
bool* keySpecialStates = new bool[256]();

// stores all the textures
GLuint texture[2];

// global variables
bool debugMode = true;	// draws bounding boxes
float carLength = 1.0f;
float carWidth = 0.5f;
float decelRate = 0.00009f;
float maxSpeed = 0.01f;

// pre-compute divisions 
float carLengthHalf = carLength / 2;
float carWidthHalf = carWidth / 2;
float piOver180 = 3.14159265359 / 180;

// camera positions
float cam_x = 0;
float cam_y = 0;

// line (track and car drawing) geometry
struct line {
	float x1, y1, x2, y2; 
};

// stores each line of the track
std::vector<line> trackPoints;

// create the lines that define the track
void initTrack() {
	trackPoints.push_back({ -1.0f, 1.0f, -2.0f, -2.0f});
	trackPoints.push_back({ -2.0f, -2.0f, 10.0f, -1.5f });
	trackPoints.push_back({ 10.0f, -1.5f , 10.0f, 10.0f});
	trackPoints.push_back({ 10.0f, 10.0f , -1.0f, 1.0f });
}

class Car {
public:
	float pos_x;	// stores the CENTRE POINT of the car
	float pos_y;
	float rot;		// rotation
	float acceleration;
	float vel_x;	// stores car velocity
	float vel_y;

	// constructor sets car's default position
	Car(float x, float y) {
	pos_x = x;
	pos_y = y;
	rot = 0.0f;
	acceleration = 0.0f;
	}

	// returns the corners used for oriented bounding box collision detection
	std::vector<line> getCarCorners() {

		// create corner coords, store in vector
		float tl_x, tl_y, tr_x, tr_y, bl_x, bl_y, br_x, br_y;
		float tl_xr, tl_yr, tr_xr, tr_yr, bl_xr, bl_yr, br_xr, br_yr;

		// set corner values for collision detection
		tl_x = pos_x - carWidthHalf;
		tl_y = pos_y + carLengthHalf;
		tr_x = pos_x + carWidthHalf;
		tr_y = pos_y + carLengthHalf;
		bl_x = pos_x - carWidthHalf;
		bl_y = pos_y - carLengthHalf;
		br_x = pos_x + carWidthHalf;
		br_y = pos_y - carLengthHalf;

		// translation to origin
		tl_x -= pos_x;
		tl_y -= pos_y;
		tr_x -= pos_x;
		tr_y -= pos_y;
		bl_x -= pos_x;
		bl_y -= pos_y;
		br_x -= pos_x;
		br_y -= pos_y;
		
		// rotation
		tl_xr = (tl_x * cos((rot * piOver180)) - (tl_y * sin((rot * piOver180))));
		tl_yr = (tl_x * sin((rot * piOver180)) + (tl_y * cos((rot * piOver180))));

		tr_xr = (tr_x * cos((rot * piOver180)) - (tr_y * sin((rot * piOver180))));
		tr_yr = (tr_x * sin((rot * piOver180)) + (tr_y * cos((rot * piOver180))));

		bl_xr = (bl_x * cos((rot * piOver180)) - (bl_y * sin((rot * piOver180))));
		bl_yr = (bl_x * sin((rot * piOver180)) + (bl_y * cos((rot * piOver180))));

		br_xr = (br_x * cos((rot * piOver180)) - (br_y * sin((rot * piOver180))));
		br_yr = (br_x * sin((rot * piOver180)) + (br_y * cos((rot * piOver180))));

		// translate back
		// translation to origin
		tl_xr += pos_x;
		tl_yr += pos_y;
		tr_xr += pos_x;
		tr_yr += pos_y;
		bl_xr += pos_x;
		bl_yr += pos_y;
		br_xr += pos_x;
		br_yr += pos_y;

		std::vector<line> carCorners;
		// left
		carCorners.push_back({ tl_xr, tl_yr, bl_xr, bl_yr });

		// top
		carCorners.push_back({ tl_xr, tl_yr, tr_xr, tr_yr });

		// right
		carCorners.push_back({ tr_xr, tr_yr, br_xr, br_yr });

		// bottom
		carCorners.push_back({ br_xr, br_yr, bl_xr, bl_yr });

		return carCorners;
	}
};

// initialise objects for game
Car playerCar = Car(0, 0);

// takes in two vectors containing <lines>, checks to see if any of them intersect
bool isColliding(std::vector<line> a, std::vector<line> b) {
	float dist1 = 0;
	float dist2 = 0;

	// loop through every combination of both vectors
	for (int i = 0; i < a.size(); i++) {
		for (int j = 0; j < b.size(); j++) {

			// calculate distance to point of intersection
			float dist1 = ((b[j].x2 - b[j].x1) * (a[i].y1 - b[j].y1) - (b[j].y2 - b[j].y1) * (a[i].x1 - b[j].x1)) /
				((b[j].y2 - b[j].y1) * (a[i].x2 - a[i].x1) - (b[j].x2 - b[j].x1) * (a[i].y2 - a[i].y1));

			float dist2 = ((a[i].x2 - a[i].x1) * (a[i].y1 - b[j].y1) - (a[i].y2 - a[i].y1) * (a[i].x1 - b[j].x1)) /
				((b[j].y2 - b[j].y1) * (a[i].x2 - a[i].x1) - (b[j].x2 - b[j].x1) * (a[i].y2 - a[i].y1));

			// if dist1 and dist1 are both between 0 and 1, there is a collision
			if (dist1 >= 0 && dist1 <= 1 && dist2 >= 0 && dist2 <= 1)
				return true;
		}
	}

	// no collisions detected
	return false;
}

// processes key presses
void keyOperations(void) {
	if (keyStates[27]) // escape
		exit(0);
		
	if (keyStates['w'] && playerCar.acceleration < maxSpeed)
		playerCar.acceleration += 0.000009f;
	

	if (keyStates['s']) 
		playerCar.acceleration -= 0.000009f;


	if (keyStates['a']) {
		playerCar.rot += 0.05f;

		// disable rotation if colliding
		if (isColliding(playerCar.getCarCorners(), trackPoints)) {
			playerCar.rot -= 0.05f;
		}
	}

	if (keyStates['d']) {
		playerCar.rot -= 0.05f;
		
		// disable rotation if colliding
		if (isColliding(playerCar.getCarCorners(), trackPoints)) {
			playerCar.rot += 0.05f;
		}
	}
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
	glVertex3f(-5000.0f, -5000.0f, 0.0f);	// bottom left

	glTexCoord2d(0.0, 5000.0);
	glVertex3f(-5000.0f, 5000.0f, 0.0f);	// top left

	glTexCoord2d(5000.0, 5000.0);
	glVertex3f(5000.0f, 5000.0f, 0.0f);	// top right

	glTexCoord2d(5000.0, 0.0);
	glVertex3f(5000.0f, -5000.0f, 0.0f);	// bottom right
	
	glEnd();
	glDisable(GL_TEXTURE_2D); // disable texture drawing
}

void renderCars(void) {
	glPushMatrix();
		
	// translate to centre of player car, rotate, then translate back
	glTranslatef(playerCar.pos_x, playerCar.pos_y, 0.0f);
	glRotatef(playerCar.rot, 0.0, 0.0, 1.0);
	glTranslatef(-playerCar.pos_x, -playerCar.pos_y, 0.0f);

	// apply velocity vector
	playerCar.vel_x = -sin(playerCar.rot * piOver180);	// get x velocity after converting to rads
	playerCar.vel_y = cos(playerCar.rot * piOver180);	// get y velocity after converting to rads
	
	// collision handling
	if (isColliding(playerCar.getCarCorners(), trackPoints)) {
		// undo car's movement that caused collision
		playerCar.pos_x -= playerCar.acceleration * playerCar.vel_x; 
		playerCar.pos_y -= playerCar.acceleration * playerCar.vel_y;


		// set acceleration to 0 so next frame doesn't re-cause collision (will cause clipping)
		playerCar.acceleration = 0;
	}
	// if no collision happens, translate car
	else {
		playerCar.pos_x += playerCar.acceleration * playerCar.vel_x;
		playerCar.pos_y += playerCar.acceleration * playerCar.vel_y;
	}

	// enable and bind texture
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture[1]);

	// draw car
	glBegin(GL_QUADS);
	glTexCoord2d(0.0, 0.0);
	glVertex3f(playerCar.pos_x - carWidthHalf, playerCar.pos_y - carLengthHalf, 0.0f); // bottom left

	glTexCoord2d(0.0, 1.0);
	glVertex3f(playerCar.pos_x - carWidthHalf, playerCar.pos_y + carLengthHalf, 0.0f); // top left

	glTexCoord2d(1.0, 1.0);
	glVertex3f(playerCar.pos_x + carWidthHalf, playerCar.pos_y + carLengthHalf, 0.0f); // top right
	
	glTexCoord2d(1.0, 0.0);
	glVertex3f(playerCar.pos_x + carWidthHalf, playerCar.pos_y - carLengthHalf, 0.0f); // bottom right
	glEnd();
	
	
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();

	// draw bounding box (toggled by debugMode flag)
	if (debugMode) {
		glBegin(GL_LINES);
		for (int i = 0; i < playerCar.getCarCorners().size(); i++)
		{
			glVertex3f(playerCar.getCarCorners()[i].x1, playerCar.getCarCorners()[i].y1, 0.0f);
			glVertex3f(playerCar.getCarCorners()[i].x2, playerCar.getCarCorners()[i].y2, 0.0f);
		}
		glEnd();
	}
}

void renderTrack(void) {
	glBegin(GL_LINES);

	for (int i = 0; i < trackPoints.size(); i++)
	{
		glVertex3f(trackPoints[i].x1, trackPoints[i].y1, 0.0f);
		glVertex3f(trackPoints[i].x2, trackPoints[i].y2, 0.0f);
	}

	glEnd();

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
	renderTrack();

	// reset rotation
	if (playerCar.rot > 360)
		playerCar.rot = 0;

	if (playerCar.rot < -360) 
		playerCar.rot = 0;
	
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

	// initialise the track geometry vector
	initTrack();

	// enter GLUTs main loop
	glutMainLoop();


}
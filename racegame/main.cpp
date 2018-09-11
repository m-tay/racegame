//#include <glew.h> // include the GLEW header file  
#include <freeglut.h> // include the GLUT header file  
#include <SOIL.h> // include the SOIL header file (for loading images)
#include <iostream>
#include <cmath>
#include <vector>
#include <iomanip>
#include <sstream>

// arrays to store all possible keystates
bool* keyStates = new bool[256]();
bool* keySpecialStates = new bool[256]();

// stores all the textures
GLuint texture[4];

// global variables
bool debugMode = true;	// draws bounding boses
float carLength = 1.0f;
float carWidth = 0.5f;
float decelRate = 0.000009f;
float maxSpeed = 0.014f;
float rotRate = 0.2f;

// pre-compute divisions 
float carLengthHalf = carLength / 2;
float carWidthHalf = carWidth / 2;
float piOver180 = 3.14159265359f / 180;

// camera positions
float cam_x = 0;
float cam_y = 0;

// line (track and car drawing) geometry
struct point {
	float x, y;
};

struct edge {
	point p1, p2;
};

// stores each line of the track
std::vector<edge> trackEdges;
std::vector<edge> startLine;


// create the lines that define the track
void initTrack() {
	// outer edge
	trackEdges.push_back({ {-6.0f, -20.0f}, {-6.0f, 20.f} });
	trackEdges.push_back({ { -6.0f, 20.0f },{ 2.0f, 40.0f } });
	trackEdges.push_back({ { 2.0f, 40.0f },{ 40.0f, 40.0f } });
	trackEdges.push_back({ { 40.0f, 40.0f },{ 47.0f, 25.0f } });
	trackEdges.push_back({ { 47.0f, 25.0f },{ 40.0f, -42.0f } });
	trackEdges.push_back({ { 40.0f, -42.0f },{ 4.0f, -40.0f } });
	trackEdges.push_back({ { 4.0f, -40.0f },{ -6.0f, -20.0f } });

	// inner edge
	trackEdges.push_back({ {6.0f, 20.0f}, {6.0f, -20.f} });
	trackEdges.push_back({ {6.0f, 20.0f}, {12.0f, 30.0f} });
	trackEdges.push_back({ { 12.0f, 30.0f },{ 30.0f, 30.0f } });
	trackEdges.push_back({ { 30.0f, 30.0f },{ 33.0f, 25.0f } });
	trackEdges.push_back({ { 33.0f, 25.0f },{ 33.0f, -30.0f } });
	trackEdges.push_back({ { 33.0f, -30.0f },{ 30.0f, -33.0f } });
	trackEdges.push_back({ { 30.0f, -33.0f },{ 14.0f, -28.0f } });
	trackEdges.push_back({ { 14.0f, -28.0f },{ 6.0f, -20.0f } });

	startLine.push_back({ {-6.0f, 0.6f}, {6.0f, 0.6f} });
	startLine.push_back({ { -6.0f, 1.5f },{ 6.0f, 1.5f } });
}

// store and create waypoints used by AI cars
std::vector<point> waypoints;

void initWaypoints() {
	waypoints.push_back({ 5.0f, 20.0f });
	waypoints.push_back({ 12.0f, 31.0f });
	waypoints.push_back({ 23.0f, 35.0f });
	waypoints.push_back({ 36.0f, 28.0f });
	waypoints.push_back({ 39.0f, 23.0f });
	waypoints.push_back({ 37.0f, -24.0f });
	waypoints.push_back({ 30.0f, -36.0f });
	waypoints.push_back({ 6.5f, -29.0f });
	waypoints.push_back({ 3.0f, -21.0f });
	waypoints.push_back({ 3.0f, -1.0f });
}

class Car {
public:
	float pos_x;				// stores the CENTRE POINT of the car
	float pos_y;
	float rot;				// rotation
	float speed;				// overall speed
	bool isAccelerating;		// flags for car controls - used in timer()
	bool isBraking;
	bool turningLeft; 
	bool turningRight;
	float vel_x;				// stores car velocity
	float vel_y;
	bool playerControlled;	// flag to set whether cpu controlled
	int nextWaypoint;		// stores the waypoint cpu cars will seek

	// create corner coords - used for collision detection
	float tl_x, tl_y, tr_x, tr_y, bl_x, bl_y, br_x, br_y;
	float tl_xr, tl_yr, tr_xr, tr_yr, bl_xr, bl_yr, br_xr, br_yr;
	
	// constructor sets car's default position
	Car(float x, float y, bool humanPlayer) {
		pos_x = x;
		pos_y = y;
		playerControlled = humanPlayer;
		rot = 0.0f;
		speed = 0.0f;
		isAccelerating = false;
		isBraking = false;
		turningLeft = false;
		turningRight = false;
		nextWaypoint = 0;
	}

	// calculates car velocity
	void calcVelocity() {
		vel_x = -sin(rot * piOver180);	// get x velocity after converting to rads
		vel_y = cos(rot * piOver180);	// get y velocity after converting to rads
	}

	void accelerate() {
		if (speed < maxSpeed)
			speed += 0.00009f;
	}

	void turnLeft() {
		rot += rotRate;
	}

	void turnRight() {
		rot -= rotRate;
	}

// returns the corners used for oriented bounding box collision detection
std::vector<edge> edges() {
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
	tl_xr = (tl_x * cos(rot * piOver180) - (tl_y * sin(rot * piOver180)));
	tl_yr = (tl_x * sin(rot * piOver180) + (tl_y * cos(rot * piOver180)));

	tr_xr = (tr_x * cos(rot * piOver180) - (tr_y * sin(rot * piOver180)));
	tr_yr = (tr_x * sin(rot * piOver180) + (tr_y * cos(rot * piOver180)));

	bl_xr = (bl_x * cos(rot * piOver180) - (bl_y * sin(rot * piOver180)));
	bl_yr = (bl_x * sin(rot * piOver180) + (bl_y * cos(rot * piOver180)));

	br_xr = (br_x * cos(rot * piOver180) - (br_y * sin(rot * piOver180)));
	br_yr = (br_x * sin(rot * piOver180) + (br_y * cos(rot * piOver180)));

	// translate back
	tl_xr += pos_x;
	tl_yr += pos_y;
	tr_xr += pos_x;
	tr_yr += pos_y;
	bl_xr += pos_x;
	bl_yr += pos_y;
	br_xr += pos_x;
	br_yr += pos_y;

	std::vector<edge> edges;
	edges.push_back({ {tl_xr, tl_yr}, {bl_xr, bl_yr} }); // left
	edges.push_back({ {tl_xr, tl_yr}, {tr_xr, tr_yr} }); // top
	edges.push_back({ {tr_xr, tr_yr}, {br_xr, br_yr} }); // right
	edges.push_back({ {br_xr, br_yr}, {bl_xr, bl_yr} }); // bottom
	return edges;
}

float getAngleToWaypoint() {
	point newPosition;
	newPosition.x = pos_x + ((vel_x * speed) + 1);
	newPosition.y = pos_y + ((vel_y * speed) + 1);

	float angle;

	float a = 0, b = 0, c = 0; // triangle lengths
	a = sqrt(pow((tl_xr - bl_xr), 2) + pow((tl_yr - bl_yr), 2));
	b = sqrt(pow((bl_xr - waypoints[nextWaypoint].x), 2) + pow((bl_yr - waypoints[nextWaypoint].y), 2));
	c = sqrt(pow((tl_xr - waypoints[nextWaypoint].x), 2) + pow((tl_yr - waypoints[nextWaypoint].y), 2));

	angle = acos((pow(a, 2) + pow(b, 2) - pow(c, 2)) / (2 * a*b));

	return angle;
}

// does circle/circle collision detection to determine whether it has hit waypoint
// this will be used to then seek the next waypoint
void checkWaypointHit() {
	float dist_x = pos_x - waypoints[nextWaypoint].x;
	float dist_y = pos_y - waypoints[nextWaypoint].y;
	float dist = sqrt(pow(dist_x, 2) + pow(dist_y, 2));

	if (dist <= 1.5) {
		if (nextWaypoint == waypoints.size() - 1) { // check if final waypoint reached
			nextWaypoint = 0;						// reset back to first waypoint
		}
		else
			nextWaypoint++;
		}		
	}

};

// initialise cars
Car playerCar = Car(0, 0, true);
Car cpuCar1 = Car(2, 0, false);

// store all cars in a vector
std::vector<Car*> allCars({ &playerCar, &cpuCar1 });

// takes in two vectors containing <edges>, checks to see if any of them intersect
bool isColliding(std::vector<edge> &a, std::vector<edge> &b) {
	float dist1 = 0;
	float dist2 = 0;

	// loop through every combination of both vectors
	for (size_t i = 0; i < a.size(); i++) {
		for (size_t j = 0; j < b.size(); j++) {

			// calculate distance to point of intersection
			float dist1 = ((b[j].p2.x - b[j].p1.x) * (a[i].p1.y - b[j].p1.y) - (b[j].p2.y - b[j].p1.y) * (a[i].p1.x - b[j].p1.x)) /
				((b[j].p2.y - b[j].p1.y) * (a[i].p2.x - a[i].p1.x) - (b[j].p2.x - b[j].p1.x) * (a[i].p2.y - a[i].p1.y));

			float dist2 = ((a[i].p2.x - a[i].p1.x) * (a[i].p1.y - b[j].p1.y) - (a[i].p2.y - a[i].p1.y) * (a[i].p1.x - b[j].p1.x)) /
				((b[j].p2.y - b[j].p1.y) * (a[i].p2.x - a[i].p1.x) - (b[j].p2.x - b[j].p1.x) * (a[i].p2.y - a[i].p1.y));

			// if dist1 and dist1 are both between 0 and 1, there is a collision
			if (dist1 >= 0 && dist1 <= 1 && dist2 >= 0 && dist2 <= 1)
				return true;
		}
	}

	// no collisions detected
	return false;
}

// compares a vector of edges against a single edge
bool isColliding(std::vector<edge> &a, edge &b) {
	float dist1 = 0;
	float dist2 = 0;

	// loop through every combination of both vectors
	for (size_t i = 0; i < a.size(); i++) {
			// calculate distance to point of intersection
			float dist1 = ((b.p2.x - b.p1.x) * (a[i].p1.y - b.p1.y) - (b.p2.y - b.p1.y) * (a[i].p1.x - b.p1.x)) /
				((b.p2.y - b.p1.y) * (a[i].p2.x - a[i].p1.x) - (b.p2.x - b.p1.x) * (a[i].p2.y - a[i].p1.y));

			float dist2 = ((a[i].p2.x - a[i].p1.x) * (a[i].p1.y - b.p1.y) - (a[i].p2.y - a[i].p1.y) * (a[i].p1.x - b.p1.x)) /
				((b.p2.y - b.p1.y) * (a[i].p2.x - a[i].p1.x) - (b.p2.x - b.p1.x) * (a[i].p2.y - a[i].p1.y));

			// if dist1 and dist1 are both between 0 and 1, there is a collision
			if (dist1 >= 0 && dist1 <= 1 && dist2 >= 0 && dist2 <= 1)
				return true;		
	}

	// no collisions detected
	return false;
}

// processes key presses
void keyOperations(void) {
	if (keyStates[27]) // escape
		exit(0);
		
	if (keyStates['w']) 
		playerCar.isAccelerating = true;
	
	if (keyStates['s'])
		playerCar.isBraking = true;

	if (keyStates['a'])
		playerCar.turningLeft = true;
		
	if (keyStates['d'])
		playerCar.turningRight = true;
		
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
		"textures/Black_viper.png",
		SOIL_LOAD_AUTO,
		SOIL_CREATE_NEW_ID,
		SOIL_FLAG_MIPMAPS
	);

	texture[2] = SOIL_load_OGL_texture
	(
		"textures/Audi.png",
		SOIL_LOAD_AUTO,
		SOIL_CREATE_NEW_ID,
		SOIL_FLAG_MIPMAPS
	);

	texture[3] = SOIL_load_OGL_texture
	(
		"textures/Car.png",
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

// lap timer 
bool startLineHit = false;	// stores if player has hit start line
bool lapStarted = false;	// stores if player has moved past the start line
float seconds = 0.0f;
float bestLap = NULL;

void doLapTimer() {
	
	// car hitting finish line after a lap
	if (startLineHit == false && lapStarted == true && isColliding(playerCar.edges(), startLine[0])) {
		
		// if no best lap set, first lap time is the best lap
		if (bestLap == NULL)
			bestLap = seconds;

		// check for best lap time
		if (seconds < bestLap) {
			bestLap = seconds;
		}

		// reset lap timer
		seconds = 0.0f;
		
	}

	// reset flag for start line being hit
	startLineHit = false;

	// checks to see if playerCar is hitting startLine
	if (isColliding(playerCar.edges(), startLine[0])) { // startLine[0] is the bottom of the start/finish line
		startLineHit = true;

		// starts lap timer if not already started
		if (!lapStarted)
			lapStarted = true;
	}
}


// draws background
void renderBackground(void) {
	// enable and bind texture
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture[0]); 
	
	// draw vertices
	glBegin(GL_QUADS);
	glTexCoord2d(0.0, 0.0);
	glVertex3f(-100.0f, -100.0f, 0.0f);	// bottom left

	glTexCoord2d(0.0, 100.0);
	glVertex3f(-100.0f, 100.0f, 0.0f);	// top left

	glTexCoord2d(100.0, 100.0);
	glVertex3f(100.0f, 100.0f, 0.0f);	// top right

	glTexCoord2d(100.0, 0.0);
	glVertex3f(100.0f, -100.0f, 0.0f);	// bottom right
	
	glEnd();
	glDisable(GL_TEXTURE_2D); // disable texture drawing
}

void renderCars(Car &car) {
	glPushMatrix();
		
	// translate to centre of player car, rotate, then translate back
	glTranslatef(car.pos_x, car.pos_y, 0.0f);
	glRotatef(car.rot, 0.0, 0.0, 1.0);
	glTranslatef(-car.pos_x, -car.pos_y, 0.0f);

	// apply velocity vector
	car.calcVelocity();
	
	// collision detection against walls
	if (isColliding(car.edges(), trackEdges)) {
		// undo car's movement that caused collision
		car.pos_x -= car.speed * car.vel_x; 
		car.pos_y -= car.speed * car.vel_y;

		// set acceleration to 0 so next frame doesn't re-cause collision 
		car.speed = 0;
	}
	// if no collision happens, translate car
	else {
		car.pos_x += car.speed * car.vel_x;
		car.pos_y += car.speed * car.vel_y;
	}

	// collision detection against cpu cars 
	if (car.playerControlled) {
		if (isColliding(car.edges(), cpuCar1.edges())) {
			// undo car's movement that caused collision
			car.pos_x -= car.speed * car.vel_x;
			car.pos_y -= car.speed * car.vel_y;

			// set acceleration to 0 so next frame doesn't re-cause collision 
			car.speed = 0;
		}
	}

	// enable and bind texture
	glEnable(GL_TEXTURE_2D);
	glBindTexture(GL_TEXTURE_2D, texture[1]);

	// draw car
	glBegin(GL_QUADS);
	glTexCoord2d(0.0, 0.0);
	glVertex3f(car.pos_x - carWidthHalf, car.pos_y - carLengthHalf, 0.0f); // bottom left

	glTexCoord2d(0.0, 1.0);
	glVertex3f(car.pos_x - carWidthHalf, car.pos_y + carLengthHalf, 0.0f); // top left

	glTexCoord2d(1.0, 1.0);
	glVertex3f(car.pos_x + carWidthHalf, car.pos_y + carLengthHalf, 0.0f); // top right
	
	glTexCoord2d(1.0, 0.0);
	glVertex3f(car.pos_x + carWidthHalf, car.pos_y - carLengthHalf, 0.0f); // bottom right
	glEnd();
	
	
	glDisable(GL_TEXTURE_2D);
	glPopMatrix();

	// draw bounding box (toggled by debugMode flag)
	if (debugMode) {
		glBegin(GL_LINES);
		for (size_t i = 0; i < car.edges().size(); i++)
		{
			glVertex3f(car.edges()[i].p1.x, car.edges()[i].p1.y, 0.0f);
			glVertex3f(car.edges()[i].p2.x, car.edges()[i].p2.y, 0.0f);
		}
		glEnd();
		
		// draws waypointing triangle
		glBegin(GL_LINES);
		glVertex3f(cpuCar1.tl_xr, cpuCar1.tl_yr, 0.0f);
		glVertex3f(cpuCar1.bl_xr, cpuCar1.bl_yr, 0.0f);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(cpuCar1.bl_xr, cpuCar1.bl_yr, 0.0f);
		glVertex3f(waypoints[cpuCar1.nextWaypoint].x, waypoints[cpuCar1.nextWaypoint].y, 0.0f);
		glEnd();

		glBegin(GL_LINES);
		glVertex3f(cpuCar1.tl_xr, cpuCar1.tl_yr, 0.0f);
		glVertex3f(waypoints[cpuCar1.nextWaypoint].x, waypoints[cpuCar1.nextWaypoint].y, 0.0f);
		glEnd();

	}

}

void renderTrack(void) {
	glBegin(GL_LINES);

	for (size_t i = 0; i < trackEdges.size(); i++)
	{
		glVertex3f(trackEdges[i].p1.x, trackEdges[i].p1.y, 0.0f);
		glVertex3f(trackEdges[i].p2.x, trackEdges[i].p2.y, 0.0f);
	}

	glEnd();

	glBegin(GL_LINES);

	for (size_t i = 0; i < startLine.size(); i++)
	{
		glVertex3f(startLine[i].p1.x, startLine[i].p1.y, 0.0f);
		glVertex3f(startLine[i].p2.x, startLine[i].p2.y, 0.0f);
	}

	glEnd();


}

void renderWaypoints() {
	for (size_t i = 0; i < waypoints.size(); i++) {
		glBegin(GL_LINES);
		glVertex3f(waypoints[i].x + 0.2f, waypoints[i].y, 0.0f);
		glVertex3f(waypoints[i].x - 0.2f, waypoints[i].y, 0.0f);
		glVertex3f(waypoints[i].x, waypoints[i].y + 0.2f, 0.0f);
		glVertex3f(waypoints[i].x, waypoints[i].y - 0.2f, 0.0f);
		glEnd();
	}
}

void camera(void) {
	glTranslatef(-cam_x, -cam_y, 0.0f);
}

// variables for holding/formatting output
std::stringstream sstr;
std::string outputString;

void renderTimer() {
	// set to projection mode to draw as a HUD
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();		// save matrix
		glLoadIdentity(); // reset matrix
		glOrtho(0, 800, 600, 0, 0, -1); // ortho transform to width/height of screen
		glMatrixMode(GL_MODELVIEW);  // set back to modelview
		glPushMatrix(); 
			glLoadIdentity();
			
			// text rendering code

			// lap timer
			glRasterPos2i(20, 20);
			sstr << "Lap time  : " << std::fixed << std::setprecision(2) << seconds; // convert float to 2dp
			outputString = sstr.str();
			glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)outputString.c_str());
			sstr.str(std::string()); // clears string stream

			// best lap display
			glRasterPos2i(20, 50);
			sstr << "Best lap time : " << std::fixed << std::setprecision(2) << bestLap; // convert float to 2dp
			outputString = sstr.str();
			glutBitmapString(GLUT_BITMAP_HELVETICA_18, (const unsigned char*)outputString.c_str());
			sstr.str(std::string()); // clears string stream
	
			glMatrixMode(GL_PROJECTION);
		glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

void drawCoords() {
	for (int x = -6; x < 47; x++) {
		for (int y = -42; y < 35; y++)
		{
			glRasterPos2i(x, y);
			sstr << "(" << x << "," << y << ")";
			outputString = sstr.str();
			glutBitmapString(GLUT_BITMAP_HELVETICA_10, (const unsigned char*)outputString.c_str());
			sstr.str(std::string()); // clears string stream
		}
	}

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
	glTranslatef(0.0f, 0.0f, -10.0f);

	renderBackground();
	renderCars(cpuCar1);
	renderCars(playerCar);
	renderTrack();
	renderTimer();
	if (debugMode) {
		renderWaypoints();
		//drawCoords();		// incredibly slow, but useful for plotting track or waypoints
							// might find it useful to move position of playerCar to see your way round
	}
	doLapTimer();

	// reset rotation
	if (playerCar.rot > 360)
		playerCar.rot = 0;

	if (playerCar.rot < -360) 
		playerCar.rot = 0;
	
	// displays newly drawn buffer
	glutSwapBuffers();

}



// 5ms timer
void timer(int t) {

	cpuCar1.isAccelerating = true;

	// apply deceleration
	for (size_t i = 0; i < allCars.size(); i++) {
		if (allCars[i]->speed > 0)
			allCars[i]->speed -= decelRate;

		if (allCars[i]->speed < 0)
			allCars[i]->speed = 0;
	}

	// increment lap timer
	if (startLineHit || lapStarted) {
		seconds += 0.005;
	}

	// apply acceleration
	for (size_t i = 0; i < allCars.size(); i++) {
		if (allCars[i]->isAccelerating) {
			allCars[i]->accelerate();
		}
	}

	if (cpuCar1.getAngleToWaypoint() > 0.005) {
		cpuCar1.rot -= rotRate;
	}

	// apply turning
	for (size_t i = 0; i < allCars.size(); i++) {
		if (allCars[i]->turningLeft) {
			allCars[i]->turnLeft();
		}
	}

	for (size_t i = 0; i < allCars.size(); i++) {
		if (allCars[i]->turningRight) {
			allCars[i]->turnRight();
		}
	}


	cpuCar1.checkWaypointHit();

	// reset car states
	for (size_t i = 0; i < allCars.size(); i++)
	{
		allCars[i]->isAccelerating = false;
		allCars[i]->isBraking = false;
		allCars[i]->turningLeft = false;
		allCars[i]->turningRight = false;
	}

	// run timer in 5ms
	glutTimerFunc(5, timer, 0);
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
	glutInitWindowSize(800, 600);
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

	// initialise the waypoints for cpu cars
	initWaypoints();

	// enter GLUTs main loop
	glutMainLoop();


}
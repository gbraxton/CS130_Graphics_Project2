// Name: George Braxton
// Quarter Fall, Year: 2014
// Project 2
//
// This file is to be modified by the student.
// main.cpp
//---------CONTROLS-------------------
// Pan: arrow keys
// Zoom in: '+'
// Zoom out: '-'
// Rotate about x: 'x' (positive) 'X' (negative)
// Rotate about y: 'y' (positive) 'Y' (negative)
// Rotate about z: 'z' (positive) 'Z' (negative)
//-----------------------------------------------
////////////////////////////////////////////////////////////
#include <algorithm>
#include <cmath>
#include <fstream>
#include <GL/glut.h>
#include <limits.h>
#include <stdio.h>
#include <vector>

// point3d.h is a modification of the provided point2d.h for 3-space.
#include "point3d.h"

// A simple wrapper to store colors
struct Color3d
{
	float r;
	float g;
	float b;
	float a;

	Color3d()
    : r(0.0), g(0.0), b(0.0), a(1.0)
	{}
	Color3d(float r, float g, float b, float a = 1.0)
    : r(r), g(g), b(b), a(a)
	{}
};

void renderPixel(int x, int y, Color3d& color, float sz = 1.0)
{
	glPointSize(sz);
	glColor4f(color.r, color.g, color.b, color.a);
	glBegin(GL_POINTS);
	glVertex2i(x, y);
	glEnd();
}

// Vector3 and Ray structs used from provided file in Lab 7
struct Vector3{
    float x;
    float y;
    float z;
    Vector3() : x(0.0), y(0.0), z(0.0){}
    Vector3(float x, float y, float z)
    : x(x), y(y), z(z){}
    Vector3(const Vector3 & v)
    : x(v.x), y(v.y), z(v.z){}
	Vector3 operator+(const Vector3 & rhs) const
	{ return Vector3(x + rhs.x, y + rhs.y, z + rhs.z); }
	Vector3 operator-(const Vector3 & rhs) const
	{ return Vector3(x - rhs.x, y - rhs.y, z - rhs.z); }
	Vector3 operator*(float rhs) const
	{ return Vector3(x * rhs, y * rhs, z * rhs); }
	Vector3 operator/(float rhs) const
	{ return Vector3(x / rhs, y / rhs, z / rhs); }
	Vector3 operator+=(const Vector3 & rhs)
	{ x += rhs.x; y += rhs.y; z += rhs.z; return *this; }
	Vector3 operator-=(const Vector3 & rhs)
	{ x -= rhs.x; y -= rhs.y; z -= rhs.z; return *this; }
	Vector3 operator*=(float rhs)
	{ x *= rhs; y *= rhs; z *= rhs; return *this; }
	Vector3 operator/=(float rhs)
	{ x /= rhs; y /= rhs; z /= rhs; return *this; }
	float magnitude() const
	{ return sqrt(x * x + y * y + z * z); }
	void normalize()
	{ *this /= magnitude(); }
	Vector3 normalized() const
	{ return *this / magnitude(); }
	float dot(const Vector3 & rhs) const{
		return x * rhs.x + y * rhs.y + z * rhs.z;
	}
	Vector3 cross(const Vector3 & rhs) const{
		return Vector3(y * rhs.z - z * rhs.y,
				   z * rhs.x - x * rhs.z,
				   x * rhs.y - y * rhs.x);
	}
};

struct Triangle{
	Vector3 p1, p2, p3;
	Triangle(Vector3 p1, Vector3 p2, Vector3 p3): p1(p1), p2(p2), p3(p3){};
};

struct Ray{
    Vector3 origin;
    Vector3 direction;
    Ray() {Vector3 d(0.0, 0.0, 1.0);  direction = d;}
    Ray(const Vector3& o, const Vector3& dir)
    {
        origin = o;
        Vector3 d(0.0, 0.0, 1.0);
        float mag = dir.magnitude();
        if (mag > 0.0) {d = dir;}
        direction = d;
    }
};

// ------------------------ Constants-----------------------------------
const int WINDOW_WIDTH = 800;
const int WINDOW_HEIGHT = 800;
const float PAN_STEP = 25.0;
const float ZOOM_STEP = 50;
const float ROTATION_STEP = M_PI/16.0;
const double PI = M_PI;
const Vector3 lightSource(300.0, 1500.0, 200.0);
const float ambientIntensity = 0.5;
const float ambientReflection = 0.3;
const float diffuseIntensity = 0.4;
const float diffuseReflection = 0.9;
const float lightIntensity = 0.7;
const float specularReflection = 0.5;
const float shininess = 10.3;
// ----------------------------------------------------------------------

float viewPlaneInitialZ = -600.0; // Z position of view plane
Vector3 viewPlaneCenter(0.0, 0.0, 0.0); // will be initially set when model loads and adjusted w/ pan
float perspectiveFocusZOffset = -800.0; // distance of focus point from view plane
Triangle groundTriangle(Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0), Vector3(0.0, 0.0, 0.0));
Color3d groundColor(0.0, 1.0, 0.0);
Color3d colors[10] = { Color3d(0.0, 0.0, 1.0),
						Color3d(0.0, 1.0, 0.0),
						Color3d(1.0, 0.0, 0.0),
						Color3d(0.0, 1.0, 1.0),
						Color3d(1.0, 0.0, 1.0),
						Color3d(1.0, 1.0, 0.0),
						Color3d(1.0, 1.0, 1.0),
						Color3d(0.5, 0.5, 1.0),
						Color3d(0.5, 1.0, 0.5),
						Color3d(1.0, 0.5, 0.5) };

std::vector<Triangle> triangles; // holds all triangles in model
Vector3 modelCenter; //computed after model file is loaded
float modelLowestY; // computed when finding center for ground plane
bool smoothShading = false;

bool rayIntersectsPlane(const Ray& ray, const Triangle& triangle, Vector3 *intersect){
	Vector3 edgeU = triangle.p2 - triangle.p1;
	Vector3 edgeV = triangle.p3 - triangle.p1;
	Vector3 triangleNormal = edgeU.cross(edgeV);
	// Taking the parameterized equation of the ray and solving for the parameter r where ray intersects plane
	float checkRayInPlane = triangleNormal.dot(ray.direction);
	if(fabs(checkRayInPlane) <= 0.0000001)
		return false;
	float r = -triangleNormal.dot(ray.origin - triangle.p1)/checkRayInPlane;
	if(r >= 0.0){
		*intersect = ray.origin + ray.direction*r;
		return true;
	}
	return false;
}

bool rayHitsTriangle(const Ray& ray, const Triangle& triangle, Vector3* intersectionPoint)
{
	if(!rayIntersectsPlane(ray, triangle, intersectionPoint)){
		return false;
	}
	//Using the Barycentric Coordinate Computation equation and solving for s and t
	//if parameters s and t are both >= 0 and s + t <= 1 then the point hits the triangle
	Vector3 edgeU = triangle.p2 - triangle.p1;
	Vector3 edgeV = triangle.p3 - triangle.p1;
	Vector3 vector1ToIntersect = *intersectionPoint - triangle.p1;
	float uu = edgeU.dot(edgeU);
	float uv = edgeU.dot(edgeV);
	float vv = edgeV.dot(edgeV);
	float wu = vector1ToIntersect.dot(edgeU);
	float wv = vector1ToIntersect.dot(edgeV);
	float stDenominator = (uv * uv) - (uu * vv);
	float s = ((uv * wv) - (vv * wu))/stDenominator;
	float t = ((uv * wu) - (uu * wv))/stDenominator;
	if( (s >= 0.0) && (t >= 0.0) && (s + t <= 1.0)){
		return true;
	}
    return false;
}

void traceRay(int col, int row){
	float totalIllumination = ambientIntensity*ambientReflection;
	bool castShadow = false;
	int lowestI = -1;
	Vector3 closestIntersectionPoint(0.0, 0.0, 100000.0);
	float originX = (float)col + ((float)WINDOW_WIDTH/2.0 - modelCenter.x);
	float originY = (float)row + ((float)WINDOW_HEIGHT/2.0 - modelCenter.y);
	Vector3 rayOrigin(originX, originY, viewPlaneInitialZ);
	Vector3 rayDir( (col-400.0)/(viewPlaneInitialZ-perspectiveFocusZOffset),
						(row-400.0)/(viewPlaneInitialZ-perspectiveFocusZOffset),
						1.0);
	Ray currentRay(rayOrigin, rayDir);
	for(size_t i = 0; i < triangles.size(); i++){
		Vector3 intersectionPoint(0.0, 0.0, 0.0);
		if(rayHitsTriangle(currentRay, triangles[i], &intersectionPoint)){
			if(intersectionPoint.z < closestIntersectionPoint.z){
				closestIntersectionPoint = intersectionPoint;
				lowestI = i;
			}
		}
	}
	if( lowestI >= 0){
		Color3d shadedColor(1.0, 1.0, 1.0);
		if(lowestI == (int)triangles.size() - 1){
			shadedColor = Color3d(0.0, 0.0, 1.0);
		}
		Vector3 shadowRayDirection(lightSource - closestIntersectionPoint);
		shadowRayDirection.normalize();
		Ray shadowRay(closestIntersectionPoint, shadowRayDirection);
		Vector3 edge1 = triangles[lowestI].p2 - triangles[lowestI].p1;
		Vector3 edge2 = triangles[lowestI].p3 - triangles[lowestI].p1;
		Vector3 surfaceNormal = (edge1.cross(edge2)).normalized();
		Vector3 reflection = (shadowRayDirection - surfaceNormal*(2*shadowRayDirection.dot(surfaceNormal)/(surfaceNormal.magnitude()*surfaceNormal.magnitude()))).normalized();
		float cosTheta = fabs(shadowRayDirection.dot(surfaceNormal));
		float cosPhi = fabs(currentRay.direction.dot(reflection));
		for(int i = 0; i < (int)triangles.size(); i++){
			if(i != lowestI){
				Vector3 shadowIntersection(0.0, 0.0, 0.0);
				if(rayHitsTriangle(shadowRay, triangles[i], &shadowIntersection)){
					float shadowDistance = fabs((shadowIntersection - closestIntersectionPoint).magnitude());
					float outside1 = (shadowIntersection - closestIntersectionPoint).dot(lightSource - closestIntersectionPoint);
					float outside2 = (shadowIntersection - lightSource).dot(closestIntersectionPoint - lightSource);
					if( outside1 >= 0.0 && outside2 >= 0.0 && shadowDistance > 1.0){
						castShadow = true;
					}
				}
			}
		}
		if(!castShadow){
			totalIllumination += diffuseReflection*diffuseIntensity*cosTheta + specularReflection*lightIntensity*powf(cosPhi, shininess);
		}
		shadedColor.r *= totalIllumination;
		shadedColor.g *= totalIllumination;
		shadedColor.b *= totalIllumination;
		renderPixel(col, row, shadedColor);
	}
}

void raytraceRender(){
	triangles.push_back(groundTriangle); // ground plane only inserted during render so transformations not applied
	for(size_t column = 0; column <= WINDOW_WIDTH; column++){
		for(size_t row = 0; row <= WINDOW_HEIGHT; row++){
			traceRay(column, row);
		}
	}
	triangles.pop_back(); // remove ground plane
}

void computeModelCenter(){
	int totalX = 0, totalY = 0, totalZ = 0;
	for(size_t i = 0; i < triangles.size(); i++){
		totalX += (int)triangles[i].p1.x;
		totalX += (int)triangles[i].p2.x;
		totalX += (int)triangles[i].p3.x;
		totalY += (int)triangles[i].p1.y;
		totalY += (int)triangles[i].p2.y;
		totalY += (int)triangles[i].p3.y;
		totalZ += (int)triangles[i].p1.z;
		totalZ += (int)triangles[i].p2.z;
		totalZ += (int)triangles[i].p3.z;
	}
	modelCenter.x = ((float)totalX)/((float)triangles.size()*3.0);
	modelCenter.y = ((float)totalY)/((float)triangles.size()*3.0);
	modelCenter.z = ((float)totalZ)/((float)triangles.size()*3.0);
}

void GL_render()
{
    glClear(GL_COLOR_BUFFER_BIT);
    raytraceRender();
    glutSwapBuffers();
}

void rotateX(bool positive){
	float multiplier = positive ? 1.0 : -1.0;
	float rotation = ROTATION_STEP * multiplier;
	double yCoord, zCoord;
	for(size_t i = 0; i < triangles.size(); i++){
		//first translate to origin
		triangles[i].p1.x -= modelCenter.x;
		triangles[i].p2.x -= modelCenter.x;
		triangles[i].p3.x -= modelCenter.x;
		triangles[i].p1.y -= modelCenter.y;
		triangles[i].p2.y -= modelCenter.y;
		triangles[i].p3.y -= modelCenter.y;
		triangles[i].p1.z -= modelCenter.z;
		triangles[i].p2.z -= modelCenter.z;
		triangles[i].p3.z -= modelCenter.z;
		//apply rotation
		yCoord = triangles[i].p1.y;
		zCoord = triangles[i].p1.z;
		triangles[i].p1.y = cos(rotation)*yCoord - sin(rotation)*zCoord;
		triangles[i].p1.z = sin(rotation)*yCoord + cos(rotation)*zCoord;
		yCoord = triangles[i].p2.y;
		zCoord = triangles[i].p2.z;
		triangles[i].p2.y = cos(rotation)*yCoord - sin(rotation)*zCoord;
		triangles[i].p2.z = sin(rotation)*yCoord + cos(rotation)*zCoord;
		yCoord = triangles[i].p3.y;
		zCoord = triangles[i].p3.z;
		triangles[i].p3.y = cos(rotation)*yCoord - sin(rotation)*zCoord;
		triangles[i].p3.z = sin(rotation)*yCoord + cos(rotation)*zCoord;
		//translate back to center
		triangles[i].p1.x += modelCenter.x;
		triangles[i].p2.x += modelCenter.x;
		triangles[i].p3.x += modelCenter.x;
		triangles[i].p1.y += modelCenter.y;
		triangles[i].p2.y += modelCenter.y;
		triangles[i].p3.y += modelCenter.y;
		triangles[i].p1.z += modelCenter.z;
		triangles[i].p2.z += modelCenter.z;
		triangles[i].p3.z += modelCenter.z;
	}
}

void rotateY(bool positive){
	float multiplier = positive ? 1.0 : -1.0;
	float rotation = ROTATION_STEP * multiplier;
	double xCoord, zCoord;
	for(size_t i = 0; i < triangles.size(); i++){
		//first translate to origin
		triangles[i].p1.x -= modelCenter.x;
		triangles[i].p2.x -= modelCenter.x;
		triangles[i].p3.x -= modelCenter.x;
		triangles[i].p1.y -= modelCenter.y;
		triangles[i].p2.y -= modelCenter.y;
		triangles[i].p3.y -= modelCenter.y;
		triangles[i].p1.z -= modelCenter.z;
		triangles[i].p2.z -= modelCenter.z;
		triangles[i].p3.z -= modelCenter.z;
		//apply rotation
		xCoord = triangles[i].p1.x;
		zCoord = triangles[i].p1.z;
		triangles[i].p1.x = cos(rotation)*xCoord + sin(rotation)*zCoord;
		triangles[i].p1.z = -sin(rotation)*xCoord + cos(rotation)*zCoord;
		xCoord = triangles[i].p2.x;
		zCoord = triangles[i].p2.z;
		triangles[i].p2.x = cos(rotation)*xCoord + sin(rotation)*zCoord;
		triangles[i].p2.z = -sin(rotation)*xCoord + cos(rotation)*zCoord;
		xCoord = triangles[i].p3.x;
		zCoord = triangles[i].p3.z;
		triangles[i].p3.x = cos(rotation)*xCoord + sin(rotation)*zCoord;
		triangles[i].p3.z = -sin(rotation)*xCoord + cos(rotation)*zCoord;
		//translate back to center
		triangles[i].p1.x += modelCenter.x;
		triangles[i].p2.x += modelCenter.x;
		triangles[i].p3.x += modelCenter.x;
		triangles[i].p1.y += modelCenter.y;
		triangles[i].p2.y += modelCenter.y;
		triangles[i].p3.y += modelCenter.y;
		triangles[i].p1.z += modelCenter.z;
		triangles[i].p2.z += modelCenter.z;
		triangles[i].p3.z += modelCenter.z;
	}
}

void rotateZ(bool positive){
	float multiplier = positive ? 1.0 : -1.0;
	float rotation = ROTATION_STEP * multiplier;
	double xCoord, yCoord;
	for(size_t i = 0; i < triangles.size(); i++){
		//first translate to origin
		triangles[i].p1.x -= modelCenter.x;
		triangles[i].p2.x -= modelCenter.x;
		triangles[i].p3.x -= modelCenter.x;
		triangles[i].p1.y -= modelCenter.y;
		triangles[i].p2.y -= modelCenter.y;
		triangles[i].p3.y -= modelCenter.y;
		triangles[i].p1.z -= modelCenter.z;
		triangles[i].p2.z -= modelCenter.z;
		triangles[i].p3.z -= modelCenter.z;
		//apply rotation
		xCoord = triangles[i].p1.x;
		yCoord = triangles[i].p1.y;
		triangles[i].p1.x = cos(rotation)*xCoord - sin(rotation)*yCoord;
		triangles[i].p1.y = sin(rotation)*xCoord + cos(rotation)*yCoord;
		xCoord = triangles[i].p2.x;
		yCoord = triangles[i].p2.y;
		triangles[i].p2.x = cos(rotation)*xCoord - sin(rotation)*yCoord;
		triangles[i].p2.y = sin(rotation)*xCoord + cos(rotation)*yCoord;
		xCoord = triangles[i].p3.x;
		yCoord = triangles[i].p3.y;
		triangles[i].p3.x = cos(rotation)*xCoord - sin(rotation)*yCoord;
		triangles[i].p3.y = sin(rotation)*xCoord + cos(rotation)*yCoord;
		//translate back to center
		triangles[i].p1.x += modelCenter.x;
		triangles[i].p2.x += modelCenter.x;
		triangles[i].p3.x += modelCenter.x;
		triangles[i].p1.y += modelCenter.y;
		triangles[i].p2.y += modelCenter.y;
		triangles[i].p3.y += modelCenter.y;
		triangles[i].p1.z += modelCenter.z;
		triangles[i].p2.z += modelCenter.z;
		triangles[i].p3.z += modelCenter.z;
	}
}
// pan is a simple translation
void pan(float x, float y, float z){
	for(size_t i = 0; i < triangles.size(); i++){
		triangles[i].p1.x += x;
		triangles[i].p1.y += y;
		triangles[i].p1.z += z;
		triangles[i].p2.x += x;
		triangles[i].p2.y += y;
		triangles[i].p2.z += z;
		triangles[i].p3.x += x;
		triangles[i].p3.y += y;
		triangles[i].p3.z += z;
	}
	computeModelCenter();
}
// zoom just adjust the view plane along the z axis
void zoom(bool positive){
	viewPlaneInitialZ += positive ? ZOOM_STEP : -ZOOM_STEP;
	computeModelCenter();
}
void keyboard(unsigned char key, int x, int y){
	switch(key){
		case 'x':
			rotateX(true);
			break;
		case 'X':
			rotateX(false);
			break;
		case 'y':
			rotateY(true);
			break;
		case 'Y':
			rotateY(false);
			break;
		case 'z':
			rotateZ(true);
			break;
		case 'Z':
			rotateZ(false);
			break;
		case 'w':
			smoothShading = !smoothShading;
			break;
		case '+':
			zoom(true);
			break;
		case '-':
			zoom(false);
			break;
		default:
			break;
	}
	glutPostRedisplay();
}

// This is the callback for arrow key events
void specialInput(int key, int x, int y){
	switch(key){
		case GLUT_KEY_UP:
			pan(0, -PAN_STEP, 0);
			break;
		case GLUT_KEY_DOWN:
			pan(0, PAN_STEP, 0);
			break;
		case GLUT_KEY_LEFT:
			pan(PAN_STEP, 0, 0);
			break;
		case GLUT_KEY_RIGHT:
			pan(-PAN_STEP, 0, 0);
			break;
		default:
			break;
	}
	glutPostRedisplay();
}

void mouse(int button, int state, int x, int y){}

void GLInit(int* argc, char** argv)
{
	glutInit(argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE);
	glutInitWindowSize(WINDOW_WIDTH, WINDOW_HEIGHT);
	glutCreateWindow("CS 130 - George Braxton: Project 2");
	glutDisplayFunc(GL_render);
	glutMouseFunc(mouse);
	glutKeyboardFunc(keyboard);
	glutSpecialFunc(specialInput);
	glMatrixMode(GL_PROJECTION_MATRIX);
	glOrtho(0, WINDOW_WIDTH, 0, WINDOW_HEIGHT, -1, 1);
}

void loadFile(char* filename){
	std::ifstream file;
	file.open(filename);
	int numPoints;
	int numTriangles;
	file >> numPoints;
	file >> numTriangles;
	Vector3* points = new Vector3[numPoints];
	modelLowestY = 100000.0;
	for(int i = 0; i < numPoints; i++){
		int x, y, z;
		file >> x;
		file >> y;
		file >> z;
		points[i] = Vector3((float)x, (float)y, (float)z);
		if((float)y < modelLowestY){//for ground plane
			modelLowestY = (float)y;
		}
	}
	for(int i=0; i < numTriangles; i++){
		int p1, p2, p3;
		file >> p1;
		file >> p2;
		file >> p3;
		Triangle newTriangle(points[p1], points[p2], points[p3]);
		triangles.push_back(newTriangle);
	}
	file.close();
	computeModelCenter();
	Vector3 ground1(modelCenter.x, modelLowestY - 200.0, modelCenter.z - 10000.0);
	Vector3 ground2(modelCenter.x + 10000.0, modelLowestY - 100.0, modelCenter.z + 10000.0);
	Vector3 ground3(modelCenter.x - 10000.0, modelLowestY - 100.0, modelCenter.z + 10000.0);
	groundTriangle = Triangle(ground1, ground2, ground3);
}

int main(int argc, char** argv)
{
	if(argc != 2){
		printf("invalid # of arguments. usage assn2 modelFile\n");
		exit(0);
	}	
	loadFile(argv[1]);
	GLInit(&argc, argv);
	GL_render();
	glutMainLoop();

	return 0;
}

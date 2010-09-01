#ifndef VECTOR3_H
#define VECTOR3_H

#include <iostream>
#include <cv.h>

using namespace std;

struct Point3
{
	float x;
	float y;	
	float z;
        operator cv::Point3f() const
        {
            return cv::Point3f(x,y,z);
        }
};

struct Point2
{
        float x;
        float y;
        operator cv::Point2f() const
        {
            return cv::Point2f(x,y);
        }
};


struct Color3
{
	float r;
	float g;
	float b;
};

class Vector3
{
 public:
	float x;
	float y;
	float z;
	
	Vector3();	
	Vector3(float x, float y, float z);
	//vector from p1 to p2 (p2 - p1)
	Vector3(Point3 p1, Point3 p2);
	
	Vector3 add(Vector3 v1);
	Vector3 sub(Vector3 v1);
	Vector3 mult(float scalar);
	float length();
        //normalize the vector input as a double*
        //overwirtes the argument and also returns a pointer to it
        static double* normalize(double*,int);

	//void versions
	void addTo(Vector3 v1);
	void divideBy(float s);
		
	//cross product between this x v1
	Vector3 cross_product(Vector3 v1);
	Vector3 normalize();        
};

ostream& operator<<(ostream& stream, Vector3 v);

#endif /*VECTOR3_H*/

#include <cmath>
#include "Vector3.h"

Vector3::Vector3()
{
  x = 0;
  y = 0;
  z = 0;
}

Vector3::Vector3(Point3 p1, Point3 p2)
{
  x = p2.x - p1.x;
  y = p2.y - p1.y;
  z = p2.z - p1.z;
}

Vector3::Vector3(float x1, float y1, float z1)
{
  x = x1;
  y = y1;
  z = z1;
}

Vector3 Vector3::add(Vector3 v1)
{
  Vector3 result;
  result.x = x + v1.x;
  result.y = y + v1.y;
  result.z = z + v1.z;
  return result;  
}

void Vector3::addTo(Vector3 v1)
{
  x = x + v1.x;
  y = y + v1.y;
  z = z + v1.z;
}

Vector3 Vector3::sub(Vector3 v1)
{
  Vector3 result;
  result.x = x - v1.x;
  result.y = y - v1.y;
  result.z = z - v1.z;
  return result;  
}

Vector3 Vector3::mult(float s)
{
  Vector3 result;
  result.x = x * s;
  result.y = y * s;  
  result.z = z * s;
  
  return result;  
}

void Vector3::divideBy(float s)
{
  x = x / s;
  y = y / s;  
  z = z / s;
}

//this x v1
Vector3 Vector3::cross_product(Vector3 v1)
{
  Vector3 result;
  result.x = y*v1.z - v1.y*z;
  result.y = -(x*v1.z - v1.x*z);
  result.z = x*v1.y - v1.x*y;
  return result;
}

float Vector3::length()
{
  return sqrt(x*x + y*y + z*z);
}

Vector3 Vector3::normalize()
{
  Vector3 result;
  float l = length();  
  result.x = x/l;
  result.y = y/l;
  result.z = z/l;
  return result;
}

ostream& operator<<(ostream& stream, Vector3 v)
{
  stream << "vector = ("<< v.x << ", " << v.y << ", " << v.z << ")";
}


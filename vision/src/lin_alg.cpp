// Copyright 2009 Erik Weitnauer
#include "lin_alg.h"

using namespace std;
using namespace icl;
 
vector<Point32f> LinAlg::transformed(const vector<Point32f> &v, const Mat3 &T) {
	vector<Point32f> result;
	Vec3 vec;
	for (unsigned int i=0; i<v.size(); i++) {
		//cout << "Point before transformation: " << v[i] << endl;
		vec = Vec3(v[i].x,v[i].y,1);
		vec = T*vec;
		result.push_back(Point32f(vec[0], vec[1]));
		//cout << "Point after transformation: " << Point32f(vec[0], vec[1]) << endl;
	}
	return result;
}

vector<Point32f> LinAlg::transformedAround(
		const icl::Point32f center,
		const std::vector<icl::Point32f> &v, const Mat3 &T) {
	vector<Point32f> result;
	Vec3 vec;
	for (unsigned int i=0; i<v.size(); i++) {
		//cout << "Point before transformation: " << v[i] << endl;
		vec = Vec3(v[i].x-center.x,v[i].y-center.y,1);
		vec = T*vec;
		result.push_back(Point32f(vec[0]+center.x, vec[1]+center.y));
		//cout << "Point after transformation: " << Point32f(vec[0], vec[1]) << endl;
	}
	return result;			
}

void LinAlg::transformAround(
		const icl::Point32f center,
		std::vector<icl::Point32f> &v, const Mat3 &T) {
	Vec3 vec;
	for (unsigned int i=0; i<v.size(); i++) {
		//cout << "Point before transformation: " << v[i] << endl;
		vec = Vec3(v[i].x-center.x,v[i].y-center.y,1);
		vec = T*vec;
		v[i].x = vec[0]+center.x;
		v[i].y = vec[1]+center.y;
		//cout << "Point after transformation: " << Point32f(vec[0], vec[1]) << endl;
	}
}


void LinAlg::transform(Point32f &p, const Mat3 &T) {
	Vec3 vec(p.x, p.y, 1);
	vec = T*vec;
	p.x = vec[0];
	p.y = vec[1];
}

Point32f LinAlg::transformed(const Point32f &p, const Mat3 &T) {
	Point32f x(p);
	transform(x, T);
	return x;
}

void LinAlg::transform(Rect32f &r, const Mat3 &T) {
	Point32f ul = transformed(r.ul(), T);
	Point32f lr = transformed(r.lr(), T);
	// call .normalized in case we got a negative width / height
	r = Rect32f(ul.x, ul.y, lr.x-ul.x, lr.y-ul.y).normalized();
}

void LinAlg::transformAround(const Point32f &center, Rect32f &r, const Mat3 &T) {
	Point32f ul = r.ul() - center; transform(ul, T);
	Point32f lr = r.lr() - center; transform(lr, T);
	// call .normalized in case we got a negative width / height
	r = Rect32f(ul.x+center.x, ul.y+center.y, lr.x-ul.x, lr.y-ul.y).normalized();
}

float LinAlg::closestDistanceSqr(const Point32f &p, const vector<Point32f> &v) {
	float dist2 = -1, curr;
	for (unsigned int i=0; i<v.size(); i++) {
		curr = distance2(p,v[i]);
		if ((dist2 == -1) || (curr < dist2)) dist2 = curr;
	}
	//cout << "Closest distance is " << dist2 << endl;
	return dist2;
}

float LinAlg::diagonalLength(const icl::Rect32f &r) {
	return sqrt(r.width*r.width + r.height*r.height);
}

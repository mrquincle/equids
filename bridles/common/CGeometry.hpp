/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief Simple geometric functions that do not need matrices
 * @file CGeometry.hpp
 * 
 * This file is created at Almende B.V. and Distributed Organisms B.V. It is open-source software and belongs to a
 * larger suite of software that is meant for research on self-organization principles and multi-agent systems where
 * learning algorithms are an important aspect.
 *
 * This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless, we personally strongly object
 * against this software being used for military purposes, factory farming, animal experimentation, and "Universal
 * Declaration of Human Rights" violations.
 *
 * Copyright (c) 2013 Anne C. van Rossum <anne@almende.org>
 *
 * @author    Anne C. van Rossum
 * @date      Aug 28, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#ifndef CGEOMETRY_HPP_
#define CGEOMETRY_HPP_

/**
 * This file contains a very small set of geometric functions, such as line intersections etc. It stays away from
 * anything sophisticated that can better be solved by dedicated libraries.
 */

// static const double M_HALF_PI = acos(0); // this is already defined as M_PI_2 in math.h

//namespace dobots {

struct Point {};

//! Local class that knows that commas can be treated as white spaces. It is by the input stream operator.
struct commasep: std::ctype<char> {
	commasep(): std::ctype<char>(get_table()) {}
	static std::ctype_base::mask const* get_table() {
		static std::vector<std::ctype_base::mask>
		rc(std::ctype<char>::table_size,std::ctype_base::mask());
		rc[','] = std::ctype_base::space;
		return &rc[0];
	}
};

/**
 * A 2-dimensional point is defined with an x and a y coordinate.
 */
template <typename R>
struct Point2D: Point {
	R x;
	R y;
	Point2D(): x(0), y(0) {}
	Point2D(R x, R y) { this->x = x; this->y = y; }
	void set(R x, R y) { this->x = x; this->y = y; }
	bool operator==(const Point2D &other) {
		return ((other.x == x) && (other.y == y));
	}
	bool operator!=(const Point2D &other) {
		return ((other.x != x) || (other.y != y));
	}
	//! Return Euclidean distance between the two points
	R operator-(const Point2D &other) {
		return std::sqrt((x-other.x)*(x-other.x) + (y-other.y)*(y-other.y));
	}
	friend std::ostream& operator<<(std::ostream& os, const Point2D & p) {
		os << p.x << ',' << p.y;
		return os;
	}
	friend std::istream& operator>>( std::istream& is, Point2D& p) {
		is.imbue(std::locale(std::locale(), new commasep));
		is >> p.x >> p.y;
		return is;
	}
};

/**
 * Returns polar coordinates of a point. If the point is at the origin (r=0) this function will return false, because
 * theta cannot be defined. The angle returned is from -pi to +pi, use the make_positive argument to add "pi" to it,
 * to make it run from 0 to 2pi.
 *
 * @param x                  Euclidean coordinate
 * @param y                  Euclidean coordinate
 * @param r                  polar coordinate
 * @param theta              polar coordinate
 * @param make_positive      return from [0,+2pi] or [-pi,+pi]
 */
template <typename R, typename T>
bool getPolarCoordinates(const R x, const R y, T & r, T & theta, bool make_positive = false) {
	r = std::sqrt(x*x+y*y);
	if (!r) return false;
	theta = atan2(y,x);
	if (make_positive) theta += M_PI;
	return true;
}

/**
 * Return Euclidean coordinates given polar coordinates.
 */
template <typename R, typename T>
void getEuclideanCoordinates(const T r, const T theta, R & x, R & y) {
	x = round(cos(theta)*r);
	y = round(sin(theta)*r);
}

/**
 * Returns a slope and intersection of the y-axis given two points. A line can be represented by ax+by+c=0, although
 * the representation y=mx+k is probably more familiar. Or: mx-y+k=x-(1/m)y+k/m=x+(b/a)y+c/a=0. Thus -m=a/b, the
 * slope corresponds to m=-a/b. The y-axis intersection can subsequently be interpolated by following the slope from
 * the y-value of the first point until you reach x=0, nothing complicated. The slope is undefined when the points
 * lie on a vertical line, the function returns false in that case and the slope and intersection values remain not
 * set.
 */
template <typename R, typename T>
bool getLineDescription(const Point2D<R> point0, const Point2D<R> point1, T & slope, T & y_intersect) {
	//reorder, such that the point with highest x comes later, this will make the slope well-defined
	Point2D<R> pnt0, pnt1;
	if (point0.x < point1.x) {
		pnt0 = point0; pnt1 = point1;
	} else {
		pnt0 = point1; pnt1 = point0;
	}
	if (!(pnt1.x - pnt0.x)) return false;

	slope = (T)(pnt1.y - pnt0.y) / (T)(pnt1.x - pnt0.x);
	//assert (slope);
	//T alpha = atan(slope);
	y_intersect = pnt0.y - (T)(pnt0.x) * slope;
	return true;
}

/**
 * Returns the closest point of a line with respect to the origin. This was first calculated in polar coordinates, but
 * that is actually an inconvenient representation for it. A line that is drawn through point0 and point1 can be
 * represented by ax+by+c=0. As before the slope corresponds to -a/b. The closest point to the origin is perpendicular
 * to the line and hence has as slope b/a. So, this point is the intersection of the lines:
 *   y = b/a x
 *   ax + by + c = 0
 * Solving for x: ax + b*b/a x + c = 0, thus: x = -ac / (a*a + b*b), and with b=-1: x = -ac (a*a+1)
 * Solving for y: y = b/a x, with b=-1, y = -1/a x
 *
 * Check for yourself if you need some pictures at:
 *   http://www.intmath.com/plane-analytic-geometry/perpendicular-distance-point-line.php
 *
 * @param slope              slope of the line
 * @param y_intersect        intersection of the line with the x-axis
 * @param x                  return coordinate on x-axis of closest point
 * @param y                  return coordinate on y-axis of closest point
 */
template <typename T>
void getClosestPoint(const T slope, const T y_intersect, T & x, T & y) {
	if (slope == 0) {
		x = 0;
		y = y_intersect;
	} else {
		x = -(slope*y_intersect)/(slope*slope+1.0);
		y = (-1.0/slope)*x;
	}
}

template <typename R, typename T>
void getClosestPoint(const Point2D<R> point0, const Point2D<R> point1, Point2D<T> & closest) {
	T slope,yisect;
	getLineDescription(point0,point1,slope,yisect);
	getClosestPoint(slope,yisect,closest.x,closest.y);
}

/**
 * Return intersection between lines indicated by points. It's not considering the line segment, but the entire line.
 * Just see https://en.wikipedia.org/wiki/Line-line_intersection.
 *
 * @param p0                 point on line
 * @param p1                 point on line
 * @param q0                 point on second line
 * @param q1                 point on second line
 * @param i                  resulting intersection
 */
template <typename R>
bool getIntersection(const Point2D<R> p0, const Point2D<R> p1, const Point2D<R> q0, const Point2D<R> q1,
		Point2D<R> & intersect) {
	R div = (p0.x - p1.x)*(q0.y - q1.y) - (p0.y - p1.y)*(q0.x - q1.x);
	static const double rounding_error = 0.00001;
	if (fabs(div) < rounding_error) return false; //< no intersection at all
	intersect.x = ((p0.x * p1.y - p0.y * p1.x) * (q0.x - q1.x) - (p0.x - p1.x)*(q0.x * q1.y - q0.y * q1.x )) / div;
	intersect.y = ((p0.x * p1.y - p0.y * p1.x) * (q0.y - q1.y) - (p0.y - p1.y)*(q0.x * q1.y - q0.y * q1.x )) / div;
	std::cout << "Intersect [" << p0.x << ',' << p0.y << " --- " << p1.x << ',' << p1.y << "] with " <<
			"[" << q0.x << ',' << q0.y << " --- " << q1.x << ',' << q1.y << "]" << " gives: " <<
			intersect.x << ',' << intersect.y << std::endl;
	return true;
}

template <typename R>
bool between(const Point2D<R> p0, const Point2D<R> p1, const Point2D<R> b) {
	bool cond0 = false, cond1 = false;
	if (b.x >= p0.x && b.x <= p1.x) cond0 = true;
	if (b.y >= p0.y && b.y <= p1.y) cond1 = true;
	if (b.x >= p1.x && b.x <= p0.x) cond0 = true;
	if (b.y >= p1.y && b.y <= p0.y) cond1 = true;
	int result = cond0 && cond1;
	std::cout << "Point b " << b << " is " << ((!result) ? "not " : "") << "in between " << p0 << " --- " << p1 << std::endl;
	return result;
}

/**
 * Calculate the two intersecting points between a line and a rectangle.
 */
template <typename R>
bool getIntersection(const Point2D<R> tl, const Point2D<R> br, const Point2D<R> p0, const Point2D<R> p1,
		Point2D<R> & intersect0, Point2D<R> & intersect1) {
	Point2D<R> tr; tr.x = br.x; tr.y = tl.y;
	Point2D<R> bl; bl.x = tl.x; bl.y = br.y;

	bool intersect_found = false;
	Point2D<R> i;
	bool inter;
	inter = getIntersection(tl, tr, p0, p1, i);
	if ((inter) && (between(tl, tr, i))) { //(i.y == tl.y)) {
		intersect0.x = i.x;
		intersect0.y = i.y;
		intersect_found = true;
	}

	inter = getIntersection(tl, bl, p0, p1, i);
	if ((inter) && (between(tl, bl, i))) {
		if (!intersect_found) {
			intersect0.x = i.x;
			intersect0.y = i.y;
			intersect_found = true;
		} else {
			intersect1.x = i.x;
			intersect1.y = i.y;
			return true;
		}
	}

	inter = getIntersection(br, bl, p0, p1, i);
	if ((inter) && (between(br, bl, i))) {
		if (!intersect_found) {
			intersect0.x = i.x;
			intersect0.y = i.y;
			intersect_found = true;
		} else {
			intersect1.x = i.x;
			intersect1.y = i.y;
			return true;
		}
	}

	inter = getIntersection(br, tr, p0, p1, i);
	if ((inter) && (between(br, tr, i))) {
		if (!intersect_found) {
			intersect0.x = i.x;
			intersect0.y = i.y;
			intersect_found = true;
		} else {
			intersect1.x = i.x;
			intersect1.y = i.y;
			return true;
		}
	}
	std::cerr << "This should not happen, except when the line does not cross the rectangle at all..." << std::endl;
	return false;
}

/**
 * Given a point on a line that is closest to the origin, in polar coordinates, return y-intersection and slope of this
 * line. If the line is vertical, there is no y-intersection.
 *
 */
template <typename T>
bool getYIntersect(const T r, const T theta, T & intersect, T & slope) {
	if (isVertical(theta)) {
		return false;
	}
	slope = tan(theta);
	intersect = std::sqrt(r*slope*r*slope + r*r);
	return true;
}

/**
 * If theta approaches PI/2 consider the line vertical for all practical purposes.
 */
template <typename T>
bool isVertical(const T theta) {
	static const double rounding_error = 0.00001;
	return (fabs(theta - M_PI_2) < rounding_error);
}

/**
 * Theta is the polar coordinate of the closest point. The slope is closely related, but not the same thing.
 */
template <typename T>
T getSlope(const T theta) {
	return tan(theta-M_PI_2);
}

/**
 * Given closest point in polar coordinates, hand over two points that are on the line that goes through this point and
 * doesn't come closer to the origin.
 */
template <typename T, typename R>
void getPoints(const T r, const T theta, Point2D<R> & p0, Point2D<R> & p1) {
	getEuclideanCoordinates(r, theta, p0.x, p0.y); // closest point
	int d = 100;
	if (isVertical(theta)) {
		p1.x = p0.x;
		p1.y = p1.x + d;
		return;
	}
	T slope = getSlope(theta);
	if (slope < 1) {
		p1.x = p0.x + d;
		p1.y = p0.y + d * slope;
	} else {
		p1.x = p0.x + d / slope;
		p1.y = p0.y + d;
	}
}


#endif /* CGEOMETRY_HPP_ */

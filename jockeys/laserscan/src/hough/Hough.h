/**
 * @brief Helper functions to pick random elements from std containers
 * @file Random.h
 *
 * This file is created at Almende B.V. It is open-source software and part of the Common Hybrid Agent Platform (CHAP).
 * A toolbox with a lot of open-source tools, ranging from thread pools and TCP/IP components to control architectures
 * and learning algorithms. This software is published under the GNU Lesser General Public license (LGPL).
 *
 * It is not possible to add usage restrictions to an open-source license. Nevertheless, we personally strongly object
 * to this software being used by the military, in factory farming, for animal experimentation, or anything that
 * violates the Universal Declaration of Human Rights.
 *
 * Copyright © 2013 Anne van Rossum <anne@almende.com>
 *
 * @author  Anne C. van Rossum
 * @date    May 7, 2013
 * @project Replicator FP7
 * @company Almende B.V.
 * @case    Artificial Intelligence Framework
 */

#ifndef HOUGH_H_
#define HOUGH_H_

#include <cstring>
#include <vector>
#include <iostream>
#include <cmath>
#include <cassert>

#include <nd-array.hpp>
#include <Accumulator.h>
#include <HoughDefs.h>
#include <Random.h>

#include <dim1algebra.hpp>

namespace dobots {

#define M_PIx2              6.2831853071795861097

/**
 * The Hough transform can be used to detect higher-order structures from a bunch of points, commonly called a point
 * cloud. It's first implementation used it to detect lines, we will use it to detect lines first, and planes later.
 */
template <typename P>
class Hough {
public:
	//! Use the nd-array also for the point cloud
	typedef nd_array < std::vector<P*>,short > pointcloud;

	//! Default constructor expects image points in size of 640x480 and uses a Hough space of size 100x100
	Hough(): type(RANDOMIZED_HOUGH) {
		std::cout << "Use standard constructor for Hough transform" << std::endl;
		clear();
		ASize size;
		size.x = ACCUMULATOR_SIZE_X; // ACCUMULATOR_DATA_TYPE
		size.y = ACCUMULATOR_SIZE_Y;
		input_size.x = 640;
		input_size.y = 480;
		max_distance = std::sqrt(input_size.x*input_size.x + input_size.y*input_size.y);
		accumulator = new Accumulator<P>(size);
		//		use_cells = true;
		use_cells = false;
		use_random_patch_picker = false;
	}

	//! Constructor with non-standard size for the Hough space
	Hough(ISize input_size, ASize hough_space_size): type(RANDOMIZED_HOUGH) {
		clear();
		this->input_size = input_size;
		max_distance = std::sqrt(input_size.x*input_size.x + input_size.y*input_size.y);
		accumulator = new Accumulator<P>(hough_space_size);
		//		use_cells = true;
		use_cells = false;
		use_random_patch_picker = false;
	}

	//! Default destructor
	virtual ~Hough() {
		clear();
		if (accumulator != NULL)
			delete accumulator;
		accumulator = NULL;
	}

	//! Set the transform, it's your responsible to clear the point cloud and or reset the accumulator
	inline void setType(HoughTransformType type) { this->type = type; }

	//! Add points one by one
	void addPoint(P * p) {
		points.push_back(p);
	}

	//! Add points in a bunch, points should just be two or three elements, so by value, not by reference
	void addPoints(std::vector<P*> & point_cloud) {
		//		std::cout << "Inserting " << point_cloud.size() << " points" << std::endl;
		points.insert(points.end(), point_cloud.begin(), point_cloud.end());
	}

	//! Add points to spatial point cloud
	void addPoints(pointcloud & spatial_point_cloud) {
		spatial_points = spatial_point_cloud;
		updatePoints();
	}

	void pickRandomPatch(int &x, int &y) {
		// pick first a random index from spatial_points patch grid
		int px = spatial_points.get_dimension(0);
		int py = spatial_points.get_dimension(1);
		//std::cout << "Dimensions are " << px << ',' << py << std::endl;
		int size = px*py;
		int linear_r = random_value(0,size-1);
		x = linear_r % px;
		y = linear_r / px;
	}

	void updatePoints() {
		points.clear();
		for (int i = 0; i < spatial_points.size(); ++i) {
			std::vector<P*> & pnts = spatial_points.getf(i);
			//std::vector<P*> pnts;
			//			pnts.resize(cpy.size());
			//			ref(cpy.begin(), cpy.end(), pnts.begin());
			//			std::cout << "Add " << pnts.size() << " points " << std::endl;
			addPoints(pnts);
			//			for (int j = 0; j < pnts.size(); ++j) {
			//				addPoint(&pnts[j]);
			//			}
		}
	}

	//! Absolutely horrible to return a reference to member data, but bare with me for now.
	std::vector<P*> & getPoints() {
		return points;
	}

	//! This is a better method if the density differs across the cells, it requires all points to be added to one
	//! data structure. It will pick a point at random and return the given patch indices.
	void pickRandomPoint(int &x, int &y) {
		int linear_e = random_value(0,points.size()-1);
		int wx = input_size.x / spatial_points.get_dimension(0);
		int wy = input_size.y / spatial_points.get_dimension(1);

		x = points[linear_e]->x / wx;
		y = points[linear_e]->y / wy;
		//		std::cout << "Got point " << points[linear_e]->x << ',' << points[linear_e]->y <<
		//				" which becomes patch " << x << ',' << y << std::endl;
	}

	//! Perform the actual transform on all the points hitherto received
	void doTransform() {
		std::vector<P*> pnts;
		if (use_cells) {
			int ix, iy;
			if (use_random_patch_picker) {
				pickRandomPatch(ix,iy);
			} else {
				pickRandomPoint(ix,iy);
			}
			//std::vector<P> & cpy = spatial_points.get(ix,iy);
			std::vector<P*> & sp = spatial_points.get(ix,iy);
			pnts.insert(pnts.end(), sp.begin(), sp.end()); // copy all references to points into pnts
			// create vector with references to points, instead of copying the points
			//			pnts.resize(cpy.size());
			//			ref(cpy.begin(), cpy.end(), pnts.begin());
		} else {
			pnts.insert(pnts.end(), points.begin(), points.end()); // copy all references to points into pnts
		}

		if (pnts.size() < 3) return;
		std::vector<P*> random_set; random_set.clear();
		random_set.resize(2);

		random_n(pnts.begin(), pnts.end(), random_set.begin(), 2);
		P* pnt0 = random_set[0];
		P* pnt1 = random_set[1];

		ACoordinates c;
		if (!transform(*pnt0, *pnt1, c)) return;
		Segment2D<P> segment;
		segment.src = pnt0;
		segment.dest = pnt1;
		getAccumulator()->Increment(c, segment);
	}

	template <typename P1, typename C1>
	C1 transform(P1 pnt0, P1 pnt1) {
		assert(false); // no general implementation possible
		return pnt0;
	}

	/**
	 * Transform the line through two points to a (theta,r) coordinate pair. The angle theta ranges from -pi to +pi,
	 * here it is scaled so it fits the accumulator type.
	 */
	bool transform(Point2D<int> point0, Point2D<int> point1, ACoordinates & coordinates) {
		// std::cout << "Transform " << pnt0.x << "," << pnt0.y << " and " << pnt1.x << "," << pnt1.y << std::endl;
		ASSERT_NEQ(point0, point1);

		typedef double ftype;
		ftype theta, r;

		int pixels_apart = 10;
		if (abs(point1 - point0) <= pixels_apart) {
			return false;
		}

		ftype slope,yisect,fx,fy;
		bool slope_defined = getLineDescription(point0,point1,slope,yisect);
		if (!slope_defined) {
			// vertical line
			theta = M_PI_2;
			r = point0.x;
			std::cout << "Undefined slope for " << point0 << " and " << point1 << ", just set it at " << theta <<
					" and " << r << std::endl;
		} else {
			//std::cout << "From " << point0 << " and " << point1 << " got slope " << slope << " and y-intersection " << yisect << std::endl;
			getClosestPoint(slope,yisect,fx,fy);

			bool not_at_origin = getPolarCoordinates<ftype>(fx,fy,r,theta);
			if (!not_at_origin) {
				// radial line (through origin), pick random point and calculate atan(y/x)
				theta = atan2(point1.y, point1.x) + M_PI_2;
				r = 0;
				std::cout << "Precisely through origin, set it at " << theta << " and " << r << std::endl;
			}

			// debugging / checks
//			ftype dist0 = sqrt(pnt0.x*pnt0.x+pnt0.y*pnt0.y);
//			ftype dist1 = sqrt(pnt1.x*pnt1.x+pnt1.y*pnt1.y);
//			// std::cout << "Point 0 is at dist=" << (int)dist0 << " and point 1 at dist=" << (int)dist1 << std::endl;
//			assert (r <= dist0);
//			assert (r <= dist1);
		}

		transform(r, theta, coordinates);
		return true;
	}

	/**
	 * Transform the polar coordinates of the closest point to a cell in Hough coordinates.
	 */
	void transform(const double r, const double theta, ACoordinates & coordinates) {
		typedef double ftype;

		coordinates.x = scale(r, max_distance, accumulator->getSize().x-1, true);
//		if (coordinates.x == accumulator->getSize().x) result.x = accumulator->getSize().x-1;

		// divide theta by maximum range, and because of negative values shift
		const ftype max_angle = (ftype)M_PIx2;
		ftype positive_theta = theta + M_PI;
		coordinates.y = scale(positive_theta, max_angle, accumulator->getSize().y-1, true);
//		if (coordinates.y == accumulator->getSize().y) result.y = accumulator->getSize().y-1;

//		std::cout << "Scaled distance " << result.x << " comes from " << r << std::endl;
//		std::cout << "Scaled angle " << result.y << " comes from " << theta << " * " <<
//				accumulator->getSize().y << " / " << (max_angle) << " + " << accumulator->getSize().y / 2 << std::endl;

		// for debugging
		ASSERT_LT(coordinates.x, accumulator->getSize().x);
		ASSERT_LT(coordinates.y, accumulator->getSize().y);
	}

	//! Remove all points from the point cloud
	void clear(bool deallocate = true) {
		if (deallocate)
			points.clear();
		else
			points.erase(points.begin(), points.end());
	}

	void transform(const ACoordinates coord, double & r, double & theta) {
		typedef double ftype;

		// r is scaled with max_dist / max, so scale back:
//		r = (coord.x * max_distance) / (ftype)accumulator->getSize().x;
		r = scale(coord.x, accumulator->getSize().x-1, max_distance);

		const ftype max_angle = (ftype)M_PIx2;
//		theta = ((coord.y-(ftype)accumulator->getSize().y/2) * max_angle) / (ftype)(accumulator->getSize().y-1);

		theta = scale(coord.y, accumulator->getSize().y-1, max_angle);
//		theta = scale<ftype,ftype>(coord.y-accumulator->getSize().y/2, accumulator->getSize().y-1, max_angle);
		theta -= (ftype)M_PI;
	}

	/**
	 * Get the cell with the maximum number of hits and return polar coordinates.
	 *
	 * @param r              distance to origin
	 * @param theta          angle
	 */
	void getLine(double & r, double & theta) {
		const ACoordinates coord = getMax();
		transform(coord, r, theta);
	}

	ACoordinates getMax() {
		ACoordinates coord = accumulator->getMaxCoord();
		std::cout << "Found cell with max. number of hits at " << coord.x << "," << coord.y << std::endl;
		return coord;
	}

	//! Get the accumulator, e.g. to reset it
	inline Accumulator<P>* getAccumulator() { return accumulator; }
private:
	//! The type of Hough transform to use
	HoughTransformType type;

	//! Accumulator
	Accumulator<P> *accumulator;

	//! Point cloud, for now store temporary all points, and only perform transform when doTransform is called
	std::vector<P*> points;

	//! Point cloud, but in a spatial structure
	pointcloud spatial_points;

	//! The input points should fall in this range
	ISize input_size;

	//! Max distance (calculated from input_size)
	double max_distance;

	//! Use grid cells
	bool use_cells;

	//! Use random patch picker (or random point picker)
	bool use_random_patch_picker;
};

}

#endif // HOUGH_H_

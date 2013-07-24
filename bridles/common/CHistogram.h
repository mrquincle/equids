/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CHistogram.h
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
 * @date      Jul 23, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#ifndef CHISTOGRAM_H_
#define CHISTOGRAM_H_

#include <vector>
#include <functional>
#include <numeric>

#include <dim1algebra.hpp>

template <typename T, typename R>
class CHistogram {
public:
	CHistogram() {
		window_size = -1;
	}

	~CHistogram() {}

	void set_sliding_window(int window_size) {
		if (data.size() > window_size) {
			size_t remainder = data.size() - window_size;
			data.erase(data.begin(), data.end() - remainder);
		}
		this->window_size = window_size;

	}

	void push(T item) {
		if (window_size == data.size()) {
			dobots::pushpop(data.begin(), data.end(), item);
		} else {
			data.push_back(item);
		}
	}

	R average() {
		if (empty()) return (R)0;
		return (sum() / (R)data.size());
	}

	T sum() {
		T sum = 0;
		std::accumulate(data.begin(), data.end(), sum);
		return sum;
	}

	bool empty() {
		return data.size() == 0;
	}

	void clear() {
		data.erase(data.begin(), data.end());
	}
private:
	std::vector<T> data;

	int window_size;
};

template <typename T, typename R>
class CVirtualHistogram {
public:
	CVirtualHistogram() {
		clear();
	}

	~CVirtualHistogram() {}

	void push(T item) {
		sum_data += item;
		count++;
	}

	inline R average() {
		if (empty) return (R)0;
		return sum() / (R)count;
	}

	inline T sum() {
		return sum_data;
	}

	inline bool empty() {
		return count == 0;
	}

	void clear() {
		count = 0;
		sum_data = 0;
	}
private:
	T sum_data;

	long int count;
};


#endif /* CHISTOGRAM_H_ */

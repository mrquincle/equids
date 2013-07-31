/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief ...
 * @file CMultiHistogram.h
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
 * @date      Jul 24, 2013
 * @project   Replicator 
 * @company   Almende B.V.
 * @company   Distributed Organisms B.V.
 * @case      Sensor fusion
 */

#ifndef CMULTIHISTOGRAM_H_
#define CMULTIHISTOGRAM_H_

#include <CHistogram.h>

template <typename T, typename R>
class CMultiHistogram {
public:
	CMultiHistogram() {
		window_size = -1;
	}

	~CMultiHistogram() {
		histograms.erase(histograms.begin(), histograms.end());
	}

	//! Add histogram
	void add(int n = 1) {
		for (int i = 0; i < n; ++i)
			histograms.push_back(new CHistogram<T,R>());
	}

	//! Set size of sliding window (-1 is no sliding window at all, risking overflow)
	void set_sliding_window(int window_size) {
		this->window_size = window_size;
		for (int i = 0; i < histograms.size(); ++i) {
			histograms[i]->set_sliding_window(window_size);
		}
	}

	//! Gets size of sliding window
	inline int get_sliding_window() { return window_size; }

	void push(int i, T item) {
		histograms[i]->push(item);
	}

	R average(int i) {
		return histograms[i]->average();
	}

	//! If you provide a STL container, previously allocated to proper size with resize(), averages can be written to it
	template<typename OutputIterator>
	OutputIterator average(OutputIterator result) {
		for (int i = 0; i < histograms.size(); ++i, ++result) {
			*result = histograms[i]->average();
		}
		return result;
	}


	T sum(int i) {
		return histograms[i].sum();
	}

private:
	std::vector<CHistogram<T,R>* > histograms;

	int window_size;
};

#endif /* CMULTIHISTOGRAM_H_ */

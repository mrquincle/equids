/**
 * 456789------------------------------------------------------------------------------------------------------------120
 *
 * @brief A collection of histogram objects
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

/**
 * A histogram is a nice object to use to smooth for example the subsequent sensor inputs of a given sensor. However, if
 * there are multiple sensors, it is convenient to have an object that contains a bunch of these histograms and which
 * can be used to set configuration options for all of them at once.
 */
template <typename T, typename R>
class CMultiHistogram {
public:
	//! Construct a collection of histograms, the sliding window has to be set still
	CMultiHistogram(): window_size(-1) {}

	//! Construct a collection of histograms with given sliding window
	CMultiHistogram(int sliding_window) {
		set_sliding_window(window_size);
	}

	//! The deconstructor deletes the collection including the allocated histograms themselves
	~CMultiHistogram() {
		for (int i = 0; i < histograms.size(); ++i)
			delete histograms[i];
		histograms.clear();
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

	//! Push an item unto histogram i
	void push(int i, T item) {
		histograms[i]->push(item);
	}

	//! Get average of histogram i
	R average(int i) {
		return histograms[i]->average();
	}

	R variance(int i) {
		return histograms[i]->variance();
	}

	//! Get sum of histogram i
	T sum(int i) {
		return histograms[i].sum();
	}

	void clear(int i) {
		histograms[i]->clear();
	}

	/**
	 * Return the average of each histogram from index 0 to index N. Use it like this:
	 *   vector<int> values;
	 *   values.resize(histogram->size()); // make sure you have indeed allocated a vector with the right size
	 *   histogram->average(values.begin()).
	 *
	 * @template OutputIterator        any iterator that defines the ++ operator
	 * @param result                   pointer to where the results should be written
	 * @return                         pointer to entry beyond last one (most often end of the container)
	 */
	template<typename OutputIterator>
	OutputIterator average(OutputIterator result) {
		for (int i = 0; i < histograms.size(); ++i, ++result) {
			*result = histograms[i]->average();
		}
		return result;
	}

private:
	//! Internally, we use a vector of pointers to histograms
	std::vector<CHistogram<T,R>* > histograms;

	//! Window size is defined across all histograms
	int window_size;
};

#endif /* CMULTIHISTOGRAM_H_ */

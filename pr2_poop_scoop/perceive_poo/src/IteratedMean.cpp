#include <iostream>
#include <vector>
#include <algorithm>
#include <numeric>
#include <math.h>

using namespace std;

// Comparison class that returns true if the absolute difference
// between the given origin and a value is greater than some
// threshold.
template <class T>
class FarFrom {
    T zero;
    T threshold;
public:
    FarFrom(T zero, T threshold) : zero(zero), threshold(threshold) {}
    bool operator() (T x) { return fabs(x - zero) > threshold; }
};

// We want to compute variance with a single pass over the data, so we
// need to sum the elements of the collection, and their squares.
template <class T>
pair<T,T> updateVariance(pair<T,T> acc, T x)
{
    acc.first += x;
    acc.second += x*x;
    return acc;
}

// Compute an iterated mean that considers only inliers with respect
// to a distance from a full-population mean expressed in terms of
// standard deviations. The last parameter is unused, but locks down
// the type parameter T so we don't have to specify the instantiation
// of the template at the callsite.
template <class InputIterator, class T>
T iteratedMean(InputIterator start, InputIterator last, float stdDevDist, T dumb)
{
    pair<T,T> p = accumulate(start,last,make_pair((T)0, (T)0), updateVariance<float>);
    T n = (T)(last - start);
    T mean = p.first / n;
    T variance = (p.second / n) - mean*mean;
    T dist = sqrt(variance) * stdDevDist;
    while(1) {
        InputIterator cend = remove_if(start,last,FarFrom<T>(mean,dist));
        if(cend - start == 0) {
	  //cout << "iteratedMean: Enlarging inlier interval" << endl;
            dist *= 1.5;
            continue;
        }
        return accumulate(start,cend,(T)0) / (T)(cend - start);
    }
}

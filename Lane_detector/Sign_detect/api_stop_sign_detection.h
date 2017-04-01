#include "opencv2/opencv.hpp"
#include <cstdio>
#include <cmath>
#include <queue>
#include <vector>

using namespace cv;
using namespace std;

namespace stopsign {
    Mat redToBinary(const Mat &bgr);
    void findComponents(const Mat& binary, vector<Rect> &boxes, double lowerRatio = 0.5, double upperRatio = 2.0, int lowerNumPoint = 150);
}

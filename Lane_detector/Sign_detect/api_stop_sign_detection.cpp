#include "api_stop_sign_detection.h"

namespace stopsign {

///BGR2HSV [0, 255]^3 -> [0, 1];[0, 1]; [0, 1]
void BGR2HSV(float b, float g, float r, float &h, float &s, float &v) {
    float mnv, mxv, delta;

    mnv = mxv = r;
    if (mnv > g) mnv = g;
    if (mnv > b) mnv = b;
    if (mxv < g) mxv = g;
    if (mxv < b) mxv = b;
    v = mxv;

    delta = mxv - mnv;

    if( mxv != 0.0)
    	s = delta / mxv;
    else {
    	// r = g = b = 0		// s = 0, v is undefined
    	s = 0.0;
    	h = -1.0;
    	return;
    }

    if(fabs(r - mxv) < 1e-6)
    	h = (g - b ) / delta;		// between yellow & magenta
    else if (fabs(g - mxv) < 1e-6)
    	h = 2 + ( b - r ) / delta;	// between cyan & yellow
    else
    	h = 4 + ( r - g ) / delta;	// between magenta & cyan

    h *= 60;				// degrees
    if(h < 0.0)
    	h += 360;
    h /= 360.0;
}


///Chuyển ảnh từ BGR sang HSV
void BGR2HSV(const Mat &bgr, Mat &hsv) {
    hsv = Mat::zeros(bgr.rows, bgr.cols, CV_32FC3);
    for(int i = 0; i < bgr.rows; ++i)
    for(int j = 0; j < bgr.cols; ++j) {
        float b = 1.0 * bgr.at<Vec3b>(i, j)[0];
        float g = 1.0 * bgr.at<Vec3b>(i, j)[1];
        float r = 1.0 * bgr.at<Vec3b>(i, j)[2];
        float h, s, v;
        BGR2HSV(b, g, r, h, s, v);
        hsv.at<Vec3f>(i, j)[0] = h;
        hsv.at<Vec3f>(i, j)[1] = s;
        hsv.at<Vec3f>(i, j)[2] = v;
    }
}


///Kiểm tra ô (u, v) có nằm trong hình chữ nhật có tọa độ trái trên (0, ) và phải dưới (n, m) không
bool inside(int u, int v, int n, int m) {
    return (0 <= u && u < n && 0 <= v && v < m);
}

///Tìm các thành phần liên thông của ảnh nhị phân, lưu lại hình chữ nhật bao chúng.
///@details Dùng loang để tìm các thành phần liên thông. Một thành phân liên thông được chấp nhận nếu mọi điểm trong đó có màu trắng và có đường đi sang các pixel kề cạnh giữa 2 cặp điểm bất kỳ.
///@details Để loại bỏ các điểm ảnh nhiễu, chỉ giữ lại các thành phần liên thông trong đó mỗi điểm kề với 3/4 hoặc tất cả các ô kề cạnh màu trắng.
///@details Đồng thời các vùng liên thông có số điểm ảnh nhỏ hơn 1/4 vùng liên thông lớn nhất sẽ bị bỏ qua.
///@param binary Ảnh nhị phân đầu vào.
///@param boxes Các hình chữ nhật bao các thành phần liên thông tương ứng.
///@param lowerRatio Tỉ lệ tối thiểu của chiều cao / chiều rộng, nhỏ hơn tỉ lệ này sẽ bị bỏ qua.
///@param upperRatio Tỉ lệ tối đa của chiều cao / chiều rộng, lớn hơn tỉ lệ này sẽ bị bỏ qua.
///@param lowerNumPoint Số lượng điểm tối thiểu trong một thành phần liên thông, ít hơn sẽ bị bỏ qua
void findComponents(const Mat& binary, vector<Rect> &boxes, double lowerRatio, double upperRatio, int lowerNumPoint) {
    boxes.clear();
	int n = binary.rows;
	int m = binary.cols;
	const static int du[] = {0, 0, 1, -1, -1, -1, 1, 1};
	const static int dv[] = {1, -1, 0, 0, 1, -1, 1, -1};
	char **mark = new char*[n];
	//'.': not visited
	//'x': visited, not satisfied
	//'o': visited and satisfied
	for(int r = 0; r < n; ++r) {
		mark[r] = new char[m];
		for(int c = 0; c < m; ++c)
			mark[r][c] = '.';
	}
	size_t largest = 0;
	vector< vector<Point2i> > regions;
	queue<int> q;
	for(int r = 0; r < n; ++r)
	for(int c = 0; c < m; ++c)
	if (binary.at<uchar>(r, c) != 0 && mark[r][c] == '.'){
		int minu = (int)1e9, maxu = -1;
		int minv = (int)1e9, maxv = -1;
		q.push(r);
		q.push(c);
		vector<Point2i> region;
		while (!q.empty()) {
			int u = q.front(); q.pop();
			int v = q.front(); q.pop();
			for(int dir = 0; dir < 4; ++dir) {
				int nu = u + du[dir];
				int nv = v + dv[dir];
				if (nu < 0 || nu >= n || nv < 0 || nv >= m) continue;
				if (mark[nu][nv] != '.') continue;
				if (binary.at<uchar>(nu, nv) == 0) {
					mark[nu][nv] = 'x';
					continue;
				}
				int ones = 0;
				int vals = 0;
				for(int dd = 0; dd  < 4; ++dd)
					if (nu + du[dd] >= 0 && nu + du[dd] < n && nv + dv[dd] >= 0 && nv + dv[dd] < m) {
						++vals;
						if (binary.at<uchar>(nu + du[dd], nv + dv[dd]) != 0) ++ones;
					}
				if (ones == vals){// || (ones == 3 && vals == 4)) {
					if (minu > nu) minu = nu;
					if (maxu < nu) maxu = nu;
					if (minv > nv) minv = nv;
					if (maxv < nv) maxv = nv;
					mark[nu][nv] = 'o';
					region.push_back(Point2i(nu, nv));
					q.push(nu);
					q.push(nv);
				}
				else
					mark[nu][nv] = 'x';
			}
		}
		if (maxu - minu < 10 || maxv - minv < 10) continue;
        if ((int)region.size() < lowerNumPoint) continue;
		double ratio = (1.0 * maxu - minu) / (maxv - minv);
		if (ratio < lowerRatio || ratio > upperRatio) continue;
		boxes.push_back(Rect(minv, minu, maxv - minv, maxu - minu));
		regions.push_back(region);
		if (largest < region.size())
			largest = region.size();
	}
	for(size_t i = 0; i < regions.size(); ++i)
		if (4 * regions[i].size() < largest) {
			swap(regions[i], regions.back());
			swap(boxes[i], boxes.back());
			regions.pop_back();
			boxes.pop_back();
		}
	for(int i = 0; i < n; ++i)
		delete[] mark[i];
	delete mark;
}

///Tìm các thành phần có màu đỏ (trong một khoảng xác định nào đó), đánh dấu các thành phần đó là màu trắng trong ảnh nhị phân, còn lại là màu đen.
Mat redToBinary(const Mat &bgr) {
    Mat binary = Mat::zeros(bgr.rows, bgr.cols, CV_8U);
    Mat hsv;
    BGR2HSV(bgr, hsv);
    imshow("hsv", hsv);
    for(int r = 0; r < hsv.rows; ++r)
    for(int c = 0; c < hsv.cols; ++c)
    	if (hsv.at<Vec3f>(r, c)[0] < 0.1 || hsv.at<Vec3f>(r, c)[0] > 0.95) // < 0.05 || >0.95
        if (hsv.at<Vec3f>(r, c)[1] > 0.4 && hsv.at<Vec3f>(r, c)[2] > 0.15) // > 0.4 && > 0.15
            binary.at<uchar>(r, c) = 255;
    return binary;
}

}

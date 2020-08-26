#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

#define VIDEO_PATH "../../track-s.mkv"

using namespace std;
using namespace cv;

int main(void) {
    int img_width, img_height;
    img_width = IMG_WIDTH;
    img_height = IMG_HEIGHT;

    Mat mat_image_org_color;
    Mat mat_image_org_gray;
    Mat mat_image_gray_result;
    Mat image;

    Scalar RED(255, 0, 0);
    Scalar GREEN(0, 255, 0);
    Scalar BLUE(0, 0, 255);
    Scalar YELLOW(255, 255, 0);

    VideoCapture cap(VIDEO_PATH);
    
    if (!cap.isOpened()) {
        cerr << "Error - Can not open " << VIDEO_PATH << '\n';
        return -1;
    }

    printf("Image size[%3d,%3d]\n", img_width, img_height);

    namedWindow("Display window", CV_WINDOW_NORMAL);
    resizeWindow("Display window", img_width, img_height);
    moveWindow("Display window", 10, 10);

    while (true) {
        cap.read(mat_image_org_color);
        if (mat_image_org_color.empty()) {
            cout << "End of video.\n";
            break;
        }

        imshow("Display window", mat_image_org_color);

        if (waitKey(10) > 0) {
            break;
        }
    }

    cap.release();
    destroyAllWindows();

    return 0;
}

#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

#define IMAGE_PATH "../../lanecolor102.bmp"

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

    mat_image_org_color = imread(IMAGE_PATH);

    img_width = mat_image_org_color.size().width;
    img_height = mat_image_org_color.size().height;

    printf("Image size[%3d,%3d]\n", img_width, img_height);

    namedWindow("Display window", CV_WINDOW_NORMAL);
    resizeWindow("Display window", img_width, img_height);
    moveWindow("Display window", 10, 10);

    while (true) {
        if (mat_image_org_color.empty()) {
            cout << "Empty image.\n";
            break;
        }

        imshow("Display window", mat_image_org_color);

        if (waitKey(10) > 0) {
            break;
        }
    }

    destroyAllWindows();

    return 0;
}

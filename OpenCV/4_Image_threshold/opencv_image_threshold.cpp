#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

#define IMAGE_PATH "../../lanecolor102.bmp"

#define ORG_NAME "Display window"
#define GRAY_NAME "Gray image window"
#define BINARY_NAME "Binary image window"

using namespace std;
using namespace cv;

int main(void) {
    int img_width, img_height;
    img_width = IMG_WIDTH;
    img_height = IMG_HEIGHT;

    Mat mat_image_org_color;
    Mat mat_image_org_gray;
    Mat mat_image_gray_result;
    Mat mat_image_binary;
    Mat image;

    Scalar RED(255, 0, 0);
    Scalar GREEN(0, 255, 0);
    Scalar BLUE(0, 0, 255);
    Scalar YELLOW(255, 255, 0);

    mat_image_org_color = imread(IMAGE_PATH);

    img_width = mat_image_org_color.size().width;
    img_height = mat_image_org_color.size().height;

    printf("Image size[%3d,%3d]\n", img_width, img_height);

    namedWindow(ORG_NAME, CV_WINDOW_NORMAL);
    resizeWindow(ORG_NAME, img_width, img_height);
    moveWindow(ORG_NAME, 10, 10);

    namedWindow(GRAY_NAME, CV_WINDOW_NORMAL);
    resizeWindow(GRAY_NAME, img_width, img_height);
    moveWindow(GRAY_NAME, 700, 10);

    namedWindow(BINARY_NAME, CV_WINDOW_NORMAL);
    resizeWindow(BINARY_NAME, img_width, img_height);
    moveWindow(BINARY_NAME, 10, 500);

    while (true) {
        cvtColor(mat_image_org_color, mat_image_org_gray, CV_RGB2GRAY); // color to gray conversion
        threshold(mat_image_org_gray, mat_image_binary, 200, 255, THRESH_BINARY); // gray to binary conversion

        if (mat_image_org_color.empty()) {
            cout << "Empty image.\n";
            break;
        }

        imshow(ORG_NAME, mat_image_org_color);
        imshow(GRAY_NAME, mat_image_org_gray);
        imshow(BINARY_NAME, mat_image_binary);

        if (waitKey(10) > 0) {
            break;
        }
    }

    destroyAllWindows();

    return 0;
}

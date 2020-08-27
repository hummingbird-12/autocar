#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>

#define IMG_WIDTH 640
#define IMG_HEIGHT 480

#define IMAGE_PATH "./images/line_1_0.jpg"

#define ORG_NAME "Display window"
#define GRAY_NAME "Gray image window"
#define CANNY_NAME "Canny edge image window"

using namespace std;
using namespace cv;

Mat canny_edge_detection(Mat img) {
    Mat mat_blur_image, mat_canny_image;

    blur(img, mat_blur_image, Size(3, 3));
    Canny(mat_blur_image, mat_canny_image, 70, 170, 3);

    return mat_canny_image;
}

int main(void) {
    int img_width, img_height;
    img_width = IMG_WIDTH;
    img_height = IMG_HEIGHT;

    Mat mat_image_org_color;
    Mat mat_image_org_gray;
    Mat mat_image_gray_result;
    Mat mat_image_canny_edge;
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

    namedWindow(CANNY_NAME, CV_WINDOW_NORMAL);
    resizeWindow(CANNY_NAME, img_width, img_height);
    moveWindow(CANNY_NAME, 10, 500);

    while (true) {
        // Color to gray conversion
        cvtColor(mat_image_org_color, mat_image_org_gray, CV_RGB2GRAY);
        // Edge detection from grayscale
        mat_image_canny_edge = canny_edge_detection(mat_image_org_gray);

        if (mat_image_org_color.empty()) {
            cout << "Empty image.\n";
            break;
        }

        imshow(ORG_NAME, mat_image_org_color);
        imshow(GRAY_NAME, mat_image_org_gray);
        imshow(CANNY_NAME, mat_image_canny_edge);

        if (waitKey(10) > 0) {
            break;
        }
    }

    destroyAllWindows();

    return 0;
}

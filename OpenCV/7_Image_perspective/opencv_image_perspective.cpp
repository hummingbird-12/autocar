#include <cstdio>
#include <iostream>
#include <opencv2/opencv.hpp>

#define IMG_WIDTH 640
#define IMG_HEIGHT 480
#define PERSP_IMG_WIDTH 640
#define PERSP_IMG_HEIGHT 480

#define IMAGE_PATH "./images/line_2_0.jpg"

#define ORG_NAME "Display window"
#define GRAY_NAME "Gray image window"
#define CANNY_NAME "Canny edge image window"

using namespace std;
using namespace cv;

const Point2f source[] = {
    Point2f(0, 0), Point2f(-270, 330),
    Point2f(640, 0), Point2f(640 + 270, 330)
};
const Point2f destination[] = {
    Point2f(0, 0), Point2f(0, PERSP_IMG_HEIGHT),
    Point2f(PERSP_IMG_WIDTH, 0), Point2f(PERSP_IMG_WIDTH, PERSP_IMG_HEIGHT)
};

Mat Perspective(Mat img) {
    Mat matrix, result_img;

    matrix = getPerspectiveTransform(source, destination);
    warpPerspective(img, result_img, matrix, Size(PERSP_IMG_WIDTH, PERSP_IMG_HEIGHT));

    return result_img;
}

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

    Scalar BLUE(255, 0, 0);
    Scalar GREEN(0, 255, 0);
    Scalar RED(0, 0, 255);
    Scalar YELLOW(0, 255, 255);

    mat_image_org_color = imread(IMAGE_PATH);

    img_width = mat_image_org_color.size().width;
    img_height = mat_image_org_color.size().height;

    printf("Image size[%3d,%3d]\n", img_width, img_height);

    namedWindow(ORG_NAME, CV_WINDOW_NORMAL);
    resizeWindow(ORG_NAME, img_width, img_height);
    moveWindow(ORG_NAME, 10, 20);

    namedWindow(GRAY_NAME, CV_WINDOW_NORMAL);
    resizeWindow(GRAY_NAME, img_width, img_height);
    moveWindow(GRAY_NAME, 700, 20);

    namedWindow(CANNY_NAME, CV_WINDOW_NORMAL);
    resizeWindow(CANNY_NAME, img_width, img_height);
    moveWindow(CANNY_NAME, 10, 520);

    while (true) {
        mat_image_org_color = imread(IMAGE_PATH);
        cvtColor(mat_image_org_color, mat_image_org_gray, CV_RGB2GRAY); // color to gray conversion
        mat_image_canny_edge = canny_edge_detection(mat_image_org_gray);
        image = Perspective(mat_image_canny_edge);

        vector < Vec4i > linesP;
        // HoughLinesP(mat_image_canny_edge, linesP, 1, CV_PI / 180, 70, 30, 40);
        HoughLinesP(image, linesP, 1, CV_PI / 180, 50, 20, 50);

        printf("Line number : %3d\n", linesP.size());
        for (int i = 0; i < (int) linesP.size(); i++) {
            const Vec4i& L = linesP[i];
            const int& cx1 = linesP[i][0];
            const int& cy1 = linesP[i][1];
            const int& cx2 = linesP[i][2];
            const int& cy2 = linesP[i][3];

            line(mat_image_org_color, Point(L[0], L[1]), Point(L[2], L[3]), RED, 3, LINE_AA);
            printf("L : [%3d,%3d] , [%3d,%3d]\n", cx1, cy1, cx2, cy2);
        }
        printf("\n\n\n");

        if (mat_image_org_color.empty()) {
            cout << "Empty image.\n";
            break;
        }

        imshow(ORG_NAME, mat_image_org_color);
        imshow(GRAY_NAME, image);
        // imshow(GRAY_NAME, mat_image_org_gray);
        imshow(CANNY_NAME, mat_image_canny_edge);

        if (waitKey(10) > 0) {
            break;
        }
    }

    destroyAllWindows();

    return 0;
}

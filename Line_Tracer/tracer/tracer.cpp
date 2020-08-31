#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <algorithm>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <softPwm.h>
#include <termio.h>
#include <wiringPi.h>
#include <wiringSerial.h>

/***** Image Processing *****/
#define IMG_WIDTH 320
#define IMG_HEIGHT 240

#define IMAGE_PATH "./images/line_1_0.jpg"

#define ORG_OVERLAY_NAME "Display window"
#define ROI_EDGE_NAME "ROI edge window"
#define LINE_IMG_NAME "Line image window"

#define ASSIST_BASE_LINE 120
#define ASSIST_BASE_WIDTH 20
#define ASSIST_PADDING 0

/***** GPIO Motor Control *****/
#define ENA 1 // Physical 12
#define IN1 4 // Physical 16
#define IN2 5 // Physical 18

#define ENB 0 // Physical 11
#define IN3 2 // Physical 13
#define IN4 3 // Physical 15

#define MAX_PWM_DUTY 100
#define PWM_BASE 15
#define P_gain 0.05

/***** Ultrasonic Sensor *****/
#define TRIG 21 // Physical 29
#define _ECHO 22 // Physical 31

/***** Serial Communication *****/
#define SERIAL_DEV "/dev/ttyACM0"
#define BAUD_RATE 115200

using namespace std;
using namespace cv;

void sig_handler(const int sig);
int getch(void);

int GPIO_control_setup(void);
void motor_control_l(const int pwm);
void motor_control_r(const int pwm);
float ultrasonic_sensor(void);
Mat canny_edge_detection(Mat img);
Mat draw_guide_lines(Mat img);
Mat region_of_interest(Mat img, Point *points);
void line_tracer_motor_control(int line_center);

const Scalar BLUE(255, 0, 0);
const Scalar GREEN(0, 255, 0);
const Scalar RED(0, 0, 255);
const Scalar YELLOW(0, 255, 255);
const Scalar WHITE(255, 255, 255);

int guide_width1 = 30;
int guide_height1 = 10;
int guide_center = IMG_WIDTH / 2;
int line_center = -1;

void mask_y(Mat& img) {
    // Mat img_mask = Mat::zeros(img.rows, img.cols, CV_8UC3);
    int i,j;
    for( i = 0; i < img.rows; ++i) {
        for ( j = 0; j < img.cols; ++j) {
            img.at< Vec3b >(i,j)[0] = 0;
            if( img.at<Vec3b>(i,j)[1] < 200 || img.at<Vec3b>(i,j)[2] < 200 || img.at<Vec3b>(i,j)[0] > 100) {
                img.at< Vec3b >(i,j)[0] = img.at< Vec3b >(i,j)[1] = img.at< Vec3b >(i,j)[2] = 0;
            }
        }
    }
}

int main(void) {
    int img_width, img_height;
    int c_x_sum = 0;
    img_width = IMG_WIDTH;
    img_height = IMG_HEIGHT;

    Mat mat_image_org_color;
    Mat mat_image_org_color_overlay;
    Mat mat_image_org_gray;
    Mat mat_image_gray_result;
    Mat mat_image_canny_edge;
    Mat mat_image_roi_canny_edge;
    Mat mat_image_line_image = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC1, Scalar(0));
    Mat image;
    Mat img_labels, stats, centroids;

    Point points[4] = {
        Point(ASSIST_PADDING, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH),
        Point(ASSIST_PADDING, ASSIST_BASE_LINE + ASSIST_BASE_WIDTH),
        Point(IMG_WIDTH - ASSIST_PADDING, ASSIST_BASE_LINE + ASSIST_BASE_WIDTH),
        Point(IMG_WIDTH - ASSIST_PADDING, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH)
    };

    VideoCapture cap(0);

    cap.set(CV_CAP_PROP_FRAME_WIDTH, img_width);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, img_height);
    
    if (!cap.isOpened()) {
        cerr << "Error - Can not open camera.\n";
        mat_image_org_color = imread(IMAGE_PATH);
    }
    else {
        cap.read(mat_image_org_color);
    }

    if (mat_image_org_color.empty()) {
        cout << "Empty image.\n";
        return -1;
    }
    mat_image_org_color.copyTo(mat_image_org_color_overlay);

    img_width = mat_image_org_color.size().width;
    img_height = mat_image_org_color.size().height;

    printf("Image size[%3d,%3d]\n", img_width, img_height);

    namedWindow(ORG_OVERLAY_NAME, CV_WINDOW_NORMAL);
    resizeWindow(ORG_OVERLAY_NAME, img_width, img_height);
    moveWindow(ORG_OVERLAY_NAME, 10, 10);

    namedWindow(ROI_EDGE_NAME, CV_WINDOW_NORMAL);
    resizeWindow(ROI_EDGE_NAME, img_width, img_height);
    moveWindow(ROI_EDGE_NAME, 700, 10);

    namedWindow(LINE_IMG_NAME, CV_WINDOW_NORMAL);
    resizeWindow(LINE_IMG_NAME, img_width, img_height);
    moveWindow(LINE_IMG_NAME, 10, 500);

    if (wiringPiSetup() == -1) {
        printf("WiringPi setup error!\n");
        return -1;
    }

    if (GPIO_control_setup() != 0) {
        printf("GPIO setup error!\n");
        return -1;
    }

    signal(SIGINT, sig_handler);

    while (true) {
        if (!cap.isOpened()) {
            cerr << "Error - Can not open camera.\n";
            motor_control_r(0);
            motor_control_l(0);
            return -1;
        }
        cap.read(mat_image_org_color);

        // Blur camera capture
        Mat blurred = Mat(IMG_HEIGHT, IMG_WIDTH, CV_8UC3, Scalar(0));
        blur(mat_image_org_color, blurred, Size(5, 5));
        // Mask out pixels very likely to be not yello
        mask_y(blurred);
        // Blur masked image
        blur(blurred, mat_image_org_color, Size(5, 5));

        mat_image_line_image = Scalar(0);

        // Color to gray conversion
        cvtColor(mat_image_org_color, mat_image_org_gray, CV_RGB2GRAY);
        // Gray to binary conversion
        threshold(mat_image_org_gray, mat_image_canny_edge, 200, 255, THRESH_BINARY);
        // Edge detection from grayscale
        mat_image_canny_edge = canny_edge_detection(mat_image_org_gray);
        // Obtain ROI
        mat_image_roi_canny_edge = region_of_interest(mat_image_canny_edge, points);

        // Hough lines detection
        vector < Vec4i > linesP;
        /*
         * threshold: The minimum number of intersections to “detect” a line
         * minLinLength: The minimum number of points that can form a line. Lines with less than this number of points are disregarded.
         * maxLineGap: The maximum gap between two points to be considered in the same line.
         */
        HoughLinesP(mat_image_roi_canny_edge, linesP, 1, CV_PI / 180, 12, 8, 20);

        c_x_sum = 0;
        // Draw Hough lines
        printf("Line number : %3d\n", linesP.size());
        for (int i = 0; i < (int) linesP.size(); i++) {
            const Vec4i& L = linesP[i];
            const int gradient = (float) (L[2] - L[0]) / (L[3] - L[1]);
            const int intersect = L[0] - gradient * L[1];
            const int intersect_base = intersect + gradient * ASSIST_BASE_LINE;
            c_x_sum += intersect_base;
            line(mat_image_line_image, Point(L[0], L[1]), Point(L[2], L[3]), Scalar(255), 4, LINE_AA);
            circle(mat_image_org_color, Point(intersect_base, ASSIST_BASE_LINE), 5, BLUE, -1);
        }

        printf("c_x_sum = %d\n", c_x_sum);
        if (!linesP.empty()) {
            line_center = c_x_sum / (linesP.size());
            printf("Line Center : %3d \n", line_center);
            line_tracer_motor_control(line_center);
        }
        printf("\n\n\n");

        // Line guidance
        mat_image_org_color_overlay = draw_guide_lines(mat_image_org_color);

        imshow(ORG_OVERLAY_NAME, mat_image_org_color_overlay);
        imshow(ROI_EDGE_NAME, mat_image_roi_canny_edge);
        imshow(LINE_IMG_NAME, mat_image_line_image);

        if (waitKey(10) > 0) {
            break;
        }
    }

    motor_control_r(0);
    motor_control_l(0);

    return 0;
}

void sig_handler(int sig) {
    printf("\n\n\nProgram and motor stop!\n\n");
    motor_control_r(0);
    motor_control_l(0);
    exit(0);
}

// `getch()` implementation for Raspberry Pi
int getch(void) {
    int ch;
    struct termios buf;
    struct termios save;

    tcgetattr(0, &save);
    buf = save;
    buf.c_lflag &= ~(ICANON|ECHO);
    buf.c_cc[VMIN] = 1;
    buf.c_cc[VTIME] = 0;
    tcsetattr(0, TCSAFLUSH, &buf);
    ch = getchar();
    tcsetattr(0, TCSAFLUSH, &save);
    return ch;
}

int GPIO_control_setup(void) {
    if (wiringPiSetup() == -1) {
        printf("WiringPi setup error!\n");
        return -1;
    }

    pinMode(ENA, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

    pinMode(ENB, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);

    pinMode(TRIG, OUTPUT);
    pinMode(_ECHO, INPUT);

    softPwmCreate(ENA, 1, MAX_PWM_DUTY);
    softPwmCreate(ENB, 1, MAX_PWM_DUTY);

    softPwmWrite(ENA, 0);
    softPwmWrite(ENB, 0);

    return 0;
}

void motor_control_l(const int pwm) {
    if (pwm > 0) {
        digitalWrite(IN3, HIGH);
        digitalWrite(IN4, LOW);
        softPwmWrite(ENB, pwm);
    }
    else if (pwm == 0) {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, LOW);
        softPwmWrite(ENB, 0);
    }
    else {
        digitalWrite(IN3, LOW);
        digitalWrite(IN4, HIGH);
        softPwmWrite(ENB, -pwm);
    }
}

void motor_control_r(const int pwm) {
    if (pwm > 0) {
        digitalWrite(IN1, HIGH);
        digitalWrite(IN2, LOW);
        softPwmWrite(ENA, pwm);
    }
    else if (pwm == 0) {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, LOW);
        softPwmWrite(ENA, 0);
    }
    else {
        digitalWrite(IN1, LOW);
        digitalWrite(IN2, HIGH);
        softPwmWrite(ENA, -pwm);
    }
}

float ultrasonic_sensor(void) {
    long temp_time;
    long start_time = 0, end_time = 0;
    int duration;
    float distance;

    digitalWrite(TRIG, LOW);
    delayMicroseconds(5);
    digitalWrite(TRIG, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG, LOW);

    delayMicroseconds(200); // Wait for burst signal (40kHz * 8 = 8 * 25us = 200)

    temp_time = micros();
    // Wait until ECHO pin is HIGH
    while (digitalRead(_ECHO) == LOW) {
        if (micros() - temp_time > 3000) {
            return -1;
        }
    }
    start_time = micros();

    // Wait until ECHO pin is LOW
    while (digitalRead(_ECHO) == HIGH) {
        if (micros() - start_time > 20000) {
            return -1;
        }
    }
    end_time = micros();
    duration = end_time - start_time;
    distance = duration / 58.0;

    return distance;
}

Mat canny_edge_detection(Mat img) {
    Mat mat_blur_image, mat_canny_image;

    blur(img, mat_blur_image, Size(3, 3));
    Canny(mat_blur_image, mat_canny_image, 70, 170, 3);

    return mat_canny_image;
}

Mat draw_guide_lines(Mat img) {
    Mat result_img;
    img.copyTo(result_img);

    // Guide box
    rectangle(result_img,
        Point(ASSIST_PADDING, ASSIST_BASE_LINE - ASSIST_BASE_WIDTH),
        Point(IMG_WIDTH - ASSIST_PADDING, ASSIST_BASE_LINE + ASSIST_BASE_WIDTH),
        GREEN, 1, LINE_AA);

    // Guide
    line(result_img,
        Point(guide_center - guide_width1, ASSIST_BASE_LINE),
        Point(guide_center, ASSIST_BASE_LINE),
        YELLOW, 1, 0);
    line(result_img,
        Point(guide_center, ASSIST_BASE_LINE),
        Point(guide_center + guide_width1, ASSIST_BASE_LINE),
        YELLOW, 1, 0);
    line(result_img,
        Point(guide_center - guide_width1, ASSIST_BASE_LINE - guide_height1),
        Point(guide_center - guide_width1, ASSIST_BASE_LINE + guide_height1),
        YELLOW, 1, 0);
    line(result_img,
        Point(guide_center + guide_width1, ASSIST_BASE_LINE - guide_height1),
        Point(guide_center + guide_width1, ASSIST_BASE_LINE + guide_height1),
        YELLOW, 1, 0);
    
    // Center guide
    line(result_img,
        Point(IMG_WIDTH / 2, ASSIST_BASE_LINE - guide_height1 * 1.5),
        Point(IMG_WIDTH / 2, ASSIST_BASE_LINE + guide_height1 * 1.5),
        WHITE, 2, 0);
    
    // Line center
    line(result_img,
        Point(line_center, ASSIST_BASE_LINE - guide_height1 * 1.2),
        Point(line_center, ASSIST_BASE_LINE + guide_height1 * 1.2),
        RED, 2, 0);
    
    return result_img;
}

Mat region_of_interest(Mat img, Point *points) {
    Mat img_mask = Mat::zeros(img.rows, img.cols, CV_8UC1);
    Scalar mask_color = WHITE;
    const Point* pt[1] = { points };
    int npt[] = { 4 };

    fillPoly(img_mask, pt, npt, 1, mask_color, LINE_8);

    Mat masked_img;
    bitwise_and(img, img_mask, masked_img);

    return masked_img;
}

void line_tracer_motor_control(int line_center) {
    int pwm_l, pwm_r;
    int steer_error = line_center - IMG_WIDTH / 2;

    pwm_l = PWM_BASE + steer_error * P_gain;
    pwm_r = PWM_BASE - steer_error * P_gain;

    pwm_l = max(pwm_l, -70);
    pwm_l = min(pwm_l, 70);
    pwm_r = max(pwm_r, -70);
    pwm_r = min(pwm_r, 70);

    printf("PWM_L : %3d | PWM_R : %3d\n", pwm_l, pwm_r);
    motor_control_l(pwm_l);
    motor_control_r(pwm_r);
    // motor_control_l(PWM_BASE * 1.2);
    // motor_control_r(PWM_BASE * 1.2);
}

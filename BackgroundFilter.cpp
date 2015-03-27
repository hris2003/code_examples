/*
 * Filter out the background of the camera and replace by movie scenes
 * Author: Ha Dang
 * Date: 27 March 2015
 */
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <stdio.h>

using namespace std;
using namespace cv;

int main(void)
{
    //variables first
    Size S = Size(320, 240);
    Mat frame, bgframe, pre_Frame, gray_image, diff_image;
    Mat result = Mat::zeros(S.height, S.width, CV_8UC3);
    Mat dst = Mat(S.height*2, S.width*2, CV_8UC3, cv::Scalar(0,0,0));

    //Define the ROI for each component in the dst frame
    cv::Rect roibg(cv::Rect(0,0,S.width, S.height));
    cv::Rect roifg(cv::Rect(S.width, 0,S.width, S.height));
    cv::Rect roiresult(cv::Rect(S.width/2, S.height,S.width, S.height));


    VideoCapture capture(0);
    VideoCapture bgVid("../../../Megamind.avi");

    if (!capture.isOpened())  // check if we have cam
        return -1;

    if (!bgVid.isOpened())  //check if we have video
        {
            cout  << "Could not open the input video" << endl;
            return -1;
        }

    //Get the background
    while (frame.empty()){
        capture >> frame;
    }
    resize(frame, frame,S);
    cvtColor( frame, pre_Frame, CV_BGR2GRAY );

    while (true)
    {
        capture >> frame;
        bgVid >> bgframe;

        if (bgframe.empty()){//If no more movie frame, just put black frame
            bgframe = Mat::zeros(S.height, S.width, CV_8UC3);
        }else{
            resize(bgframe, bgframe,S);
        }
        // Apply the filter to the frame
        if (!frame.empty())
        {
            resize(frame, frame,S);

            //Filter out the background
            result = Mat::zeros(S.height, S.width, CV_8UC3);
            cvtColor( frame, gray_image, CV_BGR2GRAY );
            absdiff(gray_image, pre_Frame, diff_image);
            inRange(diff_image, Scalar(50), Scalar(255), diff_image);

            //Move everything to the dst frame
            bgframe.copyTo(dst(roibg));
            frame.copyTo(result, diff_image);
            frame.copyTo(bgframe, diff_image);
            result.copyTo(dst(roifg));
            bgframe.copyTo(dst(roiresult));

            //Show the result
            imshow("result", dst);
        }
        else
        {
            printf(" --(!) No captured frame -- Break!");
            break;
        }

        int c = waitKey(1);
        if (27 == char(c))
        {
            break;
        }
    }

    return 0;
}



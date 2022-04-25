#include <iostream>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

const int fps = 20; // frames por second

int main()
{

    Mat frame; 
    
    VideoCapture videocapture(0); 


    if (!videocapture.isOpened()) 
        return -1;


    while (1) {

        bool ret = videocapture.read(frame);
        if (ret == false) break;


        imshow("Webcam", frame);


        int key = waitKey(1000 / fps);
        if (key == 27 ) 
            break;
    }

    return 0; 
}

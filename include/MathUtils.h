/* 
 * File:   MathUtils.h
 * Author: daniele
 *
 * Created on 22 gennaio 2014, 11.43
 */
#include "ros/ros.h"
#include <tf/tf.h>

#include <utility>
#include <stdlib.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <math.h>

#include "aruco/aruco.h"

#define PI 3.14159265

#ifndef MATHUTILS_H
#define	MATHUTILS_H

using namespace cv;
using namespace std;

namespace lar_visionsystem{
    class MathUtils {
    public:
        MathUtils();
        virtual ~MathUtils();
        static Mat getRotationMatrix(char,double);
        static Mat getRotationMatrix4(char,double);
        static Mat eulerZYZToRot(Mat* euler);
        static Mat rotToEulerZYZ(Mat* source);
        static Mat rotToRPY(Mat *source);
        static Mat getRodriguezMatrix(Mat* v);
        static Mat getRodriguezVector(Mat* m);
        static Mat getTMarker(aruco::Marker& marker);
        static Mat getTransformMatrix(Mat* rotation,Mat* translation, int cv_type);
        static Mat getRotFromTtransform(Mat* T,int cv_type);
        static Mat getTFromTtransform(Mat* T,int cv_type);
        static pair<Mat,Mat> decomposeTransformMatrix(Mat* transform,int cv_type);
        static Mat inverseTransform(Mat* transform);
        static Mat pos6Dof(Mat* transform);
        static Mat translateT(Mat* transform,float x,float y,float z);
        static Mat dumpMats(vector<Mat>);
        static tf::Transform matToTF(cv::Mat& mat);
    private:

    };
}
#endif	/* MATHUTILS_H */


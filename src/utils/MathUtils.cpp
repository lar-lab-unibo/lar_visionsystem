/*
 * File:   MathUtils.cpp
 * Author: daniele
 *
 * Created on 22 gennaio 2014, 11.43
 */

#include "MathUtils.h"

using namespace cv;

namespace lar_visionsystem {

MathUtils::MathUtils() {
}

MathUtils::~MathUtils() {
}

Mat MathUtils::getRotationMatrix(char axis, double deg) {
        Mat rot;
        double rad = deg * PI / 180;
        if (axis == 'x') {
                rot = (Mat_<double>(3, 3) << 1, 0, 0, 0, cos(rad), -sin(rad), 0, sin(rad), cos(rad));
        } else if (axis == 'y') {
                rot = (Mat_<double>(3, 3) << cos(rad), 0, sin(rad), 0, 1, 0, -sin(rad), 0, cos(rad));
        } else if (axis == 'z') {
                rot = (Mat_<double>(3, 3) << cos(rad), -sin(rad), 0, sin(rad), cos(rad), 0, 0, 0, 1);
        }
        rot.convertTo(rot, CV_32FC1);
        return rot;
}

Mat MathUtils::getTMarker(aruco::Marker& marker) {
        Mat rot = MathUtils::getRodriguezMatrix(&(marker.Rvec));
        Mat trans = marker.Tvec.clone()*1000;
        Mat T = MathUtils::getTransformMatrix(&rot, &trans, CV_32FC1);
        return T;
}

Mat MathUtils::getRotationMatrix4(char axis, double deg) {
        Mat rot = MathUtils::getRotationMatrix(axis, deg);
        Mat t = MathUtils::getTransformMatrix(&rot, NULL, CV_32FC1);
        return t;
}

Mat MathUtils::getRodriguezMatrix(Mat* v) {
        Mat mat;
        if (v != NULL) {
                Rodrigues(*v, mat);
        }
        return mat;
}

Mat MathUtils::getRodriguezVector(Mat* m) {
        Mat v;
        if (m != NULL) {
                Rodrigues(*m, v);
        }
        return v;
}

Mat MathUtils::eulerZYZToRot(Mat* euler) {
        double e1 = euler->at<double>(0, 0);
        double e2 = euler->at<double>(0, 1);
        double e3 = euler->at<double>(0, 2);
        Mat r1 = MathUtils::getRotationMatrix('z', e1);
        Mat r2 = MathUtils::getRotationMatrix('y', e2);
        Mat r3 = MathUtils::getRotationMatrix('z', e3);
        r1.convertTo(r1, CV_64FC1);
        r2.convertTo(r2, CV_64FC1);
        r3.convertTo(r3, CV_64FC1);
        r1 = r1*r2;
        r1 = r1*r3;
        r1.convertTo(r1, CV_32FC1);

        return r1;
}

Mat MathUtils::rotToEulerZYZ(Mat* source) {

        assert(source != NULL);

        Mat s(*source);
        s.convertTo(s, CV_64FC1);

        float r33 = source->at<float>(2, 2);
        float r23 = source->at<float>(1, 2);
        float r13 = source->at<float>(0, 2);
        float r32 = source->at<float>(2, 1);
        float r31 = source->at<float>(2, 0);


        float e1 = atan2(r23, r13);
        float e2 = acos(r33);
        float e3 = atan2(r32, -r31);


        float e1_deg = e1 * 180 / PI;
        float e2_deg = e2 * 180 / PI;
        float e3_deg = e3 * 180 / PI;


        Mat m(1, 3, CV_32F);
        m.at<float>(0, 0) = e1_deg;
        m.at<float>(0, 1) = e2_deg;
        m.at<float>(0, 2) = e3_deg > 0 ? e3_deg : 180 + e3_deg;
        return m;
}

/**
 * YZX
 * @param source
 * @return YZX
 */
Mat MathUtils::rotToRPY(Mat* source) {
        assert(source != NULL);

        Mat s(*source);
        s.convertTo(s, CV_64FC1);


        float r13 = source->at<float>(0, 2);
        float r11 = source->at<float>(0, 0);
        float r33 = source->at<float>(2, 2);


        float e1 = asin(r13);
        float e2 = acos(r11 / cos(e1));
        float e3 = acos(r33 / cos(e1));

        float e1_deg = e1 * 180 / PI;
        float e2_deg = e2 * 180 / PI;
        float e3_deg = e3 * 180 / PI;


        Mat m(1, 3, CV_32F);
        m.at<float>(0, 0) = e1_deg;
        m.at<float>(0, 1) = e2_deg;
        m.at<float>(0, 2) = e3_deg;
        return m;
}

Mat MathUtils::getTransformMatrix(Mat* rotation, Mat* translation, int cv_type) {

        Mat zeros = Mat::zeros(1, 3, CV_32FC1);

        if (rotation == NULL) {
                rotation = new Mat(Mat::eye(3, 3, CV_32FC1));
        }

        if (translation == NULL) {
                translation = new Mat(Mat::zeros(3, 1, CV_32FC1));
        }


        Mat rot(*rotation);
        vconcat(rot, zeros, rot);

        Mat one = (Mat_<float>(1, 1) << 1);
        Mat trans(*translation);
        if (trans.size().width > trans.size().height) {
                trans = trans.t();
        }
        if (trans.size().height == 3) {
                vconcat(trans, one, trans);
        }

        Mat t;
        hconcat(rot, trans, t);
        return t;
}

Mat MathUtils::getRotFromTtransform(Mat* T, int cv_type) {

        Mat tt(*T);
        tt.convertTo(tt, CV_64FC1);
        Mat rot(3, 3, CV_64FC1);
        for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                        rot.at<double>(i, j) = tt.at<double>(i, j);
                }
        }
        rot.convertTo(rot, cv_type);
        return rot;
}

Mat MathUtils::getTFromTtransform(Mat* T, int cv_type) {
        Mat tt(*T);
        tt.convertTo(tt, CV_64FC1);
        Mat t(1, 3, CV_64FC1);
        t.at<double>(0, 0) = tt.at<double>(0, 3);
        t.at<double>(0, 1) = tt.at<double>(1, 3);
        t.at<double>(0, 2) = tt.at<double>(2, 3);
        t.convertTo(t, cv_type);
        return t;
}

pair<Mat, Mat> MathUtils::decomposeTransformMatrix(Mat* transform, int cv_type) {
        Mat R = MathUtils::getRotFromTtransform(transform, cv_type);
        Mat T = MathUtils::getTFromTtransform(transform, cv_type);
        pair<Mat, Mat> p = make_pair(R, T);
        return p;
}

Mat MathUtils::inverseTransform(Mat* transform) {
        pair<Mat, Mat> couple = MathUtils::decomposeTransformMatrix(transform, CV_32FC1);
        Mat Rt = couple.first.t();
        Mat t1 = couple.second.t();
        t1 = -Rt*t1;
        return MathUtils::getTransformMatrix(&Rt, &t1, CV_32FC1);
}

Mat MathUtils::pos6Dof(Mat* transform) {
        Mat source(transform->clone());
        Mat pos = MathUtils::getTFromTtransform(&source, CV_32FC1);
        Mat rot = MathUtils::getRotFromTtransform(&source, CV_32FC1);
        Mat euler = MathUtils::rotToEulerZYZ(&rot);
        Mat pos6;
        hconcat(pos, euler, pos6);
        pos6.convertTo(pos6, CV_32FC1);
        return pos6;
}

Mat MathUtils::translateT(Mat* transform, float x, float y, float z) {

        Mat trans = (Mat_<float>(1, 4) << x, y, z, 1);
        trans.convertTo(trans, CV_32FC1);

        Mat t_trans = MathUtils::getTransformMatrix(NULL, &trans, CV_32FC1);
        Mat t_new(*transform);
        t_new = t_new * t_trans;
        return t_new;
}

Mat MathUtils::dumpMats(vector<Mat> mats) {

        vector<float> dump;
        int fullSize = 0;
        for (int i = 0; i < mats.size(); i++) {
                float *pt = mats[i].ptr<float>(0);
                int size = mats[i].rows * mats[i].cols;
                fullSize += size;
                vector<float> vt(pt, pt + size);
                dump.insert(dump.end(), vt.begin(), vt.end());
        }
        float *vdump = &dump[0];
        Mat mdump(1, fullSize, CV_32FC1);
        for (unsigned int j = 0; j < dump.size(); j++) {
                mdump.at<float>(0, j) = dump[j];
        }
        cout << mdump.size() << endl;
        return mdump;
}

tf::Transform MathUtils::matToTF(cv::Mat& mat){
        tf::Vector3 origin;
        tf::Matrix3x3 tf3d;
        origin.setValue(
                static_cast<float>(mat.at<float>(0,3))/1000.0f,
                static_cast<float>(mat.at<float>(1,3))/1000.0f,
                static_cast<float>(mat.at<float>(2,3))/1000.0f
                );

        tf3d.setValue(
                static_cast<float>(mat.at<float>(0,0)), static_cast<float>(mat.at<float>(0,1)), static_cast<float>(mat.at<float>(0,2)),
                static_cast<float>(mat.at<float>(1,0)), static_cast<float>(mat.at<float>(1,1)), static_cast<float>(mat.at<float>(1,2)),
                static_cast<float>(mat.at<float>(2,0)), static_cast<float>(mat.at<float>(2,1)), static_cast<float>(mat.at<float>(2,2))
                );

        for(int i = 0; i < 3; i++) {
                for(int j = 0; j < 3; j++) {
                        tf3d[i][j] = mat.at<float>(i,j);

                }
                std::cout<<std::endl;

        }


        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);
        tfqt = tfqt.normalized();

        tf::Transform transform;
        transform.setOrigin(origin);
        transform.setRotation(tfqt);
        return transform;
}

void MathUtils::getRotationsByName(std::string name, KDL::Rotation& rot_out){
        rot_out = KDL::Rotation::Identity();
        if(name=="front"){
          rot_out.DoRotY(M_PI/2.0f);
        }else if(name=="front_90"){
          rot_out.DoRotY(M_PI/2.0f);
          rot_out.DoRotZ(M_PI/2.0f);
        }else if(name=="down_y"){
          rot_out.DoRotY(M_PI);
        }else if(name=="down_x"){
          rot_out.DoRotX(M_PI);
        }
}


void MathUtils::poseToTF(geometry_msgs::Pose& pose, tf::Transform& transform, bool reverse, float meter_conversion_ratio ){
        if(!reverse) {
                transform.setOrigin( tf::Vector3(
                                             pose.position.x/ meter_conversion_ratio,
                                             pose.position.y/ meter_conversion_ratio,
                                             pose.position.z/ meter_conversion_ratio
                                             ));

                tf::Quaternion q(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w
                        );

                q.normalize();

                transform.setRotation(q);

        }else{
                pose.position.x = transform.getOrigin()[0]*meter_conversion_ratio;
                pose.position.y = transform.getOrigin()[1]*meter_conversion_ratio;
                pose.position.z = transform.getOrigin()[2]*meter_conversion_ratio;

                pose.orientation.x = transform.getRotation()[0];
                pose.orientation.y = transform.getRotation()[1];
                pose.orientation.z = transform.getRotation()[2];
                pose.orientation.w = transform.getRotation()[3];
        }
}

void MathUtils::poseToKDLRotation(geometry_msgs::Pose& pose, KDL::Rotation& rotation, bool reverse){
        if(!reverse) {
                tf::Quaternion q(
                        pose.orientation.x,
                        pose.orientation.y,
                        pose.orientation.z,
                        pose.orientation.w
                        );
                q.normalize();

                rotation = KDL::Rotation::Quaternion(
                        q[0],
                        q[1],
                        q[2],
                        q[3]
                        );

        }else{
                double qx,qy,qz,qw;
                rotation.GetQuaternion(qx,qy,qz,qw);
                pose.orientation.x = qx;
                pose.orientation.y = qy;
                pose.orientation.z = qz;
                pose.orientation.w = qw;
        }
}

}

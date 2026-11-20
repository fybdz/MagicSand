/***********************************************************************
KinectProjectorCalibration.cpp - KinectProjectorCalibration compute
the calibration of the kinect and projector.
Copyright (c) 2016 Thomas Wolf

--- Adapted from ofxKinectProjectorToolkit by Gene Kogan:
https://github.com/genekogan/ofxKinectProjectorToolkit
Copyright (c) 2014 Gene Kogan

This file is part of the Magic Sand.

The Magic Sand is free software; you can redistribute it
and/or modify it under the terms of the GNU General Public License as
published by the Free Software Foundation; either version 2 of the
License, or (at your option) any later version.

The Magic Sand is distributed in the hope that it will be
useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
General Public License for more details.

You should have received a copy of the GNU General Public License along
with the Magic Sand; if not, write to the Free Software
Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
***********************************************************************/

#include "KinectProjectorCalibration.h"
#include <cmath>
ofxKinectProjectorToolkit::ofxKinectProjectorToolkit(ofVec2f sprojRes, ofVec2f skinectRes) {
	projRes = sprojRes;
	kinectRes = skinectRes;
    calibrated = false;
}

void ofxKinectProjectorToolkit::calibrate(
    vector<ofVec3f> pairsKinect,
    vector<ofVec2f> pairsProjector)
{
    calibrated = false;

    const size_t n = pairsKinect.size();
    if (n != pairsProjector.size() || n < 6)
    {
        ofLogError("Calibration")
            << "Invalid calibration pairs: Kinect=" << n
            << ", projector=" << pairsProjector.size();
        return;
    }

    // 计算相机三维点和投影二维点的中心。
    double mx = 0.0, my = 0.0, mz = 0.0;
    double mu = 0.0, mv = 0.0;

    for (size_t i = 0; i < n; ++i)
    {
        const ofVec3f& k = pairsKinect[i];
        const ofVec2f& p = pairsProjector[i];

        if (!std::isfinite(k.x) || !std::isfinite(k.y) ||
            !std::isfinite(k.z) || !std::isfinite(p.x) ||
            !std::isfinite(p.y) || k.z <= 0.0f)
        {
            ofLogError("Calibration")
                << "Invalid point pair at index " << i;
            return;
        }

        mx += k.x;
        my += k.y;
        mz += k.z;
        mu += p.x;
        mv += p.y;
    }

    mx /= n;
    my /= n;
    mz /= n;
    mu /= n;
    mv /= n;

    // 计算平均距离，以统一两组坐标的数值量级。
    double meanWorldDist = 0.0;
    double meanProjectorDist = 0.0;

    for (size_t i = 0; i < n; ++i)
    {
        const ofVec3f& k = pairsKinect[i];
        const ofVec2f& p = pairsProjector[i];

        const double dx = k.x - mx;
        const double dy = k.y - my;
        const double dz = k.z - mz;
        meanWorldDist += std::sqrt(dx * dx + dy * dy + dz * dz);

        const double du = p.x - mu;
        const double dv = p.y - mv;
        meanProjectorDist += std::sqrt(du * du + dv * dv);
    }

    meanWorldDist /= n;
    meanProjectorDist /= n;

    if (meanWorldDist < 1e-9 || meanProjectorDist < 1e-9)
    {
        ofLogError("Calibration") << "Calibration points are degenerate.";
        return;
    }

    const double worldScale = std::sqrt(3.0) / meanWorldDist;
    const double projectorScale = std::sqrt(2.0) / meanProjectorDist;

    // 使用归一化坐标构造最小二乘方程。
    A.set_size(static_cast<long>(n * 2), 11);
    y.set_size(static_cast<long>(n * 2), 1);

    for (size_t i = 0; i < n; ++i)
    {
        const double X = (pairsKinect[i].x - mx) * worldScale;
        const double Y = (pairsKinect[i].y - my) * worldScale;
        const double Z = (pairsKinect[i].z - mz) * worldScale;
        const double u = (pairsProjector[i].x - mu) * projectorScale;
        const double v = (pairsProjector[i].y - mv) * projectorScale;

        const long r = static_cast<long>(2 * i);

        A(r, 0) = X;  A(r, 1) = Y;  A(r, 2) = Z;  A(r, 3) = 1.0;
        A(r, 4) = 0.0; A(r, 5) = 0.0; A(r, 6) = 0.0; A(r, 7) = 0.0;
        A(r, 8) = -X * u; A(r, 9) = -Y * u; A(r, 10) = -Z * u;
        y(r, 0) = u;

        A(r + 1, 0) = 0.0; A(r + 1, 1) = 0.0;
        A(r + 1, 2) = 0.0; A(r + 1, 3) = 0.0;
        A(r + 1, 4) = X; A(r + 1, 5) = Y;
        A(r + 1, 6) = Z; A(r + 1, 7) = 1.0;
        A(r + 1, 8) = -X * v; A(r + 1, 9) = -Y * v;
        A(r + 1, 10) = -Z * v;
        y(r + 1, 0) = v;
    }

    dlib::qr_decomposition<dlib::matrix<double, 0, 11> > qrd(A);
    x = qrd.solve(y);

    // 将归一化拟合结果整理成 3x4 矩阵。
    double pn[3][4] = {
        { x(0, 0), x(1, 0), x(2, 0), x(3, 0) },
        { x(4, 0), x(5, 0), x(6, 0), x(7, 0) },
        { x(8, 0), x(9, 0), x(10, 0), 1.0 }
    };

    // 世界坐标归一化变换：
    // Xn = worldScale * (X - center)
    double tw[4][4] = {
        { worldScale, 0.0,        0.0,        -worldScale * mx },
        { 0.0,        worldScale, 0.0,        -worldScale * my },
        { 0.0,        0.0,        worldScale, -worldScale * mz },
        { 0.0,        0.0,        0.0,         1.0 }
    };

    // 先计算 pn * tw。
    double temp[3][4] = {};
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            for (int k = 0; k < 4; ++k)
                temp[r][c] += pn[r][k] * tw[k][c];

    // 将投影坐标从归一化像素坐标还原到原始像素坐标。
    double p[3][4] = {};
    for (int c = 0; c < 4; ++c)
    {
        p[0][c] = temp[0][c] / projectorScale + mu * temp[2][c];
        p[1][c] = temp[1][c] / projectorScale + mv * temp[2][c];
        p[2][c] = temp[2][c];
    }

    // 原矩阵格式固定最后一个系数为 1，需要统一矩阵比例。
    const double divisor = p[2][3];
    if (!std::isfinite(divisor) || std::abs(divisor) < 1e-12)
    {
        ofLogError("Calibration")
            << "Cannot normalize projection matrix.";
        return;
    }

    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 4; ++c)
            p[r][c] /= divisor;

    // 保存反归一化后的系数；saveCalibration() 会读取 x。
    x(0, 0) = p[0][0]; x(1, 0) = p[0][1];
    x(2, 0) = p[0][2]; x(3, 0) = p[0][3];
    x(4, 0) = p[1][0]; x(5, 0) = p[1][1];
    x(6, 0) = p[1][2]; x(7, 0) = p[1][3];
    x(8, 0) = p[2][0]; x(9, 0) = p[2][1];
    x(10, 0) = p[2][2];

    for (int i = 0; i < 11; ++i)
    {
        if (!std::isfinite(x(i, 0)))
        {
            ofLogError("Calibration")
                << "Non-finite projection coefficient at " << i;
            return;
        }
    }

    projMatrice = ofMatrix4x4(
        x(0, 0), x(1, 0), x(2, 0), x(3, 0),
        x(4, 0), x(5, 0), x(6, 0), x(7, 0),
        x(8, 0), x(9, 0), x(10, 0), 1.0,
        0.0, 0.0, 0.0, 1.0
    );

    calibrated = true;
}
ofMatrix4x4 ofxKinectProjectorToolkit::getProjectionMatrix() {
    return projMatrice;
}

ofVec2f ofxKinectProjectorToolkit::getProjectedPoint(ofVec3f worldPoint) {
    ofVec4f pts = ofVec4f(worldPoint);
    pts.w = 1;
    ofVec4f rst = projMatrice*(pts);
    ofVec2f projectedPoint(rst.x/rst.z, rst.y/rst.z);
    return projectedPoint;
}

vector<double> ofxKinectProjectorToolkit::getCalibration()
{
    vector<double> coefficients;
    for (int i=0; i<11; i++) {
        coefficients.push_back(x(i, 0));
    }
    return coefficients;
}

bool ofxKinectProjectorToolkit::loadCalibration(string path){
    ofXml xml;
    if (!xml.load(path))
        return false;
	xml.setTo("RESOLUTIONS");
	ofVec2f sprojRes = xml.getValue<ofVec2f>("PROJECTOR");
	ofVec2f skinectRes = xml.getValue<ofVec2f>("KINECT");
	if (sprojRes!=projRes || skinectRes!=kinectRes)
		return false;
    xml.setTo("//CALIBRATION/COEFFICIENTS");
    for (int i=0; i<11; i++) {
        x(i, 0) = xml.getValue<float>("COEFF"+ofToString(i));
    }
    projMatrice = ofMatrix4x4(x(0,0), x(1,0), x(2,0), x(3,0),
                              x(4,0), x(5,0), x(6,0), x(7,0),
                              x(8,0), x(9,0), x(10,0), 1,
                              0, 0, 0, 0);
    calibrated = true;
    return true;
}

bool ofxKinectProjectorToolkit::saveCalibration(string path){
    ofXml xml;
	xml.addChild("CALIBRATION");
	xml.setTo("//CALIBRATION");
	xml.addChild("RESOLUTIONS");
	xml.setTo("RESOLUTIONS");
	xml.addValue("PROJECTOR", projRes);
	xml.addValue("KINECT", kinectRes);
	xml.setTo("//CALIBRATION");
	xml.addChild("COEFFICIENTS");
	xml.setTo("COEFFICIENTS");
	for (int i=0; i<11; i++) {
        ofXml coeff;
        coeff.addValue("COEFF"+ofToString(i), x(i, 0));
        xml.addXml(coeff);
    }
    xml.setToParent();
    return xml.save(path);
}



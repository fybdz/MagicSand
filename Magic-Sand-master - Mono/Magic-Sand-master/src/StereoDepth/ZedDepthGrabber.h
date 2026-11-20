#pragma once

#include "ofMain.h"
#include <sl/Camera.hpp>
#include <vector>

class ZedDepthGrabber
{
public:
    ZedDepthGrabber();
    ~ZedDepthGrabber();

    bool setup();
    void update();
    void close();

    bool isReady() const;
    bool isFrameNew() const;

    int getWidth() const;
    int getHeight() const;
    float getDepthMillimeters(int x, int y) const;

    const ofFloatPixels& getDepthPixels() const;
    const ofImage& getDepthPreview() const;

    const ofPixels& getLeftRgbPixels() const;
    ofVec3f getPoint3D(int x, int y) const;
    ofMatrix4x4 getImageToCameraMatrix() const;

private:
    sl::Camera zed;
    sl::RuntimeParameters runtimeParameters;
    sl::Mat depthMat;

    ofFloatPixels depthPixels;
    ofPixels previewPixels;
    ofImage depthPreview;

    bool ready;
    bool frameNew;
    int width;
    int height;

    sl::Mat leftImageMat;
    sl::Mat xyzMat;

    ofPixels leftRgbPixels;
    std::vector<ofVec3f> pointCloudPixels;

    void filterDepth();

    std::vector<float> filterInput;
    std::vector<float> previousDepth;
};

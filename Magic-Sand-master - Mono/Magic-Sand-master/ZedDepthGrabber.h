#pragma once

#include "ofMain.h"
#include <sl/Camera.hpp>

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
};
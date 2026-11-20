#pragma once

#include "ofMain.h"
#include "ZedDepthGrabber.h"

class ZedHeightMap
{
public:
    void captureReference(const ZedDepthGrabber& zed);
    void update(const ZedDepthGrabber& zed);

    bool hasReference() const;
    const ofImage& getPreview() const;

private:
    ofFloatPixels referenceDepthMm;
    ofPixels previewPixels;
    ofImage preview;

    bool referenceCaptured = false;
};
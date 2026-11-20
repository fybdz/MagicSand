#include "ZedHeightMap.h"

#include <cmath>
#include <algorithm>

namespace
{
    ofColor interpolateColor(
        const ofColor& a,
        const ofColor& b,
        float t)
    {
        t = std::max(0.0f, std::min(1.0f, t));

        return ofColor(
            static_cast<unsigned char>(a.r + (b.r - a.r) * t),
            static_cast<unsigned char>(a.g + (b.g - a.g) * t),
            static_cast<unsigned char>(a.b + (b.b - a.b) * t)
        );
    }
}
void ZedHeightMap::captureReference(const ZedDepthGrabber& zed)
{
    if (!zed.isReady())
    {
        return;
    }

    referenceDepthMm = zed.getDepthPixels();

    previewPixels.allocate(
        zed.getWidth(),
        zed.getHeight(),
        OF_PIXELS_RGB
    );

    preview.allocate(
        zed.getWidth(),
        zed.getHeight(),
        OF_IMAGE_COLOR
    );

    referenceCaptured = true;

    ofLogNotice("ZedHeightMap")
        << "Reference plane captured";
}

void ZedHeightMap::update(const ZedDepthGrabber& zed)
{
    if (!referenceCaptured || !zed.isReady())
    {
        return;
    }

    const float* reference = referenceDepthMm.getData();
    const float* current = zed.getDepthPixels().getData();
    unsigned char* output = previewPixels.getData();

    const int pixelCount = zed.getWidth() * zed.getHeight();

    for (int i = 0; i < pixelCount; ++i)
    {
        const float baseMm = reference[i];
        const float currentMm = current[i];

        ofColor color(0);

        if (std::isfinite(baseMm) &&
            std::isfinite(currentMm) &&
            baseMm > 0.0f &&
            currentMm > 0.0f)
        {
            const float heightMm = baseMm - currentMm;

            const ofColor blue(35, 100, 190);
            const ofColor green(45, 155, 85);
            const ofColor yellow(225, 205, 65);
            const ofColor brown(190, 120, 55);

            const float h = std::max(
                -50.0f,
                std::min(100.0f, heightMm)
            );

            if (h < 0.0f)
            {
                const float t = (h + 50.0f) / 50.0f;
                color = interpolateColor(blue, green, t);
            }
            else if (h < 30.0f)
            {
                const float t = h / 30.0f;
                color = interpolateColor(green, yellow, t);
            }
            else
            {
                const float t = (h - 30.0f) / 70.0f;
                color = interpolateColor(yellow, brown, t);
            }
        }

        output[i * 3 + 0] = color.r;
        output[i * 3 + 1] = color.g;
        output[i * 3 + 2] = color.b;
    }

    preview.setFromPixels(previewPixels);
}

bool ZedHeightMap::hasReference() const
{
    return referenceCaptured;
}

const ofImage& ZedHeightMap::getPreview() const
{
    return preview;
}
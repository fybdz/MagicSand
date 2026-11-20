#include "ZedDepthGrabber.h"

#include <cmath>

ZedDepthGrabber::ZedDepthGrabber()
    : ready(false)
    , frameNew(false)
    , width(0)
    , height(0)
{
}

ZedDepthGrabber::~ZedDepthGrabber()
{
    close();
}

bool ZedDepthGrabber::setup()
{
    sl::InitParameters initParameters;
    initParameters.camera_resolution = sl::RESOLUTION::HD720;
    initParameters.camera_fps = 30;
    initParameters.depth_mode = sl::DEPTH_MODE::PERFORMANCE;
    initParameters.coordinate_units = sl::UNIT::MILLIMETER;

    const sl::ERROR_CODE result = zed.open(initParameters);

    if (result != sl::ERROR_CODE::SUCCESS)
    {
        ofLogError("ZedDepthGrabber")
            << "无法打开 ZED: " << sl::toString(result).c_str();
        return false;
    }

    const sl::Resolution resolution =
        zed.getCameraInformation().camera_configuration.resolution;

    width = resolution.width;
    height = resolution.height;

    depthPixels.allocate(width, height, OF_PIXELS_GRAY);
    previewPixels.allocate(width, height, OF_PIXELS_GRAY);
    depthPreview.allocate(width, height, OF_IMAGE_GRAYSCALE);

    ready = true;

    ofLogNotice("ZedDepthGrabber")
        << "ZED 已打开: " << width << "x" << height;

    return true;
}

void ZedDepthGrabber::update()
{
    frameNew = false;

    if (!ready)
    {
        return;
    }

    if (zed.grab(runtimeParameters) != sl::ERROR_CODE::SUCCESS)
    {
        return;
    }

    zed.retrieveMeasure(depthMat, sl::MEASURE::DEPTH, sl::MEM::CPU);

    const float* source =
        depthMat.getPtr<sl::float1>(sl::MEM::CPU);

    if (source == nullptr)
    {
        return;
    }

    float* depth = depthPixels.getData();
    unsigned char* preview = previewPixels.getData();

    const int count = width * height;

    for (int i = 0; i < count; ++i)
    {
        const float depthMm = source[i];

        if (!std::isfinite(depthMm) || depthMm <= 0.0f)
        {
            depth[i] = 0.0f;
            preview[i] = 0;
            continue;
        }

        depth[i] = depthMm;

        // 近处亮，远处暗；仅用于调试显示。
        preview[i] = static_cast<unsigned char>(
            ofMap(depthMm, 300.0f, 2000.0f, 255.0f, 0.0f, true)
            );
    }

    depthPreview.setFromPixels(previewPixels);
    frameNew = true;
}

void ZedDepthGrabber::close()
{
    if (ready)
    {
        zed.close();
        ready = false;
    }
}

bool ZedDepthGrabber::isReady() const
{
    return ready;
}

bool ZedDepthGrabber::isFrameNew() const
{
    return frameNew;
}

int ZedDepthGrabber::getWidth() const
{
    return width;
}

int ZedDepthGrabber::getHeight() const
{
    return height;
}

float ZedDepthGrabber::getDepthMillimeters(int x, int y) const
{
    if (!ready || x < 0 || y < 0 || x >= width || y >= height)
    {
        return 0.0f;
    }

    return depthPixels.getData()[y * width + x];
}

const ofFloatPixels& ZedDepthGrabber::getDepthPixels() const
{
    return depthPixels;
}

const ofImage& ZedDepthGrabber::getDepthPreview() const
{
    return depthPreview;
}
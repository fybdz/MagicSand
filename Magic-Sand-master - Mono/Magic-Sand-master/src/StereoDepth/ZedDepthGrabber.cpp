#include "ZedDepthGrabber.h"
#include <algorithm>
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
    initParameters.coordinate_system = sl::COORDINATE_SYSTEM::IMAGE;
    runtimeParameters.confidence_threshold = 95;
    runtimeParameters.texture_confidence_threshold = 100;

    const sl::ERROR_CODE result = zed.open(initParameters);

    if (result != sl::ERROR_CODE::SUCCESS)
    {
        ofLogError("ZedDepthGrabber")
            << "�޷��� ZED: " << sl::toString(result).c_str();
        return false;
    }

    const sl::Resolution resolution =
        zed.getCameraInformation().camera_configuration.resolution;

    width = resolution.width;
    height = resolution.height;

    depthPixels.allocate(width, height, OF_PIXELS_GRAY);
    depthPixels.set(0.0f);
    filterInput.assign(width * height, 0.0f);
    previousDepth.assign(width * height, 0.0f);
    previewPixels.allocate(width, height, OF_PIXELS_GRAY);
    depthPreview.allocate(width, height, OF_IMAGE_GRAYSCALE);

    leftRgbPixels.allocate(width, height, OF_PIXELS_RGB);
    pointCloudPixels.resize(width * height);

    ready = true;

    ofLogNotice("ZedDepthGrabber")
        << "ZED �Ѵ�: " << width << "x" << height;

    return true;
}

void ZedDepthGrabber::update()
{
    frameNew = false;

    if (!ready)
    {
        return;
    }
    
    const sl::ERROR_CODE grabResult =
        zed.grab(runtimeParameters);

    if (grabResult != sl::ERROR_CODE::SUCCESS)
    {
        ofLogError("ZedDepthGrabber")
            << "grab failed: "
            << sl::toString(grabResult).c_str();
        return;
    }
    
    const sl::ERROR_CODE imageResult =
        zed.retrieveImage(leftImageMat, sl::VIEW::LEFT, sl::MEM::CPU);

    if (imageResult != sl::ERROR_CODE::SUCCESS)
    {
        ofLogError("ZedDepthGrabber")
            << "retrieve left image failed: "
            << sl::toString(imageResult).c_str();
        return;
    }

    const unsigned char* bgra =
        leftImageMat.getPtr<unsigned char>(sl::MEM::CPU);
    const size_t imageStep =
        leftImageMat.getStepBytes(sl::MEM::CPU);

    if (bgra == nullptr ||
        leftImageMat.getWidth() != width ||
        leftImageMat.getHeight() != height)
    {
        ofLogError("ZedDepthGrabber") << "invalid left image buffer/size";
        return;
    }

    unsigned char* rgb = leftRgbPixels.getData();

    for (int y = 0; y < height; ++y)
    {
        const unsigned char* src =
            bgra + static_cast<size_t>(y) * imageStep;
        unsigned char* dst =
            rgb + static_cast<size_t>(y) * width * 3;

        for (int x = 0; x < width; ++x)
        {
            dst[x * 3 + 0] = src[x * 4 + 2]; // R
            dst[x * 3 + 1] = src[x * 4 + 1]; // G
            dst[x * 3 + 2] = src[x * 4 + 0]; // B
        }
    }

    const sl::ERROR_CODE xyzResult =
        zed.retrieveMeasure(xyzMat, sl::MEASURE::XYZ, sl::MEM::CPU);

    if (xyzResult != sl::ERROR_CODE::SUCCESS)
    {
        ofLogError("ZedDepthGrabber")
            << "retrieve XYZ failed: "
            << sl::toString(xyzResult).c_str();
        return;
    }

    const sl::float4* xyz =
        xyzMat.getPtr<sl::float4>(sl::MEM::CPU);
    const size_t xyzStep =
        xyzMat.getStep<sl::float4>(sl::MEM::CPU);

    if (xyz == nullptr ||
        xyzMat.getWidth() != width ||
        xyzMat.getHeight() != height)
    {
        ofLogError("ZedDepthGrabber") << "invalid XYZ buffer/size";
        return;
    }

    for (int y = 0; y < height; ++y)
    {
        const sl::float4* row =
            xyz + static_cast<size_t>(y) * xyzStep;

        for (int x = 0; x < width; ++x)
        {
            const sl::float4& p = row[x];
            const int i = y * width + x;

            if (std::isfinite(p.x) &&
                std::isfinite(p.y) &&
                std::isfinite(p.z) &&
                p.z > 0.0f)
            {
                pointCloudPixels[i].set(p.x, p.y, p.z);
            }
            else
            {
                pointCloudPixels[i].set(0, 0, 0);
            }
        }
    }
    

    const sl::ERROR_CODE measureResult =
        zed.retrieveMeasure(depthMat, sl::MEASURE::DEPTH, sl::MEM::CPU);
    if (measureResult != sl::ERROR_CODE::SUCCESS)
    {
        ofLogWarning("ZedDepthGrabber")
            << "retrieveMeasure(DEPTH) failed: "
            << sl::toString(measureResult).c_str();
        return;
    }

    const sl::float1* source =
        depthMat.getPtr<sl::float1>(sl::MEM::CPU);

    if (source == nullptr)
    {
        return;
    }

    const size_t step = depthMat.getStep<sl::float1>(sl::MEM::CPU);
    float* depth = depthPixels.getData();
    unsigned char* preview = previewPixels.getData();
    int validCount = 0;
    float centerRawDepth = 0.0f;

    for (int y = 0; y < height; ++y)
    {
        const sl::float1* row = source + static_cast<size_t>(y) * step;
        for (int x = 0; x < width; ++x)
        {
            const int i = y * width + x;
            const float depthMm = row[x];

            if (!std::isfinite(depthMm) || depthMm <= 0.0f)
            {
                depth[i] = 0.0f;
                preview[i] = 0;
                continue;
            }

            ++validCount;

            if (x == width / 2 && y == height / 2)
            {
                centerRawDepth = depthMm;
            }
            depth[i] = depthMm;

            preview[i] = static_cast<unsigned char>(
                ofMap(depthMm, 300.0f, 2000.0f, 255.0f, 0.0f, true)
                );
        }
    }
    if (ofGetFrameNum() % 60 == 0)
    {
        ofLogNotice("ZedDepthGrabber")
            << "valid=" << validCount
            << "/" << width * height
            << " centerRaw=" << centerRawDepth
            << " step=" << step
            << " image=" << depthMat.getWidth()
            << "x" << depthMat.getHeight();
    }
    filterDepth();

    float* filteredDepth = depthPixels.getData();

    for (int i = 0; i < width * height; ++i)
    {
        ofVec3f& point = pointCloudPixels[i];
        const float rawZ = point.z;
        const float smoothZ = filteredDepth[i];

        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(rawZ) ||
            rawZ <= 0.0f ||
            !std::isfinite(smoothZ) ||
            smoothZ <= 0.0f)
        {
            point.set(0, 0, 0);
            continue;
        }

        // 同一像素的 XYZ 沿相机射线缩放到滤波后的深度。
        const float scale = smoothZ / rawZ;
        point.x *= scale;
        point.y *= scale;
        point.z = smoothZ;
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

const ofPixels& ZedDepthGrabber::getLeftRgbPixels() const
{
    return leftRgbPixels;
}

ofVec3f ZedDepthGrabber::getPoint3D(int x, int y) const
{
    if (!ready || x < 0 || y < 0 || x >= width || y >= height)
    {
        return ofVec3f(0, 0, 0);
    }

    return pointCloudPixels[y * width + x];
}

ofMatrix4x4 ZedDepthGrabber::getImageToCameraMatrix() const
{
    const auto cameraInfo = zed.getCameraInformation();
    const auto& cam =
        cameraInfo.camera_configuration
        .calibration_parameters
        .left_cam;

    const float fx = cam.fx;
    const float fy = cam.fy;
    const float cx = cam.cx;
    const float cy = cam.cy;

    return ofMatrix4x4(
        1.0f / fx, 0, 0, -cx / fx,
        0, 1.0f / fy, 0, -cy / fy,
        0, 0, 0, 1,
        0, 0, 0, 1
    );
}

void ZedDepthGrabber::filterDepth()
{
    float* output = depthPixels.getData();
    unsigned char* preview = previewPixels.getData();
    const int count = width * height;

    // 保存本帧原始深度，避免空间滤波读到已修改的数据。
    std::copy(output, output + count, filterInput.begin());

    // 初始参数，单位为毫米。
    const float spatialLimitMm = 20.0f;
    const float temporalLimitMm = 30.0f;
    const float alpha = 0.20f;

    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x)
        {
            const int i = y * width + x;
            const float current = filterInput[i];

            // 无效点不填成假地形，同时清除旧历史。
            if (!std::isfinite(current) || current <= 0.0f)
            {
                output[i] = 0.0f;
                previousDepth[i] = 0.0f;
                preview[i] = 0;
                continue;
            }

            // 3×3 空间平滑，只接受与中心深度接近的邻居。
            float sum = 0.0f;
            int samples = 0;

            for (int dy = -1; dy <= 1; ++dy)
            {
                const int yy = y + dy;
                if (yy < 0 || yy >= height)
                    continue;

                for (int dx = -1; dx <= 1; ++dx)
                {
                    const int xx = x + dx;
                    if (xx < 0 || xx >= width)
                        continue;

                    const float d = filterInput[yy * width + xx];

                    if (std::isfinite(d) &&
                        d > 0.0f &&
                        std::fabs(d - current) <= spatialLimitMm)
                    {
                        sum += d;
                        ++samples;
                    }
                }
            }

            float filtered = samples > 0 ? sum / samples : current;
            const float previous = previousDepth[i];

            // 小变化做时间平滑；大变化立即跟随，减少拖影。
            if (std::isfinite(previous) &&
                previous > 0.0f &&
                std::fabs(filtered - previous) <= temporalLimitMm)
            {
                filtered =
                    previous + alpha * (filtered - previous);
            }

            output[i] = filtered;
            previousDepth[i] = filtered;

            preview[i] = static_cast<unsigned char>(
                ofMap(filtered, 300.0f, 2000.0f,
                    255.0f, 0.0f, true)
                );
        }
    }
}
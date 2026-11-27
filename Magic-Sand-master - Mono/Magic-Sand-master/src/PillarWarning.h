#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "StereoDepth/ZedDepthGrabber.h"
#include "KinectProjector/KinectProjector.h"

#include <algorithm>
#include <cmath>
#include <vector>

     class PillarWarning
 {
 public:
     // 白色区域识别参数。
     int whiteMin = 145;
     int maxColorDifference = 85;
     int localContrast = 10;

     double minArea = 2.0;
     double maxArea = 120.0;

     // 自动扫描时间，单位：秒。
     float scanSeconds = 3.0f;

     // 柱子露出沙面的高度，单位：毫米。
     float exposedHeightMm = 35.0f;

     // 光环半径和宽度，单位：毫米。
     float ringRadiusMm = 22.0f;
     float ringWidthMm = 5.0f;

     // 倒伏目标搜索半径，单位：相机图像像素。
     float searchRadiusPx = 24.0f;

     // 倒伏形状判断参数。
     float fallenAspect = 2.2f;
     float fallenLengthScale = 1.6f;
     float minimumFallenLengthPx = 6.0f;

     // 大范围图像变化后，等待画面稳定。
     float quietSeconds = 0.8f;

     // 倒伏证据持续时间，单位：秒。
     float confirmSeconds = 1.0f;

     void restart()
     {
         pillars.clear();

         scanning = true;
         scanStarted = -1.0f;
         scanFrames = 0;

         previousGray.release();

         ofLogNotice("PillarWarning")
             << "Scanning requested. "
             << "Keep pillars upright and hands away.";
     }

     void update(
         const ZedDepthGrabber& zed,
         KinectProjector& kp)
     {
         if (!zed.isReady() || !zed.isFrameNew())
         {
             return;
         }

         const ofPixels& pixels = zed.getLeftRgbPixels();

         if (!pixels.isAllocated())
         {
             return;
         }

         const float now = ofGetElapsedTimef();

         cv::Mat rgb(
             pixels.getHeight(),
             pixels.getWidth(),
             CV_8UC3,
             const_cast<unsigned char*>(pixels.getData())
         );

         cv::Mat gray;
         cv::cvtColor(rgb, gray, CV_RGB2GRAY);

         const ofRectangle roi = kp.getKinectROI();

         const std::vector<Blob> blobs =
             detect(rgb, gray, roi);

         if (scanning)
         {
             updateScan(blobs, zed, now);
         }
         else
         {
             updateWarnings(blobs, gray, now);
         }

         previousGray = gray.clone();

         // 显示实际用于识别的左目彩色图像。
         debugImage.setFromPixels(pixels);
     }

     void drawProjector(KinectProjector& kp)
     {
         if (scanning)
         {
             return;
         }

         ofPushStyle();

         ofFill();
         ofEnableAlphaBlending();

         for (const Pillar& p : pillars)
         {
             if (!p.baseValid)
             {
                 continue;
             }

             if (p.alarm)
             {
                 ofSetColor(255, 35, 25);
             }
             else
             {
                 ofSetColor(30, 255, 70);
             }

             ofMesh mesh;
             mesh.setMode(OF_PRIMITIVE_TRIANGLE_STRIP);

             bool valid = true;

             const float inner = std::max(
                 1.0f,
                 ringRadiusMm - ringWidthMm * 0.5f
             );

             const float outer =
                 ringRadiusMm + ringWidthMm * 0.5f;

             for (int i = 0; i <= 64; ++i)
             {
                 const float angle =
                     TWO_PI * static_cast<float>(i) / 64.0f;

                 for (int edge = 0; edge < 2; ++edge)
                 {
                     const float radius =
                         edge == 0 ? outer : inner;

                     const float x =
                         p.base.x + std::cos(angle) * radius;

                     const float y =
                         p.base.y + std::sin(angle) * radius;

                     const float z =
                         p.planeA * x +
                         p.planeB * y +
                         p.planeC;

                     const ofVec2f projected =
                         kp.worldCoordToProjCoord(
                             ofVec3f(x, y, z)
                         );

                     if (!std::isfinite(projected.x) ||
                         !std::isfinite(projected.y) ||
                         std::fabs(projected.x) > 100000.0f ||
                         std::fabs(projected.y) > 100000.0f)
                     {
                         valid = false;
                         break;
                     }

                     mesh.addVertex(
                         ofVec3f(
                             projected.x,
                             projected.y,
                             0.0f
                         )
                     );
                 }

                 if (!valid)
                 {
                     break;
                 }
             }

             if (valid)
             {
                 mesh.draw();
             }
         }

         ofPopStyle();
     }

     void drawDebug(
         float x,
         float y,
         float width,
         float height)
     {
         if (!debugImage.isAllocated())
         {
             return;
         }

         ofPushStyle();

         ofSetColor(255);
         debugImage.draw(x, y, width, height);

         const float scaleX =
             width / debugImage.getWidth();

         const float scaleY =
             height / debugImage.getHeight();

         for (size_t i = 0; i < pillars.size(); ++i)
         {
             const Pillar& p = pillars[i];

             const float px =
                 x + p.image.x * scaleX;

             const float py =
                 y + p.image.y * scaleY;

             if (p.alarm)
             {
                 ofSetColor(255, 30, 20);
             }
             else
             {
                 ofSetColor(30, 255, 70);
             }

             ofNoFill();

             ofDrawRectangle(
                 px - 6.0f,
                 py - 6.0f,
                 12.0f,
                 12.0f
             );

             const std::string label =
                 ofToString(i + 1) +
                 (p.alarm ? " ALARM" : "");

             ofDrawBitmapString(
                 label,
                 px + 8.0f,
                 py
             );
         }

         ofSetColor(255);

         std::string status;

         if (scanning)
         {
             status = "PILLARS: scanning - hands away";
         }
         else
         {
             status =
                 "PILLARS: " +
                 ofToString(pillars.size()) +
                 " | F6 rescan/reset";
         }

         ofDrawBitmapString(
             status,
             x,
             y + height + 18.0f
         );

         ofPopStyle();
     }

 private:
     struct Blob
     {
         ofVec2f center;

         float length = 0.0f;
         float aspect = 0.0f;
     };

     struct Pillar
     {
         ofVec2f image;

         float uprightLength = 0.0f;
         int hits = 0;

         bool alarm = false;

         float lastMotion = 0.0f;
         float fallenSince = -1.0f;

         bool baseValid = false;
         ofVec3f base;

         // 局部沙面近似：
         // z = planeA*x + planeB*y + planeC。
         float planeA = 0.0f;
         float planeB = 0.0f;
         float planeC = 0.0f;
     };

     std::vector<Pillar> pillars;

     bool scanning = true;

     float scanStarted = -1.0f;
     int scanFrames = 0;

     cv::Mat previousGray;
     ofImage debugImage;

     void updateScan(
         const std::vector<Blob>& blobs,
         const ZedDepthGrabber& zed,
         float now)
     {
         if (scanStarted < 0.0f)
         {
             scanStarted = now;
         }

         ++scanFrames;

         std::vector<bool> matched(
             pillars.size(),
             false
         );

         for (const Blob& b : blobs)
         {
             // 扫描阶段只接受较小、相对紧凑的目标。
             if (b.length > 13.0f ||
                 b.aspect > 4.5f)
             {
                 continue;
             }

             int best = -1;
             float distance = 7.0f;

             for (size_t i = 0; i < pillars.size(); ++i)
             {
                 if (matched[i])
                 {
                     continue;
                 }

                 const float d =
                     pillars[i].image.distance(b.center);

                 if (d < distance)
                 {
                     best = static_cast<int>(i);
                     distance = d;
                 }
             }

             if (best < 0)
             {
                 Pillar p;

                 p.image = b.center;
                 p.uprightLength = b.length;
                 p.hits = 1;

                 pillars.push_back(p);
                 matched.push_back(true);
             }
             else
             {
                 Pillar& p = pillars[best];

                 p.image =
                     p.image * 0.8f +
                     b.center * 0.2f;

                 p.uprightLength =
                     p.uprightLength * 0.8f +
                     b.length * 0.2f;

                 ++p.hits;
                 matched[best] = true;
             }
         }

         if (now - scanStarted < scanSeconds)
         {
             return;
         }

         std::vector<Pillar> accepted;

         for (Pillar p : pillars)
         {
             // 至少在 70% 的扫描帧中出现。
             if (p.hits < scanFrames * 0.70f)
             {
                 continue;
             }

             if (!estimateBase(zed, p))
             {
                 ofLogNotice("PillarWarning")
                     << "Skipped candidate: invalid local surface at "
                     << p.image.x << ", " << p.image.y;

                 continue;
             }

             p.lastMotion = now;

             ofLogNotice("PillarWarning")
                 << "ID=" << accepted.size() + 1
                 << " pixel=(" << p.image.x << ", " << p.image.y << ")"
                 << " length=" << p.uprightLength
                 << " hits=" << p.hits
                 << "/" << scanFrames;

             accepted.push_back(p);
         }

         pillars.swap(accepted);
         scanning = false;

         ofLogNotice("PillarWarning")
             << "Scan completed. Pillars: "
             << pillars.size();
     }

     void updateWarnings(
         const std::vector<Blob>& blobs,
         const cv::Mat& gray,
         float now)
     {
         for (Pillar& p : pillars)
         {
             // 已报警的柱子保持锁存，不自动恢复。
             if (p.alarm)
             {
                 continue;
             }

             const bool moving =
                 hasLargeChange(
                     gray,
                     previousGray,
                     p.image
                 );

             if (moving)
             {
                 p.lastMotion = now;
                 p.fallenSince = -1.0f;
                 continue;
             }

             if (now - p.lastMotion < quietSeconds)
             {
                 p.fallenSince = -1.0f;
                 continue;
             }

             const Blob* nearest = nullptr;
             float nearestDistance = searchRadiusPx;

             for (const Blob& b : blobs)
             {
                 const float d =
                     p.image.distance(b.center);

                 if (d < nearestDistance)
                 {
                     nearestDistance = d;
                     nearest = &b;
                 }
             }

             bool fallen = false;

             if (nearest != nullptr)
             {
                 const float requiredLength =
                     std::max(
                         minimumFallenLengthPx,
                         p.uprightLength *
                         fallenLengthScale
                     );

                 fallen =
                     nearest->aspect >= fallenAspect &&
                     nearest->length >= requiredLength;
             }

             // 没找到白柱不能直接判定倒伏。
             if (!fallen)
             {
                 p.fallenSince = -1.0f;
                 continue;
             }

             if (p.fallenSince < 0.0f)
             {
                 p.fallenSince = now;
             }

             if (now - p.fallenSince >= confirmSeconds)
             {
                 p.alarm = true;

                 ofLogNotice("PillarWarning")
                     << "Alarm latched at camera pixel "
                     << p.image.x << ", " << p.image.y;
             }
         }
     }

     std::vector<Blob> detect(
         const cv::Mat& rgb,
         const cv::Mat& gray,
         const ofRectangle& roi)
     {
         cv::Mat background;

         cv::GaussianBlur(
             gray,
             background,
             cv::Size(15, 15),
             0
         );

         cv::Mat mask(
             gray.rows,
             gray.cols,
             CV_8UC1,
             cv::Scalar(0)
         );

         const int left = std::max(
             0,
             static_cast<int>(roi.getLeft())
         );

         const int right = std::min(
             rgb.cols,
             static_cast<int>(roi.getRight())
         );

         const int top = std::max(
             0,
             static_cast<int>(roi.getTop())
         );

         const int bottom = std::min(
             rgb.rows,
             static_cast<int>(roi.getBottom())
         );

         for (int y = top; y < bottom; ++y)
         {
             const cv::Vec3b* row =
                 rgb.ptr<cv::Vec3b>(y);

             const unsigned char* grayRow =
                 gray.ptr<unsigned char>(y);

             const unsigned char* backgroundRow =
                 background.ptr<unsigned char>(y);

             unsigned char* destination =
                 mask.ptr<unsigned char>(y);

             for (int x = left; x < right; ++x)
             {
                 // getLeftRgbPixels() 返回 RGB 顺序。
                 const int red = row[x][0];
                 const int green = row[x][1];
                 const int blue = row[x][2];

                 const int lowest = std::min(
                     red,
                     std::min(green, blue)
                 );

                 const int highest = std::max(
                     red,
                     std::max(green, blue)
                 );

                 const int contrast =
                     static_cast<int>(grayRow[x]) -
                     static_cast<int>(backgroundRow[x]);

                 if (lowest >= whiteMin &&
                     highest - lowest <=
                     maxColorDifference &&
                     contrast >= localContrast)
                 {
                     destination[x] = 255;
                 }
             }
         }

         std::vector<std::vector<cv::Point> > contours;

         cv::findContours(
             mask,
             contours,
             CV_RETR_EXTERNAL,
             CV_CHAIN_APPROX_SIMPLE
         );

         std::vector<Blob> result;

         for (const auto& contour : contours)
         {
             const cv::Rect bounds = cv::boundingRect(contour);

             const cv::Mat objectMask = mask(bounds);

             const int pixelCount = cv::countNonZero(objectMask);

             // Use pixel count because thin contours may have zero area.
             if (pixelCount < 3 || pixelCount > maxArea)
             {
                 continue;
             }

             cv::Mat contrastImage;

             cv::subtract(
                 gray(bounds),
                 background(bounds),
                 contrastImage
             );

             double peakContrast = 0.0;

             cv::minMaxLoc(
                 contrastImage,
                 nullptr,
                 &peakContrast,
                 nullptr,
                 nullptr,
                 objectMask
             );

             if (peakContrast < 35.0)
             {
                 continue;
             }

             const cv::RotatedRect rectangle =
                 cv::minAreaRect(contour);

             const float length = std::max(
                 rectangle.size.width,
                 rectangle.size.height
             );

             const float thickness = std::max(
                 1.0f,
                 std::min(
                     rectangle.size.width,
                     rectangle.size.height
                 )
             );

             Blob b;

             b.center.set(
                 rectangle.center.x,
                 rectangle.center.y
             );

             b.length = length;
             b.aspect = length / thickness;

             result.push_back(b);
         }

         return result;
     }

     bool hasLargeChange(
         const cv::Mat& gray,
         const cv::Mat& previous,
         const ofVec2f& center)
     {
         if (previous.empty() ||
             previous.size() != gray.size())
         {
             return true;
         }

         const int radius = 28;

         const int x0 = std::max(
             0,
             static_cast<int>(center.x) - radius
         );

         const int y0 = std::max(
             0,
             static_cast<int>(center.y) - radius
         );

         const int x1 = std::min(
             gray.cols,
             static_cast<int>(center.x) + radius + 1
         );

         const int y1 = std::min(
             gray.rows,
             static_cast<int>(center.y) + radius + 1
         );

         if (x1 <= x0 || y1 <= y0)
         {
             return true;
         }

         const cv::Rect rectangle(
             x0,
             y0,
             x1 - x0,
             y1 - y0
         );

         cv::Mat difference;

         cv::absdiff(
             gray(rectangle),
             previous(rectangle),
             difference
         );

         cv::threshold(
             difference,
             difference,
             25,
             255,
             CV_THRESH_BINARY
         );

         const double changedRatio =
             static_cast<double>(
                 cv::countNonZero(difference)
                 ) /
             static_cast<double>(rectangle.area());

         // 这是简单的变化检测，并不是真正的手部识别。
         return changedRatio > 0.12;
     }

     bool estimateBase(
         const ZedDepthGrabber& zed,
         Pillar& p)
     {
         std::vector<ofVec3f> points;
         std::vector<float> depths;

         const int centerX =
             static_cast<int>(p.image.x);

         const int centerY =
             static_cast<int>(p.image.y);

         // 避开白柱本身，在外围采样局部沙面。
         for (int dy = -24; dy <= 24; dy += 3)
         {
             for (int dx = -24; dx <= 24; dx += 3)
             {
                 const int radiusSquared =
                     dx * dx + dy * dy;

                 if (radiusSquared < 12 * 12 ||
                     radiusSquared > 24 * 24)
                 {
                     continue;
                 }

                 const ofVec3f point =
                     zed.getPoint3D(
                         centerX + dx,
                         centerY + dy
                     );

                 if (!std::isfinite(point.x) ||
                     !std::isfinite(point.y) ||
                     !std::isfinite(point.z) ||
                     point.z <= 0.0f)
                 {
                     continue;
                 }

                 points.push_back(point);
                 depths.push_back(point.z);
             }
         }

         if (points.size() < 20)
         {
             return false;
         }

         std::sort(depths.begin(), depths.end());

         const float medianDepth =
             depths[depths.size() / 2];

         std::vector<ofVec3f> surface;

         for (const ofVec3f& point : points)
         {
             if (std::fabs(point.z - medianDepth) <= 15.0f)
             {
                 surface.push_back(point);
             }
         }

         if (surface.size() < 15)
         {
             return false;
         }

         const int count =
             static_cast<int>(surface.size());

         cv::Mat coefficients(count, 3, CV_64F);
         cv::Mat depthsMatrix(count, 1, CV_64F);

         for (int i = 0; i < count; ++i)
         {
             coefficients.at<double>(i, 0) =
                 surface[i].x;

             coefficients.at<double>(i, 1) =
                 surface[i].y;

             coefficients.at<double>(i, 2) = 1.0;

             depthsMatrix.at<double>(i, 0) =
                 surface[i].z;
         }

         cv::Mat solution;

         if (!cv::solve(
             coefficients,
             depthsMatrix,
             solution,
             cv::DECOMP_SVD))
         {
             return false;
         }

         p.planeA = static_cast<float>(
             solution.at<double>(0, 0)
             );

         p.planeB = static_cast<float>(
             solution.at<double>(1, 0)
             );

         p.planeC = static_cast<float>(
             solution.at<double>(2, 0)
             );

         if (!std::isfinite(p.planeA) ||
             !std::isfinite(p.planeB) ||
             !std::isfinite(p.planeC))
         {
             return false;
         }

         // 从白柱图像中心构造相机射线。
         const ofMatrix4x4 matrix =
             zed.getImageToCameraMatrix();

         const ofVec4f ray =
             matrix * ofVec4f(
                 p.image.x,
                 p.image.y,
                 0.0f,
                 1.0f
             );

         const float denominator =
             1.0f -
             p.planeA * ray.x -
             p.planeB * ray.y;

         if (!std::isfinite(denominator) ||
             std::fabs(denominator) < 0.1f)
         {
             return false;
         }

         // 第一版近似：
         // 柱子沿相机 Z 方向直立，相机近似俯视沙面。
         // 相机倾斜明显时需要进一步修正。
         const float topZ =
             (p.planeC - exposedHeightMm) /
             denominator;

         if (!std::isfinite(topZ) || topZ <= 0.0f)
         {
             return false;
         }

         p.base.x = ray.x * topZ;
         p.base.y = ray.y * topZ;

         p.base.z =
             p.planeA * p.base.x +
             p.planeB * p.base.y +
             p.planeC;

         p.baseValid =
             std::isfinite(p.base.x) &&
             std::isfinite(p.base.y) &&
             std::isfinite(p.base.z) &&
             p.base.z > 0.0f;

         return p.baseValid;
     }
 };

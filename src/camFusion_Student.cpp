
#include <iostream>
#include <algorithm>
#include <numeric>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "camFusion.hpp"
#include "dataStructures.h"


using namespace std;


// Create groups of Lidar points whose projection into the camera falls into the same bounding box
void clusterLidarWithROI(std::vector<BoundingBox> &boundingBoxes, std::vector<LidarPoint> &lidarPoints, float shrinkFactor, cv::Mat &P_rect_xx, cv::Mat &R_rect_xx, cv::Mat &RT)
{
    // loop over all Lidar points and associate them to a 2D bounding box
    cv::Mat X(4, 1, cv::DataType<double>::type);
    cv::Mat Y(3, 1, cv::DataType<double>::type);

    for (auto it1 = lidarPoints.begin(); it1 != lidarPoints.end(); ++it1)
    {
        // assemble vector for matrix-vector-multiplication
        X.at<double>(0, 0) = it1->x;
        X.at<double>(1, 0) = it1->y;
        X.at<double>(2, 0) = it1->z;
        X.at<double>(3, 0) = 1;

        // project Lidar point into camera
        Y = P_rect_xx * R_rect_xx * RT * X;
        cv::Point pt;
        // pixel coordinates
        pt.x = Y.at<double>(0, 0) / Y.at<double>(2, 0);
        pt.y = Y.at<double>(1, 0) / Y.at<double>(2, 0);

        vector<vector<BoundingBox>::iterator> enclosingBoxes; // pointers to all bounding boxes which enclose the current Lidar point
        for (vector<BoundingBox>::iterator it2 = boundingBoxes.begin(); it2 != boundingBoxes.end(); ++it2)
        {
            // shrink current bounding box slightly to avoid having too many outlier points around the edges
            cv::Rect smallerBox;
            smallerBox.x = (*it2).roi.x + shrinkFactor * (*it2).roi.width / 2.0;
            smallerBox.y = (*it2).roi.y + shrinkFactor * (*it2).roi.height / 2.0;
            smallerBox.width = (*it2).roi.width * (1 - shrinkFactor);
            smallerBox.height = (*it2).roi.height * (1 - shrinkFactor);

            // check wether point is within current bounding box
            if (smallerBox.contains(pt))
            {
                enclosingBoxes.push_back(it2);
            }
        }
        // check wether point has been enclosed by one or by multiple boxes
        if (enclosingBoxes.size() == 1)
        {
            // add Lidar point to bounding box
            enclosingBoxes[0]->lidarPoints.push_back(*it1);
        }
    }
}

/*
* The show3DObjects() function below can handle different output image sizes, but the text output has been manually tuned to fit the 2000x2000 size.
* However, you can make this function work for other sizes too.
* For instance, to use a 1000x1000 size, adjusting the text positions by dividing them by 2.
*/
void show3DObjects(std::vector<BoundingBox> &boundingBoxes, cv::Size worldSize, cv::Size imageSize, bool bWait)
{
    // create topview image
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));

    for(auto it1=boundingBoxes.begin(); it1!=boundingBoxes.end(); ++it1)
    {
        // create randomized color for current 3D object
        cv::RNG rng(it1->boxID);
        cv::Scalar currColor = cv::Scalar(rng.uniform(0,150), rng.uniform(0, 150), rng.uniform(0, 150));

        // plot Lidar points into top view image
        int top=1e8, left=1e8, bottom=0.0, right=0.0;
        float xwmin=1e8, ywmin=1e8, ywmax=-1e8;
        for (auto it2 = it1->lidarPoints.begin(); it2 != it1->lidarPoints.end(); ++it2)
        {
            // world coordinates
            float xw = (*it2).x; // world position in m with x facing forward from sensor
            float yw = (*it2).y; // world position in m with y facing left from sensor
            xwmin = xwmin<xw ? xwmin : xw;
            ywmin = ywmin<yw ? ywmin : yw;
            ywmax = ywmax>yw ? ywmax : yw;

            // top-view coordinates
            int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
            int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

            // find enclosing rectangle
            top = top<y ? top : y;
            left = left<x ? left : x;
            bottom = bottom>y ? bottom : y;
            right = right>x ? right : x;

            // draw individual point
            cv::circle(topviewImg, cv::Point(x, y), 4, currColor, -1);
        }

        // draw enclosing rectangle
        cv::rectangle(topviewImg, cv::Point(left, top), cv::Point(right, bottom),cv::Scalar(0,0,0), 2);

        // augment object with some key data
        char str1[200], str2[200];
        sprintf(str1, "id=%d, #pts=%d", it1->boxID, (int)it1->lidarPoints.size());
        putText(topviewImg, str1, cv::Point2f(left-250, bottom+50), cv::FONT_ITALIC, 2, currColor);
        sprintf(str2, "xmin=%2.2f m, yw=%2.2f m", xwmin, ywmax-ywmin);
        putText(topviewImg, str2, cv::Point2f(left-250, bottom+125), cv::FONT_ITALIC, 2, currColor);
    }

    // plot distance markers
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing);
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0));
    }

    // display image
    string windowName = "3D Objects";
    cv::namedWindow(windowName, 1);
    cv::imshow(windowName, topviewImg);

    if(bWait)
    {
        cv::waitKey(0); // wait for key to be pressed
    }
}

// associate a given bounding box with the keypoints it contains
void clusterKptMatchesWithROI(BoundingBox &boundingBox, std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr, std::vector<cv::DMatch> &kptMatches)
{
    // augument the bounding box with findings
    // find distances between prev and curr points if within bounding box
    std::vector<double> distance;

    for(auto match : kptMatches) {
        if(boundingBox.roi.contains(kptsCurr[match.trainIdx].pt))
        {
            // get euclidean distance to calculate square root
            cv::Point2f diff = (kptsCurr[match.trainIdx].pt) - (kptsPrev[match.queryIdx].pt);
            float eucDist = cv::sqrt(diff.x*diff.x + diff.y*diff.y);
            // push the match into boundingBox
            distance.push_back(eucDist);
        }
    }

    double distMean = std::accumulate(distance.begin(), distance.end(), 0.0) / distance.size();


    //put into kptMatches enclosed by bounding box into bounding box vector
    for(auto rematch : kptMatches)
    {
        if(boundingBox.roi.contains(kptsCurr[rematch.trainIdx].pt))
        {
            cv::Point2f diff = (kptsCurr[rematch.trainIdx].pt) - (kptsPrev[rematch.queryIdx].pt);
            float eucDist = cv::sqrt(diff.x*diff.x + diff.y*diff.y);
            // if distance is less than threshold
            if(eucDist < distMean * 0.75)
            {
                // add the matches and keypoints into Box data
                boundingBox.keypoints.push_back(kptsCurr[rematch.trainIdx]);
                boundingBox.kptMatches.push_back(rematch);
            }
        }
    }
}


// Compute time-to-collision (TTC) based on keypoint correspondences in successive images
void computeTTCCamera(std::vector<cv::KeyPoint> &kptsPrev, std::vector<cv::KeyPoint> &kptsCurr,
                      std::vector<cv::DMatch> kptMatches, double frameRate, double &TTC, cv::Mat *visImg)
{
    // compute distance ratios between all matched keypoints
    vector<double> distRatios; // stores the distance ratios for all keypoints between curr. and prev. frame
    for (auto it1 = kptMatches.begin(); it1 != kptMatches.end() - 1; ++it1)
    { // outer keypoint loop

        // get current keypoint and its matched partner in the prev. frame
        cv::KeyPoint kpOuterCurr = kptsCurr.at(it1->trainIdx);
        cv::KeyPoint kpOuterPrev = kptsPrev.at(it1->queryIdx);

        for (auto it2 = kptMatches.begin() + 1; it2 != kptMatches.end(); ++it2)
        { // inner keypoint loop

            double minDist = 100.0; // min. required distance

            // get next keypoint and its matched partner in the prev. frame
            cv::KeyPoint kpInnerCurr = kptsCurr.at(it2->trainIdx);
            cv::KeyPoint kpInnerPrev = kptsPrev.at(it2->queryIdx);

            // compute distances and distance ratios
            double distCurr = cv::norm(kpOuterCurr.pt - kpInnerCurr.pt);
            double distPrev = cv::norm(kpOuterPrev.pt - kpInnerPrev.pt);

            if (distPrev > std::numeric_limits<double>::epsilon() && distCurr >= minDist)
            { // avoid division by zero

                double distRatio = distCurr / distPrev;
                distRatios.push_back(distRatio);
            }
        }
    }

    if (distRatios.size() == 0)
    {
        TTC = NAN;
        return;
    }

    std::sort(distRatios.begin(), distRatios.end());
    long medIndex = floor(distRatios.size() / 2.0);
    double medDistRatio = distRatios.size() % 2 == 0 ? (distRatios[medIndex - 1] + distRatios[medIndex]) / 2.0 : distRatios[medIndex]; // compute median dist. ratio to remove outlier influence

    // compute camera-based TTC from distance ratios
    double meanDistRatio = std::accumulate(distRatios.begin(), distRatios.end(), 0.0) / distRatios.size();

    double dT = 1 / frameRate;
    TTC = -dT / (1 - meanDistRatio);

    //TTC = -dT / (1 - medDistRatio);
}




void showLidarTopview(  std::vector<LidarPoint> &oldlidarPoints,
                        std::vector<LidarPoint> &newlidarPoints,
                        double minXPrev, double minXCurr )
{
    cv::Size worldSize(6.0, 10.0); // width and height of sensor field in m - as top view 10m 20m length
    cv::Size imageSize(1200, 2000); // corresponding top view image in pixel - map 10m in world space to 1000px, and 20m world to 2000px

    // create topview image - black
    cv::Mat topviewImg(imageSize, CV_8UC3, cv::Scalar(0, 0, 0));

    // plot Lidar points into image
    for (auto it = oldlidarPoints.begin(); it != oldlidarPoints.end(); ++it)
    {
        // coordinates x in driving direction and y in world metres - left
        float xw = (*it).x; // world position in m with x facing forward from sensor
        float yw = (*it).y; // world position in m with y facing left from sensor
        float iw = (*it).r; // get the relectivity value 0 to 1

        // convert from world coordinates to image coordinates
        // scaled and add height of image
        // ampping between pixels and world
        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;

        // get the z world coordinate 0.0 would be the height on the lidar sensor
        float zw = (*it).z; // height above the road surface
        double minZ = -1.40; // road surface is about 1.55m down, so 1.4 is ignore all points below z threshold

        // 2. Remove all Lidar points on the road surface while preserving
        // measurements on the obstacles in the scene.
        // if the z is sufficiently high above the road surface
        if(zw > minZ)
        {
            // get the scaling value
            float val = it->x; // distance in driving direction
            float maxVal = worldSize.height; //sets the max value of distinace in front of car

            // 1. Change the color of the Lidar points to show reflectivity
            int red = std::min(255, (int)(255 * iw)); // get the shade of red dependendent on relectivity
            cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, 0, red), -1);

            // could replace a distance colouring with a reflectivity colouring to see which objects reflect well and which don't
        }
    }

    // plot Lidar points into image
    for (auto it = newlidarPoints.begin(); it != newlidarPoints.end(); ++it)
    {
        // coordinates x in driving direction and y in world metres - left
        float xw = (*it).x; // world position in m with x facing forward from sensor
        float yw = (*it).y; // world position in m with y facing left from sensor
        float iw = (*it).r; // get the relectivity value 0 to 1

        // convert from world coordinates to image coordinates
        // scaled and add height of image
        // ampping between pixels and world
        int y = (-xw * imageSize.height / worldSize.height) + imageSize.height;
        int x = (-yw * imageSize.width / worldSize.width) + imageSize.width / 2;


        // get the z world coordinate 0.0 would be the height on the lidar sensor
        float zw = (*it).z; // height above the road surface
        double minZ = -1.40; // road surface is about 1.55m down, so 1.4 is ignore all points below z threshold


        // 2. Remove all Lidar points on the road surface while preserving
        // measurements on the obstacles in the scene.
        // if the z is sufficiently high above the road surface
        if(zw > minZ)
        {
            // get the scaling value
            float val = it->x; // distance in driving direction
            float maxVal = worldSize.height; //sets the max value of distinace in front of car
            int green = std::min(255, (int)(255 * iw)); // get the shade of green dependendent on relectivity
            cv::circle(topviewImg, cv::Point(x, y), 5, cv::Scalar(0, green, 0), -1);
        }

    }

    // plot distance markers - to get rough idea how far objects are away
    float lineSpacing = 2.0; // gap between distance markers
    int nMarkers = floor(worldSize.height / lineSpacing); // gives 10 markers for image
    // horizontal line every 2m
    for (size_t i = 0; i < nMarkers; ++i)
    {
        int y = (-(i * lineSpacing) * imageSize.height / worldSize.height) + imageSize.height;
        cv::line(topviewImg, cv::Point(0, y), cv::Point(imageSize.width, y), cv::Scalar(255, 0, 0)); // blue lines to show distnace markers
    }

    int p0 = (-minXPrev * imageSize.height / worldSize.height) + imageSize.height;
    cv::line(topviewImg, cv::Point(0, p0), cv::Point(imageSize.width, p0), cv::Scalar(255, 0, 255)); // magenta

    int c0 = (-minXCurr * imageSize.height / worldSize.height) + imageSize.height;
    cv::line(topviewImg, cv::Point(0, c0), cv::Point(imageSize.width, c0), cv::Scalar(255, 255, 0)); // yellow

    // display image
    string windowName = "Top-View Perspective of LiDAR data";
    cv::namedWindow(windowName, 2);
    cv::imshow(windowName, topviewImg);
    cv::waitKey(0); // wait for key to be pressed
}

double minXValue(std::vector<LidarPoint> &lidarData)
{
    std::vector<double> vecX;
    double minX = 0;
    double minZ = -1.40; // road surface is about 1.55m down,

    double vRelectivity = 0;
    int counter = 0;
    for (auto i : lidarData)
    {
        // exclude highest relectivity points which are probably mirrors
        if(i.r > 0.8)
            continue;
        else {
            vRelectivity += i.r;
            counter ++;
        }
    }
    // calculate average relectivity
    vRelectivity/=counter;

    for (auto it = lidarData.begin(); it != lidarData.end(); ++it)
    {
        // remove road and white line reflections
        if(((*it).z < minZ) || (it->r > vRelectivity*2)){
            lidarData.erase(it--);
        } else {
            vecX.push_back(it->x);
        }
    }
    // sort vector
    std::sort(vecX.begin(), vecX.end());

    if(vecX.size() != 0)
        minX = accumulate( vecX.begin(), vecX.end(), 0.0)/(vecX.size());

    return minX;
}


void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double frameRate, double &TTC)
{

    // take previous Lidar frame
    // current lidar fram
    // frame rate
    // calculate the TTC and return in &TTC

    // get minX of previous and current points
    double minXPrev = minXValue(lidarPointsPrev);
    double minXCurr = minXValue(lidarPointsCurr);

    // viewer for lidarpoints and minX positions
    //showLidarTopview(lidarPointsPrev, lidarPointsCurr, minXPrev, minXCurr);

    double interval = (1.0/frameRate);
    // compute TTC from both measurements - constant-velocity model
    TTC = (minXCurr *  interval)/ (minXPrev - minXCurr);

}

// match bounding boxes over time
void matchBoundingBoxes(std::vector<cv::DMatch> &matches, std::map<int, int> &bbBestMatches, DataFrame &prevFrame, DataFrame &currFrame)
{
    // loop over keypoint matches
    // loop through all the kp matches

    /*
    * queryIdx: previous image/image 1 descriptor index
    * trainIdx: current image/image 2 descriptor index
    * distance: distance between the descriptors - lower = better match - HAMMING or L2 - lower is better
    * imgIdx: index of the train image - always 0 ... 0th picture
    */

    // multimap to hold pairs and counts
    std::map<std::pair<int, int>, int> matchedBoxesCount;

    int count = 0;
    for (auto match : matches)
    {
        const cv::KeyPoint &previousKpIndex = prevFrame.keypoints[match.queryIdx];
        const cv::KeyPoint &currentKpIndex = currFrame.keypoints[match.trainIdx];

        // loop through bounding boxes of previous image and find if point is enclosed
        int pboxCount = 0;
        int pboxNo = 0;
        int curBoxNo = 0;
        for(auto pbox : prevFrame.boundingBoxes)
        {
            if(pbox.roi.contains(previousKpIndex.pt))
                {
                    pboxCount++;
                    pboxNo = pbox.boxID;
                }
        }
        // no point trying to match a point that doesn't exist previously
        // limit to high confidence matches by forgetting about multiple locations
        if((pboxCount == 0) || (pboxCount > 1)) continue;

        pboxCount = 0;
        for(auto cbox : currFrame.boundingBoxes)
            if(cbox.roi.contains(currentKpIndex.pt))
                {
                    curBoxNo = cbox.boxID;
                    pboxCount++;
                }

        // record match pair
        auto matchedPair = std::make_pair(pboxNo, curBoxNo);

        // count the number of matched pairs
        if(matchedBoxesCount.count(matchedPair))
            matchedBoxesCount[matchedPair] ++;
        else
            matchedBoxesCount[matchedPair] = 1; // insert new entry if count returns false

    }

    int prevValuefirst = -1;
    int currentMax = 0;

    for(auto it = matchedBoxesCount.cbegin(); it != matchedBoxesCount.cend(); ++it)
    {
        if (it->first.first == prevValuefirst){
            if(it->second > currentMax){
                currentMax = it->second;
                bbBestMatches.at(prevValuefirst) = it->first.second;

            }
        } else {
            // change to next value
            prevValuefirst = it->first.first;
            bbBestMatches[prevValuefirst] = it->first.second;
            currentMax = it->second;
        }
    }
}

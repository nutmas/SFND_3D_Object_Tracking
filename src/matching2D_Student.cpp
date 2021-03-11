#include <numeric>
#include "matching2D.hpp"

using namespace std;

// Find best matches for keypoints in two camera images based on several matching methods
void matchDescriptors(std::vector<cv::KeyPoint> &kPtsSource, std::vector<cv::KeyPoint> &kPtsRef, cv::Mat &descSource, cv::Mat &descRef,
                      std::vector<cv::DMatch> &matches, std::string descriptorType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    // set crossCheck to true for knnMatch k=1 to return only consistent pairs
    bool crossCheck = false;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    // BF Matcher - L1 or L2 for SIFT
    // HAMMING for ORB, BRISK, BRIEF
    if (matcherType.compare("MAT_BF") == 0) {
        // handle the SIFT matcher creator preference
        int normType = descriptorType.compare("DES_BINARY") == 0 ? cv::NORM_HAMMING : cv::NORM_L2;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    } else if (matcherType.compare("MAT_FLANN") == 0) {

        //BINARY descriptors: ORB, BRIEF, BRISK, FREAK, AKAZE - use hamming distance to compare
        //Floating Point descriptors: SIFT, SURF, GLOH - use euclidean distance to compare
        // Binary use FLANN + LSH index or BF + Hamming
        // Default of FlanBasedMatcher is KDTreeIndex with L2 norm (SIFT Type)

        // only for SIFT/floatig point descriptors
        if(descriptorType.compare("DES_HOG") == 0)
            matcher = cv::FlannBasedMatcher::create();
        else {
        // for binary descriptors (others)
        matcher = cv::makePtr<cv::FlannBasedMatcher>(cv::makePtr<cv::flann::LshIndexParams>(12, 20, 2));
        }
    }

    // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    {
        // nearest neighbor (best match)
        double t = (double)cv::getTickCount();
        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        //cout << "\n(NN) with n=" << matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl; ***********************
    } else if (selectorType.compare("SEL_KNN") == 0) {

        // k nearest neighbors (k=2)
        vector<vector<cv::DMatch>> knn_matches;
        double t = (double)cv::getTickCount();
        matcher->knnMatch(descSource, descRef, knn_matches, 2); // finds the 2 best matches
        t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
        //cout << "\n(KNN) with n=" << knn_matches.size() << " matches in " << 1000 * t / 1.0 << " ms" << endl;

        // to remove bad keypoint metches, filter matches using the Lowe's distance ratio test, t=0.8
        // filter matches using descriptor distance ratio test
        double minDescDistRatio = 0.8;
        for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
        {

            if ((*it)[0].distance < minDescDistRatio * (*it)[1].distance)
            {
                matches.push_back((*it)[0]);
            }
        }
        //cout << "# keypoints removed = " << knn_matches.size() - matches.size() << endl;
        cout << knn_matches.size() << "," << 1000 * t / 1.0 << "," <<knn_matches.size() - matches.size() << ",";
        //cout  <<  1000 * t / 1.0 << ",";

    }

}

// Use one of several types of state-of-art descriptors to uniquely identify keypoints
void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }else if(descriptorType.compare("BRIEF")==0)
    {
        extractor=cv::xfeatures2d::BriefDescriptorExtractor::create();
    }
    else if(descriptorType.compare("ORB")==0)
    {
        extractor=cv::ORB::create();
    }
    else if(descriptorType.compare("FREAK")==0)
    {
        extractor=cv::xfeatures2d::FREAK::create();
    }
    else if(descriptorType.compare("AKAZE")==0)
    {
        extractor=cv::AKAZE::create();
    }
    else if(descriptorType.compare("SIFT")==0)
    {
        extractor=cv::SIFT::create();
    }
    else
    {
        // not a valid descriptor type
        throw invalid_argument("\n" + descriptorType + " not supported");
    }

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
    //cout <<  1000 * t / 1.0 << ",";
}

// Detect keypoints in image using the traditional Shi-Thomasi detector
void detKeypointsShiTomasi(vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis)
{
    // compute detector parameters based on image size
    int blockSize = 4;       //  size of an average block for computing a derivative covariation matrix over each pixel neighborhood
    double maxOverlap = 0.0; // max. permissible overlap between two features in %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = img.rows * img.cols / max(1.0, minDistance); // max. num. of keypoints

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // Apply corner detection
    double t = (double)cv::getTickCount();
    vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(img, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    // add corners to result vector
    for (auto it = corners.begin(); it != corners.end(); ++it)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keypoints.push_back(newKeyPoint);
    }
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << "Shi-Tomasi detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    //cout << keypoints.size() << "," << 1000 * t / 1.0 << ",";
    //cout <<  1000 * t / 1.0 << ",";

    // visualize results
    if (bVis)
    {
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        string windowName = "Shi-Tomasi Corner Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(0);
    }
}

void detKeypointsHarris(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, bool bVis){

    // Detector parameters
    int blockSize = 2;     // for every pixel, a blockSize × blockSize neighborhood is considered
    int apertureSize = 3;  // aperture parameter for Sobel operator (must be odd)
    int minResponse = 100; // minimum value for a corner in the 8bit scaled response matrix, threshold to filter points for corners
    double k = 0.04;       // Harris parameter (see equation for details)

    double t = (double)cv::getTickCount();

    // Detect Harris corners and normalize output
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros(img.size(), CV_32FC1);
    cv::cornerHarris(img, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    double maxOverlap = 0.0; // max. permissible overlap between two features in %, used during non-maxima suppression

    // loop over normalised image rows and columns
    for( size_t j = 0; j < dst_norm.rows ; j++ )
    {
        for( size_t i = 0; i < dst_norm.cols; i++ )
        {
            // check if value at pixel position is greater than threshold
            if( (int) dst_norm.at<float>(j,i) > minResponse )
            {

                // create a keypoint
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i,j);
                newKeyPoint.size = 2 * apertureSize;
                newKeyPoint.response = (int)dst_norm.at<float>(j,i);

                // perform non-maximum suppression (NMS) in local neighbourhood around new key point
                bool bOverlap = false;
                // iterate over the keypoints already stored
                for (auto it = keypoints.begin(); it != keypoints.end(); ++it)
                {
                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, *it);
                    if (kptOverlap > maxOverlap)
                    {
                        bOverlap = true;
                        if (newKeyPoint.response > (*it).response)
                        {                      // if overlap is >t AND response is higher for new kpt
                            *it = newKeyPoint; // replace old key point with new one
                            break;             // quit loop over keypoints
                        }
                    }
                }
                if (!bOverlap)
                {                                     // only add new key point if no overlap has been found in previous keypoints
                    keypoints.push_back(newKeyPoint); // store new keypoint in dynamic list
                }
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << "Harris detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    //cout << keypoints.size() << "," << 1000 * t / 1.0 << ",";
    //cout  <<  1000 * t / 1.0 << ",";

    if(bVis)
    {
        // visualize results
        std::string windowName = "Harris Corner Detection Results";
        cv::namedWindow(windowName, 5);
        cv::Mat visImage = dst_norm_scaled.clone();
        cv::drawKeypoints(dst_norm_scaled, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);

    }



}

void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis){

    double t;
    cv::Ptr<cv::FeatureDetector> detector;
    // create the detector type
    if (detectorType.compare("FAST") == 0){
        t = (double)cv::getTickCount();
        detector = cv::FastFeatureDetector::create();
    } else if (detectorType.compare("BRISK") == 0) {
        t = (double)cv::getTickCount();
        detector=cv::BRISK::create();
    } else if (detectorType.compare("ORB") == 0) {
        t = (double)cv::getTickCount();
        detector=cv::ORB::create();
    } else if (detectorType.compare("AKAZE") == 0) {
        t = (double)cv::getTickCount();
        detector=cv::AKAZE::create();
    } else if (detectorType.compare("SIFT") == 0) {
        t = (double)cv::getTickCount();
        detector=cv::SIFT::create();
        //cv::xfeatures2d::SIFT::create()
    //SiftFeatureDetector
    } else {
        // not a valid detector type
        throw invalid_argument(detectorType + " not supported");
    }
    // detect the keypoints
    detector->detect(img, keypoints);

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    //cout << detectorType << " detection with n=" << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
    //cout << keypoints.size() << "," << 1000 * t / 1.0 << ",";
    //cout  <<  1000 * t / 1.0 << ",";

    if(bVis)
    {
        // visualize results
        std::string windowName = detectorType + " Corner Detection Results";
        cv::namedWindow(windowName, 5);
        cv::Mat visImage = img.clone();
        cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        cv::imshow(windowName, visImage);
        cv::waitKey(0);

    }
}

/*************************************************************************
*   Class LucasKanade implementation
*   This file is part of labrom_optical_flow
*
*   labrom_optical_flow is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*
*   labrom_optical_flow is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*
*   You should have received a copy of the GNU General Public License
*   along with labrom_optical_flow.  If not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

#include "labrom_optical_flow/lucas_kanade.h"

namespace optical_flow{

/**
* Constructor.
* @param max_nb_features_to_be_tracked maximum number of features to be tracked
* @param min_nb_features_to_be_tracked minimum number of features to be tracked
*/

LucasKanade::LucasKanade(int max_nb_features_to_be_tracked, int min_nb_features_to_be_tracked){
   max_nb_features_to_be_tracked_ = max_nb_features_to_be_tracked;
   min_nb_features_to_be_tracked_ = min_nb_features_to_be_tracked;

  // Lucas-Kanade tracker parameters
  lk_winSize_ = cv::Size(15,15);
  lk_maxLevel_ = 3;
  lk_termcrit_ = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,15,0.03);
  lk_flags_ = 0;
  lk_mingEigThreshold_ = 2e-3;

  // Shi-tomasi parameters
  shitom_qualityLevel_ = 0.05;
  shitom_minDistance_ = 10;
  shitom_blockSize_ = 3;
}
/**
* Destructor
*/
LucasKanade::~LucasKanade(void){};

/**
* Set Lucas-Kanade tracker parameters. See more about the function in the OpenCV official documentation for the function cv::calcOpticalFlowPyrLK.
* @param[in] winsize search area in pixels
* @param[in] maxLevel pyramid level (0 pyramid is not used)
* @param[in] termcritMaxCount maximum number of iterations before convergence
* @param[in] termcritEpsilon treshhold value that convergence is accepted
* @param[in] flags LucasKanade parameters that indicates search/error measurerement
* @param[in] minEigThreshold threshold for removing bad points
*/
void LucasKanade::SetLucasKanadeParams(int winSize, int maxLevel, int termcritMaxCount, double termcritEpsilon, int flags, double mingEigThreshold){
  // Updating parameters
  lk_winSize_ = cv::Size(winSize,winSize);
  lk_maxLevel_ = maxLevel;
  lk_termcrit_ = cv::TermCriteria(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,termcritMaxCount,termcritEpsilon);
  lk_flags_ = flags;
  lk_mingEigThreshold_ = mingEigThreshold;

  // Flushing to initialize set of features with current parameters configuration
  FlushFeatures();
};

/**
* Set Shi-tomasi parameters. See more about the function in the OpenCV official documentation for the function cv::goodFeaturesToTrack.
* @param[in] qualityLevel threshold ratio for worst feature score with respect to best feature score
* @param[in] minDistance minimum distance in pixel between two features
* @param[in] size for computing covariation matrix
*/
void LucasKanade::SetShiTomasiParams(double qualityLevel, double minDistance, int blockSize){
  // Updating parameters
  shitom_qualityLevel_ = qualityLevel;
  shitom_minDistance_  = minDistance;
  shitom_blockSize_    = blockSize;

  // Flushing to initialize set of features with current parameters configuration
  FlushFeatures();
};

/**
*  Initialize feature vector.
*  Currently, shi-tomasi detector is employed
*  @param img image to extract features from.
*/
void LucasKanade::InitializeFeatures(const cv::Mat &img){
  cv::Mat mask = cv::Mat::zeros(img.rows, img.cols, CV_8U);
  cv::rectangle(mask, cv::Rect(100,100,520,280), 255, -1);  

  // Apply corner detection
  features_pts_[0].clear();
  //! /todo use mask for region of interest. Therefore, no need to re-initialize the whole feature vector
  cv::goodFeaturesToTrack( img,
               features_pts_[0],
               max_nb_features_to_be_tracked_,
               shitom_qualityLevel_,
               shitom_minDistance_,
               mask, //cv::Mat(),
               shitom_blockSize_);
 
  // Check if enough features were detected
  if (features_pts_[0].size() <  min_nb_features_to_be_tracked_ ){ 
    //std::cerr << "Optical Flow (Lucas-Kanade): Not enough features detected. Current #features:" << features_pts_[0].size() << std::endl;
    return;
  }

  // Parameters for subpixel  (See cornerSubPix OpenCV documentation)
  cv::TermCriteria termcrit(cv::TermCriteria::COUNT|cv::TermCriteria::EPS,20,0.03);
  cv::Size subPixWinSize(10,10), zeroZone(-1,-1);

  // Refine corner location
  cv::cornerSubPix(img, features_pts_[0], subPixWinSize, zeroZone, termcrit);
}

/** 
* Compute Optical Flow
* @param prevImg previous image
* @param nextImg next image
* @param u output vector for feature coordinates (image frame)
* @param du output vector for features velocities (image frame)
*/
int LucasKanade::ComputeOpticalFlow(const cv::Mat &input_img, std::vector<cv::Point2f> &u, std::vector<cv::Point2f> &du){
    static int age = 0;
    // Re-initialize features with not enough features are being tracked
    if (features_pts_[0].size() < min_nb_features_to_be_tracked_ || ++age > 4){
        // Initialize feature in the current frame
        InitializeFeatures(input_img);
        // Save frame for next iteration
        prev_img_ = input_img.clone();
        age = 0;
        return ERROR_NOT_ENOUGH_FEATURES;
    }

    // Parameters for pyramidal optical flow (See calcOpticalFlowPyrLK OpenCV documentation)
    std::vector<uchar> status;
    std::vector<float> err; 
   
    // Calculate optical flow
    calcOpticalFlowPyrLK(prev_img_, input_img, 
                           features_pts_[0], features_pts_[1], 
                           status, 
                           err,
                           lk_winSize_,
                           lk_maxLevel_,
                           lk_termcrit_, 
                           lk_flags_, 
                           lk_mingEigThreshold_);
    
    // Remove points that tracking failed and update output.
    u.clear();
    du.clear();

    for (int i=0, j=0; i<features_pts_[0].size(); ++i){
        // No tracking correspondence (do not add..)
        if (status[i] == 0)
            continue;
        // Found correspondence, then add to feature vector
        u.push_back(features_pts_[1][i]);
        du.push_back(features_pts_[1][i] - features_pts_[0][i]);
        ++j;
    }

    // Rearrange feature points vector for next iteration
    features_pts_[0] = u;   
    // Save image for next iteration
    prev_img_ = input_img.clone();
    
    // Check if track is consistent
    int nb_features_tracked = features_pts_[0].size();
    if (nb_features_tracked < min_nb_features_to_be_tracked_)
        return ERROR_POOR_TRACKING;
    else
        return nb_features_tracked;

}

/**
* Draw optical flow. OVERWRITES input image
* @param img image to draw optical flow over.
* @param u feature vector
*/
void  LucasKanade::DrawOpticalFlow(cv::Mat &img, std::vector<cv::Point2f> &u){
  if (canvas_.rows != img.rows || canvas_.cols != img.cols)
    // Clean canvas
    canvas_ = cv::Mat::zeros(img.rows, img.cols, CV_8UC1);
  else
    // Fade
    canvas_ = 0.95*canvas_;
 
  for (int i=0; i < u.size(); ++i){
    cv::circle( canvas_, u[i], 1, cv::Scalar(255,255,255), -1, 8);
  } 

  img = 0.7*canvas_ + 0.3*img;
}

/**
* Clean feature vector to reinitialize in the next iteration
*/

void LucasKanade::FlushFeatures(void){
  features_pts_[0].clear();
}



}   // optical_flow namespace

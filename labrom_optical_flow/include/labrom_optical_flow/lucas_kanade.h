/*************************************************************************
*   Class LucasKanade header files
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

#ifndef OPTICAL_FLOW_LUCAS_KANADE_H_
#define OPTICAL_FLOW_LUCAS_KANADE_H_

// labrom_optical_flow libraries
#include "labrom_optical_flow/optical_flow.h"

// OpenCV libriares
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include "iostream"

namespace optical_flow{
class LucasKanade{
  public:
    //! Constructor
    LucasKanade(int max_nb_features_to_be_tracked=30, int min_nb_features_to_be_tracked=100);
    //! Destructor
    ~LucasKanade(void);
    //! Set Lucas-Kanade tracker parameters
    void SetLucasKanadeParams(int winSize=10, int maxLevel=3, int termcritMaxCount=15, double termcritEpsilon=0.03, int flags=0, double mingEigThreshold=2e-4);
    //! Set Shi-tomasi parameters
    void SetShiTomasiParams(double qualityLevel=0.1, double minDistance=10, int blockSize=3);
    //! Initialize feature vector
    void InitializeFeatures(const cv::Mat &img);
    //! Compute Optical Flow
    int ComputeOpticalFlow(const cv::Mat &input_img, std::vector<cv::Point2f> &u, std::vector<cv::Point2f> &du);
    //! Draw optical flow
    void DrawOpticalFlow(cv::Mat &img, std::vector<cv::Point2f> &u);
    //! Clean detected feature vector
    void FlushFeatures(void);
    
    private:
      std::vector<cv::Point2f> features_pts_[2];       //!< Tracked features coordinates
      cv::Mat prev_img_;                               //!< Previous image
      cv::Mat canvas_;                                 //!< Canvas that draws optical flow on top 

      // Optical Flow parameters
      int max_nb_features_to_be_tracked_;        //!< Maximum number of features to be tracked each frame
      int min_nb_features_to_be_tracked_;        //!< Minimum number of features to be tracked each frame

      // Lukas Kanade tracker parameters
      cv::Size lk_winSize_;
      int lk_maxLevel_;
      cv::TermCriteria lk_termcrit_;
      int lk_flags_;
      double lk_mingEigThreshold_;

      // Shi-tomasi parameters
      double shitom_qualityLevel_;
      double shitom_minDistance_;
      int shitom_blockSize_;

      


};
} // optical_flow namespace
#endif // OPTICAL_FLOW_H_
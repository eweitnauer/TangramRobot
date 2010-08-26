// Copyright 2009 Erik Weitnauer
#ifndef ICL_TANGRAM_CORNER_DETECTOR_CSS_H
#define ICL_TANGRAM_CORNER_DETECTOR_CSS_H

#include <ICLQuick/Common.h>
#include <ipp.h>

  /// Curvature Corner Detector
  /**
   * Implementation of the Curvature Scale Space corner detection algorithm
   * described in the paper "Corner detector based on global and local curvature properties",
   * by Chen He, Xiao and Yung, Nelson H.C. in Optical Engineering 2008.
   * There also is a Matlab implementation of the algorithm by Chen He, Xiao available
   * at http://www.mathworks.com/matlabcentral/fileexchange/7652
   *
   * The algorithm takes an array of contour points as input, that are then smoothed
   * by a gaussian filter. Afterwards the curvature function is calculated and its
   * maxima positions are taken as corner candidates.
   * Afterwards all candidates which belong to round or "false" (noise induced, etc.) corners
   * are removed and finally a list of corners is given back.
   * 
   * Parameters for the algorithm are:
   * rc_coeff - denotes the minimum ratio of major axis to minor axis of an ellipse, 
   *            whose vertex could be detected as a corner by proposed detector
   *            The default value is 1.5.
   * angle_thresh - denotes the maximum obtuse angle that a corner can have when 
   *            it is detected as a true corner, default value is 162.
   * sigma -    denotes the standard deviation of the Gaussian filter when
   *            computeing curvature. The default sig is 3.
   * straight_line_thresh - to estimate the angle of a corner, either a circle or a
   *            straight line approximation of the left and right surrounding is used.
   *            The straight line approximation is used, if the angle between the
   *            left neigbour, corner candidate and  and the point on the contour half
   *            way between them is smaller than straight_line_thresh.
   *            0 leads to circle approximation only, 180 to straight line approximation only.
   *
   * usage example:
   *   \code 
   *   const std::vector<icl::Region> &rs = d.detect(&image);
   *   const vector<Point32f> &boundary = getThinnedBoundary(rs[0].getBoundary());
   *   CornerDetectorCSS css;
   *   const vector<Point32f> &corners = css.detectCorners(boundary);
   *   \endcode
   **/
  class CornerDetectorCSS {
    public:
      CornerDetectorCSS(float angle_thresh=162., float rc_coeff=1.5, float sigma=3., float curvature_cutoff=100., float straight_line_thresh=0.1)
      : angle_thresh(angle_thresh), rc_coeff(rc_coeff), sigma(sigma), curvature_cutoff(curvature_cutoff), straight_line_thresh(straight_line_thresh) {};

      /// calculates a normalized 1d gaussian vector
       /**
        * @param gau adress of pointer to Ipp32f array, which will hold the calculated gauss kernel
        * @param sigma sigma^2 is the variance of the gaussian, default is 1
        * @param cutoff if value of gaussian drops below this value, it is set to zero, default is 0.001
        * @return width of the gaussian tails, so the size of the returned vector is 2*width+1
        */
      static int gaussian(Ipp32f **gau, float sigma, float cutoff);

      /// detects the corners in the passed contour
       /**
        * Use this function to detect the corners in an image.\n
        * A reference to the contour in which the corners should be detected must be passed. The contour
        * should be "thinned", meaning that it should look e.g. like
        * ooox                ooxx
        * ooxo  and not like  oxxo
        * oxoo                xxoo
        * xooo                xooo
        */
      const vector<Point32f> &detectCorners(const vector<Point32f> &boundary);
     
      /// Returns approximated angles of corners in deg. Call detectCorners method first.
      const vector<float> &getCornerAngles() { return corner_angles; }

      /// converts Ipp32f array to string
      static string ipp32f_to_string (const Ipp32f* v, int length);

      /// converts a std::vector to string
      template <class T>
          static string vector_to_string(const vector<T> v);

      /// parameters
      float angle_thresh, rc_coeff, sigma, curvature_cutoff, straight_line_thresh;

    private:
      /// finds the indicies of extrema
      /**
       * @param extrema reference to a vector in which the extrema are stored
       * @param x function values
       * @param length number of points in array x
       */
      static void findExtrema(vector<int> &extrema, Ipp32f* x, int length);

      /// removes round corners by comparing all corner candidates with adaptive local threshold
      /**
       * The round corner coefficient denotes the minimum ratio of major axis to minor
       * axis of an ellipse, whose vertex could be detected as a corner by proposed detector.
       * @param rc_coeff round corner coefficient
       * @param k curvature function
       * @param extrema indicies of extrem points in curvature function k
       */
      static void removeRoundCorners( float rc_coeff, Ipp32f* k, vector<int> &extrema);

      /// remove false corners by checking the corner angle
      /**
       * The angle threshold denotes the maximum obtuse angle that a corner can have when
       * it is detected as a true corner.
       * @param angle_thresh angle threshold
       * @param xx,yy x,y dimension of smoothed contour
       * @param k curvature function
       * @param length number of points in smoothed contour (and curvature function)
       * @param maxima indicies of maximum points in curvature function k
       */
      static void removeFalseCorners(float angle_thresh, Ipp32f* xx, Ipp32f* yy, Ipp32f* k, int length, vector<int> &maxima, vector<float> &corner_angles, float straight_line_thresh);

      /// estimates the angle of a corner
       /**
        * Eestimates the angle of a corner in a part of the curve by fitting a circle
        * on each side of the corner and calculating the angle between the two tangents.
        * @param x,y x,y dimension of the contour segment around the corner
        * @param length number of points in contour segment
        * @param center position of corner in contour segment
        */
      static float tangentAngle(Ipp32f* x, Ipp32f* y, int length, int center, float straight_line_thresh);

      /// signum function
      template<class T> static float sign(T x);

      // result lists
      vector<Point32f> corners;
      vector<float> corner_angles;
      vector<int> extrema;
  };

#endif

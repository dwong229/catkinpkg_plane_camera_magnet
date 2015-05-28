#include <cassert>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include "hw4.h"

using namespace std;
using namespace cv;

/**
 * Helper function for getClick()
 */
static void onMouse(int event, int x, int y, int, void *ptr)
{
  if(CV_EVENT_LBUTTONDBLCLK == event) {
    // left double click
    *static_cast<Point*>(ptr) = Point(x, y);
  }
}

/**
 * Display an image in a window and wait for the user to
 * double click a point in the image.
 *
 * @return A point clicked by the user.
 */
Point getClick(const string &winname, const Mat &img)
{
  Point pt(-1, -1);

  namedWindow(winname, WINDOW_FLAGS);
  setMouseCallback(winname, onMouse, &pt);

  cout << "Please double click a point in the window " << winname << endl;
  while(-1 == pt.x && -1 == pt.y) {
    imshow(winname, img);
    waitKey(30);
  }

  return pt;
}

/*
static void drawLine(Mat &im, const Vec3d &l)
{
  Vec3d l0, lm;
  if(fabs(l(0)/l(1)) > 1.0) {
    // line is predominantly vertical
    l0 = Vec3d(0, -1, 0);        // line y=0
    lm = Vec3d(0, -1, im.rows);  // line y=im.rows
  } else {
    // line is predominantly horizontal
    l0 = Vec3d(-1, 0, 0);        // line x=0
    lm = Vec3d(-1, 0, im.cols);  // line x=im.cols
  }

  // intersections of l with either vertical or horizontal
  // boundaries of im
  Vec3d pt0 = l.cross(l0); pt0 /= pt0(2);
  Vec3d ptm = l.cross(lm); ptm /= ptm(2);

  Point p0, pm;
  p0 = Point(cvRound(pt0(0)*8), cvRound(pt0(1)*8));
  pm = Point(cvRound(ptm(0)*8), cvRound(ptm(1)*8));

  line(im, p0, pm, Scalar(255, 0, 0), 3, CV_AA, 3);
}
*/

/*Vec3d getVanishingPoint(const Mat &imOrig, size_t nPPL, size_t nLPVP)
{
  Mat im = imOrig.clone();

  Mat L(nLPVP, 3, CV_64FC1);
  double *pL = L.ptr<double>(0);

  for(size_t i=0; i<nLPVP; ++i) {
    // for each line
    cout << "Please start double clicking points for a new line to compute the vanishing point" << endl;
    Vec3d l = fitLine(im, nPPL);
    for(size_t j=0; j<3; ++j)
      pL[i*3+j] = l(j);
    drawLine(im, l);
  }

  return findIntersection(L);
}
*/

void drawCross(Mat &im, const Point &pt, size_t sz)
{
  line(im, Point(pt.x-sz, pt.y),
           Point(pt.x+sz, pt.y), Scalar(255, 0, 0), (2*sz+1)/3);
  line(im, Point(pt.x, pt.y-sz),
           Point(pt.x, pt.y+sz), Scalar(255, 0, 0), (2*sz+1)/3);
}

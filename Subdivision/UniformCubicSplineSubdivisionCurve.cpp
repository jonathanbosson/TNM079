
#include "UniformCubicSplineSubdivisionCurve.h"

UniformCubicSplineSubdivisionCurve::UniformCubicSplineSubdivisionCurve(const std::vector<Vector3<float> > &joints,
                                                                       Vector3<float> lineColor,
                                                                       float lineWidth)
  : mCoefficients(joints), mControlPolygon(joints)
{
  this->mLineColor = lineColor;
  this->mLineWidth = lineWidth;
}


void UniformCubicSplineSubdivisionCurve::Subdivide()
{
  // Allocate space for new coefficients
  std::vector<Vector3<float> > newc;

  assert(mCoefficients.size() > 4 && "Need at least 5 points to subdivide");

  // Implement the subdivision scheme for a natural cubic spline here

  // nyaCoeff = S			*			gamlaCoeff
  // 2k+2   (k+1)*(2k+2)				  k+1
  // double amount of coeff

  // Create S by first
  // create zero matrix with dimensions (k+1)*(2k+2)
  // fill in 1/8(1, 4, 6, 4, 1) in the right positions of the matrix
  // with for loop over coulumns(k+1)

  for (int i = 0; i < k+1; ++i) {
	  S[2*i+0][i] = 1.0;
	  S[2*i+1][i] = 4.0;
	  S[2*i+2][i] = 6.0;
	  S[2*i+3][i] = 4.0;
	  S[2*i+4][i] = 1.0;
  }
  S = 1/8 * S;

  // If 'mCoefficients' had size N, how large should 'newc' be? Perform a check here!
  assert(true && "Incorrect number of new coefficients!");
  
  mCoefficients = newc;
}


void UniformCubicSplineSubdivisionCurve::Render()
{
  // Apply transform
  glPushMatrix(); // Push modelview matrix onto stack

  // Convert transform-matrix to format matching GL matrix format
  // Load transform into modelview matrix
  glMultMatrixf( mTransform.ToGLMatrix().GetArrayPtr() );

  mControlPolygon.Render();

  // save line point and color states
  glPushAttrib(GL_POINT_BIT | GL_LINE_BIT | GL_CURRENT_BIT);

  // draw segments
  glLineWidth(mLineWidth);
  glColor3fv(mLineColor.GetArrayPtr());
  glBegin(GL_LINE_STRIP);
  // just draw the spline as a series of connected linear segments
  for(unsigned int i = 0; i < mCoefficients.size(); i++){
    glVertex3fv( mCoefficients.at(i).GetArrayPtr() );
  }
  glEnd();

  // restore attribs
  glPopAttrib();

  glPopMatrix();

  GLObject::Render();
}


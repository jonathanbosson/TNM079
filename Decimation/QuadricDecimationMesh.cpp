/*************************************************************************************************
 *
 * Modeling and animation (TNM079) 2007
 * Code base for lab assignments. Copyright:
 *   Gunnar Johansson (gunnar.johansson@itn.liu.se)
 *   Ken Museth (ken.museth@itn.liu.se)
 *   Michael Bang Nielsen (bang@daimi.au.dk)
 *   Ola Nilsson (ola.nilsson@itn.liu.se)
 *   Andreas Sderstrm (andreas.soderstrom@itn.liu.se)
 *
 *************************************************************************************************/
#include "QuadricDecimationMesh.h"

const QuadricDecimationMesh::VisualizationMode QuadricDecimationMesh::QuadricIsoSurfaces = NewVisualizationMode("Quadric Iso Surfaces");

void QuadricDecimationMesh::Initialize()
{
  // Allocate memory for the quadric array
  unsigned int numVerts = mVerts.size();
  mQuadrics.reserve(numVerts);
  std::streamsize width = std::cerr.precision(); // store stream precision
  for (unsigned int i = 0; i < numVerts; i++) {

    // Compute quadric for vertex i here
    mQuadrics.push_back(createQuadricForVert(i));


    // Calculate initial error, should be numerically close to 0

    Vector3<float> v0 = mVerts[i].pos;
    Vector4<float> v(v0[0],v0[1],v0[2],1);
    Matrix4x4<float> m = mQuadrics.back();

    float error = v*(m*v);
    //std::cerr << std::scientific << std::setprecision(2) << error << " ";
  }
  std::cerr << std::setprecision(width) << std::fixed; // reset stream precision

  // Run the initialize for the parent class to initialize the edge collapses
  DecimationMesh::Initialize();
}

/*! \lab2 Implement the computeCollapse here */
/*!
 * \param[in,out] collapse The edge collapse object to (re-)compute, DecimationMesh::EdgeCollapse
 */
void QuadricDecimationMesh::computeCollapse(EdgeCollapse * collapse)
{
  // Compute collapse->position and collapse->cost here
  // based on the quadrics at the edge endpoints

	// det(Q) = D
	// if D != 0
	// collapse->position = inv(Q) * zerovecto
	int i1 = e(collapse->halfEdge).vert;
	int i2 = e(e(collapse->halfEdge).pair).vert;
	Matrix4x4<float> Q1 = mQuadrics.at(i1);
	Matrix4x4<float> Q2 = mQuadrics.at(i2);
	Matrix4x4<float> Q = Q1 + Q2;
	Matrix4x4<float> Qpos = Q;
	Qpos(3,0) = 0.0f; Qpos(3,1) = 0.0f; Qpos(3,2) = 0.0f; Qpos(3,3) = 1.0f;
	

	// Compute collapse->position
	if (!Qpos.IsSingular()) {
		Vector4<float> newPos(0.0f, 0.0f, 0.0f, 1.0f);
		newPos = Qpos.Inverse() * newPos;
		collapse->position = Vector3<float>(newPos[0], newPos[1], newPos[2]);

		// Compute collapse->cost
		Vector4<float> temp = Vector4<float>(collapse->position[0], collapse->position[1], collapse->position[2], 1.0f);
		collapse->cost = temp * (Q * temp);
	}
	else {
		// Calculate cost for collapse->position = v1
		Vector3<float> v1 = v(i1).pos;
		Vector4<float> vec1 = Vector4<float>(v1[0], v1[1], v1[2], 1.0);
		float c1 = vec1 * (Q * vec1);

		// Calculate cost for collapse->position = v2
		Vector3<float> v2 = v(i2).pos;
		Vector4<float> vec2 = Vector4<float>(v2[0], v2[1], v2[2], 1.0);
		float c2 = vec2 * (Q * vec2);

		// Calculate cost for collapse->position = (v1+v2)/2.0
		Vector3<float> midPos = (v1+v2)/2.0;
		Vector4<float> vec3 = Vector4<float>(midPos[0], midPos[1], midPos[2], 1.0);
		float c3 = vec3 * (Q * vec3);

		collapse->cost = std::min(std::min(c1, c2), c3);
		if (collapse->cost == c1)
			collapse->position = v1;
		else if (collapse->cost == c2)
			collapse->position = v2;
		else 
			collapse->position = midPos;

	}
	
}

/*! After each edge collapse the vertex properties need to be updated */
void QuadricDecimationMesh::updateVertexProperties(unsigned int ind)
{
  DecimationMesh::updateVertexProperties(ind);
  mQuadrics[ind] = createQuadricForVert(ind);
}

/*!
 * \param[in] indx vertex index, points into HalfEdgeMesh::mVerts
 */
Matrix4x4<float> QuadricDecimationMesh::createQuadricForVert(unsigned int indx) const{
	Vertex vert = v(indx);
	float m[4][4] = {{0, 0,  0, 0},
				{0, 0,  0, 0},
				{0, 0,  0, 0},
				{0, 0,  0, 0}};
	Matrix4x4<float> Q(m);

	std::vector<unsigned int> oneRing = HalfEdgeMesh::FindNeighborFaces(indx);

	for (int i = 0; i < oneRing.size(); i++) {
		Q += createQuadricForFace(oneRing.at(i));
	}

  // The quadric for a vertex is the sum of all the quadrics for the adjacent faces
  // Tip: Matrix4x4 has an operator +=
  return Q;
}

/*!
 * \param[in] indx face index, points into HalfEdgeMesh::mFaces
 */
Matrix4x4<float> QuadricDecimationMesh::createQuadricForFace(unsigned int indx) const{

  // Calculate the quadric (outer product of plane parameters) for a face
  // here using the formula from Garland and Heckbert
	Vector3<float> pos = v(e(f(indx).edge).vert).pos;
	// ax + by +cz = -d
	Vector3<float> nor = f(indx).normal;
	float d = pos[0]*nor[0] + pos[1]*nor[1] + pos[2]*nor[2];
	d = d*-1.0;
	float q[4][4] = {
						{nor[0]*nor[0], nor[1]*nor[0],  nor[2]*nor[0],  nor[0]*d},
						{nor[0]*nor[1],	nor[1]*nor[1],  nor[2]*nor[1],  nor[1]*d},
						{nor[0]*nor[2],	nor[1]*nor[2],  nor[2]*nor[2],  nor[2]*d},
						{nor[0]*d,		nor[1]*d,		nor[2]*d,		d*d}};
	
	return Matrix4x4<float>(q);
}


void QuadricDecimationMesh::Render()
{
  DecimationMesh::Render();

  glEnable(GL_LIGHTING);
  glMatrixMode(GL_MODELVIEW);

  if (mVisualizationMode == QuadricIsoSurfaces)
    {
      // Apply transform
      glPushMatrix(); // Push modelview matrix onto stack

      // Implement the quadric visualization here
      std::cout << "Quadric visualization not implemented" << std::endl;

      // Restore modelview matrix
      glPopMatrix();
    }
}


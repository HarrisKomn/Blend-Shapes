#include <VVRScene/canvas.h>
#include <VVRScene/mesh.h>
#include <VVRScene/settings.h>
#include <VVRScene/utils.h>
#include <MathGeoLib.h>
#include <iostream>
#include <fstream>
#include <cstring>
#include <string>
#include <set>
#include "symmetriceigensolver3x3.h"
#include "SceneIntersections_8365.h"
#include "Eigen/Dense"
#include "Eigen/SparseCore"




#define FLAG_SHOW_SOLID		4
#define FLAG_SHOW_AXES		1
#define FLAG_SHOW_WIRE		2
#define FLAG_SHOW_NORMALS	8
#define FLAG_REGISTRATION   16
#define FLAG_SHOW_PLANE		false
#define FLAG_SHOW_AABB		true
#define FLAG_SHOW_MY_REFERENCE 64
#define FLAG_SHOW_MY_NEW 128
#define FLAG_SHOW_GIVEN_REF 32
#define FLAG_SHOW_GIVEN 256
#define FLAG_SHOW_MY_NEW_TRIANGLES 512
#define FLAG_SHOW_MY_PAST_TRIANGLES 1024
#define SHOW_INTERSECTION_POINTS  32


void Task_1_FindCenterMass(std::vector<vec> &vertices, vec &cm);
void Task_2_FindAABB(std::vector<vec> &vertices, vvr::Box3D &aabb);
void Task_3_AlignOriginTo(std::vector<vec> &vertices, const vec &cm);
void Task_4_Draw_PCA(vec &center, vec &dir);
void Task_5_Intersect(std::vector<vvr::Triangle>& triangles, Plane &plane, std::vector<int> &intersection_indices);
void Task_5_Split(vvr::Mesh &mesh, Plane &plane, std::vector<int> &final_index_vector);
void pca(std::vector<vec>& vertices, vec &center, vec &dir);

vvr::Mesh linearMethodOfMakingBlendshapes(double w, int ind);

vvr::Mesh linearMethodOfMakingNewBlendshapes();
vvr::Mesh differentialMethodOfMakingNewBlendshapes(double w,int ind);



class Mesh3DScene : public vvr::Scene
{
public:
    Mesh3DScene();

	std::vector<int> final_my_face_indices, final_given_face_indices;
	void barycentricInterpolation(std::vector<vec> my_vertices, std::vector<vec> model_vertices, std::vector<vec>& my_matched_vertices, std::vector<vec>& model_matched_vertices);
	void triangulateControlPoints(std::vector<vec> &control_points, std::vector<vvr::Triangle> &triangl);
	void translationOfControlPoints(vvr::Mesh &mesh, vvr::Mesh given_ref, vvr::Mesh given_other);
	void translationOfNONControlPoints(vvr::Mesh &mesh, vvr::Mesh principal_mesh);
	void triangulationEXTRAControlPoints(vec testpoint,std::vector<vec> &inter_verts, std::vector < vvr::Triangle> &triangles);
	Eigen::SparseMatrix<double> laplacianMeshnig(vvr::Mesh mesh);

	void secondTriangulation(std::vector<vec> &control_points, std::vector<vvr::Triangle> &triangl);

	void thirdTriangulation(std::vector<vec> &control_points, std::vector<vvr::Triangle> &triangl);
	Eigen::MatrixXd calculateEmatrix();

	vvr::Mesh loadMyFace(std::string  my_string);
	vvr::Mesh loadGivenFace(std::string given_string);

	void insertMyControlPoints(vvr::Mesh mymesh,std::vector<vec> &cntrl_points);
	void insertGivenControlPoints(vvr::Mesh givenmesh, std::vector<vec> &cntrl_points);
	void insertMyExtraControlPoints(vvr::Mesh my_model, std::vector<int> my_extra_indices);
	void insertGivenExtraControlPoints(vvr::Mesh given_model, std::vector<int> given_extra_indices);

	bool flag_press;
	/*tests*/
	int time = 0;
	int princ_dr = 0;
	C2DPolygon polygon;
	C2DLineSet lineset;
	C2DPoint point_test;
    const char* getName() const { return "3D Scene"; }
    void keyEvent(unsigned char key, bool up, int modif) override;
    void arrowEvent(vvr::ArrowDir dir, int modif) override;
	void mousePressed(int x, int y, int  modif) override;
	vvr::Mesh m_model_original, m_model, model_test, model_given, model_test_given,model_given_ref,new_model;
	vvr::Mesh model_given_other;
	int m_style_flag;
	//vvr::Mesh makingDifferentMeshes();
	vvr::Colour m_obj_col;
private:
    void draw() override;
    void reset() override;
    void resize() override;
	void Tasks();
	vec Task_3_Pick_Origin();

private:
   // int m_style_flag;
    float m_plane_d;
	float m_plane_d_given;
	float horizontal_plane_d_given;
	float horizontal_plane_d;

    vvr::Canvas2D m_canvas;
    //vvr::Colour m_obj_col;
   // vvr::Mesh m_model_original, m_model;
    vvr::Box3D m_aabb;
	vvr::Box3D given_aabb;
	vvr::Box3D m_aabb_ref;
	vvr::Box3D given_aabb_ref;

    math::vec m_center_mass,given_center_mass,other_center;
    math::vec m_pca_cen;
    math::vec m_pca_dir;
    math::Plane m_plane,m_plane_given, horizontal_plane_given, horizontal_plane;
    std::vector<int> m_intersections;
};
/**
* Struct representing a triangle with pointers to its 3 vertices
*/
struct Tri
{
	C2DPoint *v1;
	C2DPoint *v2;
	C2DPoint *v3;
	float area;

	Tri(C2DPoint *v1, C2DPoint *v2, C2DPoint *v3) : v1(v1), v2(v2), v3(v3) {
		area = to_C2D().GetArea();
	}

	C2DTriangle to_C2D() const { return C2DTriangle(*v1, *v2, *v3); }

	vvr::Triangle2D to_vvr(vvr::Colour col = vvr::Colour::black, bool filled = false) const {
		vvr::Triangle2D t(v1->x, v1->y, v2->x, v2->y, v3->x, v3->y, col);
		t.setSolidRender(filled);
		return t;
	}

	bool operator < (const Tri& other) const {
		if (area != other.area) return (area < other.area);
		else if (v1 != other.v1) return v1 < other.v1;
		else if (v2 != other.v2) return v2 < other.v2;
		else if (v3 != other.v3) return v3 < other.v3;
	}
};

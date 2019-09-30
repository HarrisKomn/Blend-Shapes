#include "SceneMesh3D.h"
#include "SceneIntersections_8365.h"
#include <iostream>
#include <fstream>
#include "Eigen/Dense"
#include "Eigen/SparseCore"
#include "Eigen/SparseCholesky"	
#include "Eigen/SparseLU"
#include "Eigen/IterativeLinearSolvers"
#include <string>

#define SPLIT_INSTEAD_OF_INTERSECT 0
#define first_model true
#define second_model true
#define EXTRA_CONTROL_POINTS false

#define first_question false 
#define second_question false
#define second_2_question false
#define second_2_registration false
#define second_3_question false
#define second_4_question false
#define third_question true

#define Linear true




using namespace Eigen;
using namespace std;
using namespace vvr;


/*1st question*/
std::vector<int> ind_blend;
int numb_blend;
double weight[10];


Point3D point;
Point3D pointa;
Point3D pointb;
Point3D proj;
ofstream outFile;
ofstream outFileGiven;
//my model///
std::vector<vvr::Triangle> my_triangles_ref;
std::vector<vvr::Triangle> my_triangles_new;
std::vector<vvr::Triangle> given_triangles_ref;
std::vector<vvr::Triangle> given_triangles_new;

/*cloning*/
bool cloning = false;
bool fnow = false;
std::vector<double> wk;
std::vector<double> vk;
Eigen::MatrixXd Ematrix;

std::vector<vec> outvertices;
std::vector<int> out_index_vertices;

std::vector<int> my_extra;
std::vector<int> given_extra;
std::vector<vec> vectors_my_extra;
std::vector<vec> vectors_given_extra;

vec test_projection;

std::vector<vec> regist_vertices;
std::vector<vec> my_regist_vertices;

C2DPoint point_test;

/*___intersection/control points and indices_____*/
vector<vec> intersection_vectors;
vector<vec> intersection_vectors_new;

vector<vec> given_intersection_vectors;
std::vector<int> index_intersection_vectors;
std::vector<int> index_given_intersection_vectors;


std::vector<int> my_registered_indices;
std::vector<int> given_registered_indices;

/////*test*/////////////////////////////////////
std::vector<vec> violating_control_points;
std::vector<int> violating_indices;

int n_find_tr = 0;
/*sunartiseis taxinomisis*/

struct min {
	bool operator() (double d1, double d2) {
		return (d1 < d2);
	}
} minobject;

struct myclass {
	bool operator() (vec vertex1, vec vertex2) {
		return (vertex1.z > vertex2.z);
	}
} myobject_z;

struct myclass_y {
	bool operator() (vec vertex1, vec vertex2) {
		return (vertex1.y > vertex2.y);
	}
} myobject_y;

struct myclass_x {
	bool operator() (vec vertex1, vec vertex2) {
		return (vertex1.x > vertex2.x);
	}
} myobject_x;






Mesh3DScene::Mesh3DScene()
{
	

    //! Load settings.
    vvr::Shape::DEF_LINE_WIDTH = 4;
    vvr::Shape::DEF_POINT_SIZE = 10;
	m_perspective_proj = true;
    m_bg_col = Colour("768E77");
    m_obj_col = Colour("454545");
    const string objDir = getBasePath() + "resources/obj/";
	string SMatrix[10] = { "face-01-anger.obj","face-02-cry.obj","face-03-fury.obj","face-04-grin.obj","face-05-laugh.obj","face-06-rage.obj","face-07-sad.obj","face-08-smile.obj",
		"face-09-surprise.obj","face-reference.obj " };
		
	/*klisi a-erwtimatos*/
	if (first_question) {
		for (int i = 0; i < 10; i++) {
			weight[i] = 0.1;
		}
		m_model_original= loadGivenFace("face-reference.obj");
		
		if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0, 0); }
		else { m_model_original = differentialMethodOfMakingNewBlendshapes(0, 0); }
	}

	/*klisi b-ewtimatos*/
	if (second_question) {
		m_model = loadMyFace("personal_blendshape.obj");
		model_given = loadGivenFace("face-reference.obj");

		if (second_2_question) {
			model_given = loadGivenFace("face-reference.obj");

		}
		if (second_3_question) {
			//model_given = loadGivenFace("face-09-surprise.obj");
			//model_given_ref= loadGivenFace("face-reference.obj");
			//new_model = m_model;

			std::cout << "What pose do you want?";
			int x;
			cin >> x;
			model_given = loadGivenFace("face-reference.obj");
			model_given_ref= loadGivenFace(SMatrix[x]);
			new_model = m_model;
		}
	}

	if (second_4_question) {

		for (int i = 0; i < 10; i++) {
			weight[i] = 0.1;
		}
		//m_model_original = loadGivenFace("personal_blendshape.obj");

		if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0, 0); }
		else { m_model_original = differentialMethodOfMakingNewBlendshapes(0, 0); }


	}

	/*klisi c-erwtimatos*/
	if (third_question) {
		for (int i = 0; i < numb_blend; i++) {
			wk.push_back(0);
			vk.push_back(0);
		}
		 Ematrix=calculateEmatrix();
		 std::cout << Ematrix;

		 MatrixXd Ematr_inv = Eigen::MatrixXd::Zero(10, 10);
		 Ematr_inv = Ematrix.inverse();
		 std::cout << "\n\n";
		 std::cout << Ematr_inv;
		
	}
    reset();
}

void Mesh3DScene::reset()
{
	Scene::reset();


   
    //! Define plane
	 m_plane_d = 0;
	 m_plane = Plane(vec(0, 1, 1).Normalized(), m_plane_d);


	//!Define another plane for the given blendshape


    //! Define what will be vissible by default
    m_style_flag = 0;
    m_style_flag |= FLAG_SHOW_SOLID;
    m_style_flag |= FLAG_SHOW_WIRE;
    m_style_flag |= FLAG_SHOW_AXES;
    m_style_flag |= FLAG_SHOW_AABB;
  //  m_style_flag |= FLAG_SHOW_PLANE;
}

void Mesh3DScene::resize()
{
    //! By Making `first_pass` static and initializing it to true,
    //! we make sure that the if block will be executed only once.

    static bool first_pass = true;

    if (first_pass)
	{
		m_model.setBigSize(getSceneWidth() / 2);		
		m_model.update();
		
		new_model.setBigSize(getSceneWidth() / 2);
		new_model.update();

		model_given.setBigSize(getSceneWidth() / 2);		
		model_given.update();

		model_given_ref.setBigSize(getSceneWidth() / 2);
		model_given_ref.update();
        
		if (second_question) {
			Tasks();
			if (!second_4_question) {
				m_model_original = m_model;
			}
		}
        first_pass = false;
    }
}

void Mesh3DScene::Tasks()
{
	
	//!//////////////////////////////////////////////////////////////////////////////////
	//! m_model
	//!//////////////////////////////////////////////////////////////////////////////////
	Task_1_FindCenterMass(m_model.getVertices(), m_center_mass);
	vec my_center(m_center_mass.x + 30, m_center_mass.y, m_center_mass.z);
	Task_3_AlignOriginTo(m_model.getVertices(), my_center);
	Task_2_FindAABB(m_model.getVertices(), m_aabb);


	//!//////////////////////////////////////////////////////////////////////////////////
	//! model_given
	//!//////////////////////////////////////////////////////////////////////////////////	
	
	Task_1_FindCenterMass(model_given.getVertices(), given_center_mass);
	vec center(given_center_mass.x - 30, given_center_mass.y + 0, given_center_mass.z + 0);
	Task_3_AlignOriginTo(model_given.getVertices(), center);
	Task_2_FindAABB(model_given.getVertices(), given_aabb);


	if (second_3_question) {
		//!//////////////////////////////////////////////////////////////////////////////////
		//! new_model
		//!//////////////////////////////////////////////////////////////////////////////////

		Task_1_FindCenterMass(new_model.getVertices(), m_center_mass);
		vec my_center_new(m_center_mass.x + 30, m_center_mass.y, m_center_mass.z);
		Task_3_AlignOriginTo(new_model.getVertices(), my_center_new);
		Task_2_FindAABB(new_model.getVertices(), m_aabb);

		//!//////////////////////////////////////////////////////////////////////////////////
		//! model_given_ref
		//!//////////////////////////////////////////////////////////////////////////////////
		Task_1_FindCenterMass(model_given_ref.getVertices(), other_center);
		vec center1(other_center.x - 30, other_center.y + 0, other_center.z + 0);
		Task_3_AlignOriginTo(model_given_ref.getVertices(), center1);
		Task_2_FindAABB(model_given_ref.getVertices(), given_aabb_ref);
	}


		/*_____gemisma twn intersection vertices_________*/

		insertMyControlPoints(m_model, intersection_vectors);
		insertGivenControlPoints(model_given, given_intersection_vectors);


		/*	for (int i = 0; i < index_intersection_vectors.size(); i++) {
			intersection_vectors.push_back(m_model.getVertices()[index_intersection_vectors[i]]);
		}
		for (int i = 0; i < index_intersection_vectors.size(); i++) {
			given_intersection_vectors.push_back(model_given.getVertices()[index_intersection_vectors[i]]);
		}

		/*
		for (int i = 0; i < index_given_intersection_vectors.size(); i++) {
			given_intersection_vectors.push_back(model_given.getVertices()[index_given_intersection_vectors[i]]);
		}*/


		/*_____triangulation of control points______*/
		//triangulateControlPoints(intersection_vectors, my_triangles_ref);
		//secondTriangulation(intersection_vectors, my_triangles_ref);
		thirdTriangulation(intersection_vectors, my_triangles_ref);

		//triangulateControlPoints(given_intersection_vectors, given_triangles_ref);
		//secondTriangulation(given_intersection_vectors, given_triangles_ref);
		thirdTriangulation(given_intersection_vectors, given_triangles_ref);


		///////////////////////////////////////////////////////////triangulation EXTRA CONTROL POINTS/////////////////////////////////////////////////////////////////////
		////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		if (EXTRA_CONTROL_POINTS) {
			insertMyExtraControlPoints(m_model, my_extra);
			insertGivenExtraControlPoints(model_given, given_extra);
		}

		/*
		for (int i = 0; i < my_extra.size(); i++) {
			index_intersection_vectors.push_back(my_extra[i]);
			intersection_vectors.push_back(m_model.getVertices()[my_extra[i]]);
			triangulationEXTRAControlPoints(m_model.getVertices()[my_extra[i]], intersection_vectors, triangles3d);
		}
		for (int i = 0; i < my_extra.size(); i++) {
			index_intersection_vectors.push_back(my_extra[i]);
			given_intersection_vectors.push_back(model_given.getVertices()[my_extra[i]]);
			triangulationEXTRAControlPoints(model_given.getVertices()[my_extra[i]], given_intersection_vectors, given_triangles3d);
		}



	/*for (int i = 0; i <given_extra.size(); i++) {
			index_given_intersection_vectors.push_back(given_extra[i]);

			given_intersection_vectors.push_back(model_given.getVertices()[given_extra[i]]);
			triangulationEXTRAControlPoints(model_given.getVertices()[given_extra[i]], given_intersection_vectors, given_triangles3d);
		}*/
		
		/*___apply baricentric interpolation_____*/

		if (second_2_question) {
			//barycentricInterpolation(m_model.getVertices(), model_given.getVertices(), intersection_vectors, given_intersection_vectors);
		}

		/*_________metatwpisi control points _____*/

		if (second_3_question) {
			//translationOfControlPoints(new_model, model_given_ref, model_given);


			//insertMyControlPoints(new_model, intersection_vectors_new);
			//thirdTriangulation(intersection_vectors_new, my_triangles_new);
			
			///////secondTriangulation(intersection_vectors_new, my_triangles_new);
			///////triangulateControlPoints(intersection_vectors_new, my_triangles_new);

			if (EXTRA_CONTROL_POINTS) {
				for (int i = 0; i < my_extra.size(); i++) {
					intersection_vectors_new.push_back(new_model.getVertices()[my_extra[i]]);
					triangulationEXTRAControlPoints(new_model.getVertices()[my_extra[i]], intersection_vectors_new, my_triangles_new);
				}
			}
		}

		
		/*_____metatwpisi non_control points_____*/
		//translationOfNONControlPoints(new_model, m_model);
	

}

void Mesh3DScene::mousePressed(int x, int y, int modif)
{

	Scene::mousePressed(x, y, modif);
	point_test.x = x;
	point_test.y = y;

	Ray ray(unproject(x, y));
	std::vector<vec> &myverts = m_model.getVertices();
	std::vector<vec> &givenverts = model_given.getVertices();
	std::vector<vvr::Triangle> &myTriangles=m_model.getTriangles();
	std::vector<vvr::Triangle> &givenTriangles = model_given.getTriangles();

	math::LineSegment linesegment=ray.ToLineSegment(200);

	
	//////////////////////////////////////////////////////////////////////////////diko mas montelo/////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int j = 0; j < myTriangles.size(); j++) {
		Plane myplane = Plane(myTriangles[j].v1(), myTriangles[j].v2(), myTriangles[j].v3());

		vec a = linesegment.a;
		vec b = linesegment.b;
		pointa = Point3D(a.x, a.y, a.z, Colour::red);
		pointb = Point3D(b.x, b.y, b.z, Colour::blue);

		vec point_pl = myTriangles[j].v1();
		vec l_1 = b-a;
		vec dif_p0_l0_1 = point_pl-a;

		double d1 = dif_p0_l0_1.Dot(myplane.normal) / l_1.Dot(myplane.normal);
		vec inter_point_1 = d1*l_1 + a;

		///////*simeia tomis*/////
		vec vtest1 = myplane.normal.Cross(myTriangles[j].v2() -	myTriangles[j].v1());
		vec vtest2 = myplane.normal.Cross(myTriangles[j].v3() - myTriangles[j].v2());
		vec vtest3 = myplane.normal.Cross(myTriangles[j].v1() - myTriangles[j].v3());

		if ((vtest1.Dot(inter_point_1 - myTriangles[j].v1()) > 0.0f)&&(vtest2.Dot(inter_point_1 - myTriangles[j].v2()) > 0.0f)&&(vtest3.Dot(inter_point_1 - myTriangles[j].v3()) > 0.0f)) {

			//point = Point3D(inter_point_1.x, inter_point_1.y, inter_point_1.z, Colour::darkGreen);
			//intersection_vectors.push_back(point);

			double length_1 = inter_point_1.DistanceSq(myTriangles[j].v1());
			double length_2 = inter_point_1.DistanceSq(myTriangles[j].v2());
			double length_3 = inter_point_1.DistanceSq(myTriangles[j].v3());
			vector <double> length;
			length.push_back(length_1);
			length.push_back(length_2);
			length.push_back(length_3);
			vector<int> min_ind;
			min_ind.push_back(0);
			min_ind.push_back(1);
			min_ind.push_back(2);

			//std::sort(length.begin(), length.end(), minobject);
			std::sort(min_ind.begin(), min_ind.end(), [&](int& i1, int& i2) { return length[i1] < length[i2]; });

			if (min_ind[0] == 0) {
				inter_point_1 = myTriangles[j].v1();
				//point = Point3D(inter_point_1.x, inter_point_1.y, inter_point_1.z, Colour::darkGreen);
				intersection_vectors.push_back(inter_point_1);
				index_intersection_vectors.push_back(myTriangles[j].vi1);
				//triangulationEXTRAControlPoints(inter_point_1, intersection_vectors, triangles3d);
				//outFile << myTriangles[j].vi1 << endl;
				std::cout << "to simeio einai:" << myTriangles[j].vi1;
				//std::cout << "to trigwno einai einai:" << j;
				//myTriangles.erase(myTriangles.begin() + j);
				//m_model.update();
				triangulationEXTRAControlPoints(m_model.getVertices()[myTriangles[j].vi1], intersection_vectors, my_triangles_ref);

				break;
			}
			else if (min_ind[0] == 1) {
				inter_point_1 = myTriangles[j].v2();
				//point = Point3D(inter_point_1.x, inter_point_1.y, inter_point_1.z, Colour::darkGreen);
				intersection_vectors.push_back(inter_point_1);
				index_intersection_vectors.push_back(myTriangles[j].vi2);
				//outFile << myTriangles[j].vi2 << endl;
				//triangulationEXTRAControlPoints(inter_point_1, intersection_vectors, triangles3d);
				std::cout << "to simeio einai:" << myTriangles[j].vi2;
				//std::cout << "to trigwno einai einai:" << j;
				//myTriangles.erase(myTriangles.begin() + j);
				triangulationEXTRAControlPoints(m_model.getVertices()[myTriangles[j].vi2], intersection_vectors, my_triangles_ref);

				break;
			}
			else if (min_ind[0] == 2) {
				inter_point_1 = myTriangles[j].v3();
				//point = Point3D(inter_point_1.x, inter_point_1.y, inter_point_1.z, Colour::darkGreen);
				intersection_vectors.push_back(inter_point_1);
				index_intersection_vectors.push_back(myTriangles[j].vi3);
				//outFile << myTriangles[j].vi3 << endl;
				//triangulationEXTRAControlPoints(inter_point_1, intersection_vectors, triangles3d);
				std::cout << "to simeio einai:" << myTriangles[j].vi3;
				//std::cout << "to trigwno einai einai:" << j;

				//myTriangles.erase(myTriangles.begin() + j);
				//m_model.update();
				triangulationEXTRAControlPoints(m_model.getVertices()[myTriangles[j].vi3], intersection_vectors, my_triangles_ref);

				break;
			}
		}

		
	}


	///////////////////////////////////////////////////////////////////////////////given model///////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int j = 0; j < givenTriangles.size(); j++) {
		Plane myplane = Plane(givenTriangles[j].v1(), givenTriangles[j].v2(), givenTriangles[j].v3());

		vec a = linesegment.a;
		vec b = linesegment.b;
		pointa = Point3D(a.x, a.y, a.z, Colour::red);
		pointb = Point3D(b.x, b.y, b.z, Colour::blue);
		//intersection_vectors.push_back(pointa);
		//intersection_vectors.push_back(pointb);




		//vec point_pl = myplane.PointOnPlane();
		vec point_pl = givenTriangles[j].v1();
		//Point3D pointplane = Point3D(myplane.normal.x, myplane.normal.y, myplane.normal.z, Colour::yellow);
		//intersection_vectors.push_back(pointplane);
		vec l_1 = b - a;


		vec dif_p0_l0_1 = point_pl - a;

		double d1 = dif_p0_l0_1.Dot(myplane.normal) / l_1.Dot(myplane.normal);
		vec inter_point_1 = d1*l_1 + a;

		///////*simeia tomis*/////
		vec vtest1 = myplane.normal.Cross(givenTriangles[j].v2() - givenTriangles[j].v1());
		vec vtest2 = myplane.normal.Cross(givenTriangles[j].v3() - givenTriangles[j].v2());
		vec vtest3 = myplane.normal.Cross(givenTriangles[j].v1() - givenTriangles[j].v3());

		if ((vtest1.Dot(inter_point_1 - givenTriangles[j].v1()) > 0.0f) && (vtest2.Dot(inter_point_1 - givenTriangles[j].v2()) > 0.0f) && (vtest3.Dot(inter_point_1 - givenTriangles[j].v3()) > 0.0f)) {

			//point = Point3D(inter_point_1.x, inter_point_1.y, inter_point_1.z, Colour::darkGreen);
			//intersection_vectors.push_back(point);

			double length_1 = inter_point_1.DistanceSq(givenTriangles[j].v1());
			double length_2 = inter_point_1.DistanceSq(givenTriangles[j].v2());
			double length_3 = inter_point_1.DistanceSq(givenTriangles[j].v3());
			vector <double> length;
			length.push_back(length_1);
			length.push_back(length_2);
			length.push_back(length_3);
			vector<int> min_ind;
			min_ind.push_back(0);
			min_ind.push_back(1);
			min_ind.push_back(2);

			//std::sort(length.begin(), length.end(), minobject);
			std::sort(min_ind.begin(), min_ind.end(), [&](int& i1, int& i2) { return length[i1] < length[i2]; });

			if (min_ind[0] == 0) {
				inter_point_1 = givenTriangles[j].v1();
				//point = Point3D(inter_point_1.x, inter_point_1.y, inter_point_1.z, Colour::orange);
				given_intersection_vectors.push_back(inter_point_1);
				index_given_intersection_vectors.push_back(givenTriangles[j].vi1);
				//outFileGiven << givenTriangles[j].vi1 << endl;
				std::cout << "to simeio einai:" << givenTriangles[j].vi1;
				triangulationEXTRAControlPoints(inter_point_1, given_intersection_vectors, given_triangles_ref);

				break;
			}
			else if (min_ind[0] == 1) {
				inter_point_1 = givenTriangles[j].v2();
				//point = Point3D(inter_point_1.x, inter_point_1.y, inter_point_1.z, Colour::orange);
				given_intersection_vectors.push_back(inter_point_1);
				index_given_intersection_vectors.push_back(givenTriangles[j].vi2);
				//outFileGiven << givenTriangles[j].vi2 << endl;
				std::cout << "to simeio einai:" << givenTriangles[j].vi2;
				triangulationEXTRAControlPoints(inter_point_1, given_intersection_vectors, given_triangles_ref);

				break;
			}
			else if (min_ind[0] == 2) {
				inter_point_1 = givenTriangles[j].v3();
				//point = Point3D(inter_point_1.x, inter_point_1.y, inter_point_1.z, Colour::orange);
				given_intersection_vectors.push_back(inter_point_1);
				index_given_intersection_vectors.push_back(givenTriangles[j].vi3);
				//outFileGiven << givenTriangles[j].vi3 << endl;
				std::cout << "to simeio einai:" << givenTriangles[j].vi3;
				triangulationEXTRAControlPoints(inter_point_1, given_intersection_vectors, given_triangles_ref);

				break;
			}
		}


	}
	
	
	
	
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////apply baricentric interpolation///////////////////////////////////////////////
/*	if (intersection_vectors.size() == 50) {
		triangulateControlPoints(intersection_vectors, triangles3d);
	}*/
	
	/*	if (intersection_vectors.size() ==32) {
			//for (int m = 0; m < intersection_vectors.size() - 2; m++)
			//{
			//	vvr::Triangle tri(&intersection_vectors, intersection_vectors.size() - 1, m, m + 1);
				//triangles3d.push_back(tri);
			//}
			//vvr::Triangle tri(&intersection_vectors, intersection_vectors.size() - 1, intersection_vectors.size() - 2, 0);
			//triangles3d.push_back(tri);


			////writting basic control points///
			for (int i = 0; i<index_intersection_vectors.size(); i++) {
				outFile << index_intersection_vectors[i] << endl;
			}
			outFile.close();*/
			//triangulateControlPoints(intersection_vectors, triangles3d);

		//}
	/*	else if (intersection_vectors.size() > 10) {
			barycentricInterpolation(myverts, givenverts, intersection_vectors, given_intersection_vectors);

			////sosososososososos//////////////
			//triangulateControlPoints(intersection_vectors,triangles3d);


		}*//*
		if (given_intersection_vectors.size() == 49) {
			for (int m = 0; m < given_intersection_vectors.size() - 2; m++)
			{
				vvr::Triangle tri(&given_intersection_vectors, given_intersection_vectors.size() - 1, m, m + 1);
				given_triangles3d.push_back(tri);
				Triangle3D tri1(tri.v1().x, tri.v1().y, tri.v1().z, tri.v2().x, tri.v2().y, tri.v2().z, tri.v3().x, tri.v3().y, tri.v3().z, vvr::Colour::blue);
				tri1.draw();
			}
			vvr::Triangle tri(&given_intersection_vectors, given_intersection_vectors.size() - 1, given_intersection_vectors.size() - 2, 0);
			given_triangles3d.push_back(tri);
			Triangle3D tri1(tri.v1().x, tri.v1().y, tri.v1().z, tri.v2().x, tri.v2().y, tri.v2().z, tri.v3().x, tri.v3().y, tri.v3().z, vvr::Colour::blue);
			tri1.draw();

			////////writing to file////
		/*	for (int i = 0; i<index_given_intersection_vectors.size(); i++) {
			outFile << index_given_intersection_vectors[i] << endl;
			}
			outFile.close();
			*/
	//	}*/
	
	/*	else if (given_intersection_vectors.size() > 10) {
			//barycentricInterpolation(myverts, givenverts, intersection_vectors, given_intersection_vectors);
			triangulateControlPoints(given_intersection_vectors,given_triangles3d);



		}*/
	}
	
void Mesh3DScene::arrowEvent(ArrowDir dir, int modif)
{
    math::vec n = m_plane.normal;
    if (dir == UP) m_plane_d += 1;
    if (dir == DOWN) m_plane_d -= 1;
    else if (dir == LEFT) n = math::float3x3::RotateY(DegToRad(1)).Transform(n);
    else if (dir == RIGHT) n = math::float3x3::RotateY(DegToRad(-1)).Transform(n);
    m_plane = Plane(n.Normalized(), m_plane_d);
	if (SPLIT_INSTEAD_OF_INTERSECT == false) {
		m_intersections.clear();
		Task_5_Intersect(m_model.getTriangles(), m_plane, m_intersections);
	}
	else {
		m_model = Mesh(m_model_original);
		//Task_5_Split(m_model, m_plane);
	}

}

void Mesh3DScene::keyEvent(unsigned char key, bool up, int modif)
{
    Scene::keyEvent(key, up, modif);
    key = tolower(key);

	switch (key)
	{

		
			/*my buttons for weights*/

	case '0':if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0.1, 0); break; }
			 else { m_model_original = differentialMethodOfMakingNewBlendshapes(0.1, 0); break; }

	case '1':if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0.1, 1); break; }
			 else { m_model_original = differentialMethodOfMakingNewBlendshapes(0.1, 1); break; }

	case '2': if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0.1, 2); break; }
			  else { m_model_original = differentialMethodOfMakingNewBlendshapes(0.1, 2); break; }

	case '3': if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0.1, 3); break; }
			  else { m_model_original = differentialMethodOfMakingNewBlendshapes(0.1, 3); break; }

	case '4': if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0.1, 4); break; }
			  else { m_model_original = differentialMethodOfMakingNewBlendshapes(0.1, 4); break; }

	case '5': if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0.1, 5); break; }
			  else { m_model_original = differentialMethodOfMakingNewBlendshapes(0.1, 5); break; }

	case '6': if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0.1, 6); break; }
			  else { m_model_original = differentialMethodOfMakingNewBlendshapes(0.1, 6); break; }

	case '7': if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0.1, 7); break; }
			  else { m_model_original = differentialMethodOfMakingNewBlendshapes(0.1, 7); break; }

	case '8':if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0.1, 8); break; }
			 else { m_model_original = differentialMethodOfMakingNewBlendshapes(0.1, 8); break; }

	case '9': if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0.1, 9); break; }
			  else { m_model_original = differentialMethodOfMakingNewBlendshapes(0.1, 9); break; }



		/* project's buttons for model*/
	case 's': m_style_flag ^= FLAG_SHOW_SOLID; break;
	case 'w': m_style_flag ^= FLAG_SHOW_WIRE; break;
	case 'n': m_style_flag ^= FLAG_SHOW_NORMALS; break;
	case 'a': m_style_flag ^= FLAG_SHOW_AXES; break;
	case 'p': m_style_flag ^= FLAG_SHOW_PLANE; break;
	case 'b': m_style_flag ^= FLAG_SHOW_AABB; break;

		/*my buttons for model-triangles-control points*/
	case 'm': m_style_flag ^= FLAG_SHOW_MY_REFERENCE; break;
	case 'x': m_style_flag ^= FLAG_SHOW_MY_NEW; break;
	case 'l': m_style_flag ^= FLAG_SHOW_GIVEN; break;
	case 't': m_style_flag ^= FLAG_SHOW_MY_NEW_TRIANGLES; break;
	case 'o': m_style_flag ^= FLAG_SHOW_MY_PAST_TRIANGLES; break;
	case 'r': m_style_flag ^= SHOW_INTERSECTION_POINTS; break;
	case 'z':m_style_flag ^= FLAG_SHOW_GIVEN_REF; break;
	
		/*question buttons*/

	case 'j':if (third_question) {
		cloning = true;
		if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0, 0); break; }
		else { m_model_original = differentialMethodOfMakingNewBlendshapes(0, 0); break; }
	}

	case 'h': if (third_question) {
		cloning = false;
		if (Linear) { m_model_original = linearMethodOfMakingBlendshapes(0, 0); break; }
		else { m_model_original = differentialMethodOfMakingNewBlendshapes(0, 0); break; }

	}

	case 'q': {
		if (second_question) { barycentricInterpolation(m_model.getVertices(), model_given.getVertices(), intersection_vectors, given_intersection_vectors); break; }
	}



	case 'c': {
		if (second_3_question) {
			translationOfControlPoints(new_model, model_given, model_given_ref); 
			if (!second_2_registration) {
				insertMyControlPoints(new_model, intersection_vectors_new);
				thirdTriangulation(intersection_vectors_new, my_triangles_new);
				double extra_control = intersection_vectors_new.size() - 47;
				std::cout << intersection_vectors_new.size();
				for (int i = intersection_vectors.size() - extra_control; i < intersection_vectors.size(); i++) {
					triangulationEXTRAControlPoints(new_model.getVertices()[index_intersection_vectors[i]], intersection_vectors_new, my_triangles_new);
				}
				translationOfNONControlPoints(new_model, m_model); break;
			}
		}
	}
	
	case 'u': {

		 	MatrixXd Vmatr = Eigen::MatrixXd::Zero(10, 1);
			MatrixXd Wmatr = Eigen::MatrixXd::Zero(10, 1);

			MatrixXd Ematr_inv= Eigen::MatrixXd::Zero(10, 10);
			Ematr_inv = Ematrix.inverse();
			

			for (int i = 0; i < numb_blend; i++) {
				Wmatr(ind_blend[i], 0) = wk[i];
			}
			std::cout << "\n\n\n\n Wmatrix:\n";
			for (int i = 0; i < 10; i++) {
				std::cout << Wmatr(i);
				std::cout << "\n";
			}
				
			Vmatr = Ematr_inv*Wmatr;
			std::cout << "\n\n Vmatrix:\n";
			for (int i = 0; i < 10; i++) {
				std::cout << Vmatr(i);
				std::cout << "\n";
			}
			
			/*apo to b0 kai meta*/
			string objDir5 = getBasePath() + "resources/my_base/";
			std::vector<string> StringMatrix;
			string objFile5;
			StringMatrix.push_back("my_anger.obj");
			StringMatrix.push_back("my_cry.obj");
			StringMatrix.push_back("my_fury.obj");
			StringMatrix.push_back("my_grin.obj");
			StringMatrix.push_back("my_laugh.obj");
			StringMatrix.push_back("my_rage.obj");
			StringMatrix.push_back("my_sad.obj");
			StringMatrix.push_back("my_smile.obj");
			StringMatrix.push_back("my_surprised.obj");
			StringMatrix.push_back("personal_blendshape.obj");

			objDir5 = getBasePath() + "resources/my_base/";
			objFile5 = objDir5 + "my_grin.obj";
			
			/*______arxikopoiisi new_model______*/
			vvr::Mesh my_new_model = vvr::Mesh(objFile5);
			vector < vec> &my_new_vertices = my_new_model.getVertices();

			vec v0(0, 0, 0);
			for (int c = 0; c < my_new_vertices.size(); c++) {

				my_new_vertices[c] = v0;

			}

			/*______linear method__________*/
			for (int i = 0; i < 10; i++) {
				const string objFile = objDir5 + StringMatrix[i];
				vvr::Mesh model;
				model = vvr::Mesh(objFile);
				vector < vec> &new_vertices = model.getVertices();
				std::cout << "\n loading:" << StringMatrix[i];
								
				if (i < 9) {
					for (int j = 0; j < my_new_vertices.size(); j++) {
						new_vertices[j] = math::float3(-new_vertices[j].x-30, new_vertices[j].y, new_vertices[j].z);
					}					
				}				
				if (i == 9) {
					for (int j = 0; j < my_new_vertices.size(); j++) {
						new_vertices[j] = math::float3(new_vertices[j].x, new_vertices[j].z, -new_vertices[j].y);
					}
				}

				model.setBigSize(getSceneWidth() / 4);
				model.update();

				for (int j = 0; j < new_vertices.size(); j++) {
					new_vertices[j] = (Vmatr(i))*(new_vertices[j]);
					my_new_vertices[j] = my_new_vertices[j] + new_vertices[j];
				}

			}
			
			
			new_model = my_new_model;
			new_model.setBigSize(getSceneWidth() / 4);
			new_model.update();

			//!//////////////////////////////////////////////////////////////////////////////////
			//! new_model
			//!//////////////////////////////////////////////////////////////////////////////////

			Task_1_FindCenterMass(new_model.getVertices(), m_center_mass);
			vec my_center_new(m_center_mass.x, m_center_mass.y, m_center_mass.z);
			Task_3_AlignOriginTo(new_model.getVertices(), my_center_new);
			Task_2_FindAABB(new_model.getVertices(), m_aabb);
			break;
			  }

		      }
	
}

void Mesh3DScene::draw()
{

	//! Draw plane
	if (m_style_flag & FLAG_SHOW_PLANE) {
		vvr::Colour colPlane(0x41, 0x14, 0xB3);
		float u = 20, v = 20;
		math::vec p0(m_plane.Point(-u, -v, math::vec(0, 0, 0)));
		math::vec p1(m_plane.Point(-u, v, math::vec(0, 0, 0)));
		math::vec p2(m_plane.Point(u, -v, math::vec(0, 0, 0)));
		math::vec p3(m_plane.Point(u, v, math::vec(0, 0, 0)));
		math2vvr(math::Triangle(p0, p1, p2), colPlane).draw();
		math2vvr(math::Triangle(p2, p1, p3), colPlane).draw();

	}
	if (m_style_flag & FLAG_SHOW_MY_REFERENCE) {
		Task_1_FindCenterMass(m_model_original.getVertices(), m_center_mass);
		vec my_center(m_center_mass.x + 30, m_center_mass.y, m_center_mass.z);
		Task_3_AlignOriginTo(m_model_original.getVertices(), my_center);
		Task_2_FindAABB(m_model_original.getVertices(), m_aabb);
		if (m_style_flag & FLAG_SHOW_SOLID) m_model_original.draw(m_obj_col, SOLID);
		if (m_style_flag & FLAG_SHOW_WIRE) m_model_original.draw(Colour::black, WIRE);
		if (m_style_flag & FLAG_SHOW_NORMALS) m_model_original.draw(Colour::black, NORMALS);
		if (m_style_flag & FLAG_SHOW_AXES) m_model_original.draw(Colour::black, AXES);

	}

	if (m_style_flag & FLAG_SHOW_GIVEN_REF) {
		if (m_style_flag & FLAG_SHOW_SOLID) model_given.draw(m_obj_col, SOLID);
		if (m_style_flag & FLAG_SHOW_WIRE) model_given.draw(Colour::black, WIRE);
		if (m_style_flag & FLAG_SHOW_NORMALS) model_given.draw(Colour::black, NORMALS);
		if (m_style_flag & FLAG_SHOW_AXES) model_given.draw(Colour::black, AXES);

		//if (m_style_flag & FLAG_SHOW_SOLID) model_given_ref.draw(m_obj_col, SOLID);
		//if (m_style_flag & FLAG_SHOW_WIRE) new_model.draw(m_obj_col, SOLID);

		given_aabb.setColour(Colour::black);
		given_aabb.setTransparency(1);
		given_aabb.draw();
	}
	if (m_style_flag & FLAG_SHOW_MY_NEW) {
		/*if (third_question) {
			//!//////////////////////////////////////////////////////////////////////////////////
			//! new_model
			//!//////////////////////////////////////////////////////////////////////////////////
		
			Task_1_FindCenterMass(new_model.getVertices(), m_center_mass);
			vec my_center_new(m_center_mass.x , m_center_mass.y, m_center_mass.z);
			Task_3_AlignOriginTo(new_model.getVertices(), my_center_new);
			Task_2_FindAABB(new_model.getVertices(), m_aabb);
		}*/
		if (m_style_flag & FLAG_SHOW_SOLID) new_model.draw(m_obj_col, SOLID);
		if (m_style_flag & FLAG_SHOW_WIRE) new_model.draw(Colour::black, WIRE);
		if (m_style_flag & FLAG_SHOW_NORMALS) new_model.draw(Colour::black, NORMALS);
		if (m_style_flag & FLAG_SHOW_AXES) new_model.draw(Colour::black, AXES);
	}
		
	if (m_style_flag & FLAG_SHOW_GIVEN) {
	
		if (m_style_flag & FLAG_SHOW_SOLID) model_given_ref.draw(m_obj_col, SOLID);
		if (m_style_flag & FLAG_SHOW_WIRE) model_given_ref.draw(Colour::black, WIRE);
		if (m_style_flag & FLAG_SHOW_NORMALS) model_given_ref.draw(Colour::black, NORMALS);
		if (m_style_flag & FLAG_SHOW_AXES) model_given_ref.draw(Colour::black, AXES);
	}

	//! Draw center mass
	Point3D(m_center_mass.x, m_center_mass.y, m_center_mass.z, Colour::red).draw();

	//! Draw AABB
	if (m_style_flag & FLAG_SHOW_AABB) {
		m_aabb.setColour(Colour::black);
		m_aabb.setTransparency(1);
		m_aabb.draw();
	}



	vector<vec> temp_my_vert = m_model_original.getVertices();
	vector<vec> temp_given_vert = model_given.getVertices();

	/*____draw intersection/control points_____*/
	if (m_style_flag & SHOW_INTERSECTION_POINTS) {
		for (int k = 0; k < intersection_vectors.size(); k++) {
			Point3D inter_point(intersection_vectors[k].x, intersection_vectors[k].y, intersection_vectors[k].z, Colour::green);
			inter_point.draw();

		}
		/*for (int i = 0; i < index_intersection_vectors.size(); i++) {
		Point3D inter_point(temp_my_vert[index_intersection_vectors[i]].x, temp_my_vert[index_intersection_vectors[i]].y, temp_my_vert[index_intersection_vectors[i]].z, Colour::green);
		inter_point.draw();
		}

		for (int i = 0; i < index_given_intersection_vectors.size(); i++) {
		Point3D inter_point(temp_given_vert[index_given_intersection_vectors[i]].x, temp_given_vert[index_given_intersection_vectors[i]].y, temp_given_vert[index_given_intersection_vectors[i]].z, Colour::orange);
		inter_point.draw();
		}

		*/
		for (int k = 0; k < given_intersection_vectors.size(); k++) {
			Point3D inter_point(given_intersection_vectors[k].x, given_intersection_vectors[k].y, given_intersection_vectors[k].z, Colour::orange);
			inter_point.draw();

		}
	}

	/*	for (int k = 0; k < intersection_vectors_new.size(); k++) {
	Point3D inter_point(intersection_vectors_new[k].x, intersection_vectors_new[k].y, intersection_vectors_new[k].z, Colour::magenta);
	inter_point.draw();

	}*/
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	////////    draw extra control points /////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*	for (int k = 6; k < 10; k++) {

	Point3D inter_point(vectors_my_extra[k].x, vectors_my_extra[k].y, vectors_my_extra[k].z, Colour::blue);
	inter_point.draw();

	}


	for (int k = 6; k < 10;k++){

	Point3D inter_point(vectors_given_extra[k].x, vectors_given_extra[k].y, vectors_given_extra[k].z, Colour::yellow);
	inter_point.draw();

	}*/


	/*____draw registered points____*/

	if (second_2_question && !second_3_question) {

		for (int reg = 0; reg < regist_vertices.size(); reg++) {
			Point3D inter_point(regist_vertices[reg].x, regist_vertices[reg].y, regist_vertices[reg].z, Colour::magenta);
			inter_point.draw();
		}

		for (int reg = 0; reg < my_regist_vertices.size(); reg++) {
			Point3D inter_point(my_regist_vertices[reg].x, my_regist_vertices[reg].y, my_regist_vertices[reg].z, Colour::white);
			inter_point.draw();
		}
	}



	std::vector<vvr::Colour> Colours;
	Colours.push_back(vvr::Colour::green);
	Colours.push_back(vvr::Colour::white);
	Colours.push_back(vvr::Colour::red);
	Colours.push_back(vvr::Colour::orange);
	Colours.push_back(vvr::Colour::black);
	Colours.push_back(vvr::Colour::magenta);
	Colours.push_back(vvr::Colour::darkGreen);

	///////////////////////////////////////////draw triangulation triangles/////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	if (m_style_flag & FLAG_SHOW_MY_PAST_TRIANGLES) {
		int i = 0;
		for (int n = 0; n < my_triangles_ref.size(); n++) {

			vvr::Triangle tri_1(&intersection_vectors, my_triangles_ref[n].vi1, my_triangles_ref[n].vi2, my_triangles_ref[n].vi3);
			Triangle3D tr1(tri_1.v1().x, tri_1.v1().y, tri_1.v1().z, tri_1.v2().x, tri_1.v2().y, tri_1.v2().z, tri_1.v3().x, tri_1.v3().y, tri_1.v3().z, Colours[i]);
			tr1.draw();
			i++;
			if (i == 7) {
				i = 0;
			}
		}

		int h = 0;

		for (int n = 0; n < given_triangles_ref.size(); n++) {

			vvr::Triangle tri_1(&given_intersection_vectors, given_triangles_ref[n].vi1, given_triangles_ref[n].vi2, given_triangles_ref[n].vi3);
			Triangle3D tr1(tri_1.v1().x, tri_1.v1().y, tri_1.v1().z, tri_1.v2().x, tri_1.v2().y, tri_1.v2().z, tri_1.v3().x, tri_1.v3().y, tri_1.v3().z, Colours[h]);
			tr1.draw();
			h++;
			if (h == 7) {
				h = 0;
			}
		}

	}

	if (m_style_flag & FLAG_SHOW_MY_NEW_TRIANGLES) {
		int i = 0;
		if (my_triangles_new.size() > 3) {
			for (int n = 0; n < my_triangles_new.size(); n++) {

				vvr::Triangle tri_1(&intersection_vectors_new, my_triangles_new[n].vi1, my_triangles_new[n].vi2, my_triangles_new[n].vi3);
				Triangle3D tr1(tri_1.v1().x, tri_1.v1().y, tri_1.v1().z, tri_1.v2().x, tri_1.v2().y, tri_1.v2().z, tri_1.v3().x, tri_1.v3().y, tri_1.v3().z, Colours[i]);
				tr1.draw();
				i++;
				if (i == 7) {
					i = 0;
				}
			}
		}
	}


	/////////////////////////////////////////////////draw outside points///////////////////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	/*for (int i = 0; i < outvertices.size(); i++) {
	Point3D inter_point(outvertices[i].x, outvertices[i].y, outvertices[i].z,vvr::Colour::yellowGreen);
	inter_point.draw();

	}*/



	/*
	for (int pr = 0; pr < projectedvertices.size(); pr++) {
	Point3D p(projectedvertices[pr].x, projectedvertices[pr].y, projectedvertices[pr].z, vvr::Colour::yellow);
	p.draw();
	}*/

	/*for (int f = 0; f < aereavertices.size(); f++) {
	Point3D p(aereavertices[f].x, aereavertices[f].y, aereavertices[f].z,vvr::Colour::yellow);
	p.draw();
	}
	/*for (int l = 0; l < registertriangles.size(); l++) {
	Triangle3D tr1(registertriangles[l].v1().x, registertriangles[l].v1().y, registertriangles[l].v1().z, registertriangles[l].v2().x, registertriangles[l].v2().y,
	registertriangles[l].v2().z, registertriangles[l].v3().x, registertriangles[l].v3().y, registertriangles[l].v3().z,vvr::Colour:: blue);
	tr1.draw();
	}*/
	/////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	//////////////////pairnw apostaseis apo ta duo simeia/////////////////////////////////////

	/*
	Colour colours[5] = { Colour::white,Colour::blue,Colour::green,Colour::red,Colour::yellow, };
	int g = 0;
	if (intersection_vectors.size() == 5 && given_intersection_vectors.size() == 5) {
	for (int j = 0; j < intersection_vectors.size(); j++) {
	for (int i = 0; i < given_intersection_vectors.size(); i++) {

	float dist_nose = intersection_vectors[j].DistanceSq(temp_my_vert[final_my_face_indices[0]]);
	float dist_chin = intersection_vectors[j].DistanceSq(temp_my_vert[final_my_face_indices[1]]);
	float chin_nose = temp_my_vert[final_my_face_indices[1]].DistanceSq(temp_my_vert[final_my_face_indices[0]]);

	float dist_nose_given = intersection_vectors[j].DistanceSq(temp_my_vert[final_my_face_indices[0]]);
	float dist_chin_given = given_intersection_vectors[i].DistanceSq(temp_given_vert[final_given_face_indices[1]]);
	float chin_nose_given = temp_given_vert[final_given_face_indices[1]].DistanceSq(temp_given_vert[final_given_face_indices[0]]);

	float error_nose = (dist_nose - dist_nose_given)/ chin_nose;
	float error_chin = (dist_chin - dist_chin_given)/ chin_nose_given;
	if (error_nose < 0.5) {
	if (error_chin < 0.5) {
	Point3D inter_point1(intersection_vectors[i].x, intersection_vectors[i].y, intersection_vectors[i].z, colours[g]);
	inter_point1.draw();

	Point3D inter_point2(given_intersection_vectors[i].x, given_intersection_vectors[i].y, given_intersection_vectors[i].z,colours[g] );
	inter_point2.draw();
	g++;
	}
	}
	}

	}
	}*/



	//Task_5_Split(m_model, m_plane, final_my_face_indices);

}


/*___________insert/load control points and meshes__________________*/
Mesh Mesh3DScene::loadMyFace(string my_string) {
	
	const string objDir = getBasePath() + "resources/obj/";
	const string objFile = objDir + my_string;
	model_test = vvr::Mesh(objFile);
	std::vector<vec> &vertices = model_test.getVertices();
	for (int j = 0; j < vertices.size(); j++) {

		vertices[j] = math::float3(vertices[j].x, vertices[j].z, -vertices[j].y);
	}
	return model_test;
}
Mesh Mesh3DScene::loadGivenFace( string given_string) {
	
	const string objDir = getBasePath() + "resources/obj/";
	const string objFile_given = objDir + given_string;
	model_test_given = vvr::Mesh(objFile_given);
	return model_test_given;
}

void Mesh3DScene::insertMyControlPoints(vvr::Mesh my_mesh, std::vector<vec> &cntrl_points) {

	for (int i = 0; i < index_intersection_vectors.size(); i++) {
		cntrl_points.push_back(my_mesh.getVertices()[index_intersection_vectors[i]]);
	}
}
void Mesh3DScene::insertGivenControlPoints(vvr::Mesh given_mesh, std::vector<vec> &cntrl_points) {
	
	for (int i = 0; i < index_given_intersection_vectors.size(); i++) {
		cntrl_points.push_back(given_mesh.getVertices()[index_given_intersection_vectors[i]]);
	}
}

void Mesh3DScene::insertMyExtraControlPoints(vvr::Mesh my_model,std::vector<int> my_extra_indices) {
	for (int i = 0; i < my_extra_indices.size(); i++) {
		index_intersection_vectors.push_back(my_extra_indices[i]);
		intersection_vectors.push_back(my_model.getVertices()[my_extra_indices[i]]);
		triangulationEXTRAControlPoints(my_model.getVertices()[my_extra_indices[i]], intersection_vectors, my_triangles_ref);
	}
}
void Mesh3DScene::insertGivenExtraControlPoints(vvr::Mesh given_model, std::vector<int> given_extra_indices) {

	for (int i = 0; i <given_extra_indices.size(); i++) {
		index_given_intersection_vectors.push_back(given_extra_indices[i]);
		given_intersection_vectors.push_back(given_model.getVertices()[given_extra_indices[i]]);
		triangulationEXTRAControlPoints(given_model.getVertices()[given_extra_indices[i]], given_intersection_vectors, given_triangles_ref);
	}
}









/*__________Second Question________________*/

void Mesh3DScene::barycentricInterpolation(std::vector<vec> my_vertices, std::vector<vec> model_vertices, std::vector<vec>& my_matched_vertices, std::vector<vec>& model_matched_vertices) {


		int cnt = my_triangles_ref.size();
		int times = 0;
		
		for (int pi = 100; pi <300; pi++) {
		
			for (int g = 0; g < cnt; g++) {


			
				vec P_principal = my_vertices[pi];
						
				//my_regist_vertices.push_back(P);
			
				vec P1 = my_triangles_ref[g].v1();			
				vec P2 = my_triangles_ref[g].v2();
				vec P3 = my_triangles_ref[g].v3();

				vec u = P2 - P1;
				vec v = P3 - P1;
				vec n = u.Cross(v);
				vec w = P_principal - P1;

				double gama = ((u.Cross(w)).Dot(n)) / (n.Dot(n));
				double bita = ((w.Cross(v)).Dot(n)) / (n.Dot(n));
				double alpha = 1 - gama - bita;

				vec projected(alpha*P1 + bita*P2 + gama*P3);
				float distance = P_principal.DistanceSq(projected);
				if (distance < 8) {

					if (alpha > -0.1 && alpha < 1.1 && bita>-0.1 && bita < 1.1 && gama>-0.1 && gama < 1.1) {


					//////////////////////////////periorismos sto psaximo simeiwn//////////////
				/*	vector<vec> sort_y;
					sort_y.push_back(given_triangles3d[g].v1());
					sort_y.push_back(given_triangles3d[g].v2());
					sort_y.push_back(given_triangles3d[g].v3());
					std::sort(sort_y.begin(), sort_y.end(), myobject_y);

					vector<vec> sort_x;
					sort_x.push_back(given_triangles3d[g].v1());
					sort_x.push_back(given_triangles3d[g].v2());
					sort_x.push_back(given_triangles3d[g].v3());
					std::sort(sort_x.begin(), sort_x.end(), myobject_x);*/
					/////////////////////////////////////////////////////////////////////////

					for (int mi = 0; mi < model_vertices.size(); mi++) {

						//if (model_vertices[mi].y <sort_y[0].y && model_vertices[mi].y >sort_y[2].y && model_vertices[mi].x > sort_x[2].x && model_vertices[mi].x < sort_x[0].x) {
							//aereavertices.push_back(model_vertices[mi]);

						vec P = model_vertices[mi];
						vec P1 = given_triangles_ref[g].v1();
						vec P2 = given_triangles_ref[g].v2();
						vec P3 = given_triangles_ref[g].v3();

						vec u = P2 - P1;
						vec v = P3 - P1;
						vec n = u.Cross(v);
						vec w = P - P1;

						double gama_model = ((u.Cross(w)).Dot(n)) / (n.Dot(n));
						double bita_model = ((w.Cross(v)).Dot(n)) / (n.Dot(n));
						double alpha_model = 1 - gama - bita;

						//vec projected(alpha*P1 + bita*P2 + gama*P3);
						if (alpha_model > -0.1 && alpha_model < 1.1 && bita_model>-0.1 && bita_model < 1.1 && gama_model>-0.1 && gama_model < 1.1) {

							if (abs(alpha - alpha_model) < 0.1  && abs(bita - bita_model) < 0.1  && abs(gama - gama_model) < 0.1) {

								vec projected(alpha_model*P1 + bita_model*P2 + gama_model*P3);

								float distance = P.DistanceSq(projected);
								if (distance < 5) {


									my_registered_indices.push_back(pi);
									given_registered_indices.push_back(mi);

									my_regist_vertices.push_back(P_principal);
									regist_vertices.push_back(model_vertices[mi]);
									break;
								}
							}
						}
						//}
					}
					break;
		

				}
			}
	}
}

	//LineSeg3D line(projected.x, projected.y, projected.z, my_vertices[0].x, my_vertices[0].y, my_vertices[0].z);
	//line.draw();
	//Point3D proj(projected.x, projected.y, projected.z);
	//proj.setColour(vvr::Colour::white);
	//proj.draw();
	////na elegxw an anikei sta my_matched_vertices

	/////////////////thelw na vrw 3 geitonika simeia sto vertex mou-tha mporousa kai me kd tree///////////////
	////vriskw tis apostaseis apo ola ta vertices poy uparxoun sto matched kai taxinomw kai pairnw ta tria prwta
	vector<double> distance_sort;
	vector<int> distance_index_sort;
/*/	
	/////estw oti pairnw gia to prwto mono////
	for (int i = 0; i < my_matched_vertices.size(); i++) {
		float dist = my_vertices[0].DistanceSq(my_matched_vertices[i]);
		distance_sort.push_back(dist);
		distance_index_sort.push_back(i);
	}
	std::sort(distance_index_sort.begin(), distance_index_sort.end(), [&](int& i1, int& i2) { return distance_sort[i1] < distance_sort[i2]; });

	/*vvr::Triangle tri1(my_matched_vertices[distance_index_sort[0]].x, my_matched_vertices[distance_index_sort[0]].y, my_matched_vertices[distance_index_sort[0]].z,
		my_matched_vertices[distance_index_sort[1]].x, my_matched_vertices[distance_index_sort[1]].y, my_matched_vertices[distance_index_sort[1]].z,
		my_matched_vertices[distance_index_sort[2]].x, my_matched_vertices[distance_index_sort[2]].y, my_matched_vertices[distance_index_sort[2]].z);*/
	
	/*vvr::Triangle tri(&my_matched_vertices, distance_index_sort[0], distance_index_sort[1], distance_index_sort[2]);
	
	Triangle3D tri1(tri.v1().x, tri.v1().y, tri.v1().z, tri.v2().x, tri.v2().y, tri.v2().z, tri.v3().x, tri.v3().y, tri.v3().z);
	tri1.draw();*/
	//tri1.draw();
	
	/*Point3D test_barycentric(my_vertices[0].x, my_vertices[0].y, my_vertices[0].z);
	test_barycentric.draw();*/
	//(my_matched_vertices[distance_index_sort[0]], my_matched_vertices[distance_index_sort[1]], my_matched_vertices[distance_index_sort[2]]);
	
}

void Mesh3DScene::translationOfControlPoints(vvr::Mesh &mesh, vvr::Mesh given_ref, vvr::Mesh given_other) {

	/*______CONTROL POINTS_______*/
	if (!second_2_registration) {
		std::vector<vec> tr_other_vert = given_other.getVertices();
		std::vector<vec> tr_ref_vert = given_ref.getVertices();
		std::vector<vec> &tr_my_other_vert = mesh.getVertices();
		for (int i = 0; i < index_given_intersection_vectors.size(); i++) {

			double dist_x = tr_ref_vert[index_given_intersection_vectors[i]].x - tr_other_vert[index_given_intersection_vectors[i]].x;
			double dist_y = tr_ref_vert[index_given_intersection_vectors[i]].y - tr_other_vert[index_given_intersection_vectors[i]].y;
			double dist_z = tr_ref_vert[index_given_intersection_vectors[i]].z - tr_other_vert[index_given_intersection_vectors[i]].z;

			if (dist_x > 0) {
				tr_my_other_vert[index_intersection_vectors[i]].x = tr_my_other_vert[index_intersection_vectors[i]].x - abs(dist_x);
			}
			if (dist_x < 0) {
				tr_my_other_vert[index_intersection_vectors[i]].x = tr_my_other_vert[index_intersection_vectors[i]].x + abs(dist_x);

			}
			if (dist_y > 0) {
				tr_my_other_vert[index_intersection_vectors[i]].y = tr_my_other_vert[index_intersection_vectors[i]].y - abs(dist_y);

			}
			if (dist_y < 0) {
				tr_my_other_vert[index_intersection_vectors[i]].y = tr_my_other_vert[index_intersection_vectors[i]].y + abs(dist_y);

			}
			if (dist_z > 0) {
				tr_my_other_vert[index_intersection_vectors[i]].z = tr_my_other_vert[index_intersection_vectors[i]].z - abs(dist_z);

			}
			if (dist_z < 0) {
				tr_my_other_vert[index_intersection_vectors[i]].z = tr_my_other_vert[index_intersection_vectors[i]].z + abs(dist_z);

			}

		}

	}

	/*_______________________________Laplacian Method_____________________________*/
/*	std::vector<vec> vector_extended;
	/*	for (int i = 0; i < tr_my_other_vert.size(); i++) {

		vector_extended.push_back(tr_my_other_vert[i]);
	}*/
/*	
	vec v0(0, 0, 0);
	for (int i = 0; i < tr_my_other_vert.size(); i++) {

		vector_extended.push_back(v0);
	}
	

	for (int j = 0; j < index_intersection_vectors.size(); j++) {

		vector_extended[index_intersection_vectors[j]] = tr_my_other_vert[index_intersection_vectors[j]];
	}
	std::cout << "eftiaxa to extended";


	SparseMatrix<double> laplacian(tr_my_other_vert.size(), tr_my_other_vert.size());
	laplacian =Mesh3DScene:: laplacianMeshnig(mesh);

	SparseMatrix<double> laplacian_extended(tr_my_other_vert.size(), tr_my_other_vert.size());
	///////////////LAPLACIAN EXTENDED//////////////////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////
	for (int i=0; i<index_intersection_vectors.size(); i++) {
		for (int j = 0; j < index_intersection_vectors.size(); j++) {
			laplacian_extended.coeffRef(i, j) = laplacian.coeff(i,j);
		}
	}

	for (int m = 0; m < index_intersection_vectors.size(); m++) {
		for (int s = 0; s < tr_my_other_vert.size(); s++) {
			laplacian_extended.coeffRef(index_intersection_vectors[m], s) = 0;
		}
		laplacian_extended.coeffRef(index_intersection_vectors[m], index_intersection_vectors[m]) = 1.0;
	}
	
	//////////______((L'L)^-1)*L'______/////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////

	SparseMatrix<double> L_trans(tr_my_other_vert.size(), tr_my_other_vert.size());

	L_trans = laplacian_extended.transpose();

	//std::cout << "\neftiaxa ton transpose";
	SparseMatrix<double> B(tr_my_other_vert.size(), tr_my_other_vert.size());
	B = L_trans*laplacian_extended;
	SparseMatrix<double> Lt_L(tr_my_other_vert.size(), tr_my_other_vert.size());
	
	SparseLU<SparseMatrix<double> > solver;
	solver.compute(B);
	SparseMatrix<double> I(tr_my_other_vert.size(), tr_my_other_vert.size());
	I.setIdentity();
	Lt_L = solver.solve(I);

	std::cout << "\nekana antistrofo";

	SparseMatrix<double> FINAL(tr_my_other_vert.size(), tr_my_other_vert.size());

	FINAL = Lt_L*L_trans;
	


	for (int k = 0; k < tr_my_other_vert.size(); k++) {
		vec in(0, 0, 0);
		for (int f = 0; f < tr_my_other_vert.size(); f++) {
			in = in + FINAL.coeff(k, f)*vector_extended[f];
		}
		tr_my_other_vert[k] = in;
	}


	//mesh.update();
	//mesh.exportToObj("sad_laplacian_control.obj");
	*/
	


 /*___________________ALL REGISTERED POINTS________________________*/
	else{
		std::vector<vvr::Triangle> my_other_tr = mesh.getTriangles();
		std::vector<vec> tr_other_vert = given_other.getVertices();
		std::vector<vec> tr_ref_vert = given_ref.getVertices();
		std::vector<vec> &tr_my_other_vert = mesh.getVertices();
		std::vector<vec> starting_vert = mesh.getVertices();

		for (int i = 0; i < given_registered_indices.size(); i++) {

			double dist_x = tr_ref_vert[given_registered_indices[i]].x - tr_other_vert[given_registered_indices[i]].x;
			double dist_y = tr_ref_vert[given_registered_indices[i]].y - tr_other_vert[given_registered_indices[i]].y;
			double dist_z = tr_ref_vert[given_registered_indices[i]].z - tr_other_vert[given_registered_indices[i]].z;

			if (dist_x > 0) {
				tr_my_other_vert[my_registered_indices[i]].x = tr_my_other_vert[my_registered_indices[i]].x - abs(dist_x);
			}
			if (dist_x < 0) {
				tr_my_other_vert[my_registered_indices[i]].x = tr_my_other_vert[my_registered_indices[i]].x + abs(dist_x);

			}
			if (dist_y > 0) {
				tr_my_other_vert[my_registered_indices[i]].y = tr_my_other_vert[my_registered_indices[i]].y - abs(dist_y);

			}
			if (dist_y < 0) {
				tr_my_other_vert[my_registered_indices[i]].y = tr_my_other_vert[my_registered_indices[i]].y + abs(dist_y);

			}
			if (dist_z > 0) {
				tr_my_other_vert[my_registered_indices[i]].z = tr_my_other_vert[my_registered_indices[i]].z - abs(dist_z);

			}
			if (dist_z < 0) {
				tr_my_other_vert[my_registered_indices[i]].z = tr_my_other_vert[my_registered_indices[i]].z + abs(dist_z);

			}

		}


	}
}

void Mesh3DScene::translationOfNONControlPoints(vvr::Mesh &mesh, vvr::Mesh principal_mesh) {
	bool flag = false;
	bool check=true;
	std::vector<vec> principal_vertices = principal_mesh.getVertices();
	std::vector<vec> &final_vertices = mesh.getVertices();
	float best_distance;
	float bes_distance_viol;
	float alpha_best=0;
	float bita_best=0;
	float gama_best=0;
	float alpha_viol = 0;
	float bita_viol = 0;
	float gama_viol = 0;
	int best_tr=0;
	int best_viol_tr = 0;
	int cnt = my_triangles_ref.size();
	int p = 0;
	bool flag_viol;


	for (int pi = 0; pi < final_vertices.size(); pi++) {
		bes_distance_viol = 30;
		best_distance =4.0;
		flag_viol = false;
		flag = false;
		check = true;
		for (int ch = 0; ch < index_intersection_vectors.size(); ch++) {
			if (pi == index_intersection_vectors[ch]) {
				check=false;
				break;
			}
		}
		if (check) {
			for (int g = 0; g < my_triangles_ref.size(); g++) {
				
				vec P = principal_vertices[pi];

				vec P1 = my_triangles_ref[g].v1();
				vec P2 = my_triangles_ref[g].v2();
				vec P3 = my_triangles_ref[g].v3();

				vec u = P2 - P1;
				vec v = P3 - P1;
				vec n = u.Cross(v);
				vec w = P - P1;
				float gama =float( ((u.Cross(w)).Dot(n))) /float( (n.Dot(n)));
				float bita =float( ((w.Cross(v)).Dot(n))) /float( (n.Dot(n)));
				float alpha = 1 - gama - bita;

				vec projected(alpha*P1 + bita*P2 + gama*P3);

			
				//double distance = double(P.DistanceSq(projected));


				if (alpha > -0.1 && alpha < 1.1 && bita>-0.1 && bita < 1.1 && gama>-0.1 && gama < 1.1) {

						vec projected_final(alpha*my_triangles_new[g].v1() + bita*my_triangles_new[g].v2() + gama*my_triangles_new[g].v3());
						vec P_final = final_vertices[pi];
						//double distance =double( P_final.DistanceSq(projected_final));
						//double distance = P_final.DistanceSq(my_triangles_new[g].getCenter());
						double distance = P.DistanceSq(projected);

						if (sqrt(distance) < best_distance) {

							best_distance = sqrt(distance);

							alpha_best = alpha;
							bita_best = bita;
							gama_best = gama;
							best_tr = g;
							flag = true;

						}
				}
				
			}
			

			if (flag) {
				final_vertices[pi] = alpha_best*my_triangles_new[best_tr].v1() + bita_best*my_triangles_new[best_tr].v2() + gama_best*my_triangles_new[best_tr].v3();
				flag = true;
			}
			
			if (flag == false) {

				for (int viol = 0; viol < my_triangles_ref.size(); viol++) {
					vec P = principal_vertices[pi];

					vec P1 = my_triangles_ref[viol].v1();
					vec P2 = my_triangles_ref[viol].v2();
					vec P3 = my_triangles_ref[viol].v3();

					vec u = P2 - P1;
					vec v = P3 - P1;
					vec n = u.Cross(v);
					vec w = P - P1;
					float gama = float(((u.Cross(w)).Dot(n))) / float((n.Dot(n)));
					float bita = float(((w.Cross(v)).Dot(n))) / float((n.Dot(n)));
					float alpha = 1 - gama - bita;

					vec projected_violated(alpha*P1 + bita*P2 + gama*P3);

					vec projected_final_violated(alpha*my_triangles_new[viol].v1() + bita*my_triangles_new[viol].v2() + gama*my_triangles_new[viol].v3());
					vec P_final_violated = final_vertices[pi];

					//double distance = double(P_final_violated.DistanceSq(projected_final_violated));
					double distance = P_final_violated.DistanceSq(my_triangles_new[viol].getCenter());

					if (sqrt(distance) < bes_distance_viol) {

						bes_distance_viol = sqrt(distance);
						alpha_viol = alpha;
						bita_viol = bita;
						gama_viol = gama;
						best_viol_tr =viol;
						flag_viol = true;

					}
				}
				if (flag_viol) {
					final_vertices[pi]= alpha_viol*my_triangles_new[best_viol_tr].v1() + bita_viol*my_triangles_new[best_viol_tr].v2() + gama_viol*my_triangles_new[best_viol_tr].v3();
					outvertices.push_back(final_vertices[pi]);

				}
	
			}
		}
	}
	mesh.exportToObj("my_sad.obj");

}


/*____test for laplacian matrix_____*/
SparseMatrix<double> Mesh3DScene::laplacianMeshnig(vvr::Mesh mesh) {

	std::vector<vvr::Triangle> laplacian_triangles = mesh.getTriangles();
	std::vector<vec> laplacian_vertices = mesh.getVertices();
	

	////////////////////////////A-Matrix/////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////	
	//MatrixXd A = Eigen::MatrixXd::Zero(laplacian_vertices.size(), laplacian_vertices.size());
	SparseMatrix<double> A(laplacian_vertices.size(), laplacian_vertices.size());
	//std::cout << A;
		for (int i = 0; i <laplacian_triangles.size(); i++) {
			A.coeffRef(laplacian_triangles[i].vi1, laplacian_triangles[i].vi2) = 1.0;
			A.coeffRef(laplacian_triangles[i].vi2, laplacian_triangles[i].vi1) = 1.0;
			A.coeffRef(laplacian_triangles[i].vi1, laplacian_triangles[i].vi3) = 1.0;
			A.coeffRef(laplacian_triangles[i].vi3, laplacian_triangles[i].vi1) = 1.0;
			A.coeffRef(laplacian_triangles[i].vi2, laplacian_triangles[i].vi3) = 1.0;
			A.coeffRef(laplacian_triangles[i].vi3, laplacian_triangles[i].vi2) = 1.0;

		/*	A(laplacian_triangles[i].vi1,laplacian_triangles[i].vi2) = 1.0;
			A(laplacian_triangles[i].vi2, laplacian_triangles[i].vi1) = 1.0;
			A(laplacian_triangles[i].vi1, laplacian_triangles[i].vi3) = 1.0;
			A(laplacian_triangles[i].vi3, laplacian_triangles[i].vi1) = 1.0;
			A(laplacian_triangles[i].vi2, laplacian_triangles[i].vi3) = 1.0;
			A(laplacian_triangles[i].vi3, laplacian_triangles[i].vi2) = 1.0;*/

		}
		for (int i = 0; i <laplacian_vertices.size(); i++) {
			A.coeffRef(i,i) = 1.0;
		
		}


	std::cout << "\neftiaxa ton A";

	

	////////////////////////////D-Matrix/////////////////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////
	
	//MatrixXd D = Eigen::MatrixXd::Zero(laplacian_vertices.size(), laplacian_vertices.size());
	SparseMatrix<double> D(laplacian_vertices.size(), laplacian_vertices.size());
	D.reserve(laplacian_vertices.size());
	for (int i = 0; i < laplacian_vertices.size(); i++) {
		double d = 0;
		for (int j = 0; j < laplacian_vertices.size(); j++) {
			d = d+A.coeff(i, j);
		}
		D.coeffRef(i, i) = d;
	}
	std::cout << "\neftiaxa ton D";
	
	////////////////////////////D_INVERSE-Matrix/////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	SparseMatrix<double> D_inv;
/*	PaStiXSupport <SparseMatrix<double> > solver;
	solver.compute(D);
	SparseMatrix<double> I(laplacian_vertices.size(), laplacian_vertices.size());
	I.setIdentity();
	D_inv = solver.solve(I);

	//D_inv = D.cwiseInverse();
	std::cout << "\neftiaxa ton D ANTISTROFO"; 

	////////////////////////////D_INVERSE*A -Matrix/////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////

	SparseMatrix<double> MULTIPLICATION(laplacian_vertices.size(), laplacian_vertices.size());

	//MatrixXd mult = Eigen::MatrixXd::Zero(laplacian_vertices.size(), laplacian_vertices.size());
	MULTIPLICATION = D_inv*A;
	std::cout << "\neftiaxa ton mult";
	
	

	//SparseMatrix<double> I(laplacian_vertices.size(), laplacian_vertices.size());
	//MatrixXd I = Eigen::MatrixXd::Ones(1, laplacian_vertices.size());
//	I.diagonal();

	////////////////////////////LAPLACIAN -Matrix/////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////////
	*/
	SparseMatrix<double> L(laplacian_vertices.size(), laplacian_vertices.size());

	//L = I - MULTIPLICATION;
	std::cout << "\neftiaxa ton Laplacian";

	return L;
	
}




/*__________Third Question__________*/
Eigen::MatrixXd  Mesh3DScene::calculateEmatrix() {


	MatrixXd E_MATRIX = Eigen::MatrixXd::Zero(10, 10);;

	MatrixXd W_MATRIX = Eigen::MatrixXd::Zero(10, 11);;
	MatrixXd V_MATRIX = Eigen::MatrixXd::Zero(10, 11);

	/*___W_MATRIX_____*/
	
	W_MATRIX(0, 2) = 0.63;
	W_MATRIX(0, 4) = 0.1;
	W_MATRIX(0, 6) = 0.5;

	W_MATRIX(1, 0) = 0.4;
	W_MATRIX(1, 2) = 0.11;
	W_MATRIX(1, 7) = 0.3;

	W_MATRIX(2, 0) = 0.15;
	W_MATRIX(2, 1) = 0.05;
	W_MATRIX(2, 6) = 0.5;

	W_MATRIX(3, 0) = 0.15;
	W_MATRIX(3, 1) = 0.05;
	W_MATRIX(3, 5) = 0.2;
	W_MATRIX(3, 10) = 0.8;

	W_MATRIX(4, 0) = 0.15;
	W_MATRIX(4, 3) = 0.135;
	W_MATRIX(4, 5) = 0.8;
	W_MATRIX(4, 8) = 0.68;
	W_MATRIX(4, 9) = 0.7;


	W_MATRIX(5, 0) = 0.15;
	W_MATRIX(5,1) = 0.05;
	W_MATRIX(5, 2) = 0.26;
	
	W_MATRIX(6, 4) = 0.9;
	W_MATRIX(6, 7) = 0.7;
	W_MATRIX(6, 9) = 0.3;


	W_MATRIX(7, 1) = 0.85;
	W_MATRIX(7, 3) = 0.135;
	W_MATRIX(7, 8) = 0.191;
	W_MATRIX(7, 10) = 0.2;
	W_MATRIX(8, 3) = 0.73;
	W_MATRIX(9, 8) = 0.191;

	std::cout << "\n";
	std::cout << W_MATRIX;
	std::cout << "\n";

	/*___V_MATRIX_____*/

	V_MATRIX(0, 2) = 0.73;
	V_MATRIX(0, 4) = 0.17;
	V_MATRIX(0, 6) = 0.37;
	V_MATRIX(0, 9) = 0.168;



	V_MATRIX(1, 0) = 0.06179;
	V_MATRIX(1, 1) = 0.0044;
	V_MATRIX(1, 2) = 0.022;
	V_MATRIX(1, 3) = 0.03;
	V_MATRIX(1, 4) = 0.1;
	V_MATRIX(1, 6) = 0.31;
	V_MATRIX(1, 7) = 0.18;



	V_MATRIX(2, 0) = 0.06179;
	V_MATRIX(2, 1) = 0.0044;
	V_MATRIX(2, 2) = 0.2;
	V_MATRIX(2, 3) = 0.03;
	V_MATRIX(2, 4) = 0.002;
	V_MATRIX(2, 6) = 0.20;
	V_MATRIX(2, 7) = 0.05;



	V_MATRIX(3, 0) = 0.225;
	V_MATRIX(3, 1) = 0.026;
	V_MATRIX(3, 2) = 0.022;
	V_MATRIX(3, 3) = 0.1546;
	V_MATRIX(3, 5) = 0.09;
	V_MATRIX(3, 8) = 0.21;
	V_MATRIX(3, 10) = 0.23;



	V_MATRIX(4, 0) = 0.4658;
	V_MATRIX(4, 1) = 0.088;
	V_MATRIX(4, 2) = 0.022;
	V_MATRIX(4, 5) = 0.54;
	V_MATRIX(4, 8) = 0.21;
	V_MATRIX(4, 9) = 0.613;
	V_MATRIX(4, 10) = 0.4;


	V_MATRIX(5, 0) = 0.06179;
	V_MATRIX(5, 1) = 0.0044;
	V_MATRIX(5, 5) = 0.09;
	V_MATRIX(5, 7) = 0.06;

	V_MATRIX(6, 0) = 0.06179;
	V_MATRIX(6, 1) = 0.0044;
	V_MATRIX(6, 3) = 0.03;
	V_MATRIX(6, 4) = 0.72;
	V_MATRIX(6, 6) = 0.19;
	V_MATRIX(6, 7) = 0.69;
	V_MATRIX(6, 9) = 0.2185;

	V_MATRIX(7, 0) = 0.06179;
	V_MATRIX(7, 1) = 0.86;
	V_MATRIX(7, 3) = 0.03;
	V_MATRIX(7, 5) = 0.09;
	V_MATRIX(7, 8) = 0.388;
	V_MATRIX(7, 10) = 0.36;

	V_MATRIX(8, 1) = 0.0044;
	V_MATRIX(8, 3) = 0.72;
	V_MATRIX(8, 5) = 0.18;
	V_MATRIX(8, 8) = 0.181;

	V_MATRIX(9, 1) = 0.0044;


	std::cout << "\n";
	std::cout << V_MATRIX;
	std::cout << "\n";

	//std::cout << "\nThe matrix is:" << V_MATRIX;


	/*V-Transpose*/
	MatrixXd V_Transp = Eigen::MatrixXd::Zero(11,10);
	V_Transp = V_MATRIX.transpose();


	/*__(V*V_transp)*____*/

	MatrixXd V_V_Tr = Eigen::MatrixXd::Zero(10, 10);
	V_V_Tr = V_MATRIX*V_Transp;

	/*inverse*/
	MatrixXd inver = Eigen::MatrixXd::Zero(10, 10);

	inver = V_V_Tr.inverse();

	MatrixXd E_prim = Eigen::MatrixXd::Zero(11, 10);

	E_prim = V_Transp*inver;
	E_MATRIX = W_MATRIX*E_prim;
	
	//std::cout << E_MATRIX;
	return E_MATRIX;
}








/*_____________________________Making New Blendshapes_________________________*/

/*________________Linear________________*/
vvr::Mesh linearMethodOfMakingBlendshapes(double w, int ind) {
	std::cout << "\n_________LINEAR________";
	std::vector<string> StringMatrix;
	string objDir5;
	string objFile5;
	if (first_question || (third_question && cloning)) {
		StringMatrix.push_back("face-01-anger.obj");
		StringMatrix.push_back("face-02-cry.obj");
		StringMatrix.push_back("face-03-fury.obj");
		StringMatrix.push_back("face-04-grin.obj");
		StringMatrix.push_back("face-05-laugh.obj");
		StringMatrix.push_back("face-06-rage.obj");
		StringMatrix.push_back("face-07-sad.obj");
		StringMatrix.push_back("face-08-smile.obj");
		StringMatrix.push_back("face-09-surprise.obj");
		StringMatrix.push_back("face-reference.obj");

		objDir5 = getBasePath() + "resources/obj/";
		objFile5 = objDir5 + "face-05-laugh.obj";

	}
	else if(second_4_question ||(third_question && !cloning)) {
		StringMatrix.push_back("my_anger.obj");
		StringMatrix.push_back("my_cry.obj");
		StringMatrix.push_back("my_fury.obj");
		StringMatrix.push_back("my_grin.obj");
		StringMatrix.push_back("my_laugh.obj");
		StringMatrix.push_back("my_rage.obj");
		StringMatrix.push_back("my_sad.obj");
		StringMatrix.push_back("my_smile.obj");
		StringMatrix.push_back("my_surprised.obj");
		StringMatrix.push_back("personal_blendshape.obj");

		objDir5 = getBasePath() + "resources/my_base/";
		objFile5 = objDir5 + "my_grin.obj";

	}
	
	
	
	/*_______specific selection of faces to blend_____*/
	std::vector<string> BlendStrings;
	for (int i = 0; i < ind_blend.size(); i++) {

		BlendStrings.push_back ( StringMatrix[ind_blend[i]]);
	}
	
	
	
	///////*arxikopoiisi new_model*////////////
	vvr::Mesh new_model = vvr::Mesh(objFile5);
	vector < vec> &my_new_vertices = new_model.getVertices();

	vec v0(0, 0, 0);
	for (int c = 0; c < my_new_vertices.size(); c++) {

		my_new_vertices[c] = v0;

	}

	///////*arxika w*/////////////////
	
	if (w == 0) {
		double dw0 = 1.0 / double(numb_blend);
		for (int i = 0; i < numb_blend; i++) {

			weight[i] = dw0;
			if (third_question && cloning) {
				wk[i] = weight[i];
			}
			else if (third_question && !cloning) {

				vk[i]=weight[i];
			}
		}
	}

	///////*uplogismos neou w*///////
	double sum_other = 0;

	if (w != 0) {
		if ((ind < numb_blend) && (weight[ind] + w<1)) {
			
			sum_other = 1 - weight[ind];
			weight[ind] = weight[ind] + w;

			if (third_question && cloning) {
				wk[ind] = weight[ind];
			}
			else if (third_question && !cloning) {

				vk[ind] = weight[ind];
			}
			double left_w = 1.0 - weight[ind];
			
			//double dw = w / (numb_blend - 1);
			for (int i = 0; i < ind; i++) {
				weight[i] = weight[i] *left_w/ sum_other;
				if (third_question && cloning) {
					wk[i]=weight[i];
				}
				else if (third_question && !cloning) {

					vk[i]=weight[i];
				}
			}
			for (int i = ind + 1; i < numb_blend; i++) {
				weight[i] = weight[i] * left_w / sum_other;
				if (third_question && cloning) {
					wk[i] = weight[i];
				}
				else if (third_question && !cloning) {

					vk[i] = weight[i];
				}
			}

		}
	}
	for (int i = 0; i < numb_blend; i++) {
		std::cout << "\n weight" << weight[i];
	}
	if (third_question) {

		for (int i = 0; i < wk.size(); i++) {
			std::cout << "\n wk:" << wk[i];
		}
		for (int i = 0; i < wk.size(); i++) {
			std::cout << "\n vk:" << vk[i];
		}
	}

	/*apo to b0 kai meta*/
	for (int i = 0; i < numb_blend; i++) {
		//const string objFile = objDir5 + StringMatrix[i];
		const string objFile = objDir5 + BlendStrings[i];
		vvr::Mesh model;
		model = vvr::Mesh(objFile);
		vector < vec> &new_vertices = model.getVertices();
		//std::cout << "\n loading:" << StringMatrix[i];
		std::cout << "\n loading:" << BlendStrings[i];
		
		for (int j = 0; j < new_vertices.size(); j++) {
			new_vertices[j] = (weight[i])*(new_vertices[j] );
			my_new_vertices[j] = my_new_vertices[j] + new_vertices[j];
		}

	}
	return new_model;

}

vvr::Mesh linearMethodOfMakingNewBlendshapes() {
	string StringMatrix[10] = { "face-01-anger.obj","face-02-cry.obj","face-03-fury.obj","face-04-grin.obj","face-05-laugh.obj","face-06-rage.obj","face-07-sad.obj","face-08-smile.obj",
		"face-09-surprise.obj","face-reference.obj " };


	std::vector< double >final_bn(87897);

	for (int i = 0; i < 3; i++) {
		const string objDir = getBasePath() + "resources/obj/";
		const string objFile = objDir + StringMatrix[i];

		vector< double > double_vertice;
		vvr::Mesh model;
		model = vvr::Mesh(objFile);
		
		vector < vec> &new_vertices= model.getVertices();
		std::cout << "\n loading:" << StringMatrix[i];

		/*grafw tin mia suntetagmeni meta tin alli*/
		std::vector< double > bn(87897);
		int k = 0;
		int v = 0;
		while (k < 87897) {
			bn[k] = new_vertices[v].x;
			k++;
			bn[k] = new_vertices[v].y;
			k++;
			bn[k] = new_vertices[v].z;
			k++;
			v++;

		}
		/*pollaplasiazw me to varos */
		
		double weight_a = 0.5;
		double weight_c = 0.5;
		double weight_f = 0.5;
		double weight_g = 0.5;
		double weight_l = 0.5;
		double weight_r = 0.5;
		double weight_sd= 0.5;
		double weight_sm = 0.5;
		double weight_s = 0.5;
		double weight_rf = 0.5;
		double weight[10] = { weight_a,weight_c,weight_f ,weight_g,weight_l,weight_r,weight_sd,weight_sm,weight_s,weight_rf };
		

		for (int j = 0; j < bn.size(); j++) {
			bn[j] = (weight[i])*(bn[j]);
			final_bn[j] = final_bn[j] + bn[j];
		}

	}

	const string objDir = getBasePath() + "resources/obj/";
	const string objFile = objDir + "face-05-laugh.obj";
	
	vvr::Mesh new_model = vvr::Mesh(objFile);
	vector < vec> &my_new_vertices = new_model.getVertices();
	std::cout << "\n prwti timh twn vertices.x:" << my_new_vertices[0].x;
	std::cout << "\n prwti timh twn vertices.y:" << my_new_vertices[0].y;
	std::cout << "\n prwti timh twn vertices.z:" << my_new_vertices[0].z;
	int j = 0;
	int u = 0;
	
while (j < 87897) {

		my_new_vertices[u].x = final_bn[j];
		j++;
		my_new_vertices[u].y = final_bn[j];
		j++;
		my_new_vertices[u].z = final_bn[j];
		j++;
		u++;
	}
	std::cout << "\n telikh timh twn vertices.x:" << final_bn[0];
	std::cout << "\n telikh timh twn vertices.y:" << final_bn[1];
	std::cout << "\n telikh timh twn vertices.z:" << final_bn[2];
	


	new_model.update();
	vector < vec> &updated_vertices = new_model.getVertices();

	std::cout << "\n updated timh twn vertices.x:" << updated_vertices[0].x;
	std::cout << "\n updated timh twn vertices.y:" << updated_vertices[0].y;
	std::cout << "\n updated timh twn vertices.z:" << updated_vertices[0].z;
	
	return new_model;
}

/*_______________Differential__________*/
vvr::Mesh differentialMethodOfMakingNewBlendshapes(double w,int ind) {
	std::cout << "\n_________DIFFERENTIAL________";
	std::vector<string> StringMatrix;
	string objDir5;
	string objFile5;
	if (first_question ||(third_question && cloning)) {
		StringMatrix.push_back("face-01-anger.obj");
		StringMatrix.push_back("face-02-cry.obj");
		StringMatrix.push_back("face-03-fury.obj");
		StringMatrix.push_back("face-04-grin.obj");
		StringMatrix.push_back("face-05-laugh.obj");
		StringMatrix.push_back("face-06-rage.obj");
		StringMatrix.push_back("face-07-sad.obj");
		StringMatrix.push_back("face-08-smile.obj");
		StringMatrix.push_back("face-09-surprise.obj");
		StringMatrix.push_back("face-reference.obj");

		objDir5 = getBasePath() + "resources/obj/";
		objFile5 = objDir5 + "face-reference.obj";

	}
	else if(second_4_question || (third_question && !cloning)){
		StringMatrix.push_back("my_anger.obj");
		StringMatrix.push_back("my_cry.obj");
		StringMatrix.push_back("my_fury.obj");
		StringMatrix.push_back("my_grin.obj");
		StringMatrix.push_back("my_laugh.obj");
		StringMatrix.push_back("my_rage.obj");
		StringMatrix.push_back("my_sad.obj");
		StringMatrix.push_back("my_smile.obj");
		StringMatrix.push_back("my_surprised.obj");
		StringMatrix.push_back("personal_blendshape.obj");

		objDir5 = getBasePath() + "resources/my_base/";
		objFile5 = objDir5 + "personal_blendshape.obj";

	}


	/*_______specific selection of faces to blend_____*/
	std::vector<string> BlendStrings;
	for (int i = 0; i < ind_blend.size(); i++) {

		BlendStrings.push_back(StringMatrix[ind_blend[i]]);
	}
	

	/*gia to f=b0 arxikopoihsh*/
	const string objFile = objFile5;
	vvr::Mesh model;
	model = vvr::Mesh(objFile);

	vector < vec> &b0vertices = model.getVertices();


	vvr::Mesh new_model = vvr::Mesh(objFile5);
	vector < vec> &my_new_vertices = new_model.getVertices();


	for (int c = 0; c < my_new_vertices.size(); c++) {
		my_new_vertices[c] = b0vertices[c];

	}

	///////*arxika w*/////////////////
	if (w == 0) {
		double dw0 = 1.0 / double(numb_blend);
		for (int i = 0; i < numb_blend; i++) {

			weight[i] = dw0;
			if (third_question && cloning) {
				wk[i] = weight[i];
			}
			else if (third_question && !cloning) {

				vk[i] = weight[i];
			}
		}
	}

	///////*uplogismos neou w*///////
	double sum_other = 0;
	
	if (w != 0) {
		if ((ind < numb_blend) && (weight[ind] + w<1)) {
			
			sum_other = 1 - weight[ind];
			weight[ind] = weight[ind] + w;

			if (third_question && cloning) {
				wk[ind] = weight[ind];
			}
			else if (third_question && !cloning) {

				vk[ind] = weight[ind];
			}

			double left_w = 1.0 - weight[ind];
			//double dw = w / (numb_blend - 1);
			for (int i = 0; i < ind; i++) {
				weight[i] = weight[i] *left_w/sum_other;
				if (third_question && cloning) {
					wk[i] = weight[i];
				}
				else if (third_question && !cloning) {

					vk[i] = weight[i];
				}

			}
			for (int i = ind + 1; i < numb_blend; i++) {
				weight[i] = weight[i] * left_w / sum_other;
				if (third_question && cloning) {
					wk[i] = weight[i];
				}
				else if (third_question && !cloning) {

					vk[i] = weight[i];
				}

			}

		}
	}
	for (int i = 0; i < numb_blend; i++) {
		std::cout << "\n weight" << weight[i];
	}
	if (third_question) {

		for (int i = 0; i < wk.size(); i++) {
			std::cout << "\n wk:" << wk[i];
		}
		for (int i = 0; i < wk.size(); i++) {
			std::cout << "\n vk:" << vk[i];
		}
	}
	/*apo to b1 kai meta*/
	for (int i = 0; i < numb_blend; i++) {
		
		//const string objFile = objDir5 + StringMatrix[i];
		const string objFile = objDir5 + BlendStrings[i];

		vector<vvr::Triangle> triangles;
		vector< double > double_vertice;
		vvr::Mesh model;
		model = vvr::Mesh(objFile);
		vector < vec> &new_vertices = model.getVertices();
		//std::cout << "\n loading:" << StringMatrix[i];
		std::cout << "\n loading:" << BlendStrings[i];


		for (int j = 0; j < new_vertices.size(); j++) {
			new_vertices[j] = (weight[i])*(new_vertices[j]- b0vertices[j]);
			my_new_vertices[j] = my_new_vertices[j] + new_vertices[j];
		}

	}



	return new_model;
}









int main(int argc, char* argv[])
{
	try {
		if (first_question || second_4_question ||third_question) {


			std::cout << "How many faces do you want to blend?\n";
			cin >> numb_blend;

			std::cout << "\nChoose the faces you want to blend";
			int i = 0;
			while (i != numb_blend) {
				int x;
				cin >> x;
				ind_blend.push_back(x);
				i++;
			}
			for (int l = 0; l < numb_blend; l++) {
				std::cout << "\nyou chose :" << ind_blend[l];
		}
			
		}
		/*___read_control_points___*/
		if (second_question) {
			ifstream inFile;

			inFile.open("ta_telika_dika_mou.txt");
			while (inFile.eof() == false) {
				int x;
				inFile >> x;
				index_intersection_vectors.push_back(x);
			}
			inFile.close();
			inFile.open("ta_telika_given.txt");
			while (inFile.eof() == false) {
				int x;
				inFile >> x;
				index_given_intersection_vectors.push_back(x);
			}
			inFile.close();

			if (EXTRA_CONTROL_POINTS) {
				inFile.open("ta_telika_my_extra.txt");
				//while (inFile.eof() == false) {
				for (int i = 0; i < 6; i++) {
					int x;
					inFile >> x;
					//index_intersection_vectors.push_back(x);
					my_extra.push_back(x);

				}
				inFile.close();
				inFile.open("ta_telika_given_extra.txt");
				//while (inFile.eof() == false) {
				for (int i = 0; i < 6; i++) {

					int x;
					inFile >> x;
					//index_given_intersection_vectors.push_back(x);
					given_extra.push_back(x);
				}
				inFile.close();
			}
		}
		
		return vvr::mainLoop(argc, argv, new Mesh3DScene);
	}

    catch (std::string exc) {
        cerr << exc << endl;
        return 1;
    }
    catch (...)
    {

		cerr << "Unknown exception" << endl;
        return 1;
    }
}









/*_______Triangulation____________*/

void Mesh3DScene::triangulateControlPoints(std::vector<vec> &control_points, std::vector<vvr::Triangle>&tringls) {
	////////////////////////////////1h dokimi////////////////////////////////////////////////////////////////////////
	/*

	///////////////////////////////////////trigwnopoiisi twn periferiakwn control points//////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int m = 0; m < 16 - 1; m++)
	{
	vvr::Triangle tri(&control_points, 35 , m, m + 1);
	tringls.push_back(tri);

	}
	vvr::Triangle tri(&control_points, 35, 16 - 2, 0);
	tringls.push_back(tri);



	/////////////////////////////////////trigwnopoiisi twn upoloipwn control points///////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int size = control_points.size();
	for (int i = 17; i <18; i++) {
	if (i == 35) {
	i++;
	}
	/////////////////////////1os tropos----> na ta kanw project sto kathe trigwno/////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////

	/*	double mindist = 10000;
	int best_index = 0;
	int counter = tringls.size();
	for (int j = 0; j < counter; j++) {

	vec normal_point = tringls[j].getNormal();
	double eswteriko = (control_points[i] - tringls[j].v1()).Dot(normal_point);

	projected = control_points[i] - eswteriko*normal_point;

	double esa = (tringls[j].v1() - projected).Dot((tringls[j].v2() - projected));
	double rada = acos(esa / (((tringls[j].v1() - projected).Length())*(tringls[j].v2() - projected).Length()));
	double dega = RadToDeg(rada);

	double esb = (tringls[j].v2() - projected).Dot((tringls[j].v3() - projected));
	double radb = acos(esb / (((tringls[j].v2() - projected).Length())*(tringls[j].v3() - projected).Length()));
	double degb = RadToDeg(radb);

	double esc = (tringls[j].v3() - projected).Dot((tringls[j].v1() - projected));
	double radc = acos(esc / (((tringls[j].v3() - projected).Length())*(tringls[j].v1() - projected).Length()));
	double degc = RadToDeg(radc);

	if (359 < dega + degb + degc && dega + degb + degc < 361) {
	proj = Point3D(projected.x, projected.y, projected.z);
	proj.setColour(vvr::Colour::white);

	//count all distances from projected to vertices of triangles//
	double dist1 = abs(projected.DistanceSq(tringls[j].v1()));
	double dist2 = abs(projected.DistanceSq(tringls[j].v2()));
	double dist3 = abs(projected.DistanceSq(tringls[j].v3()));

	double sumdist = dist1 + dist2 + dist3;
	tringls.erase(tringls.begin() + j);
	if (sumdist < mindist) {
	mindist = sumdist;
	best_index = j;
	}
	}
	}*/
	/////////////////////////2os tropos----> na ta kanw project sto z=0///////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////
	/*	int counter = tringls.size();
	C2DPoint point(control_points[i].x , control_points[i].y);
	int best_index = 0;

	for (int j = 0; j < counter; j++) {
	C2DPoint *v1 = &C2DPoint(tringls[j].v1().x, tringls[j].v1().y);
	C2DPoint *v2 = &C2DPoint(tringls[j].v2().x, tringls[j].v2().y);
	C2DPoint *v3 = &C2DPoint(tringls[j].v3().x, tringls[j].v3().y);
	Tri tri2d(v1, v2, v3);
	//triangles2d.push_back(tri2d);
	if (tri2d.to_C2D().Contains(point)) {
	//best_index = j;
	vvr::Triangle tri_1(&control_points, tringls[j].vi1, tringls[j].vi2, i);
	vvr::Triangle tri_2(&control_points, tringls[j].vi2, tringls[j].vi3, i);
	vvr::Triangle tri_3(&control_points, tringls[j].vi1, tringls[j].vi3, i);

	tringls.erase(tringls.begin() + j);

	tringls.push_back(tri_1);
	tringls.push_back(tri_2);
	tringls.push_back(tri_3);

	}

	}

	////////////////////////////////////////koino teleiwma duo tropwn////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////
	/*		vvr::Triangle tri_1(&control_points, tringls[best_index].vi1, tringls[best_index].vi2, i);
	vvr::Triangle tri_2(&control_points, tringls[best_index].vi2, tringls[best_index].vi3, i);
	vvr::Triangle tri_3(&control_points, tringls[best_index].vi1, tringls[best_index].vi3, i);

	tringls.erase(tringls.begin() + best_index);

	tringls.push_back(tri_1);
	tringls.push_back(tri_2);
	tringls.push_back(tri_3);

	}*/

	///////////////////////////////////2h dokimi//////////////////////////////////////////////////////////
	////////////////////////////////arxiki trigwnopoisi///////////////////////////////////////////////
	//////////////////////////////////////eyes///////////////////////////////////////////////////////

	for (int m = 0; m < 2; m++) {

		vvr::Triangle tri(&control_points, 16, m, m + 1);
		tringls.push_back(tri);
	}


	vvr::Triangle t1(&control_points, 16, 2, 4);
	tringls.push_back(t1);
	vvr::Triangle t2(&control_points, 16, 6, 4);
	tringls.push_back(t2);
	vvr::Triangle t3(&control_points, 16, 6, 8);
	tringls.push_back(t3);

	for (int m = 8; m < 16 - 1; m++) {

		vvr::Triangle tri(&control_points, 16, m, m + 1);
		tringls.push_back(tri);
	}

	vvr::Triangle tri(&control_points, 16, 16 - 1, 0);
	tringls.push_back(tri);


	for (int m = 17; m < 24; m++) {

		vvr::Triangle tri(&control_points, 30, m, m + 1);
		tringls.push_back(tri);
	}

	vvr::Triangle t4(&control_points, 26, 24, 30);
	tringls.push_back(t4);
	vvr::Triangle t5(&control_points, 26, 28, 30);
	tringls.push_back(t5);


	vvr::Triangle tri1(&control_points, 17, 30, 0);
	tringls.push_back(tri1);
	vvr::Triangle tri2(&control_points, 28, 2, 30);
	tringls.push_back(tri2);
	vvr::Triangle tri3(&control_points, 2, 1, 30);
	tringls.push_back(tri3);
	vvr::Triangle tri4(&control_points, 1, 0, 30);
	tringls.push_back(tri4);

	////////////////////////////////nose///////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	vvr::Triangle tri5(&control_points, 2, 4, 43);
	tringls.push_back(tri5);
	//vvr::Triangle tri6(&control_points, 3, 4, 43);
	//tringls.push_back(tri6);
	vvr::Triangle tri7(&control_points, 4, 32, 43);
	tringls.push_back(tri7);

	for (int m = 32; m < 38; m++) {

		vvr::Triangle tri(&control_points, 43, m, m + 1);
		tringls.push_back(tri);
	}

	vvr::Triangle t6(&control_points, 38, 40, 43);
	tringls.push_back(t6);
	vvr::Triangle t7(&control_points, 40, 42, 43);
	tringls.push_back(t7);

	vvr::Triangle tri8(&control_points, 40, 28, 43);
	tringls.push_back(tri8);
	//vvr::Triangle tri9(&control_points, 28, 29, 43);
	//tringls.push_back(tri9);
	vvr::Triangle tri10(&control_points, 2, 28, 43);
	tringls.push_back(tri10);

	////////////////////////////cheeks////////////////////////////////////////////////
	/////////////////////////////left////////////////////////////////////////////////////

	vvr::Triangle tri11(&control_points, 4, 32, 55);
	tringls.push_back(tri11);
	//vvr::Triangle tri12(&control_points, 31, 32, 55);
	//tringls.push_back(tri12);
	vvr::Triangle tri13(&control_points, 32, 44, 55);
	tringls.push_back(tri13);

	vvr::Triangle t8(&control_points, 4, 6, 55);
	tringls.push_back(t8);
	vvr::Triangle t9(&control_points, 6, 8, 55);
	tringls.push_back(t9);


	for (int m = 8; m < 10; m++) {

		vvr::Triangle tri(&control_points, 55, m, m + 1);
		tringls.push_back(tri);
	}
	vvr::Triangle tri14(&control_points, 10, 54, 55);
	tringls.push_back(tri14);

	for (int m = 44; m < 54; m++) {

		vvr::Triangle tri(&control_points, 55, m, m + 1);
		tringls.push_back(tri);
	}
	/*	for (int m = 46; m < 56; m++) {

	vvr::Triangle tri(&control_points, 55, m, m + 1);
	//	tringls.push_back(tri);
	}*/
	//////////////////////right////////////////////////////////////////////////////////

	vvr::Triangle tri15(&control_points, 67, 41, 42);
	//tringls.push_back(tri15);

	vvr::Triangle tri16(&control_points, 67, 66, 22);
	tringls.push_back(tri16);

	vvr::Triangle tri1x(&control_points, 67, 28, 40);
	tringls.push_back(tri1x);
	for (int m = 22; m < 24; m++) {

		vvr::Triangle tri(&control_points, 67, m, m + 1);
		tringls.push_back(tri);
	}
	vvr::Triangle t10(&control_points, 67, 24, 26);
	tringls.push_back(t10);
	vvr::Triangle t11(&control_points, 67, 26, 28);
	tringls.push_back(t11);


	for (int m = 56; m < 66; m++) {

		vvr::Triangle tri(&control_points, 67, m, m + 1);
		tringls.push_back(tri);
	}

	vvr::Triangle tri17(&control_points, 40, 42, 67);
	//tringls.push_back(tri17);
	vvr::Triangle tri18(&control_points, 38, 40, 67);
	//tringls.push_back(tri18);
	vvr::Triangle tri19(&control_points, 40, 56, 67);
	tringls.push_back(tri19);

	///////////////////under nose///////////////////////////////////////////////////
	vvr::Triangle tri20(&control_points, 71, 68, 44);
	tringls.push_back(tri20);
	for (int m = 68; m < 70; m++) {

		vvr::Triangle tri(&control_points, 71, m, m + 1);
		tringls.push_back(tri);
	}
	vvr::Triangle tri21(&control_points, 71, 56, 70);
	tringls.push_back(tri21);
	for (int m = 33; m < 38; m++) {

		vvr::Triangle tri(&control_points, 71, m, m + 1);
		tringls.push_back(tri);
	}
	vvr::Triangle tri22(&control_points, 71, 33, 44);
	tringls.push_back(tri22);
	vvr::Triangle tri23(&control_points, 71, 38, 56);
	tringls.push_back(tri23);

	//////////////////////mouth/////////////////////////////
	vvr::Triangle tri29(&control_points, 72, 47, 80);
	tringls.push_back(tri29);
	vvr::Triangle tri24(&control_points, 47, 46, 80);
	tringls.push_back(tri24);
	for (int m = 56; m < 59; m++) {

		vvr::Triangle tri(&control_points, 80, m, m + 1);
		tringls.push_back(tri);
	}
	vvr::Triangle tri25(&control_points, 46, 45, 80);
	tringls.push_back(tri25);

	for (int m = 72; m < 79; m++) {

		vvr::Triangle tri(&control_points, 80, m, m + 1);
		tringls.push_back(tri);
	}
	vvr::Triangle tri26(&control_points, 45, 44, 80);
	tringls.push_back(tri26);

	for (int m = 68; m < 70; m++) {

		vvr::Triangle tri(&control_points, 80, m, m + 1);
		tringls.push_back(tri);
	}
	vvr::Triangle tri27(&control_points, 80, 68, 44);
	tringls.push_back(tri27);
	vvr::Triangle tri28(&control_points, 80, 70, 56);
	tringls.push_back(tri28);

	//////under mouth///////////////

	vvr::Triangle tri30(&control_points, 95, 94, 48);
	tringls.push_back(tri30);
	for (int m = 87; m < 94; m++) {

		vvr::Triangle tri(&control_points, 95, m, m + 1);
		tringls.push_back(tri);
	}
	vvr::Triangle tri31(&control_points, 95, 48, 47);
	tringls.push_back(tri31);

	vvr::Triangle tri33(&control_points, 95, 72, 47);
	tringls.push_back(tri33);

	for (int m = 81; m < 86; m++) {

		vvr::Triangle tri(&control_points, 95, m, m + 1);
		tringls.push_back(tri);
	}

	vvr::Triangle tri34(&control_points, 95, 72, 81);
	tringls.push_back(tri34);
	vvr::Triangle tri35(&control_points, 95, 79, 86);
	tringls.push_back(tri35);
	vvr::Triangle tri36(&control_points, 95, 79, 59);
	tringls.push_back(tri36);
	vvr::Triangle tri37(&control_points, 95, 59, 60);
	tringls.push_back(tri37);

	vvr::Triangle tri32(&control_points, 95, 60, 87);
	tringls.push_back(tri32);

	vvr::Triangle tri40(&control_points, 32, 33, 44);
	tringls.push_back(tri40);
	vvr::Triangle tri41(&control_points, 38, 40, 56);
	tringls.push_back(tri41);
	vvr::Triangle tri42(&control_points, 79, 80, 59);
	tringls.push_back(tri42);
}

void Mesh3DScene::triangulationEXTRAControlPoints(vec testpoint, std::vector<vec> &inter_verts, std::vector < vvr::Triangle> &triangles) {

	int cnt = triangles.size();
	vec P = testpoint;
	double best_distance = 10;
	int best_triangle = 0;
	bool flag = false;
	for (int g = 0; g < cnt; g++) {

		vec P1 = triangles[g].v1();
		vec P2 = triangles[g].v2();
		vec P3 = triangles[g].v3();

		vec u = P2 - P1;
		vec v = P3 - P1;
		vec n = u.Cross(v);
		vec w = P - P1;
		float gama = ((u.Cross(w)).Dot(n)) / (n.Dot(n));
		float bita = ((w.Cross(v)).Dot(n)) / (n.Dot(n));
		float alpha = 1 - gama - bita;


		if (alpha > 0 && alpha < 1 && bita>0 && bita < 1 && gama>0 && gama < 1) {


			vec projected(alpha*P1 + bita*P2 + gama*P3);

			float distance = P.DistanceSq(projected);

			if (sqrt(distance) < best_distance) {

				best_distance = sqrt(distance);
				best_triangle = g;
				flag = true;
			}
		}
	}
	if (flag) {
		vvr::Triangle tri_1(&inter_verts, triangles[best_triangle].vi1, triangles[best_triangle].vi2, inter_verts.size() - 1);
		vvr::Triangle tri_2(&inter_verts, triangles[best_triangle].vi2, triangles[best_triangle].vi3, inter_verts.size() - 1);
		vvr::Triangle tri_3(&inter_verts, triangles[best_triangle].vi1, triangles[best_triangle].vi3, inter_verts.size() - 1);

		triangles.erase(triangles.begin() + best_triangle);
		triangles.push_back(tri_1);
		triangles.push_back(tri_2);
		triangles.push_back(tri_3);
	}
	else {
		n_find_tr++;
		violating_control_points.push_back(inter_verts[inter_verts.size() - 1]);
		violating_indices.push_back(index_intersection_vectors[index_intersection_vectors.size() - 1]);
	}

}

void Mesh3DScene::secondTriangulation(std::vector<vec> &control_points, std::vector<vvr::Triangle> &tringls){
////////////////////////////////1h dokimi////////////////////////////////////////////////////////////////////////
	/*

	///////////////////////////////////////trigwnopoiisi twn periferiakwn control points//////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	for (int m = 0; m < 16 - 1; m++)
	{
	vvr::Triangle tri(&control_points, 35 , m, m + 1);
	tringls.push_back(tri);

	}
	vvr::Triangle tri(&control_points, 35, 16 - 2, 0);
	tringls.push_back(tri);



	/////////////////////////////////////trigwnopoiisi twn upoloipwn control points///////////////////////////////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	int size = control_points.size();
	for (int i = 17; i <18; i++) {
	if (i == 35) {
	i++;
	}
	/////////////////////////1os tropos----> na ta kanw project sto kathe trigwno/////////////////
	//////////////////////////////////////////////////////////////////////////////////////////////

	/*	double mindist = 10000;
	int best_index = 0;
	int counter = tringls.size();
	for (int j = 0; j < counter; j++) {

	vec normal_point = tringls[j].getNormal();
	double eswteriko = (control_points[i] - tringls[j].v1()).Dot(normal_point);

	projected = control_points[i] - eswteriko*normal_point;

	double esa = (tringls[j].v1() - projected).Dot((tringls[j].v2() - projected));
	double rada = acos(esa / (((tringls[j].v1() - projected).Length())*(tringls[j].v2() - projected).Length()));
	double dega = RadToDeg(rada);

	double esb = (tringls[j].v2() - projected).Dot((tringls[j].v3() - projected));
	double radb = acos(esb / (((tringls[j].v2() - projected).Length())*(tringls[j].v3() - projected).Length()));
	double degb = RadToDeg(radb);

	double esc = (tringls[j].v3() - projected).Dot((tringls[j].v1() - projected));
	double radc = acos(esc / (((tringls[j].v3() - projected).Length())*(tringls[j].v1() - projected).Length()));
	double degc = RadToDeg(radc);

	if (359 < dega + degb + degc && dega + degb + degc < 361) {
	proj = Point3D(projected.x, projected.y, projected.z);
	proj.setColour(vvr::Colour::white);

	//count all distances from projected to vertices of triangles//
	double dist1 = abs(projected.DistanceSq(tringls[j].v1()));
	double dist2 = abs(projected.DistanceSq(tringls[j].v2()));
	double dist3 = abs(projected.DistanceSq(tringls[j].v3()));

	double sumdist = dist1 + dist2 + dist3;
	tringls.erase(tringls.begin() + j);
	if (sumdist < mindist) {
	mindist = sumdist;
	best_index = j;
	}
	}
	}*/
	/////////////////////////2os tropos----> na ta kanw project sto z=0///////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////
	/*	int counter = tringls.size();
	C2DPoint point(control_points[i].x , control_points[i].y);
	int best_index = 0;

	for (int j = 0; j < counter; j++) {
	C2DPoint *v1 = &C2DPoint(tringls[j].v1().x, tringls[j].v1().y);
	C2DPoint *v2 = &C2DPoint(tringls[j].v2().x, tringls[j].v2().y);
	C2DPoint *v3 = &C2DPoint(tringls[j].v3().x, tringls[j].v3().y);
	Tri tri2d(v1, v2, v3);
	//triangles2d.push_back(tri2d);
	if (tri2d.to_C2D().Contains(point)) {
	//best_index = j;
	vvr::Triangle tri_1(&control_points, tringls[j].vi1, tringls[j].vi2, i);
	vvr::Triangle tri_2(&control_points, tringls[j].vi2, tringls[j].vi3, i);
	vvr::Triangle tri_3(&control_points, tringls[j].vi1, tringls[j].vi3, i);

	tringls.erase(tringls.begin() + j);

	tringls.push_back(tri_1);
	tringls.push_back(tri_2);
	tringls.push_back(tri_3);

	}

	}

	////////////////////////////////////////koino teleiwma duo tropwn////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////
	/*		vvr::Triangle tri_1(&control_points, tringls[best_index].vi1, tringls[best_index].vi2, i);
	vvr::Triangle tri_2(&control_points, tringls[best_index].vi2, tringls[best_index].vi3, i);
	vvr::Triangle tri_3(&control_points, tringls[best_index].vi1, tringls[best_index].vi3, i);

	tringls.erase(tringls.begin() + best_index);

	tringls.push_back(tri_1);
	tringls.push_back(tri_2);
	tringls.push_back(tri_3);

	}*/

	///////////////////////////////////2h dokimi//////////////////////////////////////////////////////////
	////////////////////////////////arxiki trigwnopoisi///////////////////////////////////////////////
	//////////////////////////////////////eyes///////////////////////////////////////////////////////

	for (int m = 0; m < 2; m++) {

		vvr::Triangle tri(&control_points, 16, m, m + 1);
		tringls.push_back(tri);
	}


	vvr::Triangle t1(&control_points, 16, 2, 4);
	tringls.push_back(t1);
	vvr::Triangle t2(&control_points, 16, 4, 8);
	tringls.push_back(t2);
	vvr::Triangle t3(&control_points, 16, 8, 11);
	tringls.push_back(t3);
	vvr::Triangle tri(&control_points, 16, 15, 0);
	tringls.push_back(tri);
	vvr::Triangle trig4(&control_points, 16, 13, 15);
	tringls.push_back(trig4);
	vvr::Triangle trig3(&control_points, 16, 13, 11);
	tringls.push_back(trig3);

	

	vvr::Triangle t4(&control_points, 17, 19, 30);
	tringls.push_back(t4);
	vvr::Triangle t5(&control_points, 19, 21, 30);
	tringls.push_back(t5);


	vvr::Triangle tri1(&control_points, 17, 30, 0);
	tringls.push_back(tri1);
	vvr::Triangle tri2(&control_points, 28, 2, 30);
	tringls.push_back(tri2);
	vvr::Triangle tri3(&control_points, 2, 1, 30);
	tringls.push_back(tri3);
	vvr::Triangle tri4(&control_points, 1, 0, 30);
	tringls.push_back(tri4);
	vvr::Triangle trig5(&control_points, 21, 24, 30);
	tringls.push_back(trig5);
	vvr::Triangle trng1(&control_points, 28, 24, 30);
	tringls.push_back(trng1);
	////////////////////////////////nose///////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////

	vvr::Triangle tri5(&control_points, 2, 4, 43);
	tringls.push_back(tri5);
	
	vvr::Triangle tri7(&control_points, 4, 33, 43);
	tringls.push_back(tri7);

	vvr::Triangle trig6(&control_points, 28, 38, 43);
	tringls.push_back(trig6);
	

	vvr::Triangle t6(&control_points, 38, 33, 43);
	tringls.push_back(t6);
	vvr::Triangle f(&control_points, 43, 28, 2);
	tringls.push_back(f);

	////////////////////////////cheeks////////////////////////////////////////////////
	/////////////////////////////left////////////////////////////////////////////////////

	vvr::Triangle tri11(&control_points, 4, 33, 55);
	tringls.push_back(tri11);
	vvr::Triangle tri13(&control_points, 8, 4, 55);
	tringls.push_back(tri13);
	vvr::Triangle t8(&control_points, 8, 53, 55);
	tringls.push_back(t8);
	vvr::Triangle t9(&control_points, 50, 47, 55);
	tringls.push_back(t9);
	vvr::Triangle tri14(&control_points, 47, 45, 55);
	tringls.push_back(tri14);
	vvr::Triangle trig7(&control_points, 45, 33, 55);
	tringls.push_back(trig7);
	vvr::Triangle ff(&control_points, 50, 53, 55);
	tringls.push_back(ff);
	//////////////////////right////////////////////////////////////////////////////////
	

	vvr::Triangle tri16(&control_points, 67, 28, 24);
	tringls.push_back(tri16);
	vvr::Triangle tri1x(&control_points, 67, 28, 38);
	tringls.push_back(tri1x);
	vvr::Triangle t10(&control_points, 67, 24, 65);
	tringls.push_back(t10);
	vvr::Triangle t11(&control_points, 67, 65, 62);
	tringls.push_back(t11);
	vvr::Triangle tri17(&control_points, 59, 62, 67);
	tringls.push_back(tri17);
	vvr::Triangle tri18(&control_points, 57,59, 67);
	tringls.push_back(tri18);
	vvr::Triangle tri19(&control_points, 57, 38, 67);
	tringls.push_back(tri19);

	///////////////////under nose///////////////////////////////////////////////////
	vvr::Triangle tri20(&control_points, 33, 43, 38);
	tringls.push_back(tri20);
	

	//////////////////////mouth/////////////////////////////
	vvr::Triangle tri29(&control_points, 33, 38, 80);
	tringls.push_back(tri29);
	vvr::Triangle tri24(&control_points, 33, 45, 80);
	tringls.push_back(tri24);
	vvr::Triangle tri25(&control_points, 73, 45, 80);
	tringls.push_back(tri25);

	vvr::Triangle tri27(&control_points, 80, 78, 57);
	tringls.push_back(tri27);
	vvr::Triangle tri28(&control_points, 80, 57, 38);
	tringls.push_back(tri28);

	vvr::Triangle trig28(&control_points, 73, 45, 47);
	tringls.push_back(trig28); 
	vvr::Triangle trigg28(&control_points, 78, 59, 57);
	tringls.push_back(trigg28);

	//////under mouth///////////////

	//vvr::Triangle tri30(&control_points, 95, 81, 86);
	//tringls.push_back(tri30);
	vvr::Triangle tri31(&control_points, 81, 94, 47);
	tringls.push_back(tri31);
	vvr::Triangle tri33(&control_points, 95, 94, 92);
	tringls.push_back(tri33);
	vvr::Triangle tri34(&control_points, 95, 92, 90);
	tringls.push_back(tri34);
	vvr::Triangle tri35(&control_points, 95, 88, 90);
	tringls.push_back(tri35);
	vvr::Triangle tri36(&control_points, 88, 86, 59);
	tringls.push_back(tri36);
	vvr::Triangle trng3(&control_points, 95, 81, 94);
	tringls.push_back(trng3); 
	vvr::Triangle trng4(&control_points, 95, 86, 88);
	tringls.push_back(trng4);

	vvr::Triangle addtr1(&control_points, 47, 94, 50);
	tringls.push_back(addtr1); 
	vvr::Triangle addtr2(&control_points, 59, 62, 88);
	tringls.push_back(addtr2);
	vvr::Triangle addtr3(&control_points, 8, 11, 53);
	tringls.push_back(addtr3); 
	vvr::Triangle addtr4(&control_points, 65, 24, 21);
	tringls.push_back(addtr4);

	vvr::Triangle tel1(&control_points, 95, 83, 81);
	tringls.push_back(tel1); 
	vvr::Triangle tel2(&control_points, 95, 86, 83);
	tringls.push_back(tel2);
	vvr::Triangle tel3(&control_points, 80, 75, 73);
	tringls.push_back(tel3);
	vvr::Triangle tel4(&control_points, 80, 75, 78);
	tringls.push_back(tel4);
	
}


void Mesh3DScene::thirdTriangulation(std::vector<vec> &control_points, std::vector<vvr::Triangle> &tringls) {
	

	////////////////////////////////arxiki trigwnopoisi///////////////////////////////////////////////
	//////////////////////////////////////eyes///////////////////////////////////////////////////////

	

	vvr::Triangle first_tr1(&control_points, 8, 1, 2);
	tringls.push_back(first_tr1);
	vvr::Triangle first_tr2(&control_points,0, 2, 46);
	tringls.push_back(first_tr2);
	vvr::Triangle first_tr3(&control_points, 0, 9, 46);
	tringls.push_back(first_tr3);
	vvr::Triangle first_tr4(&control_points, 0,1, 2);
	tringls.push_back(first_tr4);
	vvr::Triangle first_tr5(&control_points, 0, 1, 7);
	tringls.push_back(first_tr5);

	vvr::Triangle t1(&control_points, 8, 2, 3);
	tringls.push_back(t1);
	vvr::Triangle t2(&control_points, 8, 3, 4);
	tringls.push_back(t2);
	vvr::Triangle t3(&control_points, 8, 4, 5);
	tringls.push_back(t3);
	vvr::Triangle tri(&control_points, 8, 5, 6);
	tringls.push_back(tri);
	vvr::Triangle trig4(&control_points, 8, 6, 7);
	tringls.push_back(trig4);
	vvr::Triangle trig3(&control_points, 8, 7, 1);
	tringls.push_back(trig3);



	vvr::Triangle t4(&control_points, 14, 46, 9);
	tringls.push_back(t4);
	vvr::Triangle t5(&control_points, 14, 9, 10);
	tringls.push_back(t5);


	vvr::Triangle tri1(&control_points, 14, 10, 11);
	tringls.push_back(tri1);
	vvr::Triangle tri2(&control_points, 14, 11, 12);
	tringls.push_back(tri2);
	vvr::Triangle tri3(&control_points, 14, 12, 13);
	tringls.push_back(tri3);
	vvr::Triangle tri4(&control_points, 14, 13, 2);
	tringls.push_back(tri4);
	vvr::Triangle trig5(&control_points, 14, 2, 46);
	tringls.push_back(trig5);

	////////////////////////////////nose///////////////////////////////////////////////
	/////////////////////////////////////////////////////////////////////////////////
	
	vvr::Triangle tri5(&control_points, 2, 3, 17);
	tringls.push_back(tri5);

	vvr::Triangle tri7(&control_points, 3, 15, 17);
	tringls.push_back(tri7);

	vvr::Triangle trig6(&control_points, 15, 16, 17);
	tringls.push_back(trig6);


	vvr::Triangle t6(&control_points, 13, 16, 17);
	tringls.push_back(t6);
	vvr::Triangle f(&control_points, 17, 13, 2);
	tringls.push_back(f);

	////////////////////////////cheeks////////////////////////////////////////////////
	/////////////////////////////left////////////////////////////////////////////////////
	
	vvr::Triangle tri11(&control_points, 3, 4, 22);
	tringls.push_back(tri11);
	vvr::Triangle tri13(&control_points, 4, 21, 22);
	tringls.push_back(tri13);
	vvr::Triangle t8(&control_points, 21, 22, 22);
	tringls.push_back(t8);
	vvr::Triangle t9(&control_points, 21, 20, 22);
	tringls.push_back(t9);
	vvr::Triangle tri14(&control_points, 20, 19, 22);
	tringls.push_back(tri14);
	vvr::Triangle trig7(&control_points, 19, 18, 22);
	tringls.push_back(trig7);
	vvr::Triangle ff(&control_points, 18, 15, 22);
	tringls.push_back(ff);
	vvr::Triangle fff(&control_points, 3, 15, 22);
	tringls.push_back(fff);



	vvr::Triangle tri16(&control_points, 12, 13, 27);
	tringls.push_back(tri16);
	vvr::Triangle tri1x(&control_points, 13, 16, 27);
	tringls.push_back(tri1x);
	vvr::Triangle t10(&control_points, 16, 23, 27);
	tringls.push_back(t10);
	vvr::Triangle t11(&control_points, 23, 24, 27);
	tringls.push_back(t11);
	vvr::Triangle tri17(&control_points, 25, 26, 27);
	tringls.push_back(tri17);
	vvr::Triangle tri18(&control_points, 26, 12, 27);
	tringls.push_back(tri18);
	vvr::Triangle tri19(&control_points, 27, 25, 24);
	tringls.push_back(tri19);

	///////////////////under nose///////////////////////////////////////////////////
	vvr::Triangle tri20(&control_points, 17, 15, 16);
	tringls.push_back(tri20);


	//////////////////////above mouth/////////////////////////////
	vvr::Triangle tri29(&control_points, 30, 31, 45);
	tringls.push_back(tri29);
	vvr::Triangle trigwno29(&control_points, 31, 32, 45);
	tringls.push_back(trigwno29); 
	vvr::Triangle trigwno30(&control_points, 32, 33, 45);
	tringls.push_back(trigwno30);
	vvr::Triangle tri30(&control_points, 28, 29, 45);
	tringls.push_back(tri30);
	vvr::Triangle trian(&control_points, 29, 30, 45);
	tringls.push_back(trian);

	vvr::Triangle trig28(&control_points, 18, 15, 45);
	tringls.push_back(trig28);
	vvr::Triangle trigg28(&control_points, 28, 18, 45);
	tringls.push_back(trigg28);
	vvr::Triangle trigg30(&control_points, 15, 16, 45);
	tringls.push_back(trigg30);
	vvr::Triangle trigg31(&control_points, 16, 23, 45);
	tringls.push_back(trigg31);
	vvr::Triangle trigg32(&control_points, 23, 33, 45);
	tringls.push_back(trigg32);
	vvr::Triangle trigg33(&control_points, 24, 23, 33);
	tringls.push_back(trigg33);
	vvr::Triangle trigg34(&control_points, 18, 19, 28);
	tringls.push_back(trigg34);
	vvr::Triangle trigg35(&control_points, 28, 30, 45);
	//tringls.push_back(trigg35);

	//////under mouth///////////////

	
	vvr::Triangle tri31(&control_points, 43, 42, 44);
	tringls.push_back(tri31);
	vvr::Triangle tri33(&control_points, 42, 41, 44);
	tringls.push_back(tri33);
	vvr::Triangle tri34(&control_points, 41, 40, 44);
	tringls.push_back(tri34);
	vvr::Triangle tri35(&control_points, 40, 25, 24);
	tringls.push_back(tri35);
	vvr::Triangle tri36(&control_points, 24, 40, 39);
	tringls.push_back(tri36);
	vvr::Triangle trng3(&control_points, 39, 33, 24);
	tringls.push_back(trng3);
	vvr::Triangle trng4(&control_points, 19, 28, 34);
	tringls.push_back(trng4);

	vvr::Triangle addtr1(&control_points, 34, 35, 44);
	tringls.push_back(addtr1);
	vvr::Triangle addtr0(&control_points, 35, 36, 44);
	tringls.push_back(addtr0);
	vvr::Triangle addtr2(&control_points, 36, 37, 44);
	tringls.push_back(addtr2);

	vvr::Triangle addtr3(&control_points, 39, 44, 40);
	tringls.push_back(addtr3);
	vvr::Triangle addtr4(&control_points, 34, 43, 44);
	tringls.push_back(addtr4);
	vvr::Triangle addtr5(&control_points, 37, 38, 44);
	tringls.push_back(addtr5);
	vvr::Triangle addtr6(&control_points, 38, 39, 44);
	tringls.push_back(addtr6);

	vvr::Triangle tel1(&control_points, 43, 19, 34);
	tringls.push_back(tel1);
	vvr::Triangle tel2(&control_points, 43, 20, 19);
	tringls.push_back(tel2);
	vvr::Triangle tel3(&control_points, 4, 5, 21);
	tringls.push_back(tel3);
	vvr::Triangle tel4(&control_points, 11, 12, 26);
	tringls.push_back(tel4);
	
}







/*________Usefull Lab's Tasks_______*/

void pca(vector<vec>& vertices, vec &center, vec &dir)
{
	const int count = vertices.size();

	float w0 = 0;
	float x0 = 0, y0 = 0, z0 = 0;
	float x2 = 0, y2 = 0, z2 = 0, xy = 0, yz = 0, xz = 0;
	float dx2, dy2, dz2, dxy, dxz, dyz;
	float det[9];

	for (int i = 0; i < count; i++)
	{
		float x = vertices[i].x;
		float y = vertices[i].y;
		float z = vertices[i].z;

		x2 += x * x;
		xy += x * y;
		xz += x * z;
		y2 += y * y;
		yz += y * z;
		z2 += z * z;
		x0 += x;
		y0 += y;
		z0 += z;
	}
	w0 = (float)count;

	x2 /= w0;
	xy /= w0;
	xz /= w0;
	y2 /= w0;
	yz /= w0;
	z2 /= w0;

	x0 /= w0;
	y0 /= w0;
	z0 /= w0;

	dx2 = x2 - x0 * x0;
	dxy = xy - x0 * y0;
	dxz = xz - x0 * z0;
	dy2 = y2 - y0 * y0;
	dyz = yz - y0 * z0;
	dz2 = z2 - z0 * z0;

	det[0] = dz2 + dy2;
	det[1] = -dxy;
	det[2] = -dxz;
	det[3] = det[1];
	det[4] = dx2 + dz2;
	det[5] = -dyz;
	det[6] = det[2];
	det[7] = det[5];
	det[8] = dy2 + dx2;

	/* Searching for a eigenvector of det corresponding to the minimal eigenvalue */
	gte::SymmetricEigensolver3x3<float> solver;
	std::array<float, 3> eval;
	std::array<std::array<float, 3>, 3> evec;
	solver(det[0], det[1], det[2], det[4], det[5], det[8], true, 1, eval, evec);

	center.x = x0;
	center.y = y0;
	center.z = z0;

	dir.x = evec[0][0];
	dir.y = evec[0][1];
	dir.z = evec[0][2];
}

void Task_1_FindCenterMass(vector<vec> &vertices, vec &cm)
{
	//!//////////////////////////////////////////////////////////////////////////////////
	//! TASK:
	//!
	//!  - Breite to kentro mazas twn simeiwn `vertices`.
	//!  - Apothikeyste to apotelesma stin metavliti `cm`.
	//!
	//!//////////////////////////////////////////////////////////////////////////////////

	//...
	//...
	//...
	vec sum(0, 0, 0);
	int number = vertices.size();
	for (int i = 0; i < number; i++) {
		sum = sum + vertices[i];
	}
	sum = sum / float(vertices.size());
	cm = sum;
}

void Task_2_FindAABB(vector<vec> &vertices, Box3D &aabb)
{
	//!//////////////////////////////////////////////////////////////////////////////////
	//! TASK:
	//!
	//!  - Breite to Axis Aligned Bounding Box tou montelou
	//!
	//! HINTS:
	//!
	//!  - To `aabb` orizetai apo 2 gwniaka simeia. (V_min, V_max)
	//!  - V_min: { aabb.x1, aabb.y1, aabb.z1 }
	//!  - V_max: { aabb.x2, aabb.y2, aabb.z2 }
	//!
	//!//////////////////////////////////////////////////////////////////////////////////

	//...
	//...
	//...

	vec min = vertices.at(0);
	vec max = vertices.at(0);
	int number = vertices.size();
	for (int i = 0; i < number; i++) {
		if (vertices[i].x < min.x) {
			min.x = vertices[i].x;
		}

		if (vertices[i].y < min.y) {
			min.y = vertices[i].y;

		}

		if (vertices[i].z < min.z) {
			min.z = vertices[i].z;

		}
		if (vertices[i].x > max.x) {
			max.x = vertices[i].x;
		}
		if (vertices[i].y > max.y) {
			max.y = vertices[i].y;
		}
		if (vertices[i].z > max.z) {
			max.z = vertices[i].z;
		}
	}

	aabb.x1 = min.x;
	aabb.x2 = max.x;
	aabb.y1 = min.y;
	aabb.y2 = max.y;
	aabb.z1 = min.z;
	aabb.z2 = max.z;
}

vec Mesh3DScene::Task_3_Pick_Origin()
{
	return m_center_mass;
}
void Task_3_AlignOriginTo(vector<vec> &vertices, const vec &cm)
{
	//!//////////////////////////////////////////////////////////////////////////////////
	//! TASK:
	//!
	//!  - Metatopiste to montelo esti wste to simeio `cm` na erthei sto (0,0,0).
	//!
	//!//////////////////////////////////////////////////////////////////////////////////

	int number = vertices.size();
	for (int i = 0; i < number; i++) {

		vertices[i].x = vertices[i].x - cm.x;
		vertices[i].y = vertices[i].y - cm.y;
		vertices[i].z = vertices[i].z - cm.z;

	}

}

void Task_5_Intersect(vector<vvr::Triangle> &triangles, Plane &plane, vector<int> &intersection_indices)
{
	//!//////////////////////////////////////////////////////////////////////////////////
	//! TASK:
	//!
	//!  - Brete ta trigwna pou temnontai me to epipedo `plane`.
	//!  - Kante ta push_back sto vector intersection_indices.
	//!
	//!//////////////////////////////////////////////////////////////////////////////////

	//...
	//...
	//...
	int number = triangles.size();
	for (int j = 0; j < number; j++) {


		double 	es_gin1 = triangles[j].v1().Dot(plane.normal) - plane.d;
		double 	es_gin2 = triangles[j].v2().Dot(plane.normal) - plane.d;
		double 	es_gin3 = triangles[j].v3().Dot(plane.normal) - plane.d;


		if ((es_gin1>0 && es_gin2>0 && es_gin3>0) || (es_gin1<0 && es_gin2<0 && es_gin3<0)) {


		}
		else {
			intersection_indices.push_back(j);
		}

	}
}

void Task_5_Split(Mesh &mesh, Plane &plane, vector<int> &final_index_vector)
{

	//!//////////////////////////////////////////////////////////////////////////////////
	//! TASK:
	//!
	//!  - Kopste to antikeimeno sta 2. (Odigies tin ora tou ergasthriou)
	//!
	//!//////////////////////////////////////////////////////////////////////////////////

	std::vector<vec> &vertices = mesh.getVertices();
	std::vector<vvr::Triangle> &triangles = mesh.getTriangles();

	std::vector<vvr::Triangle> myTriangles;

	///////////////////////////////////////////*edw tha apothikeutoun ta vertices pou einai pio konta sta simeia tomis*//
	vector<vec> myVertices;
	//////////////////////////////////////////*edw tha apothikeutoun to index twn vertices pou einai pio konta sta simeia tomis*//
	vector<int> index_vector;
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	int count_triangles = triangles.size();

	for (int j = 0; j < triangles.size(); j++) {
		double 	es_gin1 = triangles[j].v1().Dot(plane.normal) - plane.d;
		double 	es_gin2 = triangles[j].v2().Dot(plane.normal) - plane.d;
		double 	es_gin3 = triangles[j].v3().Dot(plane.normal) - plane.d;

		if ((es_gin1 > 0 && es_gin2 > 0 && es_gin3 > 0)) {

		}
		else if ((es_gin1 < 0 && es_gin2 < 0 && es_gin3 < 0)) {

		}

		//////simeia tomis//////////
		else {


			vec point_pl;
			point_pl = plane.PointOnPlane();

			vec tr_1 = triangles[j].v1();
			vec tr_2 = triangles[j].v2();
			vec tr_3 = triangles[j].v3();

			vec l_1 = tr_2 - tr_1;
			vec l_2 = tr_3 - tr_2;
			vec l_3 = tr_1 - tr_3;

			vec dif_p0_l0_1 = point_pl - tr_1;
			vec dif_p0_l0_2 = point_pl - tr_2;
			vec dif_p0_l0_3 = point_pl - tr_3;

			double d1 = dif_p0_l0_1.Dot(plane.normal) / l_1.Dot(plane.normal);
			double d2 = dif_p0_l0_2.Dot(plane.normal) / l_2.Dot(plane.normal);
			double d3 = dif_p0_l0_3.Dot(plane.normal) / l_3.Dot(plane.normal);

			///////*simeia tomis*/////

			vec inter_point_1 = d1*l_1 + tr_1;
			vec inter_point_2 = d2*l_2 + tr_2;
			vec inter_point_3 = d3*l_3 + tr_3;

			if ((0 < d1 && d1 < 1) && (0 < d2 && d2 < 1)) {

				////checkaroume se poio vertex eimaste pio konta/////

				double dist_1_1 = (inter_point_1 - tr_1).Length();
				double dist_1_2 = (inter_point_1 - tr_2).Length();
				double dist_2_2 = (inter_point_2 - tr_2).Length();
				double dist_2_3 = (inter_point_2 - tr_3).Length();

				double min_dist[4] = { dist_1_1,dist_1_2,dist_2_2,dist_2_3 };
				double  n = min_dist[0];
				int size = 4;
				int index = 0;
				for (int i = 1; i < 4; i++)
				{
					if (min_dist[i] < n)
					{
						n = min_dist[i];
						index = i;
					}
				}
				if (index == 0) {
					Point3D point_inter_1(tr_1.x, tr_1.y, tr_1.z, Colour::red);
					//point_inter_1.draw();
					myVertices.push_back(tr_1);
					index_vector.push_back(triangles[j].vi1);
				}

				else if ((index == 1) || (index == 2)) {
					Point3D point_inter_1(tr_2.x, tr_2.y, tr_2.z, Colour::green);
					//point_inter_1.draw();
					myVertices.push_back(tr_2);
					index_vector.push_back(triangles[j].vi2);

				}
				else if ((index == 3)) {
					Point3D point_inter_1(tr_3.x, tr_3.y, tr_3.z, Colour::yellow);
					//point_inter_1.draw();
					myVertices.push_back(tr_3);
					index_vector.push_back(triangles[j].vi3);

				}

			}
			else if ((0 < d1 && d1 < 1) && (0 < d3 && d3 < 1)) {

				double dist_1_1 = (inter_point_1 - tr_1).Length();
				double dist_1_2 = (inter_point_1 - tr_2).Length();
				double dist_3_1 = (inter_point_2 - tr_1).Length();
				double dist_3_3 = (inter_point_3 - tr_3).Length();

				double min_dist[4] = { dist_1_1,dist_1_2,dist_3_1,dist_3_3 };
				double  n = min_dist[0];
				int size = 4;
				int index = 0;
				for (int i = 1; i < 4; i++)
				{
					if (min_dist[i] < n)
					{
						n = min_dist[i];
						index = i;
					}
				}
				if ((index == 0) || (index == 2)) {
					Point3D point_inter_1(tr_1.x, tr_1.y, tr_1.z, Colour::red);
					//point_inter_1.draw();
					myVertices.push_back(tr_1);
					index_vector.push_back(triangles[j].vi1);

				}

				else if (index == 1) {
					Point3D point_inter_1(tr_2.x, tr_2.y, tr_2.z, Colour::green);
					//point_inter_1.draw();
					myVertices.push_back(tr_2);
					index_vector.push_back(triangles[j].vi2);

				}
				else if ((index == 3)) {
					Point3D point_inter_1(tr_3.x, tr_3.y, tr_3.z, Colour::yellow);
					//point_inter_1.draw();
					myVertices.push_back(tr_3);
					index_vector.push_back(triangles[j].vi3);

				}

				Point3D point_inter_1(inter_point_1.x, inter_point_1.y, inter_point_1.z);
				//point_inter_1.draw();
				Point3D point_inter_3(inter_point_3.x, inter_point_3.y, inter_point_3.z);
				//point_inter_3.draw();

			}
			else if ((0 < d3 && d3 < 1) && (0 < d2 && d2 < 1)) {
				double dist_3_1 = (inter_point_3 - tr_1).Length();
				double dist_3_3 = (inter_point_3 - tr_3).Length();
				double dist_2_3 = (inter_point_2 - tr_3).Length();
				double dist_2_2 = (inter_point_2 - tr_2).Length();

				double min_dist[4] = { dist_3_1,dist_3_3,dist_2_3,dist_2_2 };
				double  n = min_dist[0];
				int size = 4;
				int index = 0;
				for (int i = 1; i < 4; i++)
				{
					if (min_dist[i] < n)
					{
						n = min_dist[i];
						index = i;
					}
				}
				if (index == 0) {
					Point3D point_inter_1(tr_1.x, tr_1.y, tr_1.z, Colour::red);
					//point_inter_1.draw();
					myVertices.push_back(tr_1);
					index_vector.push_back(triangles[j].vi1);

				}

				else if (index == 3) {
					Point3D point_inter_1(tr_2.x, tr_2.y, tr_2.z, Colour::green);
					//point_inter_1.draw();
					myVertices.push_back(tr_2);
					index_vector.push_back(triangles[j].vi2);

				}
				else if ((index == 1) || (index == 2)) {
					Point3D point_inter_1(tr_3.x, tr_3.y, tr_3.z, Colour::yellow);
					//point_inter_1.draw();
					myVertices.push_back(tr_3);
					index_vector.push_back(triangles[j].vi3);

				}

				Point3D point_inter_3(inter_point_3.x, inter_point_3.y, inter_point_3.z);
				//point_inter_3.draw();
				Point3D point_inter_2(inter_point_2.x, inter_point_2.y, inter_point_2.z);
				//point_inter_2.draw();
			}
		}

	}

	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	//*taxinomisi twn vertices wste na parume auta me to megalutero z*//
	vector<vec> sort_vertices_z;
	for (auto &d : myVertices)
		sort_vertices_z.push_back(d);
	std::sort(sort_vertices_z.begin(), sort_vertices_z.end(), myobject_z);

	///////taxinomisi kai twn indices me vasi tin taxinomisi twn vertices/////////////

	std::sort(index_vector.begin(), index_vector.end(), [&](int& i1, int& i2) { return vertices[i1].z > vertices[i2].z; });
	//vector<int> final_index_vector;

	//////////////////////////////////////////////////////////////////////////////////
	double mid_x = sort_vertices_z[0].x;
	double mid_y = sort_vertices_z[0].y;
	double mid_z = sort_vertices_z[0].z;

	/////////////////////////////////point_nose//////////////////////////////////////
	Point3D point_nose(sort_vertices_z[0].x, sort_vertices_z[0].y, sort_vertices_z[0].z, Colour::red);
	//point_nose.draw();
	final_index_vector.push_back(index_vector[0]);


	/////////////////////////////////point_up////////////////////////////////////////

	//*taxinomisi twn vertices wste na parume auta me to megalutero upsos(y)*//
	vector<vec> sort_vertices_y;
	for (auto &d : myVertices)
		sort_vertices_y.push_back(d);
	std::sort(sort_vertices_y.begin(), sort_vertices_y.end(), myobject_y);

	///////taxinomisi kai twn indices ws pros y me vasi tin taxinomisi twn vertices/////////////

	std::sort(index_vector.begin(), index_vector.end(), [&](int& i1, int& i2) { return vertices[i1].y > vertices[i2].y; });
	final_index_vector.push_back(index_vector[0]);
	/*testarisma gia na dw an ta indices meta apo 2 taxinomisis deixnoun swsta*/
	Point3D point_up(vertices[final_index_vector[final_index_vector.size() - 1]].x, vertices[final_index_vector[final_index_vector.size() - 1]].y, vertices[final_index_vector[final_index_vector.size() - 1]].z, Colour::magenta);
	//point_up.draw();


	//Point3D point_up(sort_vertices_y[0].x, sort_vertices_y[0].y, sort_vertices_y[0].z, Colour::green);
	//point_up.draw();

	/////////////////////////////////point_down///////////////////////////////////////

	float under_nose = (mid_y - (sort_vertices_y[sort_vertices_y.size() - 1].y)) / 5;
	Point3D point_full_down(sort_vertices_y[sort_vertices_y.size() - 1].x, sort_vertices_y[sort_vertices_y.size() - 1].y, sort_vertices_y[sort_vertices_y.size() - 1].z, Colour::green);
	//point_full_down.draw();

	float dist = mid_y - (sort_vertices_y[sort_vertices_y.size() - 1].y);

	/*xana sort ws pros z twn indices*/

	std::sort(index_vector.begin(), index_vector.end(), [&](int& i1, int& i2) { return vertices[i1].z > vertices[i2].z; });

	vector<vec> lips_chin;
	vector<int> index_lips_chin;
	for (int j = 0; j < 150; j++) //evala to 50 gia na checkarw polla simeia
	{

		if (sort_vertices_z[j].y < mid_y - 4 * under_nose) //evala to -10 giati thelw na xefugw apo ta simeia tis mutis//
		{
			//Point3D point_down(sort_vertices_z[j].x, sort_vertices_z[j].y, sort_vertices_z[j].z, Colour::green);
			//point_down.draw();

			lips_chin.push_back(sort_vertices_z[j]);
			index_lips_chin.push_back(index_vector[j]);
		}

	}

	std::sort(lips_chin.begin(), lips_chin.end(), myobject_y);
	std::sort(index_lips_chin.begin(), index_lips_chin.end(), [&](int& i1, int& i2) { return vertices[i1].z > vertices[i2].z; });
	final_index_vector.push_back(index_lips_chin[0]);
	Point3D point_down(lips_chin[0].x, lips_chin[0].y, lips_chin[0].z, Colour::red);
	//point_down.draw();

	/////////////////////////////point above nose///////////////////////////////////////////////////////////////

	/*xwrizw se 2 meri to proswpo*/
	Point3D point_ab_nose;
	/*for (int cn = 0; cn < sort_vertices_y.size() - 4; cn++) {


	if ((((sort_vertices_y[cn  +3].z) - (sort_vertices_y[cn].z)) >2) && sort_vertices_y[cn].y > mid_y) {
	std::cout << "\nmlkia\n";
	point_ab_nose=Point3D(sort_vertices_y[cn+1].x, sort_vertices_y[cn+1].y, sort_vertices_y[cn+1].z, Colour::black);
	point_ab_nose.draw();
	break;
	}


	}

	*/

	/*


	///////////////////////////sort ta vertices ws pros x/////////////////////////////////////////////////////
	vector<vec> sort_x_vertices;
	for (auto &d : vertices)
	sort_x_vertices.push_back(d);
	std::sort(sort_x_vertices.begin(), sort_x_vertices.end(), [&](vec v1, vec v2) { return v1.x > v2.x; });

	//////////////////matia///////////////////////////////////////////////////////////////////////////////////
	////////////ftiaxnw ena orthogwnio noito kai exetazw poia vertices exoun to mikrotero z///////////////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////
	vector<vec> sutisf_vertices;
	vector<vec> sutisf_vertices_down;

	float boundy= (point_ab_nose.y-abs( point_nose.y))/float(2);
	float boundx=( abs(point_ab_nose.x- sort_x_vertices[0].x))/float(4);
	for (int i = 0; i < vertices.size(); i++) {

	if ((vertices[i].y > point_ab_nose.y ) && (vertices[i].y < point_ab_nose.y + boundy) && (vertices[i].x < point_ab_nose.x + 3 * boundx) && (vertices[i].x > point_ab_nose.x - 3 * boundx)) {
	sutisf_vertices.push_back(vertices[i]);
	}
	}

	for (int i = 0; i < vertices.size(); i++) {

	if ((vertices[i].y < point_ab_nose.y) && (vertices[i].y > point_ab_nose.y - boundy) && (vertices[i].x < point_ab_nose.x + 3 * boundx) && (vertices[i].x > point_ab_nose.x - 3 * boundx)) {
	sutisf_vertices_down.push_back(vertices[i]);
	}
	}
	/////////thelw na dw tin klisi ws pros z panw apo ta matia///////////////
	for (int j = 0; j < sutisf_vertices.size() - 1; j = j + 5) {

	if (((sutisf_vertices[j + 1].z) - sutisf_vertices[j].z) <-0.4) {
	Point3D test_p(sutisf_vertices[j + 1].x, sutisf_vertices[j + 1].y, sutisf_vertices[j + 1].z, Colour::red);
	test_p.draw();
	}
	}

	////////////////////////pali me tin klisi auth th fora thetikh////////////
	for (int j = 0; j < sutisf_vertices_down.size() - 1; j = j + 5) {

	if (((sutisf_vertices_down[j + 1].z) - sutisf_vertices_down[j].z) >0.4) {
	Point3D test_p(sutisf_vertices_down[j + 1].x, sutisf_vertices_down[j + 1].y, sutisf_vertices_down[j + 1].z, Colour::green);
	test_p.draw();
	}
	}




	std::sort(sutisf_vertices.begin(), sutisf_vertices.end(), myobject_z);
	for (int j = sutisf_vertices.size()-100; j < sutisf_vertices.size(); j=j+1) {
	Point3D test_p(sutisf_vertices[j].x, sutisf_vertices[j].y, sutisf_vertices[j].z, Colour::blue);
	//	test_p.draw();
	}


	//////////////////stomaaaaa///////////////////////////////////////////////////////////////////////////////////
	/////////////////me bounding box ftiagmeno apo ta vertices tomis me to plane pou einai konta st stoma/////////
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////


	vector<vec> mouth_intersect_vertices;


	vector<vec> sutisf_vertices_mouth;

	float boundx_mouth = (abs(point_nose.x - sort_x_vertices[0].x)) / float(4);//to idio me prin
	float boundy_mouth = (abs(point_nose.y - point_full_down.y))/float(4);

	for (int k = 0; k < 80; k++) {

	if ((sort_vertices_z[k].y < point_nose.y - boundy_mouth) && (sort_vertices_z[k].y > point_nose.y - 2 * boundy_mouth) && (sort_vertices_z[k].x < point_nose.x + 2 * boundx_mouth) && (sort_vertices_z[k].x > point_nose.x - 2 * boundx_mouth)) {

	////Point3D test_p(sort_vertices_z[k].x, sort_vertices_z[k].y, sort_vertices_z[k].z, Colour::yellow);
	///test_p.draw();
	mouth_intersect_vertices.push_back(sort_vertices_z[k]);
	}
	}

	std::sort(mouth_intersect_vertices.begin(), mouth_intersect_vertices.end(), myobject_y);

	for (int i = 0; i < vertices.size(); i++) {

	if ((vertices[i].y < mouth_intersect_vertices[0].y) && (vertices[i].y > mouth_intersect_vertices[mouth_intersect_vertices.size()-1].y) && (vertices[i].x < point_nose.x + 2 * boundx_mouth) && (vertices[i].x > point_nose.x - 2 * boundx_mouth)) {
	sutisf_vertices_mouth.push_back(vertices[i]);
	}
	}
	//std::sort(sutisf_vertices_mouth.begin(), sutisf_vertices_mouth.end(), myobject_z);
	//for (int k = 0; k < 100; k++) {

	//	Point3D test_p(sutisf_vertices_mouth[k].x, sutisf_vertices_mouth[k].y, sutisf_vertices_mouth[k].z, Colour::yellow);
	//	test_p.draw();	}

	/////////thelw na dw tin klisi ws pros z stoma///////////////
	for (int j = 0; j < sutisf_vertices_mouth.size() - 1; j = j + 1) {

	if (((sutisf_vertices_mouth[j + 1].z) - sutisf_vertices_mouth[j].z) <-0.5) {
	Point3D test_p(sutisf_vertices_mouth[j + 1].x, sutisf_vertices_mouth[j + 1].y, sutisf_vertices_mouth[j + 1].z, Colour::red);
	test_p.draw();
	}
	}

	////////////////////////pali me tin klisi auth th fora thetikh////////////
	for (int j = 0; j < sutisf_vertices_mouth.size() - 1; j = j + 1) {

	if (((sutisf_vertices_mouth[j + 1].z) - sutisf_vertices_mouth[j].z) >0.4) {
	Point3D test_p(sutisf_vertices_mouth[j + 1].x, sutisf_vertices_mouth[j + 1].y, sutisf_vertices_mouth[j + 1].z, Colour::green);
	test_p.draw();
	}
	}
	*/
}

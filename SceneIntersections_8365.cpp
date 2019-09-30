#include "SceneIntersections_8365.h"

using namespace std;
using namespace vvr;

/* Construct - Load  - Setup */

SceneIntersections::SceneIntersections()
{
    m_bg_col = Colour(0x44, 0x44, 0x44);
    m_hide_log = false;
    reset();
}

void SceneIntersections::reset()
{
    Scene::reset();

    // Clear everything
    m_canvas_0.clear();
    m_canvas_1.clear();
    m_canvas_2.clear();
    m_canvas_3.clear();

    // Divide window to Tasks
    m_bound_vertical.Set(C2DPoint(0, -3000), C2DPoint(0, 3000));
    m_bound_horizontal.Set(C2DPoint(4000, 0), C2DPoint(-4000, 0));
    m_canvas_0.add(m_bound_horizontal, Colour::black);
    m_canvas_0.add(m_bound_vertical, Colour::black);

    // Setup Task 1:
    {
        C2DPoint a1(-300, 100);
        C2DPoint a2(-100, 200);
        C2DPoint b1(-350, 230);
        C2DPoint b2(-50, 50);
        m_line_1 = C2DLine(a1, a2);
        m_line_2 = C2DLine(b1, b2);
        m_canvas_0.add(a1, Colour::orange);
        m_canvas_0.add(a2, Colour::orange);
        m_canvas_0.add(m_line_1, Colour::orange);
        m_canvas_1.add(b1, Colour::cyan);
        m_canvas_1.add(b2, Colour::cyan);
        m_canvas_1.add(m_line_2, Colour::cyan);
    }

    // Task 2:
    {
        C2DPoint t1a(-300, -50);
        C2DPoint t1b(-40, -45);
        C2DPoint t1c(-70, -170);
        m_triangle_1 = C2DTriangle(t1a, t1b, t1c);
        C2DPoint t2a(-197, -266);
        C2DPoint t2b(-368, -136);
        C2DPoint t2c(-108, -76);
        m_triangle_2 = C2DTriangle(t2a, t2b, t2c);
        m_canvas_0.add(m_triangle_1, Colour::orange,true);
        m_canvas_2.add(m_triangle_2, Colour::cyan);
    }

    // Setup Task 3:
    {
        C2DPoint c1(166, 112);
        C2DPoint c2(290, 150);
        m_circle_1 = C2DCircle(c1, 80);
        m_circle_2 = C2DCircle(c2, 60);
        m_canvas_0.add(c1, Colour::orange);
        m_canvas_0.add(m_circle_1, Colour::orange);
        m_canvas_3.add(c2, Colour::cyan);
        m_canvas_3.add(m_circle_2, Colour::cyan);
    }

    Task1(m_line_2.GetPointTo());
    Task2(m_triangle_2.GetPoint3());
    Task3(m_circle_2.GetCentre());
}

/* UI Handling */

void SceneIntersections::mousePressed(int x, int y, int modif)
{
    Scene::mousePressed(x, y, modif);
    echo(x);
    echo(y);
    const C2DPoint p(x, y);
    if (m_bound_horizontal.IsOnRight(p) && !m_bound_vertical.IsOnRight(p)) Task1(p);
    if (m_bound_horizontal.IsOnRight(p) && m_bound_vertical.IsOnRight(p)) Task3(p);
    if (!m_bound_horizontal.IsOnRight(p) && !m_bound_vertical.IsOnRight(p)) Task2(p);
}

void SceneIntersections::mouseMoved(int x, int y, int modif)
{
    Scene::mouseMoved(x, y, modif);
    mousePressed(x, y, modif);
}

/* Tasks */

void SceneIntersections::Task1(const C2DPoint &p)
{
    C2DPoint p1 = m_line_2.GetPointFrom();   // To arxiko simeio paremenei idio.
    m_line_2 = C2DLine(p1, p);   // To teliko simeio tis grammis akolouthei to mouse.

    m_canvas_1.clear();
    m_canvas_1.add(p, Colour::cyan);
    m_canvas_1.add(p1, Colour::cyan);
    m_canvas_1.add(m_line_2, Colour::cyan);

    /**
     * Breite to simeio tomis twn 2 euth. tmimatwn
     */

    bool seg_intersect = false;
    C2DPoint i;

	i=InterLines( m_line_1,  m_line_2, seg_intersect);
    m_canvas_1.add(i, seg_intersect ? Colour::green : Colour::red);
}

C2DPoint SceneIntersections::InterLines(C2DLine line_1, C2DLine line_2,bool& seg_intersect) {

	//bool seg_intersect = flag;
	C2DPoint i;



	double a1 = (line_1.GetPointTo().y - line_1.GetPointFrom().y) / (line_1.GetPointTo().x - line_1.GetPointFrom().x);
	double a2 = (line_2.GetPointTo().y - line_2.GetPointFrom().y) / (line_2.GetPointTo().x - line_2.GetPointFrom().x);
	double b1 = line_1.GetPointTo().y - a1*	line_1.GetPointTo().x;
	double b2 = line_2.GetPointTo().y - a2*	line_2.GetPointTo().x;

	double x = ((b2 - b1) / (a1 - a2));
	double y = a1*((b2 - b1) / (a1 - a2)) + b1;

	i.x = x;
	i.y = y;
	double distance1 = sqrt(pow((i.x - line_1.GetPointTo().x), 2) + pow((i.y - line_1.GetPointTo().y), 2));
	double distance2 = sqrt(pow((i.x - line_1.GetPointFrom().x), 2) + pow((i.y - line_1.GetPointFrom().y), 2));
	double distance = sqrt(pow((line_1.GetPointTo().x - line_1.GetPointFrom().x), 2) + pow((line_1.GetPointTo().y - line_1.GetPointFrom().y), 2));
	double dist1 = sqrt(pow((i.x - line_2.GetPointTo().x), 2) + pow((i.y - line_2.GetPointTo().y), 2));
	double dist2 = sqrt(pow((i.x - line_2.GetPointFrom().x), 2) + pow((i.y - line_2.GetPointFrom().y), 2));
	double dist = sqrt(pow((line_2.GetPointTo().x - line_2.GetPointFrom().x), 2) + pow((line_2.GetPointTo().y - line_2.GetPointFrom().y), 2));

	
	if ((distance1 + distance2>distance+0.0001) || (dist1+dist2>dist+0.0001)) {

		seg_intersect = false;

	}
	else {
		seg_intersect = true;
	}
	return i;
}
void SceneIntersections::Task2(const C2DPoint &p)
{
    const C2DPoint &p1 = m_triangle_2.GetPoint1();
    const C2DPoint &p2 = m_triangle_2.GetPoint2();
    m_triangle_2.Set(p1, p2, p);
    m_canvas_2.clear();
    m_canvas_2.add(m_triangle_2, Colour::cyan,true);
	
    

	/*pairnw ta shmeia tou statherou trigwwnou*/
	const C2DPoint &str1= m_triangle_1.GetPoint1();
	const C2DPoint &str2 = m_triangle_1.GetPoint2();
	const C2DPoint &str3= m_triangle_1.GetPoint3();



	/*ftiaxnw eutheies me ta shmeia tou kinoumenou*/
	C2DLine line1(p1, p2);
	C2DLine line2(p1, p);
	C2DLine line3(p2, p);


	/*ftiaxnw eutheies me ta shmeia tou statherou*/
	C2DLine s_line1(str1, str2);
	C2DLine s_line2(str1, str3);
	C2DLine s_line3(str2, str3);


	C2DLineSet lineset;
	C2DLineSet s_lineset;
	C2DPointSet point_set;

	/*prosthetw tis eutheies tou kinoumenou sto lineset*/
	lineset.AddCopy(line1);
	lineset.AddCopy(line2);
	lineset.AddCopy(line3);

	/*prosthetw tis eutheies tou statherou sto s_lineset*/

	s_lineset.AddCopy(s_line1);
	s_lineset.AddCopy(s_line2);
	s_lineset.AddCopy(s_line3);
	
	/*Tsekarw kathe grammi tou enos trigwnou  me kathe grammi tou allou trigwnou an temnontai kalontas tin InterLines()
	kai apothikeuw to simeio tomis sto p_intersection */
	C2DPoint k;
	C2DPointSet p_intersection;
	bool seg_intersect_triangle = false;
	int counter=0;

	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			k = InterLines(lineset[i],s_lineset[j], seg_intersect_triangle);
			if (seg_intersect_triangle == true) {
				m_canvas_2.add(k);
				counter++;
				p_intersection.AddCopy(k);
			}
		
		}
	}

	/*CONVEXHULL TRIGWNWN*/
	const C2DPoint* p_mouse = &p;
	C2DPoint l1 = p1;
	point_set.AddCopy(l1);
	C2DPoint l2 = p2;
	point_set.AddCopy(l2);
	C2DPoint l = p;
	point_set.AddCopy(l);
	C2DPoint s_l1 = str1;
	point_set.AddCopy(s_l1);
	C2DPoint s_l2= str2;
	point_set.AddCopy(s_l2);
	C2DPoint s_l3 = str3;
	point_set.AddCopy(s_l3);
	C2DPolygon cloud_polygon;
	cloud_polygon.Create(point_set, point_set.size());
	conv_hull_triangle.CreateConvexHull(cloud_polygon);

	/*gemisma twn trigwnwn pou einai i diafora tou convex hull apo tin enwsi twn trigwnwn*/

	double save_dist;
	C2DPointSet constr_triangles;
	double dist;
	if(p_intersection.size()>0){
		for (int i = 0; i < (conv_hull_triangle.GetPointsCount() - 1); i++) {
			const C2DPoint* p_conv = conv_hull_triangle.GetPoint(i);
			const C2DPoint* q_conv = conv_hull_triangle.GetPoint(i + 1);

			dist = 50000000;
			if (((*p_conv == str1 || *p_conv == str2 || *p_conv == str3) && ((*q_conv == p1 || *q_conv == p2 || *q_conv == p))) || ((*q_conv == str1 || *q_conv == str2 || *q_conv == str3) && ((*p_conv == p1 || *p_conv == p2 || *p_conv == p)))) {
				for (int j = 0; j < p_intersection.size(); j++) {
					save_dist = dist;
					dist = 0;
					C2DLine testline(*p_conv, *q_conv);
					C2DLine testline1(*p_conv, p_intersection[j]);
					C2DLine testline2(*q_conv, p_intersection[j]);

					dist = testline.Distance(p_intersection[j]);

					if (dist < save_dist) {
						constr_triangles.AddCopy(p_intersection[j]);
					}
					else {
						dist = save_dist;
					}
				}
				
					C2DTriangle new_triangle(*p_conv, *q_conv, *constr_triangles.GetLast());
					m_canvas_2.add(new_triangle, Colour::black, true);
				
			}
		}
	}
	
	else {
		const C2DPoint* conv_p_1 = conv_hull_triangle.GetPoint(1);
		const C2DPoint* conv_p_3 = conv_hull_triangle.GetPoint(3);
		const C2DPoint* conv_p_0 = conv_hull_triangle.GetPoint(0);
		const C2DPoint* conv_p_4 = conv_hull_triangle.GetPoint(4);

		C2DPoint viol_p1 = *conv_p_1;
		C2DPoint viol_p2 = *conv_p_3;
		C2DPoint viol_p0 = *conv_p_0;
		C2DPoint viol_p4 = *conv_p_4;
		C2DLine line(l1, l2);
		if (line.IsOnRight(*p_mouse)) {

			if ((viol_p1 != *p_mouse) && (viol_p0 != *p_mouse) && (viol_p2 != *p_mouse) && (viol_p4 != *p_mouse)) {
				C2DTriangle new_triangle1(l2, s_l1, *p_mouse);
				C2DTriangle new_triangle2(s_l1, s_l3, *p_mouse);
				C2DTriangle new_triangle3(s_l3, l1, *p_mouse);
				m_canvas_2.add(new_triangle1, Colour::black, true);
				m_canvas_2.add(new_triangle2, Colour::black, true);
				m_canvas_2.add(new_triangle3, Colour::black, true);
			}
			else if ((viol_p2 == *p_mouse) || (viol_p4 == *p_mouse)) {
				if (conv_hull_triangle.GetPointsCount() == 6) {
					C2DTriangle new_triangle1(*p_mouse, s_l3, s_l1);
					C2DTriangle new_triangle2(l2, s_l1, *p_mouse);
					m_canvas_2.add(new_triangle1, Colour::black, true);
					m_canvas_2.add(new_triangle2, Colour::black, true);
				}
				else {
					C2DTriangle new_triangle1(l2, s_l3, s_l1);
					C2DTriangle new_triangle2(l2, s_l3, *p_mouse);
					C2DTriangle new_triangle3(s_l3, s_l2, *p_mouse);
					m_canvas_2.add(new_triangle1, Colour::black, true);
					m_canvas_2.add(new_triangle2, Colour::black, true);
					m_canvas_2.add(new_triangle3, Colour::black, true);
				}

			}
			else if ((viol_p1 == *p_mouse) || (viol_p0 == *p_mouse)) {

				if (conv_hull_triangle.GetPointsCount() == 6) {
					C2DTriangle new_triangle1(*p_mouse, s_l3, s_l1);
					C2DTriangle new_triangle2(l1, s_l3, *p_mouse);
					m_canvas_2.add(new_triangle1, Colour::black, true);
					m_canvas_2.add(new_triangle2, Colour::black, true);
				}
				else {
					C2DTriangle new_triangle1(*p_mouse, s_l2, s_l1);
					C2DTriangle new_triangle2(s_l1, l1, s_l3);
					C2DTriangle new_triangle3(s_l1, l1, *p_mouse);
					m_canvas_2.add(new_triangle1, Colour::black, true);
					m_canvas_2.add(new_triangle2, Colour::black, true);
					m_canvas_2.add(new_triangle3, Colour::black, true);

				}
			}
		}
	}
}

void SceneIntersections::Task3(const C2DPoint &p)
{
    m_circle_2.SetCentre(p);
    m_canvas_3.clear();
    m_canvas_3.add(p, Colour::cyan);
    m_canvas_3.add(m_circle_2, Colour::cyan);

    /*suntetagmenes statherou kuklou*/
	const double x1 = m_circle_1.GetCentre().x;
    const double y1 = m_circle_1.GetCentre().y;
    const double r1 = m_circle_1.GetRadius();
	
	/*suntetagmenes kinitou kuklou*/
	const double x2 = m_circle_2.GetCentre().x;
    const double y2 = m_circle_2.GetCentre().y;
    const double r2 = m_circle_2.GetRadius();

	C2DPoint s_centre(x1, y1);
	C2DPoint centre(x2, y2);
	C2DLine line_centres(s_centre, centre);
	
	double dist_centres = line_centres.GetLength();
	C2DPoint p_betw_centr;
	C2DPoint i1, i2;
	double s_thetarad;
	double s_theta;
	double theta;
	double thetarad;
	C2DPoint testpoint;
	C2DLine testline(i1, s_centre);
	C2DPointSet s_pointset;
	
	/*an oi kukloi temnontai se duo simeia */
	if ((r1-r2+1.5)<dist_centres && dist_centres < (r1 + r2)) {
		double a = (r1*r1 - r2*r2 + dist_centres*dist_centres) / (2 * dist_centres);//tupos gia to "a" opws sthn anafora//
		double height = abs(sqrt(r1*r1 - a*a));
		p_betw_centr.x= (x1 + a*(x2 - x1) / dist_centres);
		p_betw_centr.y = (y1 + a*(y2 - y1) / dist_centres);

		i1.x = p_betw_centr.x + height* (y2 - y1) / dist_centres;
		i1.y = p_betw_centr.y - height*(x2 - x1) / dist_centres;
		i2.x = p_betw_centr.x - height* (y2 - y1) / dist_centres;
		i2.y = p_betw_centr.y + height*(x2 - x1) / dist_centres;
		
		/*STATHEROS KYKLOS DEIGMATOLIPSIA*/
		double s_th_max1 = (RadToDeg(atan2(i1.y - s_centre.y, i1.x - s_centre.x)));
		if (s_th_max1 < 0) {
			s_th_max1 = s_th_max1 + 360;
		}
		double s_th_max2 = (RadToDeg(atan2(i2.y - s_centre.y, i2.x - s_centre.x)));
		if (s_th_max2 < 0) {
			s_th_max2 = s_th_max2 + 360;
		}
		if (s_th_max1 > s_th_max2) {
			s_th_max1 = s_th_max1 - 360;
		}
		for (theta = s_th_max1; theta < s_th_max2 + 1; theta = theta + 1) {
			thetarad = theta*pi / 180;

			testpoint.x = s_centre.x + r1*cos(thetarad);
			testpoint.y = s_centre.y + r1*sin(thetarad);
			m_canvas_3.add(testpoint);
			s_pointset.AddCopy(testpoint);

		}

		/*KINITOS KYKLOS DEIGMATOLIPSIA*/

			double  th_max1 = (RadToDeg(atan2(i1.y - centre.y, i1.x - centre.x)));
		if (th_max1 < 0) {
			th_max1 = th_max1 + 360;
		}
		double  th_max2 = (RadToDeg(atan2(i2.y - centre.y, i2.x - centre.x)));
		if (th_max2 < 0) {
			th_max2 = th_max2 + 360;
		}
		if (th_max2 > th_max1) {
			th_max2 = th_max2 - 360;
		}


		for (theta = th_max2; theta < th_max1 + 1; theta = theta + 1) {
			thetarad = theta*pi / 180;

			testpoint.x = centre.x + r2*cos(thetarad);
			testpoint.y = centre.y + r2*sin(thetarad);
			m_canvas_3.add(testpoint);
			s_pointset.AddCopy(testpoint);

		}

		/*DHMIOURGIA POLYGWNOU APO TA SHMEIA*/
		polygon.Create(s_pointset, true); 
		
	}
	/*an oi kukloi temnontai se ena simeio*/
	else if ((((r1 + r2 -1)<dist_centres) && (dist_centres< (r1 + r2+1))) || (((r1 - r2 - 1) < dist_centres) && (dist_centres < (r1 - r2 + 1)))) {
		double a = r1;
		p_betw_centr.x = (x1 + a*(x2 - x1) / dist_centres);
		p_betw_centr.y = (y1 + a*(y2 - y1) / dist_centres);
		i1.x = p_betw_centr.x;
		i1.y = p_betw_centr.y;
		
	}

    m_canvas_3.add(i1, Colour::red);
    m_canvas_3.add(i2, Colour::red);
}

/* Drawing */

void SceneIntersections::draw()
{
    enterPixelMode();
    m_canvas_0.draw();
    m_canvas_1.draw();
    m_canvas_2.draw();
	vvr::draw(polygon, Colour::darkRed,true);
	vvr::draw(conv_hull_triangle, Colour::blue);

	m_canvas_3.draw();

}

/* Application Entry Point */

int main(int argc, char* argv[])
{
    return vvr::mainLoop(argc, argv, new SceneIntersections);
}

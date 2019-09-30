#ifndef SCENE_INTERSECTIONS_H
#define SCENE_INTERSECTIONS_H

#include <VVRScene/scene.h>
#include <VVRScene/canvas.h>
#include <VVRScene/utils.h>
#include <iostream>
#include <fstream>
#include <string>
#include <ctime>
#include <algorithm>
#include <GeoLib.h>

class SceneIntersections : public vvr::Scene
{
public:
    SceneIntersections();

    const char* getName() const override {
        return "UNIVERSITY OF PATRAS - VVR GROUP - COMPUTATIONAL GEOMETRY LAB";
    }

protected: // Overriden methods
    void draw() override;
    void reset() override;
    void mousePressed(int x, int y, int modif) override;
    void mouseMoved(int x, int y, int modif) override;

private: // Methods
    void Task1(const C2DPoint &p);
    void Task2(const C2DPoint &p);
    void Task3(const C2DPoint &p);

private: // Data
    C2DLine         m_bound_horizontal;
    C2DLine         m_bound_vertical;
    vvr::Canvas2D   m_canvas_0;
    vvr::Canvas2D   m_canvas_1;
    vvr::Canvas2D   m_canvas_2;
    vvr::Canvas2D   m_canvas_3;
    C2DTriangle     m_triangle_1;
    C2DTriangle     m_triangle_2;
    C2DCircle       m_circle_1;
    C2DCircle       m_circle_2;
    C2DLine         m_line_1;
    C2DLine         m_line_2;
	C2DPolygon polygon;
	C2DPolygon conv_hull_triangle;
	C2DPoint InterLines(C2DLine line1, C2DLine line2,bool& flag);
};

#endif // SCENE_INTERSECTIONS_H

//
//  Mesh2D.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitching__Mesh2D__
#define __UglyMan_Stitching__Mesh2D__

#include "../Configure.h"
#include "Util/Transform.h"

const int EDGE_VERTEX_SIZE = 2;

class Edge {
public:
	Edge(const int _e1, const int _e2) {
		indices[0] = _e1;
		indices[1] = _e2;
	}
	int indices[EDGE_VERTEX_SIZE];
private:
};

class Indices { /* 3 or 4 */
public:
	std::vector<int> indices;
	Indices() {

	}
	Indices(const int _i0, const int _i1, const int _i2) {
		indices.emplace_back(_i0);
		indices.emplace_back(_i1);
		indices.emplace_back(_i2);
	}
	Indices(const int _i0, const int _i1, const int _i2, const int _i3) {
		indices.emplace_back(_i0);
		indices.emplace_back(_i1);
		indices.emplace_back(_i2);
		indices.emplace_back(_i3);
	}
private:
};

class InterpolateVertex {
public:
	int polygon;
	std::vector<double> weights;
	InterpolateVertex() {
		polygon = -1;
	}
	InterpolateVertex(const InterpolateVertex& _iv) {
		polygon = _iv.polygon;
		weights = _iv.weights;
	}
	InterpolateVertex(const int _polygon,
		const double _w0, const double _w1, const double _w2) {
		polygon = _polygon;
		weights.emplace_back(_w0);
		weights.emplace_back(_w1);
		weights.emplace_back(_w2);
	}
	InterpolateVertex(const int _polygon,
		const std::vector<double>& _weights) {
		polygon = _polygon;
		weights = _weights;
	}
private:
};

class Mesh2D {
public:
	int nw, nh;
	double lw, lh;
	Mesh2D(const int _cols, const int _rows);
	virtual ~Mesh2D();
	virtual const std::vector<Point2>& getVertices() const = 0;
	virtual const std::vector<Edge>& getEdges() const = 0;
	virtual const std::vector<Indices>& getPolygonsIndices() const = 0;
	virtual const std::vector<Indices>& getPolygonsNeighbors() const = 0;
	virtual const std::vector<Indices>& getPolygonsEdges() const = 0;
	virtual const std::vector<Indices>& getVertexStructures() const = 0;
	virtual const std::vector<Indices>& getEdgeStructures() const = 0; /* grid neighbor */
	virtual const std::vector<Indices>& getTriangulationIndices() const = 0;
	virtual const int& getPolygonVerticesCount() const = 0;
	virtual const std::vector<int>& getBoundaryVertexIndices() const = 0; /* clockwise order */
	virtual const std::vector<int>& getBoundaryEdgeIndices() const = 0;

	virtual InterpolateVertex getInterpolateVertex(const Point_<float>& _p) const = 0;
	virtual InterpolateVertex getInterpolateVertex(const Point_<double>& _p) const = 0;
	virtual InterpolateVertex getInterpolateVertex(const Point_<int>& _p) const = 0;

	virtual const std::vector<Point2>& getPolygonsCenter() const;

	template <typename T>
    inline int getGridIndexOfPoint(const Point_<T>& _p) const {
        int x_idx = min(max(int(_p.x / lw), 0), nw - 1);
        int y_idx = min(max(int(_p.y / lh), 0), nh - 1);
        return y_idx * nw + x_idx;
    }

protected:
	mutable std::vector<Point2> vertices;
	mutable std::vector<Point2> polygons_center;
	mutable std::vector<Edge> edges;
	mutable std::vector<Indices> polygons_indices;
	mutable std::vector<Indices> polygons_neighbors;
	mutable std::vector<Indices> polygons_edges;
	mutable std::vector<Indices> vertex_structures;
	mutable std::vector<Indices> edge_structures;
	mutable std::vector<Indices> triangulation_indices;
	mutable std::vector<int> boundary_vertex_indices;
	mutable std::vector<int> boundary_edge_indices;
};

#endif /* defined(__UglyMan_Stitching__Mesh2D__) */

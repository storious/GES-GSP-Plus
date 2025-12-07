//
//  MeshGrid.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitching__MeshGrid__
#define __UglyMan_Stitching__MeshGrid__

#include "Mesh2D.h"


class MeshGrid : public Mesh2D {
public:
	MeshGrid(const int _cols, const int _rows);
	const std::vector<Point2>& getVertices() const;
	const std::vector<Edge>& getEdges() const;
	const std::vector<Indices>& getPolygonsIndices() const;
	const std::vector<Indices>& getPolygonsNeighbors() const;
	const std::vector<Indices>& getPolygonsEdges() const;
	const std::vector<Indices>& getVertexStructures() const;
	const std::vector<Indices>& getEdgeStructures() const;
	const std::vector<Indices>& getTriangulationIndices() const;
	const int& getPolygonVerticesCount() const;
	const std::vector<int>& getBoundaryVertexIndices() const;
	const std::vector<int>& getBoundaryEdgeIndices() const;

	InterpolateVertex getInterpolateVertex(const Point_<float>& _p) const;
	InterpolateVertex getInterpolateVertex(const Point_<double>& _p) const;
	InterpolateVertex getInterpolateVertex(const Point_<int>& _p) const;

	template <typename T>
	InterpolateVertex getInterpolateVertexTemplate(const Point_<T>& _p) const;
private:

};

#endif /* defined(__UglyMan_Stitching__MeshGrid__) */

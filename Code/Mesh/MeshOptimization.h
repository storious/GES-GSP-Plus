//
//  MeshOptimization.h
//  UglyMan_Stitching
//
//  Created by uglyman.nothinglo on 2015/8/15.
//  Copyright (c) 2015 nothinglo. All rights reserved.
//

#ifndef __UglyMan_Stitching__MeshOptimization__
#define __UglyMan_Stitching__MeshOptimization__

#include "Feature/MultiImages.h"
#include "Util/Blending.h"

class MeshOptimization {
public:
	MeshOptimization(const MultiImages& _multi_images);

	virtual Mat solve(const BLENDING_METHODS& _blend_method, std::vector<std::vector<Point2> >& original_vertices) = 0;
protected:
	void setWeightToAlignmentTerm(const double _weight);
	void setWeightToLocalSimilarityTerm(const double _weight);
	void setWeightToGlobalSimilarityTerm(const double _weight_beta,
		const double _weight_gamma,
		const enum GLOBAL_ROTATION_METHODS _global_rotation_method);
	void setWeightToContentPreservingTerm(const double _weight);


	const MultiImages& getMultiImages() const;

	double getAlignmentTermWeight() const;
	double getLocalSimilarityTermWeight() const;
	double getGlobalSimilarityTermWeightBeta() const;
	double getGlobalSimilarityTermWeightGamma() const;
	enum GLOBAL_ROTATION_METHODS getGlobalRotationMethod() const;

	void reserveData(std::vector<Triplet<double> >& _triplets,
		std::vector<std::pair<int, double> >& _b_vector,
		const int _start_index);

	void reserveData_content(std::vector<Triplet<double> >& _triplets,
		std::vector<std::pair<int, double> >& _b_vector,
		const int _start_index);

	void prepareAlignmentTerm(std::vector<Triplet<double> >& _triplets) const;
	void prepareSimilarityTerm(std::vector<Triplet<double> >& _triplets,
		std::vector<std::pair<int, double> >& _b_vector) const;
	void prepareContentPreservingTerm(std::vector<Triplet<double> >& _triplets,
		std::vector<std::pair<int, double> >& _b_vector) const;

	std::vector<std::vector<Point2> > getImageVerticesBySolving(std::vector<Triplet<double> >& _triplets,
		const std::vector<std::pair<int, double> >& _b_vector) const;

private:

	int getAlignmentTermEquationsCount() const;

	int getVerticesCount() const;
	int getEdgesCount() const;
	int getEdgeNeighborVerticesCount() const;
	int getContentPreservingTermEquationCount() const;

	const MultiImages* multi_images;

	double alignment_weight;
	double local_similarity_weight;
	double global_similarity_weight_beta, global_similarity_weight_gamma;
	double content_preserving_weight;

	std::pair<int, int> alignment_equation; /* begin, count */
	std::pair<int, int> local_similarity_equation;
	std::pair<int, int> global_similarity_equation;
	std::pair<int, int> content_preserving_equation;
	enum GLOBAL_ROTATION_METHODS global_rotation_method;
};

#endif /* defined(__UglyMan_Stitching__MeshOptimization__) */

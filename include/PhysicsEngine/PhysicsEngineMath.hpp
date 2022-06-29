#pragma once

#include <cmath>
#include <cfloat>
#include <limits>
#include <vector>

#include "linalg.h"

// If you want to use floats instead, uncomment it here
#define USE_FLT

namespace std {
	double max(double a, float b);
	double max(float a, double b);
}

namespace PhysicsEngine {

	namespace phy_t {

#ifdef USE_FLT
#define PHYFLT float
#define PHYFLT_MAX FLT_MAX
#define PHYFLT_MIN FLT_MIN
#else
#define PHYFLT double
#define PHYFLT_MAX DBL_MAX
#define PHYFLT_MIN DBL_MIN
#endif

		typedef PHYFLT phyflt;

		typedef linalg::vec<phyflt, 2> phyvec;
		typedef linalg::mat<phyflt, 2, 2> phymat;

	}

	using namespace phy_t;


	typedef linalg::aliases::float2 fvec;
	typedef linalg::aliases::double2 dvec;
	//typedef linalg::aliases::float2x2 fmat22;
	//typedef linalg::aliases::double2x2 dmat22;

	

	using linalg::mul;
	using linalg::length;
	using linalg::dot;
	using linalg::cross;
	using linalg::inverse;
	using linalg::identity;

	extern const phyflt PI;
	extern const phyflt G;
	// extern const float EPSILON;

	extern const phyvec PHYVEC_NULL;

	// Struct used by clip_edge_to_line
	// Keeps track of up to two points
	struct ClipResult {
		uint8_t points_count = 0;
		phyvec points[2];
	};


	/*struct BoundingCircle {
		vec2 centre;
		float radius;
	};


	bool intersects(const BoundingCircle& a, const BoundingCircle& b);*/
	bool intersects(const phyvec& centre_a, phyflt radius_a, const phyvec& centre_b, phyflt radius_b);


	phyvec to_phyvec(const fvec& v);
	phyvec to_phyvec(const dvec& v);
	fvec to_fvec(const phyvec& v);
	dvec to_dvec(const phyvec& v);

	float to_float(phyflt n);
	double to_double(phyflt n);


	phyflt length_squared(const phyvec& v);
	phyvec normalise(const phyvec& v);

	// Convert to/from radians and degrees
	phyflt deg_to_rad(phyflt degrees);
	phyflt rad_to_deg(phyflt radians);

	// angle is in radians
	phymat rotation_matrix(phyflt angle);

	// model_centre is the centre of the object in world space
	phyvec to_model_space(const phyvec& point, const phyvec& model_centre, const phymat& rotation_matrix);
	phyvec to_model_space(const phyvec& point, const phymat& rotation_matrix);
	phyvec to_world_space(const phyvec& point, const phyvec& model_centre, const phymat& rotation_matrix);
	phyvec to_world_space(const phyvec& point, const phymat& rotation_matrix);

	// Support point is the furthest point in a given direction
	phyvec find_support_point(const phyvec& direction, const std::vector<phyvec>& points);

	// Get vector perpendicular to supplied vector
	phyvec perpendicular_acw(const phyvec& vector);
	phyvec perpendicular_cw(const phyvec& vector);

	// Clamp the point to the positive side of the line parallel to the normal, origin_distance from the origin (along the normal)
	// Origin_distance could also be thought as the closest point the line perpendicular to the normal passes to the origin
	ClipResult clip_edge_to_line(const phyvec edge[2], const phyvec& line_normal, phyflt origin_distance);

	// Find centroid of vector of points
	phyvec find_centroid(const std::vector<phyvec>& vertices);

	// Find furthest distance from origin to point
	phyflt find_bounding_radius(const std::vector<phyvec>& vertices);

	// Adds offset to every element in vertices
	std::vector<phyvec> translate(const std::vector<phyvec>& vertices, const phyvec& offset);


	//float gravitational_force(float mass_a, float mass_b, float distance_squared);
	
	// Use version with phyflt behind the scene so that we don't overflow
	phyflt gravitational_force(phyflt mass_a, phyflt mass_b, phyflt distance_squared, phyflt gravitational_constant = G);


	std::vector<phyvec> rect_vertices(const phyvec& size);

	std::vector<phyvec> isosceles_vertices(const phyvec& size);
	
	std::vector<phyvec> trapezium_vertices(const phyflt w_top, const phyflt w_base, const phyflt height);
}
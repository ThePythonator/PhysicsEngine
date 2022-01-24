#pragma once

#include <cmath>
#include <cfloat>
#include <limits>
#include <vector>
#include <vector>

#include "linalg.h"

namespace PhysicsEngine {
	typedef linalg::aliases::float2 vec2;
	typedef linalg::aliases::float2x2 mat22;

	using linalg::mul;
	using linalg::length;
	using linalg::dot;
	using linalg::cross;
	using linalg::inverse;
	using linalg::identity;

	extern const float PI;
	// extern const float EPSILON;

	// Struct used by clip_edge_to_line
	// Keeps track of up to two points
	struct ClipResult {
		uint8_t points_count = 0;
		vec2 points[2];
	};

	float length_squared(vec2 v);
	vec2 normalise(vec2 v);

	// Convert to/from radians and degrees
	float deg_to_rad(float degrees);
	float rad_to_deg(float radians);

	// angle is in radians
	mat22 rotation_matrix(float angle);

	// model_centre is the centre of the object in world space
	vec2 to_model_space(vec2 point, vec2 model_centre, mat22 rotation_matrix);
	vec2 to_model_space(vec2 point, mat22 rotation_matrix);
	vec2 to_world_space(vec2 point, vec2 model_centre, mat22 rotation_matrix);
	vec2 to_world_space(vec2 point, mat22 rotation_matrix);

	// Support point is the furthest point in a given direction
	vec2 find_support_point(vec2 direction, std::vector<vec2>& points);

	// Get vector perpendicular to supplied vector
	vec2 perpendicular_acw(vec2 vector);
	vec2 perpendicular_cw(vec2 vector);

	// Clamp the point to the positive side of the line parallel to the normal, origin_distance from the origin (along the normal)
	// Origin_distance could also be thought as the closest point the line perpendicular to the normal passes to the origin
	ClipResult clip_edge_to_line(vec2 edge[2], vec2 line_normal, float origin_distance);
}
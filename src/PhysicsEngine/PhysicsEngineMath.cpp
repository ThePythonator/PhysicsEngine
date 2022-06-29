#include "PhysicsEngineMath.hpp"

namespace std {
	double max(double a, float b) {
		return std::max(a, static_cast<double>(b));
	}

	double max(float a, double b) {
		return std::max(static_cast<double>(a), b);
	}
}

namespace PhysicsEngine {
	const phyflt PI = 3.141592653589793;
	const phyflt G = 6.674e-11;

	const phyvec PHYVEC_NULL{ 0.0, 0.0 };

	//const float EPSILON = 0.0001f; // todo: check if reasonable

	/*bool intersects(const BoundingCircle& a, const BoundingCircle& b) {
		float dist_squared = length_squared(a.centre - b.centre);

		float radii_sum = a.radius + b.radius;

		return dist_squared <= radii_sum * radii_sum;
	}*/

	bool intersects(const phyvec& centre_a, phyflt radius_a, const phyvec& centre_b, phyflt radius_b) {
		phyflt dist_squared = length_squared(centre_a - centre_b);

		phyflt radii_sum = radius_a + radius_b;

		return dist_squared <= radii_sum * radii_sum;
	}


	phyvec to_phyvec(const fvec& v) {
		return phyvec{ static_cast<phyflt>(v.x), static_cast<phyflt>(v.y) };
	}

	phyvec to_phyvec(const dvec& v) {
		return phyvec{ static_cast<phyflt>(v.x), static_cast<phyflt>(v.y) };
	}


	fvec to_fvec(const phyvec& v) {
		return fvec{ static_cast<float>(v.x), static_cast<float>(v.y) };
	}

	dvec to_dvec(const phyvec& v) {
		return dvec{ static_cast<double>(v.x), static_cast<double>(v.y) };
	}

	float to_float(phyflt n) {
		return static_cast<float>(n);
	}

	double to_double(phyflt n) {
		return static_cast<double>(n);
	}



	phyflt length_squared(const phyvec& v) {
		return linalg::length2(v);
	}

	phyvec normalise(const phyvec& v) {
		return linalg::normalize(v);
	}

	phyflt deg_to_rad(phyflt degrees) {
		return PI * degrees / 180.0;
	}
	phyflt rad_to_deg(phyflt radians) {
		return 180.0 * radians / PI;
	}

	phymat rotation_matrix(phyflt angle) {
		// Anticlockwise rotation
		phyflt s = std::sin(angle);
		phyflt c = std::cos(angle);
		return phymat{ {c, s}, {-s, c} };
	}


	// Convert world space to model space (in model space, shape is as if angle = 0)
	phyvec to_model_space(const phyvec& point, const phyvec& model_centre, const phymat& rotation_matrix) {
		return mul(inverse(rotation_matrix), point - model_centre);
	}

	phyvec to_model_space(const phyvec& point, const phymat& rotation_matrix) {
		return mul(inverse(rotation_matrix), point);
	}

	// Convert model space to world space (in world space, shape is at actual angle)
	phyvec to_world_space(const phyvec& point, const phyvec& model_centre, const phymat& rotation_matrix) {
		return mul(rotation_matrix, point) + model_centre;
	}

	phyvec to_world_space(const phyvec& point, const phymat& rotation_matrix) {
		return mul(rotation_matrix, point);
	}


	phyvec find_support_point(const phyvec& direction, const std::vector<phyvec>& points) {
		phyflt maximum_distance = -PHYFLT_MAX;
		uint16_t point_index = 0;

		for (uint16_t i = 0; i < points.size(); i++) {
			phyflt distance = dot(direction, points[i]);

			if (distance > maximum_distance) {
				maximum_distance = distance;
				point_index = i;
			}
		}

		return points[point_index];
	}

	phyvec perpendicular_acw(const phyvec& vector) {
		return phyvec{ -vector.y, vector.x };
	}

	phyvec perpendicular_cw(const phyvec& vector) {
		return phyvec{ vector.y, -vector.x };
	}

	ClipResult clip_edge_to_line(const phyvec edge[2], const phyvec& line_normal, phyflt origin_distance) {
		ClipResult result;

		// Calculate signed distance along normal from line
		phyflt signed_distance[2] = {
			dot(edge[0], line_normal) - origin_distance,
			dot(edge[1], line_normal) - origin_distance
		};

		if (signed_distance[0] <= 0.0) {
			result.points[result.points_count++] = edge[0];
		}

		if (signed_distance[1] <= 0.0) {
			result.points[result.points_count++] = edge[1];
		}

		// Is the sign of the distances different?
		// We also check that points_count is < 2 so that we don't accidentally add a third point (although it should be impossible)
		if (signed_distance[0] * signed_distance[1] < 0.0 && result.points_count < 2) {
			// Edge passes through the clamping line
			// One of the distances was negative, so we need to add the intersection point

			// Calculate intersection
			phyflt ratio = signed_distance[0] / (signed_distance[0] + signed_distance[1]);
			result.points[result.points_count++] = edge[0] + ratio * (edge[1] - edge[0]);
		}

		return result;
	}


	phyvec find_centroid(const std::vector<phyvec>& vertices) {
		// Centroid is centre of mass, initially set to 0,0
		phyvec centroid;

		phyflt area = 0.0f;

		// Split polygon into many triangles
		for (uint16_t i = 0; i < vertices.size(); i++) {
			uint16_t next_i = i + 1 < vertices.size() ? i + 1 : 0;

			// Use cross product to calculate area of trapezium
			phyflt triangle_area = 0.5 * cross(vertices[i], vertices[next_i]);

			// Halve area of trapezium to get area of triangle
			area += triangle_area;

			// Centroid is weighted average of vertices (one vertex is 0,0)
			centroid += triangle_area * (vertices[i] + vertices[next_i]);
		}

		// Need to do the averaging bit now:
		// Divide by total area, and divide by three for the vertices of each triangle (one vertex is 0,0)
		centroid /= (area * 3.0f);

		return centroid;
	}

	phyflt find_bounding_radius(const std::vector<phyvec>& vertices) {
		phyflt max_squared_dist = 0.0f;

		for (phyvec vertex : vertices) {
			max_squared_dist = std::max(max_squared_dist, length_squared(vertex));
		}

		return std::sqrt(max_squared_dist);
	}


	std::vector<phyvec> translate(const std::vector<phyvec>& vertices, const phyvec& offset) {
		std::vector<phyvec> new_vertices;

		for (const phyvec& vertex : vertices) {
			new_vertices.push_back(vertex + offset);
		}

		return new_vertices;
	}




	/*float gravitational_force(float mass_a, float mass_b, float distance_squared) {
		return G * mass_a * mass_b / distance_squared;
	}*/

	phyflt gravitational_force(phyflt mass_a, phyflt mass_b, phyflt distance_squared, phyflt gravitational_constant) {
		return gravitational_constant * mass_a * mass_b / distance_squared;
	}



	std::vector<phyvec> rect_vertices(const phyvec& size) {
		phyflt half_width = size.x / 2;
		phyflt half_height = size.y / 2;

		return {
			{ -half_width, -half_height },
			{ -half_width, half_height },
			{ half_width, half_height },
			{ half_width, -half_height }
		};
	}

	// Oriented with base in positive y direction
	//
	//    .     |
	//   / \    |  +ve y
	//  /___\   V
	//
	std::vector<phyvec> isosceles_vertices(const phyvec& size) {
		phyflt half_width = size.x / 2;
		phyflt half_height = size.y / 2;

		return {
			{ 0, -half_height },
			{ -half_width, half_height },
			{ half_width, half_height }
		};
	}


	// Trapezium, symmetrical along y axis
	std::vector<phyvec> trapezium_vertices(const phyflt w_top, const phyflt w_base, const phyflt height) {
		phyflt half_top = w_top / 2;
		phyflt half_base = w_base / 2;
		phyflt half_height = height / 2;

		return {
			{ -half_top, -half_height },
			{ -half_base, half_height },
			{ half_base, half_height },
			{ half_top, -half_height }
		};
	}
}
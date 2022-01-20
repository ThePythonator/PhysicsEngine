#include "Shape.hpp"

namespace PhysicsEngine {

	// Shape

	Shape::Shape() {

	}

	// Circle

	Circle::Circle(float _radius) : Shape() {
		radius = _radius;
	}

	Shape::ShapeType Circle::get_type() {
		return ShapeType::CIRCLE;
	}

	float Circle::get_area() {
		// Area of circle
		return PI * radius * radius;
	}

	float Circle::get_moment_of_inertia(float density) {
		// Not certain this is correct
		return 0.5f * get_area() * density * radius * radius;
	}

	vec2 Circle::get_centroid() {
		// Redundant
		return vec2{ 0.0f, 0.0f };
	}

	// Polygon

	// Note that vertices must be in anti-clockwise order
	Polygon::Polygon(std::vector<vec2> _vertices) : Shape() {
		vertices = _vertices;

		reorder_vertices();

		vec2 centroid = get_centroid();

		// Shift vertices so centre is at centroid (centre of mass)
		for (uint16_t i = 0; i < vertices.size(); i++) {
			vertices[i] -= centroid;
		}

		calculate_face_normals();
	}

	Shape::ShapeType Polygon::get_type() {
		return ShapeType::POLYGON;
	}

	float Polygon::get_area() {
		float area = 0.0f;

		// Split polygon into many triangles
		for (uint16_t i = 0; i < vertices.size(); i++) {
			uint16_t next_i = i + 1 < vertices.size() ? i + 1 : 0;

			// Use cross product to calculate area of trapezium, then half to get triangle area
			area += 0.5f * cross(vertices[i], vertices[next_i]);
		}
		return area;
	}

	float Polygon::get_moment_of_inertia(float density) {
		// This is correct :)
		float moment_of_inertia = 0.0f;

		for (uint16_t i = 0; i < vertices.size(); i++) {
			uint16_t next_i = i + 1 < vertices.size() ? i + 1 : 0;

			float dot_sum = dot(vertices[i], vertices[i]) + dot(vertices[i], vertices[next_i]) + dot(vertices[next_i], vertices[next_i]);
			moment_of_inertia += cross(vertices[i], vertices[next_i]) * dot_sum;
		}
		
		return density * moment_of_inertia / 12.0f;
	}

	vec2 Polygon::get_centroid() {
		// Similar to get_area

		// Centroid is centre of mass, initially set to 0,0
		vec2 centroid{ 0.0f, 0.0f };

		float area = 0.0f;

		// Split polygon into many triangles
		for (uint16_t i = 0; i < vertices.size(); i++) {
			uint16_t next_i = i + 1 < vertices.size() ? i + 1 : 0;

			// Use cross product to calculate area of trapezium
			float triangle_area = 0.5f * cross(vertices[i], vertices[next_i]);

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

	void Polygon::calculate_face_normals() {
		// Calculate a vector perpendicular to each face

		for (uint16_t i = 0; i < vertices.size(); i++) {
			uint16_t next_i = i + 1 < vertices.size() ? i + 1 : 0;

			// Face is defined as the vector from vertices[i] to vertices[next_i]
			// We need to calculate the vector normal to this, then normalise it

			vec2 face = vertices[next_i] - vertices[i];

			// Vertices are specified in acw order, so need normal in same direction (so it points outward)
			vec2 normal = normalise(perpendicular_cw(face));

			/*printf("v1: %f, %f\n", vertices[i].x, vertices[i].y);
			printf("v2: %f, %f\n", vertices[next_i].x, vertices[next_i].y);
			printf("face: %f, %f\n", face.x, face.y);
			printf("normal: %f, %f\n", normal.x, normal.y);*/

			face_normals.push_back(normal);
		}
	}

	void Polygon::reorder_vertices() {
		// Sort vertices by angle (starting at angle closest to 0)
		// Reorders vertices into anticlockwise order, around the centroid
		// Assumes vertices are already shifted by centroid

		// Use std::pair with:
		//		first:		vertex
		//		second:		angle


		// Argh: copying everywhere! (for some reason doesn't like vec2& in the pair, or const vec2&)

		std::vector<std::pair<vec2, float>> vertex_angle_pairs;

		// Calculate angles
		for (const vec2& vertex : vertices) {
			vertex_angle_pairs.push_back(std::pair<vec2, float>(vertex, std::atan2(vertex.y, vertex.x)));
		}

		// Now sort by angle
		std::sort(vertex_angle_pairs.begin(), vertex_angle_pairs.end(), [](const std::pair<vec2, float>& lhs, const std::pair<vec2, float>& rhs) { return lhs.second < rhs.second; });

		// Copy back to vertices
		vertices.clear();
		for (uint16_t i = 0; i < vertex_angle_pairs.size(); i++) {
			//printf("angle, ordered: %f\n", vertex_angle_pairs[i].second);
			vertices.push_back(vertex_angle_pairs[i].first);
		}
	}


	Polygon* create_rect(vec2 size) {
		float half_width = size.x / 2.0f;
		float half_height = size.y / 2.0f;

		// Vertices must be anti-clockwise
		// Start top left
		std::vector<vec2> vertices = {
			vec2{-half_width, -half_height},
			vec2{-half_width, half_height},
			vec2{half_width, half_height},
			vec2{half_width, -half_height}
		};
		return new Polygon(vertices);
	}
}
#pragma once

#include <vector>
#include <algorithm>

#include "PhysicsEngineMath.hpp"

namespace PhysicsEngine {

	class Shape {
	public:
		enum class ShapeType {
			CIRCLE,
			POLYGON,
			SHAPE_COUNT
		};

		Shape();

		virtual ShapeType get_type() = 0;
		virtual float get_area() = 0;
		virtual float get_moment_of_inertia(float density) = 0;
		virtual vec2 get_centroid() = 0;
	};

	class Circle : public Shape {
	public:
		Circle(float _radius = 1.0f);
		
		ShapeType get_type();
		float get_area();
		float get_moment_of_inertia(float density);
		vec2 get_centroid();

		float radius;
	};

	class Polygon : public Shape {
	public:
		Polygon(std::vector<vec2> _vertices = { vec2{ 0.5f, -0.5f }, vec2{ 0.5f, 0.5f }, vec2{ -0.5f, 0.5f }, vec2{ -0.5f, -0.5f } });

		ShapeType get_type();
		float get_area();
		float get_moment_of_inertia(float density);
		vec2 get_centroid();

		std::vector<vec2> vertices;
		std::vector<vec2> face_normals;

	private:
		void calculate_face_normals();
		void reorder_vertices();
	};

	Polygon* create_rect(vec2 size);
}
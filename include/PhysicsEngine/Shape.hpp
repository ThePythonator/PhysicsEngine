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
		virtual phyflt get_area() = 0;
		virtual phyflt get_moment_of_inertia(phyflt density) = 0;
		virtual phyvec get_centroid() = 0;
		virtual phyflt get_bounding_radius() = 0;
	};

	class Circle : public Shape {
	public:
		Circle(phyflt _radius = 1.0);
		
		ShapeType get_type();
		phyflt get_area();
		phyflt get_moment_of_inertia(phyflt density);
		phyvec get_centroid();
		phyflt get_bounding_radius();

		phyflt radius;
	};

	class Polygon : public Shape {
	public:
		Polygon(std::vector<phyvec> _vertices = { { 0.5, -0.5 }, { 0.5, 0.5 }, { -0.5, 0.5 }, { -0.5, -0.5 } });

		ShapeType get_type();
		phyflt get_area();
		phyflt get_moment_of_inertia(phyflt density);
		phyvec get_centroid();
		phyflt get_bounding_radius();

		std::vector<phyvec> vertices;
		std::vector<phyvec> face_normals;

	private:
		void calculate_face_normals();
		void reorder_vertices();
	};

	Polygon* create_rect(const phyvec& size);
}
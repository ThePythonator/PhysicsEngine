#pragma once

#include "PhysicsEngineMath.hpp"

#include "RigidBody.hpp"

namespace PhysicsEngine {
	struct ContactData {
		phyvec contact_point;
		double penetration_distance;
	};

	struct CollisionInformation {
		phyvec collision_normal;

		uint8_t contact_count;
		ContactData contact_data[2];
	};

	// Function signature for collision detection functions
	typedef CollisionInformation(*CollisionDetectionFunction)(RigidBody* a, RigidBody* b);

	// Jump table
	extern CollisionDetectionFunction collision_detection_functions[static_cast<int>(Shape::ShapeType::SHAPE_COUNT)][static_cast<int>(Shape::ShapeType::SHAPE_COUNT)];

	// Collision detection functions
	CollisionInformation circle_to_circle(RigidBody* a, RigidBody* b);
	CollisionInformation circle_to_polygon(RigidBody* a, RigidBody* b);
	CollisionInformation polygon_to_circle(RigidBody* a, RigidBody* b);
	CollisionInformation polygon_to_polygon(RigidBody* a, RigidBody* b);
}
#pragma once

#include "Shape.hpp"

namespace PhysicsEngine {
	// Information about a material
	struct Material {
		Material(phyflt _static_friction = 0.5, phyflt _dynamic_friction = 0.3, phyflt _restitution = 0.5, phyflt _density = 1.0);

		phyflt static_friction, dynamic_friction;
		phyflt restitution;
		phyflt density;
	};

	// Structure containing information about a body
	class RigidBody {
	public:
		RigidBody();
		RigidBody(Shape* _shape, Material* _material, phyvec _centre, phyflt _angle = 0.0, bool infinite_mass = false);

		void set_layers(uint32_t layers);
		uint32_t get_layers();

		void apply_force(const phyvec& _force, const phyvec& vector_to_contact = PHYVEC_NULL);

		phymat get_rotation_matrix() const ;

		Shape* shape = nullptr;
		Material* material = nullptr;

		phyflt bounding_radius = 0.0;

		phyvec centre, velocity, force;

		// angle is in radians
		phyflt angle = 0.0;
		phyflt angular_velocity = 0.0;
		phyflt torque = 0.0;

		phyflt mass = 0.0;
		phyflt inverse_mass = 0.0;

		phyflt moment_of_inertia = 0.0;
		phyflt inverse_moment_of_inertia = 0.0;

		// IDs can be used to:
		//		-	look up sprites and corresponding offsets to render at
		//		-	group rigidbodies when searching
		std::vector<uint32_t> ids;

	private:
		// Defaults to do nothing (i.e. angle = 0.0)
		mutable phymat cached_rotation_matrix = identity;

		mutable phyflt last_angle = 0.0;

		uint32_t _layers = 1; // Each bit corresponds to a layer
	};
}
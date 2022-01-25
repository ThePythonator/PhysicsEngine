#pragma once

#include "Shape.hpp"

// Maybe rename file
#include "GraphicsData.hpp"

namespace PhysicsEngine {
	// Information about a material
	struct Material {
		Material(float _static_friction = 0.5f, float _dynamic_friction = 0.3f, float _restitution = 0.5f, float _density = 1.0f);

		float static_friction, dynamic_friction;
		float restitution;
		float density;
	};

	// Structure containing information about a body
	class RigidBody {
	public:
		RigidBody();
		RigidBody(Shape* _shape, Material* _material, vec2 _centre, float _angle = 0.0f, bool infinite_mass = false);

		void set_layers(uint32_t layers);
		uint32_t get_layers();

		void apply_force(const vec2& _force, const vec2& vector_to_contact = vec2{ 0.0f, 0.0f });

		mat22 get_rotation_matrix();

		Shape* shape = nullptr;
		Material* material = nullptr;

		vec2 centre, velocity, force;

		// angle is in radians
		float angle = 0.0f;
		float angular_velocity = 0.0f;
		float torque = 0.0f;

		float mass = 0.0f;
		float inverse_mass = 0.0f;

		float moment_of_inertia = 0.0f;
		float inverse_moment_of_inertia = 0.0f;

		std::vector<ImageRenderData> image_render_data;

	private:
		// Defaults to do nothing (i.e. angle = 0.0f)
		mat22 cached_rotation_matrix = identity;

		float last_angle = 0.0f;

		uint32_t _layers = 1; // Each bit corresponds to a layer
	};
}
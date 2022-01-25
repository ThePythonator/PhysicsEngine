#include "RigidBody.hpp"

namespace PhysicsEngine {
	// Material
	Material::Material(float _static_friction, float _dynamic_friction, float _restitution, float _density) {
		static_friction = _static_friction;
		dynamic_friction = _dynamic_friction;
		restitution = _restitution;
		density = _density;
	}

	// RigidBody

	RigidBody::RigidBody() {

	}

	RigidBody::RigidBody(Shape* _shape, Material* _material, vec2 _centre, float _angle, bool infinite_mass) {
		shape = _shape;
		material = _material;
		centre = _centre;// - shape->get_centroid(); // Note: CENTROID IS 0 here - because we've already shifted the vertices so that centroid is at 0,0
		angle = _angle;

		// Set rotation matrix
		cached_rotation_matrix = rotation_matrix(angle);
		last_angle = angle;

		// If density is <= 0 or FLT_MAX, then infinite density is assumed - i.e. keep all mass values as 0.0f
		if (material->density > 0.0f && material->density < FLT_MAX && !infinite_mass) {
			// Calulate mass and moment of inertia as well as inverses
			mass = material->density * shape->get_area();
			inverse_mass = 1.0f / mass;

			moment_of_inertia = shape->get_moment_of_inertia(material->density);
			inverse_moment_of_inertia = 1.0f / moment_of_inertia;
		}
	}

	void RigidBody::set_layers(uint32_t layers) {
		_layers = layers;
	}

	uint32_t RigidBody::get_layers() {
		return _layers;
	}

	void RigidBody::apply_force(const vec2& _force, const vec2& vector_to_contact) {
		force += _force;
		torque += cross(vector_to_contact, _force);
	}

	mat22 RigidBody::get_rotation_matrix() {
		// Angles are defined as being anticlockwise from the positive x axis
		if (last_angle != angle) {
			cached_rotation_matrix = rotation_matrix(angle);
			last_angle = angle;
		}

		return cached_rotation_matrix;
	}
}
#include "RigidBody.hpp"

namespace PhysicsEngine {
	// Material
	Material::Material(phyflt _static_friction, phyflt _dynamic_friction, phyflt _restitution, phyflt _density) {
		static_friction = _static_friction;
		dynamic_friction = _dynamic_friction;
		restitution = _restitution;
		density = _density;
	}

	// RigidBody

	RigidBody::RigidBody() {

	}

	RigidBody::RigidBody(Shape* _shape, Material* _material, phyvec _centre, phyflt _angle, bool infinite_mass) {
		shape = _shape;
		material = _material;
		centre = _centre;// - shape->get_centroid(); // Note: CENTROID IS 0 here - because we've already shifted the vertices so that centroid is at 0,0
		angle = _angle;

		// Set rotation matrix
		cached_rotation_matrix = rotation_matrix(angle);
		last_angle = angle;

		// If density is <= 0 or PHYFLT_MAX, then infinite density is assumed - i.e. keep all mass values as 0.0
		if (material->density > 0.0 && material->density < PHYFLT_MAX && !infinite_mass) {
			// Calulate mass and moment of inertia as well as inverses
			mass = material->density * shape->get_area();
			inverse_mass = 1.0 / mass;

			moment_of_inertia = shape->get_moment_of_inertia(material->density);
			inverse_moment_of_inertia = 1.0 / moment_of_inertia;
		}

		bounding_radius = shape->get_bounding_radius();
	}

	void RigidBody::set_layers(uint32_t layers) {
		_layers = layers;
	}

	uint32_t RigidBody::get_layers() {
		return _layers;
	}

	void RigidBody::apply_force(const phyvec& _force, const phyvec& vector_to_contact) {
		force += _force;
		torque += cross(vector_to_contact, _force);
	}

	phymat RigidBody::get_rotation_matrix() const {
		// Angles are defined as being anticlockwise from the positive x axis
		if (last_angle != angle) {
			cached_rotation_matrix = rotation_matrix(angle);
			last_angle = angle;
		}

		return cached_rotation_matrix;
	}
}
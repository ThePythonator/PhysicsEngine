#include "Constraints.hpp"

namespace PhysicsEngine {
	// Constraint
	Constraint::Constraint() {

	}

	Constraint::Constraint(RigidBody* _a, RigidBody* _b, vec2 _offset_a, vec2 _offset_b) {
		a = _a;
		b = _b;
		offset_a = _offset_a;
		offset_b = _offset_b;
	}

	void Constraint::apply_force() {
		vec2 force = calculate_force(); // todo: apply torque too
		a->apply_force(force, to_world_space(offset_a, a->get_rotation_matrix()));
		b->apply_force(-force, to_world_space(offset_b, b->get_rotation_matrix()));
	}

	bool Constraint::is_broken() {
		return broken;
	}

	// Spring

	Spring::Spring(RigidBody* _a, RigidBody* _b, vec2 _offset_a, vec2 _offset_b, float _natural_length, float _modulus_of_elasticity, float max_length_factor) : Constraint(_a, _b, _offset_a, _offset_b), natural_length(_natural_length), modulus_of_elasticity(_modulus_of_elasticity), max_length(_natural_length * std::max(max_length_factor, 1.0f)) {
		
	}

	// Returns force exerted by constraint on A (B has equal and opposite force)
	vec2 Spring::calculate_force() {
		// NOTE: MIGHT NEED TO BE DAMPED

		if (broken) {
			// Broken so no force exerted
			return vec2{ 0.0f, 0.0f };
		}

		// T = mx/L
		vec2 a_to_b = (b->centre + to_world_space(offset_b, b->get_rotation_matrix())) - (a->centre + to_world_space(offset_a, a->get_rotation_matrix()));
		float a_to_b_length = length(a_to_b);

		if (a_to_b_length > max_length) {
			// Exceeded max length, so broken (no longer provides a force, and flag set so it can be removed)
			broken = true;
			return vec2{ 0.0f, 0.0f };
		}

		vec2 a_to_b_normalised = a_to_b / a_to_b_length;
		float force_magnitude = modulus_of_elasticity * (a_to_b_length - natural_length) / natural_length;

		// Force is in same direction as a_to_b for A, opposite for B (applied by PhysicsManager)
		return force_magnitude * a_to_b_normalised;
	}

	// String

	String::String(RigidBody* _a, RigidBody* _b, vec2 _offset_a, vec2 _offset_b, float _natural_length , float _modulus_of_elasticity, float max_length_factor) : Spring(_a, _b, _offset_a, _offset_b, _natural_length, _modulus_of_elasticity, max_length_factor) {
		
	}

	vec2 String::calculate_force() {
		// Only apply a force if extension is positive
		return length_squared(b->centre - a->centre) > natural_length * natural_length ? Spring::calculate_force() : vec2{ 0.0f, 0.0f };
	}
}
#include "Constraints.hpp"

namespace PhysicsEngine {
	const phyflt MINIMUM_NATURAL_LENGTH = 0.01;

	// Constraint
	Constraint::Constraint() {

	}

	Constraint::Constraint(RigidBody* _a, RigidBody* _b, phyvec _offset_a, phyvec _offset_b) {
		a = _a;
		b = _b;
		offset_a = _offset_a;
		offset_b = _offset_b;
	}

	void Constraint::apply_force() {
		phyvec force = calculate_force();

		a->apply_force(force, to_world_space(offset_a, a->get_rotation_matrix()));
		b->apply_force(-force, to_world_space(offset_b, b->get_rotation_matrix()));
	}

	bool Constraint::is_broken() {
		return broken;
	}

	// Spring

	Spring::Spring(RigidBody* _a, RigidBody* _b, phyvec _offset_a, phyvec _offset_b, phyflt _natural_length, phyflt _modulus_of_elasticity, phyflt max_extension) : Constraint(_a, _b, _offset_a, _offset_b), natural_length(std::max(_natural_length, MINIMUM_NATURAL_LENGTH)), modulus_of_elasticity(_modulus_of_elasticity), max_length(_natural_length + std::max(max_extension, 0.0f)) {
		
	}

	// Returns force exerted by constraint on A (B has equal and opposite force)
	phyvec Spring::calculate_force() {
		// NOTE: MIGHT NEED TO BE DAMPED

		if (broken) {
			// Broken so no force exerted
			return PHYVEC_NULL;
		}

		// T = mx/L
		phyvec a_to_b = (b->centre + to_world_space(offset_b, b->get_rotation_matrix())) - (a->centre + to_world_space(offset_a, a->get_rotation_matrix()));
		phyflt a_to_b_length = length(a_to_b);

		if (a_to_b_length > max_length) {
			// Exceeded max length, so broken (no longer provides a force, and flag set so it can be removed)
			broken = true;
			return PHYVEC_NULL;
		}
		else if (a_to_b_length <= 0.0) {
			// Can't divide by 0, negative length is impossible, so exit now
			return PHYVEC_NULL;
		}

		phyvec a_to_b_normalised = a_to_b / a_to_b_length;
		phyflt force_magnitude = modulus_of_elasticity * (a_to_b_length - natural_length) / natural_length;

		// Force is in same direction as a_to_b for A, opposite for B (applied by PhysicsManager)
		return force_magnitude * a_to_b_normalised;
	}

	// String

	String::String(RigidBody* _a, RigidBody* _b, phyvec _offset_a, phyvec _offset_b, phyflt _natural_length, phyflt _modulus_of_elasticity, phyflt max_extension) : Spring(_a, _b, _offset_a, _offset_b, _natural_length, _modulus_of_elasticity, max_extension) {
		
	}

	phyvec String::calculate_force() {
		// Only apply a force if extension is positive
		return length_squared(b->centre - a->centre) > natural_length * natural_length ? Spring::calculate_force() : PHYVEC_NULL;
	}
}
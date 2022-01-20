#pragma once

#include "PhysicsEngineMath.hpp"
#include "RigidBody.hpp"

namespace PhysicsEngine {
	// These constraints are treated as having no mass or volume, and cannot collide with any objects
	class Constraint {
	public:
		Constraint();
		Constraint(RigidBody* _a, RigidBody* _b);

		virtual vec2 calculate_force() = 0;
		void apply_force();

		bool is_broken();

		RigidBody* a = nullptr;
		RigidBody* b = nullptr;

	protected:
		bool broken = false;
	};

	class Spring : public Constraint {
	public:
		Spring(RigidBody* _a, RigidBody* _b, float _natural_length = 1.0f, float _modulus_of_elasticity = 1.0f, float max_length_factor = 2.0f);
		vec2 calculate_force();

	protected:
		const float natural_length = 1.0f;
		const float modulus_of_elasticity = 1.0f;
		const float max_length = 2.0f;
	};

	class String : public Spring {
	public:
		String(RigidBody* _a, RigidBody* _b, float _natural_length = 1.0f, float _modulus_of_elasticity = 1.0f, float max_length_factor = 2.0f);
		vec2 calculate_force();
	};
}
#pragma once

#include "PhysicsEngineMath.hpp"
#include "RigidBody.hpp"

namespace PhysicsEngine {
	// These constraints are treated as having no mass or volume, and cannot collide with any objects
	class Constraint {
	public:
		Constraint();
		Constraint(RigidBody* _a, RigidBody* _b, vec2 _offset_a = vec2{ 0.0f, 0.0f }, vec2 _offset_b = vec2{ 0.0f, 0.0f });

		virtual vec2 calculate_force() = 0;
		void apply_force();

		bool is_broken();

		RigidBody* a = nullptr;
		RigidBody* b = nullptr;
		vec2 offset_a;
		vec2 offset_b;

	protected:
		bool broken = false;
	};

	class Spring : public Constraint {
	public:
		Spring(RigidBody* _a, RigidBody* _b, vec2 _offset_a = vec2{ 0.0f, 0.0f }, vec2 _offset_b = vec2{ 0.0f, 0.0f }, float _natural_length = 1.0f, float _modulus_of_elasticity = 1.0f, float _max_length_factor = 2.0f);
		vec2 calculate_force();

	protected:
		const float natural_length = 1.0f;
		const float modulus_of_elasticity = 1.0f;
		const float max_length = 2.0f;
	};

	class String : public Spring {
	public:
		String(RigidBody* _a, RigidBody* _b, vec2 _offset_a = vec2{ 0.0f, 0.0f }, vec2 _offset_b = vec2{ 0.0f, 0.0f }, float _natural_length = 1.0f, float _modulus_of_elasticity = 1.0f, float _max_length_factor = 2.0f);
		vec2 calculate_force();
	};
}
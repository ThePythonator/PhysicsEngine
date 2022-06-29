#pragma once

#include "PhysicsEngineMath.hpp"
#include "RigidBody.hpp"

namespace PhysicsEngine {
	extern const phyflt MINIMUM_NATURAL_LENGTH;

	// These constraints are treated as having no mass or volume, and cannot collide with any objects
	class Constraint {
	public:
		Constraint();
		Constraint(RigidBody* _a, RigidBody* _b, phyvec _offset_a = PHYVEC_NULL, phyvec _offset_b = PHYVEC_NULL);

		virtual phyvec calculate_force() = 0;
		void apply_force();

		bool is_broken();

		RigidBody* a = nullptr;
		RigidBody* b = nullptr;
		phyvec offset_a;
		phyvec offset_b;

		// IDs can be used to:
		//		-	look up sprites and corresponding offsets to render at
		//		-	group rigidbodies when searching
		std::vector<uint32_t> ids;

	protected:
		bool broken = false;
	};

	class Spring : public Constraint {
	public:
		Spring(RigidBody* _a, RigidBody* _b, phyvec _offset_a = PHYVEC_NULL, phyvec _offset_b = PHYVEC_NULL, phyflt _natural_length = 1.0, phyflt _modulus_of_elasticity = 1.0, phyflt max_extension = 1.0);
		phyvec calculate_force();

	protected:
		const phyflt natural_length = 1.0;
		const phyflt modulus_of_elasticity = 1.0;
		const phyflt max_length = 2.0;
	};

	class String : public Spring {
	public:
		String(RigidBody* _a, RigidBody* _b, phyvec _offset_a = PHYVEC_NULL, phyvec _offset_b = PHYVEC_NULL, phyflt _natural_length = 1.0, phyflt _modulus_of_elasticity = 1.0, phyflt max_extension = 1.0);
		phyvec calculate_force();
	};
}
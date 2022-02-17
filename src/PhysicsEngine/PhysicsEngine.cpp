#include "PhysicsEngine.hpp"

namespace PhysicsEngine {
	PhysicsManager::PhysicsManager() {

	}

	void PhysicsManager::set_constants(Constants _constants) {
		constants = _constants;
	}

	void PhysicsManager::update(float dt) {
		// Basic version
		step(dt);

		// Advanced version:
		// Keep track of time and run physics at a faster rate
		// i.e. independent of number of calls to update
	}

	uint16_t PhysicsManager::add_body(RigidBody body) {
		bodies.push_back(body);
		return bodies.size() - 1;
	}

	uint16_t PhysicsManager::add_constraint(Constraint* constraint) {
		constraints.push_back(constraint);
		return constraints.size() - 1;
	}

	std::vector<RigidBody>& PhysicsManager::get_bodies() {
		return bodies;
	}

	std::vector<Constraint*>& PhysicsManager::get_constraints() {
		return constraints;
	}


	void PhysicsManager::step(float dt) {
		// Old method
		// Apply interaction forces
		// update_forces();
		//
		// Update constraints (also updates forces)
		// update_constraints();
		//
		// Move objects
		// update_velocities(dt);
		// update_positions(dt);
		//
		// Collision detection - find contact points and move objects
		// handle_collisions();

		// IF DOING BOX2D METHOD:
		// 
		// handle_collisions();
		//		>	detect collisions
		//		>	pre-steps (init)
		//		>	iterate apply impulse
		// 
		//		>	do joints at similar time
		// 
		// update_forces();
		// update_velocities();
		// update_positions();


		// New method:
		// What this means is that collision detection and constraints for _last_ frame are calculated - hopefully it isn't noticeable

		// Collision detection - find contact points and move objects
		handle_collisions(dt);
		
		// Update constraints (also updates forces)
		update_constraints();

		// Apply interaction forces and move objects
		update_forces();
		update_velocities(dt);
		update_positions(dt);
	}


	void PhysicsManager::update_velocity(RigidBody& body, float dt) {
		// Update object's velocity
		body.velocity += body.force * body.inverse_mass * dt;
		body.angular_velocity += body.torque * body.inverse_moment_of_inertia * dt;

		// Reset forces
		body.force = vec2{ 0.0f, 0.0f };
		body.torque = 0.0f;
	}

	void PhysicsManager::update_position(RigidBody& body, float dt) {
		// Update object's position
		body.centre += body.velocity * dt;
		body.angle += body.angular_velocity * dt;
	}

	void PhysicsManager::add_impulse(RigidBody& body, const vec2& impulse, const vec2& vector_to_contact) {
		// Update object's velocity
		body.velocity += impulse * body.inverse_mass;
		body.angular_velocity += cross(vector_to_contact, impulse) * body.inverse_moment_of_inertia;
	}


	void PhysicsManager::update_forces() {
		// Could maybe allow user to specify own update function?
		for (RigidBody& body : bodies) {
			// TODO

			// Gravity etc

			// example, REMOVE LATER (actual gravity should be between objects)
#define ORBIT_GRAV_DEMO
#ifndef ORBIT_GRAV_DEMO
			// gravity to floor
			body.force += body.mass * vec2{ 0.0f, 9.81f } * 0.2f;// *100.0f;
#endif

			// Note: torque should also be affected by changes in force
		}
			
		// GRAVITY DEMO
#ifdef ORBIT_GRAV_DEMO
		static float G = 6.67430e-11f;

		// Calculate gravity between every unique pair of objects
		for (uint16_t i = 0; i < bodies.size(); i++) {
			for (uint16_t j = i + 1; j < bodies.size(); j++) {

				vec2 difference = bodies[j].centre - bodies[i].centre;

				float dist_squared = length_squared(difference);

				if (dist_squared != 0.0f) {
					float force_magnitude = G * bodies[i].mass * bodies[j].mass / dist_squared;

					float dist = std::sqrt(dist_squared);

					//printf("%f\n", force_magnitude);

					vec2 force = force_magnitude * difference / dist;

					bodies[i].apply_force(force);
					bodies[j].apply_force(-force);
				}
			}
		}
#endif // ORBIT_GRAV_DEMO
	}

	void PhysicsManager::update_constraints() {
		for (Constraint* constraint : constraints) {
			// Constraint applies the force to the objects it links
			constraint->apply_force();
		}
	}

	void PhysicsManager::update_velocities(float dt) {
		// Updates velocities from forces applied to object
		for (RigidBody& body : bodies) {
			update_velocity(body, dt);
		}
	}

	void PhysicsManager::update_positions(float dt) {
		// Updates positions from current velocities
		for (RigidBody& body : bodies) {
			update_position(body, dt);
		}
	}


	void PhysicsManager::handle_collisions(float dt) {
		std::vector<CollisionPacket> collision_packets;

		// Detect collisions between every unique pair of objects
		for (uint16_t i = 0; i < bodies.size(); i++) {
			for (uint16_t j = i + 1; j < bodies.size(); j++) {
				// Only check for collisions if they have a layer in common
				if (bodies[i].get_layers() & bodies[j].get_layers()) {
					// Check for collision between objects
					CollisionInformation collision_information = detect_collision(bodies[i], bodies[j]);

					if (collision_information.contact_count) {
						// Collision occurred
						collision_packets.push_back(CollisionPacket{ collision_information, i, j });
					}
				}
			}
		}

		for (const CollisionPacket& collision_packet : collision_packets) {
			// Resolve collision - calculates impulses for both bodies and applies them
			
			// If we want to add iterations, iterate below line here:
			for (uint8_t i = 0; i < constants.iterations; i++)
			resolve_collision(bodies[collision_packet.index_a], bodies[collision_packet.index_b], collision_packet.collision_information, dt);
		}
	}

	CollisionInformation PhysicsManager::detect_collision(RigidBody& a, RigidBody& b) {
		// Lookup from jump table based on object types
		return collision_detection_functions[static_cast<int>(a.shape->get_type())][static_cast<int>(b.shape->get_type())](a, b);
	}

	void PhysicsManager::resolve_collision(RigidBody& a, RigidBody& b, const CollisionInformation& collision_information, float dt) {
		// If both objects have infinite mass, we can't move them
		if (a.inverse_mass == 0.0f && b.inverse_mass == 0.0f) {
			return;
		}

		// Calculate basic weighted average between the restitutions of the materials
		// Linear interpolation (25% along)
		float restitution = a.material->restitution < b.material->restitution ? 3.0f * a.material->restitution + b.material->restitution : a.material->restitution + 3.0f * b.material->restitution;
		restitution /= 4.0f;


		// Calculate averages for friction coefficients
		float static_friction_coefficient = 0.5f * (a.material->static_friction + b.material->static_friction);
		float dynamic_friction_coefficient = 0.5f * (a.material->dynamic_friction + b.material->dynamic_friction);

		// For each contact point
		for (uint8_t i = 0; i < collision_information.contact_count; i++) {
			// Calculate distance from each centre of mass to contact point
			vec2 centre_a_to_contact = collision_information.contact_data[i].contact_point - a.centre;
			vec2 centre_b_to_contact = collision_information.contact_data[i].contact_point - b.centre;

			// Calculate relative velocity (from A to B)
			// Uses (sort of) pre-collision velocity (because velocity is updated, then position)
			vec2 relative_velocity = (b.velocity + cross(b.angular_velocity, centre_b_to_contact)) - (a.velocity + cross(a.angular_velocity, centre_a_to_contact));
			//vec2 relative_velocity = b.velocity - a.velocity;

			// Project relative velocity onto normal
			float relative_velocity_along_normal = dot(relative_velocity, collision_information.collision_normal);

			// If objects are travelling apart, early-out:
			if (relative_velocity_along_normal > 0.0f) {
				continue;
			}

			// Used in calculating magnitude
			float cross_a = cross(centre_a_to_contact, collision_information.collision_normal);
			float cross_b = cross(centre_b_to_contact, collision_information.collision_normal);

			float denominator = a.inverse_mass + b.inverse_mass + a.inverse_moment_of_inertia * cross_a * cross_a + b.inverse_moment_of_inertia * cross_b * cross_b;

			// Calculate magnitude of impulse along normal
			float impulse_magnitude = -(1.0f + restitution) * relative_velocity_along_normal;

			// Add some extra impulse, proportional to the penetration - keeps objects from sinking into each other
			// TODO: consider allowing some slop
			impulse_magnitude += constants.bias_factor * std::max(collision_information.contact_data[i].penetration_distance - constants.penetration_slop, 0.0f) / dt;

			// Scale impulse magnitude
			impulse_magnitude /= denominator;

			// If we have two contact points, we need to half the impulse applied
			impulse_magnitude /= static_cast<float>(collision_information.contact_count);

			// Get impulse
			vec2 impulse = impulse_magnitude * collision_information.collision_normal;

			// Apply impulses!
			add_impulse(a, -impulse, centre_a_to_contact);
			add_impulse(b, impulse, centre_b_to_contact);


			// Now friction...

			// Easy way to get tangent: subtract vector along normal from original vector
			vec2 tangent = relative_velocity - relative_velocity_along_normal * collision_information.collision_normal;
			
			// Normalise tangent, only if length > 0:
			float tangent_length_squared = length_squared(tangent);
			if (tangent_length_squared > 0.0f) {
				// Normalise tangent
				tangent = tangent / std::sqrt(tangent_length_squared);
			}

			// Note: unclear whether friction should be based off relative velocity before collision, or after resolving...
			// I'm going to use relative velocity from before
			//relative_velocity = (b.velocity + cross(b.angular_velocity, centre_b_to_contact)) - (a.velocity + cross(a.angular_velocity, centre_a_to_contact));

			float relative_velocity_along_tangent = dot(relative_velocity, tangent);

			//printf("n: %f,%f\n", collision_information.collision_normal.x, collision_information.collision_normal.y);
			//printf("rv: %f,%f\n", relative_velocity.x, relative_velocity.y);
			//printf("rv_n: %f\n", relative_velocity_along_normal);
			//printf("t: %f,%f\n", tangent.x, tangent.y);
			//printf("rv_t: %f\n", relative_velocity_along_tangent);

			float friction_magnitude = -relative_velocity_along_tangent / denominator;

			// If we have two contact points, we need to half the impulse applied??
			friction_magnitude /= static_cast<float>(collision_information.contact_count);

#if 1 // We don't like friction >:(
#if 1
			// IE's version... not sure if correct

			// Coulomb's law for friction:
			// If the friction force is more than static friction, set friction force to dynamic friction
			if (std::abs(friction_magnitude) < static_friction_coefficient * impulse_magnitude) {
				// friction_magnitude is correct, don't change it
			}
			else {
				// magnitude is wrong, change it
				friction_magnitude = -dynamic_friction_coefficient * impulse_magnitude;
			}

			//printf("imp mag: %f\n", impulse_magnitude);
			//printf("fri mag: %f\n", friction_magnitude);

			vec2 tangent_impulse = friction_magnitude * tangent;

			// Apply friction impulse
			add_impulse(a, -tangent_impulse, centre_a_to_contact);
			add_impulse(b, tangent_impulse, centre_b_to_contact);
#else
			// My version... also not sure if correct

			// need to change this bit:
			float MINIMUM_STATIC_FRICTION_VELOCITY = 0.1f; // constant?
			float dt = 1.0f / 120.0f; // get dt from main prog
			 
			float friction_magnitude_a = friction_magnitude;
			float friction_magnitude_b = friction_magnitude;
			
			if (std::abs(relative_velocity_along_tangent) < MINIMUM_STATIC_FRICTION_VELOCITY) {
				// Stationary, so static friction
				float static_friction_magnitude = static_friction_coefficient * impulse_magnitude;
				// If static friction is more than net force, only counter the net force
				// We don't have dt :(
				// This method requires update_forces being done before handle_collisions!!!
				friction_magnitude_a = std::min(static_friction_magnitude, dot(a.force, tangent) * dt);
				friction_magnitude_b = std::min(static_friction_magnitude, dot(b.force, tangent) * dt);
			}
			else {
				friction_magnitude_a = friction_magnitude_b = dynamic_friction_coefficient * impulse_magnitude;
			}

			vec2 tangent_impulse_a = friction_magnitude_a * tangent;
			vec2 tangent_impulse_b = friction_magnitude_b * tangent;

			// Apply friction impulse
			add_impulse(a, -tangent_impulse_a, centre_a_to_contact);
			add_impulse(b, tangent_impulse_b, centre_b_to_contact);
#endif
#endif
		}
	}
}
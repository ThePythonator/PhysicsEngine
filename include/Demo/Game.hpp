#pragma once

#include "PhysicsEngine.hpp"

#include "BaseGame.hpp"

#include "SDL2Extras.hpp"
#include "Constants.hpp"

class Game : public Framework::BaseGame {
public:
	Game();

private:
	void start();
	void end();

	bool main_loop();

	void update(float dt);
	void render();

	void render_polygon(PhysicsEngine::RigidBody& body);
	void render_circle(PhysicsEngine::RigidBody& body);
	void render_constraint(PhysicsEngine::Constraint* constraint);

	void load_data();
	void clear_data();

	PhysicsEngine::PhysicsData ptrs;
	PhysicsEngine::PhysicsManager manager;

	bool stepping = false;

	float last_time = 0.0f;

	// Scale is number of pixels per metre
	float scale = 1.0f; // 100 pixel per metre = 1px per cm
	float inv_scale = 1.0f / scale;
};
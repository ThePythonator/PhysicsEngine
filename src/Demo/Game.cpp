#include "Game.hpp"

Game::Game() : BaseGame() {

}

void Game::start() {
	//manager.set_constants(PhysicsEngine::PhysicsManager::Constants{ 1, 0.1f, 0.1f });

	// Used to change zoom and camera 'position'
	float inv_scale = 0.005f;
	PhysicsEngine::phyvec offset{ 2.0f, 3.0f };
	//PhysicsEngine::phyvec offset;

	// All sizes are in SI units (i.e. metres or kilograms etc)
	// Material values obtained from internet
	PhysicsEngine::Material* pSteel = new PhysicsEngine::Material(0.74f, 0.57f, 0.7f, 7850);
	PhysicsEngine::Material* pWood = new PhysicsEngine::Material(0.5f, 0.25f, 0.6f, 710);
	PhysicsEngine::Material* pPlastic = new PhysicsEngine::Material(0.35f, 0.3f, 0.5f, 940);
	PhysicsEngine::Material* pGlass = new PhysicsEngine::Material(0.94f, 0.4f, 0.5f, 2500);

	PhysicsEngine::Material* pTESTMAT = new PhysicsEngine::Material(0.4f, 0.3f, 0.2f, 1.0f);

	PhysicsEngine::Material* pDENSE = new PhysicsEngine::Material(0.4f, 0.3f, 0.2f, 1.0e10f);

	//PhysicsEngine::Shape* pCircle = new PhysicsEngine::Circle(inv_scale * 19500.0f);
	PhysicsEngine::Shape* pCircle = new PhysicsEngine::Circle(inv_scale * 50.0f);
	uint16_t c_index = manager.add_body(new PhysicsEngine::RigidBody(pCircle, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ 400.0f, 350.0f }, 0.0f, true));
	//uint16_t c_index = manager.add_body(PhysicsEngine::RigidBody(pCircle, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ 0.0f, 19900.0f }, 0.0f, true));

	//manager.add_body(PhysicsEngine::RigidBody(pCircle, pDENSE, offset + inv_scale * PhysicsEngine::phyvec{ 400.0f, 0.0f }, 0.0f));


	PhysicsEngine::Shape* pIsosceles = new PhysicsEngine::Polygon({ PhysicsEngine::phyvec{ 0.0f, -30.0f } * inv_scale, PhysicsEngine::phyvec{ 35.0f, 30.0f } * inv_scale, PhysicsEngine::phyvec{ -35.0f, 30.0f } * inv_scale });
	//manager.add_body(PhysicsEngine::RigidBody(pIsosceles, pTESTMAT, offset + inv_scale * PhysicsEngine::phyvec{ 0.0f, 0.0f }, 0.0f));

	PhysicsEngine::Shape* pPoly = PhysicsEngine::create_rect(inv_scale * PhysicsEngine::phyvec{ 500.0f, 30.0f });
	manager.add_body(new PhysicsEngine::RigidBody(pPoly, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ 400.0f, 550.0f }, 0.0f, true));

	manager.add_body(new PhysicsEngine::RigidBody(pPoly, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ 680.0f, 740.0f }, PhysicsEngine::deg_to_rad(90.0f), true));
	manager.add_body(new PhysicsEngine::RigidBody(pPoly, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ -75.0f, 470.0f }, PhysicsEngine::deg_to_rad(20.0f), true));

	
	manager.add_body(new PhysicsEngine::RigidBody(pPoly, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ 0.0f, 100.0f }, PhysicsEngine::deg_to_rad(45.0f), true));
	manager.add_body(new PhysicsEngine::RigidBody(pPoly, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ -350.0f, -250.0f }, PhysicsEngine::deg_to_rad(45.0f), true));
	uint16_t e_index = manager.add_body(new PhysicsEngine::RigidBody(pPoly, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ 800.0f, 0.0f }, PhysicsEngine::deg_to_rad(-45.0f), true));
	manager.add_body(new PhysicsEngine::RigidBody(pPoly, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ 1150.0f, -350.0f }, PhysicsEngine::deg_to_rad(-45.0f), true));


	manager.add_body(new PhysicsEngine::RigidBody(pPoly, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ 0.0f, -500.0f }, PhysicsEngine::deg_to_rad(10.0f), true));
	manager.add_body(new PhysicsEngine::RigidBody(pPoly, pSteel, offset + inv_scale * PhysicsEngine::phyvec{ 910.0f, 420.0f }, PhysicsEngine::deg_to_rad(-20.0f), true));

	PhysicsEngine::Shape* pTri = new PhysicsEngine::Polygon({ inv_scale * PhysicsEngine::phyvec{ -40.0f, -20.0f }, inv_scale * PhysicsEngine::phyvec{ -40.0f, 0.0f }, inv_scale * PhysicsEngine::phyvec{ 40.0f, 0.0f } });
	//uint16_t f_index = manager.add_body(PhysicsEngine::RigidBody(pTri, pPlastic, offset + inv_scale * PhysicsEngine::phyvec{ 650.0f, -50.0f }));

	//PhysicsEngine::Shape* pTest = PhysicsEngine::create_rect(inv_scale * PhysicsEngine::phyvec{ 100.0f, 100.0f });
	PhysicsEngine::Shape* pTest = new PhysicsEngine::Circle(inv_scale * 20.0f);
	PhysicsEngine::RigidBody* b = new PhysicsEngine::RigidBody(pTest, pPlastic, offset + inv_scale * PhysicsEngine::phyvec{ 410.0f, 50.0f });
	//PhysicsEngine::RigidBody b = PhysicsEngine::RigidBody(pTest, pPlastic, offset + inv_scale * PhysicsEngine::phyvec{ 410.0f, -150.0f });
	uint16_t a_index = manager.add_body(b);

	uint16_t b_index = manager.add_body(new PhysicsEngine::RigidBody(pTest, pPlastic, offset + inv_scale * PhysicsEngine::phyvec{ 350.0f, 50.0f }));

	uint16_t d_index = manager.add_body(new PhysicsEngine::RigidBody(pTest, pPlastic, offset + inv_scale * PhysicsEngine::phyvec{ 350.0f, 200.0f }));


	PhysicsEngine::Shape* pSmallCircle = new PhysicsEngine::Circle(inv_scale * 5.0f);
	PhysicsEngine::Shape* pSmallBox = PhysicsEngine::create_rect(inv_scale * PhysicsEngine::phyvec{ 20.0f, 20.0f });
	PhysicsEngine::Shape* pBigBox = PhysicsEngine::create_rect(inv_scale * PhysicsEngine::phyvec{ 50.0f, 50.0f });

	stepping = true;

	for (uint16_t i = 0; i < 500; i++) {
		manager.add_body(new PhysicsEngine::RigidBody(pSmallCircle, pPlastic, offset + inv_scale * PhysicsEngine::phyvec{ 200.0f + rand() % 400, -100.0f + rand() % 300 }));
	}

	for (uint16_t i = 0; i < 50; i++) {
		manager.add_body(new PhysicsEngine::RigidBody(pSmallCircle, pPlastic, offset + inv_scale * PhysicsEngine::phyvec{ -200.0f + rand() % 300, -1500.0f + rand() % 500 }));
	}

	for (uint16_t i = 0; i < 40; i++) {
		manager.add_body(new PhysicsEngine::RigidBody(pSmallBox, pPlastic, offset + inv_scale * PhysicsEngine::phyvec{ -300.0f + rand() % 200, -500.0f + rand() % 200 }));
		//manager.get_bodies()[manager.get_bodies().size() - 1].set_layers(0); // Uncomment to set all small boxes' layers to 0 (i.e. they can't collide on any layer)
	}

	for (uint16_t i = 0; i < 5; i++) {
		manager.add_body(new PhysicsEngine::RigidBody(pBigBox, pPlastic, offset + inv_scale * PhysicsEngine::phyvec{ 900.0f + rand() % 200, -600.0f + rand() % 200 }));
	}

	std::vector<PhysicsEngine::RigidBody *>& rb_vec = manager.get_bodies();

	manager.add_constraint(new PhysicsEngine::Spring(rb_vec[a_index], rb_vec[b_index], inv_scale * PhysicsEngine::phyvec{ 0.0f, 0.0f }, inv_scale * PhysicsEngine::phyvec{ 0.0f, 0.0f }, inv_scale * 10.0f, 20.0f, 10.0f));

	manager.add_constraint(new PhysicsEngine::Spring(rb_vec[c_index], rb_vec[d_index], inv_scale * PhysicsEngine::phyvec{ 0.0f, 0.0f }, inv_scale * PhysicsEngine::phyvec{ 0.0f, 0.0f }, inv_scale * 20.0f, 100.0f, 10.0f));

	uint16_t f_index = manager.add_body(new PhysicsEngine::RigidBody(pBigBox, pPlastic, offset + inv_scale * PhysicsEngine::phyvec{ 650.0f, 300.0f }));
	//manager.add_constraint(new PhysicsEngine::Spring(&rb_vec[e_index], &rb_vec[f_index], inv_scale * PhysicsEngine::phyvec{ -220.0f, 0.0f }, inv_scale * PhysicsEngine::phyvec{ 25.0f, 25.0f }, inv_scale * 10.0f, 20.0f, 100.0f));
	manager.add_constraint(new PhysicsEngine::String(rb_vec[e_index], rb_vec[f_index], inv_scale * PhysicsEngine::phyvec{ -220.0f, 0.0f }, inv_scale * PhysicsEngine::phyvec{ 25.0f, -25.0f }, inv_scale * 50.0f, 50.0f, 100.0f));
	
}

void Game::end() {

}

// Limit framerate and get approximate dt
// Limiting is inaccurate: alright for 60fps (+/- ~2fps), 120fps gets inaccurate (+/- ~10fps) and higher framerates are worse.
// This is due to SDL working in milliseconds, so we can't do any better than 1000/17 = 111 fps or 1000/8 = 125 fps
bool Game::main_loop() {
	// Get start time
	float start_time = SDL_GetTicks();

	// 'Calculate' dt
	//float dt = WINDOW::TARGET_DT;

	// Or we could get last frame's dt - this is better
	float dt = (start_time - last_time) / 1000.0f;
	last_time = start_time;

	// Cap dt - stops game skipping time when window is dragged (which caused objects to phase through other objects)
	// Essentially makes game run slower if dt < MAX_DT
	// While window is dragged, dt accumulates because main_loop isn't called, so dt because very large
	dt = std::min(dt, WINDOW::MAX_DT);

	// Update input handler (updates all key states etc)
	input_handler.update();

	// Handle events
	SDL_Event sdl_event;
	while (SDL_PollEvent(&sdl_event) != 0) {
		if (sdl_event.type == SDL_QUIT) {
			// X (close) is pressed
			return false;
		}
		else {
			// Delegate to InputHandler
			input_handler.handle_sdl_event(sdl_event);
		}
	}

	update(dt);

	// Clear the screen
	Framework::SDL2Extras::SDL_SetRenderDrawColor(renderer, COLOURS::BLACK);
	SDL_RenderClear(renderer);

	// Render game
	render();

	// Update screen
	SDL_RenderPresent(renderer);

	// If we were too quick, sleep!
	float end_time = SDL_GetTicks();
	float difference = end_time - start_time;
	int ticks_to_sleep = static_cast<int>(WINDOW::TARGET_DT * 1000.0f) - difference;
	if (ticks_to_sleep > 0) {
		SDL_Delay(ticks_to_sleep);
	}

	//printf("FPS: %f\n", 1.0f / dt);

	return true;
}

void Game::update(float dt) {
	if (!stepping) {
		manager.update(dt);
	}

	if (Framework::KeyHandler::just_down(input_handler.get_key_union().keys.SPACE)) {
		// Step
		manager.step(1.0f / 60.0f);
	}
	else if (Framework::KeyHandler::just_down(input_handler.get_key_union().keys.S)) {
		// Set/unset stepping mode
		stepping = !stepping;
		// Update last_time so we don't get 'fast-forward' up to true time
		last_time = SDL_GetTicks();
	}
}

void Game::render() {
	for (PhysicsEngine::RigidBody* body : manager.get_bodies()) {
		Framework::SDL2Extras::SDL_SetRenderDrawColor(renderer, COLOURS::WHITE);

		switch (body->shape->get_type()) {
		case PhysicsEngine::Shape::ShapeType::CIRCLE:
			render_circle(body);
			break;
		case PhysicsEngine::Shape::ShapeType::POLYGON:
			render_polygon(body);
			break;
		default:
			break;
		}

		PhysicsEngine::phyvec scaled_centre = scale * body->centre;

		Framework::SDL2Extras::SDL_SetRenderDrawColor(renderer, COLOURS::RED);
		SDL_RenderDrawPoint(renderer, scaled_centre.x, scaled_centre.y);
	}

	for (PhysicsEngine::Constraint* constraint : manager.get_constraints()) {
		render_constraint(constraint);
	}

	Framework::SDL2Extras::SDL_SetRenderDrawColor(renderer, COLOURS::WHITE);
}

void Game::render_polygon(PhysicsEngine::RigidBody* body) {
	PhysicsEngine::Polygon* polygon = static_cast<PhysicsEngine::Polygon*>(body->shape);

	for (uint16_t i = 0; i < polygon->vertices.size(); i++) {
		uint16_t next_i = i + 1 < polygon->vertices.size() ? i + 1 : 0;

		PhysicsEngine::phyvec v1 = scale * PhysicsEngine::to_world_space(polygon->vertices[i], body->centre, body->get_rotation_matrix());
		PhysicsEngine::phyvec v2 = scale * PhysicsEngine::to_world_space(polygon->vertices[next_i], body->centre, body->get_rotation_matrix());

		SDL_RenderDrawLine(renderer, v1.x, v1.y, v2.x, v2.y);
	}
}

void Game::render_circle(PhysicsEngine::RigidBody* body) {
	PhysicsEngine::Circle* circle = static_cast<PhysicsEngine::Circle*>(body->shape);

	int scaled_radius = static_cast<int>(circle->radius * scale);
	PhysicsEngine::phyvec scaled_centre = body->centre * scale;

	Framework::SDL2Extras::SDL_RenderDrawCircle(renderer, scaled_centre.x, scaled_centre.y, scaled_radius);

	int x = static_cast<int>(scaled_centre.x + scaled_radius * std::cos(body->angle));
	int y = static_cast<int>(scaled_centre.y + scaled_radius * std::sin(body->angle));

	SDL_RenderDrawLine(renderer, scaled_centre.x, scaled_centre.y, x, y);
}

void Game::render_constraint(PhysicsEngine::Constraint* constraint) {
	Framework::SDL2Extras::SDL_SetRenderDrawColor(renderer, constraint->is_broken() ? COLOURS::RED : COLOURS::YELLOW);

	PhysicsEngine::phyvec scaled_centre_a = scale * (constraint->a->centre + PhysicsEngine::to_world_space(constraint->offset_a, constraint->a->get_rotation_matrix()));
	PhysicsEngine::phyvec scaled_centre_b = scale * (constraint->b->centre + PhysicsEngine::to_world_space(constraint->offset_b, constraint->b->get_rotation_matrix()));

	SDL_RenderDrawLine(renderer, scaled_centre_a.x, scaled_centre_a.y, scaled_centre_b.x, scaled_centre_b.y);
}

void Game::load_data() {

}

void Game::clear_data() {

}
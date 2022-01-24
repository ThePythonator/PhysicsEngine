# PhysicsEngine
A basic physics engine supporting circles and orientable non-regular convex polygons.
Oh, and it has undamped springs and strings.

# Interface
Everything is in the `PhysicsEngine` namespace.

`PhysicsManager` is the class which handles everything else. Just feed it the `RigidBody` instances you want it to manage, and it'll do the rest for you.
 
### `PhysicsManager`:

- `update(float dt)`
:  Updates the physics (currently does the same thing as `step`)

- `step(float dt)`
:  Updates all `RigidBody` and `Constraint` objects tracked by the manager

- `add_body(RigidBody body)`
:  Adds the `RigidBody` instance to the list of tracked objects. These will get updated every time `step` is called.

- `add_constraint(Constraint* constraint)`
:  Adds the `Constraint` pointer to the list of tracked objects. These will get updated every time `step` is called.

- `get_bodies()`
:  Returns a reference to the `vector` of `RigidBody` instances. Most useful for rendering the objects. This should really be read-only. Modify at your own risk.

- `get_constraints()`
:  Returns a reference to the `vector` of `Constraint` pointers. Most useful for rendering the objects. This should really be read-only. Modify at your own risk.

- TODO set_constants

### `RigidBody`:

- TODO
- It needs a `Shape` pointer

### `Circle`:

- TODO
- It inherits from `Shape`

### `Polygon`:

- TODO
- It inherits from `Shape`

### `Constraint`:

- TODO
- Needs pointers to two `RigidBody` instances.

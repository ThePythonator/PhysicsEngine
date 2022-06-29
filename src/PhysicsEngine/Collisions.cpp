#include "Collisions.hpp"

namespace PhysicsEngine {
	CollisionDetectionFunction collision_detection_functions[static_cast<int>(Shape::ShapeType::SHAPE_COUNT)][static_cast<int>(Shape::ShapeType::SHAPE_COUNT)] = {
	   {
		   circle_to_circle,
		   circle_to_polygon
	   },
	   {
		   polygon_to_circle,
		   polygon_to_polygon
	   }
	};

	// Note: Contact points are used for calculating the distance from the centre of mass to the contact point.
	// I believe this is necessary for rotational components. It is unclear how the contact point should be calculated.
	// Sometimes the contact point is on the boundary of one object and not the other, and it is not clear which object it should be on the boundary of.
	// It may be the case that these objects are close enough that it makes little difference.
	// 
	// For example:
	//	circle to circle	:	if circles are identical in position and size, contact is at centre of circle
	//							else contact is at edge of circle A
	//
	//	circle to polygon	:	if circles are in vertex regions, contact is at the vertex of the polygon
	//							else contact is at the edge of the circle
	//
	//	polygon to polygon	:	unknown (todo)

	CollisionInformation circle_to_circle(RigidBody* a, RigidBody* b) {
		// This will be used to store the collision data
		CollisionInformation collision_information;

		// We need to access the radius, so we need to cast to Circle
		Circle* circle_a = static_cast<Circle*>(a->shape);
		Circle* circle_b = static_cast<Circle*>(b->shape);

		// Calculate initial data needed
		phyvec normal = b->centre - a->centre;
		phyflt squared_distance = length_squared(normal);
		phyflt radii_sum = circle_a->radius + circle_b->radius;
		phyflt squared_radii_sum = radii_sum * radii_sum;

		// Are objects intersecting?
		if (squared_distance < squared_radii_sum) {
			// Collision has occurred

			// Caluclate distance between centres
			phyflt distance = std::sqrt(squared_distance);

			// Calculate penetration
			phyflt penetration_depth = radii_sum - distance;

			// The following defaults are used if distance is 0

			// Choose a default value for the normal
			// (it doesn't matter which direction)
			phyvec collision_normal = PHYVEC_NULL;

			// As default, set contact point to centre of circle
			phyvec contact_point = a->centre;

			// If distance is 0, we can't divide by it
			if (distance > 0.0) {
				// Normalise collision normal
				collision_normal = normal / distance;

				// Adjust contact point to be on edge of circle,
				// nearest the other circle
				contact_point += collision_normal * circle_a->radius;
			}

			// Populate collision infomation
			collision_information.contact_count = 1;
			collision_information.contact_data[0] = ContactData{ contact_point, penetration_depth };
			collision_information.collision_normal = collision_normal;
		}
		else {
			// No collision has occurred
			collision_information.contact_count = 0;
		}

		return collision_information;
	}

	CollisionInformation circle_to_polygon(RigidBody* a, RigidBody* b) {
		// This will be used to store the collision data
		CollisionInformation collision_information;

		// We need to access the radius and face normals, so we need to cast to Circle and Polygon respectively
		Circle* circle_a = static_cast<Circle*>(a->shape);
		Polygon* polygon_b = static_cast<Polygon*>(b->shape);

		// Convert circle's centre to the model space of the polygon
		phyvec circle_a_converted_centre = to_model_space(a->centre, b->centre, b->get_rotation_matrix());
		
		// Find minimum penetration
		phyflt separation = -PHYFLT_MAX;
		uint16_t closest_face = 0;
		for (uint16_t i = 0; i < polygon_b->vertices.size(); i++) {
			// Find which side of each polygon face the circle is on
			// signed_distance is distance from line to centre of circle
			phyflt signed_distance = dot(polygon_b->face_normals[i], circle_a_converted_centre - polygon_b->vertices[i]);

			if (signed_distance > circle_a->radius) {
				// Impossible for a collision to have occurred with ANY face, so early-out
				collision_information.contact_count = 0;
				return collision_information;
			}
			else if (signed_distance > separation) {
				// Distance is greater than current minimum separation, so update to better value
				separation = signed_distance;
				closest_face = i;
			}
		}

		uint16_t next_face = closest_face + 1 < polygon_b->vertices.size() ? closest_face + 1 : 0;

		// Vector parallel to face
		phyvec face_vector = polygon_b->vertices[next_face] - polygon_b->vertices[closest_face];

		// Vectors from vertices (on either side of closest face) to the centre of circle_a
		phyvec vertex_to_circle_centre = circle_a_converted_centre - polygon_b->vertices[closest_face];
		phyvec next_vertex_to_circle_centre = circle_a_converted_centre - polygon_b->vertices[next_face];

		// Check if centre is in face region by confirming both angles between face and vector from vertex to circle centre are acute
		// Dot product is positive if theta < 90
		// Format:
		// dot(vertex_to_circle_centre, vertex_to_other_vertex)

		if (dot(vertex_to_circle_centre, face_vector) < 0.0) {
			// In vertex region, closest to vertex vertices[closest_face]

			// Since it is possible to reach here without intersecting the polygon, we need to check:
			phyflt distance_to_vertex = length_squared(vertex_to_circle_centre);

			if (distance_to_vertex > circle_a->radius * circle_a->radius) {
				// No collision
				collision_information.contact_count = 0;
			}
			else {
				// Collision, set collision_information data
				collision_information.contact_count = 1;

				// Penetration is distance between radius and vertex
				phyflt penetration_distance = circle_a->radius - std::sqrt(distance_to_vertex);

				// Contact point is vertex (not sure why though)
				collision_information.contact_data[0] = ContactData{ to_world_space(polygon_b->vertices[closest_face], b->centre, b->get_rotation_matrix()), penetration_distance };

				// Normal is from circle to vertex, but we need to rotate it to world space (see face region comments for more details)
				// Because we're using vector from vertex to circle, we need to flip the normal at the end
				collision_information.collision_normal = -normalise(to_world_space(vertex_to_circle_centre, b->get_rotation_matrix()));
			}
		}
		else if (dot(next_vertex_to_circle_centre, -face_vector) < 0.0) {
			// In vertex region, closest to vertex vertices[next_face]

			// Since it is possible to reach here without intersecting the polygon, we need to check:
			phyflt distance_to_vertex = length_squared(next_vertex_to_circle_centre);

			if (length_squared(next_vertex_to_circle_centre) > circle_a->radius * circle_a->radius) {
				// No collision
				collision_information.contact_count = 0;
			}
			else {
				// Collision, set collision_information data
				collision_information.contact_count = 1;

				// Penetration is distance between radius and vertex
				phyflt penetration_distance = circle_a->radius - std::sqrt(distance_to_vertex);

				// Contact point is vertex (not sure why though)
				collision_information.contact_data[0] = ContactData{ to_world_space(polygon_b->vertices[next_face], b->centre, b->get_rotation_matrix()), penetration_distance };

				// Normal is from circle to vertex, but we need to rotate it to world space (see face region comments for more details)
				// Because we're using vector from vertex to circle, we need to flip the normal at the end
				collision_information.collision_normal = -normalise(to_world_space(next_vertex_to_circle_centre, b->get_rotation_matrix()));
			}
		}
		else {
			// In face region (closest to face)

			// Set collision_information data
			collision_information.contact_count = 1;

			// Face normal is opposite direction to collision_normal, so convert face normal to world space and flip direction
			// Since we just want to rotate the normal, not translate it, we just need to multiply it by the rotation matrix
			collision_information.collision_normal = -to_world_space(polygon_b->face_normals[closest_face], b->get_rotation_matrix());

			// Penetration is just radius - perpendicular distance to face
			phyflt penetration_distance = circle_a->radius - separation;

			// Contact point is on edge of circle
			collision_information.contact_data[0] = ContactData{ a->centre + collision_information.collision_normal * circle_a->radius, penetration_distance };
		}

		return collision_information;
	}

	CollisionInformation polygon_to_circle(RigidBody* a, RigidBody* b) {
		// Just swap parameters around and use existing method
		CollisionInformation collision_information = circle_to_polygon(b, a);

		// Flip normal since the collision detection works out the normal from a to b, but a and b are flipped
		collision_information.collision_normal = -collision_information.collision_normal;

		return collision_information;
	}

	CollisionInformation polygon_to_polygon(RigidBody* a, RigidBody* b) {
		// This will be used to store the collision data
		CollisionInformation collision_information;

		// We need to access the radius and face normals, so we need to cast to Circle and Polygon respectively
		Polygon* polygon_a = static_cast<Polygon*>(a->shape);
		Polygon* polygon_b = static_cast<Polygon*>(b->shape);

		// Find axis of least penetration from A to B, and then B to A

		// A separating axis exists where the signed perpendicular distance from a support point to a face (along the negative normal) is negative
		// If the distance is negative, all vertices are on the outer side of the edge, so there cannot be a collision (if this is the case, early out)

		// Iterate through A's face normals and check against B's vertices
		phyflt maximum_distance_a = -PHYFLT_MAX;
		uint16_t face_index_a = 0;

		for (uint16_t i = 0; i < polygon_a->face_normals.size(); i++) {
			// Need to rotate face normal into B's model space - note that only requires rotation since we don't want to translate the vector
			// First rotate into world space
			phyvec normal = to_world_space(polygon_a->face_normals[i], a->get_rotation_matrix());
			// Now rotate into B's model space
			normal = to_model_space(normal, b->get_rotation_matrix());

			// Calculate B's furthest vertex along negative normal
			phyvec support_point = find_support_point(-normal, polygon_b->vertices);

			// Get perpendicular distance from support point to face, using dot product
			// First we need to convert vertex of A to world space:
			phyvec vertex = to_world_space(polygon_a->vertices[i], a->centre, a->get_rotation_matrix());
			// Now convert to B's model space:
			vertex = to_model_space(vertex, b->centre, b->get_rotation_matrix());

			// Project vector from vertex to support point along normal to get distance:
			phyflt distance = dot(normal, support_point - vertex);

			// Since we've projected along the POSITIVE normal, the distance will be positive if a separating axis has been found
			// Hence if distance >= 0.0 then we can early out
			if (distance >= 0.0) {
				collision_information.contact_count = 0;
				return collision_information;
			}
			else if (distance > maximum_distance_a) {
				maximum_distance_a = distance;
				face_index_a = i;
			}
		}

		// Now B to A
		phyflt maximum_distance_b = -PHYFLT_MAX;
		uint16_t face_index_b = 0;

		for (uint16_t i = 0; i < polygon_b->face_normals.size(); i++) {
			// Need to rotate face normal into A's model space - note that only requires rotation since we don't want to translate the vector
			// First rotate into world space
			phyvec normal = to_world_space(polygon_b->face_normals[i], b->get_rotation_matrix());
			// Now rotate into A's model space
			normal = to_model_space(normal, a->get_rotation_matrix());
			
			// Calculate A's furthest vertex along negative normal
			phyvec support_point = find_support_point(-normal, polygon_a->vertices);

			// Get perpendicular distance from support point to face, using dot product
			// First we need to convert vertex of B to world space:
			phyvec vertex = to_world_space(polygon_b->vertices[i], b->centre, b->get_rotation_matrix());
			// Now convert to A's model space:
			vertex = to_model_space(vertex, a->centre, a->get_rotation_matrix());

			// Project vector from vertex to support point along normal to get distance:
			phyflt distance = dot(normal, support_point - vertex);

			// Since we've projected along the POSITIVE normal, the distance will be positive if a separating axis has been found
			// Hence if distance >= 0.0 then we can early out
			if (distance >= 0.0) {
				collision_information.contact_count = 0;
				return collision_information;
			}
			else if (distance > maximum_distance_b) {
				maximum_distance_b = distance;
				face_index_b = i;
			}
		}

		// maximum_distance is negative, we want the one closer to 0
		// Polygon with least penetration is the reference polygon
		phyflt maximum_distance = maximum_distance_a;
		phyflt reference_face_index = face_index_a;
		Polygon* reference_polygon = polygon_a;
		Polygon* incident_polygon = polygon_b;

		RigidBody* reference_body = a;
		RigidBody* incident_body = b;

		// Used to invert normal if we swapped A and B
		bool flipped = false;

		// TODO: do I need to weight this?

		if (maximum_distance_b > maximum_distance) {
			maximum_distance = maximum_distance_b;
			reference_face_index = face_index_b;
			reference_polygon = polygon_b;
			incident_polygon = polygon_a;

			reference_body = b;
			incident_body = a;

			flipped = true;
		}

		// We now know the normal(s) and separation... we still need contact points though

		// Find incident face index
		// Incident face is the face which has the largest component in the direction of the negative reference face's normal

		// Need to convert to world space
		phyvec reference_normal = to_world_space(reference_polygon->face_normals[reference_face_index], reference_body->get_rotation_matrix());
		// Now convert to incident's model space
		reference_normal = to_model_space(reference_normal, incident_body->get_rotation_matrix());

		// Find most 'anti-normal' face normal on incident polygon (i.e.
		// i.e. minimise (get most negative): dot(reference_normal, incident_normal)

		// Note: this could be done more efficiently by using the fact that the vertex used to get the minimum penetration must be one of the vertices of the incident edge
		// Since we don't store that vertex, it's easier to do it this way:
		uint16_t incident_face_index = 0;
		phyflt minimum_dot_product = PHYFLT_MAX;

		for (uint16_t i = 0; i < incident_polygon->face_normals.size(); i++) {
			phyflt d = dot(reference_normal, incident_polygon->face_normals[i]);

			// Dot product is less, this is closer to 'anti-normal'
			if (d < minimum_dot_product) {
				minimum_dot_product = d;
				incident_face_index = i;
			}
		}

		// Get incident face vertices using incident_face_index
		phyvec incident_face_vertices[2];
		incident_face_vertices[0] = incident_polygon->vertices[incident_face_index];
		incident_face_vertices[1] = incident_polygon->vertices[incident_face_index + 1 < incident_polygon->vertices.size() ? incident_face_index + 1 : 0];
		
		// Convert to world space
		incident_face_vertices[0] = to_world_space(incident_face_vertices[0], incident_body->centre, incident_body->get_rotation_matrix());
		incident_face_vertices[1] = to_world_space(incident_face_vertices[1], incident_body->centre, incident_body->get_rotation_matrix());

		// Get reference face vertices using reference_face_index
		phyvec reference_face_vertices[2];
		reference_face_vertices[0] = reference_polygon->vertices[reference_face_index];
		reference_face_vertices[1] = reference_polygon->vertices[reference_face_index + 1 < reference_polygon->vertices.size() ? reference_face_index + 1 : 0];

		// Convert to world space
		reference_face_vertices[0] = to_world_space(reference_face_vertices[0], reference_body->centre, reference_body->get_rotation_matrix());
		reference_face_vertices[1] = to_world_space(reference_face_vertices[1], reference_body->centre, reference_body->get_rotation_matrix());

		// Clip to find contact points
		// Clip lines are the two lines parallel to the normal, each passing through one of the vertices of the reference face
		
		// Find vector parallel to line normals
		phyvec reference_edge = normalise(reference_face_vertices[1] - reference_face_vertices[0]);

		// Find first clip line origin distance (from origin to second vertex)
		phyflt first_origin_distance = dot(reference_face_vertices[1], reference_edge);

		// Clip incident face to line
		ClipResult half_clipped_face = clip_edge_to_line(incident_face_vertices, reference_edge, first_origin_distance);

		// TODO: TEST IF WORKS WITH ONLY ONE CONTACT POINT

		// We have an issue: it's possible to get less than two clipped points - and other engines just give up here
		// But surely it's possible to resolve using just one point (if 0 points then obviously impossible)
		if (half_clipped_face.points_count < 2) {
			collision_information.contact_count = 0;
			return collision_information;
		}

		// Find second clip line origin distance (from origin to first vertex)
		phyflt second_origin_distance = dot(reference_face_vertices[0], -reference_edge);

		// Clip incident face to line
		ClipResult clipped_face = clip_edge_to_line(half_clipped_face.points, -reference_edge, second_origin_distance);

		// TODO: TEST IF WORKS WITH ONLY ONE CONTACT POINT
		if (clipped_face.points_count < 2) {
			collision_information.contact_count = 0;
			return collision_information;
		}

		// Get collision normal
		phyvec collision_normal = to_world_space(reference_polygon->face_normals[reference_face_index], reference_body->get_rotation_matrix());
		// Flip if necessary
		collision_information.collision_normal = flipped ? -collision_normal : collision_normal;

		// Get distance from reference face to origin along normal
		phyflt reference_face_distance = dot(collision_normal, reference_face_vertices[0]);
		
		// Only keep points behind reference face
		collision_information.contact_count = 0;

		// Each vertex
		for (uint8_t i = 0; i < clipped_face.points_count; i++) {
			phyflt separation = dot(collision_normal, clipped_face.points[i]) - reference_face_distance;

			if (separation <= 0.0) {
				// Point is behind reference face, keep it
				collision_information.contact_data[collision_information.contact_count++] = ContactData{ clipped_face.points[i], -separation };
			}
		}

		return collision_information;
	}
}
/**************************************************************************/
/*  articulated_body_2d.h                                                 */
/**************************************************************************/
/*                         This file is part of:                          */
/*                             GODOT ENGINE                               */
/*                        https://godotengine.org                         */
/**************************************************************************/
/* Copyright (c) 2014-present Godot Engine contributors (see AUTHORS.md). */
/* Copyright (c) 2007-2014 Juan Linietsky, Ariel Manzur.                  */
/*                                                                        */
/* Permission is hereby granted, free of charge, to any person obtaining  */
/* a copy of this software and associated documentation files (the        */
/* "Software"), to deal in the Software without restriction, including    */
/* without limitation the rights to use, copy, modify, merge, publish,    */
/* distribute, sublicense, and/or sell copies of the Software, and to     */
/* permit persons to whom the Software is furnished to do so, subject to  */
/* the following conditions:                                              */
/*                                                                        */
/* The above copyright notice and this permission notice shall be         */
/* included in all copies or substantial portions of the Software.        */
/*                                                                        */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,        */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF     */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. */
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY   */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,   */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE      */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                 */
/**************************************************************************/

#pragma once

#pragma once

#include "core/math/vector2.h"
#include "core/object/ref_counted.h"
#include "core/templates/local_vector.h"
#include "core/variant/variant.h"

// A planar articulated body: rigid parts joined by pin joints, simulated with substepped extended position based
// dynamics. A joint can be limited to a range of relative angles and driven toward a target relative angle with a
// compliance, which is what lets the body hold a pose rigidly, follow one loosely or go limp, per joint. Contacts
// are half-planes the caller found for points on the parts, held with friction and without bounce. The caller owns
// everything around the body: it feeds targets, impulses and contacts each step and reads the part transforms back.
// Nothing here touches the scene tree or the physics server, so a step can run anywhere and substep as often as
// its stiffness needs.
class ArticulatedBody2D : public RefCounted {
	GDCLASS(ArticulatedBody2D, RefCounted);

	struct Part {
		Vector2 position;
		Vector2 previous_position; // Where the substep started, for the velocity update at its end.
		real_t rotation = 0.0;
		real_t previous_rotation = 0.0;
		Vector2 linear_velocity;
		real_t angular_velocity = 0.0;
		real_t inverse_mass = 0.0; // Zero for a part that never moves.
		real_t inverse_inertia = 0.0;
	};

	struct Joint {
		int part_a = -1;
		int part_b = -1;
		Vector2 anchor_a; // The pinned point, in each part's local space.
		Vector2 anchor_b;
		bool limit_enabled = false;
		real_t limit_lower = 0.0; // Range of part b's rotation relative to part a, radians.
		real_t limit_upper = 0.0;
		real_t drive_target = 0.0; // Relative angle the drive pulls toward.
		real_t drive_compliance = -1.0; // Negative while the drive is off, zero for a rigid drive.
	};

	struct Contact {
		int part = -1;
		Vector2 local_point; // The point on the part that must stay on the plane's near side, in the part's local space.
		Vector2 plane_point;
		Vector2 plane_normal; // Unit, pointing toward the side the point belongs on.
		real_t friction = 0.0;
		real_t lambda_normal = 0.0; // How hard the plane pushed this substep, as a positional impulse; zero when apart.
	};

	LocalVector<Part> parts;
	LocalVector<Joint> joints;
	LocalVector<Contact> contacts;
	Vector2 gravity;
	real_t linear_damping = 0.0;
	real_t angular_damping = 0.0;

protected:
	static void _bind_methods();

public:
	void clear();

	int add_part(const Vector2 &p_position, real_t p_rotation, real_t p_mass, real_t p_inertia);
	int get_part_count() const;
	void set_part_transform(int p_part, const Vector2 &p_position, real_t p_rotation);
	void set_part_velocity(int p_part, const Vector2 &p_linear_velocity, real_t p_angular_velocity);
	Vector2 get_part_position(int p_part) const;
	real_t get_part_rotation(int p_part) const;
	Vector2 get_part_linear_velocity(int p_part) const;
	real_t get_part_angular_velocity(int p_part) const;
	PackedVector2Array get_part_positions() const;
	PackedFloat32Array get_part_rotations() const;
	void apply_impulse(int p_part, const Vector2 &p_impulse, const Vector2 &p_point);

	int add_joint(int p_part_a, int p_part_b, const Vector2 &p_anchor_a, const Vector2 &p_anchor_b);
	int get_joint_count() const;
	void set_joint_limits(int p_joint, bool p_enabled, real_t p_lower, real_t p_upper);
	void set_joint_drive(int p_joint, real_t p_target_angle, real_t p_compliance);

	void clear_contacts();
	int add_contact(int p_part, const Vector2 &p_local_point, const Vector2 &p_plane_point, const Vector2 &p_plane_normal, real_t p_friction);
	int get_contact_count() const;

	void set_gravity(const Vector2 &p_gravity);
	Vector2 get_gravity() const;
	void set_damping(real_t p_linear, real_t p_angular);

	void step(real_t p_delta, int p_substeps);
};

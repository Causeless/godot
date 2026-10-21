/**************************************************************************/
/*  articulated_body_2d.cpp                                               */
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

#include "articulated_body_2d.h"

#include "core/math/math_funcs.h"
#include "core/object/class_db.h"

void ArticulatedBody2D::clear() {
	parts.clear();
	joints.clear();
	contacts.clear();
}

int ArticulatedBody2D::add_part(const Vector2 &p_position, real_t p_rotation, real_t p_mass, real_t p_inertia) {
	Part part;
	part.position = p_position;
	part.previous_position = p_position;
	part.rotation = p_rotation;
	part.previous_rotation = p_rotation;
	part.inverse_mass = p_mass > 0.0 ? 1.0 / p_mass : 0.0;
	part.inverse_inertia = p_inertia > 0.0 ? 1.0 / p_inertia : 0.0;
	parts.push_back(part);
	return parts.size() - 1;
}

int ArticulatedBody2D::get_part_count() const {
	return parts.size();
}

void ArticulatedBody2D::set_part_transform(int p_part, const Vector2 &p_position, real_t p_rotation) {
	ERR_FAIL_UNSIGNED_INDEX((uint32_t)p_part, parts.size());
	Part &part = parts[p_part];
	part.position = p_position;
	part.previous_position = p_position;
	part.rotation = p_rotation;
	part.previous_rotation = p_rotation;
}

void ArticulatedBody2D::set_part_velocity(int p_part, const Vector2 &p_linear_velocity, real_t p_angular_velocity) {
	ERR_FAIL_UNSIGNED_INDEX((uint32_t)p_part, parts.size());
	parts[p_part].linear_velocity = p_linear_velocity;
	parts[p_part].angular_velocity = p_angular_velocity;
}

void ArticulatedBody2D::set_part_radius(int p_part, real_t p_radius) {
	ERR_FAIL_UNSIGNED_INDEX((uint32_t)p_part, parts.size());
	parts[p_part].radius = MAX(p_radius, (real_t)0.0);
}

real_t ArticulatedBody2D::get_part_radius(int p_part) const {
	ERR_FAIL_UNSIGNED_INDEX_V((uint32_t)p_part, parts.size(), 0.0);
	return parts[p_part].radius;
}

Vector2 ArticulatedBody2D::get_part_position(int p_part) const {
	ERR_FAIL_UNSIGNED_INDEX_V((uint32_t)p_part, parts.size(), Vector2());
	return parts[p_part].position;
}

real_t ArticulatedBody2D::get_part_rotation(int p_part) const {
	ERR_FAIL_UNSIGNED_INDEX_V((uint32_t)p_part, parts.size(), 0.0);
	return parts[p_part].rotation;
}

Vector2 ArticulatedBody2D::get_part_linear_velocity(int p_part) const {
	ERR_FAIL_UNSIGNED_INDEX_V((uint32_t)p_part, parts.size(), Vector2());
	return parts[p_part].linear_velocity;
}

real_t ArticulatedBody2D::get_part_angular_velocity(int p_part) const {
	ERR_FAIL_UNSIGNED_INDEX_V((uint32_t)p_part, parts.size(), 0.0);
	return parts[p_part].angular_velocity;
}

PackedVector2Array ArticulatedBody2D::get_part_positions() const {
	PackedVector2Array positions;
	positions.resize(parts.size());
	Vector2 *write = positions.ptrw();
	for (uint32_t i = 0; i < parts.size(); i++) {
		write[i] = parts[i].position;
	}
	return positions;
}

PackedFloat32Array ArticulatedBody2D::get_part_rotations() const {
	PackedFloat32Array rotations;
	rotations.resize(parts.size());
	float *write = rotations.ptrw();
	for (uint32_t i = 0; i < parts.size(); i++) {
		write[i] = parts[i].rotation;
	}
	return rotations;
}

void ArticulatedBody2D::apply_impulse(int p_part, const Vector2 &p_impulse, const Vector2 &p_point) {
	ERR_FAIL_UNSIGNED_INDEX((uint32_t)p_part, parts.size());
	Part &part = parts[p_part];
	part.linear_velocity += p_impulse * part.inverse_mass;
	part.angular_velocity += (p_point - part.position).cross(p_impulse) * part.inverse_inertia;
}

int ArticulatedBody2D::add_joint(int p_part_a, int p_part_b, const Vector2 &p_anchor_a, const Vector2 &p_anchor_b) {
	ERR_FAIL_UNSIGNED_INDEX_V((uint32_t)p_part_a, parts.size(), -1);
	ERR_FAIL_UNSIGNED_INDEX_V((uint32_t)p_part_b, parts.size(), -1);
	ERR_FAIL_COND_V_MSG(p_part_a == p_part_b, -1, "A joint needs two different parts.");
	Joint joint;
	joint.part_a = p_part_a;
	joint.part_b = p_part_b;
	joint.anchor_a = p_anchor_a;
	joint.anchor_b = p_anchor_b;
	joints.push_back(joint);
	return joints.size() - 1;
}

int ArticulatedBody2D::get_joint_count() const {
	return joints.size();
}

void ArticulatedBody2D::set_joint_pin_compliance(int p_joint, real_t p_compliance) {
	ERR_FAIL_UNSIGNED_INDEX((uint32_t)p_joint, joints.size());
	joints[p_joint].pin_compliance = MAX(p_compliance, (real_t)0.0);
}

void ArticulatedBody2D::set_joint_limits(int p_joint, bool p_enabled, real_t p_lower, real_t p_upper) {
	ERR_FAIL_UNSIGNED_INDEX((uint32_t)p_joint, joints.size());
	ERR_FAIL_COND_MSG(p_lower > p_upper, "The lower limit must not exceed the upper one.");
	Joint &joint = joints[p_joint];
	joint.limit_enabled = p_enabled;
	joint.limit_lower = p_lower;
	joint.limit_upper = p_upper;
}

void ArticulatedBody2D::set_joint_drive(int p_joint, real_t p_target_angle, real_t p_compliance) {
	ERR_FAIL_UNSIGNED_INDEX((uint32_t)p_joint, joints.size());
	joints[p_joint].drive_target = p_target_angle;
	joints[p_joint].drive_compliance = p_compliance;
}

void ArticulatedBody2D::clear_contacts() {
	contacts.clear();
}

int ArticulatedBody2D::add_contact(int p_part, const Vector2 &p_local_point, const Vector2 &p_plane_point, const Vector2 &p_plane_normal, real_t p_friction) {
	ERR_FAIL_UNSIGNED_INDEX_V((uint32_t)p_part, parts.size(), -1);
	ERR_FAIL_COND_V_MSG(p_plane_normal.is_zero_approx(), -1, "A contact plane needs a normal.");
	Contact contact;
	contact.part = p_part;
	contact.local_point = p_local_point;
	contact.plane_point = p_plane_point;
	contact.plane_normal = p_plane_normal.normalized();
	contact.friction = MAX(p_friction, (real_t)0.0);
	contacts.push_back(contact);
	return contacts.size() - 1;
}

int ArticulatedBody2D::get_contact_count() const {
	return contacts.size();
}

void ArticulatedBody2D::set_gravity(const Vector2 &p_gravity) {
	gravity = p_gravity;
}

Vector2 ArticulatedBody2D::get_gravity() const {
	return gravity;
}

void ArticulatedBody2D::set_damping(real_t p_linear, real_t p_angular) {
	linear_damping = MAX(p_linear, (real_t)0.0);
	angular_damping = MAX(p_angular, (real_t)0.0);
}

// Closes the gap between a joint's two anchors along the line between them, or as much of it as the compliance term
// (the pin compliance over the substep length squared) leaves. Each part moves and turns by its share of the
// correction, weighted by how much its anchor can move per unit of impulse.
void ArticulatedBody2D::_project_pin(const Joint &p_joint, real_t p_compliance_term) {
	Part &a = parts[p_joint.part_a];
	Part &b = parts[p_joint.part_b];
	const Vector2 r_a = p_joint.anchor_a.rotated(a.rotation);
	const Vector2 r_b = p_joint.anchor_b.rotated(b.rotation);
	const Vector2 gap = (a.position + r_a) - (b.position + r_b);
	const real_t gap_length = gap.length();
	if (gap_length <= (real_t)CMP_EPSILON) {
		return;
	}
	const Vector2 normal = gap / gap_length;
	const real_t lever_a = r_a.cross(normal);
	const real_t lever_b = r_b.cross(normal);
	const real_t w_sum = a.inverse_mass + a.inverse_inertia * lever_a * lever_a + b.inverse_mass + b.inverse_inertia * lever_b * lever_b;
	if (w_sum <= 0.0) {
		return;
	}
	const Vector2 correction = normal * (-gap_length / (w_sum + p_compliance_term));
	a.position += correction * a.inverse_mass;
	a.rotation += a.inverse_inertia * r_a.cross(correction);
	b.position -= correction * b.inverse_mass;
	b.rotation -= b.inverse_inertia * r_b.cross(correction);
}

// Pushes a joint's relative angle back inside its range, rigidly. The angle is read around the middle of the range,
// so a range anywhere up to a full turn reads right.
void ArticulatedBody2D::_project_limit(const Joint &p_joint) {
	if (!p_joint.limit_enabled) {
		return;
	}
	Part &a = parts[p_joint.part_a];
	Part &b = parts[p_joint.part_b];
	const real_t w_sum_angular = a.inverse_inertia + b.inverse_inertia;
	if (w_sum_angular <= 0.0) {
		return;
	}
	const real_t middle = (p_joint.limit_lower + p_joint.limit_upper) * (real_t)0.5;
	const real_t relative = middle + Math::wrapf(b.rotation - a.rotation - middle, (real_t)-Math::PI, (real_t)Math::PI);
	real_t error = 0.0;
	if (relative > p_joint.limit_upper) {
		error = relative - p_joint.limit_upper;
	} else if (relative < p_joint.limit_lower) {
		error = relative - p_joint.limit_lower;
	}
	if (error == 0.0) {
		return;
	}
	const real_t delta_lambda = -error / w_sum_angular;
	a.rotation -= delta_lambda * a.inverse_inertia;
	b.rotation += delta_lambda * b.inverse_inertia;
}

void ArticulatedBody2D::set_speed_limits(real_t p_linear, real_t p_angular) {
	max_linear_speed = MAX(p_linear, (real_t)0.0);
	max_angular_speed = MAX(p_angular, (real_t)0.0);
}

void ArticulatedBody2D::settle(int p_iterations) {
	ERR_FAIL_COND(p_iterations < 1);
	// Rigid pins and limits only: a compliant pin is a tether to somewhere the body needn't be yet, and drives are
	// what the body moves toward over time, not where it is.
	for (int iteration = 0; iteration < p_iterations; iteration++) {
		for (const Joint &joint : joints) {
			if (joint.pin_compliance == 0.0) {
				_project_pin(joint, 0.0);
			}
			_project_limit(joint);
		}
	}
	// Where the parts ended up is where they are now; none of it was motion.
	for (Part &part : parts) {
		part.previous_position = part.position;
		part.previous_rotation = part.rotation;
	}
}

void ArticulatedBody2D::step(real_t p_delta, int p_substeps) {
	ERR_FAIL_COND(p_delta <= 0.0);
	ERR_FAIL_COND(p_substeps < 1);
	const real_t h = p_delta / p_substeps;

	for (int substep = 0; substep < p_substeps; substep++) {
		// Predict: integrate velocities and move every part by them. Damping is applied as an implicit factor so
		// any amount of it stays stable whatever the substep length.
		for (Part &part : parts) {
			if (part.inverse_mass == 0.0) {
				continue;
			}
			part.linear_velocity += gravity * h;
			part.linear_velocity /= 1.0 + linear_damping * h;
			part.angular_velocity /= 1.0 + angular_damping * h;
			part.previous_position = part.position;
			part.previous_rotation = part.rotation;
			part.position += part.linear_velocity * h;
			part.rotation += part.angular_velocity * h;
		}

		// Constraints: every joint pins its anchors together, drives its relative angle toward its target and keeps
		// it within its limits, each as a position projection on both parts, one pass per substep. Substeps do the
		// convergence work iterations would, and do it better, see Macklin et al., "Small Steps in Physics Simulation".
		const real_t h2 = h * h;
		for (const Joint &joint : joints) {
			Part &a = parts[joint.part_a];
			Part &b = parts[joint.part_b];

			_project_pin(joint, joint.pin_compliance / h2);

			const real_t w_sum_angular = a.inverse_inertia + b.inverse_inertia;
			if (w_sum_angular <= 0.0) {
				continue;
			}

			// Drive: pull the relative angle toward its target, giving by the compliance. The error is wrapped so a
			// target of any angle works, and the compliance is scaled by the substep length squared, which is what
			// keeps the same stiffness meaning the same thing at any substep count.
			if (joint.drive_compliance >= 0.0) {
				const real_t error = Math::wrapf(b.rotation - a.rotation - joint.drive_target, (real_t)-Math::PI, (real_t)Math::PI);
				const real_t delta_lambda = -error / (w_sum_angular + joint.drive_compliance / h2);
				a.rotation -= delta_lambda * a.inverse_inertia;
				b.rotation += delta_lambda * b.inverse_inertia;
			}

			// Limits come after the drive so a limit wins over it.
			_project_limit(joint);
		}

		// Contacts: push each contact point back to its plane, then hold it there by friction. Static friction cancels
		// the point's slide over the substep outright when the normal push can account for it through the friction
		// coefficient; otherwise the point slides and dynamic friction slows it in the velocity pass below.
		for (Contact &contact : contacts) {
			Part &part = parts[contact.part];
			contact.lambda_normal = 0.0;
			if (part.inverse_mass == 0.0) {
				continue;
			}
			const Vector2 r = contact.local_point.rotated(part.rotation);
			const real_t depth = (part.position + r - contact.plane_point).dot(contact.plane_normal) - part.radius;
			if (depth >= 0.0) {
				continue;
			}
			const real_t lever = r.cross(contact.plane_normal);
			const real_t w = part.inverse_mass + part.inverse_inertia * lever * lever;
			if (w <= 0.0) {
				continue;
			}
			const real_t delta_lambda = -depth / w;
			const Vector2 correction = contact.plane_normal * delta_lambda;
			part.position += correction * part.inverse_mass;
			part.rotation += part.inverse_inertia * r.cross(correction);
			contact.lambda_normal = delta_lambda;

			// Static friction, from how far the point slid along the plane since the substep began.
			const Vector2 r_now = contact.local_point.rotated(part.rotation);
			const Vector2 slide = (part.position + r_now) - (part.previous_position + contact.local_point.rotated(part.previous_rotation));
			const Vector2 tangential = slide - contact.plane_normal * slide.dot(contact.plane_normal);
			const real_t tangential_length = tangential.length();
			if (tangential_length <= (real_t)CMP_EPSILON) {
				continue;
			}
			const Vector2 tangent = tangential / tangential_length;
			const real_t lever_t = r_now.cross(tangent);
			const real_t w_t = part.inverse_mass + part.inverse_inertia * lever_t * lever_t;
			const real_t needed = tangential_length / w_t; // The positional impulse that would undo the slide
			if (needed <= contact.friction * contact.lambda_normal) {
				const Vector2 hold = tangent * (-needed);
				part.position += hold * part.inverse_mass;
				part.rotation += part.inverse_inertia * r_now.cross(hold);
			}
		}

		// Velocity update: what the positions did over the substep is the velocity, constraints included. The caps
		// are a safety net: a constraint that has to correct a lot at once would otherwise read as a huge velocity.
		for (Part &part : parts) {
			if (part.inverse_mass == 0.0) {
				continue;
			}
			part.linear_velocity = (part.position - part.previous_position) / h;
			part.angular_velocity = (part.rotation - part.previous_rotation) / h;
			if (max_linear_speed > 0.0) {
				part.linear_velocity = part.linear_velocity.limit_length(max_linear_speed);
			}
			if (max_angular_speed > 0.0) {
				part.angular_velocity = CLAMP(part.angular_velocity, -max_angular_speed, max_angular_speed);
			}
		}

		// Contact velocities: a touching point keeps no velocity into or out of its plane, so being pushed out of it
		// doesn't read as a bounce, and dynamic friction takes as much of its slide as the normal push allows.
		for (const Contact &contact : contacts) {
			if (contact.lambda_normal <= 0.0) {
				continue;
			}
			Part &part = parts[contact.part];
			const Vector2 r = contact.local_point.rotated(part.rotation);
			const Vector2 point_velocity = part.linear_velocity + Vector2(-r.y, r.x) * part.angular_velocity;

			const real_t normal_velocity = point_velocity.dot(contact.plane_normal);
			if (normal_velocity != 0.0) {
				const real_t lever = r.cross(contact.plane_normal);
				const real_t w = part.inverse_mass + part.inverse_inertia * lever * lever;
				const Vector2 impulse = contact.plane_normal * (-normal_velocity / w);
				part.linear_velocity += impulse * part.inverse_mass;
				part.angular_velocity += part.inverse_inertia * r.cross(impulse);
			}

			const Vector2 tangential_velocity = point_velocity - contact.plane_normal * normal_velocity;
			const real_t tangential_speed = tangential_velocity.length();
			if (tangential_speed <= (real_t)CMP_EPSILON || contact.friction <= 0.0) {
				continue;
			}
			const Vector2 tangent = tangential_velocity / tangential_speed;
			const real_t lever_t = r.cross(tangent);
			const real_t w_t = part.inverse_mass + part.inverse_inertia * lever_t * lever_t;
			// The normal push over the substep, as an impulse, bounds the friction impulse through the coefficient.
			const real_t friction_impulse = MIN(contact.friction * contact.lambda_normal / h, tangential_speed / w_t);
			const Vector2 impulse = tangent * (-friction_impulse);
			part.linear_velocity += impulse * part.inverse_mass;
			part.angular_velocity += part.inverse_inertia * r.cross(impulse);
		}
	}
}

void ArticulatedBody2D::_bind_methods() {
	ClassDB::bind_method(D_METHOD("clear"), &ArticulatedBody2D::clear);

	ClassDB::bind_method(D_METHOD("add_part", "position", "rotation", "mass", "inertia"), &ArticulatedBody2D::add_part);
	ClassDB::bind_method(D_METHOD("get_part_count"), &ArticulatedBody2D::get_part_count);
	ClassDB::bind_method(D_METHOD("set_part_transform", "part", "position", "rotation"), &ArticulatedBody2D::set_part_transform);
	ClassDB::bind_method(D_METHOD("set_part_velocity", "part", "linear_velocity", "angular_velocity"), &ArticulatedBody2D::set_part_velocity);
	ClassDB::bind_method(D_METHOD("set_part_radius", "part", "radius"), &ArticulatedBody2D::set_part_radius);
	ClassDB::bind_method(D_METHOD("get_part_radius", "part"), &ArticulatedBody2D::get_part_radius);
	ClassDB::bind_method(D_METHOD("get_part_position", "part"), &ArticulatedBody2D::get_part_position);
	ClassDB::bind_method(D_METHOD("get_part_rotation", "part"), &ArticulatedBody2D::get_part_rotation);
	ClassDB::bind_method(D_METHOD("get_part_linear_velocity", "part"), &ArticulatedBody2D::get_part_linear_velocity);
	ClassDB::bind_method(D_METHOD("get_part_angular_velocity", "part"), &ArticulatedBody2D::get_part_angular_velocity);
	ClassDB::bind_method(D_METHOD("get_part_positions"), &ArticulatedBody2D::get_part_positions);
	ClassDB::bind_method(D_METHOD("get_part_rotations"), &ArticulatedBody2D::get_part_rotations);
	ClassDB::bind_method(D_METHOD("apply_impulse", "part", "impulse", "point"), &ArticulatedBody2D::apply_impulse);

	ClassDB::bind_method(D_METHOD("add_joint", "part_a", "part_b", "anchor_a", "anchor_b"), &ArticulatedBody2D::add_joint);
	ClassDB::bind_method(D_METHOD("get_joint_count"), &ArticulatedBody2D::get_joint_count);
	ClassDB::bind_method(D_METHOD("set_joint_pin_compliance", "joint", "compliance"), &ArticulatedBody2D::set_joint_pin_compliance);
	ClassDB::bind_method(D_METHOD("set_joint_limits", "joint", "enabled", "lower", "upper"), &ArticulatedBody2D::set_joint_limits);
	ClassDB::bind_method(D_METHOD("set_joint_drive", "joint", "target_angle", "compliance"), &ArticulatedBody2D::set_joint_drive);

	ClassDB::bind_method(D_METHOD("clear_contacts"), &ArticulatedBody2D::clear_contacts);
	ClassDB::bind_method(D_METHOD("add_contact", "part", "local_point", "plane_point", "plane_normal", "friction"), &ArticulatedBody2D::add_contact);
	ClassDB::bind_method(D_METHOD("get_contact_count"), &ArticulatedBody2D::get_contact_count);

	ClassDB::bind_method(D_METHOD("set_gravity", "gravity"), &ArticulatedBody2D::set_gravity);
	ClassDB::bind_method(D_METHOD("get_gravity"), &ArticulatedBody2D::get_gravity);
	ClassDB::bind_method(D_METHOD("set_damping", "linear", "angular"), &ArticulatedBody2D::set_damping);
	ClassDB::bind_method(D_METHOD("set_speed_limits", "linear", "angular"), &ArticulatedBody2D::set_speed_limits);

	ClassDB::bind_method(D_METHOD("settle", "iterations"), &ArticulatedBody2D::settle, DEFVAL(8));

	ClassDB::bind_method(D_METHOD("step", "delta", "substeps"), &ArticulatedBody2D::step, DEFVAL(4));
}

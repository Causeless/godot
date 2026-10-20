/**************************************************************************/
/*  path_search.cpp                                                       */
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

#include "path_search.h"

#include "core/math/math_funcs.h"
#include "core/templates/local_vector.h"

#include <limits>

static double _get_cost(const Dictionary &p_costs, const char *p_key, double p_default) {
	const Variant *value = p_costs.getptr(String(p_key));
	return value ? (double)(*value) : p_default;
}

// Min-heap of connection ids keyed by cost, in two parallel vectors so that pushing and popping never allocate once
// warm. The same id may be queued several times; the search skips entries that come out already expanded.
struct Frontier {
	LocalVector<int32_t> ids; // Heap-ordered ids. Index i's children are 2i + 1 and 2i + 2.
	LocalVector<double> costs; // Cost of the id at the same index.

	bool is_empty() const {
		return ids.is_empty();
	}

	void insert(int32_t p_id, double p_cost) {
		ids.push_back(p_id);
		costs.push_back(p_cost);

		// Sift up, shifting parents down rather than swapping; the moving entry is written once at the end.
		uint32_t index = ids.size() - 1;
		while (index > 0) {
			const uint32_t parent = (index - 1) / 2;
			if (costs[parent] <= p_cost) {
				break;
			}

			ids[index] = ids[parent];
			costs[index] = costs[parent];
			index = parent;
		}

		ids[index] = p_id;
		costs[index] = p_cost;
	}

	// Removes and returns the cheapest id. The heap must not be empty.
	int32_t extract_min() {
		const int32_t result = ids[0];
		const uint32_t last = ids.size() - 1;
		const int32_t id = ids[last];
		const double cost = costs[last];
		ids.resize(last);
		costs.resize(last);
		if (last == 0) {
			return result;
		}

		// Sift the last leaf down from the root until both children are no cheaper.
		uint32_t index = 0;
		while (true) {
			const uint32_t left = index * 2 + 1;
			if (left >= last) {
				break;
			}

			uint32_t child = left;
			const uint32_t right = left + 1;
			if (right < last && costs[right] < costs[left]) {
				child = right;
			}
			if (costs[child] >= cost) {
				break;
			}

			ids[index] = ids[child];
			costs[index] = costs[child];
			index = child;
		}

		ids[index] = id;
		costs[index] = cost;
		return result;
	}
};

// Follows p_previous back from p_end_id to the virtual start and returns the connections in travel order. The
// virtual start itself is never included.
static PackedInt32Array _build_path(const LocalVector<int32_t> &p_previous, int32_t p_end_id, int32_t p_start_id) {
	PackedInt32Array path;
	int32_t current_id = p_end_id;
	while (current_id != p_start_id && current_id != -1) {
		path.push_back(current_id);
		current_id = p_previous[current_id];
	}
	path.reverse();
	return path;
}

// Weighted A* over connections rather than nodes: the frontier holds connections, each reached at the cost of
// getting to it from where the previous connection landed plus its own. The actor's position acts as a virtual
// connection just taken into the start node, with the id one past the real ones, so the first real connections are
// priced from where the actor actually stands. Expanded connections are never reopened: the heuristic isn't
// consistent, and reopening would re-expand whole subtrees for little gain, while not reopening keeps weighted A*'s
// usual bound and caps the iterations at one per connection.
Dictionary PathSearch::find_path(const Dictionary &p_graph, int p_start_node, int p_goal_node, const Vector2 &p_actor_position, int p_actor_level, int p_actor_key, const Vector2 &p_goal_position, const Dictionary &p_costs, int p_max_iterations, int p_stop_at_step) {
	Dictionary result;
	result["path"] = PackedInt32Array();
	result["steps"] = 0;
	result["edges"] = 0;

	const PackedVector2Array connection_position_array = p_graph.get("connection_position", PackedVector2Array());
	const PackedVector2Array connection_landing_array = p_graph.get("connection_landing", PackedVector2Array());
	const PackedFloat32Array connection_runup_x_array = p_graph.get("connection_runup_x", PackedFloat32Array());
	const PackedInt32Array connection_runup_sign_array = p_graph.get("connection_runup_sign", PackedInt32Array());
	const PackedFloat64Array connection_cost_array = p_graph.get("connection_cost", PackedFloat64Array());
	const PackedByteArray connection_level_array = p_graph.get("connection_level", PackedByteArray());
	const PackedByteArray connection_flags_array = p_graph.get("connection_flags", PackedByteArray());
	const PackedInt32Array connection_key_array = p_graph.get("connection_key", PackedInt32Array());
	const PackedInt32Array connection_start_node_array = p_graph.get("connection_start_node", PackedInt32Array());
	const PackedInt32Array connection_end_node_array = p_graph.get("connection_end_node", PackedInt32Array());
	const PackedByteArray node_active_array = p_graph.get("node_active", PackedByteArray());
	const PackedByteArray node_is_climb_array = p_graph.get("node_is_climb", PackedByteArray());
	const PackedInt32Array node_connection_begin_array = p_graph.get("node_connection_begin", PackedInt32Array());
	const PackedInt32Array node_connection_ids_array = p_graph.get("node_connection_ids", PackedInt32Array());

	const int connection_count = connection_flags_array.size();
	const int node_count = node_active_array.size();
	ERR_FAIL_COND_V_MSG(connection_position_array.size() != connection_count || connection_landing_array.size() != connection_count || connection_runup_x_array.size() != connection_count || connection_runup_sign_array.size() != connection_count || connection_cost_array.size() != connection_count || connection_level_array.size() != connection_count || connection_key_array.size() != connection_count || connection_start_node_array.size() != connection_count || connection_end_node_array.size() != connection_count, result, "The graph's connection arrays must all be the same size.");
	ERR_FAIL_COND_V_MSG(node_is_climb_array.size() != node_count || node_connection_begin_array.size() != node_count + 1, result, "The graph's node arrays must all be the same size, with one extra trailing entry in node_connection_begin.");
	ERR_FAIL_COND_V_MSG(p_start_node < 0 || p_start_node >= node_count || p_goal_node < 0 || p_goal_node >= node_count, result, "The start and goal nodes must be in the graph.");

	const Vector2 *connection_position = connection_position_array.ptr();
	const Vector2 *connection_landing = connection_landing_array.ptr();
	const float *connection_runup_x = connection_runup_x_array.ptr();
	const int32_t *connection_runup_sign = connection_runup_sign_array.ptr();
	const double *connection_cost = connection_cost_array.ptr();
	const uint8_t *connection_level = connection_level_array.ptr();
	const uint8_t *connection_flags = connection_flags_array.ptr();
	const int32_t *connection_key = connection_key_array.ptr();
	const int32_t *connection_start_node = connection_start_node_array.ptr();
	const int32_t *connection_end_node = connection_end_node_array.ptr();
	const uint8_t *node_active = node_active_array.ptr();
	const uint8_t *node_is_climb = node_is_climb_array.ptr();
	const int32_t *node_connection_begin = node_connection_begin_array.ptr();
	const int32_t *node_connection_ids = node_connection_ids_array.ptr();

	// Every index the search follows is checked here, so the loop below needs no bounds checks.
	const int node_connection_ids_count = node_connection_ids_array.size();
	ERR_FAIL_COND_V_MSG(node_connection_begin[0] != 0 || node_connection_begin[node_count] != node_connection_ids_count, result, "node_connection_begin must start at 0 and end at the size of node_connection_ids.");
	for (int node = 0; node < node_count; node++) {
		ERR_FAIL_COND_V_MSG(node_connection_begin[node] > node_connection_begin[node + 1], result, "node_connection_begin must not decrease.");
	}
	for (int i = 0; i < node_connection_ids_count; i++) {
		ERR_FAIL_COND_V_MSG(node_connection_ids[i] < 0 || node_connection_ids[i] >= connection_count, result, "node_connection_ids holds a connection id outside the graph.");
	}
	for (int connection = 0; connection < connection_count; connection++) {
		ERR_FAIL_COND_V_MSG(connection_start_node[connection] < 0 || connection_start_node[connection] >= node_count || connection_end_node[connection] < 0 || connection_end_node[connection] >= node_count, result, "A connection starts or ends at a node outside the graph.");
	}

	if (p_start_node == p_goal_node) {
		return result;
	}

	const double walk_cost_per_unit = _get_cost(p_costs, "walk_cost_per_unit", 1.0); // What a unit of horizontal distance walked between connections costs.
	const double heuristic_per_unit = _get_cost(p_costs, "heuristic_per_unit", 1.0); // What a unit of straight-line distance still to the goal counts for. More than walk_cost_per_unit trades path quality for fewer iterations.
	const double climb_vertical_mult = _get_cost(p_costs, "climb_vertical_mult", 1.0); // On a climb node, how many walked units a unit of height to the connection counts as.
	const double heuristic_up_mult = _get_cost(p_costs, "heuristic_up_mult", 1.0); // How much a unit of height still to climb to the goal counts in the heuristic.
	const double heuristic_down_mult = _get_cost(p_costs, "heuristic_down_mult", 1.0); // How much a unit of height still to drop to the goal counts in the heuristic.

	// The virtual start connection, see above.
	const int32_t start_id = connection_count;

	// cost_so_far is the cheapest actual cost each connection has been reached with, previous the connection it was
	// reached from (-1 for none) and closed marks connections already expanded.
	Frontier frontier;
	LocalVector<double> cost_so_far;
	LocalVector<int32_t> previous;
	LocalVector<uint8_t> closed;
	cost_so_far.resize(connection_count + 1);
	previous.resize(connection_count + 1);
	closed.resize(connection_count + 1);
	for (int i = 0; i <= connection_count; i++) {
		cost_so_far[i] = std::numeric_limits<double>::infinity();
		previous[i] = -1;
		closed[i] = 0;
	}
	frontier.insert(start_id, 0.0);
	cost_so_far[start_id] = 0.0;

	int steps_taken = 0; // Iterations so far.
	int edges_evaluated = 0; // Outgoing connections looked at so far, over all iterations.

	while (!frontier.is_empty()) {
		const int32_t current_id = frontier.extract_min();

		// Already expanded: a stale duplicate entry, or a cheaper route found after the fact. Skip either.
		if (closed[current_id]) {
			continue;
		}

		if (steps_taken == p_stop_at_step) {
			result["path"] = _build_path(previous, current_id, start_id);
			break;
		}

		if (steps_taken > p_max_iterations) {
			break;
		}

		steps_taken++;
		closed[current_id] = 1;

		// Where the actor stands after taking the current connection, which node that puts it in, and which node the
		// connection came from, so the search doesn't immediately walk back into it.
		Vector2 current_landing;
		int32_t current_node;
		int32_t current_came_from_node;
		if (current_id == start_id) {
			current_landing = p_actor_position;
			current_node = p_start_node;
			current_came_from_node = -1;
		} else {
			current_landing = connection_landing[current_id];
			current_node = connection_end_node[current_id];
			current_came_from_node = connection_start_node[current_id];
		}

		if (current_node == p_goal_node) {
			result["path"] = _build_path(previous, current_id, start_id);
			break;
		}

		// Every connection out of an inactive node is unavailable, so there's nothing to expand.
		if (!node_active[current_node]) {
			continue;
		}

		const double current_cost = cost_so_far[current_id];
		const double current_x = current_landing.x;

		for (int32_t index = node_connection_begin[current_node]; index < node_connection_begin[current_node + 1]; index++) {
			const int32_t new_id = node_connection_ids[index];
			edges_evaluated++;
			const int32_t new_end_node = connection_end_node[new_id];

			if (new_end_node == current_came_from_node) {
				continue;
			}

			const uint8_t flags = connection_flags[new_id];
			if (!(flags & CONNECTION_FLAG_ACTIVE)) {
				continue;
			}
			if (!node_active[new_end_node]) {
				continue;
			}
			if (connection_level[new_id] > p_actor_level) {
				continue;
			}

			// A closed and locked connection only opens for its own key, and KEY_NONE opens nothing.
			if ((flags & (CONNECTION_FLAG_CLOSED | CONNECTION_FLAG_LOCKED)) == (CONNECTION_FLAG_CLOSED | CONNECTION_FLAG_LOCKED)) {
				if (connection_key[new_id] == KEY_NONE || connection_key[new_id] != p_actor_key) {
					continue;
				}
			}

			// Walking cost: horizontal distance from where the actor stands to the connection. A connection that needs
			// a runup, approached from the wrong side, costs the detour back to the runup point and then to the
			// connection.
			const Vector2 new_position = connection_position[new_id];
			double walk_distance = Math::abs(current_x - (double)new_position.x);
			const int32_t runup_sign = connection_runup_sign[new_id];
			if (runup_sign != 0) {
				const double runup_x = connection_runup_x[new_id];
				const double runup_side = runup_x - current_x;
				const int32_t approach_sign = runup_side > 0.0 ? 1 : (runup_side < 0.0 ? -1 : 0);
				if (approach_sign != runup_sign) {
					walk_distance = Math::abs(current_x - runup_x) + Math::abs((double)new_position.x - runup_x);
				}
			}

			// On a climb node, reaching the connection also means climbing to its height.
			if (node_is_climb[current_node]) {
				walk_distance += Math::abs((double)current_landing.y - (double)new_position.y) * climb_vertical_mult;
			}

			const double new_cost = current_cost + walk_distance * walk_cost_per_unit + connection_cost[new_id];

			// Heuristic: distance from this connection's landing point to the goal, horizontal in full and vertical
			// scaled by whether the goal is above or below.
			Vector2 to_goal = p_goal_position - connection_landing[new_id];
			to_goal.y *= to_goal.y > 0.0f ? heuristic_down_mult : heuristic_up_mult;
			const double total_cost = new_cost + (double)to_goal.length() * heuristic_per_unit;

			// Relax on the actual cost; the heuristic only decides the order things are looked at in. Closed
			// connections are left alone, see above.
			if (!closed[new_id] && new_cost < cost_so_far[new_id]) {
				cost_so_far[new_id] = new_cost;
				previous[new_id] = current_id;
				frontier.insert(new_id, total_cost);
			}
		}
	}

	// Reached with the path found, the search stopped early, or no path at all, which leaves the path empty.
	result["steps"] = steps_taken;
	result["edges"] = edges_evaluated;
	return result;
}

void PathSearch::_bind_methods() {
	ClassDB::bind_static_method("PathSearch", D_METHOD("find_path", "graph", "start_node", "goal_node", "actor_position", "actor_level", "actor_key", "goal_position", "costs", "max_iterations", "stop_at_step"), &PathSearch::find_path);

	BIND_CONSTANT(CONNECTION_FLAG_ACTIVE);
	BIND_CONSTANT(CONNECTION_FLAG_CLOSED);
	BIND_CONSTANT(CONNECTION_FLAG_LOCKED);
	BIND_CONSTANT(KEY_NONE);
	BIND_CONSTANT(NO_STOP);
}

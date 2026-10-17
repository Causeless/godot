/**************************************************************************/
/*  acoustic_flood.cpp                                                    */
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

#include "acoustic_flood.h"

#include "core/math/math_funcs.h"
#include "core/templates/local_vector.h"

#include <cstring>

// Flood steps in expansion order: the four sides first, then the diagonals, which are only taken when both sides they cut between are open.
static const int NEIGHBOUR_COUNT = 8;
static const int ORTHOGONAL_NEIGHBOUR_COUNT = 4;
static const int NEIGHBOUR_DX[NEIGHBOUR_COUNT] = { 0, -1, 1, 0, -1, 1, -1, 1 };
static const int NEIGHBOUR_DY[NEIGHBOUR_COUNT] = { -1, 0, 0, 1, -1, -1, 1, 1 };

static int _get_cost(const Dictionary &p_costs, const char *p_key, int p_default) {
	const Variant *value = p_costs.getptr(String(p_key));
	return value ? (int)(*value) : p_default;
}

// Floods the window of the grid around the origin from the seeds, in two layers of window cells, one per route: a path
// starts in the around layer and moves to the through layer the first time it enters terrain that costs anything or a
// blocker, and stays there, so the around layer holds what's reachable without going through anything and the through
// layer what's reachable by going through something, open air beyond it included.
// Dial's algorithm: cells wait in a ring of buckets indexed by their distance, and are expanded in distance order like
// Dijkstra, without a heap. Every edge cost is a small positive integer, so a ring one larger than the largest step
// never holds two distances in one bucket.
Dictionary AcousticFlood::flood(const PackedByteArray &p_solid_cells, const PackedByteArray &p_cell_weights, int p_row_length, int p_row_amount, const Vector2i &p_window_origin_cell, int p_window_side, const PackedVector2Array &p_seed_window_cells, const Vector2 &p_player_point, const PackedVector2Array &p_blocked_cells, const PackedInt32Array &p_blocked_costs, const PackedByteArray &p_blocked_occlusions, const Dictionary &p_costs) {
	Dictionary result;
	const int side = p_window_side;
	ERR_FAIL_COND_V_MSG(side <= 0, result, "The window side must be positive.");
	ERR_FAIL_COND_V_MSG(p_row_length < 0 || p_row_amount < 0 || p_solid_cells.size() != (int64_t)p_row_length * p_row_amount, result, "The solid cells don't match the grid's row length and amount.");
	ERR_FAIL_COND_V_MSG(p_blocked_costs.size() != p_blocked_cells.size() || p_blocked_occlusions.size() != p_blocked_cells.size(), result, "Every blocked cell needs a cost and an occlusion.");

	// Costs in tenths of a cell, the flood's unit. Each can be overridden through p_costs.
	const int cost_orthogonal = _get_cost(p_costs, "cost_orthogonal", 10); // Step to a side neighbour.
	const int cost_diagonal = _get_cost(p_costs, "cost_diagonal", 14); // Step to a diagonal neighbour.
	const int max_distance = _get_cost(p_costs, "max_distance", side / 2 * cost_orthogonal); // Largest distance kept, so the field is a disc rather than filling the window's corners.
	const int material_cost = _get_cost(p_costs, "material_cost", 200); // Entering a cell of weight WEIGHT_FULL; lighter cells cost their share.
	const int material_cost_max = _get_cost(p_costs, "material_cost_max", 600); // Most material cost a path may carry before it stops entering material.
	const int blocker_cost_max = _get_cost(p_costs, "blocker_cost_max", 400); // Most a blocker may charge, which sizes the bucket ring.
	const int wall_cost = _get_cost(p_costs, "wall_cost", 400); // What a wall is worth: a cell further than its straight line plus this seals whatever resolves from it.
	const int through_dominance_margin = _get_cost(p_costs, "through_dominance_margin", 80); // How much cheaper the around layer may reach a cell before the through layer stops expanding it.
	const int seal_prune_margin = _get_cost(p_costs, "seal_prune_margin", 30); // Slack past the straight line plus a wall a cell may still be flooded.
	ERR_FAIL_COND_V_MSG(cost_orthogonal <= 0 || cost_diagonal <= 0 || material_cost < 0 || blocker_cost_max < 0, result, "Step costs must be positive and material and blocker costs can't be negative.");

	const int cell_count = side * side;
	const int layered_count = cell_count * ROUTE_COUNT;
	const int through_offset = cell_count * ROUTE_THROUGH; // Where the through layer's cells start, after the around layer's.
	const int largest_step = cost_diagonal + blocker_cost_max + material_cost; // A diagonal step out of a blocker straight into material charges all three at once.
	const int bucket_count = largest_step + 1;

	// Solidity and weight of every window cell up front, so the loops below need no grid bounds checks. Cells outside the
	// grid are open air. The weight is what entering a solid cell costs, as a share of the material cost. A bake without
	// weights counts every solid cell as full.
	PackedByteArray window_solid;
	window_solid.resize(cell_count);
	uint8_t *solid = window_solid.ptrw();
	memset(solid, 0, cell_count);
	LocalVector<uint8_t> window_weights;
	window_weights.resize(cell_count);
	memset(window_weights.ptr(), 0, cell_count);
	const bool has_weights = p_cell_weights.size() == p_solid_cells.size() && p_cell_weights.size() > 0;
	const uint8_t *grid_solid = p_solid_cells.ptr();
	const uint8_t *grid_weights = has_weights ? p_cell_weights.ptr() : nullptr;
	for (int window_y = 0; window_y < side; window_y++) {
		const int grid_y = p_window_origin_cell.y + window_y;
		if (grid_y < 0 || grid_y >= p_row_amount) {
			continue;
		}

		for (int window_x = 0; window_x < side; window_x++) {
			const int grid_x = p_window_origin_cell.x + window_x;
			if (grid_x < 0 || grid_x >= p_row_length) {
				continue;
			}

			const int window_index = window_y * side + window_x;
			const int grid_index = grid_y * p_row_length + grid_x;
			if (grid_solid[grid_index]) {
				solid[window_index] = 1;
				window_weights[window_index] = has_weights ? grid_weights[grid_index] : (uint8_t)WEIGHT_FULL;
			}
		}
	}

	// Blockers (closed doors) can be passed through, each at its own price, and block line of sight like terrain.
	LocalVector<int32_t> window_blocker_cost;
	window_blocker_cost.resize(cell_count);
	LocalVector<uint8_t> window_blocker_occlusion;
	window_blocker_occlusion.resize(cell_count);
	for (int i = 0; i < cell_count; i++) {
		window_blocker_cost[i] = NOT_BLOCKED;
		window_blocker_occlusion[i] = 0;
	}

	PackedByteArray window_opaque = window_solid.duplicate();
	uint8_t *opaque = window_opaque.ptrw();
	const Vector2 *blocked_cells = p_blocked_cells.ptr();
	const int32_t *blocked_costs = p_blocked_costs.ptr();
	const uint8_t *blocked_occlusions = p_blocked_occlusions.ptr();
	for (int blocked = 0; blocked < p_blocked_cells.size(); blocked++) {
		const int window_x = (int)blocked_cells[blocked].x - p_window_origin_cell.x;
		const int window_y = (int)blocked_cells[blocked].y - p_window_origin_cell.y;
		if (window_x < 0 || window_y < 0 || window_x >= side || window_y >= side) {
			continue;
		}

		const int blocked_index = window_y * side + window_x;
		window_blocker_cost[blocked_index] = CLAMP(blocked_costs[blocked], 0, blocker_cost_max);
		window_blocker_occlusion[blocked_index] = MIN(blocked_occlusions[blocked], (uint8_t)BLOCKER_OCCLUSION_MASK);
		opaque[blocked_index] = 1;
	}

	// The field, both layers in one array each.
	PackedInt32Array distances;
	distances.resize(layered_count);
	distances.fill(UNREACHED);
	int32_t *dist = distances.ptrw();
	PackedInt32Array parents;
	parents.resize(layered_count);
	parents.fill(UNREACHED);
	int32_t *parent = parents.ptrw();
	PackedByteArray blocker_occlusions;
	blocker_occlusions.resize(layered_count);
	uint8_t *labels = blocker_occlusions.ptrw();
	memset(labels, 0, layered_count);
	PackedInt32Array material_costs;
	material_costs.resize(layered_count);
	material_costs.fill(0);
	int32_t *material = material_costs.ptrw();
	PackedInt32Array surcharges;
	surcharges.resize(layered_count);
	surcharges.fill(0);
	int32_t *surcharge = surcharges.ptrw();

	LocalVector<LocalVector<int32_t>> buckets;
	buckets.resize(bucket_count);

	// Every seed starts at its true distance from the player, so a player inside a thin wall reaches both sides of it directly
	// through whichever exit is nearer each thing. A seed in a blocker's cells counts as having entered it: a through cell with
	// the crossing pending, so what's beyond stays behind the door and nothing is reached around.
	int pending_count = 0; // Queued cells not yet expanded, including stale duplicates.
	const Vector2 *seeds = p_seed_window_cells.ptr();
	for (int seed = 0; seed < p_seed_window_cells.size(); seed++) {
		const int seed_x = (int)seeds[seed].x;
		const int seed_y = (int)seeds[seed].y;
		ERR_CONTINUE_MSG(seed_x < 0 || seed_y < 0 || seed_x >= side || seed_y >= side, "A seed lies outside the window.");
		int seed_layered_index = seed_y * side + seed_x;
		if (window_blocker_cost[seed_layered_index] != NOT_BLOCKED) {
			seed_layered_index += through_offset;
			labels[seed_layered_index] = BLOCKER_PENDING_BIT;
		}

		const Vector2 exit_point = p_player_point.clamp(Vector2(seed_x, seed_y), Vector2(seed_x + 1, seed_y + 1));
		const int seed_distance = (int)Math::round(exit_point.distance_to(p_player_point) * 10.0);
		dist[seed_layered_index] = seed_distance;
		buckets[seed_distance % bucket_count].push_back(seed_layered_index);
		pending_count++;
	}

	for (int distance = 0; pending_count > 0; distance++) {
		LocalVector<int32_t> &bucket = buckets[distance % bucket_count];
		if (bucket.is_empty()) {
			continue;
		}

		// Expansions land in later buckets, never this one, so the bucket can be walked in place and emptied after.
		pending_count -= bucket.size();
		for (uint32_t entry = 0; entry < bucket.size(); entry++) {
			const int layered_index = bucket[entry];
			if (dist[layered_index] != distance) {
				continue; // Queued again cheaper since, that entry handles it.
			}

			const bool in_through = layered_index >= through_offset;
			const int index = in_through ? layered_index - through_offset : layered_index;

			// A through cell the around layer reaches this much cheaper carries nothing of the sound there, and nothing beyond it
			// is reached any cheaper through it than from its around self, which crosses into the through layer at the same
			// price, so expanding it would be wasted. The around entry was expanded before this one if it's cheaper.
			if (in_through && dist[index] != UNREACHED && dist[index] + through_dominance_margin < distance) {
				continue;
			}

			const int window_x = index % side;
			const int window_y = index / side;
			for (int neighbour = 0; neighbour < NEIGHBOUR_COUNT; neighbour++) {
				const int dx = NEIGHBOUR_DX[neighbour];
				const int dy = NEIGHBOUR_DY[neighbour];
				const int neighbour_x = window_x + dx;
				const int neighbour_y = window_y + dy;
				if (neighbour_x < 0 || neighbour_y < 0 || neighbour_x >= side || neighbour_y >= side) {
					continue;
				}

				const int neighbour_index = neighbour_y * side + neighbour_x;
				int next_distance = distance + cost_orthogonal;
				if (neighbour >= ORTHOGONAL_NEIGHBOUR_COUNT) {
					// No cutting through the joint between two opaque cells. Both side cells lie inside the window, as the diagonal does.
					if (opaque[index + dx] || opaque[index + dy * side]) {
						continue;
					}

					next_distance = distance + cost_diagonal;
				}

				// Entering terrain that costs anything, or a blocker, takes the path into the through layer, where it stays. Terrain
				// worth nothing is passed in either layer: a one-tile gap is a way around, not through.
				const bool crossing = window_blocker_cost[neighbour_index] != NOT_BLOCKED || window_weights[neighbour_index] > 0;
				const int neighbour_layered_index = (in_through || crossing) ? neighbour_index + through_offset : neighbour_index;

				// A blocker is charged and its occlusion added once per full crossing: entering its cells from open air makes the
				// crossing pending, leaving them for open air completes it, at the price of the cell just left. Paths starting or
				// ending inside only ever do half a crossing, which is free.
				int label = labels[layered_index];
				int next_surcharge = surcharge[layered_index];
				if (window_blocker_cost[neighbour_index] != NOT_BLOCKED) {
					if (window_blocker_cost[index] == NOT_BLOCKED) {
						label |= BLOCKER_PENDING_BIT;
					}
				} else if (label & BLOCKER_PENDING_BIT) {
					label = MIN((label & BLOCKER_OCCLUSION_MASK) + window_blocker_occlusion[index], BLOCKER_OCCLUSION_MASK);
					next_distance += window_blocker_cost[index];
					next_surcharge += window_blocker_cost[index];
				}

				// Terrain is entered at its weight's share of the material cost, which the path carries along as what it occludes
				// by. A path carrying more than any unsealed route could bear stops entering material: everything beyond is sealed
				// through it, and the flood would otherwise fill every thick mass in the window.
				int next_material = material[layered_index];
				if (solid[neighbour_index]) {
					const int entry_cost = material_cost * window_weights[neighbour_index] / WEIGHT_FULL;
					next_material += entry_cost;
					if (next_material > material_cost_max) {
						continue;
					}

					next_distance += entry_cost;
					next_surcharge += entry_cost;
				}

				if (next_distance > max_distance) {
					continue;
				}

				const int current_distance = dist[neighbour_layered_index];
				if (current_distance != UNREACHED && current_distance <= next_distance) {
					continue;
				}

				// A cell this much further by any route than as the crow flies seals whatever resolves from it, so the flood needn't
				// get there, nor anywhere beyond it, which is further still. Cheap cells can't be pruned, which spares the square root
				// for most of the field.
				if (next_distance > wall_cost + seal_prune_margin) {
					const real_t straight_x = neighbour_x + 0.5 - p_player_point.x;
					const real_t straight_y = neighbour_y + 0.5 - p_player_point.y;
					const int straight_distance = (int)(Math::sqrt(straight_x * straight_x + straight_y * straight_y) * 10.0);
					if (next_distance > straight_distance + wall_cost + seal_prune_margin) {
						continue;
					}
				}

				dist[neighbour_layered_index] = next_distance;
				parent[neighbour_layered_index] = layered_index;
				labels[neighbour_layered_index] = (uint8_t)label;
				material[neighbour_layered_index] = next_material;
				surcharge[neighbour_layered_index] = next_surcharge;
				buckets[next_distance % bucket_count].push_back(neighbour_layered_index);
				pending_count++;
			}
		}

		bucket.clear();
	}

	result["distances"] = distances;
	result["parents"] = parents;
	result["blocker_occlusions"] = blocker_occlusions;
	result["material_costs"] = material_costs;
	result["surcharges"] = surcharges;
	result["window_solid"] = window_solid;
	result["window_opaque"] = window_opaque;
	return result;
}

// A point in continuous window cell coordinates, kept in doubles for precision in the geometry.
struct ResolvePoint {
	double x = 0.0;
	double y = 0.0;
	ResolvePoint() {}
	ResolvePoint(double p_x, double p_y) :
			x(p_x), y(p_y) {}
	double distance_to(const ResolvePoint &p_other) const { return Math::sqrt((p_other.x - x) * (p_other.x - x) + (p_other.y - y) * (p_other.y - y)); }
};

static double _get_cost_double(const Dictionary &p_costs, const char *p_key, double p_default) {
	const Variant *value = p_costs.getptr(String(p_key));
	return value ? (double)(*value) : p_default;
}

// Returns the point moved to the nearest spot inside the window cell, kept p_inset in from its edges so it floors to that cell.
static ResolvePoint _clamp_into_cell(const ResolvePoint &p_point, int p_cell_x, int p_cell_y, double p_inset) {
	return ResolvePoint(CLAMP(p_point.x, p_cell_x + p_inset, p_cell_x + 1.0 - p_inset), CLAMP(p_point.y, p_cell_y + p_inset, p_cell_y + 1.0 - p_inset));
}

// Returns the window index of the point's cell if it's open air, otherwise of the open window cell within p_max_ring rings
// whose center is nearest the point (so a thing just inside a floor resolves above it, one inside a ceiling below it, one
// inside a wall on its open face), ties going to the cell nearest the player. -1 if there's no open air that close.
static int _snap_to_open_window_index(const uint8_t *p_window_solid, int p_side, const ResolvePoint &p_point, const ResolvePoint &p_toward_point, int p_max_ring) {
	const int cell_x = (int)Math::floor(p_point.x);
	const int cell_y = (int)Math::floor(p_point.y);
	const int index = cell_y * p_side + cell_x;
	if (!p_window_solid[index]) {
		return index;
	}

	for (int ring = 1; ring <= p_max_ring; ring++) {
		int best_index = -1;
		int64_t best_score = 0;
		for (int offset_y = -ring; offset_y <= ring; offset_y++) {
			const int candidate_y = cell_y + offset_y;
			if (candidate_y < 0 || candidate_y >= p_side) {
				continue;
			}

			for (int offset_x = -ring; offset_x <= ring; offset_x++) {
				const int candidate_x = cell_x + offset_x;
				if (candidate_x < 0 || candidate_x >= p_side || MAX(Math::abs(offset_x), Math::abs(offset_y)) != ring) {
					continue;
				}

				const int candidate_index = candidate_y * p_side + candidate_x;
				if (p_window_solid[candidate_index]) {
					continue;
				}

				// Nearest the point first, in hundredths of a cell, the player's distance only ever breaking ties.
				const ResolvePoint candidate_center(candidate_x + 0.5, candidate_y + 0.5);
				const int64_t score = (int64_t)(candidate_center.distance_to(p_point) * 100.0) * 1000 + (int64_t)(candidate_center.distance_to(p_toward_point) * 10.0);
				if (best_index == -1 || score < best_score) {
					best_index = candidate_index;
					best_score = score;
				}
			}
		}

		if (best_index != -1) {
			return best_index;
		}
	}

	// No open air within the rings, the cell is buried inside thick terrain.
	return -1;
}

// Returns whether no opaque window cell (terrain or a blocker) lies on the straight line between two window points. Walks
// the cells the line crosses one boundary at a time and, like the flood, refuses to slip through the joint between two
// diagonal solid cells. The start and end cells themselves are never minded, as either may legitimately be terrain.
static bool _has_line_of_sight_between_points(const uint8_t *p_window_opaque, int p_side, const ResolvePoint &p_from, const ResolvePoint &p_to) {
	int cell_x = (int)Math::floor(p_from.x);
	int cell_y = (int)Math::floor(p_from.y);
	const int end_x = (int)Math::floor(p_to.x);
	const int end_y = (int)Math::floor(p_to.y);
	const double delta_x = p_to.x - p_from.x;
	const double delta_y = p_to.y - p_from.y;
	const int step_x = delta_x > 0.0 ? 1 : -1;
	const int step_y = delta_y > 0.0 ? 1 : -1;
	// Distance along the line (as a fraction of it) to the next cell boundary on each axis, and between boundaries.
	double boundary_t_x = Math::INF;
	double boundary_t_y = Math::INF;
	double boundary_t_delta_x = Math::INF;
	double boundary_t_delta_y = Math::INF;
	if (delta_x != 0.0) {
		boundary_t_delta_x = Math::abs(1.0 / delta_x);
		boundary_t_x = ((step_x > 0 ? cell_x + 1 : cell_x) - p_from.x) / delta_x;
	}

	if (delta_y != 0.0) {
		boundary_t_delta_y = Math::abs(1.0 / delta_y);
		boundary_t_y = ((step_y > 0 ? cell_y + 1 : cell_y) - p_from.y) / delta_y;
	}

	int steps_left = Math::abs(end_x - cell_x) + Math::abs(end_y - cell_y);
	while ((cell_x != end_x || cell_y != end_y) && steps_left > 0) {
		steps_left--;
		if (Math::is_equal_approx(boundary_t_x, boundary_t_y)) {
			// Straight through a cell corner: both cells beside the joint must be open.
			if (p_window_opaque[cell_y * p_side + cell_x + step_x] || p_window_opaque[(cell_y + step_y) * p_side + cell_x]) {
				return false;
			}

			cell_x += step_x;
			cell_y += step_y;
			boundary_t_x += boundary_t_delta_x;
			boundary_t_y += boundary_t_delta_y;
		} else if (boundary_t_x < boundary_t_y) {
			cell_x += step_x;
			boundary_t_x += boundary_t_delta_x;
		} else {
			cell_y += step_y;
			boundary_t_y += boundary_t_delta_y;
		}

		if ((cell_x != end_x || cell_y != end_y) && p_window_opaque[cell_y * p_side + cell_x]) {
			return false;
		}
	}

	return true;
}

// Returns the index of the farthest of the points after p_from_index, up to p_last_index, that's in view from the point,
// assuming view is lost once along the path and not regained (true along a corridor), so it can gallop and bisect rather
// than test every point.
static int _farthest_visible_point(const LocalVector<ResolvePoint> &p_points, const uint8_t *p_window_opaque, int p_side, const ResolvePoint &p_from, int p_from_index, int p_last_index) {
	if (_has_line_of_sight_between_points(p_window_opaque, p_side, p_from, p_points[p_last_index])) {
		return p_last_index;
	}

	int low_index = p_from_index; // Highest index known to be in view (the point we're at counts).
	int high_index = p_last_index; // Lowest index known to be out of view.
	int gap = 1;
	while (low_index + gap < high_index) {
		const int probe_index = low_index + gap;
		if (_has_line_of_sight_between_points(p_window_opaque, p_side, p_from, p_points[probe_index])) {
			low_index = probe_index;
			gap *= 2;
		} else {
			high_index = probe_index;
			break;
		}
	}

	while (high_index - low_index > 1) {
		const int middle_index = (low_index + high_index) / 2;
		if (_has_line_of_sight_between_points(p_window_opaque, p_side, p_from, p_points[middle_index])) {
			low_index = middle_index;
		} else {
			high_index = middle_index;
		}
	}

	return low_index;
}

// Pulls the flood path from the start point to the emitter's end cell taut, like a string wrapped around the corridor's
// corners: from each point it skips to the farthest path cell center still in view, which becomes the next bend. The
// string is pinned to the center of every terrain cell the path goes through, so it's pulled taut between them and never
// cuts through terrain the path didn't: a sound through a floor is heard from where it comes through the floor.
// The path is followed through the parents from the end cell's layered index, whichever layers it runs through.
// Returns the window index of the first bend, or -1 when the end point is in view from the start, and the string's
// length in cells in r_length. Bends are cell centers, so they sit half a cell inside the real corners.
static int _pull_string(LocalVector<int32_t> &r_chain, LocalVector<ResolvePoint> &r_points, LocalVector<int32_t> &r_pins, const uint8_t *p_window_opaque, const uint8_t *p_window_solid, const int32_t *p_parents, int p_side, int p_cell_count, const ResolvePoint &p_start, const ResolvePoint &p_end, int p_end_layered_index, double &r_length) {
	// The flood path as points, origin first, ending at the emitter itself.
	r_chain.clear();
	for (int walk = p_end_layered_index; walk != AcousticFlood::UNREACHED; walk = p_parents[walk]) {
		r_chain.push_back(walk % p_cell_count);
	}

	r_chain.reverse();
	const int point_count = r_chain.size() + 1;
	r_points.resize(point_count);
	r_pins.clear();
	for (int chain_index = 0; chain_index < (int)r_chain.size(); chain_index++) {
		const int index = r_chain[chain_index];
		r_points[chain_index] = ResolvePoint(index % p_side + 0.5, index / p_side + 0.5);
		if (p_window_solid[index]) {
			r_pins.push_back(chain_index); // The path's first and last cells are open air, so a pin is never either end.
		}
	}

	r_points[point_count - 1] = p_end;

	r_length = 0.0;
	int first_bend_index = -1;
	ResolvePoint current_point = p_start;
	int current_index = 0; // Point index we're pulling from, the origin's cell to begin with (which holds the start point).
	const int last_index = point_count - 1;
	int pin_cursor = 0; // Index into the pins of the next pin ahead of current_index.
	while (current_index < last_index) {
		// Pull up to the next pin at most, as the string can't skip past the terrain it goes through. Its cell is the walk's
		// end cell, which the line of sight walk never minds, and the start cell of the next stretch, which it never minds either.
		while (pin_cursor < (int)r_pins.size() && r_pins[pin_cursor] <= current_index) {
			pin_cursor++;
		}

		const int stretch_end_index = pin_cursor < (int)r_pins.size() ? r_pins[pin_cursor] : last_index;
		int farthest_index = _farthest_visible_point(r_points, p_window_opaque, p_side, current_point, current_index, stretch_end_index);
		if (farthest_index <= current_index) {
			// Neighbouring path cells always see each other unless the next is a blocker, which is opaque but passable, so the
			// string bends at it: sound through a closed door is heard from the door. Or the start point just sits awkwardly in its cell.
			farthest_index = current_index + 1;
		}

		const ResolvePoint bend_point = r_points[farthest_index];
		r_length += current_point.distance_to(bend_point);
		if (first_bend_index == -1 && farthest_index < last_index) {
			first_bend_index = r_chain[farthest_index];
		}

		current_point = bend_point;
		current_index = farthest_index;
	}

	return first_bend_index;
}

// Resolves each emitter against one route's layer of a flooded field: its open cell, whether sound gets there that way,
// and if so the taut path's length and first bend (see _pull_string()), which is where its sound appears from, and the
// occlusion the material and blockers (closed doors) it goes through add up to. Points are in continuous window cell
// coordinates: a point's cell is its floor and cell centers lie at .5.
Dictionary AcousticFlood::resolve(const Dictionary &p_field, int p_layer_offset, int p_window_side, float p_cell_size, const PackedVector2Array &p_seed_window_cells, const Vector2i &p_window_origin_cell, const Vector2 &p_player_point, const PackedVector2Array &p_emitter_points, const Dictionary &p_costs) {
	Dictionary result;
	const int side = p_window_side;
	const int cell_count = side * side;
	ERR_FAIL_COND_V_MSG(side <= 0, result, "The window side must be positive.");
	ERR_FAIL_COND_V_MSG(p_seed_window_cells.is_empty(), result, "At least one seed is needed.");
	const PackedByteArray window_solid_array = p_field.get("window_solid", PackedByteArray());
	const PackedByteArray window_opaque_array = p_field.get("window_opaque", PackedByteArray());
	const PackedInt32Array distances_array = p_field.get("distances", PackedInt32Array());
	const PackedInt32Array parents_array = p_field.get("parents", PackedInt32Array());
	const PackedByteArray blocker_occlusions_array = p_field.get("blocker_occlusions", PackedByteArray());
	const PackedInt32Array material_costs_array = p_field.get("material_costs", PackedInt32Array());
	const PackedInt32Array surcharges_array = p_field.get("surcharges", PackedInt32Array());
	const int layered_count = cell_count * ROUTE_COUNT;
	ERR_FAIL_COND_V_MSG(window_solid_array.size() != cell_count || window_opaque_array.size() != cell_count, result, "The field's window masks don't match the window side.");
	ERR_FAIL_COND_V_MSG(distances_array.size() != layered_count || parents_array.size() != layered_count || blocker_occlusions_array.size() != layered_count || material_costs_array.size() != layered_count || surcharges_array.size() != layered_count, result, "The field's arrays don't match the window side, or aren't a flood() result.");
	ERR_FAIL_COND_V_MSG(p_layer_offset < 0 || p_layer_offset + cell_count > layered_count, result, "The layer offset lies outside the field.");
	const uint8_t *window_solid = window_solid_array.ptr();
	const uint8_t *window_opaque = window_opaque_array.ptr();
	const int32_t *distances = distances_array.ptr();
	const int32_t *parents = parents_array.ptr();
	const uint8_t *blocker_occlusions = blocker_occlusions_array.ptr();
	const int32_t *material_costs = material_costs_array.ptr();
	const int32_t *surcharges = surcharges_array.ptr();

	const double cell_size = p_cell_size;
	const double max_attenuation_distance = _get_cost_double(p_costs, "max_attenuation_distance", 1700.0); // Px, emitters further as the crow flies are out of range.
	const double wall_cost = _get_cost(p_costs, "wall_cost", 400); // Tenths of a cell a wall is worth: a route longer than the straight line plus this seals.
	const double material_occlusion_full_cost = _get_cost_double(p_costs, "material_occlusion_full_cost", 200.0); // Tenths of a cell of material cost at which material alone occludes fully.
	const int snap_max_ring = _get_cost(p_costs, "snap_max_ring", 2); // Rings of cells around a solid cell searched for open air to resolve from.
	const double cell_inset = _get_cost_double(p_costs, "cell_inset", 0.01); // Cells, how far inside its cell a string's end is kept so it never sits on the edge.

	const int emitter_count = p_emitter_points.size();
	PackedByteArray emitter_states;
	emitter_states.resize(emitter_count);
	uint8_t *states = emitter_states.ptrw();
	PackedVector2Array emitter_cells;
	emitter_cells.resize(emitter_count);
	Vector2 *cells = emitter_cells.ptrw();
	PackedFloat32Array emitter_costs;
	emitter_costs.resize(emitter_count);
	float *costs = emitter_costs.ptrw();
	PackedFloat32Array emitter_path_distances;
	emitter_path_distances.resize(emitter_count);
	float *path_distances = emitter_path_distances.ptrw();
	PackedFloat32Array emitter_straight_distances;
	emitter_straight_distances.resize(emitter_count);
	float *straight_distances = emitter_straight_distances.ptrw();
	PackedVector2Array emitter_opening_cells;
	emitter_opening_cells.resize(emitter_count);
	Vector2 *opening_cells = emitter_opening_cells.ptrw();
	PackedFloat32Array emitter_occlusions;
	emitter_occlusions.resize(emitter_count);
	float *occlusions = emitter_occlusions.ptrw();

	// Strings start where the player stands, unless that's inside terrain (just inside a floor or a wall): then at
	// the spot nearest them of the exit cell each emitter's path came out of (its seed), the floor's face right above or the
	// wall's face on that emitter's side, plus the short hop to them. Standing in a door's cells is fine, the line of sight
	// walk never minds its own start cell. A player outside the window starts from the first seed's center.
	const ResolvePoint player_point(p_player_point.x, p_player_point.y);
	const int player_cell_x = (int)Math::floor(player_point.x);
	const int player_cell_y = (int)Math::floor(player_point.y);
	const bool player_in_window = player_cell_x >= 0 && player_cell_y >= 0 && player_cell_x < side && player_cell_y < side;
	const bool player_in_terrain = player_in_window && window_solid[player_cell_y * side + player_cell_x] != 0;
	const ResolvePoint first_seed_center(p_seed_window_cells[0].x + 0.5, p_seed_window_cells[0].y + 0.5);
	const ResolvePoint straight_from_point = player_in_window ? player_point : first_seed_center;

	LocalVector<int32_t> chain;
	LocalVector<ResolvePoint> points;
	LocalVector<int32_t> pins;
	const Vector2 *emitter_points = p_emitter_points.ptr();
	for (int emitter_index = 0; emitter_index < emitter_count; emitter_index++) {
		const ResolvePoint emitter_point(emitter_points[emitter_index].x, emitter_points[emitter_index].y);
		const int window_x = (int)Math::floor(emitter_point.x);
		const int window_y = (int)Math::floor(emitter_point.y);
		const Vector2 raw_cell(p_window_origin_cell.x + window_x, p_window_origin_cell.y + window_y);
		const double straight_distance = MAX(straight_from_point.distance_to(emitter_point) * cell_size, cell_size); // Floored at one cell so sharing the player's spot isn't a division by zero.
		states[emitter_index] = EMITTER_OUT_OF_RANGE;
		cells[emitter_index] = raw_cell;
		costs[emitter_index] = Math::INF;
		path_distances[emitter_index] = Math::INF;
		straight_distances[emitter_index] = (float)straight_distance;
		opening_cells[emitter_index] = raw_cell;
		occlusions[emitter_index] = 0.0f;
		if (straight_distance > max_attenuation_distance || window_x < 0 || window_y < 0 || window_x >= side || window_y >= side) {
			continue;
		}

		// Resolve from the open air next to the emitter when it sits in a solid cell, like the player does when standing on a floor.
		const int index = _snap_to_open_window_index(window_solid, side, emitter_point, straight_from_point, snap_max_ring);
		if (index == -1) {
			states[emitter_index] = EMITTER_SEALED;
			continue;
		}

		// A sound from inside a blocker's cells is the door's own (or something's in the doorway): it passes through those cells
		// freely, to either side, so for it only terrain stands in the way of the string.
		const bool emitter_in_blocker = window_opaque[index] != 0;
		const uint8_t *line_of_sight_mask = emitter_in_blocker ? window_solid : window_opaque;

		const int snapped_x = index % side;
		const int snapped_y = index / side;
		cells[emitter_index] = Vector2(p_window_origin_cell.x + snapped_x, p_window_origin_cell.y + snapped_y);

		// No route this way, or none better than a wall: sealed either way, so nothing jumps at the flood's edge. Routes are
		// compared with their material and blocker costs in, the same way the flood chose between them, against the straight
		// line plus the wall's cost. The flood's distances run a little longer than the taut path, which is close enough here.
		const int layered_index = index + p_layer_offset;
		const double wall_cost_distance = straight_distance + wall_cost * 0.1 * cell_size;
		if (distances[layered_index] == UNREACHED || distances[layered_index] * 0.1 * cell_size > wall_cost_distance) {
			states[emitter_index] = EMITTER_SEALED;
			continue;
		}

		// What the sound went through to get here muffles it: material at its cost, doors at their own, together at most fully.
		const double blocker_occlusion = (blocker_occlusions[layered_index] & BLOCKER_OCCLUSION_MASK) / (double)BLOCKER_OCCLUSION_MASK;
		const double occlusion = MIN(blocker_occlusion + material_costs[layered_index] / material_occlusion_full_cost, 1.0);

		// Likewise the string ends at the emitter itself, unless it sits inside terrain: then at the spot of its open cell nearest
		// it, the floor's face above it, plus the short hop into the floor.
		const bool snapped_away = snapped_x != window_x || snapped_y != window_y;
		const ResolvePoint end_point = snapped_away ? _clamp_into_cell(emitter_point, snapped_x, snapped_y, cell_inset) : emitter_point;

		// This emitter's exit: the root of its path, which for a through cell lies back in the around layer.
		int root_layered_index = layered_index;
		while (parents[root_layered_index] != UNREACHED) {
			root_layered_index = parents[root_layered_index];
		}

		const int root_index = root_layered_index % cell_count;
		const int root_x = root_index % side;
		const int root_y = root_index / side;
		ResolvePoint start_point(root_x + 0.5, root_y + 0.5);
		if (player_in_window) {
			start_point = player_in_terrain ? _clamp_into_cell(player_point, root_x, root_y, cell_inset) : player_point;
		}

		const double start_hop = start_point.distance_to(straight_from_point);
		double string_length = 0.0;
		const int first_bend_index = _pull_string(chain, points, pins, line_of_sight_mask, window_solid, parents, side, cell_count, start_point, end_point, layered_index, string_length);
		states[emitter_index] = EMITTER_REACHED;
		occlusions[emitter_index] = (float)occlusion;
		const double path_distance = (start_hop + string_length + end_point.distance_to(emitter_point)) * cell_size;
		path_distances[emitter_index] = (float)path_distance;
		costs[emitter_index] = (float)(path_distance + surcharges[layered_index] * 0.1 * cell_size); // The taut way there plus what's charged on it.
		if (first_bend_index != -1) {
			opening_cells[emitter_index] = Vector2(p_window_origin_cell.x + first_bend_index % side, p_window_origin_cell.y + first_bend_index / side);
		}
	}

	result["emitter_states"] = emitter_states;
	result["emitter_cells"] = emitter_cells;
	result["emitter_costs"] = emitter_costs;
	result["emitter_path_distances"] = emitter_path_distances;
	result["emitter_straight_distances"] = emitter_straight_distances;
	result["emitter_opening_cells"] = emitter_opening_cells;
	result["emitter_occlusions"] = emitter_occlusions;
	return result;
}

void AcousticFlood::_bind_methods() {
	ClassDB::bind_static_method("AcousticFlood", D_METHOD("flood", "solid_cells", "cell_weights", "row_length", "row_amount", "window_origin_cell", "window_side", "seed_window_cells", "player_point", "blocked_cells", "blocked_costs", "blocked_occlusions", "costs"), &AcousticFlood::flood);
	ClassDB::bind_static_method("AcousticFlood", D_METHOD("resolve", "field", "layer_offset", "window_side", "cell_size", "seed_window_cells", "window_origin_cell", "player_point", "emitter_points", "costs"), &AcousticFlood::resolve);

	BIND_CONSTANT(UNREACHED);
	BIND_CONSTANT(NOT_BLOCKED);
	BIND_CONSTANT(BLOCKER_PENDING_BIT);
	BIND_CONSTANT(BLOCKER_OCCLUSION_MASK);
	BIND_CONSTANT(WEIGHT_FULL);
	BIND_CONSTANT(ROUTE_AROUND);
	BIND_CONSTANT(ROUTE_THROUGH);
	BIND_CONSTANT(ROUTE_COUNT);
	BIND_CONSTANT(EMITTER_OUT_OF_RANGE);
	BIND_CONSTANT(EMITTER_SEALED);
	BIND_CONSTANT(EMITTER_REACHED);
}

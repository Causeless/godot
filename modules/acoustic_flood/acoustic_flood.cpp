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

void AcousticFlood::_bind_methods() {
	ClassDB::bind_static_method("AcousticFlood", D_METHOD("flood", "solid_cells", "cell_weights", "row_length", "row_amount", "window_origin_cell", "window_side", "seed_window_cells", "player_point", "blocked_cells", "blocked_costs", "blocked_occlusions", "costs"), &AcousticFlood::flood);

	BIND_CONSTANT(UNREACHED);
	BIND_CONSTANT(NOT_BLOCKED);
	BIND_CONSTANT(BLOCKER_PENDING_BIT);
	BIND_CONSTANT(BLOCKER_OCCLUSION_MASK);
	BIND_CONSTANT(WEIGHT_FULL);
	BIND_CONSTANT(ROUTE_AROUND);
	BIND_CONSTANT(ROUTE_THROUGH);
	BIND_CONSTANT(ROUTE_COUNT);
}

/**************************************************************************/
/*  acoustic_flood.h                                                      */
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

#include "core/object/class_db.h"
#include "core/object/object.h"
#include "core/variant/dictionary.h"

// A distance field flood for sound propagation: a two-layer Dial's algorithm flood over a window of a baked grid of
// terrain cells, where terrain is passable at a cost and closed doors (blockers) at theirs. The caller keeps the grid,
// the window and everything that reads the result; this class only floods.
class AcousticFlood : public Object {
	GDCLASS(AcousticFlood, Object);

protected:
	static void _bind_methods();

public:
	enum {
		UNREACHED = -1, // Distance and parent of cells the flood didn't reach.
		NOT_BLOCKED = -1, // Blocker cost of cells no blocker covers.
		BLOCKER_PENDING_BIT = 128, // Set in a cell's blocker occlusion byte while its path is inside a blocker it entered from open air.
		BLOCKER_OCCLUSION_MASK = 127, // The occlusion itself in that byte.
		WEIGHT_FULL = 255, // Weight of a cell full of the hardest material, which costs the full material cost to enter.
		ROUTE_AROUND = 0, // The flood's first layer: paths that went through nothing yet.
		ROUTE_THROUGH = 1, // Its second layer: paths that went through terrain or a blocker.
		ROUTE_COUNT = 2,
	};

	static Dictionary flood(const PackedByteArray &p_solid_cells, const PackedByteArray &p_cell_weights, int p_row_length, int p_row_amount, const Vector2i &p_window_origin_cell, int p_window_side, const PackedVector2Array &p_seed_window_cells, const Vector2 &p_player_point, const PackedVector2Array &p_blocked_cells, const PackedInt32Array &p_blocked_costs, const PackedByteArray &p_blocked_occlusions, const Dictionary &p_costs);
};

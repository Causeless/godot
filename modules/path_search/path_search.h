/**************************************************************************/
/*  path_search.h                                                         */
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

// A weighted A* search over a graph of connections between nodes, where what a connection costs to reach depends on
// where the connection before it landed, as for agents that walk between the points where they jump, climb or pass
// through something. The caller keeps the graph as packed arrays indexed by connection and node ids and hands them
// over per call, so a search never shares a buffer with whoever patches the graph; this class only searches.
class PathSearch : public Object {
	GDCLASS(PathSearch, Object);

protected:
	static void _bind_methods();

public:
	enum {
		CONNECTION_FLAG_ACTIVE = 1, // The connection can be taken at all. Connections without it are skipped outright.
		CONNECTION_FLAG_CLOSED = 2, // The connection is shut, like a closed door. Anyone may open it unless it's also locked.
		CONNECTION_FLAG_LOCKED = 4, // While closed, only an actor whose key matches the connection's may open it.
		KEY_NONE = 0, // The key that opens nothing: a locked connection holding it admits nobody.
		NO_STOP = -1, // Value of stop_at_step that lets a search run to its end.
	};

	static Dictionary find_path(const Dictionary &p_graph, int p_start_node, int p_goal_node, const Vector2 &p_actor_position, int p_actor_level, int p_actor_key, const Vector2 &p_goal_position, const Dictionary &p_costs, int p_max_iterations, int p_stop_at_step);
};

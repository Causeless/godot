/**************************************************************************/
/*  tile_set_scenes_collection_source_editor.cpp                          */
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

#include "tile_set_scenes_collection_source_editor.h"

#include "tile_set_editor.h"

#include "core/input/input_event.h"
#include "core/io/resource_loader.h"
#include "core/object/callable_mp.h"
#include "core/object/class_db.h"
#include "editor/editor_node.h"
#include "editor/editor_undo_redo_manager.h"
#include "editor/file_system/editor_file_system.h"
#include "editor/gui/editor_file_dialog.h"
#include "editor/inspector/editor_resource_preview.h"
#include "editor/settings/editor_settings.h"
#include "editor/themes/editor_scale.h"
#include "scene/gui/button.h"
#include "scene/gui/item_list.h"
#include "scene/gui/label.h"
#include "scene/gui/slider.h"
#include "scene/gui/split_container.h"

void TileSetScenesCollectionSourceEditor::TileSetScenesCollectionProxyObject::set_id(int p_id) {
	ERR_FAIL_COND(p_id < 0);
	if (source_id == p_id) {
		return;
	}
	ERR_FAIL_COND_MSG(tile_set->has_source(p_id), vformat("Cannot change TileSet Scenes Collection source ID. Another TileSet source exists with id %d.", p_id));

	int previous_source = source_id;
	source_id = p_id; // source_id must be updated before, because it's used by the source list update.
	tile_set->set_source_id(previous_source, p_id);
	emit_signal(CoreStringName(changed), "id");
}

int TileSetScenesCollectionSourceEditor::TileSetScenesCollectionProxyObject::get_id() {
	return source_id;
}

bool TileSetScenesCollectionSourceEditor::TileSetScenesCollectionProxyObject::_set(const StringName &p_name, const Variant &p_value) {
	String name = p_name;
	if (name == "name") {
		// Use the resource_name property to store the source's name.
		name = "resource_name";
	}
	bool valid = false;
	tile_set_scenes_collection_source->set(name, p_value, &valid);
	if (valid) {
		emit_signal(CoreStringName(changed), String(name).utf8().get_data());
	}
	return valid;
}

bool TileSetScenesCollectionSourceEditor::TileSetScenesCollectionProxyObject::_get(const StringName &p_name, Variant &r_ret) const {
	if (tile_set_scenes_collection_source.is_null()) {
		return false;
	}
	String name = p_name;
	if (name == "name") {
		// Use the resource_name property to store the source's name.
		name = "resource_name";
	}
	bool valid = false;
	r_ret = tile_set_scenes_collection_source->get(name, &valid);
	return valid;
}

void TileSetScenesCollectionSourceEditor::TileSetScenesCollectionProxyObject::_get_property_list(List<PropertyInfo> *p_list) const {
	p_list->push_back(PropertyInfo(Variant::STRING, "name", PROPERTY_HINT_NONE, ""));
}

void TileSetScenesCollectionSourceEditor::TileSetScenesCollectionProxyObject::_bind_methods() {
	// -- Shape and layout --
	ClassDB::bind_method(D_METHOD("set_id", "id"), &TileSetScenesCollectionSourceEditor::TileSetScenesCollectionProxyObject::set_id);
	ClassDB::bind_method(D_METHOD("get_id"), &TileSetScenesCollectionSourceEditor::TileSetScenesCollectionProxyObject::get_id);

	ADD_PROPERTY(PropertyInfo(Variant::INT, "id"), "set_id", "get_id");

	ADD_SIGNAL(MethodInfo("changed", PropertyInfo(Variant::STRING, "what")));
}

void TileSetScenesCollectionSourceEditor::TileSetScenesCollectionProxyObject::edit(Ref<TileSet> p_tile_set, TileSetScenesCollectionSource *p_tile_set_scenes_collection_source, int p_source_id) {
	ERR_FAIL_COND(p_tile_set.is_null());
	ERR_FAIL_NULL(p_tile_set_scenes_collection_source);
	ERR_FAIL_COND(p_source_id < 0);
	ERR_FAIL_COND(p_tile_set->get_source(p_source_id) != p_tile_set_scenes_collection_source);

	if (tile_set == p_tile_set && tile_set_scenes_collection_source == p_tile_set_scenes_collection_source && source_id == p_source_id) {
		return;
	}

	// Disconnect to changes.
	if (tile_set_scenes_collection_source.is_valid()) {
		tile_set_scenes_collection_source->disconnect(CoreStringName(property_list_changed), callable_mp((Object *)this, &Object::notify_property_list_changed));
	}

	tile_set = p_tile_set;
	tile_set_scenes_collection_source = p_tile_set_scenes_collection_source;
	source_id = p_source_id;

	// Connect to changes.
	if (tile_set_scenes_collection_source.is_valid()) {
		if (!tile_set_scenes_collection_source->is_connected(CoreStringName(property_list_changed), callable_mp((Object *)this, &Object::notify_property_list_changed))) {
			tile_set_scenes_collection_source->connect(CoreStringName(property_list_changed), callable_mp((Object *)this, &Object::notify_property_list_changed));
		}
	}

	notify_property_list_changed();
}

// -- Proxy object used by the tile inspector --
bool TileSetScenesCollectionSourceEditor::SceneTileProxyObject::_set(const StringName &p_name, const Variant &p_value) {
	if (tile_set_scenes_collection_source.is_null()) {
		return false;
	}

	if (p_name == "id") {
		int as_int = int(p_value);
		ERR_FAIL_COND_V(as_int < 0, false);
		ERR_FAIL_COND_V(tile_set_scenes_collection_source->has_scene_tile_id(as_int), false);
		tile_set_scenes_collection_source->set_scene_tile_id(scene_id, as_int);
		scene_id = as_int;
		emit_signal(CoreStringName(changed), "id");
		for (int i = 0; i < tile_set_scenes_collection_source_editor->scene_tiles_list->get_item_count(); i++) {
			if (int(tile_set_scenes_collection_source_editor->scene_tiles_list->get_item_metadata(i)) == scene_id) {
				tile_set_scenes_collection_source_editor->scene_tiles_list->select(i);
				break;
			}
		}
		return true;
	} else if (p_name == "scene") {
		tile_set_scenes_collection_source->set_scene_tile_scene(scene_id, p_value);
		emit_signal(CoreStringName(changed), "scene");
		return true;
	} else if (p_name == "display_placeholder") {
		tile_set_scenes_collection_source->set_scene_tile_display_placeholder(scene_id, p_value);
		emit_signal(CoreStringName(changed), "display_placeholder");
		return true;
	}

	return false;
}

bool TileSetScenesCollectionSourceEditor::SceneTileProxyObject::_get(const StringName &p_name, Variant &r_ret) const {
	if (tile_set_scenes_collection_source.is_null()) {
		return false;
	}

	if (p_name == "id") {
		r_ret = scene_id;
		return true;
	} else if (p_name == "scene") {
		r_ret = tile_set_scenes_collection_source->get_scene_tile_scene(scene_id);
		return true;
	} else if (p_name == "display_placeholder") {
		r_ret = tile_set_scenes_collection_source->get_scene_tile_display_placeholder(scene_id);
		return true;
	}

	return false;
}

void TileSetScenesCollectionSourceEditor::SceneTileProxyObject::_get_property_list(List<PropertyInfo> *p_list) const {
	if (tile_set_scenes_collection_source.is_null()) {
		return;
	}

	p_list->push_back(PropertyInfo(Variant::INT, "id", PROPERTY_HINT_NONE, ""));
	p_list->push_back(PropertyInfo(Variant::OBJECT, "scene", PROPERTY_HINT_RESOURCE_TYPE, PackedScene::get_class_static()));
	p_list->push_back(PropertyInfo(Variant::BOOL, "display_placeholder", PROPERTY_HINT_NONE, ""));
}

void TileSetScenesCollectionSourceEditor::SceneTileProxyObject::edit(TileSetScenesCollectionSource *p_tile_set_scenes_collection_source, int p_scene_id) {
	ERR_FAIL_NULL(p_tile_set_scenes_collection_source);
	ERR_FAIL_COND(!p_tile_set_scenes_collection_source->has_scene_tile_id(p_scene_id));

	if (tile_set_scenes_collection_source == p_tile_set_scenes_collection_source && scene_id == p_scene_id) {
		return;
	}

	tile_set_scenes_collection_source = p_tile_set_scenes_collection_source;
	scene_id = p_scene_id;

	notify_property_list_changed();
}

void TileSetScenesCollectionSourceEditor::SceneTileProxyObject::_bind_methods() {
	ADD_SIGNAL(MethodInfo("changed", PropertyInfo(Variant::STRING, "what")));
}

void TileSetScenesCollectionSourceEditor::_scenes_collection_source_proxy_object_changed(const String &p_what, const Ref<TileSetScenesCollectionProxyObject> &p_object) {
	if (p_what == "id") {
		emit_signal(SNAME("source_id_changed"), p_object->get_id());
	}
}

void TileSetScenesCollectionSourceEditor::_tile_set_scenes_collection_source_changed() {
	tile_set_scenes_collection_source_changed_needs_update = true;
}

void TileSetScenesCollectionSourceEditor::_scene_thumbnail_done(const String &p_path, const Ref<Texture2D> &p_preview, const Ref<Texture2D> &p_small_preview, int p_idx) {
	if (p_idx >= 0 && p_idx < scene_tiles_list->get_item_count()) {
		scene_tiles_list->set_item_icon(p_idx, p_preview);
	}
}

void TileSetScenesCollectionSourceEditor::_scenes_list_item_activated(int p_index) {
	Ref<PackedScene> packed_scene = tile_set_scenes_collection_source->get_scene_tile_scene(scene_tiles_list->get_item_metadata(p_index));
	if (packed_scene.is_valid()) {
		EditorNode::get_singleton()->open_scene(packed_scene->get_path());
	}
}

void TileSetScenesCollectionSourceEditor::_scenes_list_gui_input(const Ref<InputEvent> &p_event) {
	// Ctrl + mouse wheel resizes the thumbnails in grid view.
	const Ref<InputEventMouseButton> mb = p_event;
	if (mb.is_valid() && mb->is_pressed() && mb->is_command_or_control_pressed() && scene_tiles_grid_view) {
		if (mb->get_button_index() == MouseButton::WHEEL_UP) {
			scene_tiles_size_slider->set_value(scene_tiles_thumbnail_size + 16);
			scene_tiles_list->accept_event();
		} else if (mb->get_button_index() == MouseButton::WHEEL_DOWN) {
			scene_tiles_size_slider->set_value(scene_tiles_thumbnail_size - 16);
			scene_tiles_list->accept_event();
		}
	}
}

void TileSetScenesCollectionSourceEditor::_scenes_list_sort_toggled(bool p_pressed) {
	scene_tiles_sort_by_name = p_pressed;
	_save_scenes_list_options();
	_update_scenes_list();
}

void TileSetScenesCollectionSourceEditor::_scenes_list_grid_toggled(bool p_pressed) {
	scene_tiles_grid_view = p_pressed;
	scene_tiles_size_slider->set_visible(scene_tiles_grid_view);
	_save_scenes_list_options();
	_update_scenes_list();
}

void TileSetScenesCollectionSourceEditor::_scenes_list_thumbnail_size_changed(float p_value) {
	scene_tiles_thumbnail_size = p_value;
	_save_scenes_list_options();
	_update_scenes_list();
}

void TileSetScenesCollectionSourceEditor::_save_scenes_list_options() {
	EditorSettings::get_singleton()->set_project_metadata("editor_metadata", "scene_tiles_grid_view", scene_tiles_grid_view);
	EditorSettings::get_singleton()->set_project_metadata("editor_metadata", "scene_tiles_sort_by_name", scene_tiles_sort_by_name);
	EditorSettings::get_singleton()->set_project_metadata("editor_metadata", "scene_tiles_thumbnail_size", scene_tiles_thumbnail_size);
}

void TileSetScenesCollectionSourceEditor::_source_add_pressed() {
	if (!scene_select_dialog) {
		scene_select_dialog = memnew(EditorFileDialog);
		add_child(scene_select_dialog);
		scene_select_dialog->set_file_mode(EditorFileDialog::FILE_MODE_OPEN_FILE);
		scene_select_dialog->connect("file_selected", callable_mp(this, &TileSetScenesCollectionSourceEditor::_scene_file_selected));

		for (const String &E : Vector<String>{ "tscn", "scn" }) {
			scene_select_dialog->add_filter("*." + E, E.to_upper());
		}
	}
	scene_select_dialog->popup_file_dialog();
}

void TileSetScenesCollectionSourceEditor::_scene_file_selected(const String &p_path) {
	Ref<PackedScene> scene = ResourceLoader::load(p_path);

	int scene_id = tile_set_scenes_collection_source->get_next_scene_tile_id();
	EditorUndoRedoManager *undo_redo = EditorUndoRedoManager::get_singleton();
	undo_redo->create_action(TTR("Add a Scene Tile"));
	undo_redo->add_do_method(*tile_set_scenes_collection_source, "create_scene_tile", scene, scene_id);
	undo_redo->add_do_method(this, "_update_all");
	undo_redo->add_undo_method(*tile_set_scenes_collection_source, "remove_scene_tile", scene_id);
	undo_redo->add_undo_method(this, "_update_all");
	undo_redo->commit_action();
}

void TileSetScenesCollectionSourceEditor::_source_delete_pressed() {
	Vector<int> selected_indices = scene_tiles_list->get_selected_items();
	ERR_FAIL_COND(selected_indices.is_empty());
	int scene_id = scene_tiles_list->get_item_metadata(selected_indices[0]);

	EditorUndoRedoManager *undo_redo = EditorUndoRedoManager::get_singleton();
	undo_redo->create_action(TTR("Remove a Scene Tile"));
	undo_redo->add_do_method(*tile_set_scenes_collection_source, "remove_scene_tile", scene_id);
	undo_redo->add_do_method(this, "_update_all");
	undo_redo->add_undo_method(*tile_set_scenes_collection_source, "create_scene_tile", tile_set_scenes_collection_source->get_scene_tile_scene(scene_id), scene_id);
	undo_redo->add_undo_method(this, "_update_all");
	undo_redo->commit_action();
}

void TileSetScenesCollectionSourceEditor::_update_source_inspector() {
	// Update the proxy object.
	scenes_collection_source_proxy_object.instantiate();
	scenes_collection_source_proxy_object->connect(CoreStringName(changed), callable_mp(this, &TileSetScenesCollectionSourceEditor::_scenes_collection_source_proxy_object_changed).bind(scenes_collection_source_proxy_object));
	scenes_collection_source_proxy_object->edit(tile_set, *tile_set_scenes_collection_source, tile_set_source_id);
	scenes_collection_source_inspector->edit(scenes_collection_source_proxy_object.ptr());
}

void TileSetScenesCollectionSourceEditor::_update_tile_inspector() {
	Vector<int> selected_indices = scene_tiles_list->get_selected_items();
	bool has_atlas_tile_selected = (selected_indices.size() > 0);

	// Update the proxy object.
	if (has_atlas_tile_selected) {
		tile_proxy_object.instantiate(this);
		tile_proxy_object->connect(CoreStringName(changed), callable_mp(this, &TileSetScenesCollectionSourceEditor::_update_scenes_list).unbind(1));
		tile_proxy_object->connect(CoreStringName(changed), callable_mp(this, &TileSetScenesCollectionSourceEditor::_update_action_buttons).unbind(1));
		int scene_id = scene_tiles_list->get_item_metadata(selected_indices[0]);
		tile_proxy_object->edit(*tile_set_scenes_collection_source, scene_id);
		tile_inspector->edit(tile_proxy_object.ptr());
	}

	// Update visibility.
	tile_inspector_label->set_visible(has_atlas_tile_selected);
	tile_inspector->set_visible(has_atlas_tile_selected);
}

void TileSetScenesCollectionSourceEditor::_update_action_buttons() {
	Vector<int> selected_indices = scene_tiles_list->get_selected_items();
	scene_tile_delete_button->set_disabled(selected_indices.is_empty() || read_only);
}

void TileSetScenesCollectionSourceEditor::_update_all() {
	_update_scenes_list();
	_update_action_buttons();
	_update_tile_inspector();
}

void TileSetScenesCollectionSourceEditor::_update_scenes_list() {
	if (tile_set_scenes_collection_source.is_null()) {
		return;
	}

	// Get the previously selected id.
	Vector<int> selected_indices = scene_tiles_list->get_selected_items();
	int old_selected_scene_id = (selected_indices.size() > 0) ? int(scene_tiles_list->get_item_metadata(selected_indices[0])) : -1;

	// Clear the list.
	scene_tiles_list->clear();

	// Layout: a grid of thumbnails with the name underneath, or one row per tile.
	const int thumbnail_size = (scene_tiles_grid_view ? scene_tiles_thumbnail_size : int(EDITOR_GET("filesystem/file_dialog/thumbnail_size"))) * EDSCALE;
	if (scene_tiles_grid_view) {
		scene_tiles_list->set_icon_mode(ItemList::ICON_MODE_TOP);
		scene_tiles_list->set_max_columns(0);
		scene_tiles_list->set_fixed_column_width(thumbnail_size * 3 / 2);
		scene_tiles_list->set_max_text_lines(2);
	} else {
		scene_tiles_list->set_icon_mode(ItemList::ICON_MODE_LEFT);
		scene_tiles_list->set_max_columns(1);
		scene_tiles_list->set_fixed_column_width(0);
		scene_tiles_list->set_max_text_lines(1);
	}
	scene_tiles_list->set_fixed_icon_size(Vector2(thumbnail_size, thumbnail_size));

	// Collect the tiles, in ID order or sorted by name.
	struct SceneTileEntry {
		int scene_id = -1;
		Ref<PackedScene> scene;
		String name;
	};
	struct SceneTileNameComparator {
		bool operator()(const SceneTileEntry &p_a, const SceneTileEntry &p_b) const {
			const int cmp = p_a.name.naturalnocasecmp_to(p_b.name);
			return cmp != 0 ? cmp < 0 : p_a.scene_id < p_b.scene_id;
		}
	};
	LocalVector<SceneTileEntry> entries;
	for (int i = 0; i < tile_set_scenes_collection_source->get_scene_tiles_count(); i++) {
		SceneTileEntry entry;
		entry.scene_id = tile_set_scenes_collection_source->get_scene_tile_id(i);
		entry.scene = tile_set_scenes_collection_source->get_scene_tile_scene(entry.scene_id);
		if (entry.scene.is_valid()) {
			entry.name = entry.scene->get_path().get_file().get_basename();
		}
		entries.push_back(entry);
	}
	if (scene_tiles_sort_by_name) {
		entries.sort_custom<SceneTileNameComparator>();
	}

	// Rebuild the list.
	int to_reselect = -1;
	for (const SceneTileEntry &entry : entries) {
		int item_index = 0;
		if (entry.scene.is_valid()) {
			const String details = vformat("%s (path:%s id:%d)", entry.name, entry.scene->get_path(), entry.scene_id);
			if (scene_tiles_grid_view) {
				item_index = scene_tiles_list->add_item(entry.name);
				scene_tiles_list->set_item_tooltip(item_index, details);
			} else {
				item_index = scene_tiles_list->add_item(details);
			}
			EditorResourcePreview::get_singleton()->queue_edited_resource_preview(entry.scene, callable_mp(this, &TileSetScenesCollectionSourceEditor::_scene_thumbnail_done).bind(item_index));
		} else {
			item_index = scene_tiles_list->add_item(TTR("Tile with Invalid Scene"), get_editor_theme_icon(SNAME("PackedScene")));
		}
		scene_tiles_list->set_item_metadata(item_index, entry.scene_id);

		if (old_selected_scene_id >= 0 && entry.scene_id == old_selected_scene_id) {
			to_reselect = item_index;
		}
	}
	if (scene_tiles_list->get_item_count() == 0) {
		scene_tiles_list->add_item(TTR("Drag and drop scenes here or use the Add button."));
		scene_tiles_list->set_item_disabled(-1, true);
	}

	// Reselect if needed.
	if (to_reselect >= 0) {
		scene_tiles_list->select(to_reselect);
	}
}

void TileSetScenesCollectionSourceEditor::_notification(int p_what) {
	switch (p_what) {
		case NOTIFICATION_READY: {
			scenes_collection_source_inspector->add_custom_property_description("TileSetScenesCollectionProxyObject", "id", TTRC("The tile's unique identifier within this TileSet. Each tile stores its source ID, so changing one may make tiles invalid."));
			scenes_collection_source_inspector->add_custom_property_description("TileSetScenesCollectionProxyObject", "name", TTRC("The human-readable name for the scene collection. Use a descriptive name here for organizational purposes (such as \"obstacles\", \"decoration\", etc.)."));

			tile_inspector->add_custom_property_description("SceneTileProxyObject", "id", TTRC("ID of the scene tile in the collection. Each painted tile has associated ID, so changing this property may cause your TileMaps to not display properly."));
			tile_inspector->add_custom_property_description("SceneTileProxyObject", "scene", TTRC("Absolute path to the scene associated with this tile."));
			tile_inspector->add_custom_property_description("SceneTileProxyObject", "display_placeholder", TTRC("If [code]true[/code], a placeholder marker will be displayed on top of the scene's preview. The marker is displayed anyway if the scene has no valid preview."));
		} break;

		case NOTIFICATION_TRANSLATION_CHANGED: {
			_update_scenes_list();
		} break;

		case NOTIFICATION_THEME_CHANGED: {
			scene_tile_add_button->set_button_icon(get_editor_theme_icon(SNAME("Add")));
			scene_tile_delete_button->set_button_icon(get_editor_theme_icon(SNAME("Remove")));
			scene_tiles_sort_button->set_button_icon(get_editor_theme_icon(SNAME("Sort")));
			scene_tiles_grid_button->set_button_icon(get_editor_theme_icon(SNAME("FileThumbnail")));
			_update_scenes_list();
		} break;

		case NOTIFICATION_INTERNAL_PROCESS: {
			if (tile_set_scenes_collection_source_changed_needs_update) {
				read_only = false;
				// Add the listener again and check for read-only status.
				if (tile_set.is_valid()) {
					read_only = EditorNode::get_singleton()->is_resource_read_only(tile_set);
				}

				// Update everything.
				_update_source_inspector();
				_update_scenes_list();
				_update_action_buttons();
				_update_tile_inspector();
				tile_set_scenes_collection_source_changed_needs_update = false;
			}
		} break;

		case NOTIFICATION_VISIBILITY_CHANGED: {
			// Update things just in case.
			_update_scenes_list();
			_update_action_buttons();
		} break;
	}
}

void TileSetScenesCollectionSourceEditor::edit(Ref<TileSet> p_tile_set, TileSetScenesCollectionSource *p_tile_set_scenes_collection_source, int p_source_id) {
	ERR_FAIL_COND(p_tile_set.is_null());
	ERR_FAIL_NULL(p_tile_set_scenes_collection_source);
	ERR_FAIL_COND(p_source_id < 0);
	ERR_FAIL_COND(p_tile_set->get_source(p_source_id) != p_tile_set_scenes_collection_source);

	bool new_read_only_state = false;
	if (p_tile_set.is_valid()) {
		new_read_only_state = EditorNode::get_singleton()->is_resource_read_only(p_tile_set);
	}

	if (p_tile_set == tile_set && p_tile_set_scenes_collection_source == *tile_set_scenes_collection_source && p_source_id == tile_set_source_id && new_read_only_state == read_only) {
		return;
	}

	// Remove listener for old objects.
	if (tile_set_scenes_collection_source.is_valid()) {
		tile_set_scenes_collection_source->disconnect_changed(callable_mp(this, &TileSetScenesCollectionSourceEditor::_tile_set_scenes_collection_source_changed));
	}

	// Change the edited object.
	tile_set = p_tile_set;
	tile_set_scenes_collection_source = p_tile_set_scenes_collection_source;
	tile_set_source_id = p_source_id;

	// Read-only status is false by default
	read_only = new_read_only_state;

	if (tile_set.is_valid()) {
		scenes_collection_source_inspector->set_read_only(read_only);
		tile_inspector->set_read_only(read_only);

		scene_tile_add_button->set_disabled(read_only);
	}

	// Add the listener again.
	if (tile_set_scenes_collection_source.is_valid()) {
		tile_set_scenes_collection_source->connect_changed(callable_mp(this, &TileSetScenesCollectionSourceEditor::_tile_set_scenes_collection_source_changed));
	}

	// Update everything.
	_update_source_inspector();
	_update_scenes_list();
	_update_action_buttons();
	_update_tile_inspector();
}

void TileSetScenesCollectionSourceEditor::_drop_data_fw(const Point2 &p_point, const Variant &p_data, Control *p_from) {
	if (!_can_drop_data_fw(p_point, p_data, p_from)) {
		return;
	}

	if (p_from == scene_tiles_list) {
		// Handle dropping a texture in the list of atlas resources.
		Dictionary d = p_data;
		const Vector<String> files = d["files"];
		for (const String &file : files) {
			const Ref<PackedScene> resource = ResourceLoader::load(file);
			if (resource.is_valid()) {
				int scene_id = tile_set_scenes_collection_source->get_next_scene_tile_id();
				EditorUndoRedoManager *undo_redo = EditorUndoRedoManager::get_singleton();
				undo_redo->create_action(TTR("Add a Scene Tile"));
				undo_redo->add_do_method(*tile_set_scenes_collection_source, "create_scene_tile", resource, scene_id);
				undo_redo->add_do_method(this, "_update_all");
				undo_redo->add_undo_method(*tile_set_scenes_collection_source, "remove_scene_tile", scene_id);
				undo_redo->add_undo_method(this, "_update_all");
				undo_redo->commit_action();
			}
		}
	}
}

bool TileSetScenesCollectionSourceEditor::_can_drop_data_fw(const Point2 &p_point, const Variant &p_data, Control *p_from) const {
	if (p_from == scene_tiles_list) {
		Dictionary d = p_data;

		if (!d.has("type")) {
			return false;
		}

		// Check if we have a Texture2D.
		if (String(d["type"]) == "files") {
			Vector<String> files = d["files"];

			if (files.is_empty()) {
				return false;
			}

			for (int i = 0; i < files.size(); i++) {
				String ftype = EditorFileSystem::get_singleton()->get_file_type(files[i]);

				if (!ClassDB::is_parent_class(ftype, "PackedScene")) {
					return false;
				}
			}

			return true;
		}
	}
	return false;
}

void TileSetScenesCollectionSourceEditor::_bind_methods() {
	ClassDB::bind_method(D_METHOD("_update_all"), &TileSetScenesCollectionSourceEditor::_update_all);

	ADD_SIGNAL(MethodInfo("source_id_changed", PropertyInfo(Variant::INT, "source_id")));
}

TileSetScenesCollectionSourceEditor::TileSetScenesCollectionSourceEditor() {
	// -- Right side --
	HSplitContainer *split_container_right_side = memnew(HSplitContainer);
	split_container_right_side->set_h_size_flags(SIZE_EXPAND_FILL);
	add_child(split_container_right_side);

	// Middle panel.
	ScrollContainer *middle_panel = memnew(ScrollContainer);
	middle_panel->set_horizontal_scroll_mode(ScrollContainer::SCROLL_MODE_DISABLED);
	middle_panel->set_custom_minimum_size(Size2(200, 0) * EDSCALE);
	split_container_right_side->add_child(middle_panel);

	VBoxContainer *middle_vbox_container = memnew(VBoxContainer);
	middle_vbox_container->set_h_size_flags(SIZE_EXPAND_FILL);
	middle_panel->add_child(middle_vbox_container);

	// Scenes collection source inspector.
	scenes_collection_source_inspector_label = memnew(Label);
	scenes_collection_source_inspector_label->set_focus_mode(FOCUS_ACCESSIBILITY);
	scenes_collection_source_inspector_label->set_text(TTRC("Scenes collection properties:"));
	middle_vbox_container->add_child(scenes_collection_source_inspector_label);

	scenes_collection_source_inspector = memnew(EditorInspector);
	scenes_collection_source_inspector->set_vertical_scroll_mode(ScrollContainer::SCROLL_MODE_DISABLED);
	scenes_collection_source_inspector->set_use_doc_hints(true);
	middle_vbox_container->add_child(scenes_collection_source_inspector);

	// Tile inspector.
	tile_inspector_label = memnew(Label);
	tile_inspector_label->set_text(TTRC("Tile properties:"));
	tile_inspector_label->hide();
	middle_vbox_container->add_child(tile_inspector_label);

	tile_inspector = memnew(EditorInspector);
	tile_inspector->set_vertical_scroll_mode(ScrollContainer::SCROLL_MODE_DISABLED);
	tile_inspector->set_use_doc_hints(true);
	tile_inspector->set_use_folding(true);
	middle_vbox_container->add_child(tile_inspector);

	// Scenes list.
	VBoxContainer *right_vbox_container = memnew(VBoxContainer);
	split_container_right_side->add_child(right_vbox_container);

	scene_tiles_list = memnew(ItemList);
	scene_tiles_list->set_auto_translate_mode(AUTO_TRANSLATE_MODE_DISABLED);
	scene_tiles_list->set_h_size_flags(SIZE_EXPAND_FILL);
	scene_tiles_list->set_v_size_flags(SIZE_EXPAND_FILL);
	scene_tiles_list->set_theme_type_variation("ItemListSecondary");
	SET_DRAG_FORWARDING_CDU(scene_tiles_list, TileSetScenesCollectionSourceEditor);
	scene_tiles_list->connect(SceneStringName(item_selected), callable_mp(this, &TileSetScenesCollectionSourceEditor::_update_tile_inspector).unbind(1));
	scene_tiles_list->connect(SceneStringName(item_selected), callable_mp(this, &TileSetScenesCollectionSourceEditor::_update_action_buttons).unbind(1));
	scene_tiles_list->connect("item_activated", callable_mp(this, &TileSetScenesCollectionSourceEditor::_scenes_list_item_activated));
	scene_tiles_list->set_texture_filter(CanvasItem::TEXTURE_FILTER_NEAREST);
	scene_tiles_list->connect(SceneStringName(gui_input), callable_mp(this, &TileSetScenesCollectionSourceEditor::_scenes_list_gui_input));
	right_vbox_container->add_child(scene_tiles_list);

	scene_tiles_grid_view = EditorSettings::get_singleton()->get_project_metadata("editor_metadata", "scene_tiles_grid_view", false);
	scene_tiles_sort_by_name = EditorSettings::get_singleton()->get_project_metadata("editor_metadata", "scene_tiles_sort_by_name", false);
	scene_tiles_thumbnail_size = EditorSettings::get_singleton()->get_project_metadata("editor_metadata", "scene_tiles_thumbnail_size", 64);

	HBoxContainer *scenes_bottom_actions = memnew(HBoxContainer);
	right_vbox_container->add_child(scenes_bottom_actions);

	scene_tile_add_button = memnew(Button);
	scene_tile_add_button->set_theme_type_variation(SceneStringName(FlatButton));
	scene_tile_add_button->connect(SceneStringName(pressed), callable_mp(this, &TileSetScenesCollectionSourceEditor::_source_add_pressed));
	scene_tile_add_button->set_accessibility_name(TTRC("Add"));
	scenes_bottom_actions->add_child(scene_tile_add_button);

	scene_tile_delete_button = memnew(Button);
	scene_tile_delete_button->set_theme_type_variation(SceneStringName(FlatButton));
	scene_tile_delete_button->set_disabled(true);
	scene_tile_delete_button->connect(SceneStringName(pressed), callable_mp(this, &TileSetScenesCollectionSourceEditor::_source_delete_pressed));
	scene_tile_delete_button->set_accessibility_name(TTRC("Delete"));
	scenes_bottom_actions->add_child(scene_tile_delete_button);

	Control *scenes_bottom_spacer = memnew(Control);
	scenes_bottom_spacer->set_h_size_flags(SIZE_EXPAND_FILL);
	scenes_bottom_actions->add_child(scenes_bottom_spacer);

	scene_tiles_size_slider = memnew(HSlider);
	scene_tiles_size_slider->set_min(32);
	scene_tiles_size_slider->set_max(224);
	scene_tiles_size_slider->set_step(16);
	scene_tiles_size_slider->set_value_no_signal(scene_tiles_thumbnail_size);
	scene_tiles_size_slider->set_custom_minimum_size(Size2(64 * EDSCALE, 0));
	scene_tiles_size_slider->set_custom_maximum_size(Size2(128 * EDSCALE, -1));
	scene_tiles_size_slider->set_v_size_flags(SIZE_SHRINK_CENTER);
	scene_tiles_size_slider->set_h_size_flags(SIZE_EXPAND_FILL);
	scene_tiles_size_slider->set_visible(scene_tiles_grid_view);
	scene_tiles_size_slider->set_tooltip_text(TTRC("Thumbnail size. Ctrl + mouse wheel over the list also changes it."));
	scene_tiles_size_slider->set_accessibility_name(TTRC("Thumbnail Size"));
	scene_tiles_size_slider->connect(SceneStringName(value_changed), callable_mp(this, &TileSetScenesCollectionSourceEditor::_scenes_list_thumbnail_size_changed));
	scenes_bottom_actions->add_child(scene_tiles_size_slider);

	scene_tiles_sort_button = memnew(Button);
	scene_tiles_sort_button->set_theme_type_variation(SceneStringName(FlatButton));
	scene_tiles_sort_button->set_toggle_mode(true);
	scene_tiles_sort_button->set_pressed_no_signal(scene_tiles_sort_by_name);
	scene_tiles_sort_button->set_tooltip_text(TTRC("Sort scene tiles by name instead of by ID."));
	scene_tiles_sort_button->set_accessibility_name(TTRC("Sort by Name"));
	scene_tiles_sort_button->connect(SceneStringName(toggled), callable_mp(this, &TileSetScenesCollectionSourceEditor::_scenes_list_sort_toggled));
	scenes_bottom_actions->add_child(scene_tiles_sort_button);

	scene_tiles_grid_button = memnew(Button);
	scene_tiles_grid_button->set_theme_type_variation(SceneStringName(FlatButton));
	scene_tiles_grid_button->set_toggle_mode(true);
	scene_tiles_grid_button->set_pressed_no_signal(scene_tiles_grid_view);
	scene_tiles_grid_button->set_tooltip_text(TTRC("View scene tiles as a grid of thumbnails."));
	scene_tiles_grid_button->set_accessibility_name(TTRC("Grid View"));
	scene_tiles_grid_button->connect(SceneStringName(toggled), callable_mp(this, &TileSetScenesCollectionSourceEditor::_scenes_list_grid_toggled));
	scenes_bottom_actions->add_child(scene_tiles_grid_button);

	EditorInspector::add_inspector_plugin(memnew(TileSourceInspectorPlugin));
}

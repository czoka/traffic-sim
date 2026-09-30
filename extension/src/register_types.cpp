#include "road_editor.h"
#include "traffic_sim.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

namespace {

void initialize_traffic_sim(ModuleInitializationLevel level) {
	if (level != MODULE_INITIALIZATION_LEVEL_SCENE) {
		return;
	}
	GDREGISTER_CLASS(TrafficSim);
	GDREGISTER_CLASS(RoadEditor);
}

void uninitialize_traffic_sim(ModuleInitializationLevel level) {
	(void)level;
}

} // namespace

extern "C" {

GDExtensionBool GDE_EXPORT traffic_sim_library_init(GDExtensionInterfaceGetProcAddress p_get_proc_address,
		const GDExtensionClassLibraryPtr p_library, GDExtensionInitialization *r_initialization) {
	GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
	init_obj.register_initializer(initialize_traffic_sim);
	init_obj.register_terminator(uninitialize_traffic_sim);
	init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
	return init_obj.init();
}

}

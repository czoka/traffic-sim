class_name PathDrawTool
extends RoadDrawTool
## P: draws a footpath, bike path or shared path like the road tool (click,
## bend, double-click to finish). Footpaths never join roads: their ends link
## to the nearest sidewalk within a few metres. Bike and shared paths join
## roads and junctions like any road. 1 footpath, 2 bike path, 3 shared path.

const KINDS := {
	"footpath": {"label": "footpath", "width": 3.0, "kmh": 5.0},
	"bike_path": {"label": "bike path", "width": 2.5, "kmh": 20.0},
	"shared_path": {"label": "shared path", "width": 4.0, "kmh": 15.0},
}

var kind := "footpath"


func hint() -> String:
	return "Drawing a %s · 1 footpath, 2 bike path, 3 shared path · click to start, double-click or Enter to finish" % KINDS[kind].label


func input(event: InputEvent) -> bool:
	for k in [[KEY_1, "footpath"], [KEY_2, "bike_path"], [KEY_3, "shared_path"]]:
		if key_pressed(event, k[0]):
			kind = k[1]
			editor.ui.set_status(hint())
			return true
	return super.input(event)


func _template() -> Dictionary:
	return {"kind": kind}


func _speed_kmh() -> float:
	return KINDS[kind].kmh


func _width() -> float:
	return KINDS[kind].width


func _noun() -> String:
	return KINDS[kind].label

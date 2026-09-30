class_name Benchmark
extends Node
## Runs the POC measurement plan: frame time at 1,000 / 2,000 / 5,000 cars,
## then time slicing at 16x and 128x. Writes a report to user:// (and downloads
## it on the web). Start from the HUD, with `-- --bench` on desktop (quits when
## done), or by opening the web build with ?bench in the URL.

const PHASES := [
	{"name": "1,000 cars @ 1x", "cars": 1000, "speed": 1.0},
	{"name": "2,000 cars @ 1x", "cars": 2000, "speed": 1.0},
	{"name": "5,000 cars @ 1x", "cars": 5000, "speed": 1.0},
	{"name": "2,000 cars @ 16x", "cars": 2000, "speed": 16.0},
	{"name": "2,000 cars @ 128x", "cars": 2000, "speed": 128.0},
	{"name": "5,000 cars @ 128x", "cars": 5000, "speed": 128.0},
]
const RESULTS_PATH := "user://bench_results.json"
## Gate from the implementation plan: 2,000 cars at 60 fps in a desktop browser.
const GATE_PHASE := 1
const GATE_MIN_FPS := 58.0

var main: Main
var running := false
var results: Array = []

var _quit_when_done := false
var _warmup := 2.0
var _measure := 8.0
var _phase := -1
var _t := 0.0
var _frames := PackedFloat32Array()
var _sim_ms := PackedFloat32Array()
var _ticks := 0
var _behind := 0
var _sim_time_start := 0.0
var _measuring := false
var _golden: Dictionary = {}
var _saved := {}


func start(quick: bool, quit_when_done: bool) -> void:
	if running:
		return
	_warmup = 0.5 if quick else 2.0
	_measure = 2.0 if quick else 8.0
	_quit_when_done = quit_when_done
	_saved = {"cars": main.car_count, "speed": main.speed, "paused": main.paused}
	results.clear()
	running = true
	main.notify("Benchmark: determinism check…")
	_golden = main.run_determinism_check()
	_begin_phase(0)


func _begin_phase(i: int) -> void:
	_phase = i
	var p: Dictionary = PHASES[i]
	main.car_count = p.cars
	main.respawn()
	main.set_speed(p.speed)
	main.hud.refresh_controls()
	_t = 0.0
	_measuring = false
	_frames = PackedFloat32Array()
	_sim_ms = PackedFloat32Array()
	_ticks = 0
	_behind = 0
	main.notify("Benchmark %d/%d: %s" % [i + 1, PHASES.size(), p.name])


func _process(delta: float) -> void:
	if not running:
		return
	_t += delta
	if _t < _warmup:
		return
	var s: Dictionary = main.sim.get_stats()
	if not _measuring:
		_measuring = true
		_sim_time_start = s.sim_time
		_t = _warmup
		return
	_frames.append(delta * 1000.0)
	_sim_ms.append(s.last_frame_sim_ms)
	_ticks += int(s.last_frame_ticks)
	if s.behind:
		_behind += 1
	if _t - _warmup >= _measure:
		_finish_phase(s)


func _finish_phase(s: Dictionary) -> void:
	var p: Dictionary = PHASES[_phase]
	var real_time := _t - _warmup
	var frames := _frames.duplicate()
	frames.sort()
	var n := frames.size()
	var total_frame := 0.0
	for v in frames:
		total_frame += v
	var total_sim := 0.0
	var max_sim := 0.0
	for v in _sim_ms:
		total_sim += v
		max_sim = maxf(max_sim, v)
	results.append({
		"name": p.name,
		"cars": s.vehicles,
		"speed_requested": p.speed,
		"frames": n,
		"fps": n / real_time,
		"frame_avg_ms": total_frame / n,
		"frame_p95_ms": frames[mini(n - 1, int(n * 0.95))],
		"frame_max_ms": frames[n - 1],
		"sim_ms_per_frame_avg": total_sim / n,
		"sim_ms_per_frame_max": max_sim,
		"tick_us": (total_sim * 1000.0 / _ticks) if _ticks > 0 else 0.0,
		"ticks_per_frame": float(_ticks) / n,
		"speed_achieved": (s.sim_time - _sim_time_start) / real_time,
		"frames_over_budget_pct": 100.0 * _behind / n,
		"render_prep_us": s.render_prep_us,
	})
	if _phase + 1 < PHASES.size():
		_begin_phase(_phase + 1)
	else:
		_finish()


func _finish() -> void:
	running = false
	var report := _report_text()
	var data := {
		"environment": _environment(),
		"determinism": _golden,
		"phases": results,
		"gate": _gate(),
	}
	var json := JSON.stringify(data, "  ")
	var f := FileAccess.open(RESULTS_PATH, FileAccess.WRITE)
	if f:
		f.store_string(json)
		f.close()
	main.notify(report)
	if OS.has_feature("web"):
		JavaScriptBridge.download_buffer(json.to_utf8_buffer(), "traffic-sim-bench.json", "application/json")
	else:
		print("Results written to %s" % ProjectSettings.globalize_path(RESULTS_PATH))
	main.car_count = _saved.cars
	main.respawn()
	main.speed = _saved.speed
	main.paused = _saved.paused
	main.hud.refresh_controls()
	if _quit_when_done:
		get_tree().quit(0 if _gate().pass else 1)


func _gate() -> Dictionary:
	var fps_ok: bool = results.size() > GATE_PHASE and results[GATE_PHASE].fps >= GATE_MIN_FPS
	var det_ok: bool = _golden.get("pass", false)
	return {"fps_2000_ok": fps_ok, "determinism_ok": det_ok, "pass": fps_ok and det_ok}


func _report_text() -> String:
	var lines := ["Benchmark (%s)" % _environment().platform,
		"%-20s %6s %7s %7s %8s %8s" % ["phase", "fps", "p95 ms", "sim ms", "tick us", "speed"]]
	for r in results:
		lines.append("%-20s %6.1f %7.2f %7.2f %8.1f %7.1fx" % [
			r.name, r.fps, r.frame_p95_ms, r.sim_ms_per_frame_avg, r.tick_us, r.speed_achieved])
	var g := _gate()
	lines.append("Determinism: %s   2,000 cars ≥ %d fps: %s" % [
		"PASS" if g.determinism_ok else "FAIL", int(GATE_MIN_FPS), "PASS" if g.fps_2000_ok else "FAIL"])
	return "\n".join(lines)


func _environment() -> Dictionary:
	var env := {
		"platform": OS.get_name() + (" (web)" if OS.has_feature("web") else ""),
		"godot": Engine.get_version_info().string,
		"cpu": OS.get_processor_name(),
		"gpu": RenderingServer.get_video_adapter_name(),
		"renderer": RenderingServer.get_current_rendering_method(),
		"debug_build": OS.is_debug_build(),
		"screen_refresh_hz": DisplayServer.screen_get_refresh_rate(),
	}
	if OS.has_feature("web"):
		env["user_agent"] = str(JavaScriptBridge.eval("navigator.userAgent", true))
	return env

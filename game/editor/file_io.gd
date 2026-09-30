class_name FileIO
extends Node
## Map export/import as JSON files: native file dialogs on desktop, a download
## and a file picker in the browser.

var _pending: Callable
var _js_callback = null # keeps the JavaScript callback alive


func save_text(text: String, filename: String) -> void:
	if OS.has_feature("web"):
		JavaScriptBridge.download_buffer(text.to_utf8_buffer(), filename, "application/json")
		return
	var dlg := _dialog(FileDialog.FILE_MODE_SAVE_FILE)
	dlg.current_file = filename
	dlg.file_selected.connect(func(path: String) -> void:
		var f := FileAccess.open(path, FileAccess.WRITE)
		if f:
			f.store_string(text)
			f.close()
		dlg.queue_free())
	dlg.popup_centered_ratio(0.6)


## Calls `done(text)` with the file's contents once the user picks one.
func open_text(done: Callable) -> void:
	_pending = done
	if OS.has_feature("web"):
		_js_callback = JavaScriptBridge.create_callback(_on_web_file)
		var window = JavaScriptBridge.get_interface("window")
		window.tsimFileLoaded = _js_callback
		JavaScriptBridge.eval("""
			(function () {
				var input = document.createElement('input');
				input.type = 'file';
				input.accept = '.json,application/json';
				input.onchange = function (e) {
					var file = e.target.files[0];
					if (!file) return;
					var reader = new FileReader();
					reader.onload = function () { window.tsimFileLoaded(reader.result); };
					reader.readAsText(file);
				};
				input.click();
			})();
		""", true)
		return
	var dlg := _dialog(FileDialog.FILE_MODE_OPEN_FILE)
	dlg.file_selected.connect(func(path: String) -> void:
		var text := FileAccess.get_file_as_string(path)
		dlg.queue_free()
		_pending.call(text))
	dlg.popup_centered_ratio(0.6)


func _on_web_file(args: Array) -> void:
	if args.size() > 0 and _pending.is_valid():
		_pending.call(str(args[0]))


func _dialog(mode: FileDialog.FileMode) -> FileDialog:
	var dlg := FileDialog.new()
	dlg.file_mode = mode
	dlg.access = FileDialog.ACCESS_FILESYSTEM
	dlg.filters = PackedStringArray(["*.json ; Traffic Sim maps"])
	dlg.use_native_dialog = true
	dlg.canceled.connect(dlg.queue_free)
	add_child(dlg)
	return dlg

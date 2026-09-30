// Keeps painted lines at least `min_px` pixels wide when zoomed out, and fades
// them a little so thin paint doesn't turn into heavy lines. UV holds each
// vertex's outward normal times the line's half width (zero for filled paint
// such as arrows and stop lines, which are left untouched).
shader_type canvas_item;

uniform float min_px = 1.1;

void vertex() {
	float half_width = length(UV);
	if (half_width > 0.0) {
		float px_per_unit = length((CANVAS_MATRIX * MODEL_MATRIX)[0].xy);
		float wanted = 0.5 * min_px / max(px_per_unit, 1e-6);
		if (wanted > half_width) {
			VERTEX += (UV / half_width) * (wanted - half_width);
			COLOR.a *= clamp(half_width / wanted * 1.6, 0.45, 1.0);
		}
	}
}

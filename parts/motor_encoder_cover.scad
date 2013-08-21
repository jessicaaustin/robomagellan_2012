id = 34.9;
h = 19.1;
thickness = 2.0;
notchDepth = 10.0;
$fa = 1;

difference() {
cylinder(r = id / 2.0 + thickness, h = h);
translate([0, 0, thickness])
cylinder(r = id / 2.0, h = h - thickness);
translate([id / 2.0 - thickness * 0.4, -notchDepth / 2.0, h - notchDepth])
cube(size = [thickness * 1.4, notchDepth, notchDepth]);
}

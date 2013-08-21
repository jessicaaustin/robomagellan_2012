baseX = 69.4;
baseY = 9.0;
baseZ = 6.0;

postX = 4.0;
postY = baseY;
postZ = 29.0;

notchX = 1.3;
notchY = 2.0;
notchZ = postZ;

postSeparationX = 46.0 - (2 * notchX);
holeRadius = 2.5;
holeSeparationX = 60.4;

/*
 * The base with the screw holes.
 */
difference() {
	translate([-baseX / 2.0, -baseY / 2.0, 0.0])
	cube( size = [baseX, baseY, baseZ]);

	translate([-holeSeparationX / 2.0, 0.0, 0.0])
	cylinder(h = baseZ, r = holeRadius);

	translate([holeSeparationX / 2.0, 0.0, 0.0])
	cylinder(h = baseZ, r = holeRadius);
}

/*
 * The right post with notch.
 */
difference() {
	translate([postSeparationX / 2.0, -postY / 2.0, baseZ])
	cube(size = [postX, postY, postZ]);

	translate([postSeparationX /2.0, -notchY / 2.0, baseZ])
	cube(size = [notchX, notchY, notchZ]);
}

/*
 * The left post with notch.
 */
difference() {
	translate([-postSeparationX / 2.0 - postX, -postY / 2.0, baseZ])
	cube(size = [postX, postY, postZ]);

	translate([-postSeparationX / 2.0 - notchX, -notchY / 2.0, baseZ])
	cube(size = [notchX, notchY, notchZ]);
}


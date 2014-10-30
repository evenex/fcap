module fcap.units;

import evx.math;

alias vec 				= Vector!(3, double);
alias Position 			= Vector!(3, Meters);
alias SurfacePosition	= Vector!(2, Meters);
alias Force 			= Vector!(3, Newtons);
alias Moment 			= Vector!(3, NewtonMeters);
alias SurfaceMoment		= Vector!(2, NewtonMeters);

alias mm = millimeters;
alias ms = milliseconds;

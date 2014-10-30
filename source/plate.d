module fcap.plate;

import evx.meta;
import evx.dsp;
import evx.math;

import fcap.units;

mixin(FunctionalToolkit!());
alias stride = evx.range.stride;

struct ForcePlate
	{/*...}*/
		@property {/*force}*/
			const force ()
				{/*...}*/
					return zip (force_x, force_y, force_z).map!vector;
				}

			const force_x ()
				{/*...}*/
					return zip (fx12[], fx34[])
						.map!(v => v.vector[].sum * voltage_to_force.x);
				}
			const force_y ()
				{/*...}*/
					return zip (fy14[], fy23[])
						.map!(v => v.vector[].sum * voltage_to_force.y);
				}
			const force_z ()
				{/*...}*/
					return zip (fz1[], fz2[], fz3[], fz4[])
						.map!(v => v.vector[].sum * voltage_to_force.z);
				}
		}
		@property {/*moment}*/
			const moment ()
				{/*...}*/
					return zip (moment_x, moment_y, moment_z).map!vector;
				}

			const moment_x ()
				{/*...}*/
					return zip (fz1[], fz2[], fz3[], fz4[])
						.map!(v => ([+1,+1,-1,-1] * v.vector)[].sum)
						.map!(v => v * voltage_to_force.z)
						.map!(f => f * sensor_offset.y);
				}
			const moment_y ()
				{/*...}*/
					return zip (fz1[], fz2[], fz3[], fz4[])
						.map!(v => - v[0] + v[1] + v[2] - v[3])
						.map!(v => v * voltage_to_force.z)
						.map!(f => f * sensor_offset.x);

				}
			const moment_z ()
				{/*...}*/
					return zip (
						zip (fx34[], fx12[])
							.map!subtract
							.map!(v => v * voltage_to_force.x)
							.map!(f => f * sensor_offset.y),

						zip (fy14[], fy23[])
							.map!subtract
							.map!(v => v * voltage_to_force.y)
							.map!(f => f * sensor_offset.x)
					).map!add;
				}
		}
		@property {/*surface moment}*/
			const surface_moment ()
				{/*...}*/
					return zip (surface_moment_x, surface_moment_y).map!vector;
				}

			const surface_moment_x ()
				{/*...}*/
					return zip (
						moment_x,
						force_y.map!(f => f * sensor_offset.z)
					).map!(m => m.vector[].sum);
				}
			const surface_moment_y ()
				{/*...}*/
					return zip (
						moment_y, 
						force_x.map!(f => f * sensor_offset.z)
					).map!(m => ([+1,-1] * m.vector)[].sum);
				}
		}
		@property {/*center_of_pressure}*/
			const center_of_pressure ()
				{/*...}*/
					return zip (center_of_pressure_x, center_of_pressure_y).map!vector;
				}

			const center_of_pressure_x ()
				{/*...}*/
					return zip (
						force_x.map!(f => f * sensor_offset.z),
						moment_y
					).map!(m => ([+1,-1] * m.vector)[].sum)
						.zip (force_z).map!divide;
				}
			const center_of_pressure_y ()
				{/*...}*/
					return zip (
						force_y.map!(f => f * sensor_offset.z),
						moment_x
					).map!(m => m.vector[].sum)
						.zip (force_z).map!divide;
				}
		}
		@property {/*torque}*/
			const z_torque ()
				{/*...}*/
					return zip (
						force_y.map!(f => f * sensor_offset.x),
						force_x.map!(f => f * sensor_offset.y)
					).map!(τ => ([+1,-1] * τ.vector)[].sum)
						.zip (moment_z).map!add;
				}
		}
		public:
		@property {/*input signals}*/
			mixin Builder!(
				Stream!(Volts, Seconds), `fx12`,
				Stream!(Volts, Seconds), `fx34`,
				Stream!(Volts, Seconds), `fy14`,
				Stream!(Volts, Seconds), `fy23`,
				Stream!(Volts, Seconds), `fz1`,
				Stream!(Volts, Seconds), `fz2`,
				Stream!(Volts, Seconds), `fz3`,
				Stream!(Volts, Seconds), `fz4`,
			);
		}
		@property {/*plate settings}*/
			mixin Builder!(
				Vector!(3, typeof(newtons/volt)), `voltage_to_force`,
				Position, `sensor_offset`,
			);
		}
	}

module fcap.main;

import fcap.units;
import fcap.daq;
import fcap.plate;
import ipc;

import evx.meta;
import evx.dsp;

import opencv;

static if (1) version = microphone_enabled;
static if (0) version = camera_enabled;

enum mic_threshold = 0.3.volts;

private {/*imports}*/
	private {/*core}*/
		import core.thread;
	}
	private {/*std}*/
		import std.range;
		import std.algorithm;
		import std.stdio;
		import std.file;
		import std.conv;
		import std.string;
		import std.datetime;
		import std.concurrency;
		import std.process;
	}
	private {/*evx}*/
		import evx.utils;
		import evx.display;
		import evx.input;
		import evx.colors;
		import evx.math;
		import evx.plot;
		import evx.scribe;
	}

	mixin(FunctionalToolkit!());

	alias stride = evx.range.stride;
	alias Interval = evx.analysis.Interval;
	alias ceil = evx.analysis.ceil;
	alias seconds = evx.units.seconds;
	alias uvec = Vector!(2,ulong);
	alias vec = Vector!(2,double);
	alias contains = std.algorithm.canFind;
	alias sum = evx.arithmetic.sum;
}

__gshared {/*devices}*/
	auto daq = DAQDevice!(
		Model!`NI USB-6216`,
		Serial!0x_18DEF36,

		InputChannels!16,
		OutputChannels!2,

		InputBufferSize!4095,
		OutputBufferSize!8191,

		MaxSamplingRate!(250_000),
		MaxInputVoltageRange!(-10, 10),
		MaxOutputVoltageRange!(-10, 10),
	).init; /* source: http://www.ni.com/pdf/manuals/371932f.pdf */

	auto microphone ()()
		{/*...}*/
			version (microphone_enabled)
				return daq.input[7].sample_by_time;
			else static assert (0);
		}
	auto spike ()(Seconds timespan)
		{/*...}*/
			return microphone[$-timespan..$].reduce!(min, max).subtract.abs;
		}

	auto timing_light () 
		{/*...}*/
			return zip (
				daq.output[0].sample_by_time[],
				daq.output[1].sample_by_time[]
			).map!vector;
		}

	ForcePlate plate;

	shared static this ()
		{/*...}*/
			daq = new typeof(daq);

			with (daq) {/*settings}*/
				sampling_frequency = 24.kilohertz;
				capture_frequency = 60.hertz;
				generating_frequency = 240.hertz;

				history_length = 10.seconds;
				generating_period = (1/120.).seconds;

				voltage_range!`input` = interval (-5.volts, 5.volts);
				voltage_range!`output` = interval (0.volts, 3.volts);
			}
			with (plate) {/*input signals}*/
				voltage_to_force = vector (1250.newtons/5.volts, 1250.newtons/5.volts, 2500.newtons/5.volts);
				/* source: check the signal conditioner settings TODO maybe automate this with opencv and a webcam */

				sensor_offset = vector (210.mm, 260.mm, -41.mm);
				/* source: Kistler Type 9260AA Instruction Manual, p.33 */

		//		fx12 = daq.input[1] .sample_by_time;
		//		fx34 = daq.input[9] .sample_by_time;
		//		fy14 = daq.input[2] .sample_by_time;
		//		fy23 = daq.input[10].sample_by_time;
				fz1  = daq.input[3] .sample_by_time;
				fz2  = daq.input[11].sample_by_time;
				fz3  = daq.input[4] .sample_by_time;
				fz4  = daq.input[12].sample_by_time;
				/* source: check the wires */
			}
			version (none) with (daq) {/*output signals}*/
				immutable period = generating_period;
				immutable frequency = generating_frequency;

				//output[0].generate (t => t % period < 1/frequency? 3.volts : 0.volts); OUTSIDE BUG can't autodetect purity of these function, infers type as void
				//output[1].generate (t => t % period < 1/frequency? 0.volts : 2.volts);

				pure blue_signal (Seconds t) // function defined as HACK around failure of purity detection
					{/*...}*/
						return t % period < 1/frequency? 3.volts : 0.volts;
					}
				pure red_signal (Seconds t)
					{/*...}*/
						return t % period < 1/frequency? 0.volts : 2.volts;
					}

				output[0].generate (&blue_signal);
				output[1].generate (&red_signal);
			}

			//daq.open_channels!`input` (1,2,3,4, 9,10,11,12);
			daq.open_channels!`input` (3,4, 11,12);

			foreach (channel; daq.input)
				channel.reference_mode = ReferenceMode.single_ended;

			version (microphone_enabled)
				{/*...}*/
					daq.open_channel!`input` (7);
					daq.input[7].reference_mode = ReferenceMode.differential;
				}
		}
}
public {/*operating modes}*/
	void analysis ()
		{/*...}*/
			scope gfx = new Display (1920, 1080);
			gfx.start; scope (exit) gfx.stop;
			gfx.background (white);

			scope txt = new Scribe (gfx, [20]);

			pwriteln (gfx.extended_bounds); // BUG wrong, max(x) is 1.77778, this is correct

			struct Run
				{/*...}*/
					string name;
					Hertz sampling_frequency;

					Seconds[] time;
					Force[] forces;
					Moment[] moments;
					SurfaceMoment[] surface_moments;
					Position[] centers;
					NewtonMeters[] torques;
				}
			Run[] runs;

			auto directory = `dat`.dirEntries (SpanMode.breadth).filter!(entry => entry.name.endsWith (`.dat`)).array;
			if (directory.empty) return;

			foreach (i, path; directory) 
				{/*...}*/
					auto report_load_progress (int percent)
						{/*...}*/
							txt.write (`loading data ` ~percent.text~ `%`)
								.align_to (Alignment.center)
							();
							gfx.render;
						}

					runs ~= Run (path.text.findSplitAfter (`dat/`)[1].text);
					report_load_progress ((100*i)/directory.length);

					with (runs.back) foreach (line; File (path, `r`).byLine)
						if (line.canFind (`sampling_frequency`))
							{/*...}*/
								sampling_frequency = Hertz (line.findSplitAfter (`sampling_frequency`)[1].text);
								break;
							}

					with (runs.back) foreach (line; File (path ,`r`).byLine)
						if (line.canFind (`s}`))
							time ~= Seconds (line.text);
						else if (line.canFind (`force`))
							forces ~= Force (line.findSplitAfter (`force`)[1].text);
						else if (line.canFind (`moment`))
							moments ~= Moment (line.findSplitAfter (`moment`)[1].text);
						else if (line.canFind (`surface_moment`))
							surface_moments ~= SurfaceMoment (line.findSplitAfter (`surface_moment`)[1].text);
						else if (line.canFind (`center_of_pressure`))
							centers ~= Position (line.findSplitAfter (`center_of_pressure`)[1].text);
						else if (line.canFind (`torque`))
							torques ~= NewtonMeters (line.findSplitAfter (`torque`)[1].text);

					report_load_progress ((100*(i+1))/directory.length);
				}

			bool terminate;
			int selected_run = 0;
			scope usr = new Input (gfx, (bool){terminate = true;});
			usr.bind (Input.Key.left, (bool on){if (on) selected_run = max (0, selected_run - 1);});
			usr.bind (Input.Key.right, (bool on){if (on) selected_run = min (runs.length - 1, selected_run + 1);});
			usr.bind (Input.Key.space, (bool on){if (on) {
				auto data = new ubyte[std.math.ceil (gfx.dimensions[].product * 3).to!uint];
				gfx.screenshot (data);

				auto img = Image (gfx.dimensions.x.to!uint, gfx.dimensions.y.to!uint);
				cv.SetData (img, data.ptr, gfx.dimensions.x.to!int * 3);
				img.flip;
				cv.SaveImage (`test` ~selected_run.text~ `.png`, img);
			}});

			while (not(terminate))
				{/*...}*/
					with (runs[selected_run]) 
					plot (forces.map!(f => f.z).versus (time).stride (100))
						.title (name)
						.x_axis (`time`)
						.y_axis (`vertical force`, interval (0.newtons, 2000.newtons))
						.using (gfx, txt)
						.inside (gfx.extended_bounds[].scale (vector (0.9, 1.0)))
					();
					gfx.render;
					usr.process;
				}

			//pwriteln (gfx.extended_bounds); // BUG wrong, max(x) was 1.77778, now is 1.82476?!
		}
}

auto data_path (DateTime capture_time, string subject, string extension)
	{/*...}*/
		return `./dat/capture_` ~capture_time.toISOString~ `_` ~subject ~ extension;
	}

auto write_to_file (string subject, typeof(Clock.currTime()) capture_time, Seconds capture_duration, void delegate(double) progress_report = null)
	{/*...}*/
		auto time_of_capture = cast(DateTime)capture_time;

		version (LIVE)
			auto path = data_path (time_of_capture, subject, `.dat`);
		else auto path = `mock_data_capture.dat`;

		auto file = File (path, `w`);

		with (daq) {/*write file header}*/
			file.writefln (
				"%s\n"
				"%s\n"
				"subject: %s\n"

				"device: %s #%X\n"
				"\tsampling_frequency: %s\n"
				"\tcapture_frequency: %s\n"
				"\tvoltage_range!`input`: %s\n"
				"\tvoltage_range!`output`: %s\n"
				,

				time_of_capture.date.toSimpleString,
				time_of_capture.timeOfDay.toString,
				subject,

				daq.Model.name, daq.Serial.number,
					sampling_frequency.text,
					capture_frequency.text,
					daq.voltage_range!`input`.text,
					daq.voltage_range!`output`.text,
			);
		}
		with (plate) {/*write data}*/
			auto stream = zip (
				ℕ[0..daq.n_samples_in (capture_duration)], 
				zip(
					force[],
					moment[],
					surface_moment[],
					center_of_pressure[],
					z_torque[],
				)[$-capture_duration..$]
			);
			
			writeln;
			writeln (`writing data to disk...`);
			writeln ('_'.repeat (stream.length / (stream.length/100)), `█ 100%`);

			foreach (i, item; stream)
				{/*...}*/
					if (i % (stream.length/100) == 0)
						{/*...}*/
							if (progress_report)
								progress_report (i*100.0/stream.length);

							write (`▒`), stdout.flush;
						}

					set_abbreviation_mode.torque;

					file.writefln (
						"{%s}\n"
						"force %s\n"
						"moment %s\n"
						"surface_moment %s\n"
						"center_of_pressure %s\n"
						"z_torque %s\n",

						(i/daq.sampling_frequency).text,
						item[0].text,
						item[1].text,
						item[2].text,
						item[3].text,
						item[4].text,
					);
				}

			writeln (`█`);
			writeln (path);
		}
	}

struct Replay
	{/*...}*/
		auto force ()
			{/*...}*/
				return stream_from (&read!(`mem.force`), &measure).at (&frequency)[];
			}
		auto moment ()
			{/*...}*/
				return stream_from (&read!(`mem.moment`), &measure).at (&frequency)[];
			}
		auto surface_moment ()
			{/*...}*/
				return stream_from (&read!(`mem.surface_moment`), &measure).at (&frequency)[];
			}
		auto center_of_pressure ()
			{/*...}*/
				return stream_from (&read!(`mem.center_of_pressure`), &measure).at (&frequency)[];
			}
		auto torque ()
			{/*...}*/
				return stream_from (&read!(`mem.z_torque`), &measure).at (&frequency)[];
			}

		auto force_x ()
			{/*...}*/
				return force.map!(f => f.x);
			}
		auto force_y ()
			{/*...}*/
				return force.map!(f => f.y);
			}
		auto force_z ()
			{/*...}*/
				return force.map!(f => f.z);
			}

		auto moment_x ()
			{/*...}*/
				return moment.map!(f => f.x);
			}
		auto moment_y ()
			{/*...}*/
				return moment.map!(f => f.y);
			}
		auto moment_z ()
			{/*...}*/
				return moment.map!(f => f.z);
			}

		auto surface_moment_x ()
			{/*...}*/
				return surface_moment.map!(f => f.x);
			}
		auto surface_moment_y ()
			{/*...}*/
				return surface_moment.map!(f => f.y);
			}

		auto center_of_pressure_x ()
			{/*...}*/
				return center_of_pressure.map!(f => f.x);
			}
		auto center_of_pressure_y ()
			{/*...}*/
				return center_of_pressure.map!(f => f.y);
			}

		const length ()
			{/*...}*/
				return mem.force.length;
			}
		const measure ()
			{/*...}*/
				return time[$-1] + 1/sampling_frequency;
			}

		struct Memory
			{/*...}*/
				Force[] force;
				Moment[] moment;
				SurfaceMoment[] surface_moment;
				SurfacePosition[] center_of_pressure;
				NewtonMeters[] z_torque;
			}
		Memory mem;

		Hertz sampling_frequency;
		Seconds recording_length;

		const time ()
			{/*...}*/
				return ℕ[0..(sampling_frequency * recording_length).ceil.to!size_t + 1]
					.map!(i => i / sampling_frequency);
			}

		auto frequency ()
			{/*...}*/
				return sampling_frequency;
			}

		auto read (string memory_location)(Seconds t)
			{/*...}*/
				mixin(q{
					return } ~memory_location~ q{[(t * sampling_frequency).to!size_t];
				});
			}

		this (string path)
			{/*...}*/
				import std.file;
				import std.stdio;

				auto lines = readText (path).splitLines.map!strip.array;

				auto extract (T, string name)()
					{/*...}*/
						return lines
							.filter!(line => line.startsWith (name))
							.map!(line => 
								line.contains (`[`)?
								line.find (`[`).findSplitAfter (`]`)[0].to!string
								: line.extract_number
							)
							.map!T
							.array;
					}

				mem.force = extract!(Force, `force`);
				mem.moment = extract!(Moment, `moment`);
				mem.surface_moment = extract!(SurfaceMoment, `surface_moment`);
				mem.center_of_pressure = extract!(SurfacePosition, `center_of_pressure`);
				mem.z_torque = extract!(NewtonMeters, `z_torque`);

				assert (length == mem.moment.length);
				assert (length == mem.surface_moment.length);
				assert (length == mem.center_of_pressure.length);
				assert (length == mem.z_torque.length);

				this.sampling_frequency = extract!(Hertz, `sampling_frequency`)
					.front;
				this.recording_length = lines
					.retro.filter!(line => line.contains (`s}`))
					.front.extract_number
					.to!double.seconds;

				assert (length == time.length, (length-time.length).abs.to!string);
			}
	}
void draw_plate_visualization (P)(P plate, Seconds begin, Seconds end, BoundingBox bounds, Display gfx, Scribe txt, Input usr)
	{/*...}*/
		void draw_plate_visualization ()
			{/*...}*/
				immutable Δx = bounds[].mean;
				immutable force_to_size = 0.005/newton;
				immutable samples = daq.n_samples_in (end - begin) < 1000? 1: daq.sampling_frequency/1000.hertz;

				auto scale = 1/gfx.extended_bounds.top_right;
				auto force_plate_geometry = bounds[].map!identity;

				/* force plate bounds */
				gfx.draw (blue (0.75), chain (
					force_plate_geometry[].roundRobin (force_plate_geometry.scale (.95)),
					force_plate_geometry[0..1],
					force_plate_geometry.scale (.95)[0..1],
				), GeometryMode.t_strip);

				auto cop_size = plate.force[begin..end]
					.stride (samples)
					.mean * force_to_size;
				auto cop_point = plate.center_of_pressure[begin..end]
					.stride (samples)
					.mean.dimensionless * [-0.95*2*bounds.width, 0.85*2*bounds.height]; // BUG sometimes 2,2, sometimes -2,2
				auto cop_angle = atan2 (
					plate.force_y[begin..end].map!dimensionless.mean,
					-plate.force_x[begin..end].map!dimensionless.mean
				);

				if ((cop_point + Δx).not!within (force_plate_geometry.bounding_box))
					return;

				if (cop_size.z > 0.0)
					{/*...}*/
						auto ring_geometry = circle (cop_size.z/75, cop_point + Δx);
						gfx.draw (yellow (0.5 * cop_size.z/cop_size.norm), chain (
							ring_geometry.roundRobin (ring_geometry.scale (0.9)),
							ring_geometry[0..1],
							ring_geometry.scale (0.9)[0..1],
						), GeometryMode.t_strip);
					}

				txt.write (Unicode.arrow[`down`])
					.rotate (cop_angle + π/2)
					.size (txt.available_sizes.reduce!max)
					.color (red (1.5 * cop_size.xy.norm/cop_size.norm))
					.inside (force_plate_geometry)
					.scale (cop_size.xy.norm * 2)
					.align_to (Alignment.center)
					.translate (cop_point)
				();

				auto torque = plate.moment_z[begin..end]
					.stride (daq.sampling_frequency / 1000.hertz)
					.mean * force_to_size / meters;
				if (0)
				txt.write (torque > 0? Unicode.arrow[`cw`]: Unicode.arrow[`ccw`])
					.size (txt.available_sizes.reduce!max)
					.color (green (10 * torque.abs/cop_size.norm))
					.inside (force_plate_geometry)
					.scale (torque.abs * 10)
					.align_to (Alignment.center)
					.translate (cop_point)
				();
			}

		try draw_plate_visualization; // range measures can fail to align in debug mode due to latency
		catch (Throwable ex) writeln (ex.file, `.`, ex.line, `: `, ex.msg);
	}

void draw_plot (Style style = Style.standard, T, U)(T input, string label, Seconds begin, Seconds end, Seconds elapsed, Interval!U range, BoundingBox box, Display gfx, Scribe txt, Input usr)
	{/*...}*/
		auto lap = elapsed;

		plot!style (input[begin..end].versus (ℕ[0..daq.n_samples_in (end-begin)]
				.map!(i => lap + i/daq.sampling_frequency))
			.stride (daq.sampling_frequency / 64.hertz) // TEMP was 256 hertz
		)	.color (green)
			.y_axis (label, range)
			.x_axis (`time`, interval (lap, lap + (end-begin)))
			.inside (box)
			.using (gfx, txt)
		();
	}
void draw_force_plots (P)(P plate, Seconds begin, Seconds end, Seconds elapsed, BoundingBox bounds, Display gfx, Scribe txt, Input usr)
	{/*...}*/
		auto tl = bounds.top_left;
		auto tr = bounds.top_right;
		auto Δy = vec(0, -bounds.height / 2);
		auto δy = vec(0, -bounds.height / 9);

		draw_plot (plate.force_z, `vertical force`, 
			begin, end, elapsed,
			interval (0.newtons, 2000.newtons),
			[tl, tr + Δy].translate (2*δy).bounding_box,
			gfx, txt, usr
		);
		draw_plot (plate.force_y, `fore-aft force`, 
			begin, end, elapsed,
			interval (-1000.newtons, 1000.newtons),
			[tl + Δy, tr + 2*Δy].translate (δy).bounding_box,
			gfx, txt, usr
		);
		draw_plot (plate.force_x, `lateral force`, 
			begin, end, elapsed,
			interval (-1000.newtons, 1000.newtons),
			[tl + 2*Δy, tr + 3*Δy].bounding_box,
			gfx, txt, usr
		);
	}
void draw_plate_signals (P)(P plate, Seconds begin, Seconds end, Seconds elapsed, BoundingBox bounds, Display gfx, Scribe txt, Input usr)
	{/*...}*/
		bounds = bounds[].translate (vec(0,-bounds.height/6)).bounding_box;

		auto tl = bounds.top_left;
		auto tr = bounds.top_right;
		auto tm = (tl+tr) / 2;

		auto Δy = vec(0, -bounds.height / 3);

		if (plate.fx12.is_active)
			draw_plot!(Style.minimal) (plate.fx12, `fx12`, 
				begin, end, elapsed,
				interval (-5.volts, 5.volts),
				[tl + 0*Δy, tm + 1*Δy].bounding_box,
				gfx, txt, usr
			);
		if (plate.fx34.is_active)
			draw_plot!(Style.minimal) (plate.fx34, `fx34`, 
				begin, end, elapsed,
				interval (-5.volts, 5.volts),
				[tl + 1*Δy, tm + 2*Δy].bounding_box,
				gfx, txt, usr
			);
		if (plate.fy14.is_active)
			draw_plot!(Style.minimal) (plate.fy14, `fy14`, 
				begin, end, elapsed,
				interval (-5.volts, 5.volts),
				[tl + 2*Δy, tm + 3*Δy].bounding_box,
				gfx, txt, usr
			);
		if (plate.fy23.is_active)
			draw_plot!(Style.minimal) (plate.fy23, `fy23`, 
				begin, end, elapsed,
				interval (-5.volts, 5.volts),
				[tl + 3*Δy, tm + 4*Δy].bounding_box,
				gfx, txt, usr
			);
		if (plate.fz1.is_active)
			draw_plot!(Style.minimal) (plate.fz1, `fz1`, 
				begin, end, elapsed,
				interval (-5.volts, 5.volts),
				[tm + 0*Δy, tr + 1*Δy].bounding_box,
				gfx, txt, usr
			);
		if (plate.fz2.is_active)
			draw_plot!(Style.minimal) (plate.fz2, `fz2`, 
				begin, end, elapsed,
				interval (-5.volts, 5.volts),
				[tm + 1*Δy, tr + 2*Δy].bounding_box,
				gfx, txt, usr
			);
		if (plate.fz3.is_active)
			draw_plot!(Style.minimal) (plate.fz3, `fz3`, 
				begin, end, elapsed,
				interval (-5.volts, 5.volts),
				[tm + 2*Δy, tr + 3*Δy].bounding_box,
				gfx, txt, usr
			);
		if (plate.fz4.is_active)
			draw_plot!(Style.minimal) (plate.fz4, `fz4`, 
				begin, end, elapsed,
				interval (-5.volts, 5.volts),
				[tm + 3*Δy, tr + 4*Δy].bounding_box,
				gfx, txt, usr
			);
	}

void draw_microphone ()(Seconds begin, Seconds end, Seconds elapsed, BoundingBox bounds, Display gfx, Scribe txt)
	{/*...}*/
		try plot!(Style.minimal) (microphone[begin..end].versus (ℕ[0..daq.n_samples_in (end - begin)]
				.map!(i => elapsed + i/daq.sampling_frequency))
			.stride (daq.sampling_frequency / 256.hertz)
		)	.color (spike (end-begin) > mic_threshold? yellow:grey(0.5))
			.y_axis (`mic`)
			.x_axis (`time`, interval (elapsed, elapsed + end - begin))
			.inside (bounds)
			.using (gfx, txt)
		();
		catch (Throwable ex) ex.msg.writeln (ex.line, ex.file); // TEMP
	}
void display (Seconds timespan, Seconds elapsed, Display gfx, Scribe txt, Input usr)
	{/*...}*/
		if (elapsed < timespan)
			return;

		void draw_plate_visualization ()
			{/*...}*/
				immutable Δx = vector (-1.0, 0.0);
				immutable force_to_size = 0.005/newton;

				auto force_plate_geometry = square[].map!(v => v * vector (500.mm, 600.mm))
					.map!dimensionless.scale (2).translate (Δx);

				gfx.draw (blue (0.75), chain (
					force_plate_geometry.roundRobin (force_plate_geometry.scale (1.05)),
					force_plate_geometry[0..1],
					force_plate_geometry.scale (1.05)[0..1],
				), GeometryMode.t_strip);

				auto cop_size = plate.force[$-10.ms..$]
					.stride (daq.sampling_frequency / 1000.hertz)
					.mean * force_to_size;
				auto cop_point = plate.center_of_pressure[$-10.ms..$]
					.stride (daq.sampling_frequency / 1000.hertz)
					.mean.dimensionless * 2;
				auto cop_angle = atan2 (
					plate.force_y[$-10.ms..$].map!dimensionless.mean,
					-plate.force_x[$-10.ms..$].map!dimensionless.mean
				);

				if ((cop_point + Δx).not!within (force_plate_geometry.bounding_box))
					return;

				if (cop_size.z > 0.0)
					{/*...}*/
						auto ring_geometry = circle (cop_size.z/75, cop_point + Δx);
						gfx.draw (yellow (0.5 * cop_size.z/cop_size.norm), chain (
							ring_geometry.roundRobin (ring_geometry.scale (0.9)),
							ring_geometry[0..1],
							ring_geometry.scale (0.9)[0..1],
						), GeometryMode.t_strip);
					}

				txt.write (Unicode.arrow[`down`])
					.rotate (cop_angle + π/2)
					.size (txt.available_sizes.reduce!max)
					.color (red (1.5 * cop_size.xy.norm/cop_size.norm))
					.inside (force_plate_geometry)
					.scale (cop_size.xy.norm * 2)
					.align_to (Alignment.center)
					.translate (cop_point)
				();

				auto torque = plate.moment_z[$-10.ms..$]
					.stride (daq.sampling_frequency / 1000.hertz)
					.mean * force_to_size / meters;
				txt.write (torque > 0? Unicode.arrow[`cw`]: Unicode.arrow[`ccw`])
					.size (txt.available_sizes.reduce!max)
					.color (green (10 * torque.abs/cop_size.norm))
					.inside (force_plate_geometry)
					.scale (torque.abs * 10)
					.align_to (Alignment.center)
					.translate (cop_point)
				();
			}
		void draw_plot (T)(T input, string label, Interval!Newtons range, BoundingBox bounds)
			{/*...}*/
				auto lap = elapsed;

				plot (input[$-timespan..$].versus (ℕ[0..daq.n_samples_in (timespan)]
						.map!(i => lap + i/daq.sampling_frequency))
					.stride (daq.sampling_frequency / 256.hertz)
				)	.color (green)
					.y_axis (label, range)
					.x_axis (`time`, interval (lap, lap + timespan))
					.inside (bounds)
					.using (gfx, txt)
				();
			}

		{/*draw all}*/
			draw_plot (plate.force_z, `vertical force`, 
				interval (0.newtons, 2000.newtons),
				[vector (0.1, 1.0), vector (1.7, 1./3)].bounding_box
			);
			draw_plot (plate.force_y, `fore-aft force`, 
				interval (-1000.newtons, 1000.newtons),
				[vector (0.1, 1./3), vector (1.7, -1./3)].bounding_box
			);
			draw_plot (plate.force_x, `lateral force`, 
				interval (-1000.newtons, 1000.newtons),
				[vector (0.1, -1./3), vector (1.7, -1.0)].bounding_box
			);

			try draw_plate_visualization; // range measures can fail to align in debug mode due to latency
			catch (Throwable ex) writeln (ex.file, ex.line, ex.msg);

			version (microphone_enabled)
				draw_microphone (daq.recording_length - timespan, daq.recording_length, elapsed, gfx.extended_bounds, gfx, txt);

			gfx.render;
			usr.process;
		}
	}

void display_waiting (string reason, Display gfx, Scribe txt, Input usr)
	{/*...}*/
		static float t = 0.0;
		t += 0.05;

		txt.write (`waiting for ` ~reason~ `...`)
			.align_to (Alignment.center)
			.size (txt.font_sizes[].reduce!max)
			.scale (0.5)
			.color (red (0.2 + 0.8 * sin (t).abs))
		();

		gfx.render;
		usr.process;
	}

void download_video (string subject, typeof(Clock.currTime()) capture_time, void delegate(double) progress_report = null)
	{/*...}*/
		enum dl_progress = `./vid/download_progress`;

		while (read_ipc_flag (dl_progress) == false)
			Thread.sleep (250.msecs);

		while (1)
			{/*...}*/
				double progress = 0.0;

				if (progress_report)
					progress_report (0.0);
				
				while (1)
					{/*...}*/
						try progress = File (dl_progress, "r").byLine.front.to!string.to!double;
						catch (Exception) continue;
						break;
					}

				if (progress_report && progress <= 100 && progress >= 0)
					progress_report (progress);

				if (progress >= 100)
					break;

				Thread.sleep (250.msecs);
			}

		write_ipc_flag (dl_progress, false);

		`./vid/output.avi`.rename (data_path (cast(DateTime)capture_time, subject, `.avi`));
	}

struct CameraSettings
	{/*...}*/
		uvec resolution;

		Hertz framerate;

		Seconds pretrigger, posttrigger;
	}

version (none)
	void main (string[] args)
		{/*...}*/
			write_ipc_flag ("./vid/terminated", false);
			write_ipc_flag ("./vid/recording_finished", false);
			write_ipc_flag ("./vid/camera_ready", false);

			auto capture_history = 1.seconds;
			auto pre_trigger_time = capture_history/2;
			auto post_trigger_time = capture_history/2;

			File (`./vid/camera_settings`, `w`).write (CameraSettings (uvec (800,600), 200.hertz, capture_history));

			__gshared bool triggered, terminated, draw_frame, save, manual_trigger;

			version (camera_enabled)
				spawn ((){executeShell (`./vcap.sh`).output.writeln; terminated = true;});

			auto elapsed = 0.seconds;
			typeof(Clock.currTime()) trigger_time;

			//////

			scope gfx = new Display (1920, 1080);
			gfx.start; scope (exit) gfx.stop;
			scope txt = new Scribe (gfx, [14, 144]);
			scope usr = new Input (gfx, (bool){terminated = true;});
			usr.bind (Input.Key.space, (bool){manual_trigger = true;});

			//////

			auto sleep_frame ()
				{/*...}*/
					Thread.sleep ((1/daq.capture_frequency).to_duration);
				}
			void await_camera ()
				{/*...}*/
					await_ipc_flag ("./vid/camera_ready",
						(){/*...}*/
							display_waiting (`camera`, gfx, txt, usr);
							sleep_frame;

							return not!terminated;
						}
					);
				}
			auto save_data (string subject)
				{/*...}*/
					(cast(shared)daq).send_digital_pulse;

					daq.stop;

					write_ipc_flag ("./vid/recording_finished", true);

					auto capture_time = Clock.currTime ();

					write_to_file (subject, capture_time, capture_history,
						(double pct) {/*...}*/
							txt.write (`writing to file ` ~pct.to!string~ `%`)
								.color (red)
								.size (txt.font_sizes[].reduce!max)
								.scale (0.5)
								.align_to (Alignment.center)
							();
							gfx.render;
						}
					);

					version (camera_enabled)
						download_video (subject, capture_time,
							(double pct) {/*...}*/
								auto pct_str = pct.to!string;

								txt.write (`downloading video ` ~(pct_str.length == 2? pct_str: ` `~pct_str)~ `%`)
									.color (red)
									.size (txt.font_sizes[].reduce!max)
									.scale (0.5)
									.align_to (Alignment.center)
								();
								gfx.render;
							}
						);
				}

			//////

			auto subject = `test`;

			version (LIVE)
			if (subject.empty)
				subject = `test`;

			//////

			daq.on_capture = (size_t)
				{/*...}*/
					elapsed += 1/daq.capture_frequency;

					if (elapsed < pre_trigger_time)
						return;

					version (microphone_enabled)
						auto trigger = false;//spike (4/daq.capture_frequency) > mic_threshold;TEMP 
					else auto trigger = false;

					trigger = trigger || manual_trigger;

					if (not!triggered && trigger)
						{/*...}*/
							trigger_time = Clock.currTime;
							triggered = true;
							draw_frame = true;
							manual_trigger = false;
						}
					else if (triggered && Clock.currTime - trigger_time > post_trigger_time.to_duration)
						{/*...}*/
							save = true;
							triggered = false;
							manual_trigger = false;
						}
					else draw_frame = true;
				};

			scope (exit) write_ipc_flag (`./vid/terminated`, true);

			while (not!terminated)
				{/*main loop}*/
					manual_trigger = triggered = terminated = draw_frame = save = false;

					version (camera_enabled)
						await_camera;


					daq.start;
					elapsed = 0.seconds;

					while (not!terminated)
						{/*...}*/
							while (elapsed <= max (pre_trigger_time, capture_history))
								{/*...}*/
									display_waiting (`buffer`, gfx, txt, usr);
									sleep_frame;
								}

							if (draw_frame && daq.is_streaming)
								{/*...}*/
									draw_frame = false;

									auto bounds = gfx.extended_bounds;
									auto length = daq.recording_length;
							//		draw_plate_visualization (plate, length - 10.ms, length, bounds[].scale (1.0/3).translate (2*bounds.top_right/3).bounding_box, gfx, txt, usr);
									draw_plate_signals (plate, length - 0.5.seconds, length, elapsed, bounds[].scale (vec (1, 0.5)).bounding_box, gfx, txt, usr);
									version (microphone_enabled)
										draw_microphone (length - 200.ms, length, elapsed, bounds[].scale (vec (0.5,0.3)).translate (vec(-bounds.width/4,bounds.height/3)).bounding_box, gfx, txt);

									gfx.render;
									usr.process;
								}
							if (save)
								{/*...}*/
									save = false;

									save_data (subject);

									break;
								}
							else sleep_frame;
						}
				}

			daq.stop;
		}
else version (none)
	void main () {analysis;}
else version (all)
	void main ()
		{/*replay}*/
			import std.file;
			import std.stdio;
			alias copy = std.algorithm.copy;

			bool terminated;
			bool paused;
			bool change_video;

			scope gfx = new Display (1000, 1000);
				gfx.start; scope (exit) gfx.stop;
			scope txt = new Scribe (gfx, [14, 144]);
			scope usr = new Input (gfx, (bool){terminated = true;});
				usr.bind (Input.Key.space, (bool on){if (on) paused = not (paused);});

			auto corresponding_video (string path)
				{/*...}*/
					return `/home/vlevenfeld3/fcap` ~path[1..$-3]~`avi`;
				}

			auto paths = dirEntries (`./dat/`, SpanMode.shallow)
				.map!(entry => entry.name)
				.filter!(path => path.endsWith (`dat`))
				.filter!(path => (corresponding_video (path).exists))
				.array;

			scope frame = Image (gfx.dimensions[].map!(to!int).vector!2.to!CvSize);

			auto i = 0;
			while (1)
				{/*...}*/
					enum sleep = 32.ms;
					auto timepoint = 0.seconds;
					auto width = 1.ms;

					auto replay = Replay (paths[i]);
					replay.mem.center_of_pressure[].map!(v => v * vec(-1,1)).copy (replay.mem.center_of_pressure[]);

					usr.bind (Input.Key.n_plus, (bool on){if (on) width *= 2;});
					usr.bind (Input.Key.n_minus, (bool on){if (on && width/2 > 2/replay.sampling_frequency) width /= 2;});
					usr.bind (Input.Key.n, (bool on){if (on) i = min (i+1, paths.length-1); change_video = true;});
					usr.bind (Input.Key.b, (bool on){if (on) i = max (i-1, 0); change_video = true;});

					scope video = Video.Output (`replay` ~paths[i].find (`_`).until (`.dat`).to!string~ `.avi`, gfx.dimensions[].map!(to!int).vector!2.to!CvSize);
					scope invideo = Video.Input (`./dat/test.avi`);

					while (timepoint < replay.measure)
						{/*...}*/
							import core.thread;

							auto video_box = gfx.extended_bounds[]
								.scale (0.5)
								.bounding_box
								.align_to (Alignment.top_left, gfx.extended_bounds.top_left);
							
							gfx.draw (red*orange, video_box[], GeometryMode.t_fan);

							txt.write (`no video`)
								.inside (video_box)
								.align_to (Alignment.center)
								.size (144)
								.color (black)
							();

							draw_plate_visualization (
								replay, timepoint, timepoint + width, 
								[-0.5.vec, 0.5.vec].bounding_box.align_to (Alignment.top_right, gfx.extended_bounds.top_right),
								gfx, txt, usr
							);

							draw_force_plots (
								replay, 0.ms, replay.measure, 0.ms,
								[-vec(1, 0.65), vec (1, 0.2)].bounding_box,
								gfx, txt, usr
							);

							auto elapsed_x = (2 * (timepoint/replay.measure) - 1.0) * 0.9 + 0.06;
							gfx.draw ((white*green)(0.5), [vec(elapsed_x, 0), vec(elapsed_x, -1)], GeometryMode.l_strip);

							txt.write (paths[i], ` [`, i+1, `/`, paths.length, `]`)
								.color (white)
								.align_to (Alignment.top_right)
							();
							txt.write (width/sleep, `x speed`)
								.color (yellow)
								.inside (gfx.extended_bounds[].scale (vec(0.85, 0.95)))
								.align_to (Alignment.top_right)
							();
							txt.write (timepoint)
								.color (paused? red: green)
								.inside (gfx.extended_bounds[].scale (vec(0.85, 0.9)))
								.align_to (Alignment.top_right)
							();

							txt.write (paused? 
								Unicode.block[`3/8`].repeat (2).to!string
								: Unicode.arrow[`head`].to!string
							)
								.color (paused? red: green)
								.inside (gfx.extended_bounds[].scale (vec(0.98, 0.95)))
								.align_to (Alignment.top_right)
								.size (144)
								.scale (paused? 0.25: 0.4)
							();

							gfx.render;
							usr.process;
							//Thread.sleep (sleep.to_duration);

							scope img_data = new void[gfx.dimensions[].product.to!size_t * 3];
							gfx.access_rendering_context ((){
								gl.ReadPixels (0, 0, gfx.dimensions.x.to!int, gfx.dimensions.y.to!int, PixelFormat.bgr, PixelFormat.unsigned_byte, img_data.ptr);
							});

							cv.SetData (frame, img_data.ptr, gfx.dimensions.x.to!int * 3);
							video.put (frame);

							if (terminated)
								break;

							if (not!paused)
								timepoint += width;

							timepoint += usr.keys_pressed ([Input.Key.left, Input.Key.right])
								.zip ([-width, +width])
								.map!((pressed, increase) => pressed? increase: 0.ms)
								.sum;

							//timepoint = max (0.ms, min (timepoint, replay.measure - 1/daq.sampling_frequency));

							if (change_video)
								{/*...}*/
									change_video = false; 
									break;
								}
						}

					if (terminated)
						break;
				}
		}

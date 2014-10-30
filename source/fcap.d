module fcap.main;

import fcap.units;
import fcap.daq;
import fcap.plate;

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
	alias seconds = evx.units.seconds;
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

	auto microphone ()
		{/*...}*/
			return daq.input[0].sample_by_time;
		}
	auto spike (Seconds timespan)
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
				reference_mode = ReferenceMode.single_ended;
			}
			with (plate) {/*input signals}*/
				voltage_to_force = vector (1250.newtons/5.volts, 1250.newtons/5.volts, 2500.newtons/5.volts);
				/* source: check the signal conditioner settings TODO maybe automate this with opencv and a webcam */

				sensor_offset = vector (210.mm, 260.mm, -41.mm);
				/* source: Kistler Type 9260AA Instruction Manual, p.33 */

				fx12 = daq.input[1] .sample_by_time;
				fx34 = daq.input[9] .sample_by_time;
				fy14 = daq.input[2] .sample_by_time;
				fy23 = daq.input[10].sample_by_time;
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

			daq.open_channels!`input` (0, 1,2,3,4, 9,10,11,12);
		}

	shared static ~this ()
		{/*...}*/
			
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

			auto directory = `dat`.dirEntries (SpanMode.breadth).array;
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

			while (not(terminate))
				{/*...}*/
					with (runs[selected_run]) 
					plot (forces.map!(f => f.y).versus (time).stride (400))
						.title (name)
						.x_axis (`time`)
						.y_axis (`fore-aft force`)
						.using (gfx, txt)
						.inside (gfx.extended_bounds[].scale (vector (0.9, 1.0)))
					();
					gfx.render;
					usr.process;
				}

			pwriteln (gfx.extended_bounds); // BUG wrong, max(x) was 1.77778, now is 1.82476?!
		}
}
public {/*interprocess communication}*/
	struct CameraSettings
		{/*...}*/
			Vector!(2,int) resolution;

			Hertz framerate;

			Seconds pretrigger_length;
			Seconds posttrigger_length;

			void write ()
				{/*...}*/
					auto file = File (`./vid/settings`, "w");

					file.write (this);
				}
		}
	bool read_shared_flag (string flag_name)
		{/*...}*/
			return exists ("./vid/" ~flag_name);
		}
	void write_shared_flag (string flag_name, bool value)
		{/*...}*/
			writeln (`writing `, flag_name, ` = `, value);

			auto path = "./vid/" ~ flag_name;

			if (value)
				File (path, "w").write ("1");
			else if (path.exists)
				remove (path);
		}
	void await_shared_flag (string flag_name)
		{/*...}*/
			writeln (`awaiting `, flag_name);
			
			while (read_shared_flag (flag_name) == false)
				{}
			write_shared_flag (flag_name, false);
		}
}

auto write_to_file (string subject, typeof(Clock.currTime()) capture_time, Seconds capture_duration, void delegate(double) progress_report = null)
	{/*...}*/
		auto time_of_capture = cast(DateTime)capture_time;

		version (LIVE)
			auto path = `./dat/capture_` ~time_of_capture.toISOString~ `_` ~subject~ `.dat`;
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

void display (Seconds timespan, Seconds elapsed, Display gfx, Scribe txt, Input usr)
	{/*...}*/
		if (elapsed < 50.ms)
			return;

		void draw_microphone ()
			{/*...}*/
				try plot (microphone[$-timespan..$].versus (ℕ[0..daq.n_samples_in (timespan)]
						.map!(i => elapsed + i/daq.sampling_frequency))
					.stride (daq.sampling_frequency / 500.hertz)
				)	.color (spike (timespan) > 0.75.volts? yellow:grey(0.5))
					.y_axis (`microphone`)
					.x_axis (`time`, interval (elapsed, elapsed + timespan))
					.inside ([vector (-1.5, 1), vector (-0.5, 0.6)])
					.using (gfx, txt)
				();
				catch (Throwable ex) ex.msg.pl (ex.line, ex.file); // TEMP
			}
		void draw_cop_vector ()
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

				auto cop_size = plate.force[$-50.ms..$]
					.stride (daq.sampling_frequency / 2000.hertz)
					.mean * force_to_size;
				auto cop_point = plate.center_of_pressure[$-50.ms..$]
					.stride (daq.sampling_frequency / 2000.hertz)
					.mean.dimensionless * [-2,2];
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

				auto torque = plate.moment_z[$-50.ms..$]
					.stride (daq.sampling_frequency / 2000.hertz)
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

			try draw_cop_vector; // range measures can fail to align in debug mode due to latency
			catch (Throwable ex) pl (ex.file, ex.line, ex.msg);

			if (daq.input[0].is_open)
				draw_microphone;

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

void main (string[] args)
	{/*...}*/
		assert (args.length > 1, `specify subject name in command line`);

		enum pre_trigger_time = 0.5.seconds;
		enum post_trigger_time = 0.5.seconds;
		enum capture_history = 1.second;

		CameraSettings (vector (800, 600), 200.hertz, pre_trigger_time, post_trigger_time)
			.write;

		bool triggered, terminated, draw_frame, save;

		auto elapsed = 0.seconds;
		typeof(Clock.currTime()) trigger_time;

		//////

		scope gfx = new Display (1920, 1080);
		gfx.start; scope (exit) gfx.stop;
		scope txt = new Scribe (gfx, [14, 144]);
		scope usr = new Input (gfx, (bool){terminated = true;});

		//////

		auto sleep_frame ()
			{/*...}*/
				Thread.sleep ((1/daq.capture_frequency).to_duration);
			}
		void await_camera ()
			{/*...}*/
				while (read_shared_flag ("camera_ready") == false)
					{/*...}*/
						display_waiting (`camera`, gfx, txt, usr);
						sleep_frame;
					}
				write_shared_flag (`camera_ready`, false);
			}
		auto save_data (string subject)
			{/*...}*/
				(cast(shared)daq).send_digital_pulse;

				daq.stop;

				triggered = false;

				write_shared_flag ("recording_finished", true);

				write_to_file (subject, Clock.currTime (), capture_history,
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
			}

		//////

		//write (`enter subject name: `);
		auto subject = args[1];

		version (LIVE)
		if (subject.empty)
			subject = `test`;

		//////

		daq.on_capture = (size_t)
			{/*...}*/
				elapsed += 1/daq.capture_frequency;

				if (elapsed < pre_trigger_time)
					return;

				if (not!triggered && spike (4/daq.capture_frequency) > 0.75.volts)
					{/*...}*/
						trigger_time = Clock.currTime;
						triggered = true;
						draw_frame = true;
					}
				else if (triggered && Clock.currTime - trigger_time > post_trigger_time.to_duration)
					{/*...}*/
						save = true;
					}
				else draw_frame = true;
			};

		await_camera;
		daq.start;

		while (not!terminated)
			{/*main loop}*/
				while (elapsed <= max (pre_trigger_time, capture_history))
					{/*...}*/
						display_waiting (`buffer`, gfx, txt, usr);
						sleep_frame;
					}

				if (draw_frame && daq.is_streaming)
					{/*...}*/
						draw_frame = false;

						display (capture_history, elapsed, gfx, txt, usr);
					}
				else if (save)
					{/*...}*/
						save = false;

						save_data (subject);

						break;
					}
				else sleep_frame;
			}

		daq.stop;
		(cast(shared)daq).power_cycle;
		write_shared_flag ("terminate", true);
	}

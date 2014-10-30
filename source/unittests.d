import fcap.main;
import fcap.daq;
import fcap.plate;
import evx.math;

alias mm = millimeters;

void verify_channel_separation (size_t n_channels, Args...)(Args open_channels)
	{/*...}*/
		scope daq = new DAQDevice!(
			Model!`NI USB-6216`,
			Serial!0x_18DEF36,

			InputChannels!n_channels,
			OutputChannels!0,

			InputBufferSize!4095,
			OutputBufferSize!0,

			MaxSamplingRate!(size_t.max),
			MaxInputVoltageRange!(-10, 10),
			MaxOutputVoltageRange!(-10, 10),
		);
		
		DAQmx.mock_channel = null;
		scope (exit) DAQmx.mock_channel = null;

		daq.sampling_frequency = 100.hertz;
		daq.open_channels!`input` (open_channels);

		daq.record_for (1.second);

		foreach (i, channel; enumerate (daq.input).filter!((i, ch) => ch.is_open))
			{/*...}*/
				auto err_msg (Volts sample, size_t j)
					{/*...}*/
						string channels;

						foreach (arg; open_channels)
							channels ~= arg.text~ `, `;

						channels = channels[0..$-2];

						return `expected ` ~i.volts.text~ ` at [` ~j.text~ `], got ` ~sample.text~ ` (` ~typeof(daq).InputChannels.count.text~ ` channels, ` ~channels~ ` active):`"\n"
						~channel.sample_by_time[].text;
					}

				foreach (j, sample; enumerate (daq.input[i].sample_by_index[]))
					assert (sample == i.volts, err_msg (sample, j));

				foreach (j, sample; enumerate (daq.input[i].sample_by_time[]))
					assert (sample == i.volts, err_msg (sample, j));
			}

		daq.close_channels!`input`;
	}

unittest {/*}*/
	auto daq = new DAQDevice!(
		Model!`NI USB-6216`,
		Serial!0x_18DEF36,

		InputChannels!8,
		OutputChannels!2,

		InputBufferSize!4095,
		OutputBufferSize!8191,

		MaxSamplingRate!(250_000),
		MaxInputVoltageRange!(-10, 10),
		MaxOutputVoltageRange!(-10, 10),
	); /* source: http://www.ni.com/pdf/manuals/371932f.pdf */

	ForcePlate plate;

	with (daq) {/*settings}*/
		sampling_frequency = 30.kilohertz;
		capture_frequency = 60.hertz;
		history_length = 5.seconds;

		daq.voltage_range!`input` = interval (-5.volts, 5.volts);
	}
	with (plate) {/*signals}*/
		immutable TEMP = 1.0;
		voltage_to_force = vector (TEMP.newtons/volt, TEMP.newtons/volt, TEMP.newtons/volt);
		/* source: TODO */

		sensor_offset = vector (210.mm, 260.mm, -41.mm);
		/* source: Kistler Type 9260AA Instruction Manual, p.33 */

		fx12 = daq.input[0].sample_by_time;
		fx34 = daq.input[1].sample_by_time;
		fy14 = daq.input[2].sample_by_time;
		fy23 = daq.input[3].sample_by_time;
		fz1  = daq.input[4].sample_by_time;
		fz2  = daq.input[5].sample_by_time;
		fz3  = daq.input[6].sample_by_time;
		fz4  = daq.input[7].sample_by_time;
		/* source: check the wires */
	}

	{/*killswitch}*/
		daq.open_channel!`input` (0);
		daq.start;
		daq.stop;
		(cast(shared)daq).power_cycle;
		import core.thread;
		Thread.sleep (1000.msecs);
		daq.start;
		daq.stop;
	}

	version (MOCK_DATA) {/*...}*/
		{/*8 inputs, constant signals}*/
			daq.open_channels!`input` (0,1,2,3,4,5,6,7);

			daq.record_for (1.second);

			// TODO extract method
			{/*channel stream_separation}*/
				foreach (i; 0.. 8)
					{/*...}*/
						assert (daq.input[i].sample_at_time (daq.recording_length - (1/30_000.).seconds) == i.volts);
						assert (daq.input[i].sample_at_time (0.seconds) == i.volts);
						assert (daq.input[i].sample_at_index (0) == i.volts);
						assert (daq.input[i].sample_at_index (daq.sample_count - 1) == i.volts);
					}
			}
			{/*index ↔ time}*/
				foreach (i; 0..daq.sample_count)
					assert (daq.index_at_time (i/daq.sampling_frequency) == i);
			}
			{/*range mapping}*/
				import std.range: equal;

				auto stream = daq.input[5].sample_by_time[$-0.0005.seconds..$]; // OUTSIDE BUG dmd segfault
	static if (0)
				auto mapped = stream.map!(x => x);
	static if (0)
				auto mapped_stream = daq.input[5].sample_by_time[$-0.0005.seconds..$].map!(x => x);

	static if (0)
				assert (stream.equal (mapped));
	static if (0)
				assert (mapped.equal (mapped_stream));
	static if (0)
				assert (mapped_stream.equal (stream));
			}
	static if (0)
			{/*range reduction}*/
				import evx.statistics;

				assert (plate.fx34[0.seconds] == plate.fx34[0.seconds..0.5.seconds].mean);
				assert (plate.force_x[0.seconds] == plate.force_x[0.seconds..0.5.seconds].mean);
				assert (plate.force[0.seconds] == plate.force[0.seconds..0.5.seconds].mean);
				assert (plate.moment[0.seconds].approx (plate.moment[0.seconds..0.5.seconds].mean));
				assert (plate.center_of_pressure[0.seconds].approx (plate.center_of_pressure[0.seconds..0.5.seconds].mean));

				assert (plate.fx34[0.seconds..1.second].std_dev == 0.volts);
				assert (plate.force_x[0.seconds..1.second].std_dev == 0.newtons);
				assert (plate.force[0.seconds..1.second].std_dev == vector!3 (0.newtons));
				assert (plate.moment[0.seconds..1.second].std_dev.approx (vector!3 (0.newton*meters)));
				assert (plate.center_of_pressure[0.seconds..1.second].std_dev.approx (vector!2 (0.meters)));
			}
	static if (0)
			{/*time-slice propagation}*/
				import std.range: walkLength;

				auto t = daq.input[0].sample_by_time[0.seconds..0.001.seconds].measure;
				auto n = daq.input[0].sample_by_time[0.seconds..0.001.seconds].walkLength;

				foreach (channel; daq.input)
					{/*...}*/
						auto S = channel.sample_by_time[0.seconds..0.001.seconds];
						assert (S.walkLength == n && S.measure == t);
					}

				assert (plate.force_x[0.seconds..0.001.seconds].walkLength == n);
				assert (plate.force_x[0.seconds..0.001.seconds].measure == t);

				assert (plate.moment[0.seconds..0.001.seconds].walkLength == n);
				assert (plate.moment[0.seconds..0.001.seconds].measure == t);
			}

			daq.close_channels!`input`;
			DAQmx.mock_channel = null;
		}
	static if (0)
		{/*2 inputs, sinewaves of opposite sign}*/
			daq.open_channel!`input` (0);
			daq.open_channel!`input` (1);

			DAQmx.mock_channel[0] = (uint i) => cast(double) sin (2*π*i/100f);
			DAQmx.mock_channel[1] = (uint i) => cast(double) -sin (2*π*i/100f);

			daq.record_for (1.second);

			{/*slicing and sampling consistency}*/
				import std.range: equal, walkLength;
				import evx.statistics;

				assert (plate.force_x[0.seconds..$].mean == 0.newtons);

				assert (plate.force_x[].walkLength == daq.sample_count);

				auto t = daq.recording_length;
				assert (plate.force_x[$-t] == plate.force_x[0.seconds]);
				assert (plate.force_x[$/2] == plate.force_x[t/2]);

				assert (plate.force_x[$/2..$].walkLength == plate.force_x[0.seconds..$/2].walkLength);

				assert (daq.input[0].sample_by_index[0..$/2].walkLength == daq.input[0].sample_by_time[0.seconds..$/2].walkLength);
				assert (daq.input[0].sample_by_index[].walkLength == daq.input[0].sample_by_time[].walkLength);

				assert (daq.input[0].sample_by_index[$/2] == daq.input[0].sample_by_time[][$/2]);

				assert (daq.input[0].sample_by_index[0..$/2][$/2] == daq.input[0].sample_by_time[][$/4]);

				foreach (sample; zip (daq.input[0].sample_by_time[], daq.input[0].sample_by_index[]))
					assert (sample[0] == sample[1]);

				foreach (sample; zip (daq.input[0].sample_by_time[$/2..$], daq.input[0].sample_by_index[$/2..$]))
					assert (sample[0] == sample[1]);
			}

			daq.close_channels!`input`;
			DAQmx.mock_channel.clear;
		}
	static if (0)
		{/*1 input, 2 outputs, "live" stream processing}*/
			daq.open_channel!`input`(7);
			daq.open_channels!`output`(0,1);

			DAQmx.mock_channel[7] = i => 3*i/daq.sampling_frequency.to!double;

			daq.output[0].generate (t => t*volts/second);
			daq.output[1].generate (t => 2*t*volts/second).over_period (infinity.seconds);


			bool update_check;
			daq.on_capture = (uint n)
				{/*...}*/
					with (daq) foreach (i; 1..n+1)
						assert (input[7].sample_by_index[$-i].approx (output[0].sample_by_index[$-i] + output[1].sample_by_index[$-i]));

					update_check = true;
				};

			daq.record_for (500.milliseconds);

			assert (update_check);

			daq.on_capture = null;
			DAQmx.mock_channel.clear;
		}
	static if (0)
		{/*1 input, 1 output, ramp signal}*/
			daq.open_channel!`output` (0);
			daq.open_channel!`input` (0);

			daq.sampling_frequency = 60.hertz;

			alias signal = λ!(t => t*volts/second);

			DAQmx.mock_channel[0] = i => signal (i/daq.sampling_frequency).to!double;

			daq.output[0].generate (signal!Seconds);

			daq.record_for (1.second);

			{/*signal equivalence}*/
				foreach (sample; zip (daq.input[0].sample_by_time[], daq.output[0].sample_by_index[]))
					assert (sample[0].approx (sample[1]));
			}

			daq.close_channels!`input`;
			daq.close_channels!`output`;
			DAQmx.mock_channel.clear;
		}
	static if (0)
		{/*2 inputs, 1 output, ramp signals}*/
			template to_volts (double s)
				{/*...}*/
					auto to_volts = (Seconds t) => s*t*volts/second;
				}

			DAQmx.mock_channel[0] = i => to_volts!1.0 (i/daq.sampling_frequency).to!double;
			DAQmx.mock_channel[1] = i => to_volts!2.0 (i/daq.sampling_frequency).to!double;

			daq.open_channels!`input` (0,1);
			daq.open_channels!`output` (1);

			daq.output[1].generate (to_volts!3.0)
				.over_period (250.milliseconds);

			daq.record_for (1.second);

			{/*signal period alignment}*/
				auto together = zip (
					daq.output[1].sample_by_time[].map!(v => v*newtons/volt),
					plate.force_x				[] // because input[0] + input[1] == plate.force_x and 1.0 + 2.0 == 3.0
				);

				foreach (item; together[0.ms..250.ms]) 
					assert (item[0].approx (item[1]));

				foreach (item; together[250.ms..$]) 
					assert (not (item[0].approx (item[1])));

				auto staggered = zip (
					daq.output[1].sample_by_time[250.ms..500.ms].map!(v => v*newtons/volt),
					plate.force_x				[0.ms..250.ms]
				);

				foreach (item; staggered) 
					assert (item[0].approx (item[1]));
			}

			DAQmx.mock_channel.clear;
		}
	static if (0)
		{/*1 input, streaming to file}*/
			import std.file: remove;
			
			daq.open_channel!`input` (0);
			daq.sampling_frequency = 10000.hertz;

			DAQmx.mock_channel[0] = i => sin(1.0*i);

			auto file = File (`mock_data.dat`, `w`);
			scope (exit) remove (`mock_data.dat`);

			daq.on_capture = (uint n)
				{file.writeln (daq.input[0].sample_by_index[$-n..$]);};

			daq.record_for (1.second);

			file = File (`mock_data.dat`, `r`);

			assert (file.byLine.front == daq.input[0].sample_by_index[0..daq.n_samples_in (1/daq.capture_frequency)].text);

			DAQmx.mock_channel.clear;
		}
	}
}

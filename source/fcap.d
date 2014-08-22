module fcap;

private {/*import std}*/
	import std.typetuple:
		allSatisfy,
		TypeTuple, Filter;

	import std.functional:
		toDelegate;

	import std.range:
		drop, repeat,
		retro, stride,
		empty;

	import std.algorithm:
		min, max,
		find, canFind, findSplitBefore;

	import std.datetime:
		SysTime;

	import std.variant:
		Algebraic, visit;

	import std.math:
		floor, ceil, round;

	import std.c.stdlib:
		malloc, free;

	import std.concurrency:
		spawn, Tid, ownerTid,
		send, received_before = receiveTimeout; 
	
	import std.stdio:
		stderr, File, write;
	
	import std.conv:
		to, text;

	import std.process:
		pipeShell;

	import std.string:
		capitalize;

	static import std.datetime;
}
private {/*import evx}*/
	import evx.traits: 
		has_trait;

	import evx.meta: 
		Builder;

	import evx.functional:
		λ, map, zip, reduce;

	import evx.logic:
		Or;

	import evx.algebra:
		zero, unity;

	import evx.analysis:
		between,
		Interval, interval,
		is_contained_in,
		infinite, infinity, is_infinite;

	import evx.arithmetic:
		add, divide, sum;

	import evx.range:
		slice_within_bounds;

	import evx.utils:
		Index;

	import evx.service;
	import evx.streams;
	import evx.vectors;
	import evx.units;
}
private {/*import nidaqmx}*/
	import nidaqmx;
}

public {/*units}*/
	alias vec 				= Vector!(3, double);
	alias Position 			= Vector!(3, Meters);
	alias SurfacePosition	= Vector!(2, Meters);
	alias Force 			= Vector!(3, Newtons);
	alias Moment 			= Vector!(3, NewtonMeters);
	alias SurfaceMoment		= Vector!(2, NewtonMeters);

	alias mm = millimeters;
	alias ms = milliseconds;
}

final class DAQDevice (Specs...): Service
	{/*...}*/
		__gshared:
		static {/*assertions}*/
			mixin template check (string specification)
				{/*...}*/
					static assert (Filter!(has_trait!(`is` ~specification),	Specs).length > 0, specification~ " missing from DAQDevice declaration");
					static assert (Filter!(has_trait!(`is` ~specification),	Specs).length < 2, specification~ " ambiguous in DAQDevice declaration");
				}

			mixin check!q{Model};
			mixin check!q{InputChannels};
			mixin check!q{OutputChannels};
			mixin check!q{MaxSamplingRate};
			mixin check!q{MaxInputVoltageRange};
			mixin check!q{MaxOutputVoltageRange};
			mixin check!q{Serial};
			mixin check!q{InputBufferSize};
			mixin check!q{OutputBufferSize};
		}
		static {/*specs}*/
			template extract (string specification)
				{/*...}*/
					mixin(q{
						alias } ~specification~ q{ = Filter!(has_trait!(`is` ~specification), Specs)[0];
					});
				}

			mixin extract!q{Model};
			mixin extract!q{InputChannels};
			mixin extract!q{OutputChannels};
			mixin extract!q{MaxSamplingRate};
			mixin extract!q{MaxInputVoltageRange};
			mixin extract!q{MaxOutputVoltageRange};
			mixin extract!q{Serial};
			mixin extract!q{InputBufferSize};
			mixin extract!q{OutputBufferSize};
		}

		public:
		const @property {/*status}*/
			bool is_ready ()
				{/*...}*/
					if (parameters_invalidated)
						return false;

					if ((sampling_frequency * capture_frequency).to_scalar == 0)
						return false;

					if (not (buffer.is_ready))
						return false;

					return true;
				}
			bool is_streaming ()
				{/*...}*/
					return this.is_running;
				}
		}
		const @property {/*counters}*/
			Index sample_count ()
				{/*...}*/
					return buffer.length / count_open!`input`;
				}
				
			Seconds recording_length ()
				out (result) {/*...}*/
					if (buffer.filled)
						assert (result == history_length,
							`buffer filled, but recording length (` ~result.text~ `) doesn't match history length (` ~history_length.text~ `)`
						);
					else assert (result < history_length,
						`recording length (` ~result.text~ `) exceeded history length (` ~history_length.text~ `)`
					);
				}
				body {/*...}*/
					return sample_count / sampling_frequency;
				}

			int count_open (string channel_type)()
				{/*...}*/
					auto n = 0;

					foreach (channel; mixin(channel_type~ q{_channels}))
						if (channel.is_open)
							++n;

					return n;
				}
		}
		@property {/*parameters}*/
			Hertz sampling_frequency () const
				{/*...}*/
					return _sampling_frequency;
				}
			void sampling_frequency (Hertz frequency)
				{/*...}*/
					_sampling_frequency = frequency;
					
					invalidate_parameters;
				}

			Hertz capture_frequency () const
				{/*...}*/
					return _capture_frequency;
				}
			void capture_frequency (Hertz frequency)
				{/*...}*/
					_capture_frequency = frequency;

					invalidate_parameters;
				}

			Hertz generating_frequency () const
				{/*...}*/
					return _generating_frequency;
				}
			void generating_frequency (Hertz frequency)
				{/*...}*/
					_generating_frequency = frequency;

					invalidate_parameters;
				}

			Seconds generating_period () const
				{/*...}*/
					return _generating_period;
				}
			void generating_period (Seconds period)
				{/*...}*/
					_generating_period = period;

					invalidate_parameters;
				}

			Seconds history_length () const
				out (result) {/*...}*/
					assert (result >= min_recording_history, 
						`history length calculation error`
					);
				}
				body {/*...}*/
					return buffer_size (in_samples) / sampling_frequency;
				}
			void history_length (Seconds time)
				{/*...}*/
					min_recording_history = time;

					invalidate_parameters;
				}

			Interval!Volts voltage_range (string channel_type)() const
				{/*...}*/
					mixin(q{
						return } ~channel_type~ q{_voltage_range;
					});
				}
			void voltage_range (string channel_type)(Interval!Volts range)
				in {/*...}*/
					mixin(q{
						alias MaxVoltageRange = Max} ~channel_type.capitalize~ q{VoltageRange;
					});

					assert (range.is_contained_in (MaxVoltageRange ()), 
						range.text~ ` exceeded ` ~MaxVoltageRange.stringof~ ` for ` ~Model.name
					);
				}
				body {/*...}*/
					mixin(q{
						} ~channel_type~ q{_voltage_range = range;
					});
				}
		}
		const {/*time ↔ index}*/
			uint index_at_time (Seconds t)
				in {/*...}*/
					assert (t >= 0.seconds, `cannot index with negative time (` ~t.text~ `)`);
				}
				body {/*...}*/
					alias h = recording_length;

					auto x = sample_count * t/h;

					return cast(uint)x.round;
				}
			Seconds time_at_index (size_t i)
				{/*...}*/
					return i * recording_length / sample_count;
				}

			uint n_samples_in (Seconds seconds)
				{/*...}*/
					uint n_samples;

					return (seconds * sampling_frequency).round.to!uint;
				}
			Seconds duration_of_sample_count (uint n_samples)
				{/*...}*/
					return n_samples / sampling_frequency;
				}
		}
		public:
		public {/*controls}*/
			void reset ()
				out {/*...}*/
					assert (this.is_ready || count_open!`input` == 0, `DAQ reset error`);
				}
				body {/*...}*/
					if (this.is_streaming)
						stop;

					validate_parameters;
					
					initialize_channels;

					buffer = RingBuffer (buffer_size (in_doubles));

					n_samples_streamed = 0;
				}
		}
		public {/*channels}*/
			public {/*open/close}*/
				private mixin template Channels (string channel_type)
					{/*...}*/
						mixin(q{
							alias Channels = } ~channel_type.capitalize~ q{Channels;
						});
					}

				void open_channel (string channel_type)(Index i)
					in {/*...}*/
						mixin Channels!channel_type;

						assert (i < Channels.count, `only ` ~Channels.stringof~ ` available on ` ~Model.name);
					}
					body {/*...}*/
						mixin(q{
							} ~channel_type~ q{_channels[i].open = true;
						});

						invalidate_parameters;
					}

				void close_channel (string channel_type)(Index i)
					in {/*...}*/
						mixin Channels!channel_type;

						assert (i < Channels.count, `only ` ~Channels.stringof~ ` available on ` ~Model.name);
					}
					body {/*...}*/
						mixin(q{
							} ~channel_type~ q{_channels[i].open = false;
						});

						invalidate_parameters;
					}

				void open_channels (string channel_type)(Index[] indices...)
					{/*...}*/
						foreach (i; indices)
							open_channel!channel_type (i);
					}
				void open_channels (string channel_type)()
					{/*...}*/
						mixin Channels!channel_type;

						foreach (i; 0..Channels.count)
							open_channel!channel_type (i);
					}

				void close_channels (string channel_type)(Index[] indices...)
					{/*...}*/
						foreach (i; indices)
							close_channel!channel_type (i);
					}
				void close_channels (string channel_type)()
					{/*...}*/
						mixin Channels!channel_type;

						foreach (i; 0..Channels.count)
							close_channel!channel_type (i);
					}
			}

			const @property input ()
				{/*...}*/
					return input_channels[];
				}
			@property output ()
				{/*...}*/
					return output_channels[];
				}

			class Input
				{/*...}*/
					const is_open ()
						{/*...}*/
							return open;
						}

					const sample_by_index ()
						{/*...}*/
							return stream_from (&sample_at_index, &sample_count);
						}
					const sample_by_time ()
						{/*...}*/
							return stream_from (&sample_at_time, &recording_length).at (&sampling_frequency);
						}

					const sample_at_index (Index i)
						in {/*...}*/
							assert (this.is_open, 
								`attempted to access closed channel`
							);
							assert (i < sample_count,
								`access (` ~i.text~ `) out of bounds (` ~sample_count.text~ `)`
							);
						}
						body {/*...}*/
							return buffer[offset + stride * i].volts;
						}
					const sample_at_time (Seconds t)
						in {/*...}*/
							assert (this.is_open, 
								`attempted to access closed channel`
							);
							assert (t.between (0.seconds, recording_length),
								`access (` ~t.text~ `) out of bounds (` ~interval (0.seconds, recording_length).text~ `)`
							);
						}
						body {/*...}*/
							return sample_at_index (index_at_time (t));
						}

					private:
					private {/*data}*/
						bool open;
						size_t offset;
						size_t stride;
					}
				}
			class Output
				{/*...}*/
					const is_open ()
						{/*...}*/
							return open;
						}

					const sample_by_index ()
						{/*...}*/
							return stream_from (&sample_at_index, &sample_count);
						}
					const sample_by_time ()
						{/*...}*/
							return stream_from (&sample_at_time, &recording_length).at (&sampling_frequency);
						}

					const sample_at_index (Index i)
						in {/*...}*/
							assert (this.is_open,
								`attempted to access closed channel`
							);
						}
						body {/*...}*/
							alias period = generating_period;

							if (period.is_infinite || n_samples_streamed < n_samples_in (period))
								return generator (time_at_index (i));
							else return generator (time_at_index ((i + n_samples_streamed) % n_samples_in (period)));
						}
					const sample_at_time (Seconds t)
						in {/*...}*/
							assert (this.is_open,
								`attempted to access closed channel`
							);
							assert (t.between (0.seconds, recording_length),
								`access (` ~t.text~ `) out of bounds (` ~interval (0.seconds, recording_length).text~ `)`
							);
						}
						body {/*...}*/
							alias period = generating_period;

							if (period.is_infinite)
								return generator (t);
							else return generator (t % period);
						}

					public:
					public {/*ctor}*/
						this ()
							{/*...}*/
								generator =x=> 0.volts;
							}
					}
					public {/*settings}*/
						auto generate (Volts delegate(Seconds) generator)
							{/*...}*/
								this.generator = generator;

								invalidate_parameters;

								return this;
							}
					}
					private:
					private {/*data}*/
						Volts delegate(Seconds) generator;

						bool open;
						size_t offset;
						size_t stride;
					}
					private {/*upload}*/
						auto upload (scope double[] samples)
							in {/*...}*/
								assert (this.is_open, `attempted to access closed channel`);
							}
							body {/*...}*/
								//foreach (size_t i, ref sample; samples[offset..$].stride (this.stride)) // OUTSIDE BUG Error: cannot infer argument types
								for (size_t i = 0; offset + i*stride < samples.length; ++i)
									samples[offset + i*stride] = generator (i/generating_frequency).to_scalar;
							}
					}
				}
		}
		public {/*callbacks}*/
			void on_capture (void delegate(uint n_samples_streamed) callback) // no invalidation
				{/*...}*/
					this.callback = callback;
				}
		}
		public {/*ctor}*/
			this ()
				{/*...}*/
					version (LIVE)
					{/*identify device}*/
						auto lsdaq = pipeShell (`lsdaq`).stdout.byLine.drop (3);

						if (not (lsdaq.front.canFind (`Dev`)))
							assert (0, `no DAQ devices detected`);

						foreach (device_info; lsdaq)
							{/*...}*/
								if (not (device_info.canFind (`Dev`)))
									assert (0, `couldn't find ` ~Model.name~ ` #` ~Serial.number.text~ ` among connected DAQ devices`);


								this.device_id = device_info.find (`Dev`)[0..4].text;
								uint serial_number;

								DAQmx.GetDevSerialNum (device_id, &serial_number);

								if (serial_number == Serial.number)
									break;
							}
					}

					foreach (ref channel; input_channels)
						channel = new Input;

					foreach (ref channel; output_channels)
						channel = new Output;
				}
		}
		protected:
		@Service shared override {/*}*/
			bool initialize ()
				{/*...}*/
					auto ready_channels ()
						{/*...}*/
							if (stream_is & capture)
								with (cast()this) {/*...}*/
									DAQmx.CreateTask (``, &input_task);

									DAQmx.CreateAIVoltageChan (input_task, 
										channel_string!`input`, ``, 
										DAQmx_Val_Cfg_Default, 
										input_voltage_range.min.to_scalar, 
										input_voltage_range.max.to_scalar, 
										DAQmx_Val_Volts, 
										null
									);

									DAQmx.CfgSampClkTiming (input_task, 
										`OnboardClock`, 
										sampling_frequency.to_scalar, 
										DAQmx_Val_Rising, 
										DAQmx_Val_ContSamps, 
										0
									);

									DAQmx.CfgDigEdgeStartTrig (input_task, 
										`/` ~device_id~ `/PFI0`, 
										DAQmx_Val_Rising
									);

									DAQmx.StartTask (input_task);
								}
							if (stream_is & generate)
								with (cast()this) {/*...}*/
									DAQmx.CreateTask (``, &output_task);

									DAQmx.CreateAOVoltageChan (output_task, 
										channel_string!`output`, ``,
										output_voltage_range.min.to_scalar, 
										output_voltage_range.max.to_scalar, 
										DAQmx_Val_Volts,
										null
									);

									DAQmx.CfgSampClkTiming (output_task, 
										`OnboardClock`, 
										generating_frequency.to_scalar, 
										DAQmx_Val_Rising, 
										DAQmx_Val_ContSamps, 
										0
									);

									DAQmx.CfgDigEdgeStartTrig (output_task,
										`/` ~device_id~ `/PFI0`,
										DAQmx_Val_Rising
									);

									{/*upload samples}*/
										auto n_samples = (generating_period * generating_frequency).to!size_t;

										assert (n_samples * count_open!`output` < OutputBufferSize (),
											`number of output samples (` ~(n_samples * count_open!`output`).text~ `) exceeded ` ~OutputBufferSize.stringof
										);

										scope samples = new double[n_samples * count_open!`output`];

										foreach (channel; output_channels[])
											if (channel.is_open)
												channel.upload (samples);

										foreach (sample; samples)
											assert (sample.volts.is_contained_in (output_voltage_range),
												`output (` ~sample.text~ `) exceeded voltage range (` ~output_voltage_range.text~ `)` 
											);

										enum auto_start = false;
										enum timeout = 5.seconds;
										int n_samples_written;

										DAQmx.WriteAnalogF64 (output_task,
											n_samples,
											auto_start,
											timeout.to_scalar,
											DAQmx_Val_GroupByScanNumber,
											samples.ptr,
											&n_samples_written,
											null
										);

										version (LIVE)
										assert (n_samples_written == n_samples);
									}

									DAQmx.StartTask (output_task);
								}
						}
					auto start_trigger ()
						{/*...}*/
							TaskHandle trigger_task;

							DAQmx.CreateTask (``, &trigger_task);

							DAQmx.CreateDOChan (trigger_task, device_id~ `/port1/line0`, ``, DAQmx_Val_ChanPerLine);

							auto high = ubyte.max;
							auto timeout = 0.0;
							enum auto_start = true;
							int n_samples_written;
							DAQmx.WriteDigitalU8 (trigger_task, 1, auto_start, timeout, DAQmx_Val_GroupByScanNumber, &high, &n_samples_written, null);

							high = 0;
							DAQmx.WriteDigitalU8 (trigger_task, 1, auto_start, timeout, DAQmx_Val_GroupByScanNumber, &high, &n_samples_written, null);

							DAQmx.StopTask (trigger_task);
							DAQmx.ClearTask (trigger_task);
						}

					////

					(cast()this).reset;

					ready_channels;

					start_trigger;

					return true;
				}
			bool process ()
				{/*...}*/
					if (stream_is & capture)
						capture_block;
					else sleep (1 / (cast()this).capture_frequency);

					if (callback !is null)
						with (cast()this) callback (block_size (in_samples));

					return true;
				}
			bool listen ()
				{/*...}*/
					return false;
				}
			bool terminate ()
				{/*...}*/
					void terminate (ref TaskHandle task_handle)
						{/*...}*/
							DAQmx.StopTask (task_handle);
							DAQmx.ClearTask (task_handle);
							task_handle = null;
						}

					///

					if (stream_is & capture)
						terminate (input_task);

					if (stream_is & generate)
						terminate (output_task);

					return true;
				}
			const string name ()
				{/*...}*/
					return DAQDevice.stringof;
				}
		}
		private:
		private {/*parameter validation}*/
			void validate_parameters ()
				{/*...}*/
					if (not (sampling_frequency * count_open!`input` <= MaxSamplingRate ()))
						assert (0, `combined sampling frequency (` ~(sampling_frequency * count_open!`input`).text~ `) exceeded ` ~MaxSamplingRate.stringof);

					// TODO generating freq and period stuff

					if (not (history_length >= min_recording_history))
						assert (0, `history length calculation error`);
						
					if (not (voltage_range!`input`.is_contained_in (MaxInputVoltageRange ())))
						assert (0, `input voltage (` ~voltage_range!`input`.text~ `) exceeded ` ~MaxInputVoltageRange.stringof);

					if (not (capture_frequency <= sampling_frequency))
						assert (0, `capture frequency (` ~capture_frequency.text~ `) exceeded sampling frequency (` ~sampling_frequency.text~ `)`);

					if (not (buffer_size (in_samples) % block_size (in_samples) == 0))
						assert (0, `buffer (` ~buffer_size (in_samples).text~ `) not evenly divisible into blocks (` ~block_size (in_samples).text~ `)`);

					parameters_invalidated = false;
				}
			void invalidate_parameters ()
				in {/*...}*/
					assert (not (is_streaming), `attempted to alter device parameters while streaming in progress`);
				}
				body {/*...}*/
					parameters_invalidated = true;
				}
			bool parameters_invalidated;
		}
		shared {/*data streaming}*/
			enum {capture = 0x1, generate = 0x2}

			auto stream_is ()
				{/*...}*/
					auto n_in = (cast()this).count_open!`input`;
					auto n_out = (cast()this).count_open!`output`;

					if (n_out == 0 && n_in > 0)
						return capture;
					else if (n_in == 0 && n_out > 0)
						return generate;
					else if (n_in > 0 && n_out > 0)
						return (generate | capture);
					else assert (0, `no channels open`);
				}

			void capture_block ()
				in {/*...}*/
					assert ((cast()this).is_streaming, `attempted to capture data while not streaming`);
				}
				body {/*...}*/
					int n_samples_read = 0;

					auto timeout = 5.seconds;

					with (cast()this) 
					DAQmx.ReadAnalogF64 (input_task,
						block_size (in_samples),
						timeout.to_scalar,
						DAQmx_Val_GroupByScanNumber,
						buffer.input,
						buffer_size (in_samples),
						&n_samples_read,
						null
					);

					with (cast()this) {/*advance buffer}*/
						version (LIVE)
						assert (n_samples_read == block_size (in_samples),
							`incorrect number of samples recorded (` ~n_samples_read.text~ `, ` ~block_size (in_samples).text~ ` expected)`
						);

						n_samples_streamed += block_size (in_samples);

						buffer.advance (block_size (in_doubles));
					}
				}
		}
		const @property {/*buffer block sizes}*/
			uint block_size (size_t units)
				{/*...}*/
					return cast(uint)((sampling_frequency / capture_frequency) * units).ceil;
				}
			uint buffer_size (size_t units)
				out (result) {/*...}*/
					assert (result % block_size (units) == 0, 
						`buffer size (` ~result.text~ `) is not evenly divisible into blocks (` ~block_size (units).text~ `)`
					);
				}
				body {/*...}*/
					auto minimum_samples_to_buffer = min_recording_history * sampling_frequency;

					auto M = minimum_samples_to_buffer;
					auto B = block_size (in_samples);

					if (B == 0)
						return 0;

					auto samples_to_buffer = ceil (M/B) * B;

					return cast(uint)(samples_to_buffer * units);
				}

			enum in_samples = 1;
			alias in_doubles = count_open!`input`;
		}
		private:
		private {/*status}*/
			size_t n_samples_streamed;
		}
		private {/*parameters}*/
			auto input_voltage_range = MaxInputVoltageRange ();
			auto output_voltage_range = MaxOutputVoltageRange ();
			auto min_recording_history = 1.second;
			auto _sampling_frequency = MaxSamplingRate () / InputChannels.count;
			auto _capture_frequency = 30.hertz;
			auto _generating_frequency = 30.hertz;
			auto _generating_period = infinity.seconds;
		}
		private {/*handles}*/
			string device_id;
			Tid streaming_thread;
			TaskHandle input_task;
			TaskHandle output_task;
		}
		private {/*channels}*/
			Input[InputChannels.count] input_channels;
			Output[OutputChannels.count] output_channels;

			void initialize_channels ()
				{/*...}*/
					void initialize (string channel_type)()
						{/*...}*/
							int offset = 0;
							int stride = count_open!channel_type;

							foreach (channel; mixin(channel_type~ q{_channels[]}))
								if (channel.is_open)
									{/*...}*/
										channel.offset = offset++;
										channel.stride = stride;
									}
						}
						
					initialize!`input`;
					initialize!`output`;
				}

			auto channel_string (string channel_type)()
				if (channel_type == `input` || channel_type == `output`)
				{/*...}*/
					string channel_string;

					foreach (i, channel; mixin(channel_type))
						if (channel.is_open)
							channel_string ~= device_id~ `/a` ~channel_type[0]~i.text~ `, `;

					if (channel_string.empty)
						return ``;
					else return channel_string[0..$-2];
				}
		}
		private {/*buffer}*/
			RingBuffer buffer;

			struct RingBuffer
				{/*...}*/
					double* buffer;
					const size_t capacity;
					size_t position;
					bool filled;

					this (size_t size)
						{/*...}*/
							if (size == 0)
								return;

							buffer = cast(double*)malloc (size * double.sizeof);
							capacity = size;
						}
					~this ()
						{/*...}*/
							if (this.is_ready)
								free (buffer);
						}

					auto opIndex (size_t i) const
						in {/*...}*/
							assert (this.is_ready, `attempted to access buffer before ready`);
						}
						body {/*...}*/
							if (filled)
								return buffer[(position + i) % capacity];
							else return buffer[i];
						}
					double* input ()
						in {/*...}*/
							assert (this.is_ready, `attempted to access buffer before ready`);
						}
						body {/*...}*/
							return buffer + position;
						}
					void advance (size_t positions)
						in {/*...}*/
							assert (this.is_ready, `attempted to access buffer before ready`);
						}
						body {/*...}*/
							position += positions;

							if (not (filled) && position >= capacity)
								filled = true;


							position %= capacity;
						}

					bool is_ready () const
						{/*...}*/
							return buffer !is null && capacity > 0;
						}
					size_t length () const
						{/*...}*/
							return filled? capacity: position;
						}

					invariant(){/*}*/
						if (buffer !is null)
							assert (position < capacity, `ring buffer position (` ~position.text~ `) exceeded capacity (` ~capacity.text~ `)`);
					}
				}
		}
		private {/*callbacks}*/
			void delegate(uint) callback;
		}
		invariant (){/*}*/
			assert (n_samples_streamed < typeof(n_samples_streamed).max - _sampling_frequency / _capture_frequency,
				`n_samples_streamed approaching ` ~typeof(n_samples_streamed).stringof~ `.max`
			);
		}
	}
	public {/*Specs}*/
		struct Model (string model_name)
			{/*...}*/
				enum name = model_name;

				enum isModel;
			}
		struct InputChannels (uint n_channels)
			{/*...}*/
				enum count = n_channels;

				enum isInputChannels;
			}
		struct OutputChannels (uint n_channels)
			{/*...}*/
				enum count = n_channels;

				enum isOutputChannels;
			}
		struct MaxSamplingRate (uint max__sampling_frequency)
			{/*...}*/
				enum rate = max__sampling_frequency.hertz;

				enum isMaxSamplingRate;

				static opCall ()
					{/*...}*/
						return rate;
					}
			}
		struct MaxOutputVoltageRange (real min_voltage, real max_voltage)
			{/*...}*/
				enum min = min_voltage.volts;
				enum max = max_voltage.volts;

				enum isMaxOutputVoltageRange;

				static opCall ()
					{/*...}*/
						return Interval!Volts (min, max);
					}
			}
		struct MaxInputVoltageRange (real min_voltage, real max_voltage)
			{/*...}*/
				enum min = min_voltage.volts;
				enum max = max_voltage.volts;

				enum isMaxInputVoltageRange;

				static opCall ()
					{/*...}*/
						return Interval!Volts (min, max);
					}
			}
		struct Serial (size_t serial_number)
			{/*...}*/
				enum number = serial_number;

				enum isSerial;
			}
		struct InputBufferSize (uint fifo_size)
			{/*...}*/
				enum size = fifo_size;

				enum isInputBufferSize;

				static opCall ()
					{/*...}*/
						return size;
					}
			}
		struct OutputBufferSize (uint fifo_size)
			{/*...}*/
				enum size = fifo_size;

				enum isOutputBufferSize;

				static opCall ()
					{/*...}*/
						return size;
					}
			}
	}

/* blocking recording function 
*/
void record_for (T)(T daq, Seconds time)
	{/*...}*/
		daq.start;

		auto span = 0.seconds;

		while (span < time)
			{/*...}*/
				sleep (5.milliseconds);
				span += 5.milliseconds;
			}

		daq.stop;
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

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
			const torque_z ()
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

/////////////////////////////////////////////////////////////////////////////////////////////////////////
unittest {/*...}*/
	auto daq = new DAQDevice!(
		Model!`NI USB-6216`,

		InputChannels!8,
		OutputChannels!2,

		MaxSamplingRate!(250_000),
		MaxInputVoltageRange!(-10, 10),
		MaxOutputVoltageRange!(-5, 5),

		Serial!0x_18DEF36,
		InputBufferSize!4095,
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

	version (MOCK_DATA) {/*...}*/
		{/*8 inputs, constant signals}*/
			daq.open_channels!`input` (0,1,2,3,4,5,6,7);

			daq.record_for (1.second);

			{/*channel crosstalk}*/
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

				auto stream = daq.input[5].sample_by_time[$-0.0005.seconds..$];
				auto mapped = stream.map!(x => x);
				auto mapped_stream = daq.input[5].sample_by_time[$-0.0005.seconds..$].map!(x => x);

				assert (stream.equal (mapped));
				assert (mapped.equal (mapped_stream));
				assert (mapped_stream.equal (stream));
			}
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
			DAQmx.mock_channel.clear;
		}
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
		{/*1 input, 2 outputs, "live" stream processing}*/
			daq.open_channel!`input`(7);
			daq.open_channels!`output`(0,1);

			DAQmx.mock_channel[7] = i => 3*i/daq.sampling_frequency.to_scalar;

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
		{/*1 input, 1 output, ramp signal}*/
			daq.open_channel!`output` (0);
			daq.open_channel!`input` (0);

			daq.sampling_frequency = 60.hertz;

			alias signal = λ!(t => t*volts/second);

			DAQmx.mock_channel[0] = i => signal (i/daq.sampling_frequency).to_scalar;

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
		{/*2 inputs, 1 output, ramp signals}*/
			template to_volts (double s)
				{/*...}*/
					auto to_volts = (Seconds t) => s*t*volts/second;
				}

			DAQmx.mock_channel[0] = i => to_volts!1.0 (i/daq.sampling_frequency).to_scalar;
			DAQmx.mock_channel[1] = i => to_volts!2.0 (i/daq.sampling_frequency).to_scalar;

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

				foreach (item; together[0.seconds..250.milliseconds]) 
					assert (item[0].approx (item[1]));

				foreach (item; together[250.milliseconds..$]) 
					assert (not (item[0].approx (item[1])));

				auto staggered = zip (
					daq.output[1].sample_by_time[250.milliseconds..500.milliseconds].map!(v => v*newtons/volt),
					plate.force_x				[0.seconds..250.milliseconds]
				);

				foreach (item; staggered) 
					assert (item[0].approx (item[1]));
			}

			DAQmx.mock_channel.clear;
		}
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
/////////////////////////////////////////////////////////////////////////////////////////////////////////

void main ()
	{/*...}*/
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
			generating_frequency = 240.hertz;

			history_length = 5.seconds;
			generating_period = 1/120.hertz;

			daq.voltage_range!`input` = interval (-5.volts, 5.volts); // remove "daq." → OUTSIDE BUG: Error: need 'this' for 'voltage_range' of type 'pure nothrow @property @safe void(Interval!(Unit!(Current, -1, Mass, 1, Space, 2, Time, -3)) range)'
			daq.voltage_range!`output` = interval (0.volts, 3.volts); // remove "daq." → OUTSIDE BUG: Error: need 'this' for 'voltage_range' of type 'pure nothrow @property @safe void(Interval!(Unit!(Current, -1, Mass, 1, Space, 2, Time, -3)) range)'
		}
		with (plate) {/*signals}*/
			voltage_to_force = vector (1250.newtons/5.volts, 1250.newtons/5.volts, 2500.newtons/5.volts);
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

		daq.open_channels!`input`;
		daq.open_channels!`output`;

		with (daq) {/*output signal generation}*/
			output[0].generate (t => t % generating_period < 1/generating_frequency? 3.volts : 0.volts);
			output[1].generate (t => t % generating_period < 1/generating_frequency? 0.volts : 2.volts);
		}

		auto file = File (`test_data.dat`, `w`);

		daq.on_capture =x=> file.writeln (plate.force[$-1/daq.capture_frequency..$]);

		daq.record_for (10.seconds);
	}

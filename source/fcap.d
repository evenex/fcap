module fcap;

private {/*imports}*/
	private {/*core}*/
		import core.thread;
	}
	private {/*std}*/
		import std.typetuple;
		import std.functional;
		import std.range;
		import std.algorithm;
		import std.datetime;
		import std.variant;
		import std.math;
		import std.c.stdlib;
		import std.concurrency;
		import std.stdio;
		import std.conv;
		import std.process;
		import std.string;
		import std.datetime;
	}
	private {/*evx}*/
		import evx.traits; 
		import evx.meta; 
		import evx.utils;
		import evx.service;
		import evx.dsp;
		import evx.display;
		import evx.input;
		import evx.colors;
		import evx.math;
		import evx.range;
		import evx.plot;
		import evx.scribe;
	}
	private {/*nidaqmx}*/
		import nidaqmx;
	}

	alias zip = evx.functional.zip;
	alias map = evx.functional.map;
	alias reduce = evx.functional.reduce;
	alias stride = evx.range.stride;
	alias repeat = evx.range.repeat;
	alias Interval = evx.analysis.Interval;
	alias round = evx.analysis.round;
	alias ceil = evx.analysis.ceil;
	alias seconds = evx.units.seconds;
	alias sum = evx.arithmetic.sum;
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
public {/*daq}*/
	final class DAQDevice (Specs...): Service
		{/*...}*/
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
			pure const @property {/*status}*/
				bool is_ready ()
					{/*...}*/
						if (parameters_invalidated)
							return false;

						if ((sampling_frequency * capture_frequency).to!double == 0)
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
			pure const @property {/*counters}*/
				Index sample_count ()
					{/*...}*/
						return buffer.length / count_open!`input`;
					}
					
				Seconds recording_length ()
					out (result) {/*...}*/
						debug {/*...}*/
							if (buffer.filled)
								assert (result == history_length,
									`buffer filled, but recording length (` ~result.text~ `) doesn't match history length (` ~history_length.text~ `)`
								);
							else assert (result < history_length,
								`recording length (` ~result.text~ `) exceeded history length (` ~history_length.text~ `)`
							);
						}
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
			pure @property {/*parameters}*/
				Hertz sampling_frequency () const
					{/*...}*/
						return (min_sampling_frequency / capture_frequency).ceil * capture_frequency;
					}
				void sampling_frequency (Hertz frequency)
					{/*...}*/
						min_sampling_frequency = frequency;
						
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

						debug assert (range.is_contained_in (MaxVoltageRange ()), 
							range.text~ ` exceeded ` ~MaxVoltageRange.stringof~ ` for ` ~Model.name
						);
					}
					body {/*...}*/
						mixin(q{
							} ~channel_type~ q{_voltage_range = range;
						});
					}

				auto reference_mode () const
					{/*...}*/
						return _reference_mode;
					}
				void reference_mode (ReferenceMode mode)
					{/*...}*/
						this._reference_mode = mode;
					}
			}
			pure const {/*time ↔ index}*/
				uint index_at_time (Seconds t)
					in {/*...}*/
						debug assert (t >= 0.seconds, `cannot index with negative time (` ~t.text~ `)`);
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
				Seconds duration_of_sample_count (size_t n_samples)
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
						pure:

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
								debug assert (this.is_open, 
									`attempted to access closed channel`
								);
								debug assert (t.between (0.seconds, recording_length),
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
						pure:

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
								return stream_from (&sample_at_time, &recording_length).at (&sampling_frequency); // REVIEW constant interpolation works because of this... here is explicit upsampling, this is what makes it work despite the lack of explicit interpolation. if i changed this to generation_frequency, i would get a range measure mismatch error
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
								debug assert (this.is_open,
									`attempted to access closed channel`
								);
								debug assert (t.between (0.seconds, recording_length),
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
							auto generate (Volts delegate(Seconds) pure generator)
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
										samples[offset + i*stride] = generator (i/generating_frequency).to!double;
								}
						}
					}

				/* controls ground reference */
				enum ReferenceMode
					{/*...}*/
						differential = DAQmx_Val_Diff,
						single_ended = DAQmx_Val_RSE,
					}
			}
			public {/*callbacks}*/
				void on_capture (void delegate(uint n_samples_streamed) callback)
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
											reference_mode,
											input_voltage_range.min.to!double, 
											input_voltage_range.max.to!double, 
											DAQmx_Val_Volts, 
											null
										);

										DAQmx.CfgSampClkTiming (input_task, 
											`OnboardClock`, 
											sampling_frequency.to!double, 
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
											output_voltage_range.min.to!double, 
											output_voltage_range.max.to!double, 
											DAQmx_Val_Volts,
											null
										);

										DAQmx.CfgSampClkTiming (output_task, 
											`OnboardClock`, 
											generating_frequency.to!double, 
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
												timeout.to!double,
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
						else Thread.sleep ((1 / (cast()this).capture_frequency).to_duration);

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
						void terminate (ref shared TaskHandle task_handle)
							{/*...}*/
								DAQmx.StopTask (cast(TaskHandle)task_handle);
								DAQmx.ClearTask (cast(TaskHandle)task_handle); // REVIEW sharing
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

						if (count_open!`output` > 0 && (generating_frequency * generating_period) % 1.0 != 0)
							assert (0, `generating frequency (` ~generating_frequency.text~ `)`
								` does not evenly divide generating period (` ~generating_period.text~ `)`
								` (remainder == ` ~((generating_frequency * generating_period) % 1.0).text~ `)`
							);

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
							timeout.to!double,
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
				size_t block_size (size_t units)
					{/*...}*/
						return ((sampling_frequency / capture_frequency) * units).to!size_t;
					}
				size_t buffer_size (size_t units)
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
						return (samples_to_buffer * units).to!size_t;
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
				auto min_sampling_frequency = MaxSamplingRate () / InputChannels.count;
				auto _capture_frequency = 30.hertz;
				auto _generating_frequency = 30.hertz;
				auto _generating_period = infinity.seconds;
				auto _reference_mode = ReferenceMode.differential;
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
						size_t capacity;
						size_t position;
						bool filled;

						this (in size_t size)
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
								auto new_position = position + positions;

								position = new_position % capacity;

								if (not (filled) && new_position >= capacity)
									filled = true;
							}

						bool is_ready () const
							{/*...}*/
								return buffer !is null && capacity > 0;
							}
						size_t length () const
							{/*...}*/
								return filled? capacity: position;
							}

						invariant (){/*}*/
							if (buffer !is null)
								assert (position < capacity, `ring buffer position (` ~position.text~ `) exceeded capacity (` ~capacity.text~ `)`);
						}
					}
			}
			private {/*callbacks}*/
				void delegate(uint) callback;
			}
			invariant (){/*}*/
				assert (n_samples_streamed < typeof(n_samples_streamed).max - min_sampling_frequency / _capture_frequency,
					`n_samples_streamed approaching ` ~typeof(n_samples_streamed).stringof~ `.max`
				);
			}
		}
		static {/*Specs}*/
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
			struct MaxSamplingRate (uint max_sampling_frequency)
				{/*...}*/
					enum rate = max_sampling_frequency.hertz;

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

			while (daq.duration_of_sample_count (daq.n_samples_streamed) <= time)
				Thread.sleep (5.milliseconds.to_duration);

			daq.stop;
		}
}
public {/*force plate}*/
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

void main ()
	{/*...}*/
		template DAQDevice ()
			{/*...}*/
				alias DAQDevice = .DAQDevice!(
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
			}

		auto daq = new DAQDevice!();

		auto timing_light () 
			{/*...}*/
				return zip (
					daq.output[0].sample_by_time[],
					daq.output[1].sample_by_time[]
				).map!vector;
			}

		ForcePlate plate;
		{/*setup}*/
			with (daq) {/*settings}*/
				sampling_frequency = 24.kilohertz;
				capture_frequency = 60.hertz;
				generating_frequency = 240.hertz;

				history_length = 10.seconds;
				generating_period = (1/120.).seconds;

				voltage_range!`input` = interval (-5.volts, 5.volts);
				voltage_range!`output` = interval (0.volts, 3.volts);
				reference_mode = DAQDevice!().ReferenceMode.single_ended;
			}
			with (plate) {/*input signals}*/
				voltage_to_force = vector (1250.newtons/5.volts, 1250.newtons/5.volts, 2500.newtons/5.volts);
				/* source: check the signal conditioner settings TODO maybe automate this with opencv and a webcam */

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
			with (daq) {/*output signals}*/
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

			daq.open_channels!`input`;
			daq.open_channels!`output`;
		}
		{/*run}*/
			write (`enter subject name: `);
			auto subject = readln.strip;

			version (LIVE)
			if (subject.empty)
				subject = `test`;

			{/*record & display}*/
				bool terminate_draw_loop;

				auto gfx = new Display (1920, 1080);

				gfx.start; scope (exit) gfx.stop;
				auto txt = new Scribe (gfx, [14, 144]);
				auto usr = new Input (gfx, (bool){terminate_draw_loop = true;});

				bool draw_it;
				auto timespan = 0.5.seconds;
				auto elapsed = 0.seconds;
				daq.on_capture = (size_t x)
					{/*...}*/
						elapsed += 1/daq.capture_frequency;

						if (daq.recording_length > timespan && not(draw_it))
							draw_it = true;
					};

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

						auto cop_size = plate.force[$-50.milliseconds..$]
							.map!norm.stride (daq.sampling_frequency / 2000.hertz)
							.mean * force_to_size;
						auto cop_point = plate.center_of_pressure[$-50.milliseconds..$]
							.stride (daq.sampling_frequency / 2000.hertz)
							.mean.dimensionless * [-2,2];
						auto cop_angle = atan2 (
							plate.force_y[$-10.milliseconds..$].map!dimensionless.mean,
							-plate.force_x[$-10.milliseconds..$].map!dimensionless.mean
						);

						gfx.draw (red (0.6), circle (cop_size/50, cop_point + Δx), GeometryMode.t_fan);
						txt.write (Unicode.arrow[`down`])
							.rotate (cop_angle + π/2)
							.size (txt.available_sizes.reduce!max)
							.color (red (0.4))
							.inside (force_plate_geometry)
							.scale (cop_size)
							.align_to (Alignment.center)
							.translate (cop_point)
						();
					}

				void draw_plot (T)(T input, string label, Interval!Newtons range, BoundingBox bounds)
					{/*...}*/
						auto lap = elapsed;

						plot (input[$-timespan..$].versus (ℕ[0..daq.n_samples_in (timespan)].map!(i => lap + i/daq.sampling_frequency))
							.stride (daq.sampling_frequency / 128.hertz)
						)
							.color (green)
							.y_axis (label, range)
							.x_axis (`time`, interval (lap, lap + timespan))
							.inside (bounds)
							.using (gfx, txt)
						();
					}

				version (MOCK_DATA)
				foreach (x; 0..8)
					DAQmx.mock_channel[x] = i => cast(double) sin (π*i*gaussian/100f)^^2;

				daq.start;
				while (daq.is_streaming && not (terminate_draw_loop))
					{/*...}*/
						if (draw_it)
							{/*...}*/
								draw_plot (plate.force_z, `vertical force`, 
									interval (0.newtons, 2000.newtons),
									[vector (0.1, 1.0), vector (1.7, 1./3)].bounding_box
								);
								draw_plot (plate.force_y, `anterior force`, 
									interval (-1000.newtons, 1000.newtons),
									[vector (0.1, 1./3), vector (1.7, -1./3)].bounding_box
								);
								draw_plot (plate.force_x, `lateral force`, 
									interval (-1000.newtons, 1000.newtons),
									[vector (0.1, -1./3), vector (1.7, -1.0)].bounding_box
								);
								draw_cop_vector;
								draw_it = false;

								gfx.render;
								usr.process;
							}
					}
				daq.stop;
			}
			version (none)
			{/*write to file}*/
				auto time_of_capture = cast(DateTime)(Clock.currTime);

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
						"\tgenerating_frequency: %s\n"
						"\tgenerating_period: %s\n"
						"\tvoltage_range!`input`: %s\n"
						"\tvoltage_range!`output`: %s\n"
						,

						time_of_capture.date.toSimpleString,
						time_of_capture.timeOfDay.toString,
						subject,

						daq.Model.name, daq.Serial.number,
							sampling_frequency.text,
							capture_frequency.text,
							generating_frequency.text,
							generating_period.text,
							daq.voltage_range!`input`.text,
							daq.voltage_range!`output`.text,
					);
				}
				with (plate) {/*write data}*/
					auto stream = zip (
						ℕ[0..daq.sample_count], 
						zip(
							timing_light[],
							force[],
							moment[],
							surface_moment[],
							center_of_pressure[],
							z_torque[],
						)
					);

					
					writeln;
					writeln (`writing data to disk...`);
					writeln ('_'.repeat (stream.length / (stream.length/100)), `█ 100%`);

					foreach (i, item; stream)
						{/*...}*/
							if (i % (stream.length/100) == 0)
								write (`▒`), stdout.flush;

							file.writefln (
								"{%s}\n"
								"timing_light %s\n"
								"force %s\n"
								"moment %s\n"
								"surface_moment %s\n"
								"center_of_pressure %s\n"
								"z_torque %s\n",

								(i/daq.sampling_frequency).text,
								item[0].text,
								item[1].text,
								item[2].text (`N·m`),
								item[3].text (`N·m`),
								item[4].text,
								item[5].text (`N·m`),
							);
						}

					writeln (`█`);
				}
			}
		}
	}

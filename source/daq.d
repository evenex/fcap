module fcap.daq;

private {/*imports}*/
	private {/*core}*/
		import core.thread;
	}
	private {/*std}*/
		import std.typetuple;
		import std.range;
		import std.algorithm;
		import std.c.stdlib;
		import std.concurrency;
		import std.conv;
		import std.process;
		import std.string;
	}
	private {/*evx}*/
		import evx.traits; 
		import evx.utils;
		import evx.service;
		import evx.dsp;
		import evx.math;
	}
	private {/*nidaqmx}*/
		import nidaqmx;
	}

	alias round = evx.analysis.round;
	alias ceil = evx.analysis.ceil;
	alias seconds = evx.units.seconds;
}

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
		static {/*properties}*/
			static if (Model.name == `NI USB-6216`)
				enum supports_digital_triggering = false;
			else enum supports_digital_triggering = false;
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
					auto inputs = count_open!`input`;

					return inputs > 0?
						(buffer.length / inputs):0;
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
					return sample_count > 0? 
						i * recording_length / sample_count:
						0.seconds;
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
					if (count_open!`input` == 0)
						return;

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
					in {/*...}*/
						assert (indices.length <= InputChannels.count);

						foreach (i; indices)
							assert (i < InputChannels.count, i.text~ ` exceeds ` ~InputChannels.stringof);
					}
					body {/*...}*/
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
					in {/*...}*/
						assert (indices.length <= InputChannels.count);

						foreach (i; indices)
							assert (i < InputChannels.count, i.text~ ` exceeds ` ~InputChannels.stringof);
					}
					body {/*...}*/
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

			@property input ()
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
							debug assert (this.is_open, 
								`attempted to access closed channel`
							);
							debug assert (i < sample_count,
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
							debug assert (this.is_open,
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
								debug assert (this.is_open, `attempted to access closed channel`);
							}
							body {/*...}*/
								//foreach (size_t i, ref sample; samples[offset..$].stride (this.stride)) // OUTSIDE BUG Error: cannot infer argument types
								for (size_t i = 0; offset + i*stride < samples.length; ++i)
									samples[offset + i*stride] = generator (i/generating_frequency).to!double;
							}
					}
				}
		}
		public {/*callbacks}*/
			void on_capture (void delegate(uint n_samples_streamed) callback)
				{/*...}*/
					this.callback = callback;
				}
		}
		public {/*pulse signalling}*/
			shared void send_digital_pulse (size_t channel = 0)
				{/*...}*/
					TaskHandle pulse_task;

					DAQmx.CreateTask (``, &pulse_task);

					DAQmx.CreateDOChan (pulse_task, device_id~ `/port1/line` ~channel.text, ``, DAQmx_Val_ChanPerLine);

					auto high = ubyte.max;

					auto timeout = 0.0;
					enum auto_start = true;
					int n_samples_written;
					DAQmx.WriteDigitalU8 (pulse_task, 1, auto_start, timeout, DAQmx_Val_GroupByScanNumber, &high, &n_samples_written, null);

					Thread.sleep (250.usecs);

					high = ubyte.min;
					DAQmx.WriteDigitalU8 (pulse_task, 1, auto_start, timeout, DAQmx_Val_GroupByScanNumber, &high, &n_samples_written, null);

					DAQmx.StopTask (pulse_task);
					DAQmx.ClearTask (pulse_task);
				}
			void enqueue_digital_pulse (size_t channel = 0)
				{/*...}*/
					this.send (channel);
				}
			shared void power_cycle () // REVIEW DRY
				{/*...}*/
					TaskHandle pulse_task;

					DAQmx.CreateTask (``, &pulse_task);

					DAQmx.CreateDOChan (pulse_task, device_id~ `/port1/line1`, ``, DAQmx_Val_ChanPerLine);

					auto high = ubyte.max;

					auto timeout = 0.0;
					enum auto_start = true;
					int n_samples_written;
					DAQmx.WriteDigitalU8 (pulse_task, 1, auto_start, timeout, DAQmx_Val_GroupByScanNumber, &high, &n_samples_written, null);

					import core.thread; // REVIEW
					Thread.sleep (1000.msecs);
					DAQmx.ResetDevice (device_id.to!string);
					// TODO LABVIEW NEEDS TO BE FULLY RESTARTED, SO DAQ STUFF OCCURS ON A SEPARATE PROCESS LAUNCHED BY MAIN CONTROLLER... CAN MAIN CONTROLLER MANAGE CAMERA FUNCTIONS TOO????
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

									static if (this.supports_digital_triggering)
										DAQmx.CfgDigEdgeStartTrig (input_task,
											`/` ~device_id~ `/PFI0`, 
											DAQmx_Val_Rising
										);
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

									static if (this.supports_digital_triggering)
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
								}
						}

					auto start_streaming ()
						{/*...}*/
							static if (this.supports_digital_triggering)
								send_digital_pulse (0);
							else {/*...}*/
								if (stream_is & capture)
									with (cast()this) DAQmx.StartTask (input_task);
								if (stream_is & generate)
									with (cast()this) DAQmx.StartTask (output_task);
							}
						}

					////

					(cast()this).reset;
					ready_channels;
					start_streaming;

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
					receiveTimeout (100.usecs, (size_t channel) {send_digital_pulse (channel);});

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
					if (sampling_frequency * count_open!`input` > MaxSamplingRate ())
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

					auto timeout = 30.seconds; // TEMP was 5

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

							if (not!filled && new_position >= capacity)
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

/* controls ground reference */
enum ReferenceMode
	{/*...}*/
		differential = DAQmx_Val_Diff,
		single_ended = DAQmx_Val_RSE,
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

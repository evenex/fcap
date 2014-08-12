module fcap;

private {/*import std}*/
	import std.traits:
		isUnsigned, isFloatingPoint,
		isSomeFunction, ReturnType, ParameterTypeTuple,
		isBuiltinType, RepresentationTypeTuple;

	import std.typetuple:
		allSatisfy,
		TypeTuple, Filter;

	import std.functional:
		toDelegate;

	import std.range:
		drop, repeat,
		retro, 
		equal, empty;

	import std.algorithm:
		find;

	import std.datetime:
		SysTime;

	import std.variant:
		Algebraic, visit;

	import std.math:
		floor, ceil;

	import std.c.stdlib:
		malloc, free;

	import std.concurrency:
		spawn, Tid, ownerTid,
		send, receiveOnly; 
	
	import std.stdio:
		stderr;
	
	import std.conv:
		to, text;

	import std.process:
		pipeShell;
}
private {/*import evx}*/
	import evx.traits: 
		has_trait, has_length;

	import evx.meta: 
		Builder;

	import evx.functional:
		map, zip, reduce;

	import evx.logic:
		Or;

	import evx.algebra:
		zero, unity;

	import evx.analysis:
		Interval;

	import evx.arithmetic:
		add, divide, sum;

	import evx.range:
		slice_within_bounds;

	import evx.utils:
		Index;

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
}

nothrow:

class DAQDevice (Specs...)
	{/*...}*/
		nothrow:
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
			mixin check!q{MaxVoltageRange};
			mixin check!q{Serial};
			mixin check!q{InputBufferSize};
		}
		static {/*specs}*/
			template extract (string specification)
				{/*...}*/
					alias extract = Filter!(has_trait!(`is` ~specification), Specs)[0];
				}

			alias Model 			= extract!q{Model};
			alias InputChannels 	= extract!q{InputChannels};
			alias OutputChannels 	= extract!q{OutputChannels};
			alias MaxSamplingRate 	= extract!q{MaxSamplingRate};
			alias MaxVoltageRange 	= extract!q{MaxVoltageRange};
			alias Serial 			= extract!q{Serial};
			alias InputBufferSize 	= extract!q{InputBufferSize};
		}

		public:
		const @property {/*status}*/
			bool is_ready ()
				{/*...}*/
					if (parameters_invalidated)
						return false;

					if ((sampling_rate * read_frequency).to_scalar == 0)
						return false;

					if (not (buffer.is_ready))
						return false;

					foreach (channel; input)
						if (channel.is_open) 
							continue;
						else return false;

					return true;
				}
			bool is_recording ()
				{/*...}*/
					return recording;
				}
		}
		const @property {/*counters}*/
			Index current_sample_count ()
				{/*...}*/
					return 1;// TODO
				}
			Seconds current_recording_length ()
				{/*...}*/
					return current_sample_count / sampling_frequency;
				}
			int open_channels_count ()
				{/*...}*/
					auto n = 0;

					foreach (channel; input_channels)
						if (channel.is_open)
							++n;

					return n;
				}
		}
		@property {/*parameters}*/
			Hertz sampling_frequency () const
				{/*...}*/
					return sampling_rate;
				}
			void sampling_frequency (Hertz frequency)
				{/*...}*/
					sampling_rate = frequency;
					
					invalidate_parameters;
				}

			Seconds max_recording_length () const // REVIEW
				out (result) {/*...}*/
					assert (result >= min_recording_length);
				}
				body {/*...}*/
					return buffer_size (in_samples) / sampling_frequency;
				}
			void max_recording_length (Seconds time) // REVIEW
				{/*...}*/
					min_recording_time = time;

					invalidate_parameters;
				}

			Interval!Volts input_voltage () const
				{/*...}*/
					return input_voltage_range;
				}
			void input_voltage (Interval!Volts range)
				in {/*...}*/
					assert (range.contained_in (MaxVoltageRange ()));
				}
				body {/*...}*/
					input_voltage_range = range;
				}
		}
		public:
		public {/*controls}*/
			private enum Status {terminated}

			void record ()
				{/*...}*/
					if (not (is_ready))
						reset;

					DAQmx.StartTask (task_handle);

					try recording_thread = spawn (cast(shared)&capture);
					catch (Exception) assert (0);
				}
			void stop ()
				{/*...}*/
					if (this.is_recording)
						{/*...}*/
							recording = false;

							try receiveOnly!Status;
							catch (Exception) assert (0);

							DAQmx.StopTask (task_handle);
							DAQmx.ClearTask (task_handle);
						}
				}
			void reset ()
				out {/*...}*/
					assert (this.is_ready);
				}
				body {/*...}*/
					auto channel_string ()
						{/*...}*/
							string channel_string;

							try foreach (i, channel; input)
								if (channel.is_open)
									channel_string ~= device_id~ `/ai` ~i.text~ `, `;
							catch (Exception) assert (0); // REVIEW

							if (channel_string.empty)
								assert (0, `no open channels`); // REVIEW
							return channel_string[0..$-2];
						}

					if (this.is_recording)
						stop;

					validate_parameters;

					DAQmx.CreateTask (``, &task_handle);

					DAQmx.CreateAIVoltageChan (task_handle, 
						channel_string, ``, 
						DAQmx_Val_Cfg_Default, 
						input_voltage_range.min.to_scalar, 
						input_voltage_range.max.to_scalar, 
						DAQmx_Val_Volts, 
						null
					);

					DAQmx.CfgSampClkTiming (task_handle, 
						`OnboardClock`, 
						sampling_rate.to_scalar, 
						DAQmx_Val_Rising, 
						DAQmx_Val_ContSamps, 
						0
					);

					//DAQmx.CfgInputBuffer (task_handle, block_size (in_samples)); // REVIEW how big can this be and what does it affect?
					////////////////
					buffer = RingBuffer (buffer_size (in_doubles));

					int offset = 0;
					int stride = open_channels_count;

					foreach (channel; input_channels[])
						if (channel.is_open)
							{/*...}*/
								channel.offset = offset++;
								channel.stride = stride;
							}
				}
		}
		public {/*channels}*/
			public {/*open/close}*/
				void open_input_channel (Index i)
					in {/*...}*/
						assert (i < InputChannels.count);
					}
					body {/*...}*/
						input_channels[i].open = true;

						invalidate_parameters;
					}

				void close_input_channel (Index i)
					in {/*...}*/
						assert (i < InputChannels.count);
					}
					body {/*...}*/
						input_channels[i].open = false;

						invalidate_parameters;
					}

				void open_input_channels (Index[] indices...)
					{/*...}*/
						foreach (i; indices)
							open_input_channel (i);

						invalidate_parameters;
					}
				void open_input_channels ()
					{/*...}*/
						foreach (i; 0..InputChannels.count)
							open_input_channel (i);

						invalidate_parameters;
					}

				void close_input_channels (Index[] indices...)
					{/*...}*/
						foreach (i; indices)
							close_input_channel (i);

						invalidate_parameters;
					}
				void close_input_channels ()
					{/*...}*/
						foreach (i; 0..InputChannels.count)
							close_input_channel (i);

						invalidate_parameters;
					}
			}
			const {/*access}*/
				@property input ()
					{/*...}*/
						return input_channels[];
					}
			}

			class Channel
				{/*...}*/
					nothrow:
					const is_open ()
						{/*...}*/
							return open;
						}
					private {/*...}*/
						bool open;
						size_t offset; // REVIEW
						size_t stride; // REVIEW
					}


					const sample_by_index ()
						{/*...}*/
							return stream_from (&opIndex, &current_sample_count);
						}
					const sample_by_time ()
						{/*...}*/
							return stream_from (&opCall, &current_recording_length).at (sampling_frequency);
						}

					const opIndex (Index i)
						in {/*...}*/
							assert (this.is_open);
						}
						body {/*...}*/
							return buffer[i].volts;
						}
					const opCall (Seconds t)
						in {/*...}*/
							assert (this.is_open);
						}
						body {/*...}*/
							alias h = max_recording_length;

							auto x = current_sample_count * (t >= 0.seconds? t/h: (t+h)/h);

							return buffer[cast(size_t)x.floor].volts;
						}
				}
		}
		public {/*ctor}*/
			this ()
				{/*...}*/
					try {/*identify device}*/
						auto lsdaq = pipeShell (`lsdaq`).stdout.byLine.drop (3);

						version (NO_DAQ) {}
						else if (lsdaq.front.equal ('-'.repeat (32)))
							assert (0, `no DAQ devices detected`);
						
						foreach (device_info; lsdaq)
							{/*...}*/
								version (NO_DAQ) {}
								else if (device_info.equal ('-'.repeat (32)))
									assert (0, `couldn't find ` ~Model.name~ ` #` ~Serial.number.text~ ` among connected DAQ devices`);

								this.device_id = device_info.find (`Dev`)[0..4].text;
								uint serial_number;

								DAQmx.GetDevSerialNum (device_id, &serial_number);

								if (serial_number == Serial.number)
									break;
							}
					}
					catch (Exception) assert (0);

					foreach (ref channel; input_channels)
						channel = new Channel;
				}
		}
		private:
		private {/*data}*/
			private {/*status}*/
				bool recording;
			}
			private {/*parameters}*/
				auto input_voltage_range = MaxVoltageRange ();
				auto sampling_rate = MaxSamplingRate () / InputChannels.count;
				auto read_frequency = 30.hertz;
				auto min_recording_time = 1.second;
			}
			private {/*handles}*/
				string device_id;
				TaskHandle task_handle;
				Tid recording_thread;
			}
			private {/*channels}*/
				Channel[InputChannels.count] input_channels;
			}
			private {/*buffer}*/
				RingBuffer buffer;

				struct RingBuffer
					{/*...}*/
						double* buffer;
						const size_t capacity;
						size_t position;
						bool filled; // TODO length?

						this (size_t size)
							{/*...}*/
								buffer = cast(double*)malloc (size * double.sizeof);
								capacity = size;
							}
						~this ()
							{/*...}*/
								free (buffer);
							}

						auto opIndex (size_t i) const
							{/*...}*/
								return buffer[(position + i) % capacity];
							}
						double* input ()
							{/*...}*/
								return buffer + position;
							}
						void advance (size_t positions)
							{/*...}*/
								position += positions;

								if (not (filled) && position >= capacity)
									filled = true;

								position %= capacity;
							}

						bool is_ready () const
							{/*...}*/
								return buffer !is null;
							}
						size_t length () const
							{/*...}*/
								return filled? capacity: position;
							}

						invariant(){/*...}*/
							assert (position < capacity);
						}
					}
			}
		}
		private {/*capture}*/
			void capture ()
				{/*...}*/
					recording = true;

					do record_block; while (this.is_recording);

					try ownerTid.send (Status.terminated);
					catch (Exception) assert (0);
				}
			void record_block ()
				in {/*...}*/
					assert (this.is_recording);
				}
				body {/*...}*/
					int n_samples_read = 0;

					real timeout = 30.0;

					DAQmx.ReadAnalogF64 (task_handle,
						block_size (in_samples),
						timeout,
						DAQmx_Val_GroupByScanNumber,
						buffer.input,
						buffer_size (in_samples),
						&n_samples_read,
						null
					);

					assert (n_samples_read == block_size (in_samples));

					buffer.advance (block_size (in_doubles));
				}
		}
		private {/*parameter validation}*/
			void validate_parameters ()
				{/*...}*/
					if (sampling_frequency * open_channels_count <= MaxSamplingRate ()
						&& max_recording_length >= min_recording_length
						&& input_range.contained_in (MaxVoltageRange ())
					) parameters_invalidated = false;
					else assert (0, `invalid parameters`); // REVIEW
				}
			void invalidate_parameters ()
				{/*...}*/
					parameters_invalidated = true;
				}
			bool parameters_invalidated;
		}
		const @property {/*buffer block sizes}*/
			size_t block_size (size_t units)
				in {/*...}*/
					assert ((sampling_rate * read_frequency).to_scalar != 0);
				}
				body {/*...}*/
					return cast(size_t)((sampling_rate / read_frequency) * units).ceil;
				}
			size_t buffer_size (size_t units)
				in {/*...}*/
					assert (sampling_rate * min_recording_time != 0);
				}
				body {/*...}*/
					auto minimum_samples_to_buffer = min_recording_time * sampling_rate;

					auto M = minimum_samples_to_buffer;
					auto B = block_size (in_samples);

					auto samples_to_buffer = ceil (M/B) * B;

					return cast(size_t)(samples_to_buffer * units).ceil;
				}

			enum in_samples = 1;
			alias in_doubles = open_channels_count;
		}
		invariant (){/*}*/
			assert (block_size (in_samples) * read_frequency == buffer_size (in_samples)); // TODO
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
		struct MaxSamplingRate (uint max_sampling_rate)
			{/*...}*/
				enum rate = max_sampling_rate.hertz;

				enum isMaxSamplingRate;

				static opCall ()
					{/*...}*/
						return rate;
					}
			}
		struct MaxVoltageRange (real min_voltage, real max_voltage)
			{/*...}*/
				enum min = min_voltage.volts;
				enum max = max_voltage.volts;

				enum isMaxVoltageRange;

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
	}

struct Stream (Sample, Index)
	if (supports_arithmetic!Index)
	{/*...}*/
		nothrow:
		enum is_continuous = allSatisfy!(isFloatingPoint, RepresentationTypeTuple!Index);

		public:
		const {/*[┄]}*/
			@property opDollar ()
				in {/*...}*/
					assert (last_index !is null);
				}
				body {/*...}*/
					return last_index ();
				}

			auto opSlice ()
				in {/*...}*/
					assert (last_index !is null);
				}
				body {/*...}*/
					return Sampler!Stream (&this, zero!Index, last_index ());
				}

			auto opSlice (Index i, Index j)
				in {/*...}*/
					assert (source !is null);
				}
				body {/*...}*/
					return Sampler!Stream (&this, i, j);
				}

			auto opIndex (Index i)
				in {/*...}*/
					assert (source !is null);
				}
				body {/*...}*/
					debug try return source (i); // REVIEW
					catch (Exception) assert (0);
				}
		}
		public {/*frequency}*/
			static if (Stream.is_continuous)
				{/*...}*/
					alias Frequency = typeof(1.0/(Index.init));

					Frequency frequency = zero!Frequency;

					auto at (Frequency frequency)
						{/*...}*/
							this.frequency = frequency;

							return this;
						}
				}
		}
		private:
		private {/*signals}*/
			Sample delegate(Index) source;
			Index delegate() last_index;
		}
		private {/*ctor}*/
			this (Sample delegate(Index) nothrow source, Index delegate() nothrow last_index)
				{/*...}*/
					this.source = source;
					this.last_index = last_index;
				}
		}
	}
auto stream_from (F, G)(F source, G max)
	if (allSatisfy!(isSomeFunction, F, G))
	{/*...}*/
		static assert (ParameterTypeTuple!F.length == 1);
		static assert (ParameterTypeTuple!G.length == 0);

		static assert (is(ParameterTypeTuple!F[0] == ReturnType!G));

		return Stream!(ReturnType!F, ReturnType!G)(source.toDelegate, max.toDelegate);
	}

struct Sampler (Stream)
	{/*...}*/
		nothrow:
		alias Index = ReturnType!(Stream.opDollar);
		alias Sample = ReturnType!(Stream.opIndex);

		public:
		public {/*[┄]}*/
			@property length () const
				{/*...}*/
					return last;
				}
			static assert (has_length!Sampler);

			alias opDollar = length;

			auto opSlice ()
				{/*...}*/
					return this;
				}

			auto opSlice (Index i, Index j)
				{/*...}*/
					return Sampler (stream, i, j);
				}

			auto opIndex (Index i)
				{/*...}*/
					return (*stream)[first + i];
				}
		}
		public {/*InputRange}*/
			Sample front ()
				{/*...}*/
					return this[first];
				}

			void popFront ()
				in {/*...}*/
					assert (stride != zero!Index, no_sampling_frequency_error);
				}
				body {/*...}*/
					first += stride;
				}

			bool empty () const
				{/*...}*/
					return first >= last;
				}
		}
		public {/*BidirectionalRange}*/
			Sample back ()
				{/*...}*/
					return this[last - unity!Index];
				}
			void popBack ()
				in {/*...}*/
					assert (stride != zero!Index, no_sampling_frequency_error);
				}
				body {/*...}*/
					last -= stride;
				}
		}
		public {/*ForwardRange}*/
			@property save ()
				{/*...}*/
					return this;
				}
		}
		public {/*stride}*/
			static if (Stream.is_continuous)
				{/*...}*/
					auto at (Stream.Frequency frequency)
						{/*...}*/
							this.stride = 1.0/frequency;

							return this;
						}
				}
		}
		private:
		private {/*ctor}*/
			this (const(Stream)* stream, Index first, Index last)
				{/*...}*/
					this.first = first;
					this.last = last;

					static if (Stream.is_continuous)
						this.stride = 1.0/stream.frequency;

					this.stream = stream;
				}
		}
		private {/*data}*/
			Index first = zero!Index;
			Index last = zero!Index;

			static if (Stream.is_continuous)
				Index stride = zero!Index;
			else enum stride = unity!Index;

			const(Stream)* stream;
		}
		private {/*error message}*/
			enum no_sampling_frequency_error = `for floating-point samplers, sampling frequency must be set with "sampler.at (f)" before use`;
		}
	}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

struct ForcePlate
	{/*...}*/
		nothrow:
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

void main ()
	{/*...}*/
		auto daq = new DAQDevice!(
			Model!`NI USB-6216`,

			InputChannels!8,
			OutputChannels!2,

			MaxSamplingRate!(250_000),
			MaxVoltageRange!(-10, 10),

			Serial!0x_18DEF36,
			InputBufferSize!4095,
		); /* source: http://www.ni.com/pdf/manuals/371932f.pdf */

		immutable TEMP = 1.0;

		ForcePlate plate;
			with (plate) {/*...}*/
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

			auto x = plate.moment[-1.minute..$];
	}

static if (0)
	{/*...}*/
		auto integrate (Stream)(Sampler!Stream sampler)
			if (Stream.is_continuous)
			{/*...}*/
				immutable t0 = sampler.first;
				immutable t1 = sampler.last;
				immutable Δt = sampler.stride;

				size_t i = 0;
				T accumulator = zero!T;

				for (auto t = t0; (t < t1); t = t0 + (++i)*Δt)
					accumulator += sampler[t] * Δt;

				return accumulator;
			}
	}
////////////////
////////////////

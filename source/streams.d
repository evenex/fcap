module evx.streams;

private {/*import std}*/
	import std.traits:
		isFloatingPoint,
		isSomeFunction, ReturnType, ParameterTypeTuple,
		isBuiltinType, RepresentationTypeTuple;

	import std.typetuple:
		allSatisfy;

	import std.functional:
		toDelegate;

	import std.conv:
		to;
}
private {/*import evx}*/
	import evx.traits: 
		supports_arithmetic;

	import evx.algebra:
		zero, unity;

	import evx.analysis:
		interval, is_continuous;
}

struct Stream (Sample, Index)
	if (supports_arithmetic!Index)
	{/*...}*/
		nothrow:
		enum is_continuous = .is_continuous!Index;

		public:
		const {/*[┄]}*/
			@property opDollar ()
				in {/*...}*/
					assert (last_index !is null, `attempt to access stream before ready`);
				}
				body {/*...}*/
					return last_index ();
				}

			auto opSlice ()
				in {/*...}*/
					assert (last_index !is null, `attempt to access stream before ready`);
				}
				body {/*...}*/
					return Sampler!Stream (this, zero!Index, last_index ());
				}

			auto opSlice (Index i, Index j)
				in {/*...}*/
					assert (source !is null, `attempt to access stream before ready`);
				}
				body {/*...}*/
					return Sampler!Stream (this, i, j);
				}

			auto opIndex (Index i)
				in {/*...}*/
					assert (source !is null, `attempt to access stream before ready`);
				}
				body {/*...}*/
					return source (i);
				}
		}
		public {/*frequency}*/
			static if (Stream.is_continuous)
				{/*...}*/
					alias Frequency = typeof(1.0/(Index.init));

					Frequency delegate() frequency;

					auto at (Frequency delegate() nothrow frequency)
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

		static assert (is(ReturnType!G : ParameterTypeTuple!F[0]));

		return Stream!(ReturnType!F, ReturnType!G)(source.toDelegate, max.toDelegate);
	}

struct Sampler (Stream)
	{/*...}*/
		nothrow:
		alias Index = ReturnType!(Stream.opDollar);
		alias Sample = ReturnType!(Stream.opIndex);

		public:
		public {/*[┄]}*/
			static if (Stream.is_continuous)
				{/*...}*/
					@property measure () const
						{/*...}*/
							return last - first;
						}

					alias opDollar = measure;
			}
			else {/*...}*/
				@property length () const
					{/*...}*/
						return last - first;
					}

				alias opDollar = length;
			}

			auto opSlice ()
				{/*...}*/
					return Sampler (this, first, last);
				}

			auto opSlice (Index i, Index j)
				{/*...}*/
					return Sampler (this, first + i, first + j);
				}

			auto opIndex (Index i)
				{/*...}*/
					return source (first + i);
				}
		}
		public {/*InputRange}*/
			Sample front ()
				{/*...}*/
					return this[zero!Index];
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
					return first + stride >= last;
				}
		}
		public {/*BidirectionalRange}*/
			Sample back ()
				{/*...}*/
					return this[$ - stride];
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
		public {/*frequency}*/
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
			this (Stream stream, Index first, Index last)
				{/*...}*/
					this.first = first;
					this.last = last;

					static if (Stream.is_continuous)
						{/*...}*/
							this.frequency = stream.frequency;
							this.stride = 1 / frequency ();
						}

					this.source = stream.source;
				}
			this (Sampler that, Index first, Index last)
				{/*...}*/
					this = that;

					this.first = first;
					this.last = last;

					static if (Stream.is_continuous)
						this.stride = 1 / frequency ();
				}
		}
		private {/*data}*/
			Index first = zero!Index;
			Index last = zero!Index;

			static if (Stream.is_continuous)
				Index stride = zero!Index;
			else enum stride = unity!Index;

			Sample delegate(Index) source;

			static if (Stream.is_continuous)
				Stream.Frequency delegate() frequency;
		}
		private {/*error message}*/
			enum no_sampling_frequency_error = `for floating-point samplers, sampling frequency must be set with "sampler.at (f)" before use`;
		}
	}

// NATIONAL INSTRUMENTS
#include "/usr/local/natinst/nidaqmxbase/include/NIDAQmxBase.h"
// LIBC
#include <cmath>
#include <cassert>
#include <unistd.h>

// STL
#include <iostream>
#include <fstream>
#include <functional>
#include <algorithm>
#include <initializer_list>
#include <random>
#include <vector>
#include <chrono>
#include <thread>
#include <atomic>
// GSL
#include <gsl/gsl_fit.h>
// GRAPHICS
typedef std::ptrdiff_t ptrdiff_t;
#include <opencv/cv.h>
#include <opencv/highgui.h>
// EVX
#include "../evenex/evenex.h"
/////////////////////
static_assert (sizeof (uInt32) is sizeof (uint), "");
/////////////////////
typedef double real;
const real PI = 3.1415;
/////////////////////

//|||||||||||||||
//|| UTILITIES ||
//|||||||||||||||
template <uint base, uint exponent> 
struct Power
	{static const uint size = base * Power <base, exponent-1>::size;};
template <uint base>
struct Power <base, 0>
	{static const uint size = 1;};
////////////////
const uint Kbyte = Power <2, 10>::size;
const uint Mbyte = Power <2, 20>::size;
const uint Gbyte = Power <2, 30>::size;
////////////////
static std::string date_and_time (void)
	{/*...}*/
		auto now = std::chrono::system_clock::now ();
		auto now_tt = std::chrono::system_clock::to_time_t (now);
		auto ctime = std::ctime (from now_tt);
		std::string date_time = std::string (ctime);
		for (char& c in date_time)
			if (c is ' ') c = '_';
		date_time.pop_back ();
		return date_time;
	}
////////////////

//||||||||||||||||||||||
//|| DAQ ERROR ASPECT ||
//||||||||||||||||||||||
#define DAQmx(...) \
	({/*error check})
	*/	uint status = DAQmxBase ## __VA_ARGS__; 	\
		if (DAQmxFailed (status))	\
			{/*...}*/	\
				char error_string [512];	\
				DAQmxBaseGetExtendedErrorInfo (error_string, size_of (error_string));	\
				std::cerr << "Device error at " << __FILE__ << ":" << __LINE__ << ": " << error_string << std::endl;	\
				abort ();	\
			}	\
	})

//|||||||||||||
//|| VECTORS ||
//|||||||||||||
template <typename _t>
struct Vector
	{_t x, y, z;/*}*/
		Vector <_t> (_t x, _t y, _t z = 0):
			x (x), y (y), z (z)
			{}
		Vector <_t> (void):
			x (0), y (0), z (0)
			{}
		Vector <_t> operator + (const Vector <_t> v) const
			{return Vector <_t> (x + v.x, y + v.y, z + v.z);}
		Vector <_t> operator - (void) const
			{return Vector <_t> (-x, -y, -z);}
		Vector <_t> operator - (const Vector <_t> v) const
			{return me + -v;}
		Vector <_t> operator * (const Vector <_t> v) const
			{return Vector <_t> (x * v.x, y * v.y, z * v.z);}
		Vector <_t> operator * (_t a) const
			{return Vector <_t> (a*x, a*y, a*z);}
		Vector <_t> operator / (_t a) const
			{return Vector <_t> (x/a, y/a, z/a);}
		bool operator is (const Vector <_t> v) const
			{return (x is v.x and y is v.y and z is v.z);}
		_t length (void) const
			{/*...}*/
				Vector <_t> L = me * me;
				return sqrt (L.x + L.y + L.z);
			}
		Vector <_t> normalized (void) const
			{/*...}*/
				_t L = my length ();
				if (L is 0.0) L = 1.0;
				return Vector <_t> (x/L, y/L, z/L);
			}
		template <typename _t2>
		operator Vector <_t2> (void)
			{/*...}*/
				_t2 x, y, z;
				x = my x;
				y = my y;
				z = my z;
				return (Vector <_t2> (x, y, z));
			}
		template <typename _t2>
		Vector <_t> (Vector <_t2> v)
			{/*...}*/
				my x = v.x;
				my y = v.y;
				my z = v.z;
			}
	};
///////////////
typedef Vector <real> vec;
typedef Vector <uint> uvec;
typedef Vector <int> ivec;

//|||||||||||||||||||||||
//|| LINEAR REGRESSION ||
//|||||||||||||||||||||||
template <typename... Dependents>
void build_linear_coefficients (std::vector<real>& coeffs, std::vector <real>& independent, std::vector<real>& dependent, Dependents&... dependents)
	{/*...}*/
		build_linear_coefficients (coeffs, independent, dependent);
		build_linear_coefficients (coeffs, independent, dependents...);
	}
template <>
void build_linear_coefficients (std::vector<real>& coeffs, std::vector <real>& independent, std::vector<real>& dependent)
	{/*...}*/
		real coeff;
		real __; real* _ = &__;
		gsl_fit_linear (independent.data (), 1, dependent.data (), 1, independent.size (), _, to coeff, _,_,_,_);
		coeffs.push_back (coeff);
	}
template <typename... Dependents > 
std::vector <real> linear_coefficients (std::vector <real>& independent, Dependents&... dependents)
	{/*...}*/
		std::vector <real> coeffs;
		build_linear_coefficients (coeffs, independent, dependents...);
		return coeffs;
	}

//|||||||||
//|| DAQ ||
//|||||||||
struct Device;
struct Channel;
////////////////
typedef int Index;
////////////////
struct Buffer
	{/*...}*/
		friend Device;
		friend Channel;
		public:// constructors -------------------------------
			Buffer (void)
				{}
			Buffer (uint length):
				data ((real*) malloc (length * size_of (real))),
				length (length)
				{std::fill_n (data, length, NAN);}
			Buffer& operator = (Buffer&& that)
				{/*...}*/
					if (my data exists)
						free (my data);
					my data = that.data;
					that.data = nullptr;
					my write_position = that.write_position ;
					my length = that.length;
					return me;
				}
			~Buffer (void)
				{if (data exists) free (data);}
		private:// access -------------------------------------
			real operator [] (Index sample) const
				{/*...}*/
					unless (ready ())
						return NAN;
					if ((uint) abs (sample) - (sample < 0? 1:0) exceeds my length)
						return NAN;
					else return data [(my write_position + my length + sample) % length];
				}
			void advance (Index advancement)
				{/*...}*/
					unless (ready ()) 
						return;
					my write_position += advancement;
					my write_position %= my length;
				}
			real* input (void) const
				{/*...}*/
					unless (ready ()) 
						return nullptr;
					else return address_of (data [my write_position]);
				}
		public: // status -------------------------------------
			bool ready (void) const
				{return data exists and length > 0;}
		private:// data ---------------------------------------
			real* data = nullptr;
			Index write_position = 0;
			uint length = 0;
		public: // debug output -------------------------------
			std::string info_string (void) const
				{/*...}*/
					std::string ret;
					ret += "Buffer:\n";
					if (not ready ())	return  ret += "not ready\n";
					ret += "length = ";		ret += std::to_string (length);		ret += "\n";
					ret += "write_position = ";	ret += std::to_string (write_position);	ret += "\n";
					return ret;
				}
	};
////////////////
struct Channel
	{/*...}*/
		friend Device;
		typedef uint Number;
		public: // access -------------------------------------
			real operator [] (Index sample) const
				{if (ready ()) return (the buffer) [stride*sample + offset];/*}*/
				else return NAN;
				}
			real operator () (real time) const
				{if (ready ()) return me [time * sampling_rate];/*}*/
				else return NAN;
				}
		private:// settings -----------------------------------
			const Buffer* buffer;
			int stride = -1;
			int offset = -1;
			int sampling_rate = 0;
		protected:
			Channel::Number number;
		protected:
			Channel (Channel::Number n, Buffer& buffer):
				buffer (to buffer),
				number (n)
				{}
		private:// status -------------------------------------
			bool ready (void) const
				{return stride >= 0 and offset >= 0 and sampling_rate > 0;}
		private:// nidaqmx channel id -------------------------
			virtual std::string string (void) const = 0;
		public: // compare ------------------------------------
			bool operator < (const Channel& that) const
				{return my number < that.number;}
			bool operator is (Channel::Number n) const
				{return my number is n;}
		public: // debug output -------------------------------
			std::string info_string (void) const
				{/*...}*/
					std::string ret;
					ret += "Channel ";	   ret += std::to_string (number);		ret += ":\n";
					if (not ready ())   return ret += "not ready\n";
					ret += "stride = ";	   ret += std::to_string (stride);		ret += " doubles\n";
					ret += "offset = ";	   ret += std::to_string (offset);		ret += " doubles\n";
					ret += "sampling rate = "; ret += std::to_string (sampling_rate);	ret += " Hz\n";
					return ret;
				}
	};
struct Input: Channel
	{/*...}*/
		friend Device;
		private:// nidaqmx channel id -------------------------
			std::string string (void) const override
				{/*...}*/
					return "ai" + std::to_string (my number);
				}
		public: // constructor --------------------------------
			Input (Channel::Number n, Buffer& buffer):
				Channel (n, buffer)
				{}
	};
struct Output: Channel
	{/*...}*/
		friend Device;
		public: // settings -----------------------------------
			void set_voltage (real min, real max)
				{/*...}*/
					my voltage.min = min;
					my voltage.max = max;
				}
		private:// nidaqmx channel id -------------------------
			std::string string (void) const override
				{/*...}*/
					return "ao" + std::to_string (my number);
				}
		private:// data ---------------------------------------
			struct {/*voltage;}*/
				real min, max;
			} voltage;
		public: // constructor --------------------------------
			Output (Channel::Number n, Buffer& buffer):
				Channel (n, buffer)
				{}
		
	};
////////////////
struct Device
	{/*...}*/
		public: // constructor --------------------------------
			Device (void)
				{/*...}*/
					my device_number = 1; // TEMP
					DAQmx (GetDevSerialNum (my device_string ().c_str (), (uInt32*) to my serial_number));
				}
		public: // settings -----------------------------------
			void open_input_channel (Channel::Number n)
				{open_input_channels ({n});}
			void open_input_channels (std::initializer_list <Channel::Number> numbers)
				{/*...}*/
					for (Channel::Number n in numbers)
						{/*open channel}*/
							auto it = find (in_the (inputs), n);
							assert (it is not_in (inputs));
							inputs.emplace_back (n, buffer);
						}
					parameters_invalidated = yes;
				}
			void set_sampling_rate (uint rate)
				{my sampling_rate = rate;/*}*/
					parameters_invalidated = yes;
					assert (not recording);
				}
			void set_read_frequency (uint rate)
				{my read_frequency = rate;/*}*/
					parameters_invalidated = yes;
					assert (not recording);
				}
			void set_recording_time (real time)
				{my min_recording_time = time;/*}*/
					parameters_invalidated = yes;
					assert (not recording);
				}
			void set_input_voltage (real min, real max)
				{my input_voltage = (Voltage){min, max};/*}*/
					parameters_invalidated = yes;
					assert (not recording);
					if (max < min)
						my input_voltage.min = max,
						my input_voltage.max = min;
				}
		public: // adjusted settings --------------------------
			real recording_time (void) const
				{return (1.0 * buffer_size (in_samples)) / sampling_rate;}
		public: // access -------------------------------------
			const Input& input (Channel::Number n) const
				{/*...}*/
					auto it = find (in_the (inputs), n);
					bool channel_is_open = it is not_in (inputs)? no: yes;
					assert (channel_is_open);
					return *it;
				}
		public: // control ------------------------------------
			void record (void)
				{/*...}*/
					auto capture = [&] (void)
						{do record_block (); while (recording);};
					recording_thread = std::thread (capture);
				}
			void stop (void)
				{/*...}*/
					if (recording)
						{/*...}*/
							recording = no;
							recording_thread.join ();
							DAQmx (StopTask (task_handle));
							DAQmx (ClearTask (task_handle));
							//capture_callback = nullptr; TODO dunno if i want to do this
						}
				}
			std::function <void (uint)> capture_callback;
		private:// control ------------------------------------
			std::thread recording_thread;
			void reset (void)
				{/*...}*/
					my buffer = Buffer (buffer_size (in_doubles));
					sort (all_of_the (inputs));
					int offset = 0;
					int stride = inputs.size ();
					for (Channel& channel in inputs) 
						{/*set access parameters}*/
							channel.offset = offset++;
							channel.stride = stride;
							channel.sampling_rate = my sampling_rate;
						};
					parameters_invalidated = no;
					assert (ready ());
				}
			bool ready (void)
				{/*...}*/
					if (parameters_invalidated)
						return no;
					unless (sampling_rate > 0 and read_frequency > 0)
						return no;
					unless (buffer.ready ())
						return no;
					for (Channel& channel in inputs)
						if (channel.ready ()) 
							continue;
						else return no;
					otherwise return yes;
				}
			void record_block (void)
				{/*...}*/
					unless (recording)
						{/*start}*/
							unless (ready ()) 
								reset ();
							std::string input_string;
								{/*...}*/
									for (Channel& channel in inputs) 
										input_string += my channel_string (channel) + ", ";
									input_string.pop_back (), input_string.pop_back ();
								}
							DAQmx (CreateTask ("", to task_handle));
							DAQmx (CreateAIVoltageChan (task_handle, input_string.c_str (), "", DAQmx_Val_Cfg_Default, input_voltage.min, input_voltage.max, DAQmx_Val_Volts, NULL));
							DAQmx (CfgSampClkTiming (task_handle, "OnboardClock", sampling_rate, DAQmx_Val_Rising, DAQmx_Val_ContSamps, 0));
							DAQmx (CfgInputBuffer (task_handle, block_size (in_samples)));
							/// AND NOW
							DAQmx (StartTask (task_handle));
							recording = yes;
						}
					long int n_samples_read = 0;
					real time_out = 30.0;
					DAQmx (ReadAnalogF64 (task_handle, block_size (in_samples), time_out, DAQmx_Val_GroupByScanNumber, buffer.input (), buffer_size (in_samples), to n_samples_read, NULL));
					assert ((uint) n_samples_read is block_size (in_samples));
					buffer.advance (block_size (in_doubles));
					if (capture_callback exists)
						capture_callback (block_size (in_samples));
				}
		private:// status -------------------------------------
			bool recording = no;
			bool parameters_invalidated = no;
		private:// input parameters ---------------------------
			uint sampling_rate  = 16384;
			uint read_frequency = 30;
			real min_recording_time = 1.0;
			struct Voltage 
				{/*input_voltage;}*/
					real min = -5.0;
					real max = 5.0;
					Voltage (real min, real max):
						min (min), max (max)
						{}
					Voltage (void)
						{}
				} input_voltage;
		private:// units --------------------------------------
			typedef std::function <uint (void)> Units;
			Units in_samples = [this] (void) 
				{return 1;};
			Units in_doubles = [this] (void)
				{return inputs.size ();};
			Units in_bytes   = [this] (void)
				{return in_doubles () * size_of (real);};
		private:// derived parameters -------------------------
			uint block_size (Units units) const
				{/*...}*/
					assert (sampling_rate * read_frequency isnt 0);
					return (sampling_rate / read_frequency) * units ();
				}
			uint buffer_size (Units units) const
				{/*...}*/
					assert (sampling_rate isnt 0 and min_recording_time isnt 0);
					real minimum_samples_to_buffer = min_recording_time * sampling_rate;
					real& M = minimum_samples_to_buffer;
					uint B = block_size (in_samples);
					uint samples_to_buffer = ceil (M/B) * B;
					return samples_to_buffer * units ();
				}
		private:// nidaqmx parameters-------------------------
			uint device_number = 0;
			uint serial_number = 0;
			TaskHandle task_handle;
		private:// nidaqmx channel string ---------------------
			std::string device_string (void)
				{return "Dev" + std::to_string (device_number);}
			std::string channel_string (Channel& channel) 
				{return device_string () + "/" + channel.string ();}
		private:// data ---------------------------------------
			std::vector <Input> inputs;
			std::vector <Output> outputs;
			Buffer buffer; // TODO input and output buffers
		public: // debug output -------------------------------
			std::string info_string (void) const
				{/*...}*/
					std::string ret;
					ret += "Device "; 		ret += std::to_string (device_number);			ret += ":\n";
					ret += "sampling rate = ";	ret += std::to_string (sampling_rate);			ret += " Hz\n";
					ret += "recording time = ";	ret += std::to_string (recording_time ());  		ret += " seconds\n"; 
					ret += "read frequency = ";	ret += std::to_string (read_frequency);  		ret += " reads/sec\n";  
					ret += "block size = ";		ret += std::to_string (block_size (in_samples));	ret += " samples\n";  
					ret += "buffer size = ";	ret += std::to_string (buffer_size (in_samples));	ret += " samples ";
					ret += "("; ret+= std::to_string (buffer_size (in_samples)/block_size (in_samples));	ret += " blocks)\n";
					ret += "input = ";		ret += std::to_string (inputs.size ());			ret += inputs.size () is 1? " channel\n": " channels\n";  
					for (const Input& channel in inputs)
						ret += channel.info_string ();
					return ret;
				}
	};
struct Link
	{/*...}*/
		public: // access -------------------------------------
			real operator [] (Index sample) const
				{return device.input (channel_number) [sample];}
			real operator () (real time) const
				{return device.input (channel_number) (time);}
		public: // constructor --------------------------------
			Link (Device& device, Channel::Number channel_number):
				device (device), 
				channel_number (channel_number)
				{}
		private:// data ---------------------------------------
			Device& device;
			// NOTE assuming input only
			Channel::Number channel_number;
	};
////////////////

//|||||||||||||||||
//|| FORCE PLATE ||
//|||||||||||||||||
struct Plate
	{/*...}*/
		Link x;
		Link y;
		Link z;
		Plate (Device& daq, Channel::Number x, Channel::Number y, Channel::Number z):
			x (daq, x), y (daq, y), z (daq, z)
			{}
		vec operator [] (Index sample)
			{return (vec) {x [sample], y [sample], z [sample]};}
		vec operator () (real time)
			{return (vec) {x (time), y (time), z (time)};}
	};

//||||||||||||||||||
//|| VISUALIZATON ||
//||||||||||||||||||
struct Color
	{/*...}*/
		real red;
		real green;
		real blue;
		Color (real red, real green, real blue):
			red (red),  green (green),  blue (blue)
			{}
		operator CvScalar (void) const
			{/*...}*/
				int R = 255*red;
				int G = 255*green;
				int B = 255*blue;
				return cvScalar (B,G,R,0);
			}
		friend Color operator * (Color color, real brightness);
		friend Color operator * (real brightness, Color color);
	};
Color operator * (Color color, real brightness)
	{/*...}*/
		return Color (brightness*color.red, brightness*color.green, brightness*color.blue);
	}
Color operator * (real brightness, Color color)
	{/*...}*/
		return color * brightness;
	}
////////////////
struct Visualization
	{/*...}*/
		const Link* input;
		void link (Link& input)
			{my input = to input;}

		Color color = Color (0.0, 0.7, 0.0);
		void set_color (Color color)
			{my color = color;}

		std::string label;
		void set_label (std::string&& label)
			{my label = label;}
		void set_label (std::string& label)
			{my label = label;}

		real history;
		void set_history (real history)
			{my history = history;}

		real amplitude;
		void set_amplitude (real amplitude)
			{my amplitude = amplitude;}

		Visualization (const Link* input = nullptr, std::string label = "signal", real history = 1.0, real amplitude = 5.0):
			input (input),
			label (label),
			history (history),
			amplitude (amplitude)
			{}
	};
////////////////
struct Scope: Visualization
	{/*...}*/
		Scope (const Link* input = nullptr, std::string label = "signal", real history = 1.0, real amplitude = 5.0):
			Visualization (input, label, history, amplitude)
			{}
	};
struct Gauge: Visualization
	{/*...}*/
		Gauge (const Link* input = nullptr, std::string label = "signal", real history = 1.0, real amplitude = 5.0):
			Visualization (input, label, history, amplitude)
			{}
	};
////////////////
struct Display
	{/*...}*/
		public: // access ----------------------------
			uint width (void) const {return _width;}
			uint height (void) const {return _height;}
			IplImage* canvas (void) const {return _canvas;}
		public: // constructor -----------------------
			Display (uint width = 1200, uint height = 600):
				_width (width),
				_height (height)
				{/*...}*/
					CvSize 	size 	 = cvSize (width, height);
					uint	depth 	 = IPL_DEPTH_8U;
					uint 	channels = 3;

					_canvas = cvCreateImage (size, depth, channels);

					cvNamedWindow ("fcap", CV_WINDOW_AUTOSIZE);
					cvMoveWindow ("fcap", 0,0);
				}
		public: // drawing routines
			void draw_line (uvec initial, uvec final, Color color = Color (0.5, 0.5, 0.5), int thickness = 1) 
				{/*...}*/
					cvLine (canvas (), cvPoint (initial.x, initial.y), cvPoint (final.x, final.y), /*...)*/
						color, thickness, 8, 0
					);
					return;
				};
			void draw_line (uvec* initial, uvec* final, Color color = Color (0.5, 0.5, 0.5), int thickness = 1) 
				{/*...}*/
					draw_line (the initial, the final, color, thickness);
				}
			void draw (Scope& scope, real x_left = 0, real y_top = 0, real x_scale = 1.0, real y_scale = 1.0, uint samples = 100)
				{/*...}*/
					if (scope.input is nullptr) return;
					uint x_0 = x_left  * width ();
					uint y_0 = y_top   * height ();
					uint x_s = x_scale * width ();
					uint y_s = y_scale * height ();
					{/*draw labels}*/
						write (scope.label, uvec (x_0, y_0), scope.color);
					}
					{/*draw baseline}*/
						draw_line (uvec (x_0, y_0+y_s/2), uvec (x_0 + x_s, y_0 + y_s/2), 0.2*scope.color);
					}
					{/*draw samples}*/
						real t_H = scope.history;
						real dt = t_H / samples;
						/////////
						auto point = [&] (real t) 
							{/*...}*/
								const auto& sample = the scope.input;
								uint x = x_0 + x_s*(t/t_H);
								uint y = y_0 + (1 + sample (-t)/scope.amplitude) * y_s * 0.5;
								return uvec (x,y);
							};
						/////////
						real t = t_H - 1*dt; 
						for (uint i = t_H/dt -1; i --> 1; t = i*dt)
							draw_line (point (t), point (t - dt), scope.color, 2);
					}
					{/*draw borders}*/
						draw_line (uvec (x_0+x_s, y_0),     uvec (x_0+x_s, y_0+y_s));
						draw_line (uvec (x_0+x_s, y_0+y_s), uvec (x_0,     y_0+y_s));
						draw_line (uvec (x_0,     y_0+y_s), uvec (x_0,     y_0));
						draw_line (uvec (x_0,     y_0),     uvec (x_0+x_s, y_0));
					}
			}
			void draw (Gauge& gauge, real x_left = 0, real y_top = 0, real x_scale = 1.0, real y_scale = 1.0, uint samples = 100)
				{/*...}*/
					if (gauge.input is nullptr) return;
					uvec center ((x_left + x_scale/2)*width (), (y_top +  y_scale/2)*height ());
					auto in_pixels = [&] (real radius) {return radius * std::min (x_scale * width (), y_scale * height ());};
					enum Interior {filled, hollow};

					auto draw_circle = [&] (Interior interior, real radius)
						{/*...}*/
							cvCircle (/*...)*/
								canvas (), 
								cvPoint (center.x, center.y),
								in_pixels (radius),
								gauge.color,
								interior is filled? -1:1,
								8, 0
							);
						};
					auto radial_position = [&] (real theta, real radius)
						{/*...}*/
							uint x = ( cos (theta) * in_pixels (radius)/2);
							uint y = (-sin (theta) * in_pixels (radius)/2);
							return uvec (x,y) + center;
						};
					auto around_the_circle = [&] (std::function <void (uvec)> action, uint frequency, real radius)
						{/*...}*/
							real theta_0 = (5.0/4)*PI;
							real d_theta = (3.0/2)*PI / frequency;
							for (uint i = 0; i < frequency; i++)
								{/*...}*/
									real theta = theta_0 - i*d_theta;
									action (radial_position (theta, radius));
								}
						};
					{/*draw center}*/
						draw_circle (filled, 0.01);
					}
					{/*draw border}*/
						draw_circle (hollow, 0.5);
					}
					{/*draw ticks}*/
						uint frequency = 10;
						real value = 0.0;
						auto draw_tick = [&] (uvec pos)
							{/*...}*/
								ivec to_center = ivec (center - pos)/20;
								uvec outer = uvec (ivec (pos) - to_center);
								uvec inner = uvec (ivec (pos) + to_center);
								draw_line (from outer, to inner, with gauge.color);
							};
						auto label_tick = [&] (uvec pos) // XXX hack
							{/*...}*/
								pos = pos - uvec (in_pixels (0.03), 0);
								std::string value_string = std::to_string (value * 50);
								for (int i = value_string.size (); i --> 0;)
									if (value_string [i] is '.')
										{/*...}*/
											value_string [i] = '\0'; 
											break;
										}
								write (value_string, pos, gauge.color);
								value += gauge.amplitude / frequency;
							};
						around_the_circle (draw_tick, frequency, 1.0);
						around_the_circle (label_tick, frequency, 0.9);
					}
					{/*draw gauge label}*/
						write (gauge.label, uvec (x_left, y_top), gauge.color);
					}
					{/*draw needle}*/
						auto average = [&gauge] (uint samples)
							{/*...}*/
								real sum = 0.0;
								for (real t = gauge.history; t >= 0.0; t -= gauge.history / samples)
									sum += (the gauge.input) (-t);
								return sum / samples;
							};
						real level = average (samples) / gauge.amplitude;
						real angle = -(level * (3.0/2)*PI) + (5.0/4)*PI;
						uvec final = radial_position (angle, 0.8);
						draw_line (from center, to final, with gauge.color, 4);
					}
				}
			void write (std::string label, uvec pos = uvec (0,0), Color color = Color (0.0, 0.7, 0.0))
				{/*...}*/
					static CvFont font;
					static uint font_size; 
					init {/*...}*/
						cvInitFont (/*...)*/
							&font, CV_FONT_HERSHEY_SIMPLEX,
							0.5, 0.5, 0.0, 1, 8
						);
						{/*get font size}*/
							CvSize font_size_2D;
							int _;
							cvGetTextSize ("I", from font, to font_size_2D, &_);
							font_size = font_size_2D.height;
						}
					}
					cvPutText (canvas (), label.c_str (), /*...)*/
						cvPoint (pos.x, pos.y + font_size+1),
						from font, with color
					);
				}
		public: // control ---------------------------
			void render (bool clear_display = true)
				{/*...}*/
					{/*output to screen}*/
						cvShowImage ("fcap", _canvas);
						if (clear_display) cvZero (_canvas);
					}
					{/*return keypress}*/ 
						int key = cvWaitKey (2);
						int key_pressed = key + 1;
						if (key_pressed and keypress_callback exists)
							keypress_callback (key);
					}
				}
			void save (std::string filename = "")
				{/*...}*/
					if (filename.empty ())
						filename = std::string ("fcap_display_") + date_and_time () + ".png";
					cvSaveImage (filename.c_str (), my canvas (), nullptr);
				}
			std::function <void (char)> keypress_callback;
		private:// data ------------------------------
			IplImage* _canvas;
			uint _width;
			uint _height;
	};
////////////////

//||||||||||||||||||
//|| UNIT TESTING ||
//||||||||||||||||||
namespace Test
	{/*...}*/
		void sampling (uint sampling_rate = 1000, real recording_time = 1.0, uint read_frequency = 100, std::initializer_list <Channel::Number> channels = {0})
			{/*...}*/
				std::clog << "sampling test:" << std::endl;
				std::clog << "sampling rate = " << sampling_rate << std::endl
					<< "recording time = " << recording_time << std::endl
					<< "read frequency = " << read_frequency << std::endl;

				Device daq;
				daq.open_input_channels (channels);

				uint random_channel;
					{/*...}*/
						std::mt19937 rng (std::chrono::system_clock::now ().time_since_epoch ().count ());
						std::uniform_int_distribution <uint> dist (0, channels.size ()-1);
						uint random_index = dist (rng);
						for (const Channel::Number& channel in channels)
							if (random_index --> 0)
								continue;
							else {/*select channel}*/
								random_channel = channel;
								break;
							}
					}
				Link signal (daq, random_channel);

				uint&  f = sampling_rate;
				real& t = recording_time;
				uint&  s = read_frequency;
				daq.set_sampling_rate  (f);
				daq.set_recording_time (t);
				daq.set_read_frequency (s);

				/* a Device may record longer than requested in order to sample evenly */
				assert (daq.recording_time () >= t);
				if (t isnt daq.recording_time ())
					std::clog << "recording time has been extended" << std::endl,
					t = daq.recording_time ();

				/* this loop records at least enough blocks to fill the buffer */
				for (uint i = ceil (s*t) + 1; i --> 0;)
					daq.record ();
				daq.stop ();

				/* a Link may be read with positive or negative sample offsets */
				/* negative offsets are taken relative to the newest sample */
				/* non-negative offsets are taken relative to the oldest sample */
				uint n = f*t;
				assert (signal [-n] is signal [0]);
				assert (signal [-1] is signal [n-1]);
				for (uint i = n-1; i --> 1;)
					assert (signal [-n+i] is signal [i]);

				/* attempting to read beyond the limits of the sample buffer returns a NaN */
				assert (isnan (signal [n]));
				assert (isnan (signal [-n-1]));

				/* a Link may also be read with a time offset */
				assert (signal (-t) is signal (0));
				/* but due to floating-point noise, access is not symmetric as with sample offsets */
				real u, err = 0;
				real dt = 1.0/f;
				for (u = 0; u < t; u += dt)
					unless (signal (-t+u) is signal (u)) 
						err += 1;
				std::clog << 100*((n-err)/n) << "\% symmetric time access (iterative)" << std::endl;
				/* stabilizing the noise tends to stabilize the access */
				u = t-dt, err = 0;
				for (uint i = n; i --> 0; u = i*dt)
					unless (signal (-t+u) is signal (u)) 
						err += 1;
				std::clog << 100*((n-err)/n) << "\% symmetric time access (analytic)" << std::endl;

				/* attempting to read from beyond the range of the recording returns a NaN */
				assert (isnan (signal (t)));
				assert (not isnan (signal (t - dt)));
				/* though floating-point noise may extend the range of valid negative times by one sample */
				if (not isnan (signal (-t - dt)))
					std::clog << "access range has been extended" << std::endl;
				assert (isnan (signal (-t - 2*dt)));
				/* reading back by the Device::recording_length () is always valid */
				assert (not isnan (signal (-t)));
				/* and will return the oldest sample */
				assert (signal (-t) is signal [0]);

				std::clog << "sampling test passed" << std::endl << std::endl;
			}
		void rendering (void)
			{/*...}*/
				Device daq;
				daq.open_input_channels ({2, 3, 4, 5, 6, 7});
				daq.set_read_frequency (24);
				daq.set_sampling_rate (38400);
				daq.set_input_voltage (-5.0, 5.0);

				Display display;

				Link A (daq, 2);
				Link B (daq, 3);
				Link C (daq, 4);
				Scope s1 (of A, "ai2");
				Scope s2 (of B, "ai3");
				Scope s3 (of C, "ai4");
				s1.set_color (Color (0.7, 0.0, 0.0));
				s2.set_color (Color (0.0, 0.7, 0.0));
				s3.set_color (Color (0.0, 0.0, 0.9));

				daq.capture_callback = [&display, &s1, &s2, &s3] (uint)
					{/*...}*/
						display.draw (s1, 0, 0, 1, 0.33);
						display.draw (s2, 0, 0.33, 1, 0.34);
						display.draw (s3, 0, 0.67, 1, 0.33);
					};
				display.keypress_callback = [&daq] (char) {daq.stop ();};

				daq.record ();

				std::cout << daq.info_string () << std::endl;
			}
		void bell (void)
			{/*...}*/
				for (int x = 100; x --> 0;)
					std::this_thread::sleep_for (std::chrono::milliseconds (60)), std::cout << '\a' << std::endl;
			}
		void demo (void)
			{/*...}*/
				Device daq;
				daq.open_input_channels ({1, 3, 4});
				Link signal (daq, 3);

				double f = 2400; // Hz
				double t = 1.5;  // seconds
				daq.set_sampling_rate  (f);
				daq.set_recording_time (t);

				daq.record ();
				sleep (t);
				daq.stop ();

				uint 	n = f*t;     // number of samples
				double dt = 1.0/f;   // time duration of one sample
				double x;
				// access oldest sample
					// by index
						x = signal [0];
						x = signal [-n];
					// by time
						x = signal (0);
						x = signal (-t);
				// access latest sample
					// by index
						x = signal [n-1];
						x = signal [-1];
					// by time
						x = signal (t-dt);
						x = signal (-dt);
				IGNORE (x);
			}
	}
////////////////

//||||||||||
//|| MAIN ||
//||||||||||
int main (int argc, const char** argv) 
	{/*...}*/
		std::string filename;
		if (argc is 1)
			filename = std::string ("fcap_capture_") + date_and_time ();
		else filename = std::string (argv [1]);
		std::ofstream outfile (filename + ".dat");

		Display display;
		Device daq;
		daq.set_sampling_rate (1920);
		Link load_cell (daq, 7);
		#if 0
		Plate L (daq, 2, 3, 4);
		Plate R (daq, 5, 6, 7);
		Scope sig_lx (from L.x); sig_lx.set_label ("x axis left");
		Scope sig_ly (from L.y); sig_ly.set_label ("y axis left");
		Scope sig_lz (from L.z); sig_lz.set_label ("z axis left");
		Scope sig_rx (from R.x); sig_rx.set_label ("x axis right");
		Scope sig_ry (from R.y); sig_ry.set_label ("y axis right");
		Scope sig_rz (from R.z); sig_rz.set_label ("z axis right");
		#endif
		Gauge gauge (of load_cell);
		daq.open_input_channels ({7});
		
		auto render = [&] (void)
			{/*...}*/
				auto average = [&load_cell] (uint samples)
					{/*...}*/
						real sum = 0.0;
						for (uint i = samples; i --> 0;)
							sum += load_cell [-i-1];
						return sum / samples;
					};
				real voltage = average (500);
				real force = voltage * 50; // lbs, rough estimate from observation at 10V in for CSG110 amp (LB440 load cell)
				gauge.set_label (std::string ("load cell: ") + std::to_string (force) + " lbs"); 
				# if 0
				display.draw (sig_lx, 0.5, 0.0, 0.5, 0.33);
				display.draw (sig_ly, 0.5, 0.33, 0.5, 0.34);
				display.draw (sig_lz, 0.5, 0.67, 0.5, 0.33);
				///
				display.draw (sig_rx, 0.5, 0.5, 0.5, 0.33/2);
				display.draw (sig_ry, 0.5, 0.33/2+.5, 0.5, 0.34/2);
				display.draw (sig_rz, 0.5, 0.67/2+.5, 0.5, 0.33/2);
				display.draw (gauge, 0.05, 0.1, 0.4, 0.9);
				#endif
				display.draw (gauge, 0.05, 0.1, 0.8, 0.8);
				display.render ();
				#if 0
				if (0) {/*audio alert}*/
					static real highest_force_so_far = 0.0;
					if (force exceeds highest_force_so_far + 10)
						{/*ring a bell every 10 lbs}*/
							std::cout << "\a";
							highest_force_so_far = force;
						}
					if (force exceeds 200)
						{/*ring a bell continuously}*/
							std::cout << "\a";
							gauge.set_color (Color (1.0, 0.0, 0.0));
						}
					else gauge.set_color (Color (0.0, 0.7, 0.0));
					std::flush (std::cout);
				}
				#endif
			};
		auto dump = [&] (uint samples)
			{/*...}*/
				const uint frequency = 30;
				static uint counter = 0;
				static std::string buffer;
				init buffer.reserve (64*Mbyte);
				assert (buffer.capacity () exceeds 64*Mbyte);
				for (uint i = 0; i < samples; i++)
					buffer += std::to_string (load_cell [i - samples]) + "\n";
					#if 0
						/*+*/ "\t" + std::to_string (L.x [i - samples])
						+ "\t" + std::to_string (L.y [i - samples])
						+ "\t" + std::to_string (L.z [i - samples])
						+ "\n";
					#endif
				if (counter++ exceeds frequency)
					{/*dump and reset buffer}*/
						outfile.write (buffer.data (), buffer.length ());
						buffer.clear ();
						counter = 0;
					}
			};

		bool termination = false;
		display.keypress_callback = [&termination] (char) 
			{termination = true;};
		daq.capture_callback = [&render, &dump] (uint samples) 
			{render (); if (1) dump (samples);};

		daq.record ();
		until (termination)
			std::this_thread::yield ();
		daq.stop ();

		outfile.close ();
		#if 0
		std::ifstream infile (filename + ".dat");
		std::vector <real> F, X, Y, Z;
		while (infile.good ())
			{/*...}*/
				std::string line;
				std::getline (infile, line);
				real f, x, y, z;
				F.push_back (f);
				X.push_back (x);
				Y.push_back (y);
				Z.push_back (z);
				//sscanf (line.c_str (), "%lf\t%lf\t%lf\t%lf", to f, to x, to y, to z);
				sscanf (line.c_str (), "%lf\t%lf\t%lf", to x, to y, to z);
			}
		auto coeffs = linear_coefficients (Z, X, Y);
		std::cout << coeffs[0] << ", " << coeffs[1] << std::endl;//", " << coeffs[2] << std::endl;
		#endif

		return 0;
	}
int crosstalk_sampling (UNUSED int argc, UNUSED const char** argv) 
	{/*...}*/
		Device daq;
		Display dpy;

		daq.open_input_channels ({2,3,4});
		Plate plate (daq, 2, 3, 4);
		Scope signal (from plate.z);

		struct Sample
			{/*...}*/
				real left_contact_point; // [0-6.5]", on horiz.beam, relative to southernmost (closest to daq) hole (~3" from beam end)
				real right_contact_point;
				real x_coeff;
				real y_coeff;
				Sample ()
					{/*...}*/
						auto points = my random_points ();
						left_contact_point = points.first;
						right_contact_point = points.second;
					}
				auto random_points () -> std::pair <real,real>
					{/*...}*/
						static std::uniform_real_distribution <real> dist;
						static std::mt19937 rng;
						init {
							rng = std::mt19937 (std::chrono::system_clock::now ().time_since_epoch ().count ());
							dist = std::uniform_real_distribution <real> (0, 6.5); // 6.5 inch span
						}
						return std::pair <real,real> (dist(rng), dist(rng));
					};
			};
		std::vector <Sample> samples;

		std::vector <real> X, Y, Z;
		X.reserve (64*Power<2,20>::size);
		Y.reserve (64*Power<2,20>::size);
		Z.reserve (64*Power<2,20>::size);

		std::atomic<bool> quit (no);
		std::atomic<bool> pass (no);
		std::string prompt ("");

		daq.capture_callback = [&] (uint samples)
			{/*...}*/
				for (uint i = samples; i--> 1;)
					{/*...}*/
						X.push_back (plate.x [-i]);
						Y.push_back (plate.y [-i]);
						Z.push_back (plate.z [-i]);
					}
				dpy.draw (signal);
				dpy.write (prompt, uvec(0,20));
				dpy.render ();
			};
		dpy.keypress_callback = [&] (char key)
			{/*...}*/
				quit = key is 'q';
				pass = yes;
			};

		uint samples_taken = 0;
		until (quit or samples_taken exceeds 36)
			{/*...}*/
				pass = no;
				Sample sample;
				prompt = std::to_string(sample.left_contact_point) + ", " + std::to_string(sample.right_contact_point);
				until (pass) {dpy.write (prompt); dpy.render (); sleep (0.1);}
				pass = no;
				daq.record ();
				until (pass) sleep (0.1);
				daq.stop ();
				{/*compute coeffs}*/
					auto coeffs = linear_coefficients (Z, X, Y);
					X.clear ();
					Y.clear ();
					Z.clear ();
					sample.x_coeff = coeffs[0];
					sample.y_coeff = coeffs[1];
					samples.push_back (sample);
				}
				samples_taken++;
			}

		std::ofstream outfile ("bar_samples.dat");
		std::for_each (of_the (samples), [&outfile] (Sample sample) {
			auto line = std::string("@(") 
				+ std::to_string(sample.left_contact_point)
				+ ", "
				+ std::to_string(sample.right_contact_point)
				+ "): x,y = "
				+ std::to_string(sample.x_coeff) 
				+ ", " 
				+ std::to_string(sample.y_coeff)
			+ '\n';
			outfile.write (line.data (), line.size ());
		});

		return 0;
	}
////////////////

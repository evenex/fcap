module opencv;

private {/*imports}*/
	private {/*std}*/
		import std.conv;
		import std.stdio;
		import std.file;
		import std.algorithm;
		import std.array;
		import std.string;
	}
	private {/*evx}*/
		import evx.math;
		import evx.display;
		import evx.scribe;
		import evx.plot;
		import evx.colors;
		import evx.range;
		import evx.meta;
		import evx.utils;
	}

	alias map = evx.functional.map;
	alias zip = evx.functional.zip;
	alias filter = evx.functional.filter;
	alias reduce = evx.functional.reduce;
	alias floor = evx.analysis.floor;
	alias ceil = evx.analysis.ceil;
	alias round = evx.analysis.round;
	alias sum = evx.arithmetic.sum;
}

struct CvSize
	{/*...}*/
		int x, y;

		this (int x, int y)
			{/*...}*/
				this.x = x;
				this.y = y;
			}
	}
struct CvPoint
	{/*...}*/
		int x, y;

		this (int x, int y)
			{/*...}*/
				this.x = x;
				this.y = y;
			}
	}
struct CvRect
	{/*...}*/
		int x, y, width, height;

		this (int x, int y, int width, int height)
			{/*...}*/
				this.x = x;
				this.y = y;
				this.width = width;
				this.height = height;
			}
	}
struct CvScalar
	{/*...}*/
		double[4] val;

		this (V)(V v)
			in {/*...}*/
				assert (v.vector.length == 4);
			}
			body {/*...}*/
				v.vector[].copy (val[]);
			}
	}

alias IplImage = void;
alias CvArr = void;

struct CvCapture {}
struct CvVideoWriter {}

struct cv
	{/*...}*/
		__gshared:

		template opDispatch (string op, string data_file = __FILE__, uint line = __LINE__)
			{/*...}*/
				static opDispatch (Args...)(Args args)
					{/*...}*/
						auto c_args = to_c(args).expand;

						verify_function_call!(`cv` ~op)(c_args);

						mixin (q{
							return cv} ~op~ q{ (c_args);
						});
					}
			}

		pure static Codec (string code)
			{/*...}*/
				return (code[0] & 255) + ((code[1] & 255) << 8) + ((code[2] &255) << 16) + ((code[3] & 255) << 24);
			}

		extern (C) {/*highgui}*/
			// image read
			IplImage* function(const char* filename, int iscolor = CV_LOAD_IMAGE_COLOR) 
				cvLoadImage;

			// image write
			int function(const char* filename, const CvArr* image, const int* params = null)
				cvSaveImage;

			// input_video read
			CvCapture* function(const char* filename)
				cvCreateFileCapture;

			double function(CvCapture* capture, int property_id)
				cvGetCaptureProperty;

			int function(CvCapture* capture, int property_id, double value)
				cvSetCaptureProperty;

			IplImage* function(CvCapture* capture)
				cvQueryFrame;

			void function(CvCapture** capture)
				cvReleaseCapture;

			// input_video write
			CvVideoWriter* function(const char* filename, int fourcc, double fps, CvSize frame_size, int is_color = 1)
				cvCreateVideoWriter;

			int function(CvVideoWriter* writer, const IplImage* image)
				cvWriteFrame;

			void function(CvVideoWriter** writer)
				cvReleaseVideoWriter;

			public {/*enums}*/
				enum {/*capture properties}*/
					// modes of the controlling registers (can be: auto, manual, auto single push, absolute Latter allowed with any other mode)
					// every feature can have only one mode turned on at a time
					CV_CAP_PROP_DC1394_OFF         = -4,  //turn the feature off (not controlled manually nor automatically)
					CV_CAP_PROP_DC1394_MODE_MANUAL = -3, //set automatically when a value of the feature is set by the user
					CV_CAP_PROP_DC1394_MODE_AUTO = -2,
					CV_CAP_PROP_DC1394_MODE_ONE_PUSH_AUTO = -1,
					CV_CAP_PROP_POS_MSEC       =0,
					CV_CAP_PROP_POS_FRAMES     =1,
					CV_CAP_PROP_POS_AVI_RATIO  =2,
					CV_CAP_PROP_FRAME_WIDTH    =3,
					CV_CAP_PROP_FRAME_HEIGHT   =4,
					CV_CAP_PROP_FPS            =5,
					CV_CAP_PROP_FOURCC         =6,
					CV_CAP_PROP_FRAME_COUNT    =7,
					CV_CAP_PROP_FORMAT         =8,
					CV_CAP_PROP_MODE           =9,
					CV_CAP_PROP_BRIGHTNESS    =10,
					CV_CAP_PROP_CONTRAST      =11,
					CV_CAP_PROP_SATURATION    =12,
					CV_CAP_PROP_HUE           =13,
					CV_CAP_PROP_GAIN          =14,
					CV_CAP_PROP_EXPOSURE      =15,
					CV_CAP_PROP_CONVERT_RGB   =16,
					CV_CAP_PROP_WHITE_BALANCE_BLUE_U =17,
					CV_CAP_PROP_RECTIFICATION =18,
					CV_CAP_PROP_MONOCROME     =19,
					CV_CAP_PROP_SHARPNESS     =20,
					CV_CAP_PROP_AUTO_EXPOSURE =21, // exposure control done by camera,
												   // user can adjust refernce level
												   // using this feature
					CV_CAP_PROP_GAMMA         =22,
					CV_CAP_PROP_TEMPERATURE   =23,
					CV_CAP_PROP_TRIGGER       =24,
					CV_CAP_PROP_TRIGGER_DELAY =25,
					CV_CAP_PROP_WHITE_BALANCE_RED_V =26,
					CV_CAP_PROP_ZOOM          =27,
					CV_CAP_PROP_FOCUS         =28,
					CV_CAP_PROP_GUID          =29,
					CV_CAP_PROP_ISO_SPEED     =30,
					CV_CAP_PROP_MAX_DC1394    =31,
					CV_CAP_PROP_BACKLIGHT     =32,
					CV_CAP_PROP_PAN           =33,
					CV_CAP_PROP_TILT          =34,
					CV_CAP_PROP_ROLL          =35,
					CV_CAP_PROP_IRIS          =36,
					CV_CAP_PROP_SETTINGS      =37,
					CV_CAP_PROP_AUTOGRAB      =1024, // property for highgui class CvCapture_Android only
					CV_CAP_PROP_SUPPORTED_PREVIEW_SIZES_STRING=1025, // readonly, tricky property, returns cpnst char* indeed
					CV_CAP_PROP_PREVIEW_FORMAT=1026, // readonly, tricky property, returns cpnst char* indeed
					// OpenNI map generators
					CV_CAP_OPENNI_DEPTH_GENERATOR = 1 << 31,
					CV_CAP_OPENNI_IMAGE_GENERATOR = 1 << 30,
					CV_CAP_OPENNI_GENERATORS_MASK = CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_OPENNI_IMAGE_GENERATOR,
					// Properties of cameras available through OpenNI interfaces
					CV_CAP_PROP_OPENNI_OUTPUT_MODE     = 100,
					CV_CAP_PROP_OPENNI_FRAME_MAX_DEPTH = 101, // in mm
					CV_CAP_PROP_OPENNI_BASELINE        = 102, // in mm
					CV_CAP_PROP_OPENNI_FOCAL_LENGTH    = 103, // in pixels
					CV_CAP_PROP_OPENNI_REGISTRATION    = 104, // flag
					CV_CAP_PROP_OPENNI_REGISTRATION_ON = CV_CAP_PROP_OPENNI_REGISTRATION, // flag that synchronizes the remapping depth map to image map
																						  // by changing depth generator's view point (if the flag is "on") or
																						  // sets this view point to its normal one (if the flag is "off").
					CV_CAP_PROP_OPENNI_APPROX_FRAME_SYNC = 105,
					CV_CAP_PROP_OPENNI_MAX_BUFFER_SIZE   = 106,
					CV_CAP_PROP_OPENNI_CIRCLE_BUFFER     = 107,
					CV_CAP_PROP_OPENNI_MAX_TIME_DURATION = 108,
					CV_CAP_PROP_OPENNI_GENERATOR_PRESENT = 109,
					CV_CAP_OPENNI_IMAGE_GENERATOR_PRESENT         = CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_OPENNI_GENERATOR_PRESENT,
					CV_CAP_OPENNI_IMAGE_GENERATOR_OUTPUT_MODE     = CV_CAP_OPENNI_IMAGE_GENERATOR + CV_CAP_PROP_OPENNI_OUTPUT_MODE,
					CV_CAP_OPENNI_DEPTH_GENERATOR_BASELINE        = CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_PROP_OPENNI_BASELINE,
					CV_CAP_OPENNI_DEPTH_GENERATOR_FOCAL_LENGTH    = CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_PROP_OPENNI_FOCAL_LENGTH,
					CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION    = CV_CAP_OPENNI_DEPTH_GENERATOR + CV_CAP_PROP_OPENNI_REGISTRATION,
					CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION_ON = CV_CAP_OPENNI_DEPTH_GENERATOR_REGISTRATION,
					// Properties of cameras available through GStreamer interface
					CV_CAP_GSTREAMER_QUEUE_LENGTH   = 200, // default is 1
					CV_CAP_PROP_PVAPI_MULTICASTIP   = 300, // ip for anable multicast master mode. 0 for disable multicast
					// Properties of cameras available through XIMEA SDK interface
					CV_CAP_PROP_XI_DOWNSAMPLING  = 400,      // Change image resolution by binning or skipping.
					CV_CAP_PROP_XI_DATA_FORMAT   = 401,       // Output data format.
					CV_CAP_PROP_XI_OFFSET_X      = 402,      // Horizontal offset from the origin to the area of interest (in pixels).
					CV_CAP_PROP_XI_OFFSET_Y      = 403,      // Vertical offset from the origin to the area of interest (in pixels).
					CV_CAP_PROP_XI_TRG_SOURCE    = 404,      // Defines source of trigger.
					CV_CAP_PROP_XI_TRG_SOFTWARE  = 405,      // Generates an internal trigger. PRM_TRG_SOURCE must be set to TRG_SOFTWARE.
					CV_CAP_PROP_XI_GPI_SELECTOR  = 406,      // Selects general purpose input
					CV_CAP_PROP_XI_GPI_MODE      = 407,      // Set general purpose input mode
					CV_CAP_PROP_XI_GPI_LEVEL     = 408,      // Get general purpose level
					CV_CAP_PROP_XI_GPO_SELECTOR  = 409,      // Selects general purpose output
					CV_CAP_PROP_XI_GPO_MODE      = 410,      // Set general purpose output mode
					CV_CAP_PROP_XI_LED_SELECTOR  = 411,      // Selects camera signalling LED
					CV_CAP_PROP_XI_LED_MODE      = 412,      // Define camera signalling LED functionality
					CV_CAP_PROP_XI_MANUAL_WB     = 413,      // Calculates White Balance(must be called during acquisition)
					CV_CAP_PROP_XI_AUTO_WB       = 414,      // Automatic white balance
					CV_CAP_PROP_XI_AEAG          = 415,      // Automatic exposure/gain
					CV_CAP_PROP_XI_EXP_PRIORITY  = 416,      // Exposure priority (0.5 - exposure 50%, gain 50%).
					CV_CAP_PROP_XI_AE_MAX_LIMIT  = 417,      // Maximum limit of exposure in AEAG procedure
					CV_CAP_PROP_XI_AG_MAX_LIMIT  = 418,      // Maximum limit of gain in AEAG procedure
					CV_CAP_PROP_XI_AEAG_LEVEL    = 419,       // Average intensity of output signal AEAG should achieve(in %)
					CV_CAP_PROP_XI_TIMEOUT       = 420,       // Image capture timeout in milliseconds
					// Properties for Android cameras
					CV_CAP_PROP_ANDROID_FLASH_MODE = 8001,
					CV_CAP_PROP_ANDROID_FOCUS_MODE = 8002,
					CV_CAP_PROP_ANDROID_WHITE_BALANCE = 8003,
					CV_CAP_PROP_ANDROID_ANTIBANDING = 8004,
					CV_CAP_PROP_ANDROID_FOCAL_LENGTH = 8005,
					CV_CAP_PROP_ANDROID_FOCUS_DISTANCE_NEAR = 8006,
					CV_CAP_PROP_ANDROID_FOCUS_DISTANCE_OPTIMAL = 8007,
					CV_CAP_PROP_ANDROID_FOCUS_DISTANCE_FAR = 8008,
					// Properties of cameras available through AVFOUNDATION interface
					CV_CAP_PROP_IOS_DEVICE_FOCUS = 9001,
					CV_CAP_PROP_IOS_DEVICE_EXPOSURE = 9002,
					CV_CAP_PROP_IOS_DEVICE_FLASH = 9003,
					CV_CAP_PROP_IOS_DEVICE_WHITEBALANCE = 9004,
					CV_CAP_PROP_IOS_DEVICE_TORCH = 9005
				}
				enum {/*font style}*/
					CV_STYLE_NORMAL         = 0,//QFont::StyleNormal,
					CV_STYLE_ITALIC         = 1,//QFont::StyleItalic,
					CV_STYLE_OBLIQUE        = 2, //QFont::StyleOblique
					CV_FONT_LIGHT           = 25,//QFont::Light,
					CV_FONT_NORMAL          = 50,//QFont::Normal,
					CV_FONT_DEMIBOLD        = 63,//QFont::DemiBold,
					CV_FONT_BOLD            = 75,//QFont::Bold,
					CV_FONT_BLACK           = 87 //QFont::Black
				}
				enum {/*windows}*/
					//These 3 flags are used by cvSet/GetWindowProperty
					CV_WND_PROP_FULLSCREEN = 0,//to change/get window's fullscreen property
					CV_WND_PROP_AUTOSIZE   = 1,//to change/get window's autosize property
					CV_WND_PROP_ASPECTRATIO= 2,//to change/get window's aspectratio property
					//
					//These 2 flags are used by cvNamedWindow and cvSet/GetWindowProperty
					CV_WINDOW_NORMAL       = 0x00000000,//the user can resize the window (no constraint)  / also use to switch a fullscreen window to a normal size
					CV_WINDOW_AUTOSIZE     = 0x00000001,//the user cannot resize the window, the size is constrainted by the image displayed
					//
					//Those flags are only for Qt
					CV_GUI_EXPANDED         = 0x00000000,//status bar and tool bar
					CV_GUI_NORMAL           = 0x00000010,//old fashious way
					//
					//These 3 flags are used by cvNamedWindow and cvSet/GetWindowProperty
					CV_WINDOW_FULLSCREEN   = 1,//change the window to fullscreen
					CV_WINDOW_FREERATIO    = 0x00000100,//the image expends as much as it can (no ratio constraint)
					CV_WINDOW_KEEPRATIO    = 0x00000000//the ration image is respected.
				}
				enum {/*events}*/
					CV_EVENT_MOUSEMOVE      =0,
					CV_EVENT_LBUTTONDOWN    =1,
					CV_EVENT_RBUTTONDOWN    =2,
					CV_EVENT_MBUTTONDOWN    =3,
					CV_EVENT_LBUTTONUP      =4,
					CV_EVENT_RBUTTONUP      =5,
					CV_EVENT_MBUTTONUP      =6,
					CV_EVENT_LBUTTONDBLCLK  =7,
					CV_EVENT_RBUTTONDBLCLK  =8,
					CV_EVENT_MBUTTONDBLCLK  =9
				}
				enum {/*event flags}*/
					CV_EVENT_FLAG_LBUTTON   =1,
					CV_EVENT_FLAG_RBUTTON   =2,
					CV_EVENT_FLAG_MBUTTON   =4,
					CV_EVENT_FLAG_CTRLKEY   =8,
					CV_EVENT_FLAG_SHIFTKEY  =16,
					CV_EVENT_FLAG_ALTKEY    =32
				}
				enum {/*load image}*/
				/* 8bit, color or not */
					CV_LOAD_IMAGE_UNCHANGED  =-1,
				/* 8bit, gray */
					CV_LOAD_IMAGE_GRAYSCALE  =0,
				/* ?, color */
					CV_LOAD_IMAGE_COLOR      =1,
				/* any depth, ? */
					CV_LOAD_IMAGE_ANYDEPTH   =2,
				/* ?, any color */
					CV_LOAD_IMAGE_ANYCOLOR   =4
				}
				enum {/*imwrite compression}*/
					CV_IMWRITE_JPEG_QUALITY =1,
					CV_IMWRITE_PNG_COMPRESSION =16,
					CV_IMWRITE_PXM_BINARY =32
				}
				enum {/*cvtimg}*/
					CV_CVTIMG_FLIP      =1,
					CV_CVTIMG_SWAP_RB   =2
				}
				enum {/*cap}*/
					CV_CAP_ANY      =0,     // autodetect

					CV_CAP_MIL      =100,   // MIL proprietary drivers

					CV_CAP_VFW      =200,   // platform native
					CV_CAP_V4L      =200,
					CV_CAP_V4L2     =200,

					CV_CAP_FIREWARE =300,   // IEEE 1394 drivers
					CV_CAP_FIREWIRE =300,
					CV_CAP_IEEE1394 =300,
					CV_CAP_DC1394   =300,
					CV_CAP_CMU1394  =300,

					CV_CAP_STEREO   =400,   // TYZX proprietary drivers
					CV_CAP_TYZX     =400,
					CV_TYZX_LEFT    =400,
					CV_TYZX_RIGHT   =401,
					CV_TYZX_COLOR   =402,
					CV_TYZX_Z       =403,

					CV_CAP_QT       =500,   // QuickTime

					CV_CAP_UNICAP   =600,   // Unicap drivers

					CV_CAP_DSHOW    =700,   // DirectShow (via input_videoInput)

					CV_CAP_PVAPI    =800,   // PvAPI, Prosilica GigE SDK

					CV_CAP_OPENNI   =900,   // OpenNI (for Kinect)

					CV_CAP_ANDROID  =1000,  // Android
					
					CV_CAP_XIAPI    =1100   // XIMEA Camera API
				}
			}
		}
		extern (C) {/*core}*/
			IplImage* function(CvSize size, int depth, int channels) 
				cvCreateImage;

			void function(IplImage** image) 
				cvReleaseImage;

			IplImage* function(const IplImage* image)
				cvCloneImage;

			CvRect function(const IplImage* image)	
				cvGetImageROI;

			void function(IplImage* image, CvRect rect)	
				cvSetImageROI;

			void function(IplImage* image)
				cvResetImageROI;

			IplImage* function(IplImage* image, CvSize size, int depth, int channels, int origin = 0, int aligned = 4)
				cvInitImageHeader;

			CvScalar function(const CvArr* arr)
				cvSum;

			void function(const CvArr* src, CvArr* dst = null, int flip_mode = 0)
				cvFlip;

			CvSize function(const CvArr* arr) 
				cvGetSize;

			void function(CvArr* img, CvPoint pt1, CvPoint pt2, CvScalar color, int thickness = 1, int line_type = 8, int shift = 0)
				cvLine;

			void function(const CvArr* src, CvArr* dst, const CvArr* mask = null)
				cvCopy;

			void function(CvArr* arr, void* data, int step) 
				cvSetData;

			void function(const CvArr* src, CvScalar lower, CvScalar upper, CvArr* dst)
				cvInRangeS;

			int function(const CvArr* arr)
				cvGetElemType;

			public {/*enums}*/
				enum {/*IPL}*/
					IPL_DEPTH_SIGN  = 0x80000000,

					IPL_DEPTH_1U = 1,
					IPL_DEPTH_8U = 8,
					IPL_DEPTH_16U = 16,
					IPL_DEPTH_32F = 32,

					IPL_DEPTH_8S  = (IPL_DEPTH_SIGN| 8),
					IPL_DEPTH_16S = (IPL_DEPTH_SIGN|16),
					IPL_DEPTH_32S = (IPL_DEPTH_SIGN|32),

					IPL_DATA_ORDER_PIXEL = 0,
					IPL_DATA_ORDER_PLANE = 1,

					IPL_ORIGIN_TL = 0,
					IPL_ORIGIN_BL = 1,

					IPL_ALIGN_4BYTES = 4,
					IPL_ALIGN_8BYTES = 8,
					IPL_ALIGN_16BYTES = 16,
					IPL_ALIGN_32BYTES = 32,

					IPL_ALIGN_DWORD = IPL_ALIGN_4BYTES,
					IPL_ALIGN_QWORD = IPL_ALIGN_8BYTES,

					IPL_BORDER_CONSTANT = 0,
					IPL_BORDER_REPLICATE = 1,
					IPL_BORDER_REFLECT = 2,
					IPL_BORDER_WRAP = 3,
				}
			}
		}

		static mixin DynamicLibrary;
		shared static this () {load_library;}
	}

// theirs	↑
// mine 	↓
struct Video
	{/*...}*/
		struct Input
			{/*...}*/
				IplImage* frame;
				CvCapture* video;

				auto front ()
					in {/*...}*/
						assert (not(empty));
					}
					body {/*...}*/
						return Image (frame);
					}
				void popFront ()
					{/*...}*/
						frame = cv.QueryFrame (video);
					}
				bool empty () const
					{/*...}*/
						return frame is null;
					}

				@property length () const
					in {/*...}*/
						assert (this.is_valid);
					}
					body {/*...}*/
						return (
							cv.GetCaptureProperty (cast(CvCapture*)video, cv.CV_CAP_PROP_FRAME_COUNT)
							- cv.GetCaptureProperty (cast(CvCapture*)video, cv.CV_CAP_PROP_POS_FRAMES)
						).to!size_t;
					}
				@property size ()
					in {/*...}*/
						assert (this.is_valid);
						assert (not (empty));
					}
					body {/*...}*/
						return cv.GetSize (frame);
					}
				@property time ()
					in {/*...}*/
						assert (this.is_valid);
					}
					body {/*...}*/
						return cv.GetCaptureProperty (video, cv.CV_CAP_PROP_POS_MSEC).seconds / 1000;
					}
				@property framerate ()
					in {/*...}*/
						assert (this.is_valid);
					}
					body {/*...}*/
						return cv.GetCaptureProperty (video, cv.CV_CAP_PROP_FPS).hertz;
					}

				void seek (Seconds time)
					in {/*...}*/
						assert (this.is_valid);
					}
					body {/*...}*/
						static if (0)
							{/*...}*/
								cv.SetCaptureProperty (video, cv.CV_CAP_PROP_POS_MSEC, time.to!double * 1000); // OUTSIDE BUG this is fundamentally broken on the backend
							}

						cv.SetCaptureProperty (video, cv.CV_CAP_PROP_POS_MSEC, 0);

						foreach (i, frame; enumerate (this))
							if (i / framerate >= time)
								break;
					}

				bool is_valid () const
					{/*...}*/
						return video !is null;
					}

				this (string path)
					{/*...}*/
						video = cv.CreateFileCapture (path);
						popFront;
					}
				this (this) // REVIEW this is a cool way to preserve resources while adhering to RAII
					{/*...}*/
						is_copy = true;
					}
				~this ()
					{/*...}*/
						if (not (this.is_copy))
							cv.ReleaseCapture (&video);
					}
				bool is_copy;
			}
		struct Output
			{/*...}*/
				CvVideoWriter* video;
				const CvSize size;

				void put (IplImage* frame)
					in {/*...}*/
						assert (cv.GetSize (frame) == this.size,
							`input frame size (` ~size.vector.text~ `) does not match video size (` ~cv.GetSize (frame).vector.text~ `)`
						);
					}
					body {/*...}*/
						cv.WriteFrame (video, frame);
						
					}

				this (string path, CvSize size, Hertz framerate = 30)
					{/*...}*/
						this.size = size;

						video = cv.CreateVideoWriter (path, cv.Codec (`XVID`), framerate.to!double, this.size);
					}
				this (this)
					{/*...}*/
						is_copy = true;
					}
				~this ()
					{/*...}*/
						if (not (this.is_copy))
							cv.ReleaseVideoWriter (&video);
					}
				bool is_copy;
			}
	}

struct Image
	{/*...}*/
		IplImage* image;
		alias image this;

		this (IplImage* image)
			{/*...}*/
				this.image = image;
				this.is_copy = true;
			}
		this (int width, int height, int channels = 3)
			{/*...}*/
				this (CvSize (width, height), channels);
			}
		this (CvSize size, int channels = 3)
			{/*...}*/
				this.image = cv.CreateImage (size, cv.IPL_DEPTH_8U, channels); 
			}
		this (this)
			{/*...}*/
				is_copy = true;
			}
		~this ()
			{/*...}*/
				if (not (this.is_copy))
					cv.ReleaseImage (&image);
			}
		bool is_copy;

		@property roi (CvRect roi)
			{/*...}*/
				cv.SetImageROI (image, roi);
			}
		@property roi (typeof(null))
			{/*...}*/
				cv.ResetImageROI (image);
			}
		@property roi ()
			{/*...}*/
				return cv.GetImageROI (image);
			}

		@property size ()
			{/*...}*/
				return cv.GetSize (image);
			}
	}

void flip (Image image)
	{/*...}*/
		cv.Flip (image);
	}
auto accumulate (Image image)
	{/*...}*/
		return cv.Sum (image).val.vector / 255;
	}
auto color_range_filter (Image image, Color min, Color max)
	{/*...}*/
		auto to_image = Image (image.size, 1);

		cv.InRangeS (image, (255*min.vector.bgra).CvScalar, (255*max.vector.bgra).CvScalar, to_image);

		return to_image;
	}

static if (0)
void main ()
	{/*...}*/
		auto input_video = Video.Input (`./fcapdata/vid/vladb.mp4`);
		auto data_file = readText (`./fcapdata/dat/capture_20140901T154831_vlad.dat`).splitLines;

		auto timing_light_activation_time ()
			{/*...}*/
				static bool already_ran;

				assert (input_video.is_valid);
				assert (not (already_ran));

				already_ran = true;

				foreach (frame; input_video)
					if (frame.color_range_filter (black, blue).accumulate[].sum > 5) // REVIEW
						return input_video.time;

				assert (0, `couldn't find timing light in video`);
			}

		{/*process data}*/
			auto plot_size = CvSize (input_video.size.x, input_video.size.y/2);

			scope gfx = new Display (plot_size.x, plot_size.y * 4);
			gfx.start; scope (exit) gfx.stop;

			gfx.background (white);

			scope scr = new Scribe (gfx, [14]);

			{/*process data}*/
				auto freq = data_file.filter!(line => line.canFind (`sampling_frequency`))
					.front.findSplitAfter (`:`)[1].findSplitBefore (`Hz`)[0].strip
					.to!double.hertz;

				auto time = data_file.filter!(line => line.canFind (`{`))
					.map!(line => line.findSplitAfter (`{`)[1].findSplitBefore (`s}`)[0].strip)
					.map!(to!double)
					.map!seconds
					.array;

				alias Force = Vector!(3, Newtons);
				Force[] grf;
				{/*parse force data}*/
					auto f = data_file.filter!(line => line.canFind (`force`))
						.map!(line => line.findSplitAfter (`[`)[1].findSplitBefore (`]`)[0].strip);

					auto x = f.map!(line => line.findSplitBefore (`N`)[0].strip).map!(to!double).map!newtons;
					auto y = f.map!(line => line.findSplitAfter (`N,`)[1].findSplitBefore (`N`)[0].strip).map!(to!double).map!newtons;
					auto z = f.map!(line => line.findSplitAfter (`N,`)[1].findSplitAfter (`N,`)[1].findSplitBefore (`N`)[0].strip).map!(to!double).map!newtons;
					// TODO vector/unit parsing would be so much nicer... so i need an inverse for toString... how many "official" concepts have i added? cont.ranges, dynamic arrays?

					grf = zip (x,y,z).map!vector.array;
				}

				auto impact_data = data_file.filter!(line => line.canFind (`load_cell`))
					.map!(line => line.findSplitAfter (`V in)`)[1].findSplitBefore (`V`)[0].strip)
					.map!(to!double)
					.map!volts
					.array;

				auto mean = impact_data.mean;
				auto std_dev = impact_data.std_dev (mean);

				auto impact_times = zip (time, impact_data)
					.filter!(sample => sample[1] < mean - 5 * std_dev)
					.group!((a,b) => a[0].approx (b[0], 0.02))
					.map!(τ => τ[0][0]);

				auto time_of_data_start = timing_light_activation_time;

				foreach (time_of_impact; impact_times)
					{/*make a video}*/
						enum video_duration = 1.second;

						auto time_of_launch = time_of_impact - 0.5.seconds; // in the daq-time coordinate space
						auto ε = 1/input_video.framerate; // HACK the video's not quite aligned to the data, not sure how to fix it, could be a big job

						input_video.seek (time_of_data_start + time_of_launch + ε); // in the video-time coordinate space

						auto n_data_points_per_frame = (freq / input_video.framerate).to!size_t;

						auto n_samples = (video_duration * freq).to!size_t;
						auto data_start = (time_of_launch * freq).to!size_t;
						auto data_end = min (data_start + n_samples, grf.length);

						auto bounds = [gfx.dimensions/vec(1,4), 0.vec].from_pixel_space.to_extended_space (gfx);
						auto Δy = (gfx.dimensions * vec(1, 1/4.)).from_pixel_space.to_extended_space (gfx) * vec(0,1);

						auto plot_img = Image (gfx.dimensions[].map!(to!int).vector!2.tuple.expand);
						{/*generate plot image}*/
							plot (grf[data_start..data_end].map!(f => f.z)
								.versus (ℕ[data_start..data_end].map!(i => i / freq))
								.stride (n_data_points_per_frame))
								.color (black)
								.x_axis (`time`)
								.y_axis (`z force`, interval (0.newtons, 1200.newtons))
								.using (gfx, scr)
								.inside (bounds.bounding_box)
							.draw;

							plot (grf[data_start..data_end].map!(f => f.x)
								.versus (ℕ[data_start..data_end].map!(i => i / freq))
								.stride (n_data_points_per_frame))
								.color (black)
								.x_axis (`time`)
								.y_axis (`x force`, interval (-600.newtons, 600.newtons))
								.using (gfx, scr)
								.inside (bounds.translate (-Δy).bounding_box)
							.draw;

							plot (grf[data_start..data_end].map!(f => f.y)
								.versus (ℕ[data_start..data_end].map!(i => i / freq))
								.stride (n_data_points_per_frame))
								.color (black)
								.x_axis (`time`)
								.y_axis (`y force`, interval (-600.newtons, 600.newtons))
								.using (gfx, scr)
								.inside (bounds.translate (-2*Δy).bounding_box)
							.draw;

							plot (impact_data[data_start..data_end]
								.versus (ℕ[data_start..data_end].map!(i => i / freq))
								.stride (n_data_points_per_frame))
								.color (black)
								.x_axis (`time`)
								.y_axis (`load cell`)
								.using (gfx, scr)
								.inside (bounds.translate (-3*Δy).bounding_box)
							.draw;

							gfx.render;

							scope img_data = new void[gfx.dimensions[].product.to!size_t * 3];
							gfx.access_rendering_context ((){
								gl.ReadPixels (0, 0, gfx.dimensions.x.to!int, gfx.dimensions.y.to!int, PixelFormat.bgr, PixelFormat.unsigned_byte, img_data.ptr);
							});

							cv.SetData (plot_img, img_data.ptr, gfx.dimensions.x.to!int * 3);
							plot_img.flip; // BUG this does nothing in optimized mode
						}

						auto output_video = Video.Output (`output_video/out` ~(time_of_impact.to!double * 1000).to!size_t.to!string~ `.avi`, CvSize (input_video.size.x, (input_video.size.y + plot_size.y*4)));

						auto img = Image (output_video.size);
						foreach (i, frame; enumerate (input_video))
							{/*...}*/
								img.roi = CvRect (0, 0, input_video.size.vector.tuple.expand);
								cv.Copy (frame, img);
								img.roi = CvRect (0, input_video.size.y, gfx.dimensions[].map!(to!int).vector!2.tuple.expand);
								cv.Copy (plot_img, img);
								img.roi = null;

								auto plotbox = plot (impact_data[].versus (enumerate(impact_data[]).map!(x => x[1]))).inside (bounds).using (gfx, scr).plot_field; // TODO
								int x = ((i/(n_samples/freq * input_video.framerate) * plotbox.width + (plotbox.left + 1))/2 * gfx.dimensions.x).to!int;
								cv.Line (img, CvPoint (x, input_video.size.y), CvPoint (x, output_video.size.y), CvScalar ((255*red.vector.bgra).array));

								output_video.put (img);

								if (x > output_video.size.x)
									break;
								if (i >= video_duration * input_video.framerate)
									break;
							}
					}
			}
		}
	}

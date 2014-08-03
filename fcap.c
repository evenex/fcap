#define not	!
#define is	==
#define and	&&
#define or	||

#include "stdlib.h"
#include "stdint.h"
#include "stdio.h"
#include "math.h"
#include "time.h"
#include "string.h"
#include "/usr/local/natinst/nidaqmxbase/include/NIDAQmxBase.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <omp.h>

// FIXME
#define MAX_NEWTONS 1200.0

/* ERROR HANDLING */
static int FCAP_ERR = 0;
#define DAQ_ERROR_CHECK(__label__) { 							\
	if (DAQmxFailed(err)) { 							\
		char errStr[256]; 							\
		DAQmxBaseGetExtendedErrorInfo(errStr, 256); 				\
		printf("DAQ Error at %i: %s (%s)\n", __LINE__, #__label__, errStr); 	\
		FCAP_ERR = 1;								\
		}									\
}
#define FCAP_ASSERT(__statement__) { 							\
	if (!(__statement__)) FCAP_ERR = 1;						\
	if (FCAP_ERR) printf("Error in %s at %i: Assertion Failed (%s)\n", __FILE__, __LINE__, #__statement__);\
}
#define FATAL {										\
	if (FCAP_ERR) exit(EXIT_FAILURE); 						\
}
#define NONFATAL {									\
	FCAP_ERR = 0; 									\
}
void bchk(uint32_t N) {
	char j;
	for (int i = 0; i < 32; i++) {
		j = N & 0x80000000? '1': '0';
		N <<= 1;
		printf("%c", j);
	}
	printf("\n");
	return;
}

/* VECTOR OPS */
typedef struct vec3_struct {
	double x, y, z;
} vec3;
vec3 newvec (double x, double y, double z) {
	vec3 A;
	A.x = x;
	A.y = y;
	A.z = z;
	return A;
}
vec3 addvec (vec3 A, vec3 B) {
	vec3 C;
	C.x = A.x + B.x;
	C.y = A.y + B.y;
	C.z = A.z + B.z;
	return C;
}
vec3 mulvec (vec3 A, double S) {
	vec3 B;
	B.x = A.x*S;
	B.y = A.y*S;
	B.z = A.z*S;
	return B;
}
vec3 zerovec () {
	vec3 A = {0, 0, 0};
	return A;
}
double dotvec (vec3 A, vec3 B) {
	return A.x*B.x + A.y*B.y + A.z*B.z;
}
double magvec (vec3 A) {
	return sqrt (A.x * A.x + A.y * A.y + A.z * A.z);
}
int equalvec (vec3 A, vec3 B) {
	double eps = 1e-2 * (magvec (A) + magvec (B));
	if (magvec (addvec (A, mulvec (B, -1))) < eps)
		return 1;
	else 	return 0;
}
void printvec (vec3 A, FILE* output) {
	fprintf (output, "< %lf, %lf, %lf >\n", A.x, A.y, A.z);
}

/* SETTINGS */
static struct DAQ {
	struct DAQ_node {
		TaskHandle 	terminal;
		char*		channels;
		double		max_voltage,	
				min_voltage;
	};
	struct DAQ_input {
		struct DAQ_node main;
	} input;
	struct DAQ_output {
		struct DAQ_node red;
		struct DAQ_node blue;
	} output;
	struct DAQ_parameters {
		int	serial;
		int	sampling_rate;
	} parameters;
} DAQ;
static struct voltage_buffer {
	double	*raw_data;
	double	*scaled_data;
	vec3	calibration [6];
	vec3	baseline [2];
	int	write_index;
	enum 	mode {
		RAW,
		SCALED
	} mode;
	struct 	voltage_buffer_parameters {
		int	n_channels;
		int	read_frequency;
		int	block_count;
		int	block_length;
		int	signal_length;
		int	buffer_size;
		double	recording_length_inSeconds;
		double	max_amplitude;
	} parameters;
} voltage_buffer;
/* MAIN CONTROL */
void initialize (double requested_recording_length_inSeconds) {
	void init_DAQ () {
		/* IDENTIFY DEVICES */ {
			int32	err = 0;

			uInt32	serial_reported = 21692874;

			err = DAQmxBaseGetDevSerialNum("Dev1", &serial_reported);
				DAQ_ERROR_CHECK(detecting DAQ) FATAL;

			DAQ.parameters.serial = serial_reported;
		}
		/* SET CHANNELS */ {
			DAQ.input.main.channels = "Dev1/ai1, Dev1/ai2, Dev1/ai3, Dev1/ai4, Dev1/ai5, Dev1/ai6, Dev1/ai7";

			DAQ.output.red.channels = "Dev1/ao0";
			DAQ.output.blue.channels = "Dev1/ao1";
		}
		/* SET PARAMETERS */ {
			DAQ.parameters.sampling_rate = 8192;

			DAQ.input.main.min_voltage = -5.0;
			DAQ.input.main.max_voltage =  5.0;

			DAQ.output.red.min_voltage =  0.0;
			DAQ.output.red.max_voltage =  3.2;
			
			DAQ.output.blue.min_voltage =  0.0;
			DAQ.output.blue.max_voltage =  2.0;
		}
	}
	void init_voltage_buffer () {
		struct voltage_buffer_parameters* 
			P = &(voltage_buffer.parameters);

		/* SET PARAMETERS */ {
			(*P).n_channels; {
				int N_chn = 1;
				int i = 0;
				char c;

				do /* count commas in DAQ left channels string */ {
					c = DAQ.input.main.channels[i++];
					if (c == ',') N_chn++;
				} while (c != '\0');

				(*P).n_channels = N_chn;
			}
			(*P).read_frequency; {
				(*P).read_frequency = 60;
			}
			(*P).block_count; {
				double L_rec = requested_recording_length_inSeconds;
				int F_upd = (*P).read_frequency;
				
				int N_blk = (int) ceil( L_rec * F_upd );

				(*P).block_count = N_blk;
			}
			(*P).block_length; {
				int R_samp = DAQ.parameters.sampling_rate;
				int F_upd = voltage_buffer.parameters.read_frequency;

				int L_blk = (int) ceil(R_samp / F_upd);

				(*P).block_length = L_blk;
			}
			(*P).signal_length; {
				int L_blk = (*P).block_length;
				int N_blk = (*P).block_count;

				int L_sig = L_blk * N_blk;

				(*P).signal_length = L_sig;
			}
			(*P).buffer_size; {
				int L_sig = (*P).signal_length;
				int N_chn = (*P).n_channels;
				
				int L_buf = N_chn * L_sig;
		
				(*P).buffer_size = L_buf;
			}
			(*P).recording_length_inSeconds; {
				double L_sig = (double)((*P).signal_length);
				int R_samp = DAQ.parameters.sampling_rate;
				
				double L_rec = L_sig / R_samp;

				(*P).recording_length_inSeconds = L_rec;
			}
			(*P).max_amplitude; {
				double L = DAQ.input.main.max_voltage; 

				(*P).max_amplitude = L;
			}
		}
		/* ALLOCATE MEMORY */ {
			int L_buf = (*P).buffer_size;
			double* temp_ptr = (double*) malloc (sizeof(double) * L_buf);
			voltage_buffer.raw_data = temp_ptr;
			temp_ptr = (double*) malloc (sizeof(double) * L_buf);
			voltage_buffer.scaled_data = temp_ptr;
		}
		/* INITIALIZE STATE */ {
			voltage_buffer.write_index = 0;
		}
	}

	init_DAQ ();
	init_voltage_buffer ();

	return;
}
void terminate () {
	void free_voltage_buffer () {
		free(voltage_buffer.raw_data);
		free(voltage_buffer.scaled_data);
	}

	free_voltage_buffer ();
	return;
}
/* NODE CONTROL */
void openNodes () {
	/* INPUT */ {
		void openInput(struct DAQ_node* node) {
			int32 err;

			err = DAQmxBaseCreateTask("", &(*node).terminal);
				DAQ_ERROR_CHECK(initiate input) FATAL;
			err = DAQmxBaseCreateAIVoltageChan(
				(*node).terminal, (*node).channels, "",
				DAQmx_Val_Cfg_Default,
				(*node).min_voltage,
				(*node).max_voltage,
				DAQmx_Val_Volts, NULL
			);
				DAQ_ERROR_CHECK(create ai channels) FATAL;
			err = DAQmxBaseCfgSampClkTiming(
				(*node).terminal, "OnboardClock",
				DAQ.parameters.sampling_rate,
				DAQmx_Val_Rising, DAQmx_Val_ContSamps, 0
			);
				DAQ_ERROR_CHECK(configure sampling clock) FATAL;
			err = DAQmxBaseCfgInputBuffer((*node).terminal, voltage_buffer.parameters.buffer_size * 2);
				DAQ_ERROR_CHECK(configure input buffer) FATAL;
			err = DAQmxBaseStartTask((*node).terminal);
				DAQ_ERROR_CHECK(start capture from plate) FATAL;
			return;
		}
		openInput(&DAQ.input.main);
	}
	/* OUTPUT */ {
		void openOutput(struct DAQ_node* node) {
			int32 err;

			err = DAQmxBaseCreateTask("", &(*node).terminal);
				DAQ_ERROR_CHECK(initiate output) FATAL;
			err = DAQmxBaseCreateAOVoltageChan(
				(*node).terminal, (*node).channels, "",
				(*node).min_voltage,
				(*node).max_voltage,
				DAQmx_Val_Volts, NULL
			);
				DAQ_ERROR_CHECK(create ao channels) FATAL;
			err = DAQmxBaseStartTask((*node).terminal);
				DAQ_ERROR_CHECK(open output) FATAL;
			return;
		}
		openOutput(&DAQ.output.red);
		openOutput(&DAQ.output.blue);
	}
} 
void closeNodes () {
	void closeNode(struct DAQ_node node) {
		int32 err;
		err = DAQmxBaseStopTask(node.terminal);
			DAQ_ERROR_CHECK(stop capture from node) NONFATAL;
		err = DAQmxBaseClearTask(node.terminal);
			DAQ_ERROR_CHECK(clear node) NONFATAL;
		return;
	}
	/* INPUT */ {
		closeNode(DAQ.input.main);
	}
	/* OUTPUT */ {
		closeNode(DAQ.output.red);
		closeNode(DAQ.output.blue);
	}
}
/* MAIN ROUTINES */
void captureSignal () {
	struct voltage_buffer* 
		V = &voltage_buffer;
	struct voltage_buffer_parameters*
		P = &(*V).parameters;

	/* SEND SYNCHRONIZATION SIGNAL * {
		void light_switch(int on, struct DAQ_node* node) {
			int32 err;
			int32 pointsWritten;
			double voltage; {
				if (on) voltage = (*node).max_voltage;
				else	voltage = (*node).min_voltage;
			}

			err = DAQmxBaseWriteAnalogF64(
				(*node).terminal,
				1, 0, 1.0,
				DAQmx_Val_GroupByChannel, 
				&voltage, 
				&pointsWritten, NULL
			);
				DAQ_ERROR_CHECK(send sync signal) NONFATAL;

			return;
		}
		/* SWITCH LIGHTS VIA 4-BIT GRAY CODE * {
			int s = ((*V).sync_index++, (*V).sync_index &= 0xF);
			if 	( s % 0x4 == 0x1 )
				light_switch(1, &DAQ.output.red);
			else if ( s % 0x4 == 0x3 )
				light_switch(0, &DAQ.output.red);
			else if ( s % 0x8 == 0x2 )
				light_switch(1, &DAQ.output.yellow);
			else if ( s % 0x8 == 0x6 )
				light_switch(0, &DAQ.output.yellow);
			else if ( s % 0xF == 0x4 )
				light_switch(1, &DAQ.output.green);
			else if ( s % 0xF == 0xC )
				light_switch(0, &DAQ.output.green);
			else if ( s % 0xF == 0x8 )
				light_switch(1, &DAQ.output.blue);
			else if ( s % 0xF == 0x0 )
				light_switch(0, &DAQ.output.blue);
		}
	} /**/
	/* READ FROM DAQ AND WRITE TO BUFFER */ {
		void readSignal(struct DAQ_node* node, double* writeBuffer) {
			int32 err;
			int32 samples_read;

			err = DAQmxBaseReadAnalogF64(
				(*node).terminal,
				(*P).block_length,
				1.0, DAQmx_Val_GroupByScanNumber,
				&*(writeBuffer + (*V).write_index),
				(*P).signal_length,
				&samples_read, NULL
			); 
			DAQ_ERROR_CHECK(read from daq) FATAL;
			FCAP_ASSERT(samples_read == (*P).block_length) NONFATAL; 
			
			return;
		}
		readSignal(&DAQ.input.main, (*V).raw_data);
	}
	/* SCALE DATA */ if ((*V).mode is SCALED) {
		int L_b = (*P).block_length;
		int N_c = (*P).n_channels;
		int idx = (*V).write_index;
		vec3* M_cal = (*V).calibration;

		for (int i = 0; i < L_b*N_c; i += 6) {
			vec3 V_L, V_R; {
				V_L.x = (*V).raw_data [idx + i + 0];
				V_L.y = (*V).raw_data [idx + i + 1];
				V_L.z = (*V).raw_data [idx + i + 2];
				V_R.x = (*V).raw_data [idx + i + 3];
				V_R.y = (*V).raw_data [idx + i + 4];
				V_R.z = (*V).raw_data [idx + i + 5];

				V_L = addvec (V_L, (*V).baseline [0]);
				V_R = addvec (V_R, (*V).baseline [1]);
			}
			(*V).scaled_data [idx + i + 0] = dotvec (M_cal [0], V_L);
			(*V).scaled_data [idx + i + 1] = dotvec (M_cal [1], V_L);
			(*V).scaled_data [idx + i + 2] = dotvec (M_cal [2], V_L);
			(*V).scaled_data [idx + i + 3] = dotvec (M_cal [3], V_R);
			(*V).scaled_data [idx + i + 4] = dotvec (M_cal [4], V_R);
			(*V).scaled_data [idx + i + 5] = dotvec (M_cal [5], V_R);
		}
	}
	/* ADVANCE WRITE INDEX */ {
		int L_b = (*P).block_length;
		int N_c = (*P).n_channels;

		(*V).write_index += L_b * N_c;
	}
	/* ROTATE BUFFER */ {
		int S_b = (*P).buffer_size;

		if ( (*V).write_index == S_b )
			(*V).write_index = 0;
		FCAP_ASSERT ( (*V).write_index < S_b );
	}
	return;
}
int renderDisplay () {
	struct voltage_buffer* 
		V = &voltage_buffer;
	struct voltage_buffer_parameters*
		P = &(*V).parameters;

	static IplImage* display = NULL; {
		/* INITIALIZE */ if (display == NULL) {
			CvSize 	size 	= cvSize(1200,600);
			int	depth 	= IPL_DEPTH_8U;
			int 	channels= 3;

			display = cvCreateImage(size, depth, channels);

			cvNamedWindow("fcap", CV_WINDOW_AUTOSIZE);
			cvMoveWindow("fcap", 0,0);
		}
	}
	static struct viewport {
		struct RGB {
			int R,G,B;
		};
		struct label {
			struct RGB color;
			char* text;
		} label;
		struct signal {
			struct RGB color;
			char plate, axis;
		} signal;
	}
		LX,LY,LZ, RX,RY,RZ; {
			LX.label.text = "L X";
			LX.label.color.R = 0x00;
			LX.label.color.G = 0xCC;
			LX.label.color.B = 0x00;
			LX.signal.plate = 'L';
			LX.signal.axis = 'X';
			LX.signal.color.R = 0x00;
			LX.signal.color.G = 0xCC;
			LX.signal.color.B = 0x00;

			LY.label.text = "L Y";
			LY.label.color.R = 0x00;
			LY.label.color.G = 0xCC;
			LY.label.color.B = 0x00;
			LY.signal.plate = 'L';
			LY.signal.axis = 'Y';
			LY.signal.color.R = 0x00;
			LY.signal.color.G = 0xCC;
			LY.signal.color.B = 0x00;

			LZ.label.text = "L Z";
			LZ.label.color.R = 0x00;
			LZ.label.color.G = 0xCC;
			LZ.label.color.B = 0x00;
			LZ.signal.plate = 'L';
			LZ.signal.axis = 'Z';
			LZ.signal.color.R = 0x00;
			LZ.signal.color.G = 0xCC;
			LZ.signal.color.B = 0x00;

			RX.label.text = "R X";
			RX.label.color.R = 0x44;
			RX.label.color.G = 0x88;
			RX.label.color.B = 0xCC;
			RX.signal.plate = 'R';
			RX.signal.axis = 'X';
			RX.signal.color.R = 0x44;
			RX.signal.color.G = 0x88;
			RX.signal.color.B = 0xCC;

			RY.label.text = "R Y";
			RY.label.color.R = 0x44;
			RY.label.color.G = 0x88;
			RY.label.color.B = 0xCC;
			RY.signal.plate = 'R';
			RY.signal.axis = 'Y';
			RY.signal.color.R = 0x44;
			RY.signal.color.G = 0x88;
			RY.signal.color.B = 0xCC;

			RZ.label.text = "R Z";
			RZ.label.color.R = 0x44;
			RZ.label.color.G = 0x88;
			RZ.label.color.B = 0xCC;
			RZ.signal.plate = 'R';
			RZ.signal.axis = 'Z';
			RZ.signal.color.R = 0x44;
			RZ.signal.color.G = 0x88;
			RZ.signal.color.B = 0xCC;
		}

	/* CLEAR DISPLAY */ {
		cvZero(display);
	}
	/* DRAW VIEWPORTS */ {
		int rows, cols; {
			rows = 3;
			cols = 2;
		}
		int n_viewports; {
			n_viewports = rows*cols;
		}
		struct viewport* viewports[n_viewports]; {
			viewports[0] = &LX;
			viewports[1] = &RX;
			viewports[2] = &LY;
			viewports[3] = &RY;
			viewports[4] = &LZ;
			viewports[5] = &RZ;
		}

		for (int i = 0; i < n_viewports; i++) {
			int  x0,y0,  dx,dy; {
				dx = ((*display).width/cols);
				dy = (*display).height/rows;

				x0 = dx * (i%cols);
				y0 = dy * (i/cols);
			}
			/* DRAW LABELS */ {
				static CvFont font; {
					static int init = 0;
					/* INIT */ if (not init) {
						cvInitFont (
							&font, CV_FONT_HERSHEY_SIMPLEX,
							0.5, 0.5, 0.0, 1, 8
						);
						init = 1;
					}
				}
				int R,G,B; {
					R = viewports[i] -> label.color.R;
					G = viewports[i] -> label.color.G;
					B = viewports[i] -> label.color.B;
				}
				int fontsize; {
					CvSize fontsize2D;
					int null;
					cvGetTextSize("I", &font, &fontsize2D, &null);

					fontsize = fontsize2D.height;
				}
				cvPutText(
					display, 
					viewports[i] -> label.text,
					cvPoint(x0, y0+fontsize+1),
					&font, 
					cvScalar(B,G,R,0)
				);
			}
			/* DRAW SIGNALS */ {
				double*	source; {
					source = (*V).mode is RAW? (*V).raw_data: (*V).scaled_data;
				}
				double max; {
					max = (*V).mode is RAW? (*P).max_amplitude: MAX_NEWTONS;
				}
				int 	buffer_size; {
					buffer_size = (*P).buffer_size;
				}
				int 	signal_length; {
					signal_length = (*P).signal_length;
				}
				int	access_start; {
					access_start = (*V).write_index;
				}
				int 	access_offset; {
					switch (viewports[i] -> signal.axis) {
						case 'X': {
							access_offset = 0;
							break;
						}
						case 'Y': {
							access_offset = 1;
							break;
						}
						case 'Z': {
							access_offset = 2;
							break;
						}
						default: {
							access_offset = -1;
							break;
						}
					}
					switch (viewports[i] -> signal.plate) {
						case 'L': {
							access_offset += 0;
							break;
						}
						case 'R': {
							access_offset += 3;
							break;
						}
						default: {
							access_offset = -1;
							break;
						}
					}
					FCAP_ASSERT(access_offset >= 0);
				}
				int 	access_stride; {
					access_stride = (*P).n_channels;
				}
				int 	draw_stride; {
					draw_stride = signal_length/dx;
					if (draw_stride > 1) draw_stride = 1;
				}
				CvScalar color; {
					int R,G,B; {
						R = viewports[i] -> signal.color.R;
						G = viewports[i] -> signal.color.G;
						B = viewports[i] -> signal.color.B;
					}
					color = cvScalar(B,G,R,0);
				}
	
				/* DRAW SAMPLES */ for (int k = 1; k <= signal_length; k += draw_stride) {
					CvPoint getPoint(int index) {
						int j = index;
						int access = (	access_start 
								+ access_offset 
								+ access_stride*(j)
								) % buffer_size;

						int x = x0 + (double)(j)/signal_length * dx;
						int y = y0 + dy/2
							+ source[access] / (V -> mode is RAW? P -> max_amplitude : MAX_NEWTONS)
							* dy/2;

						return cvPoint(x,y);
					}
					cvLine(
						display, 
						getPoint(k-1), getPoint(k),
						color, 2, 8, 0
					);
				}
			}
			/* DRAW BORDERS */ {
				CvScalar color = cvScalar(100,100,100,0);
				void drawLine (int x0, int y0, int x1, int y1) {
					cvLine(
						display, 
						cvPoint(x0,y0), cvPoint(x1,y1),
						color, 1, 8, 0
					);
					return;
				}
				drawLine(x0,	y0, 	x0+dx,	y0);
				drawLine(x0+dx, y0,	x0+dx,	y0+dy);
				drawLine(x0+dx, y0+dy,	x0,	y0+dy);
				drawLine(x0, 	y0+dy,	x0,	y0);
			}
		}
	}
	/* OUTPUT TO SCREEN */ {
		cvShowImage("fcap", display);
	}
	/* RETURN KEYPRESS */ {
		int finished;
		int keypressed = cvWaitKey(2) + 1;
		if (keypressed) finished = 1;
		else finished = 0;
		return finished;
	}
}
/* RECORDING */
void record_once () {
	struct voltage_buffer_parameters* P = &(voltage_buffer.parameters);

	openNodes ();
	voltage_buffer.write_index = 0;
	do 	captureSignal ();
	while 	(voltage_buffer.write_index >= (*P).block_length);
	closeNodes ();
}
vec3 record_mean (char plate) {
	struct voltage_buffer_parameters* P = &(voltage_buffer.parameters);

	record_once ();

	vec3 mean = zerovec ();
	int N_c = (*P).n_channels;
	int offset = plate == 'L'? 0 : N_c/2;
	for (int i = 0; i < (*P).buffer_size; i += N_c) {
		vec3 A; 
		A.x = voltage_buffer.raw_data [i + offset + 0];
		A.y = voltage_buffer.raw_data [i + offset + 1];
		A.z = voltage_buffer.raw_data [i + offset + 2];

		mean = addvec (mean, A);
	}
	double divisor = 1.0 / (*P).signal_length;
	return mulvec (mean, divisor);

}
void record_baseline () {
	voltage_buffer.baseline [0] = mulvec (record_mean ('L'), -1);
	voltage_buffer.baseline [1] = mulvec (record_mean ('R'), -1);
	return;
}
/* OUTPUT */
void light_switch(int on, struct DAQ_node* node) {
	int32 err;
	int32 pointsWritten;
	double voltage = on? (*node).max_voltage : (*node).min_voltage;

	err = DAQmxBaseWriteAnalogF64(
		(*node).terminal,
		1, 0, 1.0,
		DAQmx_Val_GroupByChannel, 
		&voltage, 
		&pointsWritten, NULL
	);
	DAQ_ERROR_CHECK(send sync signal) NONFATAL;

	return;
}
/* DATA ROUTINES */
void loadCalibration (char* path) {
	struct sample_struct {
		double F;
		double X;
		double Y;
		double Z;
	} LX [6], LY [6], LZ [6], RX [6], RY [6], RZ [6], *LUT [6]; {
		LUT [0] = LX;
		LUT [1] = LY;
		LUT [2] = LZ;
		LUT [3] = RX;
		LUT [4] = RY;
		LUT [5] = RZ;
	}
	/* READ DATA */ {
		FILE* infile = fopen (path, "r");
		FCAP_ASSERT (infile);

		char line [2048];
		struct sample_struct* current_sample = NULL;
		int sample_ptr = 0;
		long read_line () {
			long ret = (long) fgets (line, 2048, infile);
			if (line [0] != 'm') {
				char plate, axis;
				plate = line [0];
				axis  = line [1];
				int sampler = (plate == 'L'? 0 : 3) + (axis == 'X'? 0 : axis == 'Y'? 1 : 2);
				current_sample = LUT [sampler];
				ret = (long) fgets (line, 2048, infile);
				sample_ptr = 0;
			}
			return ret;
		}

		while (read_line ()) {
			#define KG_TO_N 9.80665002864
			double f, x, y, z;
			sscanf (line, " mass: %lf, response: < %lf, %lf, %lf > ", &f, &x, &y, &z);
			current_sample [sample_ptr] .F = f * KG_TO_N;
			current_sample [sample_ptr] .X = x;
			current_sample [sample_ptr] .Y = y;
			current_sample [sample_ptr] .Z = z;
			sample_ptr ++;
		}

		fclose (infile);
	}
	/* COMPUTE SCALING COEFFICIENTS */ {
		#define get_coeff(plate, force_axis, voltage_axis) ({						\
			double num = 0.0, den = 0.0;								\
			for (int i = 0; i < 6; i++) {								\
				num += plate ## force_axis [i] .F * plate ## force_axis [i] .voltage_axis;	\
				den += plate ## force_axis [i] .F * plate ## force_axis [i] .F;			\
			}											\
			double res = num/den;									\
			res;											\
		})

		voltage_buffer.calibration [0].x = get_coeff (L, X, X);
		voltage_buffer.calibration [0].y = get_coeff (L, Y, X);
		voltage_buffer.calibration [0].z = get_coeff (L, Z, X);
		voltage_buffer.calibration [1].x = get_coeff (L, X, Y);
		voltage_buffer.calibration [1].y = get_coeff (L, Y, Y);
		voltage_buffer.calibration [1].z = get_coeff (L, Z, Y);
		voltage_buffer.calibration [2].x = get_coeff (L, X, Z);
		voltage_buffer.calibration [2].y = get_coeff (L, Y, Z);
		voltage_buffer.calibration [2].z = get_coeff (L, Z, Z);

		voltage_buffer.calibration [3].x = get_coeff (R, X, X);
		voltage_buffer.calibration [3].y = get_coeff (R, Y, X);
		voltage_buffer.calibration [3].z = get_coeff (R, Z, X);
		voltage_buffer.calibration [4].x = get_coeff (R, X, Y);
		voltage_buffer.calibration [4].y = get_coeff (R, Y, Y);
		voltage_buffer.calibration [4].z = get_coeff (R, Z, Y);
		voltage_buffer.calibration [5].x = get_coeff (R, X, Z);
		voltage_buffer.calibration [5].y = get_coeff (R, Y, Z);
		voltage_buffer.calibration [5].z = get_coeff (R, Z, Z);
	}
	/* INVERT SCALING MATRIX */ {
		vec3* M_cal = voltage_buffer.calibration;

		double det (vec3 A, vec3 B, vec3 C) {
			return
			A.x * (B.y*C.z - B.z*C.y) -
			A.y * (B.x*C.z - B.z*C.x) +
			A.z * (B.x*C.y - B.y*C.x);
		}
		double det_L = det (M_cal [0], M_cal [1], M_cal [2]);
		double det_R = det (M_cal [3], M_cal [4], M_cal [5]);
		FCAP_ASSERT (det_L != 0.0 and det_R != 0.0);

		/* TRANSPOSE */ {
			void swap_elements (double* i, double* j) {
				double T;
				T = *i;
				*i = *j;
				*j = T;
				return;
			}
			swap_elements (&(M_cal [0].y), &(M_cal [1].x));
			swap_elements (&(M_cal [0].z), &(M_cal [2].x));
			swap_elements (&(M_cal [1].z), &(M_cal [2].y));

			swap_elements (&(M_cal [3].y), &(M_cal [4].x));
			swap_elements (&(M_cal [3].z), &(M_cal [5].x));
			swap_elements (&(M_cal [4].z), &(M_cal [5].y));
		}
		/* ADJOINT */ {
			void adjoint (char plate) {
				int o = (plate == 'R'? 3: 0);
				double M_0x = (M_cal [o+1].y * M_cal [o+2].z) - (M_cal [o+1].z * M_cal [o+2].y);
				double M_0y = - (M_cal [o+1].x * M_cal [o+2].z) + (M_cal [o+1].z * M_cal [o+2].x);
				double M_0z = (M_cal [o+1].x * M_cal [o+2].y) - (M_cal [o+1].y * M_cal [o+2].x);

				double M_1x = - (M_cal [o+0].y * M_cal [o+2].z) + (M_cal [o+0].z * M_cal [o+2].y);
				double M_1y = (M_cal [o+0].x * M_cal [o+2].z) - (M_cal [o+0].z * M_cal [o+2].x);
				double M_1z = - (M_cal [o+0].x * M_cal [o+2].y) + (M_cal [o+0].y * M_cal [o+2].x);

				double M_2x = (M_cal [o+0].y * M_cal [o+1].z) - (M_cal [o+0].z * M_cal [o+1].y);
				double M_2y = - (M_cal [o+0].x * M_cal [o+1].z) + (M_cal [o+0].z * M_cal [o+1].x);
				double M_2z = (M_cal [o+0].x * M_cal [o+1].y) - (M_cal [o+0].y * M_cal [o+1].x);

				M_cal [o+0] = newvec (M_0x, M_0y, M_0z);
				M_cal [o+1] = newvec (M_1x, M_1y, M_1z);
				M_cal [o+2] = newvec (M_2x, M_2y, M_2z);

				return;
			}
			adjoint ('L');
			adjoint ('R');
		}
		/* RESCALE */ {
			double div_L = 1.0 / det_L;
			double div_R = 1.0 / det_R;
			M_cal [0] = mulvec (M_cal [0], div_L);
			M_cal [1] = mulvec (M_cal [1], div_L);
			M_cal [2] = mulvec (M_cal [2], div_L);
			M_cal [3] = mulvec (M_cal [3], div_R);
			M_cal [4] = mulvec (M_cal [4], div_R);
			M_cal [5] = mulvec (M_cal [5], div_R);
		}
	}
	return;
}
void printCalibration () {
	printf (
		"< %lf, %lf, %lf >\n"
		"< %lf, %lf, %lf >\n"
		"< %lf, %lf, %lf >\n"
		,
		voltage_buffer.calibration [3].x,
		voltage_buffer.calibration [3].y,
		voltage_buffer.calibration [3].z,
		voltage_buffer.calibration [4].x,
		voltage_buffer.calibration [4].y,
		voltage_buffer.calibration [4].z,
		voltage_buffer.calibration [5].x,
		voltage_buffer.calibration [5].y,
		voltage_buffer.calibration [5].z
	);
	return;
}
void printDataBuffer () {
	struct voltage_buffer* 
		V = &voltage_buffer;
	struct voltage_buffer_parameters*
		P = &(*V).parameters;

	double* data_buffer = (*V).mode == SCALED? (*V).scaled_data : (*V).raw_data;

	printf ("%i samples per second for %lf seconds:\n", DAQ.parameters.sampling_rate, (*P).recording_length_inSeconds);

	for (int i = 0; i < (*P).buffer_size; i += 6) {
		int access = ((*V).write_index + i) % (*P).buffer_size;
		printf (
			"< %f,\t%f,\t%f >\t < %f,\t%f,\t%f >\n",
			data_buffer [access + 0],
			data_buffer [access + 1],
			data_buffer [access + 2],
			data_buffer [access + 3],
			data_buffer [access + 4],
			data_buffer [access + 5]
		);
	}

	return;
}
/* MAIN LOOPS */
void calibratePlates () {
	vec3 record_reference_force (char plate, char axis, vec3 baseline) {
		printf ("apply reference force to plate %c axis %c and press enter...\n", plate, axis);
		getchar ();
		vec3 reference_force = record_mean (plate);
		printf ("remove reference force and press enter...\n");
		getchar ();
		vec3 current_zero = record_mean (plate);
		if (equalvec (current_zero, baseline)) return reference_force;
		printf ("warning: baseline drift by %g\n", magvec (addvec (mulvec (current_zero, -1), baseline)));
		return reference_force;
	}
	void calibrate_axis (char plate, char axis, FILE* calibration_file) {
		printf ("align force plate %c for calibration on %c axis\n", plate, axis);
		getchar ();
		vec3 baseline = record_mean (plate);

		fprintf (calibration_file, "%c%c:\n", plate, axis);

		char input [64];
		float mass;
		while (printf ("enter reference mass or q to quit: "), fgets (input, 64, stdin) [0] != 'q') {
			sscanf (input, " %f ", &mass);
			vec3 voltage = record_reference_force (plate, axis, baseline);
			vec3 response = addvec (mulvec (baseline, -1), voltage);
			fprintf (calibration_file, "mass: %f, response: < %g, %g, %g >\n", mass, response.x, response.y, response.z);
		}

	}
	void calibrate_plates () {
		FILE* calibration_file = fopen ("./calibration.txt", "w");
		calibrate_axis ('L', 'X', calibration_file);
		calibrate_axis ('L', 'Y', calibration_file);
		calibrate_axis ('L', 'Z', calibration_file);
		calibrate_axis ('R', 'X', calibration_file);
		calibrate_axis ('R', 'Y', calibration_file);
		calibrate_axis ('R', 'Z', calibration_file);
		fclose (calibration_file);
	}

	/* OPS */
	voltage_buffer.mode = RAW;
	initialize (1.0);
	calibrate_plates ();
	terminate ();
}
void nine_point_recording () {
	float reference_mass;
	vec3 left_points [9];
	vec3 right_points [9];
	/////////////////
	initialize (1.0);
	printf ("recording baseline...\n");
	record_baseline ();
	printf ("enter reference mass: ");
	char input [64];
	fgets (input, 64, stdin);
	sscanf (input, " %f ", &reference_mass);
	//////////////////////////////
	vec3 get_point (const char* location_string, char plate) {
		int plate_index = plate == 'L'? 0: 1;
		printf ("ready for %s\n", location_string);
		getchar ();
		printf ("recording...\n");
		return addvec (record_mean (plate), voltage_buffer.baseline [plate_index]);
	}
	void get_plate_points (char plate) {
		printf ("%s plate: ", plate == 'L'? "left" : "right");
		vec3* points = plate == 'L'? left_points: right_points;
		points [0] = get_point ("top left", 	plate);
		points [1] = get_point ("top center", 	plate);
		points [2] = get_point ("top right", 	plate);
		points [3] = get_point ("center left", 	plate);
		points [4] = get_point ("center",	plate);
		points [5] = get_point ("center right", plate);
		points [6] = get_point ("bottom left", 	plate);
		points [7] = get_point ("bottom center",plate);
		points [8] = get_point ("bottom right", plate);
	}
	void save_points () {
		FILE* output = fopen ("./nine_pt_rec.txt", "w");
		FCAP_ASSERT (output);
		fprintf (output, "data points are top-left to bottom-right\n");
		fprintf (output, "reference mass: %f kg\n", reference_mass);
		void write_points_to_file (vec3* points) {
			for (int i = 0; i < 9; i ++)
				printvec (points [i], output);
		}
		fprintf (output, "left plate: \n");
		write_points_to_file (left_points);
		fprintf (output, "right plate: \n");
		write_points_to_file (right_points);
		fclose (output);
		printf ("\ndone\n");
	}
	//////////////////////////////
	get_plate_points ('L');
	get_plate_points ('R');
	save_points ();
	//////////////////////////////
	terminate ();
}
void monitorPlates (enum mode mode) {
	loadCalibration ("calibration.txt");
	voltage_buffer.mode = mode;

	int tid, status = 0;

	initialize (3.0), record_baseline(), openNodes(); 
	#pragma omp parallel num_threads(2) private(tid) shared(status)
	/* READ AND DRAW */ {
		tid = omp_get_thread_num ();
		/* READ */ if (tid is 1) {
			int finished = 0;
			while (not finished) {
				#pragma omp critical
				/* GET JOB STATUS */ {
					finished = status;
				}
				captureSignal ();
			}
		}
		/* DRAW */ if (tid is 0) { // drawing thread must be tid 0 or else window can't be destroyed (b/c owning thread dies)
			int finished = 0;
			while (not finished) {
				#pragma omp critical
				/* GET JOB STATUS */ {
					finished = status;
				}
				finished = renderDisplay ();
			}
			#pragma omp critical
			/* TERMINATE DISPLAY */ {
				cvDestroyAllWindows();
			}
			#pragma omp atomic
			status ++;
			#pragma omp flush(status)
		}
		#pragma omp barrier
	}
	/* SIGNAL END OF RECORDING */ {
		light_switch (1, &DAQ.output.red);
		for (int i = 0; i < 20000; i++);
		light_switch (0, &DAQ.output.red);
	}
	closeNodes (), /*printDataBuffer (),*/ terminate ();
}
/* ENTRY */
int main () {
	monitorPlates (RAW);
	exit (EXIT_SUCCESS);
}

/*
 **************************************************************************************
 *                      CopyRight (C) 2013 Intel Corporation
 *
 *       Filename:  2.c
 *    Description:  NA
 *        Version:  1.0
 *        Created:  2013-08-24 11:19:16
 *         Author:  qizuyong     [zuyongx.qi@intel.com]
 *
 **************************************************************************************
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <assert.h>
#include <linux/videodev2.h>
#include <getopt.h>
#include <signal.h>

/************************************************************************************/
//#define DUMP_FILE
#define V4L2_SENSOR_DEFAULT                 V4L2_SENSOR_PRIMARY
#define NUM_OF_PREVIEW_FRAME                100
#define NUM_PREVIEW_BUFFERS                 4
#define NUM_RECORDING_BUFFERS               1
#define NUM_SNAPSHOT_BUFFERS                1
#define NUM_POSTVIEW_BUFFERS                1
#define DEFAULT_PREVIEW_WIDTH               800
#define DEFAULT_PREVIEW_HEIGHT              600
#define DEFAULT_PREVIEW_FORMAT              V4L2_PIX_FMT_NV12
#define DEFAULT_RECORDING_WIDTH             1280
#define DEFAULT_RECORDING_HEIGHT            720
#define DEFAULT_RECORDING_FORMAT            V4L2_PIX_FMT_NV12
#define DEFAULT_PRIMARY_SNAPSHOT_WIDTH      3264
#define DEFAULT_PRIMARY_SNAPSHOT_HEIGHT     2448
#define DEFAULT_PRIMARY_SNAPSHOT_FORMAT     V4L2_PIX_FMT_NV12
#define DEFAULT_SECONDARY_SNAPSHOT_WIDTH    1280
#define DEFAULT_SECONDARY_SNAPSHOT_HEIGHT   720
#define DEFAULT_SECONDARY_SNAPSHOT_FORMAT   V4L2_PIX_FMT_NV12
#define DEFAULT_POSTVIEW_WIDTH              320
#define DEFAULT_POSTVIEW_HEIGHT             240
#define DEFAULT_POSTVIEW_FORMAT             V4L2_PIX_FMT_NV12
#define DEFAULT_CAPTURE_FPS                 30
#define BPP                                 2
#define SKIP_FRAMES_OF_SECONDARY            4
#define DEFAULT_CAPTURE_COUNT               4  //can not big then NUM_SNAPSHOT_BUFFERS

#define V4L2_MAX_DEVICE_COUNT               (V4L2_ISP_SUBDEV+1)
#define ATOMISP_IOC_S_EXPOSURE              _IOW('v', BASE_VIDIOC_PRIVATE + 40, struct atomisp_exposure)
#define ATOMISP_IOC_S_CONT_CAPTURE_CONFIG   _IOWR('v', BASE_VIDIOC_PRIVATE + 62, struct atomisp_cont_capture_conf)
#define ATOMISP_IOC_S_PARAMETERS            _IOW('v', BASE_VIDIOC_PRIVATE + 61, struct atomisp_parameters)
#define CLEAR(x)                            memset (&(x), 0, sizeof (x))
#define main_fd                             video_fds[V4L2_MAIN_DEVICE]

/************************************************************************************/
/* Frame status. This is used to detect corrupted frames and flash
 * exposed frames. Usually, the first 2 frames coming out of the sensor
 * are corrupted. When using flash, the frame before and the frame after
 * the flash exposed frame may be partially exposed by flash. The ISP
 * statistics for these frames should not be used by the 3A library.
 * The frame status value can be found in the "reserved" field in the
 * v4l2_buffer struct. */
enum atomisp_frame_status {
	ATOMISP_FRAME_STATUS_OK,
	ATOMISP_FRAME_STATUS_CORRUPTED,
	ATOMISP_FRAME_STATUS_FLASH_EXPOSED,
	ATOMISP_FRAME_STATUS_FLASH_PARTIAL,
	ATOMISP_FRAME_STATUS_FLASH_FAILED,
};

/* #define V4L2_CTRL_CLASS_FLASH 0x009c0000    [> Camera flash controls <] */

/* Camera class:
 * Exposure, Flash and privacy (indicator) light controls, to be upstreamed */
#define V4L2_CID_CAMERA_LASTP1             (V4L2_CID_CAMERA_CLASS_BASE + 1024)

/* Request a number of flash-exposed frames. The frame status can be
 * found in the reserved field in the v4l2_buffer struct. */
#define V4L2_CID_REQUEST_FLASH             (V4L2_CID_CAMERA_LASTP1 + 3)
/* Query flash driver status. See enum atomisp_flash_status above. */
#define V4L2_CID_FLASH_STATUS              (V4L2_CID_CAMERA_LASTP1 + 5)
/* Set the flash mode (see enum atomisp_flash_mode) */
#define V4L2_CID_FLASH_MODE                (V4L2_CID_CAMERA_LASTP1 + 10)

#define V4L2_CID_FLASH_STROBE			(V4L2_CID_FLASH_CLASS_BASE + 3)



#define V4L2_LOCK_FOCUS				(1 << 2)

#define V4L2_CID_AUTO_FOCUS_START		(V4L2_CID_CAMERA_CLASS_BASE+28)
#define V4L2_CID_AUTO_FOCUS_STOP		(V4L2_CID_CAMERA_CLASS_BASE+29)
#define V4L2_CID_AUTO_FOCUS_STATUS		(V4L2_CID_CAMERA_CLASS_BASE+30)
#define V4L2_AUTO_FOCUS_STATUS_IDLE		(0 << 0)
#define V4L2_AUTO_FOCUS_STATUS_BUSY		(1 << 0)
#define V4L2_AUTO_FOCUS_STATUS_REACHED		(1 << 1)
#define V4L2_AUTO_FOCUS_STATUS_FAILED		(1 << 2)

#define V4L2_CID_AUTO_FOCUS_RANGE		(V4L2_CID_CAMERA_CLASS_BASE+31)
#define V4L2_CID_FOCUS_ABSOLUTE			(V4L2_CID_CAMERA_CLASS_BASE+10)
#define V4L2_CID_FOCUS_RELATIVE			(V4L2_CID_CAMERA_CLASS_BASE+11)
#define V4L2_CID_FOCUS_AUTO			(V4L2_CID_CAMERA_CLASS_BASE+12)

/* Query Focus Status */
#define V4L2_CID_FOCUS_STATUS              (V4L2_CID_CAMERA_LASTP1 + 14)

enum v4l2_auto_focus_range {
	V4L2_AUTO_FOCUS_RANGE_AUTO		= 0,
	V4L2_AUTO_FOCUS_RANGE_NORMAL		= 1,
	V4L2_AUTO_FOCUS_RANGE_MACRO		= 2,
	V4L2_AUTO_FOCUS_RANGE_INFINITY		= 3,
};

/* Flash modes. Default is off.
 * Setting a flash to TORCH or INDICATOR mode will automatically
 * turn it on. Setting it to FLASH mode will not turn on the flash
 * until the FLASH_STROBE command is sent. */
enum atomisp_flash_mode {
	ATOMISP_FLASH_MODE_OFF,
	ATOMISP_FLASH_MODE_FLASH,
	ATOMISP_FLASH_MODE_TORCH,
	ATOMISP_FLASH_MODE_INDICATOR,
};


/************************************************************************************/
enum SensorID
{
    V4L2_SENSOR_PRIMARY,
    V4L2_SENSOR_SECONDARY,
};

/*ISP binary running mode*/
enum AtomIspMode
{
    CI_MODE_PREVIEW             = 0x8000,
    CI_MODE_VIDEO               = 0x4000,
    CI_MODE_CAPTURE             = 0x2000,
    CI_MODE_CONTINUOUS_CAPTURE  = 0x1000,
    CI_MODE_NONE                = 0x0000,
};

enum CameraDevice
{
    V4L2_MAIN_DEVICE       = 0,
    V4L2_POSTVIEW_DEVICE   = 1,
    V4L2_PREVIEW_DEVICE    = 2,
    V4L2_INJECT_DEVICE     = 3,
    V4L2_ISP_SUBDEV        = 4,
};

enum DeviceState
{
    DEVICE_CLOSED = 0,  /*!< kernel device closed */
    DEVICE_OPEN,        /*!< device node opened */
    DEVICE_CONFIGURED,  /*!< device format set, IOC_S_FMT */
    DEVICE_PREPARED,    /*!< buffers queued, IOC_QBUF */
    DEVICE_STARTED,     /*!< stream started, IOC_STREAMON */
    DEVICE_ERROR        /*!< undefined state */
};

struct devices
{
    unsigned int frameCounter;
    enum DeviceState state;
};

enum BufferType
{
    BUFFER_TYPE_PREVIEW = 0,
    BUFFER_TYPE_SNAPSHOT,
    BUFFER_TYPE_POSTVIEW,
    BUFFER_TYPE_RECORDING,
};

struct camera_buffer
{
    int     index;
    int     length;
    void*   viraddr;
    long    phyaddr;
    int     buffer_type;
};

struct FrameInfo
{
    int format;     // V4L2 format
    int width;      // Frame width
    int height;     // Frame height
    int stride;     // Frame stride (can be bigger than width)
    int maxWidth;   // Frame maximum width
    int maxHeight;  // Frame maximum height
    int size;       // Frame size in bytes
};

struct atomisp_exposure {
    unsigned int integration_time[8];
    unsigned int shutter_speed[8];
    unsigned int gain[4];
    unsigned int aperture;
};

struct atomisp_cont_capture_conf {
    int num_captures;
    unsigned int skip_frames;
    int offset;
    __u32 reserved[5];
};

typedef enum {
    UNKNOWN_TEST = -1,
    CAPTURE_TEST,
    VIDEO_RECORD_TEST,
    PREVIEW_TEST,
} TEST_TYPE;				/* ----------  end of enum TEST_TYPE  ---------- */

/************************************************************************************/
static enum SensorID mCameraID = V4L2_SENSOR_DEFAULT;
static enum AtomIspMode mMode;
static struct FrameInfo preview;
static struct FrameInfo recording;
static struct FrameInfo snapshot;
static struct FrameInfo postview;
static struct camera_buffer *previewBuf = NULL;
static struct camera_buffer *recordingBuf = NULL;
static struct camera_buffer *snapshotBuf = NULL;
static struct camera_buffer *postviewBuf = NULL;
static int video_fds[V4L2_MAX_DEVICE_COUNT];
static struct devices mDevices[V4L2_MAX_DEVICE_COUNT];
static int g_width = DEFAULT_SECONDARY_SNAPSHOT_WIDTH;
static int g_height = DEFAULT_SECONDARY_SNAPSHOT_HEIGHT;
static int dump = 0;
static pthread_t preview_thread_id;
static pthread_t snapshot_thread_id;

/* if enable camera flash */
static int flash_en = 0;
static int flash_torch = 0;
static int flash_indicator = 0;
static int focus_en = 0;

/* flash default touch intensity */
static int torch_intensity = 80;
static int indicator_intensity = 100;
static int vflip_enable = 0;
static int hflip_enable = 0;

 /* if video recording resolution smaller then the preview resolution,
  * we will swap the preview device and the video record device
  */
static int device_swap = 0;

#define VIDEO0_PATH "/dev/video0"
#define VIDEO1_PATH "/dev/video1"
#define VIDEO2_PATH "/dev/video2"
#define VIDEO3_PATH "/dev/video3"

static char *dev_name_array[] = {VIDEO0_PATH,
								 VIDEO1_PATH,
                                 VIDEO2_PATH,
                                 VIDEO3_PATH};

/************************************************************************************/

void dump_frameinfo(struct FrameInfo info)
{
    printf("width=%d height=%d maxWidth=%d maxHeight=%d size=%d stride=%d format=%d\n",
    info.width, info.height, info.maxWidth, info.maxHeight, info.size, info.stride, info.format);
}

void dump_camera_buffer(struct camera_buffer *buf)
{
    int buf_count;
    int i;

    switch(buf->buffer_type)
    {
        case BUFFER_TYPE_PREVIEW:
            buf_count = NUM_PREVIEW_BUFFERS;
            break;
        case BUFFER_TYPE_SNAPSHOT:
            buf_count = NUM_SNAPSHOT_BUFFERS;
            break;
        case BUFFER_TYPE_POSTVIEW:
            buf_count = NUM_POSTVIEW_BUFFERS;
            break;
        case BUFFER_TYPE_RECORDING:
            buf_count = NUM_RECORDING_BUFFERS;
            break;
        default:
            printf("error buffer type\n");
            break;
    }

    for (i = 0; i < buf_count; i++)
    {
        printf("buf[%d].index = %d\n", i, buf[i].index);
        printf("buf[%d].length = %d\n", i, buf[i].length);
        printf("buf[%d].viraddr = 0x%p\n", i, buf[i].viraddr);
        printf("buf[%d].phyaddr = 0x%x\n", i, buf[i].phyaddr);
    }
}

/*
 **************************************************************************
 * FunctionName: time_subtract;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int time_subtract(struct timeval *result, struct timeval *x, struct timeval *y)
{
    int nsec;

    if ( x->tv_sec > y->tv_sec )
        return -1;

    if ((x->tv_sec==y->tv_sec) && (x->tv_usec>y->tv_usec))
        return -1;

    result->tv_sec = ( y->tv_sec-x->tv_sec );
    result->tv_usec = ( y->tv_usec-x->tv_usec );

    if (result->tv_usec<0)
    {
        result->tv_sec--;
        result->tv_usec+=1000000;
    }

    return 0;
}

void spare_time(struct timeval *start, char *msg)
{
	struct timeval diff, cur;
	if (start == NULL || msg == NULL)
	{
		printf("error: invalid argument\n");
		return;
	}
	gettimeofday(&cur, NULL);
	time_subtract(&diff, start, &cur);
	printf("time: %s spare time %lu.%lu\n", msg, diff.tv_sec, diff.tv_usec);
	gettimeofday(start, NULL);
}


/*
 **************************************************************************
 * FunctionName: v4l2_capture_open;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int v4l2_capture_open(int device)
{
    int fd;
    struct stat st;

    if ((device < V4L2_MAIN_DEVICE ) && device != V4L2_ISP_SUBDEV)
    {
        printf("Wrong device node %d\n", device);
        return -1;
    }

    const char *dev_name = NULL;
    if (device == V4L2_ISP_SUBDEV)
    {
        printf("device is V4L2_ISP_SUBDEV\n");
    }
    else
    {
        dev_name = dev_name_array[device];
    }

    if (stat (dev_name, &st) == -1)
    {
        printf("Error stat video device %s: %s\n", dev_name, strerror(errno));
        return -1;
    }

    if (!S_ISCHR (st.st_mode))
    {
        printf("%s is not a device", dev_name);
        return -1;
    }

    fd = open(dev_name, O_RDWR);

    if (fd <= 0)
    {
        printf("Error opening video device %s: %s\n",dev_name, strerror(errno));
        return -1;
    }
    printf("open %s sucess, fd=%d\n", dev_name, fd);

    return fd;
}

/*
 **************************************************************************
 * FunctionName: openDevice;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int openDevice(int device)
{
    if (video_fds[device] > 0)
    {
        printf("MainDevice already opened!\n");
        return video_fds[device];
    }

    video_fds[device] = v4l2_capture_open(device);

    printf("Open device %d with fd %d\n", device, video_fds[device]);

    if (video_fds[device] < 0)
    {
        printf("V4L2: capture_open failed: %s\n", strerror(errno));
        return -1;
    }

    mDevices[device].state = DEVICE_OPEN;

    return video_fds[device];
}

/*
 **************************************************************************
 * FunctionName: v4l2_capture_close;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int v4l2_capture_close(int fd)
{
    /* close video device */
    if (fd < 0)
    {
        printf("Device not opened!\n");
        return -1;
    }

    if (close(fd) < 0)
    {
        printf("Close video device failed: %s\n", strerror(errno));
        return -1;
    }
    return 0;
}

/*
 **************************************************************************
 * FunctionName: closeDevice;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static void closeDevice(int device)
{
    if (video_fds[device] < 0)
    {
        printf("Device %d already closed. Do nothing\n", device);
        return;
    }

    v4l2_capture_close(video_fds[device]);

    video_fds[device] = -1;
    mDevices[device].state = DEVICE_CLOSED;
}

/*
 **************************************************************************
 * FunctionName: v4l2_queue_buffer;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int v4l2_queue_buffer(int fd, int index)
{
    struct v4l2_buffer buf;
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = index;

    if (ioctl (fd, VIDIOC_QBUF, &buf) < 0)
    {
        printf("ioctl VIDIOC_QBUF failed %d, %s\n", errno, strerror(errno));
        return -1;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: v4l2_dequeue_buffer;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int v4l2_dequeue_buffer(int fd)
{
    struct v4l2_buffer v4l2_buffer;
    memset(&v4l2_buffer, 0, sizeof(v4l2_buffer));
    v4l2_buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    v4l2_buffer.memory = V4L2_MEMORY_MMAP;

    if (ioctl (fd, VIDIOC_DQBUF, &v4l2_buffer) < 0)
    {
        printf("ioctl VIDIOC_DQBUF failed %d, %s\n", errno, strerror(errno));
        return -1;
    }
	int frame_status = v4l2_buffer.reserved;

	// atom flag is an extended set of flags, so map V4L2 flags¬
	// we are interested into atomisp_frame_status¬
	if (v4l2_buffer.flags & V4L2_BUF_FLAG_ERROR)
		frame_status = ATOMISP_FRAME_STATUS_CORRUPTED;
    printf("fd=%d index=%d frame_status=%d\n", fd, v4l2_buffer.index, frame_status);

    return v4l2_buffer.index;
}

/*
 **************************************************************************
 * FunctionName: v4l2_stream_on;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int v4l2_stream_on(int fd)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl (fd, VIDIOC_STREAMON, &type) < 0)
    {
        printf("ioctl VIDIOC_STREAMON failed %d, %s\n", errno, strerror(errno));
        return -1;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: v4l2_stream_off;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int v4l2_stream_off(int fd)
{
    enum v4l2_buf_type type;

    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl (fd, VIDIOC_STREAMOFF, &type) < 0)
    {
        printf("ioctl VIDIOC_STREAMOFF failed %d, %s\n", errno, strerror(errno));
        return -1;
    }

    return 0;
}

int sensorMoveFocusToPosition(int position)
{
    return atomisp_set_attribute(main_fd, V4L2_CID_FOCUS_ABSOLUTE, position, "Set focus position");
}

int getFocusPosition(int *position)
{
    return atomisp_get_attribute(main_fd,V4L2_CID_FOCUS_ABSOLUTE, position);
}

int sensorMoveFocusToBySteps(int steps)
{
    return atomisp_set_attribute(main_fd, V4L2_CID_FOCUS_RELATIVE, steps, "Set focus steps");
}

int sensorGetFocusStatus(int *status)
{
    return atomisp_get_attribute(main_fd,V4L2_CID_FOCUS_STATUS, status);
}

/* manual focus test */
int focus_test()
{
	int status;
	int i;
	int position;
	for (i = 100; i < 10000;)
	{
		sensorMoveFocusToPosition(i);
		getFocusPosition(&position);
		sensorGetFocusStatus(&status);
		i += 100;
		printf("Foucus current position=%d, status=0x%x\n",position, status);
	}
}

int setFlashIndicator(int intensity)
{
	printf("@%s: intensity = %d", __FUNCTION__, intensity);
    if (mCameraID != V4L2_SENSOR_PRIMARY) {
        printf("Indicator intensity is supported only for primary camera!");
        return -1;
    }

    if (intensity) {
        if (atomisp_set_attribute(main_fd, V4L2_CID_FLASH_INDICATOR_INTENSITY, intensity, "Indicator Intensity") < 0)
            return -1;
        if (atomisp_set_attribute(main_fd, V4L2_CID_FLASH_MODE, ATOMISP_FLASH_MODE_INDICATOR, "Flash Mode") < 0)
            return -1;
    } else {
        if (atomisp_set_attribute(main_fd, V4L2_CID_FLASH_MODE, ATOMISP_FLASH_MODE_OFF, "Flash Mode") < 0)
            return -1;
    }
    return 0;
}

int setTorchHelper(int intensity)
{
    if (intensity) {
        if (atomisp_set_attribute(main_fd, V4L2_CID_FLASH_TORCH_INTENSITY, intensity, "Torch Intensity") < 0)
            return -1;
        if (atomisp_set_attribute(main_fd, V4L2_CID_FLASH_MODE, ATOMISP_FLASH_MODE_TORCH, "Flash Mode") < 0)
            return -1;
    } else {
        if (atomisp_set_attribute(main_fd, V4L2_CID_FLASH_MODE, ATOMISP_FLASH_MODE_OFF, "Flash Mode") < 0)
            return -1;
    }
    return 0;
}

int setTorch(int intensity)
{
    printf("@%s: intensity = %d", __FUNCTION__, intensity);

    if (mCameraID != V4L2_SENSOR_PRIMARY) {
        printf("Indicator intensity is supported only for primary camera!");
        return -1;
    }

    setTorchHelper(intensity);

    return 0;
}

static int setFlash(int numFrames)
{
    printf("@%s: numFrames = %d", __FUNCTION__, numFrames);
    if (mCameraID != V4L2_SENSOR_PRIMARY) {
        printf("Flash is supported only for primary camera!");
        return -1;
    }
    if (numFrames) {
        if (atomisp_set_attribute(main_fd, V4L2_CID_FLASH_MODE, ATOMISP_FLASH_MODE_FLASH, "Flash Mode flash") < 0)
            return -1;
		if (atomisp_set_attribute(main_fd, V4L2_CID_REQUEST_FLASH, numFrames, "Request Flash") < 0)
			return -1;
    } else {
        if (atomisp_set_attribute(main_fd, V4L2_CID_FLASH_MODE, ATOMISP_FLASH_MODE_OFF, "Flash Mode flash") < 0)
            return -1;
    }
    return 0;
}

static int set_vflip(int fd, int value)
{
	int vflip = 0;
	if (atomisp_set_attribute(fd, V4L2_CID_VFLIP, value, "set vflip") < 0)
		return -1;

	if (atomisp_get_attribute(fd, V4L2_CID_VFLIP, &vflip) < 0)
		return -1;
	printf("vflip value=%d\n", vflip);

	return 0;
}

static int set_hflip(int fd, int value)
{
	int hflip;
	if (atomisp_set_attribute(fd, V4L2_CID_HFLIP, value, "set hflip") < 0)
		return -1;

	if (atomisp_get_attribute(fd, V4L2_CID_HFLIP, &hflip) < 0)
		return -1;
	printf("hflip value=%d\n", hflip);

	return 0;
}

/*
 **************************************************************************
 * FunctionName: init_camera_buffers;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int init_camera_buffers(struct camera_buffer *buffer, int buffer_type)
{
    int i = 0;
    switch(buffer_type)
    {
        case BUFFER_TYPE_PREVIEW:
            memset(buffer, 0x00, sizeof(struct camera_buffer) * NUM_PREVIEW_BUFFERS);
            for(i=0; i<NUM_PREVIEW_BUFFERS; i++)
            {
                buffer[i].buffer_type = buffer_type;
            }
            break;
        case BUFFER_TYPE_SNAPSHOT:
            memset(buffer, 0x00, sizeof(struct camera_buffer) * NUM_SNAPSHOT_BUFFERS);
            for(i=0; i<NUM_SNAPSHOT_BUFFERS; i++)
            {
                buffer[i].buffer_type = buffer_type;
            }
            break;
        case BUFFER_TYPE_POSTVIEW:
            memset(buffer, 0x00, sizeof(struct camera_buffer) * NUM_POSTVIEW_BUFFERS);
            for(i=0; i<NUM_POSTVIEW_BUFFERS; i++)
            {
                buffer[i].buffer_type = buffer_type;
            }
            break;
        case BUFFER_TYPE_RECORDING:
            memset(buffer, 0x00, sizeof(struct camera_buffer) * NUM_RECORDING_BUFFERS);
            for(i=0; i<NUM_RECORDING_BUFFERS; i++)
            {
                buffer[i].buffer_type = buffer_type;
            }
            break;
        default:
            printf("error buffer type\n");
            break;
    }


    return 0;
}

/*
 **************************************************************************
 * FunctionName: deinit_camera_buffers;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int deinit_camera_buffers(struct camera_buffer *buffer, int buffer_type)
{
    int index;
    if(buffer == NULL)
    {
        printf("buffer is deinit or no init\n");
        return 0;
    }

    switch(buffer_type)
    {
        case BUFFER_TYPE_PREVIEW:
            for(index = 0; index < NUM_PREVIEW_BUFFERS; index++)
            {
                if(buffer[index].viraddr != NULL)
                {
                    munmap(buffer[index].viraddr, buffer[index].length);
                    buffer[index].viraddr = NULL;
                }
            }
            break;
        case BUFFER_TYPE_SNAPSHOT:
            for(index = 0; index < NUM_SNAPSHOT_BUFFERS; index++)
            {
                if(buffer[index].viraddr != NULL)
                {
                    munmap(buffer[index].viraddr, buffer[index].length);
                    buffer[index].viraddr = NULL;
                }
            }
            break;
        case BUFFER_TYPE_POSTVIEW:
            for(index = 0; index < NUM_POSTVIEW_BUFFERS; index++)
            {
                if(buffer[index].viraddr != NULL)
                {
                    munmap(buffer[index].viraddr, buffer[index].length);
                    buffer[index].viraddr = NULL;
                }
            }
            break;
        case BUFFER_TYPE_RECORDING:
            for(index = 0; index < NUM_RECORDING_BUFFERS; index++)
            {
                if(buffer[index].viraddr != NULL)
                {
                    munmap(buffer[index].viraddr, buffer[index].length);
                    buffer[index].viraddr = NULL;
                }
            }
            break;
        default:
            printf("error buffer type\n");
            break;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: v4l2_capture_s_input;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int v4l2_capture_s_input(int fd, int index)
{
    struct v4l2_input input;
    int ret;

    input.index = index;

    ret = ioctl(fd, VIDIOC_S_INPUT, &input);

    if (ret < 0)
    {
        printf("VIDIOC_S_INPUT index %d returned: %d (%s)\n",
            input.index, ret, strerror(errno));
        return ret;
    }

    return ret;
}

/*
 **************************************************************************
 * FunctionName: selectCameraSensor;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int selectCameraSensor()
{
    int ret = 0;
    int device = V4L2_MAIN_DEVICE;

    //Choose the camera sensor
    ret = v4l2_capture_s_input(video_fds[device], mCameraID);
    if (ret < 0)
    {
        printf("V4L2: capture_s_input failed: %s\n", strerror(errno));
        v4l2_capture_close(video_fds[device]);
        video_fds[device] = -1;
        return -1;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: atomisp_set_capture_mode;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int atomisp_set_capture_mode(int deviceMode)
{
    struct v4l2_streamparm parm;

    switch (deviceMode)
    {
        case CI_MODE_PREVIEW:
            printf("Setting CI_MODE_PREVIEW mode\n");
            break;;
        case CI_MODE_CAPTURE:
            printf("Setting CI_MODE_CAPTURE mode\n");
            break;
        case CI_MODE_VIDEO:
            printf("Setting CI_MODE_VIDEO mode\n");
            break;
        default:
            break;
    }

    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.capturemode = deviceMode;
    if (ioctl(main_fd, VIDIOC_S_PARM, &parm) < 0)
    {
        printf("error %s", strerror(errno));
        return -1;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: atomisp_set_capture_fps;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int atomisp_set_capture_fps(int fps)
{
    struct v4l2_streamparm parm;
    CLEAR(parm);

    parm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    parm.parm.capture.timeperframe.numerator = fps;
    parm.parm.capture.timeperframe.denominator = 1;
    parm.parm.capture.capturemode = CI_MODE_NONE;
    if (ioctl(main_fd, VIDIOC_S_PARM, &parm) < 0)
    {
        printf("error %s", strerror(errno));
        return -1;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: bytesPerLineToWidth;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int bytesPerLineToWidth(int format, int bytesperline)
{
    int width = 0;
    switch (format)
    {
        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_YVU420:
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV21:
        case V4L2_PIX_FMT_YUV411P:
        case V4L2_PIX_FMT_YUV422P:
            width = (bytesperline * 2 / 3);
            break;
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_Y41P:
        case V4L2_PIX_FMT_UYVY:
            width = (bytesperline / 2);
            break;
        case V4L2_PIX_FMT_RGB565:
            width = (bytesperline / 2);
            break;
        default:
            width = (bytesperline / 2);
    }

    return width;
}

/*
 **************************************************************************
 * FunctionName: v4l2_capture_s_format;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int v4l2_capture_s_format(int fd, int device, int w, int h, int fourcc, int raw, int* stride)
{
    int ret;
    struct v4l2_format v4l2_fmt;
    CLEAR(v4l2_fmt);

    v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl (fd,  VIDIOC_G_FMT, &v4l2_fmt);
    if (ret < 0)
    {
        printf("VIDIOC_G_FMT failed: %s\n", strerror(errno));
        return -1;
    }
    if (raw)
    {
        printf("Choose raw dump path\n");
        v4l2_fmt.type = V4L2_BUF_TYPE_PRIVATE;
    }
    else
    {
        v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    }

    v4l2_fmt.fmt.pix.width = w;
    v4l2_fmt.fmt.pix.height = h;
    v4l2_fmt.fmt.pix.pixelformat = fourcc;
    v4l2_fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
    printf("fd: %d, VIDIOC_S_FMT expect : width: %d, height: %d, format: %d, field: %d\n",
                fd,
                v4l2_fmt.fmt.pix.width,
                v4l2_fmt.fmt.pix.height,
                v4l2_fmt.fmt.pix.pixelformat,
                v4l2_fmt.fmt.pix.field);
    ret = ioctl(fd, VIDIOC_S_FMT, &v4l2_fmt);
    if (ret < 0)
    {
        printf("fd: %d, VIDIOC_S_FMT failed: %s\n", fd, strerror(errno));
        return -1;
    }
    printf("fd: %d, VIDIOC_S_FMT real: width: %d, height: %d, format: %d, field: %d\n",
                fd,
                v4l2_fmt.fmt.pix.width,
                v4l2_fmt.fmt.pix.height,
                v4l2_fmt.fmt.pix.pixelformat,
                v4l2_fmt.fmt.pix.field);

    //get stride from ISP
    *stride = bytesPerLineToWidth(fourcc,v4l2_fmt.fmt.pix.bytesperline);
    printf("stride: %d from ISP\n", *stride);

    CLEAR(v4l2_fmt);
    v4l2_fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl (fd,  VIDIOC_G_FMT, &v4l2_fmt);
    if (ret < 0)
    {
        printf("fd:%d, VIDIOC_G_FMT failed: %s\n", fd, strerror(errno));
        return -1;
    }

    if((v4l2_fmt.fmt.pix.width != (unsigned int)w) || (v4l2_fmt.fmt.pix.height != (unsigned int)h) || (v4l2_fmt.fmt.pix.pixelformat != (unsigned int)fourcc))
    {
        printf("set format failed, not equal the goal format\n");
        ret = -1;
    }

    printf("v4l2_fmt.fmt.pix.width=%d v4l2_fmt.fmt.pix.height=%d\n", v4l2_fmt.fmt.pix.width, v4l2_fmt.fmt.pix.height);

    return ret;

}

/**
 * Configures a particular device with a mode (preview, video or capture)
 *
 * The FrameInfo struct contains information about the frame dimensions that
 * we are requesting to ISP
 * the field stride of the FrameInfo struct will be updated with the actual
 * width that the buffers need to have to meet the ISP constrains.
 * In effect the FrameInfo struct is an IN/OUT parameter.
 */
/*
 **************************************************************************
 * FunctionName: configureDevice;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int configureDevice(int device, int deviceMode,struct FrameInfo *fInfo, int raw)
{
    int ret = 0;
    int w,h,format;
    w = fInfo->width;
    h = fInfo->height;
    format = fInfo->format;
    printf("device: %d path: %s, width:%d, height:%d, deviceMode:%d format:%d raw:%d\n", device, dev_name_array[device],
        w, h, deviceMode, format, raw);

    if ((w <= 0) || (h <= 0))
    {
        printf("Wrong Width %d or Height %d", w, h);
        return -1;
    }

    //Only update the configure for device
    int fd = video_fds[device];

    //Switch the Mode before set the format. This is the requirement of
    //atomisp
    ret = atomisp_set_capture_mode(deviceMode);
    if (ret < 0)
        return ret;

    //Set the format
    ret = v4l2_capture_s_format(fd, device, w, h, format, raw, &(fInfo->stride));
    if (ret < 0)
    {
        printf("v4l2_capture_s_format failed\n");
        return ret;
    }

    //atomisp_set_capture_fps(DEFAULT_CAPTURE_FPS);

    mDevices[device].state = DEVICE_CONFIGURED;

    //We need apply all the parameter settings when do the camera reset
    return ret;

}

/*
 **************************************************************************
 * FunctionName: frameSize;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int frameSize(int format, int width, int height)
{
    int size = 0;
    switch (format)
    {
        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_YVU420:
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV21:
        case V4L2_PIX_FMT_YUV411P:
        case V4L2_PIX_FMT_YUV422P:
            size = (width * height * 3 / 2);
            break;
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_Y41P:
        case V4L2_PIX_FMT_UYVY:
            size = (width * height *  2);
            break;
        case V4L2_PIX_FMT_RGB565:
            size = (width * height * BPP);
            break;
        default:
            size = (width * height * 2);
    }

    return size;
}

int v4l2_enum_framesize(int fd, int pixelformat)
{
    struct v4l2_frmsizeenum size;
    int w, h;
    int min_w, step_w, max_w, min_h, step_h, max_h;

    memset (&size, 0, sizeof (struct v4l2_frmsizeenum));
    size.index = 0;
    size.pixel_format = pixelformat;
    printf("framsizeenum.index = %d\n", size.index);
    printf("framsizeenum.pixel_format= %d\n", size.pixel_format);

    if (ioctl (fd, VIDIOC_ENUM_FRAMESIZES, &size) < 0)
    {
        printf("ioctl VIDIOC_ENUM_FRAMESIZES failed, %d %s\n", errno, strerror(errno));
        return -1;
    }

    switch(size.type)
    {
        case V4L2_FRMSIZE_TYPE_DISCRETE:
            printf("framesize type:V4L2_FRMSIZE_TYPE_DISCRETE\n");
            w = size.discrete.width;
            h = size.discrete.height;
            break;
        case V4L2_FRMSIZE_TYPE_STEPWISE:
            printf("framesize type:V4L2_FRMSIZE_TYPE_STEPWISE\n");
            printf("max_width=%d\n", size.stepwise.max_width);
            printf("max_height=%d\n", size.stepwise.max_height);
            printf("min_width=%d\n", size.stepwise.min_width);
            printf("min_height=%d\n", size.stepwise.min_height);
            printf("step_width=%d\n", size.stepwise.step_width);
            printf("step_height=%d\n", size.stepwise.step_height);
            break;
        case V4L2_FRMSIZE_TYPE_CONTINUOUS:
            printf("type:V4L2_FRMSIZE_TYPE_CONTINUOUS\n");
            break;
    }
    return 0;
}

int v4l2_enum_frameinterval(int fd, int pixelformat, int width, int height)
{
    struct v4l2_frmivalenum ival;
    int ret;
    memset(&ival, 0, sizeof(struct v4l2_frmivalenum));
    ival.index = 0;
    ival.pixel_format = pixelformat;
    ival.width = width;
    ival.height = height;
    ret = ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &ival);
    if (fd < 0)
    {
        printf("ioctl enum enum_frameintervals_failed, %d %s \n", errno, strerror(errno));
        return ret;
    }

    return 0;
}



/*
 **************************************************************************
 * FunctionName: xioctl;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int xioctl(int fd, int request, void *arg)
{
    int ret;
    do
    {
        ret = ioctl(fd, request, arg);
    } while (-1 == ret && EINTR == errno);

    if (ret < 0)
        printf("%s: Request 0x%x failed: %s\n", __FUNCTION__, request, strerror(errno));

    return ret;
}

/*
 **************************************************************************
 * FunctionName: sensorSetExposure;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int sensorSetExposure(struct atomisp_exposure *exposure)
{
    return xioctl(main_fd, ATOMISP_IOC_S_EXPOSURE, exposure);
}

/*
 **************************************************************************
 * FunctionName: updateCameraParams;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int updateCameraParams()
{
    if(mCameraID == V4L2_SENSOR_PRIMARY)
    {
        struct atomisp_exposure exposure;
        CLEAR(exposure);
        exposure.integration_time[0]=593;
        exposure.gain[0]=16;
        exposure.gain[1]=1024;

        int ret = sensorSetExposure(&exposure);
        if(ret < 0)
        {
            printf("set exposure failed\n");
            return -1;
        }
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: updateCameraParams;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int configurePreview()
{
    updateCameraParams();

    int  ret = openDevice(V4L2_PREVIEW_DEVICE);
    if (ret < 0)
    {
        printf("Open preview device failed!\n");
        return -1;
    }

    ret = configureDevice(V4L2_PREVIEW_DEVICE,CI_MODE_PREVIEW,&preview, 0);
    if(ret < 0)
    {
        printf("configureDevice failed\n");
    }

    return ret;

}

/*
 **************************************************************************
 * FunctionName: configureCapture;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int configureCapture()
{
    int ret;

    ret = configureDevice(V4L2_MAIN_DEVICE, CI_MODE_CAPTURE, &snapshot, 0);
    if (ret < 0)
    {
        printf("configure first device failed!");
        return ret;;
    }

    ret = openDevice(V4L2_POSTVIEW_DEVICE);
    if (ret < 0)
    {
        printf("Open second device failed!");
        return -1;
    }

    ret = configureDevice(V4L2_POSTVIEW_DEVICE, CI_MODE_CAPTURE, &postview, 0);
    if (ret < 0)
    {
        printf("configure second device failed!");
        closeDevice(V4L2_POSTVIEW_DEVICE);
        return ret;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: requestContCapture;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int requestContCapture(int numCaptures, int offset, unsigned int skip)
{
    struct atomisp_cont_capture_conf conf;

    CLEAR(conf);
    conf.num_captures = numCaptures;
    conf.offset = offset;
    conf.skip_frames = skip;

    int res = xioctl(video_fds[V4L2_MAIN_DEVICE],
                     ATOMISP_IOC_S_CONT_CAPTURE_CONFIG,
                     &conf);
    if (res != 0) {
        printf("@%s: error with CONT_CAPTURE_CONFIG, res %d\n", __FUNCTION__, res);
        return res;
    }

    return 0;

}

/**
 * atomisp_get_attribute():
 * Try to get the value of one specific attribute
 * return value: 0 for success
 *               others are errors
 */
int atomisp_get_attribute (int fd, int attribute_num, int *value)
{
    struct v4l2_control control;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control ext_control;

    if (fd < 0)
        return -1;

    control.id = attribute_num;
    controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
    controls.count = 1;
    controls.controls = &ext_control;
    ext_control.id = attribute_num;

    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &controls) == 0) {
        *value = ext_control.value;
		return 0;
    }

    if (ioctl(fd, VIDIOC_G_CTRL, &control) == 0) {
        *value = control.value;
        return 0;
    }

    printf("Failed to get value for control (%d) on device '%d', %s\n.",
          attribute_num, fd, strerror(errno));
    return -1;
}

/*
 **************************************************************************
 * FunctionName: atomisp_set_attribute;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int atomisp_set_attribute (int fd, int attribute_num,
                                             const int value, const char *name)
{
    struct v4l2_control control;
    struct v4l2_ext_controls controls;
    struct v4l2_ext_control ext_control;

    printf("setting attribute [%s]%d to %d\n", name, attribute_num, value);

    if (fd < 0)
        return -1;

    control.id = attribute_num;
    control.value = value;
    controls.ctrl_class = V4L2_CTRL_ID2CLASS(control.id);
    controls.count = 1;
    controls.controls = &ext_control;
    ext_control.id = attribute_num;
    ext_control.value = value;

    if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &controls) == 0)
        return 0;

    if (ioctl(fd, VIDIOC_S_CTRL, &control) == 0)
        return 0;

    printf("Failed to set value %d for control %s (%d) on device '%d', %s\n",
        value, name, attribute_num, fd, strerror(errno));
    return -1;
}

/*
 **************************************************************************
 * FunctionName: configureContinuous;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int configureContinuous()
{
    int ret = 0;

    requestContCapture(10, -1, 0);

    ret = configureDevice(V4L2_MAIN_DEVICE, CI_MODE_PREVIEW, &snapshot, 0);
    if (ret < 0)
    {
        printf("configure first device failed!\n");
        return -1;
    }

    ret = configurePreview();
    if (ret < 0)
    {
        printf("configure preview failed\n");
        return ret;
    }

    ret = openDevice(V4L2_POSTVIEW_DEVICE);
    if (ret < 0)
    {
        printf("Open second device failed!\n");
        return ret;
    }

    ret = configureDevice(V4L2_POSTVIEW_DEVICE, CI_MODE_PREVIEW, &postview, 0);
    if (ret < 0)
    {
        printf("configure second device failed!\n");
        return ret;
    }

    return ret;
}

/*
 **************************************************************************
 * FunctionName: configureRecording;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int configureRecording()
{
    int ret = 0;

    updateCameraParams();

    ret = openDevice(V4L2_PREVIEW_DEVICE);
    if (ret < 0)
    {
        printf("Open preview device failed!\n");
        return -1;
    }

    if (!device_swap)
    {
        ret = configureDevice(V4L2_MAIN_DEVICE, CI_MODE_VIDEO, &recording, 0);
        if (ret < 0)
        {
            printf("Configure recording main device failed!\n");
            closeDevice(V4L2_PREVIEW_DEVICE);
            return -1;
        }

        ret = configureDevice(V4L2_PREVIEW_DEVICE, CI_MODE_VIDEO, &preview, 0);
        if (ret < 0)
        {
            printf("Configure recording preivew device failed!\n");
            closeDevice(V4L2_PREVIEW_DEVICE);
            return -1;
        }
    }
    else
    {
        ret = configureDevice(V4L2_PREVIEW_DEVICE, CI_MODE_VIDEO, &preview, 0);
        if (ret < 0)
        {
            printf("Configure recording preivew device failed!\n");
            closeDevice(V4L2_PREVIEW_DEVICE);
            return -1;
        }

        ret = configureDevice(V4L2_MAIN_DEVICE, CI_MODE_VIDEO, &recording, 0);
        if (ret < 0)
        {
            printf("Configure recording main device failed!\n");
            closeDevice(V4L2_PREVIEW_DEVICE);
            return -1;
        }
        device_swap = 0;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: configure;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int configure(enum AtomIspMode mode)
{
    int status = 0;
    switch (mode)
    {
        case CI_MODE_PREVIEW:
            status = configurePreview();
            break;
        case CI_MODE_VIDEO:
            status = configureRecording();
            break;
        case CI_MODE_CAPTURE:
            status = configureCapture();
            break;
        case CI_MODE_CONTINUOUS_CAPTURE:
            status = configureContinuous();
            break;
        default:
            status = -1;
            break;
    }

    if (status == 0)
        mMode = mode;

    return status;
}

int releaseBuffer(int fd)
{
    struct v4l2_requestbuffers req_buf;
	int ret = 0;
    CLEAR(req_buf);

    req_buf.memory = V4L2_MEMORY_MMAP;
    req_buf.count = 0;
    req_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ret = ioctl(fd, VIDIOC_REQBUFS, &req_buf);
    if (ret < 0)
    {
        printf("VIDIOC_REQBUFS returned: %d (%s)\n",ret, strerror(errno));
        return ret;
    }
	return 0;
}

/*
 **************************************************************************
 * FunctionName: prepareDevice;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int prepareDevice(int device, int buffer_count)
{
    int ret;
    int fd = video_fds[device];
    struct v4l2_requestbuffers req_buf;
    CLEAR(req_buf);

    printf("prepareDevice fd = %d\n", fd);

    req_buf.memory = V4L2_MEMORY_MMAP;
    req_buf.count = buffer_count;
    req_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ret = ioctl(fd, VIDIOC_REQBUFS, &req_buf);
    if (ret < 0)
    {
        printf("VIDIOC_REQBUFS(%d) returned: %d (%s)\n", buffer_count, ret, strerror(errno));
        return ret;
    }

    if (req_buf.count < (unsigned int)buffer_count)
    {
        printf("Got less buffers than requested!\n");
    }

    int i = 0;
    for (i = 0; i < buffer_count; i++)
    {
        struct v4l2_buffer vbuf;
        CLEAR(vbuf);
        vbuf.memory = V4L2_MEMORY_MMAP;
        vbuf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        vbuf.index = i;

        ret = ioctl(fd , VIDIOC_QUERYBUF, &vbuf);
        if (ret < 0)
        {
            printf("VIDIOC_QUERYBUF failed: %s\n", strerror(errno));
            return ret;
        }

        void *data = mmap(NULL, vbuf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, vbuf.m.offset);

        if (MAP_FAILED == data)
        {
            printf("mmap failed: %s\n", strerror(errno));
            return -1;
        }

        if(device == V4L2_PREVIEW_DEVICE)
        {
            previewBuf[vbuf.index].index = vbuf.index;
            previewBuf[vbuf.index].length = vbuf.length;
            previewBuf[vbuf.index].viraddr = data;
            previewBuf[vbuf.index].phyaddr = vbuf.m.offset;
            printf("preview:index=%d, length=%d, viraddr=%p, phyaddr=%08x, size=%d\n", vbuf.index, vbuf.length, data, vbuf.m.offset, preview.size);
        }
        else if(device == V4L2_POSTVIEW_DEVICE)
        {
            postviewBuf[vbuf.index].index = vbuf.index;
            postviewBuf[vbuf.index].length = vbuf.length;
            postviewBuf[vbuf.index].viraddr = data;
            postviewBuf[vbuf.index].phyaddr = vbuf.m.offset;
            printf("postview:index=%d, length=%d, viraddr=%p, phyaddr=%08x, size=%d\n", vbuf.index, vbuf.length, data, vbuf.m.offset, postview.size);
        }
        else if(( mMode == CI_MODE_VIDEO) && (device == V4L2_MAIN_DEVICE))
        {
            recordingBuf[vbuf.index].index = vbuf.index;
            recordingBuf[vbuf.index].length = vbuf.length;
            recordingBuf[vbuf.index].viraddr = data;
            recordingBuf[vbuf.index].phyaddr = vbuf.m.offset;
            printf("recording:index=%d, length=%d, viraddr=%p, phyaddr=%08x, size=%d\n", vbuf.index, vbuf.length, data, vbuf.m.offset, recording.size);
        }
        else
        {
            snapshotBuf[vbuf.index].index = vbuf.index;
            snapshotBuf[vbuf.index].length = vbuf.length;
            snapshotBuf[vbuf.index].viraddr = data;
            snapshotBuf[vbuf.index].phyaddr = vbuf.m.offset;
            printf("snapshot:index=%d, length=%d, viraddr=%p, phyaddr=%08x, size=%d\n", vbuf.index, vbuf.length, data, vbuf.m.offset, snapshot.size);
        }
    }
    mDevices[device].state = DEVICE_PREPARED;

    printf("prepareDevice sucess\n");

    return 0;
}

/*
 **************************************************************************
 * FunctionName: activateBufferPool;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int activateBufferPool(int device, int buffer_count)
{
    int fd = video_fds[device];
    int ret;

    int index;
    for(index = 0; index<buffer_count; index++)
    {
        ret = v4l2_queue_buffer(fd, index);
        if(ret < 0)
        {
            printf("queue buffer failed\n");
            return -1;
        }
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: startDevice;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int startDevice(int device, int buffer_count)
{
    int ret;
    int fd = video_fds[device];
    printf("startDevice fd = %d\n", fd);

    /* if (mDevices[device].state != DEVICE_PREPARED) */
    {
        ret = prepareDevice(device, buffer_count);
        if (ret < 0)
        {
            printf("prepareDevice failed\n");
            return ret;
        }
    }

    //Qbuf
    ret = activateBufferPool(device, buffer_count);
    if (ret < 0)
    {
        printf("activateBufferPool failed\n");
        return ret;
    }
    printf("queue buf sucess\n");

    //stream on
    ret = v4l2_stream_on(fd);
    if(ret < 0)
    {
        printf("stream on failed\n");
        return -1;
    }
    printf("stream on sucess\n");

    mDevices[device].frameCounter = 0;
    mDevices[device].state = DEVICE_STARTED;

    //we are started now
    return 0;
}

/*
 **************************************************************************
 * FunctionName: startPreview;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int startPreview()
{
    int ret = 0;
    ret = startDevice(V4L2_PREVIEW_DEVICE, NUM_PREVIEW_BUFFERS);
    if (ret < 0)
    {
        printf("Start preview device failed!\n");
        return -1;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: startCapture;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int startCapture()
{
    int ret = startDevice(V4L2_MAIN_DEVICE, NUM_SNAPSHOT_BUFFERS);
    if (ret < 0)
    {
        printf("start capture on first device failed!\n");
        return -1;
    }

    ret = startDevice(V4L2_POSTVIEW_DEVICE, NUM_POSTVIEW_BUFFERS);
    if (ret < 0)
    {
        printf("start capture on second device failed!\n");
        return -1;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: startContinuousPreview;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int startContinuousPreview()
{
    int status = 0;
    status = prepareDevice(V4L2_MAIN_DEVICE, NUM_SNAPSHOT_BUFFERS);
    if (status < 0)
    {
        printf("prepareDevice failed\n");
        return -1;
    }
    status = prepareDevice(V4L2_POSTVIEW_DEVICE, NUM_POSTVIEW_BUFFERS);
    if (status < 0)
    {
        printf("prepareDevice failed\n");
        return -1;
    }

    status = startPreview();
    if (status < 0)
    {
        printf("startPreview failed\n");
        return -1;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: startRecording;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int startRecording()
{
    int ret = 0;

    ret = startDevice(V4L2_MAIN_DEVICE, NUM_RECORDING_BUFFERS);
    if (ret < 0)
    {
        printf("Start recording device failed\n");
        return -1;
    }

    ret = startDevice(V4L2_PREVIEW_DEVICE, NUM_PREVIEW_BUFFERS);
    if (ret < 0)
    {
        printf("Start preview device failed!\n");
        return -1;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: start;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int start()
{
    int status = 0;

    switch (mMode)
    {
        case CI_MODE_PREVIEW:
            status = startPreview();
            break;
        case CI_MODE_VIDEO:
            status = startRecording();
            break;
        case CI_MODE_CAPTURE:
            status = startCapture();
            break;
        case CI_MODE_CONTINUOUS_CAPTURE:
            status = startContinuousPreview();
            break;
        default:
            status = -1;
            break;
    };


	if (vflip_enable)
		set_vflip(main_fd, vflip_enable);

	if (hflip_enable)
		set_hflip(main_fd, hflip_enable);

	if (flash_indicator)
		setFlashIndicator(indicator_intensity);

	if (flash_en)
		setFlash(1);

	if (flash_torch)
	{
		setTorch(torch_intensity);
		sleep(5);
		setTorch(0);
	}

	if (focus_en)
		focus_test();
    return status;
}

/*
 **************************************************************************
 * FunctionName: stopDevice;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int stopDevice(int device)
{
    int fd = video_fds[device];
	struct v4l2_requestbuffers req_buf;
	int ret;

    /* if (fd >= 0 && mDevices[device].state == DEVICE_STARTED) */
    {
        //stream off
        v4l2_stream_off(fd);
    }
    mDevices[device].state = DEVICE_CONFIGURED;

    CLEAR(req_buf);

    req_buf.memory = V4L2_MEMORY_MMAP;
    req_buf.count = 0;
    req_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    ret = ioctl(fd, VIDIOC_REQBUFS, &req_buf);
    if (ret < 0)
    {
        printf("VIDIOC_REQBUFS returned: %d (%s)\n", ret, strerror(errno));
        return ret;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: stopPreview;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int stopPreview()
{
    stopDevice(V4L2_PREVIEW_DEVICE);

    /* closeDevice(V4L2_PREVIEW_DEVICE); */
    mDevices[V4L2_PREVIEW_DEVICE].state = DEVICE_CLOSED;

    return 0;
}

/*
 **************************************************************************
 * FunctionName: stopCapture;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int  stopCapture()
{
    stopDevice(V4L2_POSTVIEW_DEVICE);
    stopDevice(V4L2_MAIN_DEVICE);

    return 0;
}

/*
 **************************************************************************
 * FunctionName: stopContinuousPreview;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int stopContinuousPreview()
{
    int error = 0;
    if (requestContCapture(0, 0, 0) != 0)
        ++error;
    if (stopCapture() != 0)
        ++error;
    if (stopPreview() != 0)
        ++error;
    if (error) {
        printf("@%s: errors (%d) in stopping continuous capture\n",
             __FUNCTION__, error);
        return -1;
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: stopRecording;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int stopRecording()
{
    stopDevice(V4L2_PREVIEW_DEVICE);

    stopDevice(V4L2_MAIN_DEVICE);

    closeDevice(V4L2_PREVIEW_DEVICE);
    mDevices[V4L2_PREVIEW_DEVICE].state = DEVICE_CLOSED;

   return 0;
}

/*
 **************************************************************************
 * FunctionName: stop;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int stop()
{
    int status = 0;
    switch (mMode)
    {
        case CI_MODE_PREVIEW:
            status = stopPreview();
            break;

        case CI_MODE_VIDEO:
            status = stopRecording();
            break;

        case CI_MODE_CAPTURE:
            status = stopCapture();
            break;

        case CI_MODE_CONTINUOUS_CAPTURE:
            status = stopContinuousPreview();
            break;

        default:
            break;
    };
	/* setFlash(0); */

    if (status == 0)
        mMode = CI_MODE_NONE;

    return status;
}

/*
 **************************************************************************
 * FunctionName: initCameraParams;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int initCameraParams()
{
    preview.width  = DEFAULT_PREVIEW_WIDTH;
    preview.height = DEFAULT_PREVIEW_HEIGHT;
    preview.format = DEFAULT_PREVIEW_FORMAT;
    preview.stride = DEFAULT_PREVIEW_WIDTH;
    preview.size   = frameSize(preview.format, preview.stride, preview.height);

    recording.width  = g_width;
    recording.height = g_height;
    recording.format = DEFAULT_RECORDING_FORMAT;
    recording.stride = g_width;
    recording.size   = frameSize(recording.format, recording.stride, recording.height);

    if(mCameraID == V4L2_SENSOR_PRIMARY)
    {
        snapshot.width =  g_width;
        snapshot.height = g_height;
        snapshot.format = DEFAULT_PRIMARY_SNAPSHOT_FORMAT;
        snapshot.stride = g_width;
        snapshot.size = frameSize(snapshot.format, snapshot.stride, snapshot.height);
    }
    else
    {
        snapshot.width =  g_width;
        snapshot.height = g_height;
        snapshot.format = DEFAULT_SECONDARY_SNAPSHOT_FORMAT;
        snapshot.stride = g_width;
        snapshot.size = frameSize(snapshot.format, snapshot.stride, snapshot.height);
    }

    postview.width =  DEFAULT_POSTVIEW_WIDTH;
    postview.height = DEFAULT_POSTVIEW_HEIGHT;
    postview.format = DEFAULT_POSTVIEW_FORMAT;
    postview.stride = DEFAULT_POSTVIEW_WIDTH;
    postview.size = frameSize(postview.format, postview.stride, postview.height);

    previewBuf = (struct camera_buffer *)malloc(sizeof(struct camera_buffer) * NUM_PREVIEW_BUFFERS);
    if(previewBuf == NULL)
    {
        printf("malloc preview buffer failed\n");
        return -1;
    }

    init_camera_buffers(previewBuf, BUFFER_TYPE_PREVIEW);

    snapshotBuf = (struct camera_buffer *)malloc(sizeof(struct camera_buffer) * NUM_SNAPSHOT_BUFFERS);
    if(snapshotBuf == NULL)
    {
        printf("malloc snapshot buffer failed\n");
        return -1;
    }

    init_camera_buffers(snapshotBuf, BUFFER_TYPE_SNAPSHOT);

    postviewBuf = (struct camera_buffer *)malloc(sizeof(struct camera_buffer) * NUM_POSTVIEW_BUFFERS);
    if(postviewBuf == NULL)
    {
        printf("malloc postview buffer failed\n");
        return -1;
    }

    init_camera_buffers(postviewBuf, BUFFER_TYPE_POSTVIEW);

    recordingBuf = (struct camera_buffer *)malloc(sizeof(struct camera_buffer) * NUM_RECORDING_BUFFERS);
    if(recordingBuf == NULL)
    {
        printf("malloc recording buffer failed\n");
        return -1;
    }

    init_camera_buffers(recordingBuf, BUFFER_TYPE_RECORDING);

    return 0;
}

/*
 **************************************************************************
 * FunctionName: deinitCameraParams;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int deinitCameraParams()
{
    memset(&preview, 0x00, sizeof(struct FrameInfo));
    memset(&postview, 0x00, sizeof(struct FrameInfo));
    memset(&snapshot, 0x00, sizeof(struct FrameInfo));
    memset(&recording, 0x00, sizeof(struct FrameInfo));

    deinit_camera_buffers(previewBuf, BUFFER_TYPE_PREVIEW);
    if(previewBuf != NULL)
    {
        free(previewBuf);
        previewBuf = NULL;
        /* closeDevice(V4L2_PREVIEW_DEVICE); */
    }

    deinit_camera_buffers(snapshotBuf, BUFFER_TYPE_SNAPSHOT);
    if(snapshotBuf != NULL)
    {
        free(snapshotBuf);
        snapshotBuf = NULL;
        /* closeDevice(V4L2_MAIN_DEVICE); */
    }

    deinit_camera_buffers(postviewBuf, BUFFER_TYPE_POSTVIEW);
    if(postviewBuf != NULL)
    {
        free(postviewBuf);
        postviewBuf = NULL;
        /* closeDevice(V4L2_POSTVIEW_DEVICE); */
    }

    deinit_camera_buffers(recordingBuf, BUFFER_TYPE_RECORDING);
    if(recordingBuf != NULL)
    {
        free(recordingBuf);
        recordingBuf = NULL;
        /* closeDevice(V4L2_MAIN_DEVICE); */
    }

    return 0;
}

/*
 **************************************************************************
 * FunctionName: preview_test;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int preview_test()
{
    int i = 0;
    int j = 0;
    int loop = 0;
    char filename[64];
    int preview_fps = 0;
    int update_time = 1;
    struct timeval start_time, stop_time, diff;

    printf("%s++\n",__func__);
    for(i = 0; i < V4L2_MAX_DEVICE_COUNT; i++)
    {
        video_fds[i] = -1;
        mDevices[i].state = DEVICE_CLOSED;
    }

    int ret = openDevice(V4L2_MAIN_DEVICE);
    if (ret < 0)
    {
        printf("Failed to open first device!\n");
        return -1;
    }

    selectCameraSensor();

    initCameraParams();

    ret = configure(CI_MODE_PREVIEW);
    if(ret < 0)
    {
        printf("configure preview failed\n");
        closeDevice(V4L2_MAIN_DEVICE);
        return -1;
    }

    start();
    printf("start preview OK\n");

    sprintf(filename, "%s_preview_%dx%d.yuv", mCameraID ? "secondCamera" : "primaryCamera", preview.width, preview.height);
    while(loop < NUM_OF_PREVIEW_FRAME)
    {
        if(update_time)
        {
            gettimeofday(&start_time,0);
            update_time = 0;
        }
        int index = v4l2_dequeue_buffer(video_fds[V4L2_PREVIEW_DEVICE]);
        if(index < 0 || index >=NUM_PREVIEW_BUFFERS)
        {
            printf("dequeue buffer failed\n");
            break;
        }

        loop++;
        preview_fps++;

        if (dump)
        {
            printf("get new buffer, index=%d\n", index);
            FILE * fp = fopen(filename, "ab+");
            if(fp)
            {
                for(j = 0; j < preview.height * 3/2; j++)
                {
                    fwrite((char *)(previewBuf[index].viraddr + preview.stride * j), 1, preview.width, fp);
                }
                fflush(fp);
                fclose(fp);
                fp = NULL;
            }
            else
            {
                printf("open file[%s] failed\n", filename);
            }
        }

        v4l2_queue_buffer(video_fds[V4L2_PREVIEW_DEVICE], index);
        gettimeofday(&stop_time,0);
        time_subtract(&diff,&start_time,&stop_time);

        if(diff.tv_sec == 1)
        {
            update_time = 1;
            printf("current fps = %d\n", preview_fps);
            preview_fps = 0;
        }
    }

    stop();
    printf("stop preview OK\n");

    closeDevice(V4L2_MAIN_DEVICE);
    closeDevice(V4L2_PREVIEW_DEVICE);
    mDevices[V4L2_MAIN_DEVICE].state = DEVICE_CLOSED;

    deinitCameraParams();
    printf("%s--\n\n",__func__);

    return 0;
}

/*
 **************************************************************************
 * FunctionName: preview_test2;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int preview_test2()
{
    int i = 0;
    int j = 0;
    int loop = 0;
    char filename[64];
    int preview_fps = 0;
    int update_time = 1;
    struct timeval start_time, stop_time, diff;

    printf("%s++\n",__func__);
    for(i = 0; i < V4L2_MAX_DEVICE_COUNT; i++)
    {
        video_fds[i] = -1;
        mDevices[i].state = DEVICE_CLOSED;
    }

    int ret = openDevice(V4L2_MAIN_DEVICE);
    if (ret < 0)
    {
        printf("Failed to open first device!\n");
        return -1;
    }

    selectCameraSensor();

    initCameraParams();

    ret = configure(CI_MODE_CONTINUOUS_CAPTURE);
    if(ret < 0)
    {
        printf("configure preview failed\n");
        closeDevice(V4L2_MAIN_DEVICE);
        return -1;
    }

    start();
    printf("start preview OK\n");

    sprintf(filename, "%s_preview2_%dx%d.yuv", mCameraID ? "secondCamera" : "primaryCamera", preview.width, preview.height);
    while(loop < NUM_OF_PREVIEW_FRAME)
    {
        if(update_time)
        {
            gettimeofday(&start_time,0);
            update_time = 0;
        }
        int index = v4l2_dequeue_buffer(video_fds[V4L2_PREVIEW_DEVICE]);
        if(index < 0 || index >=NUM_PREVIEW_BUFFERS)
        {
            printf("dequeue buffer failed\n");
            break;
        }

        loop++;
        preview_fps++;

        if (dump)
        {
            printf("get new buffer, index=%d\n", index);
            FILE * fp = fopen(filename, "ab+");
            if(fp)
            {
                for(j = 0; j < preview.height * 3/2; j++)
                {
                    fwrite((char *)(previewBuf[index].viraddr + preview.stride * j), 1, preview.width, fp);
                }
                fflush(fp);
                fclose(fp);
                fp = NULL;
            }
            else
            {
                printf("open file[%s] failed\n", filename);
            }
        }

        v4l2_queue_buffer(video_fds[V4L2_PREVIEW_DEVICE], index);
        gettimeofday(&stop_time,0);
        time_subtract(&diff,&start_time,&stop_time);

        if(diff.tv_sec == 1)
        {
            update_time = 1;
            printf("current fps = %d\n", preview_fps);
            preview_fps = 0;
        }
    }

    stop();
    printf("stop preview OK\n");

    deinitCameraParams();
	closeDevice(V4L2_MAIN_DEVICE);
    printf("%s--\n\n",__func__);

    return 0;
}

/*
 **************************************************************************
 * FunctionName: recording_test;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
static int recording_test()
{
    int i = 0;
    int j = 0;
    int loop = 0;
    char filename[64];
    int preview_fps = 0;
    int update_time = 1;
    int record_resolution_size;
    int preview_resolution_size;
    struct timeval start_time, stop_time, diff;

    printf("%s++\n",__func__);
    for(i = 0; i < V4L2_MAX_DEVICE_COUNT; i++)
    {
        video_fds[i] = -1;
        mDevices[i].state = DEVICE_CLOSED;
    }
    record_resolution_size = g_width * g_height;
    preview_resolution_size = DEFAULT_PREVIEW_WIDTH * DEFAULT_PREVIEW_HEIGHT;

    /* video record resolution must bigger then preview resolution */
    if (preview_resolution_size > record_resolution_size)
    {
        printf("preview resolution[%d*%d] biger then the record resolution[%d*%d],will be swap the device\n",DEFAULT_PREVIEW_WIDTH, DEFAULT_PREVIEW_HEIGHT, g_width, g_height);
        dev_name_array[V4L2_MAIN_DEVICE] = VIDEO2_PATH;
        dev_name_array[V4L2_PREVIEW_DEVICE] = VIDEO0_PATH;
        printf("V4L2_MAIN_DEVICE:%s\n", dev_name_array[V4L2_MAIN_DEVICE]);
        printf("V4L2_POSTVIEW_DEVICE:%s\n", dev_name_array[V4L2_POSTVIEW_DEVICE]);
        printf("V4L2_PREVIEW_DEVICE:%s\n", dev_name_array[V4L2_PREVIEW_DEVICE]);
        printf("V4L2_INJECT_DEVICE:%s\n", dev_name_array[V4L2_INJECT_DEVICE]);
        device_swap = 1;
    }

    int ret = openDevice(V4L2_MAIN_DEVICE);
    if (ret < 0)
    {
        printf("Failed to open first device!\n");
        return -1;
    }

    selectCameraSensor();

    initCameraParams();

    ret = configure(CI_MODE_VIDEO);
    if(ret < 0)
    {
        printf("configure preview failed\n");
        closeDevice(V4L2_MAIN_DEVICE);
        return -1;
    }

    start();
    printf("start recording OK\n");

    sprintf(filename, "%s_record_%dx%d.yuv", mCameraID ? "secondCamera" : "primaryCamera", recording.width, recording.height);
    while(loop < NUM_OF_PREVIEW_FRAME)
    {
        if(update_time)
        {
            gettimeofday(&start_time,0);
            update_time = 0;
        }

        int preview_index = v4l2_dequeue_buffer(video_fds[V4L2_PREVIEW_DEVICE]);
        if(preview_index < 0 || preview_index >=NUM_PREVIEW_BUFFERS)
        {
            printf("dequeue buffer failed\n");
            break;
        }

        int index = v4l2_dequeue_buffer(video_fds[V4L2_MAIN_DEVICE]);
        if(index < 0 || index >=NUM_PREVIEW_BUFFERS)
        {
            printf("dequeue buffer failed\n");
            break;
        }

        loop++;
        preview_fps++;

        if (dump)
        {
            printf("get new buffer, index=%d\n", index);
            FILE * fp = fopen(filename, "ab+");
            if(fp)
            {
                for(j = 0; j < recording.height * 3/2; j++)
                {
                    fwrite((char *)(recordingBuf[index].viraddr + recording.stride * j), 1, recording.width, fp);
                }
                fflush(fp);
                fclose(fp);
                fp = NULL;
            }
            else
            {
                printf("open file[%s] failed\n", filename);
            }
        }

        v4l2_queue_buffer(video_fds[V4L2_PREVIEW_DEVICE], preview_index);
        v4l2_queue_buffer(video_fds[V4L2_MAIN_DEVICE], index);
        gettimeofday(&stop_time,0);
        time_subtract(&diff,&start_time,&stop_time);

        if(diff.tv_sec == 1)
        {
            update_time = 1;
            printf("current fps = %d\n", preview_fps);
            preview_fps = 0;
        }
    }

    stop();
    printf("stop recording OK\n");

    closeDevice(V4L2_MAIN_DEVICE);
    mDevices[V4L2_MAIN_DEVICE].state = DEVICE_CLOSED;

    deinitCameraParams();
    printf("%s--\n\n",__func__);

    return 0;
}

/**
 * Waits for frame data to be available
 *
 * \param device V4L2 device id
 * \param timeout time to wait for data (in ms), timeout of -1
 *        means to wait indefinitely for data
 *
 * \return 0: timeout, -1: error happened, positive number: success
 */
/*
 **************************************************************************
 * FunctionName: v4l2_poll;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int v4l2_poll(int device, int timeout)
{
    struct pollfd pfd;

    if (video_fds[device] < 0) {
        printf("Device %d already closed. Do nothing.\n", device);
        return -1;
    }

    pfd.fd = video_fds[device];
    pfd.events = POLLIN | POLLERR;

    return poll(&pfd, 1, timeout);
}

/**
 * Polls the preview device node fd for data
 *
 * \param timeout time to wait for data (in ms), timeout of -1
 *        means to wait indefinitely for data
 * \return -1 for error, 0 if time out, positive number
 *         if poll was successful
 */
/*
 **************************************************************************
 * FunctionName: pollCapture;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int pollCapture(int timeout)
{
    return v4l2_poll(V4L2_MAIN_DEVICE, timeout);
}

/**
 * Polls the preview device node fd for data
 *
 * \param timeout time to wait for data (in ms), timeout of -1
 *        means to wait indefinitely for data
 * \return -1 for error, 0 if time out, positive number
 *         if poll was successful
 */
/*
 **************************************************************************
 * FunctionName: pollPreview;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int pollPreview(int timeout)
{
    return v4l2_poll(V4L2_PREVIEW_DEVICE, timeout);
}

/*
 **************************************************************************
 * FunctionName: waitForCaptureStart;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int waitForCaptureStart()
{
    int status = 0;
    const int maxWaitMs = 2000;
    const int previewTimeoutMs = 20;
    const int maxCycles = maxWaitMs / previewTimeoutMs;
    int n;

    for(n = 0; n < maxCycles; n++) {
        // Check if capture frame is availble (no wait)
        int res = pollCapture(0);
        if (res > 0)
            break;

        res = pollPreview(previewTimeoutMs);
        if (res < 0) {
            status = -1;
            break;
        }
        else if (res > 0) {
            //skip frames
        }
    }

    if (n == maxCycles)
        status = -1;

    return status;
}

void thread_exit(int signal)
{
    printf("thread will be quit\n");
    pthread_exit(NULL);
}

/*
 **************************************************************************
 * FunctionName: preview_thread;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
void *preview_thread()
{
    int loop = 0;
    int preview_fps = 0;
    int update_time = 1;
    struct timeval start_time, stop_time, diff;
    int signum;
	char filename[64];
	int j;
    printf("%s++\n",__func__);
    signal(SIGQUIT, thread_exit);

    printf("enter preview thread\n");
    sprintf(filename, "%s_continuous_preview_%dx%d.yuv", mCameraID ? "secondCamera" : "primaryCamera", preview.width, preview.height);
    while(1)
    {
        if(update_time)
        {
            gettimeofday(&start_time,0);
            update_time = 0;
        }
        int index = v4l2_dequeue_buffer(video_fds[V4L2_PREVIEW_DEVICE]);
		if (index < 0 || index >= NUM_PREVIEW_BUFFERS)
			printf("preview new buffer index=%d failed\n", index);

        loop++;
        preview_fps++;
        if (dump)
        {
            printf("get new buffer, index=%d\n", index);
            FILE * fp = fopen(filename, "ab+");
            if(fp)
            {
                for(j = 0; j < preview.height * 3/2; j++)
                {
                    fwrite((char *)(previewBuf[index].viraddr + preview.stride * j), 1, preview.width, fp);
                }
                fflush(fp);
                fclose(fp);
                fp = NULL;
            }
            else
            {
                printf("open file[%s] failed\n", filename);
            }
        }

        v4l2_queue_buffer(video_fds[V4L2_PREVIEW_DEVICE], index);
        gettimeofday(&stop_time,0);
        time_subtract(&diff,&start_time,&stop_time);

        if(diff.tv_sec == 1)
        {
            update_time = 1;
            printf("preview current fps = %d\n", preview_fps);
            preview_fps = 0;
        }
    }
    printf("%s--\n",__func__);

    return NULL;
}

int grab_all_frame(int frame_count)
{
    int j = 0;
    int loop = 0;
    int ret = 0;
    char filename[64];

	printf("%s++\n",__func__);

	sprintf(filename, "%s_snapshot_%dx%d.yuv", mCameraID ? "secondCamera" : "primaryCamera", snapshot.width, snapshot.height);

    while(loop < frame_count)
    {
        int preview_index = v4l2_dequeue_buffer(video_fds[V4L2_PREVIEW_DEVICE]);
        int snapshot_index = v4l2_dequeue_buffer(main_fd);
        int postview_index = v4l2_dequeue_buffer(video_fds[V4L2_POSTVIEW_DEVICE]);

        printf("get snapshot new buffer, index=%d\n", snapshot_index);
		if (dump)
		{
			FILE * fp = fopen(filename, "ab+");
			if(fp)
			{
				for(j = 0; j < snapshot.height * 3/2; j++)
				{
					fwrite((char *)(snapshotBuf[snapshot_index].viraddr + snapshot.stride * j), 1, snapshot.width, fp);
				}
				fflush(fp);
				fclose(fp);
				fp = NULL;
			}
			else
			{
				printf("open file[%s] failed\n", filename);
			}
		}

        v4l2_queue_buffer(video_fds[V4L2_PREVIEW_DEVICE], preview_index);
        /* v4l2_queue_buffer(main_fd, snapshot_index); */
        /* v4l2_queue_buffer(video_fds[V4L2_POSTVIEW_DEVICE], postview_index); */
        loop++;
    }
    printf("%s--\n\n",__func__);
	return 0;
}

int grab_preview_frame(int frame_count)
{
    int loop = 0;
    int preview_fps = 0;
    int update_time = 1;
    struct timeval start_time, stop_time, diff;
	char filename[64];
	int j = 0;
	int i = 0;
    printf("%s++\n",__func__);

	sprintf(filename, "%s_preview_%dx%d.yuv", mCameraID ? "secondCamera" : "primaryCamera", preview.width, preview.height);
    while(loop < frame_count)
    {
        if(update_time)
        {
            gettimeofday(&start_time,0);
            update_time = 0;
        }
        int index = v4l2_dequeue_buffer(video_fds[V4L2_PREVIEW_DEVICE]);
		printf("preview index=%d\n", index);
		if (dump)
		{
			FILE * fp = fopen(filename, "ab+");
			if(fp)
			{
				for(j = 0; j < preview.height * 3/2; j++)
				{
					fwrite((char *)(previewBuf[index].viraddr + preview.stride * j), 1, preview.width, fp);
				}
				fflush(fp);
				fclose(fp);
				fp = NULL;
			}
			else
			{
				printf("open file[%s] failed\n", filename);
			}
		}
        loop++;
        preview_fps++;

        v4l2_queue_buffer(video_fds[V4L2_PREVIEW_DEVICE], index);
        gettimeofday(&stop_time,0);
        time_subtract(&diff,&start_time,&stop_time);
        if(diff.tv_sec == 1)
        {
            update_time = 1;
            printf("preview current fps = %d\n", preview_fps);
            preview_fps = 0;
        }
    }
    printf("%s--\n\n",__func__);
	return 0;
}

/*
 **************************************************************************
 * FunctionName: grab_frame;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
void grab_frame(int device, int frame_count)
{
    int loop = 0;
    int preview_fps = 0;
    int update_time = 1;
	int fd = video_fds[device];
    struct timeval start_time, stop_time, diff;
    int signum;
	char filename[64]={0};
	int j;
    printf("%s++\n",__func__);

    sprintf(filename, "%s_grab_preview_%dx%d.yuv", mCameraID ? "secondCamera" : "primaryCamera", preview.width, preview.height);
    while(loop < frame_count)
    {
        if(update_time)
        {
            gettimeofday(&start_time,0);
            update_time = 0;
        }
        int main_index = v4l2_dequeue_buffer(main_fd);
        int index = v4l2_dequeue_buffer(fd);

        loop++;
        preview_fps++;
        if (dump)
        {
            printf("get new buffer, index=%d\n", index);
            FILE * fp = fopen(filename, "ab+");
            if(fp)
            {
                for(j = 0; j < preview.height * 3/2; j++)
                {
                    fwrite((char *)(previewBuf[index].viraddr + preview.stride * j), 1, preview.width, fp);
                }
                fflush(fp);
                fclose(fp);
                fp = NULL;
            }
            else
            {
                printf("open file[%s] failed\n", filename);
            }
        }

        v4l2_queue_buffer(fd, index);
        v4l2_queue_buffer(main_fd, main_index);
        gettimeofday(&stop_time,0);
        time_subtract(&diff,&start_time,&stop_time);

        if(diff.tv_sec == 1)
        {
            update_time = 1;
            printf("current fps = %d\n", preview_fps);
            preview_fps = 0;
        }
    }
    printf("%s--\n",__func__);
}

int grab_snapshotframe(int frame_count)
{
    int j = 0;
    int loop = 0;
    int ret = 0;
    char filename[64];

	printf("%s++\n",__func__);

	sprintf(filename, "%s_snapshot_%dx%d.yuv", mCameraID ? "secondCamera" : "primaryCamera", snapshot.width, snapshot.height);

    while(loop < frame_count)
    {
        int snapshot_index = v4l2_dequeue_buffer(main_fd);
        int postview_index = v4l2_dequeue_buffer(video_fds[V4L2_POSTVIEW_DEVICE]);

        printf("get snapshot new buffer, index=%d\n", snapshot_index);
		if (dump)
		{
			FILE * fp = fopen(filename, "ab+");
			if(fp)
			{
				for(j = 0; j < snapshot.height * 3/2; j++)
				{
					fwrite((char *)(snapshotBuf[snapshot_index].viraddr + snapshot.stride * j), 1, snapshot.width, fp);
				}
				fflush(fp);
				fclose(fp);
				fp = NULL;
			}
			else
			{
				printf("open file[%s] failed\n", filename);
			}
		}

        /* v4l2_queue_buffer(main_fd, snapshot_index); */
        /* v4l2_queue_buffer(video_fds[V4L2_POSTVIEW_DEVICE], postview_index); */
        loop++;
    }
    printf("%s--\n\n",__func__);
	return 0;
}

/*
 **************************************************************************
 * FunctionName: snapshot_thread;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
void *snapshot_thread()
{
    int j = 0;
    int loop = 0;
    int ret = 0;
    char filename[64];

	printf("%s++\n",__func__);
    printf("enter snapshot thread\n");
    requestContCapture(DEFAULT_CAPTURE_COUNT, -1, 0);
    startCapture();


    ret = waitForCaptureStart();
    if(ret < 0)
    {
        printf("waitForCaptureStart failed\n");
        return NULL;
    }

    printf("start snapshot OK\n");

    loop = 0;
	sprintf(filename, "%s_snapshot_%dx%d.yuv", mCameraID ? "secondCamera" : "primaryCamera", snapshot.width, snapshot.height);

    while(loop < DEFAULT_CAPTURE_COUNT)
    {
        int snapshot_index = v4l2_dequeue_buffer(main_fd);
        int postview_index = v4l2_dequeue_buffer(video_fds[V4L2_POSTVIEW_DEVICE]);
		if (dump)
		{
        printf("get snapshot new buffer, index=%d\n", snapshot_index);
        FILE * fp = fopen(filename, "ab+");
        if(fp)
        {
            for(j = 0; j < snapshot.height * 3/2; j++)
            {
                fwrite((char *)(snapshotBuf[snapshot_index].viraddr + snapshot.stride * j), 1, snapshot.width, fp);
            }
            fflush(fp);
            fclose(fp);
            fp = NULL;
        }
        else
        {
            printf("open file[%s] failed\n", filename);
        }

		}

		loop++;
		v4l2_queue_buffer(main_fd, snapshot_index);
		v4l2_queue_buffer(video_fds[V4L2_POSTVIEW_DEVICE], postview_index);
    }
    pthread_kill(preview_thread_id, SIGQUIT);
    printf("%s--\n",__func__);

    return NULL;
}

int continuous_grab_snapshot(int preview_count, int snapshot_count)
{
	int ret;
	struct timeval start, end, diff;
	unsigned long delt;
	printf("%s++\n", __func__);
	/* stopCapture(); */

    requestContCapture(-1, 0, 0);
#if 1
	gettimeofday(&start, NULL);
	v4l2_stream_off(video_fds[V4L2_MAIN_DEVICE]);
    deinit_camera_buffers(snapshotBuf, BUFFER_TYPE_SNAPSHOT);
	releaseBuffer(video_fds[V4L2_MAIN_DEVICE]);

	v4l2_stream_off(video_fds[V4L2_POSTVIEW_DEVICE]);
    deinit_camera_buffers(postviewBuf, BUFFER_TYPE_POSTVIEW);
	releaseBuffer(video_fds[V4L2_POSTVIEW_DEVICE]);
	gettimeofday(&end, NULL);
	time_subtract(&diff, &start, &end);
	delt = (unsigned long)(diff.tv_sec * 1000000 + diff.tv_usec);
	printf("release buf spare time=%u\n", delt);

#endif
	grab_preview_frame(10);
	/* sleep(2); */
#if 1
	gettimeofday(&start, NULL);
    /* requestContCapture(1, -1, 0); */
    int status = prepareDevice(V4L2_MAIN_DEVICE, NUM_SNAPSHOT_BUFFERS);
    if (status < 0)
    {
        printf("prepareDevice  main device failed\n");
        return -1;
    }
	gettimeofday(&end, NULL);
	time_subtract(&diff, &start, &end);
	delt = (unsigned long)(diff.tv_sec * 1000000 + diff.tv_usec);
	printf("request buf spare time=%u\n", delt);
#endif

    //Qbuf
    ret = activateBufferPool(V4L2_MAIN_DEVICE, NUM_SNAPSHOT_BUFFERS);
    if (ret < 0)
    {
        printf("activateBufferPool main failed\n");
        return ret;
    }
    //stream on
    ret = v4l2_stream_on(video_fds[V4L2_MAIN_DEVICE]);
    if(ret < 0)
    {
        printf("stream on main device failed\n");
        return -1;
    }

    status = prepareDevice(V4L2_POSTVIEW_DEVICE, NUM_POSTVIEW_BUFFERS);
    if (status < 0)
    {
        printf("prepareDevice failed\n");
        return -1;
    }
    ret = activateBufferPool(V4L2_POSTVIEW_DEVICE, NUM_POSTVIEW_BUFFERS);
    if (ret < 0)
    {
        printf("activateBufferPool main failed\n");
        return ret;
    }

    ret = v4l2_stream_on(video_fds[V4L2_POSTVIEW_DEVICE]);
    if(ret < 0)
    {
        printf("stream on postview failed\n");
        return -1;
    }
	/* grab_preview_frame(10); */
	gettimeofday(&start, NULL);
#if 1
	ret = pollCapture(2000);
    if(ret < 0)
    {
        printf("waitForCaptureStart failed\n");
		return -1;
    }
#endif
	/* grab_preview_frame(1); */
	grab_snapshotframe(1);
	gettimeofday(&end, NULL);
	time_subtract(&diff, &start, &end);
	delt = (unsigned long)(diff.tv_sec * 1000000 + diff.tv_usec);
	printf("grab still frame spare time=%u\n", delt);
	printf("%s--\n", __func__);
	return 0;
}

int primary_continuous_capture()
{
    int i = 0;
    void *snapshot_ret;
	int ret;
    int status = 0;

    printf("%s++\n",__func__);
    for(i = 0; i < V4L2_MAX_DEVICE_COUNT; i++)
    {
        video_fds[i] = -1;
        mDevices[i].state = DEVICE_CLOSED;
    }

    initCameraParams();
    ret = openDevice(V4L2_MAIN_DEVICE);
    if (ret < 0)
    {
        printf("Failed to open first device!\n");
        return -1;
    }
    selectCameraSensor();
    requestContCapture(DEFAULT_CAPTURE_COUNT, -1, 0);
    updateCameraParams();
    ret = configureDevice(V4L2_MAIN_DEVICE, CI_MODE_PREVIEW, &snapshot, 0);
    if (ret < 0)
    {
        printf("configure first device failed!\n");
        return -1;
    }
    ret = openDevice(V4L2_PREVIEW_DEVICE);
    if (ret < 0)
    {
        printf("Open preview device failed!\n");
        return -1;
    }
    ret = configureDevice(V4L2_PREVIEW_DEVICE,CI_MODE_PREVIEW,&preview, 0);
    if(ret < 0)
    {
        printf("configureDevice failed\n");
    }

    ret = openDevice(V4L2_POSTVIEW_DEVICE);
    if (ret < 0)
    {
        printf("Open second device failed!");
        return -1;
    }
    ret = configureDevice(V4L2_POSTVIEW_DEVICE, CI_MODE_PREVIEW, &postview, 0);
    if (ret < 0)
    {
        printf("configure second device failed!");
        closeDevice(V4L2_POSTVIEW_DEVICE);
        return ret;
    }
	ret = prepareDevice(V4L2_PREVIEW_DEVICE, NUM_PREVIEW_BUFFERS);
	if (ret < 0)
	{
		printf("prepareDevice previw device failed\n");
		return ret;
	}

    ret = activateBufferPool(V4L2_PREVIEW_DEVICE, NUM_PREVIEW_BUFFERS);
    if (ret < 0)
    {
        printf("activateBufferPool failed\n");
        return ret;
    }
    printf("queue buf sucess\n");

    //stream on video2
    ret = v4l2_stream_on(video_fds[V4L2_PREVIEW_DEVICE]);
    if(ret < 0)
    {
        printf("stream on failed\n");
        return -1;
    }

	grab_preview_frame(10);
    /* sleep(2); */

    requestContCapture(1, -1, 0);
    status = prepareDevice(V4L2_MAIN_DEVICE, NUM_SNAPSHOT_BUFFERS);
    if (status < 0)
    {
        printf("prepareDevice  main device failed\n");
        return -1;
    }
    //Qbuf
    ret = activateBufferPool(V4L2_MAIN_DEVICE, NUM_SNAPSHOT_BUFFERS);
    if (ret < 0)
    {
        printf("activateBufferPool main failed\n");
        return ret;
    }
    //stream on
    ret = v4l2_stream_on(video_fds[V4L2_MAIN_DEVICE]);
    if(ret < 0)
    {
        printf("stream on main device failed\n");
        return -1;
    }
    status = prepareDevice(V4L2_POSTVIEW_DEVICE, NUM_POSTVIEW_BUFFERS);
    if (status < 0)
    {
        printf("prepareDevice failed\n");
        return -1;
    }
    ret = activateBufferPool(V4L2_POSTVIEW_DEVICE, NUM_POSTVIEW_BUFFERS);
    if (ret < 0)
    {
        printf("activateBufferPool main failed\n");
        return ret;
    }
    ret = v4l2_stream_on(video_fds[V4L2_POSTVIEW_DEVICE]);
    if(ret < 0)
    {
        printf("stream on postview failed\n");
        return -1;
    }

	grab_snapshotframe(1);

	for (i = 0; i < 10; i++) {
		continuous_grab_snapshot(10, DEFAULT_CAPTURE_COUNT);
	}
    stop();
    printf("stop OK\n");

    closeDevice(V4L2_MAIN_DEVICE);
    mDevices[V4L2_MAIN_DEVICE].state = DEVICE_CLOSED;

    deinitCameraParams();
    printf("%s--\n",__func__);

    return 0;
}

int primary_still_capture()
{
    int i = 0;
	int ret;
    int status = 0;

    printf("%s++\n",__func__);
    for(i = 0; i < V4L2_MAX_DEVICE_COUNT; i++)
    {
        video_fds[i] = -1;
        mDevices[i].state = DEVICE_CLOSED;
    }

    initCameraParams();
    ret = openDevice(V4L2_MAIN_DEVICE);
    if (ret < 0)
    {
        printf("Failed to open first device!\n");
        return -1;
    }
    selectCameraSensor();
    updateCameraParams();

	configure(CI_MODE_CONTINUOUS_CAPTURE);
	start();
	grab_preview_frame(10);
	stop();
	deinitCameraParams();
	/* closeDevice(V4L2_POSTVIEW_DEVICE); */
	closeDevice(V4L2_PREVIEW_DEVICE);

	initCameraParams();
	configure(CI_MODE_CAPTURE);
	start();
	grab_snapshotframe(4);
	stop();
	deinitCameraParams();
	closeDevice(V4L2_POSTVIEW_DEVICE);
	/* closeDevice(V4L2_PREVIEW_DEVICE); */

	initCameraParams();
	configure(CI_MODE_CONTINUOUS_CAPTURE);
	start();
	grab_preview_frame(20);
    stop();
    deinitCameraParams();
    printf("stop OK\n");

    closeDevice(V4L2_POSTVIEW_DEVICE);
    closeDevice(V4L2_PREVIEW_DEVICE);
    closeDevice(V4L2_MAIN_DEVICE);
    mDevices[V4L2_MAIN_DEVICE].state = DEVICE_CLOSED;

    printf("%s--\n",__func__);

    return 0;
}

/*
 **************************************************************************
 * FunctionName: primary_snapshot_test;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int primary_snapshot_test()
{
    int i = 0;
    void *snapshot_ret;

    printf("%s++\n",__func__);
    for(i = 0; i < V4L2_MAX_DEVICE_COUNT; i++)
    {
        video_fds[i] = -1;
        mDevices[i].state = DEVICE_CLOSED;
    }

    int ret = openDevice(V4L2_MAIN_DEVICE);
    if (ret < 0)
    {
        printf("Failed to open first device!\n");
        return -1;
    }

    selectCameraSensor();

    initCameraParams();

    ret = configure(CI_MODE_CONTINUOUS_CAPTURE);
    if(ret < 0)
    {
        printf("configure preview failed\n");
        closeDevice(V4L2_MAIN_DEVICE);
        return -1;
    }
    start();
    printf("start preview OK\n");

    pthread_create(&preview_thread_id, NULL, &preview_thread, NULL);

    sleep(2);

    pthread_create(&snapshot_thread_id, NULL, &snapshot_thread, NULL);

    ret = pthread_join(preview_thread_id, &snapshot_ret);
    if (ret != 0)
        printf("snapshot thread join failed\n");

    stop();
    printf("stop preview OK\n");

    deinitCameraParams();
    printf("%s--\n\n",__func__);

    return 0;
}

int secondary_snapshot_test()
{
    int i = 0;
    int j = 0;
    int loop = 0;
    char filename[64];
    int preview_fps = 0;
    int record_resolution_size;
    int preview_resolution_size;

    printf("%s++\n",__func__);

    for(i = 0; i < V4L2_MAX_DEVICE_COUNT; i++)
    {
        video_fds[i] = -1;
        mDevices[i].state = DEVICE_CLOSED;
    }

    record_resolution_size = g_width * g_height;
    preview_resolution_size = DEFAULT_PREVIEW_WIDTH * DEFAULT_PREVIEW_HEIGHT;

    if (preview_resolution_size > record_resolution_size)
    {
        printf("preview resolution[%d*%d] biger then the record resolution[%d*%d],will be swap the device\n",DEFAULT_PREVIEW_WIDTH, DEFAULT_PREVIEW_HEIGHT, g_width, g_height);
        dev_name_array[V4L2_MAIN_DEVICE] = VIDEO2_PATH;
        dev_name_array[V4L2_PREVIEW_DEVICE] = VIDEO0_PATH;
        printf("V4L2_MAIN_DEVICE:%s\n", dev_name_array[V4L2_MAIN_DEVICE]);
        printf("V4L2_POSTVIEW_DEVICE:%s\n", dev_name_array[V4L2_POSTVIEW_DEVICE]);
        printf("V4L2_PREVIEW_DEVICE:%s\n", dev_name_array[V4L2_PREVIEW_DEVICE]);
        printf("V4L2_INJECT_DEVICE:%s\n", dev_name_array[V4L2_INJECT_DEVICE]);
        device_swap = 1;
    }
    int ret = openDevice(V4L2_MAIN_DEVICE);
    if (ret < 0)
    {
        printf("Failed to open first device!\n");
        return -1;
    }

    selectCameraSensor();

    initCameraParams();

    /* second camare use video record data to snapshot picture */
    ret = configure(CI_MODE_VIDEO);
    if(ret < 0)
    {
        printf("configure preview failed\n");
        closeDevice(V4L2_MAIN_DEVICE);
        return -1;
    }

    start();
    printf("start snapshot OK\n");

    snapshotBuf = recordingBuf;
    memset((void *)(&snapshot), 0, sizeof(struct FrameInfo));
    memcpy((void *)(&snapshot), (void *)(&recording), sizeof(struct FrameInfo));

    sprintf(filename, "%s_snapshot_%dx%d.yuv", mCameraID ? "secondCamera" : "primaryCamera", snapshot.width, snapshot.height);
    while(loop < SKIP_FRAMES_OF_SECONDARY)
    {
        int preview_index = v4l2_dequeue_buffer(video_fds[V4L2_PREVIEW_DEVICE]);
        if(preview_index < 0 || preview_index >=NUM_PREVIEW_BUFFERS)
        {
            printf("dequeue preview buffer failed\n");
            break;
        }

        int index = v4l2_dequeue_buffer(video_fds[V4L2_MAIN_DEVICE]);

        loop++;
        preview_fps++;

        printf("get new buffer, index=%d\n", index);
        FILE * fp = fopen(filename, "w");
        if(fp)
        {
            for(j = 0; j < snapshot.height * 3/2; j++)
            {
                fwrite((char *)(snapshotBuf[index].viraddr + snapshot.stride * j), 1, snapshot.width, fp);
            }
            fflush(fp);
            fclose(fp);
            fp = NULL;
        }
        else
        {
            printf("open file[%s] failed\n", filename);
        }

        v4l2_queue_buffer(video_fds[V4L2_PREVIEW_DEVICE], preview_index);
        v4l2_queue_buffer(video_fds[V4L2_MAIN_DEVICE], index);
    }
    /* snapshotBuf must set null,or else recordingBuf will be free two times */
    snapshotBuf = NULL;

    stop();
    printf("stop snapshot OK\n");

    closeDevice(V4L2_MAIN_DEVICE);
    mDevices[V4L2_MAIN_DEVICE].state = DEVICE_CLOSED;

    deinitCameraParams();
    printf("%s--\n\n",__func__);

    return 0;
}

/*
 **************************************************************************
 * FunctionName: snapshot_test;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int snapshot_test()
{
    if(mCameraID == V4L2_SENSOR_PRIMARY)
    {
        return primary_snapshot_test();
    }
    else
    {
        return secondary_snapshot_test();
    }
}

void usage(int argc, char * argv[])
{
    printf("Usage: %s [option] \n",argv[0]);
    printf("-c, --capture \n\tcontinous still capture test\n");
    printf("-r, --record \n\trecord test\n");
    printf("-p, --preview \n\tpreiew test\n");
    printf("-d, --dump \n\tdump file when viedo record\n");
    printf("-i, --camera_id \n\tchoise which camera will be test\n");
    printf("\t0: the primary camera will be use\n");
    printf("\t1: the second camera will be use\n");
    printf("-f, --flash_en\n\tenable flash\n");
    printf("-I, --flash_indicator\n\tenable flash indicator\n");
    printf("-t, --flash_torch\n\tenable flash torch\n");
    printf("-F, --focus\n\tenable focus test\n");
    printf("-w, --width \n\tset width of the resolution\n");
    printf("-h, --height \n\tset height of the resolution\n");
}

/*
 **************************************************************************
 * FunctionName: main;
 * Description : NA;
 * Input       : NA;
 * Output      : NA;
 * ReturnValue : NA;
 * Other       : NA;
 **************************************************************************
 */
int main(int argc, char **argv)
{
    int width, height;
    int option_index = 0;
    int c;
    TEST_TYPE test = UNKNOWN_TEST;

    char * const short_options = "?crpdfVHI:t:Fi:w:h:p:";
    static struct option long_options[] = {
        {"capture", no_argument, 0, 'c'},
        {"record" , no_argument, 0, 'r'},
        {"preview", no_argument, 0, 'p'},
        {"camera_id",  required_argument, 0, 'i'},
        {"width",  optional_argument, 0, 'w'},
        {"height",  optional_argument, 0, 'h'},
        {"flash_en", no_argument, 0, 'f'},
        {"flash_torch", optional_argument, 0, 't'},
        {"flash_indicator", optional_argument, 0, 'I'},
        {"focus", no_argument, 0, 'F'},
        {"vflip", no_argument, 0, 'V'},
        {"hflip", no_argument, 0, 'H'},
        {0,                   0, 0, 0}
    };

    if (argc == 1)
    {
        usage(argc, argv);
        return -1;
    }

    while((c = getopt_long(argc, argv, short_options, long_options, &option_index)) != -1)
    {
        switch (c) {
        case '?':
            usage(argc, argv);
            break;

        case 'c':
            printf("capture test");
            test = CAPTURE_TEST;
            break;

        case 'r':
            printf("video record test");
            test = VIDEO_RECORD_TEST;
            break;

        case 'p':
            printf("preview test");
            test = PREVIEW_TEST;
            break;

		case 'f':
			printf("enable flash ");
			flash_en = 1;
			break;

		case 't':
			printf("enable torch flash test ");
			flash_torch = 1;
			if (atoi(optarg))
			{
				torch_intensity = atoi(optarg);
				printf("torch flash intensity is %d", torch_intensity);
			}
			break;

		case 'F':
			printf("enable focus test ");
			focus_en = 1;
			break;

		case 'V':
			printf("enable vflip");
			vflip_enable = 1;
			break;

		case 'H':
			printf("enable hflip");
			hflip_enable = 1;
			break;

		case 'I':
			printf("enable indicator flash test\n");
			flash_indicator = 1;
			if (atoi(optarg))
			{
				indicator_intensity = atoi(optarg);
				printf("indicator flash intensity is %d ", indicator_intensity);
			}
			break;

        case 'd':
            printf(" ,dump file enable,");
            dump = 1;
            break;

        case 'i':
            if (atoi(optarg))
                mCameraID = V4L2_SENSOR_SECONDARY;
            else
                mCameraID = V4L2_SENSOR_PRIMARY;
            printf("%s camera ", mCameraID ? "second" : "primary");
            break;

        case 'w':
            g_width = atoi(optarg);
            printf("\t width=%d", g_width);
            break;

        case 'h':
            g_height = atoi(optarg);
            printf("\t height=%d\n",g_height);
            break;

        default:
            printf("default resolution set for test\n");
           break;
        }				/* -----  end switch  ----- */
    }
    printf("\n");


	switch(test) {
    case CAPTURE_TEST:
		if(mCameraID == V4L2_SENSOR_PRIMARY)
		{
			/* snapshot_test(); */
			/* primary_still_capture(); */
			primary_continuous_capture();
		}
		else
		{
			secondary_snapshot_test();
		}
        break;

    case VIDEO_RECORD_TEST:
        recording_test();
        break;

    case PREVIEW_TEST:
        preview_test();
        if(mCameraID == V4L2_SENSOR_PRIMARY)
        {
            g_width =  DEFAULT_PRIMARY_SNAPSHOT_WIDTH;
            g_height = DEFAULT_PRIMARY_SNAPSHOT_HEIGHT;
            preview_test2();
        }
        break;

    }				/* -----  end switch  ----- */

    return 0;
}
/********************************** END **********************************************/

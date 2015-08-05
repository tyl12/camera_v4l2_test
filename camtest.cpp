/*
 *  V4L2 video capture example
 *
 *  This program can be used and distributed without restrictions.
 *
 *      This program is provided with the V4L2 API
 * see http://linuxtv.org/docs.php for more information
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

#include <getopt.h>             /* getopt_long() */

#include <fcntl.h>              /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#include <linux/videodev2.h>

#include <hardware/hardware.h>
#include <hardware/gralloc.h>
#include "../../../hardware/intel/libgralloc/gralloc_priv.h"
#include "../../../hardware/intel/libgralloc/gr.h"


#define CLEAR(x) memset(&(x), 0, sizeof(x))

enum io_method {
    IO_METHOD_READ,
    IO_METHOD_MMAP,
    IO_METHOD_USERPTR,
};

gralloc_module_t const* m_gralloc_module;
struct alloc_device_t *m_alloc_dev;

struct CamBuffer {
    int     width;
    int     height;
    int     format;
    void*   start;
    size_t  length;
    int     stride;
    buffer_handle_t handle;
};

struct CamNode {
    int fd;
    const char* name;
};

#define NAME_OVERLAY "/dev/video0"
#define NAME_CAPTURE "/dev/video2"
#define NAME_CONTROL "/dev/video1"

#define INDEX_OVERLAY 0
#define INDEX_CAPTURE 2
#define INDEX_CONTROL 1

static char*            dev_name;
static const char*      dev_name_list[] = {NAME_OVERLAY, NAME_CONTROL, NAME_CAPTURE};
static const int        dev_num = sizeof(dev_name_list)/sizeof(dev_name_list[0]);
static CamNode          node_list[dev_num];

#define BUFLEN_OVERLAY (8)
#define BUFLEN_CAPTURE (8)
static CamBuffer buffer_overlay[BUFLEN_OVERLAY];
static CamBuffer buffer_capture[BUFLEN_CAPTURE];

static enum io_method   flag_io = IO_METHOD_USERPTR;
static int              input_device;
static int              flag_dump=0;
static unsigned long    frame_count = 500;

static int FMT_LIST[]={V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_NV21, V4L2_PIX_FMT_YUV420, V4L2_PIX_FMT_UYVY};

#define width_overlay 640
#define height_overlay 480
//#define format_overlay_v4l2 V4L2_PIX_FMT_NV12
#define format_overlay_v4l2 FMT_LIST[0]

#define width_capture 640
#define height_capture 480
//#define format_capture_v4l2 V4L2_PIX_FMT_NV12
#define format_capture_v4l2 FMT_LIST[0]

#define DUMP_PATH "/data/media/"
#define DUMP_FILE_OVERLAY (DUMP_PATH "file_overlay.yuv")
/*********************************************************/


static int convert_fmt_v4l2_to_hal(int v4l2fmt)
{/*{{{*/
    int halfmt=-1;
    switch (v4l2fmt){
        case V4L2_PIX_FMT_NV12:
            //8-bit Y plane followed by an interleaved U/V plane with 2x2 subsampling
            halfmt = HAL_PIXEL_FORMAT_YCbCr_420_SP;
            break;
        case V4L2_PIX_FMT_NV21:
            halfmt = HAL_PIXEL_FORMAT_YCbCr_420_SP; //hack. as hal format is only used for bufferallocation, simply choose the format with same buffer layout(but different sequence)
            break;
        case V4L2_PIX_FMT_YUV420: //Y, U, V
            halfmt = HAL_PIXEL_FORMAT_YV12;
            break;
        case V4L2_PIX_FMT_YUYV: //YUY2 & YUYV
            halfmt = HAL_PIXEL_FORMAT_YCbCr_422_I;
            break;
        case V4L2_PIX_FMT_UYVY:
            halfmt = HAL_PIXEL_FORMAT_YCbCr_422_I; //hack
            break;
        case V4L2_PIX_FMT_JPEG:
            halfmt = HAL_PIXEL_FORMAT_BLOB;
            break;
        case V4L2_PIX_FMT_RGB565:
            halfmt = HAL_PIXEL_FORMAT_RGB_565;
            break;
        case V4L2_PIX_FMT_RGB32:
            halfmt = HAL_PIXEL_FORMAT_RGBA_8888;
            break;
        default:
            fprintf(stderr, "%s: unrecognized format %d\n",__FUNCTION__, v4l2fmt);
            break;
    }
    return halfmt;
}/*}}}*/

static void errno_exit(const char *s)
{
    fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
    exit(EXIT_FAILURE);
}

static int xioctl(int fh, int request, void *arg)
{
    int r;

    do {
        r = ioctl(fh, request, arg);
    } while (-1 == r && EINTR == errno);

    return r;
}

static int get_bits_per_pixel(unsigned int fmt)
{/*{{{*/
    int bpp = 0;

    switch (fmt) {
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV21:
        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_YVU420:
            bpp = 12;
            break;

        case V4L2_PIX_FMT_RGB565:
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_YVYU:
        case V4L2_PIX_FMT_UYVY:
        case V4L2_PIX_FMT_VYUY:
        case V4L2_PIX_FMT_NV16:
        case V4L2_PIX_FMT_NV61:
        case V4L2_PIX_FMT_YUV422P:
            bpp = 16;
            break;

        case V4L2_PIX_FMT_RGB32:
            bpp = 32;
            break;
        case V4L2_PIX_FMT_JPEG:
            bpp = 16;
            break;
    }

    return bpp;

}/*}}}*/

static unsigned int get_bytes_per_line(unsigned int fmt, unsigned int width)
{/*{{{*/
    int bpl = 0;

    switch (fmt) {
        case V4L2_PIX_FMT_NV12:
        case V4L2_PIX_FMT_NV21:
        case V4L2_PIX_FMT_YUV420:
        case V4L2_PIX_FMT_YVU420:
        case V4L2_PIX_FMT_NV16:
        case V4L2_PIX_FMT_NV61:
        case V4L2_PIX_FMT_YUV422P:
            /* For YcbCr (semi)planar formats the v4l2 manual says that */
            /* the bytes per line refers to the biggest plane; in this case */
            /* the Y plane, which has 1 byte per pixel */
            bpl = width;
            break;

        case V4L2_PIX_FMT_RGB565:
        case V4L2_PIX_FMT_YUYV:
        case V4L2_PIX_FMT_YVYU:
        case V4L2_PIX_FMT_UYVY:
        case V4L2_PIX_FMT_VYUY:
            /* 2 bytes per pixel */
            bpl = width * 2;
            break;

        case V4L2_PIX_FMT_RGB32:
            /* 4 bytes per pixel */
            bpl = width * 4;
            break;
        case V4L2_PIX_FMT_JPEG:
            /* not used */
            bpl = 0;
            break;
    }

    return bpl;
}/*}}}*/

int init_allocator(){
    fprintf(stderr, "%s enter\n",__FUNCTION__);
    hw_get_module(GRALLOC_HARDWARE_MODULE_ID, (const hw_module_t **)&m_gralloc_module);
    if (m_gralloc_module != NULL) {
        int ret = gralloc_open((const struct hw_module_t*)m_gralloc_module, &m_alloc_dev);

        if (m_alloc_dev == NULL) {
            fprintf(stderr,"%s: can't open gralloc device\n", __FUNCTION__);
            return -1;
        }
    }
    return 0;
}

int free_allocator(){
    gralloc_close(m_alloc_dev);
    return 0;
}

int _allocate(int width, int height, int halPixFmt, buffer_handle_t& buffHandle, int& stride)
{/*{{{*/
    int grallocFlags = 0;
    int ret = 0;
    grallocFlags |= GRALLOC_USAGE_HW_CAMERA_WRITE | GRALLOC_USAGE_HW_CAMERA_READ;

    //allocate buffer for overlay node
    ret = m_alloc_dev->alloc(m_alloc_dev,
            width,
            height,
            halPixFmt,
            grallocFlags,
            (buffer_handle_t*)&buffHandle,
            &stride);

    if (ret < 0) {
        fprintf(stderr, "%s: gralloc buffer allocation failed (error %d)\n", __FUNCTION__, ret);
        return -1;
    }
    m_gralloc_module->lock(m_gralloc_module, buffHandle, GRALLOC_USAGE_HW_CAMERA_WRITE & GRALLOC_USAGE_HW_VIDEO_ENCODER,
            0, 0, width, height, NULL);
    return 0;
}/*}}}*/

int allocate_buffer()
{/*{{{*/
    fprintf(stderr, "%s enter\n",__FUNCTION__);
    for (int i=0; i< BUFLEN_OVERLAY; i++){
        buffer_handle_t handle;
        int stride;
        int ret;
        int w = width_overlay;
        int h = height_overlay;
        int f = convert_fmt_v4l2_to_hal(format_overlay_v4l2);

        ret = _allocate(w, h, f, handle, stride);
        if (ret){
            fprintf(stderr, "%s:%d: failed to allocate buffer\n",__FUNCTION__,__LINE__);
            errno_exit(__FUNCTION__);
        }
        CamBuffer& buf = buffer_overlay[i];
        buf.width = w;
        buf.height = h;
        buf.format = f;
        buf.handle = handle;
        buf.stride = stride;

        private_handle_t* hnd = (private_handle_t*)(handle);
        buf.start = (void*)(hnd->base);
        buf.length = hnd->size;
        fprintf(stderr, "%s:%d: index=%d, w=%d, h=%d, f=0x%x, handle=0x%x, stride=%d, start=%p, length=%d\n",
                __FUNCTION__,__LINE__, i, w, h, f, (unsigned int)handle, stride, buf.start, buf.length);
    }

    for (int i=0; i< BUFLEN_CAPTURE; i++){
        buffer_handle_t handle;
        int stride;
        int ret;
        int w = width_capture;
        int h = height_capture;
        int f = convert_fmt_v4l2_to_hal(format_capture_v4l2);
        ret = _allocate(w, h, f, handle, stride);
        if (ret){
            fprintf(stderr, "%s:%d: failed to allocate buffer\n",__FUNCTION__,__LINE__);
            errno_exit(__FUNCTION__);
        }
        CamBuffer& buf = buffer_capture[i];
        buf.width = w;
        buf.height = h;
        buf.format = f;
        buf.handle = handle;
        buf.stride = stride;

        private_handle_t* hnd = (private_handle_t*)(handle);
        buf.start = (void*)(hnd->base);
        buf.length = hnd->size;

        fprintf(stderr, "%s:%d: index=%d, w=%d, h=%d, f=0x%x, handle=0x%x, stride=%d, start=%p, length=%d\n",
                __FUNCTION__,__LINE__, i, w, h, f, (unsigned int)handle, stride, buf.start, buf.length);
    }

    return 0;
}/*}}}*/

int free_buffer()
{
    for (int i=0;i<BUFLEN_OVERLAY;i++){
        buffer_handle_t handle = buffer_overlay[i].handle;
        m_gralloc_module->unlock(m_gralloc_module, handle);
        m_alloc_dev->free(m_alloc_dev, handle);
    }
    for (int i=0;i<BUFLEN_CAPTURE;i++){
        buffer_handle_t handle = buffer_capture[i].handle;
        m_gralloc_module->unlock(m_gralloc_module, handle);
        m_alloc_dev->free(m_alloc_dev, handle);
    }

    return 0;
}

static void dump_frame(const void *p, int size, int i)
{/*{{{*/
    char name[64];
    sprintf(name, "%s_%d_%dx%d",DUMP_FILE_OVERLAY, i, width_overlay, height_overlay);

    fprintf(stderr,"dump file %s\n", name);
    FILE* dump_fd = fopen(name, "wb");
    if (dump_fd == NULL) {
        fprintf(stderr,"Failed to open dump file %s\n", name);
    }
    else{
        fwrite(p, 1, size, dump_fd);
        fclose(dump_fd);
    }
}/*}}}*/

static int handle_frame(int i)
{/*{{{*/
    struct v4l2_buffer buf;

    fprintf(stderr, ".");
    CLEAR(buf);

    //FIXME:
    //buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.type = V4L2_BUF_TYPE_VIDEO_OVERLAY;
    buf.memory = V4L2_MEMORY_USERPTR;

    int ret = 0;
    ret = xioctl(node_list[INDEX_OVERLAY].fd, VIDIOC_DQBUF, &buf);
    if (ret){
        fprintf(stderr, "%s:%d: ioctl dqbuf fail, return %d\n",__FUNCTION__,__LINE__, ret);

        switch (errno) {
            case EAGAIN:
                fprintf(stderr, "%s:%d: try again\n",__FUNCTION__,__LINE__);
                return 0;

            case EIO:
                /* Could ignore EIO, see spec. */
                /* fall through */

            default:
                errno_exit("VIDIOC_DQBUF");
        }
    }
    if (flag_dump)
        dump_frame((void *)buf.m.userptr, buf.bytesused, i);

    if (-1 == xioctl(node_list[INDEX_OVERLAY].fd, VIDIOC_QBUF, &buf))
        errno_exit("VIDIOC_QBUF");

    return 1;
}/*}}}*/

static void loop_device(void)
{/*{{{*/
    fprintf(stderr, "loop device\n");

    struct timeval start;

    int cost_time_ms = 0;
    struct timeval t_start,t_end;
    gettimeofday(&t_start, NULL);

    fprintf(stderr, "\n");
    for (unsigned long i=0;i<frame_count;i++){

        for (;;) {
            fd_set fds;
            struct timeval tv;
            int r;

            FD_ZERO(&fds);
            FD_SET(node_list[INDEX_OVERLAY].fd, &fds);

            /* Timeout. */
            tv.tv_sec = 2;
            tv.tv_usec = 0;

            r = select(node_list[INDEX_OVERLAY].fd + 1, &fds, NULL, NULL, &tv);

            if (-1 == r) {
                if (EINTR == errno)
                    continue;
                errno_exit("select");
            }

            if (0 == r) {
                fprintf(stderr, "select timeout\n");
                exit(EXIT_FAILURE);
            }

            if (handle_frame(i))
                break;
            /* EAGAIN - continue select loop. */
        }
    }
    fprintf(stderr, "\n");
    gettimeofday(&t_end, NULL);

    cost_time_ms = (t_end.tv_usec - t_start.tv_usec)/1E3 + (t_end.tv_sec - t_start.tv_sec)*1E3;
    fprintf(stderr, "cost time=%lums, frame count=%lu, fps=%lu\n", cost_time_ms, frame_count, (unsigned long)(frame_count*1E3/cost_time_ms));
}/*}}}*/

static void _stop_device(int fd, enum v4l2_buf_type type)
{
    if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
        errno_exit("VIDIOC_STREAMOFF");
}

static void stop_device(void)
{
    _stop_device(node_list[INDEX_OVERLAY].fd, V4L2_BUF_TYPE_VIDEO_OVERLAY);
    _stop_device(node_list[INDEX_CAPTURE].fd, V4L2_BUF_TYPE_VIDEO_CAPTURE);
}

static void _qbuf_device(int fd, enum v4l2_buf_type type, CamBuffer* p_cambuf, int bufcnt)
{
    for (int i = 0; i < bufcnt; i++) {
        struct v4l2_buffer buf;
        CLEAR(buf);
        CamBuffer& cambuf=p_cambuf[i];

        buf.index = i;
        buf.type = type;
        buf.memory = V4L2_MEMORY_USERPTR;
        buf.m.userptr = (unsigned long)cambuf.start;
        buf.length = cambuf.length;

        fprintf(stderr, "%s:qbuf index %d\n", __FUNCTION__, i);
        if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
            errno_exit("VIDIOC_QBUF");
    }
    if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
        errno_exit("VIDIOC_STREAMON");
}
#if 0
static void uninit_device(void)
{
    unsigned int i;

    switch (io) {
        case IO_METHOD_READ:
            free(buffers[0].start);
            break;

        case IO_METHOD_MMAP:
            for (i = 0; i < n_buffers; ++i)
                if (-1 == munmap(buffers[i].start, buffers[i].length))
                    errno_exit("munmap");
            break;

        case IO_METHOD_USERPTR:
            for (i = 0; i < n_buffers; ++i)
                free(buffers[i].start);
            break;
    }

    free(buffers);
}

#endif

static void _init_device(int fd, enum v4l2_buf_type type, int width, int height, int color)
{
    /**********************************************/
    //check capability
    {
        struct v4l2_capability cap;
        unsigned int min;

        if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &cap)) {
            if (EINVAL == errno) {
                fprintf(stderr, "%s is no V4L2 device\n", dev_name);
                exit(EXIT_FAILURE);
            } else {
                errno_exit("VIDIOC_QUERYCAP");
            }
        }

        if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
            fprintf(stderr, "%s is no video capture device\n", dev_name);
            exit(EXIT_FAILURE);
        }

        fprintf(stderr, "Drv: %s\nBus: %s\n", cap.driver, cap.bus_info );
        fprintf(stderr, "Phy cap: 0x%08X\n", cap.capabilities );

        switch (flag_io) {
            case IO_METHOD_READ:
                if (!(cap.capabilities & V4L2_CAP_READWRITE)) {
                    fprintf(stderr, "%s does not support read i/o\n", dev_name);
                    exit(EXIT_FAILURE);
                }
                break;

            case IO_METHOD_MMAP:
            case IO_METHOD_USERPTR:
                if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
                    fprintf(stderr, "%s does not support streaming i/o\n", dev_name);
                    exit(EXIT_FAILURE);
                }
                break;
        }
    }

    /**********************************************/
    /* Scan for supported formats */
    {
        struct v4l2_fmtdesc         fmt_desc;
        struct v4l2_frmsizeenum     frmsize;
        struct v4l2_frmivalenum     frmival;

        fmt_desc.index = 0;
        fmt_desc.type = type;
        while( ioctl( fd, VIDIOC_ENUM_FMT, &fmt_desc ) == 0 ) 
        {
            frmsize.pixel_format = fmt_desc.pixelformat;
            frmsize.index = 0;
            fprintf( stderr, "Resolutions for %X \"%s\":\n", fmt_desc.pixelformat, fmt_desc.description );
            while( ioctl( fd, VIDIOC_ENUM_FRAMESIZES, &frmsize ) == 0  ) 
            {
                fprintf( stderr, "\t%dx%d\n", 
                        frmsize.discrete.width,
                        frmsize.discrete.height);

                frmsize.index++;
            }
            fmt_desc.index++;
        }
    }

    /**********************************************/
    //select input
    {
        fprintf( stderr, "Select V4L2 input %d\n", input_device );
        if (ioctl(fd, VIDIOC_S_INPUT, &input_device) < 0) 
        {
            errno_exit("VIDIOC_S_INPUT");
        }
    }

    /**********************************************/
    /* Select video input, video standard and tune here. */
    {
        struct v4l2_crop crop;
        struct v4l2_cropcap cropcap;
        CLEAR(cropcap);

        cropcap.type = type;

        if (0 == xioctl(fd, VIDIOC_CROPCAP, &cropcap)) {
            crop.type = type;
            crop.c = cropcap.defrect; /* reset to default */

            if (-1 == xioctl(fd, VIDIOC_S_CROP, &crop)) {
                switch (errno) {
                    case EINVAL:
                        /* Cropping not supported. */
                        break;
                    default:
                        /* Errors ignored. */
                        break;
                }
            }
        } else {
            /* Errors ignored. */
        }
    }

}

static void _set_format_dev(int fd, enum v4l2_buf_type type, int width, int height, int color, int cnt)
{
    /**********************************************/
    //setformat
    {
        struct v4l2_pix_format pixfmt;
        CLEAR(pixfmt);
        pixfmt.width = width;
        pixfmt.height = height;
        pixfmt.bytesperline = get_bytes_per_line(color, width);
        pixfmt.pixelformat = color;
        pixfmt.sizeimage = (width * height * get_bits_per_pixel(color)/8);
        pixfmt.field = V4L2_FIELD_NONE;

        struct v4l2_format fmt;
        CLEAR(fmt);
        fmt.fmt.pix = pixfmt;
        fmt.type = type;

        fprintf(stderr, "%s: %d, call S_FMT\n", __FUNCTION__,__LINE__);
        if (-1 == xioctl(fd, VIDIOC_S_FMT, &fmt))
            errno_exit("VIDIOC_S_FMT");
        /* Note VIDIOC_S_FMT may change width and height. */

        /*
        if (fmt.fmt.pix.sizeimage > buffer_overlay[0].length)
            fprintf(stderr, "error: unmatched framesize, expected %d, actual %d\n", buffer_overlay[0].length,fmt.fmt.pix.sizeimage);
            */
    }

    /**********************************************/
    //request buffer
    {
        struct v4l2_requestbuffers req;

        CLEAR(req);

        req.count  = cnt;
        req.type   = type;
        req.memory = V4L2_MEMORY_USERPTR;

        if (-1 == xioctl(fd, VIDIOC_REQBUFS, &req)) {
            if (EINVAL == errno) {
                fprintf(stderr, "%s does not support " "user pointer i/o\n", dev_name);
                exit(EXIT_FAILURE);
            } else {
                errno_exit("VIDIOC_REQBUFS");
            }
        }
    }
}

static void start_device(void)
{
    //overlay
    _init_device(node_list[INDEX_OVERLAY].fd, V4L2_BUF_TYPE_VIDEO_OVERLAY,
            width_overlay, height_overlay, format_overlay_v4l2);
#if 0
    //FIXME: for capture node, don't need querycapability & setinput
    //capture
    _init_device(node_list[INDEX_CAPTURE].fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
            width_capture, height_capture, format_capture_v4l2);
#endif

    _set_format_dev(node_list[INDEX_OVERLAY].fd, V4L2_BUF_TYPE_VIDEO_OVERLAY,
            width_overlay, height_overlay, format_overlay_v4l2, BUFLEN_OVERLAY);

    _set_format_dev(node_list[INDEX_CAPTURE].fd, V4L2_BUF_TYPE_VIDEO_CAPTURE,
            width_capture, height_capture, format_capture_v4l2, BUFLEN_CAPTURE);

    //overlay qbuf
    _qbuf_device(node_list[INDEX_OVERLAY].fd, V4L2_BUF_TYPE_VIDEO_OVERLAY, buffer_overlay, BUFLEN_OVERLAY);
    //capture qbuf
    _qbuf_device(node_list[INDEX_CAPTURE].fd, V4L2_BUF_TYPE_VIDEO_CAPTURE, buffer_capture, BUFLEN_CAPTURE);
}

static void close_device(void)
{
    for (int i=0;i<dev_num;i++){
        if (-1 == close(node_list[i].fd))
            errno_exit("close");
        node_list[i].fd = -1;
    }
}

static void open_device(void)
{
    fprintf(stderr, "%s enter\n",__FUNCTION__);
    struct stat st;
    int i=0;
    const char* dev_name;
    int fd;

    for (i=0; i<dev_num; i++){
        dev_name = dev_name_list[i];

        fprintf(stderr, "open dev %s\n", dev_name);
        if (-1 == stat(dev_name, &st)) {
            fprintf(stderr, "Cannot identify '%s': %d, %s\n",
                    dev_name, errno, strerror(errno));
            exit(EXIT_FAILURE);
        }

        if (!S_ISCHR(st.st_mode)) {
            fprintf(stderr, "%s is no device\n", dev_name);
            exit(EXIT_FAILURE);
        }

        fd = open(dev_name, O_RDWR /* required */ | O_NONBLOCK, 0);
        if (-1 == fd) {
            fprintf(stderr, "Cannot open '%s': %d, %s\n",
                    dev_name, errno, strerror(errno));
            exit(EXIT_FAILURE);
        }
        node_list[i].fd = fd;
        node_list[i].name = dev_name;
    }
}

static void usage(FILE *fp, int argc, char **argv)
{
    fprintf(fp,
            "Usage: %s [options]\n\n"
            "Version 1.3\n"
            "Options:\n"
            "-h | --help          Print this message\n"
            "-i | --input         Select input device[0:rear, 1:front]\n"
            "-m | --mmap          Use memory mapped buffers\n"
            "-r | --read          Use read() calls\n"
            "-u | --userp         Use gralloc allocated buffers [default]\n"
            "-d | --dump          Dump file to /data/media/\n"
            "-o | --formatoverlay Overlay format[0:NV12, 1:NV21, 2:YV12(I420), 3:UYVY]\n"
            "-c | --formatcapture Captuer foramt[0:NV12, 1:NV21, 2:YV12(I420), 3:UYVY]\n"
            "-n | --number         Number of frames to cycle [default: %lu]\n"
            "",
            argv[0], frame_count);
}

static const char short_options[] = "hi:mrudo:c:n:";

static const struct option
long_options[] = {
    { "help"          , no_argument       , NULL , 'h' } ,
    { "input"         , required_argument , NULL , 'i' } ,
    { "mmap"          , no_argument       , NULL , 'm' } ,
    { "read"          , no_argument       , NULL , 'r' } ,
    { "userp"         , no_argument       , NULL , 'u' } ,
    { "dump"          , no_argument       , NULL , 'd' } ,
    { "formatoverlay" , required_argument , NULL , 'o' } ,
    { "formatcapture" , required_argument , NULL , 'c' } ,
    { "number"        , required_argument , NULL , 'n' } ,
    { 0               , 0                 , 0    , 0 }
};

int main(int argc, char **argv)
{

    for (;;) {
        int idx;
        int c;
        int fmt_idx=0;

        c = getopt_long(argc, argv,
                short_options, long_options, &idx);

        if (-1 == c)
            break;

        switch (c) {
            case 0: /* getopt_long() flag */
                break;

            case 'h':
                usage(stdout, argc, argv);
                exit(EXIT_SUCCESS);

            case 'm':
                flag_io = IO_METHOD_MMAP;
                fprintf(stderr, "mmap not implemetned\n");
                exit(EXIT_SUCCESS);
                break;

            case 'r':
                flag_io = IO_METHOD_READ;
                fprintf(stderr, "read not implemetned\n");
                exit(EXIT_SUCCESS);
                break;

            case 'u':
                flag_io = IO_METHOD_USERPTR;
                break;

            case 'd':
                flag_dump = 1;
                break;

            case 'o':
                errno = 0;
                fprintf(stderr, "##@@## %s\n", optarg);
                fmt_idx = strtol(optarg, NULL, 0);
                format_overlay_v4l2 = FMT_LIST[fmt_idx];
                if (errno)
                    errno_exit(optarg);
                break;

            case 'c':
                errno = 0;
                fmt_idx = strtol(optarg, NULL, 0);
                format_capture_v4l2 = FMT_LIST[fmt_idx];
                if (errno)
                    errno_exit(optarg);
                break;

            case 'n':
                errno = 0;
                frame_count = strtol(optarg, NULL, 0);
                if (errno)
                    errno_exit(optarg);
                break;

            case 'i':
                errno = 0;
                input_device = strtol(optarg, NULL, 0);
                if (errno)
                    errno_exit(optarg);
                break;

            default:
                usage(stderr, argc, argv);
                exit(EXIT_FAILURE);
        }
    }

    init_allocator();
    allocate_buffer();

    open_device();
    start_device();

    loop_device();
    stop_device();
    close_device();

    free_buffer();
    free_allocator();
    return 0;
}

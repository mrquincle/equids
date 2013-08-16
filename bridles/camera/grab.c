#include "libcam2.h"
#include "grab.h"
#include <assert.h>

struct buffer          *gBuffers;

static void errno_exit(const char *s)
{
	fprintf(stderr, "%s error %d, %s\n", s, errno, strerror(errno));
	exit(EXIT_FAILURE);
}

int cam_format(int fd, int width, int height, int format)
{
	struct v4l2_format fmt;
	CLEAR(fmt);
	fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	fmt.fmt.pix.width = width;
	fmt.fmt.pix.height = height;
	if(format==0) fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_RGB24;
	else if(format==1 || format==2) fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;	//default
	//else if(format==1 || format==2) fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_VYUY;
	//else if(format==1 || format==2) fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_UYVY;
	//else if(format==1 || format==2) fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YVYU;

	fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
	xioctl(fd, VIDIOC_S_FMT, &fmt);

	if ((fmt.fmt.pix.width != width) || (fmt.fmt.pix.height != height))
	{
		printf("unexpected image size, %dx%d\n",
				fmt.fmt.pix.width, fmt.fmt.pix.height);
		return 0;
	}

	return 1;
}

unsigned char* cam_capture(int fd, int width, int height)
{
	unsigned int i, n_buffers=0;
	struct v4l2_buffer buf;
	struct buffer *buffers;
	int bufferCount=2;

	if(width<=0) width=640;
	if(height<=0) width=480;

	buffers = (struct buffer*) calloc(bufferCount, sizeof(struct buffer));
	for (n_buffers = 0; n_buffers < bufferCount; ++n_buffers)
	{
		CLEAR(buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = n_buffers;

		xioctl(fd, VIDIOC_QUERYBUF, &buf);

		buffers[n_buffers].length = buf.length;
		buffers[n_buffers].start = v4l2_mmap(NULL, buf.length,
				PROT_READ | PROT_WRITE, MAP_SHARED,
				fd, buf.m.offset);

		if (MAP_FAILED == buffers[n_buffers].start)
		{
			perror("mmap");
			exit(EXIT_FAILURE);
		}
	}

	for (i = 0; i < bufferCount; ++i) {
		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;
		xioctl(fd, VIDIOC_QBUF, &buf);
	}

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_STREAMON, &type);

	int r;
	fd_set fds;
	struct timeval tv;
	do {
		FD_ZERO(&fds);
		FD_SET(fd, &fds);

		// Timeout. 
		tv.tv_sec = 2;
		tv.tv_usec = 0;

		r = select(fd + 1, &fds, NULL, NULL, &tv);
	} while ((r == -1 && (errno = EINTR)));

	if (r == -1) {
		perror("select");
		return NULL;
		//return errno;
	}

	CLEAR(buf);
	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;
	xioctl(fd, VIDIOC_DQBUF, &buf);

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	xioctl(fd, VIDIOC_STREAMOFF, &type);

	//unmap memory
	for (i = 0; i < bufferCount; ++i) v4l2_munmap(buffers[i].start, buffers[i].length);

	//return buffers;
	return (unsigned char*)buffers->start;
}

unsigned char *cam_stream(int fd) {
	struct v4l2_buffer buf;
	unsigned int i;
	int n_buffers = 2;

	fd_set fds;
	struct timeval tv;
	int r;

	FD_ZERO(&fds);
	FD_SET(fd, &fds);

	/* Timeout. */
	tv.tv_sec = 2;
	tv.tv_usec = 0;

	r = select(fd + 1, &fds, NULL, NULL, &tv);

	if (-1 == r) {
		//		if (EINTR == errno)
		//			continue;
		errno_exit("select");
	}

	if (0 == r) {
		fprintf(stderr, "select timeout\n");
		exit(EXIT_FAILURE);
	}

	CLEAR(buf);

	buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buf.memory = V4L2_MEMORY_MMAP;

	if (-1 == xioctl(fd, VIDIOC_DQBUF, &buf)) {
		switch (errno) {
		case EAGAIN:
			return 0;

		case EIO:
			/* Could ignore EIO, see spec. */

			/* fall through */

		default:
			errno_exit("VIDIOC_DQBUF");
		}
	}

	assert(buf.index < n_buffers);

	//process_image(buffers[buf.index].start, buf.bytesused);

	if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
		errno_exit("VIDIOC_QBUF");

	return gBuffers[buf.index].start;
}

void start_capturing(int fd)
{
	int n_buffers = 2;
	unsigned int i;
	enum v4l2_buf_type type;

	for (i = 0; i < n_buffers; ++i) {
		struct v4l2_buffer buf;

		CLEAR(buf);
		buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory = V4L2_MEMORY_MMAP;
		buf.index = i;

		if (-1 == xioctl(fd, VIDIOC_QBUF, &buf))
			errno_exit("VIDIOC_QBUF");
	}
	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMON, &type))
		errno_exit("VIDIOC_STREAMON");
}


void stop_capturing(int fd)
{
	enum v4l2_buf_type type;

	type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (-1 == xioctl(fd, VIDIOC_STREAMOFF, &type))
		errno_exit("VIDIOC_STREAMOFF");
}

/**
 * The buffers are configured in cam_opendev, but they are not allocated yet.
 */
void init_mmap(int fd, const char* dev_name)
{
	int n_buffers = 2;

	int req_count = 2; // see libcam2.c cam_opendev with VIDIOC_REQBUFS

	gBuffers = calloc(req_count, sizeof(*gBuffers));

	if (!gBuffers) {
		fprintf(stderr, "Out of memory\n");
		exit(EXIT_FAILURE);
	}

	for (n_buffers = 0; n_buffers < req_count; ++n_buffers) {
		struct v4l2_buffer buf;

		CLEAR(buf);

		buf.type        = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		buf.memory      = V4L2_MEMORY_MMAP;
		buf.index       = n_buffers;

		if (-1 == xioctl(fd, VIDIOC_QUERYBUF, &buf))
			errno_exit("VIDIOC_QUERYBUF");

		gBuffers[n_buffers].length = buf.length;
		gBuffers[n_buffers].start =
				mmap(NULL /* start anywhere */,
						buf.length,
						PROT_READ | PROT_WRITE /* required */,
						MAP_SHARED /* recommended */,
						fd, buf.m.offset);

		if (MAP_FAILED == gBuffers[n_buffers].start)
			errno_exit("mmap");
	}
}

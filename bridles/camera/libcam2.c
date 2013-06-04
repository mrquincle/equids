#include "libcam2.h"

struct v4l2_queryctrl queryctrl;
struct v4l2_control control;


int cam_opendev(const char *dev_name, int width, int height, int format)
{
	int fd;

	fd = v4l2_open(dev_name, O_RDWR | O_NONBLOCK, 0);

        if (fd < 0)
	{
            perror("Cannot open device");
            exit(EXIT_FAILURE);
        }
	
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

	struct v4l2_requestbuffers req;
	CLEAR(req);
	req.count = 2;
	req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	req.memory = V4L2_MEMORY_MMAP;
	xioctl(fd, VIDIOC_REQBUFS, &req);

	return fd;
}


int cam_closedev(int fd)
{
    v4l2_close(fd);
    
    return 0;
}


void cam_vflip(int fd, int value)
{
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_VFLIP;

	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		} else {
			printf ("V4L2_CID_VFLIP is not supported\n");
		}
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("V4L2_CID_VFLIP is DISABLED\n");
	} else {
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_VFLIP;

		if(value==0 || value==1) control.value = value;
		else control.value = queryctrl.default_value;

		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
		else printf ("vflip is set to %d\n", control.value);
	}
}


void cam_hflip(int fd, int value)
{
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_HFLIP;

	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		} else {
			printf ("V4L2_CID_HFLIP is not supported\n");
		}
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("V4L2_CID_HFLIP is DISABLED\n");
	} else {
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_HFLIP;

		if(value==0 || value==1) control.value = value;
		else control.value = queryctrl.default_value;

		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
		else printf ("hflip is set to %d\n", control.value);
	}
}

void cam_brightness(int fd, int value)
{
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_BRIGHTNESS;

	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		} else {
			printf ("V4L2_CID_BRIGHTNESS is not supported\n");
		}
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("V4L2_CID_BRIGHTNESS is DISABLED\n");
	} else {
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_BRIGHTNESS;

		if(value>=0 && value<=255) control.value = value;
		else control.value = queryctrl.default_value;

		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
		else printf ("brightness is set to %d\n", control.value);
	}
}

void cam_contrast(int fd, int value)
{
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_CONTRAST;

	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {

		if (errno != EINVAL) {
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		} else {
			printf ("V4L2_CID_CONTRAST is not supported\n");
		}
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("V4L2_CID_CONTRAST is DISABLED\n");
	} else {
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_CONTRAST;

		if(value>=0 && value<=127) control.value = value;
		else control.value = queryctrl.default_value;

		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
		else printf ("contrast is set to %d\n", control.value);
	}
}

void cam_saturation(int fd, int value)
{
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_SATURATION;

	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		} else {
			printf ("V4L2_CID_SATURATION is not supported\n");
		}
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("V4L2_CID_SATURATION is DISABLED\n");
	} else {
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_SATURATION;

		if(value>=0 && value<=256) control.value = value;
		else control.value = queryctrl.default_value;

		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
		else printf ("saturation is set to %d\n", control.value);
	}
}

void cam_hue(int fd, int value)
{
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_HUE;

	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		} else {
			printf ("V4L2_CID_HUE is not supported\n");
		}
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("V4L2_CID_HUE is DISABLED\n");
	} else {
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_HUE;

		if(value>=-180 && value<=180) control.value = value;
		else control.value = queryctrl.default_value;

		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
		else printf ("hue is set to %d\n", control.value);
	}
}

void cam_autogain(int fd, int value)
{
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_AUTOGAIN;

	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		} else {
			printf ("V4L2_CID_AUTOGAIN is not supported\n");
		}
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("V4L2_CID_AUTOGAIN is DISABLED\n");
	} else {
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_AUTOGAIN;

		if(value==0 || value==1) control.value = value;
		else control.value = queryctrl.default_value;

		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
		else printf ("autogain is set to %d\n", control.value);
	}
}

void cam_gain(int fd, int value)
{
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_GAIN;

	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		} else {
			printf ("V4L2_CID_GAIN is not supported\n");
		}
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("V4L2_CID_GAIN is DISABLED\n");
	} else {
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_GAIN;

		if(value>=0 && value<=255) control.value = value;
		else control.value = queryctrl.default_value;

		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
		else printf ("gain is set to %d\n", control.value);
	}
}

void cam_autoexposure(int fd, int value)
{
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_EXPOSURE_AUTO;

	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		} else {
			printf ("V4L2_CID_EXPOSURE_AUTO is not supported\n");
		}
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("V4L2_CID_EXPOSURE_AUTO is DISABLED\n");
	} else {
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_EXPOSURE_AUTO;

		if(value==0 ) control.value = V4L2_EXPOSURE_MANUAL;
		else if(value==1) control.value = V4L2_EXPOSURE_AUTO;
		else control.value = queryctrl.default_value;

		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
		else printf ("auto exposure is set to %d\n", control.value);
	}
}

void cam_exposure(int fd, int value)
{
	memset (&queryctrl, 0, sizeof (queryctrl));
	queryctrl.id = V4L2_CID_EXPOSURE;

	if (-1 == ioctl (fd, VIDIOC_QUERYCTRL, &queryctrl)) {
		if (errno != EINVAL) {
			perror ("VIDIOC_QUERYCTRL");
			exit (EXIT_FAILURE);
		} else {
			printf ("V4L2_CID_EXPOSURE is not supported\n");
		}
	} else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
		printf ("V4L2_CID_EXPOSURE is DISABLED\n");
	} else {
		memset (&control, 0, sizeof (control));
		control.id = V4L2_CID_EXPOSURE;

		if(value>=0 && value<=65535) control.value = value;
		else control.value = queryctrl.default_value;

		if (-1 == ioctl (fd, VIDIOC_S_CTRL, &control)) {
			perror ("VIDIOC_S_CTRL");
			exit (EXIT_FAILURE);
		}
		else printf ("exposure is set to %d\n", control.value);
	}
}


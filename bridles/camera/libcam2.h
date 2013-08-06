#ifndef LIBCAM2_H
#define LIBCAM2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>

// Now we only support v4l2, bummer...
#include <libv4l2.h>
#include <linux/videodev2.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))


struct buffer {
        void   *start;
        size_t length;
};

static void xioctl(int fh, int request, void *arg)
{
        int r;

        do {
                r = v4l2_ioctl(fh, request, arg);
        } while (r == -1 && ((errno == EINTR) || (errno == EAGAIN)));

        if (r == -1) {
                fprintf(stderr, "error %d, %s\n", errno, strerror(errno));
                exit(EXIT_FAILURE);
        }
}
//static void xioctl(int fh, int request, void *arg);

int  cam_opendev(const char *dev_name, int width, int height, int format);
int  cam_closedev(int fd);
void cam_vflip(int fd, int value);
void cam_hflip(int fd, int value);
void cam_brightness(int fd, int value);
void cam_contrast(int fd, int value);
void cam_saturation(int fd, int value);
void cam_hue(int fd, int value);
void cam_autogain(int fd, int value);
void cam_gain(int fd, int value);
void cam_autoexposure(int fd, int value);
void cam_exposure(int fd, int value);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

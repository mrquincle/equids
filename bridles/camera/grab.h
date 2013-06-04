#ifndef GRAB_H
#define GRAB_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

int cam_format(int fd, int width, int height, int format);
unsigned char* cam_capture(int fd, int width, int height);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif

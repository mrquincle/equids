#ifndef GRAB_H
#define GRAB_H

/**
 * Warning, it is not such a smart idea to compile this with a gcc compiler and then link with object files that are
 * compile with the g++ compiler. Please, consider compiling both with the g++ compiler in that case.
 */

#ifdef __cplusplus
extern "C" {
#endif

//! Set camera format
int cam_format(int fd, int width, int height, int format);

//! Capture image from camera
unsigned char* cam_capture(int fd, int width, int height);

unsigned char* cam_stream(int fd);

//! In case cam_stream is used, use this too, after cam_opendev
void init_mmap(int fd, const char* dev_name);

void start_capturing(int fd);

#ifdef __cplusplus
}
#endif

#endif

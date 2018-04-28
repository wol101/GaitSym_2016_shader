#ifndef QTKITHELPER_H
#define QTKITHELPER_H

// define the opaque pointer for movie creation
typedef struct MyMovie *MyMovieHandle;

#if defined(__APPLE__) && !defined(USE_WI_BB)
// only Apple supports the QTKit based Quicktime routines used here

extern "C" MyMovieHandle initialiseMovieFile(const char *file, int framerate);
extern "C" void addImageToMovie(MyMovieHandle myMovie, int width, int height, unsigned char *rgb);
extern "C" void closeMovieFile(MyMovieHandle myMovie);

#else

#define initialiseMovieFile(file, framerate) 0
#define addImageToMovie(myMovie, width, height, rgb) {}
#define closeMovieFile(myMovie) {}

//inline MyMovieHandle initialiseMovieFile(const char *file, int framerate) { return 0; }
//inline void addImageToMovie(MyMovieHandle myMovie, int width, int height, unsigned char *rgb) {};
//inline void closeMovieFile(MyMovieHandle myMovie) {};

#endif

#endif // QTKITHELPER_H


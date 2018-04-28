#import <Cocoa/Cocoa.h>
#import <QTKit/QTKit.h>
#import "QTKitHelper.h"

struct MyMovie
{
    QTMovie *movie;
    int framerate;
};

MyMovieHandle initialiseMovieFile(const char *file, int framerate)
{
    MyMovie *m = new MyMovie;

    NSString *filename = [[NSString alloc] initWithCString:file encoding:NSUTF8StringEncoding];

    m->movie = [[QTMovie alloc] initToWritableFile:filename error:nil];
    m->framerate = framerate;

    [filename release];

    return m;
}

void addImageToMovie(MyMovieHandle myMovie, int width, int height, unsigned char *rgb)
{
    MyMovie *m = (MyMovie *)myMovie;
    unsigned char **planes = &rgb;

    NSBitmapImageRep *image = [[NSBitmapImageRep alloc] initWithBitmapDataPlanes:planes pixelsWide:width pixelsHigh:height
        bitsPerSample:8 samplesPerPixel:3 hasAlpha:0 isPlanar:0 colorSpaceName:@"NSDeviceRGBColorSpace"
        bitmapFormat:0 bytesPerRow:width * 3 bitsPerPixel:8 * 3];

    QTTime time = QTMakeTime(1, m->framerate);
    NSDictionary *attrs = [NSDictionary dictionaryWithObject:@"png " forKey:QTAddImageCodecType];
    NSImage *img = [[NSImage alloc] initWithData:[image TIFFRepresentation]];
    [m->movie addImage:img forDuration:time withAttributes:attrs];

    [img release];
    [image release];
}

void closeMovieFile(MyMovieHandle myMovie)
{
    MyMovie *m = (MyMovie *)myMovie;
    [m->movie updateMovieFile];
    [m->movie release];

    delete m;
}


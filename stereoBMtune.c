
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <stdio.h> 
#include <gtk/gtk.h>
#include <string.h>
#include <stdio.h>
#include <time.h>


/* Main data structure definition */
typedef struct _ChData ChData;
struct _ChData
{
	/* Widgets */
	GtkWidget *main_window;  /* Main application window */
	GtkImage *image_left;
	GtkImage *image_right;
	GtkImage *image_depth;
    GdkPixbuf *pixbuf_left;
    GdkPixbuf *pixbuf_right;
    GdkPixbuf *pixbuf_depth;


	/* OpenCV */
	CvStereoBMState *BMState; /* Block Matching State */
	IplImage *cv_image_left;
	IplImage *cv_image_right;

	CvMat *cv_image_depth;
	IplImage *cv_image_depth_aux;
	
};


/**
* Update the passed gtk image with the passed Ipl image, scaling if necessary
*/
static void update_gtk_image( GtkImage* gtk,  IplImage* ipl )
{
	//Update the depth image on the window
    GdkPixbuf *pixel_dest = gtk_image_get_pixbuf( gtk );
	GdkPixbuf *pixel_source = gdk_pixbuf_new_from_data (
		(guchar*)ipl->imageData,
		GDK_COLORSPACE_RGB,
		FALSE,
		ipl->depth,
		ipl->width,
		ipl->height,
		(ipl->widthStep),
		NULL,
		NULL
	);

    gdk_pixbuf_scale( pixel_source,pixel_dest,0,0,gdk_pixbuf_get_width(pixel_dest),gdk_pixbuf_get_height(pixel_dest),
                0,0,(double)gdk_pixbuf_get_width(pixel_dest)/(double)gdk_pixbuf_get_width(pixel_source),
                    (double)gdk_pixbuf_get_height(pixel_dest)/(double)gdk_pixbuf_get_height(pixel_source),GDK_INTERP_BILINEAR );
    gtk_image_set_from_pixbuf(gtk,pixel_dest);
    g_object_unref( pixel_source );

}
/* Function to compute StereoBM and update the result on the window */
void computeStereoBM ( ChData *data )
{
    long some_time ;
    time(&some_time);
	int i, j, aux;
	GdkPixbuf *pix; 
	IplImage *img;
	uchar *ptr_dst;
	cvFindStereoCorrespondenceBM ( 
		data->cv_image_left,
		data->cv_image_right, 
		data->cv_image_depth, 
		data->BMState
	);
	//Normalize the result so we can display it
	cvNormalize( data->cv_image_depth, data->cv_image_depth, 0, 256, CV_MINMAX, NULL );
	for ( i = 0; i < data->cv_image_depth->rows; i++)
	{
		aux = data->cv_image_depth->cols * i;
		ptr_dst = (uchar*)(data->cv_image_depth_aux->imageData + i*data->cv_image_depth_aux->widthStep);
		for ( j = 0; j < data->cv_image_depth->cols; j++ )
		{
			//((float*)(mat->data.ptr + mat->step*i))[j]
			ptr_dst[3*j] = (uchar)((short int*)(data->cv_image_depth->data.ptr + data->cv_image_depth->step*i))[j];
			ptr_dst[3*j+1] = (uchar)((short int*)(data->cv_image_depth->data.ptr + data->cv_image_depth->step*i))[j];
			ptr_dst[3*j+2] = (uchar)((short int*)(data->cv_image_depth->data.ptr + data->cv_image_depth->step*i))[j];
		}
	}
	
	//Transform IplImage to GtkImage
	update_gtk_image(data->image_depth,data->cv_image_depth_aux);
}


/* Define callbacks */

//Callback for adjustment preFilterSize
G_MODULE_EXPORT void on_adjustment1_value_changed( GtkAdjustment *adjustment, ChData *data )
{
	gint value;
	
	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}
		
	value = (gint) gtk_adjustment_get_value( adjustment );
		
	//the value must be odd, if it is not then set it to the next odd value
	if (value % 2 == 0) 
	{
		value += 1;
		gtk_adjustment_set_value( adjustment, (gdouble)value);
		return;
	}
	
	//set the parameter, 
	data->BMState->preFilterSize = value;
	computeStereoBM( data );
}

//Callback for adjustment preFilterCap
G_MODULE_EXPORT void on_adjustment2_value_changed( GtkAdjustment *adjustment, ChData *data )
{
	gint value;
	
	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}
	
	value = (gint) gtk_adjustment_get_value ( adjustment );
	
	//set the parameter
	data->BMState->preFilterCap = value;
	computeStereoBM( data );
}

//Callback for adjustment SADWindowSize
G_MODULE_EXPORT void on_adjustment3_value_changed( GtkAdjustment *adjustment, ChData *data )
{
	gint value;

	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}
	
	value = (gint) gtk_adjustment_get_value( adjustment );
		
	//the value must be odd, if it is not then set it to the next odd value
	if (value % 2 == 0) 
	{
		value += 1;
		gtk_adjustment_set_value( adjustment, (gdouble)value);
		return;
	}
	
	//the value must be smaller than the image size
	if ( value >= data->cv_image_left->width || value >= data->cv_image_left->height)
	{
		fprintf(stderr,"WARNING: SADWindowSize larger than image size\n");
		return;
	}
	
	//set the parameter, 
	data->BMState->SADWindowSize = value;
	computeStereoBM( data );
	
}

//Callback for adjustment minDisparity
G_MODULE_EXPORT void on_adjustment4_value_changed( GtkAdjustment *adjustment, ChData *data )
{
	gint value;
	
	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}
	
	value = (gint) gtk_adjustment_get_value( adjustment );
	
	data->BMState->minDisparity = value;
	computeStereoBM( data );
}

//Callback for adjustment numberOfDisparities
G_MODULE_EXPORT void on_adjustment5_value_changed( GtkAdjustment *adjustment, ChData *data )
{
	gint value;
	
	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}
	
	value = (gint) gtk_adjustment_get_value( adjustment );
	
	//te value must be divisible by 16, if it is not set it to the nearest multiple of 16
	if (value % 16 != 0)
	{
		value += (16 - value%16);
		gtk_adjustment_set_value( adjustment, (gdouble)value);
		return;
	}
	
	data->BMState->numberOfDisparities = value;
	computeStereoBM( data );
}

//Callback for adjustment textureThreshold
G_MODULE_EXPORT void on_adjustment6_value_changed( GtkAdjustment *adjustment, ChData *data )
{
	gint value;
	
	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}
	
	value = (gint) gtk_adjustment_get_value( adjustment );
	
	data->BMState->textureThreshold = value;
	computeStereoBM( data );
}

//Callback for adjustment uniquenessRatio
G_MODULE_EXPORT void on_adjustment7_value_changed( GtkAdjustment *adjustment, ChData *data )
{
	gint value;
	
	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}
	
	value = (gint) gtk_adjustment_get_value( adjustment );
	
	data->BMState->uniquenessRatio = value;
	computeStereoBM( data );
}

//Callback for adjustment speckleWindowSize
G_MODULE_EXPORT void on_adjustment8_value_changed( GtkAdjustment *adjustment, ChData *data )
{
	gint value;
	
	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}
	
	value = (gint) gtk_adjustment_get_value( adjustment );
	
	data->BMState->speckleWindowSize = value;
	computeStereoBM( data );
}

//Callback for adjustment speckleRange
G_MODULE_EXPORT void on_adjustment9_value_changed( GtkAdjustment *adjustment, ChData *data )
{
	gint value;
	
	if (data == NULL) {
		fprintf(stderr,"WARNING: data is null\n");
		return;
	}
	
	value = (gint) gtk_adjustment_get_value( adjustment );
	
	data->BMState->speckleRange = value;
	computeStereoBM( data );
}


 
int
main( int    argc,
      char **argv )
{
	char default_left_filename[] = "tsukuba/scene1.row3.col3.ppm";
	char default_right_filename[] = "tsukuba/scene1.row3.col5.ppm";
	char *left_filename = default_left_filename;
	char *right_filename = default_right_filename;
	int i;
	int norm_width = 320;
	int norm_height = 240;
	
	GtkBuilder *builder;
	GError *error = NULL;
	ChData *data;	

	/* Parse arguments to find left and right filenames */
	for ( i = 1; i < argc; i++ )
	{
		if (strcmp (argv[i], "-left") == 0)
		{
			i++;
			left_filename = argv[i];
		} 
		else if (strcmp (argv[i], "-right") == 0)
		{
			i++;
			right_filename = argv[i];
        }		
    
	}
	
	fprintf(stdout,"-left %s\n-right %s\n",left_filename,right_filename);
 
	/* Init GTK+ */
	gtk_init( &argc, &argv );

	/* Create data */
	data = g_slice_new(ChData);
	
	data->BMState = cvCreateStereoBMState(CV_STEREO_BM_BASIC, 64);
	if(data->BMState == NULL)
	{
		fprintf(stderr,"ERROR: Could not create CvStereoBMState\n");
		return 1;
	}
	data->cv_image_left = cvLoadImage(left_filename, 0);
	if (data->cv_image_left == NULL) 
	{
		fprintf(stderr, "ERROR: Could not load %s\n",left_filename);
		return 1;
	}
	data->cv_image_right = cvLoadImage(right_filename, 0);
	if (data->cv_image_right == NULL) 
	{
		fprintf(stderr, "ERROR: Could not load %s\n",right_filename);
		return 1;
	}
	if (data->cv_image_left->width != data->cv_image_right->width || data->cv_image_left->height != data->cv_image_right->height)
	{
		fprintf(stderr,"ERROR: Left and right image different size.\n");
		return 1;
	}
	data->cv_image_depth = cvCreateMat ( data->cv_image_left->height, data->cv_image_left->width, CV_16S);
	if (data->cv_image_depth == NULL)
	{
		fprintf(stderr,"ERROR: Could not create depth image.\n");
		return 1;
	}
	data->cv_image_depth_aux = cvCreateImage (cvGetSize(data->cv_image_left),IPL_DEPTH_8U, 3);
	if (data->cv_image_depth_aux == NULL)
	{
		fprintf(stderr,"ERROR: Could not create depth image.\n");
		return 1;
	}
	
	
	

	/* Create new GtkBuilder object */
	builder = gtk_builder_new();

	/* Load UI from file. If error occurs, report it and quit application.
	 * Replace "tut.glade" with your saved project. */
	if( ! gtk_builder_add_from_file( builder, "StereoBMTuner.glade", &error ) )
	{
		g_warning( "%s", error->message );
		g_free( error );
		return( 1 );
	}
 
	/* Get main window pointer from UI */
	data->main_window = GTK_WIDGET( gtk_builder_get_object( builder, "window1" ) );
	data->image_left = GTK_IMAGE( gtk_builder_get_object( builder, "image_left" ) );
	data->image_right = GTK_IMAGE( gtk_builder_get_object( builder, "image_right" ) );
	data->image_depth = GTK_IMAGE( gtk_builder_get_object( builder, "image_disparity" ) );
	
	//Put images on place
    // I'm not sure why the *3 is necessary for the left and right pixel buffers but if
    // I don't include this it is 1/3 of the expected width
    data->pixbuf_left=gdk_pixbuf_new( GDK_COLORSPACE_RGB, FALSE, 8, 320*3, 240 );
    data->pixbuf_right=gdk_pixbuf_new( GDK_COLORSPACE_RGB, FALSE, 8, 320*3, 240 );
    data->pixbuf_depth=gdk_pixbuf_new( GDK_COLORSPACE_RGB, FALSE, 8, 320, 240 );
	gtk_image_set_from_pixbuf( data->image_left, data->pixbuf_left );
	gtk_image_set_from_pixbuf( data->image_right, data->pixbuf_right );
	gtk_image_set_from_pixbuf( data->image_depth, data->pixbuf_depth );
    update_gtk_image(data->image_left, data->cv_image_left);
    update_gtk_image(data->image_right, data->cv_image_right);
	
	//TODO: Get all the BM Default parameters from the GUI definition
	data->BMState->preFilterSize 		= 5;
	data->BMState->preFilterCap 		= 63;//1;
	data->BMState->SADWindowSize 		= 11;//5;
	data->BMState->minDisparity 		= 0;
	data->BMState->numberOfDisparities 	= 64;
	data->BMState->textureThreshold 	= 0;
	data->BMState->uniquenessRatio 	= 0;
	data->BMState->speckleWindowSize 	= 0;
	data->BMState->speckleRange		= 0;
	
	/* Execute first iteration */
	computeStereoBM ( data );
	
	/* Connect signals */
	gtk_builder_connect_signals( builder, data );
 
	/* Destroy builder, since we don't need it anymore */
	g_object_unref( G_OBJECT( builder ) );
 
	/* Show window. All other widgets are automatically shown by GtkBuilder */
	gtk_widget_show( data->main_window );
 
	/* Start main loop */
	gtk_main();

    g_object_unref(data->pixbuf_left);
    g_object_unref(data->pixbuf_right);
    g_object_unref(data->pixbuf_depth);
 
	return( 0 );
}


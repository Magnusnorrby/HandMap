//------------------------------------------------------------------------------
// <copyright file="MainWindow.xaml.cs" company="Microsoft">
//     Copyright (c) Microsoft Corporation.  All rights reserved.
// </copyright>
//------------------------------------------------------------------------------

namespace Microsoft.Samples.Kinect.DepthBasics
{
    using System;
    using System.ComponentModel;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Windows;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Map depth range to byte range
        /// </summary>
        private const int MapDepthToByte = 8000 / 256;
        
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        /// <summary>
        /// Reader for depth frames
        /// </summary>
        private DepthFrameReader depthFrameReader = null;

        /// <summary>
        /// Description of the data contained in the depth frame
        /// </summary>
        private FrameDescription depthFrameDescription = null;

        /// <summary>
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        /// <summary>
        /// Bitmap to display (RGB)
        /// </summary>
        private WriteableBitmap colorBitmap = null;
            
        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap depthBitmap = null;

        /// <summary>
        /// Intermediate storage for frame data converted to color
        /// </summary>
        private byte[] depthPixels = null;

        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Reader for body frames
        /// </summary>
        private BodyFrameReader bodyFrameReader = null;

        /// <summary>
        /// definition of bones
        /// </summary>
        private List<Tuple<JointType, JointType>> bones;

        /// <summary>
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;

        /// <summary>
        /// Brush used for drawing joints that are currently tracked
        /// </summary>
        private readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));

        /// <summary>
        /// Brush used for drawing joints that are currently inferred
        /// </summary>        
        private readonly Brush inferredJointBrush = Brushes.Yellow;

        /// <summary>
        /// Pen used for drawing bones that are currently inferred
        /// </summary>        
        private readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);

        /// <summary>
        /// Color used for drawing the body
        /// </summary>        
        private readonly Pen bodyPen = new Pen(Brushes.Red, 6);

        /// <summary>
        /// Transperent background color
        /// </summary>
        private Brush bgColor = new SolidColorBrush(Color.FromArgb(0, 0, 0, 0));
        
        

        /// <summary>
        /// Thickness of drawn joint lines
        /// </summary>
        private const double JointThickness = 3;

        /// <summary>
        /// Drawing group for body rendering output
        /// </summary>
        private DrawingGroup drawingGroup;

        /// <summary>
        /// Drawing image that we will display
        /// </summary>
        private DrawingImage jointSource;


        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Constant for clamping Z values of camera space points from being negative
        /// </summary>
        private const float InferredZPositionClamp = 0.1f;

        /// <summary>
        /// Width of display (depth space)
        /// </summary>
        private int displayWidth;

        /// <summary>
        /// Height of display (depth space)
        /// </summary>
        private int displayHeight;

        /// <summary>
        /// Right palm position in depthMap coordinates
        /// </summary>
        private Point rPalmPos;

        /// <summary>
        /// Intermediate storage for the color to depth mapping
        /// </summary>
        private DepthSpacePoint[] colorMappedToDepthPoints = null;



        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            // get the depth (display) extents
            FrameDescription frameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // get size of joint space
            this.displayWidth = frameDescription.Width;
            this.displayHeight = frameDescription.Height;

            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            // wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            FrameDescription colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);


            this.colorMappedToDepthPoints = new DepthSpacePoint[colorFrameDescription.Width * colorFrameDescription.Height];

            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
            this.depthFrameReader.FrameArrived += this.Reader_DepthFrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            List<Color> colorList = new List<Color>();
            colorList.Add(Color.FromArgb(255,0, 0, 255));
            colorList.Add(Color.FromArgb(255,0, 255, 0));
            colorList.Add(Color.FromArgb(255,70, 200, 0));
            colorList.Add(Color.FromArgb(255,100, 180, 0));
            colorList.Add(Color.FromArgb(255,200, 100, 0));
            colorList.Add(Color.FromArgb(255,230, 70, 0));
            colorList.Add(Color.FromArgb(255,255, 0, 0));
            colorList.Add(Color.FromArgb(255, 0, 0, 0));



            BitmapPalette bp = new BitmapPalette(colorList);
                
            // create the bitmap to display            
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Indexed8, bp);
           
            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // get FrameDescription from the BodyFrameSource
            this.bodyFrameReader.FrameArrived += this.Reader_BodyFrameArrived;

            // a bone defined as a line between two joints
            this.bones = new List<Tuple<JointType, JointType>>();

            // Right Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowRight, JointType.WristRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.HandRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandRight, JointType.HandTipRight));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristRight, JointType.ThumbRight));

            // Left Arm
            this.bones.Add(new Tuple<JointType, JointType>(JointType.ElbowLeft, JointType.WristLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.HandLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.HandLeft, JointType.HandTipLeft));
            this.bones.Add(new Tuple<JointType, JointType>(JointType.WristLeft, JointType.ThumbLeft));

            // Create the drawing group we'll use for drawing
            this.drawingGroup = new DrawingGroup();

            // Create an image source that we can use in our image control
            this.jointSource = new DrawingImage(this.drawingGroup);
           

            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;

        /// <summary>
        /// Gets the bitmap to display
        /// </summary>
        public ImageSource ImageSource
        {
            get
            {
                return this.colorBitmap;
            }
        }

        /// <summary>
        /// Gets the depth bitmap to display
        /// </summary>
        public ImageSource DepthSource
        {
            get
            {
                return this.depthBitmap;
            }
        }

        /// <summary>
        /// Gets the bitmap containing joints to display
        /// </summary>
        public ImageSource JointSource
        {
            get
            {
                return this.jointSource;
            }
        }

        /// <summary>
        /// Gets or sets the current status text to display
        /// </summary>
        public string StatusText
        {
            get
            {
                return this.statusText;
            }

            set
            {
                if (this.statusText != value)
                {
                    this.statusText = value;

                    // notify any bound elements that the text has changed
                    if (this.PropertyChanged != null)
                    {
                        this.PropertyChanged(this, new PropertyChangedEventArgs("StatusText"));
                    }
                }
            }
        }

        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.depthFrameReader != null)
            {
                // DepthFrameReader is IDisposable
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
            }

            if (this.colorFrameReader != null)
            {
                // ColorFrameReder is IDisposable
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }

            if (this.bodyFrameReader != null)
            {
                // BodyFrameReader is IDisposable
                this.bodyFrameReader.Dispose();
                this.bodyFrameReader = null;
            }

            if (this.kinectSensor != null)
            {
                this.kinectSensor.Close();
                this.kinectSensor = null;
            }
        }

      

        /// <summary>
        /// Handles the color frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
        {
            // ColorFrame is IDisposable
            using (ColorFrame colorFrame = e.FrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    FrameDescription colorFrameDescription = colorFrame.FrameDescription;

                    using (KinectBuffer colorBuffer = colorFrame.LockRawImageBuffer())
                    {
                        this.colorBitmap.Lock();

                        // verify data and write the new color frame data to the display bitmap
                        if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                        {
                            colorFrame.CopyConvertedFrameDataToIntPtr(
                                this.colorBitmap.BackBuffer,
                                (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                                ColorImageFormat.Bgra);

                            this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                        }

                        this.colorBitmap.Unlock();
                    }
                }
            }

         

            

        }

        /// <summary>
        /// gets the color at a specific pixel
        /// </summary>
        /// <param name="colorbitmap">the rgb image</param>
        /// <param name="x">x coordiante in the rgb image</param>
        /// <param name="y">y coordiante in the rgb image</param>
        public static Color getColorFromPixel(WriteableBitmap colorbitmap, int x, int y)
        {
            Color c = new Color();

            // check that the pixel is within range
            if (x > colorbitmap.PixelWidth || y > colorbitmap.PixelHeight)
            {
                return c;
            }

            IntPtr buffer = colorbitmap.BackBuffer;
            int pos = y * colorbitmap.BackBufferStride + x * 4;
            unsafe
            {
                byte* p_buffer = (byte*)buffer.ToPointer();
                c = Color.FromArgb(p_buffer[pos + 3], p_buffer[pos + 2], p_buffer[pos + 1], p_buffer[pos]);
                //p_buffer[pos + 3] = c.A;
                //p_buffer[pos + 2] = 0;//c.R;
                //p_buffer[pos + 1] = 0; // c.G;
                //p_buffer[pos] = 0; // c.B;
            }
            return c;
        }

        ///// <summary>
        ///// sets the color of a specific pixel
        ///// </summary>
        ///// <param name="colorbitmap">the rgb image</param>
        ///// <param name="x">x coordiante in the rgb image</param>
        ///// <param name="y">y coordiante in the rgb image</param>
        //public void setColorAtPixel(WriteableBitmap colorbitmap, int x, int y)
        //{
        //    // check that the pixel is within range
        //    if (x > colorbitmap.PixelWidth || x < 0 || y > colorbitmap.PixelHeight || y < 0)
        //    {
        //        return;
        //    }

        //    IntPtr buffer = colorbitmap.BackBuffer;
        //    int pos = y * colorbitmap.BackBufferStride + x * 4;
        //    unsafe
        //    {
        //        byte* p_buffer = (byte*)buffer.ToPointer();
        //        p_buffer[pos + 3] = 255; // A
        //        p_buffer[pos + 2] = 0;//c.R;
        //        p_buffer[pos + 1] = 0; // c.G;
        //        p_buffer[pos] = 0;  // b
        //    }

        //}


        /// <summary>
        /// Handles the depth frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_DepthFrameArrived(object sender, DepthFrameArrivedEventArgs e)
        {
            bool depthFrameProcessed = false;

            using (DepthFrame depthFrame = e.FrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    // the fastest way to process the body index data is to directly access 
                    // the underlying buffer
                    using (Microsoft.Kinect.KinectBuffer depthBuffer = depthFrame.LockImageBuffer())
                    {
                        // verify data and write the color data to the display bitmap
                        if (((this.depthFrameDescription.Width * this.depthFrameDescription.Height) == (depthBuffer.Size / this.depthFrameDescription.BytesPerPixel)) &&
                            (this.depthFrameDescription.Width == this.depthBitmap.PixelWidth) && (this.depthFrameDescription.Height == this.depthBitmap.PixelHeight))
                        {
                            // Note: In order to see the full range of depth (including the less reliable far field depth)
                            // we are setting maxDepth to the extreme potential depth threshold
                            //ushort maxDepth = ushort.MaxValue;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            ushort maxDepth = depthFrame.DepthMaxReliableDistance;
                          
                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            depthFrameProcessed = true;
                        }
                    }
                }
            }

            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();
            }


        }

        /// <summary>
        /// Handles the body frame data arriving from the sensor
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Reader_BodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            bool dataReceived = false;
            Color rHandColor = Color.FromArgb(0, 0, 0, 0);
            using (BodyFrame bodyFrame = e.FrameReference.AcquireFrame())
            {
                if (bodyFrame != null)
                {
                    if (this.bodies == null)
                    {
                        this.bodies = new Body[bodyFrame.BodyCount];
                    }

                    // The first time GetAndRefreshBodyData is called, Kinect will allocate each Body in the array.
                    // As long as those body objects are not disposed and not set to null in the array,
                    // those body objects will be re-used.
                    bodyFrame.GetAndRefreshBodyData(this.bodies);
                    dataReceived = true;
                }
            }
            if (dataReceived)
            {
                using (DrawingContext dc = this.drawingGroup.Open())
                {
                    // Draw a transparent background to set the render size
                    dc.DrawRectangle(bgColor, null, new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyPen;

                        if (body.IsTracked)
                        {

                            IReadOnlyDictionary<JointType, Joint> joints = body.Joints;
                            // convert the joint points to depth (display) space
                            Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                            foreach (JointType jointType in joints.Keys)
                            {
                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = InferredZPositionClamp;
                                }

                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);

                                //ColorSpacePoint colorSpacePoint;
                                switch (jointType)
                                {
                                    //Try to find the deepth coordianates of the hand
                                    case JointType.HandRight:
                                        //colorSpacePoint = this.coordinateMapper.MapCameraPointToColorSpace(position);
                                        rPalmPos = new Point(depthSpacePoint.X,depthSpacePoint.Y);//new Point(colorSpacePoint.X,colorSpacePoint.Y);
                                        rHandColor =  getColorFromPixel(this.depthBitmap, (int)depthSpacePoint.X, (int)depthSpacePoint.Y);//getColorFromPixel(this.depthBitmap, (int)colorSpacePoint.X, (int)colorSpacePoint.Y);
                                        break;

                                }

                            }

                            //this.DrawBody(joints, jointPoints, dc, drawPen);


                            // prevent drawing outside of our render area
                            this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, this.displayWidth, this.displayHeight));

                        }
                    }

                   
                }
            }
        }



        private void DrawBody(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, DrawingContext drawingContext, Pen drawingPen)
        {
            // Draw the bones
            foreach (var bone in this.bones)
            {
                this.DrawBone(joints, jointPoints, bone.Item1, bone.Item2, drawingContext, drawingPen);
            }

            // Draw the joints
            foreach (JointType jointType in joints.Keys)
            {
                Brush drawBrush = null;

                TrackingState trackingState = joints[jointType].TrackingState;

                if (trackingState == TrackingState.Tracked)
                {
                    drawBrush = this.trackedJointBrush;
                }
                else if (trackingState == TrackingState.Inferred)
                {
                    drawBrush = this.inferredJointBrush;
                }

                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, jointPoints[jointType], JointThickness, JointThickness);
                }
            }
        }

        /// <summary>
        /// Draws one bone of a body (joint to joint)
        /// </summary>
        /// <param name="joints">joints to draw</param>
        /// <param name="jointPoints">translated positions of joints to draw</param>
        /// <param name="jointType0">first joint of bone to draw</param>
        /// <param name="jointType1">second joint of bone to draw</param>
        /// <param name="drawingContext">drawing context to draw to</param>
        /// /// <param name="drawingPen">specifies color to draw a specific bone</param>
        private void DrawBone(IReadOnlyDictionary<JointType, Joint> joints, IDictionary<JointType, Point> jointPoints, JointType jointType0, JointType jointType1, DrawingContext drawingContext, Pen drawingPen)
        {
            Joint joint0 = joints[jointType0];
            Joint joint1 = joints[jointType1];

            // If we can't find either of these joints, exit
            if (joint0.TrackingState == TrackingState.NotTracked ||
                joint1.TrackingState == TrackingState.NotTracked)
            {
                return;
            }

            // We assume all drawn bones are inferred unless BOTH joints are tracked
            Pen drawPen = this.inferredBonePen;
            if ((joint0.TrackingState == TrackingState.Tracked) && (joint1.TrackingState == TrackingState.Tracked))
            {
                drawPen = drawingPen;
            }

            drawingContext.DrawLine(drawPen, jointPoints[jointType0], jointPoints[jointType1]);
        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        /// <param name="minDepth">The minimum reliable depth value for the frame</param>
        /// <param name="maxDepth">The maximum reliable depth value for the frame</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            int index = ((int)rPalmPos.X) + ((int)rPalmPos.Y) * this.displayWidth;

            int frameDataLength = (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel);

            //make sure we don't go out of bounds
            if (index >= frameDataLength)
            {
                return;
            }
            
            
            ushort palmDepth = frameData[index];




            // convert depth to a visual representation
            for (int i = 0; i < frameDataLength; ++i)
            {

                this.depthPixels[i] = 7;

            }

            // defines the rectangle size
            int size = 60;

            int xLower = ((int)rPalmPos.X - size) > 0 ? ((int)rPalmPos.X - size) : 0;
            int yLower = ((int)rPalmPos.Y - size) > 0 ? ((int)rPalmPos.Y - size) : 0;
            for (int x = xLower; x < xLower + size*2; x++)
            {
                for (int y = yLower; y < yLower + size*2; y++)
                {
                    int i = x + y * this.displayWidth;
                    if (i < frameDataLength)
                    {
                        // Get the depth for this pixel
                        ushort depth = frameData[i];

                        //// map the coordinate to the color image
                        //DepthSpacePoint dsp = new DepthSpacePoint();
                        //dsp.X = x;
                        //dsp.Y = y;
                        //ColorSpacePoint colorPoint = coordinateMapper.MapDepthPointToColorSpace(dsp, depth);
                        //int cX = (int) colorPoint.X;
                        //int cY = (int) colorPoint.Y;
                        //setColorAtPixel(this.colorBitmap, cX, cY);


                        //only colors pixels that are close in depth to our palm depth (depth / MapDepthToByte)
                        switch ((palmDepth - depth) / 25)
                        {
                            case -1: // Behind palm depth, make blue
                                this.depthPixels[i] = 0; // On palm depth, make green
                                break;
                            case 0:
                                this.depthPixels[i] = 1; // further forward from palm depth, make red
                                break;
                            case 1:
                                this.depthPixels[i] = 2;
                                break;
                            case 2:
                                this.depthPixels[i] = 3;
                                break;
                            case 3:
                                this.depthPixels[i] = 4;
                                break;
                            case 4:
                                this.depthPixels[i] = 5;
                                break;
                            case 5:
                                this.depthPixels[i] = 6;
                                break;
                            case 6:
                            default:  // to far away, make black
                                this.depthPixels[i] = 7;
                                break;
                        }
                    
                    }
                }
            }

            // draws the rectangle
            int safe = 0;
           
            for (int i = -size; i < size; i++)
                {
                    safe = (int)rPalmPos.X + i + ((int)rPalmPos.Y + size) * this.displayWidth;
                    if (safe > 0 & safe < frameDataLength)
                        this.depthPixels[safe] = 5;
                    safe = (int)rPalmPos.X + i + ((int)rPalmPos.Y - size) * this.displayWidth;
                    if (safe > 0 & safe < frameDataLength)
                        this.depthPixels[safe] = 5;
                    safe = (int)rPalmPos.X + size + ((int)rPalmPos.Y + i) * this.displayWidth;
                    if (safe > 0 & safe < frameDataLength)
                        this.depthPixels[safe] = 5;
                    safe = (int)rPalmPos.X - size + ((int)rPalmPos.Y - i) * this.displayWidth;
                    if (safe > 0 & safe < frameDataLength)
                        this.depthPixels[safe] = 5;
                }



        }

        /// <summary>
        /// Renders color pixels into the writeableBitmap.
        /// </summary>
        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
        }

        /// <summary>
        /// Handles the event which the sensor becomes unavailable (E.g. paused, closed, unplugged).
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Sensor_IsAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            // on failure, set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.SensorNotAvailableStatusText;
        }
    }

}

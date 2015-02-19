
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
        /// Left palm position in depthMap coordinates
        /// </summary>
        private Point lPalmPos;

        /// <summary>
        /// Right wrist position in depthMap coordinates
        /// </summary>
        private Point rWristPos;

        /// <summary>
        /// Left wrist position in depthMap coordinates
        /// </summary>
        private Point lWristPos;

        /// <summary>
        /// The tracking states of the rPalmPos, lPalmPos, rWristPos, lWristPos
        /// </summary>
        private TrackingState[] trackingStates = new TrackingState[4];

        /// <summary>
        /// TackingId for the person driving the app
        /// </summary>
        private long driver = 0;


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
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgra32, null);

            // open the reader for the depth frames
            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();

            // wire handler for frame arrival
            this.depthFrameReader.FrameArrived += this.Reader_DepthFrameArrived;

            // get FrameDescription from DepthFrameSource
            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;

            // allocate space to put the pixels being received and converted
            this.depthPixels = new byte[this.depthFrameDescription.Width * this.depthFrameDescription.Height];

            List<Color> colorList = new List<Color>();
            colorList.Add(Color.FromArgb(100, 0, 0, 255));
            colorList.Add(Color.FromArgb(150, 0, 255, 0));
            colorList.Add(Color.FromArgb(100, 70, 200, 0));
            colorList.Add(Color.FromArgb(100, 100, 180, 0));
            colorList.Add(Color.FromArgb(100, 200, 100, 0));
            colorList.Add(Color.FromArgb(100, 230, 70, 0));
            colorList.Add(Color.FromArgb(255, 255, 0, 0));
            colorList.Add(Color.FromArgb(0, 0, 0, 0));



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
        /// Execute shutdown tasks. Called from the exit button
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Button_Close(object sender, RoutedEventArgs e)
        {
            CleanUp();
            this.Close();
        }
        /// <summary>
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            CleanUp();
        }

        /// <summary>
        /// Destructor
        /// </summary>
        private void CleanUp()
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
                    //ProcessColorFrameData();

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
            }
            return c;
        }

        /// <summary>
        /// sets the color of a specific pixel to transparent
        /// </summary>
        /// <param name="colorbitmap">the rgb image</param>
        /// <param name="x">x coordiante in the rgb image</param>
        /// <param name="y">y coordiante in the rgb image</param>
        public void makePixelTransparent(WriteableBitmap colorbitmap, int x, int y)
        {
            // check that the pixel is within range
            if (x > colorbitmap.PixelWidth || x < 0 || y > colorbitmap.PixelHeight || y < 0)
            {
                return;
            }

            IntPtr buffer = colorbitmap.BackBuffer;
            int pos = y * colorbitmap.BackBufferStride + x * 4;
            unsafe
            {
                byte* p_buffer = (byte*)buffer.ToPointer();
                p_buffer[pos + 3] = 0;  // A
                p_buffer[pos + 2] = 0;  // R
                p_buffer[pos + 1] = 0;  // G
                p_buffer[pos] = 0;  // B
            }

        }


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

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size);
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

                    bool driverExists = false;
                    foreach (Body body in this.bodies)
                    {
                        Pen drawPen = this.bodyPen;

                        if (body.IsTracked)
                        {

                            if (driver == 0) //if no driver has been assigned we assign one
                            {
                                driver = (long)body.TrackingId;
                            }
                            if (driver == (long)body.TrackingId) //Makes sure we only track one persons hands 
                            {
                                driverExists = true;
                                IReadOnlyDictionary<JointType, Joint> joints = body.Joints;

                                Dictionary<JointType, Point> jointPoints = new Dictionary<JointType, Point>();
                                foreach (JointType jointType in joints.Keys)
                                {

                                    TrackingState trackingState = joints[jointType].TrackingState;
                                    // sometimes the depth(Z) of an inferred joint may show as negative
                                    // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                    CameraSpacePoint position = joints[jointType].Position;
                                    if (position.Z < 0)
                                    {
                                        position.Z = InferredZPositionClamp;
                                    }

                                    // convert the joint points to depth (display) space
                                    DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);
                                    jointPoints[jointType] = new Point(depthSpacePoint.X, depthSpacePoint.Y);


                                    switch (jointType)
                                    {
                                        //Try to find the deepth coordianates of the hand
                                        case JointType.HandRight:
                                            //update the tracking state 
                                            trackingStates[0] = trackingState;
                                            if (trackingState == TrackingState.Tracked) //requiring the joint to be tracked saves jumpy behavior when Kinect guesses
                                            {
                                                rPalmPos = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                            }

                                            break;


                                        case JointType.HandLeft:
                                            trackingStates[1] = trackingState;
                                            if (trackingState == TrackingState.Tracked)
                                            {
                                                lPalmPos = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                            }
                                            break;

                                        case JointType.WristRight:
                                            trackingStates[2] = trackingState;
                                            if (trackingState == TrackingState.Tracked)
                                            {
                                                rWristPos = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                            }
                                            break;

                                        case JointType.WristLeft:
                                            trackingStates[3] = trackingState;
                                            if (trackingState == TrackingState.Tracked)
                                            {
                                                lWristPos = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                            }
                                            break;


                                    }

                                }

                                //this.DrawBody(joints, jointPoints, dc, drawPen);
                            }

                            if (!driverExists)  // no driver exists, reset the driver id so we can find a new driver
                            {
                                driver = 0;
                            }

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

        //private unsafe void ProcessColorFrameData()
        //{        
        //    // rectangle size
        //    int s = 60;

        //    //for depth coordinate conversion (3.75 for width and 2.54717 for height)
        //    int xStart = (int)(rPalmPos.X * 3.75 - s * 3.75)       < this.colorBitmap.Width  ? (int)(rPalmPos.X * 3.75 - s * 3.75)       : (int)this.colorBitmap.Width;
        //    int yStart = (int)(rPalmPos.Y * 2.54717 - s * 2.54717) < this.colorBitmap.Height ? (int)(rPalmPos.Y * 2.54717 - s * 2.54717) : (int)this.colorBitmap.Height;

        //    if (xStart < 0)
        //        xStart = 0;
        //    if (yStart < 0)
        //        yStart = 0;

        //    extractDepthData(xStart, yStart, s);

        //}

        //private void extractDepthData(int xStart, int yStart, int s)
        //{
        //    //for depth coordinate conversion
        //    float widthRatio  = this.depthFrameDescription.Width  / (float)this.colorBitmap.Width;
        //    float heightRatio = this.depthFrameDescription.Height / (float)this.colorBitmap.Height;

        //    int xEnd = xStart + s * 3.75    < this.colorBitmap.Width  ? (int)(xStart + s * 3.75)    : (int)this.colorBitmap.Width;
        //    int yEnd = yStart + s * 2.54717 < this.colorBitmap.Height ? (int)(yStart + s * 2.54717) : (int)this.colorBitmap.Height;

        //    for (int x = xStart; x < xEnd; x++)
        //    {
        //        for (int y = yStart; y < yEnd; y++)
        //        {
        //            //depth coordinates
        //            int dx = (int)(x * widthRatio);
        //            int dy = (int)(y * heightRatio);
        //            if (this.depthPixels[dx + dy * this.depthFrameDescription.Width] == 7)
        //            {
        //                makePixelTransparent(this.colorBitmap, x, y);
        //            }
        //        }

        //    }

        //}

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize)
        {

            //ReadOnlyDictionary<JointType, Joint> joints = body.Joints;



            int frameDataLength = (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel);

            // convert depth to a visual representation
            for (int i = 0; i < frameDataLength; ++i)
            {

                this.depthPixels[i] = 7; // sets a pixels to a default black

            }

            if (trackingStates[0] == TrackingState.Tracked && trackingStates[2] == TrackingState.Tracked) // we are tracking both the right palm and wrist
            {
                // right hand
                mapHand(frameDataLength, depthFrameData, (int)rPalmPos.X, (int)rPalmPos.Y);
                findFingers((int)rPalmPos.X, (int)rPalmPos.Y, (int)rWristPos.X, (int)rWristPos.Y, frameDataLength);
                findThumb((int)rPalmPos.X, (int)rPalmPos.Y, (int)rWristPos.X, (int)rWristPos.Y, frameDataLength, 1, -1);
            }


            if (trackingStates[1] == TrackingState.Tracked && trackingStates[3] == TrackingState.Tracked) // we are tracking both the left palm and wrist
            {
                // left hand
                mapHand(frameDataLength, depthFrameData, (int)lPalmPos.X, (int)lPalmPos.Y);
                findFingers((int)lPalmPos.X, (int)lPalmPos.Y, (int)lWristPos.X, (int)lWristPos.Y, frameDataLength);
                findThumb((int)lPalmPos.X, (int)lPalmPos.Y, (int)lWristPos.X, (int)lWristPos.Y, frameDataLength, -1, 1);
            }

        }



        /// <summary>
        /// Analyses the hand to find the thumb
        /// </summary>   
        /// <param name="palmPosX">The x coordinate of the palm</param>
        /// <param name="palmPosY">The y coordinate of the palm</param>
        /// <param name="wristPosX">The x coordinate of the wrist</param>
        /// <param name="wristPosY">The y coordinate of the wrist</param>
        /// <param name="frameDataLength">Size of the DepthFrame image data in pixels</param>
        /// <param name="d1">The direction variable for the thumb</param> //We presume that the right thumb is to the left of the hand and vice versa 
        /// <param name="d2">The direction variable the thumb</param>
        private void findThumb(int palmPosX, int palmPosY, int wristPosX, int wristPosY, int range, int d1, int d2)
        {
            int x = palmPosX - wristPosX;
            int y = palmPosY - wristPosY;

            int max = 0;
            int distance = 0;
            int sumDist = 0;
            int sumDistSize = 0; //all columns wont hit since we make it extra wide

            Point thumb = new Point(0, 0);

            int index;

            for (int i = 0; i < 30; i++)
            {
                //index = wristPosX + (x * i / 20) + ((y * i / 20) + wristPosY) * this.displayWidth;  // a line crossing both the center of the wrist and the center of the palm
                //if (index > 0 & index < range)
                //    this.depthPixels[index] = 6; //color it red

                //index = wristPosX + d1 * (y * i / 20) + (d2 * (x * i / 20) + wristPosY) * this.displayWidth;
                //if (index > 0 & index < range)
                //    this.depthPixels[index] = 6; //color it red

                int tempX = 0;
                int tempY = 0;
                for (int j = 40; j >= 0; j--)
                {
                    tempX = wristPosX + d1 * (y * j / 20) + (x * i / 20);
                    tempY = (d2 * (x * j / 20) + (y * i / 20) + wristPosY);
                    index = tempX + tempY * this.displayWidth;
                    if (index > 0 & index < range)
                    {
                        if (this.depthPixels[index] != 7 & this.depthPixels[index] != 5) //ignore black background and the red rectangle
                        {
                            distance = j;
                            sumDist += j;
                            sumDistSize++;
                            break;
                        }
                    }

                }
                if (distance > max)
                {
                    max = distance;
                    thumb.X = tempX;
                    thumb.Y = tempY;

                }
            }
            if (sumDistSize > 0)
            {
                if (max > 1.3 * sumDist / sumDistSize) // the thumb is stretched far enough
                    drawFingerTip(range, thumb, 0); // 0 draws the thumb circle blue
            }



        }

        /// <summary>
        /// Analyses the hand to find the fingers
        /// </summary>   
        /// <param name="palmPosX">The x coordinate of the palm</param>
        /// <param name="palmPosY">The y coordinate of the palm</param>
        /// <param name="wristPosX">The x coordinate of the wrist</param>
        /// <param name="wristPosY">The y coordinate of the wrist</param>
        /// <param name="frameDataLength">Size of the DepthFrame image data in pixels</param>
        private void findFingers(int palmPosX, int palmPosY, int wristPosX, int wristPosY, int range)
        {
            int x = palmPosX - wristPosX;
            int y = palmPosY - wristPosY;
            int width = 40;
            // the ratio between width and divi,divj decides the distance between searched pixels
            int divi = 20;
            int divj = 40;



            int sumDist = 0;
            int averageDist = 0;
            int sumDistSize = 0; //all columns wont hit since we make it extra wide

            int[] distArray = new int[width * 2]; // ranges from -width to +width
            Point[] pointArray = new Point[width * 2];

            Point tip = new Point(0, 0);

            int index;
            for (int i = -width; i < width; i++)
            {

                //index = (int)palmPosX - (y * i / divi) + ((x * i / divi) + (int)palmPosY) * this.displayWidth; // a perpendicular line starting at the palm
                //if (index > 0 & index < range)
                //    this.depthPixels[index] = 6; //color it red

                int tempX = 0;
                int tempY = 0;
                // a box from the perpendicular line and forward, attempting to box in middle finger
                for (int j = 120; j > 0; j--) // should over shoot the hand
                {
                    tempX = palmPosX + -(y * i / divi) + (x * j / divj);
                    tempY = ((x * i / divi) + (y * j / divj) + palmPosY);

                    index = tempX + tempY * this.displayWidth;
                    if (index > 0 & index < range)
                    {
                        if (this.depthPixels[index] != 7) //ignore black background 
                        {
                            distArray[i + width] = j;
                            pointArray[i + width] = new Point(tempX, tempY);

                            sumDist += j;
                            sumDistSize++;
                            break;
                        }
                    }
                }

            }

            if (sumDistSize > 0)
            {
                averageDist = sumDist / sumDistSize;

                float coef = 1.4f;
                float limit = averageDist * coef; // the limit distance for what we accept as a finger

                //for (int i = -width; i < width; i++) // a perpendicular line at the limit distance 
                //{
                //    index = (int)palmPosX + -(y * i / divi) + (x * (int)limit / divj) + ((x * i / divi) + (y * (int)limit / divj) + (int)palmPosY) * this.displayWidth;
                //    if (index > 0 & index < range)
                //        this.depthPixels[index] = 6; //color it red
                //}

                for (int i = 0; i < 3; i++) //3 since the thumb has a different angle and the pinky is to short
                {

                    int max = 0;
                    int maxIndex = 0;
                    for (int j = 0; j < distArray.Length; j++)
                    { //finds the furthest tip
                        if (distArray[j] > max)
                        {
                            max = distArray[j];
                            maxIndex = j;
                        }
                    }

                    if ((float)max > limit)
                    {
                        drawFingerTip(range, pointArray[maxIndex], 5); // we found a finger
                        coef -= 0.1f; //lower the bar for the next finger while still keeping the bar high when no finger is present
                        limit = averageDist * coef;
                    }
                    else
                    {
                        break; //no fingers left
                    }

                    int tipSize = 7;
                    int start = (maxIndex > width) ? width : maxIndex; //remove values from the middle to the detected tip + the size of the tip
                    int end = (maxIndex > width) ? maxIndex : width;

                    for (int j = start - tipSize; j <= end + tipSize; j++)
                    {
                        if (j >= 0 & j < distArray.Length)
                        {
                            distArray[j] = 0; //remove values from the finger so we dont mark it twice
                        }

                    }

                }
            }

        }

        /// <summary>
        /// Draws a circle around the tracked finger tip
        /// </summary>   
        /// <param name="range">Size of the DepthFrame image data in pixels</param>
        /// <param name="tip">A point containing the finger tip coordinates</param>
        /// <param name="color">The color of the circle</param>
        private void drawFingerTip(int range, Point tip, byte color)
        {

            int r = 4;
            for (double i = 0; i < 2 * Math.PI; i += 0.1) // a circle around the furthest finger tip
            {
                int index = (int)(tip.X + Math.Cos(i) * r) + (int)(tip.Y + Math.Sin(i) * r) * this.displayWidth; //three calls makes the circle thicker
                if (index > 0 & index < range)
                    this.depthPixels[index] = color;
                index = (int)(tip.X + Math.Cos(i) * r) + 1 + (int)(tip.Y + Math.Sin(i) * r) * this.displayWidth;
                if (index > 0 & index < range)
                    this.depthPixels[index] = color;
                index = (int)(tip.X + Math.Cos(i) * r) + (int)(tip.Y + Math.Sin(i) * r + 1) * this.displayWidth;
                if (index > 0 & index < range)
                    this.depthPixels[index] = color;
            }


        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="frameDataLength">Size of the DepthFrame image data in pixels</param>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>       
        /// <param name="palmPosX">The x coordinate of the palm</param>
        /// <param name="palmPosY">The y coordinate of the palm</param>
        private unsafe void mapHand(int frameDataLength, IntPtr depthFrameData, int palmPosX, int palmPosY)
        {

            int index = (palmPosX) + (palmPosY) * this.displayWidth;
            //make sure we don't go out of bounds
            if (index >= frameDataLength)
            {
                return;
            }

            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            ushort palmDepth = frameData[index];

            

            // defines the size of the rectangle where we search
            int size = 60;

            //used to make the edge of the rectangle look less sharp
            Random rnd = new Random();

            // the bounds of the rectangle
            int xStart = (palmPosX - size) > 0 ? (palmPosX - size) : 0;
            int yStart = (palmPosY - size) > 0 ? (palmPosY - size) : 0;
            int xEnd = (xStart + size * 2) <= this.displayWidth ? xStart + size * 2 : this.displayWidth; // size *2 since we go from -size to size when we draw the rectangle
            int yEnd = (yStart + size * 1.5) <= this.displayHeight ? (int)(yStart + size * 1.5) : this.displayHeight; // 1.5 since we dont want to go to far below the hand
            for (int x = xStart; x <= xEnd; x++)
            {
                int randomInt = rnd.Next(1, 10);
                for (int y = yStart; y <= yEnd + randomInt; y++)
                {
                    int i = x + y * this.displayWidth;
                    if (i < frameDataLength)
                    {
                        // Get the depth for this pixel
                        ushort depth = frameData[i];

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
                            default:  // to far away, make black
                                this.depthPixels[i] = 7;
                                break;
                        }


                    }
                }
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

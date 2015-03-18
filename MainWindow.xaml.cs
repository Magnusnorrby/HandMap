
namespace Microsoft.Samples.Kinect.DepthBasics
{
    using System;
    using System.ComponentModel;
    using System.Collections.Generic;
    using System.Diagnostics;
    using System.Globalization;
    using System.IO;
    using System.Runtime.InteropServices;
    using System.Windows;
    using System.Windows.Forms;
    using System.Windows.Input;
    using System.Windows.Media;
    using System.Windows.Media.Imaging;
    using Microsoft.Kinect;

    /// <summary>
    /// Interaction logic for MainWindow
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {

        /// <summary>
        /// Allows the system to simulate mouse events
        /// </summary>
        [DllImport("user32.dll")]
        static extern void mouse_event(int dwFlags, int dx, int dy, int cButtons, int dwExtraInfo);

        /// <summary>
        /// Allows the system to simulate keyboard events
        /// </summary>
        [DllImport("user32.dll")]
        static extern void keybd_event(byte bVk, byte bScan, int dwFlaags, int dwExtraInfo);

        /// <summary>
        /// Mouse variables
        /// </summary>
        private const int MOUSEEVENT_LEFTDOWN = 0x02;
        private const int MOUSEEVENT_LEFTUP = 0x04;
        private const int MOUSEEVENT_RIGHTDOWN = 0x08;
        private const int MOUSEEVENT_RIGHTUP = 0x10;
        private const int MOUSEEVENT_MOUSEWHEEL = 0x0800;

        /// <summary>
        /// Keyboard variables
        /// </summary>
        private const int KEYEVENT_DOWN = 0x0001; // key up
        private const int KEYEVENT_UP = 0x0002;  // key down
        private const int VK_CTR = 0xA2; // CTRL-key

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
        /// Array for the bodies
        /// </summary>
        private Body[] bodies = null;


        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

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
        /// HandState of the right hand
        /// </summary>
        private HandState rightHandState;

        /// <summary>
        /// HandState of the left hand
        /// </summary>
        private HandState leftHandState;

        /// <summary>
        /// The tracking states of the rPalmPos, lPalmPos, rWristPos, lWristPos
        /// </summary>
        private TrackingState[] trackingStates = new TrackingState[4];

        /// <summary>
        /// TackingId for the person driving the app
        /// </summary>
        private long driver = 0;

        /// <summary>
        /// The old position of a finger tip. Used to evaluate if the direction of the finger
        /// </summary>
        private Point oldFingerTipPosition;

        /// <summary>
        /// The depth of a finger tip. Used to evaluate if the finger is pressing something
        /// </summary>
        private ushort fingerTipDepth = 0;

        /// <summary>
        /// Keeps track of how many frames a finger has been pressing something
        /// </summary>
        private int frameCount = 0;


        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {

            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // get the coordinate mapper
            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

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
            colorList.Add(Color.FromArgb(200, 200, 100, 0));
            colorList.Add(Color.FromArgb(200, 230, 70, 0));
            colorList.Add(Color.FromArgb(255, 255, 0, 0));
            colorList.Add(Color.FromArgb(0, 0, 0, 0));



            BitmapPalette bp = new BitmapPalette(colorList);

            // create the bitmap to display            
            this.depthBitmap = new WriteableBitmap(this.depthFrameDescription.Width, this.depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Indexed8, bp);

            // open the reader for the body frames
            this.bodyFrameReader = this.kinectSensor.BodyFrameSource.OpenReader();

            // get FrameDescription from the BodyFrameSource
            this.bodyFrameReader.FrameArrived += this.Reader_BodyFrameArrived;


            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
            this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                            : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model 
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();



        }

        /// <summary>
        /// INotifyPropertyChangedPropertyChanged event to allow window controls to bind to changeable data
        /// </summary>
        public event PropertyChangedEventHandler PropertyChanged;


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

        /// <summary>
        /// Launches PyMol
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void Launch_PyMol(object sender, RoutedEventArgs e)
        {
            System.Diagnostics.Process.Start("C:/Users/kptg125/Desktop/pymol/PyMOL/PymolWin.exe", "C:/Users/kptg125/Desktop/pymol/PyMOL/2DHB.pdb");
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
                bool driverExists = false;
                foreach (Body body in this.bodies)
                {

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

                            foreach (JointType jointType in joints.Keys)
                            {

                                TrackingState trackingState = joints[jointType].TrackingState;

                                // sometimes the depth(Z) of an inferred joint may show as negative
                                // clamp down to 0.1f to prevent coordinatemapper from returning (-Infinity, -Infinity)
                                CameraSpacePoint position = joints[jointType].Position;
                                if (position.Z < 0)
                                {
                                    position.Z = 0.1f;
                                }

                                // convert the joint points to depth (display) space
                                DepthSpacePoint depthSpacePoint = this.coordinateMapper.MapCameraPointToDepthSpace(position);

                                switch (jointType)
                                {
                                    //Try to find the deepth coordianates of the hand
                                    case JointType.HandRight:
                                        //update the tracking state 
                                        trackingStates[0] = trackingState;
                                        if (trackingState == TrackingState.Tracked) //requiring the joint to be tracked saves jumpy behavior when Kinect guesses
                                        {
                                            rPalmPos = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                            rightHandState = body.HandRightState;
                                        }

                                        break;


                                    case JointType.HandLeft:
                                        trackingStates[1] = trackingState;
                                        if (trackingState == TrackingState.Tracked)
                                        {
                                            lPalmPos = new Point(depthSpacePoint.X, depthSpacePoint.Y);
                                            leftHandState = body.HandLeftState;

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

                        }

                        if (!driverExists)  // no driver exists, reset the driver id so we can find a new driver
                        {
                            driver = 0;
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
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="depthFrameData">Pointer to the DepthFrame image data</param>
        /// <param name="depthFrameDataSize">Size of the DepthFrame image data</param>
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize)
        {

            int frameDataLength = (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel);

            // sets all pixels to a default black
            for (int i = 0; i < frameDataLength; ++i)
            {
                this.depthPixels[i] = 7;
            }

            // depth frame data is a 16 bit value
            ushort* frameData = (ushort*)depthFrameData;

            bool mouseControlled = false; //used to decide which hand that controls the mouse
            bool closed = false; //used to track if the hand is closed

            int palmPosX;
            int palmPosY;
            int wristPosX;
            int wristPosY;
            HandState hs;

            for (int i = 0; i <= 1; i++)
            {
                if (i == 0) // right hand
                {
                    palmPosX = (int)rPalmPos.X;
                    palmPosY = (int)rPalmPos.Y;
                    wristPosX = (int)rWristPos.X;
                    wristPosY = (int)rWristPos.Y;
                    hs = rightHandState;
                }
                else //left hand
                {
                    palmPosX = (int)lPalmPos.X;
                    palmPosY = (int)lPalmPos.Y;
                    wristPosX = (int)lWristPos.X;
                    wristPosY = (int)lWristPos.Y;
                    hs = leftHandState;
                }
                if (trackingStates[i] == TrackingState.Tracked && trackingStates[i + 2] == TrackingState.Tracked) // we are tracking both the palm and the wrist
                {
                    mapHand(frameDataLength, frameData, palmPosX, palmPosY);

                    if (hs == HandState.Closed) //only look for fingers if the hand isn't closed
                    {
                        closed = true;
                    }
                    else
                    {
                        int maxFingers = (hs == HandState.Lasso) ? 1 : 4;
                        Point thumb = findThumb(palmPosX, palmPosY, wristPosX, wristPosY, frameDataLength, 1 + i * -2, -1 + i * 2); // 1,-1 for right -1,1 for left
                        List<Point> fingers = findFingers(palmPosX, palmPosY, wristPosX, wristPosY, frameDataLength, thumb, maxFingers);

                        if (maxFingers == 1 & fingers.Count == 1 & !mouseControlled) //only let one hand control the mouse movements
                        {
                            mouseControlled = true;

                            ushort tempDepth = frameData[(int)(fingers[0].X + fingers[0].Y * this.depthFrameDescription.Width)];
                            if (tempDepth >= fingerTipDepth) //if the depth is increasing (distance from sensor increasing) we reset the depth and counter
                            {
                                fingerTipDepth = tempDepth;
                                frameCount = 0;
                            }
                            else
                            {
                                frameCount++;
                            }


                            double dist = distance((int)fingers[0].X, (int)fingers[0].Y, (int)oldFingerTipPosition.X, (int)oldFingerTipPosition.Y);
                            if (dist < 20 & dist > 3) //make sure we don't jump to far and to often
                            {
                                System.Drawing.Point point = System.Windows.Forms.Cursor.Position;
                                point.X += (int)(fingers[0].X - oldFingerTipPosition.X) * 5;
                                point.Y += (int)(fingers[0].Y - oldFingerTipPosition.Y) * 5;
                                System.Windows.Forms.Cursor.Position = point;
                            }


                            oldFingerTipPosition = fingers[0];

                            //finger has psuhed for 60 frames, click the position
                            if (frameCount == 60) //important not to reset since we only want one click
                            {
                                Console.Out.Write("\nClick");
                                mouse_event(MOUSEEVENT_LEFTDOWN | MOUSEEVENT_LEFTUP, System.Windows.Forms.Cursor.Position.X, System.Windows.Forms.Cursor.Position.Y, 0, 0);
                            }

                        }
                        else // reset the push counter
                        {
                            fingerTipDepth = 0;
                            frameCount = 0;
                        }
                    }

                }
                if (closed & Mouse.LeftButton == MouseButtonState.Released)
                {
                    Console.Out.Write("\nclicking "+Mouse.LeftButton.ToString());
                    mouse_event(MOUSEEVENT_LEFTDOWN, System.Windows.Forms.Cursor.Position.X, System.Windows.Forms.Cursor.Position.Y, 0, 0);
                }
                else if (!closed & Mouse.LeftButton == MouseButtonState.Pressed)
                {
                    Console.Out.Write("\nup");
                    mouse_event(MOUSEEVENT_LEFTUP, System.Windows.Forms.Cursor.Position.X, System.Windows.Forms.Cursor.Position.Y, 0, 0);
                }
            }


        }

        /// <summary>
        /// Directly accesses the underlying image buffer of the DepthFrame to 
        /// create a displayable bitmap.
        /// This function requires the /unsafe compiler option as we make use of direct
        /// access to the native memory pointed to by the depthFrameData pointer.
        /// </summary>
        /// <param name="frameDataLength">Size of the DepthFrame image data in pixels</param>
        /// <param name="frameData">The DepthFrame image data</param>       
        /// <param name="palmPosX">The x coordinate of the palm</param>
        /// <param name="palmPosY">The y coordinate of the palm</param>
        private unsafe void mapHand(int frameDataLength, ushort* frameData, int palmPosX, int palmPosY)
        {

            int index = palmPosX + palmPosY * this.depthFrameDescription.Width;
            //make sure we don't go out of bounds
            if (index >= frameDataLength)
            {
                return;
            }


            ushort palmDepth = frameData[index];

            // defines the size of the rectangle where we search
            int size = 60;

            //used to make the edge of the rectangle look less sharp
            Random rnd = new Random();

            // the bounds of the rectangle
            int xStart = Math.Max((palmPosX - size), 0);
            int yStart = Math.Max((palmPosY - size), 0);
            int xEnd = Math.Min((xStart + size * 2), this.depthFrameDescription.Width); // size *2 since we go from -size to size when we draw the rectangle
            int yEnd = Math.Min((int)(yStart + size * 1.5), this.depthFrameDescription.Height); // 1.5 since we dont want to go to far below the hand
            for (int x = xStart; x <= xEnd; x++)
            {
                int randomInt = rnd.Next(1, 10);
                for (int y = yStart; y <= yEnd + randomInt; y++)
                {
                    int i = x + y * this.depthFrameDescription.Width;
                    if (i < frameDataLength)
                    {
                        // Get the depth for this pixel
                        ushort depth = frameData[i];

                        //only colors pixels that are close in depth to our palm depth 
                        switch ((palmDepth - depth) / 25)
                        {
                            case -1: // Behind palm depth, make blue
                                this.depthPixels[i] = 0;
                                break;
                            case 0: // On palm depth, make green
                                this.depthPixels[i] = 1;
                                break;
                            case 1: // further forward from palm depth, make increasingly red
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
        /// Analyses the hand to find the thumb
        /// </summary>   
        /// <param name="palmPosX">The x coordinate of the palm</param>
        /// <param name="palmPosY">The y coordinate of the palm</param>
        /// <param name="wristPosX">The x coordinate of the wrist</param>
        /// <param name="wristPosY">The y coordinate of the wrist</param>
        /// <param name="frameDataLength">Size of the DepthFrame image data in pixels</param>
        /// <param name="d1">The direction variable for the thumb</param> //We presume that the right thumb is to the left of the hand and vice versa 
        /// <param name="d2">The direction variable the thumb</param>
        private Point findThumb(int palmPosX, int palmPosY, int wristPosX, int wristPosY, int range, int d1, int d2)
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
                    index = tempX + tempY * this.depthFrameDescription.Width;
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
                if (max > 1.7 * sumDist / sumDistSize) // the thumb is stretched far enough
                {
                    drawFingerTip(range, thumb, 0); // 0 draws the thumb circle blue
                    drawLine((int)thumb.X, (int)thumb.Y, palmPosX, palmPosY, range);
                }

            }

            return thumb;

        }

        /// <summary>
        /// Analyses the hand to find the fingers
        /// </summary>   
        /// <param name="palmPosX">The x coordinate of the palm</param>
        /// <param name="palmPosY">The y coordinate of the palm</param>
        /// <param name="wristPosX">The x coordinate of the wrist</param>
        /// <param name="wristPosY">The y coordinate of the wrist</param>
        /// <param name="frameDataLength">Size of the DepthFrame image data in pixels</param>
        /// <param name="thumb">The position of the thumb</param>
        /// <param name="maxFingers">The number of figners we are looking for</param>
        private List<Point> findFingers(int palmPosX, int palmPosY, int wristPosX, int wristPosY, int range, Point thumb, int maxFingers)
        {
            int x = palmPosX - wristPosX;
            int y = palmPosY - wristPosY;
            int width = 80;
            // the ratio between width and divi,divj decides the distance between searched pixels
            int divi = 40;
            int divj = 80;

            List<Point> fingers = new List<Point>();

            double sumDist = 0;
            double averageDist = 0;
            int sumDistSize = 0; //all columns wont hit since we make it extra wide

            double[] distArray = new double[width * 2]; // ranges from -width to +width
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
                // a box from the perpendicular line and forward, attempting to box in the fingers
                for (int j = width * 3; j > 0; j--) // should over shoot the hand 
                {
                    tempX = palmPosX + -(y * i / divi) + (x * j / divj);
                    tempY = ((x * i / divi) + (y * j / divj) + palmPosY);

                    if (distance(tempX, tempY, (int)thumb.X, (int)thumb.Y) < 20) //makes sure that we dont mark the thumb again
                        break;

                    index = tempX + tempY * this.depthFrameDescription.Width;
                    if (index > 0 & index < range)
                    {
                        if (this.depthPixels[index] != 7) //ignore black background 
                        {
                            double dist = distance(palmPosX, palmPosY, tempX, tempY); //take angle into account when calculating the distance
                            distArray[i + width] = dist;
                            pointArray[i + width] = new Point(tempX, tempY);
                            sumDist += dist;
                            sumDistSize++;
                            break;
                        }
                    }
                }

            }

            if (sumDistSize > 0)
            {
                averageDist = sumDist / sumDistSize;

                double coef = 1.2;
                double limit = averageDist * coef; // the limit distance for what we accept as a finger


                //for (int i = -width; i < width; i++) // a perpendicular line at the limit distance 
                //{
                //    index = (int)palmPosX + -(y * i / divi) + (x * (int)limit / divj) + ((x * i / divi) + (y * (int)limit / divj) + (int)palmPosY) * this.depthFrameDescription.Width;
                //    if (index > 0 & index < range)
                //        this.depthPixels[index] = 6; //color it red
                //}

                for (int i = 0; i < maxFingers; i++)
                {

                    double max = 0;
                    int maxIndex = 0;
                    for (int j = 0; j < distArray.Length; j++) //finds the furthest tip
                    {
                        if (distArray[j] > max)
                        {
                            max = distArray[j];
                            maxIndex = j;
                        }
                    }

                    if (max > limit)
                    {
                        drawFingerTip(range, pointArray[maxIndex], 6); // we found a finger
                        drawLine((int)pointArray[maxIndex].X, (int)pointArray[maxIndex].Y, palmPosX, palmPosY, range);

                        fingers.Add(pointArray[maxIndex]);

                        coef -= 0.05; //lower the bar for the next finger while still keeping the bar high when no finger is present
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
            return fingers;

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
                int index = (int)(tip.X + Math.Cos(i) * r) + (int)(tip.Y + Math.Sin(i) * r) * this.depthFrameDescription.Width; //three calls makes the circle thicker
                if (index > 0 & index < range)
                    this.depthPixels[index] = color;
                index = (int)(tip.X + Math.Cos(i) * r) + 1 + (int)(tip.Y + Math.Sin(i) * r) * this.depthFrameDescription.Width;
                if (index > 0 & index < range)
                    this.depthPixels[index] = color;
                index = (int)(tip.X + Math.Cos(i) * r) + (int)(tip.Y + Math.Sin(i) * r + 1) * this.depthFrameDescription.Width;
                if (index > 0 & index < range)
                    this.depthPixels[index] = color;
            }


        }

        /// <summary>
        /// Draws a red line between two points
        /// <param name="x1">The X coordinate of the first point</param>
        /// <param name="y1">The Y coordinate of the first point</param>
        /// <param name="x2">The X coordinate of the second point</param>
        /// <param name="y2">The Y coordinate of the second point</param>
        /// <param name="range">The upper bound of the bitMap</param>
        /// </summary>
        private void drawLine(int x1, int y1, int x2, int y2, int range)
        {
            int x = x1 - x2;
            int y = y1 - y2;

            for (int i = 0; i < 20; i++)
            {
                int index = x2 + x * i / 20 + (y2 + y * i / 20) * this.depthFrameDescription.Width;
                if (index > 0 & index < range)
                    this.depthPixels[index] = 6; //color it red
            }
        }


        /// <summary>
        /// Calculates the distance between two 2D points
        /// <param name="x1">The X coordinate of the first point</param>
        /// <param name="y1">The Y coordinate of the first point</param>
        /// <param name="x2">The X coordinate of the second point</param>
        /// <param name="y2">The Y coordinate of the second point</param>
        /// </summary>
        private double distance(int x1, int y1, int x2, int y2)
        {
            return Math.Sqrt(Math.Pow(x1 - x2, 2) + Math.Pow(y1 - y2, 2));
        }



    }

}

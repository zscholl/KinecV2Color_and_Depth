using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.ComponentModel;
using System.Globalization;
using System.IO;
using System.Diagnostics;
using Microsoft.Kinect;
using System.Runtime.ExceptionServices;


namespace ColorAndDepthKinectV2
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window, INotifyPropertyChanged
    {
        /// <summary>
        /// Active Kinect sensor
        /// </summary>
        private KinectSensor kinectSensor = null;

        private FrameDescription colorFrameDescription = null;

        private readonly int bytesPerPixel = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;

        private unsafe ushort* frameData = null;
    
   
        private byte[] depthPixels = null;
        /// <summary>
        /// The size in bytes of the bitmap back buffer
        /// </summary>
        private uint bitmapBackBufferSize = 0;

        /// <summary>
        /// Coordinate mapper to map one type of point to another
        /// </summary>
        private CoordinateMapper coordinateMapper = null;

        /// <summary>
        /// Intermediate storage for the color to depth mapping
        /// </summary>
        private DepthSpacePoint[] colorMappedToDepthPoints = null;

        private ColorSpacePoint[] depthMappedToColorPoints = null;

        private const int MapDepthToByte = 8000 / 256;

        private ushort[] depthData = null;
        /// <summary>
        /// Reader for color frames
        /// </summary>
        private ColorFrameReader colorFrameReader = null;

        private DepthFrameReader depthFrameReader = null;

        FrameDescription depthFrameDescription = null;
        /// <summary>
        /// Bitmap to display
        /// </summary>
        private WriteableBitmap colorBitmap = null;

        private WriteableBitmap depthBitmap = null;
        /// <summary>
        /// Current status text to display
        /// </summary>
        private string statusText = null;

        /// <summary>
        /// Initializes a new instance of the MainWindow class.
        /// </summary>
        public MainWindow()
        {
            // get the kinectSensor object
            this.kinectSensor = KinectSensor.GetDefault();

            // open the reader for the color frames
            this.colorFrameReader = this.kinectSensor.ColorFrameSource.OpenReader();

            this.depthFrameReader = this.kinectSensor.DepthFrameSource.OpenReader();
            
            // wire handler for frame arrival
            this.colorFrameReader.FrameArrived += this.Reader_ColorFrameArrived;

            this.depthFrameReader.FrameArrived += this.Reader_DepthFrameArrived;
            // create the colorFrameDescription from the ColorFrameSource using Bgra format
            this.colorFrameDescription = this.kinectSensor.ColorFrameSource.CreateFrameDescription(ColorImageFormat.Bgra);

            this.coordinateMapper = this.kinectSensor.CoordinateMapper;

            this.depthFrameDescription = this.kinectSensor.DepthFrameSource.FrameDescription;


            this.colorMappedToDepthPoints = new DepthSpacePoint[colorFrameDescription.Width * colorFrameDescription.Height];

            this.depthMappedToColorPoints = new ColorSpacePoint[depthFrameDescription.Width * depthFrameDescription.Height];

          
            this.depthPixels = new byte[depthFrameDescription.Width * depthFrameDescription.Height];
            
            // create the bitmap to display
            this.colorBitmap = new WriteableBitmap(colorFrameDescription.Width, colorFrameDescription.Height, 96.0, 96.0, PixelFormats.Bgr32, null);
            this.depthBitmap = new WriteableBitmap(depthFrameDescription.Width, depthFrameDescription.Height, 96.0, 96.0, PixelFormats.Gray8, null);

            this.bitmapBackBufferSize = (uint)((this.colorBitmap.BackBufferStride * (this.colorBitmap.PixelHeight - 1)) + (this.colorBitmap.PixelWidth * this.bytesPerPixel));
            
            // set IsAvailableChanged event notifier
            this.kinectSensor.IsAvailableChanged += this.Sensor_IsAvailableChanged;

            // open the sensor
            this.kinectSensor.Open();

            // set the status text
           // this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                         //   : Properties.Resources.NoSensorStatusText;

            // use the window object as the view model in this simple example
            this.DataContext = this;

            // initialize the components (controls) of the window
            this.InitializeComponent();
            
            //Coordinate mapping
               

        
        }

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
                            ushort maxDepth = ushort.MaxValue;

                            // If you wish to filter by reliable depth distance, uncomment the following line:
                            //// maxDepth = depthFrame.DepthMaxReliableDistance

                            this.ProcessDepthFrameData(depthBuffer.UnderlyingBuffer, depthBuffer.Size, depthFrame.DepthMinReliableDistance, maxDepth);
                            depthFrameProcessed = true;
                            
                        }
                        
                    }
                    using (KinectBuffer depthFrameData = depthFrame.LockImageBuffer())
                    {
                        this.depthData = new ushort[this.depthFrameDescription.Width * depthFrameDescription.Height];
                        depthFrame.CopyFrameDataToArray(depthData);

                        this.coordinateMapper.MapDepthFrameToColorSpace(depthData, depthMappedToColorPoints);
                        
                        this.coordinateMapper.MapColorFrameToDepthSpaceUsingIntPtr(
                            depthFrameData.UnderlyingBuffer,
                            depthFrameData.Size,
                            this.colorMappedToDepthPoints);
                    }

                }
            }

            if (depthFrameProcessed)
            {
                this.RenderDepthPixels();
            }
        }
        private void RenderDepthPixels()
        {
            this.depthBitmap.WritePixels(
                new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.depthPixels,
                this.depthBitmap.PixelWidth,
                0);
        }
        private unsafe void ProcessDepthFrameData(IntPtr depthFrameData, uint depthFrameDataSize, ushort minDepth, ushort maxDepth)
        {
          
            
            // depth frame data is a 16 bit value
            this.frameData = (ushort*)depthFrameData;

            

            // convert depth to a visual representation
            for (int i = 0; i < (int)(depthFrameDataSize / this.depthFrameDescription.BytesPerPixel); ++i)
            {
                // Get the depth for this pixel
                ushort depth = frameData[i];

                // To convert to a byte, we're mapping the depth value to the byte range.
                // Values outside the reliable depth range are mapped to 0 (black).
                this.depthPixels[i] = (byte)(depth >= minDepth && depth <= maxDepth ? (depth / MapDepthToByte) : 0);
            }
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
        /// Execute shutdown tasks
        /// </summary>
        /// <param name="sender">object sending the event</param>
        /// <param name="e">event arguments</param>
        private void MainWindow_Closing(object sender, CancelEventArgs e)
        {
            if (this.colorFrameReader != null)
            {
                // ColorFrameReder is IDisposable
                this.colorFrameReader.Dispose();
                this.colorFrameReader = null;
            }
            if (this.depthFrameReader != null)
            {
                this.depthFrameReader.Dispose();
                this.depthFrameReader = null;
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
        [HandleProcessCorruptedStateExceptionsAttribute()]
        private unsafe void Reader_ColorFrameArrived(object sender, ColorFrameArrivedEventArgs e)
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
                        byte[] colorData = new byte[this.colorFrameDescription.Width * this.colorFrameDescription.Height * this.bytesPerPixel];

                        
                           colorFrame.CopyConvertedFrameDataToArray(colorData, ColorImageFormat.Bgra);

                        
                          /**
                           * Nothing works, not sure if it will work, I think the code is too inefficient to be executed at a rate to refresh the
                           * frame properly. Although I am leary of this assessment because the coordinate mapping sample has a for loop that 
                           * executes the same number of times and includes 4 conditionals. This code has no problem refreshing the image.
                           * 
                           * Note: Even when part of the image was displaying and being processed, the offset between the depth and color
                           * pixels was not being resolved.
                           **/
                      
                        // verify data and write the new color frame data to the display bitmap
                       // if ((colorFrameDescription.Width == this.colorBitmap.PixelWidth) && (colorFrameDescription.Height == this.colorBitmap.PixelHeight))
                       // {
                       //     colorFrame.CopyConvertedFrameDataToIntPtr(
                       //        this.colorBitmap.BackBuffer,
                       //         (uint)(colorFrameDescription.Width * colorFrameDescription.Height * 4),
                       //        ColorImageFormat.Bgra);

                            //this.colorBitmap.CopyPixels(colorData, this.colorBitmap.BackBufferStride, 0);
                          
                                for (int i = 0; i < colorMappedToDepthPoints.Length/3 ; i++)
                                {
                                    DepthSpacePoint dPoint = this.colorMappedToDepthPoints[i];

                                    if (float.IsNegativeInfinity((float)dPoint.X))
                                    {
                                        colorData[i*4] = 0;
                                        // Console.Write(i.ToString() + ",");

                                    }
                                }
                               
                                this.colorBitmap.WritePixels(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight), colorData, this.colorBitmap.BackBufferStride, 0);
                           // this.colorBitmap.AddDirtyRect(new Int32Rect(0, 0, this.colorBitmap.PixelWidth, this.colorBitmap.PixelHeight));
                      //  }


                        this.colorBitmap.Unlock();
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
           // this.StatusText = this.kinectSensor.IsAvailable ? Properties.Resources.RunningStatusText
                                                         //   : Properties.Resources.SensorNotAvailableStatusText;
        }
        [HandleProcessCorruptedStateExceptionsAttribute()]
        private unsafe void color_MouseLeftButtonDown(object sender, MouseEventArgs e)
        {
            if (sender != null)
            {
                Point cPoint = e.GetPosition(colorImage);
                DistanceText.Text = "x: " + cPoint.X + " y: " + cPoint.Y;
                DepthSpacePoint dPoint = this.colorMappedToDepthPoints[(int)cPoint.X * 3 + (int)cPoint.Y * 3 * (this.colorFrameDescription.Width)];
                try
                { 
                    Double depth = this.frameData[(int)(dPoint.X) + (int)(dPoint.Y) * depthFrameDescription.Width];
                    depth = depth / 1000.00;
                    DistanceText.Text = depth.ToString() + " meters";
                }
                //catches memory exception to avoid timing out the application
                catch (AccessViolationException ex) { }

                Ellipse ellipse = new Ellipse();
                ellipse.Width = 5;
                ellipse.Height = 5;
                ellipse.Fill = System.Windows.Media.Brushes.Orange;
                Canvas.SetLeft(ellipse, cPoint.X);
                Canvas.SetTop(ellipse, cPoint.Y);

                cameraCanvas.Children.Add(ellipse);
            }
            
        }

        [HandleProcessCorruptedStateExceptionsAttribute()]
        private unsafe void depth_MouseLeftButtonDown(object sender, MouseEventArgs e) 
        {
            if (sender != null)
            {
                Point dPoint = e.GetPosition(depthImage);

                try
                {
                    Double depth = this.frameData[(int)(dPoint.X) + (int)(dPoint.Y) * depthFrameDescription.Width];
                    depth = depth / 1000.00;
                    DistanceText.Text = depth.ToString() + " meters";
                }
                //catches memory exception to avoid timing out the application
                catch (AccessViolationException ex) { }

                Ellipse ellipse = new Ellipse();
                ellipse.Width = 5;
                ellipse.Height = 5;
                ellipse.Fill = System.Windows.Media.Brushes.Orange;
                Canvas.SetLeft(ellipse, dPoint.X);
                Canvas.SetTop(ellipse, dPoint.Y);

                depthCanvas.Children.Add(ellipse);
            }
        }
    }
}

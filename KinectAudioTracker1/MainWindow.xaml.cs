using System;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace KinectAudioTracker
{
    using System.Drawing;
    using System.Drawing.Imaging;
    using System.Speech.Recognition;
    using System.Threading;
    using System.Windows.Threading;
    using System.Collections.Generic;
    using System.IO;
    using Microsoft.Kinect;

    using Emgu.CV;
    using Emgu.CV.Util;
    using Emgu.CV.Structure;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor kinect;

        private DepthImageFrame currentDepthFrame;
        private WriteableBitmap depthBitmap;

        private ColorImageFrame currentColorFrame;
        private WriteableBitmap colorBitmap;

        // color divisors for tinting depth pixels
        private static readonly int[] IntensityShiftByPlayerR = { 1, 2, 0, 2, 0, 0, 2, 0 };
        private static readonly int[] IntensityShiftByPlayerG = { 1, 2, 2, 0, 2, 0, 0, 1 };
        private static readonly int[] IntensityShiftByPlayerB = { 1, 0, 2, 2, 0, 2, 0, 2 };

        private double soundSourceAngle;
        private double soundSourceConfidence;
        private readonly double confidenceCutoff = 0.0;
        private bool audioOn;

        private DispatcherTimer readyTimer;
        private int tickCount;

        private PlayerLocation playerLocations;

        private HaarCascade haar;

        public MainWindow()
        {
            InitializeComponent();
        }

        /// <summary>
        /// Initializes the kinect.
        /// </summary>
        private void initializeKinect()
        {
            // Get the first initialized Kinect sensor.
            this.kinect = KinectSensor.KinectSensors.FirstOrDefault(s => s.Status == KinectStatus.Connected);

            if (this.kinect == null)
            {
                logLine("Kinect initialization failed.");
                return;
            }

            // Enable data streams
            this.kinect.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            this.kinect.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            this.kinect.SkeletonStream.Enable();

            this.depthBitmap = new WriteableBitmap(this.kinect.DepthStream.FrameWidth, this.kinect.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
            depthImage.Source = this.depthBitmap;
            this.currentDepthFrame = null;

            this.colorBitmap = new WriteableBitmap(this.kinect.ColorStream.FrameWidth, this.kinect.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
            colorImage.Source = this.colorBitmap;
            this.currentColorFrame = null;
            
            this.playerLocations = new PlayerLocation();

            //this.kinect.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(kinect_DepthFrameReady);
            this.kinect.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(kinect_ColorFrameReady);

            //this.kinect.AudioSource.AutomaticGainControlEnabled = false;
            //this.kinect.AudioSource.SoundSourceAngleChanged += new EventHandler<SoundSourceAngleChangedEventArgs>(AudioSource_SoundSourceAngleChanged);

            logLine("Loading haar classifier");
            this.haar = new HaarCascade("C:/Emgu/emgucv-windows-x86 2.4.0.1717/opencv/data/haarcascades/haarcascade_frontalface_default.xml");
            logLine("Done loading haar classifier");

            this.kinect.Start();
            logLine("Kinect initialized");
            
            // Wait four seconds after initialization to start audio
            this.readyTimer = new DispatcherTimer();
            this.readyTimer.Tick += new EventHandler(readyTimer_Tick);
            this.readyTimer.Interval = new TimeSpan(0, 0, 1);
            this.readyTimer.Start();

            logLine("Kinect audio initialized.");
        }

        /// <summary>
        /// Handles the Tick event of the kinect initialization timer.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="System.EventArgs"/> instance containing the event data.</param>
        void readyTimer_Tick(object sender, EventArgs e)
        {
            if (this.tickCount == 4)
            {
                startAudio();
                this.readyTimer.Stop();
                this.readyTimer = null;
            }
            else
            {
                logLine((++this.tickCount).ToString());
            }
        }

        /// <summary>
        /// Starts the kinect audio source.
        /// </summary>
        private void startAudio()
        {
            var audioSource = this.kinect.AudioSource;
            audioSource.BeamAngleMode = BeamAngleMode.Adaptive;

            var audioStream = audioSource.Start();
            this.audioOn = true;
        }

        /// <summary>
        /// Stops the kinect audio source.
        /// </summary>
        private void stopAudio()
        {
            this.kinect.AudioSource.Stop();
            this.audioOn = false;
        }

        /// <summary>
        /// Handles the SoundSourceAngleChanged event of the AudioSource control.
        /// </summary>
        /// <param name="sender">The source of the event.</param>
        /// <param name="e">The <see cref="Microsoft.Kinect.SoundSourceAngleChangedEventArgs"/> instance containing the event data.</param>
        void AudioSource_SoundSourceAngleChanged(object sender, SoundSourceAngleChangedEventArgs e)
        {
            this.soundSourceAngle = e.Angle;
            this.soundSourceConfidence = e.ConfidenceLevel;

            // Update the sound source location display
            if (this.soundSourceConfidence > this.confidenceCutoff)
            {
                audioAngleDisplay.rotTx.Angle = -this.soundSourceAngle;
            }
        }

        private static T clamp<T>(T min, T x, T max) where T : System.IComparable<T>
        {
            if (x.CompareTo(min) < 0)
            {
                return min;
            }
            if (x.CompareTo(max) > 0)
            {
                return max;
            }
            return x;
        }

        void kinect_ColorFrameReady(object sender, ColorImageFrameReadyEventArgs e)
        {
            if (this.kinect == null)
                return;

            // If there is a color frame already stored, dispose of it
            if (this.currentColorFrame != null)
            {
                this.currentColorFrame.Dispose();
            }
            var imageFrame = this.currentColorFrame = e.OpenColorImageFrame();

            if (imageFrame != null)
            {
                var colorPixels = new byte[this.kinect.ColorStream.FramePixelDataLength];
                imageFrame.CopyPixelDataTo(colorPixels);

                // Draw the color bitmap to the screen
                colorBitmap.WritePixels(new Int32Rect(0, 0, colorBitmap.PixelWidth, colorBitmap.PixelHeight), colorPixels,
                    colorBitmap.PixelWidth * sizeof(int), 0);

                ThreadPool.QueueUserWorkItem(detectFacesAndLog, this);
            }
        }

        private Rectangle[] detectFaces(Image<Gray, byte> image, HaarCascade haar)
        {
            var faces = haar.Detect(image, 1.2, 2, Emgu.CV.CvEnum.HAAR_DETECTION_TYPE.DO_CANNY_PRUNING, new Size(20, 20), new Size(50, 50));
            IEnumerable<Rectangle> rects =
                from f in faces
                select f.rect;
            return rects.ToArray<Rectangle>();
        }

        private static void detectFacesAndLog(object data)
        {
            var t = (MainWindow)data;
            var faces = t.detectFaces(t.colorFrameToImage(t.currentColorFrame).Convert<Gray, byte>(), t.haar);
            if (faces.Length > 0)
                t.logLine("Face detected.");
        }

        private BitmapImage bitmapToBitmapImage(Bitmap b)
        {
            MemoryStream ms = new MemoryStream();
            b.Save(ms, ImageFormat.Png);
            ms.Position = 0;
            BitmapImage bi = new BitmapImage();
            bi.BeginInit();
            bi.StreamSource = ms;
            bi.EndInit();

            return bi;
        }

        private void kinect_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            return;
            if (this.kinect == null)
                return;

            // If there is a depth frame already stored, dispose of it
            if (this.currentDepthFrame != null)
            {
                this.currentDepthFrame.Dispose();
            }
            this.currentDepthFrame = e.OpenDepthImageFrame();

            if (this.currentDepthFrame != null)
            {
                var coloredPixels = processDepthFrame();
                drawDepthFrame(ref coloredPixels);
                drawPlayers();
            }
        }

        private byte[] processDepthFrame()
        {
            var depthPixels = new short[this.kinect.DepthStream.FramePixelDataLength];
            var coloredPixels = new byte[this.kinect.DepthStream.FramePixelDataLength * sizeof(int)];

            this.currentDepthFrame.CopyPixelDataTo(depthPixels);

            this.playerLocations.clear();

            int colorPixelIndex = 0;
            for (int i = 0; i < depthPixels.Length; ++i)
            {
                int player = depthPixels[i] & DepthImageFrame.PlayerIndexBitmask;
                int depth = depthPixels[i] >> DepthImageFrame.PlayerIndexBitmaskWidth;

                byte intensity = (byte)(~(depth >> 4));

                coloredPixels[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerR[player]);
                coloredPixels[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerG[player]);
                coloredPixels[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerB[player]);
                ++colorPixelIndex;

                // Update the player location average
                if (player != 0)
                {
                    var x = i % depthBitmap.PixelWidth;
                    var y = i / depthBitmap.PixelWidth;
                    playerLocations.addPixelPosition(player, x, y);
                }
            }

            // Update the angle between the player and the kinect
            for (int i = 1; i < 7; ++i)
            {
                var x = playerLocations.getWorldX(i, this.currentDepthFrame);
                var z = playerLocations.getWorldZ(i, this.currentDepthFrame);
                playerLocations.setAngle(i, radToDeg(-Math.Atan(x / z)));
            }

            return coloredPixels;
        }

        private void drawDepthFrame(ref byte[] coloredDepth)
        {
            // Draw the depth bitmap to the screen
            depthBitmap.WritePixels(new Int32Rect(0, 0, depthBitmap.PixelWidth, depthBitmap.PixelHeight), coloredDepth,
                depthBitmap.PixelWidth * sizeof(int), 0);
        }

        private void drawPlayers()
        {
            var talkingPlayer = this.playerLocations.getClosestPlayerByAngle(this.soundSourceAngle, 50.0);

            for (int i = 1; i < 7; ++i)
            {
                if (playerLocations.hasPositionData(i))
                {
                    var x = playerLocations.getPixelX(i);
                    var y = playerLocations.getPixelY(i);

                    if (i == talkingPlayer)
                    {
                        logLine(i.ToString());
                    }
                    else
                    {
                        drawRect(new Rectangle(x, y, 20, 20), ref this.depthBitmap, System.Drawing.Color.Orange);
                    }
                }
            }
        }

        private Image<Bgr, byte> colorFrameToImage(ColorImageFrame frame)
        {
            var colorPixels = new byte[this.kinect.ColorStream.FramePixelDataLength];
            this.currentColorFrame.CopyPixelDataTo(colorPixels);

            var height = frame.Height;
            var width = frame.Width;
            var arrangedPixels = new byte[height, width, 3];

            var pixelCount = 0;
            for (int i = 0; i < colorPixels.Length; i++)
            {
                arrangedPixels[pixelCount / width, pixelCount % width, 0] = colorPixels[i++];
                arrangedPixels[pixelCount / width, pixelCount % width, 1] = colorPixels[i++];
                arrangedPixels[pixelCount / width, pixelCount % width, 2] = colorPixels[i++];
                ++i;
                ++pixelCount;
            }

            return new Image<Bgr, byte>(arrangedPixels);
        }

        private static double radToDeg(double rad)
        {
            return rad * (180 / Math.PI);
        }

        private static void drawRect(Rectangle r, ref WriteableBitmap b, Color c)
        {
            for (int i = r.Y; i < r.Y + r.Height; ++i)
            {
                for (int j = r.X; j < r.X + r.Width; ++j)
                {
                    drawPixel(j, i, ref b, c);
                }
            }
        }

        private static void drawPixel(int x, int y, ref WriteableBitmap b, Color c)
        {
            if (x < 0 || x >= b.PixelWidth || y < 0 || y >= b.PixelHeight)
            {
                return;
            }

            b.Lock();

            unsafe
            {
                int backBuffer = (int)b.BackBuffer;

                backBuffer += y * b.BackBufferStride;
                backBuffer += x * 4;

                // Compute the pixel's color.
                int color_data = c.R << 16; // R
                color_data |= c.G << 8;   // G
                color_data |= c.B << 0;   // B

                *((int*)backBuffer) = color_data;
            }

            b.AddDirtyRect(new Int32Rect(x, y, 1, 1));
            b.Unlock();
        }

        private void uninitializeKinect()
        {
            if (this.kinect != null)
            {
                this.kinect.AudioSource.SoundSourceAngleChanged -= this.AudioSource_SoundSourceAngleChanged;
                this.kinect.AudioSource.Stop();

                this.kinect.DepthFrameReady -= this.kinect_DepthFrameReady;
                this.kinect.ColorFrameReady -= this.kinect_ColorFrameReady;

                this.kinect.Stop();
                this.kinect = null;
            }
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            initializeKinect();
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            uninitializeKinect();
        }

        private void start_Click(object sender, RoutedEventArgs e)
        {
            if (!this.audioOn)
            {
                startAudio();
            }
        }

        private void stop_Click(object sender, RoutedEventArgs e)
        {
            if (this.audioOn)
            {
                stopAudio();
            }
        }

        private void logLine(string line)
        {
            this.listBox1.Items.Add(line);
        }
    }

    public class PlayerLocation
    {
        private Point[] playerLocations;
        private double[] playerAngles;

        private int[] averageCount;
        private int playerCount;

        public PlayerLocation()
        {
            this.playerCount = 7;
            clear();
        }

        public PlayerLocation(int playerCount)
        {
            this.playerCount = playerCount;
            clear();
        }

        public void addPixelPosition(int playerNumber, int x, int y)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();

            ++averageCount[playerNumber];
            playerLocations[playerNumber].X += x;
            playerLocations[playerNumber].Y += y;
        }

        public int getPixelX(int playerNumber)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();

            if (averageCount[playerNumber] == 0)
            {
                return 0;
            }
            else
            {
                return playerLocations[playerNumber].X / averageCount[playerNumber];
            }
        }

        public int getPixelY(int playerNumber)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();

            if (averageCount[playerNumber] == 0)
            {
                return 0;
            }
            else
            {
                return playerLocations[playerNumber].Y / averageCount[playerNumber];
            }
        }

        public double getWorldX(int playerNumber, DepthImageFrame context)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();
            if (context == null)
                throw new ArgumentNullException();

            return context.MapToSkeletonPoint(playerLocations[playerNumber].X, playerLocations[playerNumber].Y).X;
        }

        public double getWorldY(int playerNumber, DepthImageFrame context)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();
            if (context == null)
                throw new ArgumentNullException();

            return context.MapToSkeletonPoint(playerLocations[playerNumber].X, playerLocations[playerNumber].Y).Y;
        }

        public double getWorldZ(int playerNumber, DepthImageFrame context)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();
            if (context == null)
                throw new ArgumentNullException();

            return context.MapToSkeletonPoint(playerLocations[playerNumber].X, playerLocations[playerNumber].Y).Z;
        }

        public void setAngle(int playerNumber, double angle)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();

            playerAngles[playerNumber] = angle;
        }

        public double getAngle(int playerNumber, double angle)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();

            return playerAngles[playerNumber];
        }

        /// <summary>
        /// Finds the player whos angle from the Kinect is the closest to the provided angle and is within the difference cutoff.
        /// </summary>
        /// <param name="angle"></param>
        /// <param name="cutoff"></param>
        /// <returns></returns>
        public int getClosestPlayerByAngle(double angle, double cutoff)
        {
            double min_difference = double.PositiveInfinity;
            int min_player = -1;

            for(int i = 1; i < this.playerAngles.Length; ++i)
            {
                var difference = Math.Abs(this.playerAngles[i] - angle);
                if(difference < cutoff && difference < min_difference)
                {
                    min_difference = difference;
                    min_player = i;
                }
            }

            return min_player;
        }

        public void clear()
        {
            this.playerLocations = new Point[playerCount];
            this.averageCount = new int[playerCount];

            this.playerAngles = new double[playerCount];
            for (int i = 0; i < playerCount; ++i)
            {
                this.playerAngles[i] = -1;
            }
        }

        public bool hasPositionData(int playerNumber)
        {
            if (!checkPlayerNumber(playerNumber))
                return false;

            return averageCount[playerNumber] != 0;
        }

        public bool hasAngleData(int playerNumber)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();

            return playerAngles[playerNumber] != -1;
        }

        private bool checkPlayerNumber(int n)
        {
            if (n <= 0 || n >= this.playerLocations.Length)
            {
                return false;
            }
            return true;
        }
    }
}

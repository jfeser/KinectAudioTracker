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

    public partial class MainWindow : Window
    {
        private KinectSensor kinect;

        private KinectDepthHandler depthHandler;
        private KinectColorHandler colorHandler;
        private KinectSoundHandler soundHandler;
        private WriteableBitmap depthBitmap;

        private WriteableBitmap colorBitmap;


        private bool audioOn;

        private DispatcherTimer readyTimer;
        private int tickCount;


        private Storage dataStorage;

        public MainWindow()
        {
            InitializeComponent();
        }

        private void initializeKinect()
        {
            // Get the first initialized Kinect sensor.
            this.kinect = KinectSensor.KinectSensors.FirstOrDefault(s => s.Status == KinectStatus.Connected);

            if (this.kinect == null)
            {
                logLine("Kinect initialization failed.");
                return;
            }

            this.dataStorage = Storage.Instance;

            // Enable data streams
            this.kinect.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            this.kinect.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            this.kinect.SkeletonStream.Enable();

            this.depthBitmap = new WriteableBitmap(this.kinect.DepthStream.FrameWidth, this.kinect.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
            depthImage.Source = this.depthBitmap;

            depthHandler = new KinectDepthHandler();
            depthHandler.start(this.kinect, Dispatcher.CurrentDispatcher);
            depthHandler.newDepthData += new Action(depthHandler_newDepthData);

            this.colorBitmap = new WriteableBitmap(this.kinect.ColorStream.FrameWidth, this.kinect.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
            colorImage.Source = this.colorBitmap;

            colorHandler = new KinectColorHandler();
            colorHandler.start(this.kinect, Dispatcher.CurrentDispatcher);
            colorHandler.newColorData += new Action(colorHandler_newColorData);

            soundHandler = new KinectSoundHandler();
            soundHandler.start(this.kinect, Dispatcher.CurrentDispatcher);
            soundHandler.newSoundData += new Action(soundHandler_newSoundData);

            this.kinect.AudioSource.AutomaticGainControlEnabled = false;

            this.kinect.Start();
            logLine("Kinect initialized");
            
            // Wait four seconds after initialization to start audio
            this.readyTimer = new DispatcherTimer();
            this.readyTimer.Tick += new EventHandler(readyTimer_Tick);
            this.readyTimer.Interval = new TimeSpan(0, 0, 1);
            this.readyTimer.Start();
        void soundHandler_newSoundData()
        {
            audioAngleDisplay.rotTx.Angle = -dataStorage.soundSourceAngle;
        }

        void colorHandler_newColorData()
        {
            // Draw the color bitmap to the screen
            colorBitmap.WritePixels(new Int32Rect(0, 0, colorBitmap.PixelWidth, colorBitmap.PixelHeight),
                this.dataStorage.colorPixels, colorBitmap.PixelWidth * sizeof(int), 0);

            foreach (var face in dataStorage.faceRectangles)
            {
                for(int i = 0; i < 7; ++i)
                {
                    if (dataStorage.playerLocations.hasPixelPositionData(i))
                    {
                        var overlap = Rectangle.Intersect(face, dataStorage.playerLocations.getPlayerBoundingBox(i));
                        if (overlap != new Rectangle())
                        {
                            Util.drawBoundingBox(face, colorBitmap, System.Drawing.Color.Black, 5);
                        }
                    }
                }
            }
        }

        void depthHandler_newDepthData()
        {
            // Draw the depth bitmap to the screen
            depthBitmap.WritePixels(new Int32Rect(0, 0, depthBitmap.PixelWidth, depthBitmap.PixelHeight), 
                this.dataStorage.depthPixels, depthBitmap.PixelWidth * sizeof(int), 0);

            drawPlayers();
            logLine("Kinect audio initialized.");
        }

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

        private void startAudio()
        {
            var audioSource = this.kinect.AudioSource;
            audioSource.BeamAngleMode = BeamAngleMode.Adaptive;

            var audioStream = audioSource.Start();
            this.audioOn = true;
        }

        private void stopAudio()
        {
            this.kinect.AudioSource.Stop();
            this.audioOn = false;
        }

        {

            {
            }
        private void uninitializeKinect()
        {
            if (this.kinect != null)
            {
                this.soundHandler.stop();
                this.depthHandler.stop();
                this.colorHandler.stop();

                this.kinect.Stop();
                this.kinect = null;
            }
        }

        {

            {
            }

            {
            }
        }

        {
        }

        {
        }


        }

        {


        }

        {








        }

        {
        }

    static class Util
    {
        public static Rectangle[] detectFaces(Image<Gray, byte> image, HaarCascade haar)
        {
            var faces = haar.Detect(image, 1.2, 2, Emgu.CV.CvEnum.HAAR_DETECTION_TYPE.DO_CANNY_PRUNING, new Size(20, 20), new Size(50, 50));
            IEnumerable<Rectangle> rects =
                from f in faces
                select f.rect;
            return rects.ToArray<Rectangle>();
        }

        public static Image<Bgr, byte> colorFrameToImage(ColorImageFrame frame)
        {
            var colorPixels = new byte[frame.PixelDataLength];
            frame.CopyPixelDataTo(colorPixels);

            var height = frame.Height;
            var width = frame.Width;
            var arrangedPixels = new byte[height, width, 3];

            var pixelCount = 0;
            for (int i = 0; i < colorPixels.Length; i++)
            {
                arrangedPixels[pixelCount / width, pixelCount % width, 0] = colorPixels[i++];
                arrangedPixels[pixelCount / width, pixelCount % width, 1] = colorPixels[i++];
                arrangedPixels[pixelCount / width, pixelCount % width, 2] = colorPixels[i++];
                ++pixelCount;
            }

            return new Image<Bgr, byte>(arrangedPixels);
        }

        public static void drawBoundingBox(Rectangle r, WriteableBitmap b, Color c, int pixelWidth)
        {
            Util.drawRect(new Rectangle(r.X, r.Y, r.Width, pixelWidth), b, c);
            Util.drawRect(new Rectangle(r.X, r.Y, pixelWidth, r.Height), b, c);
            Util.drawRect(new Rectangle(r.X, r.Bottom-pixelWidth, r.Width, pixelWidth), b, c);
            Util.drawRect(new Rectangle(r.Right-pixelWidth, r.Y, pixelWidth, r.Height), b, c);
        }

        public static void drawRect(Rectangle r, WriteableBitmap b, Color c)
        {
            if (r.X < 0 || r.Right >= b.PixelWidth || r.Y < 0 || r.Bottom >= b.PixelHeight)
                return;

            b.Lock();

            // Compute the pixel's color.
            int color_data = c.R << 16; // R
            color_data |= c.G << 8;   // G
            color_data |= c.B << 0;   // B

            unsafe
            {
                for (int j = r.Y; j < r.Bottom; ++j)
                {
                    for (int i = r.X; i < r.Right; ++i)
                    {
                        int backBuffer = (int)b.BackBuffer;

                        backBuffer += j * b.BackBufferStride;
                        backBuffer += i * 4;

                        *((int*)backBuffer) = color_data;
                    }
                }
            }

            b.AddDirtyRect(new Int32Rect(r.X, r.Y, r.Width, r.Height));
            b.Unlock();
        }

        public static void drawPixel(int x, int y, ref WriteableBitmap b, Color c)
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

        public static double radToDeg(double rad)

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            return rad * (180 / Math.PI);
            initializeKinect();
        }

        public static T clamp<T>(T min, T x, T max) where T : System.IComparable<T>
        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            if (x.CompareTo(min) < 0)
            uninitializeKinect();
        }

        private void start_Click(object sender, RoutedEventArgs e)
        {
            if (!this.audioOn)
            {
                return min;
                startAudio();
            }
            if (x.CompareTo(max) > 0)
        }

        private void stop_Click(object sender, RoutedEventArgs e)
        {
            if (this.audioOn)
            {
                return max;
                stopAudio();
            }
            return x;
        }

        public static BitmapImage bitmapToBitmapImage(Bitmap b)
        private void logLine(string line)
        {
            MemoryStream ms = new MemoryStream();
            b.Save(ms, ImageFormat.Png);
            ms.Position = 0;
            BitmapImage bi = new BitmapImage();
            bi.BeginInit();
            bi.StreamSource = ms;
            bi.EndInit();

            return bi;
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

    class Storage
    {
        private static Storage instance;
        private static object depthImageLock, colorImageLock, playerLocationLock,
            soundSourceAngleLock, soundSourceConfidenceLock, faceRectangleLock;
        private static byte[] depth, color;
        private static PlayerPosition location;
        private static double angle, confidence;
        private Rectangle[] faces;

        private Storage() { }

        public static Storage Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new Storage();
                    depthImageLock = new Object();
                    colorImageLock = new Object();
                    location = new PlayerPosition();
                    faceRectangleLock = new Object();
                    playerLocationLock = new Object();
                    soundSourceAngleLock = new Object();
                    soundSourceConfidenceLock = new Object();
                }
                return instance;
            }
        }

        public PlayerPosition playerLocations
        {
            get { lock (playerLocationLock) { return location; } }
            set { lock (playerLocationLock) { location = value; } }
        }

        public byte[] depthPixels
        {
            get { lock (depthImageLock) { return depth; } }
            set { lock (depthImageLock) { depth = value; } }
        }

        public byte[] colorPixels
        {
            get { lock (colorImageLock) { return color; } }
            set { lock (colorImageLock) { color = value; } }
        }

        public double soundSourceAngle
        {
            get { lock (soundSourceAngleLock) { return angle; } }
            set { lock (soundSourceAngleLock) { angle = value; } }
        }

        public double soundSourceConfidence
        {
            get { lock (soundSourceConfidenceLock) { return confidence; } }
            set { lock (soundSourceConfidenceLock) { confidence = value; } }
        }

        public Rectangle[] faceRectangles
        {
            get { lock (faceRectangleLock) { return faces; } }
            set { lock (faceRectangleLock) { faces = value; } }
        }
    }

    class KinectDepthHandler
    {
        private Thread worker;
        private bool running = true;
        private AutoResetEvent waiter = new AutoResetEvent(false);
        private Storage dataStorage = Storage.Instance;
        private KinectSensor kinect;
        private Dispatcher uiDispatcher;

        private Action newDepthDataHandler;
        public event Action newDepthData
        {
            add { newDepthDataHandler += value; }
            remove { newDepthDataHandler -= value; }
        }

        public void stop()
        {
            running = false;
            waiter.Set();
        }

        public void start(KinectSensor kinect, Dispatcher uiDispatcher)
        {
            this.kinect = kinect;
            this.uiDispatcher = uiDispatcher;

            System.Console.WriteLine("Starting new thread.");
            worker = new Thread(() => run(this));
            worker.Name = "depth handler";
            worker.Start();
        }

        private static void run(object data)
        {
            var t = (KinectDepthHandler)data;

            DepthImageFrame imageFrame = null;

            t.kinect.DepthFrameReady +=
                (object sender, DepthImageFrameReadyEventArgs e) =>
                {
                    imageFrame = e.OpenDepthImageFrame();
                    t.waiter.Set();
                };

            while (true)
            {
                System.Console.Write("Waiting");
                t.waiter.WaitOne();

                if (!t.running) break;

                if (imageFrame != null)
                {
                    t.dataStorage.depthPixels = processDepthFrame(imageFrame, t.kinect);
                    t.uiDispatcher.BeginInvoke(t.newDepthDataHandler);
                    imageFrame.Dispose();
                    imageFrame = null;
                }
            }
        }

        private static byte[] processDepthFrame(DepthImageFrame imageFrame, KinectSensor kinect)
        {
            // color divisors for tinting depth pixels
            int[] IntensityShiftByPlayerR = { 1, 2, 0, 2, 0, 0, 2, 0 };
            int[] IntensityShiftByPlayerG = { 1, 2, 2, 0, 2, 0, 0, 1 };
            int[] IntensityShiftByPlayerB = { 1, 0, 2, 2, 0, 2, 0, 2 };

            var depthPixels = new short[kinect.DepthStream.FramePixelDataLength];
            var coloredPixels = new byte[kinect.DepthStream.FramePixelDataLength * sizeof(int)];
            var dataStorage = Storage.Instance;

            imageFrame.CopyPixelDataTo(depthPixels);

            dataStorage.playerLocations.clear();

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
                    var x = i % imageFrame.Width;
                    var y = i / imageFrame.Width;
                    dataStorage.playerLocations.addPixelPosition(player, x, y);
                }
            }

            dataStorage.playerLocations.setWorldPositions(imageFrame);
            // Update the angle between the player and the kinect
            for (int i = 1; i < 7; ++i)
            {
                if(dataStorage.playerLocations.hasWorldPositionData(i))
                {
                    var v = dataStorage.playerLocations.getWorldPosition(i);
                    var x = v.X;
                    var z = v.Z;
                    dataStorage.playerLocations.setAngle(i, Util.radToDeg(-Math.Atan(x / z)));
                }
            }

            return coloredPixels;
        }
    }

    class KinectColorHandler
    {
        private Thread worker;
        private bool running = true;
        private AutoResetEvent waiter = new AutoResetEvent(false);
        private Storage dataStorage = Storage.Instance;
        private KinectSensor kinect;
        private Dispatcher uiDispatcher;

        private Action newColorDataHandler;
        public event Action newColorData
        {
            add { newColorDataHandler += value; }
            remove { newColorDataHandler -= value; }
        }

        public void stop()
        {
            running = false;
            waiter.Set();
        }

        public void start(KinectSensor kinect, Dispatcher uiDispatcher)
        {
            this.kinect = kinect;
            this.uiDispatcher = uiDispatcher;

            worker = new Thread(() => run(this));
            worker.Name = "color handler";
            worker.Start();
        }

        private static void run(object data)
        {
            var t = (KinectColorHandler)data;

            ColorImageFrame imageFrame = null;

            t.kinect.ColorFrameReady +=
                (object sender, ColorImageFrameReadyEventArgs e) =>
                {
                    imageFrame = e.OpenColorImageFrame();
                    t.waiter.Set();
                };

            HaarCascade haar = new HaarCascade("C:/Emgu/emgucv-windows-x86 2.4.0.1717/opencv/data/haarcascades/haarcascade_frontalface_default.xml");

            while (true)
            {
                t.waiter.WaitOne();

                if (!t.running) break;

                if (t.dataStorage.colorPixels == null)
                {
                    t.dataStorage.colorPixels = new byte[t.kinect.ColorStream.FramePixelDataLength];
                }

                if (imageFrame != null)
                {
                    imageFrame.CopyPixelDataTo(t.dataStorage.colorPixels);
                    t.dataStorage.faceRectangles = Util.detectFaces(Util.colorFrameToImage(imageFrame).Convert<Gray, byte>(), haar);

                    t.uiDispatcher.BeginInvoke(t.newColorDataHandler);
                    imageFrame.Dispose();
                    imageFrame = null;
                }
            }
        }
    }

    class KinectSoundHandler
    {
        private Thread worker;
        private bool running = true;
        private AutoResetEvent waiter = new AutoResetEvent(false);
        private Storage dataStorage = Storage.Instance;
        private KinectSensor kinect;
        private Dispatcher uiDispatcher;

        private Action newSoundDataHandler;
        public event Action newSoundData
        {
            add { newSoundDataHandler += value; }
            remove { newSoundDataHandler -= value; }
        }

        public void stop()
        {
            running = false;
            waiter.Set();
        }

        public void start(KinectSensor kinect, Dispatcher uiDispatcher)
        {
            this.kinect = kinect;
            this.uiDispatcher = uiDispatcher;

            worker = new Thread(() => run(this));
            worker.Name = "sound handler";
            worker.Start();
        }

        private static void run(object data)
        {
            var t = (KinectSoundHandler)data;

            double angle = 0.0f;
            double confidence = 0.0f;
            double confidenceCutoff = 0.0;

            t.kinect.AudioSource.SoundSourceAngleChanged +=
                (object sender, SoundSourceAngleChangedEventArgs e) =>
                {
                    angle = e.Angle;
                    confidence = e.ConfidenceLevel;
                    t.waiter.Set();
                };

            while (true)
            {
                t.waiter.WaitOne();

                if (!t.running) break;

                if (confidence > confidenceCutoff)
                {
                    t.dataStorage.soundSourceAngle = angle;
                    t.dataStorage.soundSourceConfidence = confidence;
                }
            }
        }
    }
}

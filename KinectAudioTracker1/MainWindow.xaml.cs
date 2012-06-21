﻿using System;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace KinectAudioTracker
{
    using System.Collections.Generic;
    using System.Drawing;
    using System.Drawing.Imaging;
    using System.IO;
    using System.Runtime.Remoting.Contexts;
    using System.Threading;
    using System.Windows.Controls;
    using System.Windows.Interop;
    using System.Windows.Threading;
    using Emgu.CV;
    using Emgu.CV.Structure;
    using Microsoft.Kinect;

    public partial class MainWindow : Window
    {
        private KinectSensor kinect;

        private KinectDepthHandler depthHandler;
        private KinectColorHandler colorHandler;
        private KinectSoundHandler soundHandler;

        private readonly Vector3 kinectMin = new Vector3(-2.2, -1.6, 0);
        private readonly Vector3 kinectMax = new Vector3(2.2, 1.6, 4);

        private WriteableBitmap depthBitmap;
        public WriteableBitmap colorBitmap;
        private Storage dataStorage;

        private EigenObjectRecognizer recognizer;
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

            createTrainingData();
            logLine("Created training data");

            this.kinect.Start();

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

            logLine("Kinect initialized");

            //this.playerField.t = this;
        }

        void createTrainingData()
        {
            char[] sep = { '\\', '/' };
            var labels = new List<string>();
            var trainingImages = new List<Image<Gray, byte>>();

            foreach(var name in Directory.GetDirectories("training_data"))//////
            {
                foreach (var imagename in Directory.GetFiles(name))
                {
                    labels.Add(name.Split(sep)[1]);
                    trainingImages.Add(new Image<Gray, byte>(imagename));
                }
            }

            var crit = new MCvTermCriteria(16, 0.01);
            recognizer = new EigenObjectRecognizer(trainingImages.ToArray(), labels.ToArray(), 3000, ref crit);
        }

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
        }

        private void drawFaces()
        {
            if (dataStorage.faceRectangles.Length == 0)
                return;

            var faceRect = dataStorage.faceRectangles[0];
            var faceImage = Util.cropBitmap(colorBitmap, faceRect).Resize(100, 100, Emgu.CV.CvEnum.INTER.CV_INTER_CUBIC).Convert<Gray, byte>();
            faceImage._EqualizeHist();

            var output = recognizer.Recognize(faceImage);
            if (output != null)
                name1.Text = output.Label;
            else
                name1.Text = "Unknown";

            var faceSource = Imaging.CreateBitmapSourceFromHBitmap(faceImage.Bitmap.GetHbitmap(), IntPtr.Zero, Int32Rect.Empty, BitmapSizeOptions.FromEmptyOptions());
            face1.Source = faceSource;
        }

        private void drawPlayers()
        {
            //var talkingPlayer = locations.getClosestPlayerByAngle(this.soundSourceAngle, 50.0);

            playerField.Children.Clear();
            for (int i = 1; i < 7; ++i)
            {
                if (dataStorage.playerLocations.hasPixelPositionData(i))
                {
                    //System.Drawing.Point p = dataStorage.playerLocations.getPixelPosition(i);
                    Rectangle rect = dataStorage.playerLocations.getPlayerBoundingBox(i);
                    Util.drawRect(rect, this.depthBitmap, System.Drawing.Color.Orange);
                }
                if (dataStorage.playerLocations.hasWorldPositionData(i))
                {
                    var v = dataStorage.playerLocations.getWorldPosition(i);
                    var drawV = Vector3.ScaleVector(kinectMin, kinectMax, new Vector3(playerField.Width, 0, playerField.Height), v);
                    var rect = new System.Windows.Shapes.Rectangle
                    {
                        Stroke = System.Windows.Media.Brushes.Black,
                        StrokeThickness = 50
                    };
                    Canvas.SetLeft(rect, drawV.X);
                    Canvas.SetTop(rect, drawV.Z);
                    playerField.Children.Add(rect);
                }
            }
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
            colorHandler.enableFaceTracking();
        }

        private void stop_Click(object sender, RoutedEventArgs e)
        {
            colorHandler.disableFaceTracking();
        }

        public void logLine(string line)
        {
            this.listBox1.Items.Add(line);
        }
    }

    class Vector3
    {
        public double X { get; set; }
        public double Y { get; set; }
        public double Z { get; set; }

        public static readonly Vector3 Infinity = new Vector3(double.PositiveInfinity, double.PositiveInfinity, double.PositiveInfinity);
        public static readonly Vector3 Zero = new Vector3();

        public Vector3()
        {
            X = 0.0f;
            Y = 0.0f;
            Z = 0.0f;
        }
        
        public Vector3(double x, double y, double z)
        {
            this.X = x;
            this.Y = y;
            this.Z = z;
        }

        public Vector3 Clone()
        {
            return new Vector3(this.X, this.Y, this.Z);
        }

        public static Vector3 operator +(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1.X+v2.X, v1.Y+v2.Y, v1.Z+v2.Z);
        }

        public static Vector3 operator -(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1.X-v2.X, v1.Y-v2.Y, v1.Z-v2.Z);
        }

        public static Vector3 operator /(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1.X/v2.X, v1.Y/v2.Y, v1.Z/v2.Z);
        }

        public static Vector3 operator *(Vector3 v1, Vector3 v2)
        {
            return new Vector3(v1.X*v2.X, v1.Y*v2.Y, v1.Z*v2.Z);
        }

        public static Vector3 operator -(Vector3 v)
        {
            return new Vector3(-v.X, -v.Y, -v.Z);
        }

        public static Vector3 Clamp(Vector3 min, Vector3 x, Vector3 max)
        {
            var v = x.Clone();

            if(x.X < min.X) { v.X = min.X; }
            if(x.Y < min.Y) { v.Y = min.Y; }
            if(x.Z < min.Z) { v.Z = min.Z; }

            if(x.X > max.X) { v.X = max.X; }
            if(x.Y > max.Y) { v.Y = max.Y; }
            if(x.Z > max.Z) { v.Z = max.Z; }

            return v;
        }

        public static Vector3 Abs(Vector3 v)
        {
            var w = v.Clone();
            if(v.X < 0) { w.X = -w.X; }
            if(v.Y < 0) { w.Y = -w.Y; }
            if(v.Z < 0) { w.Z = -w.Z; }

            return w;
        }

        public static Vector3 ScaleVector(Vector3 initMin, Vector3 initMax, Vector3 outMax, Vector3 input)
        {
            var v = Clamp(initMin, input, initMax);
            v = v - initMin;
            return v / Abs(initMax - initMin) * outMax;
        }
    }

    class RingBuffer<T> : IEnumerable<T>
    {
        private LinkedList<T> buffer;

        public RingBuffer(int capacity)
        {
            if (capacity <= 0)
                throw new ArgumentOutOfRangeException("Capacity", "Must be greater than zero.");
            Capacity = capacity;
            buffer = new LinkedList<T>();
        }

        public int Capacity { get; private set; }
        public int Length { get { return buffer.Count; } }

        public T Head
        {
            get { return buffer.First.Value; }
        }

        public void Push(T data)
        {
            buffer.AddFirst(data);
            if (buffer.Count > Capacity)
                buffer.RemoveLast();
        }

        public void PopBack()
        {
            if(buffer.Count > 0)
                buffer.RemoveLast();
        }

        public void Clear()
        {
            buffer = new LinkedList<T>();
        }

        public System.Collections.Generic.IEnumerator<T> GetEnumerator()
        {
            return buffer.GetEnumerator();
        }

        System.Collections.IEnumerator System.Collections.IEnumerable.GetEnumerator()
        {
            return GetEnumerator();
        }
    }

    {
        {
        }

        {


        }

        public static Rectangle[] detectFaces(Image<Gray, byte> image, HaarCascade haar)
        {
            var faces = haar.Detect(image, 1.2, 3, Emgu.CV.CvEnum.HAAR_DETECTION_TYPE.DO_CANNY_PRUNING, new Size(20, 20), new Size(50, 50));
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
        {
            return rad * (180 / Math.PI);
        }

        public static T clamp<T>(T min, T x, T max) where T : System.IComparable<T>
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

        public static BitmapImage bitmapToBitmapImage(Bitmap b)
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
    }

    class PlayerPosition
    {
        private RingBuffer<Rectangle>[] playerPixelPositions;
        private int historyLength;
        private bool[] hasPixelPosition;
        private bool[] hasHistory;

        private Vector3[] playerWorldPositions;
        private bool[] hasUpdatedWorldPositions;

        private double[] playerAngles;
        
        private int playerCount;

        public PlayerPosition(int playerCount, int historyLength)
        {
            this.playerCount = playerCount;
            this.historyLength = historyLength;
            clear();
        }

        public void setBoundingBox(int playerNumber, Rectangle boundingBox)
        {
            if(!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();

            playerPixelPositions[playerNumber].Push(boundingBox);
            hasPixelPosition[playerNumber] = true;
            hasHistory[playerNumber] = true;
        }

        public Point getPixelPosition(int playerNumber)
        {
            if (!checkPlayerNumber(playerNumber) || !hasPixelPositionData(playerNumber))
                throw new ArgumentOutOfRangeException();

            return new Point(playerPixelPositions[playerNumber].Head().Width / 2 + playerPixelPositions[playerNumber].Head().X,
                playerPixelPositions[playerNumber].Head().Height / 2 + playerPixelPositions[playerNumber].Head().Y);
        }

        public Rectangle getBoundingBox(int playerNumber)
        {
            if (!checkPlayerNumber(playerNumber) || !hasPixelPositionData(playerNumber))
                throw new ArgumentOutOfRangeException();

            return playerPixelPositions[playerNumber].Head();
        }

        public Rectangle getSmoothedBoundingBox(int playerNumber)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();

            var sum = new Rectangle();
            foreach (var r in playerPixelPositions[playerNumber])
            {
                sum.X += r.X;
                sum.Y += r.Y;
                sum.Width += r.Width;
                sum.Height += r.Height;
            }

            sum.X /= historyLength;
            sum.Y /= historyLength;
            sum.Width /= historyLength;
            sum.Height /= historyLength;

            return sum;
        }

        public void setWorldPositions(DepthImageFrame context)
        {
            for (int i = 0; i < this.playerCount; i++)
            {
                if (hasPixelPositionData(i))
                {
                    var p = getPixelPosition(i);

                    if (playerWorldPositions[i] == null)
                        playerWorldPositions[i] = new Vector3();

                    playerWorldPositions[i].X = context.MapToSkeletonPoint(p.X, p.Y).X;
                    playerWorldPositions[i].Y = context.MapToSkeletonPoint(p.X, p.Y).Y;
                    playerWorldPositions[i].Z = context.MapToSkeletonPoint(p.X, p.Y).Z;

                    hasUpdatedWorldPositions[i] = true;
                }
            }
        }

        public Vector3 getWorldPosition(int playerNumber)
        {
            if (!checkPlayerNumber(playerNumber) && hasWorldPositionData(playerNumber))
                throw new ArgumentOutOfRangeException();

            return playerWorldPositions[playerNumber];
        }

        public bool hasWorldPositionData(int playerNumber)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();

            return hasUpdatedWorldPositions[playerNumber];
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
            this.hasHistory = new bool[playerCount];

            this.playerPixelPositions = new RingBuffer<Rectangle>[playerCount];
            for (int i = 0; i < playerCount; ++i)
                this.playerPixelPositions[i] = new RingBuffer<Rectangle>(historyLength);

            this.playerWorldPositions = new Vector3[playerCount];
            this.hasPixelPosition = new bool[playerCount];
            this.hasUpdatedWorldPositions = new bool[playerCount];

            this.playerAngles = new double[playerCount];
            for (int i = 0; i < playerCount; ++i)
            {
                this.playerAngles[i] = -1;
            }
        }

        public bool hasPixelPositionData(int playerNumber)
        {
            return hasPixelPosition[playerNumber];
        }

        public bool hasAngleData(int playerNumber)
        {
            if (!checkPlayerNumber(playerNumber))
                throw new ArgumentOutOfRangeException();

            return playerAngles[playerNumber] != -1;
        }

        private bool checkPlayerNumber(int n)
        {
            if (n <= 0 || n >= this.playerCount)
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
                    location = new PlayerPosition(7, 3);
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
            var playerPositions = new Rectangle[7];
            var hasPixelPosition = new bool[7];
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

                    if (!hasPixelPosition[player])
                    {
                        playerPositions[player].X = x;
                        playerPositions[player].Y = y;
                        playerPositions[player].Width = 0;
                        playerPositions[player].Height = 0;
                    }

                    playerPositions[player].X = Math.Min(playerPositions[player].X, x);
                    playerPositions[player].Y = Math.Min(playerPositions[player].Y, y);

                    if (x > playerPositions[player].Right)
                        playerPositions[player].Width = x - playerPositions[player].X;

                    if (y > playerPositions[player].Bottom)
                        playerPositions[player].Height = y - playerPositions[player].Y;

                    hasPixelPosition[player] = true;
                }
            }

            for(int i = 0; i < 7; ++i)
            {
                if(hasPixelPosition[i])
                    dataStorage.playerLocations.setBoundingBox(i, playerPositions[i]);
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
        private bool isFaceTracking = true;

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

        public void enableFaceTracking()
        {
            isFaceTracking = true;
        }

        public void disableFaceTracking()
        {
            isFaceTracking = false;
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

                    if(t.isFaceTracking)
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
        private Stream audioStream;

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

            kinect.AudioSource.Stop();
        }

        public void start(KinectSensor kinect, Dispatcher uiDispatcher)
        {
            this.kinect = kinect;
            this.uiDispatcher = uiDispatcher;

            // Wait for four seconds before starting audio
            Thread.Sleep(new TimeSpan(0, 0, 4));

            kinect.AudioSource.BeamAngleMode = BeamAngleMode.Adaptive;
            kinect.AudioSource.AutomaticGainControlEnabled = false;
            audioStream = kinect.AudioSource.Start();

            worker = new Thread(() => run(this));
            worker.Name = "sound handler";
            worker.Start();
        }

        private static void run(object data)
        {
            var t = (KinectSoundHandler)data;

            double angle = 0.0;
            double confidence = 0.0;
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
                    t.uiDispatcher.Invoke(t.newSoundDataHandler);
                }
            }
        }
    }
}

using System;
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

        public WriteableBitmap colorBitmap;
        private Storage dataStorage;

        private List<Image<Gray, byte>>[] playerFaces;

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

            this.colorBitmap = new WriteableBitmap(this.kinect.ColorStream.FrameWidth, this.kinect.ColorStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);

            depthHandler = new KinectDepthHandler();
            depthHandler.start(this.kinect, Dispatcher.CurrentDispatcher);
            depthHandler.newDepthData += new Action(depthHandler_newDepthData);

            colorHandler = new KinectColorHandler();
            colorHandler.start(this.kinect, Dispatcher.CurrentDispatcher);
            colorHandler.newColorData += new Action(colorHandler_newColorData);

            soundHandler = new KinectSoundHandler();
            soundHandler.start(this.kinect, Dispatcher.CurrentDispatcher);
            soundHandler.newSoundData += new Action(soundHandler_newSoundData);

            playerFaces = new List<Image<Gray, byte>>[7];
            for (int i = 0; i < 7; ++i)
                playerFaces[i] = new List<Image<Gray, byte>>();

            logLine("Kinect initialized");
        }

        void createTrainingData()
        {
            char[] sep = { '\\', '/' };
            var labels = new List<string>();
            var trainingImages = new List<Image<Gray, byte>>();

            foreach(var name in Directory.GetDirectories("training_data"))
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
        }

        void colorHandler_newColorData()
        {
            double sourceAngle;

            lock(Storage.beamAngleLock) { sourceAngle = dataStorage.SoundSourceAngle; }
            lock (Storage.colorImageLock) { colorBitmap.WritePixels(new Int32Rect(0, 0, colorBitmap.PixelWidth, colorBitmap.PixelHeight), this.dataStorage.colorPixels, colorBitmap.PixelWidth * sizeof(int), 0); }

            Rectangle boundingBox;

            //lock (Storage.playerLocationLock)
            //{
            //    lock (Storage.soundSourceAngleLock)
            //    { 
            //        if(dataStorage.SoundSourceConfidence < 0.5)
            //            return;
                    
            //        talkingPlayer = Util.getClosestPlayerByAngle(dataStorage.SoundSourceAngle, 10, dataStorage.playerLocations);
            //    }

            //    if (talkingPlayer != -1)
            //    { 
            //        boundingBox = dataStorage.playerLocations[talkingPlayer].SmoothedBoundingBox; 
            //    }
            //    else 
            //    {
            //        if (Math.Abs(dataStorage.BeamAngle) >= 40)
            //            talkingStatus.Text = "Offscreen";
            //        talkingStatus.Text = "";
            //        talkingFace.Source = null;
            //        return; 
            //    }
            //}

            var foundFace = false;
            var talkingPlayer = -1;
            lock (Storage.faceRectangleLock)
            {
                var faceAngles = new double[dataStorage.faceRectangles.Length];
                var i = 0;

                DepthImageFrame df;
                lock (Storage.depthFrameLock) { df = dataStorage.DepthFrame; }

                foreach (var face in dataStorage.faceRectangles)
                {
                    SkeletonPoint wp;
                    try { wp = df.MapToSkeletonPoint(face.Width / 2 + face.Left, face.Height / 2 + face.Top); }
                    catch { continue; }
                    //logLine("Calculated angle");
                    faceAngles[i] = -Util.radToDeg(-Math.Atan(wp.X / wp.Z));
                }

                double difference = 0;
                talkingPlayer = Util.getClosestPlayerByAngle(sourceAngle, 10, faceAngles, ref difference);
                if (talkingPlayer != -1)
                {
                    var tf = dataStorage.faceRectangles[talkingPlayer];
                    talkingFace.Source = new CroppedBitmap(colorBitmap, new Int32Rect(tf.X, tf.Y, tf.Width, tf.Height));
                    sourceAngleText.Text = "Talking";
                    foundFace = true;
                }
                else
                {
                    if (dataStorage.faceRectangles.Length > 0)
                    {
                        var tf = dataStorage.faceRectangles[0];
                        talkingFace.Source = null;//new CroppedBitmap(colorBitmap, new Int32Rect(tf.X, tf.Y, tf.Width, tf.Height));
                        sourceAngleText.Text = sourceAngle.ToString();
                        playerAngleText.Text = faceAngles[0].ToString();
                    }
                }
            }

            if (!foundFace)
            {
                lock (Storage.playerLocationLock)
                {
                    double difference = 0;
                    var talkingPlayer2 = Util.getClosestPlayerByAngle(sourceAngle, 10, dataStorage.playerLocations, ref difference);
                    var boundingBox2 = dataStorage.playerLocations[talkingPlayer2].BoundingBox;
                    talkingFace.Source = new CroppedBitmap(colorBitmap, new Int32Rect(boundingBox2.X, boundingBox2.Y, boundingBox2.Width, boundingBox2.Height));
                    sourceAngleText.Text = "Talking";
                }
            }
        }

        void depthHandler_newDepthData()
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
            if(colorHandler != null)
                colorHandler.enableFaceTracking();
        }

        private void stop_Click(object sender, RoutedEventArgs e)
        {
            if(colorHandler != null)
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

        public static Vector3 operator /(Vector3 v, int i)
        {
            return new Vector3(v.X / i, v.Y / i, v.Z / i);
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

    static class Util
    {
        public static Rectangle lerp(Rectangle a, Rectangle b)
        {
            return new Rectangle((a.X + b.X) / 2, (a.Y + b.Y) / 2, (a.Width + b.Width) / 2, (a.Height + b.Height) / 2);
        }

        public static void saveBitmapToFile(BitmapSource src, string filename)
        {
            BitmapEncoder pngEncoder = new PngBitmapEncoder();
            pngEncoder.Frames.Add(BitmapFrame.Create(src));

            System.IO.MemoryStream ms = new MemoryStream();

            pngEncoder.Save(ms);
            ms.Close();
            System.IO.File.WriteAllBytes(filename, ms.ToArray());
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

        public static Image<Bgr, byte> cropBitmap(WriteableBitmap b, Rectangle r)
        {
            if (r.X < 0 || r.Right >= b.PixelWidth || r.Y < 0 || r.Bottom >= b.PixelHeight)
                throw new ArgumentOutOfRangeException();

            b.Lock();

            byte[, ,] pixelData = new byte[r.Height, r.Width, 3];

            unsafe
            {
                for (int j = r.Y; j < r.Bottom; ++j)
                {
                    for (int i = r.X; i < r.Right; ++i)
                    {
                        int backBuffer = (int)b.BackBuffer;

                        backBuffer += j * b.BackBufferStride;
                        backBuffer += i * 4;

                        int colorData = *((int*)backBuffer);
                        var bytes = BitConverter.GetBytes(colorData);
                        pixelData[j - r.Y, i - r.X, 0] = bytes[0];
                        pixelData[j - r.Y, i - r.X, 1] = bytes[1];
                        pixelData[j - r.Y, i - r.X, 2] = bytes[2];
                    }
                }
            }

            return new Image<Bgr, byte>(pixelData);
        }

        public static void drawBoundingBox(Rectangle r, WriteableBitmap b, Color c, int pixelWidth)
        {
            Util.drawRect(new Rectangle(r.X, r.Y, r.Width, pixelWidth), b, c);
            Util.drawRect(new Rectangle(r.X, r.Y, pixelWidth, r.Height), b, c);
            Util.drawRect(new Rectangle(r.X, r.Bottom - pixelWidth, r.Width, pixelWidth), b, c);
            Util.drawRect(new Rectangle(r.Right - pixelWidth, r.Y, pixelWidth, r.Height), b, c);
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

        public static Bitmap BitmapImageToBitmap(BitmapImage bitmapImage)
        {
            using (MemoryStream outStream = new MemoryStream())
            {
                BitmapEncoder enc = new BmpBitmapEncoder();
                enc.Frames.Add(BitmapFrame.Create(bitmapImage));
                enc.Save(outStream);
                System.Drawing.Bitmap bitmap = new System.Drawing.Bitmap(outStream);

                // return bitmap; <-- leads to problems, stream is closed/closing ...
                return new Bitmap(bitmap);
            }
        }

        public static int getClosestPlayerByAngle(double angle, double cutoff, Player[] players, ref double difference)
        {
            double min_difference = double.PositiveInfinity;
            cutoff = Math.Abs(cutoff);
            int minPlayer = -1;

            for(int i = 0; i < 7; ++i)
            {
                difference = Math.Abs(players[i].Angle - angle);
                if (difference < min_difference && difference < cutoff)
                {
                    min_difference = difference;
                    minPlayer = i;
                }
            }

            difference = min_difference;
            return minPlayer;
        }

        public static int getClosestPlayerByAngle(double angle, double cutoff, double[] players, ref double difference)
        {
            double min_difference = double.PositiveInfinity;
            cutoff = Math.Abs(cutoff);
            int minPlayer = -1;

            for (int i = 0; i < players.Length; ++i)
            {
                difference = Math.Abs(players[i] - angle);
                if (difference < min_difference && difference < cutoff)
                {
                    min_difference = difference;
                    minPlayer = i;
                }
            }

            difference = min_difference;
            return minPlayer;
        }
    }

    class Player
    {
        private RingBuffer<Rectangle> pixelPosition;
        private RingBuffer<Vector3> worldPosition;
        private double angle;
        private int historyLength;

        public Player(int historyLength)
        {
            if (historyLength <= 0)
                throw new ArgumentOutOfRangeException("historyLength", "History length must be greater than zero.");

            this.historyLength = historyLength;

            pixelPosition = new RingBuffer<Rectangle>(historyLength);
            worldPosition = new RingBuffer<Vector3>(historyLength);
        }

        public void Clear()
        {
            pixelPosition.PopBack();
            worldPosition.PopBack();
        }

        public bool hasPixelPosition
        {
            get { return pixelPosition.Length > 0; }
        }

        public bool hasWorldPosition
        {
            get { return worldPosition.Length > 0; }
        }

        public Rectangle BoundingBox
        {
            get { return pixelPosition.Head; }
            set { pixelPosition.Push(value); }
        }

        public Rectangle SmoothedBoundingBox
        {
            get
            {
                var length = pixelPosition.Length; 
                var sum = new Rectangle();

                if (length == 0)
                    return sum;

                foreach (var r in pixelPosition)
                {
                    sum.X += r.X;
                    sum.Y += r.Y;
                    sum.Width += r.Width;
                    sum.Height += r.Height;
                }

                sum.X /= length;
                sum.Y /= length;
                sum.Width /= length;
                sum.Height /= length;

                return sum;
            }
        }

        public Point PixelPosition
        {
            get
            {
                return new Point(pixelPosition.Head.Width / 2 + pixelPosition.Head.X, pixelPosition.Head.Height / 2 + pixelPosition.Head.Y);
            }
        }

        public Vector3 WorldPosition
        {
            get { return worldPosition.Head; }
        }

        public Vector3 SmoothedWorldPosition
        {
            get
            {
                var sum = new Vector3();
                foreach (var v in worldPosition)
                    sum += v;

                return sum / worldPosition.Length;
            }
        }

        public void setWorldPosition(DepthImageFrame context)
        {
            if (pixelPosition.Length > 0)
            {
                var p = PixelPosition;
                SkeletonPoint sp;

                try { sp = context.MapToSkeletonPoint(p.X, p.Y); }
                catch (NullReferenceException e) { return; }

                var wp = new Vector3(sp.X, sp.Y, sp.Z);

                worldPosition.Push(wp);
            }

            context.Dispose();
        }

        public double Angle
        {
            get { return angle; }
            set { angle = value; }
        }
    }

    class Storage
    {
        private static Storage instance;
        public static object depthImageLock, colorImageLock, playerLocationLock,
            soundSourceAngleLock, soundSourceConfidenceLock, faceRectangleLock, beamAngleLock,
            depthFrameLock;
        private static byte[] depth, color;
        private static Player[] location;
        private static double sourceAngle, sourceConfidence, beamAngle;
        private Rectangle[] faces;
        private DepthImageFrame depthFrame;

        private Storage() { }

        public static Storage Instance
        {
            get
            {
                if (instance == null)
                {
                    instance = new Storage();

                    location = new Player[7];
                    for (int i = 0; i < 7; ++i)
                        location[i] = new Player(20);

                    beamAngleLock = new Object();
                    depthImageLock = new Object();
                    colorImageLock = new Object();
                    depthFrameLock = new Object();
                    faceRectangleLock = new Object();
                    playerLocationLock = new Object();
                    soundSourceAngleLock = new Object();
                    soundSourceConfidenceLock = new Object();
                }
                return instance;
            }
        }

        public Player[] playerLocations
        {
            get { return location; }
            set { location = value; }
        }

        public byte[] depthPixels
        {
            get { return depth; }
            set { depth = value; }
        }

        public byte[] colorPixels
        {
            get { return color; }
            set { color = value; }
        }

        public double SoundSourceAngle
        {
            get { return sourceAngle; }
            set { sourceAngle = value; }
        }

        public double SoundSourceConfidence
        {
            get { return sourceConfidence; }
            set { sourceConfidence = value; }
        }

        public double BeamAngle
        {
            get { return beamAngle; }
            set { beamAngle = value; }
        }

        public Rectangle[] faceRectangles
        {
            get { return faces; }
            set { faces = value; }
        }

        public DepthImageFrame DepthFrame
        {
            get { return depthFrame; }
            set { depthFrame = value; }
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
        private short[] depthPixels;
        private byte[] coloredPixels;

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
                var success = t.waiter.WaitOne(new TimeSpan(0, 2, 0));
                if (!success)
                    throw new TimeoutException();

                if (!t.running) break;

                if (imageFrame != null)
                {
                    lock (Storage.depthImageLock)
                    {
                        t.dataStorage.depthPixels = processDepthFrame(imageFrame, t.kinect, t);
                    }
                    t.uiDispatcher.BeginInvoke(t.newDepthDataHandler);
                    imageFrame.Dispose();
                    imageFrame = null;
                }
            }
        }

        private static byte[] processDepthFrame(DepthImageFrame imageFrame, KinectSensor kinect, KinectDepthHandler t)
        {
            var dataStorage = Storage.Instance;
            lock (Storage.depthFrameLock) 
            {
                if (dataStorage.DepthFrame != null)
                    dataStorage.DepthFrame.Dispose();
                dataStorage.DepthFrame = imageFrame;
            }

            // color divisors for tinting depth pixels
            int[] IntensityShiftByPlayerR = { 1, 2, 0, 2, 0, 0, 2, 0 };
            int[] IntensityShiftByPlayerG = { 1, 2, 2, 0, 2, 0, 0, 1 };
            int[] IntensityShiftByPlayerB = { 1, 0, 2, 2, 0, 2, 0, 2 };

            if(t.depthPixels == null)
                t.depthPixels = new short[kinect.DepthStream.FramePixelDataLength];
            if(t.coloredPixels == null)
                t.coloredPixels = new byte[kinect.DepthStream.FramePixelDataLength * sizeof(int)];

            imageFrame.CopyPixelDataTo(t.depthPixels);

            int colorPixelIndex = 0;
            var boundingBoxes = new Rectangle[7];
            var hasPixelPosition = new bool[7];
            for (int i = 0; i < t.depthPixels.Length; ++i)
            {
                int player = t.depthPixels[i] & DepthImageFrame.PlayerIndexBitmask;
                int depth = t.depthPixels[i] >> DepthImageFrame.PlayerIndexBitmaskWidth;

                byte intensity = (byte)(~(depth >> 4));

                t.coloredPixels[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerR[player]);
                t.coloredPixels[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerG[player]);
                t.coloredPixels[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerB[player]);
                ++colorPixelIndex;

                // Update the player location average
                if (player != 0)
                {
                    var x = i % imageFrame.Width;
                    var y = i / imageFrame.Width;

                    if (!hasPixelPosition[player])
                        boundingBoxes[player] = new Rectangle(x, y, 0, 0);

                    boundingBoxes[player].X = Math.Min(x, boundingBoxes[player].X);
                    boundingBoxes[player].Y = Math.Min(y, boundingBoxes[player].Y);

                    if (x > boundingBoxes[player].Right)
                        boundingBoxes[player].Width = x - boundingBoxes[player].Left;

                    if (y > boundingBoxes[player].Bottom)
                        boundingBoxes[player].Height = y - boundingBoxes[player].Top;


                    hasPixelPosition[player] = true;
                }
            }

            lock (Storage.playerLocationLock)
            {
                for (int i = 1; i < 7; ++i)
                {
                    if (hasPixelPosition[i])
                    {
                        dataStorage.playerLocations[i].BoundingBox = boundingBoxes[i];
                        dataStorage.playerLocations[i].setWorldPosition(imageFrame);

                        if (dataStorage.playerLocations[i].hasWorldPosition)
                        {
                            var v = dataStorage.playerLocations[i].WorldPosition;
                            var x = v.X;
                            var z = v.Z;
                            dataStorage.playerLocations[i].Angle = Util.radToDeg(-Math.Atan(x / z));
                        }
                    }
                    else
                        dataStorage.playerLocations[i].Clear();
                }
            }

            return t.coloredPixels;
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
                var success = t.waiter.WaitOne(new TimeSpan(0, 2, 0));
                if (!success)
                    throw new TimeoutException();

                if (!t.running) break;

                lock (Storage.colorImageLock)
                {
                    if (t.dataStorage.colorPixels == null)
                    {
                        t.dataStorage.colorPixels = new byte[t.kinect.ColorStream.FramePixelDataLength];
                    }
                }

                if (imageFrame != null)
                {
                    lock (Storage.colorImageLock) { imageFrame.CopyPixelDataTo(t.dataStorage.colorPixels); }

                    lock (Storage.faceRectangleLock)
                    {
                        if (t.isFaceTracking)
                            t.dataStorage.faceRectangles = Util.detectFaces(Util.colorFrameToImage(imageFrame).Convert<Gray, byte>(), haar);
                    }
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

            double sourceAngle = 0.0, confidence = 0.0, confidenceCutoff = 0.0, beamAngle = 0.0;

            t.kinect.AudioSource.SoundSourceAngleChanged +=
                (object sender, SoundSourceAngleChangedEventArgs e) =>
                {
                    sourceAngle = e.Angle;
                    confidence = e.ConfidenceLevel;
                    t.waiter.Set();
                };

            t.kinect.AudioSource.BeamAngleChanged +=
                (object sender, BeamAngleChangedEventArgs e) =>
                {
                    beamAngle = e.Angle;
                    t.waiter.Set();
                };

            while (true)
            {
                var success = t.waiter.WaitOne(new TimeSpan(0, 2, 0));
                if (!success)
                    throw new TimeoutException();

                if (!t.running) break;

                if (confidence > confidenceCutoff)
                {
                    lock (Storage.soundSourceAngleLock) { t.dataStorage.SoundSourceAngle = sourceAngle; }
                    lock (Storage.soundSourceConfidenceLock) { t.dataStorage.SoundSourceConfidence = confidence; }
                    lock (Storage.beamAngleLock) { t.dataStorage.BeamAngle = Util.radToDeg(beamAngle); }
                    t.uiDispatcher.Invoke(t.newSoundDataHandler);
                }
            }
        }
    }
}

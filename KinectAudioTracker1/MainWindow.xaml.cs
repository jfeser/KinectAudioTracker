using System;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Imaging;

namespace KinectAudioTracker
{
    using System.Drawing;
    using System.Speech.Recognition;
    using System.Threading;
    using System.Windows.Threading;
    using Microsoft.Kinect;
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor kinect;

        private short[] depthPixels;
        private byte[] coloredDepth;
        private WriteableBitmap depthBitmap;

        // color divisors for tinting depth pixels
        private static readonly int[] IntensityShiftByPlayerR = { 1, 2, 0, 2, 0, 0, 2, 0 };
        private static readonly int[] IntensityShiftByPlayerG = { 1, 2, 2, 0, 2, 0, 0, 1 };
        private static readonly int[] IntensityShiftByPlayerB = { 1, 0, 2, 2, 0, 2, 0, 2 };

        private readonly string recognizerID = "SR_MS_en-US_Kinect_11.0";
        private SpeechRecognitionEngine speechRecognizer;

        private Thread audioThread;
        private double soundSourceAngle;
        private double soundSourceConfidence;
        private readonly double confidenceCutoff = 0.0;
        private bool audioOn;

        private DispatcherTimer readyTimer;
        private int tickCount;

        private PlayerLocation playerLocations;

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

            this.depthPixels = new short[this.kinect.DepthStream.FramePixelDataLength];
            this.coloredDepth = new byte[this.kinect.DepthStream.FramePixelDataLength * sizeof(int)];
            this.depthBitmap = new WriteableBitmap(this.kinect.DepthStream.FrameWidth, this.kinect.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
            image1.Source = this.depthBitmap;
            this.playerLocations = new PlayerLocation();

            this.kinect.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(kinect_DepthFrameReady);
            this.kinect.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(kinect_ColorFrameReady);

            this.kinect.AudioSource.AutomaticGainControlEnabled = false;
            this.kinect.AudioSource.SoundSourceAngleChanged += new EventHandler<SoundSourceAngleChangedEventArgs>(AudioSource_SoundSourceAngleChanged);

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

            // Plot the sound source location
            if (this.soundSourceConfidence > this.confidenceCutoff)
            {
                audioAngleDisplay.rotTx.Angle = -this.soundSourceAngle;
                var minAngle = KinectAudioSource.MinSoundSourceAngle;
                var maxAngle = KinectAudioSource.MaxSoundSourceAngle;
                //double soundSourceLocation = (clamp<double>(minAngle, this.soundSourceAngle, maxAngle) - minAngle) / (maxAngle - minAngle);
                //drawRect(new Rectangle((int)(640 * soundSourceLocation), 0, 20, 20), ref this.depthBitmap, Color.Azure);
            }
        }

        private T clamp<T>(T min, T x, T max) where T : System.IComparable<T>
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
            using (ColorImageFrame imageFrame = e.OpenColorImageFrame())
            {
                if (imageFrame != null)
                {
                }
            }
        }

        private void kinect_DepthFrameReady(object sender, DepthImageFrameReadyEventArgs e)
        {
            using (DepthImageFrame imageFrame = e.OpenDepthImageFrame())
            {
                if (imageFrame != null)
                {
                    imageFrame.CopyPixelDataTo(this.depthPixels);

                    playerLocations.clear();

                    int colorPixelIndex = 0;
                    for (int i = 0; i < depthPixels.Length; ++i)
                    {
                        int player = depthPixels[i] & DepthImageFrame.PlayerIndexBitmask;
                        int depth = depthPixels[i] >> DepthImageFrame.PlayerIndexBitmaskWidth;

                        byte intensity = (byte)(~(depth >> 4));

                        coloredDepth[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerR[player]);
                        coloredDepth[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerG[player]);
                        coloredDepth[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerB[player]);
                        ++colorPixelIndex;

                        // Update the player location average
                        if (player != 0)
                        {
                            var x = i % depthBitmap.PixelWidth;
                            var y = i / depthBitmap.PixelWidth;
                            playerLocations.update(player, x, y);
                        }
                    }

                    //// Update the angle between the player and the kinect
                    //for (int i = 0; i < 7; ++i)
                    //{
                    //    var playerPosition = playerLocations.getSkeletonCoordinateLocation(i, imageFrame);
                    //    playerLocations.setAngle(i, radToDeg(-Math.Atan(playerPosition.X / playerPosition.Z)));
                    //}

                    // Draw the depth bitmap to the screen
                    depthBitmap.WritePixels(new Int32Rect(0, 0, depthBitmap.PixelWidth, depthBitmap.PixelHeight), coloredDepth,
                        depthBitmap.PixelWidth * sizeof(int), 0);

                    drawPlayers();
                }
            }
        }

        private void drawPlayers()
        {
            //var talkingPlayer = this.playerLocations.getClosestAngle(this.soundSourceAngle, 10.0);

            //if (talkingPlayer != -1)
            //    logLine(talkingPlayer.ToString());

            for (int i = 0; i < 7; ++i)
            {
                if (playerLocations.hasData(i))
                {
                    var x = playerLocations.getX(i);
                    var y = playerLocations.getY(i);

                    //if (i == talkingPlayer)
                    //{
                    //    drawRect(new Rectangle(x, y, 20, 20), ref this.depthBitmap, System.Drawing.Color.Green);
                    //}
                    drawRect(new Rectangle(x, y, 20, 20), ref this.depthBitmap, System.Drawing.Color.Orange);
                }
            }
        }

        private double radToDeg(double rad)
        {
            return rad * (180 / Math.PI);
        }

        private void drawRect(Rectangle r, ref WriteableBitmap b, Color c)
        {
            for (int i = r.Y; i < r.Y + r.Height; ++i)
            {
                for (int j = r.X; j < r.X + r.Width; ++j)
                {
                    drawPixel(j, i, ref b, c);
                }
            }
        }

        private void drawPixel(int x, int y, ref WriteableBitmap b, Color c)
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
            this.playerLocations = new Point[7];
            this.playerAngles = new double[7];
            this.averageCount = new int[7];
        }

        public PlayerLocation(int playerCount)
        {
            this.playerCount = playerCount;
            this.playerLocations = new Point[playerCount];
            this.playerAngles = new double[playerCount];
            this.averageCount = new int[playerCount];
        }

        public void update(int playerNumber, int x, int y)
        {
            checkPlayerNumber(playerNumber);

            ++averageCount[playerNumber];
            playerLocations[playerNumber].X += x;
            playerLocations[playerNumber].Y += y;
        }

        public void setAngle(int playerNumber, double angle)
        {
            checkPlayerNumber(playerNumber);

            playerAngles[playerNumber] = angle;
        }

        public double getAngle(int playerNumber, double angle)
        {
            checkPlayerNumber(playerNumber);

            return playerAngles[playerNumber];
        }

        /// <summary>
        /// Finds the player whos angle from the Kinect is the closest to the provided angle and is within the difference cutoff.
        /// </summary>
        /// <param name="angle"></param>
        /// <param name="cutoff"></param>
        /// <returns></returns>
        public int getClosestAngle(double angle, double cutoff)
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
        }

        public bool hasData(int playerNumber)
        {
            return averageCount[playerNumber] != 0;
        }

        public int getX(int playerNumber)
        {
            checkPlayerNumber(playerNumber);

            if (averageCount[playerNumber] == 0)
            {
                return 0;
            }
            else
            {
                return playerLocations[playerNumber].X / averageCount[playerNumber];
            }
        }

        public int getY(int playerNumber)
        {
            checkPlayerNumber(playerNumber);

            if (averageCount[playerNumber] == 0)
            {
                return 0;
            }
            else
            {
                return playerLocations[playerNumber].Y / averageCount[playerNumber];
            }
        }

        public SkeletonPoint getSkeletonCoordinateLocation(int playerNumber, DepthImageFrame context)
        {
            checkPlayerNumber(playerNumber);

            return context.MapToSkeletonPoint(playerLocations[playerNumber].X, playerLocations[playerNumber].Y);
        }

        private void checkPlayerNumber(int n)
        {
            if (n < 0 || n >= this.playerLocations.Length)
            {
                throw new ArgumentOutOfRangeException("playerNumber", "Player number out of range.");
            }
        }
    }
}

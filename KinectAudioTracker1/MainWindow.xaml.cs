using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace KinectAudioTracker
{
    using Microsoft.Kinect;
    using System.Threading;
    using System.IO;

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private KinectSensor kinect;
        private bool initialized;

        private short[] depthPixels;
        private byte[] coloredDepth;
        private WriteableBitmap depthBitmap;

        // color divisors for tinting depth pixels
        private static readonly int[] IntensityShiftByPlayerR = { 1, 2, 0, 2, 0, 0, 2, 0 };
        private static readonly int[] IntensityShiftByPlayerG = { 1, 2, 2, 0, 2, 0, 0, 1 };
        private static readonly int[] IntensityShiftByPlayerB = { 1, 0, 2, 2, 0, 2, 0, 2 };

        private Stream audioStream;
        private double angle;
        private double confidence;

        public MainWindow()
        {
            InitializeComponent();
            initializeKinect();
        }

        private void initializeKinect()
        {
            foreach (KinectSensor sensor in KinectSensor.KinectSensors)
            {
                if (sensor.Status == KinectStatus.Connected)
                {
                    this.kinect = sensor;
                    break;
                }
            }

            if (this.kinect == null)
            {
                return;
            }

            this.kinect.ColorStream.Enable(ColorImageFormat.RgbResolution640x480Fps30);
            this.kinect.DepthStream.Enable(DepthImageFormat.Resolution640x480Fps30);
            //this.audioStream = this.kinect.AudioSource.Start();

            try
            {
                this.kinect.Start();
            }
            catch (Exception)
            {
                return;
            }

            if (this.kinect != null)
            {
                // Wait for four seconds till streaming
                //Thread.Sleep(4000);

                this.kinect.DepthFrameReady += new EventHandler<DepthImageFrameReadyEventArgs>(kinect_DepthFrameReady);
                this.depthPixels = new short[this.kinect.DepthStream.FramePixelDataLength];
                this.coloredDepth = new byte[this.kinect.DepthStream.FramePixelDataLength * sizeof(int)];
                this.depthBitmap = new WriteableBitmap(this.kinect.DepthStream.FrameWidth, this.kinect.DepthStream.FrameHeight, 96.0, 96.0, PixelFormats.Bgr32, null);
                image1.Source = this.depthBitmap;

                this.kinect.ColorFrameReady += new EventHandler<ColorImageFrameReadyEventArgs>(kinect_ColorFrameReady);

                this.kinect.AudioSource.SoundSourceAngleChanged += new EventHandler<SoundSourceAngleChangedEventArgs>(AudioSource_SoundSourceAngleChanged);
                this.initialized = true;
            }
            else
            {
                this.initialized = false;
            }

            this.listBox1.Items.Add("Kinect initialized");
        }

        void AudioSource_SoundSourceAngleChanged(object sender, SoundSourceAngleChangedEventArgs e)
        {
            this.angle = e.Angle;
            this.confidence = e.ConfidenceLevel;
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
                    updateDepthBitmap();
                }
            }
        }

        private void updateDepthBitmap()
        {
            int colorPixelIndex = 0;

            DepthImageStream depthStream = this.kinect.DepthStream;
            int tooNearDepth = depthStream.TooNearDepth;
            int tooFarDepth = depthStream.TooFarDepth;
            int unknownDepth = depthStream.UnknownDepth;

            for (int i = 0; i < this.depthPixels.Length; ++i)
            {
                int player = this.depthPixels[i] & DepthImageFrame.PlayerIndexBitmask;
                int depth = this.depthPixels[i] >> DepthImageFrame.PlayerIndexBitmaskWidth;

                byte intensity = (byte)(~(depth >> 4));

                this.coloredDepth[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerR[player]);
                this.coloredDepth[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerG[player]);
                this.coloredDepth[colorPixelIndex++] = (byte)(intensity >> IntensityShiftByPlayerB[player]);
                ++colorPixelIndex;

                if (player != 0)
                {
                    this.listBox1.Items.Add(player);
                }
            }

            this.depthBitmap.WritePixels(new Int32Rect(0, 0, this.depthBitmap.PixelWidth, this.depthBitmap.PixelHeight),
                this.coloredDepth, this.depthBitmap.PixelWidth * sizeof(int), 0);
        }

        private void uninitializeKinect()
        {
            this.initialized = false;
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
    }
}

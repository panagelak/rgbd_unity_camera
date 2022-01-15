using System;
using System.Collections.Generic;
using Unity.Robotics.ROSTCPConnector;
using UnityEngine.Serialization;
using RosMessageTypes.Sensor;
using RosMessageTypes.Std;
using RosMessageTypes.BuiltinInterfaces;
using Unity.Robotics.Core;
using Unity.Robotics.ROSTCPConnector.MessageGeneration;
using UnityEngine;
using UnityEngine.Rendering;
using System.Collections;
using System.IO;
/// <summary>
///
/// </summary>
public class RosDepthPublisher : MonoBehaviour
{
    ROSConnection ros;
    public string topicName = "unity_camera/depth/image_raw/compressed";
    public string cameraInfoTopicName = "unity_camera/rgb/camera_info";

    // The game object
    public Camera ImageCamera;
    public string FrameId = "unity_camera/depth_frame";
    public int resolutionWidth = 640;
    public int resolutionHeight = 480;
    // [Range(0, 100)]
    // public int qualityLevel = 50;
    // private Texture2D texture2D;
    // private Rect rect;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 1.0f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    private CompressedImageMsg message;

    [Header("Shader Setup")]
    public Shader uberReplacementShader;
    // public Shader opticalFlowShader;
    public float opticalFlowSensitivity = 1.0f;

    // pass configuration
    private CapturePass capturePass = new CapturePass() { name = "_depth" };
    private Texture2D texture2D;
    private Rect rect;
    struct CapturePass
    {
        // configuration
        public string name;
        public bool supportsAntialiasing;
        public bool needsRescale;
        public CapturePass(string name_) { name = name_; supportsAntialiasing = true; needsRescale = false; camera = null; }
        public Camera camera;
    };

    static private void SetupCameraWithReplacementShader(Camera cam, Shader shader, ReplacementMode mode, Color clearColor)
    {
        var cb = new CommandBuffer();
        cb.SetGlobalFloat("_OutputMode", (int)mode); // @TODO: CommandBuffer is missing SetGlobalInt() method
        cam.AddCommandBuffer(CameraEvent.BeforeForwardOpaque, cb);
        cam.AddCommandBuffer(CameraEvent.BeforeFinalPass, cb);
        cam.SetReplacementShader(shader, "");
        cam.backgroundColor = clearColor;
        cam.clearFlags = CameraClearFlags.SolidColor;
        cam.allowHDR = false;
        cam.allowMSAA = false;
    }
    public enum ReplacementMode
    {
        ObjectId = 0,
        CatergoryId = 1,
        DepthCompressed = 2,
        DepthMultichannel = 3,
        Normals = 4
    };
    void Start()
    {
        // start the ROS connection
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterPublisher<CompressedImageMsg>(topicName);
        ros.RegisterPublisher<CameraInfoMsg>(cameraInfoTopicName);
        // Initialize game Object
        // texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);
        // rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        // ImageCamera.targetTexture = new RenderTexture(resolutionWidth, resolutionHeight, 24);
        texture2D = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RFloat, false);
        rect = new Rect(0, 0, resolutionWidth, resolutionHeight);
        // default fallbacks, if shaders are unspecified
        if (!uberReplacementShader)
            uberReplacementShader = Shader.Find("Hidden/UberReplacement");

        // if (!opticalFlowShader)
        //     opticalFlowShader = Shader.Find("Hidden/OpticalFlow");

        //set up camera shader
        SetupCameraWithReplacementShader(ImageCamera, uberReplacementShader, ReplacementMode.DepthCompressed, Color.white);


        capturePass.camera = ImageCamera;

        // on scene change
        var renderers = UnityEngine.Object.FindObjectsOfType<Renderer>();
        var mpb = new MaterialPropertyBlock();
        foreach (var r in renderers)
        {
            var id = r.gameObject.GetInstanceID();
            var layer = r.gameObject.layer;
            var tag = r.gameObject.tag;

            mpb.SetColor("_ObjectColor", ColorEncoding.EncodeIDAsColor(id));
            mpb.SetColor("_CategoryColor", ColorEncoding.EncodeLayerAsColor(layer));
            r.SetPropertyBlock(mpb);
        }


        //SAVE


        // Camera.onPostRender += UpdateImage;
    }
    // private void UpdateImage(Camera _camera)
    // {
    //     if (texture2D != null && _camera == this.ImageCamera)
    //         UpdateMessage();
    // }

    void Update()
    {
        if (texture2D != null)
            // execute as coroutine to wait for the EndOfFrame before starting capture
            StartCoroutine(
                WaitForEndOfFrameAndSave());
    }


    private IEnumerator WaitForEndOfFrameAndSave()
    {
        yield return new WaitForEndOfFrame();
        UpdateMessage();
    }

    private void UpdateMessage()
    {




        // if (needsRescale)
        // {
        //     // blit to rescale (see issue with Motion Vectors in @KNOWN ISSUES)
        //     RenderTexture.active = finalRT;
        //     Graphics.Blit(renderRT, finalRT);
        //     RenderTexture.ReleaseTemporary(renderRT);
        // }

        // read offsreen texture contents into the CPU readable texture

        // texture2D.Apply();

        // encode texture into PNG
        // var bytes = tex.EncodeToPNG();


        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            //save
            bool supportsAntialiasing = true;
            bool needsRescale = false;
            var depth = 32;
            var format = RenderTextureFormat.Default;
            var readWrite = RenderTextureReadWrite.Default;
            var antiAliasing = (supportsAntialiasing) ? Mathf.Max(1, QualitySettings.antiAliasing) : 1;

            var finalRT =
                RenderTexture.GetTemporary(resolutionWidth, resolutionHeight, depth, format, readWrite, antiAliasing);
            var renderRT = (!needsRescale) ? finalRT :
                RenderTexture.GetTemporary(ImageCamera.pixelWidth, ImageCamera.pixelHeight, depth, format, readWrite, antiAliasing);
            // var tex = new Texture2D(resolutionWidth, resolutionHeight, TextureFormat.RGB24, false);

            var prevActiveRT = RenderTexture.active;
            var prevCameraRT = ImageCamera.targetTexture;

            // render to offscreen texture (readonly from CPU side)
            RenderTexture.active = renderRT;
            ImageCamera.targetTexture = renderRT;
            //
            ImageCamera.Render();
            texture2D.ReadPixels(rect, 0, 0);
            texture2D.Apply();
            int length = texture2D.GetRawTextureData().Length;
            Debug.Log("Size is : " + length);
            // texture2D.ReadPixels(rect, 0, 0);
            // var timestamp = new TimeStamp(Clock.time);
            // Message
            var timestamp = new TimeStamp(Clock.time);
            CompressedImageMsg message = new CompressedImageMsg
            {
                header = new HeaderMsg
                {
                    frame_id = FrameId,
                    stamp = new TimeMsg
                    {
                        sec = timestamp.Seconds,
                        nanosec = timestamp.NanoSeconds,
                    }
                },
                format = "32FC1; compressedDepth png",
                // data = texture2D.EncodeToPNG() // TODO : encode 32fc1 image
            };
            int length2 = message.data.Length;
            Debug.Log("Size2 is : " + length2);

            // Finally send the message to server_endpoint.py running in ROS
            ros.Publish(topicName, message);

            // Camera Info message
            CameraInfoMsg cameraInfoMessage = CameraInfoGenerator.ConstructCameraInfoMessage(ImageCamera, message.header, 0.0f, 0.01f);
            ros.Publish(cameraInfoTopicName, cameraInfoMessage);

            timeElapsed = 0;
        }



    }
}

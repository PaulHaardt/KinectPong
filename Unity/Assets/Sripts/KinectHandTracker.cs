using System;
using System.Linq;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using TMPro;
using UnityEngine;
using UnityEngine.UI;
using File = System.IO.File;
using UnityEditor;

namespace Sripts
{
    public class KinectHandTracker : MonoBehaviour
    {
        private int serverPort = 8888;
        private int clientPort = 8000;
        private string serverIP = "172.22.181.115";
        
        [Header("Paddle References")]
        public Transform leftPaddle;
        public Transform rightPaddle;
        
        [Header("Limits")]
        public RectTransform leftLimit;
        public RectTransform rightLimit;
        
        public RectTransform blueText;
        public RectTransform redText;
        public RectTransform timerText;
        
        private RectTransform leftPaddleRect;
        private RectTransform rightPaddleRect;
        
        [Header("Canvas Configuration")]
        public RectTransform canvasRect;
        public TMP_Text LeftCoordinateLogger;
        public TMP_Text RightCoordinateLogger;
        public TMP_Text rawJSONLogger;

        [Header("Movement Settings")]
        public float xMovementScale = 1.0f; // 1m movement range
    
        [Header("Smoothing")]
        public float smoothingFactor = 0.8f;
        [Range(1, 20), Tooltip("Average Frames")] public int averageFrames = 5;
        private UdpClient udpClient;
        private Thread udpThread;
        private bool isReceiving = false;
    
        // Hand coordinate data
        private Vector2 leftHandPos;
        private Vector2 rightHandPos;
        private Vector2 smoothedLeftPos;
        private Vector2 smoothedRightPos;
    
        // Smoothing arrays
        private Vector2[] leftHandHistory;
        private Vector2[] rightHandHistory;
        private int historyIndex = 0;
        
        // Thread-safe data exchange
        private readonly object dataLock = new object();
        
        // Thread-safe UI text updates
        private readonly object textLock = new object();
        private string pendingLeftText = "";
        private string pendingRightText = "";
        private string pendingRawJSONText = "";
        private bool hasLeftTextUpdate = false;
        private bool hasRightTextUpdate = false;
        private bool hasRawJSONUpdate = false;
        
        [Serializable]
        public struct DetectedObject
        {
            public float x, y, z;
            public int id;
        }

        [Serializable]
        public class CoordinatesData
        {
            public uint timestamp;
            public string mode;
            public DetectedObject[] hands;
            public DetectedObject[] objects;
        }

        private void Start()
        {
            InitializeNetwork();
            leftPaddleRect = leftPaddle.GetComponent<RectTransform>();
            rightPaddleRect = rightPaddle.GetComponent<RectTransform>();

            blueText.position = new Vector3(canvasRect.rect.width / 8, canvasRect.rect.height / 2, 0);
            blueText.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal,  canvasRect.rect.height / 2);
            blueText.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical,  canvasRect.rect.width / 8);


            redText.position = new Vector3(canvasRect.rect.width / 8 * 7, canvasRect.rect.height / 2, 0);
            redText.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal,  canvasRect.rect.height / 2);
            redText.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical,  canvasRect.rect.width / 8);

            timerText.position = new Vector3(canvasRect.rect.width / 2, canvasRect.rect.height / 2, 0);
            timerText.SetSizeWithCurrentAnchors(RectTransform.Axis.Horizontal, canvasRect.rect.width / 4);
            timerText.SetSizeWithCurrentAnchors(RectTransform.Axis.Vertical,  canvasRect.rect.height / 2);

            InitializeSmoothing();
            StartUDPListener();
        }

        private void InitializeNetwork()
        {
            if (!File.Exists(".env"))
            {
                Debug.LogWarning($"Environment file not found at {Application.dataPath}, using default settings.");
                return;
            }
            string[] envLines = File.ReadAllLines(".env");
            envLines.ToList().ForEach(line =>
            {
                if (line.StartsWith("UDP_SERVER_PORT="))
                {
                    if (int.TryParse(line.Substring("UDP_SERVER_PORT=".Length), out int port))
                    {
                        serverPort = port;
                    }
                }
                else if (line.StartsWith("UDP_CLIENT_PORT="))
                {
                    if (int.TryParse(line.Substring("UDP_CLIENT_PORT=".Length), out int port))
                    {
                        clientPort = port;
                    }
                }
                else if (line.StartsWith("UDP_SERVER_WSL="))
                {
                    serverIP = line.Substring("UDP_SERVER_WSL=".Length);
                }
                else if (line.StartsWith("UDP_SERVER_UBUNTU="))
                {
                    serverIP = line.Substring("UDP_SERVER_UBUNTU=".Length);
                }
            });
        }

        private void InitializeSmoothing()
        {
            leftHandHistory = new Vector2[averageFrames];
            rightHandHistory = new Vector2[averageFrames];
        
            for (int i = 0; i < averageFrames; i++)
            {
                leftHandHistory[i] = Vector2.zero;
                rightHandHistory[i] = Vector2.zero;
            }
        }

        void StartUDPListener()
        {
            try
            {
                udpClient = new UdpClient(clientPort);
                isReceiving = true;
            
                udpThread = new Thread(new ThreadStart(UDPListener));
                udpThread.IsBackground = true;
                udpThread.Start();
            
                Debug.Log($"UDP Client started on local port {clientPort}, connecting to server {serverIP}:{serverPort}");
                File.AppendAllText("log.txt", $"[{DateTime.Now}] UDP Client started on local port {clientPort}, connecting to server {serverIP}:{serverPort}\n");
            }
            catch (Exception e)
            {
                Debug.LogError($"Failed to start UDP client: {e.Message}");
            }
        }

        private void UDPListener()
        {
            IPAddress serverAddress;
            if (!IPAddress.TryParse(serverIP, out serverAddress))
            {
                Debug.LogError($"Invalid server IP address: {serverIP}");
                return;
            }
            
            Debug.Log($"Connecting to server at {serverAddress}:{serverPort}");
            Console.WriteLine($"Connecting to server at {serverAddress}:{serverPort}");
            
            IPEndPoint serverEndPoint = new IPEndPoint(serverAddress, serverPort);
            IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);
            
            if (!File.Exists("log.txt"))
            {
                File.WriteAllText("log.txt", $"[{DateTime.Now}] Log file created.\n");
            }
            File.AppendAllText("log.txt", $"[{DateTime.Now}] UDP Client started on local port {clientPort}, connecting to server {serverIP}:{serverPort}\n");

            // Send SUBSCRIBE message to server
            try
            {
                byte[] subscribeMessage = Encoding.UTF8.GetBytes("SUBSCRIBE");
                udpClient.Send(subscribeMessage, subscribeMessage.Length, serverEndPoint);
                Debug.Log("SUBSCRIBE message sent to server");
            }
            catch (Exception e)
            {
                Debug.LogError($"Failed to send SUBSCRIBE message: {e.Message}");
                return;
            }

            string jsonString = string.Empty;
            while (isReceiving)
            {
                byte[] data = udpClient.Receive(ref remoteEndPoint);
                jsonString = Encoding.UTF8.GetString(data);

                CoordinatesData coordinatesData;
                try
                {
                    coordinatesData = JsonUtility.FromJson<CoordinatesData>(jsonString);
                }
                catch (Exception e)
                {
                    Console.WriteLine(e);
                    throw;
                }

                lock (textLock)
                {
                    pendingRawJSONText = $"{serverAddress}: " + jsonString;
                    hasRawJSONUpdate = true;
                }
                    
                lock (dataLock)
                {
                    if (coordinatesData.hands != null)
                    {
                        foreach (var hand in coordinatesData.hands)
                        {
                            if (hand.id == 0) // Left hand
                            {
                                leftHandPos = new Vector2(hand.x, hand.y);

                                lock (textLock)
                                {
                                    pendingLeftText = $"x: {leftHandPos.x} y: {leftHandPos.y}";
                                    hasLeftTextUpdate = true;
                                }
                            }
                            else if (hand.id == 1) // Right hand
                            {
                                rightHandPos = new Vector2(hand.x, hand.y);
                                    
                                // Queue right coordinate text update
                                lock (textLock)
                                {
                                    pendingRightText = $"x: {rightHandPos.x} y: {rightHandPos.y}";
                                    hasRightTextUpdate = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        private void Update()
        {
            UpdateUITexts();
            
            lock (dataLock)
            {
                UpdateHandHistory();
                ApplySmoothing();
                MovePaddles();
            }
        }

        private void UpdateUITexts()
        {
            lock (textLock)
            {
                if (hasLeftTextUpdate)
                {
                    LeftCoordinateLogger.text = $"ip: {serverIP}\n{pendingLeftText}";
                    hasLeftTextUpdate = false;
                }
                
                if (hasRightTextUpdate)
                {
                    RightCoordinateLogger.text = $"ip: {serverPort}\n{pendingRightText}";
                    hasRightTextUpdate = false;
                }
                
                if (hasRawJSONUpdate)
                {
                    rawJSONLogger.text = pendingRawJSONText;
                    hasRawJSONUpdate = false;
                }
            }
        }
        
        private void UpdateHandHistory()
        {
            leftHandHistory[historyIndex] = leftHandPos;
            rightHandHistory[historyIndex] = rightHandPos;
        
            historyIndex = (historyIndex + 1) % averageFrames;
        }

        private void ApplySmoothing()
        {
            // Exponential smoothing
            smoothedLeftPos = Vector2.Lerp(smoothedLeftPos, leftHandPos, 1.0f - smoothingFactor);
            smoothedRightPos = Vector2.Lerp(smoothedRightPos, rightHandPos, 1.0f - smoothingFactor);
        
            // Temporal median smoothing (efficient selection)
            Vector2[] leftHistoryCopy = new Vector2[averageFrames];
            Vector2[] rightHistoryCopy = new Vector2[averageFrames];
            Array.Copy(leftHandHistory, leftHistoryCopy, averageFrames);
            Array.Copy(rightHandHistory, rightHistoryCopy, averageFrames);

            float[] leftX = leftHistoryCopy.Select(v => v.x).ToArray();
            float[] leftY = leftHistoryCopy.Select(v => v.y).ToArray();
            float[] rightX = rightHistoryCopy.Select(v => v.x).ToArray();
            float[] rightY = rightHistoryCopy.Select(v => v.y).ToArray();

            Vector2 leftMedian = new Vector2(Median(leftX), Median(leftY));
            Vector2 rightMedian = new Vector2(Median(rightX), Median(rightY));

            // Combine both smoothing methods
            smoothedLeftPos = Vector2.Lerp(smoothedLeftPos, leftMedian, 0.3f);
            smoothedRightPos = Vector2.Lerp(smoothedRightPos, rightMedian, 0.3f);
            return;

            float Median(float[] arr)
            {
                int n = arr.Length;
                Array.Sort(arr);
                if (n % 2 == 1) return arr[n / 2];
                return (arr[(n / 2) - 1] + arr[n / 2]) / 2f;
            }
        }

        private void MovePaddles()
        {
            if (!canvasRect) return;

            // Get canvas dimensions
            float canvasWidth = canvasRect.rect.width;
            float canvasHeight = canvasRect.rect.height;
            float paddleWidth = leftPaddleRect.rect.width;
            float paddleHeight = leftPaddleRect.rect.height;

            // Calculate clamp limits to prevent clipping
            float yClampMax = (canvasHeight) - (paddleHeight / 2);
            float yClampMin = (paddleHeight / 2);

            float xClampMax = (canvasWidth);
            float xClampMin = (paddleWidth / 2);
    
            if (leftPaddle)
            {
                Vector2 leftPos;
                
                float normalizedY = Mathf.Clamp01(smoothedLeftPos.y);
                leftPos.y = Mathf.Lerp(yClampMin, yClampMax, normalizedY);
        
                float normalizedX = Mathf.Clamp01(smoothedLeftPos.x);
                leftPos.x = Mathf.Lerp(xClampMin, xClampMax, normalizedX) * xMovementScale;
                leftPos.x = Mathf.Clamp(leftPos.x, xClampMin, leftLimit.position.x);
        
                leftPaddle.position = leftPos;
            }

            if (rightPaddle)
            {
                Vector3 rightPos = rightPaddle.position;
        
                float normalizedY = Mathf.Clamp01(smoothedRightPos.y);
                rightPos.y = Mathf.Lerp(yClampMin, yClampMax, normalizedY);
        
                float normalizedX = Mathf.Clamp01(smoothedRightPos.x);
                rightPos.x = Mathf.Lerp(xClampMin, xClampMax, normalizedX) * xMovementScale;
                rightPos.x = Mathf.Clamp(rightPos.x, rightLimit.position.x, xClampMax);
        
                rightPaddle.position = rightPos;
            }
        }

        private void OnApplicationQuit()
        {
            StopUDPListener();
        }

        private void OnDestroy()
        {
            StopUDPListener();
        }

        private void StopUDPListener()
        {
            isReceiving = false;
        
            if (udpThread != null && udpThread.IsAlive)
            {
                udpThread.Join(1000);
                if (udpThread.IsAlive)
                {
                    udpThread.Abort();
                }
            }
        
            if (udpClient != null)
            {
                udpClient.Close();
                udpClient = null;
            }
        }
    
        // Debug visualization in Scene view
        private void OnDrawGizmos()
        {
            Gizmos.color = Color.red;
            Gizmos.DrawWireSphere(new Vector3(smoothedLeftPos.x, smoothedLeftPos.y, 0), 0.1f);
        
            Gizmos.color = Color.blue;
            Gizmos.DrawWireSphere(new Vector3(smoothedRightPos.x, smoothedRightPos.y, 0), 0.1f);
        }
    }
}
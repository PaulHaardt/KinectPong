using System;
using System.Collections.Generic;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Threading;
using TMPro;
using Unity.VisualScripting;
using UnityEngine;
using UnityEngine.UI;

namespace Sripts
{
    public class KinectHandTracker : MonoBehaviour
    {
        [Header("UDP Configuration")]
        public int serverPort = 8888;
        public int localPort = 8888;
        public string serverIP = "172.22.181.115";
        
        [Header("Paddle References")]
        public Transform leftPaddle;
        public Transform rightPaddle;
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
        public int averageFrames = 5;
    
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
            leftPaddleRect = leftPaddle.GetComponent<RectTransform>();
            rightPaddleRect = rightPaddle.GetComponent<RectTransform>();
            InitializeSmoothing();
            StartUDPListener();
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
                udpClient = new UdpClient(localPort);
                isReceiving = true;
            
                udpThread = new Thread(new ThreadStart(UDPListener));
                udpThread.IsBackground = true;
                udpThread.Start();
            
                Debug.Log($"UDP Client started on local port {localPort}, connecting to server {serverIP}:{serverPort}");
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

            IPEndPoint serverEndPoint = new IPEndPoint(serverAddress, serverPort);
            IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);

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

            while (isReceiving)
            {
                byte[] data = udpClient.Receive(ref remoteEndPoint);
                string jsonString = Encoding.UTF8.GetString(data);
                    
                CoordinatesData coordinatesData = JsonUtility.FromJson<CoordinatesData>(jsonString);

                lock (textLock)
                {
                    pendingRawJSONText = jsonString;
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
                    LeftCoordinateLogger.text = pendingLeftText;
                    hasLeftTextUpdate = false;
                }
                
                if (hasRightTextUpdate)
                {
                    RightCoordinateLogger.text = pendingRightText;
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
        
            // Additional averaging smoothing
            Vector2 leftAverage = Vector2.zero;
            Vector2 rightAverage = Vector2.zero;
        
            for (int i = 0; i < averageFrames; i++)
            {
                leftAverage += leftHandHistory[i];
                rightAverage += rightHandHistory[i];
            }
        
            leftAverage /= averageFrames;
            rightAverage /= averageFrames;
        
            // Combine both smoothing methods
            smoothedLeftPos = Vector2.Lerp(smoothedLeftPos, leftAverage, 0.3f);
            smoothedRightPos = Vector2.Lerp(smoothedRightPos, rightAverage, 0.3f);
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
    
            float xClampMax = (canvasWidth) - (paddleWidth / 2);
            float xClampMin = (paddleWidth / 2);
    
            if (leftPaddle)
            {
                Vector2 leftPos;
                
                float normalizedY = Mathf.Clamp01(smoothedLeftPos.y);
                leftPos.y = Mathf.Lerp(yClampMin, yClampMax, normalizedY);
        
                float normalizedX = Mathf.Clamp01(smoothedLeftPos.x);
                leftPos.x = Mathf.Lerp(xClampMin, xClampMax, normalizedX) * xMovementScale;
        
                leftPaddle.position = leftPos;
            }

            if (rightPaddle)
            {
                Vector3 rightPos = rightPaddle.position;
        
                float normalizedY = Mathf.Clamp01(smoothedRightPos.y);
                rightPos.y = Mathf.Lerp(yClampMin, yClampMax, normalizedY);
        
                float normalizedX = Mathf.Clamp01(smoothedRightPos.x);
                rightPos.x = Mathf.Lerp(xClampMin, xClampMax, normalizedX) * xMovementScale;
        
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
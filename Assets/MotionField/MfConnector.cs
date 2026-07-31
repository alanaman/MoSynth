using System;
using System.Collections.Concurrent;
using System.Threading;
using MotionMatching;
using NetMQ;
using NetMQ.Sockets;
using Newtonsoft.Json;
using UnityEngine;

namespace MotionField
{
[Serializable]
public class MfConnector : MoSynthStage, IDisposable
{
    private Thread _clientThread;
    private bool _isRunning;
    
    // Thread-safe queues for communication between main thread and ZMQ thread
    private ConcurrentQueue<int> _frameRequests = new();
    private ConcurrentQueue<PoseVector> _receivedPoses = new();
    
    [SerializeField]
    private int port = 5555;
    
    
    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        // Required for NetMQ to run properly in Unity
        AsyncIO.ForceDotNet.Force(); 
        
        _isRunning = true;
        _clientThread = new Thread(ClientWorker);
        _clientThread.Start();
    }

    public override void Apply(PoseVector pose, float deltaTime)
    {
        int requestedFrame = Time.frameCount;
        _frameRequests.Enqueue(requestedFrame);
        Debug.Log($"Requested Frame: {requestedFrame}");

        // Process received poses on Unity's main thread
        while (_receivedPoses.TryDequeue(out PoseVector newPose))
        {
            Debug.Log($"Successfully received pose! Left Foot Contact: {newPose.LeftFootContact}");
            pose.CopyFrom(newPose);
        }
    }

    private void ClientWorker()
    {
        // Establish REQ socket
        using (var client = new RequestSocket())
        {
            client.Connect($"tcp://localhost:{port}");

            while (_isRunning)
            {
                if (_frameRequests.TryDequeue(out int frameRequest))
                {
                    // Send frame request to Python
                    client.SendFrame(frameRequest.ToString());
                    
                    // Block until Python replies
                    string message = client.ReceiveFrameString();
                    
                    try
                    {
                        // Deserialize JSON directly into your struct
                        PoseVector pose = JsonConvert.DeserializeObject<PoseVector>(message);
                        _receivedPoses.Enqueue(pose);
                    }
                    catch (Exception e)
                    {
                        Debug.LogError($"Error deserializing pose: {e.Message}");
                    }
                }
                else
                {
                    // Yield thread to prevent maxing out CPU core
                    Thread.Sleep(1); 
                }
            }
        }
        
        NetMQConfig.Cleanup();
    }

    public void Dispose()
    {
        _isRunning = false;
        
        // Wait for thread to finish gracefully
        if (_clientThread is { IsAlive: true })
        {
            _clientThread.Join();
        }
        
        NetMQConfig.Cleanup();
    }

    public override void OnDestroy()
    {
        Dispose();
    }
}
}
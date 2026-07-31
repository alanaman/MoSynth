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
    private Thread clientThread;
    private bool isRunning;
    
    // Thread-safe queues for communication between main thread and ZMQ thread
    private ConcurrentQueue<int> frameRequests = new ConcurrentQueue<int>();
    private ConcurrentQueue<PoseVector> receivedPoses = new ConcurrentQueue<PoseVector>();

    
    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        // Required for NetMQ to run properly in Unity
        AsyncIO.ForceDotNet.Force(); 
        
        isRunning = true;
        clientThread = new Thread(ClientWorker);
        clientThread.Start();
    }

    public override void Apply(PoseVector pose, float deltaTime)
    {
        int requestedFrame = Time.frameCount;
        frameRequests.Enqueue(requestedFrame);
        Debug.Log($"Requested Frame: {requestedFrame}");

        // Process received poses on Unity's main thread
        while (receivedPoses.TryDequeue(out PoseVector newPose))
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
            client.Connect("tcp://localhost:5555");

            while (isRunning)
            {
                if (frameRequests.TryDequeue(out int frameRequest))
                {
                    // Send frame request to Python
                    client.SendFrame(frameRequest.ToString());
                    
                    // Block until Python replies
                    string message = client.ReceiveFrameString();
                    
                    try
                    {
                        // Deserialize JSON directly into your struct
                        PoseVector pose = JsonConvert.DeserializeObject<PoseVector>(message);
                        receivedPoses.Enqueue(pose);
                    }
                    catch (System.Exception e)
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
        isRunning = false;
        
        // Wait for thread to finish gracefully
        if (clientThread != null && clientThread.IsAlive)
        {
            clientThread.Join();
        }
        
        NetMQConfig.Cleanup();
    }

    public override void OnDestroy()
    {
        Dispose();
    }
}
}
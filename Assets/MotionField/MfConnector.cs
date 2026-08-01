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
    private volatile bool _isRunning;
    private int _disposeState;
    private volatile string _workerError;
    
    // Thread-safe queues for communication between main thread and ZMQ thread
    private ConcurrentQueue<float> _deltaTimeRequests = new();
    private ConcurrentQueue<PoseVector> _receivedPoses = new();
    
    [SerializeField]
    private int port = 5555;

    [SerializeField]
    private float dequeueTimeoutMs = 2f;

    [SerializeField]
    [Min(1)]
    private int receivePollIntervalMs = 100;
    
    
    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        // A serialized stage can be initialized again after leaving and re-entering play mode.
        Interlocked.Exchange(ref _disposeState, 0);
        _workerError = null;
        ClearQueue(_deltaTimeRequests);
        ClearQueue(_receivedPoses);

        // Required for NetMQ to run properly in Unity
        AsyncIO.ForceDotNet.Force(); 
        
        _isRunning = true;
        _clientThread = new Thread(ClientWorker)
        {
            IsBackground = true,
            Name = nameof(MfConnector)
        };
        _clientThread.Start();
    }

    public override bool Apply(PoseVector pose, float deltaTime)
    {
        if (_workerError != null)
        {
            Debug.LogError($"MotionField client stopped: {_workerError}");
            _workerError = null;
        }

        // Keep at most one request queued while another request is waiting for a reply.
        // Otherwise this queue grows every frame whenever the server is unavailable.
        if (_isRunning && _deltaTimeRequests.IsEmpty)
        {
            _deltaTimeRequests.Enqueue(deltaTime);
            Debug.Log($"Requested Frame: {Time.frameCount}");
        }

        // Process received poses on Unity's main thread
        float dequeueStartTime = Time.realtimeSinceStartup;
        while (_receivedPoses.TryDequeue(out PoseVector newPose))
        {
            if ((Time.realtimeSinceStartup - dequeueStartTime) * 1000f >= dequeueTimeoutMs)
            {
                Debug.LogWarning($"Stopped dequeue loop after {dequeueTimeoutMs} ms timeout.");
                break;
            }

            Debug.Log($"Successfully received pose! Left Foot Contact: {newPose.LeftFootContact}");
            pose.CopyFrom(newPose);
            return true;
        }
        return false;
    }

    private void ClientWorker()
    {
        try
        {
            // The socket is created, used, and disposed on this thread. NetMQ sockets are
            // not thread-safe and must not be closed from Unity's main thread.
            using (var client = new RequestSocket())
            {
                client.Connect($"tcp://localhost:{port}");

                while (_isRunning)
                {
                    if (!_deltaTimeRequests.TryDequeue(out float deltaTime))
                    {
                        Thread.Sleep(1);
                        continue;
                    }

                    client.SendFrame(JsonConvert.SerializeObject(new
                    {
                        desired_dir = new[] { 0.0f, 1.0f },
                        delta_time = deltaTime
                    }));

                    // A REQ socket must receive its reply before sending again. Poll with a
                    // short timeout so play-mode shutdown can be observed between attempts.
                    while (_isRunning)
                    {
                        if (!client.TryReceiveFrameString(
                                TimeSpan.FromMilliseconds(Math.Max(1, receivePollIntervalMs)),
                                out string message))
                        {
                            continue;
                        }

                        PoseVector pose = JsonConvert.DeserializeObject<PoseVector>(message);
                        _receivedPoses.Enqueue(pose);
                        break;
                    }
                }
            }
        }
        catch (Exception exception)
        {
            if (_isRunning)
            {
                _workerError = exception.ToString();
            }
        }
        finally
        {
            _isRunning = false;
        }
    }

    public void Dispose()
    {
        if (Interlocked.Exchange(ref _disposeState, 1) != 0)
        {
            return;
        }

        _isRunning = false;
        
        // Receive polling lets this join normally finish within receivePollIntervalMs.
        if (_clientThread is { IsAlive: true })
        {
            if (!_clientThread.Join(TimeSpan.FromSeconds(2)))
            {
                Debug.LogError("MotionField client did not stop within two seconds.");
                return;
            }
        }

        _clientThread = null;
        NetMQConfig.Cleanup();
    }

    private static void ClearQueue<T>(ConcurrentQueue<T> queue)
    {
        while (queue.TryDequeue(out _))
        {
        }
    }

    public override void OnDestroy()
    {
        Dispose();
    }
}
}

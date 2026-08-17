using System;
using System.Collections.Generic;
using System.IO;
using Unity.Collections;
using UnityEngine;

namespace AnimationTools
{
/// <summary>When a sample is taken: once per <see cref="MotionSynthesisComponent"/> tick, or once per Unity frame.</summary>
public enum RecordTrigger
{
    EverySynthesisUpdate,
    EveryUnityFrame
}

/// <summary>
/// Records a configurable set of <see cref="RecorderChannel"/>s to a flat binary file plus a
/// JSON <see cref="RecordingManifest"/>, driven by <see cref="MotionSynthesisComponent.OnPoseApplied"/>.
/// Runs after every other synthesis-tied behaviour in its execution order so it always sees this
/// frame's fully-applied pose.
/// </summary>
[DefaultExecutionOrder(1000)]
public class MotionRecorder : MonoBehaviour
{
    [SerializeField] private MotionSynthesisComponent synthesizer;

    [SerializeReference] [SubclassSelector]
    public List<RecorderChannel> channels = new();

    public RecordTrigger trigger = RecordTrigger.EverySynthesisUpdate;

    [Tooltip("Relative paths resolve against the project root, next to Assets.")]
    public string outputDirectory = "Recordings";

    public string recordingName = "recording";
    public bool autoStartOnPlay = true;

    [Tooltip("0 = only flush on stop.")]
    public int flushEveryFrames = 300;

    public bool IsRecording { get; private set; }
    public int FrameCount => _frameIndex;
    public RecordingLayout Layout { get; private set; }

    private StateBuffer _frame;
    private ChannelHandle[] _handles;
    private FileStream _stream;
    private float[] _floatScratch;
    private byte[] _byteScratch;
    private PoseBuffer _lastPose;
    private string _createdUtc;
    private string _binPath;
    private string _manifestPath;
    private int _frameIndex;
    private float _elapsed;

    private void Awake()
    {
        if (synthesizer == null) synthesizer = GetComponent<MotionSynthesisComponent>();
    }

    private void Start()
    {
        if (autoStartOnPlay) StartRecording();
    }

    public void StartRecording()
    {
        if (IsRecording)
        {
            Debug.LogWarning($"MotionRecorder \"{name}\": StartRecording called while already recording.");
            return;
        }

        if (synthesizer == null)
        {
            Debug.LogError($"MotionRecorder \"{name}\": no MotionSynthesisComponent assigned.");
            return;
        }

        if (synthesizer.PoseLayout == null)
        {
            Debug.LogError($"MotionRecorder \"{name}\": synthesizer's PoseLayout is not initialized yet.");
            return;
        }

        channels.RemoveAll(c => c == null);
        if (channels.Count == 0)
        {
            Debug.LogError($"MotionRecorder \"{name}\": no channels configured.");
            return;
        }

        foreach (var channel in channels)
        {
            channel.Bind(synthesizer);
        }

        Layout = new RecordingLayout(channels);
        _handles = new ChannelHandle[channels.Count];
        for (var i = 0; i < channels.Count; i++)
        {
            _handles[i] = Layout.BindChannel(channels[i]);
        }

        _frame = StateBuffer.Allocate(Layout, Allocator.Persistent);
        _floatScratch = new float[Layout.FloatCount];
        _byteScratch = new byte[Layout.FloatCount * 4];

        var resolvedOutputDirectory = Path.IsPathRooted(outputDirectory)
            ? outputDirectory
            : Path.Combine(Directory.GetParent(Application.dataPath).FullName, outputDirectory);
        Directory.CreateDirectory(resolvedOutputDirectory);

        var baseFileName = $"{recordingName}_{DateTime.Now:yyyyMMdd_HHmmss}";
        _binPath = Path.Combine(resolvedOutputDirectory, baseFileName + ".bin");
        _manifestPath = Path.Combine(resolvedOutputDirectory, baseFileName + ".json");
        _stream = new FileStream(_binPath, FileMode.Create, FileAccess.Write, FileShare.Read, 1 << 16);

        _createdUtc = DateTime.UtcNow.ToString("o");
        _frameIndex = 0;
        _elapsed = 0f;
        _lastPose = default;

        synthesizer.OnPoseApplied += HandlePoseApplied;
        IsRecording = true;

        Debug.Log($"MotionRecorder \"{name}\": started recording to \"{_binPath}\".");
    }

    private void HandlePoseApplied(PoseBuffer pose, float dt)
    {
        _lastPose = pose;
        if (trigger == RecordTrigger.EverySynthesisUpdate) WriteSample(pose, dt);
    }

    private void LateUpdate()
    {
        if (trigger != RecordTrigger.EveryUnityFrame || !IsRecording) return;
        if (!_lastPose.IsCreated) return;
        WriteSample(_lastPose, Time.deltaTime);
    }

    private void WriteSample(PoseBuffer pose, float dt)
    {
        _elapsed += dt;
        var context = new RecorderSampleContext(synthesizer, pose, dt, _elapsed, _frameIndex);
        for (var i = 0; i < channels.Count; i++)
        {
            channels[i].Sample(in context, _handles[i], _frame);
        }

        _frame.Data.CopyTo(_floatScratch);
        Buffer.BlockCopy(_floatScratch, 0, _byteScratch, 0, _byteScratch.Length);
        _stream.Write(_byteScratch, 0, _byteScratch.Length);

        _frameIndex++;
        if (flushEveryFrames > 0 && _frameIndex % flushEveryFrames == 0) _stream.Flush();
    }

    public void StopRecording()
    {
        if (!IsRecording) return;

        if (synthesizer != null) synthesizer.OnPoseApplied -= HandlePoseApplied;

        if (_stream != null)
        {
            _stream.Flush();
            _stream.Dispose();
            _stream = null;
        }

        WriteManifest();

        if (_frame.IsCreated) _frame.Dispose();
        _lastPose = default;
        Layout = null;
        IsRecording = false;

        Debug.Log($"MotionRecorder \"{name}\": stopped recording ({_frameIndex} frames).");
    }

    private void WriteManifest()
    {
        var manifest = new RecordingManifest
        {
            recordingName = recordingName,
            createdUtc = _createdUtc,
            dataFile = Path.GetFileName(_binPath),
            trigger = trigger.ToString(),
            synthesisFrameRate = synthesizer != null ? synthesizer.synthesisFrameRate : 0f,
            frameFloatCount = Layout.FloatCount,
            frameCount = _frameIndex,
            channels = new RecordingManifestChannel[channels.Count]
        };

        for (var i = 0; i < channels.Count; i++)
        {
            var channel = channels[i];
            var entry = new RecordingManifestChannel
            {
                name = string.IsNullOrEmpty(channel.name) ? channel.GetType().Name : channel.name,
                type = channel.GetType().Name,
                floatOffset = _handles[i].FloatOffset,
                floatCount = _handles[i].FloatCount
            };
            channel.PopulateManifest(entry);
            manifest.channels[i] = entry;
        }

        RecordingManifest.Write(manifest, _manifestPath);
    }

    private void OnDisable()
    {
        StopRecording();
    }

    private void OnDestroy()
    {
        StopRecording();
    }
}
}

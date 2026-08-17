using System;
using System.Collections.Generic;
using System.Text;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Splines;

namespace AnimationTools
{
/// <summary>
/// Benchmark harness for path following: pushes one spline into a set of spline control inputs,
/// records each character's run with a MotionRecorder, then computes and logs trajectory error at
/// future time horizons, heading error against the spline tangent, and velocity error against the
/// input's target speed. Results are also shown read-only in the inspector.
/// The spline container is assumed unscaled.
/// </summary>
public class PathFollowingMetric : MonoBehaviour
{
    [SerializeField] private SplineContainer spline;

    [SerializeField] private List<IMotionSynthesisSplineControlInput> splineControlInputs = new();

    [Tooltip("Seconds to run before evaluating. 0 = run until StopAndEvaluate() is called.")]
    [SerializeField, Min(0f)] private float duration = 20f;

    [Tooltip("Initial seconds excluded from the metrics while the character reaches the path.")]
    [SerializeField, Min(0f)] private float settleTime = 2f;

    [SerializeField] private float[] horizonsSeconds = { 0f, 0.5f, 1f };

    [SerializeField] private string outputDirectory = "Recordings";

    [SerializeField] private bool recordFullPose;

    [SerializeField, Min(0f)] private float speedSmoothingWindow = 0.2f;

    [SerializeField, InspectorReadOnly] private List<PathFollowingMetricsResult> results = new();

    public bool IsRunning { get; private set; }
    public IReadOnlyList<PathFollowingMetricsResult> Results => results;

    private class Entry
    {
        public IMotionSynthesisSplineControlInput Input;
        public MonoBehaviour Behaviour;
        public MotionRecorder Recorder;
    }

    private readonly List<Entry> _entries = new();
    private float _startTime;

    // private void OnValidate()
    // {
    //     for (var i = 0; i < splineControlInputs.Count; i++)
    //     {
    //         var entry = splineControlInputs[i];
    //         if (entry == null) continue;
    //         if (entry is IMotionSynthesisSplineControlInput) continue;
    //
    //         Debug.LogError($"PathFollowingMetric \"{name}\": {entry.GetType().Name} on \"{entry.name}\" does not implement IMotionSynthesisSplineControlInput.", this);
    //         splineControlInputs[i] = null;
    //     }
    // }

    private void Start()
    {
        if (spline == null)
        {
            Debug.LogError($"PathFollowingMetric \"{name}\": no spline assigned.", this);
            enabled = false;
            return;
        }

        var validInputs = new List<IMotionSynthesisSplineControlInput>();
        foreach (var input in splineControlInputs)
        {
            if (input == null) continue;

            if (input.Synthesizer == null)
            {
                Debug.LogError($"PathFollowingMetric \"{name}\": {input.GetType().Name} has no Synthesizer assigned.", this);
                continue;
            }

            validInputs.Add(input);
        }

        if (validInputs.Count == 0)
        {
            Debug.LogError($"PathFollowingMetric \"{name}\": no valid spline control inputs configured.", this);
            enabled = false;
            return;
        }

        for (var i = 0; i < validInputs.Count; i++)
        {
            var input = validInputs[i];
            var behaviour = input as MonoBehaviour;
            Assert.IsNotNull(behaviour, $"PathFollowingMetric \"{name}\": {input.GetType().Name} is not a MonoBehaviour.");
            input.SplineContainer = spline;

            var recorder = gameObject.AddComponent<MotionRecorder>();
            recorder.autoStartOnPlay = false;
            recorder.Synthesizer = input.Synthesizer;
            recorder.trigger = RecordTrigger.EverySynthesisUpdate;
            recorder.outputDirectory = outputDirectory;
            recorder.recordingName = $"pathmetric_{behaviour.gameObject.name}_{i}";
            recorder.channels = new List<RecorderChannel>
            {
                new TimeChannel { name = "time" },
                new BoneWorldPositionChannel { name = "root", simulationBone = true },
                new BoneWorldForwardChannel { name = "rootForward", simulationBone = true }
            };
            if (recordFullPose) recorder.channels.Add(new FullPoseChannel { name = "pose" });

            recorder.StartRecording();
            if (!recorder.IsRecording)
            {
                Debug.LogError($"PathFollowingMetric \"{name}\": failed to start recording for \"{behaviour.gameObject.name}\".", this);
                Destroy(recorder);
                continue;
            }

            _entries.Add(new Entry { Input = input, Behaviour = behaviour, Recorder = recorder });
        }

        if (_entries.Count == 0)
        {
            Debug.LogError($"PathFollowingMetric \"{name}\": no recordings could be started.", this);
            enabled = false;
            return;
        }

        IsRunning = true;
        _startTime = Time.time;
        results.Clear();
    }

    private void Update()
    {
        if (IsRunning && duration > 0f && Time.time - _startTime >= duration) StopAndEvaluate();
    }

    public void StopAndEvaluate()
    {
        if (!IsRunning) return;
        IsRunning = false;

        foreach (var entry in _entries)
        {
            entry.Recorder.StopRecording();

            var label = $"{entry.Behaviour.gameObject.name} ({entry.Behaviour.GetType().Name})";
            try
            {
                var reader = RecordingReader.Load(entry.Recorder.ManifestPath);

                var timeCh = reader.GetChannel("time");
                var rootCh = reader.GetChannel("root");
                var fwdCh = reader.GetChannel("rootForward");

                var times = new float[reader.FrameCount];
                var positions = new float3[reader.FrameCount];
                var forwards = new float3[reader.FrameCount];
                for (var f = 0; f < reader.FrameCount; f++)
                {
                    times[f] = reader.GetFloat(f, timeCh, 0);
                    positions[f] = reader.GetFloat3(f, rootCh);
                    forwards[f] = reader.GetFloat3(f, fwdCh);
                }

                var result = PathFollowingMetricsCalculator.Evaluate(
                    spline.Spline,
                    (float4x4)spline.transform.localToWorldMatrix,
                    positions,
                    forwards,
                    times,
                    entry.Input.TargetSpeed,
                    horizonsSeconds,
                    settleTime,
                    speedSmoothingWindow);
                result.label = label;
                results.Add(result);

                if (result.framesEvaluated == 0)
                    Debug.LogWarning($"[PathFollowingMetric] {result.label}: not enough frames after settle time to evaluate.", this);
                else
                    Debug.Log(Format(result), this);
            }
            catch (Exception e)
            {
                Debug.LogError($"PathFollowingMetric: evaluation failed for {label}: {e}");
            }

            Destroy(entry.Recorder);
        }

        _entries.Clear();
    }

    private string Format(PathFollowingMetricsResult r)
    {
        var sb = new StringBuilder();
        sb.Append($"[PathFollowingMetric] {r.label}: {r.framesEvaluated} frames evaluated (settle {settleTime:0.0} s)\n");

        var horizonParts = new string[r.horizonsSeconds.Length];
        for (var h = 0; h < r.horizonsSeconds.Length; h++)
            horizonParts[h] = $"h={r.horizonsSeconds[h]:0.0}s: {r.meanTrajectoryError[h]:0.000} m";
        sb.Append($"  Trajectory error   {string.Join(" | ", horizonParts)}\n");

        sb.Append($"  Heading error      mean {r.meanHeadingErrorDeg:0.0} deg, max {r.maxHeadingErrorDeg:0.0} deg\n");

        sb.Append(float.IsNaN(r.targetSpeed)
            ? $"  Speed              actual {r.meanActualSpeed:0.00} m/s, target n/a (no speed model)"
            : $"  Speed              actual {r.meanActualSpeed:0.00} m/s, target {r.targetSpeed:0.00} m/s, mean abs error {r.meanVelocityError:0.00} m/s");

        return sb.ToString();
    }

    private void OnDisable()
    {
        if (!IsRunning) return;
        IsRunning = false;
        foreach (var entry in _entries) entry.Recorder.StopRecording();
        _entries.Clear();
    }
}
}

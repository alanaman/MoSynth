using System;
using System.Collections.Generic;
using Unity.Mathematics;
using UnityEngine.Splines;

namespace AnimationTools
{
/// <summary>
/// Output of <see cref="PathFollowingMetricsCalculator.Evaluate"/>: how closely a recorded run of
/// root positions/forwards tracked a spline, either as a shape (no target speed) or as a
/// shape-plus-pace (with a target speed used to project trajectory and velocity error).
/// </summary>
[Serializable]
public class PathFollowingMetricsResult
{
    public string label;
    public int framesEvaluated;

    /// <summary>Target pace, m/s. <see cref="float.NaN"/> when the run has no speed model.</summary>
    public float targetSpeed;

    /// <summary>Horizons actually evaluated; trimmed to just {0} when <see cref="targetSpeed"/> is NaN.</summary>
    public float[] horizonsSeconds;

    /// <summary>Mean trajectory error in meters, aligned index-for-index with <see cref="horizonsSeconds"/>.</summary>
    public float[] meanTrajectoryError;

    public float meanHeadingErrorDeg;
    public float maxHeadingErrorDeg;

    /// <summary>Mean actual speed in the XZ plane, m/s.</summary>
    public float meanActualSpeed;

    /// <summary>Mean |actual - target| speed, m/s. NaN when <see cref="targetSpeed"/> is NaN.</summary>
    public float meanVelocityError;
}

/// <summary>
/// Pure metrics for how well a recorded run followed a spline path. Everything here is XZ-flattened
/// (height is ignored) and the spline container transform is assumed unscaled, same as
/// <see cref="MotionField.MotionFieldSplineControlInput"/> — scaling it would skew arc-length math.
/// </summary>
public static class PathFollowingMetricsCalculator
{
    /// <summary>
    /// Evaluates path-following quality for one run against one spline.
    ///
    /// Three families of metric are computed, all XZ-flattened:
    /// - <b>Trajectory error</b>: for each horizon h and each analysis frame t, the point the
    ///   follower is expected to reach at time t+h is "walk h seconds' worth of target speed along
    ///   the spline from the point nearest the root at t"; the error is the XZ distance between
    ///   that expected point and where the root actually was at t+h (found by time, not by frame
    ///   count, so it is robust to variable frame rate). Horizon 0 is a pure shape metric — it
    ///   never depends on target speed. With no speed model only horizon 0 is evaluated, since
    ///   there is no pace to project forward with.
    /// - <b>Heading error</b>: the angle between the root's forward direction and the spline's
    ///   tangent at the nearest point, per analysis frame.
    /// - <b>Velocity error</b>: the (smoothed) actual XZ speed, finite-differenced from consecutive
    ///   analysis-frame positions, compared against the constant target speed.
    ///
    /// Frames before <paramref name="settleTime"/> are dropped so start-up transients don't skew
    /// the result. All inputs are world-space; the spline is evaluated in its own local space via
    /// <paramref name="splineLocalToWorld"/>, matching <see cref="SplineUtility.GetNearestPoint"/>'s
    /// contract.
    /// </summary>
    public static PathFollowingMetricsResult Evaluate(
        Spline spline,
        float4x4 splineLocalToWorld,
        float3[] rootPositions,
        float3[] rootForwards,
        float[] times,
        float targetSpeed,
        IReadOnlyList<float> horizonsSeconds,
        float settleTime,
        float speedSmoothingWindowSeconds)
    {
        var hasSpeed = !float.IsNaN(targetSpeed);
        var effectiveHorizons = hasSpeed ? ToArray(horizonsSeconds) : new[] { 0f };

        var result = new PathFollowingMetricsResult
        {
            targetSpeed = targetSpeed,
            horizonsSeconds = effectiveHorizons,
        };

        var analysisFrames = new List<int>(times.Length);
        for (var t = 0; t < times.Length; t++)
        {
            if (times[t] >= settleTime) analysisFrames.Add(t);
        }

        if (analysisFrames.Count < 2)
        {
            result.framesEvaluated = 0;
            result.meanTrajectoryError = FilledWithNaN(effectiveHorizons.Length);
            result.meanHeadingErrorDeg = float.NaN;
            result.maxHeadingErrorDeg = float.NaN;
            result.meanActualSpeed = float.NaN;
            result.meanVelocityError = float.NaN;
            return result;
        }

        var worldToLocal = math.inverse(splineLocalToWorld);
        var length = spline.GetLength();

        var errorSums = new double[effectiveHorizons.Length];
        var errorCounts = new int[effectiveHorizons.Length];
        var horizonPointers = new int[effectiveHorizons.Length];

        var headingSum = 0.0;
        var headingCount = 0;
        var maxHeading = float.NegativeInfinity;

        for (var i = 0; i < analysisFrames.Count; i++)
        {
            var t = analysisFrames[i];
            var localRoot = math.transform(worldToLocal, rootPositions[t]);
            // The default pick resolution (4 samples/curve) biases the nearest point several
            // centimeters along the curve, which shows up directly in the trajectory error.
            SplineUtility.GetNearestPoint(spline, localRoot, out float3 nearestLocal, out var nearestT,
                SplineUtility.PickResolutionMax, 4);

            for (var h = 0; h < effectiveHorizons.Length; h++)
            {
                var horizon = effectiveHorizons[h];
                var targetTime = times[t] + horizon;

                var j = horizonPointers[h];
                if (j < t) j = t;
                while (j < times.Length && times[j] < targetTime) j++;
                horizonPointers[h] = j;
                if (j >= times.Length) continue;

                float3 expectedLocal;
                if (horizon == 0f)
                {
                    expectedLocal = nearestLocal;
                }
                else
                {
                    var nearestDist = spline.ConvertIndexUnit(nearestT, PathIndexUnit.Normalized, PathIndexUnit.Distance);
                    var targetDist = nearestDist + targetSpeed * horizon;
                    if (spline.Closed)
                    {
                        targetDist %= length;
                        if (targetDist < 0f) targetDist += length;
                    }
                    else
                    {
                        targetDist = math.clamp(targetDist, 0f, length);
                    }

                    var expectedT = spline.ConvertIndexUnit(targetDist, PathIndexUnit.Distance, PathIndexUnit.Normalized);
                    expectedLocal = spline.EvaluatePosition(expectedT);
                }

                var expectedWorld = math.transform(splineLocalToWorld, expectedLocal);
                var actualWorld = rootPositions[j];
                var error = math.distance(new float2(expectedWorld.x, expectedWorld.z), new float2(actualWorld.x, actualWorld.z));
                errorSums[h] += error;
                errorCounts[h]++;
            }

            var tangentLocal = spline.EvaluateTangent(nearestT);
            if (math.lengthsq(tangentLocal) >= 1e-10f)
            {
                var tangentWorld = math.mul(splineLocalToWorld, new float4(tangentLocal, 0f)).xyz;
                var tangentXZ = new float2(tangentWorld.x, tangentWorld.z);
                var forward = rootForwards[t];
                var forwardXZ = new float2(forward.x, forward.z);
                if (math.lengthsq(tangentXZ) >= 1e-10f && math.lengthsq(forwardXZ) >= 1e-10f)
                {
                    var a = math.normalize(tangentXZ);
                    var b = math.normalize(forwardXZ);
                    // atan2 form: acos(dot) loses precision for nearly-parallel directions.
                    var cross = a.x * b.y - a.y * b.x;
                    var angleDeg = math.degrees(math.atan2(math.abs(cross), math.dot(a, b)));
                    headingSum += math.abs(angleDeg);
                    headingCount++;
                    if (angleDeg > maxHeading) maxHeading = angleDeg;
                }
            }
        }

        var meanTrajectoryError = new float[effectiveHorizons.Length];
        for (var h = 0; h < effectiveHorizons.Length; h++)
        {
            meanTrajectoryError[h] = errorCounts[h] > 0 ? (float)(errorSums[h] / errorCounts[h]) : float.NaN;
        }

        result.meanHeadingErrorDeg = headingCount > 0 ? (float)(headingSum / headingCount) : float.NaN;
        result.maxHeadingErrorDeg = headingCount > 0 ? maxHeading : float.NaN;

        var rawSpeeds = new List<float>(analysisFrames.Count);
        var dtSum = 0.0;
        for (var i = 0; i < analysisFrames.Count - 1; i++)
        {
            var t0 = analysisFrames[i];
            var t1 = analysisFrames[i + 1];
            var dt = times[t1] - times[t0];
            if (dt <= 0f) continue;

            var p0 = rootPositions[t0];
            var p1 = rootPositions[t1];
            var dist = math.distance(new float2(p1.x, p1.z), new float2(p0.x, p0.z));
            rawSpeeds.Add(dist / dt);
            dtSum += dt;
        }

        float meanActualSpeed;
        float meanVelocityError;
        if (rawSpeeds.Count == 0)
        {
            meanActualSpeed = float.NaN;
            meanVelocityError = float.NaN;
        }
        else
        {
            var meanDt = (float)(dtSum / rawSpeeds.Count);
            var window = meanDt > 0f ? math.max(1, (int)math.round(speedSmoothingWindowSeconds / meanDt)) : 1;
            window = math.min(window, rawSpeeds.Count);

            var smoothed = SmoothCentered(rawSpeeds, window);

            var speedSum = 0.0;
            var velocityErrorSum = 0.0;
            for (var i = 0; i < smoothed.Length; i++)
            {
                speedSum += smoothed[i];
                if (hasSpeed) velocityErrorSum += math.abs(smoothed[i] - targetSpeed);
            }

            meanActualSpeed = (float)(speedSum / smoothed.Length);
            meanVelocityError = hasSpeed ? (float)(velocityErrorSum / smoothed.Length) : float.NaN;
        }

        result.framesEvaluated = analysisFrames.Count;
        result.meanTrajectoryError = meanTrajectoryError;
        result.meanActualSpeed = meanActualSpeed;
        result.meanVelocityError = meanVelocityError;
        return result;
    }

    private static float[] SmoothCentered(List<float> raw, int window)
    {
        var smoothed = new float[raw.Count];
        var half = window / 2;
        for (var i = 0; i < raw.Count; i++)
        {
            var lo = math.max(0, i - half);
            var hi = math.min(raw.Count - 1, i + half);
            var sum = 0.0;
            for (var k = lo; k <= hi; k++) sum += raw[k];
            smoothed[i] = (float)(sum / (hi - lo + 1));
        }
        return smoothed;
    }

    private static float[] ToArray(IReadOnlyList<float> horizons)
    {
        var array = new float[horizons.Count];
        for (var i = 0; i < horizons.Count; i++) array[i] = horizons[i];
        return array;
    }

    private static float[] FilledWithNaN(int count)
    {
        var array = new float[count];
        for (var i = 0; i < count; i++) array[i] = float.NaN;
        return array;
    }
}
}

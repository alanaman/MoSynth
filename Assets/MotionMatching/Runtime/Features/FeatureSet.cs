using System;
using AnimationTools;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;

namespace MotionMatching
{
using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;
using PoseFeature = MotionMatchingData.PoseFeature;

/// <summary>
/// Stores all features vectors of all poses for Motion Matching
/// </summary>
public class FeatureSet
{
    private readonly MmFeatureLayout _featureLayout;

    public int NumberFeatureVectors { get; private set; } // Total number of feature vectors

    /// <summary>Total size in floats of a feature vector.</summary>
    public int FeatureSize => _featureLayout.FloatCount;

    /// <summary>Number of different trajectory features (e.g. 2 = position and direction).</summary>
    public int NumberTrajectoryFeatures => _featureLayout.TrajectoryHandles.Length;

    /// <summary>Number of different pose features (e.g. 3 = leftFootPosition, leftFootVelocity, hipsVelocity).</summary>
    public int PoseFeatureCount => _featureLayout.PoseHandles.Length;

    public const int FloatsPerPoseFeature = 3;
    public int PoseFloatCount => PoseFeatureCount * FloatsPerPoseFeature;

    /// <summary>
    /// Offset of the pose features in the feature vector. Pose features always follow the
    /// trajectory features.
    /// </summary>
    public int PoseOffset => _featureLayout.PoseStart;

    private NativeArray<bool> _valid; // TODO: Refactor to avoid needing this
    private PoseSequence _features; // One frame per pose: trajectory features then pose features
    private float[] _mean; // Size: FeatureSize
    private float[] _standardDeviation; // Size: FeatureSize

    // BVH acceleration structures
    private NativeArray<float> _largeBoundingBoxMin;
    private NativeArray<float> _largeBoundingBoxMax;
    private NativeArray<float> _smallBoundingBoxMin;
    private NativeArray<float> _smallBoundingBoxMax;

    public FeatureSet(MotionMatchingData mmData)
    {
        var poseSet = mmData.GetOrImportPoseSet();
        NumberFeatureVectors = poseSet.NumberPoses;
        _featureLayout = MmFeatureLayoutBuilder.Build(mmData, poseSet.SkeletonAsset);
    }

    /// <summary>Float offset of one prediction of one trajectory feature within a feature vector.</summary>
    public int GetTrajectoryFeatureOffset(int trajectoryFeatureIndex, int predictionIndex)
    {
        return _featureLayout.TrajectoryHandles[trajectoryFeatureIndex][predictionIndex].FloatOffset;
    }

    /// <summary>Floats occupied by one prediction of a trajectory feature.</summary>
    public int GetTrajectoryFeatureFloatCount(int trajectoryFeatureIndex)
    {
        var handles = _featureLayout.TrajectoryHandles[trajectoryFeatureIndex];
        return handles.Length > 0 ? handles[0].FloatCount : 0;
    }

    public int GetPredictionCount(int trajectoryFeatureIndex)
    {
        return _featureLayout.TrajectoryHandles[trajectoryFeatureIndex].Length;
    }

    public bool IsValidFeature(int featureIndex)
    {
        return _valid[featureIndex];
    }

    public void GetFeature(NativeArray<float> feature, int featureIndex)
    {
        Debug.Assert(feature.Length == FeatureSize, "Feature vector has wrong size");
        for (var i = 0; i < FeatureSize; i++)
        {
            feature[i] = _features.Data[featureIndex * FeatureSize + i];
        }
    }

    public void GetPoseFeatures(Span<float> poseFeatures, int frameIndex, bool denormalize = false)
    {
        Debug.Assert(poseFeatures.Length == PoseFloatCount, "Feature vector has wrong size");

        for (var i = 0; i < PoseFloatCount; i++)
        {
            poseFeatures[i] = _features.Data[frameIndex * FeatureSize + PoseOffset + i];
            if (denormalize)
            {
                poseFeatures[i] = poseFeatures[i] * _standardDeviation[PoseOffset + i] + _mean[PoseOffset + i];
            }
        }
    }

    public ReadOnlySpan<float> GetFeatureVector(int featureIndex)
    {
        return _features.Data.AsReadOnlySpan().Slice(featureIndex * FeatureSize, FeatureSize);
    }

    public float Get1DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex,
        bool denormalize = false)
    {
        var featureOffset = GetTrajectoryFeatureOffset(trajectoryFeatureIndex, predictionIndex);
        var startIndex = featureIndex * FeatureSize + featureOffset;
        var x = _features.Data[startIndex];
        if (denormalize)
        {
            x = x * _standardDeviation[featureOffset] + _mean[featureOffset];
        }

        return x;
    }

    public float2 Get2DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex,
        bool denormalize = false)
    {
        var featureOffset = GetTrajectoryFeatureOffset(trajectoryFeatureIndex, predictionIndex);
        var startIndex = featureIndex * FeatureSize + featureOffset;
        var x = _features.Data[startIndex];
        var y = _features.Data[startIndex + 1];
        if (denormalize)
        {
            x = x * _standardDeviation[featureOffset] + _mean[featureOffset];
            y = y * _standardDeviation[featureOffset + 1] + _mean[featureOffset + 1];
        }

        return new float2(x, y);
    }

    public float3 Get3DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex,
        bool denormalize = false)
    {
        var featureOffset = GetTrajectoryFeatureOffset(trajectoryFeatureIndex, predictionIndex);
        var startIndex = featureIndex * FeatureSize + featureOffset;
        var x = _features.Data[startIndex];
        var y = _features.Data[startIndex + 1];
        var z = _features.Data[startIndex + 2];
        if (denormalize)
        {
            x = x * _standardDeviation[featureOffset] + _mean[featureOffset];
            y = y * _standardDeviation[featureOffset + 1] + _mean[featureOffset + 1];
            z = z * _standardDeviation[featureOffset + 2] + _mean[featureOffset + 2];
        }

        return new float3(x, y, z);
    }

    public float3 GetPoseFeature(int featureIndex, int poseFeatureIndex, bool denormalize = false)
    {
        var featureOffset = _featureLayout.PoseHandles[poseFeatureIndex].FloatOffset;
        var startIndex = featureIndex * FeatureSize + featureOffset;
        var x = _features.Data[startIndex];
        var y = _features.Data[startIndex + 1];
        var z = _features.Data[startIndex + 2];
        if (denormalize)
        {
            x = x * _standardDeviation[featureOffset] + _mean[featureOffset];
            y = y * _standardDeviation[featureOffset + 1] + _mean[featureOffset + 1];
            z = z * _standardDeviation[featureOffset + 2] + _mean[featureOffset + 2];
        }

        return new float3(x, y, z);
    }

    public NativeArray<bool> GetValid()
    {
        return _valid;
    }

    public NativeArray<float> GetFeatures()
    {
        return _features != null ? _features.Data : default;
    }

    public float GetMean(int dimension)
    {
        return _mean[dimension];
    }

    public float[] GetMeans()
    {
        return _mean;
    }

    public float GetStandardDeviation(int dimension)
    {
        return _standardDeviation[dimension];
    }

    public float[] GetStandardDeviations()
    {
        return _standardDeviation;
    }

    public void GetBvhBuffers(out NativeArray<float> largeBoundingBoxMin,
        out NativeArray<float> largeBoundingBoxMax,
        out NativeArray<float> smallBoundingBoxMin,
        out NativeArray<float> smallBoundingBoxMax)
    {
        if (!_largeBoundingBoxMax.IsCreated)
        {
            // Build BVH Acceleration Structure
            var nFrames = GetFeatures().Length / FeatureSize;
            var numberBoundingBoxLarge = (nFrames + BVHConsts.LargeBVHSize - 1) / BVHConsts.LargeBVHSize;
            var numberBoundingBoxSmall = (nFrames + BVHConsts.SmallBVHSize - 1) / BVHConsts.SmallBVHSize;
            _largeBoundingBoxMin = new NativeArray<float>(numberBoundingBoxLarge * FeatureSize, Allocator.Domain);
            _largeBoundingBoxMax = new NativeArray<float>(numberBoundingBoxLarge * FeatureSize, Allocator.Domain);
            _smallBoundingBoxMin = new NativeArray<float>(numberBoundingBoxSmall * FeatureSize, Allocator.Domain);
            _smallBoundingBoxMax = new NativeArray<float>(numberBoundingBoxSmall * FeatureSize, Allocator.Domain);
            var job = new BVHMotionMatchingComputeBounds
            {
                Features = GetFeatures(),
                FeatureSize = FeatureSize,
                NumberBoundingBoxLarge = numberBoundingBoxLarge,
                NumberBoundingBoxSmall = numberBoundingBoxSmall,
                LargeBoundingBoxMin = _largeBoundingBoxMin,
                LargeBoundingBoxMax = _largeBoundingBoxMax,
                SmallBoundingBoxMin = _smallBoundingBoxMin,
                SmallBoundingBoxMax = _smallBoundingBoxMax,
            };
            job.Schedule().Complete();
        }

        largeBoundingBoxMin = _largeBoundingBoxMin;
        largeBoundingBoxMax = _largeBoundingBoxMax;
        smallBoundingBoxMin = _smallBoundingBoxMin;
        smallBoundingBoxMax = _smallBoundingBoxMax;
    }

    // Deserialize ---------------------------------------
    public void SetValid(NativeArray<bool> valid)
    {
        Debug.Assert(valid.Length == NumberFeatureVectors, "Valid array has wrong size");
        if (_valid.IsCreated)
        {
            _valid.Dispose();
        }

        _valid = valid;
    }

    public void SetFeatures(NativeArray<float> features)
    {
        Debug.Assert(features.Length == NumberFeatureVectors * FeatureSize, "Feature vector has wrong size");
        _features?.Dispose();
        _features = new PoseSequence(_featureLayout.Layout, features);
    }

    public void SetMean(float[] mean)
    {
        Debug.Assert(mean.Length == FeatureSize, mean.Length + " != " + FeatureSize);
        _mean = mean;
    }

    public void SetStandardDeviation(float[] standardDeviation)
    {
        Debug.Assert(standardDeviation.Length == FeatureSize, standardDeviation.Length + " != " + FeatureSize);
        _standardDeviation = standardDeviation;
    }
    // --------------------------------------------------

    /// <summary>
    /// Normalizes the trajectory features (pose features remaing untouched)
    /// </summary>
    public void NormalizeTrajectory(Span<float> featureVector)
    {
        Debug.Assert(_mean != null, "Mean is not initialized");
        Debug.Assert(_standardDeviation != null, "StandardDeviation is not initialized");
        Debug.Assert(featureVector.Length == FeatureSize, "Feature vector size does not match");

        for (var i = 0; i < PoseOffset; i++)
        {
            featureVector[i] = (featureVector[i] - _mean[i]) / _standardDeviation[i];
        }
    }

    /// <summary>
    /// Normalizes all features (trajectory + pose)
    /// </summary>
    public void NormalizeFeatureVector(NativeArray<float> featureVector)
    {
        Debug.Assert(_mean != null, "Mean is not initialized");
        Debug.Assert(_standardDeviation != null, "StandardDeviation is not initialized");
        Debug.Assert(featureVector.Length == FeatureSize, "Feature vector size does not match");

        for (var i = 0; i < FeatureSize; i++)
        {
            featureVector[i] = (featureVector[i] - _mean[i]) / _standardDeviation[i];
        }
    }

    /// <summary>
    /// Returns a copy of the feature vector with the features before normalization
    /// </summary>
    public void DenormalizeFeatureVector(NativeArray<float> featureVector)
    {
        Debug.Assert(_mean != null, "Mean is not initialized");
        Debug.Assert(_standardDeviation != null, "StandardDeviation is not initialized");
        Debug.Assert(featureVector.Length == FeatureSize, "Feature vector size does not match");

        for (var i = 0; i < FeatureSize; i++)
        {
            featureVector[i] = featureVector[i] * _standardDeviation[i] + _mean[i];
        }
    }

    /// <summary>
    /// Normalizes the features by subtracting mean and dividing by the standard deviation
    /// </summary>
    public void NormalizeFeatures()
    {
        // Compute Mean and Standard Deviation
        ComputeMeanAndStandardDeviation();

        // Normalize all feature vectors
        var features = _features.Data;
        for (var i = 0; i < NumberFeatureVectors; i++)
        {
            var featureIndex = i * FeatureSize;
            if (_valid[i])
            {
                for (var j = 0; j < FeatureSize; j++)
                {
                    features[featureIndex + j] = (features[featureIndex + j] - _mean[j]) / _standardDeviation[j];
                }
            }
        }
    }

    private void ComputeMeanAndStandardDeviation()
    {
        var nTotalDimensions = FeatureSize;
        // Mean for each dimension
        _mean = new float[nTotalDimensions];
        // Variance for each dimension
        Span<float> variance = stackalloc float[nTotalDimensions];
        // Standard Deviation for each dimension
        _standardDeviation = new float[nTotalDimensions];

        // Compute Means for each dimension of each feature
        var count = 0;
        for (var i = 0; i < NumberFeatureVectors; i++)
        {
            if (_valid[i])
            {
                var featureIndex = i * FeatureSize;
                for (var j = 0; j < nTotalDimensions; j++)
                {
                    _mean[j] += _features.Data[featureIndex + j];
                }

                count += 1;
            }
        }

        for (var i = 0; i < nTotalDimensions; i++)
        {
            _mean[i] /= count;
        }

        // Compute Variance for each dimension of each feature - variance = (x - mean)^2 / n
        for (var i = 0; i < NumberFeatureVectors; i++)
        {
            var featureIndex = i * FeatureSize;
            if (_valid[i])
            {
                for (var j = 0; j < nTotalDimensions; j++)
                {
                    var diff = _features.Data[featureIndex + j] - _mean[j];
                    variance[j] += diff * diff;
                }
            }
        }

        for (var i = 0; i < nTotalDimensions; i++)
        {
            variance[i] /= count;
        }

        // Compute Standard Deviations of a feature as the average std across all dimensions - std = sqrt(variance)
        for (var d = 0; d < NumberTrajectoryFeatures; d++)
        {
            var predictionCount = GetPredictionCount(d);
            if (predictionCount == 0) continue;

            var offset = GetTrajectoryFeatureOffset(d, 0);
            var nDimensions = predictionCount * GetTrajectoryFeatureFloatCount(d);
            float std = 0;
            for (var j = 0; j < nDimensions; j++)
            {
                std += math.sqrt(variance[offset + j]);
            }

            std /= nDimensions;
            Debug.Assert(std > 0, "Standard deviation is zero, feature with no variation is probably a bug");
            if (std <= 0)
            {
                std = 1.0f;
            }

            for (var j = 0; j < nDimensions; j++)
            {
                _standardDeviation[offset + j] = std;
            }
        }

        for (var d = 0; d < PoseFeatureCount; d++)
        {
            var offset = _featureLayout.PoseHandles[d].FloatOffset;
            float std = 0;
            for (var j = 0; j < FloatsPerPoseFeature; j++)
            {
                std += math.sqrt(variance[offset + j]);
            }

            std /= FloatsPerPoseFeature;
            Debug.Assert(std > 0, "Standard deviation is zero, feature with no variation is probably a bug");
            if (std <= 0)
            {
                std = 1.0f;
            }

            for (var j = 0; j < FloatsPerPoseFeature; j++)
            {
                _standardDeviation[offset + j] = std;
            }
        }
    }

    /// <summary>
    /// Extract the feature vectors from poseSet
    /// </summary>
    public void Extract(PoseSet poseSet, MotionMatchingData mmData)
    {
        var nPoses = poseSet.NumberPoses;
        _valid = new NativeArray<bool>(nPoses, Allocator.Domain);
        _features = new PoseSequence(_featureLayout.Layout,
            new NativeArray<float>(nPoses * FeatureSize, Allocator.Domain));

        for (var poseIndex = 0; poseIndex < nPoses; ++poseIndex)
        {
            if (poseSet.IsPoseValidForPrediction(poseIndex, mmData.MaximumFramesPrediction))
            {
                _valid[poseIndex] = true;
                ExtractFeatures(poseSet, poseIndex, mmData);
            }
            else _valid[poseIndex] = false;
        }
    }

    private void ExtractFeatures(PoseSet poseSet, int poseIndex, MotionMatchingData mmData)
    {
        var nextPose = poseIndex + 1;
        if (nextPose >= poseSet.NumberPoses - mmData.MaximumFramesPrediction)
        {
            nextPose = poseIndex;
        }

        var frame = _features.GetFrame(poseIndex);
        var currentPose = poseSet.GetPoseBuffer(poseIndex);
        var poseNext = poseSet.GetPoseBuffer(nextPose);
        var skeletonData = poseSet.SkeletonAsset.GetSkeletonData();
        // Compute local features based on the Simulation Bone
        // so hips and feet are local to a stable position with respect to the character
        GetWorldOriginCharacter(currentPose, out var characterOrigin, out var characterForward);

        // Trajectory Features -------------------------------------------------------------
        for (var i = 0; i < NumberTrajectoryFeatures; i++)
        {
            var trajectoryFeature = mmData.trajectoryFeatures[i];
            var channels = _featureLayout.TrajectoryChannels[i];
            var handles = _featureLayout.TrajectoryHandles[i];

            for (var p = 0; p < channels.Length; ++p)
            {
                var futurePose = poseSet.GetPoseBuffer(poseIndex + trajectoryFeature.framesPrediction[p]);
                ExtractTrajectoryFeature(channels[p], handles[p], frame, futurePose, skeletonData, mmData,
                    characterOrigin, characterForward);
            }
        }

        // Pose Features -------------------------------------------------------------
        for (var i = 0; i < PoseFeatureCount; i++)
        {
            var channel = _featureLayout.PoseChannels[i];
            var feature = float3.zero;
            switch (channel.FeatureType)
            {
                case PoseFeature.Type.Position:
                    feature = GetJointPosition(currentPose, skeletonData, channel.BoneIndex, characterOrigin,
                        characterForward);
                    break;
                case PoseFeature.Type.Velocity:
                    feature = GetJointVelocity(currentPose, poseNext, skeletonData, channel.BoneIndex, characterOrigin,
                        characterForward, poseSet.FrameTime);
                    break;
                default:
                    Debug.Assert(false, "Unknown PoseFeature.Type: " + channel.FeatureType);
                    break;
            }

            frame.SetFloat3(_featureLayout.PoseHandles[i], feature);
        }
    }

    private static void ExtractTrajectoryFeature(TrajectoryFeatureChannel channel, ChannelHandle handle,
        PoseBuffer frame, PoseBuffer futurePose, in SkeletonData skeletonData, MotionMatchingData mmData,
        float3 characterOrigin, float3 characterForward)
    {
        var value = float3.zero;
        switch (channel.FeatureType)
        {
            case TrajectoryFeature.Type.Position:
            {
                value = GetTrajectoryPosition(futurePose, skeletonData, channel.BoneIndex,
                    characterOrigin, characterForward);
            }
                break;
            case TrajectoryFeature.Type.Direction:
            {
                GetTrajectoryDirection(futurePose, skeletonData, channel.SimulationBone, channel.BoneIndex,
                    characterForward, mmData, out value);
                if (channel.ZeroX) value.x = 0;
                if (channel.ZeroY) value.y = 0;
                if (channel.ZeroZ) value.z = 0;
                value = math.normalize(value);
            }
                break;
            default:
                Debug.Assert(false, "Unsupported Feature Type: " + channel.FeatureType);
                break;
        }

        var valueIndex = 0;
        for (var f = 0; f < handle.FloatCount; ++f)
        {
            if (valueIndex == 0 && channel.ZeroX) valueIndex += 1;
            if (valueIndex == 1 && channel.ZeroY) valueIndex += 1;
            frame.SetFloat(handle, f, value[valueIndex]);
            valueIndex += 1;
        }
    }

    private static float3 GetTrajectoryPosition(PoseBuffer futurePose, in SkeletonData skeleton,
        int jointIndex, float3 characterOrigin, float3 characterForward)
    {
        var worldPosition = PoseBufferFK.WorldPosition(skeleton, futurePose, jointIndex);

        return GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
    }

    private static void GetTrajectoryDirection(PoseBuffer pose, in SkeletonData skeleton, bool simulationBone,
        int jointIndex, float3 characterForward, MotionMatchingData mmData,
        out float3 futureLocalDirection)
    {
        quaternion worldRotation;
        float3 localForward;
        if (simulationBone)
        {
            worldRotation = pose.Rotations[0];
            localForward = math.forward();
        }
        else
        {
            worldRotation = PoseBufferFK.WorldRotation(skeleton, pose, jointIndex);
            localForward = mmData.GetLocalForward(jointIndex);
        }

        var worldDirection = math.mul(worldRotation, localForward);
        futureLocalDirection = GetLocalDirectionFromCharacter(worldDirection, characterForward);
    }

    private static float3 GetJointPosition(PoseBuffer pose, in SkeletonData skeleton, int jointIndex,
        float3 characterOrigin, float3 characterForward)
    {
        var worldPosition = PoseBufferFK.WorldPosition(skeleton, pose, jointIndex);
        var localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
        return localPosition;
    }

    private static float3 GetJointVelocity(PoseBuffer pose, PoseBuffer poseNext, in SkeletonData skeleton,
        int jointIndex, float3 characterOrigin, float3 characterForward, float frameTime)
    {
        var worldPosition = PoseBufferFK.WorldPosition(skeleton, pose, jointIndex);
        var worldPositionNext = PoseBufferFK.WorldPosition(skeleton, poseNext, jointIndex);
        var localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
        var localVelocity =
            (GetLocalPositionFromCharacter(worldPositionNext, characterOrigin, characterForward) - localPosition) /
            frameTime;
        return localVelocity;
    }

    /// <summary>
    /// Returns the position and forward vector of the character in world space using the pose vector simulation bone
    /// </summary>
    public static void GetWorldOriginCharacter(PoseBuffer pose, out float3 center, out float3 forward)
    {
        center = pose.Positions[0]; // Simulation Bone World Position
        forward = math.mul(pose.Rotations[0], math.forward()); // Simulation Bone World Rotation
    }

    public static float3 GetLocalPositionFromCharacter(float3 worldPos, float3 characterOrigin, float3 characterForward)
    {
        return math.mul(math.inverse(quaternion.LookRotation(characterForward, math.up())), worldPos - characterOrigin);
    }

    public static float3 GetLocalDirectionFromCharacter(float3 worldDir, float3 characterForward)
    {
        var localDir = math.mul(math.inverse(quaternion.LookRotation(characterForward, math.up())), worldDir);
        return localDir;
    }

    public static float3 GetWorldPositionFromCharacter(float3 localPos, float3 characterOrigin, float3 characterForward)
    {
        return characterOrigin + math.mul(quaternion.LookRotation(characterForward, math.up()), localPos);
    }

    public static float3 GetWorldDirectionFromCharacter(float3 localDir, float3 characterForward)
    {
        return math.mul(quaternion.LookRotation(characterForward, math.up()), localDir);
    }

    public void Dispose()
    {
        if (_valid.IsCreated) _valid.Dispose();
        _features?.Dispose();
        if (_largeBoundingBoxMin.IsCreated) _largeBoundingBoxMin.Dispose();
        if (_largeBoundingBoxMax.IsCreated) _largeBoundingBoxMax.Dispose();
        if (_smallBoundingBoxMin.IsCreated) _smallBoundingBoxMin.Dispose();
        if (_smallBoundingBoxMax.IsCreated) _smallBoundingBoxMax.Dispose();
    }

    public float3 Get3DValuePositionOrDirectionFeature(TrajectoryFeature trajectoryFeature, int currentFrame,
        int trajectoryFeatureIndex, int predictionIndex)
    {
        var t = trajectoryFeatureIndex;
        var p = predictionIndex;

        float3 value;
        if (!trajectoryFeature.zeroX && !trajectoryFeature.zeroY && !trajectoryFeature.zeroZ)
        {
            value = Get3DTrajectoryFeature(currentFrame, t, p, true);
        }
        else if (!trajectoryFeature.zeroX && !trajectoryFeature.zeroY)
        {
            var value2D = Get2DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(value2D.x, value2D.y, 0);
        }
        else if (!trajectoryFeature.zeroX && !trajectoryFeature.zeroZ)
        {
            var value2D = Get2DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(value2D.x, 0.0f, value2D.y);
        }
        else if (!trajectoryFeature.zeroY && !trajectoryFeature.zeroZ)
        {
            var value2D = Get2DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(0.0f, value2D.x, value2D.y);
        }
        else if (!trajectoryFeature.zeroX)
        {
            var value1D = Get1DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(value1D, 0.0f, 0.0f);
        }
        else if (!trajectoryFeature.zeroY)
        {
            var value1D = Get1DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(0.0f, value1D, 0.0f);
        }
        else if (!trajectoryFeature.zeroZ)
        {
            var value1D = Get1DTrajectoryFeature(currentFrame, t, p, true);
            value = new float3(0.0f, 0.0f, value1D);
        }
        else
        {
            value = float3.zero;
            Debug.Assert(false, "Invalid trajectory feature");
        }

        return value;
    }
}
}

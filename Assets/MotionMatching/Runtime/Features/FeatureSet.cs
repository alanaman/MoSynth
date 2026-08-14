using System;
using AnimationTools;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using UnityEngine.Assertions;

namespace MotionMatching
{
    using TrajectoryFeature = MotionMatchingData.TrajectoryFeature;

    /// <summary>
    /// Stores all features vectors of all poses for Motion Matching
    /// </summary>
    public class FeatureSet
    {
        public int NumberFeatureVectors { get; private set; } // Total number of feature vectors
        public int FeatureSize { get; private set; } // Total size in floats of a feature vector
        public int FeatureStaticSize { get; private set; } // Total size in floats of a feature vector without environment features

        // Trajectory features
        public int NumberTrajectoryFeatures { get; private set; } // Number of different trajectory features (e.g. 2 = position and direction)
        private readonly int[] _numberPredictionsTrajectory; // Size: NumberTrajectoryFeatures. Number of predictions per trajectory feature (e.g. {3, 4}, means 3 predictions for position and 4 for direction)
        private readonly int[] _numberFloatsTrajectory; // Size: NumberTrajectoryFeatures. Number of floats per trajectory feature (e.g. {2, 3}, means 2 floats for position (float2) and 3 for direction (float3))
        private readonly int[] _trajectoryOffset; // Size: NumberTrajectoryFeatures. Offset of the trajectory feature in the feature vector (e.g. {0, 6}, means the position feature is at the beginning of the feature vector, and the direction feature starts at the sixth float)

        // Pose Features
        public int PoseFeatureCount { get; private set; } // Number of different pose features (e.g. 3 = leftFootPosition, leftFootVelocity, hipsVelocity)
        public const int FloatsPerPoseFeature = 3; // Number of floats per pose feature (e.g. 3 = float3)
        public int PoseFloatCount => PoseFeatureCount * FloatsPerPoseFeature;
        
        /// <summary>
        /// Offset of the pose feature in the feature vector.
        /// (e.g., 3 = leftFootPosition is at the third float)
        /// Pose Features are always after the Trajectory Features
        /// </summary>
        public int PoseOffset { get; private set; }

        // Environment
        public int NumberEnvironmentFeatures { get; private set; } // Number of environment features (e.g. 2 = spheres and ellipses)
        private readonly int[] _numberFloatsEnvironment; // Size: NumberEnvironmentFeatures. Number of floats per environment feature (e.g. {1, 2}, means 1 floats for spheres (float) and 2 for ellipses (float2))
        
        /// <summary>
        /// Offsets of the environment features in the feature vector
        /// (e.g., 6 = spheres is at the sixth float)
        /// Size: NumberEnvironmentFeatures.
        /// Environment Features are always after the Pose Features
        /// </summary>
        public int[] EnvironmentOffset { get; private set; }

        private NativeArray<bool> _valid; // TODO: Refactor to avoid needing this
        private NativeArray<float> _features; // Each feature: Trajectory + Pose + Environment 
        private float[] _mean; // Size: EnvironmentOffset[0]. Environment features are never normalized
        private float[] _standardDeviation; // Size: EnvironmentOffset[0]. Environment features are never normalized

        // BVH acceleration structures
        private NativeArray<float> _largeBoundingBoxMin;
        private NativeArray<float> _largeBoundingBoxMax;
        private NativeArray<float> _smallBoundingBoxMin;
        private NativeArray<float> _smallBoundingBoxMax;

        // Environment acceleration structures
        private NativeArray<int> _adaptativeFeaturesIndices; // Index to the real Features array

        public FeatureSet(MotionMatchingData mmData)
        {
            NumberFeatureVectors = mmData.GetOrImportPoseSet().NumberPoses;

            // Trajectory Features
            NumberTrajectoryFeatures = mmData.trajectoryFeatures.Count;
            _numberPredictionsTrajectory = new int[NumberTrajectoryFeatures];
            _numberFloatsTrajectory = new int[NumberTrajectoryFeatures];
            _trajectoryOffset = new int[NumberTrajectoryFeatures];
            var offset = 0;
            for (var i = 0; i < NumberTrajectoryFeatures; i++)
            {
                _trajectoryOffset[i] = offset;
                _numberPredictionsTrajectory[i] = mmData.trajectoryFeatures[i].framesPrediction.Length;
                _numberFloatsTrajectory[i] = mmData.trajectoryFeatures[i].GetSize();
                offset += _numberPredictionsTrajectory[i] * _numberFloatsTrajectory[i];
            }

            // Pose Features
            PoseOffset = offset;
            PoseFeatureCount = mmData.poseFeatures.Count;
            offset += PoseFloatCount; // + Pose

            FeatureStaticSize = offset;

            // Environment Features
            NumberEnvironmentFeatures = mmData.environmentFeatures.Count;
            var numberPredictionsEnvironment = new int[NumberEnvironmentFeatures];
            _numberFloatsEnvironment = new int[NumberEnvironmentFeatures];
            EnvironmentOffset = new int[NumberEnvironmentFeatures];
            for (var i = 0; i < NumberEnvironmentFeatures; i++)
            {
                EnvironmentOffset[i] = offset;
                numberPredictionsEnvironment[i] = mmData.environmentFeatures[i].framesPrediction.Length;
                _numberFloatsEnvironment[i] = mmData.environmentFeatures[i].GetSize();
                offset += numberPredictionsEnvironment[i] * _numberFloatsEnvironment[i];
            }

            FeatureSize = offset;
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
                feature[i] = _features[featureIndex * FeatureSize + i];
            }
        }
        
        public void GetPoseFeatures(Span<float> poseFeatures, int frameIndex, bool denormalize = false)
        {
            Debug.Assert(poseFeatures.Length == PoseFloatCount, "Feature vector has wrong size");
            
            for (var i = 0; i < PoseFloatCount; i++)
            {
                poseFeatures[i] = _features[frameIndex * FeatureSize + PoseOffset + i];
                if (denormalize)
                {
                    poseFeatures[i] = poseFeatures[i] * _standardDeviation[PoseOffset + i] + _mean[PoseOffset + i];
                }
            }
        }

        public ReadOnlySpan<float> GetFeatureVector(int featureIndex)
        {
            return _features.AsReadOnlySpan().Slice(featureIndex * FeatureSize, FeatureSize);
        }

        public float Get1DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            var featureOffset = _trajectoryOffset[trajectoryFeatureIndex] + predictionIndex * _numberFloatsTrajectory[trajectoryFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = _features[startIndex];
            if (denormalize)
            {
                x = x * _standardDeviation[featureOffset] + _mean[featureOffset];
            }
            return x;
        }
        public float2 Get2DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            var featureOffset = _trajectoryOffset[trajectoryFeatureIndex] + predictionIndex * _numberFloatsTrajectory[trajectoryFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = _features[startIndex];
            var y = _features[startIndex + 1];
            if (denormalize)
            {
                x = x * _standardDeviation[featureOffset] + _mean[featureOffset];
                y = y * _standardDeviation[featureOffset + 1] + _mean[featureOffset + 1];
            }
            return new float2(x, y);
        }
        public float3 Get3DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            var featureOffset = _trajectoryOffset[trajectoryFeatureIndex] + predictionIndex * _numberFloatsTrajectory[trajectoryFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = _features[startIndex];
            var y = _features[startIndex + 1];
            var z = _features[startIndex + 2];
            if (denormalize)
            {
                x = x * _standardDeviation[featureOffset] + _mean[featureOffset];
                y = y * _standardDeviation[featureOffset + 1] + _mean[featureOffset + 1];
                z = z * _standardDeviation[featureOffset + 2] + _mean[featureOffset + 2];
            }
            return new float3(x, y, z);
        }
        public float4 Get4DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            var featureOffset = _trajectoryOffset[trajectoryFeatureIndex] + predictionIndex * _numberFloatsTrajectory[trajectoryFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = _features[startIndex];
            var y = _features[startIndex + 1];
            var z = _features[startIndex + 2];
            var w = _features[startIndex + 3];
            if (denormalize)
            {
                x = x * _standardDeviation[featureOffset] + _mean[featureOffset];
                y = y * _standardDeviation[featureOffset + 1] + _mean[featureOffset + 1];
                z = z * _standardDeviation[featureOffset + 2] + _mean[featureOffset + 2];
                w = w * _standardDeviation[featureOffset + 3] + _mean[featureOffset + 3];
            }
            return new float4(x, y, z, w);
        }
        public float3 GetPoseFeature(int featureIndex, int poseFeatureIndex, bool denormalize = false)
        {
            var featureOffset = PoseOffset + poseFeatureIndex * FloatsPerPoseFeature;
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = _features[startIndex];
            var y = _features[startIndex + 1];
            var z = _features[startIndex + 2];
            if (denormalize)
            {
                x = x * _standardDeviation[featureOffset] + _mean[featureOffset];
                y = y * _standardDeviation[featureOffset + 1] + _mean[featureOffset + 1];
                z = z * _standardDeviation[featureOffset + 2] + _mean[featureOffset + 2];
            }
            return new float3(x, y, z);
        }
        public float Get1DEnvironmentFeature(int featureIndex, int environmentFeatureIndex, int predictionIndex)
        {
            var featureOffset = EnvironmentOffset[environmentFeatureIndex] + predictionIndex * _numberFloatsEnvironment[environmentFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = _features[startIndex];
            return x;
        }
        public float2 Get2DEnvironmentFeature(int featureIndex, int environmentFeatureIndex, int predictionIndex)
        {
            var featureOffset = EnvironmentOffset[environmentFeatureIndex] + predictionIndex * _numberFloatsEnvironment[environmentFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = _features[startIndex];
            var y = _features[startIndex + 1];
            return new float2(x, y);
        }
        public float3 Get3DEnvironmentFeature(int featureIndex, int environmentFeatureIndex, int predictionIndex)
        {
            var featureOffset = EnvironmentOffset[environmentFeatureIndex] + predictionIndex * _numberFloatsEnvironment[environmentFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = _features[startIndex];
            var y = _features[startIndex + 1];
            var z = _features[startIndex + 2];
            return new float3(x, y, z);
        }
        public float4 Get4DEnvironmentFeature(int featureIndex, int environmentFeatureIndex, int predictionIndex)
        {
            var featureOffset = EnvironmentOffset[environmentFeatureIndex] + predictionIndex * _numberFloatsEnvironment[environmentFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = _features[startIndex];
            var y = _features[startIndex + 1];
            var z = _features[startIndex + 2];
            var w = _features[startIndex + 3];
            return new float4(x, y, z, w);
        }

        public NativeArray<bool> GetValid()
        {
            return _valid;
        }
        public NativeArray<float> GetFeatures()
        {
            return _features;
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
                _largeBoundingBoxMin = new NativeArray<float>(numberBoundingBoxLarge * FeatureStaticSize, Allocator.Domain);
                _largeBoundingBoxMax = new NativeArray<float>(numberBoundingBoxLarge * FeatureStaticSize, Allocator.Domain);
                _smallBoundingBoxMin = new NativeArray<float>(numberBoundingBoxSmall * FeatureStaticSize, Allocator.Domain);
                _smallBoundingBoxMax = new NativeArray<float>(numberBoundingBoxSmall * FeatureStaticSize, Allocator.Domain);
                var job = new BVHMotionMatchingComputeBounds
                {
                    Features = GetFeatures(),
                    FeatureSize = FeatureSize,
                    FeatureStaticSize = FeatureStaticSize,
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

        public void GetEnvironmentAccelerationStructures(EnvironmentAccelerationConsts consts,
                                                     out NativeArray<int> adaptativeFeaturesIndices)
        {
            if (!_adaptativeFeaturesIndices.IsCreated)
            {
                // Build Environment Acceleration Structure
                NativeList<int> adaptativeIndices = new(Allocator.Domain);
                var job = new EnvironmentAccelerationComputeAdaptativeIndices
                {
                    Features = GetFeatures(),
                    Valid = GetValid(),
                    FeatureSize = FeatureSize,
                    PoseOffset = PoseOffset,
                    EnvironmentAccelerationConsts = consts,
                    FeatureStaticSize = FeatureStaticSize,
                    AdaptativeIndices = adaptativeIndices,
                };
                job.Schedule().Complete();
                _adaptativeFeaturesIndices = adaptativeIndices.AsArray();
            }
            // Debug.Log("Number of features: " + GetFeatures().Length / FeatureSize + " -----  Adaptative Length: " + AdaptativeFeaturesIndices.Length);
            adaptativeFeaturesIndices = _adaptativeFeaturesIndices;
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
            if (_features.IsCreated)
            {
                _features.Dispose();
            }
            _features = features;
        }
        public void SetMean(float[] mean)
        {
            Debug.Assert(mean.Length == FeatureStaticSize, mean.Length + " != " + FeatureStaticSize);
            _mean = mean;
        }
        public void SetStandardDeviation(float[] standardDeviation)
        {
            Debug.Assert(standardDeviation.Length == FeatureStaticSize, standardDeviation.Length + " != " + FeatureStaticSize);
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

            for (var i = 0; i < FeatureStaticSize; i++)
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

            for (var i = 0; i < FeatureStaticSize; i++)
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
            for (var i = 0; i < NumberFeatureVectors; i++)
            {
                var featureIndex = i * FeatureSize;
                if (_valid[i])
                {
                    for (var j = 0; j < FeatureStaticSize; j++)
                    {
                        _features[featureIndex + j] = (_features[featureIndex + j] - _mean[j]) / _standardDeviation[j];
                    }
                }
            }
        }

        private void ComputeMeanAndStandardDeviation()
        {
            var nTotalDimensions = FeatureStaticSize;
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
                        _mean[j] += _features[featureIndex + j];
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
                        var diff = _features[featureIndex + j] - _mean[j];
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
                var offset = _trajectoryOffset[d];
                var nDimensions = _numberPredictionsTrajectory[d] * _numberFloatsTrajectory[d];
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
                var offset = PoseOffset + d * FloatsPerPoseFeature;
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
            // Init
            var nPoses = poseSet.NumberPoses;
            _valid = new NativeArray<bool>(nPoses, Allocator.Domain);
            _features = new NativeArray<float>(nPoses * FeatureSize, Allocator.Domain);
            // Check skeleton has all needed joints
            var jointsTrajectory = new int[NumberTrajectoryFeatures];
            var i = 0;
            foreach (var trajectoryFeature in mmData.trajectoryFeatures)
            {
                if ((trajectoryFeature.featureType == TrajectoryFeature.Type.Position ||
                    trajectoryFeature.featureType == TrajectoryFeature.Type.Direction)
                    && !trajectoryFeature.simulationBone)
                {
                    jointsTrajectory[i] = poseSet.SkeletonAsset.FindJointIndexOrZero(trajectoryFeature.bone);
                }
                i += 1;
            }
            var jointsPose = new int[PoseFeatureCount];
            i = 0;
            foreach (var poseFeature in mmData.poseFeatures)
            {
                jointsPose[i] = poseSet.SkeletonAsset.FindJointIndexOrZero(poseFeature.bone);
                i += 1;
            }
            var jointsEnvironment = new int[NumberEnvironmentFeatures];
            i = 0;
            foreach (var environmentFeature in mmData.environmentFeatures)
            {
                if ((environmentFeature.featureType == TrajectoryFeature.Type.Position ||
                    environmentFeature.featureType == TrajectoryFeature.Type.Direction)
                    && !environmentFeature.simulationBone)
                {
                    jointsEnvironment[i] = poseSet.SkeletonAsset.FindJointIndexOrZero(environmentFeature.bone);
                }
                i += 1;
            }
            // Extract Features
            for (var poseIndex = 0; poseIndex < nPoses; ++poseIndex)
            {
                if (poseSet.IsPoseValidForPrediction(poseIndex, mmData.MaximumFramesPrediction))
                {
                    _valid[poseIndex] = true;
                    ExtractFeature(poseSet, poseIndex, jointsTrajectory, jointsPose, jointsEnvironment, mmData);
                }
                else _valid[poseIndex] = false;
            }
        }

        /// <summary>
        /// Extract the feature vectors from poseSet
        /// </summary>
        private void ExtractFeature(PoseSet poseSet, int poseIndex, int[] jointsTrajectory, int[] jointsPose, int[] jointsEnvironment, MotionMatchingData mmData)
        {
            var featureIndex = poseIndex * FeatureSize;
            var nextPose = poseIndex + 1;
            if (nextPose >= poseSet.NumberPoses - mmData.MaximumFramesPrediction)
            {
                nextPose = poseIndex;
            }
            var nextFeatureIndex = nextPose * FeatureSize;
            var pose = poseSet.GetPoseBuffer(poseIndex);
            var poseNext = poseSet.GetPoseBuffer(nextPose);
            var skeletonData = poseSet.SkeletonAsset.GetSkeletonData();
            // Compute local features based on the Simulation Bone
            // so hips and feet are local to a stable position with respect to the character
            GetWorldOriginCharacter(pose, out var characterOrigin, out var characterForward);

            // Trajectory Features -------------------------------------------------------------
            for (var i = 0; i < NumberTrajectoryFeatures; i++)
            {
                var trajectoryFeature = mmData.trajectoryFeatures[i];
                var featureOffset = featureIndex + _trajectoryOffset[i];
                var nextFeatureOffset = nextFeatureIndex + _trajectoryOffset[i];
                var isStartFeature = true;
                for (var p = 0; p < trajectoryFeature.framesPrediction.Length; ++p)
                {
                    var predictionOffset = featureOffset + p * _numberFloatsTrajectory[i];
                    var nextPredictionOffset = nextFeatureOffset + p * _numberFloatsTrajectory[i];
                    var futurePoseIndex = poseIndex + trajectoryFeature.framesPrediction[p];
                    var nextFuturePoseIndex = nextPose + trajectoryFeature.framesPrediction[p];

                    isStartFeature = ExtractTrajectoryFeature(trajectoryFeature, poseSet, futurePoseIndex, nextFuturePoseIndex,
                                                              jointsTrajectory, i, predictionOffset, characterOrigin, characterForward,
                                                              mmData, isStartFeature, skeletonData);
                }
            }

            // Pose Features -------------------------------------------------------------
            for (var i = 0; i < PoseFeatureCount; i++)
            {
                var poseFeature = mmData.poseFeatures[i];
                var featureOffset = featureIndex + PoseOffset + i * FloatsPerPoseFeature;
                float3 feature = new();
                switch (poseFeature.featureType)
                {
                    case MotionMatchingData.PoseFeature.Type.Position:
                        feature = GetJointPosition(pose, skeletonData, jointsPose[i], characterOrigin, characterForward);
                        break;
                    case MotionMatchingData.PoseFeature.Type.Velocity:
                        feature = GetJointVelocity(pose, poseNext, skeletonData, jointsPose[i], characterOrigin, characterForward, poseSet.FrameTime);
                        break;
                    default:
                        Debug.Assert(false, "Unknown PoseFeature.Type: " + poseFeature.featureType);
                        break;
                }
                _features[featureOffset + 0] = feature.x;
                _features[featureOffset + 1] = feature.y;
                _features[featureOffset + 2] = feature.z;
            }

            // Environment Features -------------------------------------------------------------
            for (var i = 0; i < NumberEnvironmentFeatures; i++)
            {
                var environmentFeature = mmData.environmentFeatures[i];
                var featureOffset = featureIndex + EnvironmentOffset[i];
                var nextFeatureOffset = nextFeatureIndex + EnvironmentOffset[i];
                var isStartFeature = true;
                for (var p = 0; p < environmentFeature.framesPrediction.Length; ++p)
                {
                    var predictionOffset = featureOffset + p * _numberFloatsEnvironment[i];
                    var nextPredictionOffset = nextFeatureOffset + p * _numberFloatsEnvironment[i];
                    var futurePoseIndex = poseIndex + environmentFeature.framesPrediction[p];
                    var nextFuturePoseIndex = nextPose + environmentFeature.framesPrediction[p];

                    isStartFeature = ExtractTrajectoryFeature(environmentFeature, poseSet, futurePoseIndex, nextFuturePoseIndex,
                                                              jointsEnvironment, i, predictionOffset, characterOrigin, characterForward,
                                                              mmData, isStartFeature, skeletonData);
                }
            }
        }

        private bool ExtractTrajectoryFeature(TrajectoryFeature feature, PoseSet poseSet, int futurePoseIndex, int nextFuturePoseIndex,
                                              int[] joints, int featureIt, int predictionOffset, float3 characterOrigin, float3 characterForward,
                                              MotionMatchingData mmData, bool isStartFeature, in SkeletonData skeletonData)
        {
            var futurePose = poseSet.GetPoseBuffer(futurePoseIndex);
            var animationClip = poseSet.GetAnimationClipIndex(futurePoseIndex);
            var nextFuturePose = poseSet.GetPoseBuffer(nextFuturePoseIndex);

            float3 value = new();
            switch (feature.featureType)
            {
                case TrajectoryFeature.Type.Position:
                    {
                        GetTrajectoryPosition(futurePose, skeletonData, feature.simulationBone, joints[featureIt], characterOrigin, characterForward,
                                              out value);
                    }
                    break;
                case TrajectoryFeature.Type.Direction:
                    {
                        GetTrajectoryDirection(futurePose, skeletonData, feature.simulationBone, joints[featureIt], characterForward, mmData,
                                               out value);
                        if (feature.zeroX) value.x = 0;
                        if (feature.zeroY) value.y = 0;
                        if (feature.zeroZ) value.z = 0;
                        value = math.normalize(value);
                    }
                    break;
                case TrajectoryFeature.Type.Custom1D:
                    {
                        var extractor1D = feature.featureExtractor as Feature1DExtractor;
                        Assert.IsNotNull(extractor1D);
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor1D.StartExtracting(poseSet.SkeletonAsset);
                        }
                        var value1D = extractor1D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.SkeletonAsset, characterOrigin, characterForward);
                        _features[predictionOffset + 0] = value1D;
                    }
                    break;
                case TrajectoryFeature.Type.Custom2D:
                    {
                        var extractor2D = feature.featureExtractor as Feature2DExtractor;
                        Assert.IsNotNull(extractor2D);
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor2D.StartExtracting(poseSet.SkeletonAsset);
                        }
                        var value2D = extractor2D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.SkeletonAsset, characterOrigin, characterForward);
                        _features[predictionOffset + 0] = value2D.x;
                        _features[predictionOffset + 1] = value2D.y;
                    }
                    break;
                case TrajectoryFeature.Type.Custom3D:
                    {
                        var extractor3D = feature.featureExtractor as Feature3DExtractor;
                        Assert.IsNotNull(extractor3D);
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor3D.StartExtracting(poseSet.SkeletonAsset);
                        }
                        var value3D = extractor3D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.SkeletonAsset, characterOrigin, characterForward);
                        _features[predictionOffset + 0] = value3D.x;
                        _features[predictionOffset + 1] = value3D.y;
                        _features[predictionOffset + 2] = value3D.z;
                    }
                    break;
                case TrajectoryFeature.Type.Custom4D:
                    {
                        var extractor4D = feature.featureExtractor as Feature4DExtractor;
                        Assert.IsNotNull(extractor4D);
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor4D.StartExtracting(poseSet.SkeletonAsset);
                        }
                        var value4D = extractor4D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.SkeletonAsset, characterOrigin, characterForward);
                        _features[predictionOffset + 0] = value4D.x;
                        _features[predictionOffset + 1] = value4D.y;
                        _features[predictionOffset + 2] = value4D.z;
                        _features[predictionOffset + 3] = value4D.w;
                    }
                    break;
                default:
                    Debug.Assert(false, "Unsupported Feature Type: " + feature.featureType);
                    break;
            }
            if (feature.featureType == TrajectoryFeature.Type.Position ||
                feature.featureType == TrajectoryFeature.Type.Direction)
            {
                var offsetIndex = 0;
                var valueIndex = 0;
                var size = feature.GetSize();
                for (var f = 0; f < size; ++f)
                {
                    if (valueIndex == 0 && feature.zeroX) valueIndex += 1;
                    if (valueIndex == 1 && feature.zeroY) valueIndex += 1;
                    _features[predictionOffset + offsetIndex] = value[valueIndex];
                    valueIndex += 1;
                    offsetIndex += 1;
                }
            }
            return isStartFeature;
        }

        private static void GetTrajectoryPosition(PoseBuffer pose, in SkeletonData skeleton, bool simulationBone, int jointIndex, float3 characterOrigin, float3 characterForward,
                                                  out float3 futureLocalPosition)
        {
            float3 worldPosition;
            if (simulationBone)
            {
                worldPosition = pose.Positions[0];
            }
            else
            {
                worldPosition = PoseBufferFK.WorldPosition(skeleton, pose, jointIndex);
            }
            futureLocalPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
        }
        private static void GetTrajectoryDirection(PoseBuffer pose, in SkeletonData skeleton, bool simulationBone, int jointIndex, float3 characterForward, MotionMatchingData mmData,
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

        private static float3 GetJointPosition(PoseBuffer pose, in SkeletonData skeleton, int jointIndex, float3 characterOrigin, float3 characterForward)
        {
            var worldPosition = PoseBufferFK.WorldPosition(skeleton, pose, jointIndex);
            var localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
            return localPosition;
        }
        private static float3 GetJointVelocity(PoseBuffer pose, PoseBuffer poseNext, in SkeletonData skeleton, int jointIndex, float3 characterOrigin, float3 characterForward, float frameTime)
        {
            var worldPosition = PoseBufferFK.WorldPosition(skeleton, pose, jointIndex);
            var worldPositionNext = PoseBufferFK.WorldPosition(skeleton, poseNext, jointIndex);
            var localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
            var localVelocity = (GetLocalPositionFromCharacter(worldPositionNext, characterOrigin, characterForward) - localPosition) / frameTime;
            return localVelocity;
        }

        /// <summary>
        /// Returns the position and forward vector of the character in world space using the pose vector simulation bone
        /// </summary>
        /// <param name="poseVector"></param>
        /// <param name="center"></param>
        /// <param name="forward"></param>
        public static void GetWorldOriginCharacter(PoseBuffer poseVector, out float3 center, out float3 forward)
        {
            center = poseVector.Positions[0]; // Simulation Bone World Position
            forward = math.mul(poseVector.Rotations[0], math.forward()); // Simulation Bone World Rotation
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
            if (_features.IsCreated) _features.Dispose();
            if (_largeBoundingBoxMin.IsCreated) _largeBoundingBoxMin.Dispose();
            if (_largeBoundingBoxMax.IsCreated) _largeBoundingBoxMax.Dispose();
            if (_smallBoundingBoxMin.IsCreated) _smallBoundingBoxMin.Dispose();
            if (_smallBoundingBoxMax.IsCreated) _smallBoundingBoxMax.Dispose();
            if (_adaptativeFeaturesIndices.IsCreated) _adaptativeFeaturesIndices.Dispose();
        }
        
        public float3 Get3DValuePositionOrDirectionFeature(TrajectoryFeature trajectoryFeature, int currentFrame, int trajectoryFeatureIndex, int predictionIndex, bool isEnvironment)
        {
            var t = trajectoryFeatureIndex;
            var p = predictionIndex;

            float3 value;
            if (!trajectoryFeature.zeroX && !trajectoryFeature.zeroY && !trajectoryFeature.zeroZ)
            {
                value = isEnvironment ? Get3DEnvironmentFeature(currentFrame, t, p) : Get3DTrajectoryFeature(currentFrame, t, p, true);
            }
            else if (!trajectoryFeature.zeroX && !trajectoryFeature.zeroY)
            {
                var value2D = isEnvironment ? Get2DEnvironmentFeature(currentFrame, t, p) : Get2DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(value2D.x, value2D.y, 0);
            }
            else if (!trajectoryFeature.zeroX && !trajectoryFeature.zeroZ)
            {
                var value2D = isEnvironment ? Get2DEnvironmentFeature(currentFrame, t, p) : Get2DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(value2D.x, 0.0f, value2D.y);
            }
            else if (!trajectoryFeature.zeroY && !trajectoryFeature.zeroZ)
            {
                var value2D = isEnvironment ? Get2DEnvironmentFeature(currentFrame, t, p) : Get2DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(0.0f, value2D.x, value2D.y);
            }
            else if (!trajectoryFeature.zeroX)
            {
                var value1D = isEnvironment ? Get1DEnvironmentFeature(currentFrame, t, p) : Get1DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(value1D, 0.0f, 0.0f);
            }
            else if (!trajectoryFeature.zeroY)
            {
                var value1D = isEnvironment ? Get1DEnvironmentFeature(currentFrame, t, p) : Get1DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(0.0f, value1D, 0.0f);
            }
            else if (!trajectoryFeature.zeroZ)
            {
                var value1D = isEnvironment ? Get1DEnvironmentFeature(currentFrame, t, p) : Get1DTrajectoryFeature(currentFrame, t, p, true);
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
using System;
using Unity.Collections;
using UnityEngine;
using Unity.Mathematics;
using Unity.Jobs;
using System.Collections.Generic;

namespace MotionMatching
{
    using Joint = Skeleton.Joint;
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
        private readonly int[] NumberPredictionsTrajectory; // Size: NumberTrajectoryFeatures. Number of predictions per trajectory feature (e.g. {3, 4}, means 3 predictions for position and 4 for direction)
        private readonly int[] NumberFloatsTrajectory; // Size: NumberTrajectoryFeatures. Number of floats per trajectory feature (e.g. {2, 3}, means 2 floats for position (float2) and 3 for direction (float3))
        private readonly int[] TrajectoryOffset; // Size: NumberTrajectoryFeatures. Offset of the trajectory feature in the feature vector (e.g. {0, 6}, means the position feature is at the beginning of the feature vector, and the direction feature starts at the sixth float)

        // Pose Features
        public int NumberPoseFeatures { get; private set; } // Number of different pose features (e.g. 3 = leftFootPosition, leftFootVelocity, hipsVelocity)
        public const int NumberFloatsPose = 3; // Number of floats per pose feature (e.g. 3 = float3)
        
        /// <summary>
        /// Offset of the pose feature in the feature vector.
        /// (e.g., 3 = leftFootPosition is at the third float)
        /// Pose Features are always after the Trajectory Features
        /// </summary>
        public int PoseOffset { get; private set; }

        // Environment
        public int NumberEnvironmentFeatures { get; private set; } // Number of environment features (e.g. 2 = spheres and ellipses)
        private readonly int[] NumberPredictionsEnvironment; // Size: NumberEnvironmentFeatures. Number of predictions per environment feature (e.g. {3, 4}, means 3 predictions for spheres and 4 for ellipses)
        private readonly int[] NumberFloatsEnvironment; // Size: NumberEnvironmentFeatures. Number of floats per environment feature (e.g. {1, 2}, means 1 floats for spheres (float) and 2 for ellipses (float2))
        
        /// <summary>
        /// Offsets of the environment features in the feature vector
        /// (e.g., 6 = spheres is at the sixth float)
        /// Size: NumberEnvironmentFeatures.
        /// Environment Features are always after the Pose Features
        /// </summary>
        public int[] EnvironmentOffset { get; private set; }

        private NativeArray<bool> Valid; // TODO: Refactor to avoid needing this
        private NativeArray<float> Features; // Each feature: Trajectory + Pose + Environment 
        private float[] Mean; // Size: EnvironmentOffset[0]. Environment features are never normalized
        private float[] StandardDeviation; // Size: EnvironmentOffset[0]. Environment features are never normalized

        // BVH acceleration structures
        private NativeArray<float> LargeBoundingBoxMin;
        private NativeArray<float> LargeBoundingBoxMax;
        private NativeArray<float> SmallBoundingBoxMin;
        private NativeArray<float> SmallBoundingBoxMax;

        // Environment acceleration structures
        private NativeArray<int> AdaptativeFeaturesIndices; // Index to the real Features array

        public FeatureSet(MotionMatchingData mmData, int numberFeatureVectors)
        {
            NumberFeatureVectors = numberFeatureVectors;

            // Trajectory Features
            NumberTrajectoryFeatures = mmData.TrajectoryFeatures.Count;
            NumberPredictionsTrajectory = new int[NumberTrajectoryFeatures];
            NumberFloatsTrajectory = new int[NumberTrajectoryFeatures];
            TrajectoryOffset = new int[NumberTrajectoryFeatures];
            var offset = 0;
            for (var i = 0; i < NumberTrajectoryFeatures; i++)
            {
                TrajectoryOffset[i] = offset;
                NumberPredictionsTrajectory[i] = mmData.TrajectoryFeatures[i].FramesPrediction.Length;
                NumberFloatsTrajectory[i] = mmData.TrajectoryFeatures[i].GetSize();
                offset += NumberPredictionsTrajectory[i] * NumberFloatsTrajectory[i];
            }

            // Pose Features
            PoseOffset = offset;
            NumberPoseFeatures = mmData.PoseFeatures.Count;
            offset += NumberPoseFeatures * NumberFloatsPose; // + Pose

            FeatureStaticSize = offset;

            // Environment Features
            NumberEnvironmentFeatures = mmData.EnvironmentFeatures.Count;
            NumberPredictionsEnvironment = new int[NumberEnvironmentFeatures];
            NumberFloatsEnvironment = new int[NumberEnvironmentFeatures];
            EnvironmentOffset = new int[NumberEnvironmentFeatures];
            for (var i = 0; i < NumberEnvironmentFeatures; i++)
            {
                EnvironmentOffset[i] = offset;
                NumberPredictionsEnvironment[i] = mmData.EnvironmentFeatures[i].FramesPrediction.Length;
                NumberFloatsEnvironment[i] = mmData.EnvironmentFeatures[i].GetSize();
                offset += NumberPredictionsEnvironment[i] * NumberFloatsEnvironment[i];
            }

            FeatureSize = offset;
        }

        public bool IsValidFeature(int featureIndex)
        {
            return Valid[featureIndex];
        }

        public void GetFeature(NativeArray<float> feature, int featureIndex)
        {
            Debug.Assert(feature.Length == FeatureSize, "Feature vector has wrong size");
            for (var i = 0; i < FeatureSize; i++)
            {
                feature[i] = Features[featureIndex * FeatureSize + i];
            }
        }

        public ReadOnlySpan<float> GetFeatureVector(int featureIndex)
        {
            return Features.AsReadOnlySpan().Slice(featureIndex * FeatureSize, FeatureSize);
        }

        public float Get1DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            var featureOffset = TrajectoryOffset[trajectoryFeatureIndex] + predictionIndex * NumberFloatsTrajectory[trajectoryFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = Features[startIndex];
            if (denormalize)
            {
                x = x * StandardDeviation[featureOffset] + Mean[featureOffset];
            }
            return x;
        }
        public float2 Get2DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            var featureOffset = TrajectoryOffset[trajectoryFeatureIndex] + predictionIndex * NumberFloatsTrajectory[trajectoryFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = Features[startIndex];
            var y = Features[startIndex + 1];
            if (denormalize)
            {
                x = x * StandardDeviation[featureOffset] + Mean[featureOffset];
                y = y * StandardDeviation[featureOffset + 1] + Mean[featureOffset + 1];
            }
            return new float2(x, y);
        }
        public float3 Get3DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            var featureOffset = TrajectoryOffset[trajectoryFeatureIndex] + predictionIndex * NumberFloatsTrajectory[trajectoryFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = Features[startIndex];
            var y = Features[startIndex + 1];
            var z = Features[startIndex + 2];
            if (denormalize)
            {
                x = x * StandardDeviation[featureOffset] + Mean[featureOffset];
                y = y * StandardDeviation[featureOffset + 1] + Mean[featureOffset + 1];
                z = z * StandardDeviation[featureOffset + 2] + Mean[featureOffset + 2];
            }
            return new float3(x, y, z);
        }
        public float4 Get4DTrajectoryFeature(int featureIndex, int trajectoryFeatureIndex, int predictionIndex, bool denormalize = false)
        {
            var featureOffset = TrajectoryOffset[trajectoryFeatureIndex] + predictionIndex * NumberFloatsTrajectory[trajectoryFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = Features[startIndex];
            var y = Features[startIndex + 1];
            var z = Features[startIndex + 2];
            var w = Features[startIndex + 3];
            if (denormalize)
            {
                x = x * StandardDeviation[featureOffset] + Mean[featureOffset];
                y = y * StandardDeviation[featureOffset + 1] + Mean[featureOffset + 1];
                z = z * StandardDeviation[featureOffset + 2] + Mean[featureOffset + 2];
                w = w * StandardDeviation[featureOffset + 3] + Mean[featureOffset + 3];
            }
            return new float4(x, y, z, w);
        }
        public float3 GetPoseFeature(int featureIndex, int poseFeatureIndex, bool denormalize = false)
        {
            var featureOffset = PoseOffset + poseFeatureIndex * NumberFloatsPose;
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = Features[startIndex];
            var y = Features[startIndex + 1];
            var z = Features[startIndex + 2];
            if (denormalize)
            {
                x = x * StandardDeviation[featureOffset] + Mean[featureOffset];
                y = y * StandardDeviation[featureOffset + 1] + Mean[featureOffset + 1];
                z = z * StandardDeviation[featureOffset + 2] + Mean[featureOffset + 2];
            }
            return new float3(x, y, z);
        }
        public float Get1DEnvironmentFeature(int featureIndex, int environmentFeatureIndex, int predictionIndex)
        {
            var featureOffset = EnvironmentOffset[environmentFeatureIndex] + predictionIndex * NumberFloatsEnvironment[environmentFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = Features[startIndex];
            return x;
        }
        public float2 Get2DEnvironmentFeature(int featureIndex, int environmentFeatureIndex, int predictionIndex)
        {
            var featureOffset = EnvironmentOffset[environmentFeatureIndex] + predictionIndex * NumberFloatsEnvironment[environmentFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = Features[startIndex];
            var y = Features[startIndex + 1];
            return new float2(x, y);
        }
        public float3 Get3DEnvironmentFeature(int featureIndex, int environmentFeatureIndex, int predictionIndex)
        {
            var featureOffset = EnvironmentOffset[environmentFeatureIndex] + predictionIndex * NumberFloatsEnvironment[environmentFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = Features[startIndex];
            var y = Features[startIndex + 1];
            var z = Features[startIndex + 2];
            return new float3(x, y, z);
        }
        public float4 Get4DEnvironmentFeature(int featureIndex, int environmentFeatureIndex, int predictionIndex)
        {
            var featureOffset = EnvironmentOffset[environmentFeatureIndex] + predictionIndex * NumberFloatsEnvironment[environmentFeatureIndex];
            var startIndex = featureIndex * FeatureSize + featureOffset;
            var x = Features[startIndex];
            var y = Features[startIndex + 1];
            var z = Features[startIndex + 2];
            var w = Features[startIndex + 3];
            return new float4(x, y, z, w);
        }

        public NativeArray<bool> GetValid()
        {
            return Valid;
        }
        public NativeArray<float> GetFeatures()
        {
            return Features;
        }

        public float GetMean(int dimension)
        {
            return Mean[dimension];
        }
        public float[] GetMeans()
        {
            return Mean;
        }
        public float GetStandardDeviation(int dimension)
        {
            return StandardDeviation[dimension];
        }
        public float[] GetStandardDeviations()
        {
            return StandardDeviation;
        }

        public void GetBVHBuffers(out NativeArray<float> largeBoundingBoxMin,
                                  out NativeArray<float> largeBoundingBoxMax,
                                  out NativeArray<float> smallBoundingBoxMin,
                                  out NativeArray<float> smallBoundingBoxMax)
        {
            if (LargeBoundingBoxMax == null || !LargeBoundingBoxMax.IsCreated)
            {
                // Build BVH Acceleration Structure
                var nFrames = GetFeatures().Length / FeatureSize;
                var numberBoundingBoxLarge = (nFrames + BVHConsts.LargeBVHSize - 1) / BVHConsts.LargeBVHSize;
                var numberBoundingBoxSmall = (nFrames + BVHConsts.SmallBVHSize - 1) / BVHConsts.SmallBVHSize;
                LargeBoundingBoxMin = new NativeArray<float>(numberBoundingBoxLarge * FeatureStaticSize, Allocator.Persistent);
                LargeBoundingBoxMax = new NativeArray<float>(numberBoundingBoxLarge * FeatureStaticSize, Allocator.Persistent);
                SmallBoundingBoxMin = new NativeArray<float>(numberBoundingBoxSmall * FeatureStaticSize, Allocator.Persistent);
                SmallBoundingBoxMax = new NativeArray<float>(numberBoundingBoxSmall * FeatureStaticSize, Allocator.Persistent);
                var job = new BVHMotionMatchingComputeBounds
                {
                    Features = GetFeatures(),
                    FeatureSize = FeatureSize,
                    FeatureStaticSize = FeatureStaticSize,
                    NumberBoundingBoxLarge = numberBoundingBoxLarge,
                    NumberBoundingBoxSmall = numberBoundingBoxSmall,
                    LargeBoundingBoxMin = LargeBoundingBoxMin,
                    LargeBoundingBoxMax = LargeBoundingBoxMax,
                    SmallBoundingBoxMin = SmallBoundingBoxMin,
                    SmallBoundingBoxMax = SmallBoundingBoxMax,
                };
                job.Schedule().Complete();
            }
            largeBoundingBoxMin = LargeBoundingBoxMin;
            largeBoundingBoxMax = LargeBoundingBoxMax;
            smallBoundingBoxMin = SmallBoundingBoxMin;
            smallBoundingBoxMax = SmallBoundingBoxMax;
        }

        public void GetEnvironmentAccelerationStructures(EnvironmentAccelerationConsts consts,
                                                     out NativeArray<int> adaptativeFeaturesIndices)
        {
            if (AdaptativeFeaturesIndices == null || !AdaptativeFeaturesIndices.IsCreated)
            {
                // Build Environment Acceleration Structure
                NativeList<int> adaptativeIndices = new(Allocator.Persistent);
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
                AdaptativeFeaturesIndices = adaptativeIndices.AsArray();
            }
            // Debug.Log("Number of features: " + GetFeatures().Length / FeatureSize + " -----  Adaptative Length: " + AdaptativeFeaturesIndices.Length);
            adaptativeFeaturesIndices = AdaptativeFeaturesIndices;
        }

        // Deserialize ---------------------------------------
        public void SetValid(NativeArray<bool> valid)
        {
            Debug.Assert(valid.Length == NumberFeatureVectors, "Valid array has wrong size");
            if (Valid != null && Valid.IsCreated)
            {
                Valid.Dispose();
            }
            Valid = valid;
        }
        public void SetFeatures(NativeArray<float> features)
        {
            Debug.Assert(features.Length == NumberFeatureVectors * FeatureSize, "Feature vector has wrong size");
            if (Features != null && Features.IsCreated)
            {
                Features.Dispose();
            }
            Features = features;
        }
        public void SetMean(float[] mean)
        {
            Debug.Assert(mean.Length == FeatureStaticSize, mean.Length + " != " + FeatureStaticSize);
            Mean = mean;
        }
        public void SetStandardDeviation(float[] standardDeviation)
        {
            Debug.Assert(standardDeviation.Length == FeatureStaticSize, standardDeviation.Length + " != " + FeatureStaticSize);
            StandardDeviation = standardDeviation;
        }
        // --------------------------------------------------

        /// <summary>
        /// Normalizes the trajectory features (pose features remaing untouched)
        /// </summary>
        public void NormalizeTrajectory(NativeArray<float> featureVector)
        {
            Debug.Assert(Mean != null, "Mean is not initialized");
            Debug.Assert(StandardDeviation != null, "StandardDeviation is not initialized");
            Debug.Assert(featureVector.Length == FeatureSize, "Feature vector size does not match");

            for (var i = 0; i < PoseOffset; i++)
            {
                featureVector[i] = (featureVector[i] - Mean[i]) / StandardDeviation[i];
            }
        }

        /// <summary>
        /// Normalizes all features (trajectory + pose)
        /// </summary>
        public void NormalizeFeatureVector(NativeArray<float> featureVector)
        {
            Debug.Assert(Mean != null, "Mean is not initialized");
            Debug.Assert(StandardDeviation != null, "StandardDeviation is not initialized");
            Debug.Assert(featureVector.Length == FeatureSize, "Feature vector size does not match");

            for (var i = 0; i < FeatureStaticSize; i++)
            {
                featureVector[i] = (featureVector[i] - Mean[i]) / StandardDeviation[i];
            }
        }

        /// <summary>
        /// Returns a copy of the feature vector with the features before normalization
        /// </summary>
        public void DenormalizeFeatureVector(NativeArray<float> featureVector)
        {
            Debug.Assert(Mean != null, "Mean is not initialized");
            Debug.Assert(StandardDeviation != null, "StandardDeviation is not initialized");
            Debug.Assert(featureVector.Length == FeatureSize, "Feature vector size does not match");

            for (var i = 0; i < FeatureStaticSize; i++)
            {
                featureVector[i] = featureVector[i] * StandardDeviation[i] + Mean[i];
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
                if (Valid[i])
                {
                    for (var j = 0; j < FeatureStaticSize; j++)
                    {
                        Features[featureIndex + j] = (Features[featureIndex + j] - Mean[j]) / StandardDeviation[j];
                    }
                }
            }
        }

        private void ComputeMeanAndStandardDeviation()
        {
            var nTotalDimensions = FeatureStaticSize;
            // Mean for each dimension
            Mean = new float[nTotalDimensions];
            // Variance for each dimension
            Span<float> variance = stackalloc float[nTotalDimensions];
            // Standard Deviation for each dimension
            StandardDeviation = new float[nTotalDimensions];

            // Compute Means for each dimension of each feature
            var count = 0;
            for (var i = 0; i < NumberFeatureVectors; i++)
            {
                if (Valid[i])
                {
                    var featureIndex = i * FeatureSize;
                    for (var j = 0; j < nTotalDimensions; j++)
                    {
                        Mean[j] += Features[featureIndex + j];
                    }
                    count += 1;
                }
            }
            for (var i = 0; i < nTotalDimensions; i++)
            {
                Mean[i] /= count;
            }
            // Compute Variance for each dimension of each feature - variance = (x - mean)^2 / n
            for (var i = 0; i < NumberFeatureVectors; i++)
            {
                var featureIndex = i * FeatureSize;
                if (Valid[i])
                {
                    for (var j = 0; j < nTotalDimensions; j++)
                    {
                        var diff = Features[featureIndex + j] - Mean[j];
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
                var offset = TrajectoryOffset[d];
                var nDimensions = NumberPredictionsTrajectory[d] * NumberFloatsTrajectory[d];
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
                    StandardDeviation[offset + j] = std;
                }
            }
            for (var d = 0; d < NumberPoseFeatures; d++)
            {
                var offset = PoseOffset + d * NumberFloatsPose;
                float std = 0;
                for (var j = 0; j < NumberFloatsPose; j++)
                {
                    std += math.sqrt(variance[offset + j]);
                }
                std /= NumberFloatsPose;
                Debug.Assert(std > 0, "Standard deviation is zero, feature with no variation is probably a bug");
                if (std <= 0)
                {
                    std = 1.0f;
                }
                for (var j = 0; j < NumberFloatsPose; j++)
                {
                    StandardDeviation[offset + j] = std;
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
            Valid = new NativeArray<bool>(nPoses, Allocator.Persistent);
            Features = new NativeArray<float>(nPoses * FeatureSize, Allocator.Persistent);
            // Check skeleton has all needed joints
            var jointsTrajectory = new Joint[NumberTrajectoryFeatures];
            var i = 0;
            foreach (var trajectoryFeature in mmData.TrajectoryFeatures)
            {
                if ((trajectoryFeature.FeatureType == TrajectoryFeature.Type.Position ||
                    trajectoryFeature.FeatureType == TrajectoryFeature.Type.Direction)
                    && !trajectoryFeature.SimulationBone)
                {
                    if (!poseSet.Skeleton.TryFind(trajectoryFeature.Bone, out jointsTrajectory[i])) Debug.Assert(false, "The skeleton does not contain any joint of type " + trajectoryFeature.Bone);
                }
                i += 1;
            }
            var jointsPose = new Joint[NumberPoseFeatures];
            i = 0;
            foreach (var poseFeature in mmData.PoseFeatures)
            {
                if (!poseSet.Skeleton.TryFind(poseFeature.Bone, out jointsPose[i])) Debug.Assert(false, "The skeleton does not contain any joint of type " + poseFeature.Bone);
                i += 1;
            }
            var jointsEnvironment = new Joint[NumberEnvironmentFeatures];
            i = 0;
            foreach (var environmentFeature in mmData.EnvironmentFeatures)
            {
                if ((environmentFeature.FeatureType == TrajectoryFeature.Type.Position ||
                    environmentFeature.FeatureType == TrajectoryFeature.Type.Direction)
                    && !environmentFeature.SimulationBone)
                {
                    if (!poseSet.Skeleton.TryFind(environmentFeature.Bone, out jointsEnvironment[i])) Debug.Assert(false, "The skeleton does not contain any joint of type " + environmentFeature.Bone);
                }
                i += 1;
            }
            // Extract Features
            for (var poseIndex = 0; poseIndex < nPoses; ++poseIndex)
            {
                if (poseSet.IsPoseValidForPrediction(poseIndex))
                {
                    Valid[poseIndex] = true;
                    ExtractFeature(poseSet, poseIndex, jointsTrajectory, jointsPose, jointsEnvironment, mmData);
                }
                else Valid[poseIndex] = false;
            }
        }

        /// <summary>
        /// Extract the feature vectors from poseSet
        /// </summary>
        private void ExtractFeature(PoseSet poseSet, int poseIndex, Joint[] jointsTrajectory, Joint[] jointsPose, Joint[] jointsEnvironment, MotionMatchingData mmData)
        {
            var featureIndex = poseIndex * FeatureSize;
            var nextPose = poseIndex + 1;
            if (nextPose >= poseSet.NumberPoses - poseSet.MaximumFramesPrediction)
            {
                nextPose = poseIndex;
            }
            var nextFeatureIndex = nextPose * FeatureSize;
            poseSet.GetPose(poseIndex, out var pose);
            poseSet.GetPose(nextPose, out var poseNext);
            // Compute local features based on the Simulation Bone
            // so hips and feet are local to a stable position with respect to the character
            GetWorldOriginCharacter(pose, out var characterOrigin, out var characterForward);

            // Trajectory Features -------------------------------------------------------------
            for (var i = 0; i < NumberTrajectoryFeatures; i++)
            {
                var trajectoryFeature = mmData.TrajectoryFeatures[i];
                var featureOffset = featureIndex + TrajectoryOffset[i];
                var nextFeatureOffset = nextFeatureIndex + TrajectoryOffset[i];
                var isStartFeature = true;
                for (var p = 0; p < trajectoryFeature.FramesPrediction.Length; ++p)
                {
                    var predictionOffset = featureOffset + p * NumberFloatsTrajectory[i];
                    var nextPredictionOffset = nextFeatureOffset + p * NumberFloatsTrajectory[i];
                    var futurePoseIndex = poseIndex + trajectoryFeature.FramesPrediction[p];
                    var nextFuturePoseIndex = nextPose + trajectoryFeature.FramesPrediction[p];

                    isStartFeature = ExtractTrajectoryFeature(trajectoryFeature, poseSet, futurePoseIndex, nextFuturePoseIndex,
                                                              jointsTrajectory, i, predictionOffset, characterOrigin, characterForward,
                                                              mmData, isStartFeature);
                }
            }

            // Pose Features -------------------------------------------------------------
            for (var i = 0; i < NumberPoseFeatures; i++)
            {
                var poseFeature = mmData.PoseFeatures[i];
                var featureOffset = featureIndex + PoseOffset + i * NumberFloatsPose;
                float3 feature = new();
                switch (poseFeature.FeatureType)
                {
                    case MotionMatchingData.PoseFeature.Type.Position:
                        feature = GetJointPosition(pose, poseSet.Skeleton, jointsPose[i], characterOrigin, characterForward);
                        break;
                    case MotionMatchingData.PoseFeature.Type.Velocity:
                        feature = GetJointVelocity(pose, poseNext, poseSet.Skeleton, jointsPose[i], characterOrigin, characterForward, poseSet.FrameTime);
                        break;
                    default:
                        Debug.Assert(false, "Unknown PoseFeature.Type: " + poseFeature.FeatureType);
                        break;
                }
                Features[featureOffset + 0] = feature.x;
                Features[featureOffset + 1] = feature.y;
                Features[featureOffset + 2] = feature.z;
            }

            // Environment Features -------------------------------------------------------------
            for (var i = 0; i < NumberEnvironmentFeatures; i++)
            {
                var environmentFeature = mmData.EnvironmentFeatures[i];
                var featureOffset = featureIndex + EnvironmentOffset[i];
                var nextFeatureOffset = nextFeatureIndex + EnvironmentOffset[i];
                var isStartFeature = true;
                for (var p = 0; p < environmentFeature.FramesPrediction.Length; ++p)
                {
                    var predictionOffset = featureOffset + p * NumberFloatsEnvironment[i];
                    var nextPredictionOffset = nextFeatureOffset + p * NumberFloatsEnvironment[i];
                    var futurePoseIndex = poseIndex + environmentFeature.FramesPrediction[p];
                    var nextFuturePoseIndex = nextPose + environmentFeature.FramesPrediction[p];

                    isStartFeature = ExtractTrajectoryFeature(environmentFeature, poseSet, futurePoseIndex, nextFuturePoseIndex,
                                                              jointsEnvironment, i, predictionOffset, characterOrigin, characterForward,
                                                              mmData, isStartFeature);
                }
            }
        }

        private bool ExtractTrajectoryFeature(TrajectoryFeature feature, PoseSet poseSet, int futurePoseIndex, int nextFuturePoseIndex,
                                              Joint[] joints, int featureIt, int predictionOffset, float3 characterOrigin, float3 characterForward,
                                              MotionMatchingData mmData, bool isStartFeature)
        {
            poseSet.GetPose(futurePoseIndex, out var futurePose, out var animationClip);
            poseSet.GetPose(nextFuturePoseIndex, out var nextFuturePose, out var nextAnimationClip);

            float3 value = new();
            switch (feature.FeatureType)
            {
                case TrajectoryFeature.Type.Position:
                    {
                        GetTrajectoryPosition(futurePose, poseSet.Skeleton, feature.SimulationBone, joints[featureIt], characterOrigin, characterForward,
                                              out value);
                    }
                    break;
                case TrajectoryFeature.Type.Direction:
                    {
                        GetTrajectoryDirection(futurePose, poseSet.Skeleton, feature.SimulationBone, joints[featureIt], characterForward, mmData,
                                               out value);
                        if (feature.ZeroX) value.x = 0;
                        if (feature.ZeroY) value.y = 0;
                        if (feature.ZeroZ) value.z = 0;
                        value = math.normalize(value);
                    }
                    break;
                case TrajectoryFeature.Type.Custom1D:
                    {
                        var extractor1D = feature.FeatureExtractor as Feature1DExtractor;
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor1D.StartExtracting(poseSet.Skeleton);
                        }
                        var value1D = extractor1D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.Skeleton, characterOrigin, characterForward);
                        Features[predictionOffset + 0] = value1D;
                    }
                    break;
                case TrajectoryFeature.Type.Custom2D:
                    {
                        var extractor2D = feature.FeatureExtractor as Feature2DExtractor;
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor2D.StartExtracting(poseSet.Skeleton);
                        }
                        var value2D = extractor2D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.Skeleton, characterOrigin, characterForward);
                        Features[predictionOffset + 0] = value2D.x;
                        Features[predictionOffset + 1] = value2D.y;
                    }
                    break;
                case TrajectoryFeature.Type.Custom3D:
                    {
                        var extractor3D = feature.FeatureExtractor as Feature3DExtractor;
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor3D.StartExtracting(poseSet.Skeleton);
                        }
                        var value3D = extractor3D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.Skeleton, characterOrigin, characterForward);
                        Features[predictionOffset + 0] = value3D.x;
                        Features[predictionOffset + 1] = value3D.y;
                        Features[predictionOffset + 2] = value3D.z;
                    }
                    break;
                case TrajectoryFeature.Type.Custom4D:
                    {
                        var extractor4D = feature.FeatureExtractor as Feature4DExtractor;
                        if (isStartFeature)
                        {
                            isStartFeature = false;
                            extractor4D.StartExtracting(poseSet.Skeleton);
                        }
                        var value4D = extractor4D.ExtractFeature(futurePose, futurePoseIndex, nextFuturePose, animationClip, poseSet.Skeleton, characterOrigin, characterForward);
                        Features[predictionOffset + 0] = value4D.x;
                        Features[predictionOffset + 1] = value4D.y;
                        Features[predictionOffset + 2] = value4D.z;
                        Features[predictionOffset + 3] = value4D.w;
                    }
                    break;
                default:
                    Debug.Assert(false, "Unsupported Feature Type: " + feature.FeatureType);
                    break;
            }
            if (feature.FeatureType == TrajectoryFeature.Type.Position ||
                feature.FeatureType == TrajectoryFeature.Type.Direction)
            {
                var offsetIndex = 0;
                var valueIndex = 0;
                var size = feature.GetSize();
                for (var f = 0; f < size; ++f)
                {
                    if (valueIndex == 0 && feature.ZeroX) valueIndex += 1;
                    if (valueIndex == 1 && feature.ZeroY) valueIndex += 1;
                    Features[predictionOffset + offsetIndex] = value[valueIndex];
                    valueIndex += 1;
                    offsetIndex += 1;
                }
            }
            return isStartFeature;
        }

        private static void GetTrajectoryPosition(PoseVector pose, Skeleton skeleton, bool simulationBone, Joint joint, float3 characterOrigin, float3 characterForward,
                                                  out float3 futureLocalPosition)
        {
            float3 worldPosition;
            if (simulationBone)
            {
                worldPosition = pose.JointLocalPositions[0];
            }
            else
            {
                worldPosition = pose.GetWorldSpacePosition(skeleton, joint);
            }
            futureLocalPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
        }
        private static void GetTrajectoryDirection(PoseVector pose, Skeleton skeleton, bool simulationBone, Joint joint, float3 characterForward, MotionMatchingData mmData,
                                                   out float3 futureLocalDirection)
        {
            quaternion worldRotation;
            float3 localForward;
            if (simulationBone)
            {
                worldRotation = pose.JointLocalRotations[0];
                localForward = math.forward();
            }
            else
            {
                worldRotation = pose.GetWorldSpaceRotation(skeleton, joint);
                localForward = mmData.GetLocalForward(joint.index);
            }
            var worldDirection = math.mul(worldRotation, localForward);
            futureLocalDirection = GetLocalDirectionFromCharacter(worldDirection, characterForward);
        }

        private static float3 GetJointPosition(PoseVector pose, Skeleton skeleton, Joint joint, float3 characterOrigin, float3 characterForward)
        {
            var worldPosition = pose.GetWorldSpacePosition(skeleton, joint);
            var localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
            return localPosition;
        }
        private static float3 GetJointVelocity(PoseVector pose, PoseVector poseNext, Skeleton skeleton, Joint joint, float3 characterOrigin, float3 characterForward, float frameTime)
        {
            var worldPosition = pose.GetWorldSpacePosition(skeleton, joint);
            var worldPositionNext = poseNext.GetWorldSpacePosition(skeleton, joint);
            var localPosition = GetLocalPositionFromCharacter(worldPosition, characterOrigin, characterForward);
            var localVelocity = (GetLocalPositionFromCharacter(worldPositionNext, characterOrigin, characterForward) - localPosition) / frameTime;
            return localVelocity;
        }

        /// <summary>
        /// Returns the position and forward vector of the character in world space using the pose vector simulation bone
        /// </summary>
        /// <param name="hipsForwardLocalVector">forward vector of the hips in world space when in bind pose</param>
        public static void GetWorldOriginCharacter(PoseVector poseVector, out float3 center, out float3 forward)
        {
            center = poseVector.JointLocalPositions[0]; // Simulation Bone World Position
            forward = math.mul(poseVector.JointLocalRotations[0], math.forward()); // Simulation Bone World Rotation
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
            if (Valid != null && Valid.IsCreated) Valid.Dispose();
            if (Features != null && Features.IsCreated) Features.Dispose();
            if (LargeBoundingBoxMin != null && LargeBoundingBoxMin.IsCreated) LargeBoundingBoxMin.Dispose();
            if (LargeBoundingBoxMax != null && LargeBoundingBoxMax.IsCreated) LargeBoundingBoxMax.Dispose();
            if (SmallBoundingBoxMin != null && SmallBoundingBoxMin.IsCreated) SmallBoundingBoxMin.Dispose();
            if (SmallBoundingBoxMax != null && SmallBoundingBoxMax.IsCreated) SmallBoundingBoxMax.Dispose();
            if (AdaptativeFeaturesIndices != null && AdaptativeFeaturesIndices.IsCreated) AdaptativeFeaturesIndices.Dispose();
        }
        
        public float3 Get3DValuePositionOrDirectionFeature(TrajectoryFeature trajectoryFeature, int currentFrame, int trajectoryFeatureIndex, int predictionIndex, bool isEnvironment)
        {
            var t = trajectoryFeatureIndex;
            var p = predictionIndex;

            float3 value;
            if (!trajectoryFeature.ZeroX && !trajectoryFeature.ZeroY && !trajectoryFeature.ZeroZ)
            {
                value = isEnvironment ? Get3DEnvironmentFeature(currentFrame, t, p) : Get3DTrajectoryFeature(currentFrame, t, p, true);
            }
            else if (!trajectoryFeature.ZeroX && !trajectoryFeature.ZeroY)
            {
                var value2D = isEnvironment ? Get2DEnvironmentFeature(currentFrame, t, p) : Get2DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(value2D.x, value2D.y, 0);
            }
            else if (!trajectoryFeature.ZeroX && !trajectoryFeature.ZeroZ)
            {
                var value2D = isEnvironment ? Get2DEnvironmentFeature(currentFrame, t, p) : Get2DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(value2D.x, 0.0f, value2D.y);
            }
            else if (!trajectoryFeature.ZeroY && !trajectoryFeature.ZeroZ)
            {
                var value2D = isEnvironment ? Get2DEnvironmentFeature(currentFrame, t, p) : Get2DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(0.0f, value2D.x, value2D.y);
            }
            else if (!trajectoryFeature.ZeroX)
            {
                var value1D = isEnvironment ? Get1DEnvironmentFeature(currentFrame, t, p) : Get1DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(value1D, 0.0f, 0.0f);
            }
            else if (!trajectoryFeature.ZeroY)
            {
                var value1D = isEnvironment ? Get1DEnvironmentFeature(currentFrame, t, p) : Get1DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(0.0f, value1D, 0.0f);
            }
            else if (!trajectoryFeature.ZeroZ)
            {
                var value1D = isEnvironment ? Get1DEnvironmentFeature(currentFrame, t, p) : Get1DTrajectoryFeature(currentFrame, t, p, true);
                value = new float3(0.0f, 0.0f, value1D);
            }
            else
            {
                Debug.Assert(false, "Invalid trajectory feature");
                value = float3.zero;
            }
            return value;
        }
    }
}
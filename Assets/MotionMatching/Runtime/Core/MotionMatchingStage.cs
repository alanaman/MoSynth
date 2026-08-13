using System;
using System.Collections.Generic;
using System.Diagnostics.Contracts;
using AnimationTools;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.Serialization;
using System.Linq;

namespace MotionMatching
{
[Serializable]
public class MotionMatchingStage : MoSynthStage
{
    private MotionSynthesisComponent _owner;
    
    [FormerlySerializedAs("characterController")] public MoSynthControlInput controlInput;
    
    
    public MotionMatchingData mmData;
    private PoseSet _poseSet;
    
    [SerializeReference] [SubclassSelector]
    public MotionMatchingSearch mmSearch = new BvhMotionMatchingSearch();
    
    /// <summary>
    /// The interval in seconds between two Motion Matching searches when there are no sudden input changes.
    /// </summary>
    public float searchInterval = 10.0f / 60.0f;
    
    /// <summary>
    /// The time left until the next search.
    /// </summary>
    private float _searchTimeLeft;

    [Tooltip("How important is the trajectory (future positions + future directions)")]
    [Range(0.0f, 1.0f)]
    public float responsiveness = 1.0f;

    [Tooltip("How important is the current pose")] [Range(0.0f, 1.0f)]
    public float quality = 1.0f;

    [SerializeField]
    private int minFrameSwitchDistance = 20;

    
    // TODO: editor inspector for feature weights
    [SerializeField]
    private List<float> featureWeights = new();
    NativeArray<float> _featureWeights;
    public NativeArray<float> FeatureWeights => _featureWeights;

    private NativeArray<float> _queryFeatureVector;
    public NativeArray<float> QueryFeatureVector => _queryFeatureVector;

    
    /// <summary>
    /// Current frame index in the pose/feature set
    /// </summary>
    public int CurrentFrame { get; private set; }

    private NativeArray<bool> _tagMask;

    /// <summary>
    /// Current frame index as float to keep track of variable frame rate
    /// </summary>
    private float _currentFrameTime;

    private float _databaseFrameRate;


    // Contact TODO: this frame? prev frame ?
    public bool IsLeftFootContact { get; private set; }
    public bool IsRightFootContact { get; private set; }

    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        _owner = motionSynthesisComponent;
        _poseSet = mmData.GetOrImportPoseSet();
        var featureSet = mmData.GetOrImportFeatureSet();
        
        Assert.IsTrue(controlInput, "mmCharacterController not set");
        // Force search on significant input change
        controlInput.OnHighInputChange += () => { _searchTimeLeft = 0; };
        
        Assert.IsTrue(
            motionSynthesisComponent.SkeletonTransforms.Length == _poseSet.SkeletonAsset.BoneCount,
            "Number of Skeleton transforms does not match skeleton bones " +
            "in MotionMatchingData.");
        
        _databaseFrameRate = 1f / _poseSet.FrameTime;
        
        _featureWeights = new NativeArray<float>(featureSet.FeatureSize, Allocator.Domain);
        // copy serialized weights
        for (int i = 0; i < math.min(featureWeights.Count, _featureWeights.Length); i++)
        {
            _featureWeights[i] = featureWeights[i];
        }
        _queryFeatureVector = new NativeArray<float>(featureSet.FeatureSize, Allocator.Domain);
        
        _tagMask = new NativeArray<bool>(featureSet.NumberFeatureVectors, Allocator.Domain);
        for (var i = 0; i < _tagMask.Length; i++)
        {
            _tagMask[i] = true;
        }
        
        // Search first Frame valid (to start with a valid pose)
        for (var i = 0; i < featureSet.NumberFeatureVectors; i++)
        {
            if (!featureSet.IsValidFeature(i)) continue;
            CurrentFrame = i;
            break;
        }
        
        mmSearch.Initialize(featureSet, _tagMask, _featureWeights);
    }

    public override SkeletonAsset GetSkeleton(SkeletonAsset inSkeleton)
    {
        _poseSet = mmData.GetOrImportPoseSet();
        return _poseSet.SkeletonAsset;
    }
    
    public override bool Apply(PoseBuffer pose, float deltaTime)
    {
        // _searchTimeLeft -= deltaTime;
        if (_searchTimeLeft <= 0)
        {
            FillQueryVector();
            
            var currentDistance = float.MaxValue;
            var featureSet = mmData.FeatureSet;
            var isCurrentFrameValid = featureSet.IsValidFeature(CurrentFrame) && _tagMask[CurrentFrame];
            if(isCurrentFrameValid)
            {
                var currentFeatureVector = featureSet.GetFeatureVector(CurrentFrame);
                currentDistance = SqrDistance(_queryFeatureVector, currentFeatureVector, _featureWeights);
            }

            var bestFrame = mmSearch.FindBestFrame(_queryFeatureVector, currentDistance);
            
            if(isCurrentFrameValid && bestFrame == -1) bestFrame = CurrentFrame;
            Debug.Assert(bestFrame != -1, "Motion Matching is not able to find any valid pose. Maybe the motion database is empty or the query tag used produces an empty set of poses?");
            
            if(math.abs(CurrentFrame - bestFrame) > minFrameSwitchDistance)
            {
                CurrentFrame = bestFrame;
            }
            
            _searchTimeLeft = searchInterval;
        }
        else
        {
            _searchTimeLeft -= deltaTime;
        }
        
        // Advance frames with time
        _currentFrameTime = CurrentFrame + math.frac(_currentFrameTime);
        _currentFrameTime += deltaTime * _databaseFrameRate;
        CurrentFrame = (int)math.floor(_currentFrameTime);
        
        pose.CopyFrom(_poseSet.GetPoseBuffer(CurrentFrame));
        
        // pose.jointLocalPositions[0] = math.transform(_animToWorld, pose.jointLocalPositions[0]);
        // pose.jointLocalRotations[0] = math.mul(new quaternion(_animToWorld), pose.jointLocalRotations[0]);
        return true;
    }
    
    [Pure]
    public static float SqrDistance(ReadOnlySpan<float> featureVectorA, ReadOnlySpan<float> featureVectorB, ReadOnlySpan<float> featureWeights)
    {
        var sqrDistance = 0.0f;
        for (int i = 0; i < featureVectorA.Length; i++)
        {
            var diff = featureVectorA[i] - featureVectorB[i];
            sqrDistance += diff * diff * featureWeights[i];
        }
        return sqrDistance;
    }
    
    public void FillQueryVector()
    {
        var simulationBone = _owner.SkeletonTransforms[0];
        var queryFeatureSpan = _queryFeatureVector.AsSpan();
        
        // Trajectory features
        var offset = 0;
        foreach (var featureDef in mmData.trajectoryFeatures)
        {
            var featureSize = featureDef.GetSize();
            for (var p = 0; p < featureDef.framesPrediction.Length; ++p)
            {
                var feature = queryFeatureSpan.Slice(offset, featureSize);
                controlInput.GetTrajectoryFeature(featureDef, p, simulationBone, feature);
                offset += featureSize;
            }
        }
        
        var featureSet = mmData.GetOrImportFeatureSet();
        featureSet.NormalizeTrajectory(queryFeatureSpan);

        // TODO:
        // The currentPose of the character could be quite different from the 
        // one poses stored in mmData due to retargeting. 
        // We can use the currentPose if we implement a backpropagation
        // that can inverse the retargeting.
        
        // // Pose features
        // for (int i = 0; i < featureSet.NumberPoseFeatures; i++)
        // {
        //     var poseFeatureDef = mmData.PoseFeatures[i];
        //     var featureOffset = featureSet.PoseOffset + i * FeatureSet.NumberFloatsPose;
        //     var currPose = _owner.CurrentPose;
        //     var skeleton = _poseSet.Skeleton;
        //     var joint = skeleton.Find(poseFeatureDef.Bone);
        //     if (poseFeatureDef.FeatureType == MotionMatchingData.PoseFeature.Type.Position)
        //     {
        //         var feature = currPose.GetRootSpacePosition(skeleton, joint);
        //         
        //         queryFeatureSpan[featureOffset + 0] = feature.x;
        //         queryFeatureSpan[featureOffset + 1] = feature.y;
        //         queryFeatureSpan[featureOffset + 2] = feature.z;
        //     }
        //     else if (poseFeatureDef.FeatureType == MotionMatchingData.PoseFeature.Type.Velocity)
        //     {
        //         var feature = currPose.GetRootSpaceVelocity(skeleton, joint);
        //         queryFeatureSpan[featureOffset + 0] = feature.x;
        //         queryFeatureSpan[featureOffset + 1] = feature.y;
        //         queryFeatureSpan[featureOffset + 2] = feature.z;
        //     }
        //     else
        //     {
        //         throw new Exception("Unknown PoseFeature.Type: " + poseFeatureDef.FeatureType);
        //     }
        // }
        
        featureSet.GetPoseFeatures(queryFeatureSpan.Slice(offset, featureSet.PoseFloatCount), CurrentFrame);
        
        // Environment features
        if (featureSet.EnvironmentOffset.Length > 0)
        {
            offset = featureSet.EnvironmentOffset[0];
            foreach (var featureDef in mmData.environmentFeatures)
            {
                for (var p = 0; p < featureDef.framesPrediction.Length; p++)
                {
                    var featureSize = featureDef.GetSize();
                    var feature = queryFeatureSpan.Slice(offset, featureSize);
                    controlInput.GetEnvironmentFeature(featureDef, p, simulationBone, feature);
                    offset += featureSize;
                }
            }
        }
        
        // featureSet.NormalizeFeatureVector(_queryFeatureVector);
    }

    // TODO call from editor
    public void UpdateFeatureWeights()
    {
        var offset = 0;
        for (var i = 0; i < mmData.trajectoryFeatures.Count; i++)
        {
            var feature = mmData.trajectoryFeatures[i];
            var featureSize = feature.GetSize();
            var weight = _featureWeights[i] * responsiveness;
            for (var p = 0; p < feature.framesPrediction.Length; ++p)
            {
                for (var f = 0; f < featureSize; f++)
                {
                    _featureWeights[offset + f] = weight;
                }

                offset += featureSize;
            }
        }

        for (var i = 0; i < mmData.poseFeatures.Count; i++)
        {
            var weight = _featureWeights[i + mmData.trajectoryFeatures.Count] * quality;
            _featureWeights[offset + 0] = weight;
            _featureWeights[offset + 1] = weight;
            _featureWeights[offset + 2] = weight;
            offset += 3;
        }

        for (var i = 0; i < mmData.environmentFeatures.Count; i++)
        {
            var feature = mmData.environmentFeatures[i];
            var featureSize = feature.GetSize();
            var baseWeight = _featureWeights[i + mmData.trajectoryFeatures.Count + mmData.poseFeatures.Count];
            for (var p = 0; p < feature.framesPrediction.Length; ++p)
            {
                for (var f = 0; f < featureSize; f++)
                {
                    _featureWeights[offset + f] = baseWeight;
                }

                offset += featureSize;
            }
        }
    }

    public override void OnValidate()
    {
        if(mmData == null) return;
        var featureSize = mmData.GetOrImportFeatureSet().FeatureSize;
        if(featureWeights.Count < featureSize)
        {
            for (var i = featureWeights.Count; i < featureSize; i++)
            {
                featureWeights.Add(1.0f);
            }
        }
        else if(featureWeights.Count > featureSize)
        {
            featureWeights.RemoveRange(featureSize, featureWeights.Count - featureSize);
        }

        if (_featureWeights.Length != featureSize)
        {
            _featureWeights = new NativeArray<float>(featureSize, Allocator.Domain);
        }

        for (int i = 0; i < featureWeights.Count; i++)
        {
            _featureWeights[i] = featureWeights[i];
        }
    }

    public MotionMatchingData MmData => mmData;
    public float DatabaseFrameTime => mmData.GetOrImportPoseSet().FrameTime;
    public float3 RootVelocity { get; }
    public float3 RootAngularVelocity { get; }
    public float3 RootPosition { get; }
    public quaternion RootRotation { get; }
    public void SetRotAdjustment(quaternion adjustmentRotation)
    {
        throw new NotImplementedException();
    }

    public void SetPosAdjustment(float3 adjustmentPosition)
    {
        throw new NotImplementedException();
    }

    public float3 GetMainPositionFeature(int trajectoryIndex)
    {
        throw new NotImplementedException();
    }

    public float4 GetEnvironmentFeature(string featureName, int trajectoryIndex)
    {
        throw new NotImplementedException();
    }
}
}
using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Assertions;

namespace MotionMatching
{
[Serializable]
public class MotionMatchingStage : MoSynthStage
{
    private MotionSynthesisComponent _owner;
    
    // TODO: rename: MM shouldn't be in the name?
    public MotionMatchingCharacterController characterController;
    
    
    public MotionMatchingData mmData;
    private PoseSet _poseSet;
    
    [SerializeReference] [SubclassSelector]
    public MotionMatchingSearch mmSearch = new BvhMotionMatchingSearch();
    
    public bool lockFPS = true;
    
    /// <summary>
    /// The interval in seconds between two Motion Matching searches when there are no sudden input changes.
    /// </summary>
    public float searchInterval = 10.0f / 60.0f;
    
    /// <summary>
    /// The time left until the next search.
    /// </summary>
    private float _searchTimeLeft = 0.0f;

    [Tooltip("How important is the trajectory (future positions + future directions)")]
    [UnityEngine.Range(0.0f, 1.0f)]
    public float responsiveness = 1.0f;

    [Tooltip("How important is the current pose")] [UnityEngine.Range(0.0f, 1.0f)]
    public float quality = 1.0f;

    
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

    /// <summary>
    /// Frame before the last Motion Matching Search
    /// </summary>
    public int LastMmSearchFrame { get; private set; }

    private NativeArray<bool> _tagMask;
    
    private float3 _animationSpaceOriginPos;
    private quaternion _animationSpaceOriginRot;
    private quaternion _inverseAnimationSpaceOriginRot;

    /// <summary>
    /// Position of the transform right after motion matching search
    /// </summary>
    private float3 _mmTransformOriginPos;
    /// <summary>
    /// Rotation of the transform right after motion matching search
    /// </summary>
    private quaternion _mmTransformOriginRot; 
    /// <summary>
    /// Current frame index as float to keep track of variable frame rate
    /// </summary>
    private float _currentFrameTime;

    

    // Contact TODO: this frame? prev frame ?
    public bool IsLeftFootContact { get; private set; }
    public bool IsRightFootContact { get; private set; }

    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        _owner = motionSynthesisComponent;
        _poseSet = mmData.GetOrImportPoseSet();
        var featureSet = mmData.GetOrImportFeatureSet();

        Assert.IsTrue(
            motionSynthesisComponent.skeletonTransforms.Length == _poseSet.Skeleton.Joints.Count,
            "Number of Skeleton transforms does not match skeleton bones " +
            "in MotionMatchingData.");
        
        // FPS
        var databaseFrameTime = _poseSet.FrameTime;
        var databaseFrameRate = Mathf.RoundToInt(1.0f / databaseFrameTime);
        if (lockFPS)
        {
            Application.targetFrameRate = databaseFrameRate;
            Debug.Log(
                "[Motion Matching] Updated Target FPS: " +
                Application.targetFrameRate);
        }
        else
        {
            Application.targetFrameRate = -1;
            Debug.LogWarning(
                "[Motion Matching] LockFPS is not set. Motion Matching" +
                " will malfunction if the application frame rate is higher" +
                " than the animation database.");
        }
        
        var numberFeatures = mmData.TrajectoryFeatures.Count + mmData.PoseFeatures.Count + mmData.EnvironmentFeatures.Count;
        
        Assert.IsTrue(
            _featureWeights.Length == numberFeatures, 
            "Feature weights length does not match the number of features.");
        
        
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
            if (featureSet.IsValidFeature(i))
            {
                LastMmSearchFrame = i;
                CurrentFrame = i;
                _poseSet.GetPose(i, out var pose);
                _animationSpaceOriginPos = pose.JointLocalPositions[0];
                _animationSpaceOriginRot = pose.JointLocalRotations[0];
                _inverseAnimationSpaceOriginRot = math.inverse(_animationSpaceOriginRot);
                
                // TODO: is this needed?
                // _mmTransformOriginPos = skeletonTransforms[0].position;
                // _mmTransformOriginRot = skeletonTransforms[0].rotation;
                break;
            }
        }
        
        mmSearch.Initialize(featureSet, _tagMask, _featureWeights);
        
    }

    public override Skeleton GetSkeleton(in Skeleton inSkeleton)
    {
        _poseSet = mmData.GetOrImportPoseSet();
        return _poseSet.Skeleton;
    }

    // // TODO: not here
    // PoseVector ConstructPoseFromSkeletonTransforms(in PoseVector prevPose, Transform[] skeletonTransforms)
    // {
    //     var pose = new PoseVector();
    //     
    //     // Simulation Bone
    //     float3 pos = skeletonTransforms[0].position;
    //     var rot = skeletonTransforms[0].rotation;
    //     
    //     // world space to local space
    //     var localSpacePos = math.mul(math.inverse(_mmTransformOriginRot), (pos - _mmTransformOriginPos));
    //     var localSpaceRot = math.mul(math.inverse(_mmTransformOriginRot), rot);
    //     
    //     // local space to animation space
    //     pose.JointLocalRotations[0] = math.mul(_animationSpaceOriginRot, localSpaceRot);
    //     
    //     pose.JointLocalPositions[0] = math.mul(_inverseAnimationSpaceOriginRot, localSpacePos) + _animationSpaceOriginPos;
    //     
    //     for (var i = 1; i < skeletonTransforms.Length; i++)
    //     {
    //         pose.JointLocalRotations[i] = skeletonTransforms[i].localRotation;
    //     }
    //
    //     // hip
    //     pose.JointLocalPositions[1] = skeletonTransforms[1].localPosition;
    //
    //     for (int i = 0; i < pose.JointLocalAngularVelocities.Length; i++)
    //     {
    //         pose.JointLocalAngularVelocities[i] = math.Euler(math.mul(pose.JointLocalRotations[i], prevPose.JointLocalRotations[i]));
    //         pose.JointLocalVelocities[i] = pose.JointLocalPositions[i] - prevPose.JointLocalPositions[i];
    //     }
    //     
    //     pose.LeftFootContact = ?;
    //     pose.RightFootContact = ?;
    //             
    //     return pose;
    // }

    public override void Apply(PoseVector pose, float deltaTime)
    {
        _searchTimeLeft -= deltaTime;
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
            
            
            CurrentFrame = bestFrame;
            
            _searchTimeLeft = searchInterval;
        }
        
        // Advance frames with time
        _currentFrameTime = CurrentFrame + math.frac(_currentFrameTime);
        _currentFrameTime += deltaTime / _poseSet.FrameTime;
        CurrentFrame = (int)math.floor(_currentFrameTime);
        
        _poseSet.GetPose(CurrentFrame, out pose);
    }

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
        var simulationBone = _owner.skeletonTransforms[0];
        var queryFeatureSpan = _queryFeatureVector.AsSpan();
        
        // Trajectory features
        var offset = 0;
        foreach (var featureDef in mmData.TrajectoryFeatures)
        {
            var featureSize = featureDef.GetSize();
            var feature = queryFeatureSpan.Slice(offset, featureSize);
            for (var p = 0; p < featureDef.FramesPrediction.Length; ++p)
            {
                characterController.GetTrajectoryFeature(featureDef, p, simulationBone, feature);
                offset += featureSize;
            }
        }
        var featureSet = mmData.GetOrImportFeatureSet();

        // Pose features
        for (int i = 0; i < featureSet.NumberPoseFeatures; i++)
        {
            var poseFeatureDef = mmData.PoseFeatures[i];
            var featureOffset = featureSet.PoseOffset + i * FeatureSet.NumberFloatsPose;
            var currPose = _owner.CurrentPose;
            var skeleton = _poseSet.Skeleton;
            var joint = skeleton.Find(poseFeatureDef.Bone);
            if (poseFeatureDef.FeatureType == MotionMatchingData.PoseFeature.Type.Position)
            {
                var feature = currPose.GetCharacterSpacePosition(skeleton, joint);
                
                queryFeatureSpan[featureOffset + 0] = feature.x;
                queryFeatureSpan[featureOffset + 1] = feature.y;
                queryFeatureSpan[featureOffset + 2] = feature.z;
            }
            else if (poseFeatureDef.FeatureType == MotionMatchingData.PoseFeature.Type.Velocity)
            {
                var feature = currPose.GetCharacterSpaceVelocity(skeleton, joint);
                queryFeatureSpan[featureOffset + 0] = feature.x;
                queryFeatureSpan[featureOffset + 1] = feature.y;
                queryFeatureSpan[featureOffset + 2] = feature.z;
            }
            else
            {
                throw new Exception("Unknown PoseFeature.Type: " + poseFeatureDef.FeatureType);
            }
        }
        
        // Environment features
        if (featureSet.EnvironmentOffset.Length > 0)
        {
            offset = featureSet.EnvironmentOffset[0];
            foreach (var featureDef in mmData.EnvironmentFeatures)
            {
                for (var p = 0; p < featureDef.FramesPrediction.Length; p++)
                {
                    var featureSize = featureDef.GetSize();
                    var feature = queryFeatureSpan.Slice(offset, featureSize);
                    characterController.GetEnvironmentFeature(featureDef, p, simulationBone, feature);
                    offset += featureSize;
                }
            }
        }
        
        featureSet.NormalizeFeatureVector(_queryFeatureVector);
    }
    
    /// <summary>
    /// Motion Matching will only search over those poses belonging to the query tag
    /// </summary>
    public void SetQueryTag(QueryTag query)
    {
        var poseSet = mmData.GetOrImportPoseSet();
        query.ComputeRanges(poseSet);
        var job = new SetTagBurst
        {
            MaximumFramesPrediction = poseSet.MaximumFramesPrediction,
            TagMask = _tagMask,
            StartRanges = query.GetStartRanges(),
            EndRanges = query.GetEndRanges(),
        };
        job.Schedule().Complete();
    }
    
    // TODO call from editor
    public void UpdateFeatureWeights()
    {
        var offset = 0;
        for (var i = 0; i < mmData.TrajectoryFeatures.Count; i++)
        {
            var feature = mmData.TrajectoryFeatures[i];
            var featureSize = feature.GetSize();
            var weight = _featureWeights[i] * responsiveness;
            for (var p = 0; p < feature.FramesPrediction.Length; ++p)
            {
                for (var f = 0; f < featureSize; f++)
                {
                    _featureWeights[offset + f] = weight;
                }

                offset += featureSize;
            }
        }

        for (var i = 0; i < mmData.PoseFeatures.Count; i++)
        {
            var weight = _featureWeights[i + mmData.TrajectoryFeatures.Count] * quality;
            _featureWeights[offset + 0] = weight;
            _featureWeights[offset + 1] = weight;
            _featureWeights[offset + 2] = weight;
            offset += 3;
        }

        for (var i = 0; i < mmData.EnvironmentFeatures.Count; i++)
        {
            var feature = mmData.EnvironmentFeatures[i];
            var featureSize = feature.GetSize();
            var baseWeight = _featureWeights[i + mmData.TrajectoryFeatures.Count + mmData.PoseFeatures.Count];
            for (var p = 0; p < feature.FramesPrediction.Length; ++p)
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
        var numFeatures = mmData.TrajectoryFeatures.Count + mmData.PoseFeatures.Count + mmData.EnvironmentFeatures.Count;
        
        if(featureWeights.Count < numFeatures)
        {
            featureWeights.AddRange(new float[numFeatures - featureWeights.Count]);
        }
        else if(featureWeights.Count > numFeatures)
        {
            featureWeights.RemoveRange(numFeatures, featureWeights.Count - numFeatures);
        }

        if (_featureWeights.Length != numFeatures)
        {
            _featureWeights = new NativeArray<float>(numFeatures, Allocator.Domain);
        }

        for (int i = 0; i < featureWeights.Count; i++)
        {
            _featureWeights[i] = featureWeights[i];
        }
    }
}
}
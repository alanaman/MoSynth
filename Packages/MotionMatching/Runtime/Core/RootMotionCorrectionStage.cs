using System;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
[Serializable]
public class RootMotionCorrectionStage : MoSynthStage
{
    Transform _root;
    
    MotionSynthesizer _owner;
    
    float3 _rootPosition;
    quaternion _rootRotation;

    private bool _hasRootJumped = true;
    private float3 _animSpacePos;
    private quaternion _animSpaceRot;
    private float3 _transformPosAtLastJump;
    private quaternion _transformRotAtLastJump;

    public override void Init(MotionSynthesizer motionSynthesisComponent)
    {
        _owner = motionSynthesisComponent;
        _root = motionSynthesisComponent.transform;
        _rootPosition = _root.position;
        _rootRotation = _root.rotation;
    }

    public override void Apply(PoseVector pose, float deltaTime)
    {
        if (_hasRootJumped)
        {
            _animSpacePos = pose.JointLocalPositions[0];
            _animSpaceRot = pose.JointLocalRotations[0];
            _transformPosAtLastJump = _owner.transform.position;
            _transformRotAtLastJump = _owner.transform.rotation;
            
            _hasRootJumped = false;
        }
        else
        {
            var newAnimSpacePos = pose.JointLocalPositions[0];
            var newAnimSpaceRot = pose.JointLocalRotations[0];
            var posWrtLastJump = math.mul(math.inverse(_animSpaceRot), 
                                   (newAnimSpacePos - _animSpacePos));
            var rotWrtLastJump = math.mul(math.inverse(_animSpaceRot), newAnimSpaceRot);
            
            pose.JointLocalPositions[0] = _transformPosAtLastJump + math.mul(_transformRotAtLastJump, posWrtLastJump);
            pose.JointLocalRotations[0] = math.mul(_transformRotAtLastJump, rotWrtLastJump);
        }
    }
}
}
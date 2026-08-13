using System;
using AnimationTools;
using Unity.Mathematics;
using UnityEngine;

namespace MotionMatching
{
[Serializable]
public class RootMotionCorrectionStage : MoSynthStage
{
    Transform _root;
    
    MotionSynthesisComponent _owner;
    
    float3 _rootPosition;
    quaternion _rootRotation;

    private bool _hasRootJumped = true;
    private float3 _animSpacePos;
    private quaternion _animSpaceRot;
    private float3 _transformPosAtLastJump;
    private quaternion _transformRotAtLastJump;

    public override void Init(MotionSynthesisComponent motionSynthesisComponent)
    {
        _owner = motionSynthesisComponent;
        _root = motionSynthesisComponent.transform;
        _rootPosition = _root.position;
        _rootRotation = _root.rotation;
    }

    public override bool Apply(PoseBuffer pose, float deltaTime)
    {
        var positions = pose.Positions;
        var rotations = pose.Rotations;

        if (_hasRootJumped)
        {
            _animSpacePos = positions[0];
            _animSpaceRot = rotations[0];
            _transformPosAtLastJump = _owner.transform.position;
            _transformRotAtLastJump = _owner.transform.rotation;

            _hasRootJumped = false;
        }
        else
        {
            var newAnimSpacePos = positions[0];
            var newAnimSpaceRot = rotations[0];
            var posWrtLastJump = math.mul(math.inverse(_animSpaceRot),
                                   (newAnimSpacePos - _animSpacePos));
            var rotWrtLastJump = math.mul(math.inverse(_animSpaceRot), newAnimSpaceRot);

            positions[0] = _transformPosAtLastJump + math.mul(_transformRotAtLastJump, posWrtLastJump);
            rotations[0] = math.mul(_transformRotAtLastJump, rotWrtLastJump);
        }
        return true;
    }
}
}
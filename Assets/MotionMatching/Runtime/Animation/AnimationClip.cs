using MotionMatching;
using UnityEngine;

namespace MoSynth
{
public class AnimationClip : ScriptableObject
{
    [SerializeField] private float frameTime;
    public float FrameTime => frameTime;

    [SerializeField] private Skeleton skeleton;
    public Skeleton Skeleton => skeleton;
}
}
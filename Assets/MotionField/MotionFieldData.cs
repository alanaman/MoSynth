using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

namespace MotionMatching.MotionField
{
public class MotionFieldData : ScriptableObject
{
    [SerializeField]
    private List<AnnotatedAnimationClip> animationDataList = new List<AnnotatedAnimationClip>();
    
    
    
}
}
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.Serialization;

namespace MotionMatching.MotionField
{
public class MotionFieldData : ScriptableObject
{
    [SerializeField]
    private List<AnimationData> animationDataList = new List<AnimationData>();
    
    
    
}
}
using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace AnimationTools
{
    [Serializable]
    public struct JointToMecanim
    {
        [FormerlySerializedAs("Name")] public string name;
        [FormerlySerializedAs("MecanimBone")] public HumanBodyBones mecanimBone;

        public JointToMecanim(string name, HumanBodyBones mecanimBone)
        {
            this.name = name;
            this.mecanimBone = mecanimBone;
        }
    }
}

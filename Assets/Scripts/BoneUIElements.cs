// using UnityEngine;
// using UnityEngine.UI;
//
// /// <summary>
// /// Automatically attached to generated buttons. 
// /// Handles registering the onClick event to pass the specific bone transform back to the manager.
// /// </summary>
// [RequireComponent(typeof(Button))]
// public class BoneUIElement : MonoBehaviour
// {
//     public Transform targetBone;
//
//     private void Awake()
//     {
//         Button btn = GetComponent<Button>();
//         if (btn != null)
//         {
//             btn.onClick.AddListener(HandleClick);
//         }
//     }
//
//     private void HandleClick()
//     {
//         if (manager != null && targetBone != null)
//         {
//             manager.NotifyBoneClicked(targetBone);
//         }
//     }
// }
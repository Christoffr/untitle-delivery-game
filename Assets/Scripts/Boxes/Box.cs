using UnityEngine;

public class Box : MonoBehaviour
{
    private void OnJointBreak(float breakForce)
    {
        Debug.Log(breakForce);
    }
}

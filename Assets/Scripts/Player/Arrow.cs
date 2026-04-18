using UnityEngine;

public class Arrow : MonoBehaviour
{
    public Transform LookAtTarget;
    void Update()
    {
        if (LookAtTarget != null)
            transform.LookAt(LookAtTarget);
    }
}

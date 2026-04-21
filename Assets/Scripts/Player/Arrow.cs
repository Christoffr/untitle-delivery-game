using UnityEngine;

public class Arrow : MonoBehaviour
{
    [SerializeField]
    private GameObject _prefab;
    private Transform _lookAtTarget;

    private void Update()
    {
        if (_lookAtTarget != null)
            transform.LookAt(_lookAtTarget);
        else
            _prefab.SetActive(false);

    }

    public void SetTarget(Transform target)
    {
        _prefab.SetActive(target != null);
        _lookAtTarget = target;
    }
}
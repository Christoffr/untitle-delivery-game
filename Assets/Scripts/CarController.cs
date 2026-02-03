using UnityEngine;

public class CarController : MonoBehaviour
{
    [Header("References")]
    [SerializeField] private Rigidbody _carRB;
    [SerializeField] private Transform[] _raypoints;
    [SerializeField] private LayerMask _drivable;

    [Header("Suspensions Settings")]
    [SerializeField] private float _springStiffness;
    [SerializeField] private float _damperStiffness;
    [SerializeField] private float _restLenght;
    [SerializeField] private float _springTravel;
    [SerializeField] private float _wheelRadius;

    private void Start()
    {
        _carRB = GetComponent<Rigidbody>();
    }
    private void FixedUpdate()
    {
        Susspension();
    }

    private void Susspension()
    {
        foreach (Transform raypoint in _raypoints)
        {
            RaycastHit hit;
            float maxLenght = _restLenght + _springTravel;

            if (Physics.Raycast(raypoint.position, -raypoint.up, out hit, maxLenght + _wheelRadius, _drivable))
            {
                float currentSpringLenght = hit.distance - _wheelRadius;
                float springCompression = Mathf.Max(0, (_restLenght - currentSpringLenght) / _springTravel);

                float springVelocity = Vector3.Dot(_carRB.GetPointVelocity(raypoint.position), raypoint.up);
                float dampForce = _damperStiffness * springVelocity;

                float springForce = _springStiffness * springCompression;

                float netForce = springForce - dampForce;

                _carRB.AddForceAtPosition(netForce * raypoint.up, raypoint.position);

                Debug.DrawLine(raypoint.position, hit.point, Color.red);
            }
            else
            {
                Debug.DrawLine(raypoint.position, raypoint.position + (_wheelRadius + maxLenght) * -raypoint.up, Color.green);
            }
        }


    }
}

using UnityEngine;

public class QuestEnd : MonoBehaviour
{
    [Header("references")]
    [SerializeField] private GameObject _visuals;

    [Header("Quest Information")]
    [SerializeField] private string _endName;

    private bool _isAnActiveQuest = false;

    public string EndName { get { return _endName; } }

    public void Activate()
    {
        _visuals.SetActive(true);
        _isAnActiveQuest = true;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.tag == "Player" && _isAnActiveQuest)
        {
            other.GetComponent<QuestManager>().GetPackage();
            _visuals.SetActive(false);
            _isAnActiveQuest = false;
        }
    }
}

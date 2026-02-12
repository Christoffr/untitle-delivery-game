using UnityEngine;

public class QuestStart : MonoBehaviour
{
    [Header("references")]
    [SerializeField] private GameObject _visuals;

    [Header("Quest Information")]
    [SerializeField] private string _startName;

    private bool _isAnActiveQuest = false;

    public string StartName { get { return _startName; } }

    public void Activate()
    {
        _visuals.SetActive(true);
        _isAnActiveQuest = true;
    }

    private void OnTriggerEnter(Collider other)
    {
        if (other.tag == "Player" && _isAnActiveQuest)
        {
            other.GetComponent<QuestManager>().SetPackage(_startName);
            _visuals.SetActive(false);
            _isAnActiveQuest = false;
        }
    }
}

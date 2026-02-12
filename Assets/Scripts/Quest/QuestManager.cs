using System.Collections.Generic;
using UnityEngine;

public class QuestManager : MonoBehaviour
{
    [Header("Available Quests")]
    [SerializeField] private List<Quest> _availableQuests = new List<Quest>();

    [Header("References")]
    [SerializeField] private List<QuestStart> _questStart = new List<QuestStart>();
    [SerializeField] private List<QuestEnd> _questEnd = new List<QuestEnd>();

    [Header("Current Quest")]
    [SerializeField] private Quest _currentQuest;
    [SerializeField] private QuestStart _start;
    [SerializeField] private QuestEnd _end;

    [Header("Current Rewards")]
    [SerializeField] private int _currentRewards;

    private void Start()
    {
        if (_availableQuests.Count < 1)
            OfferQuest();
    }

    private void OfferQuest()
    {
        _start = _questStart[Random.Range(0, _questStart.Count)];
        _end = _questEnd[Random.Range(0, _questEnd.Count)];

        _currentQuest = new Quest(_start.StartName, _end.EndName, 10);

        _start.Activate();
    }

    public void SetPackage(string msg)
    {
        _end.Activate();
    }

    public void GetPackage()
    {
        _currentRewards += _currentQuest.Reward;
        _currentQuest = null;
    }
}

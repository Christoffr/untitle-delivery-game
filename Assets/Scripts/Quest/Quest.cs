    using UnityEngine;

[System.Serializable]
public class Quest
{
    [SerializeField] private string _startName;
    [SerializeField] private string _endName;
    [SerializeField] private int _reward;

    public string Name { get { return _startName; } }
    public string Description { get { return _endName; } }
    public int Reward { get { return _reward; } }

    public Quest(string startName, string endName, int reward)
    {
        _startName = startName;
        _endName = endName;
        _reward = reward;
    }
}
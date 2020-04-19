using System.Collections;
using System.Collections.Generic;
using UnityEngine;

using HoloToolkit.Unity.InputModule;


public class AirTapManager : MonoBehaviour, IInputClickHandler
{
    [SerializeField]
    public GameObject map;
    [SerializeField]
    public GameObject left_sensor;
    [SerializeField]
    public GameObject right_sensor;

    // 0: init
    // 1: running
    // 2: stopped
    private int localization_status_ = 0;


	// Use this for initialization
	void Start () {
        InputManager.Instance.AddGlobalListener(gameObject);
    }
	
	// Update is called once per frame
	void Update () {
		
	}

    void IInputClickHandler.OnInputClicked(InputClickedEventData eventData)
    {
        bool activate = !map.activeSelf;
        map.SetActive(activate);

        if(localization_status_ == 0)
        {
            localization_status_ = 1;

            CamSensorStreamer lstreamer = left_sensor.GetComponent<CamSensorStreamer>();
            lstreamer.OnTapped();
            CamSensorStreamer rstreamer = right_sensor.GetComponent<CamSensorStreamer>();
            rstreamer.OnTapped();
        }
        else if(localization_status_ == 1)
        {
            localization_status_ = 2;

            CamSensorStreamer lstreamer = left_sensor.GetComponent<CamSensorStreamer>();
            lstreamer.OnTapped();
            CamSensorStreamer rstreamer = right_sensor.GetComponent<CamSensorStreamer>();
            rstreamer.OnTapped();
        }
    }
}

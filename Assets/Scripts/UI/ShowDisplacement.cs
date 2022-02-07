﻿/****************************************************
 * File: ShowDisplacement.cs
   * Author: Eduardo Alvarado
   * Email: eduardo.alvarado-pinero@polytechnique.edu
   * Date: Created by LIX on 01/08/2021
   * Project: Real-Time Locomotion on Soft Grounds with Dynamic Footprints
   * Last update: 07/02/2022
*****************************************************/

using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UI;

public class ShowDisplacement : MonoBehaviour
{
    private Text displacement;

    void Start()
    {
        displacement = this.GetComponent<Text>();
    }

    void Update()
    {
        displacement.text = "Deform. - LF: " + ((FindObjectOfType<PhysicalFootprint>().heightCellDisplacementYoungLeft)*1000).ToString("#.#") +
            " mm | RF: " + ((FindObjectOfType<PhysicalFootprint>().heightCellDisplacementYoungRight)*1000).ToString("#.#") + " mm";
    }
}

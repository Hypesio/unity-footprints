using UnityEngine;

[ExecuteInEditMode]
public class WeightsTest : MonoBehaviour
{
    public bool launchTest = false;

    // Update is called once per frame
    void Update()
    {
        if (launchTest)
        {
            launchTest = false;
            LaunchTests();
        }
    }

    void LaunchTests()
    {
        int gridSize = 8;
        int[,] heightBoolMap = InitHeightBoolMap(gridSize);
        for (int j = 1; j < gridSize - 1; j++)
        {
            heightBoolMap[2, j] = (int)PhysicalFootprint.CellState.Contact;
        }
        Vector3 speed = new Vector3(1, 0, 1); 
        Debug.Log("Start test - Size " + gridSize + " Speed " + speed);
        PhysicalFootprintWeights.TestGrid(speed, gridSize, heightBoolMap);
        speed = new Vector3(12, 3, 1); 
        Debug.Log("Start test - Size " + gridSize + " Speed " + speed);
        PhysicalFootprintWeights.TestGrid(speed, gridSize, heightBoolMap);
        speed = new Vector3(0.3f, 0, 0.66f); 
        Debug.Log("Start test - Size " + gridSize + " Speed " + speed);
        PhysicalFootprintWeights.TestGrid(speed, gridSize, heightBoolMap);
        speed = new Vector3(-0.3f, 0, 1.3f); 
        Debug.Log("Start test - Size " + gridSize + " Speed " + speed);
        PhysicalFootprintWeights.TestGrid(speed, gridSize, heightBoolMap);
        speed = new Vector3(-0.3f, 0, -0.1f); 
        Debug.Log("Start test - Size " + gridSize + " Speed " + speed);
        PhysicalFootprintWeights.TestGrid(speed, gridSize, heightBoolMap);
    }

    int[,] InitHeightBoolMap(int gridSize)
    {
        int[,] heightBoolMap = new int[gridSize, gridSize];
        for (int i = 0; i < gridSize; i++)
        {
            for (int j = 0; j < gridSize; j++)
            {
                heightBoolMap[i, j] = (int)PhysicalFootprint.CellState.Contour;
            }
        }

        return heightBoolMap;
    }
}

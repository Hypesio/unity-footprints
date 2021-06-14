﻿using System;
using UnityEngine;
using System.Collections;

public static class DrawForce
{
    public static void ForGizmo(Vector3 pos, Vector3 direction, float arrowLength, float lengthMultiplier = 1f, float arrowHeadLength = 0.25f, float arrowHeadAngle = 20.0f)
    {
        Gizmos.DrawRay(pos, direction * Math.Abs(arrowLength) * lengthMultiplier);

        Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0) * new Vector3(0, 0, 1);
        Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0) * new Vector3(0, 0, 1);
        Gizmos.DrawRay(pos + direction * arrowLength * lengthMultiplier, right * arrowHeadLength);
        Gizmos.DrawRay(pos + direction * arrowLength * lengthMultiplier, left * arrowHeadLength);
    }

    public static void ForGizmo(Vector3 pos, Vector3 direction, float arrowLength, Color color, float lengthMultiplier = 1f, float arrowHeadLength = 0.25f, float arrowHeadAngle = 20.0f)
    {
        Gizmos.color = color;
        Gizmos.DrawRay(pos, direction * Math.Abs(arrowLength) * lengthMultiplier);

        Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0) * new Vector3(0, 0, 1);
        Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0) * new Vector3(0, 0, 1);
        Gizmos.DrawRay(pos + direction * arrowLength * lengthMultiplier, right * arrowHeadLength);
        Gizmos.DrawRay(pos + direction * arrowLength * lengthMultiplier, left * arrowHeadLength);
    }

    public static void ForDebug(Vector3 pos, float arrowLength, float lengthMultiplier = 1f, float arrowHeadLength = 0.05f, float arrowHeadAngle = 20.0f)
    {
        Vector3 direction;

        if (arrowLength >= 0)
            direction = Vector3.up;
        else
            direction = Vector3.down;

        Debug.DrawRay(pos, direction * Math.Abs(arrowLength) * lengthMultiplier);

        Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0) * new Vector3(0, 0, 1);
        Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0) * new Vector3(0, 0, 1);
        Debug.DrawRay(pos + (direction * arrowLength * lengthMultiplier), right * arrowHeadLength);
        Debug.DrawRay(pos + (direction * arrowLength * lengthMultiplier), left * arrowHeadLength);
    }
    public static void ForDebug(Vector3 pos, float arrowLength, Color color, float lengthMultiplier = 1f, float arrowHeadLength = 0.05f, float arrowHeadAngle = 20.0f)
    {
        Vector3 direction;

        if (arrowLength >= 0)
            direction = Vector3.up;
        else
            direction = Vector3.down;

        Debug.DrawRay(pos, direction * Math.Abs(arrowLength) * lengthMultiplier, color);

        Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0) * new Vector3(0, 0, 1);
        Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0) * new Vector3(0, 0, 1);
        Debug.DrawRay(pos + (direction * Math.Abs(arrowLength) * lengthMultiplier), right * arrowHeadLength, color);
        Debug.DrawRay(pos + (direction * Math.Abs(arrowLength) * lengthMultiplier), left * arrowHeadLength, color);
    }

    public static void ForDebug3D(Vector3 pos, Vector3 direction, float lengthMultiplier = 1f, float arrowHeadLength = 0.05f, float arrowHeadAngle = 20.0f)
    {
        if(direction != Vector3.zero)
        {
            Debug.DrawRay(pos, direction * lengthMultiplier);

            Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0) * new Vector3(0, 0, 1);
            Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0) * new Vector3(0, 0, 1);
            Debug.DrawRay(pos + (direction * lengthMultiplier), right * arrowHeadLength);
            Debug.DrawRay(pos + (direction * lengthMultiplier), left * arrowHeadLength);
        }
    }
    public static void ForDebug3D(Vector3 pos, Vector3 direction, Color color, float lengthMultiplier = 1f, float arrowHeadLength = 0.05f, float arrowHeadAngle = 20.0f)
    {
        if (direction != Vector3.zero)
        {
            Debug.DrawRay(pos, direction * lengthMultiplier, color);

            Vector3 right = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 + arrowHeadAngle, 0) * new Vector3(0, 0, 1);
            Vector3 left = Quaternion.LookRotation(direction) * Quaternion.Euler(0, 180 - arrowHeadAngle, 0) * new Vector3(0, 0, 1);
            Debug.DrawRay(pos + (direction * lengthMultiplier), right * arrowHeadLength, color);
            Debug.DrawRay(pos + (direction * lengthMultiplier), left * arrowHeadLength, color);
        }
    }
}
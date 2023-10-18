using Godot;
using System;

public partial class ParticleCam : Camera3D
{
	// Called when the node enters the scene tree for the first time.
	private float _distance = 150;
	private float _angle = Mathf.Pi / 4f;
	
	public override void _Ready()
	{
	}

	// Called every frame. 'delta' is the elapsed time since the previous frame.
	public override void _Process(double delta)
	{
		var move = Input.GetAxis("ui_left", "ui_right");
		_angle += (float)delta * move;

		var zoom = -Input.GetAxis("ui_down", "ui_up");
		_distance += (float)delta * zoom * 100;

		GlobalPosition = new Vector3(Mathf.Cos(_angle), 0f, Mathf.Sin(_angle)) * _distance;
		LookAt(Vector3.Zero);
	}
}

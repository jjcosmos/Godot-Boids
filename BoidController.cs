using Godot;
using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading.Tasks;

public partial class BoidController : Node3D
{
    [Export] private MultiMeshInstance3D _boidMultimesh;
    [Export] private Label _perfCounters;
    [Export] private CheckBox _showGridBox;
    [Export] private CheckBox _showVolumeBox;
    [Export] private Slider _numBoidsSlider;
    [Export] private Slider _speedSlider;

    [Export] private ColorPickerButton _lowDensityColorPicker;
    [Export] private ColorPickerButton _highDensityColorPicker;

    private Color _lowDensityColor = Colors.Aqua;
    private Color _highDensityColor = Colors.Red;

    private const int MAX_BOIDS = 4096 * 2;
    private int _activeBoids = MAX_BOIDS - 1;
    private const int BOID_SCRATCH_BUF_SIZE = MAX_BOIDS / 8;
    private Boid[] Boids = new Boid[MAX_BOIDS];
    private readonly Boid[][] _boidScratchBuffers = new Boid[MAX_BOIDS][];
    private float _simSpeed;

    public override void _Ready()
    {
        _boidMultimesh.Multimesh.InstanceCount = MAX_BOIDS;
        _lowDensityColorPicker.Color = _lowDensityColor;
        _highDensityColorPicker.Color = _highDensityColor;

        _lowDensityColorPicker.ColorChanged += LowDensityColorPickerOnColorChanged;
        _highDensityColorPicker.ColorChanged += HighDensityColorPickerOnColorChanged;

        MakeOffsets();

        for (var i = 0; i < MAX_BOIDS; i++)
        {
            var boid = new Boid()
            {
                MeshID = i, Position = RandInBounds(), Velocity = RandInBounds().Normalized() * 20f,
            };
            Boids[i] = boid;
        }

        for (var i = 0; i < MAX_BOIDS; i++)
        {
            _boidScratchBuffers[i] = new Boid[BOID_SCRATCH_BUF_SIZE];
        }

        _numBoidsSlider.MinValue = 1;
        _numBoidsSlider.MaxValue = MAX_BOIDS;
        _numBoidsSlider.Value = MAX_BOIDS;
    }

    private void HighDensityColorPickerOnColorChanged(Color color)
    {
        _highDensityColor = color;
    }

    private void LowDensityColorPickerOnColorChanged(Color color)
    {
        _lowDensityColor = color;
    }

    private readonly System.Diagnostics.Stopwatch _perfSw = new();

    public override void _Process(double delta)
    {
        delta *= _simSpeed;

        _activeBoids = (int)_numBoidsSlider.Value;
        _simSpeed = (float)_speedSlider.Value;

        _perfSw.Restart();

        BuildPartitions();
        SteerSpatial((float)delta);
        Move((float)delta);
        UpdateMeshes();
        if (_showGridBox.ButtonPressed) DrawGrid();
        if (_showVolumeBox.ButtonPressed) DrawVolume();

        DrawLines();

        _perfSw.Stop();
        _perfCounters.Text =
            $"FPS: {Engine.GetFramesPerSecond()}\nBoids: {_activeBoids}\nUpdate (ms): {_perfSw.ElapsedMilliseconds}";
    }

    private const float HALF_BOUNDS_X = 50;
    private const float HALF_BOUNDS_Y = 50;
    private const float HALF_BOUNDS_Z = 50;
    private const float DOT_VISION_THRESHOLD = 0.3f;
    private const float DIST_THRESHOLD = 10f;
    private static readonly RandomNumberGenerator _rng = new();

    private const int GRID_DIVISIONS = (int)((HALF_BOUNDS_X * 2) / DIST_THRESHOLD);
    private readonly List<Boid>[,,] _partitions = new List<Boid>[GRID_DIVISIONS, GRID_DIVISIONS, GRID_DIVISIONS];

    // Should occur at the start of the frame, before movement and integration
    private void BuildPartitions()
    {
        for (var i0 = 0; i0 < GRID_DIVISIONS; i0++)
        for (var i1 = 0; i1 < GRID_DIVISIONS; i1++)
        for (var i2 = 0; i2 < GRID_DIVISIONS; i2++)
        {
            _partitions[i0, i1, i2] ??= new List<Boid>();
            _partitions[i0, i1, i2].Clear();
        }

        for (var i = 0; i < _activeBoids; i++)
        {
            var boid = Boids[i];
            var indexInPartitian = IndexInGrid(boid);
            if (IsValidGrid(indexInPartitian))
            {
                GetPartition(indexInPartitian).Add(boid);
            }
            else
            {
                GD.Print($"{indexInPartitian} is invalid");
            }
        }
    }

    private List<Boid> GetPartition(Vector3I indexInPartitian)
    {
        return _partitions[indexInPartitian.X, indexInPartitian.Y, indexInPartitian.Z];
    }

    private Vector3I IndexInGrid(Boid boid)
    {
        return WrapGridPosition(
            new Vector3I((int)(boid.Position.X) / GRID_DIVISIONS, (int)(boid.Position.Y) / GRID_DIVISIONS,
                (int)(boid.Position.Z) / GRID_DIVISIONS) +
            new Vector3I(GRID_DIVISIONS / 2, GRID_DIVISIONS / 2, GRID_DIVISIONS / 2));
    }

    private Vector3I WrapGridPosition(Vector3I gridPos)
    {
        return new Vector3I(WrapInt(gridPos.X, GRID_DIVISIONS), WrapInt(gridPos.Y, GRID_DIVISIONS),
            WrapInt(gridPos.Z, GRID_DIVISIONS));
    }

    private static int WrapInt(int a, int wrap)
    {
        if (a >= wrap) return Math.Abs(a) % wrap;
        if (a < 0) return wrap - Math.Abs(a) % wrap;

        return a;
    }

    private readonly Vector3I[] AllOffsets = new Vector3I[27];

    private void MakeOffsets()
    {
        var counter = 0;
        for (var x = -1; x <= 1; x++)
        {
            for (var y = -1; y <= 1; y++)
            {
                for (var z = -1; z <= 1; z++)
                {
                    var vec = new Vector3I(x, y, z);
                    AllOffsets[counter] = vec;
                    counter++;
                }
            }
        }
    }

    private int GetAllBoidsSurroundingPosition(Boid b, Boid[] buffer)
    {
        var partitionIndex = IndexInGrid(b);

        var boidCount = 0;
        for (var i = 0; i < AllOffsets.Length; i++)
        {
            var grid = WrapGridPosition(partitionIndex + AllOffsets[i]);
            if (!IsValidGrid(grid)) continue;

            var boids = GetPartition(grid);
            foreach (var boid in boids)
            {
                if (boid.MeshID >= _activeBoids) continue;

                buffer[boidCount] = boid;
                boidCount++;

                // Don't write past the buffer length
                if (boidCount >= buffer.Length - 1) return boidCount;
            }
        }

        return boidCount;
    }

    private bool IsValidGrid(Vector3I grid)
    {
        return grid.X is < GRID_DIVISIONS and >= 0 && grid.Y is < GRID_DIVISIONS and >= 0 &&
               grid.Z is < GRID_DIVISIONS and >= 0;
    }

    private static Vector3 RandInBounds()
    {
        return new Vector3(_rng.RandfRange(-HALF_BOUNDS_X, HALF_BOUNDS_X),
            _rng.RandfRange(-HALF_BOUNDS_Y, HALF_BOUNDS_Y), _rng.RandfRange(-HALF_BOUNDS_Z, HALF_BOUNDS_Z));
    }

    private void DrawVolume()
    {
        // upper
        var upperFarLeft = new Vector3(-HALF_BOUNDS_X, HALF_BOUNDS_Y, -HALF_BOUNDS_Z);
        var upperFarRight = new Vector3(HALF_BOUNDS_X, HALF_BOUNDS_Y, -HALF_BOUNDS_Z);

        var upperCloseLeft = new Vector3(-HALF_BOUNDS_X, HALF_BOUNDS_Y, HALF_BOUNDS_Z);
        var upperCloseRight = new Vector3(HALF_BOUNDS_X, HALF_BOUNDS_Y, HALF_BOUNDS_Z);

        // lower
        var lowerFarLeft = new Vector3(-HALF_BOUNDS_X, -HALF_BOUNDS_Y, -HALF_BOUNDS_Z);
        var lowerFarRight = new Vector3(HALF_BOUNDS_X, -HALF_BOUNDS_Y, -HALF_BOUNDS_Z);

        var lowerCloseLeft = new Vector3(-HALF_BOUNDS_X, -HALF_BOUNDS_Y, HALF_BOUNDS_Z);
        var lowerCloseRight = new Vector3(HALF_BOUNDS_X, -HALF_BOUNDS_Y, HALF_BOUNDS_Z);

        var color = Colors.White;

        _linesToDraw.Add((upperFarLeft, upperFarRight, color));
        _linesToDraw.Add((upperFarRight, upperCloseRight, color));
        _linesToDraw.Add((upperCloseRight, upperCloseLeft, color));
        _linesToDraw.Add((upperCloseLeft, upperFarLeft, color));

        _linesToDraw.Add((lowerFarLeft, lowerFarRight, color));
        _linesToDraw.Add((lowerFarRight, lowerCloseRight, color));
        _linesToDraw.Add((lowerCloseRight, lowerCloseLeft, color));
        _linesToDraw.Add((lowerCloseLeft, lowerFarLeft, color));

        _linesToDraw.Add((upperFarLeft, lowerFarLeft, color));
        _linesToDraw.Add((upperCloseLeft, lowerCloseLeft, color));
        _linesToDraw.Add((upperFarRight, lowerFarRight, color));
        _linesToDraw.Add((upperCloseRight, lowerCloseRight, color));
    }

    private void DrawGrid()
    {
        for (var y = 0; y <= GRID_DIVISIONS; y++)
        {
            var start = new Vector3(-HALF_BOUNDS_X, HALF_BOUNDS_Y - y * GRID_DIVISIONS, 0);
            var end = new Vector3(HALF_BOUNDS_X, HALF_BOUNDS_Y - y * GRID_DIVISIONS, 0);

            _linesToDraw.Add((start, end, Colors.Red));
        }

        for (var x = 0; x <= GRID_DIVISIONS; x++)
        {
            var start = new Vector3(-HALF_BOUNDS_X + x * GRID_DIVISIONS, -HALF_BOUNDS_Y, 0);
            var end = new Vector3(-HALF_BOUNDS_X + x * GRID_DIVISIONS, HALF_BOUNDS_Y, 0);

            _linesToDraw.Add((start, end, Colors.Red));
        }
    }

    private void SteerSpatial(float delta)
    {
        Parallel.For(0, _activeBoids, i =>
        {
            var scratch = _boidScratchBuffers[i];
            var boidA = Boids[i];
            var surrounding = GetAllBoidsSurroundingPosition(boidA, scratch);

            // Sum and div neighbor velocity to get average direction
            var neighborCount = 0;
            var totalPositions = Vector3.Zero;
            var totalVelocity = Vector3.Zero;
            for (var nIndex = 0; nIndex < surrounding; nIndex++)
            {
                var boidB = scratch[nIndex];

                // Ignore self
                if (boidA.MeshID == boidB.MeshID) continue;
                var distToOther = boidA.Position.DistanceTo(boidB.Position);
                // Ignore boids out of range
                if (distToOther > DIST_THRESHOLD) continue;
                // Ignore boids not in front of A
                if (boidA.Velocity.Normalized().Dot((boidB.Position - boidA.Position).Normalized()) <=
                    DOT_VISION_THRESHOLD)
                    continue;

                // Steer away from neighbor proportional to the squared distance
                var steeringStrength = (1f / Mathf.Max((distToOther), Mathf.Epsilon)) * 1;
                var awayVec = -(boidB.Position - boidA.Position).Normalized();
                boidA.SteerInDirection(awayVec, delta * steeringStrength);

                totalVelocity += boidB.Velocity;
                totalPositions += boidB.Position;

                //_linesToDraw.Add((boidA.Position, boidB.Position, Colors.Green.Lerp(Colors.Red, distToOther/DistThreshold)));
                neighborCount++;
            }

            if (neighborCount > 0)
            {
                var averageDirection = totalVelocity / neighborCount;
                var averagePosition = totalPositions / neighborCount;

                boidA.SteerInDirection(averageDirection.Normalized(), delta * .05f);
                boidA.SteerInDirection((averagePosition - boidA.Position).Normalized(), delta * 2);
            }

            _boidMultimesh.Multimesh.SetInstanceColor(boidA.MeshID,
                _lowDensityColor.Lerp(_highDensityColor, neighborCount / 50f));
        });
    }

    private ConcurrentBag<(Vector3, Vector3, Color)> _linesToDraw = new();
    private ImmediateMesh _immediateMesh;
    private MeshInstance3D _debugMeshInstance;
    private StandardMaterial3D _debugMat = new();

    private void DrawLines()
    {
        if (_debugMeshInstance == null)
        {
            _debugMeshInstance = new();
            AddChild(_debugMeshInstance);
        }

        if (_immediateMesh == null)
        {
            _immediateMesh = new();
            _debugMat.NoDepthTest = false;
            _debugMat.VertexColorUseAsAlbedo = true;
            _debugMat.ShadingMode = BaseMaterial3D.ShadingModeEnum.Unshaded;
            _debugMeshInstance.Mesh = _immediateMesh;
            _debugMeshInstance.MaterialOverride = _debugMat;
        }

        _immediateMesh.ClearSurfaces();
        if (_linesToDraw.Count == 0) return;
        _immediateMesh.SurfaceBegin(Mesh.PrimitiveType.Lines);

        foreach (var tuple in _linesToDraw)
        {
            _immediateMesh.SurfaceSetColor(tuple.Item3);
            _immediateMesh.SurfaceAddVertex(tuple.Item1);
            _immediateMesh.SurfaceAddVertex(tuple.Item2);
        }

        _immediateMesh.SurfaceEnd();

        _linesToDraw.Clear();
    }

    private void Move(float delta)
    {
        Parallel.For(0, _activeBoids, i =>
        {
            var boid = Boids[i];
            boid.Position += boid.Velocity * delta;
            boid.Position.X = WrapValue(boid.Position.X, HALF_BOUNDS_X);
            boid.Position.Y = WrapValue(boid.Position.Y, HALF_BOUNDS_Y);
            boid.Position.Z = WrapValue(boid.Position.Z, HALF_BOUNDS_Z);
        });
    }

    private float WrapValue(float value, float halfBounds)
    {
        if (value > halfBounds) return -halfBounds + Mathf.Abs(value % halfBounds);
        if (value < -halfBounds) return halfBounds - Mathf.Abs(value % halfBounds);

        return value;
    }

    private void UpdateMeshes()
    {
        _boidMultimesh.Multimesh.VisibleInstanceCount = _activeBoids;
        for (var i = 0; i < _activeBoids; i++)
        {
            var boid = Boids[i];
            var xForm = new Transform3D(Basis.Identity, boid.Position).LookingAt(boid.Position + Vector3.Forward,
                boid.Velocity);
            _boidMultimesh.Multimesh.SetInstanceTransform(i, xForm);
        }
    }

    private class Boid
    {
        public int MeshID;
        public Vector3 Position;
        public Vector3 Velocity;

        public void SteerInDirection(Vector3 direction, float strength)
        {
            Velocity = Velocity.Normalized().MoveToward(direction, strength).Normalized() * Velocity.Length();
        }
    }
}
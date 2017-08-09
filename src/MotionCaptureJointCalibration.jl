__precompile__(true)

module MotionCaptureJointCalibration

export
    PoseData,
    CalibrationProblem,
    CalibrationResult,
    solve,
    calibration_joints,
    free_joints,
    num_poses,
    num_calibration_params,
    num_markers,
    num_bodies

using Requires
using StaticArrays
using RigidBodyDynamics
using JuMP
using MathProgBase: SolverInterface.AbstractMathProgSolver

import RigidBodyDynamics.Graphs: TreePath
import RigidBodyDynamics: GenericJoint

include("util.jl")
include("problem.jl")
include("result.jl")
include("deconstruct.jl")
include("residual.jl")
include("solve.jl")
include("visualization.jl")
include("synthetic.jl")

end # module

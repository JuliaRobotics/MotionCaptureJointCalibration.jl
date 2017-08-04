__precompile__(false) # because of Requires

module MotionCaptureJointCalibration

export
    PoseData,
    CalibrationProblem,
    CalibrationResult,
    solve,
    num_poses

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

end # module

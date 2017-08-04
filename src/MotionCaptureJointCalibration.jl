__precompile__()

module MotionCaptureJointCalibration

# types
export
    PoseData,
    CalibrationProblem,
    CalibrationResult,
    solve,
    num_poses

using StaticArrays
using RigidBodyDynamics
import RigidBodyDynamics.Graphs: TreePath
using JuMP
using MathProgBase: SolverInterface.AbstractMathProgSolver

import RigidBodyDynamics: GenericJoint

include("util.jl")
include("problem.jl")
include("result.jl")
include("deconstruct.jl")
include("residual.jl")
include("solve.jl")
include("visualization.jl")

end # module

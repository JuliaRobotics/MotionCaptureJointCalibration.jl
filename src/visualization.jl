using DrakeVisualizer
using RigidBodyTreeInspector
using RigidBodyTreeInspector: RGBA, HyperSphere, Point
using Interact

function RigidBodyTreeInspector.inspect!(state::MechanismState, vis::Visualizer, problem::CalibrationProblem, result::CalibrationResult)
    radius = 0.01
    semitransparent_green = RGBA(0., 1, 0, 0.5)
    for points in values(result.marker_positions)
        for point in points
            sphere = HyperSphere(Point{3, Float64}(point.v), radius)
            addgeometry!(vis, state.mechanism, point.frame, GeometryData(sphere, semitransparent_green))
        end
    end
    markervis = vis[:marker_measurements]

    cal_selector = radiobuttons(Dict("Before calibration" => false, "After calibration" => true))
    pose_slider = slider(1 : num_poses(result), value = 1, label = "Pose number")
    map(pose_slider, cal_selector) do i, cal
        set_configuration!(state, result.configurations[i])
        if !cal
            q_before_cal = problem.pose_data[i].configuration
            for joint in keys(problem.calibration_param_bounds)
                set_configuration!(state, joint, q_before_cal[joint])
            end
        end
        settransform!(vis, state)

        delete!(markervis)
        for points in values(problem.pose_data[i].marker_positions)
            for point in points
                num_measured_coordinates = 3 - count(isnan.(point.v))
                if num_measured_coordinates > 0
                    blueness = num_measured_coordinates / 3
                    color = RGBA(0., 0., blueness, 0.5)
                    sphere = HyperSphere(Point{3, Float64}(point.v), radius)
                    addgeometry!(markervis, state.mechanism, point.frame, GeometryData(sphere, color))
                end
            end
        end
    end

    vbox(cal_selector, pose_slider)
end
